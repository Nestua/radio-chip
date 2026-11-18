//! Receiver front, float reference (R0-08a): DC blocker, packet detection, carrier frequency
//! offset (CFO) estimation and symbol timing for 802.11a/g PPDUs.
//!
//! Design, requirements and the theory the gate checks: `docs/gates/rx_front.md`. This is the
//! floating-point reference; the fixed-point model the RTL must match follows in R0-08b. All
//! algorithms are textbook (Schmidl & Cox 1997; matched filtering; least-squares timing).
//!
//! Index conventions: a metric at index `n` uses samples up to and including `n` (causal), so
//! `detected_at` is the newest sample the detector had seen when it fired.
//!
//! Sign convention: a received r(n) = s(n)·e^(j2π·Δf·n/fs) gives Σ r(k)·r*(k+D) with angle
//! −2π·Δf·D/fs, so Δf = −∠/(2π·D/fs).

use crate::ppdu::{long_training_sequence, SHORT_TRAINING};
use std::f64::consts::PI;

/// A complex baseband sample `(re, im)`.
pub type Sample = (f64, f64);

/// Period of the short training field in samples.
pub const STF_PERIOD: usize = 16;
/// Length of one long training symbol (and of the FFT).
pub const LTF_SYMBOL_LEN: usize = 64;
/// Length of the long training field's guard interval GI2.
pub const GI2_LEN: usize = 32;
/// Cyclic prefix of a data symbol.
pub const GI_LEN: usize = 16;

/// Products in the detector-A (autocorrelation) window. 48 products + lag 16 = 64 samples
/// (3.2 µs at 20 MHz) of the 80-sample (4 µs) detection budget.
pub const AUTOCORR_LEN: usize = 48;
/// Detector-A threshold on |C|/√(P₁P₂). With [`AUTOCORR_RUN`] = 4: no event in 10⁹ noise
/// samples (budget 5·10⁻⁸ per sample = 1 false alarm/s at 20 MHz; `rxfront tail`).
pub const AUTOCORR_THRESHOLD: f64 = 0.575;
/// Consecutive samples above the threshold detector A needs.
pub const AUTOCORR_RUN: usize = 4;
/// STF periods summed non-coherently by detector B (matched filter).
pub const MATCHED_PERIODS: usize = 4;
/// Detector-B threshold on Σ|m|²/(‖q‖²·E), which is ≤ 1.
pub const MATCHED_THRESHOLD: f64 = 0.35;
/// Threshold of detector B with exact taps.
pub const MATCHED_EXACT_THRESHOLD: f64 = 0.35;
/// Consecutive samples above the threshold detector B needs.
pub const MATCHED_RUN: usize = 1;

/// Lag-16 products in the coarse CFO estimate.
pub const COARSE_LEN: usize = 96;
/// Samples kept clear at each end of the fine window (field boundaries smeared by the channel).
pub const FINE_GUARD: usize = 4;
/// Lag-64 products in the fine CFO estimate: GI2 + both LTF symbols (96 + 64 samples) less a
/// guard at each end.
pub const FINE_LEN: usize = 96 - 2 * FINE_GUARD;
/// Weight of the fine (residual) estimate when combining it with the coarse one, by inverse
/// variance at high SNR: var(Δf) ∝ D/(L²·SNR)/D² = 1/(L²·D), so with L = 96 and 88,
/// D = 16 and 64: v_coarse/v_fine = (88²·64)/(96²·16) = 3.36, w = 3.36/4.36 = 0.771.
pub const FINE_WEIGHT: f64 = {
    let v_coarse = 1.0 / ((COARSE_LEN * COARSE_LEN * STF_PERIOD) as f64);
    let v_fine = 1.0 / ((FINE_LEN * FINE_LEN * LTF_SYMBOL_LEN) as f64);
    v_coarse / (v_coarse + v_fine)
};
/// The LTF search starts this many samples after the detection.
pub const SEARCH_OFFSET: usize = 24;
/// Candidate LTF positions searched. A maximum in the last 64 + (GI − B) positions is not
/// trusted (the true T1, 64 later, may lie beyond the region: GI2 is half an LTF symbol, so the
/// position 64 early still matches partly); the next detection retries.
pub const SEARCH_LEN: usize = 240;
/// The energy window keeps the latest start whose energy is at least this fraction of the
/// maximum (so a single path is not placed anywhere on a flat plateau).
pub const ENERGY_KEEP: f64 = 0.875;
/// Minimum normalised LTF energy (≈ SNR/(1 + SNR) for a packet; noise alone gives
/// ≈ (GI − B)/128) to confirm a packet.
pub const LTF_CONFIRM: f64 = 0.5;

/// Packet detector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Detector {
    /// A: normalised delay-and-correlate on the STF (lag 16).
    Autocorrelation,
    /// B: 16-tap STF matched filter with sign-quantised taps (±1 ± j: adders only),
    /// magnitudes summed over periods.
    MatchedFilter,
    /// B with exact taps (the unquantised STF period): the bound on what tap quantisation
    /// can recover, and the float reference if R0-08b quantises the taps more finely.
    MatchedExact,
}

/// Receiver-front configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct RxConfig {
    /// Sample rate (20, 10 or 5 MHz).
    pub sample_rate_hz: f64,
    /// Packet detector.
    pub detector: Detector,
    /// Detection threshold on the detector's normalised metric.
    pub threshold: f64,
    /// Consecutive samples at or above the threshold that make a detection.
    pub run: usize,
    /// DC blocker pole: the DC estimate moves by 2^(−shift) of the error per sample; `None`
    /// disables the blocker (test only).
    pub dc_shift: Option<u32>,
    /// FFT window back-off into the cyclic prefix, in ns.
    pub backoff_ns: f64,
}

impl RxConfig {
    /// Defaults: detector A (chosen by measurement, `docs/gates/rx_front.md`), DC blocker 2^−7 (−3 dB at ≈ 25 kHz at 20 MHz), 200 ns
    /// back-off.
    pub fn new(sample_rate_hz: f64) -> Self {
        Self {
            sample_rate_hz,
            detector: Detector::Autocorrelation,
            threshold: AUTOCORR_THRESHOLD,
            run: AUTOCORR_RUN,
            dc_shift: Some(7),
            backoff_ns: 200.0,
        }
    }

    /// The same configuration with `detector` and that detector's threshold and run length.
    pub fn with_detector(mut self, detector: Detector) -> Self {
        let (threshold, run) = match detector {
            Detector::Autocorrelation => (AUTOCORR_THRESHOLD, AUTOCORR_RUN),
            Detector::MatchedFilter => (MATCHED_THRESHOLD, MATCHED_RUN),
            Detector::MatchedExact => (MATCHED_EXACT_THRESHOLD, MATCHED_RUN),
        };
        self.detector = detector;
        self.threshold = threshold;
        self.run = run;
        self
    }
}

/// The provisional coarse window ends this far after the detection: products
/// k ∈ [d + 56 − L, d + 56). It only has to be good enough for the LTF search; where a
/// detection fires early (a tone keeps detector A firing) it may sit on noise or the spur.
const PROVISIONAL_LEAD: usize = 56;
/// The final coarse window ends this far before the first-path LTF symbol T1: products
/// k ∈ [T1 − 64 − L, T1 − 64), so k + 16 stays 16 samples clear of GI2 (delay spread) and the
/// window starts 32 samples into the STF (AGC settling).
const COARSE_END_BEFORE_T1: usize = 64;

/// Result of acquiring one PPDU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Acquisition {
    /// Newest sample the detector had seen when it fired.
    pub detected_at: usize,
    /// First sample of GI2 (start of the long training field) on the first-path grid.
    pub ltf_start: usize,
    /// First sample of the FFT window of the first long training symbol: the first-path
    /// estimate minus the back-off. Later symbols follow at +64 (T2) and +144 + 80·i.
    pub fft_start: usize,
    /// Combined CFO estimate in Hz.
    pub cfo_hz: f64,
    /// Coarse (STF) estimate alone.
    pub coarse_cfo_hz: f64,
    /// Fine estimate alone (coarse + the LTF residual).
    pub fine_cfo_hz: f64,
    /// Normalised LTF energy that confirmed the packet.
    pub ltf_metric: f64,
}

fn mul(a: Sample, b: Sample) -> Sample {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn conj(a: Sample) -> Sample {
    (a.0, -a.1)
}

fn abs2(a: Sample) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

fn add(a: Sample, b: Sample) -> Sample {
    (a.0 + b.0, a.1 + b.1)
}

fn sub(a: Sample, b: Sample) -> Sample {
    (a.0 - b.0, a.1 - b.1)
}

fn idft_symbol(bins: &[(i8, Sample)]) -> [Sample; LTF_SYMBOL_LEN] {
    let mut out = [(0.0, 0.0); LTF_SYMBOL_LEN];
    for (n, o) in out.iter_mut().enumerate() {
        for &(k, v) in bins {
            let a = 2.0 * PI * f64::from(k) * n as f64 / LTF_SYMBOL_LEN as f64;
            *o = add(*o, mul(v, (a.cos(), a.sin())));
        }
    }
    out
}

/// One 16-sample period of the short training field (IEEE 802.11-2020 §17.3.3), unscaled.
pub fn short_training_period() -> [Sample; STF_PERIOD] {
    let level = (13.0f64 / 6.0).sqrt();
    let bins: Vec<(i8, Sample)> = SHORT_TRAINING
        .iter()
        .map(|&(k, s)| (k, (level * f64::from(s), level * f64::from(s))))
        .collect();
    let symbol = idft_symbol(&bins);
    let mut out = [(0.0, 0.0); STF_PERIOD];
    out.copy_from_slice(&symbol[..STF_PERIOD]);
    out
}

/// One long training symbol (IEEE 802.11-2020 §17.3.3), unscaled.
pub fn long_training_symbol() -> [Sample; LTF_SYMBOL_LEN] {
    let bins: Vec<(i8, Sample)> = (-26i8..=26)
        .zip(long_training_sequence())
        .filter(|(_, l)| *l != 0)
        .map(|(k, l)| (k, (f64::from(l), 0.0)))
        .collect();
    idft_symbol(&bins)
}

/// One-pole DC blocker: y[n] = x[n] − m[n−1], m[n] = m[n−1] + 2^(−shift)·(x[n] − m[n−1]).
///
/// m[−1] is the mean of the first 2^shift samples (a block average at start-up), so neither a
/// stream that starts on a DC offset nor one that starts on a noise sample rings: a ringing
/// DC estimate correlates at lag 16 and would trigger the detector.
pub fn dc_block(x: &[Sample], shift: u32) -> Vec<Sample> {
    let shift = shift.min(30);
    let mu = 1.0 / f64::from(1u32 << shift);
    let head = &x[..x.len().min(1usize << shift)];
    let mut m = if head.is_empty() {
        (0.0, 0.0)
    } else {
        let s = head.iter().fold((0.0, 0.0), |a, v| add(a, *v));
        (s.0 / head.len() as f64, s.1 / head.len() as f64)
    };
    x.iter()
        .map(|&s| {
            let y = sub(s, m);
            m = add(m, (mu * y.0, mu * y.1));
            y
        })
        .collect()
}

/// Σ_{k<len} x(start+k)·x*(start+k+lag). (0, 0) if the window does not fit.
pub fn lag_correlation(x: &[Sample], start: usize, len: usize, lag: usize) -> Sample {
    if start + len + lag > x.len() {
        return (0.0, 0.0);
    }
    (start..start + len).fold((0.0, 0.0), |acc, k| add(acc, mul(x[k], conj(x[k + lag]))))
}

/// The CFO in Hz that a lag-`lag` correlation `c` indicates (unambiguous to ±fs/(2·lag)).
pub fn cfo_from_correlation(c: Sample, lag: usize, sample_rate_hz: f64) -> f64 {
    -c.1.atan2(c.0) * sample_rate_hz / (2.0 * PI * lag as f64)
}

/// Rotates the stream by −`cfo_hz`: y[n] = x[n]·e^(−j2π·cfo·n/fs).
pub fn correct(x: &[Sample], cfo_hz: f64, sample_rate_hz: f64) -> Vec<Sample> {
    let step = -2.0 * PI * cfo_hz / sample_rate_hz;
    x.iter()
        .enumerate()
        .map(|(n, &s)| {
            let (sin, cos) = (step * n as f64).sin_cos();
            mul(s, (cos, sin))
        })
        .collect()
}

/// FFT back-off in samples: `backoff_ns` at the sample rate, rounded.
pub fn backoff_samples(cfg: &RxConfig) -> usize {
    (cfg.backoff_ns * 1e-9 * cfg.sample_rate_hz)
        .round()
        .max(0.0) as usize
}

/// Detector A's metric |C(n)|/√(P₁(n)·P₂(n)) with C(n) = Σ x(k)·x*(k+16) over the `len`
/// products whose later sample is at most n; 0 where the window is incomplete or empty.
pub fn autocorrelation_metric(x: &[Sample], len: usize) -> Vec<f64> {
    let d = STF_PERIOD;
    let mut out = vec![0.0; x.len()];
    if len == 0 || x.len() < len + d {
        return out;
    }
    let (mut c, mut p1, mut p2) = ((0.0, 0.0), 0.0, 0.0);
    // Sums are recomputed every 4096 samples so rounding cannot accumulate.
    for n in d..x.len() {
        let k = n - d; // newest product: x(k)·x*(n)
        if (k + 1).is_multiple_of(4096) || k < len {
            if k + 1 >= len {
                let lo = k + 1 - len;
                c = (lo..=k).fold((0.0, 0.0), |a, j| add(a, mul(x[j], conj(x[j + d]))));
                p1 = (lo..=k).map(|j| abs2(x[j])).sum();
                p2 = (lo..=k).map(|j| abs2(x[j + d])).sum();
            }
        } else {
            let old = k - len;
            c = add(sub(c, mul(x[old], conj(x[old + d]))), mul(x[k], conj(x[n])));
            p1 += abs2(x[k]) - abs2(x[old]);
            p2 += abs2(x[n]) - abs2(x[old + d]);
        }
        if k + 1 >= len && p1 > 0.0 && p2 > 0.0 {
            out[n] = (abs2(c) / (p1 * p2)).sqrt();
        }
    }
    out
}

/// The sign-quantised STF period q (±1 ± j) detector B correlates with.
pub fn matched_filter_taps() -> [Sample; STF_PERIOD] {
    let sgn = |v: f64| if v < 0.0 { -1.0 } else { 1.0 };
    short_training_period().map(|s| (sgn(s.0), sgn(s.1)))
}

/// Detector B's metric Σ_{p<P}|m(n−16p)|² / (‖q‖²·E(n)), m(n) = Σ_{i<16} q*_i·x(n−15+i),
/// E(n) the energy of the 16·P samples ending at n. ≤ 1 by Cauchy–Schwarz; 0 where incomplete.
/// `q` is [`matched_filter_taps`] or, for the exact variant, [`short_training_period`].
pub fn matched_filter_metric(x: &[Sample], q: &[Sample; STF_PERIOD]) -> Vec<f64> {
    let q_energy: f64 = q.iter().map(|s| abs2(*s)).sum();
    let span = STF_PERIOD * MATCHED_PERIODS;
    let mut out = vec![0.0; x.len()];
    if x.len() < span {
        return out;
    }
    let m2: Vec<f64> = (0..x.len())
        .map(|n| {
            if n + 1 < STF_PERIOD {
                return 0.0;
            }
            let base = n + 1 - STF_PERIOD;
            abs2(
                q.iter()
                    .enumerate()
                    .fold((0.0, 0.0), |a, (i, qi)| add(a, mul(conj(*qi), x[base + i]))),
            )
        })
        .collect();
    let mut energy = 0.0;
    for n in 0..x.len() {
        energy += abs2(x[n]);
        if n >= span {
            energy -= abs2(x[n - span]);
        }
        if (n + 1).is_multiple_of(4096) && n + 1 >= span {
            energy = x[n + 1 - span..=n].iter().map(|s| abs2(*s)).sum();
        }
        if n + 1 >= span && energy > 0.0 {
            let num: f64 = (0..MATCHED_PERIODS).map(|p| m2[n - STF_PERIOD * p]).sum();
            out[n] = num / (q_energy * energy);
        }
    }
    out
}

/// The DC blocker as configured (a copy of the stream when it is disabled).
pub fn front(x: &[Sample], cfg: &RxConfig) -> Vec<Sample> {
    match cfg.dc_shift {
        Some(shift) => dc_block(x, shift),
        None => x.to_vec(),
    }
}

/// The configured detector's metric over an already DC-blocked stream.
pub fn detector_metric(y: &[Sample], cfg: &RxConfig) -> Vec<f64> {
    match cfg.detector {
        Detector::Autocorrelation => autocorrelation_metric(y, AUTOCORR_LEN),
        Detector::MatchedFilter => matched_filter_metric(y, &matched_filter_taps()),
        Detector::MatchedExact => matched_filter_metric(y, &short_training_period()),
    }
}

/// Detection events in `metric` at or after `from`: the threshold held for `run` consecutive
/// samples; after an event the run counter restarts.
fn events(metric: &[f64], from: usize, cfg: &RxConfig, first_only: bool) -> Vec<usize> {
    let (threshold, run) = (cfg.threshold, cfg.run.max(1));
    let mut out = Vec::new();
    let mut count = 0;
    for (n, &m) in metric.iter().enumerate().skip(from) {
        if m >= threshold {
            count += 1;
            if count >= run {
                out.push(n);
                count = 0;
                if first_only {
                    break;
                }
            }
        } else {
            count = 0;
        }
    }
    out
}

/// First detection at or after `from` (DC blocker, then the configured detector).
pub fn detect(x: &[Sample], from: usize, cfg: &RxConfig) -> Option<usize> {
    let y = front(x, cfg);
    events(&detector_metric(&y, cfg), from, cfg, true)
        .first()
        .copied()
}

/// Every detection event in the stream (the detector re-arms after each).
pub fn detections(x: &[Sample], cfg: &RxConfig) -> Vec<usize> {
    let y = front(x, cfg);
    events(&detector_metric(&y, cfg), 0, cfg, false)
}

/// Acquires the first PPDU whose detection is at or after `from`: detection, coarse CFO over
/// the STF, LTF timing on the coarse-corrected stream, confirmation by the LTF energy, fine
/// CFO over GI2 + LTF, the two combined. A detection the LTF does not confirm is skipped.
pub fn acquire(x: &[Sample], from: usize, cfg: &RxConfig) -> Option<Acquisition> {
    let y = front(x, cfg);
    let m = detector_metric(&y, cfg);
    let mut start = from;
    loop {
        let d = *events(&m, start, cfg, true).first()?;
        if let Some(a) = acquire_at(&y, d, cfg) {
            return Some(a);
        }
        start = d + 1;
    }
}

/// Every PPDU acquired in the stream; after an acquisition the search resumes past its LTF.
pub fn acquisitions(x: &[Sample], cfg: &RxConfig) -> Vec<Acquisition> {
    let y = front(x, cfg);
    let m = detector_metric(&y, cfg);
    let mut out = Vec::new();
    let mut start = 0;
    while let Some(&d) = events(&m, start, cfg, true).first() {
        match acquire_at(&y, d, cfg) {
            Some(a) => {
                start = a.fft_start + 2 * LTF_SYMBOL_LEN;
                out.push(a);
            }
            None => start = d + 1,
        }
    }
    out
}

fn acquire_at(y: &[Sample], d: usize, cfg: &RxConfig) -> Option<Acquisition> {
    let fs = cfg.sample_rate_hz;
    let b = backoff_samples(cfg);
    let w = GI_LEN.checked_sub(b).filter(|w| *w > 0)?;

    // Provisional coarse CFO, lag 16, anchored on the detection: only for the LTF search.
    let k0 = (d + PROVISIONAL_LEAD).checked_sub(COARSE_LEN)?;
    let provisional = cfo_from_correlation(
        lag_correlation(y, k0, COARSE_LEN, STF_PERIOD),
        STF_PERIOD,
        fs,
    );

    // Coarse-corrected copy of the region the timing and fine estimate need.
    let lo = (d + SEARCH_OFFSET).checked_sub(GI2_LEN)?;
    let hi = d + SEARCH_OFFSET + SEARCH_LEN + w + 2 * LTF_SYMBOL_LEN;
    if hi > y.len() {
        return None;
    }
    let step = -2.0 * PI * provisional / fs;
    let z: Vec<Sample> = (lo..hi)
        .map(|n| {
            let (sin, cos) = (step * n as f64).sin_cos();
            mul(y[n], (cos, sin))
        })
        .collect();
    let at = |n: usize| z[n - lo];

    // Coherent cross-correlation with both long symbols (T1 T2, 128 samples): 64 late (T2 +
    // SIGNAL) keeps 0.23 of the peak, 64 early (GI2 is half a symbol) 0.59 — hence the
    // edge-peak rule below. Energy over a GI − B window.
    let ltf = long_training_symbol();
    let ltf_energy: f64 = ltf.iter().map(|s| abs2(*s)).sum();
    let s0 = d + SEARCH_OFFSET;
    let xc = |n: usize| {
        ltf.iter()
            .enumerate()
            .fold((0.0, 0.0), |a, (k, l)| add(a, mul(at(n + k), conj(*l))))
    };
    let pair: Vec<f64> = (s0..s0 + SEARCH_LEN + w)
        .map(|n| abs2(add(xc(n), xc(n + LTF_SYMBOL_LEN))))
        .collect();
    let energy: Vec<f64> = (0..SEARCH_LEN)
        .map(|i| pair[i..i + w].iter().sum())
        .collect();
    let (imax, emax) =
        energy.iter().enumerate().fold(
            (0, f64::MIN),
            |best, (i, e)| if *e > best.1 { (i, *e) } else { best },
        );
    if imax + LTF_SYMBOL_LEN + w > SEARCH_LEN {
        return None;
    }
    let ibest = (imax..(imax + w + 1).min(SEARCH_LEN))
        .rev()
        .find(|&i| energy[i] >= ENERGY_KEEP * emax)?;
    let sym = s0 + ibest;

    // Confirmation: the window's LTF energy against the stream's energy over both symbols.
    let rx_energy: f64 = (sym..sym + 2 * LTF_SYMBOL_LEN).map(|n| abs2(at(n))).sum();
    if rx_energy <= 0.0 {
        return None;
    }
    // |2·g·E_L|² against E_L·(2·g²·E_L): a clean LTF gives 1.
    let ltf_metric = energy[ibest] / (2.0 * ltf_energy * rx_energy);
    if ltf_metric < LTF_CONFIRM {
        return None;
    }

    // Fine CFO, lag 64, over GI2 + T1 against T1 + T2, on the coarse-corrected stream.
    let fine_start = (sym + FINE_GUARD).checked_sub(GI2_LEN)?;
    let c = (fine_start..fine_start + FINE_LEN).fold((0.0, 0.0), |a, n| {
        add(a, mul(at(n), conj(at(n + LTF_SYMBOL_LEN))))
    });
    // Fine alone: the provisional coarse plus the LTF residual (unambiguous while the
    // provisional error is below fs/128).
    let fine = provisional + cfo_from_correlation(c, LTF_SYMBOL_LEN, fs);

    // Final coarse CFO over the STF located from the timing, so a mis-anchored detection does
    // not leak into the estimate; then the inverse-variance combination.
    let kc = sym.checked_sub(COARSE_END_BEFORE_T1 + COARSE_LEN)?;
    let coarse = cfo_from_correlation(
        lag_correlation(y, kc, COARSE_LEN, STF_PERIOD),
        STF_PERIOD,
        fs,
    );

    Some(Acquisition {
        detected_at: d,
        ltf_start: sym - GI2_LEN,
        fft_start: sym.checked_sub(b)?,
        cfo_hz: coarse + FINE_WEIGHT * (fine - coarse),
        coarse_cfo_hz: coarse,
        fine_cfo_hz: fine,
        ltf_metric,
    })
}
