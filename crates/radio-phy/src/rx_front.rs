//! Receiver front, float reference: DC blocker, packet detection, carrier frequency offset
//! (CFO) estimation and symbol timing for 802.11a/g PPDUs.
//!
//! Design, requirements and the theory the gate checks: `docs/gates/rx_front.md`. R0-08a chose
//! the algorithms; R0-08b restated them as a streaming receiver ([`RxFront`]: one sample in,
//! every decision causal, bounded memory) — the form the fixed-point model and the RTL follow.
//! All algorithms are textbook (Schmidl & Cox 1997; matched filtering; least-squares timing).
//!
//! Index conventions: a metric at index `n` uses samples up to and including `n` (causal), so
//! `detected_at` is the newest sample the detector had seen when it fired, and `decided_at`
//! the sample at which the acquisition was declared.
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
/// The LTF search starts this many samples after the detection that opens it (detection can
/// fire as early as STF + 15, T1 is at STF + 192).
pub const SEARCH_OFFSET: usize = 24;
/// Candidate LTF positions after the latest detection. A maximum in the last 64 + (GI − B)
/// positions of the window cannot be accepted (the true T1, 64 later, may lie beyond it: GI2
/// is half an LTF symbol, so the position 64 early still matches partly); every detection
/// moves the window's end, and with no detection for this long the search closes.
pub const SEARCH_LEN: usize = 240;
/// The energy window keeps the latest start whose energy is at least this fraction of the
/// maximum (so a single path is not placed anywhere on a flat plateau).
pub const ENERGY_KEEP: f64 = 0.875;
/// Minimum normalised LTF energy (≈ SNR/(1 + SNR) for a packet; noise alone gives
/// ≈ (GI − B)/128) to confirm a packet.
pub const LTF_CONFIRM: f64 = 0.5;

/// Minimum coherent normalised energy of T1 + T2 at the confirmed position (noise alone gives
/// ≈ (GI − B)/128 ≈ 0.09, a clean LTF 1, a true LTF under a CFO error δ ≈ sinc²(128·δ/fs)):
/// the second confirmation rule, rejecting inputs that correlate without repeating.
pub const LTF_COHERENCE: f64 = 0.25;

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

/// The final coarse window ends this far before the first-path LTF symbol T1: products
/// k ∈ [T1 − 64 − L, T1 − 64), so k + 16 stays 16 samples clear of GI2 (delay spread) and the
/// window starts 32 samples into the STF (AGC settling).
pub(crate) const COARSE_END_BEFORE_T1: usize = 64;

/// Result of acquiring one PPDU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Acquisition {
    /// The sample at which the acquisition was declared (every sample it used is at or
    /// before it).
    pub decided_at: usize,
    /// The detection that opened the LTF search (newest sample the detector had seen).
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

/// One-pole DC blocker, causal: y[n] = x[n] − m[n−1], m[n] = m[n−1] + μ_n·(x[n] − m[n−1]),
/// m[−1] = 0.
///
/// μ is gear-shifted at start-up: μ_n = 2^(−k), k = min(shift, ⌊log₂(n + 1)⌋), so the estimate
/// starts as a (power-of-two approximated) running mean and settles into the steady-state pole
/// 2^(−shift) after 2^shift − 1 samples. A blocker starting at μ = 2^(−shift) from m = 0 would
/// leave a DC offset decaying over hundreds of samples, which correlates at lag 16 and triggers
/// the detector. Shifts only: the RTL form is an accumulator and a barrel shifter.
#[derive(Debug, Clone, PartialEq)]
pub struct DcBlocker {
    shift: u32,
    m: Sample,
    n: u64,
}

impl DcBlocker {
    /// A blocker whose steady-state pole is 2^(−shift) (`shift` is capped at 30).
    pub fn new(shift: u32) -> Self {
        Self {
            shift: shift.min(30),
            m: (0.0, 0.0),
            n: 0,
        }
    }

    /// The output for the next input sample.
    pub fn push(&mut self, x: Sample) -> Sample {
        // ⌊log₂(n + 1)⌋, saturating at the steady-state shift.
        let k = (u64::BITS - 1 - (self.n.saturating_add(1)).leading_zeros()).min(self.shift);
        let mu = 1.0 / f64::from(1u32 << k);
        let y = sub(x, self.m);
        self.m = add(self.m, (mu * y.0, mu * y.1));
        self.n = self.n.saturating_add(1);
        y
    }
}

/// [`DcBlocker`] over a whole stream.
pub fn dc_block(x: &[Sample], shift: u32) -> Vec<Sample> {
    let mut b = DcBlocker::new(shift);
    x.iter().map(|&s| b.push(s)).collect()
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

/// A ring buffer addressed by absolute sample index: holds the newest `capacity` entries.
/// Reading an index older than that returns a newer entry, so every reader keeps within the
/// capacity by construction (each user states the span it needs).
#[derive(Debug, Clone)]
pub(crate) struct Ring<T: Copy> {
    buf: Vec<T>,
    mask: usize,
}

impl<T: Copy> Ring<T> {
    /// A ring of at least `span` entries (rounded up to a power of two, as an RTL memory).
    pub(crate) fn new(span: usize, fill: T) -> Self {
        let cap = span.max(1).next_power_of_two();
        Self {
            buf: vec![fill; cap],
            mask: cap - 1,
        }
    }

    pub(crate) fn put(&mut self, n: usize, v: T) {
        let i = n & self.mask;
        self.buf[i] = v;
    }

    pub(crate) fn get(&self, n: usize) -> T {
        self.buf[n & self.mask]
    }
}

/// Detector A's running sums, one sample at a time; see [`autocorrelation_metric`].
#[derive(Debug, Clone)]
struct AutocorrState {
    len: usize,
    hist: Ring<Sample>,
    n: usize,
    c: Sample,
    p1: f64,
    p2: f64,
}

impl AutocorrState {
    fn new(len: usize) -> Self {
        Self {
            len,
            // Oldest sample read: k − len = n − 16 − len.
            hist: Ring::new(len + STF_PERIOD + 1, (0.0, 0.0)),
            n: 0,
            c: (0.0, 0.0),
            p1: 0.0,
            p2: 0.0,
        }
    }

    fn push(&mut self, s: Sample) -> f64 {
        let (d, len, n) = (STF_PERIOD, self.len, self.n);
        self.hist.put(n, s);
        self.n += 1;
        if len == 0 || n < d {
            return 0.0;
        }
        let x = |j: usize| self.hist.get(j);
        let k = n - d; // newest product: x(k)·x*(n)
                       // Sums are recomputed every 4096 samples so float rounding cannot accumulate (the
                       // fixed-point RTL sums are exact).
        if (k + 1).is_multiple_of(4096) || k < len {
            if k + 1 >= len {
                let lo = k + 1 - len;
                self.c = (lo..=k).fold((0.0, 0.0), |a, j| add(a, mul(x(j), conj(x(j + d)))));
                self.p1 = (lo..=k).map(|j| abs2(x(j))).sum();
                self.p2 = (lo..=k).map(|j| abs2(x(j + d))).sum();
            }
        } else {
            let old = k - len;
            self.c = add(
                sub(self.c, mul(x(old), conj(x(old + d)))),
                mul(x(k), conj(x(n))),
            );
            self.p1 += abs2(x(k)) - abs2(x(old));
            self.p2 += abs2(x(n)) - abs2(x(old + d));
        }
        if k + 1 >= len && self.p1 > 0.0 && self.p2 > 0.0 {
            (abs2(self.c) / (self.p1 * self.p2)).sqrt()
        } else {
            0.0
        }
    }
}

/// Detector A's metric |C(n)|/√(P₁(n)·P₂(n)) with C(n) = Σ x(k)·x*(k+16) over the `len`
/// products whose later sample is at most n; 0 where the window is incomplete or empty.
pub fn autocorrelation_metric(x: &[Sample], len: usize) -> Vec<f64> {
    let mut st = AutocorrState::new(len);
    x.iter().map(|&s| st.push(s)).collect()
}

/// The sign-quantised STF period q (±1 ± j) detector B correlates with.
pub fn matched_filter_taps() -> [Sample; STF_PERIOD] {
    let sgn = |v: f64| if v < 0.0 { -1.0 } else { 1.0 };
    short_training_period().map(|s| (sgn(s.0), sgn(s.1)))
}

/// Detector B's running state, one sample at a time; see [`matched_filter_metric`].
#[derive(Debug, Clone)]
struct MatchedState {
    q: [Sample; STF_PERIOD],
    q_energy: f64,
    hist: Ring<Sample>,
    m2: Ring<f64>,
    energy: f64,
    n: usize,
}

impl MatchedState {
    fn new(q: [Sample; STF_PERIOD]) -> Self {
        let span = STF_PERIOD * MATCHED_PERIODS;
        Self {
            q,
            q_energy: q.iter().map(|s| abs2(*s)).sum(),
            hist: Ring::new(span + 1, (0.0, 0.0)),
            m2: Ring::new(span, 0.0),
            energy: 0.0,
            n: 0,
        }
    }

    fn push(&mut self, s: Sample) -> f64 {
        let span = STF_PERIOD * MATCHED_PERIODS;
        let n = self.n;
        self.hist.put(n, s);
        self.n += 1;
        let m2 = if n + 1 < STF_PERIOD {
            0.0
        } else {
            let base = n + 1 - STF_PERIOD;
            abs2(self.q.iter().enumerate().fold((0.0, 0.0), |a, (i, qi)| {
                add(a, mul(conj(*qi), self.hist.get(base + i)))
            }))
        };
        self.m2.put(n, m2);
        self.energy += abs2(s);
        if n >= span {
            self.energy -= abs2(self.hist.get(n - span));
        }
        if (n + 1).is_multiple_of(4096) && n + 1 >= span {
            self.energy = (n + 1 - span..=n).map(|j| abs2(self.hist.get(j))).sum();
        }
        if n + 1 >= span && self.energy > 0.0 {
            let num: f64 = (0..MATCHED_PERIODS)
                .map(|p| self.m2.get(n - STF_PERIOD * p))
                .sum();
            num / (self.q_energy * self.energy)
        } else {
            0.0
        }
    }
}

/// Detector B's metric Σ_{p<P}|m(n−16p)|² / (‖q‖²·E(n)), m(n) = Σ_{i<16} q*_i·x(n−15+i),
/// E(n) the energy of the 16·P samples ending at n. ≤ 1 by Cauchy–Schwarz; 0 where incomplete.
/// `q` is [`matched_filter_taps`] or, for the exact variant, [`short_training_period`].
pub fn matched_filter_metric(x: &[Sample], q: &[Sample; STF_PERIOD]) -> Vec<f64> {
    let mut st = MatchedState::new(*q);
    x.iter().map(|&s| st.push(s)).collect()
}

/// The configured detector, streaming.
#[derive(Debug, Clone)]
enum DetectorState {
    Autocorrelation(AutocorrState),
    Matched(Box<MatchedState>),
}

impl DetectorState {
    fn new(detector: Detector) -> Self {
        match detector {
            Detector::Autocorrelation => Self::Autocorrelation(AutocorrState::new(AUTOCORR_LEN)),
            Detector::MatchedFilter => {
                Self::Matched(Box::new(MatchedState::new(matched_filter_taps())))
            }
            Detector::MatchedExact => {
                Self::Matched(Box::new(MatchedState::new(short_training_period())))
            }
        }
    }

    fn push(&mut self, s: Sample) -> f64 {
        match self {
            Self::Autocorrelation(st) => st.push(s),
            Self::Matched(st) => st.push(s),
        }
    }
}

/// The DC blocker as configured (a copy of the stream when it is disabled).
pub fn front(x: &[Sample], cfg: &RxConfig) -> Vec<Sample> {
    match cfg.dc_shift {
        Some(shift) => dc_block(x, shift),
        None => x.to_vec(),
    }
}

/// The configured detector's metric over an already DC-blocked stream (the same streaming
/// detector [`RxFront`] runs).
pub fn detector_metric(y: &[Sample], cfg: &RxConfig) -> Vec<f64> {
    let mut st = DetectorState::new(cfg.detector);
    y.iter().map(|&s| st.push(s)).collect()
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

/// The first PPDU the streaming receiver ([`RxFront`]) acquires whose opening detection
/// (`detected_at`) is at or after `from`. The receiver always runs from sample 0; under a spur
/// that keeps the detector firing, the opening detection can precede the packet by far, so
/// `from > 0` can skip a packet — pass 0 unless that is intended.
pub fn acquire(x: &[Sample], from: usize, cfg: &RxConfig) -> Option<Acquisition> {
    let mut rx = RxFront::new(cfg.clone());
    x.iter()
        .filter_map(|&s| rx.push(s))
        .find(|a| a.detected_at >= from)
}

/// Every PPDU the streaming receiver ([`RxFront`]) acquires in the stream.
pub fn acquisitions(x: &[Sample], cfg: &RxConfig) -> Vec<Acquisition> {
    let mut rx = RxFront::new(cfg.clone());
    x.iter().filter_map(|&s| rx.push(s)).collect()
}

/// Samples of the DC-blocked stream the receiver keeps. The oldest one a decision reads is
/// the start of the final coarse window, T1 − 160, and the decision is made by
/// T1 + 64 + 2·(GI − B) + 127 ≤ T1 + 223 (see [`RxFront`]): 384 samples, rounded up to 512.
pub const HISTORY: usize = 512;
/// The provisional latch at a detection event at n uses the lag-16 products whose later sample
/// is at most n − this: the last event on a packet falls ≈ 10–25 samples into GI2 (the
/// detector window is still mostly STF), so without the lag the latch would include GI2.
pub const LATCH_LAG: usize = 24;
/// Template length of the LTF search correlator: both long symbols, T1 T2.
pub const PAIR_LEN: usize = 2 * LTF_SYMBOL_LEN;

/// The LTF search in progress.
#[derive(Debug, Clone, Copy)]
struct Search {
    /// The detection that opened it.
    opened_by: usize,
    /// First candidate position (T1 estimate) it considers.
    first: usize,
    /// Candidates at or beyond this position are not considered; each detection moves it to
    /// detection + [`SEARCH_OFFSET`] + [`SEARCH_LEN`].
    deadline: usize,
    /// Running maximum of the energy metric: (position, energy).
    best: Option<(usize, f64)>,
}

/// The streaming receiver front: one sample in, at most one acquisition out, every decision
/// taken from the samples seen so far with bounded memory ([`HISTORY`]).
///
/// Per sample n, after the DC blocker:
/// 1. **Detector** (streaming A or B) and its run counter. A detection event opens an LTF
///    search if none is open (positions from event + [`SEARCH_OFFSET`]), or extends the open
///    one's deadline; it also **latches the provisional CFO** from the lag-16 correlation over
///    the [`COARSE_LEN`] products whose later sample is at most n − [`LATCH_LAG`]. Under a spur the detector
///    keeps firing before the packet and the latch follows the STF once it arrives; on a clean
///    packet the last event falls ≈ 10–25 samples into GI2, hence the lag.
/// 2. **NCO**: z(n) = y(n)·e^(jφ(n)), φ advancing by −2π·(provisional)/fs per sample.
/// 3. **LTF search**: |Σ_{k<128} z(i + k)·[L L]*(k)|² for position i = n − 127, energy over
///    GI − B positions; a running maximum, **accepted once 64 + (GI − B) later positions
///    are no larger** (the position 64 early matches GI2 partly; the true T1 then beats it
///    in time). The FFT grid is the latest start within GI − B of the maximum with at least
///    [`ENERGY_KEEP`] of its energy; confirmation ([`LTF_CONFIRM`]) by the non-coherent
///    normalised energy of the two long symbols on the history corrected by the final coarse
///    CFO, so neither the provisional latch's error nor a spur's bias of the coarse estimate
///    costs the coherent 128-sample loss there.
///    A candidate that fails confirmation is dropped and the running maximum restarts at the
///    next position: the search goes on, it does not wait for another detection.
/// 4. **CFO**: the final coarse estimate over STF products [T1 − 160, T1 − 64) and the fine
///    lag-64 estimate over GI2 + T1 against T1 + T2 (less the guard), both read from the
///    history buffer; fine resolved against coarse, combined with [`FINE_WEIGHT`].
///
/// A search with no accepted candidate by its deadline closes; after an acquisition the
/// detector re-arms. No sample is processed twice and no search runs in parallel.
#[derive(Debug, Clone)]
pub struct RxFront {
    cfg: RxConfig,
    w: usize,
    b: usize,
    n: usize,
    dc: Option<DcBlocker>,
    det: DetectorState,
    count: usize,
    y: Ring<Sample>,
    z: Ring<Sample>,
    pair: Ring<f64>,
    energy: Ring<f64>,
    step: f64,
    phase: f64,
    template: [Sample; PAIR_LEN],
    template_energy: f64,
    search: Option<Search>,
}

impl RxFront {
    /// A receiver in the idle state.
    pub fn new(cfg: RxConfig) -> Self {
        let b = backoff_samples(&cfg);
        let w = GI_LEN.saturating_sub(b);
        let ltf = long_training_symbol();
        let mut template = [(0.0, 0.0); PAIR_LEN];
        for (k, t) in template.iter_mut().enumerate() {
            *t = conj(ltf[k % LTF_SYMBOL_LEN]);
        }
        Self {
            dc: cfg.dc_shift.map(DcBlocker::new),
            det: DetectorState::new(cfg.detector),
            cfg,
            w,
            b,
            n: 0,
            count: 0,
            y: Ring::new(HISTORY, (0.0, 0.0)),
            z: Ring::new(PAIR_LEN, (0.0, 0.0)),
            pair: Ring::new(GI_LEN, 0.0),
            // The acceptance and the latest-start rule look back 64 + (GI − B) positions.
            energy: Ring::new(LTF_SYMBOL_LEN + GI_LEN + 1, 0.0),
            step: 0.0,
            phase: 0.0,
            // Σ|L|² over T1 T2.
            template_energy: 2.0 * ltf.iter().map(|s| abs2(*s)).sum::<f64>(),
            template,
            search: None,
        }
    }

    /// Whether an LTF search is open (the correlator and the NCO matter only then; idle, the
    /// RTL may gate them).
    pub fn searching(&self) -> bool {
        self.search.is_some()
    }

    /// Processes the next input sample; returns the acquisition decided at it, if any.
    pub fn push(&mut self, x: Sample) -> Option<Acquisition> {
        let n = self.n;
        self.n += 1;
        let yv = match &mut self.dc {
            Some(dc) => dc.push(x),
            None => x,
        };
        self.y.put(n, yv);

        // 1. Detector, run counter, search control, provisional CFO latch.
        let metric = self.det.push(yv);
        let mut event = false;
        if metric >= self.cfg.threshold {
            self.count += 1;
            if self.count >= self.cfg.run.max(1) {
                self.count = 0;
                event = true;
            }
        } else {
            self.count = 0;
        }
        if event {
            let deadline = n + SEARCH_OFFSET + SEARCH_LEN;
            match &mut self.search {
                Some(s) => s.deadline = deadline,
                None => {
                    self.search = Some(Search {
                        opened_by: n,
                        first: n + SEARCH_OFFSET,
                        deadline,
                        best: None,
                    })
                }
            }
            if let Some(k0) = (n + 1).checked_sub(LATCH_LAG + STF_PERIOD + COARSE_LEN) {
                let c = self.lag_sum(k0, COARSE_LEN, STF_PERIOD);
                let cfo = cfo_from_correlation(c, STF_PERIOD, self.cfg.sample_rate_hz);
                self.step = -2.0 * PI * cfo / self.cfg.sample_rate_hz;
            }
        }

        // 2. NCO.
        let (sin, cos) = self.phase.sin_cos();
        self.z.put(n, mul(yv, (cos, sin)));
        self.phase = (self.phase + self.step).rem_euclid(2.0 * PI);

        // 3. LTF search: correlator output for position n − 127, energy for n − 126 − w.
        let i = n.checked_sub(PAIR_LEN - 1)?;
        let c = self
            .template
            .iter()
            .enumerate()
            .fold((0.0, 0.0), |a, (k, t)| add(a, mul(self.z.get(i + k), *t)));
        self.pair.put(i, abs2(c));
        let j = (i + 1).checked_sub(self.w)?;
        if self.w == 0 {
            return None;
        }
        let e: f64 = (j..j + self.w).map(|p| self.pair.get(p)).sum();
        self.energy.put(j, e);
        let mut s = self.search?;
        if j < s.first {
            return None;
        }
        if j >= s.deadline {
            self.search = None;
            return None;
        }
        if s.best.is_none_or(|(_, emax)| e > emax) {
            s.best = Some((j, e));
        }
        let mut result = None;
        if let Some((imax, _)) = s.best {
            // Accepted once 64 + (GI − B) later positions, all inside the window, are no larger:
            // the grid may move up to GI − B past the maximum (latest start), and the position
            // 64 before the true T1 matches GI2 partly, so T1 must have had its chance relative
            // to the latest start too.
            if j >= imax + LTF_SYMBOL_LEN + self.w - 1 {
                result = self.decide(&s, imax, n);
                if result.is_none() {
                    // Drop the candidate; the running maximum restarts at the next position. A
                    // true T1 weaker (in absolute energy) than a failing candidate within the
                    // 63 + w positions after it is lost — a strong burst ending just before a
                    // packet; the block model retried from the next detection instead. Accepted:
                    // not observed in any gate or `rxfront` case (journal, R0-08b).
                    s.best = None;
                }
            }
        }
        if result.is_some() {
            self.search = None;
            self.count = 0;
        } else if j + 1 >= s.deadline {
            // Every position of the window considered; a maximum too close to its end to be
            // accepted is not trusted (the true T1 may lie 64 later, beyond it).
            self.search = None;
        } else {
            self.search = Some(s);
        }
        result
    }

    /// Σ_{k<len} y(start+k)·y*(start+k+lag) from the history buffer.
    fn lag_sum(&self, start: usize, len: usize, lag: usize) -> Sample {
        (start..start + len).fold((0.0, 0.0), |a, k| {
            add(a, mul(self.y.get(k), conj(self.y.get(k + lag))))
        })
    }

    /// Timing, confirmation and CFO for the accepted maximum at `imax`; at sample `n`.
    fn decide(&self, s: &Search, imax: usize, n: usize) -> Option<Acquisition> {
        let (w, fs) = (self.w, self.cfg.sample_rate_hz);
        let emax = self.energy.get(imax);
        let sym = (imax..=imax + w)
            .rev()
            .find(|&p| p < s.deadline && self.energy.get(p) >= ENERGY_KEEP * emax)?;

        // Final coarse CFO over the STF located from the timing.
        let kc = sym.checked_sub(COARSE_END_BEFORE_T1 + COARSE_LEN)?;
        if kc + HISTORY <= n {
            return None;
        }
        let coarse = cfo_from_correlation(self.lag_sum(kc, COARSE_LEN, STF_PERIOD), STF_PERIOD, fs);

        // Confirmation, non-coherent over the two long symbols: Σ_{p<w} |xc(p)|² + |xc(p + 64)|²
        // with xc the 64-sample correlation, on the history corrected by the final coarse CFO.
        // A residual CFO error δ costs it sinc²(64·δ/fs) instead of the coherent pair's
        // sinc²(128·δ/fs): at 57 kHz (a spur at the noise level biases both estimators) 0.4 dB
        // instead of 1.9 dB. Normalised by E_L·(energy over both symbols): a clean LTF gives 1,
        // noise ≈ 2(GI − B)/128. Positions up to sym + w − 1 + 127 ≤ n.
        let ltf = long_training_symbol();
        let step = -2.0 * PI * coarse / fs;
        let rot: Vec<Sample> = (0..PAIR_LEN + w)
            .map(|m| {
                let (sin, cos) = (step * m as f64).sin_cos();
                mul(self.y.get(sym + m), (cos, sin))
            })
            .collect();
        let xc = |o: usize| {
            ltf.iter()
                .enumerate()
                .fold((0.0, 0.0), |a, (k, l)| add(a, mul(rot[o + k], conj(*l))))
        };
        let (conf, coh) = (0..w).fold((0.0, 0.0), |(nc, c), o| {
            let (a, b) = (xc(o), xc(o + LTF_SYMBOL_LEN));
            (nc + abs2(a) + abs2(b), c + abs2(add(a, b)))
        });
        let rx_energy: f64 = (sym..sym + PAIR_LEN).map(|p| abs2(self.y.get(p))).sum();
        if rx_energy <= 0.0 {
            return None;
        }
        let ltf_metric = conf / (self.template_energy / 2.0 * rx_energy);
        // The LTF is two identical symbols: their correlations must also add coherently. A
        // non-stationary input (the DC transient after a large AGC step down) spreads a small
        // correlation over every position of the window, which the non-coherent sum collects
        // (0.62 measured) but which does not repeat from T1 to T2 (coherent 0.007).
        let coherence = coh / (self.template_energy * rx_energy);
        if ltf_metric < LTF_CONFIRM || coherence < LTF_COHERENCE {
            return None;
        }

        // Fine CFO, lag 64, over GI2 + T1 against T1 + T2, resolved against the coarse one
        // (unambiguous while the coarse error is below fs/128).
        let fine_start = (sym + FINE_GUARD).checked_sub(GI2_LEN)?;
        let raw = cfo_from_correlation(
            self.lag_sum(fine_start, FINE_LEN, LTF_SYMBOL_LEN),
            LTF_SYMBOL_LEN,
            fs,
        );
        let span = fs / LTF_SYMBOL_LEN as f64;
        let fine = coarse + (raw - coarse + span / 2.0).rem_euclid(span) - span / 2.0;

        Some(Acquisition {
            decided_at: n,
            detected_at: s.opened_by,
            ltf_start: sym - GI2_LEN,
            fft_start: sym.checked_sub(self.b)?,
            cfo_hz: coarse + FINE_WEIGHT * (fine - coarse),
            coarse_cfo_hz: coarse,
            fine_cfo_hz: fine,
            ltf_metric,
        })
    }
}
