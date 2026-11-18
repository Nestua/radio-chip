//! G1 gate for the receiver front, float reference (R0-08a): packet detection, CFO, timing.
//!
//! Design and requirements: `docs/gates/rx_front.md`. References we did not write:
//! 1. Ground truth: our PPDUs (G1 against Annex I) through the channel simulator (G1 against
//!    theory) with a known STF start, CFO, fractional delay and channel.
//! 2. Theory, derived in the journal and computed here from the actual training samples: the
//!    first-order variance of a lag-D correlation's angle, the Cramér–Rao bound for the
//!    frequency of a known waveform, and the exact false-alarm law of a normalised correlation
//!    of two independent complex Gaussian vectors, P(|ρ̂| ≥ θ) = (1 − θ²)^(L−1).
//! 3. End to end: front end + a least-squares channel estimate from the LTF + the R0-10 Viterbi
//!    decoder recover the transmitted SIGNAL and PSDU.
//!
//! The black-box comparison with an outside receiver lives in `g1_rx_front_openofdm.rs`.
//!
//! Monte-Carlo checks use fixed seeds, so every run is identical.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, resample, tgn_taps,
    Doppler, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{
    acquire, acquisitions, autocorrelation_metric, backoff_samples, cfo_from_correlation, correct,
    detect, detections, lag_correlation, long_training_symbol, short_training_period, Detector,
    RxConfig, COARSE_LEN, FINE_LEN, FINE_WEIGHT,
};
use std::f64::consts::PI;

type C = (f64, f64);

const FS: f64 = 20e6;
const DETECTORS: [Detector; 3] = [
    Detector::Autocorrelation,
    Detector::MatchedFilter,
    Detector::MatchedExact,
];
/// Start of the first long training symbol T1 within a PPDU: STF 160 + GI2 32.
const T1: usize = 192;

mod common;
use common::decode_6mbps;

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

fn frame(psdu: &[u8], rate: Rate, seed: u8) -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

fn preamble() -> Vec<C> {
    // STF + LTF + SIGNAL + one data symbol: enough for every front-end check.
    frame(&[0x5a; 3], Rate::R6, 0x5d)
}

/// `lead` zeros, the frame, `trail` zeros; delayed by `delay` samples, rotated by `cfo_hz`
/// with a random phase, plus noise at `snr_db` relative to the frame's mean power.
#[allow(clippy::too_many_arguments)]
fn scene(
    f: &[C],
    lead: usize,
    trail: usize,
    delay: f64,
    cfo_hz: f64,
    fs: f64,
    snr_db: f64,
    rng: &mut Rng,
) -> Vec<C> {
    let mut x = vec![(0.0, 0.0); lead];
    x.extend_from_slice(f);
    x.extend(vec![(0.0, 0.0); trail]);
    let x = if delay == 0.0 {
        x
    } else {
        fractional_delay(&x, delay)
    };
    let x = frequency_offset(&x, cfo_hz, fs, 2.0 * PI * rng.uniform());
    awgn(&x, snr_db, mean_power(f), rng)
}

fn noise(len: usize, rng: &mut Rng) -> Vec<C> {
    awgn(&vec![(0.0, 0.0); len], 0.0, 1.0, rng)
}

fn cmul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn conj(a: C) -> C {
    (a.0, -a.1)
}

fn abs2(a: C) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

fn count_detections(x: &[C], cfg: &RxConfig) -> usize {
    detections(x, cfg).len()
}

fn mean_var(v: &[f64]) -> (f64, f64) {
    let m = v.iter().sum::<f64>() / v.len() as f64;
    let var = v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64;
    (m, var)
}

// ---------------------------------------------------------------------------------------------
// The receiver's training references are the transmitter's
// ---------------------------------------------------------------------------------------------

#[test]
fn training_references_match_the_transmitted_preamble() {
    let f = preamble();
    let corr = |a: &[C], b: &[C]| {
        let c = a.iter().zip(b).fold((0.0, 0.0), |s, (x, y)| {
            let t = cmul(*x, conj(*y));
            (s.0 + t.0, s.1 + t.1)
        });
        abs2(c)
            / (a.iter().map(|s| abs2(*s)).sum::<f64>() * b.iter().map(|s| abs2(*s)).sum::<f64>())
    };
    let stf = short_training_period();
    for p in 0..10 {
        let rho = corr(&f[16 * p..16 * p + 16], &stf);
        assert!(rho > 0.9999, "STF period {p}: {rho}");
    }
    let ltf = long_training_symbol();
    assert!(corr(&f[T1..T1 + 64], &ltf) > 0.9999);
    assert!(corr(&f[T1 + 64..T1 + 128], &ltf) > 0.9999);
    assert!(
        corr(&f[160..192], &ltf[32..]) > 0.9999,
        "GI2 is the symbol's second half"
    );
}

// ---------------------------------------------------------------------------------------------
// CFO estimators against theory
// ---------------------------------------------------------------------------------------------

/// First-order variance (rad²) of ∠Σ_{k<L} r(s+k)·r*(s+k+D), r = x + complex noise of variance
/// σ², from the noise-free samples. Each noise sample n_m enters as a·n_m (first factor,
/// a = x*(m+D)) and/or b·n_m* (second factor, b = x(m−D)); its contribution to the angle is
/// Im(e^(−jθ)(a·n + b·n*)) with variance σ²/2·|α − β*|², α = e^(−jθ)a, β = e^(−jθ)b. The
/// noise×noise products add L·σ⁴/2. Divided by |A|², A the noise-free correlation.
fn angle_variance_theory(x: &[C], start: usize, len: usize, lag: usize, sigma2: f64) -> f64 {
    let a_sum = (0..len).fold((0.0, 0.0), |s, k| {
        let t = cmul(x[start + k], conj(x[start + k + lag]));
        (s.0 + t.0, s.1 + t.1)
    });
    let rot = conj((a_sum.0 / abs2(a_sum).sqrt(), a_sum.1 / abs2(a_sum).sqrt()));
    let mut lin = 0.0;
    for m in start..start + len + lag {
        let a = if m < start + len {
            conj(x[m + lag])
        } else {
            (0.0, 0.0)
        };
        let b = if m >= start + lag {
            x[m - lag]
        } else {
            (0.0, 0.0)
        };
        let (al, be) = (cmul(rot, a), cmul(rot, b));
        lin += abs2((al.0 - be.0, al.1 + be.1));
    }
    (sigma2 / 2.0 * lin + len as f64 * sigma2 * sigma2 / 2.0) / abs2(a_sum)
}

/// Cramér–Rao bound (rad²/sample²) on the frequency of a known waveform with unknown phase in
/// complex noise of variance σ²: σ² / (2·Σ (n − n̄)²|x_n|²), n̄ the energy centroid.
fn crb(x: &[C], sigma2: f64) -> f64 {
    let e: f64 = x.iter().map(|s| abs2(*s)).sum();
    let nbar = x
        .iter()
        .enumerate()
        .map(|(n, s)| n as f64 * abs2(*s))
        .sum::<f64>()
        / e;
    let j: f64 = x
        .iter()
        .enumerate()
        .map(|(n, s)| (n as f64 - nbar).powi(2) * abs2(*s))
        .sum();
    sigma2 / (2.0 * j)
}

#[test]
fn lag_correlation_cfo_is_unbiased_with_the_theoretical_variance() {
    let f = preamble();
    let p = mean_power(&f[..320]);
    let mut rng = Rng::new(0x08a1);
    let true_cfo = 37_000.0;
    // (name, start, L, D): the coarse estimator on the STF and the fine one on GI2 + LTF.
    for (name, start, len, lag) in [("STF", 16usize, 96usize, 16usize), ("LTF", 160, 96, 64)] {
        for snr_db in [6.0, 10.0, 15.0, 25.0] {
            let sigma2 = p / 10f64.powf(snr_db / 10.0);
            let clean = frequency_offset(&f[..400], true_cfo, FS, 0.0);
            let trials = 4000;
            let est: Vec<f64> = (0..trials)
                .map(|_| {
                    let x = awgn(&clean, snr_db, p, &mut rng);
                    cfo_from_correlation(lag_correlation(&x, start, len, lag), lag, FS)
                })
                .collect();
            let (m, var) = mean_var(&est);
            let hz = FS / (2.0 * PI * lag as f64);
            let theory = angle_variance_theory(&clean, start, len, lag, sigma2) * hz * hz;
            let snr = p / sigma2;
            let textbook =
                (lag as f64 * snr + len as f64 / 2.0) / ((len * len) as f64 * snr * snr) * hz * hz;
            let bound = crb(&clean[start..start + len + lag], sigma2) * (FS / (2.0 * PI)).powi(2);
            println!(
                "{name} {snr_db:>4} dB: mean {:+8.1} Hz, std {:7.1} Hz, theory {:7.1}, textbook \
                 {:7.1}, CRB {:6.1}",
                m - true_cfo,
                var.sqrt(),
                theory.sqrt(),
                textbook.sqrt(),
                bound.sqrt()
            );
            assert!(
                (m - true_cfo).abs() < 5.0 * (var / trials as f64).sqrt(),
                "{name} {snr_db} dB: biased, mean {m}"
            );
            let ratio = var / theory;
            assert!(
                (0.8..1.2).contains(&ratio),
                "{name} {snr_db} dB: variance {var:e}, theory {theory:e} (ratio {ratio:.3})"
            );
            assert!(var > bound, "{name} {snr_db} dB: below the CRB");
        }
    }
}

// ---------------------------------------------------------------------------------------------
// False alarms on noise
// ---------------------------------------------------------------------------------------------

#[test]
fn autocorrelation_false_alarm_law_on_noise() {
    // With L = D the two windows are independent and |ρ̂|² ~ Beta(1, L−1) exactly.
    let mut rng = Rng::new(0x08a2);
    let x = noise(2_000_000, &mut rng);
    let len = 16;
    let metric = autocorrelation_metric(&x, len);
    let valid = &metric[len + 16..];
    for theta in [0.3, 0.45, 0.6] {
        let hits = valid.iter().filter(|m| **m >= theta).count();
        let rate = hits as f64 / valid.len() as f64;
        let want = (1.0 - theta * theta).powi(len as i32 - 1);
        println!("L {len}, θ {theta}: rate {rate:.3e}, theory {want:.3e}");
        // Neighbouring metrics share samples, so allow for correlated counts.
        assert!(
            (rate / want - 1.0).abs() < 0.1,
            "θ {theta}: {rate:e} vs {want:e}"
        );
    }
}

#[test]
fn noise_alone_never_triggers_the_detector() {
    // ≥ 6·10⁷ samples (3 s at 20 MHz) with no detection: ≤ 1 false alarm per second at 95 %
    // confidence (rx_front.md). The other detectors' false-alarm laws are in the journal.
    let mut rng = Rng::new(0x08a3);
    let cfg = RxConfig::new(FS);
    let (mut total, mut samples) = (0, 0);
    while samples < 60_000_000 {
        let x = noise(1_000_000, &mut rng);
        total += count_detections(&x, &cfg);
        samples += x.len();
    }
    assert_eq!(total, 0, "{total} false alarms in {samples} samples");
}

// ---------------------------------------------------------------------------------------------
// Detection probability
// ---------------------------------------------------------------------------------------------

/// Fraction of frames detected within 4 µs (80 samples) of the STF start, at `snr_db`.
fn detection_rate(cfg: &RxConfig, snr_db: f64, trials: usize, seed: u64) -> f64 {
    let f = preamble();
    let mut rng = Rng::new(seed);
    let mut ok = 0;
    for _ in 0..trials {
        let lead = 200 + (rng.uniform() * 200.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let x = scene(&f, lead, 0, 0.0, cfo, cfg.sample_rate_hz, snr_db, &mut rng);
        if let Some(d) = detect(&x, 0, cfg) {
            if d >= lead && d < lead + 80 {
                ok += 1;
            }
        }
    }
    ok as f64 / trials as f64
}

#[test]
fn chosen_detector_meets_the_4_db_target() {
    let cfg = RxConfig::new(FS);
    let pd = detection_rate(&cfg, 4.0, 2000, 0x08a4);
    println!("{:?}: Pd at 4 dB = {pd:.4}", cfg.detector);
    assert!(pd >= 0.90, "Pd {pd}");
}

#[test]
fn chosen_detector_meets_the_standard_at_9_db() {
    // IEEE 802.11-2024 §17.3.10.6.1: > 90 % within 4 µs at the sensitivity level (SNR 9 dB).
    let cfg = RxConfig::new(FS);
    let pd = detection_rate(&cfg, 9.0, 1000, 0x08a5);
    println!("{:?}: Pd at 9 dB = {pd:.4}", cfg.detector);
    assert!(pd > 0.99, "Pd {pd}");
}

// ---------------------------------------------------------------------------------------------
// Acquisition against ground truth
// ---------------------------------------------------------------------------------------------

/// Combined CFO standard deviation (Hz) predicted from the coarse (STF) and fine (LTF)
/// variances combined by inverse variance, at the high-SNR limit D/(L²·SNR).
fn combined_cfo_std(snr: f64, fs: f64) -> f64 {
    let v =
        |d: f64, l: f64| (d * snr + l / 2.0) / (l * l * snr * snr) * (fs / (2.0 * PI * d)).powi(2);
    let (vc, vf) = (v(16.0, COARSE_LEN as f64), v(64.0, FINE_LEN as f64));
    // The receiver combines with the fixed high-SNR weight w, not the optimum at this SNR.
    let w: f64 = FINE_WEIGHT;
    ((1.0 - w).powi(2) * vc + w * w * vf).sqrt()
}

#[test]
fn acquires_timing_and_cfo_in_awgn() {
    let f = preamble();
    let mut rng = Rng::new(0x08a6);
    for &fs in &[20e6, 10e6, 5e6] {
        let cfg = RxConfig::new(fs);
        let b = backoff_samples(&cfg);
        assert_eq!(b, (200e-9 * fs).round() as usize);
        let max_cfo = if fs < 10e6 { 25e3 } else { 235e3 }; // TCXO ±2 ppm pair at 5 MHz
        for snr_db in [10.0, 20.0] {
            let trials = 1000;
            let mut errs = Vec::with_capacity(trials);
            for _ in 0..trials {
                // ≥ 2^7 samples ahead: the DC blocker has settled (its start-up average sees only noise).
                let lead = 200 + (rng.uniform() * 300.0) as usize;
                let delay = rng.uniform();
                let cfo = (rng.uniform() * 2.0 - 1.0) * max_cfo;
                let x = scene(&f, lead, 0, delay, cfo, fs, snr_db, &mut rng);
                let acq = acquire(&x, 0, &cfg)
                    .unwrap_or_else(|| panic!("{fs} {snr_db} dB: no acquisition"));
                let first_path = (lead + T1) as f64 + delay;
                let want = first_path - b as f64;
                assert!(
                    (acq.fft_start as f64 - want).abs() <= 1.5,
                    "{fs} {snr_db} dB: fft_start {} want {want:.1}",
                    acq.fft_start
                );
                assert_eq!(acq.ltf_start + 32, acq.fft_start + b);
                errs.push(acq.cfo_hz - cfo);
            }
            let (m, var) = mean_var(&errs);
            let theory = combined_cfo_std(10f64.powf(snr_db / 10.0), fs);
            println!(
                "fs {:>4.0} MHz {snr_db} dB: CFO error mean {m:+.1} Hz, rms {:.1} Hz, theory {theory:.1}",
                fs / 1e6,
                var.sqrt()
            );
            // An unwindowed frame through the sinc interpolator rings across field boundaries;
            // what reaches the fine window inside its guard leaves a bias of order 100 Hz at a
            // half-sample delay (journal). Required: small against the noise.
            assert!(
                m.abs() < 5.0 * (var / trials as f64).sqrt() + 0.25 * var.sqrt(),
                "biased: {m}"
            );
            let ratio = var.sqrt() / theory;
            assert!((0.85..1.2).contains(&ratio), "rms/theory {ratio:.3}");
        }
    }
}

#[test]
fn cfo_at_the_edges() {
    let f = preamble();
    let mut rng = Rng::new(0x08a7);
    // ±40 ppm at 5.885 GHz; just inside the coarse range ±fs/32; the 5 MHz coarse edge and the
    // 5 MHz fine edge ±fs/128 (where the fine estimate alone would wrap).
    for (fs, cfo) in [
        (20e6, 235e3),
        (20e6, -235e3),
        (20e6, 600e3),
        (20e6, -600e3),
        (5e6, 150e3),
        (5e6, -150e3),
        (5e6, 39e3),
        (5e6, -45e3),
    ] {
        let cfg = RxConfig::new(fs);
        for _ in 0..50 {
            let x = scene(&f, 300, 0, 0.0, cfo, fs, 20.0, &mut rng);
            let acq = acquire(&x, 0, &cfg).unwrap_or_else(|| panic!("{fs} {cfo}: none"));
            let tol = 0.01 * fs / 64.0; // 1 % of the subcarrier spacing
            assert!(
                (acq.cfo_hz - cfo).abs() < tol,
                "{fs} Hz, CFO {cfo}: estimated {}",
                acq.cfo_hz
            );
            assert_eq!(acq.fft_start, 300 + T1 - backoff_samples(&cfg));
        }
    }
}

#[test]
fn sample_clock_offset_does_not_disturb_acquisition() {
    // One oscillator: 40 ppm is both 232 kHz of CFO at 5.8 GHz and a 40 ppm sample clock.
    let f = preamble();
    let mut rng = Rng::new(0x08a8);
    let cfg = RxConfig::new(FS);
    for ppm in [40.0, -40.0] {
        let cfo = ppm * 1e-6 * 5.8e9;
        for _ in 0..50 {
            let mut x = vec![(0.0, 0.0); 300];
            x.extend_from_slice(&f);
            let x = resample(&x, ppm);
            let x = frequency_offset(&x, cfo, FS, 0.0);
            let x = awgn(&x, 15.0, mean_power(&f), &mut rng);
            let acq = acquire(&x, 0, &cfg).unwrap();
            // The LTF arrives at (300 + 192)/(1 + ε) receiver samples.
            let first_path = (300 + T1) as f64 / (1.0 + ppm * 1e-6);
            assert!((acq.fft_start as f64 - (first_path - 4.0)).abs() <= 1.5);
            assert!((acq.cfo_hz - cfo).abs() < 0.01 * FS / 64.0);
        }
    }
}

#[test]
fn truncated_stf_without_ltf_does_not_acquire() {
    let f = preamble();
    let mut rng = Rng::new(0x08a9);
    let cfg = RxConfig::new(FS);
    let mut detected = 0;
    for _ in 0..200 {
        let x = scene(&f[..160], 300, 800, 0.0, 100e3, FS, 20.0, &mut rng);
        if detect(&x, 0, &cfg).is_some() {
            detected += 1;
        }
        assert!(
            acquire(&x, 0, &cfg).is_none(),
            "acquired a frame with no LTF"
        );
    }
    assert!(
        detected > 190,
        "the STF itself should still be detected ({detected})"
    );
}

#[test]
fn timing_follows_the_first_path_through_multipath() {
    let f = preamble();
    let mut rng = Rng::new(0x08aa);
    let cfg = RxConfig::new(FS);
    let b = backoff_samples(&cfg);
    // A weaker first path and a stronger echo 6 samples later: the window covers both and the
    // FFT starts B before the first.
    let taps = [(0usize, 0.3f64.sqrt()), (6, 0.7f64.sqrt())];
    for _ in 0..100 {
        let mut x = vec![(0.0, 0.0); 300 + f.len() + 8];
        for (d, g) in taps {
            for (n, s) in f.iter().enumerate() {
                x[300 + n + d].0 += g * s.0;
                x[300 + n + d].1 += g * s.1;
            }
        }
        let x = awgn(&x, 20.0, mean_power(&f), &mut rng);
        let acq = acquire(&x, 0, &cfg).unwrap();
        assert_eq!(acq.fft_start, 300 + T1 - b, "two-path");
    }
    // TGn: the fraction of channel power whose path falls outside the ISI-free range of the
    // chosen window (tap τ is clean iff fft_start − first ∈ [τ − 16, τ]). B and D fit the
    // guard with the back-off to spare; E's tail reaches 730 ns, so some leakage is expected.
    for (model, limit) in [(TgnModel::B, 0.0), (TgnModel::D, 0.0), (TgnModel::E, 0.01)] {
        let taps = tgn_taps(model, false);
        let trials = 200;
        let mut isi = Vec::with_capacity(trials);
        for _ in 0..trials {
            let mut x = vec![(0.0, 0.0); 300];
            x.extend_from_slice(&f);
            let ch = Multipath::new(taps.clone(), Doppler::Static, 1, FS, &mut rng);
            let y = awgn(&ch.apply(&x), 20.0, mean_power(&f), &mut rng);
            let acq = acquire(&y, 0, &cfg).unwrap_or_else(|| panic!("{model:?}: none"));
            let off = acq.fft_start as f64 - (300 + T1) as f64;
            let (mut bad, mut all) = (0.0, 0.0);
            for (k, t) in taps.iter().enumerate() {
                let p = abs2(ch.gain(k, 0));
                let tau = t.delay_s * FS;
                all += p;
                if off < tau - 16.0 || off > tau {
                    bad += p;
                }
            }
            isi.push(bad / all);
        }
        let mean = isi.iter().sum::<f64>() / trials as f64;
        let clean = isi.iter().filter(|v| **v == 0.0).count() as f64 / trials as f64;
        println!("{model:?}: ISI-free in {clean:.3} of realisations, mean ISI power {mean:.2e}");
        if limit == 0.0 {
            assert!(clean >= 0.95, "{model:?}: ISI-free only in {clean}");
        } else {
            assert!(mean < limit, "{model:?}: mean ISI power {mean}");
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Front-end impairments (R0-07c)
// ---------------------------------------------------------------------------------------------

#[test]
fn dc_offset_needs_the_blocker_and_the_blocker_handles_it() {
    let mut rng = Rng::new(0x08ab);
    let x = add_tone(&noise(1_000_000, &mut rng), 10.0, 0.0, FS, 0.4); // DC 10 dB over noise
    let mut cfg = RxConfig::new(FS).with_detector(Detector::Autocorrelation);
    cfg.dc_shift = None;
    assert!(
        count_detections(&x, &cfg) > 0,
        "without a DC blocker DC correlates perfectly at lag 16"
    );
    for detector in DETECTORS {
        let cfg = RxConfig::new(FS).with_detector(detector);
        assert_eq!(
            count_detections(&x, &cfg),
            0,
            "{detector:?} with the blocker"
        );
    }
    // A DC offset that appears mid-stream (an AGC gain change moves the LO leakage): once the
    // blocker has re-converged (2000 samples ≫ 2^7) nothing more may trigger.
    let mut y = noise(400_000, &mut rng);
    let dc = add_tone(&y[200_000..], 10.0, 0.0, FS, 2.0);
    y[200_000..].copy_from_slice(&dc);
    let late = detections(&y, &RxConfig::new(FS))
        .iter()
        .filter(|&&d| d > 202_000)
        .count();
    assert_eq!(
        late, 0,
        "DC switched on mid-stream still triggers after settling"
    );
    // And frames under a DC offset as strong as the signal are acquired as without it (20 dB
    // SNR, so the 1 % tolerance is ~7σ of the CFO estimate).
    let f = preamble();
    let cfg = RxConfig::new(FS);
    for _ in 0..100 {
        let x = scene(&f, 400, 0, 0.0, 120e3, FS, 20.0, &mut rng);
        let x = add_tone(&x, mean_power(&f), 0.0, FS, 1.1);
        let acq = acquire(&x, 0, &cfg).unwrap();
        assert_eq!(acq.fft_start, 400 + T1 - 4);
        assert!(
            (acq.cfo_hz - 120e3).abs() < 0.01 * FS / 64.0,
            "{}",
            acq.cfo_hz
        );
    }
}

#[test]
fn cw_tone_robustness() {
    // A spur at 1.3 MHz. The autocorrelation detector fires on any tone above the noise
    // (metric → P_tone/(P_tone + σ²)); the LTF confirmation must turn none of that into an
    // acquisition, and packets must still be acquired under a spur below the noise.
    let mut rng = Rng::new(0x08ac);
    let x = add_tone(&noise(1_000_000, &mut rng), 2.0, 1.3e6, FS, 0.0); // +3 dB over noise
    let cfg = RxConfig::new(FS);
    assert!(
        count_detections(&x, &cfg) > 0,
        "the tone should trigger detector A"
    );
    for detector in DETECTORS {
        let c = RxConfig::new(FS).with_detector(detector);
        assert!(
            acquisitions(&x, &c).is_empty(),
            "{detector:?}: false acquisition"
        );
    }
    let f = preamble();
    let mut ok = 0;
    for _ in 0..100 {
        let x = scene(&f, 400, 0, 0.0, 150e3, FS, 20.0, &mut rng);
        // Spur 10 dB below the noise (noise power = signal / 100).
        let x = add_tone(&x, mean_power(&f) / 1000.0, 1.3e6, FS, 0.0);
        if let Some(a) = acquire(&x, 0, &cfg) {
            if a.fft_start == 400 + T1 - 4 && (a.cfo_hz - 150e3).abs() < 0.01 * FS / 64.0 {
                ok += 1;
            }
        }
    }
    assert_eq!(ok, 100, "{ok}/100 under a −10 dB spur");
    // A spur 3 dB above the noise keeps detector A firing before the packet; the LTF stage
    // must still place the packet (timing exact) rather than accept the STF or the GI2 match
    // 64 samples early. 10 dB SNR, so the spur is 7 dB below the signal. 1.25 MHz lies on the
    // STF's subcarrier grid (period 16), the hardest place for it.
    let mut placed = 0;
    for _ in 0..200 {
        let lead = 300 + (rng.uniform() * 200.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let x = scene(&f, lead, 0, 0.0, cfo, FS, 10.0, &mut rng);
        let x = add_tone(
            &x,
            mean_power(&f) / 10.0 * 2.0,
            1.25e6,
            FS,
            2.0 * PI * rng.uniform(),
        );
        if acquire(&x, 0, &cfg).is_some_and(|a| a.fft_start == lead + T1 - 4) {
            placed += 1;
        }
    }
    assert!(placed >= 190, "{placed}/200 placed under a +3 dB spur");
}

#[test]
fn gain_step_robustness() {
    let mut rng = Rng::new(0x08ad);
    let cfg = RxConfig::new(FS);
    // AGC steps in noise, with a DC offset that follows the gain: no false alarm.
    let mut x = noise(400_000, &mut rng);
    x = add_tone(&x, 1.0, 0.0, FS, 0.3);
    for (k, db) in [(100_000usize, 20.0), (200_000, -20.0), (300_000, 10.0)] {
        x = gain_step(&x, k, db);
    }
    // Detector A may fire on the DC transient after a step (the blocker re-converges); the
    // LTF stage must reject it. A real AGC would rescale the DC estimate with the step (R1/R3).
    for detector in DETECTORS {
        let c = cfg.clone().with_detector(detector);
        assert!(
            acquisitions(&x, &c).is_empty(),
            "{detector:?}: false acquisition"
        );
    }
    // A −10 dB step early in the STF (AGC settling on the packet): still acquired, CFO right.
    let f = preamble();
    for _ in 0..50 {
        let x = scene(&f, 400, 0, 0.0, -80e3, FS, 15.0, &mut rng);
        let x = gain_step(&x, 400 + 24, -10.0);
        let acq = acquire(&x, 0, &cfg).unwrap();
        assert_eq!(acq.fft_start, 400 + T1 - 4);
        assert!(
            (acq.cfo_hz + 80e3).abs() < 0.01 * FS / 64.0,
            "{}",
            acq.cfo_hz
        );
    }
}

// ---------------------------------------------------------------------------------------------
// End to end: front end + LS channel estimate + R0-10 Viterbi
// ---------------------------------------------------------------------------------------------

#[test]
fn front_end_and_viterbi_decode_frames_end_to_end() {
    let mut rng = Rng::new(0x08ae);
    let cfg = RxConfig::new(FS);
    let mut decoded = 0;
    let trials = 100;
    for t in 0..trials {
        let psdu: Vec<u8> = (0..100).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let seed = 1 + (t % 127) as u8;
        let f = frame(&psdu, Rate::R6, seed);
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let x = scene(&f, 250, 200, rng.uniform(), cfo, FS, 8.0, &mut rng);
        let Some(acq) = acquire(&x, 0, &cfg) else {
            continue;
        };
        let y = correct(&x, acq.cfo_hz, FS);
        if let Some((_, len, got)) = decode_6mbps(&y, acq.fft_start) {
            if len == psdu.len() && got == psdu {
                decoded += 1;
            }
        }
    }
    println!("6 Mb/s, 100-octet PSDU, 8 dB AWGN, CFO ±235 kHz: {decoded}/{trials} decoded");
    assert!(decoded >= 98, "{decoded}/{trials}");
}
