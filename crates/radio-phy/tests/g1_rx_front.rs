//! G1 gate for the receiver front, float reference (R0-08a, streaming since R0-08b): packet
//! detection, CFO, timing.
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
//! R0-08b adds the streaming properties: a causal DC blocker, decisions that depend only on
//! samples already seen and come within a bounded delay of the LTF, a search that closes.
//!
//! Monte-Carlo checks use fixed seeds, so every run is identical.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{
    add_tone, agc_transient, awgn, fractional_delay, frequency_offset, gain_step, mean_power,
    resample, tgn_taps, Doppler, GainState, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{
    acquire, acquisitions, autocorrelation_metric, backoff_samples, cfo_from_correlation, correct,
    dc_block, detect, detections, lag_correlation, long_training_symbol, short_training_period,
    Acquisition, Detector, RxConfig, RxFront, COARSE_LEN, DECISION_LATENCY, FINE_LEN, FINE_WEIGHT,
    GI_LEN, LTF_SYMBOL_LEN,
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

/// Standard deviation (Hz) of the fine (LTF, lag 64) CFO estimate alone: the estimate the
/// acquisition hands on (ADR 0007).
fn fine_cfo_std(snr: f64, fs: f64) -> f64 {
    let (d, l) = (64.0, FINE_LEN as f64);
    ((d * snr + l / 2.0) / (l * l * snr * snr)).sqrt() * fs / (2.0 * PI * d)
}

/// ADR 0007, decision 1. The acquisition hands on the fine estimate; the coarse one only
/// resolves its ambiguity. The weight of the coarse estimate stays a parameter.
#[test]
fn the_acquisition_hands_on_the_fine_estimate() {
    let f = preamble();
    let mut rng = Rng::new(0x08d1);
    let cfg = RxConfig::new(FS);
    let mut awgn_weight = cfg.clone();
    awgn_weight.coarse_weight = 1.0 - FINE_WEIGHT;
    assert_eq!(cfg.coarse_weight, 0.0);

    // By construction, on any scene.
    let x = scene(&f, 300, 0, 0.3, 77e3, FS, 15.0, &mut rng);
    let a = acquire(&x, 0, &cfg).unwrap();
    assert_eq!(a.cfo_hz, a.fine_cfo_hz);
    assert!(
        (a.coarse_cfo_hz - a.fine_cfo_hz).abs() > 1.0,
        "a degenerate scene"
    );
    let b = acquire(&x, 0, &awgn_weight).unwrap();
    assert_eq!(
        (b.coarse_cfo_hz, b.fine_cfo_hz),
        (a.coarse_cfo_hz, a.fine_cfo_hz)
    );
    let want = b.fine_cfo_hz + (1.0 - FINE_WEIGHT) * (b.coarse_cfo_hz - b.fine_cfo_hz);
    assert!((b.cfo_hz - want).abs() < 1e-6, "{} vs {want}", b.cfo_hz);

    // What the decision buys: a gain stage switching 16 samples into the coarse window (STF
    // sample 48) turns the carrier by 30°. Over the 96 lag-16 products, 16 see the jump: the
    // coarse estimate moves by about (16/96)·30°/(2π·16) of the sample rate ≈ 17 kHz (less,
    // as a vector sum); the fine window is untouched.
    let (mut fine2, mut comb2, trials) = (0.0, 0.0, 300);
    for _ in 0..trials {
        let lead = 200 + (rng.uniform() * 300.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 100e3;
        let x = scene(&f, lead, 0, 0.0, cfo, FS, 20.0, &mut rng);
        let jump = [GainState {
            until: lead + 48,
            gain_db: 0.0,
            phase: 30f64.to_radians(),
            dc: (0.0, 0.0),
        }];
        let x = agc_transient(&x, &jump);
        fine2 += (acquire(&x, 0, &cfg).unwrap().cfo_hz - cfo).powi(2);
        comb2 += (acquire(&x, 0, &awgn_weight).unwrap().cfo_hz - cfo).powi(2);
    }
    let (fine, comb) = (
        (fine2 / trials as f64).sqrt(),
        (comb2 / trials as f64).sqrt(),
    );
    let theory = fine_cfo_std(100.0, FS);
    println!("30° at STF sample 48, 20 dB: fine {fine:.0} Hz (theory {theory:.0}), AWGN weight {comb:.0} Hz");
    assert!(
        (0.85..1.15).contains(&(fine / theory)),
        "fine {fine} vs {theory}"
    );
    assert!(
        comb > 4.0 * fine,
        "the combination should be damaged: {comb} vs {fine}"
    );
}

/// Combined CFO standard deviation (Hz) predicted from the coarse (STF) and fine (LTF)
/// variances combined by inverse variance, at the high-SNR limit D/(L²·SNR).
fn combined_cfo_std(snr: f64, fs: f64) -> f64 {
    let v =
        |d: f64, l: f64| (d * snr + l / 2.0) / (l * l * snr * snr) * (fs / (2.0 * PI * d)).powi(2);
    let (vc, vf) = (v(16.0, COARSE_LEN as f64), v(64.0, FINE_LEN as f64));
    // The optimum weight at the high-SNR limit, derived here (not the library constant): var of
    // a lag-D estimate ∝ D/(L²·SNR)/D², so w = v_c(∞)/(v_c(∞) + v_f(∞)).
    let vinf = |d: f64, l: f64| 1.0 / (l * l * d);
    let (ic, i_f) = (vinf(16.0, COARSE_LEN as f64), vinf(64.0, FINE_LEN as f64));
    let w = ic / (ic + i_f);
    assert!(
        (w - FINE_WEIGHT).abs() < 1e-12,
        "library weight {FINE_WEIGHT} vs {w}"
    );
    ((1.0 - w).powi(2) * vc + w * w * vf).sqrt()
}

#[test]
fn acquires_timing_and_cfo_in_awgn() {
    let f = preamble();
    let mut rng = Rng::new(0x08a6);
    for &fs in &[20e6, 10e6, 5e6] {
        let cfg = RxConfig::new(fs);
        let mut cfg_w = cfg.clone();
        cfg_w.coarse_weight = 1.0 - FINE_WEIGHT;
        let b = backoff_samples(&cfg);
        assert_eq!(b, (200e-9 * fs).round() as usize);
        let max_cfo = if fs < 10e6 { 25e3 } else { 235e3 }; // TCXO ±2 ppm pair at 5 MHz
        for snr_db in [10.0, 20.0] {
            let trials = 1000;
            let mut errs = Vec::with_capacity(trials);
            let mut errs_w = Vec::with_capacity(trials);
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
                errs_w.push(acquire(&x, 0, &cfg_w).unwrap().cfo_hz - cfo);
            }
            // The coarse weight that is optimal in AWGN still meets its own theory.
            let (_, var_w) = mean_var(&errs_w);
            let ratio_w = var_w.sqrt() / combined_cfo_std(10f64.powf(snr_db / 10.0), fs);
            assert!(
                (0.92..1.08).contains(&ratio_w),
                "combined rms/theory {ratio_w:.3}"
            );
            let (m, var) = mean_var(&errs);
            let theory = fine_cfo_std(10f64.powf(snr_db / 10.0), fs);
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
            assert!((0.92..1.08).contains(&ratio), "rms/theory {ratio:.3}");
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
    let mut cfo_err = Vec::new();
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
        if let Some(a) = acquire(&x, 0, &cfg) {
            cfo_err.push(a.cfo_hz - cfo);
            if a.fft_start == lead + T1 - 4 {
                placed += 1;
            }
        }
    }
    // The spur also enters both CFO windows (7 dB under the signal) and pulls the lag
    // correlations toward its own frequency: an estimator limit, not a placement fault. It is
    // measured, not gated, here (journal open question 4; `rxfront robust` tabulates it).
    let rms = (cfo_err.iter().map(|e| e * e).sum::<f64>() / cfo_err.len() as f64).sqrt();
    println!("+3 dB spur at 1.25 MHz: {placed}/200 placed, CFO rms {rms:.0} Hz");
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
    // ±30 dB steps every 10⁵ samples with DC at the noise level: after a step down the blocker
    // still holds the 30 dB larger DC estimate, and the decaying transient keeps detector A
    // firing for hundreds of samples. The recipe and seeds of `rxfront robust` (its case 27),
    // where a non-coherent-only LTF confirmation once accepted such a transient (R0-08b).
    for t in 0..10u64 {
        let mut r = Rng::new(0xb0b0_0000 + 27 * 1000 + t);
        let x = noise(1_000_000, &mut r);
        let mut y = add_tone(&x, 1.0, 0.0, FS, 0.3);
        for (i, k) in (100_000..x.len()).step_by(100_000).enumerate() {
            y = gain_step(&y, k, if i % 2 == 0 { 30.0 } else { -30.0 });
        }
        assert!(
            acquisitions(&y, &cfg).is_empty(),
            "±30 dB steps, seed {t}: false acquisition"
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
// Streaming (R0-08b): every decision is causal, made within a bounded delay of the LTF
// ---------------------------------------------------------------------------------------------

#[test]
fn dc_blocker_is_causal() {
    // The output up to n may depend only on the input up to n: a stream cut anywhere gives the
    // same prefix, including within the start-up and on a stream that starts on a DC offset.
    let mut rng = Rng::new(0x08b1);
    let x = add_tone(&noise(4000, &mut rng), 10.0, 0.0, FS, 0.7);
    let full = dc_block(&x, 7);
    for n in [1usize, 2, 3, 17, 64, 127, 128, 129, 300, 3999] {
        assert_eq!(dc_block(&x[..n], 7), full[..n], "prefix {n}");
    }
}

#[test]
fn dc_blocker_start_up_settles_and_packets_near_the_stream_start_are_acquired() {
    // A stream that opens on a DC offset 10 dB over the noise: the start-up must not trigger an
    // acquisition, and once 2^7 samples have passed the residual DC must be small.
    let mut rng = Rng::new(0x08b2);
    let cfg = RxConfig::new(FS);
    for _ in 0..50 {
        let x = add_tone(
            &noise(3000, &mut rng),
            10.0,
            0.0,
            FS,
            2.0 * PI * rng.uniform(),
        );
        assert!(acquisitions(&x, &cfg).is_empty(), "start-up acquired");
        // Right after the start-up (samples 128..384) the residual DC must already be at the
        // noise level: the mean of 256 unit-power noise samples has power 1/256; a blocker
        // that starts at its steady-state pole from m = 0 still leaves ≈ 0.25 there.
        let y = dc_block(&x, 7);
        let m = y[128..384]
            .iter()
            .fold((0.0, 0.0), |a, s| (a.0 + s.0, a.1 + s.1));
        assert!(abs2((m.0 / 256.0, m.1 / 256.0)) < 0.05, "residual DC {m:?}");
    }
    // Packets that start 10–130 samples into the stream (the blocker still converging; for the
    // earliest the detector window is not even full) are acquired with exact timing.
    let f = preamble();
    for lead in [10usize, 40, 70, 100, 130] {
        let mut ok = 0;
        for _ in 0..50 {
            let x = scene(&f, lead, 0, 0.0, 90e3, FS, 20.0, &mut rng);
            if let Some(a) = acquire(&x, 0, &cfg) {
                if a.fft_start == lead + T1 - 4 && (a.cfo_hz - 90e3).abs() < 0.01 * FS / 64.0 {
                    ok += 1;
                }
            }
        }
        assert_eq!(ok, 50, "lead {lead}: {ok}/50");
    }
}

#[test]
fn streaming_decisions_are_causal_and_bounded() {
    // The receiver decides from the samples seen so far: cutting the stream right after the
    // decision, or replacing everything after it, gives the same acquisition. The decision rule
    // (running maximum of the energy metric, accepted once 64 + (GI − B) later positions are no
    // larger) bounds the delay: the energy at position i needs samples to i + (GI − B) − 1 + 127,
    // and the maximum sits at most GI − B before T1, so the candidate is accepted by
    // T1 + 64 + 2·(GI − B) + 127 at the latest, and its result applied decision_latency later.
    let mut rng = Rng::new(0x08b3);
    let f = preamble();
    for (fs, snr) in [(FS, 4.0), (FS, 10.0), (FS, 25.0), (10e6, 10.0), (5e6, 10.0)] {
        let cfg = RxConfig::new(fs);
        let w = GI_LEN - backoff_samples(&cfg);
        let bound = LTF_SYMBOL_LEN + 2 * w + 2 * LTF_SYMBOL_LEN - 1 + cfg.decision_latency;
        let max_cfo = if fs < 10e6 { 25e3 } else { 235e3 };
        for _ in 0..100 {
            let lead = 200 + (rng.uniform() * 300.0) as usize;
            let cfo = (rng.uniform() * 2.0 - 1.0) * max_cfo;
            let x = scene(&f, lead, 400, rng.uniform(), cfo, fs, snr, &mut rng);
            let Some(a) = acquire(&x, 0, &cfg) else {
                continue;
            };
            assert!(
                a.decided_at <= lead + T1 + bound,
                "{snr} dB: decided {} after T1",
                a.decided_at as i64 - (lead + T1) as i64
            );
            assert!(a.decided_at >= a.fft_start + 2 * LTF_SYMBOL_LEN);
            assert_eq!(acquire(&x[..=a.decided_at], 0, &cfg), Some(a), "cut");
            let mut y = x.clone();
            let tail = noise(y.len() - a.decided_at - 1, &mut rng);
            y[a.decided_at + 1..].copy_from_slice(&tail);
            assert_eq!(acquire(&y, 0, &cfg), Some(a), "tail replaced");
        }
    }
}

/// Streams that exercise the decision path: clean packets, a CW spur at and above the noise
/// (candidates failing confirmation before the packet), an AGC step down before the packet
/// (the DC transient), and two packets 0–40 samples apart (re-arm after an acquisition).
fn decision_scenes(rng: &mut Rng) -> Vec<Vec<C>> {
    let f = preamble();
    let p = mean_power(&f);
    let mut out = Vec::new();
    for k in 0..150 {
        let lead = 250 + (rng.uniform() * 300.0) as usize;
        let snr = if k % 2 == 0 { 4.0 } else { 10.0 };
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let mut f2 = f.clone();
        if k % 5 == 4 {
            // 0–40 samples: down to RIFS (2 µs), so the second packet's detections come while
            // the first packet's search window is still open.
            f2.extend(vec![(0.0, 0.0); (rng.uniform() * 40.0) as usize]);
            f2.extend_from_slice(&f);
        }
        let x = scene(&f2, lead, 400, rng.uniform(), cfo, FS, snr, rng);
        let noise_p = p * 10f64.powf(-snr / 10.0);
        let x = match k % 5 {
            1 => add_tone(&x, noise_p, (rng.uniform() * 2.0 - 1.0) * 5e6, FS, 0.3),
            2 => add_tone(
                &x,
                4.0 * noise_p,
                (rng.uniform() * 2.0 - 1.0) * 5e6,
                FS,
                0.3,
            ),
            3 => gain_step(&x, lead - 40 - (rng.uniform() * 150.0) as usize, -20.0),
            _ => x,
        };
        out.push(x);
    }
    out
}

#[test]
fn decision_latency_only_delays_the_result() {
    // Open question 13: the decision is computed from the history as of the acceptance and
    // applied decision_latency samples later while the search keeps a fresh running maximum.
    // Up to 63 + (GI − B) the acquisitions are those of latency 0, each decided exactly L
    // samples later.
    let mut rng = Rng::new(0x0d13);
    let base = RxConfig {
        decision_latency: 0,
        ..RxConfig::new(FS)
    };
    let w = GI_LEN - backoff_samples(&base);
    let mut acquired = 0;
    for (k, x) in decision_scenes(&mut rng).into_iter().enumerate() {
        let a0 = acquisitions(&x, &base);
        acquired += a0.len();
        if k % 5 == 4 && k % 2 == 1 {
            // Two packets at 10 dB: an acquisition closes its search and re-arms the detector,
            // so the second packet opens a search of its own.
            assert_eq!(a0.len(), 2, "scene {k}: {a0:?}");
            assert!(a0[1].detected_at > a0[0].decided_at, "scene {k}: {a0:?}");
        }
        for l in [1, 17, DECISION_LATENCY, LTF_SYMBOL_LEN - 1 + w] {
            let cfg = RxConfig {
                decision_latency: l,
                ..base.clone()
            };
            let shifted: Vec<_> = a0
                .iter()
                .map(|a| Acquisition {
                    decided_at: a.decided_at + l,
                    ..*a
                })
                .collect();
            assert_eq!(acquisitions(&x, &cfg), shifted, "L = {l}");
        }
    }
    assert!(acquired >= 150, "{acquired} acquisitions in 150 scenes");

    // A CW tone near the signal level keeps the detector firing after an acquisition. The run
    // counter is reset when the result is applied, L samples after the acceptance, so a search
    // opened in between may start at another sample, with another provisional latch (the
    // independent review's counterexample). The first acquisition of each stream is unchanged.
    let f = preamble();
    let p = mean_power(&f);
    let (mut moved, mut later) = (0, 0);
    for _ in 0..60 {
        let mut f2 = f.clone();
        f2.extend(vec![(0.0, 0.0); (rng.uniform() * 200.0) as usize]);
        f2.extend_from_slice(&f);
        let lead = 250 + (rng.uniform() * 300.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let x = scene(&f2, lead, 400, rng.uniform(), cfo, FS, 20.0, &mut rng);
        let x = add_tone(&x, p, (rng.uniform() * 2.0 - 1.0) * 5e6, FS, 0.3);
        let a0 = acquisitions(&x, &base);
        for l in [DECISION_LATENCY, LTF_SYMBOL_LEN - 1 + w] {
            let cfg = RxConfig {
                decision_latency: l,
                ..base.clone()
            };
            let al = acquisitions(&x, &cfg);
            if let (Some(a), Some(b)) = (al.first(), a0.first()) {
                let shifted = Acquisition {
                    decided_at: b.decided_at + l,
                    ..*b
                };
                assert_eq!(*a, shifted, "first acquisition, L = {l}");
            }
            later += a0.len().saturating_sub(1);
            moved += al
                .iter()
                .zip(&a0)
                .skip(1)
                .filter(|(a, b)| a.detected_at != b.detected_at)
                .count();
        }
    }
    assert!(later >= 20, "{later} later acquisitions");
    assert!(moved < later, "{moved} of {later} moved");
}

#[test]
fn spur_at_the_noise_level_at_the_4_db_target() {
    // A CW spur at the noise level keeps detector A firing before the packet, so the
    // provisional CFO comes from the last latch before T1 alone (no retries), and the spur
    // biases both CFO estimators (tens of kHz). The 802.11-mode target, Pd ≥ 0.90 at 4 dB, must
    // still hold for placement within ±1 sample. (The R0-08a block model, which retried from
    // every later detection, placed 332/400 of these; the first streaming draft 261.)
    let mut rng = Rng::new(0x08b6);
    let cfg = RxConfig::new(FS);
    let f = preamble();
    let trials = 400;
    let mut placed = 0;
    for _ in 0..trials {
        let lead = 300 + (rng.uniform() * 200.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let x = scene(&f, lead, 0, rng.uniform(), cfo, FS, 4.0, &mut rng);
        // Noise power = signal / 10^0.4; the spur at the same power.
        let x = add_tone(
            &x,
            mean_power(&f) / 10f64.powf(0.4),
            1.3e6,
            FS,
            2.0 * PI * rng.uniform(),
        );
        if let Some(a) = acquire(&x, 0, &cfg) {
            if a.fft_start.abs_diff(lead + T1 - 4) <= 1 {
                placed += 1;
            }
        }
    }
    println!("0 dB spur, 4 dB SNR: {placed}/{trials} placed");
    assert!(placed >= 360, "{placed}/{trials}");
}

#[test]
fn a_search_without_a_packet_closes_in_bounded_time() {
    // A truncated STF (no LTF) opens a search; with no further detection it must close once
    // its window [d + 24, d + 264) has been evaluated — position i is evaluated at sample
    // i + (GI − B) − 1 + 127 — and the receiver is idle again (power: the RTL gates the 128-tap
    // correlator while idle). A searching receiver must also be able to stay open: the window
    // moves with every detection.
    let mut rng = Rng::new(0x08b5);
    let cfg = RxConfig::new(FS);
    let w = GI_LEN - backoff_samples(&cfg);
    let f = preamble();
    for _ in 0..50 {
        let x = scene(&f[..160], 300, 1500, 0.0, 70e3, FS, 20.0, &mut rng);
        let ev = detections(&x, &cfg);
        let last = *ev.last().unwrap();
        let close_by = last + 24 + 240 + w + 126;
        let mut rx = RxFront::new(cfg.clone());
        let mut opened = None;
        for (n, &s) in x.iter().enumerate() {
            assert!(rx.push(s).is_none());
            if rx.searching() && opened.is_none() {
                opened = Some(n);
            }
            if n >= close_by {
                assert!(
                    !rx.searching(),
                    "still searching {} after the last detection",
                    n - last
                );
            }
            if opened.is_some() && n > ev[0] && n < last + 24 + 240 {
                assert!(
                    rx.searching(),
                    "closed early at {n} (last detection {last})"
                );
            }
        }
        assert_eq!(opened, Some(ev[0]), "the first detection opens the search");
    }
}

#[test]
fn streaming_receiver_acquires_back_to_back_frames() {
    // Pushed one sample at a time, the receiver re-arms after each acquisition and finds every
    // frame of a burst separated by short gaps; `acquisitions` is the same receiver.
    let mut rng = Rng::new(0x08b4);
    let cfg = RxConfig::new(FS);
    let f = preamble();
    let mut x = Vec::new();
    let mut starts = Vec::new();
    for k in 0..20 {
        let gap = 100 + (rng.uniform() * 200.0) as usize;
        x.extend(vec![(0.0, 0.0); gap]);
        starts.push(x.len());
        let fr = frequency_offset(&f, (k as f64 - 10.0) * 20e3, FS, 2.0 * PI * rng.uniform());
        x.extend(fr);
    }
    x.extend(vec![(0.0, 0.0); 300]);
    let x = awgn(&x, 15.0, mean_power(&f), &mut rng);
    let mut rx = RxFront::new(cfg.clone());
    let pushed: Vec<_> = x.iter().filter_map(|&s| rx.push(s)).collect();
    assert_eq!(pushed, acquisitions(&x, &cfg));
    let got: Vec<usize> = pushed.iter().map(|a| a.fft_start + 4 - T1).collect();
    assert_eq!(got, starts);
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

#[test]
fn a_failed_candidate_does_not_block_a_weaker_packet_after_it() {
    // Open question 15: a strong STF fragment opens a search, a strong noise burst after it
    // makes the running maximum, which fails confirmation; the real packet follows, 10 dB
    // weaker. The maximum restarts after the failed candidate, so the weaker true T1 (after
    // the acceptance point) is acquired with the right grid.
    let mut rng = Rng::new(0x0d15);
    let f = preamble();
    let stf: Vec<C> = f[..160]
        .iter()
        .map(|&(a, b)| (a * 3.16, b * 3.16))
        .collect();
    let cfg = RxConfig::new(FS);
    let b = backoff_samples(&cfg);
    let mut ok = 0;
    for _ in 0..40 {
        let lead = 200 + (rng.uniform() * 200.0) as usize;
        let burst = 120;
        let gap = 20 + (rng.uniform() * 40.0) as usize;
        let mut x = vec![(0.0, 0.0); lead];
        x.extend_from_slice(&stf);
        let p = mean_power(&f);
        x.extend(awgn(&vec![(0.0, 0.0); burst], 0.0, 10.0 * p, &mut rng));
        x.extend(vec![(0.0, 0.0); gap]);
        let start = x.len();
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); 400]);
        let x = frequency_offset(&x, 50e3, FS, 2.0 * PI * rng.uniform());
        let x = awgn(&x, 15.0, p, &mut rng);
        let acq = acquisitions(&x, &cfg);
        if acq
            .iter()
            .any(|a| a.fft_start.abs_diff(start + T1 - b) <= 1)
        {
            ok += 1;
        }
    }
    assert!(
        ok >= 38,
        "{ok}/40 packets acquired after a failed candidate"
    );
}

// ---------------------------------------------------------------------------------------------
// The DC blocker's controls at a gain change (R0-07e, candidates for ADR 0008). The reference
// is the definition worked by hand on short inputs: a consistency gate of the model, not an
// outside reference (`docs/gates/rx_front.md`).
// ---------------------------------------------------------------------------------------------

#[test]
fn dc_blocker_hold_freezes_the_estimate_and_the_gear() {
    use radio_phy::rx_front::DcBlocker;
    // Pole 2^−2. n = 0: μ = 1, y = 8, m = 8.
    let mut b = DcBlocker::new(2);
    assert_eq!(b.push((8.0, -8.0)), (8.0, -8.0));
    // Held: y = x − m with m unchanged, however many samples pass.
    b.hold(true);
    for _ in 0..5 {
        assert_eq!(b.push((16.0, 0.0)), (8.0, 8.0));
    }
    // Released: the gear resumes where it stopped (n = 1: μ = 1/2): y = 8, m = 12 (and −4);
    // n = 2: μ = 1/2: y = 4, m = 14 (−2); n = 3: μ = 1/4: y = 2, m = 14.5 (−1.5).
    b.hold(false);
    assert_eq!(b.push((16.0, 0.0)), (8.0, 8.0));
    assert_eq!(b.push((16.0, 0.0)), (4.0, 4.0));
    assert_eq!(b.push((16.0, 0.0)), (2.0, 2.0));
    assert_eq!(b.push((16.0, 0.0)), (1.5, 1.5));
}

#[test]
fn dc_blocker_rescale_moves_the_estimate_by_the_gain_step() {
    use radio_phy::rx_front::DcBlocker;
    // m = (8, −4) after the first sample. A step of −20 dB is a factor of 0.1 in amplitude:
    // m = (0.8, −0.4); a DC that fell with the gain is removed at once.
    let mut b = DcBlocker::new(7);
    b.push((8.0, -4.0));
    b.rescale(-20.0);
    let y = b.push((0.8, -0.4));
    assert!(y.0.abs() < 1e-12 && y.1.abs() < 1e-12, "{y:?}");
    // +6.0206 dB doubles it. The gear is not touched (n = 2: μ = 1/2).
    let mut b = DcBlocker::new(7);
    b.push((8.0, -4.0));
    b.push((8.0, -4.0));
    b.rescale(20.0 * 2f64.log10());
    let y = b.push((20.0, -8.0));
    assert!((y.0 - 4.0).abs() < 1e-12 && y.1.abs() < 1e-12, "{y:?}");
    let y = b.push((20.0, -8.0)); // m = 16 + 4/2 = 18
    assert!((y.0 - 2.0).abs() < 1e-12, "{y:?}");
    // A step that is not a number leaves the estimate alone.
    let mut b = DcBlocker::new(7);
    b.push((8.0, -4.0));
    b.rescale(f64::NAN);
    b.rescale(f64::INFINITY);
    b.rescale(f64::NEG_INFINITY);
    assert_eq!(b.push((8.0, -4.0)), (0.0, 0.0));
}

#[test]
fn dc_blocker_controls_combine() {
    use radio_phy::rx_front::DcBlocker;
    // Held at m = 8: a rescale by −20 dB still acts (m = 0.8), a restart does not release
    // the hold; released, the restarted gear runs (μ = 1/2: m = 0.8 + 3.6 = 4.4).
    let mut b = DcBlocker::new(7);
    b.push((8.0, 0.0));
    b.hold(true);
    b.rescale(-20.0);
    assert!((b.push((8.0, 0.0)).0 - 7.2).abs() < 1e-12);
    b.restart(1);
    assert!((b.push((8.0, 0.0)).0 - 7.2).abs() < 1e-12);
    assert!((b.push((8.0, 0.0)).0 - 7.2).abs() < 1e-12);
    b.hold(false);
    assert!((b.push((8.0, 0.0)).0 - 7.2).abs() < 1e-12);
    assert!((b.push((8.0, 0.0)).0 - 3.6).abs() < 1e-12);
}

#[test]
fn dc_blocker_restart_averages_again_from_the_present_estimate() {
    use radio_phy::rx_front::DcBlocker;
    // Pole 2^−3, run to steady state on zero, then the DC jumps to 8.
    let mut b = DcBlocker::new(3);
    for _ in 0..100 {
        b.push((0.0, 0.0));
    }
    // restart(1): μ = 1/2 for two samples (as n = 1, 2), then 1/4 for four, then 1/8.
    b.restart(1);
    let got: Vec<f64> = (0..8).map(|_| b.push((8.0, 0.0)).0).collect();
    // m: 0 → 4 → 6 → 6.5 → 6.875 → 7.15625 → 7.3671875 → (1/8) …
    let want = [
        8.0,
        4.0,
        2.0,
        1.5,
        1.125,
        0.84375,
        0.6328125,
        0.6328125 * 7.0 / 8.0,
    ];
    assert_eq!(got, want);
    // A restart beyond the steady-state pole is the steady state; restart(0) takes the
    // next sample whole.
    let mut b = DcBlocker::new(3);
    for _ in 0..100 {
        b.push((0.0, 0.0));
    }
    b.restart(u32::MAX);
    assert_eq!(b.push((8.0, 0.0)).0, 8.0);
    assert_eq!(b.push((8.0, 0.0)).0, 7.0);
    b.restart(0);
    assert_eq!(b.push((8.0, 0.0)).0, 6.125);
    assert_eq!(b.push((8.0, 0.0)).0, 0.0);
}
