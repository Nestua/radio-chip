//! G1 gate for the channel simulator, part I (R0-07a): RNG, AWGN, CFO, phase noise.
//!
//! References we did not write:
//! 1. CPython's `math.erfc`, run as a black box (`tools/ref/erfc_python.py` →
//!    `tests/fixtures/erfc_python.txt`).
//! 2. Textbook exact symbol-error rates in AWGN (Proakis, Digital Communications, §5.2): BPSK
//!    P = Q(√(2γ)); square M-QAM P = 1 − (1 − P_√M)², P_√M = 2(1 − 1/√M)·Q(√(3γ/(M−1))),
//!    γ = Es/N0. Written here, in the test, not in the library.
//! 3. The DFT shift theorem (a frequency offset of m·fs/N moves a tone by exactly m bins).
//! 4. The spectrum of a Wiener-phase oscillator: e^(jφ) has autocorrelation ρ^|k|,
//!    ρ = e^(−πβ/fs), so its PSD is Lorentzian (exact sampled form used below).
//!
//! Every Monte-Carlo check uses a fixed seed and a ±5σ bound, so it is deterministic and a
//! failure means a real deviation, not bad luck.

use radio_phy::channel::{
    awgn, erfc, frequency_offset, linewidth_from_dbc, mean_power, phase_noise, ppm_to_hz, q,
    wiener_phase, Rng,
};
use radio_phy::mapper::{map_group, mean_energy};
use radio_phy::modulation::Modulation;
use std::f64::consts::PI;

type C = (f64, f64);

const ERFC: &str = include_str!("fixtures/erfc_python.txt");

#[test]
fn erfc_matches_cpython() {
    let (mut n, mut worst) = (0, 0.0f64);
    for line in ERFC.lines().filter(|l| !l.starts_with('#')) {
        let (x, want) = line.split_once(' ').unwrap();
        let (x, want): (f64, f64) = (x.parse().unwrap(), want.parse().unwrap());
        let got = erfc(x);
        let rel = ((got - want) / want).abs();
        assert!(
            rel <= 1e-13,
            "erfc({x}) = {got:e}, CPython {want:e}, rel {rel:e}"
        );
        worst = worst.max(rel);
        n += 1;
    }
    println!("erfc: {n} points, worst relative error {worst:e}");
    assert_eq!(n, 697);
    // Q(x) = erfc(x/√2)/2; Q(0) = ½ exactly.
    assert_eq!(q(0.0), 0.5);
    assert!((q(3.0) - 0.5 * erfc(3.0 / 2f64.sqrt())).abs() < 1e-18);
}

fn moments(xs: &[f64]) -> (f64, f64, f64, f64) {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    let c = |p: i32| xs.iter().map(|x| (x - m).powi(p)).sum::<f64>() / n;
    let v = c(2);
    (m, v, c(3) / v.powf(1.5), c(4) / (v * v))
}

#[test]
fn rng_is_deterministic_and_uniform() {
    let take = |seed| {
        let mut r = Rng::new(seed);
        (0..8).map(|_| r.next_u64()).collect::<Vec<_>>()
    };
    assert_eq!(take(7), take(7));
    assert_ne!(take(7), take(8));
    let mut r = Rng::new(1);
    let n = 400_000;
    let u: Vec<f64> = (0..n).map(|_| r.uniform()).collect();
    assert!(u.iter().all(|&x| (0.0..1.0).contains(&x)));
    let (m, v, _, _) = moments(&u);
    let s = (n as f64).sqrt();
    assert!(
        (m - 0.5).abs() < 5.0 * (1.0 / 12f64).sqrt() / s,
        "uniform mean {m}"
    );
    assert!(
        (v - 1.0 / 12.0).abs() < 5.0 * 0.0745 / s,
        "uniform variance {v}"
    );
    // 16 equal bins: chi-square with 15 degrees of freedom, far tail at 50.
    let mut bins = [0usize; 16];
    for &x in &u {
        bins[(x * 16.0) as usize] += 1;
    }
    let e = n as f64 / 16.0;
    let chi2: f64 = bins.iter().map(|&b| (b as f64 - e).powi(2) / e).sum();
    assert!(chi2 < 50.0, "chi-square {chi2}");
}

#[test]
fn gaussian_moments_and_whiteness() {
    let mut r = Rng::new(2);
    let n = 400_000;
    let g: Vec<f64> = (0..n).map(|_| r.gaussian()).collect();
    let (m, v, skew, kurt) = moments(&g);
    let s = (n as f64).sqrt();
    // Standard errors: mean 1/√n, variance √2/√n, skew √6/√n, kurtosis √24/√n.
    assert!(m.abs() < 5.0 / s, "mean {m}");
    assert!((v - 1.0).abs() < 5.0 * 2f64.sqrt() / s, "variance {v}");
    assert!(skew.abs() < 5.0 * 6f64.sqrt() / s, "skew {skew}");
    assert!(
        (kurt - 3.0).abs() < 5.0 * 24f64.sqrt() / s,
        "kurtosis {kurt}"
    );
    let lag1 = g.windows(2).map(|w| w[0] * w[1]).sum::<f64>() / n as f64;
    assert!(lag1.abs() < 5.0 / s, "lag-1 correlation {lag1}");
    // Tail: P(|x| > 3) = erfc(3/√2) = 0.0027.
    let p = 2.0 * q(3.0);
    let k = g.iter().filter(|x| x.abs() > 3.0).count() as f64;
    let sd = (n as f64 * p * (1.0 - p)).sqrt();
    assert!((k - n as f64 * p).abs() < 5.0 * sd, "tail count {k}");
}

#[test]
fn awgn_has_the_requested_power_split_evenly() {
    let mut r = Rng::new(3);
    let n = 200_000;
    let x: Vec<C> = vec![(0.0, 0.0); n];
    for snr_db in [-5.0, 0.0, 10.0, 30.0] {
        let p_sig = 0.25;
        let y = awgn(&x, snr_db, p_sig, &mut r);
        let want = p_sig / 10f64.powf(snr_db / 10.0);
        let (pi, pq) = (
            y.iter().map(|s| s.0 * s.0).sum::<f64>() / n as f64,
            y.iter().map(|s| s.1 * s.1).sum::<f64>() / n as f64,
        );
        // Each axis: χ² with n degrees, relative sd √(2/n).
        let tol = 5.0 * (2.0 / n as f64).sqrt();
        assert!(
            (pi / (want / 2.0) - 1.0).abs() < tol,
            "{snr_db} dB: I power {pi}"
        );
        assert!(
            (pq / (want / 2.0) - 1.0).abs() < tol,
            "{snr_db} dB: Q power {pq}"
        );
        let iq = y.iter().map(|s| s.0 * s.1).sum::<f64>() / n as f64 / (want / 2.0);
        assert!(iq.abs() < 5.0 / (n as f64).sqrt(), "I/Q correlated: {iq}");
    }
    // Noise is added, not substituted: a signal survives on average.
    let x: Vec<C> = (0..n)
        .map(|k| (if k % 2 == 0 { 1.0 } else { -1.0 }, 0.5))
        .collect();
    let y = awgn(&x, 10.0, mean_power(&x), &mut r);
    let d: Vec<f64> = x.iter().zip(&y).map(|(a, b)| b.0 - a.0).collect();
    let (m, _, _, _) = moments(&d);
    assert!(m.abs() < 5.0 * 0.125f64.sqrt() / (n as f64).sqrt());
    assert!((mean_power(&x) - 1.25).abs() < 1e-12);
}

/// Every point of the normalised constellation.
fn constellation(m: Modulation) -> Vec<C> {
    let n = m.bits_per_subcarrier();
    let k = 1.0 / f64::from(mean_energy(m)).sqrt();
    (0..1usize << n)
        .map(|v| {
            let bits: Vec<u8> = (0..n).map(|i| ((v >> i) & 1) as u8).collect();
            let p = map_group(&bits, m).unwrap_or_else(|e| panic!("{e:?}"));
            (f64::from(p.i) * k, f64::from(p.q) * k)
        })
        .collect()
}

fn ser_theory(m: Modulation, es_n0: f64) -> f64 {
    match m {
        Modulation::Bpsk => 0.5 * erfc_ref(es_n0.sqrt()),
        _ => {
            let big_m = f64::from(1u32 << m.bits_per_subcarrier());
            let a = 2.0 * (1.0 - 1.0 / big_m.sqrt()) * q_ref((3.0 * es_n0 / (big_m - 1.0)).sqrt());
            1.0 - (1.0 - a).powi(2)
        }
    }
}

// The theory uses the fixture-checked erfc; Q(√(2γ)) = erfc(√γ)/2.
fn erfc_ref(x: f64) -> f64 {
    erfc(x)
}
fn q_ref(x: f64) -> f64 {
    0.5 * erfc(x / 2f64.sqrt())
}

#[test]
fn symbol_error_rate_matches_theory() {
    let mut r = Rng::new(4);
    for (m, points_db) in [
        (Modulation::Bpsk, [0.0, 4.0, 7.0]),
        (Modulation::Qpsk, [3.0, 7.0, 10.0]),
        (Modulation::Qam16, [10.0, 14.0, 17.0]),
        (Modulation::Qam64, [16.0, 20.0, 23.0]),
    ] {
        let points = constellation(m);
        for es_n0_db in points_db {
            let p = ser_theory(m, 10f64.powf(es_n0_db / 10.0));
            // Enough symbols for ~2000 expected errors (at most 400k).
            let n = ((2000.0 / p) as usize).clamp(20_000, 400_000);
            let tx: Vec<usize> = (0..n)
                .map(|_| (r.next_u64() % points.len() as u64) as usize)
                .collect();
            let x: Vec<C> = tx.iter().map(|&i| points[i]).collect();
            let y = awgn(&x, es_n0_db, 1.0, &mut r);
            let errors = tx
                .iter()
                .zip(&y)
                .filter(|(&i, s)| {
                    let d = |p: &C| (p.0 - s.0).powi(2) + (p.1 - s.1).powi(2);
                    let best = (0..points.len())
                        .min_by(|&a, &b| d(&points[a]).total_cmp(&d(&points[b])))
                        .unwrap();
                    best != i
                })
                .count() as f64;
            let sd = (n as f64 * p * (1.0 - p)).sqrt();
            println!(
                "{m:?} {es_n0_db:>4} dB: {errors} errors / {n}, theory {:.1} ({:+.2} σ)",
                n as f64 * p,
                (errors - n as f64 * p) / sd
            );
            assert!(
                (errors - n as f64 * p).abs() < 5.0 * sd,
                "{m:?} at {es_n0_db} dB: {errors} errors in {n}, theory {:.1}",
                n as f64 * p
            );
        }
    }
}

fn dft_bin(x: &[C], k: usize) -> C {
    let n = x.len();
    x.iter().enumerate().fold((0.0, 0.0), |acc, (t, s)| {
        let a = -2.0 * PI * ((k * t) % n) as f64 / n as f64;
        (
            acc.0 + s.0 * a.cos() - s.1 * a.sin(),
            acc.1 + s.0 * a.sin() + s.1 * a.cos(),
        )
    })
}

#[test]
fn frequency_offset_follows_the_shift_theorem() {
    let fs = 20e6;
    let n = 64;
    let tone: Vec<C> = (0..n)
        .map(|t| {
            let a = 2.0 * PI * 5.0 * t as f64 / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    for m in [-7i32, 3, 11] {
        let y = frequency_offset(&tone, f64::from(m) * fs / n as f64, fs, 0.0);
        let target = (5 + m).rem_euclid(n as i32) as usize;
        for k in 0..n {
            let (re, im) = dft_bin(&y, k);
            let mag = (re * re + im * im).sqrt();
            if k == target {
                assert!((mag - n as f64).abs() < 1e-9, "offset {m}: bin {k} = {mag}");
            } else {
                assert!(mag < 1e-9, "offset {m}: leakage into bin {k} = {mag}");
            }
        }
    }
    // The initial phase is applied at sample 0, and the rotation preserves power exactly.
    let y = frequency_offset(&[(1.0, 0.0); 4], 0.0, fs, PI / 2.0);
    assert!(y[0].0.abs() < 1e-15 && (y[0].1 - 1.0).abs() < 1e-15);
    let y = frequency_offset(&tone, 123_456.7, fs, 0.3);
    assert!((mean_power(&y) - 1.0).abs() < 1e-12);
    // Phase advances 2π·Δf/fs per sample.
    let y = frequency_offset(&[(1.0, 0.0); 2], 1e6, fs, 0.0);
    assert!((y[1].1.atan2(y[1].0) - 2.0 * PI * 1e6 / fs).abs() < 1e-15);
    // §17.3.9.5: 20 ppm of 5.8 GHz is 116 kHz.
    assert!((ppm_to_hz(20.0, 5.8e9) - 116e3).abs() < 1e-6);
}

#[test]
fn wiener_phase_variance_grows_linearly() {
    let fs = 20e6;
    let beta = 1000.0;
    let mut r = Rng::new(5);
    let (runs, len) = (4000, 2000);
    let mut end = Vec::with_capacity(runs);
    let mut mid = Vec::with_capacity(runs);
    for _ in 0..runs {
        let phi = wiener_phase(len, beta, fs, &mut r);
        assert_eq!(phi.len(), len);
        mid.push(phi[len / 2 - 1]);
        end.push(phi[len - 1]);
    }
    for (samples, steps) in [(&mid, len / 2), (&end, len)] {
        let (_, v, _, _) = moments(samples);
        let want = 2.0 * PI * beta * steps as f64 / fs;
        let tol = 5.0 * (2.0 / runs as f64).sqrt();
        assert!(
            (v / want - 1.0).abs() < tol,
            "var after {steps}: {v:e} vs {want:e}"
        );
    }
}

#[test]
fn phase_noise_spectrum_is_lorentzian() {
    let fs = 20e6;
    // −80 dBc/Hz at 100 kHz: β = 2π·(1e5)²·1e-8 = 628.3 Hz.
    let beta = linewidth_from_dbc(-80.0, 100e3);
    assert!((beta - 2.0 * PI * 100.0).abs() < 1e-9, "β = {beta}");
    let rho = (-PI * beta / fs).exp();
    let psd =
        |f: f64| (1.0 - rho * rho) / (1.0 - 2.0 * rho * (2.0 * PI * f / fs).cos() + rho * rho) / fs;
    // Welch: Hann-windowed segments (sidelobes far below the skirt), periodogram averaged.
    let (seg, segs) = (4096usize, 300);
    let w: Vec<f64> = (0..seg)
        .map(|t| 0.5 - 0.5 * (2.0 * PI * t as f64 / seg as f64).cos())
        .collect();
    let u = w.iter().map(|v| v * v).sum::<f64>();
    let bins = [21usize, 41, 82, 164]; // ≈ 100, 200, 400, 800 kHz
    let mut acc = vec![0.0; bins.len()];
    let mut r = Rng::new(6);
    let ones = vec![(1.0, 0.0); seg];
    for _ in 0..segs {
        let y = phase_noise(&ones, beta, fs, &mut r);
        let yw: Vec<C> = y.iter().zip(&w).map(|(s, w)| (s.0 * w, s.1 * w)).collect();
        for (a, &k) in acc.iter_mut().zip(&bins) {
            // Average the bin and its neighbours for a smoother estimate.
            for kk in [k - 1, k, k + 1] {
                let (re, im) = dft_bin(&yw, kk);
                *a += (re * re + im * im) / (u * fs) / 3.0;
            }
        }
    }
    for (a, &k) in acc.iter().zip(&bins) {
        let f = k as f64 * fs / seg as f64;
        let ours = 10.0 * (a / segs as f64).log10();
        let theory = 10.0 * psd(f).log10();
        println!(
            "{:>4.0} kHz: {ours:.2} dBc/Hz, Lorentzian {theory:.2}",
            f / 1e3
        );
        assert!(
            (ours - theory).abs() < 0.6,
            "{:.0} kHz: {ours:.2} dBc/Hz, Lorentzian {theory:.2}",
            f / 1e3
        );
    }
    // Far from the carrier the SSB level is the datasheet figure: −80 dBc/Hz at 100 kHz.
    assert!((10.0 * psd(100e3).log10() + 80.0).abs() < 0.05);
}

#[test]
fn ofdm_per_subcarrier_snr_is_snr_plus_64_over_52() {
    use radio_phy::ofdm::{data_subcarriers, fft_bin, modulate_symbol_f64, CP_LEN, FFT_SIZE};
    let m = Modulation::Qam16;
    let mut r = Rng::new(7);
    let mut tx_syms = Vec::new();
    let mut frame = Vec::new();
    for s in 0..1500 {
        let data: Vec<_> = (0..48)
            .map(|_| {
                let bits: Vec<u8> = (0..4).map(|_| (r.next_u64() & 1) as u8).collect();
                map_group(&bits, m).unwrap()
            })
            .collect();
        frame.extend(modulate_symbol_f64(&data, m, s % 127).unwrap());
        tx_syms.push(frame.len());
    }
    let snr_db = 12.0;
    let y = awgn(&frame, snr_db, mean_power(&frame), &mut r);
    let (mut sig, mut err) = (0.0, 0.0);
    for &end in &tx_syms {
        let start = end - FFT_SIZE;
        assert_eq!((end - start) + CP_LEN, 80);
        for &c in &data_subcarriers() {
            let k = fft_bin(c);
            let a = dft_bin(&frame[start..end], k);
            let b = dft_bin(&y[start..end], k);
            sig += a.0 * a.0 + a.1 * a.1;
            err += (b.0 - a.0).powi(2) + (b.1 - a.1).powi(2);
        }
    }
    let measured = 10.0 * (sig / err).log10();
    let want = snr_db + 10.0 * (64.0f64 / 52.0).log10();
    println!("per-subcarrier SNR {measured:.3} dB, expected {want:.3}");
    assert!(
        (measured - want).abs() < 0.1,
        "per-subcarrier SNR {measured:.3} dB, want {want:.3}"
    );
}
