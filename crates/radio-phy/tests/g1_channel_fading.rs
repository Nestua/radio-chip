//! G1 gate for the channel simulator, part II (R0-07b): FFT, fractional delay, SFO, fading taps,
//! TGn multipath.
//!
//! References we did not write:
//! 1. TGn channel models, IEEE 802.11-03/940r4 Appendix C, extracted by
//!    `tools/ref/tgn_pdp_extract.py` → `tests/fixtures/tgn_pdp.txt`; Table II K-factors.
//! 2. The Fourier shift theorem: a band-limited signal delayed or resampled by any real amount is
//!    known exactly from its spectrum.
//! 3. Rayleigh/Rician moments (E|g|² = 1, E|g|⁴ = (2 + 4K + K²)/(1 + K)²), Clarke's
//!    autocorrelation J0(2π·fd·τ) (J0 checked against Abramowitz & Stegun Table 9.1), the
//!    cosine transform of the TGn Bell spectrum (numerical integration here).
//! 4. BPSK in flat Rayleigh fading with coherent detection: P = ½·(1 − √(γ/(1 + γ)))
//!    (Proakis, Digital Communications, eq. 14.3-7).
//!
//! Monte-Carlo checks use fixed seeds; statistics of correlated fading are judged across
//! independent realisations (mean ± 5 standard errors of the per-realisation estimates).

use radio_phy::channel::{
    fft, fractional_delay, ifft, resample, tgn_taps, Doppler, Multipath, Rng, Sample, Tap, TgnModel,
};
use std::f64::consts::PI;

const TGN: &str = include_str!("fixtures/tgn_pdp.txt");

fn dft(x: &[Sample]) -> Vec<Sample> {
    let n = x.len();
    (0..n)
        .map(|k| {
            x.iter().enumerate().fold((0.0, 0.0), |a, (t, s)| {
                let w = -2.0 * PI * ((k * t) % n) as f64 / n as f64;
                (
                    a.0 + s.0 * w.cos() - s.1 * w.sin(),
                    a.1 + s.0 * w.sin() + s.1 * w.cos(),
                )
            })
        })
        .collect()
}

fn err_db(reference: &[Sample], got: &[Sample]) -> f64 {
    let p: f64 = reference.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
    let e: f64 = reference
        .iter()
        .zip(got)
        .map(|(a, b)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2))
        .sum();
    10.0 * (e / p).log10()
}

fn random_signal(n: usize, r: &mut Rng) -> Vec<Sample> {
    (0..n).map(|_| (r.gaussian(), r.gaussian())).collect()
}

#[test]
fn fft_matches_the_dft_definition() {
    let mut r = Rng::new(10);
    for log in 0..=9 {
        let x = random_signal(1 << log, &mut r);
        let ours = fft(&x).unwrap();
        assert!(err_db(&dft(&x), &ours) < -250.0, "N = {}", x.len());
        let back = ifft(&ours).unwrap();
        assert!(err_db(&x, &back) < -250.0, "round trip N = {}", x.len());
    }
    assert!(fft(&[(0.0, 0.0); 48]).is_err());
}

/// 64-periodic OFDM-like signal on the 52 occupied bins, and its exact value at real time t.
struct Ofdm {
    bins: Vec<(i32, Sample)>,
}

impl Ofdm {
    fn new(r: &mut Rng) -> Self {
        let bins = (-26..=26)
            .filter(|&k| k != 0)
            .map(|k| (k, (r.gaussian(), r.gaussian())))
            .collect();
        Self { bins }
    }
    fn at(&self, t: f64) -> Sample {
        self.bins.iter().fold((0.0, 0.0), |a, &(k, x)| {
            let w = 2.0 * PI * f64::from(k) * t / 64.0;
            (
                a.0 + x.0 * w.cos() - x.1 * w.sin(),
                a.1 + x.0 * w.sin() + x.1 * w.cos(),
            )
        })
    }
}

#[test]
fn fractional_delay_obeys_the_shift_theorem() {
    let mut r = Rng::new(11);
    let s = Ofdm::new(&mut r);
    let n = 640;
    let x: Vec<Sample> = (0..n).map(|t| s.at(t as f64)).collect();
    for d in [0.2, 0.5, 0.73, 3.4, 17.9] {
        let y = fractional_delay(&x, d);
        assert_eq!(y.len(), n);
        // Away from the edges (filter half-length 24 plus the delay) the result is exact.
        let lo = 24 + d.ceil() as usize;
        let reference: Vec<Sample> = (lo..n - 24).map(|t| s.at(t as f64 - d)).collect();
        let e = err_db(&reference, &y[lo..n - 24]);
        println!("fractional delay {d}: error {e:.1} dB");
        assert!(e < -50.0, "delay {d}: {e:.1} dB");
    }
    // Integer delays are pure shifts.
    let y = fractional_delay(&x, 3.0);
    assert!(err_db(&x[..n - 3], &y[3..]) < -250.0);
    assert!(y[..3]
        .iter()
        .all(|s| s.0.abs() < 1e-12 && s.1.abs() < 1e-12));
}

#[test]
fn resampling_follows_the_clock_offset() {
    let mut r = Rng::new(12);
    let s = Ofdm::new(&mut r);
    let n = 4000;
    let x: Vec<Sample> = (0..n).map(|t| s.at(t as f64)).collect();
    for ppm in [-40.0, 20.0, 1000.0] {
        let y = resample(&x, ppm);
        assert_eq!(y.len(), n);
        let eps = ppm * 1e-6;
        let hi = ((n - 25) as f64 / (1.0 + eps)) as usize;
        let reference: Vec<Sample> = (24..hi).map(|t| s.at(t as f64 * (1.0 + eps))).collect();
        let e = err_db(&reference, &y[24..hi]);
        println!("resample {ppm} ppm: error {e:.1} dB");
        assert!(e < -50.0, "{ppm} ppm: {e:.1} dB");
    }
}

/// J0 by its power series (exact enough for |x| ≤ 20 in f64).
fn bessel_j0(x: f64) -> f64 {
    let (mut term, mut sum) = (1.0f64, 1.0f64);
    let q = -x * x / 4.0;
    for k in 1..80 {
        term *= q / f64::from(k * k);
        sum += term;
    }
    sum
}

#[test]
fn bessel_j0_matches_abramowitz_stegun() {
    // A&S Table 9.1 and the first zero j0,1 = 2.404825557695773.
    for (x, j) in [
        (1.0, 0.765_197_686_6),
        (2.0, 0.223_890_779_1),
        (5.0, -0.177_596_771_3),
    ] {
        assert!((bessel_j0(x) - j).abs() < 1e-9, "J0({x})");
    }
    assert!(bessel_j0(2.404_825_557_695_773).abs() < 1e-14);
}

/// Per-realisation estimates → (mean, standard error).
fn ensemble(v: &[f64]) -> (f64, f64) {
    let n = v.len() as f64;
    let m = v.iter().sum::<f64>() / n;
    let var = v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (n - 1.0);
    (m, (var / n).sqrt())
}

fn assert_ensemble(what: &str, v: &[f64], want: f64) {
    let (m, se) = ensemble(v);
    println!("{what}: {m:.4} ± {se:.4} (theory {want:.4})");
    assert!(
        (m - want).abs() < 5.0 * se.max(1e-6),
        "{what}: {m} ± {se}, theory {want}"
    );
}

const FS: f64 = 10e3;

fn one_tap(k: f64, los_doppler_hz: f64) -> Vec<Tap> {
    vec![Tap {
        delay_s: 0.0,
        power: 1.0,
        k_factor: k,
        los_doppler_hz,
    }]
}

#[test]
fn rayleigh_tap_has_exponential_power() {
    let mut r = Rng::new(13);
    let (runs, len) = (200, 20_000);
    let (mut p1, mut c01, mut c1) = (vec![], vec![], vec![]);
    for _ in 0..runs {
        let ch = Multipath::new(
            one_tap(0.0, 0.0),
            Doppler::Jakes { fd_hz: 100.0 },
            len,
            FS,
            &mut r,
        );
        let p: Vec<f64> = (0..len)
            .map(|n| {
                let g = ch.gain(0, n);
                g.0 * g.0 + g.1 * g.1
            })
            .collect();
        p1.push(p.iter().sum::<f64>() / len as f64);
        c01.push(p.iter().filter(|&&x| x < 0.1).count() as f64 / len as f64);
        c1.push(p.iter().filter(|&&x| x < 1.0).count() as f64 / len as f64);
    }
    assert_ensemble("Rayleigh E|g|²", &p1, 1.0);
    assert_ensemble("Rayleigh P(|g|² < 0.1)", &c01, 1.0 - (-0.1f64).exp());
    assert_ensemble("Rayleigh P(|g|² < 1)", &c1, 1.0 - (-1.0f64).exp());
}

#[test]
fn rician_tap_moments_and_los_component() {
    let mut r = Rng::new(14);
    let k = 4.0;
    let f_los = 30.0;
    let (runs, len) = (200, 20_000);
    let (mut m2, mut m4, mut los) = (vec![], vec![], vec![]);
    for _ in 0..runs {
        let ch = Multipath::new(
            one_tap(k, f_los),
            Doppler::Jakes { fd_hz: 100.0 },
            len,
            FS,
            &mut r,
        );
        let g: Vec<Sample> = (0..len).map(|n| ch.gain(0, n)).collect();
        let p: Vec<f64> = g.iter().map(|g| g.0 * g.0 + g.1 * g.1).collect();
        m2.push(p.iter().sum::<f64>() / len as f64);
        m4.push(p.iter().map(|x| x * x).sum::<f64>() / len as f64);
        // Undo the LOS Doppler: the mean is the LOS phasor, |·| = √(K/(K+1)).
        let (mut re, mut im) = (0.0, 0.0);
        for (n, g) in g.iter().enumerate() {
            let w = -2.0 * PI * f_los * n as f64 / FS;
            re += g.0 * w.cos() - g.1 * w.sin();
            im += g.0 * w.sin() + g.1 * w.cos();
        }
        los.push((re * re + im * im).sqrt() / len as f64);
    }
    assert_ensemble("Rician E|g|²", &m2, 1.0);
    assert_ensemble(
        "Rician E|g|⁴",
        &m4,
        (2.0 + 4.0 * k + k * k) / (1.0 + k).powi(2),
    );
    // |mean| is biased up by the residual scatter (≈ (1/(K+1))/N_eff), negligible here.
    let (m, _) = ensemble(&los);
    let want = (k / (k + 1.0)).sqrt();
    println!("Rician |LOS| {m:.4} (theory {want:.4})");
    assert!((m - want).abs() < 0.01);
}

fn acf(runs: usize, len: usize, doppler: Doppler, lags: &[usize], seed: u64) -> Vec<f64> {
    let mut r = Rng::new(seed);
    let mut acc = vec![0.0; lags.len()];
    let mut p0 = 0.0;
    for _ in 0..runs {
        let ch = Multipath::new(one_tap(0.0, 0.0), doppler, len, FS, &mut r);
        let g: Vec<Sample> = (0..len).map(|n| ch.gain(0, n)).collect();
        let span = len - lags.iter().max().copied().unwrap_or(0);
        p0 += g[..span].iter().map(|g| g.0 * g.0 + g.1 * g.1).sum::<f64>();
        for (a, &l) in acc.iter_mut().zip(lags) {
            *a += (0..span)
                .map(|n| g[n].0 * g[n + l].0 + g[n].1 * g[n + l].1)
                .sum::<f64>();
        }
    }
    acc.iter().map(|a| a / p0).collect()
}

#[test]
fn jakes_autocorrelation_is_j0() {
    let fd = 50.0;
    let taus = [0.1, 0.25, 0.5, 1.0, 2.0]; // in units of 1/fd
    let lags: Vec<usize> = taus
        .iter()
        .map(|t| (t / fd * FS).round() as usize)
        .collect();
    let got = acf(300, 4000, Doppler::Jakes { fd_hz: fd }, &lags, 15);
    for ((t, l), g) in taus.iter().zip(&lags).zip(&got) {
        let want = bessel_j0(2.0 * PI * fd * *l as f64 / FS);
        println!("Jakes ACF fd·τ = {t}: {g:.4}, J0 {want:.4}");
        assert!((g - want).abs() < 0.03, "fd·τ = {t}: {g} vs {want}");
    }
}

#[test]
fn bell_autocorrelation_matches_its_spectrum() {
    let fd = 50.0;
    // Cosine transform of S(f) = 1/(1 + 9(f/fd)²) on |f| ≤ 5 fd, Simpson's rule.
    let bell_acf = |tau: f64| {
        let m = 20_000;
        let h = 10.0 * fd / m as f64;
        let s = |f: f64| 1.0 / (1.0 + 9.0 * (f / fd).powi(2));
        let integrate = |g: &dyn Fn(f64) -> f64| {
            (0..=m)
                .map(|i| {
                    let f = -5.0 * fd + i as f64 * h;
                    let w = if i == 0 || i == m {
                        1.0
                    } else if i % 2 == 1 {
                        4.0
                    } else {
                        2.0
                    };
                    w * g(f)
                })
                .sum::<f64>()
                * h
                / 3.0
        };
        integrate(&|f| s(f) * (2.0 * PI * f * tau).cos()) / integrate(&s)
    };
    let taus = [0.05, 0.1, 0.2, 0.5, 1.0];
    let lags: Vec<usize> = taus
        .iter()
        .map(|t| (t / fd * FS).round() as usize)
        .collect();
    let got = acf(300, 4000, Doppler::Bell { fd_hz: fd }, &lags, 16);
    for ((t, l), g) in taus.iter().zip(&lags).zip(&got) {
        let want = bell_acf(*l as f64 / FS);
        println!("Bell ACF fd·τ = {t}: {g:.4}, spectrum {want:.4}");
        assert!((g - want).abs() < 0.03, "fd·τ = {t}: {g} vs {want}");
    }
}

#[test]
fn tgn_tables_match_the_published_document() {
    let mut delays = std::collections::BTreeMap::new();
    let mut powers: std::collections::BTreeMap<String, Vec<f64>> = Default::default();
    for line in TGN.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = line.split_whitespace().collect();
        match f[0] {
            "delays" => {
                let d: Vec<f64> = f[2..]
                    .iter()
                    .map(|v| v.parse::<f64>().unwrap() * 1e-9)
                    .collect();
                powers.insert(f[1].to_string(), vec![0.0; d.len()]);
                delays.insert(f[1].to_string(), d);
            }
            "cluster" => {
                let start: usize = f[3].parse().unwrap();
                let p = powers.get_mut(f[1]).unwrap();
                for (j, v) in f[4..].iter().enumerate() {
                    p[start + j] += 10f64.powf(v.parse::<f64>().unwrap() / 10.0);
                }
            }
            k => panic!("{k}"),
        }
    }
    let nominal_ns = [
        ("B", 15.0),
        ("C", 30.0),
        ("D", 50.0),
        ("E", 100.0),
        ("F", 150.0),
    ];
    for (model, (name, rms_nominal)) in [
        TgnModel::B,
        TgnModel::C,
        TgnModel::D,
        TgnModel::E,
        TgnModel::F,
    ]
    .into_iter()
    .zip(nominal_ns)
    {
        let taps = tgn_taps(model, false);
        let total: f64 = powers[name].iter().sum();
        assert_eq!(taps.len(), delays[name].len(), "model {name}");
        for ((t, d), p) in taps.iter().zip(&delays[name]).zip(&powers[name]) {
            assert!((t.delay_s - d).abs() < 1e-15, "model {name} delay");
            assert!((t.power - p / total).abs() < 1e-12, "model {name} at {d}");
            assert_eq!(t.k_factor, 0.0);
        }
        let mean: f64 = taps.iter().map(|t| t.power * t.delay_s).sum();
        let rms = taps
            .iter()
            .map(|t| t.power * (t.delay_s - mean).powi(2))
            .sum::<f64>()
            .sqrt();
        println!(
            "model {name}: rms delay spread {:.1} ns (nominal {rms_nominal})",
            rms * 1e9
        );
        // The nominal figures are rounded descriptions of the profiles (C computes to 33 ns).
        assert!((rms * 1e9 / rms_nominal - 1.0).abs() < 0.12, "model {name}");
    }
    // Table II: LOS K-factor on the first tap only.
    for (model, k_db) in [
        (TgnModel::A, 0.0),
        (TgnModel::B, 0.0),
        (TgnModel::C, 0.0),
        (TgnModel::D, 3.0),
        (TgnModel::E, 6.0),
        (TgnModel::F, 6.0),
    ] {
        let taps = tgn_taps(model, true);
        assert!((taps[0].k_factor - 10f64.powf(k_db / 10.0)).abs() < 1e-12);
        assert!(taps[1..].iter().all(|t| t.k_factor == 0.0));
    }
    let a = tgn_taps(TgnModel::A, false);
    assert_eq!(a.len(), 1);
    assert_eq!((a[0].delay_s, a[0].power), (0.0, 1.0));
}

#[test]
fn tgn_tap_powers_average_to_the_profile() {
    let mut r = Rng::new(17);
    let taps = tgn_taps(TgnModel::E, false);
    let runs = 2000;
    let mut acc = vec![vec![]; taps.len()];
    for _ in 0..runs {
        let ch = Multipath::new(taps.clone(), Doppler::Static, 1, 20e6, &mut r);
        for (k, a) in acc.iter_mut().enumerate() {
            let g = ch.gain(k, 0);
            a.push(g.0 * g.0 + g.1 * g.1);
        }
    }
    for (k, a) in acc.iter().enumerate() {
        assert_ensemble(&format!("model E tap {k} power"), a, taps[k].power);
    }
}

#[test]
fn static_multipath_has_the_frequency_response_of_its_taps() {
    // Model D (longest delay 390 ns < 800 ns guard interval): after the guard, each 64-sample
    // symbol sees Y_k = H(f_k)·X_k with H(f) = Σ g_i·e^(−j2πf·τ_i), the continuous-time
    // response of the taps — independent of how the simulator realises fractional delays.
    let mut r = Rng::new(18);
    let fs = 20e6;
    let taps = tgn_taps(TgnModel::D, false);
    let s = Ofdm::new(&mut r);
    let n = 80 * 20;
    let x: Vec<Sample> = (0..n).map(|t| s.at(t as f64)).collect();
    let ch = Multipath::new(taps.clone(), Doppler::Static, n, fs, &mut r);
    let y = ch.apply(&x);
    assert!(y.len() >= n);
    let gains: Vec<Sample> = (0..taps.len()).map(|k| ch.gain(k, 0)).collect();
    let want: Vec<Sample> = s
        .bins
        .iter()
        .map(|&(k, xk)| {
            let f = f64::from(k) * fs / 64.0;
            let h = taps.iter().zip(&gains).fold((0.0, 0.0), |a, (t, g)| {
                let w = -2.0 * PI * f * t.delay_s;
                (
                    a.0 + g.0 * w.cos() - g.1 * w.sin(),
                    a.1 + g.0 * w.sin() + g.1 * w.cos(),
                )
            });
            (h.0 * xk.0 - h.1 * xk.1, h.0 * xk.1 + h.1 * xk.0)
        })
        .collect();
    // The input is 64-periodic, so any 64-sample window well inside the frame is one period.
    let start = 800;
    let spec = fft(&y[start..start + 64]).unwrap();
    let got: Vec<Sample> = s
        .bins
        .iter()
        .map(|&(k, _)| {
            // A window starting at `start` sees bin k rotated by e^(j2πk·start/64); undo it.
            // `Ofdm::at` carries no 1/64, so the forward FFT returns 64·Y_k.
            let v = spec[k.rem_euclid(64) as usize];
            let v = (v.0 / 64.0, v.1 / 64.0);
            let w = -2.0 * PI * f64::from(k) * start as f64 / 64.0;
            (v.0 * w.cos() - v.1 * w.sin(), v.0 * w.sin() + v.1 * w.cos())
        })
        .collect();
    let e = err_db(&want, &got);
    println!("static model D frequency response error {e:.1} dB");
    assert!(e < -45.0, "{e:.1} dB");
}

#[test]
fn bpsk_ber_in_flat_rayleigh_fading_matches_theory() {
    let mut r = Rng::new(19);
    let (runs, len) = (200, 5000);
    for snr_db in [5.0, 10.0, 20.0] {
        let g = 10f64.powf(snr_db / 10.0);
        let theory = 0.5 * (1.0 - (g / (1.0 + g)).sqrt());
        let mut ber = vec![];
        for _ in 0..runs {
            let ch = Multipath::new(
                tgn_taps(TgnModel::A, false),
                Doppler::Jakes { fd_hz: 50.0 },
                len,
                FS,
                &mut r,
            );
            let bits: Vec<f64> = (0..len)
                .map(|_| if r.next_u64() & 1 == 0 { 1.0 } else { -1.0 })
                .collect();
            let x: Vec<Sample> = bits.iter().map(|&b| (b, 0.0)).collect();
            let y = ch.apply(&x);
            let sigma = (0.5 / g).sqrt();
            let errors = (0..len)
                .filter(|&n| {
                    let h = ch.gain(0, n);
                    let (yr, yi) = (y[n].0 + sigma * r.gaussian(), y[n].1 + sigma * r.gaussian());
                    // Coherent detection with the known gain: sign of Re(h*·y).
                    let d = h.0 * yr + h.1 * yi;
                    (d >= 0.0) != (bits[n] > 0.0)
                })
                .count();
            ber.push(errors as f64 / len as f64);
        }
        assert_ensemble(&format!("BPSK Rayleigh BER at {snr_db} dB"), &ber, theory);
    }
}
