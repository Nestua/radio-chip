//! G1 gate for the channel simulator, front-end impairments (R0-07c): DC offset, CW tone,
//! AGC gain step.
//!
//! References we did not write (the formulas live here, in the test, not in the library):
//! 1. The DFT of a sampled complex exponential: a tone of amplitude A, phase φ at exactly
//!    m·fs/N has DFT N·A·e^(jφ) at bin m and 0 everywhere else. The DFT is computed directly
//!    (O(N²)) in this file, not with the library's FFT.
//! 2. The finite geometric series: Σ_{n<N} A·e^(j(ωn+φ)) = A·e^(jφ)·(1 − e^(jωN))/(1 − e^(jω))
//!    for an off-bin ω, so the tone's frequency is checked at an arbitrary value too.
//! 3. The lag-D autocorrelation of any pure tone has magnitude equal to its energy: the
//!    Schmidl–Cox metric is exactly 1 — the reason R0-08 needs a DC blocker.
//! 4. Decibels: a gain step of g dB scales amplitude by 10^(g/20), power by 10^(g/10).

use radio_phy::channel::{add_tone, agc_transient, awgn, gain_step, mean_power, GainState, Rng};
use std::f64::consts::PI;

type C = (f64, f64);

const FS: f64 = 20e6;

fn dft(x: &[C]) -> Vec<C> {
    let n = x.len();
    (0..n)
        .map(|k| {
            x.iter().enumerate().fold((0.0, 0.0), |acc, (i, s)| {
                let a = -2.0 * PI * (k * i % n) as f64 / n as f64;
                let (sin, cos) = a.sin_cos();
                (acc.0 + s.0 * cos - s.1 * sin, acc.1 + s.0 * sin + s.1 * cos)
            })
        })
        .collect()
}

fn cmul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn cdiv(a: C, b: C) -> C {
    let d = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
}

fn cis(a: f64) -> C {
    let (s, c) = a.sin_cos();
    (c, s)
}

fn dist(a: C, b: C) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

#[test]
fn tone_on_a_bin_lands_in_that_bin_only() {
    let n = 64;
    for (m, power, phase) in [(5usize, 0.25, 0.7), (60, 2.0, -2.1), (0, 0.5, 1.0)] {
        // Bin 60 of 64 is the negative frequency −4·fs/64: the sign of the exponent matters.
        let f = if m < n / 2 {
            m as f64
        } else {
            m as f64 - n as f64
        } * FS
            / n as f64;
        let y = add_tone(&vec![(0.0, 0.0); n], power, f, FS, phase);
        let spec = dft(&y);
        let amp = power.sqrt();
        let want = (n as f64 * amp * phase.cos(), n as f64 * amp * phase.sin());
        for (k, s) in spec.iter().enumerate() {
            let w = if k == m { want } else { (0.0, 0.0) };
            assert!(
                dist(*s, w) < 1e-9,
                "tone {f} Hz: bin {k} = {s:?}, want {w:?}"
            );
        }
    }
}

#[test]
fn off_bin_tone_matches_the_geometric_series() {
    let n = 1000;
    for (f, power, phase) in [(123_456.7, 0.3, 0.4), (-2_345_678.9, 1.7, -1.3)] {
        let y = add_tone(&vec![(0.0, 0.0); n], power, f, FS, phase);
        let w = 2.0 * PI * f / FS;
        let sum = y.iter().fold((0.0, 0.0), |a, s| (a.0 + s.0, a.1 + s.1));
        let one_minus = |z: C| (1.0 - z.0, -z.1);
        let a = power.sqrt();
        let want = cmul(
            (a * phase.cos(), a * phase.sin()),
            cdiv(one_minus(cis(w * n as f64)), one_minus(cis(w))),
        );
        assert!(dist(sum, want) < 1e-9, "f {f}: sum {sum:?}, want {want:?}");
        // Every sample has the stated power, and the last one the stated phase.
        assert!((mean_power(&y) - power).abs() < 1e-12);
        let last = y[n - 1];
        let want_last = (
            a * (phase + w * (n - 1) as f64).cos(),
            a * (phase + w * (n - 1) as f64).sin(),
        );
        assert!(dist(last, want_last) < 1e-9);
    }
}

#[test]
fn dc_is_a_zero_hertz_tone_and_adds_to_the_signal() {
    let mut rng = Rng::new(7);
    let x = awgn(&vec![(0.0, 0.0); 4096], 0.0, 1.0, &mut rng);
    let phase = 0.9;
    let y = add_tone(&x, 0.04, 0.0, FS, phase);
    let dc = (0.2 * phase.cos(), 0.2 * phase.sin());
    for (a, b) in x.iter().zip(&y) {
        assert!(dist((b.0 - a.0, b.1 - a.1), dc) < 1e-15);
    }
}

#[test]
fn any_tone_has_schmidl_cox_metric_one_at_lag_16() {
    // DC, a CW spur, a tone on the STF's own subcarrier spacing: all correlate perfectly at the
    // STF lag, so a detector without a DC blocker / plateau check would fire on them forever.
    const D: usize = 16;
    const L: usize = 48;
    for f in [0.0, 1.25e6, -3.3e6, 777_777.0] {
        let y = add_tone(&vec![(0.0, 0.0); L + D], 0.01, f, FS, 0.3);
        let (mut c, mut p1, mut p2) = ((0.0, 0.0), 0.0, 0.0);
        for k in 0..L {
            let (a, b) = (y[k], y[k + D]);
            let t = cmul(a, (b.0, -b.1));
            c = (c.0 + t.0, c.1 + t.1);
            p1 += a.0 * a.0 + a.1 * a.1;
            p2 += b.0 * b.0 + b.1 * b.1;
        }
        let metric = c.0.hypot(c.1) / (p1 * p2).sqrt();
        assert!((metric - 1.0).abs() < 1e-12, "f {f}: metric {metric}");
        // And the angle is −2π·f·D/fs: the coarse CFO estimator reads the tone's frequency.
        let angle = c.1.atan2(c.0);
        let want = -2.0 * PI * f * D as f64 / FS;
        let wrapped = (angle - want + PI).rem_euclid(2.0 * PI) - PI;
        assert!(wrapped.abs() < 1e-9, "f {f}: angle {angle}, want {want}");
    }
}

#[test]
fn gain_step_scales_from_its_sample_on() {
    let mut rng = Rng::new(11);
    let x = awgn(&vec![(0.0, 0.0); 1000], 0.0, 1.0, &mut rng);
    for (at, db) in [
        (0usize, 6.0),
        (1, -20.0),
        (400, 3.0),
        (999, -6.0),
        (1000, 10.0),
        (5000, 10.0),
    ] {
        let y = gain_step(&x, at, db);
        assert_eq!(y.len(), x.len());
        let g = 10f64.powf(db / 20.0);
        for (n, (a, b)) in x.iter().zip(&y).enumerate() {
            let want = if n < at { *a } else { (a.0 * g, a.1 * g) };
            assert!(dist(*b, want) < 1e-15, "at {at}, {db} dB: sample {n}");
        }
        if at > 0 && at < x.len() {
            let ratio = mean_power(&y[at..]) / mean_power(&x[at..]);
            assert!((10.0 * ratio.log10() - db).abs() < 1e-12);
            assert_eq!(&y[..at], &x[..at]);
        }
    }
}

/// `agc_transient` against its definition and against the two functions it generalises: with one
/// state and no phase or DC it is the inverse-ordered `gain_step` (gain before `until`, unity
/// after), and a DC-only state is `add_tone` at 0 Hz on that span.
#[test]
fn agc_transient_states() {
    let mut rng = Rng::new(0xa6c1);
    let x = awgn(&vec![(0.0, 0.0); 300], 0.0, 1.0, &mut rng);
    assert_eq!(agc_transient(&x, &[]), x);

    // One state: +20 dB before sample 100 is the whole signal at +20 dB with a −20 dB step at 100.
    let st = GainState {
        until: 100,
        gain_db: 20.0,
        phase: 0.0,
        dc: (0.0, 0.0),
    };
    let y = agc_transient(&x, &[st]);
    let all: Vec<_> = x.iter().map(|s| (s.0 * 10.0, s.1 * 10.0)).collect();
    let want = gain_step(&all, 100, -20.0);
    for (a, b) in y.iter().zip(&want) {
        assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
    }

    // Phase: a quarter turn maps (a, b) to (−b, a); DC is added after the gain.
    let st = GainState {
        until: 50,
        gain_db: 6.0,
        phase: std::f64::consts::FRAC_PI_2,
        dc: (3.0, -2.0),
    };
    let st2 = GainState {
        until: 120,
        gain_db: 0.0,
        phase: 0.0,
        dc: (-1.0, 0.5),
    };
    let y = agc_transient(&x, &[st, st2]);
    let g = 10f64.powf(6.0 / 20.0);
    for n in 0..300 {
        let want = if n < 50 {
            (-g * x[n].1 + 3.0, g * x[n].0 - 2.0)
        } else if n < 120 {
            (x[n].0 - 1.0, x[n].1 + 0.5)
        } else {
            x[n]
        };
        assert!(
            (y[n].0 - want.0).abs() < 1e-9 && (y[n].1 - want.1).abs() < 1e-9,
            "sample {n}"
        );
    }
    // The DC-only span agrees with add_tone at 0 Hz.
    let dc_only = GainState {
        until: 300,
        gain_db: 0.0,
        phase: 0.0,
        dc: (0.6, 0.8),
    };
    let y = agc_transient(&x, &[dc_only]);
    let t = add_tone(&x, 1.0, 0.0, 20e6, (0.8f64).atan2(0.6));
    for (a, b) in y.iter().zip(&t) {
        assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
    }
}
