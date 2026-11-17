//! G1 gate for the fixed-point framework: references we did not write.
//!
//! 1. A black-box run of fxpmath 0.4.10 (MIT): 2160 quantisations over nine Q formats, three
//!    rounding modes and two overflow modes, dense in exact ties:
//!    `tests/fixtures/fixedpoint_fxpmath.txt`, produced by `tools/ref/fixedpoint_fxpmath.py`.
//! 2. The textbook SQNR of a full-scale sine quantised to W bits, 6.02·W + 1.76 dB.
//! 3. Exact integer arithmetic (`i128` Euclidean division) as an independent formulation of
//!    `requantize`, plus error bounds and overflow invariants.

use radio_phy::fixed::{
    evm_percent, quantize, requantize, sqnr_db, sweep, FixedError, Overflow, QFormat, Rounding,
};

const FXPMATH: &str = include_str!("fixtures/fixedpoint_fxpmath.txt");
const ROUNDINGS: [Rounding; 3] = [Rounding::Floor, Rounding::HalfUp, Rounding::HalfEven];

fn q(w: u32, f: u32) -> QFormat {
    QFormat::new(w, f).unwrap_or_else(|e| panic!("test format Q({w},{f}): {e}"))
}

#[test]
fn quantize_matches_fxpmath() {
    let mut n = 0;
    for line in FXPMATH
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(f.len(), 6, "malformed fixture line: {line}");
        let x: f64 = f[0].parse().unwrap();
        let fmt = q(f[1].parse().unwrap(), f[2].parse().unwrap());
        let rounding = match f[3] {
            "floor" => Rounding::Floor,
            "half_up" => Rounding::HalfUp,
            "half_even" => Rounding::HalfEven,
            other => panic!("unknown rounding {other}"),
        };
        let overflow = match f[4] {
            "saturate" => Overflow::Saturate,
            "wrap" => Overflow::Wrap,
            other => panic!("unknown overflow {other}"),
        };
        let want: i64 = f[5].parse().unwrap();
        assert_eq!(
            quantize(x, fmt, rounding, overflow).unwrap(),
            want,
            "{line}"
        );
        n += 1;
    }
    assert!(n >= 2000, "fixture too small: {n}");
}

/// Independent formulation: exact rational rounding of raw / 2^s with i128 Euclidean division.
fn oracle_shift(raw: i64, s: u32, rounding: Rounding) -> i128 {
    let d: i128 = 1 << s;
    let r = i128::from(raw);
    match rounding {
        Rounding::Floor => r.div_euclid(d),
        Rounding::HalfUp => (2 * r + d).div_euclid(2 * d),
        Rounding::HalfEven => {
            let (fl, rem) = (r.div_euclid(d), r.rem_euclid(d));
            match (2 * rem).cmp(&d) {
                std::cmp::Ordering::Less => fl,
                std::cmp::Ordering::Greater => fl + 1,
                std::cmp::Ordering::Equal => fl + (fl & 1),
            }
        }
    }
}

fn oracle_overflow(v: i128, fmt: QFormat, overflow: Overflow) -> i64 {
    let (lo, hi) = (i128::from(fmt.min_raw()), i128::from(fmt.max_raw()));
    let out = match overflow {
        Overflow::Saturate => v.clamp(lo, hi),
        Overflow::Wrap => (v - lo).rem_euclid(hi - lo + 1) + lo,
    };
    out as i64
}

#[test]
fn requantize_matches_exact_integer_oracle() {
    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    for _ in 0..20_000 {
        let from_frac = (next() % 40) as u32;
        let to = q(1 + (next() % 32) as u32, (next() % 40) as u32);
        let raw = (next() >> 20) as i64 - (1 << 43);
        for rounding in ROUNDINGS {
            for overflow in [Overflow::Saturate, Overflow::Wrap] {
                let unbounded = if to.frac_bits() >= from_frac {
                    i128::from(raw) << (to.frac_bits() - from_frac)
                } else {
                    oracle_shift(raw, from_frac - to.frac_bits(), rounding)
                };
                let want = oracle_overflow(unbounded, to, overflow);
                let got = requantize(raw, from_frac, to, rounding, overflow);
                assert_eq!(
                    got, want,
                    "raw={raw} from={from_frac} to={to:?} {rounding:?} {overflow:?}"
                );
            }
        }
    }
}

#[test]
fn rounding_error_bounds_hold_in_range() {
    let fmt = q(16, 12);
    for k in -60_000i64..60_000 {
        let x = k as f64 / 8192.0 + 0.000_013;
        for rounding in ROUNDINGS {
            let raw = quantize(x, fmt, rounding, Overflow::Saturate).unwrap();
            let err = fmt.to_f64(raw) - x;
            match rounding {
                Rounding::Floor => assert!(err <= 0.0 && -err < fmt.lsb(), "x={x} err={err}"),
                _ => assert!(err.abs() <= fmt.lsb() / 2.0, "x={x} {rounding:?} err={err}"),
            }
        }
    }
}

#[test]
fn format_limits_and_errors() {
    let fmt = q(8, 7);
    assert_eq!((fmt.min_raw(), fmt.max_raw()), (-128, 127));
    assert_eq!(fmt.lsb(), 1.0 / 128.0);
    assert_eq!(fmt.to_f64(-128), -1.0);
    assert!(QFormat::new(0, 0).is_err());
    assert!(QFormat::new(49, 10).is_err());
    assert_eq!(
        quantize(f64::NAN, fmt, Rounding::Floor, Overflow::Saturate),
        Err(FixedError::NotFinite)
    );
    assert_eq!(
        quantize(1e300, fmt, Rounding::Floor, Overflow::Saturate),
        Err(FixedError::OutOfRange)
    );
    assert_eq!(
        quantize(5.0, fmt, Rounding::Floor, Overflow::Saturate),
        Ok(127)
    );
    assert_eq!(
        quantize(-5.0, fmt, Rounding::Floor, Overflow::Saturate),
        Ok(-128)
    );
}

#[test]
fn full_scale_sine_sqnr_follows_six_db_per_bit() {
    // Textbook result for uniform quantisation of a full-scale sine: 6.02·W + 1.76 dB.
    // Full scale in two's complement is 1 - LSB: an amplitude of exactly 1 clips the positive
    // peaks and costs 0.8 dB at W = 8 (cross-checked with numpy).
    let n = 1 << 16;
    let curve = sweep(8..=16, |w| {
        let fmt = q(w, w - 1);
        let amplitude = 1.0 - fmt.lsb();
        let reference: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * std::f64::consts::PI * 1031.0 * i as f64 / n as f64).sin())
            .collect();
        let approx: Vec<f64> = reference
            .iter()
            .map(|&x| fmt.to_f64(quantize(x, fmt, Rounding::HalfEven, Overflow::Saturate).unwrap()))
            .collect();
        sqnr_db(&reference, &approx).unwrap()
    });
    for (w, sqnr) in curve {
        let theory = 6.02 * f64::from(w) + 1.76;
        assert!(
            (sqnr - theory).abs() < 0.2,
            "W={w}: {sqnr:.2} dB vs {theory:.2} dB"
        );
    }
}

#[test]
fn metrics_on_known_errors() {
    let reference = [1.0, -1.0, 0.5, -0.5];
    assert_eq!(sqnr_db(&reference, &reference), Some(f64::INFINITY));
    let scaled: Vec<f64> = reference.iter().map(|x| x * 1.1).collect();
    let s = sqnr_db(&reference, &scaled).unwrap();
    assert!(
        (s - 20.0).abs() < 1e-9,
        "10% amplitude error is 20 dB, got {s}"
    );
    assert_eq!(sqnr_db(&reference, &reference[..3]), None);
    assert_eq!(sqnr_db(&[0.0, 0.0], &[0.0, 0.0]), None);

    let iq = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
    let off: Vec<(f64, f64)> = iq.iter().map(|&(i, q)| (i * 1.01, q * 1.01)).collect();
    let evm = evm_percent(&iq, &off).unwrap();
    assert!(
        (evm - 1.0).abs() < 1e-9,
        "1% scale error is 1% EVM, got {evm}"
    );
}
