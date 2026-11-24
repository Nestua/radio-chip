//! The channel block's G0 (`docs/gates/rx_channel.md`): the architecture chosen for
//! `hw/rtl/rx_channel.sv` computes the model's channel and per-packet quotients bit for bit,
//! in the widths the RTL gives each register.
//!
//! The architecture (the founder chose the bin store's accumulation, 2026-11-24):
//! - T1's bins are written to the store as L(k)·T1(k); as T2's bin k comes, word k is read
//!   and Hs(k) = word + L(k)·T2(k) written back (the store becomes Hs for the packet). Its
//!   |Hs|² and the running largest are taken at the same time (the channel's first pass).
//! - The shift from the largest; a second pass reads Hs again, squares it, rounds and
//!   saturates the gain, writes it, sums Σg and keeps the four pilots' gains.
//! - One multiplier, used six times, makes the tracker's products; the three quotients and
//!   the soft scales on one serial divider, a quotient bit a clock, that takes the remainder
//!   as ⌊num / 2^q⌋ and shifts the numerator's last q bits in (legal when num < den·2^q, which
//!   is asserted at every division here). The variance saturates to 12 bits: a check
//!   before the division replaces the quotient by 4095 when it would not fit (the model uses
//!   the variance only when it is at most 3 nominal ones, 3072).
//!
//! Every register width the RTL takes from this file is asserted on every case.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{fft_bin, PILOT_SUBCARRIERS};
use radio_phy::ppdu::long_training_sequence;
use radio_phy::rx_front_fixed::IQ;
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{
    channel_from_bins, scale_constant, soft_scale, tracker_constants, variance_max, FixedOfdmConfig,
};

const N: usize = 64;
/// Hs's parts: W + 2 bits (+2^W from L = −1 on two bins at −2^(W−1)).
const HS_BITS: u32 = 18;
/// |Hs|² as an unsigned word: at most 2·2^32.
const FULL_BITS: u32 = 34;
/// Σg: 52 gains of at most 2^15 − 1.
const GSUM_BITS: u32 = 21;
/// The divider's widths (the RTL's parameters for this block).
const NUM_BITS: u32 = 48;
const DEN_BITS: u32 = 24;
const COEF_Q: u32 = 14;
const VAR_Q: u32 = 12;

fn bit_len(v: i128) -> u32 {
    i128::BITS - v.unsigned_abs().leading_zeros()
}

/// v / 2^s, half to even (the model's `rsh_even`, restated from its doc).
fn rsh_even(v: i128, s: u32) -> i128 {
    if s == 0 {
        return v;
    }
    let floor = v >> s;
    let rem = v - (floor << s);
    let half = 1i128 << (s - 1);
    if rem > half || (rem == half && floor & 1 == 1) {
        floor + 1
    } else {
        floor
    }
}

fn fits_signed(v: i128, bits: u32) -> bool {
    v >= -(1i128 << (bits - 1)) && v < (1i128 << (bits - 1))
}

fn fits_unsigned(v: i128, bits: u32) -> bool {
    v >= 0 && v < (1i128 << bits)
}

/// The serial divider: the remainder starts as ⌊num/2^q⌋ and the numerator's last q bits are
/// shifted in, one quotient bit a clock. Asserts its precondition and widths.
fn divide(num: i128, den: i128, q: u32) -> i128 {
    assert!(
        fits_unsigned(num, NUM_BITS),
        "numerator {num} wider than {NUM_BITS} bits"
    );
    assert!(
        fits_unsigned(den, DEN_BITS),
        "denominator {den} wider than {DEN_BITS} bits"
    );
    if den == 0 {
        // Every remainder is ≥ 0: every quotient bit is 1. The callers replace this quotient;
        // the divider's run is the same length.
        return (1i128 << q) - 1;
    }
    let mut p = num >> q;
    assert!(p < den, "num {num} ≥ den {den}·2^{q}");
    let mut quot = 0i128;
    for i in (0..q).rev() {
        p = (p << 1) | ((num >> i) & 1);
        assert!(fits_unsigned(p, DEN_BITS + 1));
        quot <<= 1;
        if p >= den {
            p -= den;
            quot |= 1;
        }
    }
    assert_eq!(quot, num / den);
    quot
}

fn qbits(m: Modulation) -> u32 {
    let cfg = RxOfdmConfig::default();
    bit_len(i128::from(scale_constant(
        m,
        &cfg,
        &FixedOfdmConfig::default(),
    ))) + 1
}

/// What the block hands on.
#[derive(Debug, PartialEq, Eq)]
struct Out {
    hs: [IQ; N],
    gain: [i64; N],
    shift: u32,
    gain_sum: i64,
    n: u32,
    scales: [i128; 4],
    outer: i64,
    inner: i64,
    variance: i64,
}

const MODS: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

/// The architecture, a register at a time.
fn arch(t1: &[IQ; N], t2: &[IQ; N]) -> Out {
    let fx = FixedOfdmConfig::default();
    let w = fx.fft_bits;
    let mut l = [0i64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = i64::from(v);
    }
    // Pass 1: T1 written as L·T1, T2 accumulated, |Hs|² and the largest.
    let mut store = [(0i128, 0i128); N];
    for b in 0..N {
        if l[b] != 0 {
            assert!(fits_signed(i128::from(t1[b].0), w) && fits_signed(i128::from(t1[b].1), w));
            store[b] = (i128::from(l[b] * t1[b].0), i128::from(l[b] * t1[b].1));
        }
    }
    let mut largest = 0i128;
    for b in 0..N {
        if l[b] != 0 {
            let hs = (
                store[b].0 + i128::from(l[b] * t2[b].0),
                store[b].1 + i128::from(l[b] * t2[b].1),
            );
            assert!(fits_signed(hs.0, HS_BITS) && fits_signed(hs.1, HS_BITS));
            store[b] = hs;
            let full = hs.0 * hs.0 + hs.1 * hs.1;
            assert!(fits_unsigned(full, FULL_BITS));
            largest = largest.max(full);
        }
    }
    let shift = bit_len(largest).saturating_sub(fx.prod_bits - 1);
    assert!(shift <= FULL_BITS - (fx.prod_bits - 1));
    // Pass 2: gains, Σg, the pilots'.
    let top = (1i128 << (fx.prod_bits - 1)) - 1;
    let mut gain = [0i64; N];
    let mut gsum = 0i128;
    for b in 0..N {
        if l[b] != 0 {
            let full = store[b].0 * store[b].0 + store[b].1 * store[b].1;
            let g = rsh_even(full, shift).min(top);
            gain[b] = g as i64;
            gsum += g;
        }
    }
    assert!(fits_unsigned(gsum, GSUM_BITS));
    let n = bit_len(gsum);
    let pg: Vec<i128> = PILOT_SUBCARRIERS
        .iter()
        .map(|&c| i128::from(gain[fft_bin(c)]))
        .collect();
    // The products, one multiplier: operand widths 37 × 16, product 53 bits.
    let mul = |a: i128, b: i128| -> i128 {
        assert!(fits_unsigned(a, 37) && fits_unsigned(b, 16), "{a} × {b}");
        a * b
    };
    let a = (pg[0] + pg[3]).max(1);
    let b = (pg[1] + pg[2]).max(1);
    let po = mul(pg[0], pg[3]);
    let pi = mul(pg[1], pg[2]);
    let no = mul(po, b);
    let ni = mul(pi, a);
    let sa = mul(gsum, a);
    let spread = mul(sa, b);
    let d = (no << 3) + no + ni;
    assert!(fits_unsigned(d, 50));
    let s = bit_len(d).saturating_sub(fx.track_bits);
    let dn = d >> s;
    let (nos, nis, sps) = (no >> s, ni >> s, spread >> s);
    let half = 14 * dn;
    let (outer, inner, variance) = if d == 0 {
        (0, 0, 0)
    } else {
        let outer = divide(((3 * nos) << (fx.coef_frac + 1)) + half, 28 * dn, COEF_Q);
        let inner = divide((nis << (fx.coef_frac + 1)) + half, 28 * dn, COEF_Q);
        let x = sps * 5;
        let max = (1i128 << VAR_Q) - 1;
        let variance = if x >= (52 * dn) << (VAR_Q - fx.var_frac) {
            divide(0, 52 * dn, VAR_Q);
            max
        } else {
            divide(x << fx.var_frac, 52 * dn, VAR_Q)
        };
        (outer, inner, variance)
    };
    let cfg = RxOfdmConfig::default();
    let mut scales = [0i128; 4];
    for (q, m) in scales.iter_mut().zip(MODS) {
        let a = i128::from(scale_constant(m, &cfg, &fx));
        let v = divide(a << n, gsum, qbits(m));
        *q = if gsum == 0 { 0 } else { v };
    }
    let mut hs = [(0i64, 0i64); N];
    for b in 0..N {
        hs[b] = (store[b].0 as i64, store[b].1 as i64);
    }
    Out {
        hs,
        gain,
        shift,
        gain_sum: gsum as i64,
        n,
        scales,
        outer: outer as i64,
        inner: inner as i64,
        variance: variance as i64,
    }
}

/// The model, its outputs in the block's form (the variance saturated to 12 bits).
fn model(t1: &[IQ; N], t2: &[IQ; N]) -> Out {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let ch = channel_from_bins(t1, t2, &fx).unwrap();
    let c = tracker_constants(&ch, &fx);
    let mut scales = [0i128; 4];
    let mut n = 0;
    for (q, m) in scales.iter_mut().zip(MODS) {
        let (v, nn) = soft_scale(ch.gain_sum, m, &cfg, &fx);
        *q = v;
        n = nn;
    }
    Out {
        hs: ch.hs,
        gain: ch.gain,
        shift: ch.shift,
        gain_sum: ch.gain_sum,
        n,
        scales,
        outer: c.outer,
        inner: c.inner,
        variance: c.variance,
    }
}

/// The cases: random full-scale bins, channels of a few taps at levels across the range,
/// and the corners (constructed in `corners`).
fn cases() -> Vec<([IQ; N], [IQ; N])> {
    let fx = FixedOfdmConfig::default();
    let hi = (1i64 << (fx.fft_bits - 1)) - 1;
    let lo = -hi - 1;
    let mut rng = Rng::new(0xc4a7_0010);
    let mut out = Vec::new();
    let span = (hi - lo + 1) as u64;
    for _ in 0..300 {
        let mut t = [[(0i64, 0i64); N]; 2];
        for w in &mut t {
            for v in w.iter_mut() {
                *v = (
                    (rng.next_u64() % span) as i64 + lo,
                    (rng.next_u64() % span) as i64 + lo,
                );
            }
        }
        out.push((t[0], t[1]));
    }
    let mut l = [0f64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = f64::from(v);
    }
    for level in 0..=16 {
        for _ in 0..40 {
            let amp = 2f64.powi(level) * (0.5 + rng.uniform());
            let taps: Vec<(f64, f64)> = (0..1 + (rng.next_u64() % 8) as usize)
                .map(|_| (rng.gaussian(), rng.gaussian()))
                .collect();
            let noise = amp * 10f64.powf(-rng.uniform() * 3.0);
            let mut t = [[(0i64, 0i64); N]; 2];
            for (b, &lb) in l.iter().enumerate() {
                let k = if b < 32 { b as f64 } else { b as f64 - 64.0 };
                let (mut hr, mut hi_) = (0.0, 0.0);
                for (n, &(tr, ti)) in taps.iter().enumerate() {
                    let ph = -2.0 * std::f64::consts::PI * k * n as f64 / 64.0;
                    hr += tr * ph.cos() - ti * ph.sin();
                    hi_ += tr * ph.sin() + ti * ph.cos();
                }
                for w in &mut t {
                    let q = |v: f64| (v.round() as i64).clamp(lo, hi);
                    w[b] = (
                        q(amp * lb * hr + noise * rng.gaussian()),
                        q(amp * lb * hi_ + noise * rng.gaussian()),
                    );
                }
            }
            out.push((t[0], t[1]));
        }
    }
    out.extend(corners());
    out
}

fn corners() -> Vec<([IQ; N], [IQ; N])> {
    let fx = FixedOfdmConfig::default();
    let hi = (1i64 << (fx.fft_bits - 1)) - 1;
    let lo = -hi - 1;
    let mut out = Vec::new();
    let zero = [(0i64, 0i64); N];
    // Silence: Σg = 0, no pair, every quotient 0.
    out.push((zero, zero));
    // Every bin at the negative rail: Hs = ±2^W, the widest parts.
    out.push(([(lo, lo); N], [(lo, lo); N]));
    out.push(([(hi, lo); N], [(lo, hi); N]));
    // Flat, then the pilots removed: one pair, then none.
    let flat = [(1000i64, -700i64); N];
    out.push((flat, flat));
    for gone in [
        &[0usize][..],
        &[0, 3],
        &[1, 2],
        &[0, 1],
        &[0, 1, 2, 3],
        &[1],
    ] {
        let mut t = flat;
        for &p in gone {
            t[fft_bin(PILOT_SUBCARRIERS[p])] = (0, 0);
        }
        out.push((t, t));
    }
    // Strong data, weak pilots: the variance saturates.
    let mut t = [(20000i64, 0i64); N];
    for &c in &PILOT_SUBCARRIERS {
        t[fft_bin(c)] = (1, 0);
    }
    out.push((t, t));
    // Small gains: the shift is 0 (largest |Hs|² below 2^15).
    for v in [1i64, 7, 50, 90] {
        out.push(([(v, -v); N], [(v, v); N]));
    }
    // The rounding corner: the largest |Hs|² just under 2^(15 + s) rounds to 2^15 and
    // saturates to 2^15 − 1. One part alone cannot reach that window below 2^34 (the squares
    // are further apart than it is wide): a pair (re, im) is searched for, Hs on bin 1
    // (L = +1) from T1 = (re, im), T2 = 0 or both halves.
    for m in 16u32..=33 {
        if let Some((re, im)) = rounding_corner(m, 2 * hi) {
            let mut a = flat;
            let mut b = flat;
            if re <= hi && im <= hi {
                a[1] = (re, im);
                b[1] = (0, 0);
            } else {
                a[1] = (re / 2, im / 2);
                b[1] = (re - re / 2, im - im / 2);
            }
            out.push((a, b));
        }
    }
    // The variance's check at its edge: pilots Hs = 1 (a = b = 2, D = 20), Σg = 207, 208, 209
    // from data gains 5 (Hs = (2, 1)) and 4 (Hs = (2, 0)); at 208 the quotient is exactly 2^12.
    let mut l = [0i64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = i64::from(v);
    }
    for fives in [11usize, 12, 13] {
        let mut t1 = [(0i64, 0i64); N];
        let mut data = 0;
        for b in 0..N {
            if l[b] == 0 {
                continue;
            }
            let hs = if PILOT_SUBCARRIERS.iter().any(|&c| fft_bin(c) == b) {
                (1, 0)
            } else {
                data += 1;
                if data <= fives {
                    (2, 1)
                } else {
                    (2, 0)
                }
            };
            t1[b] = (l[b] * hs.0, l[b] * hs.1);
        }
        out.push((t1, [(0i64, 0i64); N]));
    }
    out
}

/// A pair with re² + im² in [2^m − 2^(s−1), 2^m), s = m − 15: the largest gain rounds to 2^15.
fn rounding_corner(m: u32, limit: i64) -> Option<(i64, i64)> {
    let top = 1i128 << m;
    let low = top - (1i128 << (m - 16));
    let mut re = ((top - 1) as f64).sqrt().floor() as i64;
    while re > 0 {
        let rest = low - i128::from(re) * i128::from(re);
        let mut im = if rest <= 0 {
            0
        } else {
            (rest as f64).sqrt().ceil() as i64
        };
        while i128::from(im) * i128::from(im) < rest {
            im += 1;
        }
        let full = i128::from(re) * i128::from(re) + i128::from(im) * i128::from(im);
        if full >= low && full < top && re <= limit && im <= limit {
            return Some((re, im));
        }
        re -= 1;
    }
    None
}

#[test]
fn architecture_matches_the_model() {
    let cases = cases();
    let mut corner_hits = 0;
    for (i, (t1, t2)) in cases.iter().enumerate() {
        let a = arch(t1, t2);
        let m = model(t1, t2);
        assert_eq!(a, m, "case {i}");
        // The corner the saturation exists for: the largest gain rounded up to 2^15.
        let largest =
            a.hs.iter()
                .map(|h| i128::from(h.0).pow(2) + i128::from(h.1).pow(2))
                .max()
                .unwrap();
        if rsh_even(largest, a.shift) == 1 << 15 {
            corner_hits += 1;
        }
    }
    assert!(
        corner_hits >= 10,
        "{corner_hits} cases reached the gain's saturation"
    );
}

#[test]
fn the_cases_reach_every_branch() {
    let cases = cases();
    let outs: Vec<Out> = cases.iter().map(|(a, b)| arch(a, b)).collect();
    assert!(outs.iter().any(|o| o.shift == 0 && o.gain_sum > 0));
    assert!(outs.iter().any(|o| o.shift >= 18));
    assert!(outs.iter().any(|o| o.gain_sum == 0));
    assert!(outs.iter().any(|o| o.variance == 4095));
    // The saturation check at its edge: a quotient of exactly 2^12.
    let fx = FixedOfdmConfig::default();
    assert!(cases.iter().any(|(a, b)| variance_at_edge(a, b, &fx)));
    assert!(outs.iter().any(|o| o.variance > 0 && o.variance <= 3072));
    assert!(outs.iter().any(|o| o.outer == 0 && o.inner > 0));
    assert!(outs.iter().any(|o| o.inner == 0 && o.outer > 0));
    assert!(outs
        .iter()
        .any(|o| o.hs.iter().any(|h| h.0 == 1 << 16 || h.1 == 1 << 16)));
}

/// Whether the variance's quotient is exactly 2^VAR_Q (the saturation check's edge), from the
/// model's gains and the formula of `tracker_constants` before its saturation.
fn variance_at_edge(t1: &[IQ; N], t2: &[IQ; N], fx: &FixedOfdmConfig) -> bool {
    let ch = channel_from_bins(t1, t2, fx).unwrap();
    let g: Vec<i128> = PILOT_SUBCARRIERS
        .iter()
        .map(|&c| i128::from(ch.gain[fft_bin(c)]))
        .collect();
    let (a, b) = ((g[0] + g[3]).max(1), (g[1] + g[2]).max(1));
    let d = 9 * g[0] * g[3] * b + g[1] * g[2] * a;
    if d == 0 {
        return false;
    }
    let s = bit_len(d).saturating_sub(fx.track_bits);
    let dn = d >> s;
    let spread = i128::from(ch.gain_sum) * a * b;
    (((spread >> s) * 5) << fx.var_frac) == (52 * dn) << VAR_Q
}

#[test]
fn quotient_widths() {
    let fx = FixedOfdmConfig::default();
    // The package's widths follow the model's: V + 2 for the variance, K − 2 for the
    // coefficients, and the saturation above the largest usable variance (3 nominal ones).
    assert_eq!(VAR_Q, fx.var_frac + 2);
    assert_eq!(COEF_Q, fx.coef_frac - 2);
    assert_eq!(variance_max(&fx), (1 << VAR_Q) - 1);
    assert!(3i128 << fx.var_frac <= variance_max(&fx));
    // The schedule's widths (`radio_phy::rx_data`): BPSK 18, 64-QAM 20 at 8 / 3.
    assert_eq!(qbits(Modulation::Bpsk), 18);
    assert_eq!(qbits(Modulation::Qam64), 20);
}
