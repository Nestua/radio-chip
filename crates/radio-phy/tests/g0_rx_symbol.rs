//! The symbol block's G0 (`docs/gates/rx_symbol.md`): the architecture chosen for
//! `hw/rtl/rx_symbol.sv` computes the model's pilots, common phase and soft values bit for
//! bit, in the widths the RTL gives each register.
//!
//! The architecture (the founder chose two slots, 2026-11-24):
//! - A symbol's bins are written to its slot (one of two 64-word stores) as they come. Once
//!   its 52 used bins are in and the channel is ready, the products pass reads Z(k) and Hs(k),
//!   multiplies (four real products), rounds by the packet's shift half to even, saturates to
//!   U bits and writes u(k) back in place: the pilots first (their signs removed, U + 1 bits,
//!   to the tracker), then the 48 data subcarriers.
//! - With the tracker's θ: the pilots read again, rotated by −c·θ (the phase in P bits: only
//!   θ's low P bits matter), summed (U + 2 bits), the CORDIC's angle of the sum widened to φ.
//! - The soft pass, a subcarrier a clock from −26 to 26: u rotated by −(φ + c·θ) through the
//!   front's quarter-wave table (half a table step added), rounded half to even, saturated;
//!   the two thresholds of the modulation from g by constant products (κ, C fractional bits);
//!   the demapper's values (U + 1 bits, magnitude at most 2^(U−1)); each times the packet's
//!   quotient q, rounded half away from zero by n + F, clipped to the soft limit.
//!
//! Every register width the RTL takes from this file is asserted on every case.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{data_subcarriers, fft_bin, pilot_polarity, PILOT_BASE, PILOT_SUBCARRIERS};
use radio_phy::ppdu::long_training_sequence;
use radio_phy::rx_front_fixed::{cordic_angle, nco_rotator, IQ};
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{
    channel_from_bins, soft_scale, symbol_from_bins, symbol_pilots, threshold_constant,
    FixedChannel, FixedOfdmConfig,
};

const N: usize = 64;
/// Bins: W bits.
const Z_BITS: u32 = 16;
/// Hs's parts: W + 2 bits.
const HS_BITS: u32 = 18;
/// Z·Hs* before the shift: two products of W and W + 2 bits, summed (|·| ≤ 2^(2W+1)).
const ZH_BITS: u32 = 35;
/// u: U bits.
const U_BITS: u32 = 16;
/// A pilot with its sign removed: U + 1 bits.
const PILOT_BITS: u32 = 17;
/// a·e^(jφ) before the narrowing: two products of U + 1 and Q bits, summed.
const ROT_BITS: u32 = 28;
/// The four rotated pilots summed.
const SUM_BITS: u32 = 18;
/// g·κ: U − 1 and the constant's bits.
const THR_PROD_BITS: u32 = 24;
/// The demapper's values: U + 1 bits signed.
const M_BITS: u32 = 17;
/// The packet's quotient q < 2·A: at most 20 bits (64-QAM).
const Q_BITS: u32 = 20;
/// |m|·q.
const MAG_BITS: u32 = 36;
/// θ: P + 1 bits as the tracker saturates it.
const THETA_BITS: u32 = 23;
const MODS: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

fn bit_len(v: i128) -> u32 {
    i128::BITS - v.unsigned_abs().leading_zeros()
}

/// v fits `bits` as a two's-complement word.
fn fits(v: i128, bits: u32) -> bool {
    v >= -(1i128 << (bits - 1)) && v < (1i128 << (bits - 1))
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

fn sat(v: i128, bits: u32) -> i128 {
    let hi = (1i128 << (bits - 1)) - 1;
    v.clamp(-hi - 1, hi)
}

struct Out {
    pilots: [IQ; 4],
    phi: i64,
    soft: Vec<i32>,
}

/// The RTL's arithmetic, register by register.
fn arch(
    z: &[IQ; N],
    ch: &FixedChannel,
    pol: usize,
    m: Modulation,
    theta: i64,
    fx: &FixedOfdmConfig,
) -> Out {
    let cfg = RxOfdmConfig::default();
    let p_bits = fx.front.nco_phase_bits;
    let mask = (1i128 << p_bits) - 1;
    let half = 1i128 << (p_bits - fx.front.nco_lut_bits - 1);
    let down_rot = fx.front.nco_amp_bits - 1;
    assert!(fits(i128::from(theta), THETA_BITS));
    assert!(ch.shift <= 19);
    // The products pass.
    let product = |b: usize| -> (i128, i128) {
        let (zr, zi) = (i128::from(z[b].0), i128::from(z[b].1));
        let (hr, hi) = (i128::from(ch.hs[b].0), i128::from(ch.hs[b].1));
        assert!(fits(zr, Z_BITS) && fits(zi, Z_BITS));
        assert!(fits(hr, HS_BITS) && fits(hi, HS_BITS));
        let (re, im) = (zr * hr + zi * hi, zi * hr - zr * hi);
        assert!(fits(re, ZH_BITS) && fits(im, ZH_BITS));
        (
            sat(rsh_even(re, ch.shift), U_BITS),
            sat(rsh_even(im, ch.shift), U_BITS),
        )
    };
    // The rotator: the phase in P bits (θ's low bits only), the table, the narrowing.
    let rotate = |a: (i128, i128), phase: i128| -> (i128, i128) {
        let ph = (phase + half) & mask;
        let r = nco_rotator(ph as i64, &fx.front);
        let (rr, ri) = (i128::from(r.0), i128::from(r.1));
        assert!(fits(rr, fx.front.nco_amp_bits) && fits(ri, fx.front.nco_amp_bits));
        let (re, im) = (a.0 * rr - a.1 * ri, a.0 * ri + a.1 * rr);
        assert!(fits(re, ROT_BITS) && fits(im, ROT_BITS));
        (
            sat(rsh_even(re, down_rot), U_BITS),
            sat(rsh_even(im, down_rot), U_BITS),
        )
    };
    let th = i128::from(theta) & mask;
    let polarity = i128::from(pilot_polarity(pol));
    let mut pilots = [(0i64, 0i64); 4];
    let mut sum = (0i128, 0i128);
    for ((slot, &c), &base) in pilots.iter_mut().zip(&PILOT_SUBCARRIERS).zip(&PILOT_BASE) {
        let s = polarity * i128::from(base);
        let u = product(fft_bin(c));
        let p = (s * u.0, s * u.1);
        assert!(fits(p.0, PILOT_BITS) && fits(p.1, PILOT_BITS));
        *slot = (p.0 as i64, p.1 as i64);
        let r = rotate(p, -(i128::from(c) * th));
        sum = (sum.0 + r.0, sum.1 + r.1);
        assert!(fits(sum.0, SUM_BITS) && fits(sum.1, SUM_BITS));
    }
    let angle = cordic_angle(sum.0 as i64, sum.1 as i64, &fx.front);
    let phi = angle << (p_bits - fx.front.angle_bits);
    // The soft pass.
    let n_bpsc = m.bits_per_subcarrier();
    let axis_bits = if m == Modulation::Bpsk { 1 } else { n_bpsc / 2 };
    let (k_hi, k_lo) = match axis_bits {
        2 => (threshold_constant(2, m, fx), 0),
        3 => (threshold_constant(4, m, fx), threshold_constant(2, m, fx)),
        _ => (0, 0),
    };
    let (q, n) = soft_scale(ch.gain_sum, m, &cfg, fx);
    assert!(q >= 0 && bit_len(q) <= Q_BITS);
    let down = n + fx.scale_frac;
    assert!((5..=25).contains(&down));
    let limit = i128::from(cfg.soft_max);
    let soft = |v: i128| -> i32 {
        assert!(fits(v, M_BITS) && v.abs() <= 1 << (U_BITS - 1));
        let mag = v.abs() * q;
        assert!(mag < 1i128 << MAG_BITS);
        let r = ((mag + (1i128 << (down - 1))) >> down).min(limit) as i32;
        if v < 0 {
            -r
        } else {
            r
        }
    };
    let mut out = Vec::new();
    for c in data_subcarriers() {
        let b = fft_bin(c);
        let v = rotate(
            product(b),
            -(i128::from(phi) + ((i128::from(c) * th) & mask)),
        );
        let g = i128::from(ch.gain[b]);
        assert!((0..1 << (U_BITS - 1)).contains(&g));
        let (ph, pl) = (g * i128::from(k_hi), g * i128::from(k_lo));
        assert!(ph < 1 << THR_PROD_BITS && pl < 1 << THR_PROD_BITS);
        let (thi, tlo) = (rsh_even(ph, fx.kappa_bits), rsh_even(pl, fx.kappa_bits));
        let axes = if m == Modulation::Bpsk { 1 } else { 2 };
        for a in [v.0, v.1].into_iter().take(axes) {
            out.push(soft(-a));
            if axis_bits >= 2 {
                let m1 = a.abs() - thi;
                out.push(soft(m1));
                if axis_bits == 3 {
                    out.push(soft(m1.abs() - tlo));
                }
            }
        }
    }
    Out {
        pilots,
        phi,
        soft: out,
    }
}

/// L(k) per FFT bin.
fn ltf_signs() -> [i64; N] {
    let mut l = [0i64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = i64::from(v);
    }
    l
}

fn rand_in(rng: &mut Rng, bits: u32) -> i64 {
    let span = 1u64 << bits;
    (rng.next_u64() % span) as i64 - (1i64 << (bits - 1))
}

/// A channel of one to eight taps at amplitude 2^level, through the model's estimate.
fn channel_case(rng: &mut Rng, level: i32, fx: &FixedOfdmConfig) -> FixedChannel {
    let hi = (1i64 << (fx.fft_bits - 1)) - 1;
    let l = ltf_signs();
    let amp = 2f64.powi(level) * (0.5 + rng.uniform());
    let taps: Vec<(f64, f64)> = (0..1 + (rng.next_u64() % 8) as usize)
        .map(|_| (rng.gaussian(), rng.gaussian()))
        .collect();
    let noise = amp * 10f64.powf(-rng.uniform() * 1.5);
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
            let q = |v: f64| (v.round() as i64).clamp(-hi - 1, hi);
            w[b] = (
                q(amp * lb as f64 * hr + noise * rng.gaussian()),
                q(amp * lb as f64 * hi_ + noise * rng.gaussian()),
            );
        }
    }
    channel_from_bins(&t[0], &t[1], fx).unwrap()
}

/// A channel built by hand at the inputs' limits: Hs, g, the shift and Σg anywhere in their
/// formats (the block takes them as they come; nothing ties them together at its pins).
fn hand_channel(rng: &mut Rng, hs_bits: u32, shift: u32, g_max: i64, gsum: i64) -> FixedChannel {
    let mut hs = [(0i64, 0i64); N];
    let mut gain = [0i64; N];
    for b in 0..N {
        hs[b] = (rand_in(rng, hs_bits), rand_in(rng, hs_bits));
        gain[b] = (rng.next_u64() % (g_max as u64 + 1)) as i64;
    }
    FixedChannel {
        hs,
        shift,
        gain,
        gain_sum: gsum,
    }
}

/// Bins of a symbol on channel `ch`: constellation points of `m` through it, a common phase
/// and a slope, noise; or anything at all.
fn symbol_bins(rng: &mut Rng, ch: &FixedChannel, level: f64, noise: f64) -> [IQ; N] {
    let hi = (1i64 << (Z_BITS - 1)) - 1;
    let peak = ch
        .hs
        .iter()
        .map(|h| ((h.0 * h.0 + h.1 * h.1) as f64).sqrt())
        .fold(1.0, f64::max);
    let ph0 = rng.uniform() * 2.0 * std::f64::consts::PI;
    let slope = (rng.uniform() - 0.5) * 0.1;
    let mut z = [(0i64, 0i64); N];
    for (b, v) in z.iter_mut().enumerate() {
        let k = if b < 32 { b as f64 } else { b as f64 - 64.0 };
        let (xr, xi) = (
            (rng.next_u64() % 8) as f64 - 3.5,
            (rng.next_u64() % 8) as f64 - 3.5,
        );
        let (hr, hi_) = (ch.hs[b].0 as f64 / peak, ch.hs[b].1 as f64 / peak);
        let (yr, yi) = (hr * xr - hi_ * xi, hr * xi + hi_ * xr);
        let a = ph0 + slope * k;
        let (cr, ci) = (a.cos(), a.sin());
        let q = |x: f64| (x.round() as i64).clamp(-hi - 1, hi);
        *v = (
            q(level * (yr * cr - yi * ci) + noise * rng.gaussian()),
            q(level * (yr * ci + yi * cr) + noise * rng.gaussian()),
        );
    }
    z
}

fn random_bins(rng: &mut Rng) -> [IQ; N] {
    let mut z = [(0i64, 0i64); N];
    for v in z.iter_mut() {
        *v = (rand_in(rng, Z_BITS), rand_in(rng, Z_BITS));
    }
    z
}

fn check(
    z: &[IQ; N],
    ch: &FixedChannel,
    pol: usize,
    m: Modulation,
    theta: i64,
    fx: &FixedOfdmConfig,
) {
    let cfg = RxOfdmConfig::default();
    let a = arch(z, ch, pol, m, theta, fx);
    let pilots = symbol_pilots(z, ch, pol, fx).unwrap();
    let (soft, phi) = symbol_from_bins(z, ch, pol, m, theta, &cfg, fx).unwrap();
    assert_eq!(a.pilots, pilots, "pilots, pol {pol}");
    assert_eq!(a.phi, phi, "phi");
    assert_eq!(a.soft, soft, "soft values, {m:?}");
}

#[test]
fn architecture_matches_the_model_on_channels() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x5e3b_0001);
    let mut count = 0;
    for level in 0..=16 {
        for _ in 0..12 {
            let ch = channel_case(&mut rng, level, &fx);
            if ch.gain_sum <= 0 {
                continue;
            }
            for &m in &MODS {
                let lvl = 2f64.powf(4.0 + rng.uniform() * 11.0);
                let noise = lvl * 0.05 * rng.uniform();
                let z = symbol_bins(&mut rng, &ch, lvl, noise);
                let theta = rand_in(&mut rng, 16);
                check(&z, &ch, (rng.next_u64() % 200) as usize, m, theta, &fx);
                count += 1;
            }
        }
    }
    assert!(count > 700, "{count} symbols");
}

#[test]
fn architecture_matches_the_model_at_the_limits() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x5e3b_0002);
    let theta_max = (1i64 << (THETA_BITS - 1)) - 1;
    for i in 0..1500 {
        let shift = (rng.next_u64() % 20) as u32;
        let g_max = if i % 3 == 0 {
            32767
        } else {
            1 + (rng.next_u64() % 32767) as i64
        };
        let gsum = if i % 5 == 0 {
            1
        } else {
            1 + (rng.next_u64() % ((1 << 21) - 1)) as i64
        };
        let ch = hand_channel(&mut rng, HS_BITS, shift, g_max, gsum);
        let z = random_bins(&mut rng);
        let theta = match i % 4 {
            0 => theta_max,
            1 => -theta_max - 1,
            _ => rand_in(&mut rng, THETA_BITS),
        };
        check(
            &z,
            &ch,
            (rng.next_u64() % 300) as usize,
            MODS[i % 4],
            theta,
            &fx,
        );
    }
}

/// The rails: every bin and Hs at −2^(W−1) and −2^(W+1) (the products saturate), shift 0,
/// the largest gain and Σg of 1 (the largest quotient and the smallest n).
#[test]
fn architecture_matches_the_model_on_the_rails() {
    let fx = FixedOfdmConfig::default();
    let lo_z = -(1i64 << (Z_BITS - 1));
    let lo_h = -(1i64 << (HS_BITS - 1));
    for &(zr, zi, hr, hi) in &[
        (lo_z, lo_z, lo_h, lo_h),
        (lo_z, -lo_z - 1, lo_h, -lo_h - 1),
        (-lo_z - 1, -lo_z - 1, -lo_h - 1, -lo_h - 1),
        (lo_z, 0, lo_h, 0),
        (0, 0, 0, 0),
        (1, 0, 1, 0),
    ] {
        for shift in [0u32, 1, 19] {
            for &gsum in &[1i64, (1 << 21) - 1] {
                for &m in &MODS {
                    for theta in [0i64, (1 << 22) - 1, -(1 << 22)] {
                        let ch = FixedChannel {
                            hs: [(hr, hi); N],
                            shift,
                            gain: [32767; N],
                            gain_sum: gsum,
                        };
                        check(&[(zr, zi); N], &ch, 0, m, theta, &fx);
                        check(&[(zr, zi); N], &ch, 126, m, theta, &fx);
                    }
                }
            }
        }
    }
}

/// θ's high bits do not matter: θ and θ + 2^P give the same symbol (the RTL keeps P bits).
#[test]
fn theta_matters_modulo_a_turn() {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x5e3b_0003);
    let ch = channel_case(&mut rng, 10, &fx);
    let z = symbol_bins(&mut rng, &ch, 3000.0, 30.0);
    for _ in 0..50 {
        let theta = rand_in(&mut rng, 22);
        let wrapped = if theta < 0 {
            theta + (1 << 22)
        } else {
            theta - (1 << 22)
        };
        let a = symbol_from_bins(&z, &ch, 5, Modulation::Qam64, theta, &cfg, &fx).unwrap();
        let b = symbol_from_bins(&z, &ch, 5, Modulation::Qam64, wrapped, &cfg, &fx).unwrap();
        assert_eq!(a, b);
    }
}

/// The widths here are the model's defaults: a change of the defaults has to come back here.
#[test]
fn widths_are_the_defaults() {
    let fx = FixedOfdmConfig::default();
    assert_eq!(fx.fft_bits, Z_BITS);
    assert_eq!(fx.prod_bits, U_BITS);
    assert_eq!(fx.front.nco_phase_bits + 1, THETA_BITS);
    assert_eq!(fx.front.nco_amp_bits, 11);
    assert_eq!(fx.kappa_bits, 10);
    assert_eq!(fx.scale_frac, 4);
    let cfg = RxOfdmConfig::default();
    assert_eq!(cfg.soft_max, 127);
    assert!(cfg.pilot_tracking && cfg.timing_tracking);
    // The thresholds' constants: 16-QAM's c = 2, 64-QAM's c = 4 and 2 (9 bits).
    assert_eq!(threshold_constant(2, Modulation::Qam16, &fx), 324);
    assert_eq!(threshold_constant(4, Modulation::Qam64, &fx), 316);
    assert_eq!(threshold_constant(2, Modulation::Qam64, &fx), 158);
}
