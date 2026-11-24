//! The tracker block's G0 (`docs/gates/rx_track.md`): the architecture chosen for
//! `hw/rtl/rx_track.sv` computes the model's slope θ, its correction δ and the window's moves
//! bit for bit, in the widths the RTL gives each register.
//!
//! The architecture (the founder chose one shared multiplier, 2026-11-24):
//! - Every product goes through one signed multiplier of A × B bits (A = 24, B = 44), one
//!   product a clock; a small sequencer orders them. No other multiplier in the block.
//! - A packet starts with the front's CFO word and the fs/fc register: rate0 = word·ratio
//!   rounded by 22 − R, half to even (it always fits P − 6 + R bits: the model's saturation
//!   never acts), with the prior's weight of a known clock offset; a ratio of 0 (no register
//!   written) gives rate0 = 0 and the weight of an unknown one (two ROM constants). SIGNAL's θ
//!   is rate0·7 rounded by R (δ is 0 before SIGNAL and after it: with one measurement the fit's
//!   numerator is 7z − 7z), out at the packet's start with its move.
//! - From the channel block's quotients: the two coefficients, the variance, then
//!   λ = weight·variance rounded by V, and whether the packet's measurements are used.
//! - A symbol's pilots (one set held, free once its pair products are made: the symbol
//!   block's pilots are at least 48 clocks apart, and later symbols' wait for the last θ):
//!   the two pair products hi·conj(lo) (four products each), their angles on the shared
//!   iterative CORDIC widened by P − A; the predicted slope's turn subtracted modulo 2^P by
//!   shifts and adds (42 = 32 + 8 + 2, 14 = 8 + 4 + 2: only the prediction's low P bits
//!   matter); the measurement e = (co·∠o + ci·∠i) rounded by K; then the fit's sums, the
//!   determinant and the numerator as products; |num|·2^R on the shared serial divider for 26
//!   quotient bits, with no comparison for the model's saturation of δ (the quotient is below
//!   2^26 on every frame: `the_fit_never_saturates`); δ with the numerator's sign; the drift;
//!   θ and the move. SIGNAL needs no division (its quotient is 0); a packet whose measurements
//!   are not used none at all.
//! - The sums' saturations in the model never act within a frame (at most 1367 measurements,
//!   asserted here at the longest frame); the RTL has none. The window's shift is 8 bits: the
//!   drift saturates at 2^P, 64 samples, and a shift follows it to at most 65.
//!
//! Every register width the RTL takes from this file is asserted on every case.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::rx_front_fixed::{cordic_angle, IQ};
use radio_phy::rx_ofdm_fixed::{
    tracker_constants, FixedChannel, FixedOfdmConfig, FixedTracking, TrackerConstants,
};

/// The multiplier's operands: A and B bits signed.
const MUL_A: u32 = 24;
const MUL_B: u32 = 44;
/// Its largest product.
const PROD_BITS: u32 = 56;
/// A pilot: U + 1 bits.
const PILOT_BITS: u32 = 17;
/// A pair product's parts: the CORDIC's input.
const PAIR_BITS: u32 = 34;
/// rate0: P − 6 + R bits; δ: FIT_QBITS + 1 (the model's register is P + R − 4 = 28 bits and
/// never saturates); rate0 + δ one more.
const RATE0_BITS: u32 = 26;
const DELTA_BITS: u32 = 27;
const RATE_BITS: u32 = 28;
/// θ and the drift: P + 1 bits; the measurement e: P; z: P − 2.
const THETA_BITS: u32 = 23;
const ERR_BITS: u32 = 22;
const Z_BITS: u32 = 20;
/// co·∠o + ci·∠i.
const ERRSUM_BITS: u32 = 38;
/// The window's shift.
const SHIFT_BITS: u32 = 8;
/// τ = 7 + 5i, i ≤ 1366; n.
const TAU_BITS: u32 = 13;
const N_BITS: u32 = 11;
/// Στ, Στ², Σz, Στz (signed), λ, Στ² + λ.
const S1_BITS: u32 = 23;
const S2_BITS: u32 = 35;
const Y0_BITS: u32 = 31;
const Y1_BITS: u32 = 43;
const LAMBDA_BITS: u32 = 27;
const S2L_BITS: u32 = 36;
/// The determinant (unsigned), the numerator (signed), |num|·2^R (the divider's numerator).
const DET_BITS: u32 = 47;
const NUM_BITS: u32 = 55;
const DIV_NUM_BITS: u32 = 64;
/// The coefficients (K − 2 bits) and the variance (V + 2) as the channel block gives them.
const COEF_BITS: u32 = 14;
const VAR_BITS: u32 = 12;
/// The divider's quotient bits for the fit: |δ| < 2^26 on every frame, below the model's
/// saturation at 2^27 (`the_fit_never_saturates`).
const FIT_QBITS: u32 = 26;
const LAST_SYMBOL: usize = 1366;

fn bit_len(v: i128) -> u32 {
    i128::BITS - v.unsigned_abs().leading_zeros()
}

fn fits(v: i128, bits: u32) -> bool {
    v >= -(1i128 << (bits - 1)) && v < (1i128 << (bits - 1))
}

fn ufits(v: i128, bits: u32) -> bool {
    v >= 0 && v < (1i128 << bits)
}

/// v / 2^s, half to even.
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

/// The shared multiplier: both operands in their widths, the product in its register's.
fn mul(a: i128, b: i128) -> i128 {
    assert!(fits(a, MUL_A), "operand A {a}");
    assert!(fits(b, MUL_B), "operand B {b}");
    let p = a * b;
    assert!(fits(p, PROD_BITS), "product {p}");
    p
}

/// `div_serial`'s restoring division, restated: floor(num/den) in `qbits` bits, the remainder
/// starting as num / 2^qbits (below den: the caller's bound).
fn divide(num: i128, den: i128, qbits: u32) -> i128 {
    assert!(ufits(num, DIV_NUM_BITS) && ufits(den, DET_BITS) && den > 0);
    assert!(num >> qbits < den, "divider bound");
    let mut rem = num >> qbits;
    let mut q = 0i128;
    for k in (0..qbits).rev() {
        let trial = (rem << 1) | ((num >> k) & 1);
        let take = trial >= den;
        rem = if take { trial - den } else { trial };
        assert!(ufits(rem, DET_BITS));
        q = (q << 1) | i128::from(take);
    }
    q
}

/// The block's registers.
struct Arch {
    rate0: i128,
    weight: i128,
    delta: i128,
    shift: i128,
    n: i128,
    s1: i128,
    s2: i128,
    y0: i128,
    y1: i128,
    co: i128,
    ci: i128,
    lambda: i128,
    usable: bool,
}

const P: u32 = 22;
const R: u32 = 10;
const K: u32 = 16;
const V: u32 = 10;
const A_BITS: u32 = 16;

impl Arch {
    /// The packet's start: rate0, the weight; SIGNAL's θ (returned) and its move.
    fn start(cfo: i64, ratio: u16, weights: (i64, i64)) -> (Self, i128) {
        assert!(fits(i128::from(cfo), P));
        let (rate0, weight) = if ratio == 0 {
            (0, weights.1)
        } else {
            let prod = mul(i128::from(ratio), i128::from(cfo));
            // No saturation in the RTL: it never acts (`widths_are_the_defaults`).
            let r = rsh_even(prod, 22 - R);
            assert!(fits(r, RATE0_BITS));
            (r, weights.0)
        };
        let mut a = Self {
            rate0,
            weight: i128::from(weight),
            delta: 0,
            shift: 0,
            n: 0,
            s1: 0,
            s2: 0,
            y0: 0,
            y1: 0,
            co: 0,
            ci: 0,
            lambda: 0,
            usable: false,
        };
        let theta = a.drift(7);
        a.move_window(theta);
        (a, theta)
    }

    /// The channel block's three quotients.
    fn constants(&mut self, c: &TrackerConstants) {
        let (co, ci, v) = (
            i128::from(c.outer),
            i128::from(c.inner),
            i128::from(c.variance),
        );
        assert!(ufits(co, COEF_BITS) && ufits(ci, COEF_BITS) && ufits(v, VAR_BITS));
        self.co = co;
        self.ci = ci;
        self.lambda = rsh_even(mul(v, self.weight), V);
        assert!(ufits(self.lambda, LAMBDA_BITS), "lambda {}", self.lambda);
        self.usable = v > 0 && v <= 3 << V;
    }

    /// (rate0 + δ)·τ rounded by R, saturated to P + 1 bits.
    fn drift(&self, tau: i128) -> i128 {
        let rate = self.rate0 + self.delta;
        assert!(fits(rate, RATE_BITS));
        sat(rsh_even(mul(tau, rate), R), THETA_BITS)
    }

    fn move_window(&mut self, theta: i128) {
        let half = 1i128 << (P - 7);
        if theta > half {
            self.shift -= 1;
        } else if theta < -half {
            self.shift += 1;
        }
        assert!(fits(self.shift, SHIFT_BITS), "shift {}", self.shift);
    }

    /// A data symbol's (i ≥ 1) or SIGNAL's (i = 0) step; θ for a data symbol.
    fn step(&mut self, p: &[IQ; 4], i: usize, fx: &FixedOfdmConfig) -> Option<i128> {
        assert!(i <= LAST_SYMBOL);
        let tau = 7 + 5 * i as i128;
        assert!(ufits(tau, TAU_BITS));
        for v in p {
            assert!(fits(i128::from(v.0), PILOT_BITS) && fits(i128::from(v.1), PILOT_BITS));
        }
        // SIGNAL's θ and move were made at the start: its prediction has no move in it.
        let moved = if i == 0 { 0 } else { self.shift << (P - 6) };
        let predicted = self.drift(tau) + moved;
        let pm = predicted & ((1 << P) - 1);
        let angle = |hi: IQ, lo: IQ, span: i128| -> i128 {
            let (ar, ai) = (i128::from(hi.0), i128::from(hi.1));
            let (br, bi) = (i128::from(lo.0), i128::from(lo.1));
            let re = mul(ar, br) + mul(ai, bi);
            let im = mul(ai, br) - mul(ar, bi);
            assert!(fits(re, PAIR_BITS) && fits(im, PAIR_BITS));
            let a = i128::from(cordic_angle(re as i64, im as i64, &fx.front));
            assert!(fits(a, A_BITS));
            let turn = match span {
                42 => (pm << 5) + (pm << 3) + (pm << 1),
                _ => (pm << 3) + (pm << 2) + (pm << 1),
            };
            let d = ((a << (P - A_BITS)) - turn) & ((1 << P) - 1);
            if d >= 1 << (P - 1) {
                d - (1 << P)
            } else {
                d
            }
        };
        let (outer, inner) = (angle(p[3], p[0], 42), angle(p[2], p[1], 14));
        let sum = mul(self.co, outer) + mul(self.ci, inner);
        assert!(fits(sum, ERRSUM_BITS));
        let err = rsh_even(sum, K);
        // No saturation: |e| < 2^(P−1) with coefficients below 2^14.
        assert!(fits(err, ERR_BITS));
        if self.usable {
            let back = rsh_even(mul(tau, self.delta), R);
            let z = sat(err + back, Z_BITS);
            self.n += 1;
            self.s1 += tau;
            self.s2 += mul(tau, tau);
            self.y0 += z;
            self.y1 += mul(tau, z);
            assert!(ufits(self.n, N_BITS) && ufits(self.s1, S1_BITS) && ufits(self.s2, S2_BITS));
            assert!(fits(self.y0, Y0_BITS) && fits(self.y1, Y1_BITS));
            let s2l = self.s2 + self.lambda;
            assert!(ufits(s2l, S2L_BITS));
            let det = mul(self.n, s2l) - mul(self.s1, self.s1);
            assert!(ufits(det, DET_BITS), "det {det}");
            if i > 0 && det > 0 {
                let num = mul(self.n, self.y1) - mul(self.s1, self.y0);
                assert!(fits(num, NUM_BITS));
                let mag = num.abs() << R;
                assert!(ufits(mag, DIV_NUM_BITS));
                // No comparison for the saturation: the quotient is below 2^FIT_QBITS on every
                // frame (`the_fit_never_saturates`); the divider's bound asserts it here.
                let q = divide(mag, det, FIT_QBITS);
                self.delta = if num < 0 { -q } else { q };
                assert!(fits(self.delta, DELTA_BITS));
            }
        }
        if i == 0 {
            return None;
        }
        let theta = sat(self.drift(tau) + moved, THETA_BITS);
        self.move_window(theta);
        Some(theta)
    }
}

/// The two weights: a known clock offset (the CFO word and the ratio), an unknown one.
fn weights(fx: &FixedOfdmConfig) -> (i64, i64) {
    (
        FixedTracking::from_cfo_word(0, 1, fx).weight,
        FixedTracking::unknown(fx).weight,
    )
}

fn model_start(cfo: i64, ratio: u16, fx: &FixedOfdmConfig) -> FixedTracking {
    if ratio == 0 {
        FixedTracking::unknown(fx)
    } else {
        FixedTracking::from_cfo_word(cfo, ratio, fx)
    }
}

fn gaussian_pair(rng: &mut Rng, s: f64) -> (f64, f64) {
    (s * rng.gaussian(), s * rng.gaussian())
}

/// The pilots of a symbol whose slope is `slope` (units 2π/2^P a subcarrier): gains
/// `amp`, a common phase, noise; or anything in their format.
fn pilots(rng: &mut Rng, amp: &[f64; 4], slope: f64, noise: f64, wild: bool) -> [IQ; 4] {
    let lim = 1i64 << 15;
    let mut p = [(0i64, 0i64); 4];
    if wild {
        for v in p.iter_mut() {
            let pick = |rng: &mut Rng| match rng.next_u64() % 4 {
                0 => lim,
                1 => -lim,
                _ => (rng.next_u64() % (2 * lim as u64 + 1)) as i64 - lim,
            };
            *v = (pick(rng), pick(rng));
        }
        return p;
    }
    let phi = rng.uniform() * 2.0 * std::f64::consts::PI;
    for ((v, &c), &a) in p.iter_mut().zip(&[-21.0f64, -7.0, 7.0, 21.0]).zip(amp) {
        let ph = phi + c * slope * 2.0 * std::f64::consts::PI / f64::from(1u32 << P);
        let (nr, ni) = gaussian_pair(rng, noise);
        let q = |x: f64| (x.round() as i64).clamp(-lim, lim);
        *v = (q(a * ph.cos() + nr), q(a * ph.sin() + ni));
    }
    p
}

/// A channel's four pilot gains and Σg (all the constants read).
fn channel(gains: [i64; 4], gsum: i64) -> FixedChannel {
    let mut gain = [0i64; 64];
    for (&c, &g) in [43usize, 57, 7, 21].iter().zip(&gains) {
        gain[c] = g;
    }
    FixedChannel {
        hs: [(0, 0); 64],
        shift: 0,
        gain,
        gain_sum: gsum,
    }
}

struct Packet {
    cfo: i64,
    ratio: u16,
    c: TrackerConstants,
    /// True drift rate (R fractional bits per τ) the pilots carry.
    rate: f64,
    amp: [f64; 4],
    noise: f64,
    wild: bool,
    symbols: usize,
}

/// Runs a packet through the model and the architecture, symbol by symbol.
fn run(rng: &mut Rng, pk: &Packet, fx: &FixedOfdmConfig) -> (usize, i128, i128) {
    let mut model = model_start(pk.cfo, pk.ratio, fx);
    let (mut a, theta0) = Arch::start(pk.cfo, pk.ratio, weights(fx));
    a.constants(&pk.c);
    let mut moves = 0usize;
    let mut max_shift = 0i128;
    let mut max_delta = 0i128;
    for i in 0..pk.symbols {
        // The slope the pilots carry: the true drift less the window's moves so far.
        let tau = 7.0 + 5.0 * i as f64;
        let slope = pk.rate * tau / 1024.0 + (model.shift as f64) * 65536.0;
        let p = pilots(rng, &pk.amp, slope, pk.noise, pk.wild);
        let theta = model.step(&p, &pk.c, i, fx).unwrap();
        let before = model.shift;
        model.move_window(fx);
        let arch_theta = a.step(&p, i, fx);
        if i == 0 {
            assert_eq!(theta0, i128::from(theta), "SIGNAL's theta");
            assert!(arch_theta.is_none());
        } else {
            assert_eq!(arch_theta, Some(i128::from(theta)), "theta, symbol {i}");
        }
        assert_eq!(a.delta, i128::from(model.delta), "delta, symbol {i}");
        assert_eq!(a.shift, i128::from(model.shift), "shift, symbol {i}");
        if model.shift != before {
            moves += 1;
        }
        max_shift = max_shift.max(a.shift.abs());
        max_delta = max_delta.max(a.delta.abs());
    }
    (moves, max_shift, max_delta)
}

/// A clock offset of `ppm` as the pilots' drift rate (R fractional bits per τ: 16 samples a
/// fifth of a symbol, 2^(P−6) a sample), and the front's CFO word for `seen` ppm at 5.8 GHz and
/// 20 MHz (ratio 3616).
fn offset(ppm: f64, seen: f64) -> (f64, i64) {
    let rate = ppm * 1e-6 * 16.0 * 65536.0 * 1024.0;
    let cfo = (seen * 1e-6 * 5.8e9 / 20e6 * f64::from(1u32 << P)).round() as i64;
    (rate, cfo.clamp(-(1 << (P - 1)), (1 << (P - 1)) - 1))
}

fn real_packet(rng: &mut Rng, fx: &FixedOfdmConfig, symbols: usize, ppm_max: f64) -> Packet {
    let level = 1i64 << (4 + rng.next_u64() % 11);
    let mut gains = [0i64; 4];
    for g in gains.iter_mut() {
        *g = ((level as f64) * (0.05 + 1.5 * rng.uniform())) as i64;
        *g = (*g).clamp(0, 32767);
    }
    // A pilot lost in a fade now and then.
    if rng.next_u64().is_multiple_of(8) {
        gains[(rng.next_u64() % 4) as usize] = 0;
    }
    let gsum = gains.iter().sum::<i64>() * 13 + (rng.next_u64() % 1000) as i64;
    let c = tracker_constants(&channel(gains, gsum), fx);
    let ppm = (rng.uniform() - 0.5) * 2.0 * ppm_max;
    let ratio: u16 = if rng.next_u64().is_multiple_of(4) {
        0
    } else {
        3616
    };
    let (rate, cfo) = offset(ppm, ppm + 3.0 * rng.gaussian());
    let amp = gains.map(|g| (g as f64).sqrt() * 180.0);
    let amp = amp.map(|a| a.min(30000.0));
    Packet {
        cfo,
        ratio,
        c,
        rate,
        amp,
        noise: 30.0 + 2000.0 * rng.uniform(),
        wild: false,
        symbols,
    }
}

#[test]
fn architecture_matches_the_model_on_packets() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x7_2ac0);
    let (mut moves, mut learned) = (0, 0);
    for _ in 0..400 {
        let n = 1 + (rng.next_u64() % 300) as usize;
        let pk = real_packet(&mut rng, &fx, n, 300.0);
        if pk.c.usable(&fx) && n > 1 {
            learned += 1;
        }
        moves += run(&mut rng, &pk, &fx).0;
    }
    assert!(
        learned > 200 && moves > 100,
        "learned {learned}, moves {moves}"
    );
}

#[test]
fn architecture_matches_the_model_at_the_limits() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x7_2ac1);
    let mut top = 0;
    for case in 0..600 {
        let pick = |rng: &mut Rng, bits: u32| -> i64 {
            match rng.next_u64() % 4 {
                0 => 0,
                1 => (1 << bits) - 1,
                _ => (rng.next_u64() % (1 << bits)) as i64,
            }
        };
        let c = TrackerConstants {
            outer: pick(&mut rng, COEF_BITS),
            inner: pick(&mut rng, COEF_BITS),
            variance: match case % 3 {
                0 => 1 + (rng.next_u64() % 3072) as i64,
                1 => [1, 3072, 3073, 4095, 0][(rng.next_u64() % 5) as usize],
                _ => pick(&mut rng, VAR_BITS),
            },
        };
        let cfo_lim = 1i64 << (P - 1);
        let cfo = match rng.next_u64() % 3 {
            0 => -cfo_lim,
            1 => cfo_lim - 1,
            _ => (rng.next_u64() % (2 * cfo_lim as u64)) as i64 - cfo_lim,
        };
        let ratio = match rng.next_u64() % 4 {
            0 => 0,
            1 => u16::MAX,
            _ => rng.next_u64() as u16,
        };
        let pk = Packet {
            cfo,
            ratio,
            c,
            rate: (rng.uniform() - 0.5) * 4.0e6,
            amp: [1.0, 1.0, 1.0, 1.0].map(|_: f64| 32768.0 * rng.uniform()),
            noise: 20000.0 * rng.uniform(),
            wild: case % 2 == 0,
            symbols: 1 + (rng.next_u64() % 40) as usize,
        };
        top = top.max(run(&mut rng, &pk, &fx).2);
    }
    // Close to the bound of `the_fit_never_saturates`: the wild cases reach it.
    assert!(top > 1 << (FIT_QBITS - 1), "largest |delta| {top}");
}

#[test]
fn the_longest_frame_fits_the_registers() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x7_2ac2);
    let mut max_shift = 0;
    for k in 0..6 {
        let mut pk = real_packet(&mut rng, &fx, LAST_SYMBOL + 1, 0.0);
        // A variance just usable (the smallest λ) and a large drift, then the extremes; the
        // largest offsets drive the drift into its saturation, 64 samples.
        pk.c.variance = [1, 3072, 1, 3072, 1500, 7][k];
        let ppm = [700.0, -700.0, 200.0, -200.0, 1200.0, -1200.0][k];
        (pk.rate, pk.cfo) = offset(ppm, ppm);
        pk.ratio = 3616;
        pk.noise = [10.0, 10.0, 3000.0, 3000.0, 10.0, 10.0][k];
        pk.amp = [30000.0; 4];
        let (_, s, _) = run(&mut rng, &pk, &fx);
        max_shift = max_shift.max(s);
    }
    assert!(max_shift >= 60, "the shift reached only {max_shift}");
}

/// The fit's quotient is below 2^FIT_QBITS on every frame: for n measurements at τ = 7, 12, …
/// (SIGNAL first; a packet measures every symbol or none), |num| ≤ 2^19·Σ|n·τ − Στ| with z
/// saturated to P − 2 bits, and det ≥ n·(Στ² + λ_min) − (Στ)² with λ_min the smallest λ of a
/// usable packet (variance 1, the smaller weight). So the model's saturation of δ never acts,
/// and the divider's bound |num|·2^R < det·2^FIT_QBITS holds.
#[test]
fn the_fit_never_saturates() {
    let fx = FixedOfdmConfig::default();
    let w = weights(&fx);
    let lambda_min = rsh_even(i128::from(w.0.min(w.1)), V);
    assert!(lambda_min > 0);
    let zmax = 1i128 << (Z_BITS - 1);
    let mut worst = (0f64, 0usize);
    for n in 2..=(LAST_SYMBOL + 1) {
        let taus: Vec<i128> = (0..n).map(|i| 7 + 5 * i as i128).collect();
        let (s1, s2): (i128, i128) = (taus.iter().sum(), taus.iter().map(|t| t * t).sum());
        let n_ = n as i128;
        let num_max = zmax * taus.iter().map(|t| (n_ * t - s1).abs()).sum::<i128>();
        let det_min = n_ * (s2 + lambda_min) - s1 * s1;
        assert!(det_min > 0);
        let mag = num_max << R;
        assert!(mag >> FIT_QBITS < det_min, "n = {n}");
        let r = mag as f64 / det_min as f64;
        if r > worst.0 {
            worst = (r, n);
        }
    }
    // The largest quotient is at four measurements, about 5.8·10^7 against 2^26 = 6.7·10^7.
    assert_eq!(worst.1, 4, "worst {worst:?}");
}

#[test]
fn widths_are_the_defaults() {
    let fx = FixedOfdmConfig::default();
    assert_eq!(fx.front.nco_phase_bits, P);
    assert_eq!(fx.front.angle_bits, A_BITS);
    assert_eq!(fx.rate_frac, R);
    assert_eq!(fx.coef_frac, K);
    assert_eq!(fx.var_frac, V);
    assert_eq!(fx.prod_bits + 1, PILOT_BITS);
    // The weights fit the multiplier's B operand; λ for the largest usable variance fits.
    // rate0 never saturates: a P-bit word times a 16-bit ratio, rounded by 22 − R.
    for cfo in [-(1i128 << (P - 1)), (1 << (P - 1)) - 1] {
        assert!(fits(
            rsh_even(cfo * i128::from(u16::MAX), 22 - R),
            RATE0_BITS
        ));
    }
    let w = weights(&fx);
    assert!(w.0 > w.1 && w.1 > 0, "weights {w:?}");
    assert!(ufits(i128::from(w.0), MUL_B - 1));
    assert!(ufits(rsh_even(i128::from(w.0) * 4095, V), LAMBDA_BITS));
    assert!(bit_len(i128::from(w.0)) <= 25);
}
