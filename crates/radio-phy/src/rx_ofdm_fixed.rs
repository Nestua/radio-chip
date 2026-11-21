//! The OFDM receiver's data path in integers (R0-09 G2, stage A): FFT, channel estimate,
//! equaliser, demapper and soft values as the hardware computes them. The definitions, the
//! table of widths and the budget are in `docs/gates/rx_ofdm.md` (G0 of 2026-11-19, stage A);
//! the float receiver [`crate::rx_ofdm`] is the reference it is measured against (gate G2,
//! `tests/g2_rx_ofdm.rs`).
//!
//! The float receiver divides every data point by |H|² and multiplies its metric by the weight
//! |H|²/mean. Here the two cancel: with Hs = T1 + T2 (twice the channel), u = Z·Hs* and
//! g = |Hs|², the soft value is (2·√E·52·soft_max/clip)·m/Σg, where m is the demapper's value
//! on u with its thresholds scaled by g. One division per packet and modulation.
//!
//! The timing tracker (stage B, [`FixedTracking`]) is the float one's
//! ([`crate::rx_ofdm::Tracking`]) least-squares fit in its batch form: integer sums and one
//! division per symbol, the pilot pairs' weights three quotients per packet
//! ([`tracker_constants`]). No floating point between the samples and the soft values; the
//! tracker's start from a clock offset in ppm is the one conversion at the boundary.

use std::f64::consts::PI;

use crate::channel::Sample;
use crate::fixed::{quantize, Overflow, QFormat, Rounding};
use crate::interleaver::interleaved_index;
use crate::mapper::mean_energy;
use crate::modulation::Modulation;
use crate::ofdm::{
    data_subcarriers, fft_bin, pilot_polarity, FFT_SIZE, PILOT_BASE, PILOT_SUBCARRIERS,
};
use crate::ppdu::{data_symbols, long_training_sequence, MAX_PSDU_LEN};
use crate::rx_front_fixed::{
    cordic_table, cordic_with, nco_table, rotate_with, FixedRxConfig, IQ, Y_BITS,
};
use crate::rx_ofdm::{
    descramble, signal_from_soft, symbol_start, Decoded, RxOfdmConfig, RxOfdmError, SymbolLog,
    TIMING_MEASUREMENT_STD, TIMING_PRIOR_PPM,
};
use crate::viterbi::{decode, Termination};

/// Widths of the integer data path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedOfdmConfig {
    /// Bits Z of the input samples (the front's CFO-corrected stream,
    /// [`crate::rx_front_fixed::handoff_fixed`]).
    pub in_bits: u32,
    /// Left shift of the input into the FFT word (fractional bits kept through the stages).
    pub in_shift: u32,
    /// Bits W of every FFT stage and of the bins, saturating.
    pub fft_bits: u32,
    /// Bit s set: the outputs of stage s (0 first) are halved. Three bits set: the float
    /// transform over 8.
    pub fft_halve: u8,
    /// Bits T of the twiddle ROM, Q(T, T − 2) as the transmitter's.
    pub twiddle_bits: u32,
    /// Bits U of the products u = Z·Hs* and g = |Hs|² after the per-packet shift.
    pub prod_bits: u32,
    /// Fractional bits C of the demapper's threshold constants.
    pub kappa_bits: u32,
    /// Fractional bits F of the per-modulation scale constant.
    pub scale_frac: u32,
    /// Fractional bits R of the tracker's drift rate (units of 2π/2^P per subcarrier per fifth
    /// of a symbol).
    pub rate_frac: u32,
    /// Bits M the denominator of the tracker's per-packet quotients is normalised to.
    pub track_bits: u32,
    /// Fractional bits K of the two pilot-pair coefficients.
    pub coef_frac: u32,
    /// Fractional bits V of the measurement's relative variance.
    pub var_frac: u32,
    /// Half a table address is added to the data path's rotation phases before the table is
    /// addressed (rounding instead of the front's truncation).
    pub rot_round: bool,
    /// The front's rotator (P, L, Q) and CORDIC (N, W_c, A), shared.
    pub front: FixedRxConfig,
}

impl Default for FixedOfdmConfig {
    /// The working widths (`docs/gates/rx_ofdm.md`; the G2 sweep confirms or changes them).
    fn default() -> Self {
        Self {
            // ADR 0008, decision 1: the hand-off's 13 bits, one fractional bit.
            in_bits: Y_BITS,
            in_shift: 1,
            fft_bits: 16,
            fft_halve: 0b10_1010,
            twiddle_bits: 12,
            prod_bits: 16,
            kappa_bits: 10,
            scale_frac: 4,
            rate_frac: 10,
            track_bits: 16,
            coef_frac: 16,
            var_frac: 10,
            rot_round: true,
            front: FixedRxConfig::default(),
        }
    }
}

impl FixedOfdmConfig {
    /// Whether the widths are usable: every product fits 64 bits (W ≤ 30), the input fits
    /// the FFT word, the front's widths are valid. The functions here do nothing otherwise.
    pub fn is_valid(&self) -> bool {
        (2..=24).contains(&self.in_bits)
            && self.in_shift <= 8
            && self.in_bits + self.in_shift <= self.fft_bits
            && self.fft_bits <= 30
            && self.fft_halve < 64
            && (4..=24).contains(&self.twiddle_bits)
            && (6..=40).contains(&self.prod_bits)
            && (2..=20).contains(&self.kappa_bits)
            && self.scale_frac <= 8
            && self.rate_frac <= 20
            && (4..=30).contains(&self.track_bits)
            && (4..=24).contains(&self.coef_frac)
            && (2..=16).contains(&self.var_frac)
            && self.front.nco_phase_bits >= 8
            && self.front.is_valid()
            && self.front.angle_bits <= self.front.nco_phase_bits
    }
}

/// v / 2^s, rounded half to even.
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

/// `v` saturated to a `bits`-wide two's-complement word.
fn sat(v: i128, bits: u32) -> i64 {
    let top = (1i128 << (bits - 1)) - 1;
    v.clamp(-top - 1, top) as i64
}

fn bit_len(v: i128) -> u32 {
    i128::BITS - v.unsigned_abs().leading_zeros()
}

/// Float samples as `bits`-wide integers: round(x·`full_scale`), half away from zero,
/// saturated (an ADC and the scaling in front of it, for the gates and measurements). A
/// sample that is not a number is 0.
pub fn to_fixed(x: &[Sample], full_scale: f64, bits: u32) -> Vec<IQ> {
    let bits = bits.clamp(2, 48);
    let q = |v: f64| {
        let r = (v * full_scale).round();
        if r.is_nan() {
            0
        } else {
            sat(r.clamp(-1e18, 1e18) as i128, bits)
        }
    };
    x.iter().map(|s| (q(s.0), q(s.1))).collect()
}

/// e^(−j2πk/64), k < 32, in Q(T, T − 2), half to even: the transmitter's twiddle ROM
/// conjugated. A constant ROM in the RTL.
fn twiddles(fx: &FixedOfdmConfig) -> Option<[IQ; FFT_SIZE / 2]> {
    let format = QFormat::new(fx.twiddle_bits, fx.twiddle_bits - 2).ok()?;
    let mut rom = [(0i64, 0i64); FFT_SIZE / 2];
    for (k, w) in rom.iter_mut().enumerate() {
        let phase = 2.0 * PI * k as f64 / FFT_SIZE as f64;
        let q = |v: f64| quantize(v, format, Rounding::HalfEven, Overflow::Saturate).ok();
        *w = (q(phase.cos())?, q(-phase.sin())?);
    }
    Some(rom)
}

/// The 64-point forward FFT of 64 samples already in the W-bit word: radix 2, decimation in
/// time, every butterfly output narrowed to W bits (half to even, saturating), the stages of
/// [`FixedOfdmConfig::fft_halve`] halved. `None` for another length or unusable widths.
pub fn fft64_fixed(x: &[IQ], fx: &FixedOfdmConfig) -> Option<[IQ; FFT_SIZE]> {
    if !fx.is_valid() {
        return None;
    }
    fft_with(x, fx, &twiddles(fx)?)
}

// IEEE 802.11-2020 §17.3.5.10: the transmitter's 64-point inverse transform; this is its
// forward counterpart, over 8.
fn fft_with(x: &[IQ], fx: &FixedOfdmConfig, rom: &[IQ; FFT_SIZE / 2]) -> Option<[IQ; FFT_SIZE]> {
    if x.len() != FFT_SIZE {
        return None;
    }
    let ft = fx.twiddle_bits - 2;
    let w_bits = fx.fft_bits;
    let mut v = [(0i64, 0i64); FFT_SIZE];
    for (k, slot) in v.iter_mut().enumerate() {
        // Bit-reversed order; the input saturates into the word.
        let s = x[(k as u8).reverse_bits() as usize >> 2];
        *slot = (sat(i128::from(s.0), w_bits), sat(i128::from(s.1), w_bits));
    }
    let mut half = 1;
    let mut stage = 0;
    while half < FFT_SIZE {
        let step = FFT_SIZE / (2 * half);
        let down = ft + u32::from(fx.fft_halve >> stage & 1);
        let narrow = |t: i128| sat(rsh_even(t, down), w_bits);
        for start in (0..FFT_SIZE).step_by(2 * half) {
            for j in 0..half {
                let w = rom[j * step];
                let (a, b) = (v[start + j], v[start + j + half]);
                let (wr, wi) = (i128::from(w.0), i128::from(w.1));
                let (br, bi) = (i128::from(b.0), i128::from(b.1));
                // w·b with T − 2 fractional bits, exact.
                let (tr, ti) = (wr * br - wi * bi, wr * bi + wi * br);
                let (ar, ai) = (i128::from(a.0) << ft, i128::from(a.1) << ft);
                v[start + j] = (narrow(ar + tr), narrow(ai + ti));
                v[start + j + half] = (narrow(ar - tr), narrow(ai - ti));
            }
        }
        half *= 2;
        stage += 1;
    }
    Some(v)
}

/// The channel as the integer path keeps it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FixedChannel {
    /// Hs(k) = L(k)·(T1(k) + T2(k)) per FFT bin: twice the channel, W + 1 bits; 0 off the 52
    /// used subcarriers.
    pub hs: [IQ; FFT_SIZE],
    /// The per-packet right shift of u and g.
    pub shift: u32,
    /// g(k) = |Hs(k)|² after the shift, U − 1 bits.
    pub gain: [i64; FFT_SIZE],
    /// Σ g over the 52 used subcarriers.
    pub gain_sum: i64,
}

/// The ROMs and derived constants of one configuration.
struct Tables {
    twiddle: [IQ; FFT_SIZE / 2],
    nco: Vec<IQ>,
    cordic: Vec<i64>,
}

fn tables(fx: &FixedOfdmConfig) -> Option<Tables> {
    fx.is_valid().then_some(())?;
    Some(Tables {
        twiddle: twiddles(fx)?,
        nco: nco_table(&fx.front),
        cordic: cordic_table(&fx.front),
    })
}

fn window_fft(window: &[IQ], fx: &FixedOfdmConfig, t: &Tables) -> Option<[IQ; FFT_SIZE]> {
    if window.len() != FFT_SIZE {
        return None;
    }
    let x: Vec<IQ> = window
        .iter()
        .map(|s| {
            // The input saturates to its stated width, then takes its place in the word.
            (
                sat(i128::from(s.0), fx.in_bits) << fx.in_shift,
                sat(i128::from(s.1), fx.in_bits) << fx.in_shift,
            )
        })
        .collect();
    fft_with(&x, fx, &t.twiddle)
}

/// The channel from the two long training symbols: `ltf` holds T1 and T2 (128 samples from
/// the FFT window's start, `in_bits` wide). `None` if it is shorter or the widths unusable.
pub fn estimate_channel_fixed(ltf: &[IQ], fx: &FixedOfdmConfig) -> Option<FixedChannel> {
    channel_with(ltf, fx, &tables(fx)?)
}

fn channel_with(ltf: &[IQ], fx: &FixedOfdmConfig, t: &Tables) -> Option<FixedChannel> {
    let t1 = window_fft(ltf.get(..FFT_SIZE)?, fx, t)?;
    let t2 = window_fft(ltf.get(FFT_SIZE..2 * FFT_SIZE)?, fx, t)?;
    let mut hs = [(0i64, 0i64); FFT_SIZE];
    let mut full = [0i128; FFT_SIZE];
    // IEEE 802.11-2020 §17.3.3: L(k) = ±1 on the 52 used subcarriers.
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        if l != 0 {
            let b = fft_bin(k);
            let l = i64::from(l);
            hs[b] = (l * (t1[b].0 + t2[b].0), l * (t1[b].1 + t2[b].1));
            full[b] = i128::from(hs[b].0) * i128::from(hs[b].0)
                + i128::from(hs[b].1) * i128::from(hs[b].1);
        }
    }
    // One shift for the packet: the largest gain just fits U − 1 bits.
    let largest = full.iter().copied().max().unwrap_or(0);
    let shift = bit_len(largest).saturating_sub(fx.prod_bits - 1);
    let mut gain = [0i64; FFT_SIZE];
    for (g, f) in gain.iter_mut().zip(&full) {
        *g = sat(rsh_even(*f, shift), fx.prod_bits);
    }
    let gain_sum = gain.iter().sum();
    Some(FixedChannel {
        hs,
        shift,
        gain,
        gain_sum,
    })
}

/// a·e^(jφ) for a P-bit phase φ, narrowed back to U bits (half to even, saturating).
fn rotate(a: IQ, phase: i64, fx: &FixedOfdmConfig, t: &Tables) -> IQ {
    let step = fx.front.nco_phase_bits - fx.front.nco_lut_bits;
    let half = if fx.rot_round && step > 0 {
        1i64 << (step - 1)
    } else {
        0
    };
    let r = rotate_with(phase.wrapping_add(half), &fx.front, &t.nco);
    let (ar, ai) = (i128::from(a.0), i128::from(a.1));
    let (rr, ri) = (i128::from(r.0), i128::from(r.1));
    let down = fx.front.nco_amp_bits - 1;
    (
        sat(rsh_even(ar * rr - ai * ri, down), fx.prod_bits),
        sat(rsh_even(ar * ri + ai * rr, down), fx.prod_bits),
    )
}

/// κ_c = round(c/(2·√E)·2^C): the constant of the demapper's threshold at grid value `c`, a
/// ROM entry (for C = 10: 324 for 16-QAM's c = 2; 316 and 158 for 64-QAM's c = 4 and 2).
/// 0 for unusable widths.
pub fn threshold_constant(c: u32, modulation: Modulation, fx: &FixedOfdmConfig) -> i64 {
    if !fx.is_valid() {
        return 0;
    }
    let kappa = f64::from(c) / (2.0 * f64::from(mean_energy(modulation)).sqrt());
    (kappa * f64::from(1u32 << fx.kappa_bits)).round() as i64
}

/// A = round(2·√E·52·soft_max/clip·2^F): the constant the per-packet scale is made from, one
/// per modulation and soft format (for six bits, clip 2, F = 4: 25792, 36475, 81561, 167151).
/// 0 — every soft value an erasure — for a clip that is not a positive number, a negative
/// `soft_max` or unusable widths.
pub fn scale_constant(modulation: Modulation, cfg: &RxOfdmConfig, fx: &FixedOfdmConfig) -> i64 {
    if !fx.is_valid() {
        return 0;
    }
    let limit = f64::from(cfg.soft_max.max(0));
    let a = 2.0 * f64::from(mean_energy(modulation)).sqrt() * 52.0 * limit / cfg.clip
        * f64::from(1u32 << fx.scale_frac);
    if a.is_finite() && a > 0.0 {
        a.round().min(1e15) as i64
    } else {
        0
    }
}

/// round(κ_c·g): the demapper's threshold at grid value `c` on a subcarrier of gain `g`.
fn threshold(g: i64, c: u32, modulation: Modulation, fx: &FixedOfdmConfig) -> i64 {
    let k = i128::from(threshold_constant(c, modulation, fx));
    rsh_even(i128::from(g) * k, fx.kappa_bits) as i64
}

/// The demapper's values of one axis `v` for `n` bits (IEEE 802.11-2020 §17.3.5.8 Gray
/// levels, as [`crate::rx_ofdm::demap`]), thresholds scaled by the gain.
fn demap_axis(
    v: i64,
    g: i64,
    n: usize,
    modulation: Modulation,
    fx: &FixedOfdmConfig,
    out: &mut Vec<i64>,
) {
    out.push(-v);
    let mut fold = v.abs();
    let mut centre = 1u32 << (n - 1);
    for _ in 1..n {
        let m = fold - threshold(g, centre, modulation, fx);
        out.push(m);
        fold = m.abs();
        centre /= 2;
    }
}

/// The timing tracker's per-packet constants, from the channel gains of the four pilots.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TrackerConstants {
    /// Coefficient of the outer pair's angle (pilots at ±21), K fractional bits:
    /// 3·N_o/(14·D).
    pub outer: i64,
    /// Coefficient of the inner pair's angle (pilots at ±7), K fractional bits: N_i/(14·D).
    pub inner: i64,
    /// The measurement's variance against a flat channel of the same mean gain, V fractional
    /// bits: 5·Σg·a·b/(52·D). 0: neither pair has both its pilots (or the variance is below
    /// 2^−V).
    pub variance: i64,
}

impl TrackerConstants {
    /// Whether the packet's measurements go into the fit: a variance above 0 and at most 3
    /// nominal ones ([`crate::rx_ofdm::TIMING_MAX_VARIANCE`]).
    pub fn usable(&self, fx: &FixedOfdmConfig) -> bool {
        fx.var_frac <= 16 && self.variance > 0 && self.variance <= MAX_VARIANCE << fx.var_frac
    }
}

/// The tracker's constants for one packet. With g the gains at −21, −7, 7, 21, a = g₀ + g₃
/// and b = g₁ + g₂, the float tracker's pair weights g₀g₃/a and g₁g₂/b are cleared of their
/// divisions: N_o = g₀·g₃·b, N_i = g₁·g₂·a, D = 9·N_o + N_i. D is cut to M bits (and the
/// numerators by the same shift, truncating) and three quotients are taken: the slope left
/// in a symbol is `outer`·∠outer + `inner`·∠inner, its variance `variance`. All zero for
/// unusable widths, no pair with both its pilots, or gains no estimate can produce. A pair
/// whose two pilots are both gone has no weight; the other pair measures alone.
pub fn tracker_constants(ch: &FixedChannel, fx: &FixedOfdmConfig) -> TrackerConstants {
    constants_of(ch, fx).unwrap_or_default()
}

fn constants_of(ch: &FixedChannel, fx: &FixedOfdmConfig) -> Option<TrackerConstants> {
    fx.is_valid().then_some(())?;
    let mut g = [0i128; 4];
    for (slot, &c) in g.iter_mut().zip(&PILOT_SUBCARRIERS) {
        *slot = i128::from(ch.gain[fft_bin(c)].max(0));
    }
    // A pair with both pilots gone has no weight and must not take the other pair's with
    // it: its sum counts as 1 (the pair's own numerator is 0 either way).
    let (a, b) = (
        g[0].checked_add(g[3])?.max(1),
        g[1].checked_add(g[2])?.max(1),
    );
    let no = g[0].checked_mul(g[3])?.checked_mul(b)?;
    let ni = g[1].checked_mul(g[2])?.checked_mul(a)?;
    let d = no.checked_mul(9)?.checked_add(ni)?;
    if d <= 0 {
        return None;
    }
    let s = bit_len(d).saturating_sub(fx.track_bits);
    let dn = d >> s;
    // Rounded half up; everything here is non-negative.
    let coefficient = |num: i128| -> Option<i64> {
        let q = ((num << (fx.coef_frac + 1)).checked_add(14 * dn)?) / (28 * dn);
        i64::try_from(q).ok()
    };
    let spread = i128::from(ch.gain_sum.max(0))
        .checked_mul(a)?
        .checked_mul(b)?;
    let variance = ((spread >> s).checked_mul(5)? << fx.var_frac) / (52 * dn);
    Some(TrackerConstants {
        outer: coefficient(3 * (no >> s))?,
        inner: coefficient(ni >> s)?,
        variance: i64::try_from(variance).ok()?,
    })
}

/// A measurement whose variance is more than this many times the nominal one is not used
/// ([`crate::rx_ofdm::TIMING_MAX_VARIANCE`], an integer here).
const MAX_VARIANCE: i64 = 3;

/// Widths of the tracker's counting registers ([`FixedTracking`]): n, Στ, Στ². The others
/// follow the phase width P: z is P − 2 bits, Σz P + 9, Στz P + 21, δ P + R − 4.
const N_BITS: u32 = 11;
const S1_BITS: u32 = 23;
const S2_BITS: u32 = 35;

/// Fractional bits of the clock-ratio register ([`clock_ratio`]).
pub const CLOCK_RATIO_FRAC: u32 = 20;

/// The channel's fs/fc as the 16-bit register of ADR 0008's decision 7:
/// round(`sample_rate_hz`/`carrier_hz`·2^20). 3616 for 20 MHz at 5.8 GHz; ratios below
/// 1/16 fit (160 MHz at 2.6 GHz is 64 528). `None` when the ratio is not a number, rounds
/// to 0 or does not fit. A float helper for the MAC's side: the receiver sees the register only.
pub fn clock_ratio(sample_rate_hz: f64, carrier_hz: f64) -> Option<u16> {
    let v = (sample_rate_hz / carrier_hz * f64::from(1u32 << CLOCK_RATIO_FRAC)).round();
    (v >= 1.0 && v <= f64::from(u16::MAX)).then_some(v as u16)
}

/// The timing tracker's state across the symbols of a PPDU, in integers: the float
/// [`crate::rx_ofdm::Tracking`]'s fit measured(τ) = offset + rate·τ in its batch form. Time τ
/// is in fifths of a symbol from the middle of the long training field (7 for SIGNAL, then
/// 5 per symbol: 112 samples and 80), phases in units of 2π/2^P per subcarrier (one sample
/// of timing is 2^(P−6)), rates with R more fractional bits per unit of τ.
///
/// The sums n, Στ, Στ², Σz, Στz of the measurements z (each taken against the prior rate)
/// give the correction to the prior rate as one quotient per symbol,
/// δ = 2^R·(n·Στz − Στ·Σz) / (n·(Στ² + λ) − (Στ)²), truncated toward zero, where λ is the
/// prior's weight times the packet's variance.
///
/// The registers, for P = 22 and R = 10. z is 20 bits signed (P − 2) and saturates: 8
/// samples of timing, a prior on the clock offset wrong by 190 ppm over the longest frame
/// (the standard's worst case is 40). δ is 28 bits signed (P + R − 4) and saturates. n is
/// 11 bits, Στ 23 and Στ² 35 unsigned, Σz 31 (P + 9) and Στz 43 (P + 21) signed: the longest
/// frame, 1367 symbols with one measurement each, cannot overflow them (the model
/// saturates them all the same, for a caller that feeds a symbol twice). The numerator is
/// then 55 bits, 65 with its R fractional bits, and the divisor 43. With a signal present
/// nothing saturates; when the signal ends inside a frame the pilots are noise, the
/// measurement no longer answers the prediction and the rate runs away, in the float
/// tracker and here alike — here up to z's limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedTracking {
    /// Prior drift rate (R fractional bits).
    pub rate0: i64,
    /// Weight of the prior: 25·(σ/drift per symbol of the prior's deviation)², σ the nominal
    /// measurement's deviation ([`TIMING_MEASUREMENT_STD`]). Negative: the tracker learns
    /// nothing and applies `rate0`.
    pub weight: i64,
    /// Learned correction δ to `rate0` (R fractional bits).
    pub delta: i64,
    /// Whole samples the FFT window has been moved (negative: earlier).
    pub shift: i64,
    /// Slope removed from the last symbol's data: its drift plus what the window's moves
    /// took.
    pub theta: i64,
    /// The last symbol's drift, (rate0 + δ)·τ.
    pub drift: i64,
    /// The last symbol's measured slope against what was predicted.
    pub measured: i64,
    n: i128,
    s1: i128,
    s2: i128,
    y0: i128,
    y1: i128,
}

impl FixedTracking {
    /// A tracker whose prior is a clock offset of `ppm` (the transmitter fast against the
    /// receiver) known to `std_ppm` (1 σ); `std_ppm` of 0 or less: applied and never
    /// corrected. The one place a float enters: in hardware the rate comes from the front's
    /// CFO word times a constant of the channel, and the two weights used are ROM constants.
    /// Values that are not numbers give a rate of 0 and a tracker that learns nothing.
    pub fn start(ppm: f64, std_ppm: f64, fx: &FixedOfdmConfig) -> Self {
        let p = fx.front.nco_phase_bits.clamp(8, 40);
        // A fifth of a symbol is 16 samples; a sample of timing is 2^(P−6) units of slope.
        let per_ppm = 16e-6 * (1u64 << (p - 6)) as f64 * f64::from(1u32 << fx.rate_frac.min(20));
        let limit = (1u64 << (p - 6 + fx.rate_frac.min(20))) as f64;
        let rate0 = if ppm.is_finite() {
            (ppm * per_ppm).round().clamp(-limit, limit) as i64
        } else {
            0
        };
        // σ against the drift per symbol of one deviation of the prior, squared, in τ units.
        let per_symbol = 2.0 * PI / FFT_SIZE as f64 * 80e-6 * std_ppm;
        let weight = if std_ppm.is_finite() && std_ppm > 0.0 && ppm.is_finite() {
            (25.0 * (TIMING_MEASUREMENT_STD / per_symbol).powi(2))
                .round()
                .clamp(0.0, (1u64 << 40) as f64) as i64
        } else {
            -1
        };
        Self {
            rate0,
            weight,
            delta: 0,
            shift: 0,
            theta: 0,
            drift: 0,
            measured: 0,
            n: 0,
            s1: 0,
            s2: 0,
            y0: 0,
            y1: 0,
        }
    }

    /// Nothing known about the clock offset: the standard's worst case as the prior
    /// ([`TIMING_PRIOR_PPM`]).
    pub fn unknown(fx: &FixedOfdmConfig) -> Self {
        Self::start(0.0, TIMING_PRIOR_PPM, fx)
    }

    /// The start when the front's CFO gave the clock offset
    /// ([`crate::rx_ofdm::Tracking::from_clock_offset`]).
    pub fn from_clock_offset(ppm: f64, fx: &FixedOfdmConfig) -> Self {
        Self::start(ppm, crate::rx_ofdm::TIMING_HINT_PPM, fx)
    }

    /// The start from the front's CFO word, in integers (ADR 0008, decision 7): `cfo_word`
    /// in 2π/2^P per sample ([`crate::rx_front_fixed::FixedAcquisition::cfo_phase`], P the
    /// one of `fx.front`) and `ratio` the channel's fs/fc from [`clock_ratio`], the register
    /// the MAC writes when it tunes. Both ends derive carrier and sample clock from one
    /// oscillator (IEEE 802.11-2020 §17.3.9.5–6), so the clock offset is
    /// ε = word·(fs/fc)/2^P, the drift in a fifth of a symbol 16·ε samples of 2^(P−6) slope
    /// units each: rate0 = word·`ratio`/2^(22−R), half to even, in a signed register of
    /// P − 6 + R bits — one multiplication per packet, where the hand-off's stream starts.
    /// A P-bit word and a 16-bit ratio stay below 2^(P+R−7) and always fit; a word no
    /// accumulator holds saturates at the register's ends. The
    /// prior's weight is the constant of [`Self::from_clock_offset`]
    /// ([`crate::rx_ofdm::TIMING_HINT_PPM`]).
    pub fn from_cfo_word(cfo_word: i64, ratio: u16, fx: &FixedOfdmConfig) -> Self {
        let r = fx.rate_frac.min(20);
        let p = fx.front.nco_phase_bits.clamp(8, 40);
        let product = i128::from(cfo_word) * i128::from(ratio);
        let rate0 = sat(rsh_even(product, CLOCK_RATIO_FRAC + 2 - r), p - 6 + r);
        Self {
            rate0,
            ..Self::from_clock_offset(0.0, fx)
        }
    }

    /// A tracker that applies the drift of a clock offset of `ppm` and learns nothing.
    pub fn fixed(ppm: f64, fx: &FixedOfdmConfig) -> Self {
        Self::start(ppm, 0.0, fx)
    }

    /// How late the nominal window of the last symbol was, in samples.
    pub fn timing(&self, fx: &FixedOfdmConfig) -> f64 {
        self.drift as f64 / (1u64 << (fx.front.nco_phase_bits.clamp(8, 40) - 6)) as f64
    }

    /// (rate0 + δ)·τ for symbol `i`, in slope units.
    fn drift_at(&self, i: usize, fx: &FixedOfdmConfig) -> i64 {
        let tau = 7 + 5 * i as i128;
        let rate = i128::from(self.rate0) + i128::from(self.delta);
        sat(
            rsh_even(rate.saturating_mul(tau), fx.rate_frac),
            fx.front.nco_phase_bits + 1,
        )
    }

    /// The prior's weight for a packet: `weight`·variance, half to even. Negative for a
    /// tracker that learns nothing.
    pub fn lambda(&self, c: &TrackerConstants, fx: &FixedOfdmConfig) -> i128 {
        if self.weight < 0 || fx.var_frac > 16 {
            return -1;
        }
        rsh_even(
            i128::from(self.weight) * i128::from(c.variance.max(0)),
            fx.var_frac,
        )
    }

    /// One symbol's measurement into the fit: `err` is the slope measured in symbol `i`
    /// (0 for SIGNAL) after the predicted drift was removed, `lambda` the prior's weight for
    /// this packet ([`FixedTracking::lambda`]). Updates δ. A tracker that learns nothing, a
    /// negative `lambda`, unusable widths or a symbol index beyond any frame's (1366) leave
    /// δ as it is.
    pub fn learn(&mut self, err: i64, i: usize, lambda: i128, fx: &FixedOfdmConfig) {
        self.measured = err;
        if self.weight < 0 || lambda < 0 || i > 1366 || !fx.is_valid() {
            return;
        }
        // Unsigned registers saturate at 2^bits − 1.
        let cap = |v: i128, bits: u32| v.clamp(0, (1i128 << bits) - 1);
        let tau = 7 + 5 * i as i128;
        // The measurement against the prior rate: what was predicted with δ comes back.
        let back = rsh_even(i128::from(self.delta) * tau, fx.rate_frac);
        let p = fx.front.nco_phase_bits;
        let z = i128::from(sat(i128::from(err).saturating_add(back), p - 2));
        self.n = cap(self.n + 1, N_BITS);
        self.s1 = cap(self.s1 + tau, S1_BITS);
        self.s2 = cap(self.s2 + tau * tau, S2_BITS);
        self.y0 = i128::from(sat(self.y0 + z, p + 9));
        self.y1 = i128::from(sat(self.y1 + tau * z, p + 21));
        // λ beyond 64 bits is a prior nothing outweighs; the divisor stays in range.
        let lambda = lambda.min(1i128 << 64);
        let det = self.n * (self.s2 + lambda) - self.s1 * self.s1;
        if det > 0 {
            let num = self.n * self.y1 - self.s1 * self.y0;
            self.delta = sat((num << fx.rate_frac) / det, p + fx.rate_frac - 4);
        }
    }
}

/// `v` wrapped to a `bits`-wide two's-complement word.
fn wrap(v: i128, bits: u32) -> i64 {
    let full = 1i128 << bits;
    ((v + full / 2).rem_euclid(full) - full / 2) as i64
}

/// The tracker's step for one symbol: the pilots `p` at −21, −7, 7, 21 (U bits, the
/// transmitted signs removed), symbol `i`. The angles of the two pairs' products come from
/// the CORDIC, widened to P bits; the predicted slope's turn (42 and 14 subcarriers of it)
/// is subtracted from them modulo a turn, which is the float tracker's rotation of the pilots
/// before it measures. Leaves the slope to remove in `tracking.theta`.
fn track_fixed(
    p: &[IQ; 4],
    ch: &FixedChannel,
    i: usize,
    tracking: &mut FixedTracking,
    fx: &FixedOfdmConfig,
    t: &Tables,
) {
    let p_bits = fx.front.nco_phase_bits;
    let moved = i128::from(tracking.shift) << (p_bits - 6);
    let predicted = i128::from(tracking.drift_at(i, fx)) + moved;
    let c = tracker_constants(ch, fx);
    let angle = |hi: IQ, lo: IQ, span: i128| -> i128 {
        // hi·conj(lo), cut to the CORDIC's 64-bit inputs (it normalises further itself).
        let (ar, ai) = (i128::from(hi.0), i128::from(hi.1));
        let (br, bi) = (i128::from(lo.0), i128::from(lo.1));
        let (re, im) = (ar * br + ai * bi, ai * br - ar * bi);
        let s = bit_len(re).max(bit_len(im)).saturating_sub(62);
        let a = cordic_with((re >> s) as i64, (im >> s) as i64, &fx.front, &t.cordic);
        let wide = i128::from(a) << (p_bits - fx.front.angle_bits);
        i128::from(wrap(wide - span * predicted, p_bits))
    };
    let (outer, inner) = (angle(p[3], p[0], 42), angle(p[2], p[1], 14));
    let err = rsh_even(
        i128::from(c.outer) * outer + i128::from(c.inner) * inner,
        fx.coef_frac,
    );
    // |e| is at most half a turn over 14: it fits P − 2 bits; the saturation is for
    // coefficients no estimate produces.
    let err = sat(err, p_bits);
    if c.usable(fx) {
        tracking.learn(err, i, tracking.lambda(&c, fx), fx);
    } else {
        tracking.measured = err;
    }
    tracking.drift = tracking.drift_at(i, fx);
    tracking.theta = sat(i128::from(tracking.drift) + moved, p_bits + 1);
}

/// The soft values of one symbol in coded order (deinterleaved), 48·N_BPSC of them, and the
/// common phase removed (units of 2π/2^P): `window` is the symbol's 64-sample FFT window as
/// the tracker has moved it, `polarity_index` 0 for SIGNAL and k + 1 for DATA symbol k.
/// `None` for a short window or unusable widths.
pub fn symbol_soft_fixed(
    window: &[IQ],
    ch: &FixedChannel,
    polarity_index: usize,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
    fx: &FixedOfdmConfig,
    tracking: &mut FixedTracking,
) -> Option<(Vec<i32>, i64)> {
    symbol_with(
        window,
        ch,
        polarity_index,
        modulation,
        cfg,
        fx,
        tracking,
        &tables(fx)?,
    )
}

#[allow(clippy::too_many_arguments)]
fn symbol_with(
    window: &[IQ],
    ch: &FixedChannel,
    polarity_index: usize,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
    fx: &FixedOfdmConfig,
    tracking: &mut FixedTracking,
    t: &Tables,
) -> Option<(Vec<i32>, i64)> {
    // No estimate has a shift this large (a gain is at most 2·W + 3 bits); a record built by
    // hand must not reach the shifter with one.
    if ch.shift > 2 * fx.fft_bits + 3 {
        return None;
    }
    let z = window_fft(window, fx, t)?;
    let p_bits = fx.front.nco_phase_bits;
    // u(k) = Z(k)·Hs*(k) after the packet's shift, U bits.
    let u = |c: i8| -> IQ {
        let b = fft_bin(c);
        let (zr, zi) = (i128::from(z[b].0), i128::from(z[b].1));
        let (hr, hi) = (i128::from(ch.hs[b].0), i128::from(ch.hs[b].1));
        (
            sat(rsh_even(zr * hr + zi * hi, ch.shift), fx.prod_bits),
            sat(rsh_even(zi * hr - zr * hi, ch.shift), fx.prod_bits),
        )
    };
    let (mut theta, mut phi) = (0i64, 0i64);
    if cfg.pilot_tracking {
        // IEEE 802.11-2020 §17.3.5.9: pilots at −21, −7, 7, 21, base {1, 1, 1, −1} times the
        // polarity sequence p_n (index 0 for SIGNAL).
        let polarity = i64::from(pilot_polarity(polarity_index));
        let mut p = [(0i64, 0i64); 4];
        for ((slot, &c), &base) in p.iter_mut().zip(&PILOT_SUBCARRIERS).zip(&PILOT_BASE) {
            let s = polarity * i64::from(base);
            let v = u(c);
            *slot = (s * v.0, s * v.1);
        }
        if cfg.timing_tracking {
            track_fixed(&p, ch, polarity_index, tracking, fx, t);
            theta = tracking.theta;
        }
        let mut sum = (0i64, 0i64);
        for (v, &c) in p.iter().zip(&PILOT_SUBCARRIERS) {
            let r = rotate(*v, -i64::from(c) * theta, fx, t);
            sum = (sum.0 + r.0, sum.1 + r.1);
        }
        // The CORDIC's angle in A bits, widened to the rotator's P.
        phi = cordic_with(sum.0, sum.1, &fx.front, &t.cordic) << (p_bits - fx.front.angle_bits);
    }
    let n_bpsc = modulation.bits_per_subcarrier();
    // Per packet and modulation in hardware: A·2ⁿ/Σg, n the bit length of Σg.
    let limit = i128::from(cfg.soft_max.max(0));
    let a = i128::from(scale_constant(modulation, cfg, fx));
    let n = bit_len(i128::from(ch.gain_sum));
    let q = if ch.gain_sum > 0 {
        (a << n) / i128::from(ch.gain_sum)
    } else {
        0
    };
    let down = n + fx.scale_frac;
    let soft = |m: i64| -> i32 {
        let mag = i128::from(m).abs() * q;
        // Half away from zero, as the float contract.
        let r = if down == 0 {
            mag
        } else {
            (mag + (1i128 << (down - 1))) >> down
        };
        let r = r.min(limit) as i32;
        if m < 0 {
            -r
        } else {
            r
        }
    };
    let mut on_air = Vec::with_capacity(48 * n_bpsc);
    let mut m = Vec::with_capacity(n_bpsc);
    for c in data_subcarriers() {
        let v = rotate(u(c), -(phi + i64::from(c) * theta), fx, t);
        let g = ch.gain[fft_bin(c)];
        m.clear();
        if modulation == Modulation::Bpsk {
            demap_axis(v.0, g, 1, modulation, fx, &mut m);
        } else {
            demap_axis(v.0, g, n_bpsc / 2, modulation, fx, &mut m);
            demap_axis(v.1, g, n_bpsc / 2, modulation, fx, &mut m);
        }
        on_air.extend(m.iter().map(|&v| soft(v)));
    }
    // The window follows the slope left in it, as the float receiver's.
    if cfg.pilot_tracking && cfg.timing_tracking {
        // Half a sample of slope: 2^(P−7).
        let half = 1i64 << (p_bits - 7);
        if tracking.theta > half {
            tracking.shift = tracking.shift.saturating_sub(1);
        } else if tracking.theta < -half {
            tracking.shift = tracking.shift.saturating_add(1);
        }
    }
    let coded = (0..on_air.len())
        .map(|k| {
            on_air
                .get(interleaved_index(k, modulation))
                .copied()
                .unwrap_or(0)
        })
        .collect();
    Some((coded, phi))
}

/// The symbol's window as the tracker has moved it.
fn window_of<'a>(
    z: &'a [IQ],
    fft_start: usize,
    i: usize,
    tracking: &FixedTracking,
) -> Result<&'a [IQ], RxOfdmError> {
    let start = i64::try_from(symbol_start(fft_start, i))
        .ok()
        .and_then(|n| n.checked_add(tracking.shift))
        .and_then(|n| usize::try_from(n).ok());
    let end = start.and_then(|s| s.checked_add(FFT_SIZE));
    start
        .zip(end)
        .and_then(|(s, e)| z.get(s..e))
        .ok_or(RxOfdmError::Truncated {
            needed: end.unwrap_or(usize::MAX),
            have: z.len(),
        })
}

/// Decodes the PPDU whose first long training symbol's FFT window starts at `fft_start`, from
/// `in_bits`-wide samples, with the timing tracker started from `tracking`: the integer
/// counterpart of [`crate::rx_ofdm::decode_ppdu_from`]. Unusable widths are reported as a
/// truncated stream of no samples.
pub fn decode_ppdu_fixed(
    z: &[IQ],
    fft_start: usize,
    cfg: &RxOfdmConfig,
    fx: &FixedOfdmConfig,
    mut tracking: FixedTracking,
) -> Result<Decoded, RxOfdmError> {
    let unusable = RxOfdmError::Truncated { needed: 0, have: 0 };
    let t = tables(fx).ok_or(unusable)?;
    let needed = fft_start.saturating_add(2 * FFT_SIZE);
    let short = RxOfdmError::Truncated {
        needed,
        have: z.len(),
    };
    let ch = z
        .get(fft_start..needed)
        .and_then(|ltf| channel_with(ltf, fx, &t))
        .ok_or(short)?;
    let phase = |phi: i64| phi as f64 * 2.0 * PI / (1u64 << fx.front.nco_phase_bits) as f64;
    let symbol = |i: usize, m: Modulation, tr: &mut FixedTracking| {
        let w = window_of(z, fft_start, i, tr)?;
        symbol_with(w, &ch, i, m, cfg, fx, tr, &t).ok_or(short)
    };
    let (soft, phi) = symbol(0, Modulation::Bpsk, &mut tracking)?;
    let signal = signal_from_soft(&soft)?;
    let mut log = SymbolLog {
        common_phase: vec![phase(phi)],
        timing: vec![tracking.timing(fx)],
    };
    if !(1..=MAX_PSDU_LEN).contains(&signal.length) {
        return Err(RxOfdmError::SignalLength);
    }
    let symbols = data_symbols(signal.rate, signal.length);
    let end = symbol_start(fft_start, symbols).saturating_add(FFT_SIZE);
    if z.len() < end {
        return Err(RxOfdmError::Truncated {
            needed: end,
            have: z.len(),
        });
    }
    let modulation = signal.rate.modulation();
    let mut all = Vec::with_capacity(symbols * 48 * modulation.bits_per_subcarrier());
    for i in 1..=symbols {
        let (soft, phi) = symbol(i, modulation, &mut tracking)?;
        all.extend(soft);
        log.common_phase.push(phase(phi));
        log.timing.push(tracking.timing(fx));
    }
    // §17.3.5.3: the encoder is back in state 0 after SERVICE, the PSDU and the six tail bits.
    let tail = 16 + 8 * signal.length + 6;
    let bits = decode(&all, signal.rate.code_rate(), Termination::ZeroAfter(tail))
        .map_err(RxOfdmError::Viterbi)?;
    descramble(&bits, signal, log)
}
