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
//! Stage A leaves the timing tracker's arithmetic in float ([`crate::rx_ofdm::Tracking`]):
//! it takes the four pilots and their gains as integers and returns the slope as a phase.

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
use crate::rx_front_fixed::{cordic_table, cordic_with, nco_table, rotate_with, FixedRxConfig, IQ};
use crate::rx_ofdm::{
    descramble, signal_from_soft, symbol_start, track_timing, Decoded, RxOfdmConfig, RxOfdmError,
    SymbolLog, Tracking, ONE_SAMPLE,
};
use crate::viterbi::{decode, Termination};

/// Widths of the integer data path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedOfdmConfig {
    /// Bits Z of the input samples (the front's CFO-corrected stream).
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
    /// The front's rotator (P, L, Q) and CORDIC (N, W_c, A), shared.
    pub front: FixedRxConfig,
}

impl Default for FixedOfdmConfig {
    /// The working widths (`docs/gates/rx_ofdm.md`; the G2 sweep confirms or changes them).
    fn default() -> Self {
        Self {
            in_bits: 12,
            in_shift: 2,
            fft_bits: 16,
            fft_halve: 0b10_1010,
            twiddle_bits: 12,
            prod_bits: 16,
            kappa_bits: 10,
            scale_frac: 4,
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
    let r = rotate_with(phase, &fx.front, &t.nco);
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
    tracking: &mut Tracking,
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
    tracking: &mut Tracking,
    t: &Tables,
) -> Option<(Vec<i32>, i64)> {
    // No estimate has a shift this large (a gain is at most 2·W + 3 bits); a record built by
    // hand must not reach the shifter with one.
    if ch.shift > 2 * fx.fft_bits + 3 {
        return None;
    }
    let z = window_fft(window, fx, t)?;
    let p_bits = fx.front.nco_phase_bits;
    let turn_units = (1u64 << p_bits) as f64;
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
            // Stage B: this step in integers. Today the float tracker on the integer pilots.
            let pf = p.map(|v| (v.0 as f64, v.1 as f64));
            let gain = PILOT_SUBCARRIERS.map(|c| ch.gain[fft_bin(c)] as f64);
            track_timing(
                &pf,
                gain,
                ch.gain_sum as f64 / 52.0,
                polarity_index,
                tracking,
            );
            let slope = tracking.slope / (2.0 * PI) * turn_units;
            // A slope that is not a number (a hostile tracker state) removes nothing.
            theta = if slope.is_finite() {
                slope.round().clamp(-turn_units, turn_units) as i64
            } else {
                0
            };
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
        if tracking.slope > ONE_SAMPLE / 2.0 {
            tracking.shift = tracking.shift.saturating_sub(1);
        } else if tracking.slope < -ONE_SAMPLE / 2.0 {
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
    tracking: &Tracking,
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
    mut tracking: Tracking,
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
    let symbol = |i: usize, m: Modulation, tr: &mut Tracking| {
        let w = window_of(z, fft_start, i, tr)?;
        symbol_with(w, &ch, i, m, cfg, fx, tr, &t).ok_or(short)
    };
    let (soft, phi) = symbol(0, Modulation::Bpsk, &mut tracking)?;
    let signal = signal_from_soft(&soft)?;
    let mut log = SymbolLog {
        common_phase: vec![phase(phi)],
        timing: vec![tracking.timing()],
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
        log.timing.push(tracking.timing());
    }
    // §17.3.5.3: the encoder is back in state 0 after SERVICE, the PSDU and the six tail bits.
    let tail = 16 + 8 * signal.length + 6;
    let bits = decode(&all, signal.rate.code_rate(), Termination::ZeroAfter(tail))
        .map_err(RxOfdmError::Viterbi)?;
    descramble(&bits, signal, log)
}
