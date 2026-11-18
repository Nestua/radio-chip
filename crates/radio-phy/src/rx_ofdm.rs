//! OFDM receiver after acquisition (R0-09), float reference: channel estimate from the long
//! training field, pilot phase tracking, one-tap equaliser, soft demapper, and the decode of
//! SIGNAL and DATA down to the PSDU (IEEE 802.11-2020 §17.3). The mirror of [`crate::ppdu`].
//!
//! Input: the stream after the front's CFO correction ([`crate::rx_front::correct`]) and the
//! first sample of T1's FFT window ([`crate::rx_front::Acquisition::fft_start`], which already
//! includes the timing back-off). Definitions that the fixed-point model and the RTL will have
//! to match — the soft-value scaling above all — are in `docs/gates/rx_ofdm.md`.

use crate::channel::{fft, Sample};
use crate::convolutional::CodeRate;
use crate::interleaver::interleaved_index;
use crate::mapper::mean_energy;
use crate::modulation::Modulation;
use crate::ofdm::{
    data_subcarriers, fft_bin, pilot_polarity, CP_LEN, FFT_SIZE, PILOT_BASE, PILOT_SUBCARRIERS,
    SYMBOL_LEN,
};
use crate::ppdu::{data_symbols, long_training_sequence, Rate, MAX_PSDU_LEN};
use crate::scrambler::Scrambler;
use crate::viterbi::{decode, Termination, ViterbiError, SOFT_CLIP, SOFT_MAX};

/// A channel estimate: the complex gain of each FFT bin (zero on unused bins).
pub type ChannelEstimate = [Sample; FFT_SIZE];

/// Receiver settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RxOfdmConfig {
    /// Largest soft magnitude handed to the Viterbi decoder.
    pub soft_max: i32,
    /// Soft metric (in half minimum distances of the constellation, on a subcarrier of mean
    /// gain) that maps to `soft_max`.
    pub clip: f64,
    /// Remove the common phase of each symbol measured on its four pilots.
    pub pilot_tracking: bool,
}

impl Default for RxOfdmConfig {
    /// The Viterbi decoder's contract: 4-bit soft values, clip 1.5 (`docs/gates/viterbi.md`).
    fn default() -> Self {
        Self {
            soft_max: SOFT_MAX,
            clip: SOFT_CLIP,
            pilot_tracking: true,
        }
    }
}

/// Why a PPDU was not decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxOfdmError {
    /// The stream ends before the field: `needed` samples, `have` given.
    Truncated {
        /// Samples the field needs.
        needed: usize,
        /// Samples in the stream.
        have: usize,
    },
    /// SIGNAL fails its even parity.
    SignalParity,
    /// SIGNAL's RATE bits are none of the eight rates.
    SignalRate([u8; 4]),
    /// SIGNAL's LENGTH is 0.
    SignalLength,
    /// The Viterbi decoder rejected the stream (cannot happen for whole symbols).
    Viterbi(ViterbiError),
    /// The first seven descrambled bits fit no scrambler state (they are all zero).
    ScramblerSeed,
}

impl std::fmt::Display for RxOfdmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RxOfdmError::Truncated { needed, have } => {
                write!(f, "stream of {have} samples, the field needs {needed}")
            }
            RxOfdmError::SignalParity => write!(f, "SIGNAL parity"),
            RxOfdmError::SignalRate(r) => write!(f, "SIGNAL rate bits {r:?}"),
            RxOfdmError::SignalLength => write!(f, "SIGNAL length 0"),
            RxOfdmError::Viterbi(e) => write!(f, "{e}"),
            RxOfdmError::ScramblerSeed => write!(f, "no scrambler seed"),
        }
    }
}

impl std::error::Error for RxOfdmError {}

/// The decoded SIGNAL field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Signal {
    /// Rate of the DATA field.
    pub rate: Rate,
    /// PSDU length in octets, 1…4095.
    pub length: usize,
}

/// A decoded PPDU.
#[derive(Debug, Clone, PartialEq)]
pub struct Decoded {
    /// The SIGNAL field.
    pub signal: Signal,
    /// The PSDU, `signal.length` octets.
    pub psdu: Vec<u8>,
    /// The transmitter's scrambler seed, recovered from the SERVICE field.
    pub scrambler_seed: u8,
    /// Common phase removed from each symbol (SIGNAL first), radians.
    pub common_phase: Vec<f64>,
}

/// One equalised symbol: the 48 data points on the unnormalised constellation grid (±1, ±3, …)
/// and the weight of each (|H|² over the mean |H|² of the used subcarriers).
#[derive(Debug, Clone, PartialEq)]
pub struct Equalised {
    /// Data points in subcarrier order (−26 … 26 without DC and pilots), grid units.
    pub points: Vec<Sample>,
    /// Channel-state weight of each point.
    pub weights: Vec<f64>,
    /// Common phase measured on the pilots (0 without tracking), radians.
    pub common_phase: f64,
}

fn cmul(a: Sample, b: Sample) -> Sample {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn conj(a: Sample) -> Sample {
    (a.0, -a.1)
}

fn abs2(a: Sample) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// The 64-point FFT of a 64-sample window (`None` for any other length).
fn fft64(window: &[Sample]) -> Option<Vec<Sample>> {
    (window.len() == FFT_SIZE)
        .then(|| fft(window).ok())
        .flatten()
}

/// Least-squares channel estimate from the two long training symbols: `ltf` holds T1 and T2
/// (128 samples from the FFT window's start). `None` if it is shorter.
///
/// H(k) = (T1(k) + T2(k)) / (2·L(k)) on the 52 used subcarriers (IEEE 802.11-2020 §17.3.3,
/// L(k) = ±1). No smoothing across subcarriers: the estimate's noise is half a data symbol's.
pub fn estimate_channel(ltf: &[Sample]) -> Option<ChannelEstimate> {
    let t1 = fft64(ltf.get(..FFT_SIZE)?)?;
    let t2 = fft64(ltf.get(FFT_SIZE..2 * FFT_SIZE)?)?;
    let mut h = [(0.0, 0.0); FFT_SIZE];
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        if l != 0 {
            let b = fft_bin(k);
            let s = 0.5 * f64::from(l);
            h[b] = (s * (t1[b].0 + t2[b].0), s * (t1[b].1 + t2[b].1));
        }
    }
    Some(h)
}

/// Soft metrics of one axis value `v` (grid units) for `n` bits, appended to `out`.
///
/// Gray levels of §17.3.5.8: the first bit is the sign, each further bit folds the axis about
/// the middle of what is left (2 for 16-QAM; 4, then 2 for 64-QAM). The first bit's metric is
/// the distance to its only boundary, 0; this is the exact max-log metric near the boundaries
/// and smaller in magnitude away from them (`g1_rx_ofdm`).
fn demap_axis(v: f64, n: usize, out: &mut Vec<f64>) {
    // Bit 0 = 0 is the negative half.
    out.push(-v);
    let mut fold = v.abs();
    let mut centre = f64::from(1u32 << (n - 1));
    for _ in 1..n {
        // Outer levels carry 0, inner levels 1.
        let m = fold - centre;
        out.push(m);
        fold = m.abs();
        centre /= 2.0;
    }
}

/// Max-log soft metrics of one point, N_BPSC values, b0 first: positive means bit 0, and the
/// unit is half the minimum distance (a noiseless BPSK point gives ±1).
pub fn demap(point: Sample, modulation: Modulation) -> Vec<f64> {
    let n = modulation.bits_per_subcarrier();
    let mut out = Vec::with_capacity(n);
    if modulation == Modulation::Bpsk {
        demap_axis(point.0, 1, &mut out);
    } else {
        demap_axis(point.0, n / 2, &mut out);
        demap_axis(point.1, n / 2, &mut out);
    }
    out
}

/// Equalises one symbol: `window` is its 64-sample FFT window, `polarity_index` the pilot
/// polarity index (0 for SIGNAL, k + 1 for DATA symbol k). `None` for a short window.
///
/// With Z the window's FFT: the pilots give the common phase φ = ∠Σ Z(p)·H*(p)·s(p) (a
/// maximum-ratio sum over the four pilots, s the transmitted ±1); each data point is
/// Z(k)·H*(k)·e^(−jφ) / (|H(k)|²·K_MOD), back on the integer grid.
pub fn equalise(
    window: &[Sample],
    h: &ChannelEstimate,
    polarity_index: usize,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
) -> Option<Equalised> {
    let z = fft64(window)?;
    let common_phase = if cfg.pilot_tracking {
        let polarity = f64::from(pilot_polarity(polarity_index));
        let sum = PILOT_SUBCARRIERS
            .iter()
            .zip(&PILOT_BASE)
            .fold((0.0, 0.0), |a, (&c, &base)| {
                let b = fft_bin(c);
                let t = cmul(z[b], conj(h[b]));
                let s = polarity * f64::from(base);
                (a.0 + s * t.0, a.1 + s * t.1)
            });
        sum.1.atan2(sum.0)
    } else {
        0.0
    };
    let derotate = (common_phase.cos(), -common_phase.sin());
    let used = (-26i8..=26).filter(|&k| k != 0);
    let mean_gain = used.map(|k| abs2(h[fft_bin(k)])).sum::<f64>() / 52.0;
    let grid = f64::from(mean_energy(modulation)).sqrt();
    let (mut points, mut weights) = (Vec::with_capacity(48), Vec::with_capacity(48));
    for c in data_subcarriers() {
        let b = fft_bin(c);
        let gain = abs2(h[b]);
        let v = cmul(cmul(z[b], conj(h[b])), derotate);
        // A subcarrier with no gain carries no information: weight 0, any point.
        let scale = if gain > 0.0 { grid / gain } else { 0.0 };
        points.push((v.0 * scale, v.1 * scale));
        weights.push(if mean_gain > 0.0 {
            gain / mean_gain
        } else {
            0.0
        });
    }
    Some(Equalised {
        points,
        weights,
        common_phase,
    })
}

/// The soft values of one equalised symbol in coded order (deinterleaved), 48·N_BPSC values:
/// weight × metric × soft_max / clip, rounded half away from zero and saturated to ±soft_max.
/// This scaling is the demapper's contract with the Viterbi decoder (`docs/gates/viterbi.md`).
pub fn soft_bits(eq: &Equalised, modulation: Modulation, cfg: &RxOfdmConfig) -> Vec<i32> {
    let scale = f64::from(cfg.soft_max) / cfg.clip;
    let limit = f64::from(cfg.soft_max);
    let on_air: Vec<i32> = eq
        .points
        .iter()
        .zip(&eq.weights)
        .flat_map(|(p, w)| {
            demap(*p, modulation)
                .into_iter()
                .map(move |m| (m * w * scale).round().clamp(-limit, limit) as i32)
        })
        .collect();
    // The interleaver sends coded bit k to position interleaved_index(k).
    (0..on_air.len())
        .map(|k| on_air[interleaved_index(k, modulation)])
        .collect()
}

/// First sample of the FFT window of symbol `i` after the long training field (0 = SIGNAL),
/// keeping the back-off `fft_start` has inside the cyclic prefix.
fn symbol_start(fft_start: usize, i: usize) -> usize {
    fft_start + 2 * FFT_SIZE + CP_LEN + SYMBOL_LEN * i
}

fn symbol_soft(
    y: &[Sample],
    fft_start: usize,
    i: usize,
    h: &ChannelEstimate,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
) -> Result<(Vec<i32>, f64), RxOfdmError> {
    let start = symbol_start(fft_start, i);
    let truncated = RxOfdmError::Truncated {
        needed: start + FFT_SIZE,
        have: y.len(),
    };
    let window = y.get(start..start + FFT_SIZE).ok_or(truncated)?;
    let eq = equalise(window, h, i, modulation, cfg).ok_or(truncated)?;
    Ok((soft_bits(&eq, modulation, cfg), eq.common_phase))
}

fn signal_with_phase(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
) -> Result<(Signal, f64), RxOfdmError> {
    // IEEE 802.11-2020 §17.3.4: one BPSK rate-1/2 symbol; RATE, reserved, LENGTH (LSB first),
    // even parity over the first 17 bits, six tail zeros.
    let (soft, phase) = symbol_soft(y, fft_start, 0, h, Modulation::Bpsk, cfg)?;
    let bits =
        decode(&soft, CodeRate::R1_2, Termination::ZeroAfter(24)).map_err(RxOfdmError::Viterbi)?;
    let field = bits.get(..18).ok_or(RxOfdmError::SignalParity)?;
    if field.iter().fold(0u8, |p, b| p ^ b) != 0 {
        return Err(RxOfdmError::SignalParity);
    }
    let rate_bits = [field[0], field[1], field[2], field[3]];
    let rate = Rate::from_signal_bits(rate_bits).ok_or(RxOfdmError::SignalRate(rate_bits))?;
    let length = (5..17)
        .rev()
        .fold(0usize, |a, i| (a << 1) | usize::from(field[i]));
    if !(1..=MAX_PSDU_LEN).contains(&length) {
        return Err(RxOfdmError::SignalLength);
    }
    Ok((Signal { rate, length }, phase))
}

/// Decodes the SIGNAL field with the given channel estimate; `fft_start` as in [`decode_ppdu`].
///
/// The reserved bit is not checked (a receiver must not reject a frame for it, §17.3.4.3).
pub fn decode_signal(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
) -> Result<Signal, RxOfdmError> {
    signal_with_phase(y, fft_start, h, cfg).map(|s| s.0)
}

/// The decoded DATA field before descrambling: N_SYM·N_DBPS bits (SERVICE, PSDU, tail, pad as
/// the transmitter's [`crate::ppdu::data_bits`]), and the common phase of each DATA symbol.
pub fn decode_data_bits(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    signal: Signal,
    cfg: &RxOfdmConfig,
) -> Result<(Vec<u8>, Vec<f64>), RxOfdmError> {
    let Signal { rate, length } = signal;
    let symbols = data_symbols(rate, length);
    let needed = symbol_start(fft_start, symbols) + FFT_SIZE;
    if y.len() < needed {
        return Err(RxOfdmError::Truncated {
            needed,
            have: y.len(),
        });
    }
    let mut common_phase = Vec::with_capacity(symbols);
    let mut soft = Vec::with_capacity(symbols * 48 * rate.modulation().bits_per_subcarrier());
    for i in 1..=symbols {
        let (s, phase) = symbol_soft(y, fft_start, i, h, rate.modulation(), cfg)?;
        soft.extend(s);
        common_phase.push(phase);
    }
    // §17.3.5.3: the encoder is back in state 0 after SERVICE, the PSDU and the six tail bits.
    let end = 16 + 8 * length + 6;
    let bits = decode(&soft, rate.code_rate(), Termination::ZeroAfter(end))
        .map_err(RxOfdmError::Viterbi)?;
    Ok((bits, common_phase))
}

/// Decodes a PPDU with a given channel estimate (a genie's, in the gates).
pub fn decode_ppdu_with_channel(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
) -> Result<Decoded, RxOfdmError> {
    let (signal, phase) = signal_with_phase(y, fft_start, h, cfg)?;
    let (bits, data_phase) = decode_data_bits(y, fft_start, h, signal, cfg)?;
    let mut common_phase = vec![phase];
    common_phase.extend(data_phase);
    // §17.3.5.5: the first seven SERVICE bits are zero, so their scrambled values are the
    // scrambler's first seven outputs and identify its seed. (Hardware loads them as the state;
    // the model searches the 127 seeds.) An error in these seven bits loses the frame.
    let head = bits.get(..7).ok_or(RxOfdmError::ScramblerSeed)?;
    let scrambler_seed = (1u8..128)
        .find(|&s| Scrambler::new(s).is_ok_and(|mut sc| sc.scramble(head).iter().all(|b| *b == 0)))
        .ok_or(RxOfdmError::ScramblerSeed)?;
    let clear = Scrambler::new(scrambler_seed)
        .map_err(|_| RxOfdmError::ScramblerSeed)?
        .scramble(&bits);
    let psdu = clear
        .get(16..16 + 8 * signal.length)
        .ok_or(RxOfdmError::ScramblerSeed)?
        .chunks(8)
        .map(|c| c.iter().rev().fold(0u8, |a, &b| (a << 1) | b))
        .collect();
    Ok(Decoded {
        signal,
        psdu,
        scrambler_seed,
        common_phase,
    })
}

/// Decodes the PPDU whose first long training symbol's FFT window starts at `fft_start`.
pub fn decode_ppdu(
    y: &[Sample],
    fft_start: usize,
    cfg: &RxOfdmConfig,
) -> Result<Decoded, RxOfdmError> {
    let needed = fft_start + 2 * FFT_SIZE;
    let h = y
        .get(fft_start..needed)
        .and_then(estimate_channel)
        .ok_or(RxOfdmError::Truncated {
            needed,
            have: y.len(),
        })?;
    decode_ppdu_with_channel(y, fft_start, &h, cfg)
}
