//! OFDM receiver after acquisition (R0-09), float reference: channel estimate from the long
//! training field, pilot phase tracking, one-tap equaliser, soft demapper, and the decode of
//! SIGNAL and DATA down to the PSDU (IEEE 802.11-2020 §17.3). The mirror of [`crate::ppdu`].
//!
//! Input: the stream after the front's CFO correction ([`crate::rx_front::correct`]) and the
//! first sample of T1's FFT window ([`crate::rx_front::Acquisition::fft_start`], which already
//! includes the timing back-off). Definitions that the fixed-point model and the RTL will have
//! to match — the soft-value scaling above all — are in `docs/gates/rx_ofdm.md`.

use std::f64::consts::PI;

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
    /// Track the timing drift of a sample-clock offset on the pilots' phase slope (needs
    /// `pilot_tracking`).
    pub timing_tracking: bool,
}

impl Default for RxOfdmConfig {
    /// The Viterbi decoder's contract: 4-bit soft values, clip 1.5 (`docs/gates/viterbi.md`).
    fn default() -> Self {
        Self {
            soft_max: SOFT_MAX,
            clip: SOFT_CLIP,
            pilot_tracking: true,
            timing_tracking: true,
        }
    }
}

/// Standard deviation assumed for one symbol's slope measurement on a flat channel, radians
/// per subcarrier: what it is at the sensitivity limit (0.0155 measured at 3.5 dB). At a
/// higher SNR the measurement is better than assumed and the tracker merely learns more slowly
/// than it could. It sets how fast the measurements outweigh the prior on the drift.
pub const TIMING_MEASUREMENT_STD: f64 = 0.016;
/// A measurement whose variance is more than this many times the nominal one is not used: a
/// pilot that weak has a phase that wraps, and the channel estimate's own noise makes its gain
/// look larger than it is, so the variance computed for it is too small as well.
pub const TIMING_MAX_VARIANCE: f64 = 3.0;
/// Prior on the clock offset when nothing is known: the standard's worst case between two
/// stations, ppm (IEEE 802.11-2020 §17.3.9.5–6: ±20 ppm each).
pub const TIMING_PRIOR_PPM: f64 = 40.0;
/// Prior on the error of a clock offset taken from the front's CFO, ppm.
pub const TIMING_HINT_PPM: f64 = 2.0;

/// The timing tracker's state across the symbols of a PPDU.
///
/// The pilots of each symbol measure the slope of the phase across the subcarriers. A window
/// late by δ samples against the channel estimate's turns subcarrier k by 2π·k·δ/64, so a
/// sample-clock offset shows as a slope that grows linearly with time: slope(t) = rate·t, t in
/// symbols from the middle of the long training field. But the measured slope also has a part
/// that does not grow: the channel estimate's own noise on the four pilots looks like a
/// constant slope (about 0.1 samples at the sensitivity limit). Following it would turn every
/// data subcarrier by an error the pilots alone have. So the tracker fits both, measured(t) =
/// offset + rate·t, by recursive least squares, and removes only rate·t from the data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tracking {
    /// Estimated drift of the slope, radians per subcarrier per symbol.
    pub rate: f64,
    /// Estimated constant part of the measured slope (not removed from the data).
    pub offset: f64,
    /// Covariance of (offset, rate) in units of the measurement variance: [offset, cross, rate].
    pub covariance: [f64; 3],
    /// Whole samples the FFT window has been moved (negative: earlier) to keep the slope left
    /// in the window within half a sample.
    pub shift: i64,
    /// Slope removed from the last symbol's data, radians per subcarrier (rate·t, less what
    /// the window's move took).
    pub slope: f64,
    /// The last symbol's measured slope against what was predicted, radians per subcarrier.
    pub measured: f64,
    /// rate·t of the last symbol, radians per subcarrier.
    pub drift: f64,
}

/// Slope per subcarrier of one sample of timing offset.
const ONE_SAMPLE: f64 = 2.0 * PI / FFT_SIZE as f64;

/// Drift of the slope per symbol for a clock offset in ppm.
fn rate_of(ppm: f64) -> f64 {
    ONE_SAMPLE * SYMBOL_LEN as f64 * ppm * 1e-6
}

impl Default for Tracking {
    /// Nothing known about the clock offset: [`TIMING_PRIOR_PPM`].
    fn default() -> Self {
        Self::with_prior(0.0, TIMING_PRIOR_PPM)
    }
}

impl Tracking {
    fn with_prior(ppm: f64, std_ppm: f64) -> Self {
        Self {
            rate: rate_of(ppm),
            offset: 0.0,
            // The offset's prior is flat: nothing is assumed about it.
            covariance: [
                1e6,
                0.0,
                (rate_of(std_ppm) / TIMING_MEASUREMENT_STD).powi(2),
            ],
            shift: 0,
            slope: 0.0,
            measured: 0.0,
            drift: 0.0,
        }
    }

    /// The tracker's start when the transmitter's clock is `ppm` fast against the receiver's
    /// ([`crate::channel::resample`]'s parameter), known to [`TIMING_HINT_PPM`]. A transmitter
    /// derives its carrier and its symbol clock from one oscillator (IEEE 802.11-2020
    /// §17.3.9.5–6), and so does this receiver, so a fast transmitter is also high in
    /// frequency and the front's CFO gives the offset: ppm = +CFO / carrier · 10⁶.
    pub fn from_clock_offset(ppm: f64) -> Self {
        Self::with_prior(ppm, TIMING_HINT_PPM)
    }

    /// A tracker that applies the drift of a clock offset of `ppm` and learns nothing.
    pub fn fixed(ppm: f64) -> Self {
        Self {
            covariance: [0.0; 3],
            ..Self::with_prior(ppm, 0.0)
        }
    }

    /// How late the nominal window of the last symbol was, in samples (the drift since the
    /// long training field).
    pub fn timing(&self) -> f64 {
        self.drift / ONE_SAMPLE
    }

    /// One symbol's measurement `error` (slope left after `predicted` was removed) at time `t`
    /// symbols, with variance `variance` in units of the nominal measurement's: the recursive
    /// least-squares step for measured = offset + rate·t.
    fn update(&mut self, error: f64, predicted: f64, t: f64, variance: f64) {
        self.measured = error;
        if !(variance > 0.0 && variance <= TIMING_MAX_VARIANCE) {
            // No usable pilot pair: the measurement says nothing.
            return;
        }
        let [pbb, pbr, prr] = self.covariance;
        // Gain K = P·φ / (φᵀ·P·φ + R), φ = (1, t).
        let (gb, gr) = (pbb + pbr * t, pbr + prr * t);
        let s = gb + gr * t + variance;
        let (kb, kr) = (gb / s, gr / s);
        let innovation = error + predicted - (self.offset + self.rate * t);
        self.offset += kb * innovation;
        self.rate += kr * innovation;
        self.covariance = [pbb - kb * gb, pbr - kb * gr, prr - kr * gr];
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
    /// SIGNAL's LENGTH is 0 (or a [`Signal`] handed in has a length outside 1…4095).
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
    /// Timing drift tracked at each symbol (SIGNAL first), samples: [`Tracking::timing`].
    pub timing: Vec<f64>,
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

/// Equalises one symbol: `window` is its 64-sample FFT window (moved by `tracking.shift`
/// samples from its nominal place), `polarity_index` the pilot polarity index (0 for SIGNAL,
/// k + 1 for DATA symbol k), which is also the symbol's place in time. `None` for a short
/// window.
///
/// With Z the window's FFT and P(k) = Z(k)·H*(k)·s(k) on the four pilots (s the transmitted
/// ±1, so P is a maximum-ratio weighted phasor):
/// 1. Timing: the slope predicted for this symbol (rate·t, less the window's moves) is
///    removed from the pilots; what is left between the pilots at ±21 and at ±7 is the
///    measurement (least squares over the two pairs, weighted by the channel gains of their
///    pilots), which updates the tracker ([`Tracking`]) with a variance from the same gains. The data are then turned back by the updated rate·t.
///    Beyond half a sample the window moves by one sample for the next symbol.
/// 2. Common phase φ = ∠Σ P(k)·e^(−j·k·slope).
/// 3. Each data point is Z(k)·H*(k)·e^(−j(φ + k·slope)) / (|H(k)|²·K_MOD), on the integer grid.
pub fn equalise(
    window: &[Sample],
    h: &ChannelEstimate,
    polarity_index: usize,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
    tracking: &mut Tracking,
) -> Option<Equalised> {
    let z = fft64(window)?;
    let turn = |v: Sample, angle: f64| cmul(v, (angle.cos(), angle.sin()));
    let used = (-26i8..=26).filter(|&k| k != 0);
    let mean_gain = used.map(|k| abs2(h[fft_bin(k)])).sum::<f64>() / 52.0;
    let mut common_phase = 0.0;
    if cfg.pilot_tracking {
        let polarity = f64::from(pilot_polarity(polarity_index));
        let mut p = [(0.0, 0.0); 4];
        for ((slot, &c), &base) in p.iter_mut().zip(&PILOT_SUBCARRIERS).zip(&PILOT_BASE) {
            let b = fft_bin(c);
            let t = cmul(z[b], conj(h[b]));
            let s = polarity * f64::from(base);
            *slot = (s * t.0, s * t.1);
        }
        if cfg.timing_tracking {
            // Middle of T1 and T2 to the middle of this symbol's window: 112 samples for
            // SIGNAL, then 80 per symbol.
            let t = (FFT_SIZE + CP_LEN + FFT_SIZE / 2) as f64 / SYMBOL_LEN as f64
                + polarity_index as f64;
            let moved = tracking.shift as f64 * ONE_SAMPLE;
            let predicted = tracking.rate * t;
            let q: Vec<Sample> = p
                .iter()
                .zip(&PILOT_SUBCARRIERS)
                .map(|(v, &c)| turn(*v, -f64::from(c) * (predicted + moved)))
                .collect();
            // Pilots at −21, −7, 7, 21: the outer pair spans 42 subcarriers, the inner 14.
            let outer = cmul(q[3], conj(q[0]));
            let inner = cmul(q[2], conj(q[1]));
            // A pilot's phase noise goes as 1/|H|², a pair's as the sum of its two: the pair's
            // weight is the harmonic combination of the channel gains (from the estimate, which
            // is the same for every symbol; the pilots' own magnitudes are noisy). One weak
            // pilot makes its pair worthless, as it should.
            let gain = |c: i8| abs2(h[fft_bin(c)]);
            let pair = |a: f64, b: f64| if a + b > 0.0 { a * b / (a + b) } else { 0.0 };
            let (wo, wi) = (pair(gain(21), gain(-21)), pair(gain(7), gain(-7)));
            let den = 42.0 * 42.0 * wo + 14.0 * 14.0 * wi;
            let error = if den > 0.0 {
                (42.0 * wo * outer.1.atan2(outer.0) + 14.0 * wi * inner.1.atan2(inner.0)) / den
            } else {
                0.0
            };
            // The measurement's variance against a flat channel of the same mean gain (where
            // each pair's weight is half that gain): what the tracker trusts it by.
            let flat = (42.0 * 42.0 + 14.0 * 14.0) * mean_gain / 2.0;
            tracking.update(error, predicted, t, flat / den);
            tracking.drift = tracking.rate * t;
            tracking.slope = tracking.drift + moved;
        }
        let removed = if cfg.timing_tracking {
            tracking.slope
        } else {
            0.0
        };
        let sum = p
            .iter()
            .zip(&PILOT_SUBCARRIERS)
            .map(|(v, &c)| turn(*v, -f64::from(c) * removed))
            .fold((0.0, 0.0), |a, v| (a.0 + v.0, a.1 + v.1));
        common_phase = sum.1.atan2(sum.0);
    }
    // Without the tracker nothing is removed and the window stays, whatever the state says.
    let slope = if cfg.pilot_tracking && cfg.timing_tracking {
        tracking.slope
    } else {
        0.0
    };
    let grid = f64::from(mean_energy(modulation)).sqrt();
    let (mut points, mut weights) = (Vec::with_capacity(48), Vec::with_capacity(48));
    for c in data_subcarriers() {
        let b = fft_bin(c);
        let gain = abs2(h[b]);
        let v = turn(
            cmul(z[b], conj(h[b])),
            -(common_phase + f64::from(c) * slope),
        );
        // A subcarrier with no gain carries no information: weight 0, any point.
        let scale = if gain > 0.0 { grid / gain } else { 0.0 };
        points.push((v.0 * scale, v.1 * scale));
        weights.push(if mean_gain > 0.0 {
            gain / mean_gain
        } else {
            0.0
        });
    }
    // A window one sample earlier takes 2π/64 off the slope left in it: keep that within half a
    // sample.
    if slope > ONE_SAMPLE / 2.0 {
        tracking.shift = tracking.shift.saturating_sub(1);
    } else if slope < -ONE_SAMPLE / 2.0 {
        tracking.shift = tracking.shift.saturating_add(1);
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
    // A negative `soft_max` would make an empty range to clamp to.
    let limit = f64::from(cfg.soft_max.max(0));
    let scale = limit / cfg.clip;
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
    // The interleaver sends coded bit k to position interleaved_index(k). (A symbol with
    // fewer than 48 points has no such position: an erasure.)
    (0..on_air.len())
        .map(|k| {
            on_air
                .get(interleaved_index(k, modulation))
                .copied()
                .unwrap_or(0)
        })
        .collect()
}

/// First sample of the FFT window of symbol `i` after the long training field (0 = SIGNAL),
/// keeping the back-off `fft_start` has inside the cyclic prefix.
fn symbol_start(fft_start: usize, i: usize) -> usize {
    // Saturating: a start beyond any stream is reported as truncation, not as an overflow.
    fft_start
        .saturating_add(2 * FFT_SIZE + CP_LEN)
        .saturating_add(SYMBOL_LEN.saturating_mul(i))
}

fn symbol_soft(
    y: &[Sample],
    fft_start: usize,
    i: usize,
    h: &ChannelEstimate,
    modulation: Modulation,
    cfg: &RxOfdmConfig,
    tracking: &mut Tracking,
) -> Result<(Vec<i32>, f64), RxOfdmError> {
    let nominal = symbol_start(fft_start, i);
    // The window as the timing loop has moved it.
    let start = i64::try_from(nominal)
        .ok()
        .and_then(|n| n.checked_add(tracking.shift))
        .and_then(|n| usize::try_from(n).ok());
    let end = start.and_then(|s| s.checked_add(FFT_SIZE));
    let truncated = RxOfdmError::Truncated {
        needed: end.unwrap_or(usize::MAX),
        have: y.len(),
    };
    let window = start
        .zip(end)
        .and_then(|(s, e)| y.get(s..e))
        .ok_or(truncated)?;
    let eq = equalise(window, h, i, modulation, cfg, tracking).ok_or(truncated)?;
    Ok((soft_bits(&eq, modulation, cfg), eq.common_phase))
}

fn signal_with_phase(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
    tracking: &mut Tracking,
) -> Result<(Signal, f64), RxOfdmError> {
    // IEEE 802.11-2020 §17.3.4: one BPSK rate-1/2 symbol; RATE, reserved, LENGTH (LSB first),
    // even parity over the first 17 bits, six tail zeros.
    let (soft, phase) = symbol_soft(y, fft_start, 0, h, Modulation::Bpsk, cfg, tracking)?;
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
    signal_with_phase(y, fft_start, h, cfg, &mut Tracking::default()).map(|s| s.0)
}

/// What the tracking loops did at each symbol.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SymbolLog {
    /// Common phase removed, radians.
    pub common_phase: Vec<f64>,
    /// Timing drift tracked, samples ([`Tracking::timing`]).
    pub timing: Vec<f64>,
}

/// The decoded DATA field before descrambling: N_SYM·N_DBPS bits (SERVICE, PSDU, tail, pad as
/// the transmitter's [`crate::ppdu::data_bits`]). `tracking` is the timing loop's state after
/// SIGNAL (or its default); `log` receives one entry per DATA symbol.
pub fn decode_data_bits(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    signal: Signal,
    cfg: &RxOfdmConfig,
    tracking: &mut Tracking,
    log: &mut SymbolLog,
) -> Result<Vec<u8>, RxOfdmError> {
    let Signal { rate, length } = signal;
    if !(1..=MAX_PSDU_LEN).contains(&length) {
        return Err(RxOfdmError::SignalLength);
    }
    let symbols = data_symbols(rate, length);
    let needed = symbol_start(fft_start, symbols).saturating_add(FFT_SIZE);
    if y.len() < needed {
        return Err(RxOfdmError::Truncated {
            needed,
            have: y.len(),
        });
    }
    let mut soft = Vec::with_capacity(symbols * 48 * rate.modulation().bits_per_subcarrier());
    for i in 1..=symbols {
        let (s, phase) = symbol_soft(y, fft_start, i, h, rate.modulation(), cfg, tracking)?;
        soft.extend(s);
        log.common_phase.push(phase);
        log.timing.push(tracking.timing());
    }
    // §17.3.5.3: the encoder is back in state 0 after SERVICE, the PSDU and the six tail bits.
    let end = 16 + 8 * length + 6;
    decode(&soft, rate.code_rate(), Termination::ZeroAfter(end)).map_err(RxOfdmError::Viterbi)
}

/// Decodes a PPDU with a given channel estimate (a genie's, in the gates).
pub fn decode_ppdu_with_channel(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
) -> Result<Decoded, RxOfdmError> {
    decode_tracked(y, fft_start, h, cfg, Tracking::default())
}

fn decode_tracked(
    y: &[Sample],
    fft_start: usize,
    h: &ChannelEstimate,
    cfg: &RxOfdmConfig,
    mut tracking: Tracking,
) -> Result<Decoded, RxOfdmError> {
    let (signal, phase) = signal_with_phase(y, fft_start, h, cfg, &mut tracking)?;
    let mut log = SymbolLog {
        common_phase: vec![phase],
        timing: vec![tracking.timing()],
    };
    let bits = decode_data_bits(y, fft_start, h, signal, cfg, &mut tracking, &mut log)?;
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
        common_phase: log.common_phase,
        timing: log.timing,
    })
}

/// Decodes the PPDU whose first long training symbol's FFT window starts at `fft_start`.
pub fn decode_ppdu(
    y: &[Sample],
    fft_start: usize,
    cfg: &RxOfdmConfig,
) -> Result<Decoded, RxOfdmError> {
    decode_ppdu_from(y, fft_start, cfg, Tracking::default())
}

/// [`decode_ppdu`] with the timing loop started from `tracking`
/// ([`Tracking::from_clock_offset`]).
pub fn decode_ppdu_from(
    y: &[Sample],
    fft_start: usize,
    cfg: &RxOfdmConfig,
    tracking: Tracking,
) -> Result<Decoded, RxOfdmError> {
    let needed = fft_start.saturating_add(2 * FFT_SIZE);
    let h = y
        .get(fft_start..needed)
        .and_then(estimate_channel)
        .ok_or(RxOfdmError::Truncated {
            needed,
            have: y.len(),
        })?;
    decode_tracked(y, fft_start, &h, cfg, tracking)
}
