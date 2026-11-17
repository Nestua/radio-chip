//! 802.11a/g OFDM modulation (IEEE 802.11-2020 §17.3.5.10 and the pilot rules of §17.3.5.9).
//!
//! One OFDM symbol: 48 data subcarriers and 4 pilots (±7, ±21) placed in a 64-bin spectrum with
//! DC and the band edges empty, a 64-point inverse FFT, and a 16-sample cyclic prefix: 80
//! samples at 20 MHz.
//!
//! The IFFT is radix-2 decimation in time, six stages, each butterfly output scaled by ½, so the
//! whole transform carries the 1/64 of the inverse DFT and can never overflow (a butterfly never
//! grows the larger of its input magnitudes). Each butterfly rounds once:
//! `(a·2^Ft ± w·b) / 2` is narrowed from Fd + Ft + 1 fractional bits to the data format.
//! Twiddles are a 32-entry ROM of e^(+j2πk/64), quantised half-even.

use crate::fixed::{quantize, requantize, FixedError, Overflow, QFormat, Rounding};
use crate::mapper::{mean_energy, Point};
use crate::modulation::Modulation;
use crate::scrambler::Scrambler;

/// FFT size.
pub const FFT_SIZE: usize = 64;
/// Cyclic prefix length in samples (0.8 µs at 20 MHz).
pub const CP_LEN: usize = 16;
/// Samples per OFDM symbol including the cyclic prefix.
pub const SYMBOL_LEN: usize = FFT_SIZE + CP_LEN;
/// Pilot subcarriers. IEEE 802.11-2020 §17.3.5.9.
pub const PILOT_SUBCARRIERS: [i8; 4] = [-21, -7, 7, 21];
/// Pilot values before the polarity sequence is applied, in the order of [`PILOT_SUBCARRIERS`].
pub const PILOT_BASE: [i8; 4] = [1, 1, 1, -1];
/// Widest data word the fixed-point IFFT accepts (keeps every product inside `i64`).
pub const MAX_IFFT_WORD_BITS: u32 = 24;
/// Transmit IFFT word length chosen by the G2 sweep (`g2 ofdm`): the smallest W whose worst-case
/// SQNR against the float reference clears the 45 dB budget by at least 3 dB (49.1 dB at W = 13).
pub const TX_IFFT_WORD_BITS: u32 = 13;

/// A complex fixed-point sample (raw integers; the format is given by the [`IfftConfig`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Iq {
    /// Real part.
    pub re: i64,
    /// Imaginary part.
    pub im: i64,
}

/// Errors from the OFDM modulator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OfdmError {
    /// A symbol needs exactly 48 data points.
    DataLength(usize),
    /// The IFFT configuration is not usable.
    Config(FixedError),
}

impl std::fmt::Display for OfdmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OfdmError::DataLength(n) => write!(f, "an OFDM symbol needs 48 data points, got {n}"),
            OfdmError::Config(e) => write!(f, "IFFT configuration: {e}"),
        }
    }
}

impl std::error::Error for OfdmError {}

/// Arithmetic of the fixed-point IFFT.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IfftConfig {
    /// Format of the frequency bins, every intermediate stage and the time samples.
    pub data: QFormat,
    /// Format of the twiddle ROM.
    pub twiddle: QFormat,
    /// Rounding at each butterfly output.
    pub rounding: Rounding,
}

impl IfftConfig {
    /// The transmit configuration: [`TX_IFFT_WORD_BITS`] via [`IfftConfig::with_word_bits`].
    pub fn tx_default() -> Self {
        Self::with_word_bits(TX_IFFT_WORD_BITS).unwrap_or(Self {
            // unreachable: TX_IFFT_WORD_BITS is inside the accepted range
            data: QFormat::MIN_SIGNED,
            twiddle: QFormat::MIN_SIGNED,
            rounding: Rounding::HalfUp,
        })
    }

    /// Data and twiddles both in Q(W, W−2) (range ±2: holds 7/√42 ≈ 1.08 and exactly +1),
    /// half-up rounding. `word_bits` must be in 4..=[`MAX_IFFT_WORD_BITS`].
    pub fn with_word_bits(word_bits: u32) -> Result<Self, OfdmError> {
        if !(4..=MAX_IFFT_WORD_BITS).contains(&word_bits) {
            return Err(OfdmError::Config(FixedError::InvalidFormat {
                word_bits,
                frac_bits: word_bits.saturating_sub(2),
            }));
        }
        let format = QFormat::new(word_bits, word_bits - 2).map_err(OfdmError::Config)?;
        Ok(Self {
            data: format,
            twiddle: format,
            rounding: Rounding::HalfUp,
        })
    }
}

/// The 48 data subcarriers in the order data points are assigned to them (−26 … 26, skipping
/// DC and the pilots).
pub fn data_subcarriers() -> [i8; 48] {
    let mut out = [0i8; 48];
    let carriers = (-26i8..=26).filter(|c| *c != 0 && !PILOT_SUBCARRIERS.contains(c));
    for (slot, c) in out.iter_mut().zip(carriers) {
        *slot = c;
    }
    out
}

/// IFFT bin of a subcarrier: positive frequencies at 1…26, negative at 64 + c.
pub fn fft_bin(subcarrier: i8) -> usize {
    (i64::from(subcarrier).rem_euclid(FFT_SIZE as i64)) as usize
}

/// Pilot polarity p_n ∈ {+1, −1}, n taken modulo 127. The SIGNAL symbol uses p_0 and DATA
/// symbol k uses p_(k+1) (applied by the PPDU builder).
pub fn pilot_polarity(n: usize) -> i8 {
    // IEEE 802.11-2020 §17.3.5.10: the sequence is the scrambler (§17.3.5.5) output with the
    // all-ones initial state, 0 -> +1 and 1 -> -1.
    let Ok(mut scrambler) = Scrambler::new(0x7f) else {
        return 1; // unreachable: 0x7f is a valid seed
    };
    let mut bit = 0;
    for _ in 0..=n % 127 {
        bit = scrambler.next_bit();
    }
    1 - 2 * bit as i8
}

/// The 64 frequency bins of one symbol in `config.data` format.
pub fn frequency_bins(
    data: &[Point],
    modulation: Modulation,
    polarity_index: usize,
    config: &IfftConfig,
) -> Result<[Iq; FFT_SIZE], OfdmError> {
    if data.len() != 48 {
        return Err(OfdmError::DataLength(data.len()));
    }
    // K_MOD-scaled levels are constants (a small ROM in hardware), quantised half-even.
    let k_mod = 1.0 / f64::from(mean_energy(modulation)).sqrt();
    let level = |l: i8| {
        quantize(
            f64::from(l) * k_mod,
            config.data,
            Rounding::HalfEven,
            Overflow::Saturate,
        )
    };
    let mut bins = [Iq::default(); FFT_SIZE];
    for (&c, p) in data_subcarriers().iter().zip(data) {
        bins[fft_bin(c)] = Iq {
            re: level(p.i).map_err(OfdmError::Config)?,
            im: level(p.q).map_err(OfdmError::Config)?,
        };
    }
    let polarity = pilot_polarity(polarity_index);
    let one = 1i64 << config.data.frac_bits();
    for (&c, &base) in PILOT_SUBCARRIERS.iter().zip(&PILOT_BASE) {
        bins[fft_bin(c)] = Iq {
            re: i64::from(polarity * base) * one,
            im: 0,
        };
    }
    Ok(bins)
}

/// Fixed-point 64-point inverse FFT including the 1/64 (½ per stage).
pub fn ifft64(bins: &[Iq; FFT_SIZE], config: &IfftConfig) -> [Iq; FFT_SIZE] {
    let twiddles = twiddle_rom(config.twiddle);
    let (fd, ft) = (config.data.frac_bits(), config.twiddle.frac_bits());
    let narrow = |v: i64| {
        requantize(
            v,
            fd + ft + 1,
            config.data,
            config.rounding,
            Overflow::Saturate,
        )
    };

    // Bit-reversed input order for decimation in time.
    let mut x = [Iq::default(); FFT_SIZE];
    for (k, slot) in x.iter_mut().enumerate() {
        *slot = bins[(k as u8).reverse_bits() as usize >> 2];
    }
    let mut half = 1;
    while half < FFT_SIZE {
        let step = FFT_SIZE / (2 * half);
        for start in (0..FFT_SIZE).step_by(2 * half) {
            for j in 0..half {
                let w = twiddles[j * step];
                let (a, b) = (x[start + j], x[start + j + half]);
                // w·b with Fd + Ft fractional bits, exact.
                let t_re = w.re * b.re - w.im * b.im;
                let t_im = w.re * b.im + w.im * b.re;
                let (a_re, a_im) = (a.re << ft, a.im << ft);
                x[start + j] = Iq {
                    re: narrow(a_re + t_re),
                    im: narrow(a_im + t_im),
                };
                x[start + j + half] = Iq {
                    re: narrow(a_re - t_re),
                    im: narrow(a_im - t_im),
                };
            }
        }
        half *= 2;
    }
    x
}

/// e^(+j2πk/64) for k = 0…31 in `format`, half-even: the twiddle ROM of the inverse transform.
fn twiddle_rom(format: QFormat) -> [Iq; FFT_SIZE / 2] {
    let mut rom = [Iq::default(); FFT_SIZE / 2];
    for (k, w) in rom.iter_mut().enumerate() {
        let phase = 2.0 * std::f64::consts::PI * k as f64 / FFT_SIZE as f64;
        // |v| <= 1 and finite, so quantize cannot fail for any accepted format.
        let q = |v: f64| quantize(v, format, Rounding::HalfEven, Overflow::Saturate).unwrap_or(0);
        *w = Iq {
            re: q(phase.cos()),
            im: q(phase.sin()),
        };
    }
    rom
}

/// One OFDM symbol, cyclic prefix first: 80 samples in `config.data` format.
pub fn modulate_symbol(
    data: &[Point],
    modulation: Modulation,
    polarity_index: usize,
    config: &IfftConfig,
) -> Result<Vec<Iq>, OfdmError> {
    let time = ifft64(
        &frequency_bins(data, modulation, polarity_index, config)?,
        config,
    );
    Ok(with_cyclic_prefix(&time))
}

fn with_cyclic_prefix<T: Copy>(time: &[T; FFT_SIZE]) -> Vec<T> {
    time[FFT_SIZE - CP_LEN..]
        .iter()
        .chain(time.iter())
        .copied()
        .collect()
}

/// Floating-point reference of [`modulate_symbol`]: exact normalised bins and a direct inverse
/// DFT with 1/64. Used for the G2 budget, never by hardware.
pub fn modulate_symbol_f64(
    data: &[Point],
    modulation: Modulation,
    polarity_index: usize,
) -> Result<Vec<(f64, f64)>, OfdmError> {
    if data.len() != 48 {
        return Err(OfdmError::DataLength(data.len()));
    }
    let k_mod = 1.0 / f64::from(mean_energy(modulation)).sqrt();
    let mut bins = [(0.0f64, 0.0f64); FFT_SIZE];
    for (&c, p) in data_subcarriers().iter().zip(data) {
        bins[fft_bin(c)] = (f64::from(p.i) * k_mod, f64::from(p.q) * k_mod);
    }
    let polarity = pilot_polarity(polarity_index);
    for (&c, &base) in PILOT_SUBCARRIERS.iter().zip(&PILOT_BASE) {
        bins[fft_bin(c)] = (f64::from(polarity * base), 0.0);
    }
    let mut time = [(0.0f64, 0.0f64); FFT_SIZE];
    for (n, out) in time.iter_mut().enumerate() {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, &(br, bi)) in bins.iter().enumerate() {
            let phase = 2.0 * std::f64::consts::PI * ((k * n) % FFT_SIZE) as f64 / FFT_SIZE as f64;
            let (c, s) = (phase.cos(), phase.sin());
            re += br * c - bi * s;
            im += br * s + bi * c;
        }
        *out = (re / FFT_SIZE as f64, im / FFT_SIZE as f64);
    }
    Ok(with_cyclic_prefix(&time))
}
