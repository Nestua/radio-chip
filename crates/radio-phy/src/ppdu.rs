//! 802.11a/g PPDU builder (IEEE 802.11-2020 §17.3.2–17.3.5): preamble, SIGNAL, DATA.
//!
//! Frame layout at 20 MHz: short training field (160 samples), long training field (160),
//! SIGNAL (one BPSK rate-1/2 symbol, 80), DATA (N_SYM symbols, 80 each). No transition window is
//! applied between fields (the standard leaves windowing to the implementation).
//!
//! DATA bits: SERVICE (16 zeros) + PSDU (each octet LSB first) + 6 tail zeros + pad zeros up to
//! N_SYM · N_DBPS; the whole stream is scrambled, then the 6 tail bits are reset to zero so the
//! encoder returns to the zero state; then convolutional coding with puncturing, per-symbol
//! interleaving, mapping and OFDM modulation with pilot polarity p_(k+1) for DATA symbol k.

use crate::convolutional::{encode, CodeRate};
use crate::fixed::{quantize, Overflow, Rounding};
use crate::interleaver::interleave;
use crate::mapper::map_bits;
use crate::modulation::Modulation;
use crate::ofdm::{fft_bin, ifft64, modulate_symbol, IfftConfig, Iq, OfdmError, FFT_SIZE};
use crate::scrambler::Scrambler;

/// Samples in each training field.
pub const TRAINING_FIELD_LEN: usize = 160;
/// Largest PSDU the 12-bit LENGTH field can describe.
pub const MAX_PSDU_LEN: usize = 4095;

/// The eight 802.11a/g data rates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rate {
    /// 6 Mb/s: BPSK, rate 1/2.
    R6,
    /// 9 Mb/s: BPSK, rate 3/4.
    R9,
    /// 12 Mb/s: QPSK, rate 1/2.
    R12,
    /// 18 Mb/s: QPSK, rate 3/4.
    R18,
    /// 24 Mb/s: 16-QAM, rate 1/2.
    R24,
    /// 36 Mb/s: 16-QAM, rate 3/4.
    R36,
    /// 48 Mb/s: 64-QAM, rate 2/3.
    R48,
    /// 54 Mb/s: 64-QAM, rate 3/4.
    R54,
}

impl Rate {
    /// All rates, slowest first.
    pub const ALL: [Rate; 8] = [
        Rate::R6,
        Rate::R9,
        Rate::R12,
        Rate::R18,
        Rate::R24,
        Rate::R36,
        Rate::R48,
        Rate::R54,
    ];

    /// Nominal rate in Mb/s.
    pub fn mbps(self) -> u32 {
        match self {
            Rate::R6 => 6,
            Rate::R9 => 9,
            Rate::R12 => 12,
            Rate::R18 => 18,
            Rate::R24 => 24,
            Rate::R36 => 36,
            Rate::R48 => 48,
            Rate::R54 => 54,
        }
    }

    /// Subcarrier modulation.
    pub fn modulation(self) -> Modulation {
        match self {
            Rate::R6 | Rate::R9 => Modulation::Bpsk,
            Rate::R12 | Rate::R18 => Modulation::Qpsk,
            Rate::R24 | Rate::R36 => Modulation::Qam16,
            Rate::R48 | Rate::R54 => Modulation::Qam64,
        }
    }

    /// Coding rate after puncturing.
    pub fn code_rate(self) -> CodeRate {
        match self {
            Rate::R6 | Rate::R12 | Rate::R24 => CodeRate::R1_2,
            Rate::R48 => CodeRate::R2_3,
            Rate::R9 | Rate::R18 | Rate::R36 | Rate::R54 => CodeRate::R3_4,
        }
    }

    /// Data bits per OFDM symbol, N_DBPS.
    pub fn data_bits_per_symbol(self) -> usize {
        // 4 µs symbols: N_DBPS = rate in Mb/s × 4.
        self.mbps() as usize * 4
    }

    /// The four RATE bits of the SIGNAL field in transmission order.
    pub fn signal_rate_bits(self) -> [u8; 4] {
        // R1…R4, R1 first. IEEE 802.11-2020 §17.3.4.2; confirmed by OpenOFDM's receiver table
        // and by its full decode of our frames (an earlier draft took OpenOFDM's hardware-LUT
        // table, which lists the same bits mirrored, and the decode caught it).
        match self {
            Rate::R6 => [1, 1, 0, 1],
            Rate::R9 => [1, 1, 1, 1],
            Rate::R12 => [0, 1, 0, 1],
            Rate::R18 => [0, 1, 1, 1],
            Rate::R24 => [1, 0, 0, 1],
            Rate::R36 => [1, 0, 1, 1],
            Rate::R48 => [0, 0, 0, 1],
            Rate::R54 => [0, 0, 1, 1],
        }
    }
}

/// Errors from the PPDU builder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PpduError {
    /// PSDU length outside 1..=[`MAX_PSDU_LEN`].
    PsduLength(usize),
    /// The scrambler seed must be a non-zero 7-bit value.
    ScramblerSeed(u8),
    /// A lower block rejected its input (cannot happen for valid frames).
    Ofdm(OfdmError),
    /// A coding, interleaving or mapping step saw a length it does not accept (cannot happen
    /// for valid frames; reported instead of panicking).
    Internal(&'static str),
}

impl std::fmt::Display for PpduError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PpduError::PsduLength(n) => write!(f, "PSDU length {n} outside 1..={MAX_PSDU_LEN}"),
            PpduError::ScramblerSeed(s) => write!(f, "scrambler seed {s:#04x} is not 1..=0x7f"),
            PpduError::Ofdm(e) => write!(f, "OFDM: {e}"),
            PpduError::Internal(what) => write!(f, "internal length error in {what}"),
        }
    }
}

impl std::error::Error for PpduError {}

/// Number of DATA symbols for a PSDU of `length` octets: ⌈(16 + 8·length + 6) / N_DBPS⌉.
pub fn data_symbols(rate: Rate, length: usize) -> usize {
    (SERVICE_BITS + 8 * length + TAIL_BITS).div_ceil(rate.data_bits_per_symbol())
}

const SERVICE_BITS: usize = 16;
const TAIL_BITS: usize = 6;

fn check_length(length: usize) -> Result<(), PpduError> {
    if (1..=MAX_PSDU_LEN).contains(&length) {
        Ok(())
    } else {
        Err(PpduError::PsduLength(length))
    }
}

/// The 24 SIGNAL bits in transmission order: RATE, reserved 0, LENGTH (LSB first), even parity
/// over the first 17 bits, 6 tail zeros.
pub fn signal_bits(rate: Rate, length: usize) -> Result<[u8; 24], PpduError> {
    check_length(length)?;
    let mut bits = [0u8; 24];
    bits[..4].copy_from_slice(&rate.signal_rate_bits());
    for i in 0..12 {
        bits[5 + i] = ((length >> i) & 1) as u8;
    }
    bits[17] = bits[..17].iter().fold(0, |p, &b| p ^ b);
    Ok(bits)
}

/// The scrambled DATA bit stream (SERVICE, PSDU, zeroed tail, pad), N_SYM · N_DBPS bits.
pub fn data_bits(psdu: &[u8], rate: Rate, scrambler_seed: u8) -> Result<Vec<u8>, PpduError> {
    check_length(psdu.len())?;
    let mut scrambler =
        Scrambler::new(scrambler_seed).map_err(|_| PpduError::ScramblerSeed(scrambler_seed))?;
    let total = data_symbols(rate, psdu.len()) * rate.data_bits_per_symbol();
    let mut clear = vec![0u8; SERVICE_BITS];
    for byte in psdu {
        clear.extend((0..8).map(|i| (byte >> i) & 1));
    }
    clear.resize(total, 0);
    let mut bits = scrambler.scramble(&clear);
    let tail = SERVICE_BITS + 8 * psdu.len();
    bits[tail..tail + TAIL_BITS].fill(0);
    Ok(bits)
}

/// The long training sequence L_k for k = −26…26 (index 0 is k = −26; L_0 = 0).
pub fn long_training_sequence() -> [i8; 53] {
    // IEEE 802.11-2020 §17.3.3, L_(-26..26).
    [
        1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 1,
        -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1,
    ]
}

/// Non-zero short-training subcarriers and their sign s (S_k = √(13/6)·s·(1 + j)).
/// IEEE 802.11-2020 §17.3.3.
pub const SHORT_TRAINING: [(i8, i8); 12] = [
    (-24, 1),
    (-20, -1),
    (-16, 1),
    (-12, -1),
    (-8, -1),
    (-4, 1),
    (4, -1),
    (8, -1),
    (12, 1),
    (16, 1),
    (20, 1),
    (24, 1),
];

fn training_symbol(bins: &[Iq; FFT_SIZE], config: &IfftConfig) -> [Iq; FFT_SIZE] {
    ifft64(bins, config)
}

/// Short training field: ten repetitions of a 16-sample period, 160 samples.
pub fn short_training_field(config: &IfftConfig) -> Result<Vec<Iq>, PpduError> {
    let level = quantize(
        (13.0f64 / 6.0).sqrt(),
        config.data,
        Rounding::HalfEven,
        Overflow::Saturate,
    )
    .map_err(|e| PpduError::Ofdm(OfdmError::Config(e)))?;
    let mut bins = [Iq::default(); FFT_SIZE];
    for &(k, sign) in &SHORT_TRAINING {
        let v = i64::from(sign) * level;
        bins[fft_bin(k)] = Iq { re: v, im: v };
    }
    let symbol = training_symbol(&bins, config);
    // Only every 4th bin is used, so the symbol repeats every 16 samples: 2.5 symbols = 10 periods.
    Ok((0..TRAINING_FIELD_LEN)
        .map(|n| symbol[n % FFT_SIZE])
        .collect())
}

/// Long training field: 32-sample guard (GI2) and two 64-sample long symbols, 160 samples.
pub fn long_training_field(config: &IfftConfig) -> Result<Vec<Iq>, PpduError> {
    let one = 1i64 << config.data.frac_bits();
    let mut bins = [Iq::default(); FFT_SIZE];
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        bins[fft_bin(k)] = Iq {
            re: i64::from(l) * one,
            im: 0,
        };
    }
    let symbol = training_symbol(&bins, config);
    Ok(symbol[FFT_SIZE / 2..]
        .iter()
        .chain(symbol.iter())
        .chain(symbol.iter())
        .copied()
        .collect())
}

/// Codes, interleaves, maps and modulates one field's bits (whole symbols) into OFDM symbols,
/// the first with polarity index `first_polarity`.
fn modulate_bits(
    bits: &[u8],
    rate: Rate,
    first_polarity: usize,
    config: &IfftConfig,
) -> Result<Vec<Iq>, PpduError> {
    let m = rate.modulation();
    let coded = encode(bits, rate.code_rate()).map_err(|_| PpduError::Internal("puncturing"))?;
    let interleaved = interleave(&coded, m).map_err(|_| PpduError::Internal("interleaver"))?;
    let points = map_bits(&interleaved, m).map_err(|_| PpduError::Internal("mapper"))?;
    let mut out = Vec::with_capacity(points.len() / 48 * crate::ofdm::SYMBOL_LEN);
    for (k, symbol) in points.chunks(48).enumerate() {
        out.extend(
            modulate_symbol(symbol, m, first_polarity + k, config).map_err(PpduError::Ofdm)?,
        );
    }
    Ok(out)
}

/// The complete PPDU: STF, LTF, SIGNAL, DATA.
pub fn build_ppdu(
    psdu: &[u8],
    rate: Rate,
    scrambler_seed: u8,
    config: &IfftConfig,
) -> Result<Vec<Iq>, PpduError> {
    let signal = signal_bits(rate, psdu.len())?;
    let data = data_bits(psdu, rate, scrambler_seed)?;
    let mut frame = short_training_field(config)?;
    frame.extend(long_training_field(config)?);
    // SIGNAL: BPSK rate 1/2, not scrambled, polarity p_0. DATA symbol k: polarity p_(k+1).
    frame.extend(modulate_bits(&signal, Rate::R6, 0, config)?);
    frame.extend(modulate_bits(&data, rate, 1, config)?);
    Ok(frame)
}

/// FNV-1a 64-bit hash of a frame's raw samples (re then im, little-endian `i64`): lets a test
/// pin a recorded frame without committing every sample.
pub fn frame_hash(frame: &[Iq]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for s in frame {
        for byte in s.re.to_le_bytes().iter().chain(s.im.to_le_bytes().iter()) {
            h ^= u64::from(*byte);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    h
}
