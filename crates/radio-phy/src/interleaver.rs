//! 802.11a/g OFDM block interleaver (IEEE 802.11-2020 §17.3.5.7).
//!
//! One OFDM symbol of N_CBPS coded bits is permuted twice. The first permutation puts adjacent
//! coded bits on non-adjacent subcarriers (write in rows of 16 columns, read by columns). The
//! second alternates adjacent coded bits between the more and less significant bits of the
//! constellation, so long runs of low-reliability bits are avoided.

/// Modulation order expressed as coded bits per subcarrier (N_BPSC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modulation {
    /// BPSK, N_BPSC = 1.
    Bpsk,
    /// QPSK, N_BPSC = 2.
    Qpsk,
    /// 16-QAM, N_BPSC = 4.
    Qam16,
    /// 64-QAM, N_BPSC = 6.
    Qam64,
}

impl Modulation {
    /// Coded bits per subcarrier, N_BPSC.
    pub fn bits_per_subcarrier(self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Coded bits per OFDM symbol, N_CBPS = 48 data subcarriers × N_BPSC.
    pub fn bits_per_symbol(self) -> usize {
        DATA_SUBCARRIERS * self.bits_per_subcarrier()
    }

    /// The modulation with `n_bpsc` coded bits per subcarrier, if 802.11a/g defines one.
    pub fn from_bits_per_subcarrier(n_bpsc: usize) -> Option<Self> {
        match n_bpsc {
            1 => Some(Modulation::Bpsk),
            2 => Some(Modulation::Qpsk),
            4 => Some(Modulation::Qam16),
            6 => Some(Modulation::Qam64),
            _ => None,
        }
    }
}

/// Data subcarriers per 20 MHz OFDM symbol. IEEE 802.11-2020 §17.3.5.10.
pub const DATA_SUBCARRIERS: usize = 48;

/// The input is not a whole number of OFDM symbols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartialSymbol {
    /// Bits offered.
    pub len: usize,
    /// N_CBPS the length must be a multiple of.
    pub n_cbps: usize,
}

impl std::fmt::Display for PartialSymbol {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} bits is not a whole number of {}-bit OFDM symbols",
            self.len, self.n_cbps
        )
    }
}

impl std::error::Error for PartialSymbol {}

/// Position `j` after both permutations of coded bit `k` within one symbol.
///
/// `k` must be below N_CBPS of `modulation`.
pub fn interleaved_index(k: usize, modulation: Modulation) -> usize {
    let n_cbps = modulation.bits_per_symbol();
    let s = (modulation.bits_per_subcarrier() / 2).max(1);
    // IEEE 802.11-2020 §17.3.5.7, first permutation: rows of 16 written, columns read.
    let i = (n_cbps / 16) * (k % 16) + k / 16;
    // Second permutation: rotate within groups of s so significance alternates.
    s * (i / s) + (i + n_cbps - (16 * i) / n_cbps) % s
}

fn check_whole_symbols(len: usize, n_cbps: usize) -> Result<(), PartialSymbol> {
    if len.is_multiple_of(n_cbps) {
        Ok(())
    } else {
        Err(PartialSymbol { len, n_cbps })
    }
}

/// Interleaves whole OFDM symbols: output position `interleaved_index(k)` carries input bit `k`.
pub fn interleave(bits: &[u8], modulation: Modulation) -> Result<Vec<u8>, PartialSymbol> {
    let n_cbps = modulation.bits_per_symbol();
    check_whole_symbols(bits.len(), n_cbps)?;
    let mut out = vec![0u8; bits.len()];
    for (symbol_in, symbol_out) in bits.chunks(n_cbps).zip(out.chunks_mut(n_cbps)) {
        for (k, &bit) in symbol_in.iter().enumerate() {
            symbol_out[interleaved_index(k, modulation)] = bit;
        }
    }
    Ok(out)
}

/// Inverse of [`interleave`].
pub fn deinterleave(bits: &[u8], modulation: Modulation) -> Result<Vec<u8>, PartialSymbol> {
    let n_cbps = modulation.bits_per_symbol();
    check_whole_symbols(bits.len(), n_cbps)?;
    let mut out = vec![0u8; bits.len()];
    for (symbol_in, symbol_out) in bits.chunks(n_cbps).zip(out.chunks_mut(n_cbps)) {
        for (k, bit) in symbol_out.iter_mut().enumerate() {
            *bit = symbol_in[interleaved_index(k, modulation)];
        }
    }
    Ok(out)
}
