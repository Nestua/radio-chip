//! 802.11a/g OFDM block interleaver (IEEE 802.11-2020 §17.3.5.7).
//!
//! One OFDM symbol of N_CBPS coded bits is permuted twice. The first permutation puts adjacent
//! coded bits on non-adjacent subcarriers (write in rows of 16 columns, read by columns). The
//! second alternates adjacent coded bits between the more and less significant bits of the
//! constellation, so long runs of low-reliability bits are avoided.

pub use crate::modulation::{Modulation, DATA_SUBCARRIERS};

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
