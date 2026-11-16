//! 802.11 OFDM data scrambler, generator S(x) = x^7 + x^4 + 1.
//!
//! Reference: IEEE 802.11-2020 §17.3.5.5.

use std::fmt;

/// A seed that cannot drive the LFSR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidSeed(pub u8);

impl fmt::Display for InvalidSeed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "scrambler seed {:#04x} must be in 1..=0x7f", self.0)
    }
}

impl std::error::Error for InvalidSeed {}

/// 7-bit LFSR scrambler.
///
/// State bit `i` holds the standard's register `x(i+1)`; `x1` is the most recent output.
/// Each step outputs `x7 XOR x4` and shifts it in as the new `x1` (§17.3.5.5, Figure 17-7).
#[derive(Debug, Clone)]
pub struct Scrambler {
    state: u8,
}

impl Scrambler {
    /// Creates a scrambler with a 7-bit, non-zero initial state.
    pub fn new(seed: u8) -> Result<Self, InvalidSeed> {
        if seed == 0 || seed > 0x7f {
            return Err(InvalidSeed(seed));
        }
        Ok(Self { state: seed })
    }

    /// Returns the next bit of the scrambling sequence and advances the state.
    pub fn next_bit(&mut self) -> u8 {
        let out = ((self.state >> 6) ^ (self.state >> 3)) & 1;
        self.state = ((self.state << 1) | out) & 0x7f;
        out
    }

    /// XORs `bits` with the scrambling sequence.
    pub fn scramble(&mut self, bits: &[u8]) -> Vec<u8> {
        bits.iter().map(|b| (b & 1) ^ self.next_bit()).collect()
    }
}
