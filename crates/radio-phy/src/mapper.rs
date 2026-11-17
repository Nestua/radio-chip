//! 802.11a/g subcarrier modulation mapping (IEEE 802.11-2020 §17.3.5.8).
//!
//! Groups of N_BPSC coded bits become one constellation point. The model returns the point on
//! the unnormalised odd-integer grid (±1, ±3, ±5, ±7), which is exact and is what hardware holds
//! before scaling. The normalisation K_MOD = 1/√E, with E = [`mean_energy`], is applied in fixed
//! point together with the IFFT scaling (R0-05, R0-11).
//!
//! Bit labels are Gray coded per axis: b0 (and b1, b2 for 16/64-QAM) select I, the remaining
//! half select Q; BPSK uses I only.

use crate::modulation::Modulation;

/// A constellation point on the unnormalised odd-integer grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    /// In-phase level.
    pub i: i8,
    /// Quadrature level (0 for BPSK).
    pub q: i8,
}

/// The input is not a whole number of N_BPSC-bit groups (or, for [`map_group`], not exactly one).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartialGroup {
    /// Bits offered.
    pub len: usize,
    /// N_BPSC the length must be a multiple of.
    pub n_bpsc: usize,
}

impl std::fmt::Display for PartialGroup {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} bits is not a whole number of {}-bit groups",
            self.len, self.n_bpsc
        )
    }
}

impl std::error::Error for PartialGroup {}

/// Mean energy E of the unnormalised constellation; the standard's K_MOD is 1/√E.
pub fn mean_energy(modulation: Modulation) -> u32 {
    match modulation {
        Modulation::Bpsk => 1,
        Modulation::Qpsk => 2,
        Modulation::Qam16 => 10,
        Modulation::Qam64 => 42,
    }
}

/// Gray-coded level per axis, indexed by the axis bits read b0 first as a binary number.
/// IEEE 802.11-2020 §17.3.5.8 (the 16-QAM and 64-QAM encoding tables).
const LEVELS_1: [i8; 2] = [-1, 1];
const LEVELS_2: [i8; 4] = [-3, -1, 3, 1];
const LEVELS_3: [i8; 8] = [-7, -5, -1, -3, 7, 5, 1, 3];

fn axis_level(bits: &[u8]) -> i8 {
    let index = bits
        .iter()
        .fold(0usize, |acc, &b| (acc << 1) | usize::from(b & 1));
    match bits.len() {
        1 => LEVELS_1[index],
        2 => LEVELS_2[index],
        _ => LEVELS_3[index],
    }
}

/// Maps one group of exactly N_BPSC bits (b0 first) to its constellation point.
pub fn map_group(group: &[u8], modulation: Modulation) -> Result<Point, PartialGroup> {
    let n_bpsc = modulation.bits_per_subcarrier();
    if group.len() != n_bpsc {
        return Err(PartialGroup {
            len: group.len(),
            n_bpsc,
        });
    }
    Ok(map_whole_group(group, modulation))
}

fn map_whole_group(group: &[u8], modulation: Modulation) -> Point {
    match modulation {
        Modulation::Bpsk => Point {
            i: axis_level(&group[..1]),
            q: 0,
        },
        _ => {
            let half = modulation.bits_per_subcarrier() / 2;
            Point {
                i: axis_level(&group[..half]),
                q: axis_level(&group[half..2 * half]),
            }
        }
    }
}

/// Maps a bit stream to constellation points, N_BPSC bits per point.
pub fn map_bits(bits: &[u8], modulation: Modulation) -> Result<Vec<Point>, PartialGroup> {
    let n_bpsc = modulation.bits_per_subcarrier();
    if !bits.len().is_multiple_of(n_bpsc) {
        return Err(PartialGroup {
            len: bits.len(),
            n_bpsc,
        });
    }
    Ok(bits
        .chunks(n_bpsc)
        .map(|g| map_whole_group(g, modulation))
        .collect())
}
