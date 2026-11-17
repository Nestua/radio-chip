//! Soft-decision Viterbi decoder for the 802.11 K = 7, (133, 171)₈ convolutional code with the
//! rate-2/3 and rate-3/4 puncturing of IEEE 802.11-2024 §17.3.5.6.
//!
//! The mirror of [`crate::convolutional`]: same state numbering, same output order. Maximum
//! likelihood over the whole block (full traceback); the definitions — soft-value sign, branch
//! metric, tie rules, termination — are in `docs/gates/viterbi.md` and are part of the spec the
//! RTL will match.

use crate::convolutional::{CodeRate, PartialPeriod, G0, G1};

/// Number of trellis states (2^(K−1)).
pub const STATES: usize = 64;

/// Where the decoder may assume the encoder's state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Nothing known: trace back from the state with the best final metric.
    Free,
    /// The encoder was in state 0 after this many input bits (802.11: 16 + 8·LENGTH + 6, the
    /// end of the tail). Later bits (the pad) follow the best final state.
    ZeroAfter(usize),
}

/// Errors from the decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViterbiError {
    /// The soft stream does not end on a whole puncturing period.
    Punctured(PartialPeriod),
    /// The depunctured stream must hold an (A, B) pair per input bit.
    OddLength(usize),
    /// `ZeroAfter(at)` lies beyond the `len` decoded bits.
    TerminationBeyondEnd {
        /// Requested termination point.
        at: usize,
        /// Number of input bits in the block.
        len: usize,
    },
}

impl std::fmt::Display for ViterbiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ViterbiError::Punctured(p) => write!(f, "{p}"),
            ViterbiError::OddLength(n) => write!(f, "rate-1/2 stream of odd length {n}"),
            ViterbiError::TerminationBeyondEnd { at, len } => {
                write!(f, "termination after {at} bits in a block of {len}")
            }
        }
    }
}

impl std::error::Error for ViterbiError {}

/// Re-inserts an erasure (0) at every position the rate's pattern stole, giving the rate-1/2
/// stream A0 B0 A1 B1 …
pub fn depuncture(soft: &[i32], rate: CodeRate) -> Result<Vec<i32>, ViterbiError> {
    let pattern = rate.pattern();
    let kept = pattern.iter().filter(|&&k| k == 1).count();
    if !soft.len().is_multiple_of(kept) {
        return Err(ViterbiError::Punctured(PartialPeriod {
            len: soft.len(),
            period: kept,
        }));
    }
    let mut out = Vec::with_capacity(soft.len() / kept * pattern.len());
    for chunk in soft.chunks(kept) {
        let mut it = chunk.iter();
        for &keep in pattern {
            out.push(if keep == 1 {
                it.next().copied().unwrap_or(0)
            } else {
                0
            });
        }
    }
    Ok(out)
}

/// The (A, B) output bits of the branch leaving `state` with input `bit`.
fn branch_output(state: usize, bit: usize) -> (i64, i64) {
    let window = ((bit << 6) | state) as u8;
    let parity = |g: u8| i64::from((window & g).count_ones() & 1);
    (parity(G0), parity(G1))
}

/// Decodes a transmitted soft stream at `rate`: depunctures, then runs the Viterbi algorithm.
/// Returns one bit per rate-1/2 pair.
pub fn decode(
    soft: &[i32],
    rate: CodeRate,
    termination: Termination,
) -> Result<Vec<u8>, ViterbiError> {
    let full = depuncture(soft, rate)?;
    if !full.len().is_multiple_of(2) {
        return Err(ViterbiError::OddLength(full.len()));
    }
    let len = full.len() / 2;
    if let Termination::ZeroAfter(at) = termination {
        if at > len {
            return Err(ViterbiError::TerminationBeyondEnd { at, len });
        }
    }

    // Branch outputs as ±1 signs: c = 0 → +1, c = 1 → −1, so the metric is Σ s·(1 − 2c).
    let mut sign = [[(0i64, 0i64); 2]; STATES];
    for (s, row) in sign.iter_mut().enumerate() {
        for (u, out) in row.iter_mut().enumerate() {
            let (a, b) = branch_output(s, u);
            *out = (1 - 2 * a, 1 - 2 * b);
        }
    }

    const UNREACHABLE: i64 = i64::MIN / 4;
    let mut metric = [UNREACHABLE; STATES];
    metric[0] = 0;
    // survivor[t][s]: the dropped (oldest) bit of the predecessor chosen for state s at step t.
    let mut survivor = vec![[0u8; STATES]; len];
    // State 0 at the termination point: keep a snapshot so the forced traceback can start there.
    for (t, pair) in full.chunks(2).enumerate() {
        let (sa, sb) = (i64::from(pair[0]), i64::from(pair[1]));
        let mut next = [UNREACHABLE; STATES];
        for (s, slot) in next.iter_mut().enumerate() {
            let u = s >> 5;
            // Predecessors: (s << 1) & 63 with the dropped bit 0 or 1.
            let p0 = (s << 1) & (STATES - 1);
            let p1 = p0 | 1;
            let m = |p: usize| {
                let (ca, cb) = sign[p][u];
                metric[p].saturating_add(sa * ca + sb * cb)
            };
            let (m0, m1) = (m(p0), m(p1));
            // Ties keep the predecessor whose dropped bit is 0.
            if m1 > m0 {
                *slot = m1;
                survivor[t][s] = 1;
            } else {
                *slot = m0;
            }
        }
        metric = next;
    }

    let best_end = (0..STATES).fold(0, |best, s| if metric[s] > metric[best] { s } else { best });
    let mut bits = vec![0u8; len];
    let trace = |bits: &mut [u8], from_step: usize, to_step: usize, mut state: usize| {
        for t in (to_step..from_step).rev() {
            bits[t] = (state >> 5) as u8;
            state = ((state << 1) & (STATES - 1)) | usize::from(survivor[t][state]);
        }
        state
    };
    match termination {
        Termination::Free => {
            trace(&mut bits, len, 0, best_end);
        }
        Termination::ZeroAfter(at) => {
            trace(&mut bits, len, at, best_end);
            trace(&mut bits, at, 0, 0);
        }
    }
    Ok(bits)
}
