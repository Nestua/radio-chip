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

/// Decodes a transmitted soft stream at `rate`: depunctures, then runs the Viterbi algorithm
/// with a whole-block traceback (maximum likelihood). Returns one bit per rate-1/2 pair.
pub fn decode(
    soft: &[i32],
    rate: CodeRate,
    termination: Termination,
) -> Result<Vec<u8>, ViterbiError> {
    let trellis = forward(soft, rate, termination)?;
    Ok(trellis.traceback(termination, None))
}

/// The hardware-style variant: bit i is decided `depth` steps later by tracing back from the
/// best state at step i + depth (ties: lowest state). Bits within `depth` of the termination
/// point are flushed from there, as in [`decode`]. With `depth` ≥ the block length this equals
/// [`decode`]. Its loss against [`decode`] is a G2 figure (`docs/gates/viterbi.md`).
pub fn decode_windowed(
    soft: &[i32],
    rate: CodeRate,
    termination: Termination,
    depth: usize,
) -> Result<Vec<u8>, ViterbiError> {
    let trellis = forward(soft, rate, termination)?;
    Ok(trellis.traceback(termination, Some(depth.max(1))))
}

/// Largest spread (best − worst reachable path metric) seen over the block: what a fixed-width
/// metric register must hold (G2). Bounded by 24·max|soft| (any state is reachable from any
/// other in K − 1 = 6 steps, each changing a metric by at most 2·max|soft|).
pub fn metric_spread(soft: &[i32], rate: CodeRate) -> Result<i64, ViterbiError> {
    Ok(forward(soft, rate, Termination::Free)?.max_spread)
}

struct Trellis {
    len: usize,
    /// survivor[t][s]: dropped (oldest) bit of the predecessor chosen for state s at step t.
    survivor: Vec<[u8; STATES]>,
    /// best[t]: the best state after t + 1 steps (ties: lowest index).
    best: Vec<usize>,
    max_spread: i64,
}

const UNREACHABLE: i64 = i64::MIN / 4;

fn forward(
    soft: &[i32],
    rate: CodeRate,
    termination: Termination,
) -> Result<Trellis, ViterbiError> {
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

    let mut metric = [UNREACHABLE; STATES];
    metric[0] = 0;
    let mut survivor = vec![[0u8; STATES]; len];
    let mut best = vec![0usize; len];
    let mut max_spread = 0;
    for (t, pair) in full.chunks(2).enumerate() {
        let (sa, sb) = (i64::from(pair[0]), i64::from(pair[1]));
        let mut next = [UNREACHABLE; STATES];
        let (mut best_s, mut best_m, mut lo) = (0usize, i64::MIN, i64::MAX);
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
            // Best state (ties: lowest index) and the lowest metric, in the same pass.
            if *slot > best_m {
                best_m = *slot;
                best_s = s;
            }
            lo = lo.min(*slot);
        }
        metric = next;
        best[t] = best_s;
        if t >= STATES.trailing_zeros() as usize {
            // Every state is reachable after K − 1 steps; before that the spread is not defined.
            max_spread = max_spread.max(best_m - lo);
        }
    }
    Ok(Trellis {
        len,
        survivor,
        best,
        max_spread,
    })
}

impl Trellis {
    /// Follows survivors back from `state` after `from` steps down to step `to`, writing the
    /// bits of steps to..from into `bits` when given; returns the state after `to` steps.
    fn walk(&self, mut bits: Option<&mut [u8]>, from: usize, to: usize, mut state: usize) -> usize {
        for t in (to..from).rev() {
            if let Some(b) = bits.as_deref_mut() {
                b[t] = (state >> 5) as u8;
            }
            state = ((state << 1) & (STATES - 1)) | usize::from(self.survivor[t][state]);
        }
        state
    }

    fn traceback(&self, termination: Termination, depth: Option<usize>) -> Vec<u8> {
        let len = self.len;
        let mut bits = vec![0u8; len];
        if len == 0 {
            return bits;
        }
        let best_end = self.best[len - 1];
        let (anchor, anchor_state) = match termination {
            Termination::Free => (len, best_end),
            Termination::ZeroAfter(at) => (at, 0),
        };
        // Pad bits after the anchor follow the best end state.
        self.walk(Some(&mut bits), len, anchor, best_end);
        // The last `depth` bits before the anchor (all of them for a full traceback) are
        // flushed from the anchor state.
        let flush_from = depth.map_or(0, |d| anchor.saturating_sub(d));
        self.walk(Some(&mut bits), anchor, flush_from, anchor_state);
        if let Some(d) = depth {
            // Earlier bits: decided d steps later from the best state at that step.
            for (i, bit) in bits.iter_mut().enumerate().take(flush_from) {
                let state = self.walk(None, i + d, i + 1, self.best[i + d - 1]);
                *bit = (state >> 5) as u8;
            }
        }
        bits
    }
}
