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

/// Soft-input width of the hardware decoder (G2, `docs/gates/viterbi.md`): values in
/// ±[`SOFT_MAX`], 0 = erasure.
pub const SOFT_BITS: u32 = 8;
/// Largest soft magnitude, 2^(SOFT_BITS−1) − 1.
pub const SOFT_MAX: i32 = (1 << (SOFT_BITS - 1)) - 1;
/// Matched-filter level (signal amplitude 1) that maps to ±[`SOFT_MAX`]: the demapper's
/// scaling contract (R0-09). Chosen by G2 together with [`SOFT_BITS`].
pub const SOFT_CLIP: f64 = 3.0;
/// Traceback window of the hardware decoder ([`decode_windowed`]). 96 by the founder's
/// decision of 2026-11-23 on the receiver's frames lost (`docs/gates/rx_ofdm.md`,
/// `viterbi.md`); R0-10's G2 had chosen 64 on the bit error rate of BPSK in white noise,
/// which through the receiver loses 11 to 25 % more frames of 1000 octets at the rates
/// with a code rate of 3/4.
pub const TRACEBACK_DEPTH: usize = 96;
/// Path-metric register width for modulo arithmetic: 2^METRIC_BITS > 2·24·SOFT_MAX.
pub const METRIC_BITS: u32 = 13;

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

/// Branch outputs as ±1 signs: c = 0 → +1, c = 1 → −1, so the metric is Σ s·(1 − 2c).
type Signs = [[(i64, i64); 2]; STATES];

fn sign_table() -> Signs {
    let mut sign = [[(0i64, 0i64); 2]; STATES];
    for (s, row) in sign.iter_mut().enumerate() {
        for (u, out) in row.iter_mut().enumerate() {
            let (a, b) = branch_output(s, u);
            *out = (1 - 2 * a, 1 - 2 * b);
        }
    }
    sign
}

/// The metrics before the first step: the encoder starts in state 0.
fn start_metrics() -> [i64; STATES] {
    let mut metric = [UNREACHABLE; STATES];
    metric[0] = 0;
    metric
}

/// What one trellis step leaves besides the new metrics.
struct Step {
    /// The dropped (oldest) bit of the predecessor chosen for each state.
    survivor: [u8; STATES],
    /// The best state after the step (ties: lowest index).
    best: usize,
    /// Best minus worst metric after the step.
    spread: i64,
}

/// One add-compare-select step on a rate-1/2 pair of soft values: the decoder's arithmetic,
/// shared by the block decoders and [`WindowedViterbi`].
fn acs(metric: &mut [i64; STATES], sign: &Signs, pair: &[i32]) -> Step {
    let sa = i64::from(pair.first().copied().unwrap_or(0));
    let sb = i64::from(pair.get(1).copied().unwrap_or(0));
    let mut next = [UNREACHABLE; STATES];
    let mut survivor = [0u8; STATES];
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
            survivor[s] = 1;
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
    *metric = next;
    Step {
        survivor,
        best: best_s,
        spread: best_m - lo,
    }
}

/// [`decode_windowed`] as a stream, for a block terminated in state 0 after `anchor` steps
/// (802.11: 16 + 8·LENGTH + 6): soft values go in a whole number of puncture periods at a
/// time (a symbol), and each bit comes out in the push that brings the trellis `depth` steps
/// past it; the last `depth` bits before the anchor come out in the push that reaches it,
/// traced back from state 0. Its bits are those of `decode_windowed` up to the anchor, bit
/// for bit (`tests/viterbi_stream.rs`); the pad after the anchor is not decoded. The model of
/// the hardware decoder in the streaming data path (ADR 0010, decisions 4 and 6.1b).
#[derive(Debug, Clone)]
pub struct WindowedViterbi {
    rate: CodeRate,
    /// The window, at most the anchor (a longer one decides nothing before the flush).
    depth: usize,
    anchor: usize,
    sign: Signs,
    metric: [i64; STATES],
    /// Steps taken.
    steps: usize,
    /// Survivors of the last `depth` − 6 steps, by step modulo the length: a decided bit is
    /// the top bit of the state after i + 1 steps, which is bit 0 of the state after i + 6
    /// (each step back shifts the state), so the walk's first five steps back read nothing
    /// that matters. The same count as the register exchange's 90 bits a survivor for a
    /// window of 96 (ADR 0010, decision 4).
    survivor: Vec<[u8; STATES]>,
    /// The best state after the latest step.
    best: usize,
}

impl WindowedViterbi {
    /// A decoder at `rate` with a window of `depth` steps (0 is taken as 1, as
    /// [`decode_windowed`] does) for a block terminated after `anchor` steps.
    pub fn new(rate: CodeRate, depth: usize, anchor: usize) -> Self {
        let depth = depth.max(1).min(anchor);
        Self {
            rate,
            depth,
            anchor,
            sign: sign_table(),
            metric: start_metrics(),
            steps: 0,
            survivor: vec![
                [0u8; STATES];
                depth
                    .saturating_sub(STATES.trailing_zeros() as usize)
                    .max(1)
            ],
            best: 0,
        }
    }

    /// Whether every bit up to the anchor has come out.
    pub fn is_done(&self) -> bool {
        self.steps >= self.anchor
    }

    fn at(&self, t: usize) -> usize {
        t % self.survivor.len()
    }

    /// Follows survivors back from `state` after `from` steps down to `to` steps (at most
    /// `depth` back); returns the state after `to` steps, and pushes the bits of steps
    /// to..from, newest first, into `bits` when given.
    fn walk(
        &self,
        mut bits: Option<&mut Vec<u8>>,
        from: usize,
        to: usize,
        mut state: usize,
    ) -> usize {
        for t in (to..from).rev() {
            if let Some(b) = bits.as_deref_mut() {
                b.push((state >> 5) as u8);
            }
            state = ((state << 1) & (STATES - 1)) | usize::from(self.survivor[self.at(t)][state]);
        }
        state
    }

    /// The next soft values, a whole number of puncture periods at `rate`; returns the bits
    /// decided by them, in order. Values after the anchor are taken and decide nothing.
    pub fn push(&mut self, soft: &[i32]) -> Result<Vec<u8>, ViterbiError> {
        let full = depuncture(soft, self.rate)?;
        if !full.len().is_multiple_of(2) {
            return Err(ViterbiError::OddLength(full.len()));
        }
        let mut out = Vec::new();
        for pair in full.chunks(2) {
            if self.is_done() {
                break;
            }
            let t = self.steps;
            let s = acs(&mut self.metric, &self.sign, pair);
            let k = self.at(t);
            self.survivor[k] = s.survivor;
            self.best = s.best;
            self.steps += 1;
            let d = self.depth;
            if self.steps == self.anchor {
                // The flush: the last d bits from state 0 at the anchor, oldest first.
                let mut tail = Vec::with_capacity(d);
                self.walk(Some(&mut tail), self.anchor, self.anchor - d, 0);
                out.extend(tail.iter().rev());
            } else if self.steps >= d && self.steps - d < self.anchor - d {
                // Bit i = steps − d, decided from the best state after i + d steps.
                let i = self.steps - d;
                let state = self.walk(None, i + d, i + 1, self.best);
                out.push((state >> 5) as u8);
            }
        }
        Ok(out)
    }
}

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

    let sign = sign_table();
    let mut metric = start_metrics();
    let mut survivor = vec![[0u8; STATES]; len];
    let mut best = vec![0usize; len];
    let mut max_spread = 0;
    for (t, pair) in full.chunks(2).enumerate() {
        let s = acs(&mut metric, &sign, pair);
        survivor[t] = s.survivor;
        best[t] = s.best;
        if t >= STATES.trailing_zeros() as usize {
            // Every state is reachable after K − 1 steps; before that the spread is not defined.
            max_spread = max_spread.max(s.spread);
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
