//! The data path of [`RxDataFixed`] as a schedule in clocks (ADR 0010, decision 6.1c), restated
//! from the RTL as built (decision 6.2; the top level's G0, `docs/gates/rx_data.md`): in which
//! clock the top level's reader takes each beat of the hand-off's stream, when `fft64` gives
//! each window's bins, when `rx_channel`, `rx_symbol`, `rx_track`, `rx_deint`, the Viterbi
//! decoder and `rx_signal` give what they give, the shared CORDIC's and divider's order, and in
//! which clock the verdict's strobe and every event for the MAC are out.
//!
//! The model decides *what* comes out ([`RxDataFixed`] is called with every beat taken, and its
//! events are given back as they are); this module decides *when*. It is a reader of the
//! hand-off ([`HandoffReader`]), so [`crate::rx_handoff::run_handoff`] runs the front's stream
//! and this data path together, the verdict going back in the clock the schedule gives it. The
//! top level's RTL (`hw/rtl/rx_data.sv`) is held to it cycle for cycle at its pins (gate
//! `make -C hw rx_data`, vectors `rvec rx_data`).
//!
//! # The clocks
//!
//! Cycle c ends at clock edge c. A signal *registered* on edge E is seen in cycle E + 1 and
//! *taken* by the block it is wired to on edge E + 1; every wire between two blocks of the data
//! path is direct (no register) except where this text says. The blocks' latencies are their
//! headers' (in edges after the edge that takes the trigger), each checked by the block's own
//! bench; [`RxDataSchedule::symbols`] and the other lists give registered edges. A beat of the
//! stream is *taken in cycle c* when `out_valid` and the reader's `out_ready` are high in it.
//! The verdict's strobe *in cycle c* is `rx_signal`'s `sig_valid` registered on edge c − 1
//! (`rx_handoff` takes it on edge c), and an event for the MAC *in cycle c* is one registered
//! on edge c − 1.
//!
//! # The reader (the top level's; ready is a register)
//!
//! Windows: T1 at samples 0 … 63, T2 at 64 … 127, SIGNAL at 144 + σ₀ (σ the tracker's shift;
//! σ₀ the packet's start, 0), data symbol k at 144 + 80·k + σ, σ after symbol k − 1's move. A
//! window's samples go to `fft64` through one register (taken in cycle c, seen by the FFT in
//! c + 1); T1 and T2 carry `in_next` on their 64th sample (the founder's choice, 2026-11-24:
//! T1 → T2 → SIGNAL chained), SIGNAL and the data symbols are flushed. Other samples are taken
//! and dropped. `out_ready` in cycle c is registered on edge c − 1 from what is seen in cycle
//! c − 1, so a beat that waits for a block's output registered on edge E is taken in cycle
//! E + 2 at the earliest. For data symbol k's window, with S the first sample of its old place
//! less one (a shift moves by a sample a symbol at most):
//! - samples before S are taken whenever they come;
//! - from S on, cycle ≥ θ + 2, θ the edge symbol k − 1's θ is registered (SIGNAL's for k = 1),
//!   and for k ≥ 2 also cycle ≥ V + 2, V the edge of the verdict (the reader learns the
//!   number of data symbols there);
//! - the window's first sample also waits for the FFT (cycle ≥ W + 65, W the cycle of the last
//!   flushed window's 64th sample: the flush's 64 tokens) and for `rx_symbol`'s slot (cycle ≥
//!   F + 43, F the edge of the first soft value of the symbol two before, whose last read
//!   empties the slot on F + 41).
//!
//! From the frame's last window on (once the reader knows it: cycle ≥ V + 2), and from V + 2
//! after a SIGNAL that failed, the reader takes the stream's tail and drops it. After a mark it
//! takes nothing until the packet's last event is out (ADR 0010, Context 3, rule 5); every
//! block takes the top level's drop with that event.
//!
//! # The blocks, in edges
//!
//! - **`fft64`**: a window's 64th sample taken in cycle c; bin j (in the order it leaves) taken
//!   by the block behind it on edge c + 14 + j when the window is flushed; when it is chained,
//!   bin 0 on c + 14 and bin j ≥ 1 on c′ + 14, c′ the cycle the next window's sample j − 1 is
//!   taken. Bin 63 is bin −1, used: a window's last used bin is its 64th.
//! - **`rx_channel`**: chan_valid registered on CH = T2's last bin + 63; the quotients on q₀ =
//!   CH + 3 + 18 (BPSK's scale), then each 2 + its quotient bits later (outer 14, inner 14,
//!   variance 12): CH + 21, 37, 53, 67. The data's scale (not asked for BPSK: its scale is
//!   already in): asked with the verdict (taken V + 1), the divider takes it no earlier than
//!   V + 3, quotient bits 18, 19, 20 (QPSK, 16-QAM, 64-QAM), q_valid on the division's done + 1.
//! - **The divider** (`div_share` → `div_serial`): a division taken on edge e is done on e + q
//!   and the divider takes again on e + q + 1; `rx_channel` (a) before the tracker (b) when
//!   both wait.
//! - **The CORDIC** (`cordic_share` → `cordic_iter`): a request registered on R is taken on
//!   max(R + 1, the CORDIC free), the angle registered 17 later, the CORDIC free 18 after the
//!   take. In a packet the requests come in the order SIGNAL's outer, inner, its φ, then per
//!   data symbol outer, inner, φ (each is checked; a break is a violation).
//! - **`rx_symbol`**, symbol s: pil_valid on max(its last bin, CH + 1, the last pilots + 48) +
//!   10 (its slot's and Hs's reads registered before the products, 2026-11-25); its φ request on max(pilots + 48, θ, the last first soft value + 41) + 10; φ 18 after
//!   the CORDIC took it; the first soft value on F = max(φ, the scale's take, for data the
//!   verdict's take V + 1) + 8, then one a clock, 48.
//! - **`rx_track`** (its multiplier pipelined, the founder's choice of 2026-11-25: a product
//!   read three edges after its issue): start taken on c₀ + 1 (registered from the first
//!   beat); SIGNAL's θ on start + 7 (+ 4 without a ratio). A step: T = max(pilots + 1, the last
//!   step's end) + 1; outer request T + 7, inner max(T + 11, the outer's take); λ at C =
//!   max(T + 15, the variance's take + 1); G₀ = max(C + 9, outer angle + 2), G₁ = max(G₀ + 1,
//!   inner angle + 2). SIGNAL's step ends on G₁ + 13 (G₁ + 3 when the packet measures nothing).
//!   A data symbol: its division asked on G₁ + 13 (26 quotient bits), θ 6 after its done; with
//!   nothing measured θ on G₁ + 4. The end of a data step is its θ. (`rx_channel`'s multiplier,
//!   pipelined the same day, moves none of its latencies.)
//! - **`rx_deint`**: SIGNAL's block taken c₀ + 1, the data's V + 1; a symbol's first step
//!   v_valid on max(its last soft value + 1, the block's take, the last step − 1) + 2, then a
//!   step an edge.
//! - **The Viterbi decoder**: step t taken on e(t) (= v_valid + 1); the bit of step t − 95
//!   registered on e(t) + 4 for 95 ≤ t < steps − 1; at the last step the min(96, steps) bits not
//!   yet out from e(last) + 4, one an edge.
//! - **`rx_signal`**: the verdict registered on V = e(SIGNAL's 24th step) + 29.
//! - **The octets** (`hw/rtl/rx_octets.sv`, [`DataTiming::octet`] = 1): an octet registered on
//!   the edge that takes its last bit, one edge after the decoder registered that bit; the
//!   frame's end with its last octet, and a seed of zero ends where the last octet would have
//!   been.
//!
//! An end by a mark (an abort, or the stream ending early) comes in the cycle after the mark,
//! or when the windows already complete have been decoded, if that is later: the bit of their
//! last data step registered and an octet's latency after it; and never before the verdict when
//! SIGNAL's window was complete (the verdict's strobe and `rx_start` come as without the mark,
//! the end in the same cycle as `rx_start` at the earliest). Events of one cycle keep the
//! model's order (start, octet, end). The top level's drop comes with the end, and clears its
//! own register in front of the FFT too.
//!
//! # The case of ADR 0010's table, by hand (`tests/g2_rx_data_schedule.rs`)
//!
//! 24 octets at 54 Mb/s (one data symbol), the window not moved, a beat offered every cycle
//! from c₀ = 0, the CFO's ratio known; registered edges unless "taken":
//!
//! | What | Edge |
//! |---|---|
//! | T1, T2, SIGNAL taken in cycles | 0 … 63, 64 … 127, 144 … 207 |
//! | T2's last bin taken (with SIGNAL's sample 62); chan_valid | 220; 283 |
//! | quotients: BPSK's scale, outer, inner, variance | 304, 320, 336, 350 |
//! | SIGNAL: its bins taken; pilots; θ₀ | 221 … 284; 294; 8 |
//! | its step: T; outer taken, inner taken; G₁; end | 296; 304, 322; 362; 375 |
//! | its φ asked, taken, out; first soft value | 352, 353, 371; 379 |
//! | its 24 steps; **the verdict** (seen in cycle 483) | 429 … 452; **482** |
//! | data window (waits for the flush: 207 + 65) | 272 … 335 |
//! | its bins taken; pilots; T | 349 … 412; 422; 424 |
//! | outer taken, inner taken; G₁; the fit taken, done; θ | 432, 450; 469; 483, 509; 515 |
//! | the data's scale taken (after the fit), q_valid | 510; 531 |
//! | φ asked, out; first soft value | 525, 544; 552 |
//! | 214 steps | 602 … 815 |
//! | bit 0 (step 95), bit 117; the flush's 96 bits | 702, 819; 820 … 915 |
//! | octet 0 (bit 23), 11 (bit 111), 12 (bit 119), **23, the end** | 726, 814, 822, **910** |
//!
//! [`RxDataFixed`]: crate::rx_data_fixed::RxDataFixed

use crate::modulation::Modulation;
use crate::ppdu::Rate;
use crate::rx_data_fixed::{RxDataEnd, RxDataEvent, RxDataFixed};
use crate::rx_front_fixed::{HandoffBeat, HandoffConfig, SignalVerdict};
use crate::rx_handoff::{HandoffReader, HandoffRun};
use crate::rx_ofdm::RxOfdmConfig;
use crate::rx_ofdm_fixed::{scale_constant, FixedOfdmConfig};
use std::collections::VecDeque;

/// Samples of an FFT window.
const FFT_SIZE: usize = 64;
/// SIGNAL's nominal window, from T1's.
const SIGNAL_START: usize = 2 * FFT_SIZE + 16;
/// Samples of a symbol.
const SYMBOL: usize = 80;
/// SIGNAL's trellis steps.
const SIGNAL_STEPS: usize = 24;

/// `fft64`: from a window's 64th sample taken to its bin 0 taken (12 pipeline registers
/// behind the input register, and the reader's register in front).
const FFT_BIN0: usize = 14;
/// `fft64`: from a flushed window's 64th sample taken to the next window's first sample
/// taken (the flush's 64 tokens).
const FFT_FLUSH: usize = 65;
/// `rx_channel`: chan_valid after the edge that takes T2's last used bin.
const CHAN: usize = 63;
/// `rx_channel`: from chan_valid to the first division's take (asked an edge later).
const CHAN_ASK: usize = 2;
/// `rx_channel`: between a done and the next division's take, and from a done to q_valid.
const CHAN_NEXT: usize = 1;
/// The data scale's request (taken with the verdict) to the divider's earliest take.
const SCALE_ASK: usize = 2;
/// `rx_symbol`: pil_valid after its triggers.
const PIL: usize = 10;
/// `rx_symbol`: the products pass, from pil_valid to its last product written.
const PRODUCTS: usize = 48;
/// `rx_symbol`: the CORDIC request after its triggers.
const PHI_ASK: usize = 10;
/// `rx_symbol`: φ after the CORDIC's take.
const PHI: usize = 18;
/// `rx_symbol`: the first soft value after its triggers.
const SOFT: usize = 8;
/// `rx_symbol`: the soft pass's last read after its first soft value (the slot empty).
const SOFT_READ: usize = 41;
/// `cordic_iter`: the angle after its take; the next take after the angle.
const ANGLE: usize = 17;
/// `rx_track`: SIGNAL's θ after start (with a ratio, without).
const TH0: usize = 7;
const TH0_NONE: usize = 4;
/// `rx_track`: the outer and inner requests, λ, the angles' products, from T.
const REQ_OUTER: usize = 7;
const REQ_INNER: usize = 11;
const LAMBDA: usize = 15;
const G0: usize = 9;
/// `rx_track`: SIGNAL's end after G₁ (measured, not), a data symbol's division asked after
/// G₁, θ after the division's done, θ without a measurement after G₁.
const SIG_END: usize = 13;
const SIG_END_NONE: usize = 3;
const FIT_ASK: usize = 13;
const TH_FIT: usize = 6;
const TH_NONE: usize = 4;
/// `rx_track`: the fit's quotient bits (`rx_track.md`, G0: |δ| < 2^26 on every frame).
const FIT_QBITS: usize = 26;
/// `rx_deint`: the first step after its triggers.
const DEINT: usize = 2;
/// `viterbi.sv`: a bit after its step's take; the traceback window.
const VITERBI: usize = 4;
const DEPTH: usize = 96;
/// `rx_signal` behind `viterbi.sv`: the verdict after SIGNAL's last step's take.
const VERDICT: usize = 29;

/// The latencies kept as parameters (every other rule is fixed by the RTL as built).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataTiming {
    /// From an octet's last bit registered to the octet registered: `rx_octets.sv`'s 1
    /// (`docs/gates/rx_octets.md`).
    pub octet: usize,
}

impl Default for DataTiming {
    fn default() -> Self {
        Self { octet: 1 }
    }
}

/// Which window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Window {
    /// The first long training symbol.
    T1,
    /// The second.
    T2,
    /// Symbol s: 0 SIGNAL, 1 … the data symbols.
    Symbol(usize),
}

/// One FFT window on the schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WindowRun {
    /// The packet's tag.
    pub packet: u8,
    /// Which window.
    pub window: Window,
    /// The cycles its first and last samples were taken in.
    pub first: usize,
    /// See `first`.
    pub last: usize,
    /// Its last bin taken (for T1 and T2: once the next window's samples have pushed it out;
    /// 0 when they never did).
    pub bins: usize,
}

/// One symbol's work on the schedule (registered edges unless "taken").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SymbolRun {
    /// The packet's tag.
    pub packet: u8,
    /// 0 SIGNAL, 1 … the data symbols.
    pub symbol: usize,
    /// Its last bin taken.
    pub bins: usize,
    /// pil_valid.
    pub pilots: usize,
    /// The tracker's step: T.
    pub step: usize,
    /// The pair angles: (request, take), outer then inner.
    pub outer: (usize, usize),
    /// See `outer`.
    pub inner: (usize, usize),
    /// G₁.
    pub g1: usize,
    /// The fit's division (take, done); `None` for SIGNAL and with nothing measured.
    pub fit: Option<(usize, usize)>,
    /// θ (SIGNAL: θ₀; its step's end is `end`).
    pub theta: usize,
    /// The step's end.
    pub end: usize,
    /// φ: (request, take, φ).
    pub phi: (usize, usize, usize),
    /// The scale this symbol uses, taken.
    pub scale: usize,
    /// The first and the last soft value.
    pub soft: (usize, usize),
    /// The trellis steps' v_valid: (first, last); `None`: none.
    pub steps: Option<(usize, usize)>,
}

/// A job of the divider.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Division {
    /// A modulation's soft scale.
    Scale(Modulation),
    /// The outer pair's coefficient.
    Outer,
    /// The inner pair's coefficient.
    Inner,
    /// The measurement's variance.
    Variance,
    /// Data symbol i's fit.
    Fit(usize),
}

/// One division on the schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DivisionRun {
    /// The packet's tag.
    pub packet: u8,
    /// Which.
    pub job: Division,
    /// The first edge it could be taken on.
    pub asked: usize,
    /// The edge the divider took it on.
    pub start: usize,
    /// Its done registered.
    pub done: usize,
}

/// The data symbols' trellis steps: the first data step's index (0-based), how many, the
/// first one's v_valid.
#[derive(Debug, Clone, Copy)]
struct Steps {
    from: usize,
    count: usize,
    first: usize,
}

/// The frame being decoded, once SIGNAL has said what it is.
#[derive(Debug, Clone, Copy)]
struct FrameInfo {
    rate: Rate,
    symbols: usize,
    length: usize,
    anchor: usize,
}

/// A window to come: its index (0 T1, 1 T2, 2 + s symbol s), first sample, the first sample
/// that waits, the cycle from which those may be taken, the cycle from which its first may.
#[derive(Debug, Clone, Copy)]
struct Next {
    index: usize,
    start: usize,
    free_below: usize,
    after: usize,
    gate: usize,
}

/// The open packet.
#[derive(Debug, Clone)]
struct Packet {
    tag: u8,
    first_beat: usize,
    /// Beats taken.
    n: usize,
    next: Option<Next>,
    /// The cycles of the window being filled; the last complete one's (for chained bins).
    cycles: Vec<usize>,
    last_cycles: Vec<usize>,
    /// The tracker's shift as the last window was placed.
    shift: i64,
    /// θ₀ registered.
    theta0: usize,
    measures: bool,
    /// chan_valid and the variance's q_valid registered; BPSK's scale taken.
    chan: usize,
    variance: usize,
    bpsk_scale: usize,
    /// The verdict registered.
    verdict: Option<usize>,
    frame: Option<FrameInfo>,
    /// The cycle from which the next flushed window's first sample may be taken.
    fft_free: usize,
    /// Per symbol (0 SIGNAL): its first soft value.
    first_soft: Vec<usize>,
    /// The first of this packet's entries in the symbols' list.
    sym_from: usize,
    last_pilots: usize,
    last_end: usize,
    /// The last step's v_valid (any block).
    last_step: Option<usize>,
    signal_last_take: usize,
    data_steps: Vec<Steps>,
    data_done: usize,
    octets: usize,
    /// The data scale's request: the first edge it could be taken on, its quotient bits; and
    /// once taken, its q_valid taken.
    scale_ask: Option<(usize, usize, Modulation)>,
    data_scale: Option<usize>,
    /// The shared units: the first edge each may take a job on; the last CORDIC request.
    cordic_free: usize,
    divider_free: usize,
    last_request: usize,
    /// From this cycle the reader drops the stream's tail.
    drain_from: Option<usize>,
}

/// The streaming data path on a schedule in clocks; see the module's documentation.
#[derive(Debug, Clone)]
pub struct RxDataSchedule {
    model: RxDataFixed,
    cfg: RxOfdmConfig,
    fx: FixedOfdmConfig,
    t: DataTiming,
    ratio_known: bool,
    packet: Option<Packet>,
    /// A mark was taken: nothing more until this cycle has passed.
    wait_until: Option<usize>,
    events: Vec<(usize, RxDataEvent)>,
    strobes: VecDeque<(usize, u8, Option<SignalVerdict>)>,
    windows: Vec<WindowRun>,
    symbols: Vec<SymbolRun>,
    divisions: Vec<DivisionRun>,
    violations: Vec<(usize, &'static str)>,
}

impl RxDataSchedule {
    /// [`RxDataFixed::new`]'s data path on the timing `t`. `None` where the model is.
    pub fn new(
        cfg: RxOfdmConfig,
        fx: FixedOfdmConfig,
        pace: HandoffConfig,
        ratio: Option<u16>,
        t: DataTiming,
    ) -> Option<Self> {
        Some(Self {
            model: RxDataFixed::new(cfg, fx, pace, ratio)?,
            cfg,
            fx,
            t,
            ratio_known: ratio.is_some_and(|r| r != 0),
            packet: None,
            wait_until: None,
            events: Vec::new(),
            strobes: VecDeque::new(),
            windows: Vec::new(),
            symbols: Vec::new(),
            divisions: Vec::new(),
            violations: Vec::new(),
        })
    }

    /// Every event given so far, with the cycle it is out in (in the model's order; the
    /// cycles do not decrease).
    pub fn events(&self) -> &[(usize, RxDataEvent)] {
        &self.events
    }

    /// Every window transformed.
    pub fn windows(&self) -> &[WindowRun] {
        &self.windows
    }

    /// Every symbol's work.
    pub fn symbols(&self) -> &[SymbolRun] {
        &self.symbols
    }

    /// Every division.
    pub fn divisions(&self) -> &[DivisionRun] {
        &self.divisions
    }

    /// The blocks' contracts the schedule found broken (cycle, which): none on any run the
    /// gates make; the RTL stops its simulation on each.
    pub fn violations(&self) -> &[(usize, &'static str)] {
        &self.violations
    }

    /// The cycle of the last event (0: none).
    pub fn last_event(&self) -> usize {
        self.events.last().map_or(0, |e| e.0)
    }

    /// Whether a beat at the stream's pins is taken in cycle `clock` (the reader's ready).
    pub fn ready_at(&self, clock: usize) -> bool {
        if self.wait_until.is_some_and(|w| clock <= w) {
            return false;
        }
        let Some(p) = &self.packet else {
            return true;
        };
        if p.drain_from.is_some_and(|d| clock >= d) {
            return true;
        }
        let Some(next) = p.next else {
            return true;
        };
        if p.n < next.free_below {
            return true;
        }
        if clock < next.after {
            return false;
        }
        p.n != next.start || clock >= next.gate
    }

    /// The verdict's strobe of cycle `clock`, if any.
    pub fn strobe_at(&mut self, clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        while self.strobes.front().is_some_and(|s| s.0 < clock) {
            self.strobes.pop_front();
        }
        if self.strobes.front().is_some_and(|s| s.0 == clock) {
            return self.strobes.pop_front().map(|s| (s.1, s.2));
        }
        None
    }

    /// The beat taken in cycle `clock`.
    pub fn take(&mut self, clock: usize, beat: HandoffBeat) {
        if self.wait_until.is_some_and(|w| clock > w) {
            self.wait_until = None;
        }
        let out = self.model.push(beat);
        match beat {
            HandoffBeat::End(_) => {
                self.end_by_mark(clock, &out);
                self.packet = None;
                self.wait_until = Some(self.last_event());
            }
            HandoffBeat::Sample { first: Some(f), .. } => {
                // An end for a packet in flight (no mark before it: the hand-off as built
                // never does this), then the new one.
                let (ends, rest): (Vec<_>, Vec<_>) = out
                    .iter()
                    .partition(|e| matches!(e, RxDataEvent::End(RxDataEnd::Aborted(_))));
                if !ends.is_empty() {
                    self.violations
                        .push((clock, "a first beat with no mark before it"));
                }
                self.end_by_mark(clock, &ends);
                self.open(clock, f.packet);
                self.sample(clock, &rest);
            }
            HandoffBeat::Sample { first: None, .. } => self.sample(clock, &out),
        }
    }

    fn open(&mut self, clock: usize, tag: u8) {
        let th0 = if self.ratio_known { TH0 } else { TH0_NONE };
        self.packet = Some(Packet {
            tag,
            first_beat: clock,
            n: 0,
            next: Some(Next {
                index: 0,
                start: 0,
                free_below: 0,
                after: 0,
                gate: 0,
            }),
            cycles: Vec::with_capacity(FFT_SIZE),
            last_cycles: Vec::new(),
            shift: self.model.shift(),
            // start registered from the first beat, taken an edge later.
            theta0: clock + 1 + th0,
            measures: false,
            chan: 0,
            variance: 0,
            bpsk_scale: 0,
            verdict: None,
            frame: None,
            fft_free: 0,
            first_soft: Vec::new(),
            sym_from: self.symbols.len(),
            last_pilots: 0,
            last_end: 0,
            last_step: None,
            signal_last_take: 0,
            data_steps: Vec::new(),
            data_done: 0,
            octets: 0,
            scale_ask: None,
            data_scale: None,
            cordic_free: 0,
            divider_free: 0,
            last_request: 0,
            drain_from: None,
        });
    }

    /// Ends given by a mark (or a first beat with no mark): the cycle after it, or once the
    /// complete windows have been decoded.
    fn end_by_mark(&mut self, clock: usize, out: &[RxDataEvent]) {
        let decoded = self
            .packet
            .as_ref()
            .and_then(|p| p.last_step.filter(|_| !p.data_steps.is_empty()))
            .map_or(0, |s| s + 1 + VITERBI + self.t.octet + 1);
        for e in out {
            self.push_event((clock + 1).max(decoded), *e);
        }
    }

    fn push_event(&mut self, cycle: usize, e: RxDataEvent) {
        let at = cycle.max(self.last_event());
        self.events.push((at, e));
    }

    /// A sample of the open packet taken in `clock`; `out` what the model gave for it.
    fn sample(&mut self, clock: usize, out: &[RxDataEvent]) {
        let Some(mut p) = self.packet.take() else {
            for e in out {
                self.push_event(clock + 1, *e);
            }
            return;
        };
        let n = p.n;
        p.n += 1;
        let mut completed = None;
        let draining = p.drain_from.is_some_and(|d| clock >= d);
        if let Some(next) = p.next.filter(|_| !draining) {
            if n >= next.start && n < next.start + FFT_SIZE {
                p.cycles.push(clock);
            }
            if n == next.start + FFT_SIZE - 1 {
                completed = Some(next.index);
            }
        }
        match completed {
            Some(index) => self.window_done(&mut p, index, clock, out),
            None => {
                for e in out {
                    self.push_event(clock + 1, *e);
                }
            }
        }
        self.packet = Some(p);
    }

    /// A window's 64th sample taken in `clock`.
    fn window_done(&mut self, p: &mut Packet, index: usize, clock: usize, out: &[RxDataEvent]) {
        let cycles = std::mem::take(&mut p.cycles);
        let first = cycles.first().copied().unwrap_or(clock);
        // The window before, chained into this one: its bins, pushed out by these samples.
        if index == 1 || index == 2 {
            let chained = if index == 1 { Window::T1 } else { Window::T2 };
            let bins = cycles.get(FFT_SIZE - 2).map_or(clock, |c| c + FFT_BIN0);
            let prev = &p.last_cycles;
            self.windows.push(WindowRun {
                packet: p.tag,
                window: chained,
                first: prev.first().copied().unwrap_or(0),
                last: prev.last().copied().unwrap_or(0),
                bins,
            });
            if index == 2 {
                self.channel(p, bins);
            }
        }
        let flushed_bins = clock + FFT_BIN0 + FFT_SIZE - 1;
        if index >= 2 {
            p.fft_free = clock + FFT_FLUSH;
            self.windows.push(WindowRun {
                packet: p.tag,
                window: Window::Symbol(index - 2),
                first,
                last: clock,
                bins: flushed_bins,
            });
        }
        p.last_cycles = cycles;
        let s = index.saturating_sub(2);
        if index == 2 {
            p.measures = self.model.measures().unwrap_or(false);
            self.signal(p, flushed_bins);
        } else if index > 2 {
            let real = p.frame.is_some_and(|f| s <= f.symbols);
            if real {
                self.data_symbol(p, s, flushed_bins);
            }
        }
        self.stamp(p, clock, out);
        // The frame's last window: the tail is dropped from here, once the reader knows.
        if let (Some(f), Some(v)) = (p.frame, p.verdict) {
            if index >= 2 && s == f.symbols {
                p.drain_from = Some((clock + 1).max(v + 2));
            }
        }
        p.next = self.next_window(p, index);
        p.shift = self.model.shift();
    }

    /// Where the window after `index` is, as the reader knows it.
    fn next_window(&self, p: &Packet, index: usize) -> Option<Next> {
        let shift = self.model.shift();
        let fixed = |start: usize| Next {
            index: index + 1,
            start,
            free_below: start,
            after: 0,
            gate: 0,
        };
        match index {
            0 => Some(fixed(FFT_SIZE)),
            1 => Some(fixed(place(SIGNAL_START, shift))),
            _ => {
                // Data symbol k = index − 1.
                let k = index - 1;
                let nominal = SIGNAL_START + SYMBOL * k;
                let start = place(nominal, shift);
                let free_below = place(nominal, p.shift).saturating_sub(1).min(start);
                let theta = if k == 1 {
                    p.theta0
                } else {
                    self.symbols[p.sym_from..]
                        .last()
                        .filter(|r| r.symbol == k - 1)
                        .map_or(0, |r| r.theta)
                };
                let mut after = theta + 2;
                if k >= 2 {
                    after = after.max(p.verdict.map_or(usize::MAX, |v| v + 2));
                }
                let slot = k
                    .checked_sub(2)
                    .and_then(|i| p.first_soft.get(i))
                    .map_or(0, |&f| f + SOFT_READ + 2);
                Some(Next {
                    index: index + 1,
                    start,
                    free_below,
                    after,
                    gate: after.max(p.fft_free).max(slot),
                })
            }
        }
    }

    /// The channel and the packet's quotients, T2's last bin taken on `t2`.
    fn channel(&mut self, p: &mut Packet, t2: usize) {
        p.chan = t2 + CHAN;
        let q = |m| bit_len(scale_constant(m, &self.cfg, &self.fx)) + 1;
        let coef = self.fx.coef_frac.saturating_sub(2) as usize;
        let var = self.fx.var_frac as usize + 2;
        let jobs = [
            (Division::Scale(Modulation::Bpsk), q(Modulation::Bpsk)),
            (Division::Outer, coef),
            (Division::Inner, coef),
            (Division::Variance, var),
        ];
        let mut take = p.chan + CHAN_ASK;
        let mut last_q = 0;
        for (i, (job, bits)) in jobs.into_iter().enumerate() {
            let done = take + bits;
            self.divisions.push(DivisionRun {
                packet: p.tag,
                job,
                asked: take,
                start: take,
                done,
            });
            last_q = done + CHAN_NEXT;
            if i == 0 {
                p.bpsk_scale = last_q + 1;
            }
            take = done + 1 + CHAN_NEXT;
        }
        p.variance = last_q;
        p.divider_free = last_q;
    }

    /// The CORDIC: a request registered on `req`; its take.
    fn cordic(&mut self, p: &mut Packet, req: usize) -> usize {
        if req < p.last_request {
            self.violations
                .push((req, "CORDIC requests out of the schedule's order"));
        }
        p.last_request = req;
        let take = (req + 1).max(p.cordic_free);
        p.cordic_free = take + ANGLE + 1;
        take
    }

    /// The divider: a job that may be taken from `asked`; its (take, done).
    fn divide(
        &mut self,
        p: &mut Packet,
        job: Division,
        asked: usize,
        bits: usize,
    ) -> (usize, usize) {
        let take = asked.max(p.divider_free);
        let done = take + bits;
        p.divider_free = done + 1;
        self.divisions.push(DivisionRun {
            packet: p.tag,
            job,
            asked,
            start: take,
            done,
        });
        (take, done)
    }

    /// The data scale, if asked and not yet taken: taken now if it comes before a job that
    /// may be taken from `before` (rx_channel first on a tie), or unconditionally.
    fn data_scale(&mut self, p: &mut Packet, before: Option<usize>) {
        let Some((asked, bits, m)) = p.scale_ask else {
            return;
        };
        let first = match before {
            Some(b) => asked.max(p.divider_free) <= b.max(p.divider_free),
            None => true,
        };
        if first {
            let (_, done) = self.divide(p, Division::Scale(m), asked, bits);
            p.scale_ask = None;
            p.data_scale = Some(done + CHAN_NEXT + 1);
        }
    }

    /// The tracker's step of symbol s from its pilots registered on `pilots`, the last step's
    /// end `prev`: (T, outer, inner, G₁).
    fn step(
        &mut self,
        p: &mut Packet,
        pilots: usize,
        prev: usize,
    ) -> (usize, (usize, usize), (usize, usize), usize) {
        let t = (pilots + 1).max(prev) + 1;
        let outer_req = t + REQ_OUTER;
        let outer = self.cordic(p, outer_req);
        let inner_req = (t + REQ_INNER).max(outer);
        let inner = self.cordic(p, inner_req);
        let c = (t + LAMBDA).max(p.variance + 2);
        let g0 = (c + G0).max(outer + ANGLE + 2);
        let g1 = (g0 + 1).max(inner + ANGLE + 2);
        (t, (outer_req, outer), (inner_req, inner), g1)
    }

    /// rx_symbol's φ: (request, take, φ).
    fn phi(&mut self, p: &mut Packet, pilots: usize, theta: usize) -> (usize, usize, usize) {
        let prev_soft = p.first_soft.last().map_or(0, |&f| f + SOFT_READ);
        let req = (pilots + PRODUCTS).max(theta).max(prev_soft) + PHI_ASK;
        let take = self.cordic(p, req);
        (req, take, take + PHI)
    }

    /// rx_deint's steps for a symbol whose last soft value is registered on `last_soft`, the
    /// block taken on `block`: (first, last) v_valid.
    fn steps(
        &mut self,
        p: &mut Packet,
        last_soft: usize,
        block: usize,
        count: usize,
    ) -> (usize, usize) {
        let prev = p.last_step.map_or(0, |s| s.saturating_sub(1));
        let first = (last_soft + 1).max(block).max(prev) + DEINT;
        let last = first + count - 1;
        p.last_step = Some(last);
        (first, last)
    }

    /// SIGNAL's work from its last bin taken on `bins`.
    fn signal(&mut self, p: &mut Packet, bins: usize) {
        let pilots = bins.max(p.chan + 1) + PIL;
        let theta0 = p.theta0;
        let (t, outer, inner, g1) = self.step(p, pilots, theta0);
        let end = g1 + if p.measures { SIG_END } else { SIG_END_NONE };
        let phi = self.phi(p, pilots, theta0);
        let scale = p.bpsk_scale;
        let soft = phi.2.max(scale) + SOFT;
        let block = p.first_beat + 1;
        let steps = self.steps(p, soft + DATA_LAST, block, SIGNAL_STEPS);
        p.signal_last_take = steps.1 + 1;
        p.verdict = Some(steps.1 + 1 + VERDICT);
        p.first_soft.push(soft);
        p.last_pilots = pilots;
        p.last_end = end;
        self.symbols.push(SymbolRun {
            packet: p.tag,
            symbol: 0,
            bins,
            pilots,
            step: t,
            outer,
            inner,
            g1,
            fit: None,
            theta: theta0,
            end,
            phi,
            scale,
            soft: (soft, soft + DATA_LAST),
            steps: Some(steps),
        });
    }

    /// Data symbol s's work from its last bin taken on `bins`.
    fn data_symbol(&mut self, p: &mut Packet, s: usize, bins: usize) {
        let (Some(v), Some(f)) = (p.verdict, p.frame) else {
            return;
        };
        let pilots = bins.max(p.chan + 1).max(p.last_pilots + PRODUCTS) + PIL;
        // rx_track's one pilot register is free 8 edges after the last step started.
        if pilots + 1 < self.symbols[p.sym_from..].last().map_or(0, |r| r.step + 8) {
            self.violations
                .push((pilots, "pilots before the tracker's register is free"));
        }
        let prev = p.last_end;
        let (t, outer, inner, g1) = self.step(p, pilots, prev);
        let (fit, theta) = if p.measures {
            let asked = g1 + FIT_ASK + 1;
            self.data_scale(p, Some(asked));
            if p.scale_ask.is_some() && s > 1 {
                self.violations
                    .push((asked, "the data's scale still waiting at a later fit"));
            }
            let (take, done) = self.divide(p, Division::Fit(s), asked, FIT_QBITS);
            (Some((take, done)), done + TH_FIT)
        } else {
            (None, g1 + TH_NONE)
        };
        let phi = self.phi(p, pilots, theta);
        let modulation = f.rate.modulation();
        let scale = if modulation == Modulation::Bpsk {
            p.bpsk_scale
        } else {
            self.data_scale(p, None);
            p.data_scale.unwrap_or(0)
        };
        let soft = phi.2.max(scale).max(v + 1) + SOFT;
        // rx_deint's slot: free from the edge before the last step of the symbol two before.
        if let Some(&(_, last)) = self.symbols[p.sym_from..]
            .iter()
            .rev()
            .nth(1)
            .and_then(|r| r.steps.as_ref())
        {
            if last > soft + 1 {
                self.violations
                    .push((soft, "a soft pass into rx_deint's full slot"));
            }
        }
        let count = f
            .rate
            .data_bits_per_symbol()
            .min(f.anchor.saturating_sub(p.data_done));
        let steps = (count > 0).then(|| {
            let (first, last) = self.steps(p, soft + DATA_LAST, v + 1, count);
            if s == 1 && first + 1 < p.signal_last_take + SIGNAL_STEPS + 2 {
                self.violations
                    .push((first, "the decoder's contract between two blocks"));
            }
            (first, last)
        });
        if let Some((first, _)) = steps {
            p.data_steps.push(Steps {
                from: p.data_done,
                count,
                first,
            });
            p.data_done += count;
        }
        p.first_soft.push(soft);
        p.last_pilots = pilots;
        p.last_end = theta;
        self.symbols.push(SymbolRun {
            packet: p.tag,
            symbol: s,
            bins,
            pilots,
            step: t,
            outer,
            inner,
            g1,
            fit,
            theta,
            end: theta,
            phi,
            scale,
            soft: (soft, soft + DATA_LAST),
            steps,
        });
    }

    /// Data step k's (0-based) take by the decoder.
    fn step_take(p: &Packet, k: usize) -> Option<usize> {
        p.data_steps
            .iter()
            .find(|s| k >= s.from && k < s.from + s.count)
            .map(|s| s.first + (k - s.from) + 1)
    }

    /// Data bit `b`'s edge out of the decoder.
    fn bit_out(p: &Packet, b: usize) -> Option<usize> {
        let anchor = p.frame?.anchor;
        let d = DEPTH.min(anchor);
        let regular = anchor - d;
        if b < regular {
            Some(Self::step_take(p, b + DEPTH - 1)? + VITERBI)
        } else {
            Some(Self::step_take(p, anchor - 1)? + VITERBI + (b - regular))
        }
    }

    /// The cycle an octet whose last bit is `b` is out in.
    fn octet_at(&self, p: &Packet, b: usize, clock: usize) -> usize {
        Self::bit_out(p, b).map_or(clock + 1, |e| e + self.t.octet + 1)
    }

    /// The model's events of a window, with their cycles.
    fn stamp(&mut self, p: &mut Packet, clock: usize, out: &[RxDataEvent]) {
        let v = p.verdict.map_or(clock + 1, |v| v + 1);
        for &e in out {
            let at = match e {
                RxDataEvent::Verdict {
                    packet, verdict, ..
                } => {
                    self.strobes.push_back((v, packet, verdict));
                    v
                }
                RxDataEvent::Start { signal, .. } => {
                    let symbols = crate::ppdu::data_symbols(signal.rate, signal.length);
                    p.frame = Some(FrameInfo {
                        rate: signal.rate,
                        symbols,
                        length: signal.length,
                        anchor: 16 + 8 * signal.length + 6,
                    });
                    let m = signal.rate.modulation();
                    if m != Modulation::Bpsk {
                        // rx_channel serves a request made during its quotients only after the
                        // variance's (2 edges later than this model would): the verdict always
                        // comes long after, and a break of that is a violation.
                        if v + SCALE_ASK < p.variance + 3 {
                            self.violations
                                .push((v, "the data's scale asked during the quotients"));
                        }
                        let bits = bit_len(scale_constant(m, &self.cfg, &self.fx)) + 1;
                        // The verdict seen in v: the request taken there, asked an edge later.
                        p.scale_ask = Some((v + SCALE_ASK, bits, m));
                    }
                    v
                }
                RxDataEvent::End(RxDataEnd::Signal(_)) => {
                    p.drain_from = Some(v + 1);
                    v
                }
                RxDataEvent::Octet { .. } => {
                    let b = 16 + 8 * p.octets + 7;
                    p.octets += 1;
                    self.octet_at(p, b, clock)
                }
                RxDataEvent::End(RxDataEnd::Decoded) => self.last_event(),
                RxDataEvent::End(RxDataEnd::Seed) => {
                    let last = p.frame.map_or(0, |f| f.length);
                    self.octet_at(p, 16 + 8 * last - 1, clock)
                }
                RxDataEvent::End(_) => clock + 1,
            };
            self.push_event(at, e);
        }
    }
}

/// From a symbol's first soft value to its last.
const DATA_LAST: usize = 47;

/// The age `rx_end` carries (the founder, 2026-11-25; `docs/gates/rx_data.md`): the samples
/// written since the frame's last one, `adc_count` − `end_stamp` modulo 2^16, on the decoded
/// and the seed-zero ends only (`rx_age_valid`). The frame's end is the nominal one from
/// SIGNAL, without the tracker's σ.
pub fn rx_end_age(end: RxDataEnd, adc_count: u16, end_stamp: u16) -> Option<u16> {
    matches!(end, RxDataEnd::Decoded | RxDataEnd::Seed).then(|| adc_count.wrapping_sub(end_stamp))
}

/// Every end of a run of the hand-off with this schedule as its reader, with its cycle and
/// its age: `run`'s count and stamp as they stand in the end's cycle. An end that carries an
/// age with no stamp yet gives `None` (the gates require one).
pub fn rx_end_ages(run: &HandoffRun, s: &RxDataSchedule) -> Vec<(usize, Option<u16>)> {
    s.events()
        .iter()
        .filter_map(|&(c, e)| match e {
            RxDataEvent::End(end) => Some((
                c,
                run.stamp_at(c)
                    .and_then(|stamp| rx_end_age(end, run.count_before(c), stamp)),
            )),
            _ => None,
        })
        .collect()
}

impl HandoffReader for RxDataSchedule {
    fn ready(&mut self, clock: usize) -> bool {
        self.ready_at(clock)
    }

    fn beat(&mut self, clock: usize, beat: &HandoffBeat) {
        self.take(clock, *beat);
    }

    fn verdict(&mut self, clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        self.strobe_at(clock)
    }
}

/// A nominal place moved by the shift (never before 0).
fn place(nominal: usize, shift: i64) -> usize {
    i64::try_from(nominal)
        .ok()
        .and_then(|n| n.checked_add(shift))
        .and_then(|v| usize::try_from(v).ok())
        .unwrap_or(0)
}

/// Bits of a non-negative value (0 for 0 or less).
fn bit_len(v: i64) -> usize {
    if v <= 0 {
        0
    } else {
        (64 - v.leading_zeros()) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rx_front_fixed::HandoffEnd;
    use crate::rx_ofdm::RxOfdmError;

    #[test]
    fn only_the_decoded_and_the_seed_zero_ends_carry_an_age_and_it_wraps() {
        // The founder's decisions of 2026-11-25: which ends, 16 bits wrapping.
        assert_eq!(rx_end_age(RxDataEnd::Decoded, 900, 300), Some(600));
        assert_eq!(rx_end_age(RxDataEnd::Seed, 5, 65_530), Some(11));
        for end in [
            RxDataEnd::Signal(RxOfdmError::SignalParity),
            RxDataEnd::Truncated,
            RxDataEnd::Aborted(HandoffEnd::Acquisition),
            RxDataEnd::Aborted(HandoffEnd::Overrun),
        ] {
            assert_eq!(rx_end_age(end, 900, 300), None, "{end:?}");
        }
    }
}
