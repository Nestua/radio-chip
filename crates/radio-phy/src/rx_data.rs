//! The data path of [`RxDataFixed`] as a schedule in clocks (ADR 0010, decision 6.1c): in
//! which clock the reader takes each beat of the hand-off's stream, when each FFT window is
//! transformed, when the shared CORDIC, divider, products unit and rotator take each job,
//! when the Viterbi decoder steps, and in which clock the verdict's strobe and every event
//! for the MAC are out.
//!
//! The model decides *what* comes out ([`RxDataFixed`] is called with every beat taken, and
//! its events are given back as they are); this module decides *when*. It is a reader of the
//! hand-off ([`HandoffReader`]), so [`crate::rx_handoff::run_handoff`] runs the front's
//! stream and this data path together, the verdict going back in the clock the schedule
//! gives it. Every latency below is an **allowance for RTL not yet written** ([`DataTiming`]):
//! the structure — what waits for what, and the order on each shared unit — is the decision
//! here; the numbers are replaced block by block as the RTL comes (ADR 0010, decision 6.2).
//!
//! # The clocks
//!
//! Counts are from cycle to cycle, as in [`crate::rx_decision`]: a unit with a latency of k
//! that has its input in cycle X has its output registered in cycle X + k. Beat n of a packet
//! (n = 0 the first sample of T1's window) is taken in cycle c(n). A window is *complete* in
//! the cycle W its last sample is taken.
//!
//! **The windows.** T1's at samples 0 … 63, T2's at 64 … 127, SIGNAL's at 144 + σ₀ … and data
//! symbol i's at 144 + 80·i + σᵢ₋₁ …, σ the tracker's shift (decision 2: one symbol after
//! another). SIGNAL's and the first data symbol's places are known at the first beat (SIGNAL's
//! fit is identically zero, ADR 0010 decision 3). For data symbol i + 1 (i ≥ 1) a shift moves
//! by at most a sample a symbol, so the samples before 144 + 80·(i + 1) + σᵢ₋₁ − 1 are taken
//! (and dropped) whenever they come; from there on the reader waits for cycle Sᵢ + 1, Sᵢ the
//! cycle symbol i's shift is registered. A window's first sample also waits for a window
//! store: `window_stores` of them, each held from its window's first sample to its
//! transform's start + `fft_read`.
//!
//! **The FFT**, one transform at a time: F = max(W + 1, the last transform's bins), the bins
//! B = F + `fft`.
//!
//! **The channel**: Hs, the gains, the packet's shift and Σg registered in C = B(T2) +
//! `channel`.
//!
//! **The divider** (one, serial, a quotient bit a clock and `divide_overhead`), in a fixed
//! order per packet: from C, BPSK's scale (quotient bits: the bit length of its constant A,
//! plus one: A·2ⁿ/Σg lies in (A, 2A]), the tracker's two coefficients (K − 2 bits each), the
//! variance (V + 2 bits: it is compared with 3 nominal ones); then per data symbol i its fit
//! (P + R − 4 bits, the model's saturation of δ), and after the first symbol's fit the data's
//! scale (asked at the verdict: it waits for the rate).
//!
//! **The CORDIC** (iterative, one angle at a time, `cordic` clocks each), in a fixed order:
//! SIGNAL's common phase, then its two pair angles (they feed only the fit's sums); for each
//! data symbol its two pair angles, then its common phase.
//!
//! **Symbol s** (0 = SIGNAL), its products u = Z·Hs* on the products unit from
//! P = max(B, C, the cycle the last symbol's soft pass has read its products): the four
//! pilots in P … P + 3; the two pair products hi·conj(lo) in Q = P + 3 + `product` and Q + 1
//! (the pilots registered), normalised a clock later; the 48 data products in the other
//! slots from P + 4 on. Then:
//!
//! | What | Cycle |
//! |---|---|
//! | the outer pair's angle, the inner's | asked at Q + `product` + 1 and Q + `product` + 2, on the CORDIC in its order |
//! | the fit (data symbols) | its division asked `fit_in` after the inner angle |
//! | θ and the shift, S | the fit's quotient + `after_fit`; SIGNAL's θ at the first beat |
//! | the pilots rotated by θ | four inputs to the rotator from R = max(S, the pilots registered, the rotator free) |
//! | the common phase φ | asked R + 3 + `rotator` + 1 (the sum), on the CORDIC in its order |
//! | the soft pass | from D₀ = max(φ + 1, the scale, the rotator free, the soft buffer's slot free): subcarrier j issued in D₀ + j·k, k = ⌈N_BPSC / `soft_products`⌉ |
//! | the symbol's soft values written | L = D₀ + 47·k + `demap` |
//! | its trellis steps | from max(L + 1, the decoder free), a step a clock: 24 for SIGNAL, N_DBPS for data (the last symbol up to the anchor 16 + 8·LENGTH + 6) |
//!
//! Two waits need no rule of their own, whatever the latencies: the soft pass never reads a
//! data product before it is registered (D₀ > φ ≥ P + 3 + `product` + 4 + `rotator`, and
//! subcarrier j's product, registered by P + 6 + j + `product`, is read in D₀ + j·k); and a data symbol's fit never comes
//! before the previous symbol's sums are in (its angles follow that symbol's on the CORDIC).
//! Mutants that drop either wait change no cycle (`docs/gates/rx_ofdm.md`).
//!
//! The soft buffer holds `soft_symbols` symbols (SIGNAL counted): symbol s's slot is free the
//! cycle after the decoder's last step of symbol s − `soft_symbols`.
//!
//! **Out.** The verdict's strobe, the start, or SIGNAL's failure: `signal_check` after
//! SIGNAL's 24th step. Data bit b (from the first SERVICE bit), with the window
//! d = min(depth, anchor): for b < anchor − d, `viterbi_out` after step b + d; the rest from
//! state 0's register at the anchor, a bit a clock, `viterbi_out` + 1 + (b − anchor + d)
//! after the anchor's step. An octet (and the seed's end, at bit 6) `octet` after its last
//! bit; the frame's end with its last octet. An end by a mark: the cycle after the mark, or
//! the last event before it if that is later. Events keep the model's order.
//!
//! **The reader** after the frame's last window (or once the model has ended the frame)
//! takes the stream's tail and drops it. After a mark it takes nothing until the packet's
//! last event is out (ADR 0010, Context 3, rule 5: the next stream waits in the hand-off's
//! history).
//!
//! # The case of ADR 0010's table, by hand (default timing)
//!
//! 24 octets at 54 Mb/s (one data symbol), the window not moved, a beat offered every
//! cycle from c₀ (the first beat):
//!
//! | What | Cycle − c₀ |
//! |---|---|
//! | T1, T2, SIGNAL, data windows complete | 63, 127, 207, 287 |
//! | their transforms start; bins | 64, 160, 256, 352; 160, 256, 352, 448 |
//! | the channel | 366 |
//! | divider: BPSK's scale, two coefficients, the variance | 366 → 386 → 402 → 418 → 432 |
//! | SIGNAL: products from; φ; its soft pass; values written | 366; 398; 399 … 446; 455 |
//! | SIGNAL's 24 steps; **the verdict** | 456 … 479; **489** |
//! | data: products from; pair angles | 448; 458 → 476, 476 → 494 |
//! | its fit; θ; the data's scale | 499 → 529; 531; 529 → 551 |
//! | φ; the soft pass; values written | 557; 558 … 605; 614 |
//! | 214 steps | 615 … 828 |
//! | octet 0 (bit 23 at step 119); octet 11 (bit 111) | 737; 825 |
//! | octet 12 (bit 119, the flush's second bit); **octet 23, the end** | 834; **922** |
//!
//! [`RxDataFixed`]: crate::rx_data_fixed::RxDataFixed

use crate::modulation::Modulation;
use crate::ppdu::Rate;
use crate::rx_data_fixed::{RxDataEnd, RxDataEvent, RxDataFixed};
use crate::rx_front_fixed::{HandoffBeat, HandoffConfig, SignalVerdict};
use crate::rx_handoff::HandoffReader;
use crate::rx_ofdm::RxOfdmConfig;
use crate::rx_ofdm_fixed::{scale_constant, FixedOfdmConfig};
use std::collections::VecDeque;

/// Samples of an FFT window.
const FFT_SIZE: usize = 64;
/// The first sample of SIGNAL's nominal window, from T1's.
const SIGNAL_START: usize = 2 * FFT_SIZE + 16;
/// Samples of a symbol.
const SYMBOL: usize = 80;
/// Data subcarriers, pilots.
const DATA: usize = 48;
const PILOTS: usize = 4;
/// SIGNAL's trellis steps.
const SIGNAL_STEPS: usize = 24;

/// What the data path's schedule is laid out against: allowances for blocks not yet built.
/// Latencies are from cycle to cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataTiming {
    /// From a transform's start to its 64 bins registered; also the least time between two
    /// starts (ADR 0010's 96; `fft64`'s G0 settles it).
    pub fft: usize,
    /// From a transform's start to its window store free (its 64 samples read).
    pub fft_read: usize,
    /// Window stores of 64 samples.
    pub window_stores: usize,
    /// From T2's bins to Hs, the gains, the packet's shift and Σg: two passes over 52 bins.
    pub channel: usize,
    /// The products unit (one complex product a clock).
    pub product: usize,
    /// The iterative CORDIC: one angle in this many clocks.
    pub cordic: usize,
    /// A division's clocks beyond its quotient bits (the operands' load, the rounding).
    pub divide_overhead: usize,
    /// From the inner pair's angle to the fit's division asked: the error's two products,
    /// the sums, the determinant and the numerator.
    pub fit_in: usize,
    /// From the fit's quotient to θ and the shift: δ's saturation, the drift's product.
    pub after_fit: usize,
    /// The data path's rotator, as `hw/rtl/nco.sv`.
    pub rotator: usize,
    /// From a subcarrier's issue to its soft values written: the rotator, the thresholds,
    /// the demapper, the scale's product, the clip.
    pub demap: usize,
    /// The soft scale's multipliers: soft values a clock.
    pub soft_products: usize,
    /// Symbols the soft buffer holds.
    pub soft_symbols: usize,
    /// From a trellis step to its decided bit registered (the best state's tree).
    pub viterbi_out: usize,
    /// From SIGNAL's last step to the verdict: the field's checks, the re-encoding, the
    /// number of symbols.
    pub signal_check: usize,
    /// From an octet's last bit to the octet out.
    pub octet: usize,
}

impl Default for DataTiming {
    fn default() -> Self {
        Self {
            fft: 96,
            fft_read: 64,
            window_stores: 2,
            channel: 110,
            product: 3,
            cordic: 18,
            divide_overhead: 2,
            fit_in: 5,
            after_fit: 2,
            rotator: 4,
            demap: 9,
            soft_products: 6,
            soft_symbols: 2,
            viterbi_out: 3,
            signal_check: 10,
            octet: 1,
        }
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
    /// Its transform's start, and its bins registered.
    pub fft: usize,
    /// See `fft`.
    pub bins: usize,
}

/// One symbol's work on the schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SymbolRun {
    /// The packet's tag.
    pub packet: u8,
    /// 0 SIGNAL, 1 … the data symbols.
    pub symbol: usize,
    /// The products' first slot.
    pub products: usize,
    /// The pair angles: (asked, done) each, outer then inner.
    pub outer: (usize, usize),
    /// See `outer`.
    pub inner: (usize, usize),
    /// The fit's division (start, quotient); `None` for SIGNAL.
    pub fit: Option<(usize, usize)>,
    /// θ and the shift registered (SIGNAL: the first beat).
    pub theta: usize,
    /// The common phase registered.
    pub phi: usize,
    /// The scale this symbol uses registered.
    pub scale: usize,
    /// The soft pass's first issue, and the symbol's soft values written.
    pub soft: (usize, usize),
    /// The trellis steps: (first cycle, last cycle); `None`: none.
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
    /// Asked, started, quotient registered.
    pub asked: usize,
    /// See `asked`.
    pub start: usize,
    /// See `asked`.
    pub done: usize,
}

/// The data symbols' trellis steps: the first data step's index (0-based) and its cycle.
#[derive(Debug, Clone, Copy)]
struct Steps {
    from: usize,
    count: usize,
    cycle: usize,
}

/// The frame being decoded, once SIGNAL has said what it is.
#[derive(Debug, Clone, Copy)]
struct FrameInfo {
    rate: Rate,
    symbols: usize,
    anchor: usize,
}

/// The open packet.
#[derive(Debug, Clone)]
struct Packet {
    tag: u8,
    /// The tracker's shift as the last window was placed.
    shift: i64,
    /// Beats taken.
    n: usize,
    /// The window being filled: its index (0 T1, 1 T2, 2 + s symbol s), first sample, the
    /// first sample the reader may take freely, the cycle after which the rest may be taken.
    next: Option<Next>,
    /// The first-sample cycle of the window being filled.
    filling: Option<usize>,
    /// Windows whose store is not yet free: the cycle each is.
    held: Vec<usize>,
    channel: usize,
    first_beat: usize,
    frame: Option<FrameInfo>,
    verdict: usize,
    data_steps: Vec<Steps>,
    data_done: usize,
    octets: usize,
    /// Symbol ends of the decoder, by symbol, for the soft buffer's slots.
    decoded: Vec<usize>,
    /// The model has ended the frame, or it is past its last window: the tail is dropped.
    draining: bool,
}

#[derive(Debug, Clone, Copy)]
struct Next {
    index: usize,
    start: usize,
    free_below: usize,
    after: usize,
}

/// The streaming data path on a schedule in clocks; see the module's documentation.
#[derive(Debug, Clone)]
pub struct RxDataSchedule {
    model: RxDataFixed,
    cfg: RxOfdmConfig,
    fx: FixedOfdmConfig,
    t: DataTiming,
    depth: usize,
    packet: Option<Packet>,
    /// A mark was taken: nothing more until this cycle has passed.
    wait_until: Option<usize>,
    // The shared units: the first cycle each is free.
    fft_free: usize,
    product_free: usize,
    cordic_free: usize,
    divider_free: usize,
    rotator_free: usize,
    decoder_free: usize,
    /// The products' store: free from this cycle (the last soft pass has read it).
    u_free: usize,
    events: Vec<(usize, RxDataEvent)>,
    strobes: VecDeque<(usize, u8, Option<SignalVerdict>)>,
    windows: Vec<WindowRun>,
    symbols: Vec<SymbolRun>,
    divisions: Vec<DivisionRun>,
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
            depth: match fx.traceback {
                0 => usize::MAX,
                d => d,
            },
            packet: None,
            wait_until: None,
            fft_free: 0,
            product_free: 0,
            cordic_free: 0,
            divider_free: 0,
            rotator_free: 0,
            decoder_free: 0,
            u_free: 0,
            events: Vec::new(),
            strobes: VecDeque::new(),
            windows: Vec::new(),
            symbols: Vec::new(),
            divisions: Vec::new(),
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

    /// The cycle of the last event (0: none).
    pub fn last_event(&self) -> usize {
        self.events.last().map_or(0, |e| e.0)
    }

    /// Whether a beat at the stream's pins is taken in cycle `clock`.
    pub fn ready_at(&self, clock: usize) -> bool {
        if let Some(w) = self.wait_until {
            if clock <= w {
                return false;
            }
        }
        let Some(p) = &self.packet else {
            return true;
        };
        if p.draining {
            return true;
        }
        let Some(next) = p.next else {
            return true;
        };
        if p.n < next.free_below {
            return true;
        }
        if clock <= next.after {
            return false;
        }
        if p.n == next.start {
            // A window store: those not yet free in this cycle.
            let busy = p.held.iter().filter(|&&f| clock < f).count();
            return busy < self.t.window_stores;
        }
        true
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
        if let Some(w) = self.wait_until {
            if clock > w {
                self.wait_until = None;
            }
        }
        let starts = matches!(beat, HandoffBeat::Sample { first: Some(_), .. });
        let out = self.model.push(beat);
        match beat {
            HandoffBeat::End(_) => {
                self.stamp_ends(clock, &out);
                self.packet = None;
                self.wait_until = Some(self.last_event());
            }
            HandoffBeat::Sample { first, .. } => {
                if starts {
                    // An end for a packet in flight (no mark before it), then the new one.
                    let (ends, rest): (Vec<_>, Vec<_>) = out
                        .iter()
                        .partition(|e| matches!(e, RxDataEvent::End(RxDataEnd::Aborted(_))));
                    self.stamp_ends(clock, &ends);
                    let tag = first.map_or(0, |f| f.packet);
                    self.open(clock, tag);
                    self.sample(clock, &rest);
                } else {
                    self.sample(clock, &out);
                }
            }
        }
    }

    fn open(&mut self, clock: usize, tag: u8) {
        self.packet = Some(Packet {
            tag,
            shift: self.model.shift(),
            n: 0,
            next: Some(Next {
                index: 0,
                start: 0,
                free_below: 0,
                after: 0,
            }),
            filling: None,
            held: Vec::new(),
            channel: 0,
            first_beat: clock,
            frame: None,
            verdict: 0,
            data_steps: Vec::new(),
            data_done: 0,
            octets: 0,
            decoded: Vec::new(),
            draining: false,
        });
    }

    /// Ends given by a mark or a new first beat: the cycle after, or the last event's.
    fn stamp_ends(&mut self, clock: usize, out: &[RxDataEvent]) {
        for e in out {
            self.push_event(clock + 1, *e);
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
        if let Some(next) = p.next {
            if n == next.start {
                p.filling = Some(clock);
            }
            if n == next.start + FFT_SIZE - 1 {
                completed = Some((next.index, p.filling.unwrap_or(clock)));
            }
        }
        if let Some((index, first)) = completed {
            self.window_done(&mut p, index, first, clock);
            // The events of this window, then where the next one is.
            self.stamp(&mut p, clock, out);
            p.filling = None;
            p.next = self.next_window(&p, index);
            p.shift = self.model.shift();
        } else {
            self.stamp(&mut p, clock, out);
        }
        if !self.model.busy() {
            p.draining = true;
        }
        self.packet = Some(p);
    }

    /// Where the window after `index` is, as far as the reader may know it.
    fn next_window(&self, p: &Packet, index: usize) -> Option<Next> {
        let shift = self.model.shift();
        match index {
            0 => Some(Next {
                index: 1,
                start: FFT_SIZE,
                free_below: FFT_SIZE,
                after: 0,
            }),
            // SIGNAL's and the first data symbol's places are known at the first beat.
            1 | 2 => {
                let start = place(SIGNAL_START + SYMBOL * (index - 1), shift);
                Some(Next {
                    index: index + 1,
                    start,
                    free_below: start,
                    after: 0,
                })
            }
            _ => {
                let s = index - 2;
                let frame = p.frame?;
                if s >= frame.symbols {
                    return None;
                }
                let nominal = SIGNAL_START + SYMBOL * (s + 1);
                let start = place(nominal, shift);
                // A shift moves by a sample a symbol at most: before the old place less one
                // nothing can be the window.
                let free_below = place(nominal, p.shift).saturating_sub(1).min(start);
                let theta = self
                    .symbols
                    .last()
                    .filter(|r| r.packet == p.tag && r.symbol == s)
                    .map_or(0, |r| r.theta);
                Some(Next {
                    index: index + 1,
                    start,
                    free_below,
                    after: theta,
                })
            }
        }
    }

    fn window_done(&mut self, p: &mut Packet, index: usize, first: usize, clock: usize) {
        let t = self.t;
        let fft = (clock + 1).max(self.fft_free);
        let bins = fft + t.fft;
        self.fft_free = bins;
        p.held.retain(|&f| f > clock);
        p.held.push(fft + t.fft_read);
        let window = match index {
            0 => Window::T1,
            1 => Window::T2,
            i => Window::Symbol(i - 2),
        };
        self.windows.push(WindowRun {
            packet: p.tag,
            window,
            first,
            last: clock,
            fft,
            bins,
        });
        match window {
            Window::T1 => {}
            Window::T2 => {
                p.channel = bins + t.channel;
                let c = p.channel;
                let a = |m| bit_len(scale_constant(m, &self.cfg, &self.fx)) + 1;
                let bpsk = a(Modulation::Bpsk);
                let coef = self.fx.coef_frac.saturating_sub(2) as usize;
                let var = self.fx.var_frac as usize + 2;
                self.divide(p.tag, Division::Scale(Modulation::Bpsk), c, bpsk);
                self.divide(p.tag, Division::Outer, c, coef);
                self.divide(p.tag, Division::Inner, c, coef);
                self.divide(p.tag, Division::Variance, c, var);
            }
            Window::Symbol(s) => self.symbol(p, s, bins),
        }
    }

    fn divide(&mut self, packet: u8, job: Division, asked: usize, bits: usize) -> usize {
        let start = asked.max(self.divider_free);
        let done = start + bits + self.t.divide_overhead;
        self.divider_free = done;
        self.divisions.push(DivisionRun {
            packet,
            job,
            asked,
            start,
            done,
        });
        done
    }

    fn angle(&mut self, asked: usize) -> (usize, usize) {
        let start = asked.max(self.cordic_free);
        let done = start + self.t.cordic;
        self.cordic_free = done;
        (asked, done)
    }

    /// The pilots' rotation by θ and the common phase: φ registered.
    fn phase(&mut self, theta: usize, pilots: usize) -> usize {
        let r = theta.max(pilots).max(self.rotator_free);
        self.rotator_free = r + PILOTS;
        self.angle(r + PILOTS - 1 + self.t.rotator + 1).1
    }

    /// Symbol s's work from its bins.
    fn symbol(&mut self, p: &mut Packet, s: usize, bins: usize) {
        let t = self.t;
        let products = bins.max(p.channel).max(self.u_free).max(self.product_free);
        let pilots = products + PILOTS - 1 + t.product;
        let q = pilots;
        // The data products' slots: from P + 4, around the pair products'.
        let mut slots = Vec::with_capacity(DATA);
        let mut slot = products + PILOTS;
        while slots.len() < DATA {
            if slot != q && slot != q + 1 {
                slots.push(slot);
            }
            slot += 1;
        }
        self.product_free = slot.max(q + 2);
        let outer_in = q + t.product + 1;
        let inner_in = q + 1 + t.product + 1;
        let (theta, phi, outer, inner, fit, scale, modulation) = if s == 0 {
            let phi = self.phase(p.first_beat, pilots);
            let outer = self.angle(outer_in);
            let inner = self.angle(inner_in);
            let scale = self
                .divisions
                .iter()
                .rev()
                .find(|d| d.packet == p.tag && d.job == Division::Scale(Modulation::Bpsk))
                .map_or(0, |d| d.done);
            (
                p.first_beat,
                phi,
                outer,
                inner,
                None,
                scale,
                Modulation::Bpsk,
            )
        } else {
            let outer = self.angle(outer_in);
            let inner = self.angle(inner_in);
            let asked = inner.1 + t.fit_in;
            let bits = (self.fx.front.nco_phase_bits + self.fx.rate_frac).saturating_sub(4);
            let fit_done = self.divide(p.tag, Division::Fit(s), asked, bits as usize);
            let fit_start = self.divisions.last().map_or(asked, |d| d.start);
            let theta = fit_done + t.after_fit;
            let modulation = p.frame.map_or(Modulation::Bpsk, |f| f.rate.modulation());
            if s == 1 {
                let bits = bit_len(scale_constant(modulation, &self.cfg, &self.fx)) + 1;
                self.divide(p.tag, Division::Scale(modulation), p.verdict, bits);
            }
            let scale = self
                .divisions
                .iter()
                .rev()
                .find(|d| d.packet == p.tag && d.job == Division::Scale(modulation))
                .map_or(0, |d| d.done);
            let phi = self.phase(theta, pilots);
            (
                theta,
                phi,
                outer,
                inner,
                Some((fit_start, fit_done)),
                scale,
                modulation,
            )
        };
        let k = modulation
            .bits_per_subcarrier()
            .div_ceil(t.soft_products.max(1));
        let slot_free = s
            .checked_sub(t.soft_symbols)
            .and_then(|i| p.decoded.get(i))
            .map_or(0, |&z| z + 1);
        let d0 = (phi + 1).max(scale).max(self.rotator_free).max(slot_free);
        self.rotator_free = d0 + DATA * k;
        self.u_free = d0 + DATA * k;
        let written = d0 + (DATA - 1) * k + t.demap;
        // The trellis steps.
        let count = if s == 0 {
            SIGNAL_STEPS
        } else {
            match p.frame {
                Some(f) => f
                    .rate
                    .data_bits_per_symbol()
                    .min(f.anchor.saturating_sub(p.data_done)),
                None => 0,
            }
        };
        let steps = (count > 0).then(|| {
            let first = (written + 1).max(self.decoder_free);
            let last = first + count - 1;
            self.decoder_free = last + 1;
            (first, last)
        });
        p.decoded.push(steps.map_or(written, |(_, last)| last));
        if let Some((first, _)) = steps {
            if s == 0 {
                p.verdict = first + count - 1 + t.signal_check;
            } else {
                p.data_steps.push(Steps {
                    from: p.data_done,
                    count,
                    cycle: first,
                });
                p.data_done += count;
            }
        }
        self.symbols.push(SymbolRun {
            packet: p.tag,
            symbol: s,
            products,
            outer,
            inner,
            fit,
            theta,
            phi,
            scale,
            soft: (d0, written),
            steps,
        });
    }

    /// The cycle data step k (1-based) is taken in.
    fn step_cycle(p: &Packet, k: usize) -> Option<usize> {
        p.data_steps
            .iter()
            .find(|s| k > s.from && k <= s.from + s.count)
            .map(|s| s.cycle + (k - s.from - 1))
    }

    /// The cycle data bit `b` is out of the decoder.
    fn bit_out(&self, p: &Packet, b: usize) -> Option<usize> {
        let anchor = p.frame?.anchor;
        let d = self.depth.min(anchor).max(1);
        let regular = anchor - d;
        if b < regular {
            Some(Self::step_cycle(p, b + d)? + self.t.viterbi_out)
        } else {
            Some(Self::step_cycle(p, anchor)? + self.t.viterbi_out + 1 + (b - regular))
        }
    }

    /// The model's events of one beat, with their cycles.
    fn stamp(&mut self, p: &mut Packet, clock: usize, out: &[RxDataEvent]) {
        for &e in out {
            let at = match e {
                RxDataEvent::Verdict {
                    packet, verdict, ..
                } => {
                    self.strobes.push_back((p.verdict, packet, verdict));
                    p.verdict
                }
                RxDataEvent::Start { signal, .. } => {
                    let symbols = crate::ppdu::data_symbols(signal.rate, signal.length);
                    p.frame = Some(FrameInfo {
                        rate: signal.rate,
                        symbols,
                        anchor: 16 + 8 * signal.length + 6,
                    });
                    p.verdict
                }
                RxDataEvent::End(RxDataEnd::Signal(_)) => p.verdict,
                RxDataEvent::Octet { .. } => {
                    let b = 16 + 8 * p.octets + 7;
                    p.octets += 1;
                    self.bit_out(p, b).map_or(clock + 1, |c| c + self.t.octet)
                }
                RxDataEvent::End(RxDataEnd::Decoded) => self.last_event(),
                RxDataEvent::End(RxDataEnd::Seed) => {
                    self.bit_out(p, 6).map_or(clock + 1, |c| c + self.t.octet)
                }
                RxDataEvent::End(_) => clock + 1,
            };
            self.push_event(at, e);
        }
    }
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
