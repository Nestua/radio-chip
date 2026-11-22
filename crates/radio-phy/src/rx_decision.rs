//! The acquisition decision of [`RxFrontFixed`] as a schedule in clocks (R0-08d part d
//! step 3; ADR 0006 decision 1, ADR 0008 decision 4): which memory word is read in which
//! clock, when each shared block is asked, and when the result is registered.
//!
//! The model's `decide` reads the history and the energy ring in the instant a candidate
//! is accepted. Hardware has `decision_latency` samples for it, on memories that go on
//! being written. This module makes the same decision from memories *as they stand in the
//! clock each word is read*, so a schedule that reads a word after it has been overwritten,
//! uses a read port twice in a clock, or ends after its deadline is an error here, before
//! any RTL. It is not a second definition of the decision: the gate
//! (`tests/g2_rx_decision.rs`) holds its results to [`RxFrontFixed::push`], bit for bit.
//!
//! # The clocks
//!
//! Every count here is from cycle to cycle: a block with a latency of k that has its input
//! in cycle X shows its registered output in cycle X + k. (The RTL headers count the clock
//! edges after the edge that takes the input, which is one less: `ltf_corr`'s 14 is 15
//! here, the CORDIC's 17 is 18, the NCO's 3 is 4.)
//!
//! Cycle Y(n) is the one in which sample n of the DC blocker's output is at the history's
//! write port and at detector A's input; samples are at least R cycles apart. The NCO, the
//! search correlator and the search run D samples behind (ADR 0006 decision 3), so the
//! acceptance at model sample n is a registered strobe in cycle
//!
//! ```text
//! a = Y(n + D) + z_latency + corr_latency + search_latency        (5 + 15 + 3 = 23)
//! ```
//!
//! and the result is applied at model sample n + L: detector A's run counter is cleared in
//! the cycle sample n + L is in its last stage, so the result must be a register's output
//! in cycle Y(n + L) + det_latency − 1, and in any case within that sample's R cycles (the
//! hand-off and the gain's hold start with it). With samples exactly R apart that leaves
//! (L − D)·R − 23 + 3 = 460 cycles at R = 8, D = 4, L = 64.
//!
//! # The schedule, in cycles after the strobe (w = GI − B, F the correlator's fold)
//!
//! | Cycles | What | Reads |
//! |---|---|---|
//! | 0 … w | the grid: the energies at imax … imax + w, oldest first; the last one that keeps 7/8 of the maximum is T1 | ring, one a cycle |
//! | w + 2 | T1 registered (the word, the comparison) | |
//! | c = w + 3 … c + 95 | the final coarse sum: product i from y(T1 − 160 + i) and y(T1 − 144 + i), narrowed to E bits, one product a cycle on one complex multiplier | port 0 and port 1 |
//! | c + 99 | the sum registered (the words 1, narrowed 1, the product 1, the accumulator 1); the CORDIC asked. A latch asking in the same cycle goes first and the decision asks again in the next | |
//! | g + 1 | the sum at the CORDIC's input, g the cycle it is granted (the arbiter's register) | |
//! | g + 19, g + 20 | the coarse angle, the step | |
//! | r = g + 20 … | the rotation of y(T1 + m), m = 0 … 126 + w, through a second NCO into the decision's correlator, one sample a cycle; after each sample that completes a window to be correlated (m = 63 … 62 + w and 127 … 126 + w) the stream waits F − 1 cycles | port 0 |
//! | f = c + 96 … f + 175 | the fine sum on the same multiplier: product i from y(T1 − 28 + i) in cycle f + 2i and y(T1 + 36 + i) in the next | port 1 |
//! | f + 179 | the fine sum registered, the CORDIC asked; its angle 19 cycles after the grant, the resolved CFO 2 later | |
//! | last read + 5 | the last sample out of the NCO, at the correlator (memory 1, NCO 4) | |
//! | + F + 7 | its correlation (`ltf_corr`'s stages: the phase 1, F of products, select, fold, product, two of sums, the accumulator) | |
//! | + 3, + 1, + 2 | the squares, the three sums, the two comparisons | |
//! | + 1 | **the result registered** | |
//!
//! Without a wait at the CORDIC that is 8·w + 268 cycles at F = 4 (364 at w = 12, 396 at
//! w = 16); the fine sum's branch ends at w + 300 and is the longer one only below w = 5.
//!
//! The stage counts of blocks that exist are theirs (`cordic`, `nco`, the memory; the
//! correlator's are `ltf_corr`'s with F in place of 8). Those of blocks not yet written
//! (the search's three, the multiplier's, the squares and comparisons) are this
//! schedule's allowance for the RTL of steps 2 and 3, as ADR 0006's were: none has been
//! through synthesis, and a stage more in any of them is a cycle more here.

use crate::rx_front::{
    backoff_samples, RxConfig, COARSE_END_BEFORE_T1, COARSE_LEN, FINE_GUARD, FINE_LEN, GI2_LEN,
    GI_LEN, HISTORY, LATCH_LAG, LTF_SYMBOL_LEN, MAX_DECISION_LATENCY, PAIR_LEN, STF_PERIOD,
};
use crate::rx_front_fixed::{
    confirmed, cordic_table, cordic_with, iabs2, iadd, iconj, imul, ltf_taps, narrow_iq,
    nco_mix_with, nco_table, phase_per_sample, Accepted, DecisionSums, FixedAcquisition,
    FixedRxConfig, RxFrontFixed, IQ, Y_BITS,
};
use std::collections::BTreeSet;

/// A synchronous memory's read: the word is registered the cycle after its address.
const MEM: usize = 1;
/// From a registered pair of memory words to the registered sum: the narrowing, the
/// product, the accumulator, a register after each (an allowance: `nco`'s product alone,
/// from flip-flops, routed with 0.9 ns to spare at 160 MHz on IHP 130 nm).
const MAC: usize = 3;
/// The register between the CORDIC's arbiter and its input.
const ARB: usize = 1;
/// The CORDIC of `hw/rtl/cordic.sv`: the angle 18 cycles after the input's cycle.
const CORDIC: usize = 18;
/// The NCO of `hw/rtl/nco.sv`: the rotated sample 4 cycles after its input's cycle.
const NCO: usize = 4;
/// The decision's correlator beyond its F cycles of products, as `ltf_corr` is built: the
/// phase register, select, fold, product, two of sums, the accumulator's last clock.
const CORR: usize = 7;
/// Squares of a correlation pair (3), the running sums (1), the comparisons with the
/// energy (2).
const TAIL: usize = 3 + 1 + 2;

/// What the schedule is laid out against: the clock, the path delays of the blocks around
/// the decision, and the memories' depths. The defaults are the front as built and decided
/// on 2026-11-22: 160 MHz at 20 MS/s, D = 4 (the latch's contract D·R ≥ 24 with the
/// arbiter's register, and a sample to spare), a fold of 4 (ADR 0008 decision 4).
/// Latencies are from cycle to cycle (see the module's text).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecisionTiming {
    /// R: the least number of clocks between two samples.
    pub clocks_per_sample: usize,
    /// D: samples the NCO path, the search correlator and the search run behind
    /// (`rx_latch`'s `D_SAMPLES`).
    pub delay_samples: usize,
    /// F: clocks the decision's correlator takes for one 64-tap correlation (128 real
    /// products on 128 / F multipliers).
    pub fold: usize,
    /// Cycles from Y(k + D) to the rotated sample k at the search correlator's input
    /// (`rx_latch` and `nco`, as `tb_latch` requires: 5).
    pub z_latency: usize,
    /// The search correlator's latency (`ltf_corr`: 15, its header's 14 edges).
    pub corr_latency: usize,
    /// Cycles from a correlator output to the registered acceptance: the square, the
    /// energy's running sum, the comparison with the maximum (an allowance for step 2: 3).
    pub search_latency: usize,
    /// Cycles from a sample at detector A's input to its event (`det_a`: 4). The run
    /// counter's clear is due one cycle before; a latch asks the CORDIC in the event's own
    /// cycle (`rx_latch`'s request is combinational on it).
    pub det_latency: usize,
    /// Words of the history memory (the model's `HISTORY`: 512).
    pub history_depth: usize,
    /// Words of the energy ring (64 + GI + 1 = 81).
    pub ring_depth: usize,
}

impl Default for DecisionTiming {
    fn default() -> Self {
        Self {
            clocks_per_sample: 8,
            delay_samples: 4,
            fold: 4,
            z_latency: 5,
            corr_latency: 15,
            search_latency: 3,
            det_latency: 4,
            history_depth: HISTORY,
            ring_depth: LTF_SYMBOL_LEN + GI_LEN + 1,
        }
    }
}

/// One decision made on the schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecisionRun {
    /// The model sample at which the candidate was accepted.
    pub accepted_at: usize,
    /// The decision, as [`RxFrontFixed::push`] returns it `decision_latency` samples
    /// later (`decided_at` is that sample); `None` for a rejected candidate.
    pub result: Option<FixedAcquisition>,
    /// The sums it was taken from (`SampleTrace::sums` of the model at `accepted_at`).
    pub sums: Option<DecisionSums>,
    /// The cycle of the acceptance strobe.
    pub strobe: usize,
    /// Cycles after the strobe at which the result is a register's output.
    pub done: usize,
    /// Cycles after the strobe at which it is due.
    pub budget: usize,
    /// Cycles the coarse sum waited for the CORDIC behind a latch.
    pub coarse_wait: usize,
    /// Cycles the fine sum waited for it.
    pub fine_wait: usize,
    /// Words read on the history's port 0 and port 1.
    pub reads: (usize, usize),
    /// Words of the history to spare at the read that came closest to an overwritten one.
    pub history_spare: usize,
    /// The same of the energy ring.
    pub ring_spare: usize,
}

/// A memory of the decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Store {
    /// The history of the DC blocker's output.
    History,
    /// The ring of the search's energies.
    Ring,
}

/// Why a schedule cannot be built in hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    /// A word is read after it was overwritten, or in the cycle that overwrites it.
    Stale {
        /// The memory.
        store: Store,
        /// The sample or position asked for.
        index: usize,
        /// The cycle of the read.
        clock: usize,
    },
    /// A word is read before it is written, or in the cycle that writes it.
    Unwritten {
        /// The memory.
        store: Store,
        /// The sample or position asked for.
        index: usize,
        /// The cycle of the read.
        clock: usize,
    },
    /// A read port is used twice in a cycle.
    PortBusy {
        /// The memory.
        store: Store,
        /// The port (0 or 1; the ring has one).
        port: u8,
        /// The cycle.
        clock: usize,
    },
    /// A result is not registered by the cycle it is due.
    Late,
}

/// Windows of the rotated stream that are correlated: those ending at m = 63 … 62 + w
/// (the first long symbol at each of the w offsets) and 64 later.
fn correlated(m: usize, w: usize) -> bool {
    let k = m % LTF_SYMBOL_LEN;
    m >= LTF_SYMBOL_LEN - 1 && (k == LTF_SYMBOL_LEN - 1 || k + 1 < w) && m < PAIR_LEN + w
}

/// The cycle, after the first, in which sample m of the rotation is read.
fn rotation_read(m: usize, w: usize, fold: usize) -> usize {
    m + (fold - 1) * (0..m).filter(|&k| correlated(k, w)).count()
}

/// The cycle the first rotated sample is read in, the coarse sum granted the CORDIC in
/// cycle `granted`: the cycle the step is registered (the address does not wait for it).
fn rotation_start(granted: usize) -> usize {
    granted + ARB + CORDIC + 1
}

/// The cycle the result is registered in, from the last read of the rotation and the
/// cycle the fine sum was granted the CORDIC.
fn result_at(last_rotation_read: usize, fold: usize, fine_granted: usize) -> usize {
    let rotation = last_rotation_read + MEM + NCO + fold + CORR + TAIL;
    let fine = fine_granted + ARB + CORDIC + 2;
    rotation.max(fine) + 1
}

/// Cycles from the acceptance strobe to the registered result when nothing else asks for
/// the CORDIC, at w = GI − B: the schedule's table added up, 8·w + 268 at a fold of 4 for
/// w ≥ 5.
pub fn schedule_len(w: usize, t: &DecisionTiming) -> usize {
    let fold = t.fold.max(1);
    let coarse = w + MEM + 1 + 1;
    let coarse_ask = coarse + COARSE_LEN - 1 + MEM + MAC;
    let fine = coarse + COARSE_LEN;
    let fine_ask = fine + 2 * FINE_LEN - 1 + MEM + MAC;
    let last = rotation_start(coarse_ask) + rotation_read(PAIR_LEN + w - 2, w, fold);
    result_at(last, fold, fine_ask)
}

/// A memory written once per index in known cycles, read by absolute index on its ports.
struct Memory<'a, T> {
    store: Store,
    depth: usize,
    /// Cycle in which index k is written (increasing).
    written: &'a [usize],
    data: &'a [T],
    /// The least number of spare words seen.
    spare: usize,
    /// The (port, cycle) of every read.
    used: BTreeSet<(u8, usize)>,
}

impl<'a, T: Copy> Memory<'a, T> {
    fn new(store: Store, depth: usize, written: &'a [usize], data: &'a [T]) -> Self {
        Self {
            store,
            depth,
            written,
            data,
            spare: depth,
            used: BTreeSet::new(),
        }
    }

    fn read(&mut self, port: u8, index: usize, clock: usize) -> Result<T, ScheduleError> {
        let store = self.store;
        if !self.used.insert((port, clock)) {
            return Err(ScheduleError::PortBusy { store, port, clock });
        }
        // Words whose write is over before this cycle, and those written by its end.
        let before = self.written.partition_point(|&c| c < clock);
        let by_end = self.written.partition_point(|&c| c <= clock);
        let Some(&v) = self.data.get(index).filter(|_| index < before) else {
            return Err(ScheduleError::Unwritten {
                store,
                index,
                clock,
            });
        };
        // The newest word written by the end of this cycle is by_end − 1: the word read
        // must not share its address with that one or any before it down to its own.
        let age = by_end - 1 - index;
        if age >= self.depth {
            return Err(ScheduleError::Stale {
                store,
                index,
                clock,
            });
        }
        self.spare = self.spare.min(self.depth - 1 - age);
        Ok(v)
    }

    fn reads(&self, port: u8) -> usize {
        self.used.iter().filter(|u| u.0 == port).count()
    }

    /// The cycle of the latest read on `port` so far (0 before any).
    fn last(&self, port: u8) -> usize {
        let on_port = self.used.iter().filter(|u| u.0 == port);
        on_port.map(|u| u.1).max().unwrap_or(0)
    }
}

/// Whether the energy `e` at position `p` may be the grid: it keeps 7/8 of the maximum
/// and lies before the search's deadline.
fn keeps(e: i128, p: usize, acc: &Accepted) -> bool {
    p < acc.deadline && e >= acc.emax - (acc.emax >> 3)
}

/// The first cycle from `ask` on in which no latch asks for the CORDIC, if it comes
/// before `limit`.
fn granted(
    ask: usize,
    limit: usize,
    latch: &dyn Fn(usize) -> bool,
) -> Result<usize, ScheduleError> {
    (ask..limit).find(|&c| !latch(c)).ok_or(ScheduleError::Late)
}

/// Every decision of [`RxFrontFixed`] on the ADC samples `x`, made on the schedule of this
/// module: the candidates accepted early enough for their result to fall inside `x`, in
/// order.
///
/// `idle(n)` is the number of idle clocks before sample n beyond the R − 1 of
/// `t.clocks_per_sample`. `latch(clock)` says whether a latch asks for the CORDIC in that
/// cycle besides those of the stream's own detection events (which ask
/// `t.det_latency` cycles after their sample: the event's own cycle).
///
/// # Errors
/// The first thing that cannot be built: a word read when the memory no longer or not yet
/// holds it, a port used twice in a cycle, a result later than its deadline.
pub fn run_schedule(
    x: &[(i16, i16)],
    cfg: &RxConfig,
    fx: &FixedRxConfig,
    t: &DecisionTiming,
    idle: &dyn Fn(usize) -> usize,
    latch: &dyn Fn(usize) -> bool,
) -> Result<Vec<DecisionRun>, ScheduleError> {
    let b = backoff_samples(cfg);
    let w = GI_LEN.saturating_sub(b);
    let latency = cfg.decision_latency.min(MAX_DECISION_LATENCY);
    let fold = t.fold.max(1);
    let period = t.clocks_per_sample.max(1);
    let to_strobe = t.z_latency + t.corr_latency + t.search_latency;
    if w == 0 || !fx.is_valid() {
        return Ok(Vec::new());
    }

    // The model's own run: what each memory is written with, and what the search hands on.
    let mut rx = RxFrontFixed::new(cfg.clone(), *fx);
    let mut y = Vec::with_capacity(x.len());
    let mut energy = Vec::new();
    let mut events = Vec::new();
    let mut accepted: Vec<(usize, Accepted)> = Vec::new();
    for (n, s) in x.iter().enumerate() {
        let trace = rx.push_traced(*s).1;
        y.push(trace.y);
        energy.extend(trace.energy);
        if trace.event && n + 1 >= LATCH_LAG + STF_PERIOD + COARSE_LEN {
            events.push(n);
        }
        accepted.extend(trace.accepted.map(|a| (n, a)));
    }

    // Cycle Y(n) of every sample, on past the stream's end as far as a decision looks.
    let extra = latency + t.delay_samples + 2 * PAIR_LEN;
    let mut clock = 0;
    let y_clock: Vec<usize> = (0..x.len() + extra)
        .map(|n| {
            clock += period + idle(n);
            clock
        })
        .collect();
    // Energy j is complete at model sample j + 126 + w and registered a cycle before that
    // sample's strobe (the comparison with the maximum is the search's last stage).
    let energy_at = |j: usize| j + PAIR_LEN - 2 + w;
    let ring_clock: Vec<usize> = (0..)
        .map_while(|j| y_clock.get(energy_at(j) + t.delay_samples))
        .map(|c| c + to_strobe - 1)
        .collect();
    let own: BTreeSet<usize> = events.iter().map(|&e| y_clock[e] + t.det_latency).collect();
    let asked = |c: usize| own.contains(&c) || latch(c);

    let taps = ltf_taps(fx);
    let tap_energy: i128 = taps.iter().map(|v| iabs2(*v)).sum();
    let cordic = cordic_table(fx);
    let nco = nco_table(fx);
    let est = |v: IQ| narrow_iq(v, Y_BITS, fx.est_bits);

    let mut runs = Vec::new();
    for (n, acc) in accepted {
        let due = n + latency;
        if due >= x.len() {
            continue;
        }
        let a = y_clock[n + t.delay_samples] + to_strobe;
        // The run counter's clear, and in any case inside the sample's own period.
        let deadline = y_clock[due] + (t.det_latency.max(1) - 1).min(period - 1);
        let budget = deadline.checked_sub(a).ok_or(ScheduleError::Late)?;
        if schedule_len(w, t) > budget {
            return Err(ScheduleError::Late);
        }
        let mut ring = Memory::new(Store::Ring, t.ring_depth, &ring_clock, &energy);
        let mut history = Memory::new(Store::History, t.history_depth, &y_clock, &y);

        // The grid: the latest start within GI − B of the maximum keeping 7/8 of it. The
        // oldest energy is read first: it is the first to be overwritten.
        let mut sym = None;
        for k in 0..=w {
            let e = ring.read(0, acc.imax + k, a + k)?;
            if keeps(e, acc.imax + k, &acc) {
                sym = Some(acc.imax + k);
            }
        }
        // T1 registered: the last word, its comparison.
        let grid = ring.last(0) + MEM + 1;

        let mut run = DecisionRun {
            accepted_at: n,
            result: None,
            sums: None,
            strobe: a,
            done: schedule_len(w, t),
            budget,
            coarse_wait: 0,
            fine_wait: 0,
            reads: (0, 0),
            history_spare: t.history_depth,
            ring_spare: ring.spare,
        };
        let kc = sym.and_then(|s| s.checked_sub(COARSE_END_BEFORE_T1 + COARSE_LEN));
        let (Some(sym), Some(kc)) = (sym, kc.filter(|&k| k + HISTORY > n)) else {
            runs.push(run);
            continue;
        };

        // The final coarse sum, both ports, one product a cycle.
        let mut cc = (0, 0);
        for i in 0..COARSE_LEN {
            let v0 = history.read(0, kc + i, grid + 1 + i)?;
            let v1 = history.read(1, kc + STF_PERIOD + i, grid + 1 + i)?;
            cc = iadd(cc, imul(est(v0), iconj(est(v1))));
        }
        let coarse_end = history.last(0).max(history.last(1));
        let coarse_ask = coarse_end + MEM + MAC;
        let coarse_granted = granted(coarse_ask, deadline, &asked)?;
        let angle_c = cordic_with(cc.0, cc.1, fx, &cordic);
        let step = phase_per_sample(angle_c, STF_PERIOD as u32, fx);

        // The rotation into the decision's correlator, port 0. The model rotates
        // PAIR_LEN + w samples and no sum reads the last one: the newest window
        // correlated ends at m = 126 + w.
        let first = rotation_start(coarse_granted);
        let mut rot = Vec::with_capacity(PAIR_LEN + w);
        for m in 0..PAIR_LEN + w - 1 {
            let v = history.read(0, sym + m, first + rotation_read(m, w, fold))?;
            let ph = (i128::from(step) * m as i128).rem_euclid(1i128 << fx.nco_phase_bits) as i64;
            rot.push(nco_mix_with(v, ph, fx, &nco));
        }
        let rotation_end = history.last(0);
        let xc = |o: usize| {
            (0..LTF_SYMBOL_LEN).fold((0, 0), |s, k| iadd(s, imul(rot[o + k], iconj(taps[k]))))
        };
        let (conf, coh) = (0..w).fold((0i128, 0i128), |(nc, ch), o| {
            let (u, v) = (xc(o), xc(o + LTF_SYMBOL_LEN));
            (nc + iabs2(u) + iabs2(v), ch + iabs2(iadd(u, v)))
        });
        let rx_energy: i128 = rot[..PAIR_LEN].iter().map(|v| iabs2(*v)).sum();

        // The fine sum on port 1 behind the coarse one, two reads a product.
        let fine_start = (sym + FINE_GUARD).checked_sub(GI2_LEN);
        let mut cf = (0, 0);
        if let Some(fs) = fine_start {
            for i in 0..FINE_LEN {
                let v0 = history.read(1, fs + i, coarse_end + 1 + 2 * i)?;
                let v1 = history.read(1, fs + LTF_SYMBOL_LEN + i, coarse_end + 2 + 2 * i)?;
                cf = iadd(cf, imul(est(v0), iconj(est(v1))));
            }
        }
        // (With a coarse window in the stream there is a fine one: T1 ≥ 160.)
        let fine_ask = history.last(1) + MEM + MAC;
        let fine_granted = granted(fine_ask, deadline, &asked)?;
        let coarse = -step;
        let raw = -phase_per_sample(
            cordic_with(cf.0, cf.1, fx, &cordic),
            LTF_SYMBOL_LEN as u32,
            fx,
        );
        let span = 1i64 << (fx.nco_phase_bits - 6);
        let half = span >> 1;
        let d = (raw - coarse + half).rem_euclid(span) - half;

        run.coarse_wait = coarse_granted - coarse_ask;
        run.fine_wait = fine_granted - fine_ask;
        run.done = result_at(rotation_end, fold, fine_granted) - a;
        run.history_spare = history.spare;
        run.reads = (history.reads(0), history.reads(1));
        if run.done > budget {
            return Err(ScheduleError::Late);
        }
        let mut sums = DecisionSums {
            coarse: cc,
            confirmation: conf,
            coherence: coh,
            energy: rx_energy,
            fine: None,
        };
        let confirmed = confirmed(&sums, tap_energy);
        sums.fine = fine_start.filter(|_| confirmed).map(|_| cf);
        run.sums = Some(sums);
        run.result = match (confirmed, fine_start, sym.checked_sub(b)) {
            (true, Some(_), Some(fft_start)) => Some(FixedAcquisition {
                decided_at: due,
                detected_at: acc.opened_by,
                ltf_start: sym - GI2_LEN,
                fft_start,
                cfo_phase: coarse + (((64 - fx.coarse_weight_64) * d + 32) >> 6),
                coarse_phase: coarse,
                fine_phase: coarse + d,
            }),
            _ => None,
        };
        runs.push(run);
    }
    Ok(runs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_grid_keeps_seven_eighths_of_the_maximum_before_the_deadline() {
        let acc = Accepted {
            imax: 100,
            emax: 800,
            deadline: 110,
            opened_by: 0,
        };
        // 800 − 800 / 8 = 700 is kept, 699 is not.
        assert!(keeps(700, 105, &acc) && keeps(800, 100, &acc));
        assert!(!keeps(699, 105, &acc));
        assert!(keeps(700, 109, &acc) && !keeps(800, 110, &acc));
        // The shift floors: 7 − 0 = 7.
        let small = Accepted { emax: 7, ..acc };
        assert!(keeps(7, 100, &small) && !keeps(6, 100, &small));
    }

    #[test]
    fn a_port_gives_one_word_a_cycle() {
        let written: Vec<usize> = (1..=40).collect();
        let data = vec![(0i64, 0i64); 40];
        let mut m = Memory::new(Store::History, 16, &written, &data);
        assert_eq!(m.read(0, 30, 41), Ok((0, 0)));
        assert_eq!(m.read(1, 30, 41), Ok((0, 0)));
        assert_eq!(m.read(0, 31, 42), Ok((0, 0)));
        assert_eq!(
            m.read(0, 32, 41),
            Err(ScheduleError::PortBusy {
                store: Store::History,
                port: 0,
                clock: 41
            })
        );
        assert_eq!((m.reads(0), m.reads(1)), (2, 1));
        assert_eq!((m.last(0), m.last(1), m.last(2)), (42, 41, 0));
    }

    #[test]
    fn a_memory_holds_its_newest_words_and_not_the_one_being_written() {
        // Index k is written in cycle 10·(k + 1).
        let written: Vec<usize> = (1..=40).map(|k| 10 * k).collect();
        let data: Vec<usize> = (0..40).collect();
        let mut m = Memory::new(Store::Ring, 8, &written, &data);
        let store = Store::Ring;
        // In cycle 205 the newest is index 19 (cycle 200): 12 … 19 are held.
        assert_eq!(m.read(0, 19, 205), Ok(19));
        assert_eq!(m.read(0, 12, 206), Ok(12));
        assert_eq!(m.spare, 0);
        let stale = |index, clock| {
            Err(ScheduleError::Stale {
                store,
                index,
                clock,
            })
        };
        assert_eq!(m.read(0, 11, 207), stale(11, 207));
        // Index 20 is written in cycle 210 into the word of index 12: neither can be read
        // in that cycle, both the cycle before and after as the memory then stands.
        assert_eq!(m.read(0, 12, 209), Ok(12));
        assert_eq!(m.read(0, 12, 210), stale(12, 210));
        assert_eq!(
            m.read(1, 20, 210),
            Err(ScheduleError::Unwritten {
                store,
                index: 20,
                clock: 210
            })
        );
        assert_eq!(m.read(0, 20, 211), Ok(20));
        assert_eq!(m.read(2, 13, 210), Ok(13));
    }
}
