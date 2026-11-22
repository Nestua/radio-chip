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
//! Cycle Y(n) is the one in which sample n of the DC blocker's output is at the history's
//! write port and at detector A's input; samples are at least R cycles apart. The NCO, the
//! search correlator and the search run D samples behind (ADR 0006 decision 3), so the
//! acceptance at model sample n is a registered strobe in cycle
//!
//! ```text
//! a = Y(n + D) + z_latency + corr_latency + search_latency        (5 + 14 + 3 = 22)
//! ```
//!
//! and the result is applied at model sample n + L: detector A's run counter is cleared in
//! the cycle sample n + L is in its last stage, so the result must be a register's output
//! in cycle Y(n + L) + det_latency − 1. With samples exactly R apart that leaves
//! (L − D)·R − 22 + 3 = 461 cycles at R = 8, D = 4, L = 64.
//!
//! # The schedule, in cycles after the strobe (w = GI − B, F the correlator's fold)
//!
//! | Cycles | What | Reads |
//! |---|---|---|
//! | 0 … w | the grid: the energies at imax … imax + w, oldest first; the last one that keeps 7/8 of the maximum and lies before the deadline is T1 | ring, one a cycle |
//! | w + 2 | T1 registered | |
//! | c = w + 3 … c + 95 | the final coarse sum: product i from y(T1 − 160 + i) and y(T1 − 144 + i), narrowed to E bits, one product a cycle on one complex multiplier | port 0 and port 1 |
//! | c + 98 | the sum registered; the CORDIC asked (a latch asking in the same cycle goes first, the decision asks again in the next) | |
//! | g + 17, g + 18 | the coarse angle, the step (g the cycle the CORDIC took the sum) | |
//! | r = g + 19 … | the rotation of y(T1 + m), m = 0 … 126 + w, into the decision's correlator, one sample a cycle; after each sample that completes a window to be correlated (m = 63 … 62 + w and 127 … 126 + w) the stream waits F − 1 cycles | port 0 |
//! | f = c + 96 … f + 175 | the fine sum on the same multiplier: product i from y(T1 − 28 + i) in cycle f + 2i and y(T1 + 36 + i) in the next | port 1 |
//! | f + 178 | the fine sum registered, the CORDIC asked; its angle 17 cycles after it is taken, the resolved CFO 2 later | |
//! | last read + 4 | the last sample in the correlator's window (memory 1, NCO 3) | |
//! | + F + 5 | its correlation | |
//! | + 3, + 1, + 2 | the squares, the three sums, the two comparisons | |
//! | + 1 | **the result registered** | |
//!
//! Without a wait at the CORDIC that is 8·w + 263 cycles at F = 4 (359 at w = 12, 391 at
//! w = 16); the fine sum's branch ends at w + 296 and is the longer one only below w = 5.
//!
//! The stage counts inside a branch (two cycles from a memory word to a registered product,
//! one for the accumulator, three for the squares) are this schedule's allowance for the
//! RTL of step 3, as ADR 0006's were: the RTL may be faster, and must not be slower.

use crate::rx_front::{
    backoff_samples, RxConfig, COARSE_END_BEFORE_T1, COARSE_LEN, FINE_GUARD, FINE_LEN, GI2_LEN,
    GI_LEN, HISTORY, LATCH_LAG, LTF_SYMBOL_LEN, MAX_DECISION_LATENCY, PAIR_LEN, STF_PERIOD,
};
use crate::rx_front_fixed::{
    cordic_table, cordic_with, iabs2, iadd, iconj, imul, ltf_taps, narrow_iq, nco_mix_with,
    nco_table, phase_per_sample, Accepted, FixedAcquisition, FixedRxConfig, RxFrontFixed, IQ,
    Y_BITS,
};
use std::collections::BTreeSet;

/// A synchronous memory's read: the word is registered the cycle after its address.
const MEM: usize = 1;
/// From a registered pair of memory words to their registered product (narrowing, the
/// multiplier), and one cycle more to the accumulator.
const MAC: usize = 2;
/// The CORDIC of `hw/rtl/cordic.sv`: the angle 17 cycles after the cycle that takes the
/// input.
const CORDIC: usize = 17;
/// The NCO of `hw/rtl/nco.sv`: the rotated sample 3 cycles after its input.
const NCO: usize = 3;
/// The decision's correlator beyond its F cycles of products: select, fold, product, two
/// of sums (the search correlator's stages without the narrowing).
const CORR: usize = 5;
/// Squares of a correlation pair, the running sums, the comparisons with the energy.
const TAIL: usize = 3 + 1 + 2;

/// What the schedule is laid out against: the clock, the path delays of the blocks around
/// the decision, and the memories' depths. The defaults are the front as built and decided
/// on 2026-11-22: 160 MHz at 20 MS/s, D = 4 (the latch's contract with a register for the
/// CORDIC's arbiter), a fold of 4 (ADR 0008 decision 4).
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
    /// Cycles from Y(k + D) to the rotated sample k at the search correlator (`rx_latch`
    /// and `nco`: 5).
    pub z_latency: usize,
    /// The search correlator's latency (`ltf_corr`: 14).
    pub corr_latency: usize,
    /// Cycles from a correlator output to the registered acceptance: the square, the
    /// energy's running sum, the comparison with the maximum (an allowance for step 2: 3).
    pub search_latency: usize,
    /// Cycles from a sample at detector A's input to its event (`det_a`: 4). The run
    /// counter's clear is due one cycle before; a latch asks the CORDIC one cycle after.
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
            corr_latency: 14,
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
    /// Words of the history to spare at the read that came closest to an overwritten one.
    pub history_spare: usize,
    /// The same of the energy ring.
    pub ring_spare: usize,
}

/// Why a schedule cannot be built in hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    /// A word is read after it was overwritten, or in the cycle that overwrites it.
    Stale {
        /// Whether the memory is the energy ring (otherwise the history).
        ring: bool,
        /// The sample or position asked for.
        index: usize,
        /// The cycle of the read.
        clock: usize,
    },
    /// A word is read before it is written.
    Unwritten {
        /// Whether the memory is the energy ring.
        ring: bool,
        /// The sample or position asked for.
        index: usize,
        /// The cycle of the read.
        clock: usize,
    },
    /// A read port of the history is used twice in a cycle.
    PortBusy {
        /// The port (0 or 1).
        port: u8,
        /// The cycle.
        clock: usize,
    },
    /// A result is not registered by the cycle it is due.
    Late,
}

/// Where the branches of one decision stand, in cycles after the strobe.
#[derive(Debug, Clone, Copy)]
struct Plan {
    /// First read of the coarse sum.
    coarse: usize,
    /// The coarse sum asks the CORDIC.
    coarse_ask: usize,
    /// First read of the fine sum.
    fine: usize,
    /// The fine sum asks the CORDIC.
    fine_ask: usize,
}

fn plan(w: usize) -> Plan {
    let coarse = w + 1 + MEM + 1;
    let fine = coarse + COARSE_LEN;
    Plan {
        coarse,
        coarse_ask: coarse + COARSE_LEN - 1 + MEM + MAC,
        fine,
        fine_ask: fine + 2 * FINE_LEN - 1 + MEM + MAC,
    }
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

/// The cycle after the strobe at which the result is registered, the coarse sum taken by
/// the CORDIC in cycle `coarse_taken` and the fine sum in `fine_taken`.
fn end(w: usize, fold: usize, coarse_taken: usize, fine_taken: usize) -> usize {
    let first = coarse_taken + CORDIC + 1 + 1;
    let last = first + rotation_read(PAIR_LEN + w - 2, w, fold);
    let rotation = last + MEM + NCO + fold + CORR + TAIL;
    let fine = fine_taken + CORDIC + 2;
    rotation.max(fine) + 1
}

/// Cycles from the acceptance strobe to the registered result when nothing else asks for
/// the CORDIC, at w = GI − B: 8·w + 263 at a fold of 4 for w ≥ 5.
pub fn schedule_len(w: usize, t: &DecisionTiming) -> usize {
    let p = plan(w);
    end(w, t.fold.max(1), p.coarse_ask, p.fine_ask)
}

/// A memory written once per index in known cycles, read by absolute index.
struct Memory<'a, T> {
    ring: bool,
    depth: usize,
    /// Cycle in which index k is written (increasing).
    written: &'a [usize],
    data: &'a [T],
    /// The least number of spare words seen.
    spare: usize,
}

impl<T: Copy> Memory<'_, T> {
    fn read(&mut self, index: usize, clock: usize) -> Result<T, ScheduleError> {
        let (ring, err) = (self.ring, (index, clock));
        // Words whose write is over before this cycle, and those written by its end.
        let before = self.written.partition_point(|&c| c < clock);
        let by_end = self.written.partition_point(|&c| c <= clock);
        let Some(&v) = self.data.get(index).filter(|_| index < before) else {
            return Err(ScheduleError::Unwritten {
                ring,
                index: err.0,
                clock: err.1,
            });
        };
        // The newest word written by the end of this cycle is by_end − 1: the word read
        // must not share its address with that one or any before it down to its own.
        let age = by_end - 1 - index;
        if age >= self.depth {
            return Err(ScheduleError::Stale { ring, index, clock });
        }
        self.spare = self.spare.min(self.depth - 1 - age);
        Ok(v)
    }
}

/// The history's two read ports, each good for one word a cycle.
struct Ports<'a> {
    history: Memory<'a, IQ>,
    used: BTreeSet<(u8, usize)>,
}

impl Ports<'_> {
    fn read(&mut self, port: u8, index: usize, clock: usize) -> Result<IQ, ScheduleError> {
        if !self.used.insert((port, clock)) {
            return Err(ScheduleError::PortBusy { port, clock });
        }
        self.history.read(index, clock)
    }
}

/// The first cycle from `ask` on in which no latch asks for the CORDIC, if it comes
/// before `limit`.
fn taken(ask: usize, limit: usize, latch: &dyn Fn(usize) -> bool) -> Result<usize, ScheduleError> {
    (ask..limit).find(|&c| !latch(c)).ok_or(ScheduleError::Late)
}

/// Every decision of [`RxFrontFixed`] on the ADC samples `x`, made on the schedule of this
/// module: the candidates accepted early enough for their result to fall inside `x`, in
/// order.
///
/// `idle(n)` is the number of idle clocks before sample n beyond the R − 1 of
/// `t.clocks_per_sample`. `latch(clock)` says whether a latch asks for the CORDIC in that
/// cycle besides those of the stream's own detection events (which ask
/// `t.det_latency` + 1 cycles after their sample).
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
            clock += t.clocks_per_sample.max(1) + idle(n);
            clock
        })
        .collect();
    // Energy j is complete at model sample j + 126 + w, a cycle before that sample's strobe.
    let energy_at = |j: usize| j + PAIR_LEN - 2 + w;
    let ring_clock: Vec<usize> = (0..)
        .map_while(|j| y_clock.get(energy_at(j) + t.delay_samples))
        .map(|c| c + to_strobe - 1)
        .collect();
    let own: BTreeSet<usize> = events
        .iter()
        .map(|&e| y_clock[e] + t.det_latency + 1)
        .collect();
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
        let deadline = y_clock[due] + t.det_latency.max(1) - 1;
        let budget = deadline.checked_sub(a).ok_or(ScheduleError::Late)?;
        let p = plan(w);
        let mut ring = Memory {
            ring: true,
            depth: t.ring_depth,
            written: &ring_clock,
            data: &energy,
            spare: t.ring_depth,
        };
        let mut ports = Ports {
            history: Memory {
                ring: false,
                depth: t.history_depth,
                written: &y_clock,
                data: &y,
                spare: t.history_depth,
            },
            used: BTreeSet::new(),
        };

        // The grid: the latest start within GI − B of the maximum keeping 7/8 of it. The
        // oldest energy is read first: it is the first to be overwritten.
        let keep = acc.emax - (acc.emax >> 3);
        let mut sym = None;
        for k in 0..=w {
            let e = ring.read(acc.imax + k, a + k)?;
            if acc.imax + k < acc.deadline && e >= keep {
                sym = Some(acc.imax + k);
            }
        }

        let mut run = DecisionRun {
            accepted_at: n,
            result: None,
            strobe: a,
            done: schedule_len(w, t),
            budget,
            coarse_wait: 0,
            fine_wait: 0,
            history_spare: t.history_depth,
            ring_spare: ring.spare,
        };
        let kc = sym.and_then(|s| s.checked_sub(COARSE_END_BEFORE_T1 + COARSE_LEN));
        let (Some(sym), Some(kc)) = (sym, kc.filter(|&k| k + HISTORY > n)) else {
            if run.done > budget {
                return Err(ScheduleError::Late);
            }
            runs.push(run);
            continue;
        };

        // The final coarse sum, both ports, one product a cycle.
        let mut cc = (0, 0);
        for i in 0..COARSE_LEN {
            let c = a + p.coarse + i;
            let v0 = ports.read(0, kc + i, c)?;
            let v1 = ports.read(1, kc + STF_PERIOD + i, c)?;
            cc = iadd(cc, imul(est(v0), iconj(est(v1))));
        }
        let coarse_taken = taken(a + p.coarse_ask, deadline, &asked)?;
        let angle_c = cordic_with(cc.0, cc.1, fx, &cordic);
        let step = phase_per_sample(angle_c, STF_PERIOD as u32, fx);

        // The rotation into the decision's correlator, port 0.
        let first = coarse_taken + CORDIC + 1 + 1;
        // The model rotates PAIR_LEN + w samples and no sum reads the last one: the newest
        // window correlated ends at m = 126 + w.
        let mut rot = Vec::with_capacity(PAIR_LEN + w);
        for m in 0..PAIR_LEN + w - 1 {
            let v = ports.read(0, sym + m, first + rotation_read(m, w, fold))?;
            let ph = (i128::from(step) * m as i128).rem_euclid(1i128 << fx.nco_phase_bits) as i64;
            rot.push(nco_mix_with(v, ph, fx, &nco));
        }
        let xc = |o: usize| {
            (0..LTF_SYMBOL_LEN).fold((0, 0), |s, k| iadd(s, imul(rot[o + k], iconj(taps[k]))))
        };
        let (conf, coh) = (0..w).fold((0i128, 0i128), |(nc, ch), o| {
            let (u, v) = (xc(o), xc(o + LTF_SYMBOL_LEN));
            (nc + iabs2(u) + iabs2(v), ch + iabs2(iadd(u, v)))
        });
        let rx_energy: i128 = rot[..PAIR_LEN].iter().map(|v| iabs2(*v)).sum();
        let rhs = tap_energy * rx_energy;
        let confirmed = rx_energy > 0 && 2 * conf >= rhs && 2 * coh >= rhs;

        // The fine sum on port 1 behind the coarse one, two reads a product.
        let fine_start = (sym + FINE_GUARD).checked_sub(GI2_LEN);
        let mut cf = (0, 0);
        if let Some(fs) = fine_start {
            for i in 0..FINE_LEN {
                let c = a + p.fine + 2 * i;
                let v0 = ports.read(1, fs + i, c)?;
                let v1 = ports.read(1, fs + LTF_SYMBOL_LEN + i, c + 1)?;
                cf = iadd(cf, imul(est(v0), iconj(est(v1))));
            }
        }
        let fine_taken = taken(a + p.fine_ask, deadline, &asked)?;
        let coarse = -step;
        let raw = -phase_per_sample(
            cordic_with(cf.0, cf.1, fx, &cordic),
            LTF_SYMBOL_LEN as u32,
            fx,
        );
        let span = 1i64 << (fx.nco_phase_bits - 6);
        let half = span >> 1;
        let d = (raw - coarse + half).rem_euclid(span) - half;

        run.coarse_wait = coarse_taken - (a + p.coarse_ask);
        run.fine_wait = fine_taken - (a + p.fine_ask);
        run.done = end(w, fold, coarse_taken, fine_taken) - a;
        run.history_spare = ports.history.spare;
        if run.done > budget {
            return Err(ScheduleError::Late);
        }
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
