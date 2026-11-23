//! The hand-off of [`RxHandoffFixed`] as a schedule in clocks (R0-08d part d step 4b; ADR
//! 0008, decisions 1 to 3): in which clock each word of the second history is read, when the
//! DC estimate is a register's output, when a beat is at the stream's pins, and in which
//! clock the hold, the marks and the decoder's verdict take effect.
//!
//! The model works in sample periods: `push(x, room)` reads the history in the instant of a
//! declaration, hands on up to `room` beats "in the period" and takes the decoder's verdict
//! between two samples. Hardware has one read port on a memory that goes on being written,
//! a rotator of four stages that does not stall in front of a reader that does, and a
//! verdict that arrives in any clock. This module makes the same stream from the memory *as
//! it stands in the clock each word is read*, a beat a clock. It is not a second definition
//! of the hand-off: the gate (`tests/g2_rx_handoff_schedule.rs`) holds every period's
//! beats, the hold and the acquisitions to [`RxHandoffFixed::push`], bit for bit, the
//! model's `room` being the clocks of that period in which the reader was ready.
//!
//! # The clocks
//!
//! Counts are from cycle to cycle, as in [`crate::rx_decision`]. Cycle X(n) is the one in
//! which ADC sample n is at the front's input pins and at the second history's write port
//! (address n modulo the depth; the same `in_valid` and reset as the front, so the front's
//! `acq_fft` is this memory's address). Samples are at least R cycles apart.
//!
//! **The tick.** Everything the model does "at sample n" is done in one cycle,
//!
//! ```text
//! T(n) = X(n) + K,    K = memory 1 + rotator 4 + buffer 1 = 6,
//! ```
//!
//! the last cycle before sample n itself can be at the stream's pins. The front's
//! `acq_valid` comes in cycle X(n) + 4 and is carried to the tick with the window's address
//! and the CFO word. *Period n* is the cycles T(n) + 1 … T(n + 1): the beats taken in them
//! are the beats of the model's `push` of sample n, and the model's `room` is the number of
//! those cycles in which the reader is ready. Nothing but the taking of beats changes
//! state between two ticks.
//!
//! In the tick, in this order (the order of `push`): the verdict that arrived since the
//! last tick; the hold down at the frame's last sample; a declaration (the open stream
//! closed with its mark, the buffer and the rotator's pipeline dropped, the new packet
//! opened, the hold up); the rail window and the release; the overrun. A beat taken in the
//! tick's own cycle belongs to the period before and is counted before the tick looks at
//! the reader's position.
//!
//! **The verdict** (`signal`) is a strobe with the tag, a failed flag, the number of data
//! symbols and the patient bit, in any cycle. The first strobe since the last tick that
//! carries the open packet's tag is kept in one register and applied at the next tick, if
//! that packet is still open then and has no verdict yet: a strobe in a tick's own cycle
//! waits for the next. A verdict therefore acts up to one sample period late and never in
//! the middle of a period, which is what lets a period's beats be one call of the model.
//!
//! # The DC estimate, in cycles after the tick t of a declaration
//!
//! | Cycles | What |
//! |---|---|
//! | t | the window's address and the CFO word latched; an estimate or a replay in flight is dropped |
//! | t + 1 … t + 128 | the addresses of the two long symbols' 128 samples, the oldest first, one a cycle on the history's one read port |
//! | t + 2 … t + 129 | the words; each added to the first or the second symbol's sum (18 bits a component) |
//! | t + 130 | both sums registered. Meanwhile: e, the rotator's table at 64 CFO words and half an address; d = A − e, 64·\|d\|², the test 256·\|d\|² < A² |
//! | t + 132 | S = A·Σ₂ − e·Σ₁ (products, then the difference) |
//! | t + 134 | the numerator S·d* (two stages) |
//! | t + 135 | its components' magnitudes in two dividers side by side |
//! | t + 146 | 11 quotient bits each (restoring, a bit a cycle). A quotient of 2¹¹ or more is known before the first step, from the dividend's upper bits, and is a rail of the ADC's range whatever its other bits |
//! | t + 147 | **the offset registered**: half to even on the remainder, the sign, the ADC's 12 bits; the first address of the replay in this cycle |
//! | t + 153 | the first sample of T1's window at the stream's pins (memory 1, the offset subtracted in the word's cycle, rotator 4, buffer 1) |
//! | T(n + 20) + 1 | the first cycle the stream may hand it on: t + 161 with samples exactly 8 cycles apart, eight to spare |
//!
//! One divider used twice ends 12 cycles later and is late by four at R = 8.
//!
//! # The replay
//!
//! One address a cycle on the same port (the estimate has ended, or was dropped with its
//! packet), each word less the offset through the rotator, whose phase is zero at the
//! window's first sample and steps back by the CFO word a sample. The rotator does not
//! stall and the reader may: an address is issued only while the words on their way and
//! the beats in the buffer are fewer than the buffer's depth (the count as it stood at the
//! cycle's start: a beat taken frees its place a cycle later). With a depth of K + 1 = 7 a
//! reader that takes a beat every cycle gets one every cycle; with 6 it does not. A sample
//! the antenna gave in cycle X(n) is addressed in X(n) + 1 at the earliest and at the pins
//! in T(n) + 1, the first cycle of its own period: the model's "never a sample the antenna
//! has not given" costs no test.
//!
//! The stream's pins give, in this order of precedence: the mark waiting in its register;
//! the end mark of a packet whose last sample has been taken; the buffer's oldest beat. The
//! reader's position (the model's `read`) counts beats taken, not words read: what is in
//! the buffer when a stream is closed is dropped.
//!
//! [`RxHandoffFixed`]: crate::rx_front_fixed::RxHandoffFixed
//! [`RxHandoffFixed::push`]: crate::rx_front_fixed::RxHandoffFixed::push

use crate::rx_decision::{Memory, ScheduleError, Store};
use crate::rx_front::{backoff_samples, RxConfig, GI_LEN, LTF_SYMBOL_LEN, PAIR_LEN};
use crate::rx_front_fixed::{
    iabs2, imul, nco_table, remove_dc, rotate_with, turn, FixedAcquisition, FixedRxConfig,
    HandoffBeat, HandoffConfig, HandoffEnd, HandoffStart, RxFrontFixed, SignalVerdict, ADC_BITS,
    ADC_FULL_SCALE, ADC_HISTORY, HANDOFF_TAIL, IQ, MAX_DATA_SYMBOLS, Y_BITS,
};
use std::collections::VecDeque;

/// A synchronous memory's read: the word is registered the cycle after its address.
const MEM: usize = 1;
/// Quotient bits of the offset's dividers: the magnitudes below 2^11. A larger quotient
/// is known from the dividend's upper bits before the first step, and rounds to a rail of
/// the ADC's 12 bits or beyond it whatever the rest.
const DIVIDE_STEPS: u32 = ADC_BITS - 1;
/// Samples from the FFT window of T1 to the end of SIGNAL's window.
const SIGNAL_END: usize = PAIR_LEN + GI_LEN + LTF_SYMBOL_LEN;
/// Samples of one data symbol.
const SYMBOL: usize = GI_LEN + LTF_SYMBOL_LEN;

/// What the hand-off's schedule is laid out against. The defaults are the front as built
/// (160 MHz at 20 MS/s, `acq_valid` four cycles after its sample) and this module's
/// allowances for RTL not yet written. Latencies are from cycle to cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HandoffTiming {
    /// R: the least number of clocks between two samples.
    pub clocks_per_sample: usize,
    /// Cycles from X(n) to the front's `acq_valid` for an acquisition applied at sample n
    /// (`rx_front`: the cycle after the third edge from the taking edge, 4).
    pub acq_latency: usize,
    /// The replay's rotator, as `hw/rtl/nco.sv` is built: 4.
    pub rotator: usize,
    /// The register of the buffer between the rotator and the stream's pins: 1.
    pub buffer: usize,
    /// Beats the buffer holds, the rotator's pipeline counted against it.
    pub buffer_depth: usize,
    /// Cycles from a beat taken to its place being counted free: 1 (a registered count),
    /// or 0 (free in the cycle of the beat; anything above 0 is 1).
    pub credit: usize,
    /// From the last word to the two sums registered: 1.
    pub sums: usize,
    /// From the sums to S = A·Σ₂ − e·Σ₁: 2.
    pub products: usize,
    /// From S to the numerator S·d*: 2.
    pub numerator: usize,
    /// Dividers: 2 side by side, or 1 used twice.
    pub dividers: usize,
    /// Words of the second history ([`ADC_HISTORY`]: 512).
    pub history_depth: usize,
}

impl Default for HandoffTiming {
    fn default() -> Self {
        Self {
            clocks_per_sample: 8,
            acq_latency: 4,
            rotator: 4,
            buffer: 1,
            buffer_depth: 7,
            credit: 1,
            sums: 1,
            products: 2,
            numerator: 2,
            dividers: 2,
            history_depth: ADC_HISTORY,
        }
    }
}

impl HandoffTiming {
    /// K: cycles from X(n) to the tick of sample n.
    pub fn tick(&self) -> usize {
        MEM + self.rotator + self.buffer
    }

    /// Cycles from a declaration's tick to the offset as a register's output.
    pub fn estimate_len(&self) -> usize {
        // The dividers' registers and their steps, once for both components or twice.
        let divisions = if self.dividers >= 2 { 1 } else { 2 };
        PAIR_LEN
            + MEM
            + self.sums
            + self.products
            + self.numerator
            + divisions * (1 + DIVIDE_STEPS as usize)
            + 1
    }
}

/// Who reads the stream: asked once in every cycle, in this order.
pub trait HandoffReader {
    /// Whether a beat is taken in cycle `clock` if one is at the pins.
    fn ready(&mut self, clock: usize) -> bool;
    /// The beat taken in cycle `clock`.
    fn beat(&mut self, _clock: usize, _beat: &HandoffBeat) {}
    /// The verdict strobe of cycle `clock`: the tag and what [`RxHandoffFixed::signal`]
    /// takes.
    ///
    /// [`RxHandoffFixed::signal`]: crate::rx_front_fixed::RxHandoffFixed::signal
    fn verdict(&mut self, _clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        None
    }
}

/// One DC estimate made on the schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EstimateRun {
    /// The sample of the declaration.
    pub declared_at: usize,
    /// The cycle of its tick.
    pub tick: usize,
    /// The offset, [`ltf_dc_offset_fixed`]'s.
    ///
    /// [`ltf_dc_offset_fixed`]: crate::rx_front_fixed::ltf_dc_offset_fixed
    pub dc: IQ,
    /// The cycle the offset is a register's output.
    pub done: usize,
    /// The cycle the window's first sample is at the stream's pins.
    pub head: usize,
    /// The first cycle the stream may hand it on (`None`: the stream ends before).
    pub opens: Option<usize>,
}

/// A verdict strobe and the tick that takes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerdictStrobe {
    /// The cycle of the strobe.
    pub clock: usize,
    /// The sample at whose tick it is applied: the model's `signal` is called before the
    /// `push` of this sample.
    pub before: usize,
    /// The tag.
    pub packet: u8,
    /// The verdict.
    pub verdict: Option<SignalVerdict>,
}

/// A stream handed on in clocks.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HandoffRun {
    /// T(n) of every sample.
    pub ticks: Vec<usize>,
    /// The last cycle of the last sample's period.
    pub end: usize,
    /// The acquisition declared at each sample's tick.
    pub acquired: Vec<Option<FixedAcquisition>>,
    /// The hold after each tick.
    pub hold: Vec<bool>,
    /// The cycles of each period in which the reader was ready.
    pub room: Vec<usize>,
    /// Every beat with the cycle it was taken in.
    pub beats: Vec<(usize, HandoffBeat)>,
    /// Every verdict strobe.
    pub verdicts: Vec<VerdictStrobe>,
    /// The estimates that ran to their end.
    pub estimates: Vec<EstimateRun>,
    /// Words of the history to spare at the read that came closest to an overwritten one.
    pub history_spare: usize,
    /// Words read from the history.
    pub reads: usize,
    /// The most beats the buffer and the rotator's pipeline held together.
    pub buffered: usize,
}

/// The DC offset of [`ltf_dc_offset_fixed`] from the 128 samples `field` of the two long
/// symbols, in the form the hardware computes it: two sums, S = A·Σ₂ − e·Σ₁, the numerator
/// S·d*, and two restoring divisions of 11 quotient bits. `None` unless `field` has 128
/// samples and the configuration is valid.
///
/// [`ltf_dc_offset_fixed`]: crate::rx_front_fixed::ltf_dc_offset_fixed
pub fn dc_offset_serial(field: &[(i16, i16)], cfo_phase: i64, fx: &FixedRxConfig) -> Option<IQ> {
    if !fx.is_valid() || field.len() != PAIR_LEN {
        return None;
    }
    let (first, second) = field.split_at(LTF_SYMBOL_LEN);
    let sum = |part: &[(i16, i16)]| {
        part.iter().fold((0i64, 0i64), |s, v| {
            (s.0 + i64::from(v.0), s.1 + i64::from(v.1))
        })
    };
    Some(offset_of_sums(
        sum(first),
        sum(second),
        cfo_phase,
        fx,
        &nco_table(fx),
    ))
}

/// The offset of [`dc_offset_serial`] from the two sums it starts with: Σ₁ of the first long
/// symbol's 64 samples and Σ₂ of the second's (what `hw/rtl/dc_estimate.sv` is handed).
/// `None` for an invalid configuration. A component of a sum beyond 64 samples of the ADC's
/// range is taken as it is.
pub fn dc_offset_of_sums(first: IQ, second: IQ, cfo_phase: i64, fx: &FixedRxConfig) -> Option<IQ> {
    fx.is_valid()
        .then(|| offset_of_sums(first, second, cfo_phase, fx, &nco_table(fx)))
}

/// The offset from the sums Σ₁ and Σ₂ of the first and the second long symbol's samples.
/// With the default rotator and 12-bit samples: the sums 18 bits a component, S 30, the
/// numerator 42, the divisor 64·|d|² 28.
fn offset_of_sums(s1: IQ, s2: IQ, cfo_phase: i64, fx: &FixedRxConfig, table: &[IQ]) -> IQ {
    let (p, l) = (fx.nco_phase_bits, fx.nco_lut_bits);
    // e: the rotator's table at 64 CFO words and half an address (a tie goes up).
    let half_address = p.checked_sub(l + 1).map_or(0, |k| 1i64 << k);
    let phase = cfo_phase.rem_euclid(1i64 << p) * LTF_SYMBOL_LEN as i64 + half_address;
    let e = rotate_with(phase, fx, table);
    let a = rotate_with(0, fx, table).0;
    let d = (a - e.0, -e.1);
    let d2 = iabs2(d);
    if 256 * d2 < i128::from(a) * i128::from(a) {
        return (0, 0);
    }
    let turned = imul(e, s1);
    let s = (a * s2.0 - turned.0, a * s2.1 - turned.1);
    let wide = |v: i64| i128::from(v);
    let num = (
        wide(s.0) * wide(d.0) + wide(s.1) * wide(d.1),
        wide(s.1) * wide(d.0) - wide(s.0) * wide(d.1),
    );
    let den = LTF_SYMBOL_LEN as u128 * d2.unsigned_abs();
    // |d| < 2^Q (the table's rounding lets it pass 2·A by a little, never 2^Q), so the
    // divisor is below 2^(2Q + 6): 28 bits with the default rotator, and a remainder
    // register of 29.
    let rem_bits = 2 * fx.nco_amp_bits + 7;
    (divide(num.0, den, rem_bits), divide(num.1, den, rem_bits))
}

/// `num` / `den` rounded half to even and saturated to the ADC's 12 bits, as a restoring
/// divider makes it: on the magnitude, [`DIVIDE_STEPS`] quotient bits from the top, a bit a
/// step, in a remainder register of `rem_bits` (one more than the divisor's); the rounding
/// from the remainder; the sign last (half to even is the same on both sides of zero).
/// `den` is not 0 and below 2^(`rem_bits` − 1).
fn divide(num: i128, den: u128, rem_bits: u32) -> i64 {
    let (lo, hi) = (-(1i64 << (ADC_BITS - 1)), (1i64 << (ADC_BITS - 1)) - 1);
    let register = (1u128 << rem_bits.min(127)) - 1;
    let mag = num.unsigned_abs();
    // A quotient of 2^11 or more: the dividend's upper bits are not below the divisor.
    // Negative it is −2^11 or less, the lower rail; positive it is above the upper one.
    // Only below that does the remainder stay under twice the divisor, in its register.
    let mut rem = mag >> DIVIDE_STEPS;
    if rem >= den {
        return if num < 0 { lo } else { hi };
    }
    let mut q = 0i64;
    for k in (0..DIVIDE_STEPS).rev() {
        rem = (rem << 1 | (mag >> k & 1)) & register;
        q <<= 1;
        if rem >= den {
            rem -= den;
            q |= 1;
        }
    }
    q += match (2 * rem).cmp(&den) {
        std::cmp::Ordering::Greater => 1,
        std::cmp::Ordering::Equal => q & 1,
        std::cmp::Ordering::Less => 0,
    };
    (if num < 0 { -q } else { q }).clamp(lo, hi)
}

/// The frame's last sample and one, as the model's `packet_end`: the back-off after the
/// window of the last of `symbols` data symbols.
fn packet_end(fft_start: usize, symbols: usize, backoff: usize) -> usize {
    fft_start
        .saturating_add(SIGNAL_END + SYMBOL * symbols.min(MAX_DATA_SYMBOLS))
        .saturating_add(backoff)
}

/// The open packet.
#[derive(Debug, Clone, Copy)]
struct Open {
    packet: u8,
    fft_start: usize,
    cfo_phase: i64,
    /// The sample and the cycle of the declaration's tick.
    declared_at: usize,
    tick: usize,
    dc: Offset,
    /// The first cycle a beat may be handed on.
    opens: Option<usize>,
    /// The next sample the reader takes (the model's `read`), the next to be read from
    /// the history, and the phase that one is rotated by.
    taken: usize,
    fetched: usize,
    phase: i64,
    hold_end: usize,
    last: usize,
    verdict: bool,
}

/// The open packet's offset.
#[derive(Debug, Clone, Copy)]
enum Offset {
    /// The estimate's two sums as the words come: `next` of the 128 is the next read.
    Summing { next: usize, first: IQ, second: IQ },
    /// The offset, a register's output from cycle `at` on.
    Known { dc: IQ, at: usize },
}

/// Everything a tick or a beat changes.
struct Machine {
    open: Option<Open>,
    /// Words on their way through the rotator, with the cycle each is at the pins.
    flight: VecDeque<(usize, HandoffBeat)>,
    buffer: VecDeque<HandoffBeat>,
    mark: Option<HandoffEnd>,
    hold: bool,
    patient: bool,
    rails: u64,
    opened: u8,
    /// The verdict kept for the next tick, and whether one was kept since the last.
    slot: Option<Option<SignalVerdict>>,
    kept: bool,
}

impl Machine {
    /// Ends the open packet with `mark`: the hold down, the estimate and whatever was read
    /// ahead dropped. A mark that finds one waiting is not sent.
    fn close(&mut self, mark: HandoffEnd) {
        self.drop_stream();
        self.hold = false;
        self.mark.get_or_insert(mark);
    }

    fn drop_stream(&mut self) {
        self.open = None;
        self.flight.clear();
        self.buffer.clear();
    }
}

/// The stream of [`RxHandoffFixed`] on the ADC samples `x`, handed on in clocks.
///
/// `idle(n)` is the number of idle clocks before sample n beyond the R − 1 of
/// `t.clocks_per_sample`. `pace` gives the start delay and the release's numbers; its
/// `rate` is not used (a beat a clock: a period of more than 8 clocks hands on more than
/// the model does at its default rate of 8). An empty run, without ticks, for an invalid
/// configuration.
///
/// # Errors
/// The first thing that cannot be built: a word read when the history no longer or not yet
/// holds it, an offset not ready for its stream's first cycle, a tick before `acq_valid`,
/// or a ready reader left without a beat that the model hands on in that period
/// ([`ScheduleError::Late`]). (The read port is used once a cycle by construction.)
///
/// [`RxHandoffFixed`]: crate::rx_front_fixed::RxHandoffFixed
pub fn run_handoff(
    x: &[(i16, i16)],
    cfg: &RxConfig,
    fx: &FixedRxConfig,
    pace: &HandoffConfig,
    t: &HandoffTiming,
    idle: &dyn Fn(usize) -> usize,
    reader: &mut dyn HandoffReader,
) -> Result<HandoffRun, ScheduleError> {
    if !fx.is_valid() {
        return Ok(HandoffRun::default());
    }
    let k = t.tick();
    if k < t.acq_latency {
        return Err(ScheduleError::Late);
    }
    let period = t.clocks_per_sample.max(1);
    let backoff = backoff_samples(cfg);
    let table = nco_table(fx);
    let through = MEM + t.rotator + t.buffer;

    // X(n) of every sample and of one more, whose tick closes the last period.
    let mut clock = 0;
    let x_clock: Vec<usize> = (0..=x.len())
        .map(|n| {
            clock += period + idle(n);
            clock
        })
        .collect();
    let ticks: Vec<usize> = x_clock.iter().map(|c| c + k).collect();
    let (written, end) = (&x_clock[..x.len()], ticks[x.len()]);
    let mut history = Memory::new(Store::Adc, t.history_depth, written, x);
    let mut front = RxFrontFixed::new(cfg.clone(), *fx);

    let mut m = Machine {
        open: None,
        flight: VecDeque::new(),
        buffer: VecDeque::new(),
        mark: None,
        hold: false,
        patient: false,
        rails: 0,
        opened: 0,
        slot: None,
        kept: false,
    };
    let mut run = HandoffRun {
        ticks: ticks[..x.len()].to_vec(),
        end,
        room: vec![0; x.len()],
        history_spare: t.history_depth,
        ..HandoffRun::default()
    };
    // The sample of the next tick; the period a cycle is in is the one before it.
    let mut next = 0usize;

    for c in 0..=end {
        // The words whose rotation and buffer register end here are at the pins.
        while m.flight.front().is_some_and(|f| f.0 <= c) {
            m.buffer.extend(m.flight.pop_front().map(|f| f.1));
        }
        let occupied = m.flight.len() + m.buffer.len();
        run.buffered = run.buffered.max(occupied);

        // The pins: the waiting mark, the end of a packet read to its last sample, the
        // buffer's oldest beat.
        let ready = reader.ready(c);
        let period_of = next.checked_sub(1);
        if let (true, Some(n)) = (ready, period_of) {
            run.room[n] += 1;
        }
        let stream = m.open.filter(|o| o.opens.is_some_and(|at| c >= at));
        let at_pins = match (m.mark, stream) {
            (Some(mark), _) => Some(HandoffBeat::End(mark)),
            (None, Some(o)) if o.taken >= o.last => Some(HandoffBeat::End(HandoffEnd::Packet)),
            (None, Some(o)) => {
                let beat = m.buffer.front().copied();
                // A sample of a period that has begun, a reader that would take it, and
                // nothing at the pins: the pipeline is not a beat a clock.
                if beat.is_none() && ready && period_of.is_some_and(|n| o.taken <= n) {
                    return Err(ScheduleError::Late);
                }
                beat
            }
            (None, None) => None,
        };
        if let (true, Some(beat)) = (ready, at_pins) {
            run.beats.push((c, beat));
            reader.beat(c, &beat);
            match beat {
                HandoffBeat::End(_) if m.mark.is_some() => m.mark = None,
                HandoffBeat::End(_) => m.drop_stream(),
                HandoffBeat::Sample { .. } => {
                    m.buffer.pop_front();
                    if let Some(o) = m.open.as_mut() {
                        o.taken += 1;
                    }
                }
            }
        }

        // The history's read port: the estimate's 128 words, then the replay's.
        let free = if t.credit == 0 {
            m.flight.len() + m.buffer.len()
        } else {
            occupied
        } < t.buffer_depth;
        match m.open.as_mut().map(|o| (o.dc, o)) {
            Some((
                Offset::Summing {
                    next: i,
                    first,
                    second,
                },
                o,
            )) => {
                let v = history.read(0, o.fft_start + i, c)?;
                let add = |s: IQ| (s.0 + i64::from(v.0), s.1 + i64::from(v.1));
                let (first, second) = if i < LTF_SYMBOL_LEN {
                    (add(first), second)
                } else {
                    (first, add(second))
                };
                o.dc = if i + 1 == PAIR_LEN {
                    Offset::Known {
                        dc: offset_of_sums(first, second, o.cfo_phase, fx, &table),
                        at: o.tick + t.estimate_len(),
                    }
                } else {
                    Offset::Summing {
                        next: i + 1,
                        first,
                        second,
                    }
                };
            }
            Some((Offset::Known { dc, at }, o))
                if c >= at
                    && free
                    && o.fetched < o.last
                    && written.get(o.fetched).is_some_and(|&w| w < c) =>
            {
                let v = history.read(0, o.fetched, c)?;
                let first = (o.fetched == o.fft_start).then_some(HandoffStart {
                    cfo_phase: o.cfo_phase,
                    packet: o.packet,
                });
                let z = turn(
                    remove_dc(v, dc),
                    &mut o.phase,
                    o.cfo_phase,
                    Y_BITS,
                    fx,
                    &table,
                );
                if first.is_some() {
                    // Too late for the stream's first cycle?
                    if o.opens.is_some_and(|at| c + through > at) {
                        return Err(ScheduleError::Late);
                    }
                    run.estimates.push(EstimateRun {
                        declared_at: o.declared_at,
                        tick: o.tick,
                        dc,
                        done: o.tick + t.estimate_len(),
                        head: c + through,
                        opens: o.opens,
                    });
                }
                m.flight
                    .push_back((c + through, HandoffBeat::Sample { z, first }));
                o.fetched += 1;
            }
            _ => {}
        }

        // The tick of sample `next`, in the order of the model's `push`.
        if let Some(&sample) = x.get(next).filter(|_| ticks[next] == c) {
            let n = next;
            next += 1;
            if let Some(verdict) = m.slot.take() {
                match (m.open.as_mut().filter(|o| !o.verdict), verdict) {
                    (Some(o), Some(v)) => {
                        o.verdict = true;
                        o.hold_end = packet_end(o.fft_start, v.data_symbols, backoff);
                        o.last = o.hold_end.saturating_add(HANDOFF_TAIL);
                        m.patient = v.patient;
                    }
                    (Some(_), None) => m.close(HandoffEnd::Stopped),
                    (None, _) => {}
                }
            }
            m.kept = false;
            let acquired = front.push(sample);
            if m.open.is_some_and(|o| n >= o.hold_end) {
                m.hold = false;
            }
            if let Some(a) = acquired {
                let patient = m.patient && m.hold;
                if let Some(old) = m.open {
                    m.close(if old.taken >= old.last {
                        HandoffEnd::Packet
                    } else {
                        HandoffEnd::Acquisition
                    });
                }
                m.opened = m.opened.wrapping_add(1);
                let hold_end = packet_end(a.fft_start, MAX_DATA_SYMBOLS, backoff);
                m.open = Some(Open {
                    packet: m.opened,
                    fft_start: a.fft_start,
                    cfo_phase: a.cfo_phase,
                    declared_at: n,
                    tick: c,
                    dc: Offset::Summing {
                        next: 0,
                        first: (0, 0),
                        second: (0, 0),
                    },
                    opens: ticks
                        .get(n.saturating_add(pace.start_delay))
                        .map(|at| at + 1),
                    taken: a.fft_start,
                    fetched: a.fft_start,
                    phase: 0,
                    hold_end,
                    last: hold_end.saturating_add(HANDOFF_TAIL),
                    verdict: false,
                });
                m.hold = true;
                m.patient = patient;
            }
            // The release: the rail flags of the last samples, whatever the hold is.
            let full = i32::from(sample.0).min(i32::from(sample.1)) <= -ADC_FULL_SCALE
                || i32::from(sample.0).max(i32::from(sample.1)) >= ADC_FULL_SCALE - 1;
            m.rails = m.rails << 1 | u64::from(full);
            let (window, count) = if m.patient {
                (pace.patient_window, pace.patient_count)
            } else {
                (pace.clip_window, pace.clip_count)
            };
            let window = if window >= 64 {
                u64::MAX
            } else {
                (1u64 << window) - 1
            };
            if count > 0 && (m.rails & window).count_ones() as usize >= count {
                m.hold = false;
            }
            // The reader a whole history behind: its next sample is being overwritten.
            if m.open
                .is_some_and(|o| o.taken < o.last && o.taken + ADC_HISTORY <= n)
            {
                m.close(HandoffEnd::Overrun);
            }
            run.acquired.push(acquired);
            run.hold.push(m.hold);
        }

        // The verdict's strobe: the first since the last tick with the open packet's tag
        // waits for the next tick.
        if let Some((packet, verdict)) = reader.verdict(c) {
            run.verdicts.push(VerdictStrobe {
                clock: c,
                before: next,
                packet,
                verdict,
            });
            if !m.kept && packet == m.opened {
                m.slot = Some(verdict);
                m.kept = true;
            }
        }
    }
    run.history_spare = history.spare;
    run.reads = history.reads(0);
    Ok(run)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// n / d from the definition: the nearest integer, and of two equally near the even one.
    fn nearest_even(n: i128, d: i128) -> i128 {
        let (floor, rest) = (n.div_euclid(d), n.rem_euclid(d));
        if 2 * rest > d || (2 * rest == d && floor % 2 != 0) {
            floor + 1
        } else {
            floor
        }
    }

    #[test]
    fn the_divider_rounds_half_to_even_and_saturates_to_twelve_bits() {
        for den in 1..=24u128 {
            // A remainder register of the divisor's bits and one.
            let rem_bits = 128 - den.leading_zeros() + 1;
            let span = 2100 * den as i128;
            for num in -span..=span {
                let want = nearest_even(num, den as i128).clamp(-2048, 2047) as i64;
                assert_eq!(divide(num, den, rem_bits), want, "{num} / {den}");
            }
            // Far outside: the rails, from the dividend's upper bits alone.
            assert_eq!(divide(1 << 100, den, rem_bits), 2047);
            assert_eq!(divide(-(1 << 100), den, rem_bits), -2048);
        }
        // The widths of the default rotator: a divisor of 28 bits, a numerator of 42.
        let den = (1u128 << 28) - 3;
        for q in [-2049i128, -2048, -2047, -1, 0, 1, 1000, 2046, 2047, 2048] {
            for part in [
                0i128,
                1,
                (den as i128) / 2,
                (den as i128) / 2 + 1,
                den as i128 - 1,
            ] {
                let num = q * den as i128 + part;
                let want = nearest_even(num, den as i128).clamp(-2048, 2047) as i64;
                assert_eq!(divide(num, den, 29), want, "{q} and {part}");
            }
        }
    }
}
