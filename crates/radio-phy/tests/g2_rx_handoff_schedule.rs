//! Gate for the hand-off's schedule in clocks (R0-08d part d step 4b; ADR 0008 decisions 1
//! to 3): `rx_handoff::run_handoff` hands on the stream of `RxHandoffFixed` a beat a clock,
//! from a history read at the clocks the schedule says.
//!
//! References: every period's beats, the hold and the acquisitions are those of
//! `RxHandoffFixed::push` (gated in `g2_rx_handoff.rs`), the model's `room` being the clocks
//! of the period in which the reader was ready and its verdicts the strobes of the period
//! before; the offset is `ltf_dc_offset_fixed`'s (gated by hand in `g2_rx_front.rs`); the
//! cycle counts are the schedule's table worked by hand (the module's text, G0 in
//! `docs/gates/rx_front.md`). A model gate: no reference from outside the project.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_decision::{ScheduleError, Store};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    ltf_dc_offset_fixed, to_adc, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd,
    RxHandoffFixed, SignalVerdict, ADC_FULL_SCALE,
};
use radio_phy::rx_handoff::{
    dc_offset_serial, run_handoff, HandoffReader, HandoffRun, HandoffTiming,
};

type C = (f64, f64);

const FS: f64 = 20e6;
const CARRIER: f64 = 5.8e9;
/// Samples from the FFT window of T1 to the end of SIGNAL's window.
const SIGNAL_END: usize = 128 + 80;
/// The front's back-off at 20 MS/s (`RxConfig::backoff_ns` of 200 ns).
const B: usize = 4;
/// 100 octets at 54 Mb/s: four data symbols.
const SHORT: (Rate, usize) = (Rate::R54, 100);
const SHORT_SYMBOLS: usize = 4;
/// 100 octets at 6 Mb/s: 16 + 800 + 6 bits in symbols of 24, 35 data symbols.
const SLOW: (Rate, usize) = (Rate::R6, 100);
const SLOW_SYMBOLS: usize = 35;

/// `g2_rx_handoff`'s scene: frames of random octets at −12 dBFS and 30 dB behind `lead`
/// samples of noise, `gap` samples after each, the clock off by `ppm`, on a DC offset of
/// −20 dBFS.
fn scene(
    frames: &[(Rate, usize)],
    lead: usize,
    gap: usize,
    ppm: f64,
    seed: u64,
) -> Vec<(i16, i16)> {
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(seed);
    let mut x = vec![(0.0, 0.0); lead];
    let mut p = 1.0;
    for &(rate, octets) in frames {
        let psdu: Vec<u8> = (0..octets).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f: Vec<C> = build_ppdu(&psdu, rate, 0x5d, &tx)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        p = mean_power(&f);
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); gap]);
    }
    let x = if ppm == 0.0 { x } else { resample(&x, ppm) };
    let x = frequency_offset(&x, ppm * 1e-6 * CARRIER, FS, 2.0);
    let x = awgn(&x, 30.0, p, &mut rng);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
    let dc = 10f64.powf(-20.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
    let x: Vec<C> = x.iter().map(|s| (s.0 - 0.8 * dc, s.1 + 0.6 * dc)).collect();
    to_adc(&x, gain)
}

fn plain(data_symbols: usize) -> SignalVerdict {
    SignalVerdict {
        data_symbols,
        patient: false,
    }
}

/// What the decoder says of a stream: how many cycles after the beat that completes
/// SIGNAL's window, and what.
type Say = Option<(usize, Option<SignalVerdict>)>;

/// A reader of the stream: ready by a rule on the cycle, a verdict for the k-th stream it
/// reads (with that stream's tag), and strobes scripted by cycle.
struct Decoder<'a> {
    ready: &'a dyn Fn(usize) -> bool,
    say: &'a dyn Fn(usize) -> Say,
    due: Vec<(usize, u8, Option<SignalVerdict>)>,
    streams: usize,
    taken: usize,
    tag: u8,
}

impl<'a> Decoder<'a> {
    fn new(ready: &'a dyn Fn(usize) -> bool, say: &'a dyn Fn(usize) -> Say) -> Self {
        Self {
            ready,
            say,
            due: Vec::new(),
            streams: 0,
            taken: 0,
            tag: 0,
        }
    }

    fn with_strobes(mut self, strobes: &[(usize, u8, Option<SignalVerdict>)]) -> Self {
        self.due.extend_from_slice(strobes);
        self
    }
}

impl HandoffReader for Decoder<'_> {
    fn ready(&mut self, clock: usize) -> bool {
        (self.ready)(clock)
    }

    fn beat(&mut self, clock: usize, beat: &HandoffBeat) {
        let HandoffBeat::Sample { first, .. } = beat else {
            return;
        };
        if let Some(s) = first {
            self.tag = s.packet;
            self.taken = 0;
            self.streams += 1;
        }
        self.taken += 1;
        if self.taken == SIGNAL_END {
            if let Some((after, v)) = (self.say)(self.streams - 1) {
                self.due.push((clock + after, self.tag, v));
            }
        }
    }

    fn verdict(&mut self, clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        let i = self.due.iter().position(|d| d.0 == clock)?;
        let d = self.due.remove(i);
        Some((d.1, d.2))
    }
}

fn always(_: usize) -> bool {
    true
}

fn silent(_: usize) -> Say {
    None
}

/// What the model gave on the run's rooms and verdicts.
struct Model {
    /// The hold after each push.
    hold: Vec<bool>,
    /// The marks, with the push that handed each.
    marks: Vec<(usize, HandoffEnd)>,
    /// Beats handed by each push.
    count: Vec<usize>,
}

impl Model {
    /// The hold as (first, one past the last) push of each stretch it was up.
    fn holds(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        let mut from = None;
        for (n, &h) in self.hold.iter().enumerate() {
            match (from, h) {
                (None, true) => from = Some(n),
                (Some(a), false) => {
                    out.push((a, n));
                    from = None;
                }
                _ => {}
            }
        }
        if let Some(a) = from {
            out.push((a, self.hold.len()));
        }
        out
    }

    fn ends(&self) -> Vec<HandoffEnd> {
        self.marks.iter().map(|m| m.1).collect()
    }
}

/// Holds the run in clocks to `RxHandoffFixed`: the model is pushed the same samples with
/// each period's ready cycles as its room and each strobe as a `signal` before the push of
/// the tick that takes it; every period's beats, the hold and the acquisition are its.
fn held_to_the_model(
    adc: &[(i16, i16)],
    cfg: &RxConfig,
    fx: FixedRxConfig,
    pace: HandoffConfig,
    run: &HandoffRun,
    what: &str,
) -> Model {
    assert_eq!(
        run.ticks.len(),
        adc.len(),
        "{what}: a tick for every sample"
    );
    assert_eq!(run.room.len(), adc.len(), "{what}");
    assert_eq!(run.hold.len(), adc.len(), "{what}");
    assert_eq!(run.acquired.len(), adc.len(), "{what}");
    // A beat a clock: the model's own rate is out of the way.
    let free = HandoffConfig {
        rate: usize::MAX,
        ..pace
    };
    let mut rx = RxHandoffFixed::new(cfg.clone(), fx, free);
    let mut model = Model {
        hold: Vec::new(),
        marks: Vec::new(),
        count: Vec::new(),
    };
    let (mut b, mut v) = (0, 0);
    for (n, &x) in adc.iter().enumerate() {
        while let Some(s) = run.verdicts.get(v).filter(|s| s.before <= n) {
            assert_eq!(s.before, n, "{what}: strobes in the order of their ticks");
            rx.signal(s.packet, s.verdict);
            v += 1;
        }
        let step = rx.push(x, run.room[n]);
        let end = run.ticks.get(n + 1).copied().unwrap_or(run.end);
        let from = b;
        while run.beats.get(b).is_some_and(|x| x.0 <= end) {
            assert!(run.beats[b].0 > run.ticks[n], "{what}: beats in order");
            b += 1;
        }
        let got: Vec<HandoffBeat> = run.beats[from..b].iter().map(|x| x.1).collect();
        assert_eq!(got, step.beats, "{what}: the beats of period {n}");
        assert_eq!(run.hold[n], step.hold, "{what}: the hold after sample {n}");
        assert_eq!(run.acquired[n], step.acquired, "{what}: sample {n}");
        for beat in &step.beats {
            if let HandoffBeat::End(e) = beat {
                model.marks.push((n, *e));
            }
        }
        model.count.push(step.beats.len());
        model.hold.push(step.hold);
    }
    assert_eq!(b, run.beats.len(), "{what}: no beat outside a period");
    model
}

/// One run with the default timing, held to the model.
fn check(
    adc: &[(i16, i16)],
    pace: HandoffConfig,
    idle: &dyn Fn(usize) -> usize,
    reader: &mut Decoder,
    what: &str,
) -> (HandoffRun, Model) {
    check_fx(adc, FixedRxConfig::default(), pace, idle, reader, what)
}

/// [`check`] with a front of other numbers.
fn check_fx(
    adc: &[(i16, i16)],
    fx: FixedRxConfig,
    pace: HandoffConfig,
    idle: &dyn Fn(usize) -> usize,
    reader: &mut Decoder,
    what: &str,
) -> (HandoffRun, Model) {
    check_cfg(adc, &RxConfig::new(FS), fx, pace, idle, reader, what)
}

/// [`check_fx`] with a receiver of other numbers.
fn check_cfg(
    adc: &[(i16, i16)],
    cfg: &RxConfig,
    fx: FixedRxConfig,
    pace: HandoffConfig,
    idle: &dyn Fn(usize) -> usize,
    reader: &mut Decoder,
    what: &str,
) -> (HandoffRun, Model) {
    let t = HandoffTiming::default();
    let run = run_handoff(adc, cfg, &fx, &pace, &t, idle, reader)
        .unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let model = held_to_the_model(adc, cfg, fx, pace, &run, what);
    (run, model)
}

/// `adc` with the samples at `at` replaced.
fn with(adc: &[(i16, i16)], at: &[(usize, (i16, i16))]) -> Vec<(i16, i16)> {
    let mut out = adc.to_vec();
    for &(n, v) in at {
        out[n] = v;
    }
    out
}

#[test]
fn each_rail_of_each_component_counts_and_a_declaring_sample_can_give_up_its_hold() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SLOW], 700, 900, 12.5, 4);
    let mut reader = Decoder::new(&always, &silent);
    let (run, _) = check(&adc, pace, &no_idle, &mut reader, "as it is");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    let at = a.decided_at + 150;
    // Two samples in a row, each with one component on one rail, in every pairing; a unit
    // inside a rail is not on it.
    let rails = [(2047, 0), (-2048, 0), (0, 2047), (0, -2048)];
    for first in rails {
        for second in rails {
            let clipped = with(&adc, &[(at, first), (at + 1, second)]);
            let mut reader = Decoder::new(&always, &silent);
            let what = format!("{first:?} then {second:?}");
            let (_, model) = check(&clipped, pace, &no_idle, &mut reader, &what);
            assert_eq!(model.holds(), [(a.decided_at, at + 1)], "{what}");
        }
        let near = with(&adc, &[(at, first), (at + 1, (2046, -2047))]);
        let mut reader = Decoder::new(&always, &silent);
        let (_, model) = check(&near, pace, &no_idle, &mut reader, "a unit inside");
        assert_eq!(model.holds(), [(a.decided_at, adc.len())], "{first:?}");
    }
    // The sample before the declaration's and the declaration's own: the rail window runs
    // whatever the hold is, so the hold raised by that sample is given up by it, and the
    // stream goes on.
    let d = a.decided_at;
    let clipped = with(&adc, &[(d - 1, (2047, 0)), (d, (0, -2048))]);
    let mut reader = Decoder::new(&always, &silent);
    let (run, model) = check(&clipped, pace, &no_idle, &mut reader, "at the declaration");
    assert_eq!(run.acquired[d], Some(a));
    assert_eq!(model.holds(), []);
    assert!(run.beats.len() > 1000);
    // A count of 0 never gives up; a window of 64 looks back over 64 samples.
    let never = HandoffConfig {
        clip_count: 0,
        ..pace
    };
    let burst = on_rails(&adc, at, 12);
    let mut reader = Decoder::new(&always, &silent);
    let (_, model) = check(&burst, never, &no_idle, &mut reader, "a count of 0");
    assert_eq!(model.holds(), [(d, adc.len())]);
    let wide = HandoffConfig {
        clip_window: 64,
        clip_count: 2,
        ..pace
    };
    for (gap, down) in [(63, Some(at + 63)), (64, None)] {
        let two = with(&adc, &[(at, (2047, 0)), (at + gap, (2047, 0))]);
        let mut reader = Decoder::new(&always, &silent);
        let (_, model) = check(&two, wide, &no_idle, &mut reader, "a window of 64");
        assert_eq!(model.holds(), [(d, down.unwrap_or(adc.len()))], "{gap}");
    }
}

#[test]
fn a_hold_that_ends_at_a_declaration_and_a_loss_at_one_follow_the_models_order() {
    let pace = HandoffConfig::default();
    // The gap for which the second declaration falls on the sample a length of the first
    // frame ends its hold at: the hold is down before the declaration looks at it, so the
    // patience is not inherited and two samples on a rail take the new hold.
    let mut found = None;
    for gap in 320..480 {
        let adc = scene(&[SLOW, SHORT], 700, gap, 12.5, 9);
        let mut reader = Decoder::new(&always, &silent);
        let (run, _) = check(&adc, pace, &no_idle, &mut reader, "search");
        let acq: Vec<_> = run.acquired.iter().flatten().copied().collect();
        if let [a, b] = acq[..] {
            let span = b.decided_at - a.fft_start - SIGNAL_END - B;
            if span.is_multiple_of(80) {
                found = Some((adc, a, b, span / 80));
                break;
            }
        }
    }
    let (adc, a, b, symbols) = found.unwrap();
    let told = move |k: usize| {
        (k == 0).then_some((
            100,
            Some(SignalVerdict {
                data_symbols: symbols,
                patient: true,
            }),
        ))
    };
    let clipped = on_rails(&adc, b.decided_at + 30, 2);
    let mut reader = Decoder::new(&always, &told);
    let (_, model) = check(&clipped, pace, &no_idle, &mut reader, "the hold ends there");
    assert_eq!(model.holds(), [(a.decided_at, b.decided_at + 31)]);
    // A symbol longer and the hold is up at the declaration: inherited, nothing happens.
    let longer = move |k: usize| {
        (k == 0).then_some((
            100,
            Some(SignalVerdict {
                data_symbols: symbols + 1,
                patient: true,
            }),
        ))
    };
    let mut reader = Decoder::new(&always, &longer);
    let (_, model) = check(&clipped, pace, &no_idle, &mut reader, "a symbol longer");
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);

    // A reader that stops so that its stream is lost at the very sample of the next
    // declaration: the declaration's mark, an abort. A sample earlier: the overrun's.
    let adc = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let mut reader = Decoder::new(&always, &silent);
    let (run, _) = check(&adc, pace, &no_idle, &mut reader, "always");
    let acq: Vec<_> = run.acquired.iter().flatten().copied().collect();
    let (a, b) = (acq[0], acq[1]);
    // At the antenna the reader takes sample m in the first cycle of its period.
    assert!(b.decided_at - 512 > a.decided_at + 100);
    let back = tick(b.decided_at) + 20;
    let (d2, len) = (b.decided_at, adc.len());
    // The abort leaves the hold up across both packets; the loss a sample earlier takes
    // it down for that one sample.
    let cases = [
        (0, HandoffEnd::Acquisition, vec![(a.decided_at, len)]),
        (
            1,
            HandoffEnd::Overrun,
            vec![(a.decided_at, d2 - 1), (d2, len)],
        ),
    ];
    for (short, mark, holds) in cases {
        let m = d2 - 512 - 1 - short;
        let stop = tick(m) + 1;
        let away = move |c: usize| c <= stop || c > back;
        let mut reader = Decoder::new(&away, &silent);
        let (_, model) = check(&adc, pace, &no_idle, &mut reader, "lost at the declaration");
        assert_eq!(model.marks[0], (d2 + 2, mark), "{short}");
        assert_eq!(model.holds(), holds, "{short}");
    }
}

#[test]
fn the_back_off_is_the_receivers() {
    let pace = HandoffConfig::default();
    let cfg = RxConfig {
        backoff_ns: 400.0,
        ..RxConfig::new(FS)
    };
    let fx = FixedRxConfig::default();
    let adc = scene(&[SHORT], 700, 111_000, 12.5, 1);
    // With a length, and by the watchdog: the frame's last sample is 8 after the window.
    let length = |_: usize| Some((400usize, Some(plain(SHORT_SYMBOLS))));
    for (say, symbols) in [
        (&length as &dyn Fn(usize) -> Say, SHORT_SYMBOLS),
        (&silent, 1366),
    ] {
        let mut reader = Decoder::new(&always, say);
        let (run, model) = check_cfg(&adc, &cfg, fx, pace, &no_idle, &mut reader, "400 ns");
        let a = run.acquired.iter().flatten().next().copied().unwrap();
        assert_eq!(a.fft_start, 700 + 192 - 8);
        let end = a.fft_start + SIGNAL_END + 80 * symbols + 8;
        assert_eq!(model.holds(), [(a.decided_at, end)]);
        assert_eq!(model.marks, [(end + 8 - 1, HandoffEnd::Packet)]);
    }
}

fn no_idle(_: usize) -> usize {
    0
}

/// The testbenches' pattern: some samples a clock or two late.
fn some_idle(n: usize) -> usize {
    (n * 7) % 3
}

/// T(n) with samples exactly 8 cycles apart: X(n) = 8·(n + 1), K = 6.
fn tick(n: usize) -> usize {
    8 * (n + 1) + 6
}

/// A cheap hash of the cycle: ready in about `of_256` cycles of 256.
fn sometimes(clock: usize, of_256: usize) -> bool {
    (clock.wrapping_mul(0x9e37_79b9).wrapping_add(clock >> 3) >> 7) & 0xff < of_256
}

#[test]
fn the_tick_and_the_estimate_are_the_tables() {
    let t = HandoffTiming::default();
    // Memory 1, rotator 4, buffer 1.
    assert_eq!(t.tick(), 6);
    // 128 addresses, the last word 1, the sums 1, S 2, the numerator 2, the dividers'
    // registers 1, 11 steps, the rounding 1.
    assert_eq!(t.estimate_len(), 128 + 1 + 1 + 2 + 2 + 1 + 11 + 1);
    assert_eq!(t.estimate_len(), 147);
    // One divider: its register and its steps a second time.
    let once = HandoffTiming { dividers: 1, ..t };
    assert_eq!(once.estimate_len(), 147 + 12);
}

#[test]
fn every_periods_beats_hold_and_acquisition_are_the_models() {
    let pace = HandoffConfig::default();
    // The verdict 400 cycles after SIGNAL's last beat, as a length; the same as a failure;
    // none; and one that comes 500 sample periods late, after the frame has ended.
    let length = |_: usize| Some((400, Some(plain(SHORT_SYMBOLS))));
    let failed = |_: usize| Some((400, None));
    let late = |_: usize| Some((4000, Some(plain(SHORT_SYMBOLS))));
    type Scene<'a> = (
        &'a str,
        Vec<(i16, i16)>,
        &'a dyn Fn(usize) -> Say,
        Vec<HandoffEnd>,
    );
    let scenes: [Scene; 5] = [
        (
            "a frame and its length",
            scene(&[SHORT], 700, 900, 12.5, 1),
            &length,
            vec![HandoffEnd::Packet],
        ),
        (
            "a failed SIGNAL",
            scene(&[SHORT], 700, 900, -33.0, 2),
            &failed,
            vec![HandoffEnd::Stopped],
        ),
        (
            "two frames and no verdict",
            scene(&[SHORT, SHORT], 700, 600, -33.0, 8),
            &silent,
            vec![HandoffEnd::Acquisition],
        ),
        (
            "a late verdict",
            scene(&[SHORT], 700, 900, 0.3, 3),
            &late,
            vec![HandoffEnd::Packet],
        ),
        (
            "three frames after a SIFS each",
            scene(&[SHORT, SLOW, SHORT], 500, 320, 20.0, 5),
            &length,
            // The second frame is told the first one's length: its stream ends early,
            // as a packet.
            vec![HandoffEnd::Packet, HandoffEnd::Packet, HandoffEnd::Packet],
        ),
    ];
    let half = |c: usize| c.is_multiple_of(2);
    let random = |c: usize| sometimes(c, 128);
    let seldom = |c: usize| sometimes(c, 40);
    // Ready in the first cycle of each period only, with samples 8 cycles apart.
    let first = |c: usize| c % 8 == 7;
    // Ready in the tick's own cycle only.
    let in_tick = |c: usize| c % 8 == 6;
    let bursts = |c: usize| (c / 700) % 3 != 1;
    let readers: [(&str, &dyn Fn(usize) -> bool); 7] = [
        ("always", &always),
        ("every other cycle", &half),
        ("half the cycles at random", &random),
        ("a cycle in six at random", &seldom),
        ("the first cycle of a period", &first),
        ("the tick's cycle", &in_tick),
        ("700 cycles in 2100 not", &bursts),
    ];
    let idles: [(&str, &dyn Fn(usize) -> usize); 2] = [("no idle", &no_idle), ("idle", &some_idle)];
    let mut seen = Vec::new();
    let mut finished = 0;
    for (scene_name, adc, say, ends) in &scenes {
        for (reader_name, ready) in &readers {
            for (idle_name, idle) in &idles {
                let what = format!("{scene_name}, {reader_name}, {idle_name}");
                let mut reader = Decoder::new(*ready, *say);
                let (run, model) = check(adc, pace, *idle, &mut reader, &what);
                assert!(
                    run.acquired.iter().flatten().count() >= ends.len(),
                    "{what}"
                );
                assert!(run.beats.len() > 200, "{what}: {} beats", run.beats.len());
                if *reader_name == "always" {
                    assert_eq!(&model.ends(), ends, "{what}");
                }
                // A stream read to its end mark, at any pace: the estimate's 128 words
                // and each of its samples once, none past the last.
                if *scene_name == "a frame and its length" && model.ends() == [HandoffEnd::Packet] {
                    let samples = SIGNAL_END + 80 * SHORT_SYMBOLS + B + 8;
                    assert_eq!(run.reads, 128 + samples, "{what}");
                    finished += 1;
                }
                seen.extend(model.ends());
            }
        }
    }
    // Slow readers among those that finished (ahead of them the replay reads to the last).
    assert!(finished >= 8, "{finished}");
    // Every kind of mark but the overrun's (its own test) went through the comparison.
    for e in [
        HandoffEnd::Packet,
        HandoffEnd::Stopped,
        HandoffEnd::Acquisition,
    ] {
        assert!(seen.contains(&e), "{e:?}");
    }
}

#[test]
fn the_estimate_ends_and_the_stream_opens_at_the_clocks_of_the_table() {
    let pace = HandoffConfig::default();
    let fx = FixedRxConfig::default();
    let adc = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let mut reader = Decoder::new(&always, &silent);
    let (run, _) = check(&adc, pace, &no_idle, &mut reader, "no idle");
    assert_eq!(run.estimates.len(), 2);
    for e in &run.estimates {
        let a = run.acquired[e.declared_at].unwrap();
        assert_eq!(e.tick, tick(e.declared_at));
        assert_eq!(e.tick, run.ticks[e.declared_at]);
        assert_eq!(
            Some(e.dc),
            ltf_dc_offset_fixed(&adc, a.fft_start, a.cfo_phase, &fx)
        );
        // The offset of a scene with one: not the zero of a skipped division.
        assert_ne!(e.dc, (0, 0));
        assert_eq!(e.done, e.tick + 147);
        assert_eq!(e.head, e.tick + 153);
        // T(n + 20) + 1 with samples 8 cycles apart: eight cycles to spare.
        assert_eq!(e.opens, Some(e.tick + 161));
        let first = run
            .beats
            .iter()
            .find(|b| b.0 > e.tick && matches!(b.1, HandoffBeat::Sample { first: Some(_), .. }))
            .unwrap();
        assert_eq!(first.0, e.tick + 161);
    }
    // 128 words an estimate and one a sample handed on.
    let samples = run
        .beats
        .iter()
        .filter(|b| matches!(b.1, HandoffBeat::Sample { .. }))
        .count();
    assert!(run.reads >= 2 * 128 + samples);
    assert!(run.reads <= 2 * 128 + samples + 2 * 7);

    // Idle clocks move the stream's first cycle and nothing of the estimate.
    let mut reader = Decoder::new(&always, &silent);
    let (idle, _) = check(&adc, pace, &some_idle, &mut reader, "idle");
    for (e, i) in run.estimates.iter().zip(&idle.estimates) {
        assert_eq!(i.dc, e.dc);
        assert_eq!((i.done - i.tick, i.head - i.tick), (147, 153));
        // Twenty periods with 0, 1 or 2 idle clocks each (7·n mod 3 over 20 samples in a
        // row: 20 or 21 in all).
        let spare = i.opens.unwrap() - i.head;
        assert!((8 + 19..=8 + 21).contains(&spare), "{spare}");
    }
}

#[test]
fn a_ready_reader_gets_a_beat_a_clock_until_it_is_at_the_antenna() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SHORT], 700, 900, 12.5, 1);
    let length = |_: usize| Some((400, Some(plain(SHORT_SYMBOLS))));
    let mut reader = Decoder::new(&always, &length);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "always");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    let (n0, e) = (a.decided_at + 20, run.estimates[0]);
    // The samples waiting in the first period: T1's window to that period's own.
    let waiting = n0 - a.fft_start + 1;
    // Eight a period while seven of them shrink the backlog (`g2_rx_handoff`'s hand count).
    let fast = (waiting - 8) / 7 + 1;
    let rest = waiting + fast - 8 * fast;
    assert!((1..=8).contains(&rest), "{rest}");
    for k in 0..8 * fast + rest {
        assert_eq!(run.beats[k].0, e.tick + 161 + k, "beat {k}: a clock apart");
    }
    for p in 0..fast {
        assert_eq!(model.count[n0 + p], 8);
    }
    assert_eq!(model.count[n0 + fast], rest);
    // At the antenna: each sample in the first cycle of its own period.
    let end = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B + 8;
    let mut k = 8 * fast + rest;
    for n in n0 + fast + 1..end {
        assert_eq!(run.beats[k].0, tick(n) + 1, "sample {n}");
        assert!(matches!(
            run.beats[k].1,
            HandoffBeat::Sample { first: None, .. }
        ));
        k += 1;
    }
    // The end mark in the cycle after the last sample.
    assert_eq!(
        run.beats[k],
        (tick(end - 1) + 2, HandoffBeat::End(HandoffEnd::Packet))
    );
    assert_eq!(run.beats.len(), k + 1);
    // The buffer was full once (the start, before the stream opens) and never more.
    assert_eq!(run.buffered, 7);

    // The same periods as the model at its own rate of eight with a reader of any room.
    let mut rx = RxHandoffFixed::new(RxConfig::new(FS), FixedRxConfig::default(), pace);
    let mut v = 0;
    for (n, &x) in adc.iter().enumerate() {
        while let Some(s) = run.verdicts.get(v).filter(|s| s.before <= n) {
            rx.signal(s.packet, s.verdict);
            v += 1;
        }
        assert_eq!(rx.push(x, usize::MAX).beats.len(), model.count[n], "{n}");
    }
    assert_eq!(v, 1);
}

#[test]
fn a_reader_too_slow_loses_the_stream_and_no_word_is_read_stale() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SHORT], 700, 2600, 12.5, 1);
    // A reader that takes nothing for the first 9000 cycles after the stream could open,
    // then everything: the overrun's mark is what it finds.
    let wake = |c: usize| c > 8 * 2200;
    let mut reader = Decoder::new(&wake, &silent);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "asleep");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    // Lost at the sample 512 after T1's window; the hold with it.
    assert_eq!(model.holds(), [(a.decided_at, a.fft_start + 512)]);
    assert_eq!(model.ends(), [HandoffEnd::Overrun]);
    assert_eq!(run.beats.len(), 1);
    assert_eq!(run.estimates.len(), 1);

    // A beat every 16 cycles, half the antenna's pace: lost in the middle of the stream.
    let slow = |c: usize| c.is_multiple_of(16);
    let mut reader = Decoder::new(&slow, &silent);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "half pace");
    assert_eq!(model.ends(), [HandoffEnd::Overrun]);
    // The reader starts `lag` samples behind and falls half a sample further a period: it
    // is 512 behind after 2·(512 − lag) periods and as many halves of a beat.
    let lag = a.decided_at + 20 - a.fft_start;
    let samples = run.beats.len() - 1;
    assert!(
        (512 - lag..=512 - lag + 2).contains(&samples),
        "{samples}, {lag}"
    );

    // Asleep after 100 beats and awake again in the last period before the loss, for every
    // length of sleep around it: the stream goes on while the model's does, the words
    // behind the buffer are still in the history, and one period more is the overrun.
    let mut edge = None;
    for sleep in 290..=320 {
        let from = tick(a.decided_at + 20) + 100;
        let nap = move |c: usize| c <= from || c > from + 8 * sleep;
        let mut reader = Decoder::new(&nap, &silent);
        let (run, model) = check(&adc, pace, &no_idle, &mut reader, &format!("sleep {sleep}"));
        match model.ends().as_slice() {
            [] => edge = Some((sleep, run.history_spare)),
            [HandoffEnd::Overrun] => break,
            other => panic!("sleep {sleep}: {other:?}"),
        }
    }
    // Awake three cycles after sample n is written and three before its tick, the reader
    // 512 behind it (511 at the tick before, or the stream were lost): the buffer holds
    // seven samples, so the next word read is 505 old of the 511 that are safe. Six to
    // spare, the buffer's depth less one.
    let (sleep, spare) = edge.unwrap();
    assert!((291..320).contains(&sleep), "{sleep}");
    assert_eq!(spare, 6, "sleep {sleep}");
}

#[test]
fn a_verdict_acts_at_the_tick_after_its_strobe() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SLOW], 700, 900, 12.5, 4);
    let mut reader = Decoder::new(&always, &silent);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "no verdict");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);
    let tag = 1u8;
    let end = a.fft_start + SIGNAL_END + 80 * SLOW_SYMBOLS + B;

    // A length in the cycle before a tick is that tick's; in the tick's own cycle, the
    // next one's. Told after the frame's last sample, the hold is down with that tick.
    let m = end + 40;
    for (clock, before) in [(tick(m) - 1, m), (tick(m), m + 1), (tick(m) + 1, m + 1)] {
        let strobes = [(clock, tag, Some(plain(SLOW_SYMBOLS)))];
        let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
        let (run, model) = check(&adc, pace, &no_idle, &mut reader, &format!("at {clock}"));
        assert_eq!(run.verdicts.len(), 1);
        assert_eq!(
            (run.verdicts[0].clock, run.verdicts[0].before),
            (clock, before)
        );
        assert_eq!(model.holds(), [(a.decided_at, before)], "at {clock}");
        assert_eq!(model.marks, [(before, HandoffEnd::Packet)], "at {clock}");
    }
    // In time, the hold lasts to the frame's last sample whatever the strobe's cycle.
    for clock in [tick(end - 300) - 1, tick(end - 300), tick(end - 1) - 1] {
        let strobes = [(clock, tag, Some(plain(SLOW_SYMBOLS)))];
        let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
        let (_, model) = check(&adc, pace, &no_idle, &mut reader, &format!("at {clock}"));
        assert_eq!(model.holds(), [(a.decided_at, end)], "at {clock}");
        assert_eq!(
            model.marks,
            [(end + 8 - 1, HandoffEnd::Packet)],
            "at {clock}"
        );
    }
    // A failure: the hold down and the stream stopped at the tick that takes it.
    let m = a.decided_at + 100;
    let strobes = [(tick(m), tag, None)];
    let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "failed");
    assert_eq!(model.holds(), [(a.decided_at, m + 1)]);
    assert_eq!(model.marks, [(m + 1, HandoffEnd::Stopped)]);

    // Several strobes between two ticks: the first with the open packet's tag counts.
    let short = Some(plain(3));
    let long = Some(plain(SLOW_SYMBOLS));
    let c = tick(m);
    type Strobes = Vec<(usize, u8, Option<SignalVerdict>)>;
    let cases: [(&str, Strobes, usize); 5] = [
        (
            "another packet's, then its own",
            vec![(c, 7, short), (c + 2, tag, long)],
            SLOW_SYMBOLS,
        ),
        (
            "two of its own",
            vec![(c, tag, long), (c + 1, tag, short)],
            SLOW_SYMBOLS,
        ),
        (
            "its own, then a failure",
            vec![(c + 3, tag, long), (c + 7, tag, None)],
            SLOW_SYMBOLS,
        ),
        (
            "a second one periods later",
            vec![(c, tag, long), (c + 80, tag, short)],
            SLOW_SYMBOLS,
        ),
        (
            "another packet's failure",
            vec![(c, 0, None), (c + 100, 2, None)],
            1366,
        ),
    ];
    for (what, strobes, symbols) in cases {
        let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
        let (run, model) = check(&adc, pace, &no_idle, &mut reader, what);
        assert_eq!(run.verdicts.len(), strobes.len(), "{what}");
        let end = (a.fft_start + SIGNAL_END + 80 * symbols + B).min(adc.len());
        assert_eq!(model.holds(), [(a.decided_at, end)], "{what}");
    }
    // A strobe before any packet was opened, with the tag a reset leaves: nothing.
    let strobes = [(tick(100), 0, None), (tick(300) + 3, 0, long)];
    let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "before any packet");
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);
}

/// `adc` with `count` samples from `from` on a rail: I on the upper one and Q on the lower
/// in turn.
fn on_rails(adc: &[(i16, i16)], from: usize, count: usize) -> Vec<(i16, i16)> {
    let mut out = adc.to_vec();
    for (i, v) in out[from..from + count].iter_mut().enumerate() {
        *v = if i % 2 == 0 {
            (2047, v.1)
        } else {
            (v.0, -2048)
        };
    }
    out
}

#[test]
fn a_declaration_inherits_the_patience_only_under_a_hold() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SLOW, SHORT], 700, 320, 12.5, 9);
    let told = |data_symbols: usize| {
        move |k: usize| {
            (k == 0).then_some((
                100,
                Some(SignalVerdict {
                    data_symbols,
                    patient: true,
                }),
            ))
        }
    };
    // The first frame told patient with the longest length: its hold is up when the
    // second is declared, which stays patient; two samples on a rail do nothing.
    let long = told(1366);
    let mut reader = Decoder::new(&always, &long);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "as it is");
    let found: Vec<_> = run.acquired.iter().flatten().copied().collect();
    assert_eq!(found.len(), 2);
    let (a, b) = (found[0], found[1]);
    assert_eq!(model.ends(), [HandoffEnd::Acquisition]);
    let clipped = on_rails(&adc, b.decided_at + 30, 2);
    let mut reader = Decoder::new(&always, &long);
    let (_, model) = check(&clipped, pace, &no_idle, &mut reader, "inherited");
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);
    // Told its true length, the first frame's hold is down before the second is
    // declared: that one is not patient, and the two samples take its hold.
    let true_length = told(SLOW_SYMBOLS);
    let end = a.fft_start + SIGNAL_END + 80 * SLOW_SYMBOLS + B;
    assert!(end < b.decided_at);
    let mut reader = Decoder::new(&always, &true_length);
    let (_, model) = check(&clipped, pace, &no_idle, &mut reader, "not inherited");
    assert_eq!(
        model.holds(),
        [(a.decided_at, end), (b.decided_at, b.decided_at + 31)]
    );
    assert_eq!(model.ends(), [HandoffEnd::Packet]);
}

#[test]
fn a_packet_read_to_its_last_sample_ends_as_a_packet_whatever_comes_first() {
    let pace = HandoffConfig::default();
    let length = |k: usize| (k == 0).then_some((400, Some(plain(SHORT_SYMBOLS))));
    let adc = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let mut reader = Decoder::new(&always, &length);
    let (run, _) = check(&adc, pace, &no_idle, &mut reader, "always");
    let found: Vec<_> = run.acquired.iter().flatten().copied().collect();
    assert_eq!(found.len(), 2);
    let (a, b) = (found[0], found[1]);
    // One past the stream's last sample, which a ready reader takes in the first cycle
    // of its own period.
    let last = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B + 8;
    assert!(last < b.decided_at);
    let taken = tick(last - 1) + 1;

    // The reader leaves before the end mark and is back 20 cycles after the second
    // declaration's tick: the mark it finds is a packet's, not an abort.
    let back = tick(b.decided_at) + 20;
    let away = |c: usize| c <= taken || c > back;
    let mut reader = Decoder::new(&away, &length);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "away for the mark");
    assert_eq!(model.marks, [(b.decided_at + 2, HandoffEnd::Packet)]);
    // Left a sample earlier, the stream is not at its end: the second declaration comes
    // more than a history later, and the same absence has lost it by then.
    assert!(b.decided_at > last + 512);
    let earlier = |c: usize| c < taken || c > back;
    let mut reader = Decoder::new(&earlier, &length);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "away a sample earlier");
    assert_eq!(model.marks, [(b.decided_at + 2, HandoffEnd::Overrun)]);
    assert_eq!(model.holds()[0].1, last - 8);

    // An end mark that waits 600 periods is not an overrun.
    let adc = scene(&[SHORT], 700, 900, -33.0, 8);
    let back = tick(last + 600);
    let asleep = |c: usize| c <= taken || c > back;
    let mut reader = Decoder::new(&asleep, &length);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "asleep at the mark");
    assert_eq!(model.marks, [(last + 600, HandoffEnd::Packet)]);
}

#[test]
fn a_failure_told_during_the_estimate_stops_it_and_the_first_mark_stays() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SHORT, SHORT], 700, 900, 12.5, 6);
    let mut reader = Decoder::new(&always, &silent);
    let (run, _) = check(&adc, pace, &no_idle, &mut reader, "always");
    let found: Vec<_> = run.acquired.iter().flatten().copied().collect();
    assert_eq!(found.len(), 2);
    let (a, b) = (found[0], found[1]);

    // The failure 50 cycles after the declaration's tick, which is six sample periods and
    // two cycles: the seventh tick takes it, 56 words into the estimate.
    let strobes = [(tick(a.decided_at) + 50, 1u8, None)];
    let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "stopped in the estimate");
    assert_eq!(model.marks[0], (a.decided_at + 7, HandoffEnd::Stopped));
    assert_eq!(model.holds()[0], (a.decided_at, a.decided_at + 7));
    assert_eq!(run.estimates.len(), 1);
    assert_eq!(run.estimates[0].declared_at, b.decided_at);
    let samples = run
        .beats
        .iter()
        .filter(|x| matches!(x.1, HandoffBeat::Sample { .. }))
        .count();
    assert_eq!(run.reads, 56 + 128 + samples);

    // The same with a reader that is away until both streams are gone: the second is
    // lost to it 512 samples after its window, and the one mark is the first one's.
    let back = tick(b.fft_start + 600);
    let away = |c: usize| c > back;
    let mut reader = Decoder::new(&away, &silent).with_strobes(&strobes);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "two streams, one mark");
    assert_eq!(
        model.holds(),
        [
            (a.decided_at, a.decided_at + 7),
            (b.decided_at, b.fft_start + 512)
        ]
    );
    assert_eq!(model.ends(), [HandoffEnd::Stopped]);
    assert_eq!(run.beats.len(), 1);
}

#[test]
fn the_release_and_the_patience_act_at_the_tick() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SLOW], 700, 900, 12.5, 4);
    let mut reader = Decoder::new(&always, &silent);
    let (run, _) = check(&adc, pace, &some_idle, &mut reader, "as it is");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    let end = a.fft_start + SIGNAL_END + 80 * SLOW_SYMBOLS + B;
    let rail = on_rails;
    // Two samples on a rail under a hold that is not patient: down with the second.
    let at = a.decided_at + 150;
    let clipped = rail(&adc, at, 2);
    let mut reader = Decoder::new(&always, &silent);
    let (_, model) = check(&clipped, pace, &some_idle, &mut reader, "two of four");
    assert_eq!(model.holds(), [(a.decided_at, at + 1)]);
    // Told patient before: the two do nothing, 24 in a row do, with the 24th.
    let patient = |_: usize| {
        Some((
            100,
            Some(SignalVerdict {
                data_symbols: SLOW_SYMBOLS,
                patient: true,
            }),
        ))
    };
    let at = a.fft_start + SIGNAL_END + 400;
    let clipped = rail(&adc, at, 2);
    let mut reader = Decoder::new(&always, &patient);
    let (_, model) = check(&clipped, pace, &some_idle, &mut reader, "patient, two");
    assert_eq!(model.holds(), [(a.decided_at, end)]);
    let clipped = rail(&adc, at, 30);
    let mut reader = Decoder::new(&always, &patient);
    let (run, model) = check(&clipped, pace, &some_idle, &mut reader, "patient, thirty");
    assert_eq!(model.holds(), [(a.decided_at, at + 23)]);
    // The stream is not touched by the release: it ends as the packet it is.
    assert_eq!(model.ends(), [HandoffEnd::Packet]);
    assert_eq!(run.verdicts.len(), 1);

    // The windows' lengths. Two of four: a sample on a rail and the third after it; with
    // one more between them, nothing.
    let apart = |gap: usize| on_rails(&on_rails(&adc, at, 1), at + gap, 1);
    let told = |_: usize| Some((100, Some(plain(SLOW_SYMBOLS))));
    let mut reader = Decoder::new(&always, &told);
    let (_, model) = check(&apart(3), pace, &no_idle, &mut reader, "two in four");
    assert_eq!(model.holds(), [(a.decided_at, at + 3)]);
    let mut reader = Decoder::new(&always, &told);
    let (_, model) = check(&apart(4), pace, &no_idle, &mut reader, "two in five");
    assert_eq!(model.holds(), [(a.decided_at, end)]);
    // (A verdict that is not patient leaves the first rule: the two above were told one.)
    // 24 of 32: one sample, eight that are not, and 23 more; with nine between, nothing.
    let spread = |gap: usize| on_rails(&on_rails(&adc, at, 1), at + 1 + gap, 23);
    let mut reader = Decoder::new(&always, &patient);
    let (_, model) = check(&spread(8), pace, &no_idle, &mut reader, "24 in 32");
    assert_eq!(model.holds(), [(a.decided_at, at + 31)]);
    let mut reader = Decoder::new(&always, &patient);
    let (_, model) = check(&spread(9), pace, &no_idle, &mut reader, "24 in 33");
    assert_eq!(model.holds(), [(a.decided_at, end)]);
}

#[test]
fn without_a_verdict_the_watchdog_ends_the_hold_and_the_stream() {
    let pace = HandoffConfig::default();
    // A short frame and 5.6 ms of noise: the longest PPDU's 1366 data symbols.
    let adc = scene(&[SHORT], 700, 111_000, 12.5, 1);
    let mut reader = Decoder::new(&always, &silent);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "no verdict");
    let found: Vec<_> = run.acquired.iter().flatten().copied().collect();
    assert_eq!(found.len(), 1);
    let end = found[0].fft_start + SIGNAL_END + 80 * 1366 + B;
    assert_eq!(model.holds(), [(found[0].decided_at, end)]);
    assert_eq!(model.marks, [(end + 8 - 1, HandoffEnd::Packet)]);
    // A length above the longest is the longest.
    let too_long = |_: usize| Some((100, Some(plain(5000))));
    let mut reader = Decoder::new(&always, &too_long);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "5000 symbols");
    assert_eq!(model.holds(), [(found[0].decided_at, end)]);
    assert_eq!(model.marks, [(end + 8 - 1, HandoffEnd::Packet)]);
}

#[test]
fn a_verdict_is_taken_before_the_declaration_of_its_tick() {
    let pace = HandoffConfig::default();
    let adc = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let mut reader = Decoder::new(&always, &silent);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "no verdict");
    let found: Vec<_> = run.acquired.iter().flatten().copied().collect();
    assert_eq!(found.len(), 2);
    let (a, b) = (found[0], found[1]);
    assert_eq!(model.marks, [(b.decided_at, HandoffEnd::Acquisition)]);
    // The first frame's length in the cycle before the tick that declares the second:
    // the first stream has been read past that end, and is closed as a packet.
    let strobes = [(tick(b.decided_at) - 1, 1u8, Some(plain(SHORT_SYMBOLS)))];
    let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
    let (run, model) = check(&adc, pace, &no_idle, &mut reader, "the cycle before");
    assert_eq!(run.verdicts[0].before, b.decided_at);
    assert_eq!(model.marks, [(b.decided_at, HandoffEnd::Packet)]);
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);
    // A cycle later it is the next tick's, the tag is no longer the open packet's, and
    // the new packet is not touched by it.
    let strobes = [(tick(b.decided_at), 1u8, Some(plain(SHORT_SYMBOLS)))];
    let mut reader = Decoder::new(&always, &silent).with_strobes(&strobes);
    let (_, model) = check(&adc, pace, &no_idle, &mut reader, "the tick's cycle");
    assert_eq!(model.marks, [(b.decided_at, HandoffEnd::Acquisition)]);
    assert_eq!(model.holds(), [(a.decided_at, adc.len())]);
}

#[test]
fn the_word_handed_on_is_the_acquisitions_with_a_coarse_weight() {
    let pace = HandoffConfig::default();
    let fx = FixedRxConfig {
        coarse_weight_64: 15,
        ..FixedRxConfig::default()
    };
    let adc = scene(&[SHORT], 700, 900, -33.0, 2);
    let length = |_: usize| Some((400, Some(plain(SHORT_SYMBOLS))));
    let mut reader = Decoder::new(&half, &length);
    let (run, model) = check_fx(&adc, fx, pace, &some_idle, &mut reader, "15/64");
    let a = run.acquired.iter().flatten().next().copied().unwrap();
    // Neither estimate is the word: the stream's first beat and the offset use the word.
    assert!(a.cfo_phase != a.fine_phase && a.cfo_phase != a.coarse_phase);
    assert_eq!(model.ends(), [HandoffEnd::Packet]);
    assert_eq!(
        Some(run.estimates[0].dc),
        ltf_dc_offset_fixed(&adc, a.fft_start, a.cfo_phase, &fx)
    );
}

fn half(c: usize) -> bool {
    c.is_multiple_of(2)
}

#[test]
fn what_cannot_be_built_is_refused() {
    let fx = FixedRxConfig::default();
    let cfg = RxConfig::new(FS);
    let adc = scene(&[SHORT], 700, 900, 12.5, 1);
    let run = |pace: HandoffConfig, t: HandoffTiming, ready: &dyn Fn(usize) -> bool| {
        let mut reader = Decoder::new(ready, &silent);
        run_handoff(&adc, &cfg, &fx, &pace, &t, &no_idle, &mut reader).inspect(|run| {
            held_to_the_model(&adc, &cfg, fx, pace, run, "a variant that fits");
        })
    };
    let (pace, t) = (HandoffConfig::default(), HandoffTiming::default());
    assert!(run(pace, t, &always).is_ok());
    let late = Err(ScheduleError::Late);

    // One divider used twice: 159 cycles to the offset, 165 to the first sample, 161 given.
    let once = HandoffTiming { dividers: 1, ..t };
    assert_eq!(run(pace, once, &always).map(|_| ()), late);
    // ... which a start delay of 21 periods gives.
    let later = HandoffConfig {
        start_delay: 21,
        ..pace
    };
    let e = run(later, once, &always).unwrap().estimates[0];
    assert_eq!((e.head, e.opens), (e.tick + 165, Some(e.tick + 169)));
    // Two dividers and 19 periods: 153 given and 153 needed, the first beat in that cycle.
    let sooner = HandoffConfig {
        start_delay: 19,
        ..pace
    };
    let exact = run(sooner, t, &always).unwrap();
    let e = exact.estimates[0];
    assert_eq!((e.head, e.opens), (e.tick + 153, Some(e.tick + 153)));
    assert_eq!(exact.beats[0].0, e.tick + 153);
    // 18 periods: 145 given.
    let sooner = HandoffConfig {
        start_delay: 18,
        ..pace
    };
    assert_eq!(run(sooner, t, &always).map(|_| ()), late);
    // Seven clocks a sample: 141 given.
    let r7 = HandoffTiming {
        clocks_per_sample: 7,
        ..t
    };
    assert_eq!(run(pace, r7, &always).map(|_| ()), late);

    // A buffer of six with a registered count: a ready reader misses a beat in seven.
    let shallow = HandoffTiming {
        buffer_depth: 6,
        ..t
    };
    assert_eq!(run(pace, shallow, &always).map(|_| ()), late);
    // Six do with a count that is free in the cycle of the beat ...
    let comb = HandoffTiming {
        credit: 0,
        ..shallow
    };
    assert_eq!(run(pace, comb, &always).unwrap().buffered, 6);
    // ... and five do not.
    let five = HandoffTiming {
        buffer_depth: 5,
        ..comb
    };
    assert_eq!(run(pace, five, &always).map(|_| ()), late);
    // A reader that is never there makes no demand on the buffer.
    let never = |_: usize| false;
    assert!(run(pace, shallow, &never).is_ok());
    // An offset that is late is late whether the reader is there to see it or not.
    assert_eq!(run(pace, once, &never).map(|_| ()), late);

    // A history of 256 words: T1's window is some 275 samples old at the declaration.
    let small = HandoffTiming {
        history_depth: 256,
        ..t
    };
    assert!(matches!(
        run(pace, small, &always),
        Err(ScheduleError::Stale {
            store: Store::Adc,
            ..
        })
    ));
    // A rotator of one stage would put the tick before the front's acq_valid.
    let quick = HandoffTiming { rotator: 1, ..t };
    assert_eq!(quick.tick(), 3);
    // Two stages put it in acq_valid's own cycle, which is in time.
    let level = HandoffTiming { rotator: 2, ..t };
    assert_eq!((level.tick(), level.acq_latency), (4, 4));
    assert!(run(pace, level, &always).is_ok());
    assert_eq!(run(pace, quick, &always).map(|_| ()), late);
}

/// A field of 128 samples from a generator, each component within `amp` of `mid`.
fn field(rng: &mut Rng, mid: (i32, i32), amp: i32) -> Vec<(i16, i16)> {
    let mut draw = |m: i32| {
        let r = (rng.next_u64() % (2 * amp as u64 + 1)) as i32 - amp;
        (m + r).clamp(-2048, 2047) as i16
    };
    (0..128).map(|_| (draw(mid.0), draw(mid.1))).collect()
}

#[test]
fn the_offset_of_two_sums_and_two_dividers_is_the_models() {
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(77);
    // The CFO words: none, the edge of the skipped division (704 with the default table)
    // and its neighbours, both signs, the largest, and some in between.
    let mut words: Vec<i64> = vec![0, 1, -1, 703, 704, 705, -703, -704, -705, 5000, -123_456];
    words.extend([(1 << 21) - 1, -(1 << 21), 1 << 15, -(1 << 16) - 1]);
    for _ in 0..40 {
        words.push((rng.next_u64() % (1 << 22)) as i64 - (1 << 21));
    }
    let (mut skipped, mut high, mut low, mut cases) = (0, 0, 0, 0);
    for &word in &words {
        for k in 0..60 {
            // Noise about an offset; full-scale noise; and two symbols at opposite rails,
            // which no offset explains: the quotient leaves the ADC's range.
            let f = match k % 4 {
                0 => field(&mut rng, (300, -150), 600),
                1 => field(&mut rng, (0, 0), 2048),
                2 => field(&mut rng, (-2000, 1990), 60),
                _ => {
                    let mut f = field(&mut rng, (-2040, 2040), 8);
                    for v in &mut f[64..] {
                        *v = (-v.0 - 1, -v.1);
                    }
                    f
                }
            };
            let want = ltf_dc_offset_fixed(&f, 0, word, &fx).unwrap();
            assert_eq!(
                dc_offset_serial(&f, word, &fx),
                Some(want),
                "word {word}, {k}"
            );
            skipped += usize::from(want == (0, 0) && word.abs() < 704);
            high += usize::from(want.0 == 2047 || want.1 == 2047);
            low += usize::from(want.0 == -2048 || want.1 == -2048);
            cases += 1;
        }
    }
    assert!(
        cases >= 3000 && skipped >= 300 && high >= 100 && low >= 100,
        "{skipped} {high} {low}"
    );

    // A constant is its own offset, to the bit; and ties go to the even number. With both
    // symbols' sums 64·c + 32 the quotient is c + 1/2 exactly: S = (A − e)·(64·c + 32).
    // (Not the largest word: 64 of it are a whole turn less 64 units, e is A, and the
    // division is skipped as for no CFO at all.)
    assert_eq!(
        dc_offset_serial(&vec![(9, 9); 128], (1 << 21) - 1, &fx),
        Some((0, 0))
    );
    for word in [704i64, -9000, 77_777, (1 << 21) - 12_345] {
        for c in [-2048i16, -2047, -5, -4, -1, 0, 1, 6, 7, 2046] {
            let flat = vec![(c, -1 - c); 128];
            assert_eq!(
                dc_offset_serial(&flat, word, &fx),
                Some((i64::from(c), i64::from(-1 - c))),
                "word {word}, {c}"
            );
            let mut tie = flat.clone();
            for at in [3, 64 + 40] {
                tie[at].0 += 32;
                tie[at].1 -= 32;
            }
            let even = |v: i64| if v % 2 == 0 { v } else { v + 1 };
            let want = (even(i64::from(c)), even(i64::from(-1 - c) - 1));
            assert_eq!(ltf_dc_offset_fixed(&tie, 0, word, &fx), Some(want));
            assert_eq!(
                dc_offset_serial(&tie, word, &fx),
                Some(want),
                "tie {word}, {c}"
            );
        }
    }

    // Not 128 samples, or a configuration the front refuses: nothing.
    assert_eq!(dc_offset_serial(&vec![(1, 1); 127], 5000, &fx), None);
    assert_eq!(dc_offset_serial(&vec![(1, 1); 129], 5000, &fx), None);
    let bad = FixedRxConfig {
        nco_lut_bits: 1,
        ..fx
    };
    assert!(!bad.is_valid());
    assert_eq!(dc_offset_serial(&vec![(1, 1); 128], 5000, &bad), None);
}
