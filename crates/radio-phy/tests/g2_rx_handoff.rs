//! Gate for the streaming hand-off (R0-08d, ADR 0008 decisions 1 to 3): `RxHandoffFixed`,
//! the integer front with the second history, the replay and the hold.
//!
//! References: the stream's samples are [`handoff_fixed`] of the ADC's samples from the FFT
//! window of T1 on (gated on its own in `g2_rx_front.rs`, by hand and against the float
//! chain); the pace, the hold and the end marks are worked by hand from the ADR's rules
//! (G0 in `docs/gates/rx_front.md`).

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    acquisitions_fixed, cordic_angle, cordic_table, handoff_fixed, ltf_taps, nco_rotator,
    nco_table, to_adc, FixedAcquisition, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd,
    RxHandoffFixed, ADC_FULL_SCALE, ADC_HISTORY, HANDOFF_TAIL, IQ, MAX_DATA_SYMBOLS,
};
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{clock_ratio, decode_ppdu_fixed, FixedOfdmConfig, FixedTracking};

type C = (f64, f64);

const FS: f64 = 20e6;
const CARRIER: f64 = 5.8e9;
/// Samples from the FFT window of T1 to the end of SIGNAL's window.
const SIGNAL_END: usize = 128 + 80;
/// The front's back-off at 20 MS/s (`RxConfig::backoff_ns` of 200 ns).
const B: usize = 4;

/// Frames of random octets at −12 dBFS and 30 dB behind `lead` samples of noise, `gap`
/// samples of noise after each, the transmitter's clock off by `ppm`, on a DC offset of
/// −20 dBFS at the ADC.
fn scene(
    frames: &[(Rate, usize)],
    lead: usize,
    gap: usize,
    ppm: f64,
    seed: u64,
) -> (Vec<(i16, i16)>, Vec<Vec<u8>>) {
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(seed);
    let mut x = vec![(0.0, 0.0); lead];
    let mut psdus = Vec::new();
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
        psdus.push(psdu);
    }
    let x = if ppm == 0.0 { x } else { resample(&x, ppm) };
    let x = frequency_offset(&x, ppm * 1e-6 * CARRIER, FS, 2.0);
    let x = awgn(&x, 30.0, p, &mut rng);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
    let dc = 10f64.powf(-20.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
    let x: Vec<C> = x.iter().map(|s| (s.0 - 0.8 * dc, s.1 + 0.6 * dc)).collect();
    (to_adc(&x, gain), psdus)
}

/// What a run of the streaming front gave: per pushed sample the acquisition and the hold,
/// and every beat with the index of the push that handed it.
#[derive(Default)]
struct Log {
    acquired: Vec<FixedAcquisition>,
    hold: Vec<bool>,
    beats: Vec<(usize, HandoffBeat)>,
    /// The hold right after each verdict, with the push it followed.
    after_verdict: Vec<(usize, bool)>,
}

impl Log {
    /// The beats without their push indices.
    fn stream(&self) -> Vec<HandoffBeat> {
        self.beats.iter().map(|b| b.1).collect()
    }

    /// The samples of the beats `from..to`.
    fn samples(&self, from: usize, to: usize) -> Vec<IQ> {
        self.beats[from..to]
            .iter()
            .map(|b| match b.1 {
                HandoffBeat::Sample { z, .. } => z,
                HandoffBeat::End(e) => panic!("an end mark {e:?} among the samples"),
            })
            .collect()
    }

    /// Beats handed by push `n`.
    fn at(&self, n: usize) -> usize {
        self.beats.iter().filter(|b| b.0 == n).count()
    }

    /// The hold over the pushes, as (first, one past the last) of each stretch it was up.
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
}

/// Pushes `adc` through a streaming front. `room(n)` is what the reader takes in the period
/// of sample n; `verdict(n, log)` is the decoder's call after that push, if it makes one.
fn run(
    adc: &[(i16, i16)],
    fx: FixedRxConfig,
    pace: HandoffConfig,
    room: impl Fn(usize) -> usize,
    verdict: impl Fn(usize, &Log) -> Option<Option<usize>>,
) -> Log {
    run_with(RxConfig::new(FS), adc, fx, pace, room, verdict)
}

fn run_with(
    cfg: RxConfig,
    adc: &[(i16, i16)],
    fx: FixedRxConfig,
    pace: HandoffConfig,
    room: impl Fn(usize) -> usize,
    verdict: impl Fn(usize, &Log) -> Option<Option<usize>>,
) -> Log {
    let mut rx = RxHandoffFixed::new(cfg, fx, pace);
    let mut log = Log::default();
    for (n, &x) in adc.iter().enumerate() {
        let step = rx.push(x, room(n));
        assert!(step.beats.len() <= room(n).min(pace.rate), "push {n}");
        assert_eq!(step.hold, rx.hold(), "push {n}");
        log.acquired.extend(step.acquired);
        log.hold.push(step.hold);
        log.beats.extend(step.beats.into_iter().map(|b| (n, b)));
        if let Some(v) = verdict(n, &log) {
            rx.signal(v);
            log.after_verdict.push((n, rx.hold()));
        }
    }
    log
}

fn reference(adc: &[(i16, i16)], a: &FixedAcquisition, fx: &FixedRxConfig) -> Vec<IQ> {
    handoff_fixed(&adc[a.fft_start..], 0, a.cfo_phase, fx).unwrap()
}

/// 100 octets at 54 Mb/s: 16 + 800 + 6 bits in symbols of 216, four data symbols.
const SHORT: (Rate, usize) = (Rate::R54, 100);
const SHORT_SYMBOLS: usize = 4;

/// The decoder's verdict `after` samples past SIGNAL's window of the first acquisition.
fn verdict_at(after: usize, v: Option<usize>) -> impl Fn(usize, &Log) -> Option<Option<usize>> {
    move |n, log| {
        let a = log.acquired.first()?;
        (n == a.fft_start + SIGNAL_END + after).then_some(v)
    }
}

#[test]
fn the_stream_is_the_hand_off_from_t1_and_the_hold_lasts_the_packet() {
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    assert_eq!((pace.rate, pace.start_delay), (8, 20));
    for (seed, ppm) in [(1u64, 12.5), (2, -33.0), (3, 0.3)] {
        let (adc, _) = scene(&[SHORT], 700, 900, ppm, seed);
        let log = run(
            &adc,
            fx,
            pace,
            |_| usize::MAX,
            verdict_at(180, Some(SHORT_SYMBOLS)),
        );
        assert_eq!(log.acquired.len(), 1, "seed {seed}");
        let a = log.acquired[0];
        // T1's window: 700 of lead, 192 of the preamble before T1, the back-off.
        assert_eq!(a.fft_start, 700 + 192 - B, "seed {seed}");

        // The hold: from the push that declares to the frame's last sample, which is the
        // back-off after the last window's end.
        let end = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B;
        assert_eq!(log.holds(), [(a.decided_at, end)], "seed {seed}");

        // The stream: the hand-off's samples from T1's window to the tail after the hold's
        // end, the CFO word on the first, one end mark.
        let count = end - a.fft_start + HANDOFF_TAIL;
        assert_eq!(log.beats.len(), count + 1, "seed {seed}");
        assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
        for (k, b) in log.beats[..count].iter().enumerate() {
            let HandoffBeat::Sample { first, .. } = b.1 else {
                panic!()
            };
            assert_eq!(
                first,
                (k == 0).then_some(a.cfo_phase),
                "seed {seed} beat {k}"
            );
        }
        assert_eq!(log.beats[0].0, a.decided_at + 20);
        // The last sample is handed in its own period, the mark with it.
        assert_eq!(
            log.beats[count],
            (
                a.fft_start + count - 1,
                HandoffBeat::End(HandoffEnd::Packet)
            )
        );
        assert_eq!(log.beats[count - 1].0, a.fft_start + count - 1);
    }
}

#[test]
fn the_replay_catches_up_at_eight_samples_a_period() {
    // ADR 0008, decision 3: the backlog shrinks by seven samples in every period. With L
    // samples waiting at the first push (the newest included), push j of the replay finds
    // L − 7j: eight are handed while that is at least eight, then the rest, then one a
    // period at the antenna's pace.
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 900, 12.5, 1);
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        verdict_at(180, Some(SHORT_SYMBOLS)),
    );
    let a = log.acquired[0];
    let open = a.decided_at + 20;
    let backlog = open - a.fft_start + 1;
    // The declaration is 270 to 290 samples after T1's window began (ADR 0008: T1 + 288 at
    // the latest), so the reader is at the antenna some 43 periods after the replay starts.
    assert!((290..=315).contains(&backlog), "{backlog}");
    let full = (backlog - 8) / 7 + 1;
    let rest = backlog - 7 * full;
    assert!((1..8).contains(&rest));
    for j in 0..full {
        assert_eq!(log.at(open + j), 8, "push {j}");
    }
    assert_eq!(log.at(open + full), rest);
    assert!((42..=45).contains(&(full + 1)), "{full}");
    let last = log.beats.last().unwrap().0;
    for n in open + full + 1..last {
        assert_eq!(log.at(n), 1, "push {n}");
    }
    // The last push hands the last sample and the end mark.
    assert_eq!(log.at(last), 2);
    assert_eq!(log.at(open - 1), 0);

    // A slower clock: three beats a period.
    let slow = HandoffConfig {
        rate: 3,
        ..HandoffConfig::default()
    };
    let log3 = run(
        &adc,
        fx,
        slow,
        |_| usize::MAX,
        verdict_at(180, Some(SHORT_SYMBOLS)),
    );
    let full3 = (backlog - 3) / 2 + 1;
    for j in 0..full3 {
        assert_eq!(log3.at(open + j), 3, "push {j}");
    }
    assert_eq!(log3.at(open + full3), backlog - 2 * full3);
    assert_eq!(log3.stream(), log.stream());
}

#[test]
fn the_readers_pace_does_not_change_the_stream() {
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 900, -33.0, 2);
    let verdict = || verdict_at(180, Some(SHORT_SYMBOLS));
    let free = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        verdict(),
    );
    // A reader that takes five beats every third period (the run asserts no push hands
    // more than its room), and one that takes one beat a period with a start delay of 100.
    let bursty = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| if n % 3 == 0 { 5 } else { 0 },
        verdict(),
    );
    assert_eq!(bursty.stream(), free.stream());
    assert_eq!(bursty.hold, free.hold);
    let late = HandoffConfig {
        start_delay: 100,
        ..HandoffConfig::default()
    };
    let single = run(&adc, fx, late, |_| 1, verdict());
    assert_eq!(single.stream(), free.stream());
    assert_eq!(single.hold, free.hold);
    assert_eq!(single.beats[0].0, free.acquired[0].decided_at + 100);
    // One beat a period never catches up: the end mark waits for the last sample.
    let count = free.beats.len() - 1;
    assert_eq!(single.beats[count - 1].0, single.beats[0].0 + count - 1);
    assert_eq!(single.beats[count].0, single.beats[0].0 + count);
}

#[test]
fn a_failed_signal_drops_the_hold_and_ends_the_stream_and_the_front_goes_on() {
    let fx = FixedRxConfig::default();
    // Two frames, 600 samples apart; the decoder rejects the first one's SIGNAL.
    let (adc, _) = scene(&[SHORT, SHORT], 700, 600, 12.5, 4);
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        verdict_at(180, None),
    );
    assert_eq!(log.acquired.len(), 2);
    let (a, b) = (log.acquired[0], log.acquired[1]);
    let stop = a.fft_start + SIGNAL_END + 180;
    // The hold is down in the period after the verdict; the second frame's lasts to the
    // watchdog, beyond this stream.
    assert_eq!(
        log.holds(),
        [(a.decided_at, stop + 1), (b.decided_at, adc.len())]
    );
    // The reader had caught up: every sample to the verdict's, then the mark, alone, in the
    // next period.
    let count = stop - a.fft_start + 1;
    assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
    assert_eq!(
        log.beats[count],
        (stop + 1, HandoffBeat::End(HandoffEnd::Stopped))
    );
    assert_eq!(log.at(stop + 1), 1);
    // Nothing until the second frame's replay, which is its own stream.
    assert_eq!(
        log.beats[count + 1],
        (
            b.decided_at + 20,
            HandoffBeat::Sample {
                z: reference(&adc, &b, &fx)[0],
                first: Some(b.cfo_phase)
            }
        )
    );
    let rest = log.beats.len() - count - 1;
    assert_eq!(rest, adc.len() - b.fft_start);
    assert_eq!(
        log.samples(count + 1, log.beats.len()),
        reference(&adc, &b, &fx)
    );
}

#[test]
fn without_a_verdict_the_watchdog_ends_the_hold_at_the_longest_ppdu() {
    // 4095 octets at 6 Mb/s: 16 + 32 760 + 6 bits in symbols of 24 (IEEE 802.11-2020
    // §17.3.5.4, Table 17-4), 1366 data symbols.
    assert_eq!(MAX_DATA_SYMBOLS, (16usize + 8 * 4095 + 6).div_ceil(24));
    assert_eq!(MAX_DATA_SYMBOLS, 1366);
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 111_000, 12.5, 5);
    let longest = SIGNAL_END + 80 * MAX_DATA_SYMBOLS + B;
    for v in [None, Some(Some(5000)), Some(Some(MAX_DATA_SYMBOLS))] {
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            |n, log| {
                let a = log.acquired.first()?;
                (n == a.fft_start + 400).then_some(v).flatten()
            },
        );
        assert_eq!(log.acquired.len(), 1);
        let a = log.acquired[0];
        assert_eq!(log.holds(), [(a.decided_at, a.fft_start + longest)]);
        let count = longest + HANDOFF_TAIL;
        assert_eq!(log.beats.len(), count + 1);
        assert_eq!(
            log.beats[count],
            (
                a.fft_start + count - 1,
                HandoffBeat::End(HandoffEnd::Packet)
            )
        );
        if v.is_none() {
            assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
        }
    }
}

#[test]
fn a_verdict_is_taken_once_and_a_late_one_ends_at_once() {
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 3000, 12.5, 6);
    // The verdict comes 2000 samples after T1, long after the four symbols it announces;
    // a second one, and one with no packet open, change nothing.
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        |n, log| {
            if n == 100 {
                return Some(Some(7));
            }
            let a = log.acquired.first()?;
            match n - a.fft_start {
                2000 => Some(Some(SHORT_SYMBOLS)),
                2001 => Some(Some(900)),
                2500 => Some(None),
                _ => None,
            }
        },
    );
    assert_eq!(log.acquired.len(), 1);
    let a = log.acquired[0];
    assert_eq!(log.holds(), [(a.decided_at, a.fft_start + 2001)]);
    // The hold is down as the late verdict is given, not a period later; the calls after
    // it and the one before any packet find none.
    let f = a.fft_start;
    assert_eq!(
        log.after_verdict,
        [
            (100, false),
            (f + 2000, false),
            (f + 2001, false),
            (f + 2500, false)
        ]
    );
    // The samples to the verdict's had been handed; the mark follows alone.
    assert_eq!(log.beats.len(), 2002);
    assert_eq!(log.samples(0, 2001), reference(&adc, &a, &fx)[..2001]);
    assert_eq!(
        log.beats[2001],
        (a.fft_start + 2001, HandoffBeat::End(HandoffEnd::Packet))
    );

    // A second verdict before the first one's end does not move it either.
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        |n, log| {
            let a = log.acquired.first()?;
            match n - a.fft_start {
                300 => Some(Some(SHORT_SYMBOLS)),
                301 => Some(Some(20)),
                302 => Some(None),
                _ => None,
            }
        },
    );
    let end = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B;
    assert_eq!(log.holds(), [(a.decided_at, end)]);
    assert_eq!(log.beats.len(), end - a.fft_start + HANDOFF_TAIL + 1);
    assert_eq!(
        log.beats.last().unwrap().1,
        HandoffBeat::End(HandoffEnd::Packet)
    );
    assert_eq!(
        log.after_verdict,
        [(f + 300, true), (f + 301, true), (f + 302, true)]
    );

    // "At once if that is past": a verdict that follows the push of the frame's last
    // sample finds the hold down as it is given; one a sample earlier leaves it up for
    // that last sample.
    for (at, down) in [(end - 2, false), (end - 1, true)] {
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            |n, _| (n == at).then_some(Some(SHORT_SYMBOLS)),
        );
        assert_eq!(log.after_verdict, [(at, !down)]);
        assert_eq!(log.holds(), [(a.decided_at, end)]);
    }
}

#[test]
fn a_reader_too_slow_loses_the_stream() {
    assert_eq!(ADC_HISTORY, 512);
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 3000, 12.5, 7);
    let verdict = || verdict_at(180, Some(200));

    // A reader that never takes a beat, and a replay that starts too late: the first sample
    // is overwritten 512 periods after its own and the hold goes down with it. The reader
    // with room gets the abort mark in that period, alone: it has seen no sample.
    for pace in [
        HandoffConfig::default(),
        HandoffConfig {
            rate: 0,
            ..HandoffConfig::default()
        },
        HandoffConfig {
            start_delay: 5000,
            ..HandoffConfig::default()
        },
    ] {
        let room = if pace.rate == 8 && pace.start_delay == 20 {
            0
        } else {
            usize::MAX
        };
        let log = run(&adc, fx, pace, |_| room, verdict());
        let a = log.acquired[0];
        let lost = a.fft_start + ADC_HISTORY;
        assert_eq!(log.holds(), [(a.decided_at, lost)]);
        if pace.start_delay == 5000 {
            assert_eq!(log.beats, [(lost, HandoffBeat::End(HandoffEnd::Overrun))]);
        } else {
            assert!(log.beats.is_empty());
        }
    }

    // A reader at half the antenna's pace, one beat every second period from the replay's
    // first: before push n it has taken ⌈(n − open)/2⌉ samples, and the next one is lost
    // when it is 512 behind the newest.
    let probe = run(&adc, fx, HandoffConfig::default(), |_| 0, verdict());
    let a = probe.acquired[0];
    let open = a.decided_at + 20;
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| usize::from(n >= open && (n - open) % 2 == 0),
        verdict(),
    );
    let lost = (open..)
        .find(|n| a.fft_start + (n - open).div_ceil(2) + ADC_HISTORY <= *n)
        .unwrap();
    let taken = (lost - open).div_ceil(2);
    assert!(taken > 150 && lost < a.fft_start + SIGNAL_END + 80 * 200);
    assert_eq!(log.holds(), [(a.decided_at, lost)]);
    assert_eq!(log.beats.len(), taken + 1);
    assert_eq!(log.samples(0, taken), reference(&adc, &a, &fx)[..taken]);
    // The abort mark takes a beat of its own, in the reader's next period with room.
    let mark = if (lost - open) % 2 == 0 {
        lost
    } else {
        lost + 1
    };
    assert_eq!(
        log.beats[taken],
        (mark, HandoffBeat::End(HandoffEnd::Overrun))
    );

    // A reader that took the stream's last sample and has no room for the end mark for
    // 700 periods has lost nothing: the mark it gets is the packet's end.
    let last = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B + HANDOFF_TAIL - 1;
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| match n {
            n if n < last => usize::MAX,
            n if n == last => 1,
            n if n <= last + 700 => 0,
            _ => usize::MAX,
        },
        verdict_at(180, Some(SHORT_SYMBOLS)),
    );
    assert_eq!(log.beats.len(), last - a.fft_start + 2);
    assert_eq!(
        log.beats.last(),
        Some(&(last + 701, HandoffBeat::End(HandoffEnd::Packet)))
    );

    // The same reader given one beat more, once, in the period before the loss, is not
    // lost there.
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| {
            if n == lost - 1 {
                2
            } else {
                usize::from(n >= open && (n - open) % 2 == 0)
            }
        },
        verdict(),
    );
    assert!(log.holds()[0].1 > lost);
}

#[test]
fn a_new_declaration_aborts_the_open_stream_and_opens_its_own() {
    let fx = FixedRxConfig::default();
    // Two frames 600 samples apart and no verdict: the first one's hold and stream are
    // still open when the second is declared.
    let (adc, _) = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        |_, _| None,
    );
    assert_eq!(log.acquired.len(), 2);
    let (a, b) = (log.acquired[0], log.acquired[1]);
    // One hold, never dropped between the two.
    assert_eq!(log.holds(), [(a.decided_at, adc.len())]);
    // The first stream to the sample of the second declaration's period, then the abort
    // mark in that period; the second stream starts 20 periods later with its own word.
    let count = b.decided_at - a.fft_start;
    assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
    assert_eq!(
        log.beats[count],
        (b.decided_at, HandoffBeat::End(HandoffEnd::Acquisition))
    );
    assert_eq!(log.at(b.decided_at), 1);
    assert_eq!(
        log.beats[count + 1],
        (
            b.decided_at + 20,
            HandoffBeat::Sample {
                z: reference(&adc, &b, &fx)[0],
                first: Some(b.cfo_phase)
            }
        )
    );
    assert_eq!(
        log.samples(count + 1, log.beats.len()),
        reference(&adc, &b, &fx)
    );

    // A reader with no room in the period of the second declaration gets the mark first
    // in its next one, before any sample of the second stream.
    let blocked = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| {
            if (b.decided_at..b.decided_at + 40).contains(&n) {
                0
            } else {
                usize::MAX
            }
        },
        |_, _| None,
    );
    assert_eq!(blocked.stream(), log.stream());
    assert_eq!(
        blocked.beats[count],
        (b.decided_at + 40, HandoffBeat::End(HandoffEnd::Acquisition))
    );
    assert_eq!(blocked.beats[count + 1].0, b.decided_at + 40);

    // A reader with no room until both streams are lost (each 512 periods after its T1)
    // gets one abort mark for the two: a mark that finds the one before it still waiting
    // is not sent.
    let open = b.fft_start + ADC_HISTORY + 10;
    assert!(open + 10 < adc.len());
    let stalled = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| if n < open { 0 } else { usize::MAX },
        |_, _| None,
    );
    assert_eq!(
        stalled.beats,
        [(open, HandoffBeat::End(HandoffEnd::Overrun))]
    );
    assert_eq!(
        stalled.holds(),
        [
            (a.decided_at, a.fft_start + ADC_HISTORY),
            (b.decided_at, b.fft_start + ADC_HISTORY)
        ]
    );

    // The mark that waits is the one of the stream the reader was in: it reads the first
    // stream, the decoder stops it, and the reader stalls until the second one is lost.
    let stop = a.fft_start + SIGNAL_END + 180;
    let stopped = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| if n > stop && n < open { 0 } else { usize::MAX },
        verdict_at(180, None),
    );
    let count = stop - a.fft_start + 1;
    assert_eq!(stopped.beats.len(), count + 1);
    assert_eq!(
        stopped.beats[count],
        (open, HandoffBeat::End(HandoffEnd::Stopped))
    );
}

#[test]
fn the_decoder_reads_the_longest_frame_from_the_stream() {
    // 4095 octets at 6 Mb/s from a transmitter 40 ppm slow: its frame is 4.4 samples
    // longer on the receiver's clock and the tracker moves the last window four samples
    // late. With the default back-off of four that window ends where the hold does, on the
    // frame's nominal last sample; a front with no back-off needs the stream's tail.
    let fx = FixedRxConfig::default();
    let fo = FixedOfdmConfig::default();
    let (adc, psdus) = scene(&[(Rate::R6, 4095)], 700, 900, -40.0, 9);
    let ratio = clock_ratio(FS, CARRIER).unwrap();
    for backoff in [B, 0] {
        let cfg = RxConfig {
            backoff_ns: 50.0 * backoff as f64,
            ..RxConfig::new(FS)
        };
        let log = run_with(
            cfg,
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            verdict_at(180, Some(MAX_DATA_SYMBOLS)),
        );
        assert_eq!(log.acquired.len(), 1);
        let a = log.acquired[0];
        assert_eq!(a.fft_start, 700 + 192 - backoff);
        let end = SIGNAL_END + 80 * MAX_DATA_SYMBOLS + backoff;
        assert_eq!(log.holds(), [(a.decided_at, a.fft_start + end)]);
        let count = end + HANDOFF_TAIL;
        assert_eq!(log.beats.len(), count + 1);
        let z = log.samples(0, count);
        let start = FixedTracking::from_cfo_word(a.cfo_phase, ratio, &fo);
        let d = decode_ppdu_fixed(&z, 0, &RxOfdmConfig::default(), &fo, start).unwrap();
        assert_eq!(d.psdu, psdus[0], "back-off {backoff}");
        let late = *d.timing.last().unwrap();
        assert!(late < -4.0 && late > -5.0, "{late}");
        // Cut at the hold's end the stream still holds the last window with the default
        // back-off, and not without one.
        let cut = decode_ppdu_fixed(&z[..end], 0, &RxOfdmConfig::default(), &fo, start);
        assert_eq!(cut.is_ok(), backoff == B, "back-off {backoff}");
        // And the stream is what the whole-buffer hand-off gives the decoder.
        let whole = reference(&adc, &a, &fx);
        let w = decode_ppdu_fixed(&whole, 0, &RxOfdmConfig::default(), &fo, start).unwrap();
        assert_eq!((w.psdu, w.timing), (d.psdu, d.timing));
    }
}

#[test]
fn an_invalid_configuration_hands_on_nothing_and_builds_no_table() {
    let (adc, _) = scene(&[SHORT], 700, 900, 12.5, 1);
    let d = FixedRxConfig::default();
    let bad = [
        FixedRxConfig {
            nco_lut_bits: 1,
            ..d
        },
        // A table of 2^28 entries if it were built.
        FixedRxConfig {
            nco_lut_bits: 30,
            ..d
        },
        FixedRxConfig {
            nco_amp_bits: 0,
            ..d
        },
        FixedRxConfig {
            nco_amp_bits: 40,
            ..d
        },
        FixedRxConfig { tap_bits: 40, ..d },
        FixedRxConfig {
            angle_bits: 40,
            ..d
        },
        FixedRxConfig {
            cordic_iters: u32::MAX,
            ..d
        },
        FixedRxConfig {
            nco_phase_bits: 70,
            ..d
        },
    ];
    for fx in bad {
        assert!(!fx.is_valid());
        assert!(nco_table(&fx).is_empty() && cordic_table(&fx).is_empty());
        assert_eq!(nco_rotator(12345, &fx), (0, 0));
        assert_eq!(cordic_angle(3, 4, &fx), 0);
        assert!(ltf_taps(&fx).iter().all(|t| *t == (0, 0)));
        assert!(acquisitions_fixed(&adc, &RxConfig::new(FS), &fx).is_empty());
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            |_, _| None,
        );
        assert!(log.acquired.is_empty() && log.beats.is_empty() && log.holds().is_empty());
    }
}
