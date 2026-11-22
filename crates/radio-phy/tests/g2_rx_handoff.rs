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
    acquisitions_fixed, cordic_angle, cordic_table, handoff_fixed, ltf_dc_offset_fixed, ltf_taps,
    nco_rotator, nco_table, to_adc, FixedAcquisition, FixedRxConfig, HandoffBeat, HandoffConfig,
    HandoffEnd, HandoffStart, RxHandoffFixed, SignalVerdict, ADC_FULL_SCALE, ADC_HISTORY,
    HANDOFF_TAIL, IQ, MAX_DATA_SYMBOLS,
};
use radio_phy::rx_ofdm::{RxOfdmConfig, Signal, SignalCheck};
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
    scene_at(30.0, frames, lead, gap, ppm, seed)
}

/// [`scene`] at another signal-to-noise ratio.
fn scene_at(
    snr_db: f64,
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
    let x = awgn(&x, snr_db, p, &mut rng);
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
    /// The tag on the latest first beat, 0 before any.
    fn tag(&self) -> u8 {
        self.beats
            .iter()
            .rev()
            .find_map(|b| match b.1 {
                HandoffBeat::Sample { first: Some(s), .. } => Some(s.packet),
                _ => None,
            })
            .unwrap_or(0)
    }

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
    run_verdicts(cfg, adc, fx, pace, room, |n, log| {
        verdict(n, log).map(|v| {
            v.map(|data_symbols| SignalVerdict {
                data_symbols,
                patient: false,
            })
        })
    })
}

/// A verdict with a length and nothing else.
fn plain(data_symbols: usize) -> SignalVerdict {
    SignalVerdict {
        data_symbols,
        patient: false,
    }
}

/// [`run_with`] with the verdicts as the front takes them.
fn run_verdicts(
    cfg: RxConfig,
    adc: &[(i16, i16)],
    fx: FixedRxConfig,
    pace: HandoffConfig,
    room: impl Fn(usize) -> usize,
    verdict: impl Fn(usize, &Log) -> Option<Option<SignalVerdict>>,
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
            // The reader gives back the tag of the stream it is in (0 before any).
            rx.signal(log.tag(), v);
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
                (k == 0).then_some(HandoffStart {
                    cfo_phase: a.cfo_phase,
                    packet: 1
                }),
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
                first: Some(HandoffStart {
                    cfo_phase: b.cfo_phase,
                    packet: 2
                })
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

    // The hold covers the frame's samples, to `end` − 1, whenever the verdict comes before
    // sample `end` is pushed: a verdict after the push of the last sample leaves it up for
    // the rest of that period, as a timely one does. One period later it is down at once.
    for (at, up, dropped) in [(end - 1, true, end), (end, false, end + 1)] {
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            |n, _| (n == at).then_some(Some(SHORT_SYMBOLS)),
        );
        assert_eq!(log.after_verdict, [(at, up)]);
        assert_eq!(log.holds(), [(a.decided_at, dropped)]);
    }

    // A SIGNAL of no data symbols (none exists; the call is the decoder's to make) is a
    // packet that ended with SIGNAL's window: given 320 periods after T1 the hold is down
    // at once and the mark is the packet's end, not a stop.
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        |n, _| (n == f + 320).then_some(Some(0)),
    );
    assert_eq!(log.after_verdict, [(f + 320, false)]);
    assert_eq!(log.holds(), [(a.decided_at, f + 321)]);
    // The replay, 27 periods old, had handed on 216 samples; it ends at the tail after
    // SIGNAL's window.
    let count = SIGNAL_END + B + HANDOFF_TAIL;
    assert_eq!(log.beats.len(), count + 1);
    assert_eq!(
        log.beats[count],
        (f + 321, HandoffBeat::End(HandoffEnd::Packet))
    );
}

#[test]
fn a_verdict_on_a_replaced_packet_does_not_touch_the_new_one() {
    // Two frames 600 samples apart. The decoder's verdict on the first comes five periods
    // after the second was declared: it carries the first one's tag and is ignored, as a
    // failed SIGNAL and as a length; the second packet's own verdict is taken.
    let fx = FixedRxConfig::default();
    let (adc, _) = scene(&[SHORT, SHORT], 700, 600, 12.5, 4);
    for stale in [None, Some(SHORT_SYMBOLS + 3)] {
        let mut rx = RxHandoffFixed::new(RxConfig::new(FS), fx, HandoffConfig::default());
        let mut acquired = Vec::new();
        let mut tags = Vec::new();
        let mut marks = Vec::new();
        let mut hold = Vec::new();
        for (n, &x) in adc.iter().enumerate() {
            let step = rx.push(x, usize::MAX);
            acquired.extend(step.acquired);
            hold.push(step.hold);
            for b in step.beats {
                match b {
                    HandoffBeat::Sample { first: Some(s), .. } => tags.push(s.packet),
                    HandoffBeat::End(e) => marks.push((n, e)),
                    HandoffBeat::Sample { .. } => {}
                }
            }
            if let [_, b] = acquired[..] {
                if n == b.decided_at + 5 {
                    rx.signal(1, stale.map(plain));
                    assert!(rx.hold());
                }
                if n == b.fft_start + SIGNAL_END + 180 {
                    rx.signal(2, Some(plain(SHORT_SYMBOLS)));
                }
            }
        }
        let (a, b) = (acquired[0], acquired[1]);
        assert_eq!(tags, [1, 2]);
        let end = b.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B;
        assert_eq!(
            marks,
            [
                (b.decided_at, HandoffEnd::Acquisition),
                (end + HANDOFF_TAIL - 1, HandoffEnd::Packet)
            ]
        );
        assert!(hold[a.decided_at..end].iter().all(|h| *h));
        assert!(hold[end..].iter().all(|h| !*h));
        assert!(!hold[a.decided_at - 1]);
    }
}

#[test]
fn the_word_handed_on_is_the_acquisitions_also_with_a_coarse_weight() {
    // With a coarse weight the CFO handed on is neither the fine nor the coarse estimate:
    // the stream's word, its rotation and its offset estimate use the one handed on.
    let fx = FixedRxConfig {
        coarse_weight_64: 15,
        ..FixedRxConfig::default()
    };
    let (adc, _) = scene(&[SHORT], 700, 900, -33.0, 2);
    let log = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |_| usize::MAX,
        verdict_at(180, Some(SHORT_SYMBOLS)),
    );
    let a = log.acquired[0];
    assert!(a.cfo_phase != a.fine_phase && a.cfo_phase != a.coarse_phase);
    let count = log.beats.len() - 1;
    assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
    assert_eq!(
        log.beats[0].1,
        HandoffBeat::Sample {
            z: reference(&adc, &a, &fx)[0],
            first: Some(HandoffStart {
                cfo_phase: a.cfo_phase,
                packet: 1
            })
        }
    );

    // The offset's estimate too: at 6 dB the two estimates of the CFO are far enough apart
    // in some frames for the offset to come out differently from each (the rotator's table
    // is read one address on from 610 Hz of difference), and the stream has the one of the
    // word handed on.
    let mut told_apart = 0;
    for seed in 0..40 {
        let (adc, _) = scene_at(6.0, &[SHORT], 700, 900, -33.0, 100 + seed);
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |_| usize::MAX,
            |_, _| None,
        );
        let Some(&a) = log.acquired.first() else {
            continue;
        };
        let field = &adc[a.fft_start..];
        let of = |word| ltf_dc_offset_fixed(field, 0, word, &fx).unwrap();
        if of(a.cfo_phase) == of(a.fine_phase) {
            continue;
        }
        told_apart += 1;
        let count = 400;
        assert_eq!(log.samples(0, count), reference(&adc, &a, &fx)[..count]);
    }
    assert!(told_apart >= 3, "{told_apart}");
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
                first: Some(HandoffStart {
                    cfo_phase: b.cfo_phase,
                    packet: 2
                })
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

    // A first packet whose last sample was taken, its end mark still waiting for the
    // reader when the second is declared, ends as a packet.
    let last = a.fft_start + SIGNAL_END + 80 * SHORT_SYMBOLS + B + HANDOFF_TAIL - 1;
    let done = run(
        &adc,
        fx,
        HandoffConfig::default(),
        |n| match n {
            n if n < last => usize::MAX,
            n if n == last => 1,
            n if n <= b.decided_at => 0,
            _ => usize::MAX,
        },
        verdict_at(180, Some(SHORT_SYMBOLS)),
    );
    let count = last - a.fft_start + 1;
    assert_eq!(
        done.beats[count],
        (b.decided_at + 1, HandoffBeat::End(HandoffEnd::Packet))
    );
    assert!(matches!(
        done.beats[count + 1].1,
        HandoffBeat::Sample { first: Some(s), .. } if s.packet == 2
    ));

    // A reader that stops with the next sample 512 behind the second declaration's: the
    // loss and the declaration fall in one period, and the mark is the declaration's.
    // Stopped one sample earlier, the stream is lost a period before, and the mark says so.
    for (stop, mark, lost_at) in [
        (b.decided_at - ADC_HISTORY, HandoffEnd::Acquisition, None),
        (
            b.decided_at - ADC_HISTORY - 1,
            HandoffEnd::Overrun,
            Some(b.decided_at - 1),
        ),
    ] {
        let log = run(
            &adc,
            fx,
            HandoffConfig::default(),
            |n| {
                if n >= stop && n < b.decided_at {
                    0
                } else {
                    usize::MAX
                }
            },
            |_, _| None,
        );
        assert_eq!(
            log.beats[stop - a.fft_start],
            (b.decided_at, HandoffBeat::End(mark))
        );
        match lost_at {
            None => assert_eq!(log.holds(), [(a.decided_at, adc.len())]),
            Some(l) => assert_eq!(log.holds(), [(a.decided_at, l), (b.decided_at, adc.len())]),
        }
    }

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
        assert_eq!(cordic_angle(-3, 4, &fx), 0);
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

/// The 12-bit ADC's rails.
const LO: i16 = -2048;
const HI: i16 = 2047;

/// A sample's index and what is put there.
type Swap = (usize, (i16, i16));

/// `adc` with the samples at `at` replaced.
fn with(adc: &[(i16, i16)], at: &[Swap]) -> Vec<(i16, i16)> {
    let mut out = adc.to_vec();
    for &(n, v) in at {
        out[n] = v;
    }
    out
}

#[test]
fn two_of_four_samples_on_a_rail_give_up_the_hold_and_the_stream_goes_on() {
    // ADR 0008, decision 2 as amended 2026-11-22. By hand: the window is the last four ADC
    // samples (n − 3 to n); the hold is down after the push of the sample that makes two of
    // them rail samples; a rail sample has I or Q at −2048 or 2047.
    assert_eq!(
        (i32::from(LO), i32::from(HI)),
        (-ADC_FULL_SCALE, ADC_FULL_SCALE - 1)
    );
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    assert_eq!((pace.clip_window, pace.clip_count), (4, 2));
    // No verdict: the watchdog's hold lasts beyond this stream.
    let (adc, _) = scene(&[SHORT], 700, 3000, 12.5, 21);
    let go =
        |adc: &[(i16, i16)], pace: HandoffConfig| run(adc, fx, pace, |_| usize::MAX, |_, _| None);
    let plain = go(&adc, pace);
    assert_eq!(plain.acquired.len(), 1);
    let a = plain.acquired[0];
    assert_eq!(plain.holds(), [(a.decided_at, adc.len())]);
    // The scene itself has no rail sample under the hold.
    assert!(adc[a.decided_at - 3..]
        .iter()
        .all(|s| s.0 > LO && s.0 < HI && s.1 > LO && s.1 < HI));

    let p = a.decided_at + 100;
    let kept = [(a.decided_at, adc.len())];
    let cases: [(&[Swap], Option<usize>); 12] = [
        // One rail sample is not enough.
        (&[(p, (HI, 0))], None),
        // Two that never share a window of four.
        (&[(p, (HI, 0)), (p + 4, (HI, 0))], None),
        (&[(p, (LO, LO)), (p + 4, (HI, HI)), (p + 8, (0, LO))], None),
        // Two in one window: down with the second, wherever it is in the window.
        (&[(p, (HI, 0)), (p + 3, (HI, 0))], Some(p + 3)),
        (&[(p, (HI, 0)), (p + 2, (HI, 0))], Some(p + 2)),
        (&[(p, (HI, 0)), (p + 1, (HI, 0))], Some(p + 1)),
        // Either rail of either part.
        (&[(p, (LO, 5)), (p + 3, (-7, HI))], Some(p + 3)),
        (&[(p, (9, LO)), (p + 1, (LO, HI))], Some(p + 1)),
        // One count short of a rail is not a rail.
        (
            &[
                (p, (HI - 1, LO + 1)),
                (p + 1, (LO + 1, HI - 1)),
                (p + 2, (HI - 1, HI - 1)),
            ],
            None,
        ),
        (
            &[
                (p, (HI, 0)),
                (p + 1, (HI - 1, 0)),
                (p + 2, (0, LO + 1)),
                (p + 4, (HI, 0)),
            ],
            None,
        ),
        // The first pair decides; what follows changes nothing.
        (
            &[
                (p, (0, HI)),
                (p + 3, (0, HI)),
                (p + 4, (0, HI)),
                (p + 50, (LO, LO)),
            ],
            Some(p + 3),
        ),
        // The hold's first sample can be its last.
        (
            &[(a.decided_at + 1, (HI, 0)), (a.decided_at + 2, (0, LO))],
            Some(a.decided_at + 2),
        ),
    ];
    for (k, (at, down)) in cases.iter().enumerate() {
        let x = with(&adc, at);
        let log = go(&x, pace);
        assert_eq!(log.acquired, plain.acquired, "case {k}");
        match down {
            None => assert_eq!(log.holds(), kept, "case {k}"),
            Some(n) => assert_eq!(log.holds(), [(a.decided_at, *n)], "case {k}"),
        }
        // The stream does not notice: the same number of beats in the same periods, no end
        // mark, and the samples are the hand-off's of what the ADC gave.
        assert_eq!(log.beats.len(), plain.beats.len(), "case {k}");
        assert!(
            log.beats.iter().zip(&plain.beats).all(|(u, v)| u.0 == v.0),
            "case {k}"
        );
        let count = log.beats.len();
        assert_eq!(
            log.samples(0, count),
            reference(&x, &a, &fx)[..count],
            "case {k}"
        );
    }

    // A count of zero: no release, whatever the ADC does.
    let off = HandoffConfig {
        clip_count: 0,
        ..pace
    };
    let railed: Vec<Swap> = (0..40).map(|k| (p + k, (HI, LO))).collect();
    let log = go(&with(&adc, &railed), off);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), kept);
    // The same samples with the rule on: down with the second of them.
    assert_eq!(
        go(&with(&adc, &railed), pace).holds(),
        [(a.decided_at, p + 1)]
    );

    // Other numbers: three of eight.
    let wide = HandoffConfig {
        clip_window: 8,
        clip_count: 3,
        ..pace
    };
    let r = (HI, 0);
    assert_eq!(
        go(&with(&adc, &[(p, r), (p + 2, r), (p + 7, r)]), wide).holds(),
        [(a.decided_at, p + 7)]
    );
    assert_eq!(
        go(&with(&adc, &[(p, r), (p + 2, r), (p + 8, r)]), wide).holds(),
        kept
    );
    assert_eq!(go(&with(&adc, &[(p, r), (p + 7, r)]), wide).holds(), kept);
    // A count the window cannot reach never releases.
    let never = HandoffConfig {
        clip_window: 4,
        clip_count: 5,
        ..pace
    };
    assert_eq!(go(&with(&adc, &railed), never).holds(), kept);
}

#[test]
fn a_hold_given_up_stays_down_through_the_verdict_and_a_new_declaration_raises_it() {
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    // The scene of `a_new_declaration_aborts…`: two frames, the first still open at the
    // second's declaration.
    let (adc, _) = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    let plain = run(&adc, fx, pace, |_| usize::MAX, |_, _| None);
    assert_eq!(plain.acquired.len(), 2);
    let (a, b) = (plain.acquired[0], plain.acquired[1]);
    assert_eq!(plain.holds(), [(a.decided_at, adc.len())]);

    // Two rail samples 30 samples after the first declaration (in the noise between the
    // frames, 370 samples before the second frame).
    let p = a.decided_at + 30;
    assert!(p + 300 < b.fft_start);
    let x = with(&adc, &[(p, (HI, HI)), (p + 1, (LO, LO))]);
    let log = run(&x, fx, pace, |_| usize::MAX, |_, _| None);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(
        log.holds(),
        [(a.decided_at, p + 1), (b.decided_at, adc.len())]
    );
    // The streams are as without the rails: the first aborted by the second.
    assert_eq!(log.beats.len(), plain.beats.len());
    assert_eq!(
        log.beats[b.decided_at - a.fft_start],
        (b.decided_at, HandoffBeat::End(HandoffEnd::Acquisition))
    );

    // A verdict with a length after the release: the hold stays down, the stream ends at
    // the length's end as it would have.
    let (adc, _) = scene(&[SHORT], 700, 900, 12.5, 1);
    let v = verdict_at(180, Some(SHORT_SYMBOLS));
    let plain = run(&adc, fx, pace, |_| usize::MAX, &v);
    let a = plain.acquired[0];
    let p = a.decided_at + 10;
    assert!(p + 1 < a.fft_start + SIGNAL_END + 180);
    let x = with(&adc, &[(p, (HI, 0)), (p + 1, (HI, 0))]);
    let log = run(&x, fx, pace, |_| usize::MAX, &v);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), [(a.decided_at, p + 1)]);
    assert_eq!(log.after_verdict, [(a.fft_start + SIGNAL_END + 180, false)]);
    assert_eq!(log.beats.len(), plain.beats.len());
    assert_eq!(log.beats.last(), plain.beats.last());
}

#[test]
fn rail_samples_before_the_declaration_count_in_the_window() {
    // The window does not start with the hold: it is the ADC's last four samples.
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    let (adc, _) = scene(&[SHORT], 700, 3000, 12.5, 21);
    let go = |adc: &[(i16, i16)]| run(adc, fx, pace, |_| usize::MAX, |_, _| None);
    let plain = go(&adc);
    let a = plain.acquired[0];
    let d = a.decided_at;
    // One before the declaration and one after it: up for one push.
    let x = with(&adc, &[(d - 1, (HI, 0)), (d + 1, (0, LO))]);
    let log = go(&x);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), [(d, d + 1)]);
    // Three samples earlier it has left the window by then.
    let x = with(&adc, &[(d - 3, (HI, 0)), (d + 1, (0, LO))]);
    let log = go(&x);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), [(d, adc.len())]);
    // Two in the window at the declaring sample: the hold is never shown.
    let x = with(&adc, &[(d - 2, (HI, 0)), (d - 1, (0, LO))]);
    let log = go(&x);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), []);
}

#[test]
fn the_release_acts_anywhere_under_the_hold_and_for_any_window() {
    // Review of 2026-11-22: the release late in a packet, after the verdict, a sample beyond
    // the rails, and windows of 0, 1, 16, 63, 64 and more.
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    let r = (HI, 0);

    // A long frame with its verdict: 1000 octets at 6 Mb/s, 16 + 8000 + 6 bits in symbols
    // of 24, 335 data symbols. The release 1000 and 20 000 samples after the verdict.
    let symbols = (16usize + 8000 + 6).div_ceil(24);
    assert_eq!(symbols, 335);
    let (adc, _) = scene(&[(Rate::R6, 1000)], 700, 900, 12.5, 31);
    let v = verdict_at(180, Some(symbols));
    let plain = run(&adc, fx, pace, |_| usize::MAX, &v);
    assert_eq!(plain.acquired.len(), 1);
    let a = plain.acquired[0];
    let said = a.fft_start + SIGNAL_END + 180;
    let end = a.fft_start + SIGNAL_END + 80 * symbols + B;
    assert_eq!(plain.holds(), [(a.decided_at, end)]);
    for p in [said + 1000, said + 20_000, end - 2] {
        let x = with(&adc, &[(p, r), (p + 1, (0, LO))]);
        let log = run(&x, fx, pace, |_| usize::MAX, &v);
        assert_eq!(log.acquired, plain.acquired, "at {p}");
        assert_eq!(log.holds(), [(a.decided_at, p + 1)], "at {p}");
        assert_eq!(log.beats.len(), plain.beats.len(), "at {p}");
        assert_eq!(log.beats.last(), plain.beats.last(), "at {p}");
    }
    // Under the watchdog's hold, with no verdict, 20 000 samples in.
    let p = a.decided_at + 20_000;
    let x = with(&adc, &[(p, r), (p + 3, r)]);
    let log = run(&x, fx, pace, |_| usize::MAX, |_, _| None);
    assert_eq!(log.acquired, plain.acquired);
    assert_eq!(log.holds(), [(a.decided_at, p + 3)]);

    // A sample beyond the rails (the input is 16 bits wide) is a rail sample.
    let (adc, _) = scene(&[SHORT], 700, 3000, 12.5, 21);
    let go =
        |adc: &[(i16, i16)], pace: HandoffConfig| run(adc, fx, pace, |_| usize::MAX, |_, _| None);
    let plain = go(&adc, pace);
    let a = plain.acquired[0];
    let kept = [(a.decided_at, adc.len())];
    let p = a.decided_at + 100;
    for far in [(i16::MAX, 0), (0, i16::MIN), (HI + 1, 0), (0, LO - 1)] {
        let x = with(&adc, &[(p, far), (p + 2, far)]);
        assert_eq!(go(&x, pace).holds(), [(a.decided_at, p + 2)], "{far:?}");
    }

    // Windows. (window, count, distance between two rail samples, released?)
    let two = |d: usize| with(&adc, &[(p, r), (p + d, r)]);
    for (window, count, d, released) in [
        (16, 2, 15, true),
        (16, 2, 16, false),
        (63, 2, 62, true),
        (63, 2, 63, false),
        (64, 2, 63, true),
        (64, 2, 64, false),
        // A window above 64 is 64.
        (65, 2, 64, false),
        (1000, 2, 63, true),
        (usize::MAX, 2, 64, false),
        // A window of one sees the newest sample alone.
        (1, 2, 1, false),
        // A window of none sees nothing.
        (0, 1, 1, false),
        (0, 2, 1, false),
    ] {
        let cfg = HandoffConfig {
            clip_window: window,
            clip_count: count,
            ..pace
        };
        let want = if released {
            vec![(a.decided_at, p + d)]
        } else {
            kept.to_vec()
        };
        assert_eq!(
            go(&two(d), cfg).holds(),
            want,
            "window {window} count {count} at {d}"
        );
    }
    // One of one: down with the first rail sample.
    let one = HandoffConfig {
        clip_window: 1,
        clip_count: 1,
        ..pace
    };
    assert_eq!(go(&with(&adc, &[(p, r)]), one).holds(), [(a.decided_at, p)]);
    // Sixty-four of sixty-four: down with the 64th in a row, not the 63rd.
    let all = HandoffConfig {
        clip_window: 64,
        clip_count: 64,
        ..pace
    };
    let row = |n: usize| -> Vec<Swap> { (0..n).map(|k| (p + k, r)).collect() };
    assert_eq!(go(&with(&adc, &row(63)), all).holds(), kept);
    assert_eq!(
        go(&with(&adc, &row(64)), all).holds(),
        [(a.decided_at, p + 63)]
    );
}

/// A verdict with a length for the first acquisition's stream, `after` samples past SIGNAL's
/// window: `patient` or not.
fn told(
    after: usize,
    data_symbols: usize,
    patient: bool,
) -> impl Fn(usize, &Log) -> Option<Option<SignalVerdict>> {
    move |n, log| {
        let a = log.acquired.first()?;
        (n == a.fft_start + SIGNAL_END + after).then_some(Some(SignalVerdict {
            data_symbols,
            patient,
        }))
    }
}

#[test]
fn a_patient_hold_is_given_up_by_24_of_32_and_not_by_2_of_4() {
    // ADR 0008, decision 2, fourth amendment (2026-11-22). By hand: from the verdict that
    // says patient on, the window is the last 32 ADC samples and the count 24; before it,
    // and after a verdict that does not say so, 4 and 2.
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    assert_eq!(
        (pace.patient_window, pace.patient_count, pace.patient_agree),
        (32, 24, 40)
    );
    let r = (HI, 0);
    // 1000 octets at 6 Mb/s, 335 data symbols; the verdict 180 samples after SIGNAL.
    let (adc, _) = scene(&[(Rate::R6, 1000)], 700, 900, 12.5, 31);
    let go = |adc: &[(i16, i16)], pace: HandoffConfig, patient: bool| {
        run_verdicts(
            RxConfig::new(FS),
            adc,
            fx,
            pace,
            |_| usize::MAX,
            told(180, 335, patient),
        )
    };
    let plain = go(&adc, pace, true);
    assert_eq!(plain.acquired.len(), 1);
    let a = plain.acquired[0];
    let said = a.fft_start + SIGNAL_END + 180;
    let end = a.fft_start + SIGNAL_END + 80 * 335 + B;
    let kept = [(a.decided_at, end)];
    assert_eq!(plain.holds(), kept);
    let p = said + 1000;
    let row = |from: usize, n: usize| -> Vec<Swap> { (0..n).map(|k| (from + k, r)).collect() };

    // Patient: two in four do nothing, nor do 23 in a row; the 24th in a row takes it.
    assert_eq!(go(&with(&adc, &row(p, 2)), pace, true).holds(), kept);
    assert_eq!(go(&with(&adc, &row(p, 23)), pace, true).holds(), kept);
    assert_eq!(
        go(&with(&adc, &row(p, 24)), pace, true).holds(),
        [(a.decided_at, p + 23)]
    );
    // 24 spread over 32: every fourth sample clean (p + 3, p + 7, … p + 31), so samples
    // p … p + 30 hold 24 rail samples and the 24th is p + 30.
    let spread: Vec<Swap> = (0..32).filter(|k| k % 4 != 3).map(|k| (p + k, r)).collect();
    assert_eq!(spread.len(), 24);
    assert_eq!(
        go(&with(&adc, &spread), pace, true).holds(),
        [(a.decided_at, p + 30)]
    );
    // The same 24 over 33 samples (one more clean sample in the middle) never share a
    // window of 32.
    let wide: Vec<Swap> = (0..33)
        .filter(|k| k % 4 != 3 && *k != 16)
        .map(|k| (p + k, r))
        .collect();
    assert_eq!(wide.len(), 24);
    assert_eq!(go(&with(&adc, &wide), pace, true).holds(), kept);

    // Not patient: the same verdict without the word leaves 2 of 4.
    assert_eq!(
        go(&with(&adc, &row(p, 2)), pace, false).holds(),
        [(a.decided_at, p + 1)]
    );
    // Before the verdict the rule is 2 of 4 whatever the verdict will say; the samples
    // the verdict's own push sees are still judged by 2 of 4, the next by 24 of 32.
    for at in [said - 100, said - 1] {
        assert_eq!(
            go(&with(&adc, &row(at, 2)), pace, true).holds(),
            [(a.decided_at, at + 1)],
            "at {at}"
        );
    }
    assert_eq!(go(&with(&adc, &row(said, 2)), pace, true).holds(), kept);
    // Rail samples from before the verdict count in the patient window: 20 before it and
    // 4 after make 24 of 32 — but those 20 would have released by 2 of 4, so put them one
    // in four apart, where 2 of 4 never sees two: no release before, none after either
    // (8 in 32).
    let thin: Vec<Swap> = (0..8).map(|k| (said - 30 + 4 * k, r)).collect();
    assert_eq!(go(&with(&adc, &thin), pace, true).holds(), kept);

    // Other numbers, and a patient count of 0: never, while patient.
    let cfg = HandoffConfig {
        patient_window: 8,
        patient_count: 3,
        ..pace
    };
    assert_eq!(
        go(&with(&adc, &[(p, r), (p + 2, r), (p + 7, r)]), cfg, true).holds(),
        [(a.decided_at, p + 7)]
    );
    assert_eq!(
        go(&with(&adc, &[(p, r), (p + 2, r), (p + 8, r)]), cfg, true).holds(),
        kept
    );
    let never = HandoffConfig {
        patient_count: 0,
        ..pace
    };
    assert_eq!(go(&with(&adc, &row(p, 200)), never, true).holds(), kept);
    assert_eq!(
        go(&with(&adc, &row(p, 200)), never, false).holds(),
        [(a.decided_at, p + 1)]
    );
    // The stream does not notice any of it.
    let log = go(&with(&adc, &row(p, 24)), pace, true);
    assert_eq!(log.beats.len(), plain.beats.len());
    assert_eq!(log.beats.last(), plain.beats.last());
}

#[test]
fn a_declaration_under_a_patient_hold_inherits_the_patience_until_its_own_verdict() {
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    let r = (HI, 0);
    // Two short frames 600 samples apart; the first one's verdict claims the longest
    // frame, so its hold is still up when the second is declared.
    let (adc, _) = scene(&[SHORT, SHORT], 700, 600, -33.0, 8);
    // `first`: what the first stream's verdict says; `second`: the second's, if it comes
    // (60 samples after its stream's first beat).
    let go = |adc: &[(i16, i16)], first: Option<bool>, second: Option<Option<bool>>| {
        run_verdicts(
            RxConfig::new(FS),
            adc,
            fx,
            pace,
            |_| usize::MAX,
            |n, log| {
                let verdict = |patient| SignalVerdict {
                    data_symbols: MAX_DATA_SYMBOLS,
                    patient,
                };
                let a = log.acquired.first()?;
                if n == a.fft_start + SIGNAL_END + 180 {
                    return first.map(|p| Some(verdict(p)));
                }
                let b = log.acquired.get(1)?;
                if n == b.decided_at + 80 {
                    return second.map(|s| s.map(verdict));
                }
                None
            },
        )
    };
    let plain = go(&adc, Some(true), None);
    assert_eq!(plain.acquired.len(), 2);
    let (a, b) = (plain.acquired[0], plain.acquired[1]);
    assert!(a.fft_start + SIGNAL_END + 180 < b.decided_at);
    let all = [(a.decided_at, adc.len())];
    assert_eq!(plain.holds(), all);
    let two = |at: usize| with(&adc, &[(at, r), (at + 1, r)]);
    let (after, late) = (b.decided_at + 10, b.decided_at + 200);

    // Patient, declared again: still patient before the second verdict.
    assert_eq!(go(&two(after), Some(true), None).holds(), all);
    // The first verdict did not say patient: the second declaration finds 2 of 4.
    assert_eq!(
        go(&two(after), Some(false), None).holds(),
        [(a.decided_at, after + 1)]
    );
    // No verdict on the first at all: 2 of 4 all through.
    assert_eq!(
        go(&two(after), None, None).holds(),
        [(a.decided_at, after + 1)]
    );
    // The second stream's own verdict decides from then on.
    assert_eq!(go(&two(late), Some(true), Some(Some(true))).holds(), all);
    assert_eq!(
        go(&two(late), Some(true), Some(Some(false))).holds(),
        [(a.decided_at, late + 1)]
    );
    // And a verdict of "patient" on the second makes it patient though the first was not.
    assert_eq!(go(&two(late), Some(false), Some(Some(true))).holds(), all);
    // A SIGNAL that fails ends the hold (it is up through the push the verdict follows)
    // and the patience: the scene has no third frame, so the hold stays down.
    assert_eq!(
        go(&adc, Some(true), Some(None)).holds(),
        [(a.decided_at, b.decided_at + 81)]
    );

    // A patient hold given up before the declaration (24 in a row, in the noise between
    // the frames: the first ends at sample 1420, the second begins at 2020) is not
    // inherited: the second declaration raises the hold by itself, with 2 of 4.
    let gap = 1600;
    assert!(a.fft_start + SIGNAL_END + 180 < gap && gap + 24 < 2020);
    let gone: Vec<Swap> = (0..24).map(|k| (gap + k, r)).collect();
    let x = with(&adc, &gone);
    let log = go(&x, Some(true), None);
    // The burst sets the detector off, and the second frame is declared at the same
    // sample with another detection behind it.
    let declared = |log: &Log| {
        log.acquired
            .iter()
            .map(|q| q.decided_at)
            .collect::<Vec<_>>()
    };
    assert_eq!(declared(&log), [a.decided_at, b.decided_at]);
    assert_eq!(
        log.holds(),
        [(a.decided_at, gap + 23), (b.decided_at, adc.len())]
    );
    let mut y = x.clone();
    y[after] = r;
    y[after + 1] = r;
    let log = go(&y, Some(true), None);
    assert_eq!(declared(&log), [a.decided_at, b.decided_at]);
    assert_eq!(
        log.holds(),
        [(a.decided_at, gap + 23), (b.decided_at, after + 1)]
    );
}

#[test]
fn the_verdict_is_patient_for_a_slow_rate_with_forty_agreeing_bits() {
    let pace = HandoffConfig::default();
    let check = |agree| SignalCheck {
        agree,
        disagree: 48 - agree,
    };
    for rate in Rate::ALL {
        let slow = matches!(rate, Rate::R6 | Rate::R9 | Rate::R12);
        for (agree, enough) in [(48, true), (40, true), (39, false), (0, false)] {
            let v = SignalVerdict::of(Signal { rate, length: 1000 }, check(agree), &pace);
            assert_eq!(v.patient, slow && enough, "{rate:?} {agree}");
        }
    }
    // The length in symbols (IEEE 802.11-2020 §17.3.5.4): 16 + 8·LENGTH + 6 bits in
    // symbols of N_DBPS.
    let v = SignalVerdict::of(
        Signal {
            rate: Rate::R6,
            length: 1000,
        },
        check(48),
        &pace,
    );
    assert_eq!(v.data_symbols, (16usize + 8000 + 6).div_ceil(24));
    let v = SignalVerdict::of(
        Signal {
            rate: Rate::R54,
            length: 100,
        },
        check(48),
        &pace,
    );
    assert_eq!((v.data_symbols, v.patient), (SHORT_SYMBOLS, false));
    // It is the agreeing bits that count: a SIGNAL of erasures with no bit against it is
    // not believed, and one with eight against it and forty for it is.
    let slow = Signal {
        rate: Rate::R6,
        length: 1000,
    };
    let faint = SignalCheck {
        agree: 30,
        disagree: 0,
    };
    assert!(!SignalVerdict::of(slow, faint, &pace).patient);
    assert!(SignalVerdict::of(slow, check(40), &pace).patient);
    // Another threshold.
    let strict = HandoffConfig {
        patient_agree: 44,
        ..pace
    };
    let s = Signal {
        rate: Rate::R9,
        length: 1,
    };
    assert!(!SignalVerdict::of(s, check(43), &strict).patient);
    assert!(SignalVerdict::of(s, check(44), &strict).patient);
}
