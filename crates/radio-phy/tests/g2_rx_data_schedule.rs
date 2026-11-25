//! Gate for the data path's schedule in clocks (ADR 0010, decisions 6.1c and 6.2), restated
//! from the RTL as built: `rx_data::RxDataSchedule` reads the hand-off's stream a beat a clock
//! when the top level's reader may, and gives `RxDataFixed`'s events, each in a cycle the
//! schedule says.
//!
//! References: the events are `RxDataFixed`'s on the beats the schedule took (gated in
//! `g2_rx_data_stream.rs`); the rules are the blocks' headers, each checked by its own bench;
//! the cycles are the schedule's table worked by hand (the module's text and the G0 in
//! `docs/gates/rx_data.md`); the hand-off run with this reader is
//! `run_handoff`'s (gated in `g2_rx_handoff_schedule.rs`). A model gate: no reference from
//! outside the project.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, build_ppdu_bits, data_symbols, Rate};
use radio_phy::rx_data::{rx_end_ages, DataTiming, Division, RxDataSchedule, Window};
use radio_phy::rx_data_fixed::{RxDataEnd, RxDataEvent, RxDataFixed};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    to_adc, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd, RxHandoffFixed, ADC_FULL_SCALE,
    ADC_HISTORY,
};
use radio_phy::rx_handoff::{run_handoff, HandoffRun, HandoffTiming};
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{clock_ratio, FixedOfdmConfig};
use std::f64::consts::PI;

const FS: f64 = 20e6;
const CARRIER_HZ: f64 = 5.8e9;

const RATES: [Rate; 8] = [
    Rate::R6,
    Rate::R9,
    Rate::R12,
    Rate::R18,
    Rate::R24,
    Rate::R36,
    Rate::R48,
    Rate::R54,
];

fn frame(psdu: &[u8], rate: Rate, seed: u8) -> Vec<(f64, f64)> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

/// The ADC's samples, the PSDUs, each frame's first sample.
type Train = (Vec<(i16, i16)>, Vec<Vec<u8>>, Vec<usize>);

/// Frames (rate, octets, SNR in dB, clock offset in ppm) each after `gap` samples of
/// silence (the first after 400), at −15 dBFS. Returns the ADC's samples, the PSDUs and
/// where each frame starts.
fn train(frames: &[(Rate, usize, f64, f64)], gap: usize, seed: u64) -> Train {
    let mut rng = Rng::new(seed);
    let mut x: Vec<(f64, f64)> = Vec::new();
    let mut psdus = Vec::new();
    let mut starts = Vec::new();
    let mut p_ref = 0.0;
    for (k, &(rate, octets, snr, ppm)) in frames.iter().enumerate() {
        let psdu: Vec<u8> = (0..octets).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f = frame(&psdu, rate, 1 + (rng.next_u64() % 127) as u8);
        let p = mean_power(&f);
        if k == 0 {
            p_ref = p;
        }
        let lead = if k == 0 { 400 } else { gap };
        let mut part = vec![(0.0, 0.0); lead];
        part.extend_from_slice(&f);
        let part = if ppm == 0.0 {
            part
        } else {
            resample(&part, ppm)
        };
        let part = frequency_offset(&part, ppm * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
        let part = awgn(&part, snr, p, &mut rng);
        starts.push(x.len() + lead);
        x.extend(part);
        psdus.push(psdu);
    }
    x.extend(vec![(0.0, 0.0); 600]);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p_ref.sqrt();
    (to_adc(&x, gain), psdus, starts)
}

fn schedule(t: DataTiming) -> RxDataSchedule {
    RxDataSchedule::new(
        RxOfdmConfig::default(),
        FixedOfdmConfig::default(),
        HandoffConfig::default(),
        clock_ratio(FS, CARRIER_HZ),
        t,
    )
    .unwrap()
}

/// The first packet's beats (and its mark) as the hand-off hands them on with no limit of
/// pace.
fn first_stream(adc: &[(i16, i16)]) -> Vec<HandoffBeat> {
    let free = HandoffConfig {
        rate: usize::MAX,
        ..HandoffConfig::default()
    };
    let mut rx = RxHandoffFixed::new(RxConfig::new(FS), FixedRxConfig::default(), free);
    let mut beats = Vec::new();
    for &x in adc {
        for b in rx.push(x, usize::MAX).beats {
            let end = matches!(b, HandoffBeat::End(_));
            beats.push(b);
            if end {
                return beats;
            }
        }
    }
    beats
}

/// A beat offered every cycle from `c0`, taken when the schedule is ready; the cycle each
/// was taken in.
fn offer(s: &mut RxDataSchedule, beats: &[HandoffBeat], c0: usize) -> Vec<usize> {
    let mut clock = c0;
    let mut i = 0;
    let mut taken = Vec::with_capacity(beats.len());
    while i < beats.len() {
        if s.ready_at(clock) {
            s.take(clock, beats[i]);
            taken.push(clock);
            i += 1;
        }
        clock += 1;
        assert!(
            clock < c0 + 100 * beats.len() + 100_000,
            "stalled at beat {i}"
        );
    }
    taken
}

#[test]
fn the_table_of_one_symbol_at_54_mbps_worked_by_hand() {
    let (adc, psdus, _) = train(&[(Rate::R54, 24, 30.0, 0.0)], 0, 1);
    let beats = first_stream(&adc);
    let c0 = 1000;
    let mut s = schedule(DataTiming::default());
    let taken = offer(&mut s, &beats, c0);
    assert!(s.violations().is_empty(), "{:?}", s.violations());
    let rel = |c: usize| c - c0;
    // The tail: up to the next window's old place less one (302) at once, the rest from the
    // cycle after the reader has seen the verdict (481 + 2), the frame's last window known.
    assert_eq!((rel(taken[288]), rel(taken[302])), (336, 350));
    assert_eq!(rel(taken[303]), 483);

    // T1 and T2 chained (their bins pushed out by the next window's samples), SIGNAL and the
    // data symbol flushed; the data window waits for SIGNAL's flush (207 + 65).
    let w: Vec<_> = s
        .windows()
        .iter()
        .map(|w| (w.window, rel(w.first), rel(w.last), rel(w.bins)))
        .collect();
    assert_eq!(
        w,
        [
            (Window::T1, 0, 63, 140),
            (Window::T2, 64, 127, 220),
            (Window::Symbol(0), 144, 207, 284),
            (Window::Symbol(1), 272, 335, 412),
        ]
    );
    // chan_valid on 220 + 63 = 283; the four quotients 18 + 2, 14 + 2, 14 + 2, 12 + 2 apart;
    // the fit before the data's scale (asked at the verdict, 481 + 3).
    let d: Vec<_> = s
        .divisions()
        .iter()
        .map(|d| (d.job, rel(d.asked), rel(d.start), rel(d.done)))
        .collect();
    assert_eq!(
        d,
        [
            (Division::Scale(Modulation::Bpsk), 285, 285, 303),
            (Division::Outer, 305, 305, 319),
            (Division::Inner, 321, 321, 335),
            (Division::Variance, 337, 337, 349),
            (Division::Fit(1), 478, 478, 504),
            (Division::Scale(Modulation::Qam64), 484, 505, 525),
        ]
    );
    let sig = s.symbols()[0];
    assert_eq!(
        (rel(sig.bins), rel(sig.pilots), rel(sig.step)),
        (284, 293, 295)
    );
    assert_eq!((rel(sig.outer.0), rel(sig.outer.1)), (301, 302));
    assert_eq!((rel(sig.inner.0), rel(sig.inner.1)), (305, 320));
    assert_eq!((rel(sig.g1), rel(sig.end), rel(sig.theta)), (360, 370, 6));
    assert_eq!(
        (rel(sig.phi.0), rel(sig.phi.1), rel(sig.phi.2)),
        (351, 352, 370)
    );
    assert_eq!(rel(sig.scale), 305);
    assert_eq!((rel(sig.soft.0), rel(sig.soft.1)), (378, 425));
    let (a, b) = sig.steps.unwrap();
    assert_eq!((rel(a), rel(b)), (428, 451));
    let data = s.symbols()[1];
    assert_eq!(
        (rel(data.bins), rel(data.pilots), rel(data.step)),
        (412, 421, 423)
    );
    assert_eq!((rel(data.outer.0), rel(data.outer.1)), (429, 430));
    assert_eq!((rel(data.inner.0), rel(data.inner.1)), (433, 448));
    assert_eq!(rel(data.g1), 467);
    let (ft, fd) = data.fit.unwrap();
    assert_eq!((rel(ft), rel(fd), rel(data.theta)), (478, 504, 509));
    assert_eq!(
        (rel(data.phi.0), rel(data.phi.1), rel(data.phi.2)),
        (519, 520, 538)
    );
    assert_eq!(rel(data.scale), 527);
    assert_eq!((rel(data.soft.0), rel(data.soft.1)), (546, 593));
    let (a, b) = data.steps.unwrap();
    assert_eq!((rel(a), rel(b)), (596, 809));

    // The events (in the cycle after their register): the verdict and the start at 482, the
    // octets, the end with the last.
    let ev = s.events();
    assert!(matches!(ev[0].1, RxDataEvent::Verdict { .. }));
    assert_eq!(rel(ev[0].0), 482);
    assert!(matches!(ev[1].1, RxDataEvent::Start { .. }));
    assert_eq!(rel(ev[1].0), 482);
    let octets: Vec<(usize, u8)> = ev
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::Octet { octet, .. } => Some((rel(e.0), octet)),
            _ => None,
        })
        .collect();
    assert_eq!(octets.iter().map(|o| o.1).collect::<Vec<_>>(), psdus[0]);
    assert_eq!(octets[0].0, 721);
    assert_eq!(octets[11].0, 809);
    assert_eq!(octets[12].0, 817);
    assert_eq!(octets[23].0, 905);
    // Bits 0 … 117 a step apart, the flush's 96 bits from the edge after the last regular
    // one: octets eight clocks apart throughout.
    for j in 1..24 {
        assert_eq!(octets[j].0 - octets[j - 1].0, 8, "octet {j}");
    }
    let end = ev
        .iter()
        .find(|e| e.1 == RxDataEvent::End(RxDataEnd::Decoded))
        .unwrap();
    assert_eq!(rel(end.0), 905);
}

#[test]
fn a_long_frame_runs_at_one_symbol_in_240_clocks_whatever_the_rate() {
    // Beats offered every cycle (the reader behind the antenna): from one θ to the next the
    // sample before the window (θ + 2) and the window (1 + 63), the transform's last bin
    // (77), the pilots (9), the step's start (2), the step with its division (86): 240.
    for rate in RATES {
        let (adc, _, _) = train(&[(rate, 1500, 30.0, 0.0)], 0, 2);
        let beats = first_stream(&adc);
        let mut s = schedule(DataTiming::default());
        offer(&mut s, &beats, 0);
        assert!(s.violations().is_empty(), "{rate:?}: {:?}", s.violations());
        let thetas: Vec<usize> = s.symbols().iter().skip(1).map(|r| r.theta).collect();
        assert!(thetas.len() > 50, "{rate:?}");
        for (i, w) in thetas.windows(2).enumerate().skip(1) {
            assert_eq!(w[1] - w[0], 240, "{rate:?}, data symbol {}", i + 2);
        }
        // Each data window's first sample waits for the last θ (+ 2) and the FFT's flush.
        let wins: Vec<_> = s
            .windows()
            .iter()
            .filter(|w| matches!(w.window, Window::Symbol(k) if k >= 1))
            .collect();
        for (k, w) in wins.iter().enumerate().skip(1) {
            let prev_theta = s.symbols()[k].theta;
            assert!(w.first >= prev_theta + 2, "{rate:?} window {k}");
            assert!(w.first >= wins[k - 1].last + 65, "{rate:?} window {k}");
        }
        // BPSK's data needs no second scale.
        let scales = s
            .divisions()
            .iter()
            .filter(|d| matches!(d.job, Division::Scale(_)))
            .count();
        let want = if rate.modulation() == Modulation::Bpsk {
            1
        } else {
            2
        };
        assert_eq!(scales, want, "{rate:?}");
        assert_eq!(
            s.events().last().unwrap().1,
            RxDataEvent::End(RxDataEnd::Decoded),
            "{rate:?}"
        );
    }
}

/// The run with the schedule as the hand-off's reader, and what it gave.
fn closed_loop(adc: &[(i16, i16)], t: DataTiming) -> (HandoffRun, RxDataSchedule) {
    closed_loop_idle(adc, t, &|_| 0)
}

/// [`closed_loop`] with `idle(n)` idle clocks before sample n.
fn closed_loop_idle(
    adc: &[(i16, i16)],
    t: DataTiming,
    idle: &dyn Fn(usize) -> usize,
) -> (HandoffRun, RxDataSchedule) {
    let mut s = schedule(t);
    let run = run_handoff(
        adc,
        &RxConfig::new(FS),
        &FixedRxConfig::default(),
        &HandoffConfig::default(),
        &HandoffTiming::default(),
        idle,
        &mut s,
    )
    .unwrap();
    (run, s)
}

/// The events are the model's on the beats taken; the cycles never go back; each strobe the
/// hand-off saw is a verdict event in the same cycle with the same tag; no overrun.
fn held_to_the_model(run: &HandoffRun, s: &RxDataSchedule, what: &str) {
    let mut model = RxDataFixed::new(
        RxOfdmConfig::default(),
        FixedOfdmConfig::default(),
        HandoffConfig::default(),
        clock_ratio(FS, CARRIER_HZ),
    )
    .unwrap();
    let mut want = Vec::new();
    for (_, b) in &run.beats {
        want.extend(model.push(*b));
    }
    assert!(s.violations().is_empty(), "{what}: {:?}", s.violations());
    let got: Vec<RxDataEvent> = s.events().iter().map(|e| e.1).collect();
    assert_eq!(got, want, "{what}: the model's events");
    for w in s.events().windows(2) {
        assert!(w[0].0 <= w[1].0, "{what}: cycles in order");
    }
    let verdicts: Vec<(usize, u8)> = s
        .events()
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::Verdict { packet, .. } => Some((e.0, packet)),
            _ => None,
        })
        .filter(|v| v.0 <= run.end)
        .collect();
    let strobes: Vec<(usize, u8)> = run.verdicts.iter().map(|v| (v.clock, v.packet)).collect();
    assert_eq!(strobes, verdicts, "{what}: the strobes");
    assert!(
        !run.beats
            .iter()
            .any(|b| b.1 == HandoffBeat::End(HandoffEnd::Overrun)),
        "{what}: an overrun"
    );
    // The age: on the decoded and the seed-zero ends only (the founder, 2026-11-25), one
    // for every end.
    let ends: Vec<(usize, RxDataEnd)> = s
        .events()
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::End(end) => Some((e.0, end)),
            _ => None,
        })
        .collect();
    let ages = rx_end_ages(run, s);
    assert_eq!(ages.len(), ends.len(), "{what}: an age for every end");
    for ((c, end), (ca, age)) in ends.iter().zip(&ages) {
        assert_eq!(c, ca, "{what}: the age's cycle");
        let carries = matches!(end, RxDataEnd::Decoded | RxDataEnd::Seed);
        assert_eq!(age.is_some(), carries, "{what}: {end:?} at {c}");
    }
}

/// The ages of the decoded ends against the frames' true ends at the antenna (sample
/// `frame_ends[k]` is the first after frame k), in samples: (age − true age) for each.
fn age_errors(run: &HandoffRun, s: &RxDataSchedule, frame_ends: &[usize], what: &str) -> Vec<i64> {
    let decoded: Vec<(usize, u16)> = s
        .events()
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::End(end) => Some(end),
            _ => None,
        })
        .zip(rx_end_ages(run, s))
        .filter(|(end, _)| *end == RxDataEnd::Decoded)
        .map(|(_, (c, age))| (c, age.unwrap()))
        .collect();
    assert_eq!(decoded.len(), frame_ends.len(), "{what}: decoded frames");
    decoded
        .iter()
        .zip(frame_ends)
        .map(|(&(c, age), &end)| {
            // The truth: samples at the ADC's pins before rx_end's cycle, less the frame's.
            let written = run
                .ticks
                .iter()
                .filter(|&&t| t - HandoffTiming::default().tick() < c)
                .count();
            i64::from(age) - (written as i64 - end as i64)
        })
        .collect()
}

/// The largest error of an age against the frame's true end, in samples. The true end is the
/// transmitter's (`train`'s start + 400 + 80 per data symbol, in its samples): at 30 dB the
/// acquisition finds T1's window exactly on every run here, so the nominal end from SIGNAL
/// (the founder, 2026-11-25: the tracker's σ left out) is that sample, and any error is a
/// defect. The drift of the sample clock moves the end at the ADC by up to 40 ppm of the
/// frame (1.6 samples for 40 000) and is not in the age, by that decision.
const AGE_TOLERANCE: i64 = 0;

#[test]
fn the_age_of_every_decoded_frame_is_its_true_end_at_the_antenna() {
    let mut worst = 0i64;
    for (k, rate) in RATES.iter().enumerate() {
        for octets in [1usize, 24, 300, 1500] {
            for ppm in [-40.0, 0.0, 40.0] {
                let what = format!("{rate:?} {octets} {ppm}");
                let (adc, _, starts) = train(&[(*rate, octets, 30.0, ppm)], 0, 20 + k as u64);
                let (run, s) = closed_loop(&adc, DataTiming::default());
                held_to_the_model(&run, &s, &what);
                let end = starts[0] + 400 + 80 * data_symbols(*rate, octets);
                let err = age_errors(&run, &s, &[end], &what);
                worst = worst.max(err[0].abs());
                assert!(err[0].abs() <= AGE_TOLERANCE, "{what}: {err:?}");
            }
        }
    }
    println!("the worst age against the frame's true end: {worst} samples");
}

#[test]
fn in_a_train_each_frame_has_its_own_age() {
    // Frames behind each other at the shortest gap: a later packet's stamp must not reach an
    // earlier one's end.
    let mut frames = vec![(Rate::R54, 1500, 30.0, 0.0), (Rate::R6, 1, 30.0, 0.0)];
    for rate in RATES {
        frames.push((rate, 24, 30.0, 0.0));
    }
    let (adc, _, starts) = train(&frames, 320, 4);
    let (run, s) = closed_loop(&adc, DataTiming::default());
    held_to_the_model(&run, &s, "a train");
    let ends: Vec<usize> = frames
        .iter()
        .zip(&starts)
        .map(|(f, st)| st + 400 + 80 * data_symbols(f.0, f.1))
        .collect();
    let err = age_errors(&run, &s, &ends, "a train");
    println!("a train: the ages against the true ends {err:?}");
    assert!(err.iter().all(|e| e.abs() <= AGE_TOLERANCE), "{err:?}");
}

/// The cycle sample n of the run was at the ADC's pins (X(n) = T(n) − 6).
fn at_pins(run: &HandoffRun, n: usize) -> usize {
    run.ticks[n] - HandoffTiming::default().tick()
}

#[test]
fn with_the_front_every_rate_decodes_and_the_latency_is_what_the_table_says() {
    let mut worst_verdict = 0.0f64;
    let mut worst_last = 0.0f64;
    let mut best_last = f64::MAX;
    for (k, rate) in RATES.iter().enumerate() {
        for octets in [1usize, 24, 300] {
            for ppm in [-40.0, 40.0] {
                let what = format!("{rate:?} {octets} {ppm}");
                let (adc, psdus, starts) = train(&[(*rate, octets, 30.0, ppm)], 0, 10 + k as u64);
                let (run, s) = closed_loop(&adc, DataTiming::default());
                held_to_the_model(&run, &s, &what);
                let got: Vec<u8> = s
                    .events()
                    .iter()
                    .filter_map(|e| match e.1 {
                        RxDataEvent::Octet { octet, .. } => Some(octet),
                        _ => None,
                    })
                    .collect();
                assert_eq!(got, psdus[0], "{what}");
                // The frame at the antenna: SIGNAL ends 400 samples after its start (short and
                // long training, SIGNAL), the frame 80 per data symbol later (to a sample
                // at these offsets).
                let signal_end = starts[0] + 400;
                let frame_end = signal_end + 80 * data_symbols(*rate, octets);
                let verdict = s
                    .events()
                    .iter()
                    .find(|e| matches!(e.1, RxDataEvent::Verdict { .. }))
                    .unwrap()
                    .0;
                let last = s.last_event();
                let us = |c: usize, n: usize| (c as f64 - at_pins(&run, n) as f64) / 160.0;
                let v = us(verdict, signal_end);
                let l = us(last, frame_end);
                worst_verdict = worst_verdict.max(v);
                worst_last = worst_last.max(l);
                best_last = best_last.min(l);
                println!("{what}: verdict {v:.2} µs after SIGNAL's end, last octet {l:.2} µs after the frame's");
            }
        }
    }
    println!(
        "worst: the verdict {worst_verdict:.2} µs after SIGNAL; the last octet {best_last:.2} … {worst_last:.2} µs after the frame"
    );
    // ADR 0010's estimates: 6.9 to 7.5 µs to the verdict, 5.6 to 6.2 µs to the last octet of
    // a one-symbol frame; at 2.6 times real time the longer frames are caught up with.
    assert!(worst_verdict < 8.0, "{worst_verdict}");
    assert!(worst_last < 7.0, "{worst_last}");
}

#[test]
fn a_frame_after_a_long_one_waits_in_the_history_and_decodes() {
    // The longest back end (54 Mb/s: 214 trellis steps a symbol and the flush) and the
    // shortest gap between frames (a SIFS, 16 µs); then frames at every rate behind each
    // other.
    let mut frames = vec![(Rate::R54, 1500, 30.0, 20.0), (Rate::R54, 24, 30.0, 20.0)];
    for rate in RATES {
        frames.push((rate, 100, 30.0, 20.0));
    }
    let (adc, psdus, _) = train(&frames, 320, 3);
    let (run, s) = closed_loop(&adc, DataTiming::default());
    held_to_the_model(&run, &s, "a train");
    let mut decoded = Vec::new();
    let mut octets = Vec::new();
    for e in s.events() {
        match e.1 {
            RxDataEvent::Octet { octet, .. } => octets.push(octet),
            RxDataEvent::End(RxDataEnd::Decoded) => decoded.push(std::mem::take(&mut octets)),
            RxDataEvent::End(_) => octets.clear(),
            _ => {}
        }
    }
    assert_eq!(decoded, psdus);
    // The closest the reader comes to the history's end is at a first beat, the
    // acquisition's own lag: the same as for a frame alone. The wait of rule 5 adds nothing.
    let (alone, _, _) = train(&frames[1..2], 0, 3);
    let (lone, _) = closed_loop(&alone, DataTiming::default());
    println!(
        "history to spare at the closest read: {} of {ADC_HISTORY} (a frame alone: {})",
        run.history_spare, lone.history_spare
    );
    assert!(
        run.history_spare + 16 >= lone.history_spare,
        "{}",
        run.history_spare
    );
}

#[test]
fn a_stronger_frame_inside_the_data_aborts_the_first_in_order() {
    // Frame B, 10 dB stronger, starts in frame A's data: the hand-off's abort mark ends A
    // (after the octets of the windows A had taken), and B's stream waits until A's end is
    // out.
    let (a, _, _) = train(&[(Rate::R6, 300, 25.0, 10.0)], 0, 5);
    let (b, psdus, _) = train(&[(Rate::R24, 100, 25.0, -10.0)], 0, 6);
    let at = 3000;
    let mut adc = a.clone();
    for (k, s) in b.iter().enumerate() {
        let v = (i32::from(s.0) * 3, i32::from(s.1) * 3);
        let clip = |x: i32| x.clamp(-2048, 2047) as i16;
        if let Some(x) = adc.get_mut(at + k) {
            *x = (clip(i32::from(x.0) + v.0), clip(i32::from(x.1) + v.1));
        } else {
            adc.push((clip(v.0), clip(v.1)));
        }
    }
    let (run, s) = closed_loop(&adc, DataTiming::default());
    held_to_the_model(&run, &s, "a burst");
    let ends: Vec<(usize, RxDataEnd)> = s
        .events()
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::End(x) => Some((e.0, x)),
            _ => None,
        })
        .collect();
    assert_eq!(ends.len(), 2, "{ends:?}");
    assert_eq!(ends[0].1, RxDataEnd::Aborted(HandoffEnd::Acquisition));
    assert_eq!(ends[1].1, RxDataEnd::Decoded);
    // B's first beat is taken only after A's end is out.
    let mark = run
        .beats
        .iter()
        .position(|b| b.1 == HandoffBeat::End(HandoffEnd::Acquisition))
        .unwrap();
    let first_b = run.beats[mark + 1].0;
    assert!(first_b > ends[0].0, "{first_b} {:?}", ends[0]);
    let got: Vec<u8> = s
        .events()
        .iter()
        .skip_while(|e| !matches!(e.1, RxDataEvent::End(_)))
        .filter_map(|e| match e.1 {
            RxDataEvent::Octet { octet, .. } => Some(octet),
            _ => None,
        })
        .collect();
    assert_eq!(got, psdus[0]);
}

#[test]
fn a_stream_cut_after_a_window_waits_for_the_last_ones_octets() {
    // Frame A's stream cut by an abort mark just after a data window is complete, frame B's
    // stream right behind the mark: A's end comes after the octets of the windows it took,
    // and B's first beat waits for it (rule 5). The products of A's next symbol wait for
    // the soft pass of the one before to have read the store.
    let (a, _, _) = train(&[(Rate::R6, 300, 30.0, 0.0)], 0, 7);
    let (b, psdus, _) = train(&[(Rate::R54, 100, 30.0, 0.0)], 0, 8);
    let a = first_stream(&a);
    let b = first_stream(&b);
    // Data symbol 6's window ends at sample 144 + 80·6 + 63.
    let cut = 144 + 80 * 6 + 64;
    let mut beats: Vec<HandoffBeat> = a[..cut].to_vec();
    beats.push(HandoffBeat::End(HandoffEnd::Acquisition));
    beats.extend_from_slice(&b);
    let mut s = schedule(DataTiming::default());
    let mut clock = 0;
    let mut i = 0;
    let mut taken = Vec::new();
    while i < beats.len() {
        if s.ready_at(clock) {
            s.take(clock, beats[i]);
            taken.push(clock);
            i += 1;
        }
        clock += 1;
    }
    let ends: Vec<(usize, RxDataEnd)> = s
        .events()
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::End(x) => Some((e.0, x)),
            _ => None,
        })
        .collect();
    assert_eq!(ends[0].1, RxDataEnd::Aborted(HandoffEnd::Acquisition));
    let mark = taken[cut];
    assert!(
        ends[0].0 > mark + 100,
        "the end waits for the octets: {} {mark}",
        ends[0].0
    );
    // Exactly when the complete windows are decoded: the last step's bit (+ 1 + 4), an
    // octet's latency (+ 1), seen a cycle later; not the last octet's cycle.
    // A's symbols: up to B's SIGNAL (both streams carry the same tag here).
    let b_signal = s
        .symbols()
        .iter()
        .skip(1)
        .position(|r| r.symbol == 0)
        .unwrap()
        + 1;
    let last_step = s.symbols()[..b_signal]
        .iter()
        .filter_map(|r| r.steps)
        .map(|st| st.1)
        .max()
        .unwrap();
    assert_eq!(ends[0].0, last_step + 7);
    let last_octet = s
        .events()
        .iter()
        .take_while(|e| !matches!(e.1, RxDataEvent::End(_)))
        .filter(|e| matches!(e.1, RxDataEvent::Octet { .. }))
        .last()
        .unwrap()
        .0;
    assert!(last_octet < ends[0].0, "{last_octet}");
    assert!(
        taken[cut + 1] > ends[0].0,
        "B waits: {} {}",
        taken[cut + 1],
        ends[0].0
    );
    assert_eq!(ends[1].1, RxDataEnd::Decoded);
    let got: Vec<u8> = s
        .events()
        .iter()
        .skip_while(|e| !matches!(e.1, RxDataEvent::End(_)))
        .filter_map(|e| match e.1 {
            RxDataEvent::Octet { octet, .. } => Some(octet),
            _ => None,
        })
        .collect();
    assert_eq!(got, psdus[0]);
}

#[test]
fn a_mark_after_signals_window_waits_for_the_verdict() {
    // Frame A's stream cut by an abort mark after SIGNAL's window but long before its
    // verdict: the verdict's strobe and the start come as without the mark, the end in the
    // same cycle (after them), and B's first beat after it.
    let (a, _, _) = train(&[(Rate::R54, 100, 30.0, 0.0)], 0, 9);
    let (b, psdus, _) = train(&[(Rate::R24, 50, 30.0, 0.0)], 0, 10);
    let a = first_stream(&a);
    let b = first_stream(&b);
    let cut = 218;
    let mut beats: Vec<HandoffBeat> = a[..cut].to_vec();
    beats.push(HandoffBeat::End(HandoffEnd::Acquisition));
    beats.extend_from_slice(&b);
    let mut s = schedule(DataTiming::default());
    let taken = offer(&mut s, &beats, 0);
    assert!(s.violations().is_empty(), "{:?}", s.violations());
    let ev = s.events().to_vec();
    assert!(matches!(ev[0].1, RxDataEvent::Verdict { .. }));
    assert!(matches!(ev[1].1, RxDataEvent::Start { .. }));
    assert_eq!(
        ev[2].1,
        RxDataEvent::End(RxDataEnd::Aborted(HandoffEnd::Acquisition))
    );
    let mark = taken[cut];
    assert!(ev[0].0 > mark + 100, "{} {mark}", ev[0].0);
    assert_eq!((ev[1].0, ev[2].0), (ev[0].0, ev[0].0));
    let RxDataEvent::Verdict { packet, .. } = ev[0].1 else {
        unreachable!()
    };
    assert_eq!(s.strobe_at(ev[0].0).map(|v| v.0), Some(packet));
    assert!(taken[cut + 1] > ev[2].0);
    let got: Vec<u8> = ev[3..]
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::Octet { octet, .. } => Some(octet),
            _ => None,
        })
        .collect();
    assert_eq!(got, psdus[0]);
}

#[test]
fn a_seed_of_zero_ends_with_the_frames_age() {
    // A transmitter's seed of 0: SERVICE and the PSDU sent clear, so SERVICE's first seven
    // bits are all zero and the receiver finds no seed; the seed-zero end comes where the last
    // octet would have been and carries the age too (the founder, 2026-11-25).
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    for rate in [Rate::R6, Rate::R54] {
        let octets = 24;
        let mut clear = vec![0u8; 16];
        for _ in 0..octets {
            clear.extend((0..8).map(|i| (0x5au8 >> i) & 1));
        }
        clear.resize(data_symbols(rate, octets) * rate.data_bits_per_symbol(), 0);
        let f: Vec<(f64, f64)> = build_ppdu_bits(&clear, rate, octets, &cfg)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let p = mean_power(&f);
        let mut x = vec![(0.0, 0.0); 400];
        x.extend(f);
        x.extend(vec![(0.0, 0.0); 600]);
        let x = awgn(&x, 30.0, p, &mut Rng::new(5));
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p.sqrt();
        let adc = to_adc(&x, gain);
        let (run, s) = closed_loop(&adc, DataTiming::default());
        held_to_the_model(&run, &s, "seed 0");
        let ends: Vec<(usize, RxDataEnd)> = s
            .events()
            .iter()
            .filter_map(|e| match e.1 {
                RxDataEvent::End(end) => Some((e.0, end)),
                _ => None,
            })
            .collect();
        assert_eq!(ends.len(), 1, "{rate:?}: {ends:?}");
        assert_eq!(ends[0].1, RxDataEnd::Seed, "{rate:?}");
        let written = run
            .ticks
            .iter()
            .filter(|&&t| t - HandoffTiming::default().tick() < ends[0].0)
            .count();
        let end = 400 + 400 + 80 * data_symbols(rate, octets);
        let age = rx_end_ages(&run, &s)[0].1.unwrap();
        assert_eq!(i64::from(age), written as i64 - end as i64, "{rate:?}");
    }
}

#[test]
fn a_pause_of_the_adc_after_a_frame_does_not_take_the_last_frames_stamp() {
    // The ADC stops for 3000 clocks a few samples after the second frame's end: no tick comes,
    // while the data path decodes the frame from the history and ends it. Its age must be its
    // own frame's (the independent review of 2026-11-25: a stamp taken at the tick that
    // applies the verdict was the first frame's here).
    let frames = [(Rate::R6, 24, 30.0, 0.0), (Rate::R54, 1, 30.0, 0.0)];
    let (adc, _, starts) = train(&frames, 320, 4);
    let ends: Vec<usize> = frames
        .iter()
        .zip(&starts)
        .map(|(f, st)| st + 400 + 80 * data_symbols(f.0, f.1))
        .collect();
    let mut worst = 0i64;
    for after in [1usize, 6, 20, 60, 120] {
        let gap_at = ends[1] + after;
        let (run, s) = closed_loop_idle(&adc, DataTiming::default(), &|n| {
            if n == gap_at {
                3000
            } else {
                0
            }
        });
        let what = format!("a pause {after} samples after the frame");
        held_to_the_model(&run, &s, &what);
        let err = age_errors(&run, &s, &ends, &what);
        worst = worst.max(err.iter().map(|e| e.abs()).max().unwrap());
        assert!(
            err.iter().all(|e| e.abs() <= AGE_TOLERANCE),
            "{what}: {err:?}"
        );
    }
    println!("pauses: the worst age error {worst}");
}
