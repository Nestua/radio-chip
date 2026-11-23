//! Gate for the data path's schedule in clocks (ADR 0010, decision 6.1c):
//! `rx_data::RxDataSchedule` reads the hand-off's stream a beat a clock when it may, and
//! gives `RxDataFixed`'s events, each in a cycle the schedule says.
//!
//! References: the events are `RxDataFixed`'s on the beats the schedule took (gated in
//! `g2_rx_data_stream.rs`); the cycles are the schedule's table worked by hand (the module's
//! text and the G0 in `docs/gates/rx_ofdm.md`); the hand-off run with this reader is
//! `run_handoff`'s (gated in `g2_rx_handoff_schedule.rs`). A model gate: no reference from
//! outside the project.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, data_symbols, Rate};
use radio_phy::rx_data::{DataTiming, Division, RxDataSchedule, Window};
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
fn train(
    frames: &[(Rate, usize, f64, f64)],
    gap: usize,
    seed: u64,
) -> Train {
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

/// A beat offered every cycle from `c0`, taken when the schedule is ready.
fn offer(s: &mut RxDataSchedule, beats: &[HandoffBeat], c0: usize) {
    let mut clock = c0;
    let mut i = 0;
    while i < beats.len() {
        if s.ready_at(clock) {
            s.take(clock, beats[i]);
            i += 1;
        }
        clock += 1;
        assert!(
            clock < c0 + 100 * beats.len() + 100_000,
            "stalled at beat {i}"
        );
    }
}

#[test]
fn the_table_of_one_symbol_at_54_mbps_worked_by_hand() {
    let (adc, psdus, _) = train(&[(Rate::R54, 24, 30.0, 0.0)], 0, 1);
    let beats = first_stream(&adc);
    let c0 = 1000;
    let mut s = schedule(DataTiming::default());
    offer(&mut s, &beats, c0);
    let rel = |c: usize| c - c0;

    let w: Vec<_> = s
        .windows()
        .iter()
        .map(|w| (w.window, rel(w.first), rel(w.last), rel(w.fft), rel(w.bins)))
        .collect();
    assert_eq!(
        w,
        [
            (Window::T1, 0, 63, 64, 160),
            (Window::T2, 64, 127, 160, 256),
            (Window::Symbol(0), 144, 207, 256, 352),
            (Window::Symbol(1), 224, 287, 352, 448),
        ]
    );
    let d: Vec<_> = s
        .divisions()
        .iter()
        .map(|d| (d.job, rel(d.asked), rel(d.start), rel(d.done)))
        .collect();
    assert_eq!(
        d,
        [
            (Division::Scale(Modulation::Bpsk), 366, 366, 386),
            (Division::Outer, 366, 386, 402),
            (Division::Inner, 366, 402, 418),
            (Division::Variance, 366, 418, 432),
            (Division::Fit(1), 499, 499, 529),
            (Division::Scale(Modulation::Qam64), 489, 529, 551),
        ]
    );
    let sig = s.symbols()[0];
    assert_eq!(rel(sig.products), 366);
    assert_eq!((rel(sig.phi), rel(sig.scale)), (398, 386));
    assert_eq!((rel(sig.outer.0), rel(sig.outer.1)), (376, 416));
    assert_eq!((rel(sig.inner.0), rel(sig.inner.1)), (377, 434));
    assert_eq!((rel(sig.soft.0), rel(sig.soft.1)), (399, 455));
    let (a, b) = sig.steps.unwrap();
    assert_eq!((rel(a), rel(b)), (456, 479));
    let data = s.symbols()[1];
    assert_eq!(rel(data.products), 448);
    assert_eq!((rel(data.outer.0), rel(data.outer.1)), (458, 476));
    assert_eq!((rel(data.inner.0), rel(data.inner.1)), (459, 494));
    assert_eq!(rel(data.theta), 531);
    assert_eq!((rel(data.phi), rel(data.scale)), (557, 551));
    assert_eq!((rel(data.soft.0), rel(data.soft.1)), (558, 614));
    let (a, b) = data.steps.unwrap();
    assert_eq!((rel(a), rel(b)), (615, 828));

    // The events: the verdict and the start at 489, the octets, the end with the last.
    let ev = s.events();
    assert!(matches!(ev[0].1, RxDataEvent::Verdict { .. }));
    assert_eq!(rel(ev[0].0), 489);
    assert!(matches!(ev[1].1, RxDataEvent::Start { .. }));
    assert_eq!(rel(ev[1].0), 489);
    let octets: Vec<(usize, u8)> = ev
        .iter()
        .filter_map(|e| match e.1 {
            RxDataEvent::Octet { octet, .. } => Some((rel(e.0), octet)),
            _ => None,
        })
        .collect();
    assert_eq!(octets.iter().map(|o| o.1).collect::<Vec<_>>(), psdus[0]);
    assert_eq!(octets[0].0, 737);
    assert_eq!(octets[11].0, 825);
    assert_eq!(octets[12].0, 834);
    assert_eq!(octets[23].0, 922);
    // Bits 0 … 117 a step apart, the flush a bit a clock: octets eight clocks apart, but
    // the flush's first bit a clock after the last step's (the best state's tree, + 1).
    for j in 1..24 {
        let gap = if j == 12 { 9 } else { 8 };
        assert_eq!(octets[j].0 - octets[j - 1].0, gap, "octet {j}");
    }
    let end = ev
        .iter()
        .find(|e| e.1 == RxDataEvent::End(RxDataEnd::Decoded))
        .unwrap();
    assert_eq!(rel(end.0), 922);
}

#[test]
fn a_long_frame_runs_at_one_symbol_in_245_clocks_whatever_the_rate() {
    // Beats offered every cycle (the reader behind the antenna): the loop from one shift to
    // the next is the window (1 + 64), the transform (1 + 96), the pilots (3 + 3), the pair
    // products (2 + 3 + 1), two angles (36), the fit (5 + 30 + 2): 245 clocks.
    for rate in RATES {
        let (adc, _, _) = train(&[(rate, 1500, 30.0, 0.0)], 0, 2);
        let beats = first_stream(&adc);
        let mut s = schedule(DataTiming::default());
        offer(&mut s, &beats, 0);
        let thetas: Vec<usize> = s.symbols().iter().skip(1).map(|r| r.theta).collect();
        assert!(thetas.len() > 50, "{rate:?}");
        for (i, w) in thetas.windows(2).enumerate().skip(1) {
            assert_eq!(w[1] - w[0], 245, "{rate:?}, data symbol {}", i + 2);
        }
        assert_eq!(
            s.events().last().unwrap().1,
            RxDataEvent::End(RxDataEnd::Decoded),
            "{rate:?}"
        );
    }
}

/// The run with the schedule as the hand-off's reader, and what it gave.
fn closed_loop(adc: &[(i16, i16)], t: DataTiming) -> (HandoffRun, RxDataSchedule) {
    let mut s = schedule(t);
    let run = run_handoff(
        adc,
        &RxConfig::new(FS),
        &FixedRxConfig::default(),
        &HandoffConfig::default(),
        &HandoffTiming::default(),
        &|_| 0,
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
fn a_window_waits_for_a_store_and_the_soft_buffer_for_the_decoder() {
    // One window store: SIGNAL's first sample waits for T2's transform to start and read.
    let one = DataTiming {
        window_stores: 1,
        ..DataTiming::default()
    };
    let (adc, _, _) = train(&[(Rate::R54, 24, 30.0, 0.0)], 0, 1);
    let beats = first_stream(&adc);
    let mut s = schedule(one);
    offer(&mut s, &beats, 0);
    let w = s.windows();
    // T2's first sample after T1's store is free (64 + 64), SIGNAL's after T2's.
    assert_eq!(w[1].first, w[0].fft + 64);
    assert_eq!(w[2].first, w[1].fft + 64);
    // A slow decoder: one soft slot; the next symbol's soft pass waits for its steps.
    let slow = DataTiming {
        soft_symbols: 1,
        ..DataTiming::default()
    };
    let (adc, _, _) = train(&[(Rate::R54, 600, 30.0, 0.0)], 0, 4);
    let beats = first_stream(&adc);
    let mut s = schedule(slow);
    offer(&mut s, &beats, 0);
    let mut store_waits = 0;
    for p in s.symbols().windows(2).skip(1) {
        assert!(p[1].soft.0 > p[0].steps.unwrap().1, "{p:?}");
        // The products' store: the next symbol's products after this one's soft pass.
        assert!(p[1].products >= p[0].soft.0 + 48, "{p:?}");
        let bins = s
            .windows()
            .iter()
            .find(|w| w.window == Window::Symbol(p[1].symbol));
        if p[1].products > bins.unwrap().bins {
            store_waits += 1;
        }
    }
    assert!(store_waits > 10, "{store_waits}");
    assert_eq!(
        s.events().last().unwrap().1,
        RxDataEvent::End(RxDataEnd::Decoded)
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
