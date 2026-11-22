//! Gate for the cycle-exact schedule of the acquisition decision (R0-08d part d step 3, ADR
//! 0008 decision 4): `rx_decision::run_schedule` makes every decision of `RxFrontFixed` from
//! memories read at the clocks the schedule says, and reports when it ends.
//!
//! References: the decisions are those of `RxFrontFixed::push` (`decide`, gated in
//! `g2_rx_front.rs`), bit for bit; the clock counts are the schedule's table worked by hand
//! (G0 in `docs/gates/rx_front.md`): 8·w + 268 clocks at a fold of 4, against a budget of
//! (L − D)·R − 23 + 3.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{add_tone, awgn, fractional_delay, frequency_offset, mean_power, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_decision::{
    run_schedule, schedule_len, DecisionRun, DecisionTiming, ScheduleError, Store,
};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    acquisitions_fixed, ltf_taps, to_adc, DecisionSums, FixedAcquisition, FixedRxConfig,
    RxFrontFixed, ADC_FULL_SCALE,
};
use std::f64::consts::PI;

type C = (f64, f64);

const FS: f64 = 20e6;

fn frame(rate: Rate, octets: usize) -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(&vec![0x5a; octets], rate, 0x5d, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

/// `frames` frames of 40 octets behind random leads, each with its own CFO, in noise at
/// `snr_db`, the packet at −15 dBFS.
fn frames_scene(frames: usize, snr_db: f64, seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let f = frame(Rate::R6, 40);
    let p = mean_power(&f);
    let mut x = Vec::new();
    for _ in 0..frames {
        let mut part = vec![(0.0, 0.0); 200 + (rng.uniform() * 300.0) as usize];
        part.extend_from_slice(&f);
        part.extend(vec![(0.0, 0.0); 300]);
        let part = fractional_delay(&part, rng.uniform());
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        x.extend(frequency_offset(&part, cfo, FS, 2.0 * PI * rng.uniform()));
    }
    let x = awgn(&x, snr_db, p, &mut rng);
    let rms = (p * (1.0 + 10f64.powf(-snr_db / 10.0))).sqrt();
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / rms,
    )
}

/// The scene of `g2_rx_front`'s failed-candidate gate: a strong STF fragment and a noise
/// burst (a candidate that fails confirmation), then a frame 10 dB weaker; `n` times.
fn failed_candidate_scene(n: usize, seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let f = frame(Rate::R6, 3);
    let stf: Vec<C> = f[..160]
        .iter()
        .map(|&(a, b)| (a * 3.16, b * 3.16))
        .collect();
    let p = mean_power(&f);
    let mut x = Vec::new();
    for _ in 0..n {
        let mut part = vec![(0.0, 0.0); 200 + (rng.uniform() * 200.0) as usize];
        part.extend_from_slice(&stf);
        part.extend(awgn(&vec![(0.0, 0.0); 120], 0.0, 10.0 * p, &mut rng));
        part.extend(vec![(0.0, 0.0); 20 + (rng.uniform() * 40.0) as usize]);
        part.extend_from_slice(&f);
        part.extend(vec![(0.0, 0.0); 400]);
        x.extend(frequency_offset(&part, 50e3, FS, 2.0 * PI * rng.uniform()));
    }
    let x = awgn(&x, 15.0, p, &mut rng);
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-25.0 / 20.0) / p.sqrt(),
    )
}

/// Frames under a tone of their own power: the detector keeps firing, the search stays
/// open, and most candidates are rejected.
fn tone_scene(seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let f = frame(Rate::R6, 40);
    let p = mean_power(&f);
    let mut x = Vec::new();
    for _ in 0..8 {
        x.extend(vec![(0.0, 0.0); 400]);
        x.extend_from_slice(&f);
    }
    x.extend(vec![(0.0, 0.0); 400]);
    let x = add_tone(&x, p, 1.3e6, FS, 0.3);
    let x = awgn(&x, 25.0, p, &mut rng);
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-18.0 / 20.0) / (2.0 * p).sqrt(),
    )
}

/// Frames whose second long symbol is negated: the search finds no maximum at T1 and the
/// front settles a symbol away (the model's decisions on a preamble that is not one).
fn negated_t2_scene(n: usize, seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let mut f = frame(Rate::R6, 40);
    for v in &mut f[256..320] {
        *v = (-v.0, -v.1);
    }
    let p = mean_power(&f);
    let mut x = Vec::new();
    for _ in 0..n {
        x.extend(vec![(0.0, 0.0); 300 + (rng.uniform() * 100.0) as usize]);
        x.extend_from_slice(&f);
    }
    x.extend(vec![(0.0, 0.0); 400]);
    let x = awgn(&x, 25.0, p, &mut rng);
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p.sqrt(),
    )
}

/// A stream that begins 40 samples into a frame's short training field (its T1 is at
/// sample 152, and the final coarse window would begin before the stream), then frames.
fn cut_scene(seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let f = frame(Rate::R6, 40);
    let p = mean_power(&f);
    let mut x = f[40..].to_vec();
    for _ in 0..3 {
        x.extend(vec![(0.0, 0.0); 300]);
        x.extend_from_slice(&f);
    }
    x.extend(vec![(0.0, 0.0); 400]);
    let x = awgn(&x, 25.0, p, &mut rng);
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p.sqrt(),
    )
}

/// The model's own decisions: the sample of each acceptance whose result falls inside the
/// stream, and the sums it was taken from.
fn model_sums(x: &[(i16, i16)], cfg: &RxConfig) -> Vec<(usize, Option<DecisionSums>)> {
    let mut rx = RxFrontFixed::new(cfg.clone(), FixedRxConfig::default());
    let mut out = Vec::new();
    for (n, s) in x.iter().enumerate() {
        let t = rx.push_traced(*s).1;
        if t.accepted.is_some() && n + cfg.decision_latency < x.len() {
            out.push((n, t.sums));
        }
    }
    out
}

fn with_backoff(ns: f64) -> RxConfig {
    let mut cfg = RxConfig::new(FS);
    cfg.backoff_ns = ns;
    cfg
}

fn no_idle(_: usize) -> usize {
    0
}

fn no_latch(_: usize) -> bool {
    false
}

fn run(x: &[(i16, i16)], cfg: &RxConfig, t: &DecisionTiming) -> Vec<DecisionRun> {
    run_schedule(x, cfg, &FixedRxConfig::default(), t, &no_idle, &no_latch).unwrap()
}

fn accepted(runs: &[DecisionRun]) -> Vec<FixedAcquisition> {
    runs.iter().filter_map(|r| r.result).collect()
}

/// A scene's name, its ADC samples and its back-off in nanoseconds.
type Scene = (&'static str, Vec<(i16, i16)>, f64);

/// The scenes of the gate with the back-off each is run at: w = GI − B = 12, 16 and 8.
fn scenes() -> Vec<Scene> {
    vec![
        ("frames 30 dB", frames_scene(12, 30.0, 0x51), 200.0),
        ("frames 6 dB", frames_scene(12, 6.0, 0x52), 200.0),
        ("frames 2 dB", frames_scene(12, 2.0, 0x53), 200.0),
        ("failed candidates", failed_candidate_scene(12, 0x54), 200.0),
        ("under a tone", tone_scene(0x55), 200.0),
        ("no back-off", frames_scene(8, 20.0, 0x56), 0.0),
        ("no back-off, failed", failed_candidate_scene(8, 0x57), 0.0),
        ("back-off 400 ns", frames_scene(8, 20.0, 0x58), 400.0),
        ("T2 negated", negated_t2_scene(8, 0x59), 200.0),
        (
            "back-off 50 ns, failed",
            failed_candidate_scene(8, 0x5b),
            50.0,
        ),
        ("cut at its start", cut_scene(0x5a), 200.0),
        ("under a tone, no back-off", tone_scene(0x5c), 0.0),
    ]
}

#[test]
fn the_schedule_makes_the_models_decisions() {
    let fx = FixedRxConfig::default();
    let t = DecisionTiming::default();
    let (mut some, mut none) = (0, 0);
    for (name, x, backoff) in scenes() {
        let cfg = with_backoff(backoff);
        let runs = run(&x, &cfg, &t);
        let model = acquisitions_fixed(&x, &cfg, &fx);
        // Every decision whose result falls inside the stream, in order; a rejected one is
        // a run with no result. And the sums behind each, as far as the model's got.
        assert_eq!(accepted(&runs), model, "{name}");
        let sums: Vec<_> = runs.iter().map(|r| (r.accepted_at, r.sums)).collect();
        assert_eq!(sums, model_sums(&x, &cfg), "{name}");
        assert!(!runs.is_empty(), "{name}: the scene has no decision");
        some += model.len();
        none += runs.iter().filter(|r| r.result.is_none()).count();
    }
    // The gate has seen both ends of a decision.
    assert!(
        some >= 80 && none >= 100,
        "{some} accepted, {none} rejected"
    );
}

#[test]
fn each_way_out_of_a_decision_is_taken() {
    let fx = FixedRxConfig::default();
    let t = DecisionTiming::default();
    let cfg = RxConfig::new(FS);
    let tap_energy: i128 = ltf_taps(&fx)
        .iter()
        .map(|v| i128::from(v.0).pow(2) + i128::from(v.1).pow(2))
        .sum();
    let ways = |x: &[(i16, i16)]| {
        let mut seen = [0usize; 5];
        for r in run(x, &cfg, &t) {
            let way = match (r.sums, r.result) {
                // No final coarse window in the stream.
                (None, _) => 0,
                (Some(_), Some(_)) => 4,
                (Some(s), None) => {
                    let rhs = tap_energy * s.energy;
                    match (2 * s.confirmation >= rhs, 2 * s.coherence >= rhs) {
                        (false, false) => 1,
                        // Each long symbol alone is there, their sum is not.
                        (true, false) => 2,
                        (false, true) => 3,
                        (true, true) => panic!("rejected with both rules met"),
                    }
                }
            };
            seen[way] += 1;
            // A decision that ends early is still given its whole schedule.
            assert_eq!((r.done, r.budget), (364, 460));
        }
        seen
    };
    let cut = ways(&cut_scene(0x5a));
    assert_eq!((cut[0], cut[4]), (1, 3), "{cut:?}");
    let failed = ways(&failed_candidate_scene(12, 0x54));
    assert!(failed[1] >= 12 && failed[4] == 12, "{failed:?}");
    // Under the tone candidates fail the confirmation alone too (the two symbols' sum
    // is there, each symbol's own energy is not enough of the window's). No scene of this
    // gate fails the coherence alone (the search's own correlator already asks for two
    // symbols that add up): the rules' comparison has its own test beside the model
    // (`rx_front_fixed::tests`).
    let tone = ways(&tone_scene(0x55));
    assert!(tone[1] >= 30 && tone[3] >= 5 && tone[4] == 8, "{tone:?}");
    println!("ways out: cut {cut:?}, failed {failed:?}, tone {tone:?}");
}

#[test]
fn a_decision_ends_at_the_clock_the_schedule_says() {
    let t = DecisionTiming::default();
    // 8·w + 268 clocks from the acceptance strobe to the registered result, fold 4.
    assert_eq!(schedule_len(12, &t), 364);
    assert_eq!(schedule_len(16, &t), 396);
    assert_eq!(schedule_len(8, &t), 332);
    // A fold of 8 (16 multipliers) adds 4 clocks to each of the 2·w − 1 correlations that
    // hold the window, and one to the last.
    let slow = DecisionTiming { fold: 8, ..t };
    assert_eq!(schedule_len(12, &slow), 364 + 4 * 23 + 4);
    // Where the fine sum is the long pole: w + 299 and the result's register.
    assert_eq!(schedule_len(2, &t), 2 + 299 + 1);
    assert_eq!(schedule_len(4, &t), 4 + 299 + 1);
    assert_eq!(schedule_len(5, &t), 8 * 5 + 268);

    for (name, x, backoff) in scenes() {
        let cfg = with_backoff(backoff);
        let w = 16 - (backoff / 50.0) as usize;
        for r in run(&x, &cfg, &t) {
            // No latch asked for the CORDIC in the clock a decision did, or it did and the
            // decision waited one clock for each.
            assert_eq!(r.done, schedule_len(w, &t) + r.coarse_wait, "{name}");
            assert!(r.coarse_wait <= 1 && r.fine_wait <= 1, "{name}");
            // Port 0: the coarse sum's 96 words and the 127 + w rotated; port 1: 96 and
            // the fine sum's 176. A decision with no coarse window in the stream reads none.
            let reads = [(0, 0), (96 + 127 + w, 96 + 176)];
            assert_eq!(r.reads, reads[usize::from(r.sums.is_some())], "{name}");
            // Samples every 8 clocks, D = 4, L = 64: (64 − 4)·8 − (5 + 15 + 3) + (4 − 1).
            assert_eq!(r.budget, 460, "{name}");
        }
    }
}

#[test]
fn the_latch_goes_first_at_the_cordic_and_costs_a_clock() {
    let fx = FixedRxConfig::default();
    let t = DecisionTiming::default();
    let cfg = RxConfig::new(FS);
    let x = frames_scene(6, 20.0, 0x61);
    let plain = run(&x, &cfg, &t);
    let (mut coarse, mut fine) = (0, 0);
    // A latch every 4 samples (the detector's run length), at each of its 32 phases: one of
    // them meets the coarse request and one the fine request of every decision.
    for phase in 0..32 {
        let latch = move |clock: usize| clock % 32 == phase;
        let runs = run_schedule(&x, &cfg, &fx, &t, &no_idle, &latch).unwrap();
        assert_eq!(accepted(&runs), accepted(&plain), "phase {phase}");
        for r in &runs {
            // The rotation waits for the coarse angle; the fine angle has clocks to spare.
            assert_eq!(r.done, 364 + r.coarse_wait);
            coarse += r.coarse_wait;
            fine += r.fine_wait;
        }
    }
    // Each decision met the latch once with each of its two requests.
    assert_eq!((coarse, fine), (plain.len(), plain.len()));
    // With no cycle to spare (a model latency of 52 samples leaves 48·8 − 20 = 364), the
    // one wait is one too many.
    let mut tight = cfg.clone();
    tight.decision_latency = 52;
    let nine = t;
    let fits = run_schedule(&x, &tight, &fx, &nine, &no_idle, &no_latch).unwrap();
    assert!(fits.iter().all(|r| (r.done, r.budget) == (364, 364)));
    let late = (0..32)
        .filter(|&phase| {
            let latch = move |clock: usize| clock % 32 == phase;
            run_schedule(&x, &tight, &fx, &nine, &no_idle, &latch) == Err(ScheduleError::Late)
        })
        .count();
    assert!((1..=fits.len()).contains(&late), "{late}");
    // The stream's own latches: under a tone the detector's events go on while decisions
    // are made. An event's latch asks 5 cycles after its sample; with no back-off both of
    // a decision's requests fall in that cycle of a sample (23 + 19 + 99 and 23 + 19 + 96
    // + 179 are 5 more than a multiple of 8), with the default back-off neither does.
    let tone = tone_scene(0x5c);
    let own = |backoff: f64| {
        let runs = run(&tone, &with_backoff(backoff), &t);
        let w = 16 - (backoff / 50.0) as usize;
        for r in &runs {
            assert_eq!(r.done, schedule_len(w, &t) + r.coarse_wait);
        }
        (
            runs.iter().map(|r| r.coarse_wait).sum::<usize>(),
            runs.iter().map(|r| r.fine_wait).sum::<usize>(),
        )
    };
    let (met, missed) = (own(0.0), own(200.0));
    println!("own latches met: {met:?} without back-off, {missed:?} with 200 ns");
    assert!(met.0 > 0 && met.1 > 0, "{met:?}");
    assert_eq!(missed, (0, 0));
    // Below w = 5 the fine sum's branch is the longer one, and its wait is the one that
    // counts (back-off 600 ns, w = 4).
    let narrow = with_backoff(600.0);
    let mut fine = 0;
    for phase in 0..32 {
        let latch = move |clock: usize| clock % 32 == phase;
        for r in run_schedule(&x, &narrow, &fx, &t, &no_idle, &latch).unwrap() {
            assert_eq!(
                r.done,
                304 + r.fine_wait.max(r.coarse_wait.saturating_sub(4))
            );
            fine += r.fine_wait;
        }
    }
    assert!(fine > 0);
    // The latch asking in every clock would starve a decision: the schedule says so.
    let always = |_: usize| true;
    assert_eq!(
        run_schedule(&x, &cfg, &fx, &t, &no_idle, &always).unwrap_err(),
        ScheduleError::Late
    );
}

#[test]
fn the_memories_hold_what_the_schedule_reads() {
    let fx = FixedRxConfig::default();
    let t = DecisionTiming::default();
    for (name, x, backoff) in scenes() {
        let cfg = with_backoff(backoff);
        let w = 16 - (backoff / 50.0) as usize;
        let runs = run(&x, &cfg, &t);
        // The oldest sample read is the coarse sum's first, T1 − 160 with T1 at the
        // earliest n − 189 − 2·w. It is read w + 3 cycles after the strobe, itself D = 4
        // samples and 23 cycles behind sample n: the newest sample written by the end of
        // that cycle is n + 4 + (23 + w + 3) / 8.
        let spare = runs.iter().map(|r| r.history_spare).min().unwrap();
        let oldest = 349 + 2 * w + 4 + (26 + w) / 8;
        assert!(spare >= 512 - (oldest + 1), "{name}: {spare}");
        assert!(spare <= 512 - (oldest + 1) + w, "{name}: {spare}");
        // The energy ring: position imax = j − 63 − w is read first, in the strobe's clock.
        let ring = runs.iter().map(|r| r.ring_spare).min().unwrap();
        assert_eq!(ring, 81 - (64 + w), "{name}");
        if name == "failed candidates" {
            // A scene where T1 is as early as it can be: the bound is met.
            assert_eq!(spare, 512 - (oldest + 1));
            // With two cycles more before the strobe the coarse sum's first read falls in
            // the cycle that writes sample n + 9: the word that write takes is gone a
            // cycle before a later read would lose it.
            let moved = DecisionTiming {
                search_latency: 5,
                ..t
            };
            let runs = run(&x, &cfg, &moved);
            let spare = runs.iter().map(|r| r.history_spare).min().unwrap();
            assert_eq!(spare, 512 - (349 + 2 * w + 4 + 5 + 1));
        }

        // One slot fewer than what is needed, and the schedule reads a sample that is gone.
        let short = DecisionTiming {
            history_depth: 512 - spare - 1,
            ..t
        };
        assert!(
            matches!(
                run_schedule(&x, &cfg, &fx, &short, &no_idle, &no_latch),
                Err(ScheduleError::Stale {
                    store: Store::History,
                    ..
                })
            ),
            "{name}"
        );
        let exact = DecisionTiming {
            history_depth: 512 - spare,
            ring_depth: 81 - ring,
            ..t
        };
        assert_eq!(
            accepted(&run_schedule(&x, &cfg, &fx, &exact, &no_idle, &no_latch).unwrap()),
            accepted(&runs),
            "{name}"
        );
        let short = DecisionTiming {
            ring_depth: 81 - ring - 1,
            ..t
        };
        assert!(
            matches!(
                run_schedule(&x, &cfg, &fx, &short, &no_idle, &no_latch),
                Err(ScheduleError::Stale {
                    store: Store::Ring,
                    ..
                })
            ),
            "{name}"
        );
    }
}

#[test]
fn idle_clocks_between_samples_only_add_slack() {
    let fx = FixedRxConfig::default();
    let t = DecisionTiming::default();
    let cfg = RxConfig::new(FS);
    let x = frames_scene(6, 20.0, 0x71);
    let plain = run(&x, &cfg, &t);
    assert!(plain.len() >= 6);
    // The testbenches' pattern: an idle clock after every 5th sample, two after every 13th.
    let idle = |n: usize| usize::from(n % 5 == 4) + 2 * usize::from(n % 13 == 12);
    let runs = run_schedule(&x, &cfg, &fx, &t, &idle, &no_latch).unwrap();
    assert_eq!(accepted(&runs), accepted(&plain));
    // The strobe is 23 cycles after sample n + 4's own cycle, wherever the idle clocks fell.
    let cycle = |n: usize| (0..=n).map(|k| 8 + idle(k)).sum::<usize>();
    for (a, b) in runs.iter().zip(&plain) {
        assert_eq!(a.strobe, cycle(a.accepted_at + 4) + 23);
        assert_eq!(b.strobe, 8 * (b.accepted_at + 5) + 23);
        assert_eq!(a.budget, cycle(a.accepted_at + 64) + 3 - a.strobe);
        assert_eq!(a.done, b.done);
        assert!(a.budget > b.budget);
        assert!(a.history_spare >= b.history_spare);
    }
}

#[test]
fn what_does_not_fit_is_refused() {
    let fx = FixedRxConfig::default();
    let cfg = RxConfig::new(FS);
    let x = frames_scene(4, 20.0, 0x81);
    let t = DecisionTiming::default();
    let late = |t: DecisionTiming| run_schedule(&x, &cfg, &fx, &t, &no_idle, &no_latch);
    // The decision's correlator folded over 8: 460 clocks of 460, none to spare, and one
    // wait at the CORDIC or one stage more anywhere is late.
    let f8 = late(DecisionTiming { fold: 8, ..t }).unwrap();
    assert!(f8.iter().all(|r| r.done == 460 && r.budget == 460));
    let once = |clock: usize| clock % 32 == 1;
    assert_eq!(
        run_schedule(
            &x,
            &cfg,
            &fx,
            &DecisionTiming { fold: 8, ..t },
            &no_idle,
            &once
        )
        .unwrap_err(),
        ScheduleError::Late
    );
    // 6 clocks per sample: (64 − 4)·6 − 20 = 340 clocks for 364.
    assert_eq!(
        late(DecisionTiming {
            clocks_per_sample: 6,
            ..t
        })
        .unwrap_err(),
        ScheduleError::Late
    );
    // The NCO path 16 samples behind leaves 48·8 − 20 = 364 clocks, 17 leave 356.
    assert!(late(DecisionTiming {
        delay_samples: 16,
        ..t
    })
    .is_ok());
    assert_eq!(
        late(DecisionTiming {
            delay_samples: 17,
            ..t
        })
        .unwrap_err(),
        ScheduleError::Late
    );
    // A detector that answers later than a sample's period (it is to be made serial) does
    // not move the deadline past the sample the result is applied at: D = 5 and an event
    // after 12 cycles leave (64 − 5)·8 − 23 + 7, not + 11.
    let serial = late(DecisionTiming {
        delay_samples: 5,
        det_latency: 12,
        ..t
    })
    .unwrap();
    assert!(serial.iter().all(|r| r.budget == 59 * 8 - 23 + 7));
    // A decision that ends early is refused all the same: the stream cut at its start, up
    // to its first decision and no further.
    let cut = cut_scene(0x5a);
    let slow = DecisionTiming {
        clocks_per_sample: 6,
        ..t
    };
    let first = run(&cut, &cfg, &t)[0];
    assert!(first.sums.is_none() && first.accepted_at < 500);
    let cut = &cut[..first.accepted_at + 65];
    assert_eq!(run(cut, &cfg, &t).len(), 1);
    assert_eq!(
        run_schedule(cut, &cfg, &fx, &slow, &no_idle, &no_latch).unwrap_err(),
        ScheduleError::Late
    );
    // A model latency of 52 samples leaves 364 clocks, 51 leaves 356.
    let mut short = cfg.clone();
    short.decision_latency = 52;
    assert!(run_schedule(&x, &short, &fx, &t, &no_idle, &no_latch).is_ok());
    short.decision_latency = 51;
    assert_eq!(
        run_schedule(&x, &short, &fx, &t, &no_idle, &no_latch).unwrap_err(),
        ScheduleError::Late
    );
}

/// What the schedule is fed from: the trace's energy is the sum of the last GI − B squared
/// correlator outputs, and an acceptance names the maximum's position, 63 + w positions
/// back, and its energy.
#[test]
fn the_models_trace_names_the_energy_and_the_accepted_maximum() {
    for (name, x, backoff) in scenes() {
        let cfg = with_backoff(backoff);
        let w = 16 - (backoff / 50.0) as usize;
        let mut rx = RxFrontFixed::new(cfg.clone(), FixedRxConfig::default());
        let (mut pairs, mut energies, mut seen) = (Vec::new(), Vec::new(), 0);
        let (mut events, mut last_event) = (Vec::new(), 0);
        for (n, s) in x.iter().enumerate() {
            let t = rx.push_traced(*s).1;
            if t.event {
                last_event = n;
                events.push(n);
            }
            if let Some(c) = t.corr {
                pairs.push(i128::from(c.0).pow(2) + i128::from(c.1).pow(2));
            }
            // Position j of the energy is complete at sample j + 126 + w.
            assert_eq!(t.energy.is_some(), n >= 126 + w, "{name} {n}");
            if let Some(e) = t.energy {
                assert_eq!(
                    e,
                    pairs[pairs.len() - w..].iter().sum::<i128>(),
                    "{name} {n}"
                );
                energies.push(e);
            }
            if let Some(a) = t.accepted {
                let j = energies.len() - 1;
                assert_eq!(a.imax + 63 + w, j, "{name} {n}");
                assert_eq!(a.emax, energies[a.imax], "{name} {n}");
                // The deadline is 264 positions after the last event up to this sample,
                // and the search was opened by an event.
                assert_eq!(a.deadline, last_event + 24 + 240, "{name} {n}");
                assert!(a.imax + w < a.deadline, "{name} {n}");
                assert!(events.contains(&a.opened_by), "{name} {n}");
                seen += 1;
            }
            assert!(t.accepted.is_some() || t.sums.is_none(), "{name} {n}");
        }
        assert!(seen > 0, "{name}");
    }
}
