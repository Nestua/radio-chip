//! Gate for the cycle-exact schedule of the acquisition decision (R0-08d part d step 3, ADR
//! 0008 decision 4): `rx_decision::run_schedule` makes every decision of `RxFrontFixed` from
//! memories read at the clocks the schedule says, and reports when it ends.
//!
//! References: the decisions are those of `RxFrontFixed::push` (`decide`, gated in
//! `g2_rx_front.rs`), bit for bit; the clock counts are the schedule's table worked by hand
//! (G0 in `docs/gates/rx_front.md`): 8·w + 263 clocks at a fold of 4, against a budget of
//! (L − D)·R − 22 + 3.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{add_tone, awgn, fractional_delay, frequency_offset, mean_power, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_decision::{
    run_schedule, schedule_len, DecisionRun, DecisionTiming, ScheduleError,
};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    acquisitions_fixed, to_adc, FixedAcquisition, FixedRxConfig, ADC_FULL_SCALE,
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
        assert!(!model.is_empty(), "{name}: the scene has no acquisition");
        // Every decision whose result falls inside the stream, in order; a rejected one is
        // a run with no result.
        assert_eq!(accepted(&runs), model, "{name}");
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
fn a_decision_ends_at_the_clock_the_schedule_says() {
    let t = DecisionTiming::default();
    // 8·w + 263 clocks from the acceptance strobe to the registered result, fold 4.
    assert_eq!(schedule_len(12, &t), 359);
    assert_eq!(schedule_len(16, &t), 391);
    assert_eq!(schedule_len(8, &t), 327);
    // A fold of 8 (16 multipliers) adds 4 clocks to each of the 2·w − 1 correlations that
    // hold the window, and one to the last.
    let slow = DecisionTiming { fold: 8, ..t };
    assert_eq!(schedule_len(12, &slow), 359 + 4 * 23 + 4);
    // Where the fine sum is the long pole: w + 296 + 1.
    assert_eq!(schedule_len(2, &t), 2 + 296 + 1);

    for (name, x, backoff) in scenes() {
        let cfg = with_backoff(backoff);
        let w = 16 - (backoff / 50.0) as usize;
        for r in run(&x, &cfg, &t) {
            // No latch asked for the CORDIC in the clock a decision did, or it did and the
            // decision waited one clock for each.
            assert_eq!(r.done, schedule_len(w, &t) + r.coarse_wait, "{name}");
            assert!(r.coarse_wait <= 1 && r.fine_wait <= 1, "{name}");
            // Samples every 8 clocks, D = 4, L = 64: (64 − 4)·8 − (5 + 14 + 3) + (4 − 1).
            assert_eq!(r.budget, 461, "{name}");
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
            assert_eq!(r.done, 359 + r.coarse_wait);
            coarse += r.coarse_wait;
            fine += r.fine_wait;
        }
    }
    // Each decision met the latch once with each of its two requests.
    assert_eq!((coarse, fine), (plain.len(), plain.len()));
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
        // earliest n − 189 − 2·w. It is read w + 3 clocks after the strobe, itself D = 4
        // samples and 22 clocks behind sample n: the newest sample written by then is
        // n + 4 + (22 + w + 3 − 1) / 8.
        let spare = runs.iter().map(|r| r.history_spare).min().unwrap();
        let oldest = 349 + 2 * w + 4 + (24 + w) / 8;
        assert!(spare >= 512 - (oldest + 1), "{name}: {spare}");
        assert!(spare <= 512 - (oldest + 1) + w, "{name}: {spare}");
        // The energy ring: position imax = j − 63 − w is read first, in the strobe's clock.
        let ring = runs.iter().map(|r| r.ring_spare).min().unwrap();
        assert_eq!(ring, 81 - (64 + w), "{name}");

        // One slot fewer than what is needed, and the schedule reads a sample that is gone.
        let short = DecisionTiming {
            history_depth: 512 - spare - 1,
            ..t
        };
        assert!(
            matches!(
                run_schedule(&x, &cfg, &fx, &short, &no_idle, &no_latch),
                Err(ScheduleError::Stale { .. })
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
                Err(ScheduleError::Stale { .. })
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
    for (a, b) in runs.iter().zip(&plain) {
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
    // The decision's correlator folded over 8: 455 clocks of 461 — it fits, by 6 clocks.
    let f8 = late(DecisionTiming { fold: 8, ..t }).unwrap();
    assert!(f8.iter().all(|r| r.done == 455 && r.budget == 461));
    // 6 clocks per sample: (64 − 4)·6 − 19 = 341 clocks for 359.
    assert_eq!(
        late(DecisionTiming {
            clocks_per_sample: 6,
            ..t
        })
        .unwrap_err(),
        ScheduleError::Late
    );
    // The NCO path 16 samples behind leaves 365 clocks, 17 leave 357.
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
    // A model latency of 52 samples leaves 365 clocks, 51 leaves 357.
    let mut short = cfg.clone();
    short.decision_latency = 52;
    assert!(run_schedule(&x, &short, &fx, &t, &no_idle, &no_latch).is_ok());
    short.decision_latency = 51;
    assert_eq!(
        run_schedule(&x, &short, &fx, &t, &no_idle, &no_latch).unwrap_err(),
        ScheduleError::Late
    );
}
