//! G1 gate of the fast-attack gain control model (R0-07e, `radio_phy::channel::FastAttackAgc`).
//!
//! Reference: the published behaviour of the AD9361's fast-attack mode (Analog Devices
//! UG-570, "Fast Attack AGC Mode", Fig. 25 and Table 20), as tabulated in
//! `docs/gates/channel.md` (G0, R0-07e). The states, steps and counts are the document's;
//! the exact sample each falls on is our reading of it (G0 lists what the model defines).
//! The cases below are worked by hand from that table in the comments; the model must give
//! every gain change at its sample. A capture of the chip (R1) is the reference this lacks.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{AgcTrace, FastAttackAgc, Sample};

/// The parameters the hand-worked cases use, all explicit.
fn agc() -> FastAttackAgc {
    FastAttackAgc {
        max_gain_db: 70.0,
        min_gain_db: 0.0,
        both_peak: 2.0,
        large_peak: 1.0,
        small_peak: 0.8,
        step_both: 5.0,
        step_large: 2.0,
        step_small: 2.0,
        peak_wait: 2,
        energy_detect: 4,
        settling_delay: 10,
        power_samples: 16,
        lock_dbfs: -10.0,
        lock_window: 64,
        exit_count: 8,
        stronger_db: 10.0,
        lost_db: 10.0,
        unlock_on_overload: true,
        unlock_to_max: true,
    }
}

/// `len` samples of a constant at `db` dB (amplitude 10^(db/20) on the I rail).
fn level(db: f64, len: usize) -> Vec<Sample> {
    vec![(10f64.powf(db / 20.0), 0.0); len]
}

fn changes(t: &AgcTrace) -> Vec<(usize, f64)> {
    t.changes.clone()
}

#[test]
fn a_strong_signal_is_stepped_down_and_locked() {
    // Input at −50 dB, gain 70: +20 dBFS, a peak of 10.
    // n = 0: 10 ≥ 2 (both) → −5: gain 65 from sample 1; samples 1, 2 not examined.
    // n = 3: +15 dB, 5.62 ≥ 2 → 60 from 4.   n = 6: +10 dB, 3.16 ≥ 2 → 55 from 7.
    // n = 9: +5 dB, 1.78 ≥ 1 (large) → 53 from 10.   n = 12: +3 dB, 1.41 → 51 from 13.
    // n = 15: +1 dB, 1.12 → 49 from 16.   n = 18: −1 dB, 0.89 ≥ 0.8 (small) → 47 from 19.
    // n = 21: −3 dB, 0.71: quiet. Four quiet samples after the change, 19 … 22: state 2
    // from 23. The settling delay of 10 counts from the change: 19 … 28. Power over 29 … 44:
    // −3 dBFS. Lock change round(−10 − (−3)) = −7: gain 40 from 45, locked from 45.
    let t = agc().run(&level(-50.0, 120));
    assert_eq!(
        changes(&t),
        [
            (0, 70.0),
            (1, 65.0),
            (4, 60.0),
            (7, 55.0),
            (10, 53.0),
            (13, 51.0),
            (16, 49.0),
            (19, 47.0),
            (45, 40.0)
        ]
    );
    assert_eq!((t.locks.clone(), t.unlocks.clone()), (vec![45], vec![]));
    assert_eq!(t.gain_db.len(), 120);
    assert_eq!(
        (t.gain_db[44], t.gain_db[45], t.gain_db[119]),
        (47.0, 40.0, 40.0)
    );
}

#[test]
fn a_weak_signal_locks_at_the_largest_gain() {
    // −90 dB at gain 70: −20 dBFS, no overload. Quiet 0 … 3, state 2 from 4, settling to
    // sample 9, power over 10 … 25: −20 dBFS; the lock change would be +10 but the gain is
    // at its maximum: no change, locked from 26.
    let t = agc().run(&level(-90.0, 100));
    assert_eq!(changes(&t), [(0, 70.0)]);
    assert_eq!(t.locks, [26]);
    // −75 dB: −5 dBFS, a peak of 0.56, quiet; the lock change is −5: 65 from 26.
    let t = agc().run(&level(-75.0, 100));
    assert_eq!(changes(&t), [(0, 70.0), (26, 65.0)]);
}

#[test]
fn a_packet_after_noise_unlocks_the_gain() {
    // 200 samples at −100 dB (−30 dBFS at 70: locked from 26, as above), then −50 dB.
    let mut x = level(-100.0, 200);
    x.extend(level(-50.0, 200));
    // With the overload exit: sample 200 has a peak of 10 ≥ 1: unlocked there, state 1
    // from 201 at the maximum gain, and the first case follows 201 samples later.
    let t = agc().run(&x);
    assert_eq!(
        (t.locks.clone(), t.unlocks.clone()),
        (vec![26, 246], vec![200])
    );
    assert_eq!(
        changes(&t)[1..],
        [
            (202, 65.0),
            (205, 60.0),
            (208, 55.0),
            (211, 53.0),
            (214, 51.0),
            (217, 49.0),
            (220, 47.0),
            (246, 40.0)
        ]
    );
    // Without it, by the power, which updates once per 64 samples from the lock at 26:
    // 26 … 89, 90 … 153, 154 … 217. The third holds 18 strong samples, a mean of 28, far
    // more than 10 dB above −30 dBFS: stronger from 217, stands eight samples, 217 … 224:
    // unlocked at 224, state 1 from 225, locked 45 samples later.
    let t = FastAttackAgc {
        unlock_on_overload: false,
        ..agc()
    }
    .run(&x);
    assert_eq!(
        (t.locks.clone(), t.unlocks.clone()),
        (vec![26, 270], vec![224])
    );
    assert_eq!(changes(&t)[1], (226, 65.0));
}

#[test]
fn the_end_of_a_packet_unlocks_the_gain() {
    // −50 dB for 150 samples (locked at 40 from 45, −10 dBFS, a power of 0.1), then nothing.
    // The power updates over 45 … 108, 109 … 172 and 173 … 236. The second holds 41 full
    // samples of 64: a mean of 0.064, 2 dB down, not lost. The third is empty: lost from
    // 236, stands eight samples, 236 … 243: unlocked at 243, the maximum gain from 244. Then
    // silence: power over 254 … 269 is nothing, no change, locked from 270 on nothing, and
    // nothing does not unlock it again.
    let mut x = level(-50.0, 150);
    x.extend(vec![(0.0, 0.0); 450]);
    let t = agc().run(&x);
    assert_eq!(
        (t.locks.clone(), t.unlocks.clone()),
        (vec![45, 270], vec![243])
    );
    assert_eq!(changes(&t).last(), Some(&(244, 70.0)));
    // Staying instead of returning to the maximum.
    let t = FastAttackAgc {
        unlock_to_max: false,
        ..agc()
    }
    .run(&x);
    assert_eq!(t.unlocks, [243]);
    // In silence the lock change is the largest increase the range allows.
    assert_eq!(changes(&t).last(), Some(&(270, 70.0)));
}

#[test]
fn an_overload_during_the_measurement_returns_to_peak_detect() {
    // −90 dB (quiet, measuring over 10 … 25), then −50 dB from sample 20: a peak of 10 at
    // sample 20 → −5, gain 65 from 21, and the first case from there (its sample 3 is 23).
    let mut x = level(-90.0, 20);
    x.extend(level(-50.0, 100));
    let t = agc().run(&x);
    assert_eq!(changes(&t)[..3], [(0, 70.0), (21, 65.0), (24, 60.0)]);
    assert_eq!(t.locks, [65]);
    assert_eq!(t.gain_db[119], 40.0);
}

#[test]
fn a_constant_envelope_locks_within_half_a_step() {
    // Any level the range can serve ends within 0.5 dB of the lock level, and the gain
    // never rises in state 1.
    for tenth in (-1050..=-250).step_by(7) {
        let db = f64::from(tenth) / 10.0;
        let t = agc().run(&level(db, 150));
        let last = *t.gain_db.last().unwrap();
        assert_eq!(t.locks.len(), 1, "{db}");
        let out = db + last;
        if db + 70.0 >= -10.5 {
            assert!(
                (out + 10.0).abs() <= 0.5 + 1e-9,
                "{db} dB ends at {out} dBFS"
            );
        } else {
            assert_eq!(last, 70.0);
        }
        let before_lock =
            &t.changes[..t.changes.len() - usize::from(last != t.gain_db[t.locks[0] - 1])];
        assert!(
            before_lock.windows(2).all(|w| w[1].1 < w[0].1),
            "{db}: {:?}",
            t.changes
        );
        assert!(t.gain_db.iter().all(|g| (0.0..=70.0).contains(g)));
    }
}

#[test]
fn the_gain_stops_at_its_lower_end() {
    // +30 dB at the input: even gain 0 overloads; the gain goes to 0 and stays, and no
    // lock is reached while every examined sample overloads.
    let t = agc().run(&level(30.0, 200));
    assert_eq!(*t.gain_db.last().unwrap(), 0.0);
    assert!(t.locks.is_empty());
}

#[test]
fn gain_states_describe_the_transient_relative_to_the_last_gain() {
    use radio_phy::channel::{agc_transient, Rng};
    let x = level(-50.0, 120);
    let t = agc().run(&x);
    let mut rng = Rng::new(7);
    let (settled, states) = t.gain_states(0.0, &mut rng);
    assert_eq!(settled, 40.0);
    // The states before the last change, gains relative to the settled one.
    let got: Vec<(usize, f64)> = states.iter().map(|s| (s.until, s.gain_db)).collect();
    assert_eq!(
        got,
        [
            (1, 30.0),
            (4, 25.0),
            (7, 20.0),
            (10, 15.0),
            (13, 13.0),
            (16, 11.0),
            (19, 9.0),
            (45, 7.0)
        ]
    );
    // Applied to the input at the settled gain it is the gain trace applied sample by sample.
    let g = 10f64.powf(settled / 20.0);
    let at_settled: Vec<Sample> = x.iter().map(|s| (s.0 * g, s.1 * g)).collect();
    let y = agc_transient(&at_settled, &states);
    for (n, v) in y.iter().enumerate() {
        let want = x[n].0 * 10f64.powf(t.gain_db[n] / 20.0);
        assert!((v.0 - want).abs() < 1e-9 * want, "sample {n}");
    }
    // With jumps: every state's phase differs from the next by at most the range, and the
    // last state before the settled chain is within it of zero.
    let (_, states) = t.gain_states(30.0, &mut rng);
    let lim = 30f64.to_radians() + 1e-12;
    assert!(states.last().unwrap().phase.abs() <= lim);
    assert!(states
        .windows(2)
        .all(|w| (w[0].phase - w[1].phase).abs() <= lim));
    assert!(states.iter().any(|s| s.phase != 0.0));
}

#[test]
fn the_locked_power_waits_for_a_whole_window() {
    // Locked from 26 on −100 dB; −50 dB from sample 30, the overload exit off. The power
    // first updates when 64 samples have passed since the lock, at sample 89 (26 … 89); it
    // is far above: the comparison stands eight samples, 89 … 96, unlocked at 96.
    let mut x = level(-100.0, 30);
    x.extend(level(-50.0, 100));
    let t = FastAttackAgc {
        unlock_on_overload: false,
        ..agc()
    }
    .run(&x);
    assert_eq!((t.locks.clone(), t.unlocks.clone()), (vec![26], vec![96]));
}

#[test]
fn the_peak_is_taken_on_both_rails() {
    // The first case with the signal on the Q rail, and negative.
    let x: Vec<Sample> = level(-50.0, 120).iter().map(|s| (0.0, -s.0)).collect();
    assert_eq!(agc().run(&x), agc().run(&level(-50.0, 120)));
}

// ---------------------------------------------------------------------------------------------
// ADR 0007's two requirements on a gain control, on 802.11 packets
// ---------------------------------------------------------------------------------------------

/// A short 6 Mb/s PPDU as float samples.
fn packet() -> Vec<Sample> {
    use radio_phy::ofdm::IfftConfig;
    use radio_phy::ppdu::{build_ppdu, Rate};
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

#[test]
fn packets_are_still_by_the_end_of_the_stf_and_inside_the_level_window() {
    use radio_phy::channel::{agc_transient, awgn, frequency_offset, mean_power, Rng};
    use radio_phy::rx_front::{backoff_samples, RxConfig};
    use radio_phy::rx_front_fixed::{acquire_fixed, to_adc, FixedRxConfig, ADC_FULL_SCALE};
    const FS: f64 = 20e6;
    // The noise at −30 dBFS at the largest gain, where the control idles.
    const IDLE_DBFS: f64 = -30.0;
    let frame = packet();
    let p = mean_power(&frame);
    // The reset values with the power measured over 16 samples, the shortest the chip
    // allows (the reset value, 512, outlasts the preamble: see the last test), and the lock
    // left on a large overload (by the power alone a strong packet clips for up to 72
    // samples before the gain moves, and frames are lost: `rxfront agcmodel`).
    let agc = FastAttackAgc {
        power_samples: 16,
        unlock_on_overload: true,
        ..FastAttackAgc::ad9361_reset(2)
    };
    let (cfg, fx) = (RxConfig::new(FS), FixedRxConfig::default());
    let mut rng = Rng::new(0xa6c2);
    for snr in [5.0, 12.0, 20.0, 30.0, 45.0, 60.0] {
        let k = (10f64.powf((IDLE_DBFS - agc.max_gain_db + snr) / 10.0) / p).sqrt();
        for _ in 0..40 {
            let lead = 600 + (rng.uniform() * 50.0) as usize;
            let mut x = vec![(0.0, 0.0); lead];
            x.extend_from_slice(&frame);
            let cfo = (2.0 * rng.uniform() - 1.0) * 200e3;
            let x = frequency_offset(&x, cfo, FS, 0.3);
            let x: Vec<Sample> = awgn(&x, snr, p, &mut rng)
                .iter()
                .map(|s| (s.0 * k, s.1 * k))
                .collect();
            let t = agc.run(&x);
            // The noise before the packet moves nothing once the control has locked on it.
            assert!(
                t.changes.iter().all(|c| c.0 < 100 || c.0 >= lead),
                "{snr} dB: {:?}",
                t.changes
            );
            // Requirement 1: still from the end of the short training field.
            let last = t.changes.last().unwrap().0;
            assert!(
                last <= lead + 159,
                "{snr} dB: last change at {}",
                last as i64 - lead as i64
            );
            // Requirement 2: the packet inside −15 … −3 dBFS, or the gain at its maximum
            // (a packet less than 15 dB above the noise cannot be brought higher).
            let (settled, states) = t.gain_states(30.0, &mut rng);
            let level = 10.0 * (p * k * k).log10() + settled;
            assert!(
                (-15.0..=-3.0).contains(&level) || (settled == agc.max_gain_db && level < -15.0),
                "{snr} dB: {level} dBFS at gain {settled}"
            );
            // And the fixed receiver finds the packet where it is, through the phase jumps.
            let g = 10f64.powf(settled / 20.0);
            let at_settled: Vec<Sample> = x.iter().map(|s| (s.0 * g, s.1 * g)).collect();
            let adc = to_adc(
                &agc_transient(&at_settled, &states),
                f64::from(ADC_FULL_SCALE),
            );
            let a = acquire_fixed(&adc, &cfg, &fx).unwrap();
            assert_eq!(a.fft_start, lead + 192 - backoff_samples(&cfg), "{snr} dB");
        }
    }
}

#[test]
fn details_the_first_cases_do_not_reach() {
    // A settling delay shorter than the energy-detect count: the measurement still waits
    // for the count. Quiet 0 … 3, power over 4 … 19, locked from 20.
    let short = FastAttackAgc {
        settling_delay: 2,
        ..agc()
    };
    assert_eq!(short.run(&level(-90.0, 60)).locks, [20]);
    // A decision at the last sample has no sample to act on: one sample of the strong
    // signal decides a step that is not recorded, and 45 samples do not reach the lock.
    assert_eq!(changes(&agc().run(&level(-50.0, 1))), [(0, 70.0)]);
    let t = agc().run(&level(-50.0, 45));
    assert!(t.locks.is_empty());
    assert_eq!(changes(&t).last(), Some(&(19, 47.0)));
    assert_eq!(agc().run(&[]).gain_db.len(), 0);
    // The two thresholds are separate: locked at −10 dBFS (gain 40 from 45), the signal
    // drops by 5 dB at sample 60. With 3 dB to lose it unlocks (the update at 108 holds 49
    // of 64 samples 5 dB down: 3.4 dB; stands 108 … 115), with 10 dB it does not.
    let mut x = level(-50.0, 60);
    x.extend(level(-55.0, 200));
    let quick = FastAttackAgc {
        lost_db: 3.0,
        ..agc()
    };
    assert_eq!(quick.run(&x).unlocks, [115]);
    assert!(agc().run(&x).unlocks.is_empty());
    // The overload exit is the large overload's: a rise of 9 dB after the lock (a peak of
    // 0.89, above the small threshold and below the large; less than 10 dB) unlocks nothing.
    let mut x = level(-50.0, 60);
    x.extend(level(-41.0, 200));
    assert!(agc().run(&x).unlocks.is_empty());
    // A comparison must stand: with 100 samples to stand and 64 between updates, one
    // strong window (154 … 217, as above) followed by quiet ones does not unlock.
    let mut x = level(-100.0, 200);
    x.extend(level(-50.0, 18));
    x.extend(level(-100.0, 400));
    let slow = FastAttackAgc {
        unlock_on_overload: false,
        exit_count: 100,
        ..agc()
    };
    assert!(slow.run(&x).unlocks.is_empty());
    // Nor do two strong windows with a quiet one between them add up: bursts in the
    // windows 154 … 217 and 282 … 345 each stand 64 samples, never 100.
    let mut x = level(-100.0, 200);
    for _ in 0..2 {
        x.extend(level(-50.0, 18));
        x.extend(level(-100.0, 82));
    }
    x.extend(level(-100.0, 100));
    assert!(slow.run(&x).unlocks.is_empty());
    // Parameters given the wrong way round and samples that are not numbers do not panic.
    let odd = FastAttackAgc {
        min_gain_db: 80.0,
        ..agc()
    };
    let _ = odd.run(&level(-90.0, 100));
    let mut x = level(-50.0, 120);
    // (Sample 35 is inside the power measurement, 29 … 44: 15 samples of 16 measure
    // 0.3 dB less, and the lock change is the same.)
    x[35] = (f64::NAN, f64::NAN);
    let t = agc().run(&x);
    assert!(t.gain_db.iter().all(|g| g.is_finite()));
    assert_eq!((t.locks.clone(), t.gain_db[119]), (vec![45], 40.0));
}

#[test]
fn the_reset_value_of_the_power_measurement_outlasts_the_preamble() {
    // 512 samples of power from the last change: no lock inside 320 samples of preamble,
    // whatever the level. This is why a receiver of 802.11 shortens it.
    let reset = FastAttackAgc::ad9361_reset(2);
    assert_eq!(reset.power_samples, 512);
    for db in [-95.0, -70.0, -40.0] {
        let t = reset.run(&level(db, 600));
        assert!(t.locks[0] >= 522, "{db}: {:?}", t.locks);
    }
}
