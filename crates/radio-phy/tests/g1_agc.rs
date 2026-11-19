//! G1 gate of the fast-attack gain control model (R0-07e, `radio_phy::channel::FastAttackAgc`).
//!
//! Reference we did not write: the published behaviour of the AD9361's fast-attack mode
//! (Analog Devices UG-570, "Fast Attack AGC Mode", Fig. 25 and Table 20), as tabulated in
//! `docs/gates/channel.md` (G0, R0-07e). The cases below are worked by hand from that table
//! in the comments; the model must give every gain change at its sample.

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
    // Without it, by the power: the mean of the last 64 samples is 10 dB above −30 dBFS
    // from the first strong sample on ((100 + 63·0.001)/64 = 1.56 ≥ 0.01); eight samples in
    // a row, 200 … 207: unlocked at 207, state 1 from 208, locked 45 samples later.
    let t = FastAttackAgc {
        unlock_on_overload: false,
        ..agc()
    }
    .run(&x);
    assert_eq!(
        (t.locks.clone(), t.unlocks.clone()),
        (vec![26, 253], vec![207])
    );
    assert_eq!(changes(&t)[1], (209, 65.0));
}

#[test]
fn the_end_of_a_packet_unlocks_the_gain() {
    // −50 dB for 150 samples (locked at 40 from 45, −10 dBFS, a power of 0.1), then nothing.
    // The mean of the last 64 samples is 0.1·(64 − k)/64 after k empty samples; 10 dB down
    // is 0.01: k = 58 (0.0094; k = 57 gives 0.0109), sample 207. Eight in a row, 207 … 214:
    // unlocked at 214, the maximum gain from 215. Then silence: quiet 215 … 218, power over
    // 225 … 240 is nothing, no change, locked from 241.
    let mut x = level(-50.0, 150);
    x.extend(vec![(0.0, 0.0); 150]);
    let t = agc().run(&x);
    assert_eq!(
        (t.locks.clone(), t.unlocks.clone()),
        (vec![45, 241], vec![214])
    );
    assert_eq!(changes(&t).last(), Some(&(215, 70.0)));
    // Staying instead of returning to the maximum.
    let t = FastAttackAgc {
        unlock_to_max: false,
        ..agc()
    }
    .run(&x);
    assert_eq!(t.unlocks, [214]);
    // In silence the lock change is the largest increase the range allows.
    assert_eq!(changes(&t).last(), Some(&(241, 70.0)));
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
