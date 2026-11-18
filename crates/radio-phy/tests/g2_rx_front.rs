//! G2 gate for the receiver front (R0-08b part 2): the fixed-point model `RxFrontFixed`
//! against the float reference `RxFront` on the same ADC samples.
//!
//! Budget (`docs/gates/rx_front.md`, G0 entries): Pd within Monte-Carlo noise of the float
//! model; no false alarm in 6·10⁷ noise samples at several input levels; CFO RMS at most +10 %
//! of the float model's; FFT start identical in ≥ 99.9 % of the frames both acquire.
//!
//! The arithmetic primitives are checked against references we did not write: the CORDIC
//! against `f64::atan2`, the NCO table against `f64::sin_cos`, each within a bound derived from its construction (loose, not tight).

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, tgn_taps, Doppler,
    Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, detections, RxConfig};
use radio_phy::rx_front_fixed::{
    acquire_fixed, acquisitions_fixed, cordic_angle, dc_block_fixed, detections_fixed, nco_rotator,
    to_adc, FixedRxConfig, ADC_FULL_SCALE,
};
use std::f64::consts::PI;

type C = (f64, f64);

/// Start of T1 within a PPDU.
const T1: usize = 192;
/// Packet RMS after the AGC: −15 dBFS.
const PACKET_DBFS: f64 = -15.0;

fn preamble() -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

/// A scene as in the G1 gate, scaled so the packet sits at `PACKET_DBFS`, then through the
/// 12-bit ADC. Returns the ADC samples and the T1 position.
fn adc_scene(
    f: &[C],
    channel: Option<&Multipath>,
    snr_db: f64,
    fs: f64,
    rng: &mut Rng,
) -> (Vec<(i16, i16)>, usize, f64) {
    let lead = 200 + (rng.uniform() * 300.0) as usize;
    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
    let mut x = vec![(0.0, 0.0); lead];
    x.extend_from_slice(f);
    x.extend(vec![(0.0, 0.0); 400]);
    let x = match channel {
        Some(ch) => ch.apply(&x),
        None => fractional_delay(&x, rng.uniform()),
    };
    let x = frequency_offset(&x, cfo, fs, 2.0 * PI * rng.uniform());
    let p = mean_power(f);
    let x = awgn(&x, snr_db, p, rng);
    // Packet power after noise: p·(1 + 10^(−SNR/10)).
    let rms = (p * (1.0 + 10f64.powf(-snr_db / 10.0))).sqrt();
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(PACKET_DBFS / 20.0) / rms;
    (to_adc(&x, gain), lead + T1, cfo)
}

fn as_f64(x: &[(i16, i16)]) -> Vec<C> {
    x.iter()
        .map(|&(a, b)| (f64::from(a), f64::from(b)))
        .collect()
}

#[test]
fn cordic_angle_matches_atan2_within_its_bound() {
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x08c1);
    let unit = 2.0 * PI / f64::from(1u32 << fx.angle_bits);
    // Residual after N micro-rotations ≤ atan(2^−(N−1)); each table entry within ½ LSB; the
    // larger component is ≥ 2^(W_c−4) after block normalisation, and the normalising shift plus
    // the N floored shifts each move the vector by < √2 LSB, i.e. ≤ (N + 1)·√2·2^−(W_c−4) rad;
    // one LSB for the final wrap.
    let bound = (2f64.powi(-(fx.cordic_iters as i32 - 1))).atan()
        + f64::from(fx.cordic_iters) * 0.5 * unit
        + f64::from(fx.cordic_iters + 1) * 2f64.sqrt() * 2f64.powi(-(fx.cordic_bits as i32 - 4))
        + unit;
    let mut worst: f64 = 0.0;
    for k in 0..20_000 {
        // Magnitudes from 1 to 2^40, every quadrant, the axes included.
        let mag = 2f64.powf(rng.uniform() * 40.0);
        let a = if k < 8 {
            f64::from(k) * PI / 4.0
        } else {
            2.0 * PI * rng.uniform()
        };
        let (re, im) = (
            (mag * a.cos()).round() as i64,
            (mag * a.sin()).round() as i64,
        );
        if re == 0 && im == 0 {
            continue;
        }
        let got = cordic_angle(re, im, &fx) as f64 * unit;
        let want = (im as f64).atan2(re as f64);
        let mut err = got - want;
        err -= 2.0 * PI * (err / (2.0 * PI)).round();
        worst = worst.max(err.abs());
    }
    println!("CORDIC worst error {worst:.2e} rad, bound {bound:.2e}");
    assert!(worst <= bound, "{worst} > {bound}");
}

#[test]
fn nco_rotator_matches_sin_cos_within_its_bound() {
    let fx = FixedRxConfig::default();
    let amp = f64::from((1u32 << (fx.nco_amp_bits - 1)) - 1);
    // Phase truncated to L address bits (< 2π/2^L of phase error), each component rounded
    // (½ LSB, so √2/2 LSB as a vector).
    let bound = 2.0 * PI / f64::from(1u32 << fx.nco_lut_bits) + 0.5 * 2f64.sqrt() / amp;
    let mut worst: f64 = 0.0;
    for k in 0..(1u64 << 16) {
        let phase = (k << (fx.nco_phase_bits - 16)) as i64;
        let (c, s) = nco_rotator(phase, &fx);
        let a = 2.0 * PI * phase as f64 / 2f64.powi(fx.nco_phase_bits as i32);
        let e = ((c as f64 / amp - a.cos()).powi(2) + (s as f64 / amp - a.sin()).powi(2)).sqrt();
        worst = worst.max(e);
    }
    println!("NCO worst error {worst:.2e}, bound {bound:.2e}");
    assert!(worst <= bound, "{worst} > {bound}");
}

#[test]
fn fixed_point_meets_the_g2_budget_on_frames() {
    let fs = 20e6;
    let cfg = RxConfig::new(fs);
    let fx = FixedRxConfig::default();
    let f = preamble();
    let mut rng = Rng::new(0x08c3);
    let d_taps = tgn_taps(TgnModel::D, false);
    let (mut all_both, mut all_same) = (0usize, 0usize);
    for (name, snr, trials) in [
        ("AWGN 4 dB", 4.0, 400),
        ("AWGN 10 dB", 10.0, 400),
        ("AWGN 25 dB", 25.0, 400),
        ("TGn D 9 dB", 9.0, 300),
    ] {
        let (mut det_f, mut det_x, mut det_disagree) = (0usize, 0usize, 0usize);
        let (mut both, mut same) = (0usize, 0usize);
        let (mut only_f, mut only_x) = (0usize, 0usize);
        let (mut e_f, mut e_x) = (Vec::new(), Vec::new());
        for _ in 0..trials {
            let ch = (snr == 9.0)
                .then(|| Multipath::new(d_taps.clone(), Doppler::Static, 1, fs, &mut rng));
            let (adc, t1, cfo) = adc_scene(&f, ch.as_ref(), snr, fs, &mut rng);
            let stf = t1 - T1;
            let xf = as_f64(&adc);
            // Pd: detection within 80 samples of the STF start.
            let df = detections(&xf, &cfg)
                .iter()
                .any(|&d| d >= stf && d < stf + 80);
            let dx = detections_fixed(&adc, &cfg, &fx)
                .iter()
                .any(|&d| d >= stf && d < stf + 80);
            det_f += usize::from(df);
            det_x += usize::from(dx);
            det_disagree += usize::from(df != dx);
            let a = acquire(&xf, 0, &cfg);
            let b = acquire_fixed(&adc, &cfg, &fx);
            match (a, b) {
                (Some(a), Some(b)) => {
                    both += 1;
                    if a.fft_start == b.fft_start {
                        same += 1;
                    }
                    e_f.push(a.cfo_hz - cfo);
                    e_x.push(b.cfo_hz(&fx, fs) - cfo);
                }
                (Some(_), None) => only_f += 1,
                (None, Some(_)) => only_x += 1,
                (None, None) => {}
            }
        }
        let rms = |v: &[f64]| (v.iter().map(|e| e * e).sum::<f64>() / v.len() as f64).sqrt();
        let (rf, rx) = (rms(&e_f), rms(&e_x));
        println!(
            "{name}: Pd float {det_f} fixed {det_x} / {trials} (disagree {det_disagree}); \
             acquired both {both}, \
             float only {only_f}, fixed only {only_x}; same FFT start {same}/{both}; \
             CFO rms float {rf:.0} fixed {rx:.0} Hz"
        );
        // Pd and acquisition within Monte-Carlo noise: paired frames, so the frames on which the
        // two models disagree (counted both ways, not netted) are bounded: ≤ 1 % of the frames
        // plus 3σ of that binomial.
        let tol = trials as f64 * 0.01 + 3.0 * (trials as f64 * 0.01).sqrt();
        assert!(
            det_disagree as f64 <= tol,
            "{name}: detection disagrees in {det_disagree}"
        );
        assert!(
            (only_f + only_x) as f64 <= tol,
            "{name}: acquisition disagrees in {}",
            only_f + only_x
        );
        all_both += both;
        all_same += same;
        assert!(rx <= 1.10 * rf, "{name}: CFO rms {rx} vs {rf}");
    }
    // Timing identical in ≥ 99.9 % of the frames both acquired, over all cases.
    assert!(
        (all_both - all_same) * 1000 <= all_both,
        "timing identical in {all_same}/{all_both}"
    );
}

#[test]
fn fixed_point_detector_has_no_false_alarm_on_noise() {
    // 6·10⁷ noise samples (≤ 1 false alarm/s at 95 % confidence) spread over idle noise levels
    // from −15 dBFS down to −65 dBFS (≈ 1 LSB rms). Narrowing the detector's input to D < 13
    // bits makes the noise sparse at some level (D = 8 at −45, D = 10 at −55, D = 12 at −65
    // dBFS) and the normalised correlation of sparse sequences has a heavy tail; below ≈ 1 LSB
    // the ADC itself does it, for the float model too (an AGC floor requirement, journal).
    let cfg = RxConfig::new(20e6);
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x08c4);
    let mut total = 0;
    for (level_dbfs, blocks) in [
        (-15.0, 12),
        (-30.0, 12),
        (-45.0, 12),
        (-55.0, 12),
        (-65.0, 12),
    ] {
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(level_dbfs / 20.0);
        for _ in 0..blocks {
            let x = awgn(&vec![(0.0, 0.0); 1_000_000], 0.0, 1.0, &mut rng);
            total += detections_fixed(&to_adc(&x, gain), &cfg, &fx).len();
        }
    }
    assert_eq!(total, 0, "false alarms on 6e7 noise samples");
}

#[test]
fn fixed_point_keeps_the_robustness_of_the_float_model() {
    // The impairment cases of the G1 gate, through the ADC: no false acquisition under DC
    // 10 dB over the noise, a CW tone 3 dB over the noise, ±30 dB AGC steps with DC (the
    // `rxfront robust` seeds); and the 4 dB target under a spur at the noise level.
    let cfg = RxConfig::new(20e6);
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x08c5);
    let at = |dbfs: f64| f64::from(ADC_FULL_SCALE) * 10f64.powf(dbfs / 20.0);
    let noise = |rng: &mut Rng| awgn(&vec![(0.0, 0.0); 1_000_000], 0.0, 1.0, rng);
    let x = add_tone(&noise(&mut rng), 10.0, 0.0, 20e6, 0.4);
    assert!(
        acquisitions_fixed(&to_adc(&x, at(-30.0)), &cfg, &fx).is_empty(),
        "DC"
    );
    let x = add_tone(&noise(&mut rng), 2.0, 1.3e6, 20e6, 0.0);
    assert!(
        acquisitions_fixed(&to_adc(&x, at(-30.0)), &cfg, &fx).is_empty(),
        "CW"
    );
    for t in 0..10u64 {
        let mut r = Rng::new(0xb0b0_0000 + 27 * 1000 + t);
        let x = noise(&mut r);
        let mut y = add_tone(&x, 1.0, 0.0, 20e6, 0.3);
        for (i, k) in (100_000..x.len()).step_by(100_000).enumerate() {
            y = gain_step(&y, k, if i % 2 == 0 { 30.0 } else { -30.0 });
        }
        // Noise at −50 dBFS, so the +30 dB segments sit at −20 dBFS (no clipping).
        let adc = to_adc(&y, at(-50.0));
        assert!(
            acquisitions_fixed(&adc, &cfg, &fx).is_empty(),
            "±30 dB steps, seed {t}"
        );
    }
    let f = preamble();
    let (mut placed, mut placed_float) = (0, 0);
    for _ in 0..400 {
        let lead = 300 + (rng.uniform() * 200.0) as usize;
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let mut x = vec![(0.0, 0.0); lead];
        x.extend_from_slice(&f);
        let x = fractional_delay(&x, rng.uniform());
        let x = frequency_offset(&x, cfo, 20e6, 2.0 * PI * rng.uniform());
        let p = mean_power(&f);
        let x = awgn(&x, 4.0, p, &mut rng);
        let x = add_tone(
            &x,
            p / 10f64.powf(0.4),
            1.3e6,
            20e6,
            2.0 * PI * rng.uniform(),
        );
        // Packet + noise + spur ≈ p·(1 + 2·10^−0.4) at −15 dBFS.
        let gain = at(-15.0) / (p * (1.0 + 2.0 * 10f64.powf(-0.4))).sqrt();
        let adc = to_adc(&x, gain);
        if let Some(a) = acquire_fixed(&adc, &cfg, &fx) {
            if a.fft_start.abs_diff(lead + T1 - 4) <= 1 {
                placed += 1;
            }
        }
        if let Some(a) = acquire(&as_f64(&adc), 0, &cfg) {
            if a.fft_start.abs_diff(lead + T1 - 4) <= 1 {
                placed_float += 1;
            }
        }
    }
    println!("0 dB spur, 4 dB SNR: fixed {placed}/400, float {placed_float}/400 placed");
    assert!(placed >= 360, "{placed}/400");
}

#[test]
fn fixed_dc_blocker_starts_up_like_the_float_one() {
    // A stream opening on a DC offset 10 dB over the noise (noise at −40 dBFS): by samples
    // 128..384 the fixed blocker's residual DC must be at the noise level (the mean of 256
    // samples of unit-power noise has power 1/256; without the gear shift ≈ 0.25 remains), as
    // the float gate requires of the float blocker.
    let cfg = RxConfig::new(20e6);
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x08c6);
    let sigma = f64::from(ADC_FULL_SCALE) * 10f64.powf(-40.0 / 20.0);
    for _ in 0..50 {
        let x = awgn(&vec![(0.0, 0.0); 3000], 0.0, 1.0, &mut rng);
        let x = add_tone(&x, 10.0, 0.0, 20e6, 2.0 * PI * rng.uniform());
        let adc = to_adc(&x, sigma);
        let y = dc_block_fixed(&adc, &cfg, &fx);
        let m = y[128..384]
            .iter()
            .fold((0.0, 0.0), |a, s| (a.0 + s.0 as f64, a.1 + s.1 as f64));
        let p = (m.0 * m.0 + m.1 * m.1) / (256.0 * 256.0) / (sigma * sigma);
        assert!(p < 0.05, "residual DC power {p} (noise = 1)");
        assert!(
            acquisitions_fixed(&adc, &cfg, &fx).is_empty(),
            "start-up acquired"
        );
    }
}

#[test]
fn fixed_detector_follows_the_float_one_at_onsets_and_high_snr() {
    // A power step inside the 64-sample detector window makes P₁ and P₂ differ by up to the
    // step; the threshold comparison must stay exact there (pre-PR review of R0-08b: a common
    // normalising shift zeroed the smaller sum and fired on every onset).
    let cfg = RxConfig::new(20e6);
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x08c7);
    let at = |dbfs: f64| f64::from(ADC_FULL_SCALE) * 10f64.powf(dbfs / 20.0);
    // Noise bursts −65 → −15 dBFS, 200 onsets.
    let mut adc = Vec::new();
    let mut onsets = Vec::new();
    for _ in 0..200 {
        adc.extend(to_adc(
            &awgn(&vec![(0.0, 0.0); 1000], 0.0, 1.0, &mut rng),
            at(-65.0),
        ));
        onsets.push(adc.len());
        adc.extend(to_adc(
            &awgn(&vec![(0.0, 0.0); 1000], 0.0, 1.0, &mut rng),
            at(-15.0),
        ));
    }
    let near = |ev: &[usize]| {
        ev.iter()
            .filter(|&&d| onsets.iter().any(|&o| d >= o && d < o + 64))
            .count()
    };
    let (fixed, float) = (
        near(&detections_fixed(&adc, &cfg, &fx)),
        near(&detections(&as_f64(&adc), &cfg)),
    );
    println!("events within 64 samples of 200 onsets: fixed {fixed}, float {float}");
    assert!(fixed <= float, "fixed {fixed} vs float {float}");
    // Clean packets at 40 and 50 dB: the detection (which opens the search) must be the float
    // model's in nearly every frame. (With θ² = 85/256, θ_eff 0.5762, it came one sample late
    // in 15 % of the frames at 40 dB — the steep plateau ramp; 21668/2^16 gives θ_eff 0.575002.)
    let f = preamble();
    for snr in [40.0, 50.0] {
        let mut same = 0;
        for _ in 0..200 {
            let (adc, _, _) = adc_scene(&f, None, snr, 20e6, &mut rng);
            let a = acquire(&as_f64(&adc), 0, &cfg).unwrap();
            let b = acquire_fixed(&adc, &cfg, &fx).unwrap();
            assert_eq!(a.fft_start, b.fft_start, "{snr} dB timing");
            if a.detected_at == b.detected_at {
                same += 1;
            }
        }
        println!("{snr} dB: same detection in {same}/200");
        assert!(same >= 194, "{snr} dB: same detection in {same}/200");
    }
}

#[test]
fn fixed_decision_latency_only_delays_the_result() {
    // Open question 13 in the fixed model: up to 63 + (GI − B) samples of decision latency the
    // acquisitions are those of latency 0, bit for bit, each decided exactly L samples later.
    // Clean packets, a CW spur at the noise level, an AGC step and two packets back to back.
    let mut rng = Rng::new(0x0d14);
    let f = preamble();
    let fx = FixedRxConfig::default();
    let base = RxConfig {
        decision_latency: 0,
        ..RxConfig::new(20e6)
    };
    let mut acquired = 0;
    for k in 0..80 {
        let mut g = f.clone();
        if k % 4 == 3 {
            g.extend(vec![(0.0, 0.0); (rng.uniform() * 300.0) as usize]);
            g.extend_from_slice(&f);
        }
        let (adc, t1, _) = adc_scene(
            &g,
            None,
            if k % 2 == 0 { 4.0 } else { 10.0 },
            20e6,
            &mut rng,
        );
        let x = as_f64(&adc);
        let x = match k % 4 {
            1 => add_tone(&x, mean_power(&x[..t1 - 200]), 1.3e6, 20e6, 0.3),
            2 => gain_step(&x, t1 - 300, -20.0),
            _ => x,
        };
        let adc = to_adc(&x, 1.0);
        let a0 = acquisitions_fixed(&adc, &base, &fx);
        acquired += a0.len();
        for l in [1, 64, 75] {
            let cfg = RxConfig {
                decision_latency: l,
                ..base.clone()
            };
            let got = acquisitions_fixed(&adc, &cfg, &fx);
            assert_eq!(got.len(), a0.len(), "L = {l}");
            for (g, a) in got.iter().zip(&a0) {
                assert_eq!(g.decided_at, a.decided_at + l, "L = {l}");
                assert_eq!(
                    (
                        g.detected_at,
                        g.ltf_start,
                        g.fft_start,
                        g.cfo_phase,
                        g.coarse_phase,
                        g.fine_phase
                    ),
                    (
                        a.detected_at,
                        a.ltf_start,
                        a.fft_start,
                        a.cfo_phase,
                        a.coarse_phase,
                        a.fine_phase
                    ),
                    "L = {l}"
                );
            }
        }
    }
    assert!(acquired >= 80, "{acquired} acquisitions in 80 scenes");
}

#[test]
fn fixed_failed_candidate_does_not_block_a_weaker_packet_after_it() {
    // Open question 15 in the fixed model: a strong STF fragment and noise burst make a
    // candidate that fails confirmation; the real packet, 10 dB weaker, follows and is acquired.
    let mut rng = Rng::new(0x0d16);
    let f = preamble();
    let stf: Vec<C> = f[..160]
        .iter()
        .map(|&(a, b)| (a * 3.16, b * 3.16))
        .collect();
    let cfg = RxConfig::new(20e6);
    let fx = FixedRxConfig::default();
    let p = mean_power(&f);
    let mut ok = 0;
    for _ in 0..40 {
        let lead = 200 + (rng.uniform() * 200.0) as usize;
        let mut x = vec![(0.0, 0.0); lead];
        x.extend_from_slice(&stf);
        x.extend(awgn(&vec![(0.0, 0.0); 120], 0.0, 10.0 * p, &mut rng));
        x.extend(vec![(0.0, 0.0); 20 + (rng.uniform() * 40.0) as usize]);
        let start = x.len();
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); 400]);
        let x = frequency_offset(&x, 50e3, 20e6, 2.0 * PI * rng.uniform());
        let x = awgn(&x, 15.0, p, &mut rng);
        // the packet at −25 dBFS, so the stronger burst stays inside the ADC range
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-25.0 / 20.0) / p.sqrt();
        let acq = acquisitions_fixed(&to_adc(&x, gain), &cfg, &fx);
        if acq
            .iter()
            .any(|a| a.fft_start.abs_diff(start + T1 - 4) <= 1)
        {
            ok += 1;
        }
    }
    assert!(
        ok >= 38,
        "{ok}/40 packets acquired after a failed candidate"
    );
}
