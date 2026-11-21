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
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, resample, tgn_taps,
    Doppler, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, detections, ltf_dc_offset, RxConfig};
use radio_phy::rx_front_fixed::{
    acquire_fixed, acquisitions_fixed, cordic_angle, correct_fixed, dc_block_fixed,
    detections_fixed, handoff_fixed, ltf_dc_offset_fixed, nco_mix, nco_rotator, remove_dc_fixed,
    to_adc, FixedRxConfig, ADC_FULL_SCALE, IQ, Y_BITS,
};
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{clock_ratio, decode_ppdu_fixed, FixedOfdmConfig, FixedTracking};
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

/// ADR 0007, decision 1, in the fixed model: the CFO handed on is the fine estimate, bit for
/// bit; the coarse weight (in 64ths) stays a parameter and at 15 gives the old 49/64 rule.
#[test]
fn fixed_acquisition_hands_on_the_fine_estimate() {
    let fs = 20e6;
    let cfg = RxConfig::new(fs);
    let fx = FixedRxConfig::default();
    assert_eq!(fx.coarse_weight_64, 0);
    let old = FixedRxConfig {
        coarse_weight_64: 15,
        ..fx
    };
    for w in [-1, 65] {
        let bad = FixedRxConfig {
            coarse_weight_64: w,
            ..fx
        };
        assert!(!bad.is_valid(), "a weight of {w}/64");
    }
    assert!(old.is_valid());
    let f = preamble();
    let mut rng = Rng::new(0x08d2);
    let mut differ = 0;
    for _ in 0..200 {
        let (adc, _, _) = adc_scene(&f, None, 12.0, fs, &mut rng);
        let a = acquire_fixed(&adc, &cfg, &fx).unwrap();
        assert_eq!(a.cfo_phase, a.fine_phase);
        let b = acquire_fixed(&adc, &cfg, &old).unwrap();
        assert_eq!(
            (b.coarse_phase, b.fine_phase),
            (a.coarse_phase, a.fine_phase)
        );
        let d = b.fine_phase - b.coarse_phase;
        assert_eq!(b.cfo_phase, b.coarse_phase + ((49 * d + 32) >> 6));
        differ += usize::from(d != 0);
    }
    assert!(differ > 150, "coarse and fine hardly ever differ: {differ}");
}

// ---------------------------------------------------------------------------------------------
// R0-09 G2, stage B, second part: the CFO taken out of the decoder's samples in integers
// (`correct_fixed`, G0 in `docs/gates/rx_ofdm.md`).
// ---------------------------------------------------------------------------------------------

/// The definition on samples worked by hand: amplitude 1023 (Q = 11), a 13-bit sample times
/// it narrowed from 23 bits to O, half to even; a CFO word of 2^20 is a quarter turn per
/// sample at P = 22 and is taken out, so sample n is turned by −n quarter turns.
#[test]
fn correct_fixed_worked_by_hand() {
    let fx = FixedRxConfig::default();
    assert_eq!(
        (fx.nco_phase_bits, fx.nco_lut_bits, fx.nco_amp_bits),
        (22, 9, 11)
    );
    let y: Vec<IQ> = vec![(1000, -500); 4];
    // 1000·1023/2048 = 499.51, 500·1023/2048 = 249.76.
    // ·1, ·(−j), ·(−1), ·(+j) of 1000 − 500j: (1000, −500), (−500, −1000), (−1000, 500),
    // (500, 1000).
    assert_eq!(
        correct_fixed(&y, 1 << 20, 12, &fx),
        vec![(500, -250), (-250, -500), (-500, 250), (250, 500)]
    );
    // The other way round for a negative CFO.
    assert_eq!(
        correct_fixed(&y, -(1 << 20), 12, &fx),
        vec![(500, -250), (250, 500), (-500, 250), (-250, -500)]
    );
    // One bit more keeps the ADC's last bit: 1000·1023/1024 = 999.02, 500·1023/1024 = 499.51.
    assert_eq!(
        correct_fixed(&y, 1 << 20, 13, &fx),
        vec![(999, -500), (-500, -999), (-999, 500), (500, 999)]
    );
    // No CFO: every sample by the same real amplitude. Ties go to the even value:
    // 1024·1023/2048 = 511.5 → 512, 3072·1023/2048 = 1534.5 → 1534, and the negatives.
    assert_eq!(
        correct_fixed(&[(1024, 3072), (-1024, -3072)], 0, 12, &fx),
        vec![(512, 1534), (-512, -1534)]
    );
    // The same ties one bit up: 512·1023/1024 = 511.5 → 512, 1536·1023/1024 = 1534.5 → 1534.
    assert_eq!(
        correct_fixed(&[(512, 1536), (-512, -1536)], 0, 13, &fx),
        vec![(512, 1534), (-512, -1534)]
    );
    // An eighth of a turn (the table's entry 64 of 512: round(1023·cos 45°) = 723):
    // (4095 − 4095j)·(723 + 723j) = 5 921 370, beyond either word: it saturates.
    let z = correct_fixed(&[(0, 0), (4095, -4095)], -(1 << 19), 12, &fx);
    assert_eq!(z[1], (2047, 0));
    let z = correct_fixed(&[(0, 0), (4095, -4095)], -(1 << 19), 13, &fx);
    assert_eq!(z[1], (4095, 0));
    let z = correct_fixed(&[(0, 0), (-4095, 4095)], -(1 << 19), 12, &fx);
    assert_eq!(z[1], (-2048, 0));
    let z = correct_fixed(&[(0, 0), (-4095, 4095)], -(1 << 19), 13, &fx);
    assert_eq!(z[1], (-4096, 0));
    // The widths at the guard's edges are taken: 1000·1023 = 1 023 000 of 23 bits cut to 2 is
    // 0.49 → 0, and widened to 30 is ·2^7.
    assert_eq!(correct_fixed(&y[..1], 0, 2, &fx), vec![(0, 0)]);
    assert_eq!(
        correct_fixed(&y[..1], 0, 30, &fx),
        vec![(1_023_000 << 7, -(511_500 << 7))]
    );
    // Unusable widths give nothing.
    assert!(correct_fixed(&y, 1, 1, &fx).is_empty());
    assert!(correct_fixed(&y, 1, 31, &fx).is_empty());
    let bad = FixedRxConfig {
        nco_lut_bits: 30,
        ..fx
    };
    assert!(correct_fixed(&y, 1, 12, &bad).is_empty());
    assert!(correct_fixed(&[], 1, 12, &fx).is_empty());
}

/// Against the float rotation y·e^(−j2πφn/2^P) (what `rx_front::correct` does with the CFO in
/// Hz), over streams long enough for the phase to wrap thousands of times. The bound is the
/// construction's: the phase truncated to L bits (at most 2π/2^L late), a table entry rounded
/// (half a unit of 2^(Q−1) − 1 per component), the output rounded (half a unit per component).
#[test]
fn correct_fixed_is_the_float_rotation_within_its_bound() {
    let fx = FixedRxConfig::default();
    let p = f64::from(1u32 << fx.nco_phase_bits);
    let amp = f64::from((1u32 << (fx.nco_amp_bits - 1)) - 1);
    let mut rng = Rng::new(0xc0fe);
    for out_bits in [12u32, 13] {
        // 13 bits of sample times Q − 1 of amplitude, cut to O.
        let scale = amp / f64::from(1u32 << (13 + fx.nco_amp_bits - 1 - out_bits));
        for cfo in [
            0i64,
            1,
            -1,
            49_283,
            -49_283,
            12_345,
            (1 << 21) - 1,
            -(1 << 21),
        ] {
            // Within ±2896: no rotation of it leaves 13 bits, so nothing saturates.
            // Three periods of a table address (2^13 phase units) for the slowest CFOs, so
            // that their phase visits every remainder equally often.
            let y: Vec<IQ> = (0..24_576)
                .map(|_| {
                    let v = |r: &mut Rng| ((r.uniform() * 2.0 - 1.0) * 2896.0) as i64;
                    (v(&mut rng), v(&mut rng))
                })
                .collect();
            let z = correct_fixed(&y, cfo, out_bits, &fx);
            assert_eq!(z.len(), y.len());
            let on_address = cfo == 0 || cfo == -(1 << 21);
            let (mut worst, mut mean_phase) = (0.0f64, 0.0f64);
            for (n, (z, y)) in z.iter().zip(&y).enumerate() {
                let turn = (-(cfo as i128) * n as i128).rem_euclid(1 << fx.nco_phase_bits);
                let (sin, cos) = (2.0 * PI * turn as f64 / p).sin_cos();
                let (a, b) = (y.0 as f64, y.1 as f64);
                let want = ((a * cos - b * sin) * scale, (a * sin + b * cos) * scale);
                let mag = (a * a + b * b).sqrt();
                // A phase that stays on table addresses (no CFO, or half a turn per sample)
                // is not truncated.
                let late = if on_address {
                    0.0
                } else {
                    2.0 * PI / f64::from(1u32 << fx.nco_lut_bits)
                };
                let bound = mag * scale * late
                    + mag * scale * 0.5 * 2f64.sqrt() / amp
                    + 0.5 * 2f64.sqrt()
                    + 1e-9;
                let e = ((z.0 as f64 - want.0).powi(2) + (z.1 as f64 - want.1).powi(2)).sqrt();
                assert!(e <= bound, "O {out_bits} CFO {cfo} n {n}: {e} > {bound}");
                worst = worst.max(e / bound);
                // The angle from the wanted value to the one given.
                mean_phase += (z.1 as f64 * want.0 - z.0 as f64 * want.1)
                    .atan2(z.0 as f64 * want.0 + z.1 as f64 * want.1);
            }
            mean_phase /= y.len() as f64;
            // The truncation is one-sided: with a CFO whose phase visits every remainder the
            // rotation lags by half a table address on average (π/2^L = 0.0061 rad), and a
            // phase on table addresses not at all.
            let half = PI / f64::from(1u32 << fx.nco_lut_bits);
            // A CFO of a unit or two visits three table entries in the whole stream, and their
            // own rounding (up to half a unit of 1023, 5·10⁻⁴ rad) does not average out.
            let tolerance = if cfo.abs() < 1000 || cfo == (1 << 21) - 1 {
                0.1
            } else {
                0.01
            };
            if on_address {
                assert!(mean_phase.abs() < 2e-4, "{mean_phase}");
            } else {
                assert!(
                    (mean_phase + half).abs() < tolerance * half,
                    "CFO {cfo}: mean phase {mean_phase}, wanted {}",
                    -half
                );
            }
            assert!(worst > 0.3, "the bound is never approached: {worst}");
        }
    }
}

/// At O = Z it is the front's own mixer at the phase −φ·n mod 2^P: the same table, the same
/// narrowing. (That the accumulator itself wraps cannot be seen from outside: the table
/// reduces the phase again.)
#[test]
fn correct_fixed_is_the_fronts_mixer_with_a_wrapping_accumulator() {
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x51de);
    for cfo in [77_777i64, -(1 << 21), (1 << 21) - 1] {
        let y: Vec<IQ> = (0..5000)
            .map(|_| {
                let v = |r: &mut Rng| ((r.uniform() * 2.0 - 1.0) * 4095.0) as i64;
                (v(&mut rng), v(&mut rng))
            })
            .collect();
        let z = correct_fixed(&y, cfo, fx.z_bits, &fx);
        for (n, (z, y)) in z.iter().zip(&y).enumerate() {
            let phase = (-(cfo as i128) * n as i128).rem_euclid(1 << fx.nco_phase_bits) as i64;
            assert_eq!(Some(*z), nco_mix(*y, phase, &fx), "CFO {cfo} n {n}");
        }
    }
}

/// The whole chain in integers on the ADC's samples: DC blocker, acquisition, the CFO taken
/// out by `correct_fixed`, the integer decoder started from the clock offset the CFO word
/// implies at 5.8 GHz. A frame with that clock offset in its CFO and its sample clock, a DC
/// offset of −30 dBFS and noise 30 dB down comes back at every hand-off width of the G0
/// table; with the CFO left in the samples it does not.
#[test]
fn the_all_integer_chain_decodes_a_frame_from_adc_samples() {
    let fs = 20e6;
    let cfg = RxConfig::new(fs);
    let fx = FixedRxConfig::default();
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(0xadc0);
    let carrier = 5.8e9;
    for (rate, ppm) in [(Rate::R54, -31.0), (Rate::R6, 20.7), (Rate::R24, 0.5)] {
        let cfo = ppm * 1e-6 * carrier;
        let psdu: Vec<u8> = (0..300).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f: Vec<C> = build_ppdu(&psdu, rate, 0x5d, &tx)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let mut x = vec![(0.0, 0.0); 500];
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); 200]);
        let x = resample(&x, ppm);
        let x = frequency_offset(&x, cfo, fs, 1.0);
        let p = mean_power(&f);
        let x = awgn(&x, 30.0, p, &mut rng);
        // The frame at −12 dBFS, the offset at −30 dBFS.
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
        let dc = 10f64.powf(-30.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
        let x: Vec<C> = x.iter().map(|s| (s.0 + 0.6 * dc, s.1 - 0.8 * dc)).collect();
        let adc = to_adc(&x, gain);
        let a = acquire_fixed(&adc, &cfg, &fx).unwrap();
        assert!(
            (a.cfo_hz(&fx, fs) - cfo).abs() < 2e3,
            "{}",
            a.cfo_hz(&fx, fs)
        );
        let y = dc_block_fixed(&adc, &cfg, &fx);
        for (out_bits, in_shift) in [(12u32, 2u32), (13, 1), (13, 2)] {
            let fo = FixedOfdmConfig {
                in_bits: out_bits,
                in_shift,
                ..FixedOfdmConfig::default()
            };
            assert!(fo.is_valid());
            let decode = |z: &[IQ]| {
                decode_ppdu_fixed(
                    z,
                    a.fft_start,
                    &RxOfdmConfig::default(),
                    &fo,
                    FixedTracking::from_clock_offset(a.cfo_hz(&fx, fs) / carrier * 1e6, &fo),
                )
            };
            let z = correct_fixed(&y, a.cfo_phase, out_bits, &fx);
            let d = decode(&z).unwrap();
            assert_eq!(d.psdu, psdu, "{rate:?} O {out_bits} shift {in_shift}");
            // The same samples with the CFO left in.
            let raw = correct_fixed(&y, 0, out_bits, &fx);
            if cfo.abs() > 50e3 {
                assert!(decode(&raw).map_or(true, |d| d.psdu != psdu), "{rate:?}");
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The DC offset from the two long training symbols, in integers (candidate for ADR 0008's
// hand-off; G0 in `docs/gates/rx_front.md`).
// ---------------------------------------------------------------------------------------------

/// The default rotator: P = 22, a table of 512 entries of amplitude A = 1023, so the phase
/// after 64 samples is 64·φ and one table address is 2^13 of it.
#[test]
fn ltf_dc_offset_fixed_worked_by_hand() {
    let fx = FixedRxConfig::default();
    assert_eq!(
        (fx.nco_phase_bits, fx.nco_lut_bits, fx.nco_amp_bits),
        (22, 9, 11)
    );
    let one = |at: usize, v: (i16, i16), cfo_phase: i64| {
        let mut x = vec![(0i16, 0i16); 400];
        x[at] = v;
        ltf_dc_offset_fixed(&x, 100, cfo_phase, &fx).unwrap()
    };
    // 64·φ = π: φ = 2^21/64 = 2^15. e = −A, d = A − e = 2A, D = S·d/(64·d²) = b/128 for a
    // sample b of the second symbol alone, a/128 for one of the first.
    let pi = 1i64 << 15;
    assert_eq!(one(164, (640, 0), pi), (5, 0));
    assert_eq!(one(100, (0, -640), pi), (0, -5));
    // The window: samples 100..=163 and 164..=227.
    assert_eq!(one(163, (640, 0), pi), (5, 0));
    assert_eq!(one(227, (640, 0), pi), (5, 0));
    assert_eq!(one(99, (640, 0), pi), (0, 0));
    assert_eq!(one(228, (640, 0), pi), (0, 0));
    // Rounding half to even, either sign: 0.5 → 0, 1.5 → 2, 2.5 → 2, and just off the tie.
    assert_eq!(one(164, (64, -64), pi), (0, 0));
    assert_eq!(one(164, (192, -192), pi), (2, -2));
    assert_eq!(one(164, (320, -320), pi), (2, -2));
    assert_eq!(one(164, (65, -65), pi), (1, -1));
    assert_eq!(one(164, (63, -63), pi), (0, 0));
    assert_eq!(one(164, (191, -193), pi), (1, -2));
    // 64·φ = π/2: φ = 2^14. e = jA, d = A(1 − j), 1/d = (1 + j)/(2A).
    // b = 256: D = 256·(1 + j)/128 = 2 + 2j. a = 256: S = −jA·256, D = 2 − 2j.
    let half = 1i64 << 14;
    assert_eq!(one(164, (256, 0), half), (2, 2));
    assert_eq!(one(100, (256, 0), half), (2, -2));
    // The other sign of the CFO, given as a negative word or wrapped into P bits.
    assert_eq!(one(164, (256, 0), -half), (2, -2));
    assert_eq!(one(100, (256, 0), -half), (2, 2));
    assert_eq!(one(164, (256, 0), (1 << 22) - half), (2, -2));
    assert_eq!(one(164, (256, 0), half + (1 << 22)), (2, 2));
}

/// A constant stream is its own offset wherever the turn after 64 samples can be divided by:
/// |d|²·256 ≥ A². Table address 5: e = (1021, 63), d = (2, −63), 3973·256 < 1023²; address
/// 6: e = (1020, 75), d = (3, −75), 5634·256 > 1023². The address is the phase 64·φ rounded
/// to the table: 5 up to φ = 703 (64·703 = 5.49·2^13), 6 from 704. Half an address is added
/// and the phase cut, so a tie goes up: −704 (−5.5) is address −5, −705 is −6.
#[test]
fn ltf_dc_offset_fixed_guard_and_limits() {
    let fx = FixedRxConfig::default();
    let x = vec![(100i16, -50i16); 400];
    for phi in [0i64, 1, 64, 640, 703, -640, -703, -704, (1 << 22) - 704] {
        assert_eq!(
            ltf_dc_offset_fixed(&x, 100, phi, &fx),
            Some((0, 0)),
            "{phi}"
        );
    }
    for phi in [
        704i64,
        768,
        -705,
        -768,
        1 << 14,
        1 << 15,
        48_600,
        -48_600,
        (1 << 22) - 705,
    ] {
        assert_eq!(
            ltf_dc_offset_fixed(&x, 100, phi, &fx),
            Some((100, -50)),
            "{phi}"
        );
    }
    // A full-scale constant stream, either sign: the estimate is the stream, 12 bits.
    for v in [(2047i16, -2048i16), (-2048, 2047)] {
        let x = vec![v; 400];
        assert_eq!(
            ltf_dc_offset_fixed(&x, 100, 1 << 15, &fx),
            Some((i64::from(v.0), i64::from(v.1)))
        );
    }
    // Two symbols that are not a repeating field can ask for more than 12 bits: the first
    // at one rail and the second at the other, just above the guard. S = 64·(b·A − e·a),
    // of the order of 2·2047·A against |d| = 0.07·A: saturated to 12 bits.
    let mut y = vec![(-2048i16, 2047i16); 400];
    for v in y.iter_mut().skip(164) {
        *v = (2047, -2048);
    }
    let d = ltf_dc_offset_fixed(&y, 100, 704, &fx).unwrap();
    assert!(
        [d.0, d.1].iter().all(|c| *c == 2047 || *c == -2048),
        "{d:?}"
    );
    // The 64 pairs must be in the stream, whatever the CFO; an invalid configuration gives
    // nothing.
    assert!(ltf_dc_offset_fixed(&x[..227], 100, 0, &fx).is_none());
    assert!(ltf_dc_offset_fixed(&x[..228], 100, 0, &fx).is_some());
    assert!(ltf_dc_offset_fixed(&x[..228], 100, 1 << 15, &fx).is_some());
    assert!(ltf_dc_offset_fixed(&x[..227], 100, 1 << 15, &fx).is_none());
    assert!(ltf_dc_offset_fixed(&x, usize::MAX, 1 << 15, &fx).is_none());
    let bad = FixedRxConfig {
        nco_lut_bits: 1,
        ..fx
    };
    assert!(!bad.is_valid());
    assert!(ltf_dc_offset_fixed(&x, 100, 1 << 15, &bad).is_none());
}

/// Off the default rotator. A table of 4096 entries (L = 12): one address is 2^10 of the
/// phase 64·φ, so φ = 16 per address. Address 39: e = round(1023·(cos, sin)(2π·39/4096)) =
/// (1021, 61), d = (2, −61), |d|² = 3725, 256·3725 < 1023²; address 41: e = (1021, 64),
/// d = (2, −64), |d|² = 4100, 256·4100 > 1023². The tie at half an address goes up.
/// A table as long as the phase (L = P): no half address to add. The widest rotator the
/// configuration allows (Q = 30) and samples of the full 16 bits: nothing overflows.
#[test]
fn ltf_dc_offset_fixed_off_the_default_configuration() {
    let x = vec![(100i16, -50i16); 400];
    let fine = FixedRxConfig {
        nco_lut_bits: 12,
        ..FixedRxConfig::default()
    };
    assert!(fine.is_valid());
    let at = |phi: i64| ltf_dc_offset_fixed(&x, 100, phi, &fine);
    // Address 39 = φ 624; up to 39.49 addresses (φ 631) it stays 39; φ 632 is 39.5 → 40.
    assert_eq!(at(624), Some((0, 0)));
    assert_eq!(at(631), Some((0, 0)));
    // Address 40: e = (1021, 63), d = (2, −63), |d|² = 3973 < 4087.9: still inside.
    assert_eq!(at(632), Some((0, 0)));
    assert_eq!(at(647), Some((0, 0)));
    // Address 41 from φ = 648 (40.5): outside the guard.
    assert_eq!(at(648), Some((100, -50)));
    assert_eq!(at(656), Some((100, -50)));
    let whole = FixedRxConfig {
        nco_lut_bits: 22,
        ..FixedRxConfig::default()
    };
    assert!(whole.is_valid());
    assert_eq!(
        ltf_dc_offset_fixed(&x, 100, 1 << 15, &whole),
        Some((100, -50))
    );
    // A quarter turn: e = jA exactly, d = A(1 − j).
    assert_eq!(
        ltf_dc_offset_fixed(&x, 100, 1 << 14, &whole),
        Some((100, -50))
    );
    let wide = FixedRxConfig {
        nco_amp_bits: 30,
        ..FixedRxConfig::default()
    };
    assert!(wide.is_valid());
    let mut y = vec![(i16::MIN, i16::MAX); 400];
    for v in y.iter_mut().skip(164) {
        *v = (i16::MAX, i16::MIN);
    }
    for phi in [704i64, 1 << 13, 3 << 13, 1 << 14, 1 << 15, -(1 << 14)] {
        let d = ltf_dc_offset_fixed(&y, 100, phi, &wide).unwrap();
        assert!((-2048..=2047).contains(&d.0) && (-2048..=2047).contains(&d.1));
    }
    // And a constant stream is still its own offset with it.
    assert_eq!(
        ltf_dc_offset_fixed(&x, 100, 1 << 15, &wide),
        Some((100, -50))
    );
}

#[test]
fn remove_dc_fixed_worked_by_hand() {
    let x = [(0i16, 0i16), (5, -7), (2047, -2048), (-2048, 2047)];
    assert_eq!(
        remove_dc_fixed(&x, (3, -2)),
        vec![(-3, 2), (2, -5), (2044, -2046), (-2051, 2049)]
    );
    // 12-bit samples less a 12-bit offset fit the 13 bits of the rotator's input.
    assert_eq!(
        remove_dc_fixed(&x, (-2048, 2047)),
        vec![(2048, -2047), (2053, -2054), (4095, -4095), (0, 0)]
    );
    // An offset outside 12 bits cannot come from the estimator; the output saturates.
    assert_eq!(
        remove_dc_fixed(&x[2..], (-5000, 5000)),
        vec![(4095, -4096), (2952, -2953)]
    );
    assert_eq!(remove_dc_fixed(&x, (0, 0))[1], (5, -7));
}

/// On ADC samples of frames with an offset added: the integer estimate is the float one of
/// the same samples within its rounding (half an LSB) and what the table's address rounds
/// away (below), and both are the offset within the noise's reach.
#[test]
fn ltf_dc_offset_fixed_follows_the_float_one() {
    let fs = 20e6;
    let fx = FixedRxConfig::default();
    let f = preamble();
    let mut rng = Rng::new(0x1dc0);
    let d = Multipath::new(
        tgn_taps(TgnModel::D, false),
        Doppler::Static,
        1,
        fs,
        &mut rng,
    );
    let (mut n, mut worst, mut worst_true) = (0, 0.0f64, 0.0f64);
    for k in 0..600 {
        let snr = [10.0, 20.0, 35.0][k % 3];
        let ch = (k % 2 == 1).then_some(&d);
        let (adc, t1, cfo) = adc_scene(&f, ch, snr, fs, &mut rng);
        let dc = (
            (rng.uniform() * 600.0 - 300.0).round(),
            (rng.uniform() * 600.0 - 300.0).round(),
        );
        let adc: Vec<(i16, i16)> = adc
            .iter()
            .map(|&(a, b)| {
                (
                    (f64::from(a) + dc.0).clamp(-2048.0, 2047.0) as i16,
                    (f64::from(b) + dc.1).clamp(-2048.0, 2047.0) as i16,
                )
            })
            .collect();
        let phi = (cfo / fs * 2f64.powi(22)).round() as i64;
        let fixed = ltf_dc_offset_fixed(&adc, t1, phi, &fx).unwrap();
        let float = ltf_dc_offset(&as_f64(&adc), t1, cfo, fs).unwrap();
        // The integers differ from the float estimate by their rounding (half an LSB) and by
        // their rotator: its phase is off by at most half an address (π/512) and each of
        // its components by half a unit of 1023. To first order the estimate moves by that
        // error times the difference of the two symbols' means over |d|² (the reviewer's
        // derivation; 10 % for the second order).
        let turn = 2.0 * (PI * 64.0 * cfo / fs).sin().abs();
        let mean = |r: std::ops::Range<usize>| {
            let s = adc[r].iter().fold((0.0, 0.0), |s, v| {
                (s.0 + f64::from(v.0), s.1 + f64::from(v.1))
            });
            (s.0 / 64.0, s.1 / 64.0)
        };
        let (m1, m2) = (mean(t1..t1 + 64), mean(t1 + 64..t1 + 128));
        let apart = (m2.0 - m1.0).hypot(m2.1 - m1.1);
        if turn < 0.08 {
            // At the guard the two may decide differently; both sides of it are tested above.
            continue;
        }
        n += 1;
        let bound = 0.5 + 1.1 * (PI / 512.0 + 2f64.sqrt() / 1023.0) * apart / (turn * turn);
        let e = (fixed.0 as f64 - float.0)
            .abs()
            .max((fixed.1 as f64 - float.1).abs());
        assert!(e <= bound, "frame {k}: {fixed:?} {float:?} bound {bound}");
        worst = worst.max(e / bound);
        // Against the offset put in: the noise of 64 differences, 2σ²/(64·|d|²) a
        // component pair, five standard deviations and the bound above.
        let sigma = 364.0 * 10f64.powf(-snr / 20.0);
        let reach = 5.0 * sigma * (2.0f64 / 64.0).sqrt() / turn + bound;
        let t = (fixed.0 as f64 - dc.0)
            .abs()
            .max((fixed.1 as f64 - dc.1).abs());
        assert!(t <= reach, "frame {k}: {fixed:?} {dc:?} reach {reach}");
        worst_true = worst_true.max(t / reach);
    }
    assert!(n > 500, "{n}");
    // Neither bound is idle: something comes within a factor of two of the first and of
    // twenty of the second.
    assert!(worst > 0.5, "{worst}");
    assert!(worst_true > 0.05, "{worst_true}");
}

/// A DC offset 8 dB below the frame (−20 dBFS under −12 dBFS), a clock offset that puts the
/// offset between two subcarriers once the CFO is out: the frame comes back when the offset
/// estimated from the long training field is taken out of the ADC's samples, at every
/// hand-off width, and the 54 Mb/s one does not when it is left in.
#[test]
fn the_ltf_estimate_takes_a_dc_offset_out_of_the_decoders_samples() {
    let fs = 20e6;
    let cfg = RxConfig::new(fs);
    let fx = FixedRxConfig::default();
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(0xdc20);
    let carrier = 5.8e9;
    for (rate, ppm) in [(Rate::R54, -31.0), (Rate::R54, 27.0), (Rate::R6, 20.7)] {
        let cfo = ppm * 1e-6 * carrier;
        let psdu: Vec<u8> = (0..300).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f: Vec<C> = build_ppdu(&psdu, rate, 0x5d, &tx)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let mut x = vec![(0.0, 0.0); 500];
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); 200]);
        let x = resample(&x, ppm);
        let x = frequency_offset(&x, cfo, fs, 1.0);
        let p = mean_power(&f);
        let x = awgn(&x, 30.0, p, &mut rng);
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
        let dc = 10f64.powf(-20.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
        let x: Vec<C> = x.iter().map(|s| (s.0 + 0.6 * dc, s.1 - 0.8 * dc)).collect();
        let adc = to_adc(&x, gain);
        let a = acquire_fixed(&adc, &cfg, &fx).unwrap();
        let est = ltf_dc_offset_fixed(&adc, a.fft_start, a.cfo_phase, &fx).unwrap();
        // 0.6 and −0.8 of 204.8 LSB.
        assert!(
            (est.0 - 123).abs() <= 4 && (est.1 + 164).abs() <= 4,
            "{est:?}"
        );
        for (out_bits, in_shift) in [(12u32, 2u32), (13, 1), (13, 2)] {
            let fo = FixedOfdmConfig {
                in_bits: out_bits,
                in_shift,
                ..FixedOfdmConfig::default()
            };
            let decode = |y: &[IQ]| {
                decode_ppdu_fixed(
                    &correct_fixed(y, a.cfo_phase, out_bits, &fx),
                    a.fft_start,
                    &RxOfdmConfig::default(),
                    &fo,
                    FixedTracking::from_clock_offset(a.cfo_hz(&fx, fs) / carrier * 1e6, &fo),
                )
            };
            let d = decode(&remove_dc_fixed(&adc, est)).unwrap();
            assert_eq!(d.psdu, psdu, "{rate:?} O {out_bits} shift {in_shift}");
            if rate == Rate::R54 {
                let left_in = decode(&remove_dc_fixed(&adc, (0, 0)));
                assert!(left_in.map_or(true, |d| d.psdu != psdu), "{ppm}");
            }
        }
    }
}

#[test]
fn handoff_fixed_worked_by_hand() {
    let fx = FixedRxConfig::default();
    assert_eq!(
        (fx.nco_phase_bits, fx.nco_lut_bits, fx.nco_amp_bits),
        (22, 9, 11)
    );
    // A CFO word of zero: inside the estimate's guard, so nothing is taken off, and the
    // rotator stands at (1023, 0): every sample is v·1023/1024 rounded half to even, in the
    // ADC's own LSB (13 bits out). 511·1023/1024 = 510.50 → 511; 512 → 511.5 → 512 (even);
    // 513 → 512.50 − 0.001 → 512; 2047 → 2045.001 → 2045; −2048 → −2046 exactly.
    let mut adc = vec![(0i16, 0i16); 200];
    adc[3] = (1, -1);
    adc[70] = (511, -511);
    adc[71] = (512, -512);
    adc[72] = (513, -513);
    adc[150] = (2047, -2048);
    let z = handoff_fixed(&adc, 40, 0, &fx).unwrap();
    assert_eq!(z.len(), 200);
    assert_eq!(z[3], (1, -1));
    assert_eq!(z[70], (511, -511));
    assert_eq!(z[71], (512, -512));
    assert_eq!(z[72], (512, -512));
    assert_eq!(z[150], (2045, -2046));
    assert!(z
        .iter()
        .enumerate()
        .all(|(n, v)| [3, 70, 71, 72, 150].contains(&n) || *v == (0, 0)));

    // A word of 2^18 + 2^15: half a turn over the 64 samples between the two symbols, so
    // e = −A, d = 2A, S = 64·2A·D and the estimate is the offset exactly. A stream that is
    // the offset (100, −50) alone comes out as nothing; one sample 1024 above it before the
    // two symbols, at index 32 where the rotator has gone 32·(2^18 + 2^15) = two turns and a
    // quarter backwards (·(−j)), comes out as 1024·1023/1024·(−j) = (0, −1023).
    let word = (1i64 << 18) + (1 << 15);
    let mut adc = vec![(100i16, -50i16); 40 + 128 + 10];
    adc[32] = (1124, -50);
    let z = handoff_fixed(&adc, 40, word, &fx).unwrap();
    assert_eq!(z[32], (0, -1023));
    assert!(z.iter().enumerate().all(|(n, v)| n == 32 || *v == (0, 0)));
    // The same after the two symbols, at index 192: 192·(2^18 + 2^15) = thirteen turns and
    // a half, ·(−1).
    let mut adc = vec![(100i16, -50i16); 200];
    adc[192] = (1124, -50);
    let z = handoff_fixed(&adc, 40, word, &fx).unwrap();
    assert_eq!(z[192], (-1023, 0));
    // The two symbols are taken where the caller says: eight samples early the sample at 32
    // is inside them and the estimate is no longer the offset.
    let mut adc = vec![(100i16, -50i16); 200];
    adc[32] = (1124, -50);
    let early = handoff_fixed(&adc, 24, word, &fx).unwrap();
    assert_ne!(early[100], (0, 0));

    // No two symbols in the stream, or widths that are not usable: nothing.
    let adc = vec![(100i16, -50i16); 167];
    assert_eq!(handoff_fixed(&adc, 40, word, &fx), None);
    assert!(handoff_fixed(&adc, 39, word, &fx).is_some());
    let bad = FixedRxConfig {
        nco_lut_bits: 1,
        ..fx
    };
    assert_eq!(handoff_fixed(&adc, 39, word, &bad), None);
    // The width is the rotator's input, whatever the front's own mixer narrows to.
    let z20 = FixedRxConfig { z_bits: 20, ..fx };
    let mut adc = vec![(0i16, 0i16); 200];
    adc[150] = (2047, -2048);
    assert_eq!(
        handoff_fixed(&adc, 40, 0, &z20).unwrap()[150],
        (2045, -2046)
    );
}

#[test]
fn handoff_fixed_is_the_estimate_the_subtraction_and_the_rotation() {
    // The stream's definition (ADR 0008, decision 1) step by step, on samples that are
    // nothing in particular, at the rotator's input width out.
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0x0ff5);
    for word in [0i64, 703, 705, -9_000, 48_660, -48_660, (1 << 21) + 77] {
        let adc: Vec<(i16, i16)> = (0..400)
            .map(|_| {
                let mut v = || ((rng.next_u64() % 4096) as i64 - 2048) as i16;
                (v(), v())
            })
            .collect();
        let dc = ltf_dc_offset_fixed(&adc, 211, word, &fx).unwrap();
        let by_steps = correct_fixed(&remove_dc_fixed(&adc, dc), word, Y_BITS, &fx);
        assert_eq!(handoff_fixed(&adc, 211, word, &fx).unwrap(), by_steps);
        assert!(
            by_steps
                .iter()
                .any(|v| v.0.abs() > 2048 || v.1.abs() > 2048)
                || dc == (0, 0)
        );
    }
}

#[test]
fn the_default_receiver_decodes_the_hand_off() {
    // The chain ADR 0008 decided, each piece at its default: the integer acquisition, the
    // hand-off and the decoder's default input format, on a frame with a clock offset and a
    // DC offset of −20 dBFS at the ADC.
    let fs = 20e6;
    let cfg = RxConfig::new(fs);
    let fx = FixedRxConfig::default();
    let fo = FixedOfdmConfig::default();
    assert_eq!(fo.front, fx);
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(0x0ada);
    let carrier = 5.8e9;
    for (rate, ppm) in [(Rate::R54, 33.0), (Rate::R36, -12.5), (Rate::R6, -38.0)] {
        let psdu: Vec<u8> = (0..300).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f: Vec<C> = build_ppdu(&psdu, rate, 0x5d, &tx)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let mut x = vec![(0.0, 0.0); 500];
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); 200]);
        let x = resample(&x, ppm);
        let x = frequency_offset(&x, ppm * 1e-6 * carrier, fs, 2.0);
        let p = mean_power(&f);
        let x = awgn(&x, 30.0, p, &mut rng);
        let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
        let dc = 10f64.powf(-20.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
        let x: Vec<C> = x.iter().map(|s| (s.0 - 0.8 * dc, s.1 + 0.6 * dc)).collect();
        let adc = to_adc(&x, gain);
        let a = acquire_fixed(&adc, &cfg, &fx).unwrap();
        let z = handoff_fixed(&adc, a.fft_start, a.cfo_phase, &fx).unwrap();
        // The tracker is started from the CFO word and the channel's register (ADR 0008,
        // decision 7): no floating point between the ADC and the PSDU.
        let ratio = clock_ratio(fs, carrier).unwrap();
        let start = FixedTracking::from_cfo_word(a.cfo_phase, ratio, &fo);
        let d = decode_ppdu_fixed(&z, a.fft_start, &RxOfdmConfig::default(), &fo, start).unwrap();
        assert_eq!(d.psdu, psdu, "{rate:?} {ppm}");
        // The sign is the clock's: a transmitter fast by ε has moved the last symbol's
        // window ε·(112 + 80·i) samples late (i symbols after SIGNAL; 0.31 of a sample
        // early at the end of the 6 Mb/s frame), and the start from the word knows it from
        // the first symbol on.
        let i = d.timing.len() - 1;
        let late = ppm * 1e-6 * (112 + 80 * i) as f64;
        assert!(
            (d.timing[i] - late).abs() < 0.03,
            "{rate:?} {ppm}: {} against {late}",
            d.timing[i]
        );
        if rate == Rate::R6 {
            assert!(i == 101 && late < -0.3, "{i} {late}");
            let wrong = FixedTracking::from_cfo_word(-a.cfo_phase, ratio, &fo);
            let w = decode_ppdu_fixed(&z, a.fft_start, &RxOfdmConfig::default(), &fo, wrong);
            assert!(w.map_or(true, |w| (w.timing[i] - late).abs() > 0.1));
        }
    }
}
