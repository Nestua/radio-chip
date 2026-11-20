//! G2 gate of the OFDM receiver's integer data path (R0-09 G2, stage A,
//! `radio_phy::rx_ofdm_fixed`): FFT, channel estimate, equaliser without a division per
//! subcarrier, demapper and soft values (`docs/gates/rx_ofdm.md`, G0 of 2026-11-19).
//!
//! References:
//! 1. The FFT: transforms known by hand (a constant, an impulse, an alternating sequence), and
//!    the float FFT of the same samples (`channel::fft`, which has its own G1).
//! 2. The float receiver (`rx_ofdm`, G1 against the standard's printed waveform): with wide
//!    words the integer path must give its soft values.
//! 3. The transmitter: every rate comes back with the default words.
//!
//! The loss in dB against the float receiver and the sweep of each width are a measurement
//! (`rxper fixed`), not a test.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, fft, mean_power, Rng, Sample};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front_fixed::{FixedRxConfig, IQ};
use radio_phy::rx_ofdm::{
    decode_ppdu, decode_ppdu_from, equalise, estimate_channel, soft_bits, RxOfdmConfig, Tracking,
};
use radio_phy::rx_ofdm_fixed::{
    decode_ppdu_fixed, estimate_channel_fixed, fft64_fixed, scale_constant, symbol_soft_fixed,
    threshold_constant, to_fixed, FixedOfdmConfig,
};

/// A frame with 192 samples of silence before T1's window (as after an acquisition with no
/// back-off: STF 160, GI2 32) — the frame itself starts at sample 0.
fn frame(psdu: &[u8], rate: Rate) -> Vec<Sample> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, 0x5d, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

fn psdu(len: usize, rng: &mut Rng) -> Vec<u8> {
    (0..len).map(|_| (rng.next_u64() & 0xff) as u8).collect()
}

/// `x` scaled so that its mean power is `dbfs` against a full scale of 1.
fn at_level(x: &[Sample], dbfs: f64) -> Vec<Sample> {
    let k = (10f64.powf(dbfs / 10.0) / mean_power(x)).sqrt();
    x.iter().map(|s| (s.0 * k, s.1 * k)).collect()
}

/// A two-path channel: x[n] + a·x[n − d].
fn two_path(x: &[Sample], a: Sample, d: usize) -> Vec<Sample> {
    (0..x.len())
        .map(|n| {
            let e = if n >= d { x[n - d] } else { (0.0, 0.0) };
            (
                x[n].0 + a.0 * e.0 - a.1 * e.1,
                x[n].1 + a.0 * e.1 + a.1 * e.0,
            )
        })
        .collect()
}

#[test]
fn fft_of_inputs_known_by_hand() {
    let fx = FixedOfdmConfig::default();
    // A constant c: bin 0 is 64·c, the transform divides by 8: 8·c; every other bin 0.
    let x = [(100i64, -40i64); 64];
    let y = fft64_fixed(&x, &fx).unwrap();
    assert_eq!(y[0], (800, -320));
    assert!(y[1..].iter().all(|v| *v == (0, 0)), "{y:?}");
    // An impulse of 8·a at n = 0: every bin a.
    let mut x = [(0i64, 0i64); 64];
    x[0] = (808, -16);
    assert!(fft64_fixed(&x, &fx)
        .unwrap()
        .iter()
        .all(|v| *v == (101, -2)));
    // +c, −c, +c, …: bin 32 alone, 8·c.
    let x: Vec<IQ> = (0..64)
        .map(|n| if n % 2 == 0 { (50, 7) } else { (-50, -7) })
        .collect();
    let y = fft64_fixed(&x, &fx).unwrap();
    assert_eq!(y[32], (400, 56));
    assert!((0..64).filter(|k| *k != 32).all(|k| y[k] == (0, 0)));
    // e^(−j2π·16n/64) = (−j)^n times c lands in bin 48 = −16 (the forward transform's sign).
    let c = 64i64;
    let x: Vec<IQ> = (0..64)
        .map(|n| [(c, 0), (0, -c), (-c, 0), (0, c)][n % 4])
        .collect();
    let y = fft64_fixed(&x, &fx).unwrap();
    assert_eq!(y[48], (8 * c, 0));
    assert!((0..64).filter(|k| *k != 48).all(|k| y[k] == (0, 0)));
    // Ties go to the even value: an impulse of 12 is 1.5 in every bin, 2; one of 20 is 2.5,
    // 2; one of −12, −2.
    for (a, want) in [(12, 2), (20, 2), (-12, -2), (4, 0), (28, 4)] {
        let mut x = [(0i64, 0i64); 64];
        x[0] = (a, 0);
        let y = fft64_fixed(&x, &fx).unwrap();
        assert!(y.iter().all(|v| *v == (want, 0)), "{a}: {:?}", y[0]);
    }
    // The word saturates: a constant at the top of 16 bits would be 8 times too large.
    let y = fft64_fixed(&[(32767, -32768); 64], &fx).unwrap();
    assert_eq!(y[0], (32767, -32768));
    // Lengths other than 64 and unusable widths are refused.
    assert!(fft64_fixed(&x[..63], &fx).is_none());
    let bad = FixedOfdmConfig { fft_bits: 40, ..fx };
    assert!(fft64_fixed(&x, &bad).is_none());
}

#[test]
fn fft_against_the_float_transform() {
    // Random 14-bit samples in the 16-bit word: against the float FFT over 8 the error is the
    // rounding of six stages and of the 12-bit twiddles: under 1 LSB rms, 4 LSB at most.
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x0f17);
    let (mut sum, mut worst, mut n) = (0.0f64, 0.0f64, 0);
    for _ in 0..200 {
        let x: Vec<IQ> = (0..64)
            .map(|_| {
                (
                    (rng.gaussian() * 1500.0).round() as i64,
                    (rng.gaussian() * 1500.0).round() as i64,
                )
            })
            .collect();
        let xf: Vec<Sample> = x.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
        let want = fft(&xf).unwrap();
        let got = fft64_fixed(&x, &fx).unwrap();
        for (g, w) in got.iter().zip(&want) {
            for e in [g.0 as f64 - w.0 / 8.0, g.1 as f64 - w.1 / 8.0] {
                sum += e * e;
                worst = worst.max(e.abs());
                n += 1;
            }
        }
    }
    let rms = (sum / f64::from(n)).sqrt();
    assert!(rms < 1.0 && rms > 0.2, "rms error {rms} LSB");
    assert!(worst < 4.0, "worst error {worst} LSB");
    // Narrow twiddles show: with 6 bits the error is several times larger.
    let coarse = FixedOfdmConfig {
        twiddle_bits: 6,
        ..fx
    };
    let x: Vec<IQ> = (0..64)
        .map(|n| (((n * 37) % 2001) as i64 - 1000, 300))
        .collect();
    let xf: Vec<Sample> = x.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
    let want = fft(&xf).unwrap();
    let err = |fx: &FixedOfdmConfig| {
        fft64_fixed(&x, fx)
            .unwrap()
            .iter()
            .zip(&want)
            .map(|(g, w)| (g.0 as f64 - w.0 / 8.0).powi(2) + (g.1 as f64 - w.1 / 8.0).powi(2))
            .sum::<f64>()
    };
    assert!(err(&coarse) > 9.0 * err(&fx));
}

/// Words wide enough that the integer path's own rounding is far below the soft values' step.
fn wide() -> FixedOfdmConfig {
    FixedOfdmConfig {
        in_bits: 20,
        in_shift: 4,
        fft_bits: 28,
        fft_halve: 0b10_1010,
        twiddle_bits: 24,
        prod_bits: 40,
        kappa_bits: 20,
        scale_frac: 8,
        front: FixedRxConfig {
            cordic_iters: 24,
            cordic_bits: 30,
            angle_bits: 24,
            nco_phase_bits: 30,
            nco_lut_bits: 16,
            nco_amp_bits: 20,
            ..FixedRxConfig::default()
        },
    }
}

#[test]
fn wide_words_give_the_float_receivers_soft_values() {
    // The same samples (quantised to 20 bits, so both receivers see one input) through the
    // float receiver and the integer path with wide words, symbol by symbol: SIGNAL and the
    // data of every modulation, through a two-path channel and noise, six-bit soft values.
    assert!(wide().is_valid());
    let cfg = RxOfdmConfig {
        soft_max: 31,
        clip: 2.0,
        ..RxOfdmConfig::default()
    };
    let fx = wide();
    let mut rng = Rng::new(0x0f18);
    for rate in [Rate::R6, Rate::R12, Rate::R36, Rate::R54] {
        let x = frame(&psdu(120, &mut rng), rate);
        let x = two_path(&x, (0.3, -0.5), 3);
        let x = at_level(&awgn(&x, 22.0, mean_power(&x), &mut rng), -12.0);
        let full = f64::from(1u32 << 19);
        let z = to_fixed(&x, full, 20);
        let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
        let fft_start = 192 - 4;
        let h = estimate_channel(&y[fft_start..fft_start + 128]).unwrap();
        let ch = estimate_channel_fixed(&z[fft_start..fft_start + 128], &fx).unwrap();
        let (mut tf, mut ti) = (Tracking::default(), Tracking::default());
        let symbols = (x.len() - 320) / 80;
        let (mut same, mut total) = (0usize, 0usize);
        for i in 0..symbols {
            let m = if i == 0 {
                Modulation::Bpsk
            } else {
                rate.modulation()
            };
            let start = fft_start + 144 + 80 * i;
            let eq = equalise(&y[start..start + 64], &h, i, m, &cfg, &mut tf).unwrap();
            let want = soft_bits(&eq, m, &cfg);
            let (got, _) =
                symbol_soft_fixed(&z[start..start + 64], &ch, i, m, &cfg, &fx, &mut ti).unwrap();
            assert_eq!(got.len(), want.len());
            for (g, w) in got.iter().zip(&want) {
                assert!((g - w).abs() <= 1, "{rate:?} symbol {i}: {g} against {w}");
                same += usize::from(g == w);
                total += 1;
            }
        }
        // A value on a rounding boundary may fall either way; nothing else may differ.
        assert!(same * 1000 >= total * 995, "{rate:?}: {same} of {total}");
    }
}

#[test]
fn every_rate_comes_back_with_the_default_words() {
    // 12-bit samples, the packet at −12 dBFS, 30 dB SNR; a flat channel and one with a notch
    // (two paths of nearly equal strength). The integer receiver's PSDU is the transmitted
    // one, its SIGNAL the frame's, and the float receiver agrees on the same samples.
    let fx = FixedOfdmConfig::default();
    assert!(fx.is_valid());
    let mut rng = Rng::new(0x0f19);
    for cfg in [
        RxOfdmConfig::default(),
        RxOfdmConfig {
            soft_max: 31,
            clip: 2.0,
            ..RxOfdmConfig::default()
        },
    ] {
        for rate in Rate::ALL {
            for notch in [false, true] {
                let data = psdu(1 + (rng.next_u64() % 300) as usize, &mut rng);
                let mut x = frame(&data, rate);
                if notch {
                    x = two_path(&x, (-0.6, 0.5), 2);
                }
                x.extend(vec![(0.0, 0.0); 8]);
                let x = at_level(&awgn(&x, 30.0, mean_power(&x), &mut rng), -12.0);
                let z = to_fixed(&x, 2048.0, 12);
                let d = decode_ppdu_fixed(&z, 192 - 6, &cfg, &fx, Tracking::default())
                    .unwrap_or_else(|e| panic!("{rate:?} notch {notch}: {e}"));
                assert_eq!(d.signal.rate, rate);
                assert_eq!(d.psdu, data, "{rate:?} notch {notch}");
                assert_eq!(d.scrambler_seed, 0x5d);
                let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
                assert_eq!(decode_ppdu(&y, 192 - 6, &cfg).unwrap().psdu, data);
            }
        }
    }
}

#[test]
fn the_level_window_and_hostile_arguments() {
    // The per-packet shift makes the path indifferent to the level: −3 to −40 dBFS decode.
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f1a);
    let data = psdu(100, &mut rng);
    let x = frame(&data, Rate::R24);
    for dbfs in [-3.0, -15.0, -30.0, -40.0] {
        let z = to_fixed(&at_level(&x, dbfs), 2048.0, 12);
        let d = decode_ppdu_fixed(&z, 190, &cfg, &fx, Tracking::default()).unwrap();
        assert_eq!(d.psdu, data, "{dbfs} dBFS");
    }
    // Nothing panics: silence, a truncated stream, a start beyond it, unusable widths,
    // samples outside the stated width (they saturate).
    let z = to_fixed(&at_level(&x, -12.0), 2048.0, 12);
    assert!(decode_ppdu_fixed(&vec![(0, 0); 2000], 190, &cfg, &fx, Tracking::default()).is_err());
    assert!(decode_ppdu_fixed(&z[..600], 190, &cfg, &fx, Tracking::default()).is_err());
    assert!(decode_ppdu_fixed(&z, usize::MAX - 3, &cfg, &fx, Tracking::default()).is_err());
    assert!(decode_ppdu_fixed(&[], 0, &cfg, &fx, Tracking::default()).is_err());
    for bad in [
        FixedOfdmConfig { in_bits: 0, ..fx },
        FixedOfdmConfig { in_shift: 9, ..fx },
        // The shifted input must fit the FFT's word.
        FixedOfdmConfig { in_shift: 6, ..fx },
        FixedOfdmConfig { fft_bits: 31, ..fx },
        FixedOfdmConfig {
            fft_halve: 64,
            ..fx
        },
        FixedOfdmConfig {
            twiddle_bits: 2,
            ..fx
        },
        FixedOfdmConfig {
            prod_bits: 60,
            ..fx
        },
        FixedOfdmConfig {
            kappa_bits: 40,
            ..fx
        },
        FixedOfdmConfig {
            scale_frac: 30,
            ..fx
        },
        FixedOfdmConfig {
            front: FixedRxConfig {
                nco_lut_bits: 1,
                ..fx.front
            },
            ..fx
        },
    ] {
        assert!(!bad.is_valid(), "{bad:?}");
        assert!(decode_ppdu_fixed(&z, 190, &cfg, &bad, Tracking::default()).is_err());
        assert!(estimate_channel_fixed(&z[190..318], &bad).is_none());
    }
    let wild: Vec<IQ> = z
        .iter()
        .map(|v| (v.0 * 1_000_000, v.1 * 1_000_000))
        .collect();
    let _ = decode_ppdu_fixed(&wild, 190, &cfg, &fx, Tracking::default());
    let hostile = RxOfdmConfig {
        soft_max: -5,
        clip: 0.0,
        ..cfg
    };
    let _ = decode_ppdu_fixed(&z, 190, &hostile, &fx, Tracking::default());
    let hostile = RxOfdmConfig {
        soft_max: i32::MAX,
        clip: f64::NAN,
        ..cfg
    };
    let _ = decode_ppdu_fixed(&z, 190, &hostile, &fx, Tracking::default());
}

#[test]
fn the_window_follows_a_clock_offset() {
    // 1000 octets at 6 Mb/s are 27 000 samples: at ±40 ppm the symbols drift by more than a
    // sample, the tracker's slope is removed by the rotator and the window moves once. Started
    // from the clock offset and from nothing, both signs.
    use radio_phy::channel::resample;
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f1b);
    for ppm in [40.0, -40.0] {
        let data = psdu(1000, &mut rng);
        let mut x = frame(&data, Rate::R6);
        x.extend(vec![(0.0, 0.0); 16]);
        let x = resample(&x, ppm);
        let x = at_level(&awgn(&x, 25.0, mean_power(&x), &mut rng), -12.0);
        let z = to_fixed(&x, 2048.0, 12);
        for start in [Tracking::from_clock_offset(ppm), Tracking::default()] {
            let d = decode_ppdu_fixed(&z, 192 - 8, &cfg, &fx, start).unwrap();
            assert_eq!(d.psdu, data, "{ppm} ppm");
            let end = *d.timing.last().unwrap();
            assert!(
                (end - ppm * 27e3 * 1e-6).abs() < 0.2,
                "{ppm} ppm: drift {end}"
            );
        }
        // Without the tracker the frame is lost (the float receiver's finding, `rx_ofdm.md`).
        let off = RxOfdmConfig {
            timing_tracking: false,
            ..cfg
        };
        let d = decode_ppdu_fixed(&z, 192 - 8, &off, &fx, Tracking::default());
        assert!(d.map_or(true, |d| d.psdu != data));
    }
}

#[test]
fn the_window_walks_over_a_drift_longer_than_the_guard_interval() {
    // Ten times the standard's worst case, as the float gate: ±400 ppm over 4095 octets at
    // 6 Mb/s is a drift of 44 samples, almost three guard intervals. Only a window that moves
    // decodes it.
    use radio_phy::channel::resample;
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f1c);
    let data = psdu(4095, &mut rng);
    let mut x = frame(&data, Rate::R6);
    x.extend(vec![(0.0, 0.0); 80]);
    for ppm in [400.0, -400.0] {
        let z = to_fixed(&at_level(&resample(&x, ppm), -12.0), 2048.0, 12);
        let d =
            decode_ppdu_fixed(&z, 192 - 4, &cfg, &fx, Tracking::from_clock_offset(ppm)).unwrap();
        assert_eq!(d.psdu, data, "{ppm} ppm");
        assert!(d.timing.last().unwrap().abs() > 43.0);
    }
}

/// Six-bit soft values with a clip of 2, the working format.
fn six_bits() -> RxOfdmConfig {
    RxOfdmConfig {
        soft_max: 31,
        clip: 2.0,
        ..RxOfdmConfig::default()
    }
}

/// The soft values and common phases of every symbol of `z` from the integer path.
fn all_soft(
    z: &[IQ],
    fft_start: usize,
    rate: Rate,
    cfg: &RxOfdmConfig,
    fx: &FixedOfdmConfig,
    mut tracking: Tracking,
) -> Vec<i64> {
    let ch = estimate_channel_fixed(&z[fft_start..fft_start + 128], fx).unwrap();
    let mut out = Vec::new();
    let mut i = 0;
    loop {
        let m = if i == 0 {
            Modulation::Bpsk
        } else {
            rate.modulation()
        };
        // The window as the tracker has moved it.
        let start = (fft_start + 144 + 80 * i) as i64 + tracking.shift;
        let Some(w) = z.get(start as usize..start as usize + 64) else {
            return out;
        };
        let (soft, phi) = symbol_soft_fixed(w, &ch, i, m, cfg, fx, &mut tracking).unwrap();
        out.extend(soft.iter().map(|&v| i64::from(v)));
        out.push(phi);
        i += 1;
    }
}

#[test]
fn wide_words_with_a_slope_on_unequal_pilots_and_with_the_loops_off() {
    // What the first comparison does not reach: a slope removed while the pilots have
    // unequal gains (the tracker starts with a drift, as after an acquisition that measured
    // a clock offset, through a two-path channel), and the receiver with its timing loop or
    // both loops switched off.
    let fx = wide();
    let mut rng = Rng::new(0x0f1c);
    let drift = Tracking {
        rate: 2.5e-3,
        ..Tracking::default()
    };
    for (pilot_tracking, timing_tracking, start) in [
        (true, true, drift),
        (true, false, Tracking::default()),
        (false, false, Tracking::default()),
    ] {
        let cfg = RxOfdmConfig {
            pilot_tracking,
            timing_tracking,
            ..six_bits()
        };
        for rate in [Rate::R9, Rate::R24, Rate::R54] {
            let x = frame(&psdu(60, &mut rng), rate);
            let x = two_path(&x, (0.3, -0.5), 3);
            let x = at_level(&awgn(&x, 25.0, mean_power(&x), &mut rng), -12.0);
            let z = to_fixed(&x, f64::from(1u32 << 19), 20);
            let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
            let fft_start = 192 - 4;
            let h = estimate_channel(&y[fft_start..fft_start + 128]).unwrap();
            let ch = estimate_channel_fixed(&z[fft_start..fft_start + 128], &fx).unwrap();
            let (mut tf, mut ti) = (start, start);
            let (mut same, mut total, mut slope) = (0usize, 0usize, 0f64);
            for i in 0..(x.len() - 320) / 80 {
                let m = if i == 0 {
                    Modulation::Bpsk
                } else {
                    rate.modulation()
                };
                assert_eq!(tf.shift, ti.shift, "{rate:?} symbol {i}: the windows part");
                let at = (fft_start + 144 + 80 * i) as i64 + tf.shift;
                let w = at as usize..at as usize + 64;
                let eq = equalise(&y[w.clone()], &h, i, m, &cfg, &mut tf).unwrap();
                let want = soft_bits(&eq, m, &cfg);
                let (got, _) = symbol_soft_fixed(&z[w], &ch, i, m, &cfg, &fx, &mut ti).unwrap();
                slope = slope.max(tf.slope.abs());
                for (g, w) in got.iter().zip(&want) {
                    assert!((g - w).abs() <= 1, "{rate:?} symbol {i}: {g} against {w}");
                    same += usize::from(g == w);
                    total += 1;
                }
            }
            assert!(same * 1000 >= total * 990, "{rate:?}: {same} of {total}");
            // The case is what it says: a slope of a tenth of a sample or more was removed.
            assert_eq!(slope > 0.01, timing_tracking, "{rate:?}: slope {slope}");
        }
    }
}

#[test]
fn samples_outside_the_input_width_saturate() {
    // A stream that overdrives the 12-bit input gives exactly what the same stream clamped
    // to 12 bits gives; and the FFT's own word clamps what does not fit it.
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f1d);
    let x = at_level(&frame(&psdu(40, &mut rng), Rate::R36), -6.0);
    let hot = to_fixed(&x, 2048.0, 16);
    let clamped: Vec<IQ> = hot
        .iter()
        .map(|v| (v.0.clamp(-2048, 2047), v.1.clamp(-2048, 2047)))
        .collect();
    assert!(hot != clamped, "nothing was overdriven");
    let a = all_soft(&hot, 188, Rate::R36, &cfg, &fx, Tracking::default());
    let b = all_soft(&clamped, 188, Rate::R36, &cfg, &fx, Tracking::default());
    assert!(!a.is_empty());
    assert_eq!(a, b);
    let big: Vec<IQ> = (0..64)
        .map(|n| (40_000 - 1_300 * n, 1_250 * n - 40_000))
        .collect();
    let fit: Vec<IQ> = big
        .iter()
        .map(|v| (v.0.clamp(-32768, 32767), v.1.clamp(-32768, 32767)))
        .collect();
    assert_eq!(
        fft64_fixed(&big, &fx).unwrap(),
        fft64_fixed(&fit, &fx).unwrap()
    );
}

#[test]
fn the_common_phase_log_is_the_float_receivers() {
    // A residual carrier offset turns each symbol a little further; the phases the integer
    // receiver reports are the float receiver's.
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f1e);
    let data = psdu(200, &mut rng);
    let x: Vec<Sample> = frame(&data, Rate::R12)
        .iter()
        .enumerate()
        .map(|(n, s)| {
            let (sn, cs) = (2.0e-4 * n as f64).sin_cos();
            (s.0 * cs - s.1 * sn, s.0 * sn + s.1 * cs)
        })
        .collect();
    let z = to_fixed(&at_level(&x, -12.0), 2048.0, 12);
    let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
    let f = decode_ppdu_from(&y, 188, &cfg, Tracking::default()).unwrap();
    let d = decode_ppdu_fixed(&z, 188, &cfg, &fx, Tracking::default()).unwrap();
    assert_eq!(d.psdu, data);
    assert_eq!(d.common_phase.len(), f.common_phase.len());
    assert!(f.common_phase.iter().any(|p| p.abs() > 0.2));
    for (a, b) in d.common_phase.iter().zip(&f.common_phase) {
        // Not closer than the rotator's step: its table is addressed by the phase's top nine
        // bits, truncated, so once the tracker's slope is not zero the pilots at +7 and +21
        // are turned 2π/512 too far and the sum reads π/512 = 0.006 low (`rx_ofdm.md`,
        // 2026-11-20: open, with stage B).
        assert!((a - b).abs() < 8e-3, "{a} against {b}");
    }
}

#[test]
fn a_channel_with_an_impossible_shift_is_refused() {
    // `FixedChannel` is a plain record: a shift no estimate can produce must not reach a
    // shifter (in a debug build that was a panic).
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f1f);
    let z = to_fixed(
        &at_level(&frame(&psdu(40, &mut rng), Rate::R12), -12.0),
        2048.0,
        12,
    );
    let mut ch = estimate_channel_fixed(&z[188..316], &fx).unwrap();
    let mut t = Tracking::default();
    let w = &z[188 + 144..188 + 208];
    assert!(symbol_soft_fixed(w, &ch, 0, Modulation::Bpsk, &cfg, &fx, &mut t).is_some());
    for shift in [64, 200, u32::MAX] {
        ch.shift = shift;
        assert!(symbol_soft_fixed(w, &ch, 0, Modulation::Bpsk, &cfg, &fx, &mut t).is_none());
    }
}

#[test]
fn the_default_words_bit_for_bit() {
    // A pin, not a reference: the soft values and common phases of four frames (every
    // modulation; a two-path channel, noise, 12-bit samples, the tracker started with a
    // drift) with the default words, as one hash each. Which rounding is right is the G0
    // table's business; this keeps every rounding, saturation and constant where the table
    // put it, so that a change of one bit anywhere is a decision and not an accident. (The
    // RTL's G3 will compare whole vectors.)
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f20);
    let drift = Tracking {
        rate: 2.5e-3,
        ..Tracking::default()
    };
    let mut got = Vec::new();
    // The fifth frame's data is four times louder than its training field: products and
    // rotations beyond their words, which must saturate.
    for (rate, loud) in [
        (Rate::R6, false),
        (Rate::R18, false),
        (Rate::R36, false),
        (Rate::R54, false),
        (Rate::R54, true),
    ] {
        let x = frame(&psdu(1000, &mut rng), rate);
        let x = two_path(&x, (0.3, -0.5), 3);
        let x = at_level(&awgn(&x, 25.0, mean_power(&x), &mut rng), -12.0);
        let mut z = to_fixed(&x, 2048.0, 12);
        if loud {
            for (n, v) in z.iter_mut().enumerate() {
                *v = if n < 188 + 128 {
                    (v.0 / 4, v.1 / 4)
                } else {
                    (v.0.clamp(-500, 500), v.1.clamp(-500, 500))
                };
            }
        }
        // The format the reports of stage A were measured at, and the Viterbi decoder's.
        let mut v = all_soft(&z, 188, rate, &cfg, &fx, drift);
        v.extend(all_soft(
            &z,
            188,
            rate,
            &RxOfdmConfig::default(),
            &fx,
            drift,
        ));
        assert!(v.len() > 10_000);
        // FNV-1a, 64 bits, over the values as little-endian words.
        let hash = v
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .fold(0xcbf2_9ce4_8422_2325u64, |h, b| {
                (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
            });
        got.push(hash);
    }
    assert_eq!(
        got,
        [
            12_483_171_598_162_887_096u64,
            1_581_067_360_675_375_941,
            6_175_653_753_861_819_291,
            858_923_681_772_431_541,
            13_600_387_931_793_789_518
        ],
        "{got:?}"
    );
}

#[test]
fn the_window_stays_when_the_timing_loop_is_off() {
    // A tracker state with a slope in it (left by an earlier packet, say) moves no window
    // when the loop is off: the float receiver's rule.
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x0f21);
    let x = at_level(&frame(&psdu(40, &mut rng), Rate::R12), -12.0);
    let z = to_fixed(&x, 2048.0, 12);
    let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
    let h = estimate_channel(&y[188..316]).unwrap();
    let ch = estimate_channel_fixed(&z[188..316], &fx).unwrap();
    let w = 188 + 144..188 + 208;
    for (pilot_tracking, timing_tracking) in [(true, false), (false, true), (false, false)] {
        let cfg = RxOfdmConfig {
            pilot_tracking,
            timing_tracking,
            ..six_bits()
        };
        for slope in [0.3, -0.3] {
            let start = Tracking {
                slope,
                ..Tracking::default()
            };
            let (mut tf, mut ti) = (start, start);
            equalise(&y[w.clone()], &h, 0, Modulation::Bpsk, &cfg, &mut tf).unwrap();
            symbol_soft_fixed(&z[w.clone()], &ch, 0, Modulation::Bpsk, &cfg, &fx, &mut ti).unwrap();
            assert_eq!(tf.shift, 0);
            assert_eq!(ti.shift, 0, "{pilot_tracking} {timing_tracking} {slope}");
        }
    }
}

#[test]
fn the_constants_worked_by_hand() {
    // κ_c = c/(2√E)·2^10 and A = 2√E·52·soft_max/clip·2^4, worked outside the model
    // (√2 = 1.414214, √10 = 3.162278, √42 = 6.480741), rounded to nearest.
    use Modulation::{Bpsk, Qam16, Qam64, Qpsk};
    let fx = FixedOfdmConfig::default();
    // 2/(2·3.162278)·1024 = 323.82; 4/(2·6.480741)·1024 = 316.01; half of it 158.01.
    assert_eq!(threshold_constant(2, Qam16, &fx), 324);
    assert_eq!(threshold_constant(4, Qam64, &fx), 316);
    assert_eq!(threshold_constant(2, Qam64, &fx), 158);
    // Six bits, clip 2: 2·√E·52·15.5·16 = 12896·2√E → 25792, 36475.40, 81561.47, 167151.26.
    let six: Vec<i64> = [Bpsk, Qpsk, Qam16, Qam64]
        .iter()
        .map(|&m| scale_constant(m, &six_bits(), &fx))
        .collect();
    assert_eq!(six, [25792, 36475, 81561, 167151]);
    // Eight bits, clip 3 (the Viterbi decoder's format): 2·√E·52·127/3·16 = 35221.33·2√E
    // → 70442.67, 99620.97, 222759.27, 456520.66.
    let eight: Vec<i64> = [Bpsk, Qpsk, Qam16, Qam64]
        .iter()
        .map(|&m| scale_constant(m, &RxOfdmConfig::default(), &fx))
        .collect();
    assert_eq!(eight, [70443, 99621, 222759, 456521]);
    // No scale without a usable clip, soft range or widths.
    for (soft_max, clip) in [(31, 0.0), (31, -1.0), (31, f64::NAN), (-5, 2.0)] {
        let cfg = RxOfdmConfig {
            soft_max,
            clip,
            ..six_bits()
        };
        assert_eq!(scale_constant(Qpsk, &cfg, &fx), 0);
    }
    let bad = FixedOfdmConfig { in_bits: 0, ..fx };
    assert_eq!(scale_constant(Qpsk, &six_bits(), &bad), 0);
    assert_eq!(threshold_constant(2, Qam16, &bad), 0);
}
