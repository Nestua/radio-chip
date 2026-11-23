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
use radio_phy::rx_front_fixed::{FixedRxConfig, IQ, Y_BITS};
use radio_phy::rx_ofdm::{
    decode_ppdu, decode_ppdu_from, equalise, estimate_channel, soft_bits, RxOfdmConfig,
    RxOfdmError, Tracking,
};
use radio_phy::rx_ofdm_fixed::{
    clock_ratio, decode_ppdu_fixed, decode_signal_fixed, estimate_channel_fixed, fft64_fixed,
    scale_constant, symbol_soft_fixed, threshold_constant, to_fixed, tracker_constants,
    FixedOfdmConfig, FixedTracking, TrackerConstants,
};

/// The integer tracker with nothing known about the clock.
fn unknown(fx: &FixedOfdmConfig) -> FixedTracking {
    FixedTracking::unknown(fx)
}

/// A drift of 2.5·10⁻³ rad per subcarrier per symbol as a clock offset in ppm (a sample of
/// timing is 2π/64 rad per subcarrier, a symbol 80 samples): 318.3 ppm.
const DRIFT_RATE: f64 = 2.5e-3;
const DRIFT_PPM: f64 = DRIFT_RATE / (2.0 * std::f64::consts::PI / 64.0 * 80e-6);

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
        rate_frac: 20,
        track_bits: 30,
        coef_frac: 24,
        var_frac: 16,
        rot_round: false,
        // As the float receiver decodes: the whole block.
        traceback: 0,
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
        let (mut tf, mut ti) = (Tracking::default(), unknown(&fx));
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
                let d = decode_ppdu_fixed(&z, 192 - 6, &cfg, &fx, unknown(&fx))
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
fn signal_alone_is_the_frames_signal_from_its_first_208_samples() {
    // What the front's hold is told (ADR 0008, decision 2): SIGNAL decoded from the two
    // long symbols and its own window, before any data symbol is there. The reference is
    // the transmitted field; the whole-frame decoder must report the same.
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f1b);
    for rate in Rate::ALL {
        let len = 1 + (rng.next_u64() % 4095) as usize;
        let data = psdu(len, &mut rng);
        let x = frame(&data, rate);
        let x = at_level(&awgn(&x, 30.0, mean_power(&x), &mut rng), -12.0);
        let z = to_fixed(&x, 2048.0, 12);
        let t1 = 192 - 6;
        // The window of SIGNAL ends 128 + 16 + 64 samples after T1's begins.
        let s = decode_signal_fixed(&z[..t1 + 208], t1, &cfg, &fx, unknown(&fx)).unwrap();
        assert_eq!((s.rate, s.length), (rate, len), "{rate:?}");
        let d = decode_ppdu_fixed(&z, t1, &cfg, &fx, unknown(&fx)).unwrap();
        assert_eq!(d.signal, s);
        // The caller's tracker places SIGNAL's window: moved one sample late it needs one
        // sample more.
        let mut late = unknown(&fx);
        late.shift = 1;
        assert!(matches!(
            decode_signal_fixed(&z[..t1 + 208], t1, &cfg, &fx, late),
            Err(RxOfdmError::Truncated { .. })
        ));
        assert!(decode_signal_fixed(&z[..t1 + 209], t1, &cfg, &fx, late).is_ok());
        // One sample short there is no SIGNAL yet.
        assert!(matches!(
            decode_signal_fixed(&z[..t1 + 207], t1, &cfg, &fx, unknown(&fx)),
            Err(RxOfdmError::Truncated { .. })
        ));
    }
    // The errors are the whole decoder's: noise in place of the frame, a stream without
    // the long symbols, unusable widths.
    let noise = to_fixed(
        &awgn(&vec![(0.0, 0.0); 600], 0.0, 0.01, &mut rng),
        2048.0,
        12,
    );
    let mut failed = 0;
    for t1 in 0..300 {
        let a = decode_signal_fixed(&noise, t1, &cfg, &fx, unknown(&fx));
        let b = decode_ppdu_fixed(&noise, t1, &cfg, &fx, unknown(&fx));
        match (a, b) {
            (Err(a), Err(b)) => {
                assert_eq!(a, b, "{t1}");
                failed += 1;
            }
            // A SIGNAL of noise that passes its checks: the whole decoder then fails
            // further on (the stream is too short for the length, or the data do not
            // descramble).
            (Ok(_), Err(b)) => assert!(!matches!(
                b,
                RxOfdmError::SignalParity | RxOfdmError::SignalRate(_) | RxOfdmError::SignalLength
            )),
            (a, b) => panic!("{t1}: {a:?} {b:?}"),
        }
    }
    assert!(failed > 150 && failed < 300, "{failed}");
    assert!(decode_signal_fixed(&noise[..100], 0, &cfg, &fx, unknown(&fx)).is_err());
    let bad = FixedOfdmConfig { in_bits: 0, ..fx };
    assert!(decode_signal_fixed(&noise, 0, &cfg, &bad, unknown(&fx)).is_err());
}

#[test]
fn the_timing_log_starts_at_signal() {
    // A tracker that applies a known clock offset and learns nothing reports the drift it
    // took out of each symbol: ε·(112 + 80·i) samples for symbol i, SIGNAL being symbol 0
    // (its window's middle is 112 samples after the long symbols' middle, each further one
    // 80 on).
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f1c);
    let data = psdu(60, &mut rng);
    let z = to_fixed(&at_level(&frame(&data, Rate::R12), -12.0), 2048.0, 12);
    let d = decode_ppdu_fixed(&z, 190, &cfg, &fx, FixedTracking::fixed(40.0, &fx)).unwrap();
    assert_eq!(d.psdu, data);
    for (i, t) in d.timing.iter().enumerate().take(4) {
        let by_hand = 40e-6 * (112 + 80 * i) as f64;
        assert!(
            (t - by_hand).abs() < 2e-5,
            "symbol {i}: {t} against {by_hand}"
        );
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
        let d = decode_ppdu_fixed(&z, 190, &cfg, &fx, unknown(&fx)).unwrap();
        assert_eq!(d.psdu, data, "{dbfs} dBFS");
    }
    // Nothing panics: silence, a truncated stream, a start beyond it, unusable widths,
    // samples outside the stated width (they saturate).
    let z = to_fixed(&at_level(&x, -12.0), 2048.0, 12);
    assert!(decode_ppdu_fixed(&vec![(0, 0); 2000], 190, &cfg, &fx, unknown(&fx)).is_err());
    assert!(decode_ppdu_fixed(&z[..600], 190, &cfg, &fx, unknown(&fx)).is_err());
    assert!(decode_ppdu_fixed(&z, usize::MAX - 3, &cfg, &fx, unknown(&fx)).is_err());
    assert!(decode_ppdu_fixed(&[], 0, &cfg, &fx, unknown(&fx)).is_err());
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
        assert!(decode_ppdu_fixed(&z, 190, &cfg, &bad, unknown(&bad)).is_err());
        assert!(estimate_channel_fixed(&z[190..318], &bad).is_none());
    }
    let wild: Vec<IQ> = z
        .iter()
        .map(|v| (v.0 * 1_000_000, v.1 * 1_000_000))
        .collect();
    let _ = decode_ppdu_fixed(&wild, 190, &cfg, &fx, unknown(&fx));
    let hostile = RxOfdmConfig {
        soft_max: -5,
        clip: 0.0,
        ..cfg
    };
    let _ = decode_ppdu_fixed(&z, 190, &hostile, &fx, unknown(&fx));
    let hostile = RxOfdmConfig {
        soft_max: i32::MAX,
        clip: f64::NAN,
        ..cfg
    };
    let _ = decode_ppdu_fixed(&z, 190, &hostile, &fx, unknown(&fx));
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
        for start in [FixedTracking::from_clock_offset(ppm, &fx), unknown(&fx)] {
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
        let d = decode_ppdu_fixed(&z, 192 - 8, &off, &fx, unknown(&fx));
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
        let d = decode_ppdu_fixed(
            &z,
            192 - 4,
            &cfg,
            &fx,
            FixedTracking::from_clock_offset(ppm, &fx),
        )
        .unwrap();
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
    mut tracking: FixedTracking,
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
        rate: DRIFT_RATE,
        ..Tracking::default()
    };
    for (pilot_tracking, timing_tracking, start, start_fixed) in [
        (
            true,
            true,
            drift,
            FixedTracking::start(DRIFT_PPM, 40.0, &fx),
        ),
        (true, false, Tracking::default(), unknown(&fx)),
        (false, false, Tracking::default(), unknown(&fx)),
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
            let (mut tf, mut ti) = (start, start_fixed);
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
    // A stream that overdrives the 13-bit input gives exactly what the same stream clamped
    // to 13 bits gives; and the FFT's own word clamps what does not fit it.
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f1d);
    let x = at_level(&frame(&psdu(40, &mut rng), Rate::R36), -6.0);
    let hot = to_fixed(&x, 4096.0, 16);
    let clamped: Vec<IQ> = hot
        .iter()
        .map(|v| (v.0.clamp(-4096, 4095), v.1.clamp(-4096, 4095)))
        .collect();
    assert!(hot != clamped, "nothing was overdriven");
    let a = all_soft(&hot, 188, Rate::R36, &cfg, &fx, unknown(&fx));
    let b = all_soft(&clamped, 188, Rate::R36, &cfg, &fx, unknown(&fx));
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
    let d = decode_ppdu_fixed(&z, 188, &cfg, &fx, unknown(&fx)).unwrap();
    assert_eq!(d.psdu, data);
    assert_eq!(d.common_phase.len(), f.common_phase.len());
    assert!(f.common_phase.iter().any(|p| p.abs() > 0.2));
    let mut worst = 0f64;
    for (a, b) in d.common_phase.iter().zip(&f.common_phase) {
        // The rotator's phase is rounded to its table's address (half an address added):
        // truncated, the pilots at +7 and +21 are turned 2π/512 too far once the slope is
        // not zero and the sum reads π/512 = 0.006 low.
        worst = worst.max((a - b).abs());
        assert!((a - b).abs() < 1.5e-3, "{a} against {b}");
    }
    println!("largest difference of the common phase: {worst:.2e} rad");
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
    let mut t = unknown(&fx);
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
    // The tracker's prior (318 ppm) is wrong by more than its registers hold over the first
    // frame's 335 symbols (8.6 samples against 8): that frame pins their saturation too.
    let fx = FixedOfdmConfig::default();
    let cfg = six_bits();
    let mut rng = Rng::new(0x0f20);
    let drift = FixedTracking::start(DRIFT_PPM, 40.0, &fx);
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
        // Since ADR 0008 with the input at 13 bits and one fractional bit (the earlier values are in
        // `docs/gates/rx_ofdm.md`, 2026-11-21).
        [
            5_260_962_740_838_614_106u64,
            7_941_093_826_470_765_633,
            3_171_262_636_499_517_186,
            4_732_824_824_092_512_215,
            7_729_731_060_040_634_013
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
            let mut ti = unknown(&fx);
            // The same slope in the integer tracker's units, 2π/2^22 per subcarrier.
            ti.theta = (slope / (2.0 * std::f64::consts::PI) * 4_194_304.0) as i64;
            let mut tf = start;
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

/// A channel record with the given gains on the pilots at −21, −7, 7, 21 and a gain sum.
fn pilots_of(g: [i64; 4], gain_sum: i64) -> radio_phy::rx_ofdm_fixed::FixedChannel {
    let mut gain = [0i64; 64];
    // FFT bins of −21, −7, 7, 21.
    for (bin, v) in [43usize, 57, 7, 21].into_iter().zip(g) {
        gain[bin] = v;
    }
    radio_phy::rx_ofdm_fixed::FixedChannel {
        hs: [(0, 0); 64],
        shift: 0,
        gain,
        gain_sum,
    }
}

#[test]
fn the_tracker_constants_worked_by_hand() {
    // a = g₀ + g₃, b = g₁ + g₂, N_o = g₀g₃b, N_i = g₁g₂a, D = 9N_o + N_i, cut to 16 bits;
    // outer = 3N_o/(14D), inner = N_i/(14D) in 16 fractional bits, variance = 5Σg·ab/(52D)
    // in 10.
    let fx = FixedOfdmConfig::default();
    let c = |g, sum| tracker_constants(&pilots_of(g, sum), &fx);
    // Equal gains: D = 20g³, so 3/140 and 1/140 (1404.3 and 468.1) and a variance of 1.
    assert_eq!(
        c([1000; 4], 52_000),
        TrackerConstants {
            outer: 1404,
            inner: 468,
            variance: 1024
        }
    );
    // Inner pilots twice as strong, mean gain 1500: N_o = 4·10⁹, N_i = 8·10⁹, D = 44·10⁹
    // (36 bits: shift 20 → 3814, 7629, 41961). 3·3814/(14·41961) = 1276.4/2¹⁶,
    // 7629/(14·41961) = 851.1/2¹⁶; variance 5·1500·8·10⁶/(44·10⁹) = 1.3636 → 1396.
    assert_eq!(
        c([1000, 2000, 2000, 1000], 78_000),
        TrackerConstants {
            outer: 1276,
            inner: 851,
            variance: 1396
        }
    );
    // Small numbers, no shift: N_o = 3·2·12 = 72, N_i = 5·7·5 = 175, D = 823:
    // 216/11522 → 1228.6, 175/11522 → 995.4, 5·208·60/(52·823) = 1.458 → 1493.
    assert_eq!(
        c([3, 5, 7, 2], 208),
        TrackerConstants {
            outer: 1229,
            inner: 995,
            variance: 1493
        }
    );
    // One outer pilot gone: only the inner pair measures (1/14 → 4681), and with a variance
    // of 7.5 nominal ones the measurement is not used (the limit is 3).
    let weak = c([0, 2000, 2000, 1000], 78_000);
    assert_eq!((weak.outer, weak.inner, weak.variance), (0, 4681, 7680));
    // No pair at all, gains no estimate produces, unusable widths: nothing.
    let none = TrackerConstants::default();
    assert_eq!(c([0, 0, 5, 5], 100), none);
    assert_eq!(c([i64::MAX; 4], i64::MAX), none);
    assert_eq!(c([-5, -5, -5, -5], -20), none);
    let bad = FixedOfdmConfig {
        track_bits: 2,
        ..fx
    };
    assert_eq!(tracker_constants(&pilots_of([1000; 4], 52_000), &bad), none);
}

#[test]
fn the_fit_worked_by_hand() {
    // The prior of 40 ppm weighs 25·(0.016/(2π/64·80·40·10⁻⁶))² = 64 845.6 → 64 846 (times a
    // variance of 1). Times τ = 7, 12, 17.
    let fx = FixedOfdmConfig::default();
    let mut t = FixedTracking::unknown(&fx);
    assert_eq!((t.rate0, t.weight), (0, 64_846));
    let lambda = 64_846;
    // One measurement moves the offset alone: det = 49 + λ − 49, numerator 0.
    t.learn(-777, 0, lambda, &fx);
    assert_eq!((t.delta, t.measured), (0, -777));
    let mut t = FixedTracking::unknown(&fx);
    t.learn(0, 0, lambda, &fx);
    // Second: n = 2, Στ = 19, Στ² = 193, Σz = e, Στz = 12e; det = 2·(193 + λ) − 361
    // = 129 717, numerator 24e − 19e = 5e. With e = 129 717 the rate is 5 units per τ:
    // δ = 5·2¹⁰.
    t.learn(129_717, 1, lambda, &fx);
    assert_eq!(t.delta, 5120);
    // Third, measured where predicted (error 0): z = 5·17 = 85; n = 3, Στ = 36, Στ² = 482,
    // Σz = 129 802, Στz = 1 558 049; det = 3·65 328 − 1296 = 194 688, numerator
    // 4 674 147 − 4 672 872 = 1275; δ = 1275·1024/194 688 = 6.7 → 6 (toward zero).
    t.learn(0, 2, lambda, &fx);
    assert_eq!(t.delta, 6);
    // Toward zero for a negative quotient too: the mirror image.
    let mut t = FixedTracking::unknown(&fx);
    t.learn(0, 0, lambda, &fx);
    t.learn(-129_717, 1, lambda, &fx);
    assert_eq!(t.delta, -5120);
    t.learn(0, 2, lambda, &fx);
    assert_eq!(t.delta, -6);
    // The starts: 40 ppm is 40·16·10⁻⁶ samples per τ, 2¹⁶ units a sample, 2¹⁰ fractional:
    // 42 949.67 → 42 950. The hint's 2 ppm weighs 400 times the 40 ppm prior.
    let hint = FixedTracking::from_clock_offset(40.0, &fx);
    assert_eq!((hint.rate0, hint.weight), (42_950, 25_938_223));
    let fixed = FixedTracking::fixed(-40.0, &fx);
    assert_eq!(fixed.rate0, -42_950);
    assert!(fixed.weight < 0);
    // A tracker that learns nothing keeps δ; so does one fed a negative weight.
    let mut f = fixed;
    f.learn(50_000, 0, lambda, &fx);
    f.learn(90_000, 1, lambda, &fx);
    assert_eq!(f.delta, 0);
    let mut t = FixedTracking::unknown(&fx);
    t.learn(1, 0, -1, &fx);
    t.learn(90_000, 1, -1, &fx);
    assert_eq!(t.delta, 0);
    // Not numbers: a rate of 0 and nothing learned. A symbol index beyond any frame: no
    // overflow, δ kept.
    for (ppm, std) in [(f64::NAN, 40.0), (f64::INFINITY, 2.0), (10.0, f64::NAN)] {
        let s = FixedTracking::start(ppm, std, &fx);
        assert!(s.weight < 0, "{ppm} {std}");
        assert!(ppm.is_finite() || s.rate0 == 0);
    }
    assert_eq!(FixedTracking::start(1e300, 40.0, &fx).rate0, 1 << 26);
    let mut t = FixedTracking::unknown(&fx);
    t.learn(i64::MAX, usize::MAX, i128::MAX, &fx);
    t.learn(i64::MIN, usize::MAX - 1, 5, &fx);
    assert_eq!(t.delta, 0);
}

#[test]
fn the_fit_is_the_recursive_least_squares_of_the_float_tracker() {
    // The batch quotient against the recursion the float tracker runs (written out here in
    // the textbook form: gain K = P·φ/(φᵀPφ + v), φ = (1, τ)), on measurements with a drift,
    // an offset and noise, for the two priors and three variances. The measurement fed to
    // the integer fit is what is left after its own prediction, as in the receiver.
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x0f30);
    for (weight, var_q) in [
        (64_846i64, 1024i64),
        (64_846, 400),
        (64_846, 3000),
        (25_938_223, 1024),
    ] {
        let lambda = (i128::from(weight) * i128::from(var_q) + 512) >> 10;
        let v = var_q as f64 / 1024.0;
        let mut t = FixedTracking::unknown(&fx);
        t.weight = weight;
        // Float recursion in the same units: state (offset, rate per τ), covariance in units
        // of the nominal variance; prior on the rate 1/weight, flat on the offset.
        let (mut ob, mut rb) = (0f64, 0f64);
        let (mut pbb, mut pbr, mut prr) = (1e12f64, 0f64, 1.0 / weight as f64);
        let (offset, rate) = (9000.0, 37.3);
        for i in 0..1367usize {
            let tau = 7.0 + 5.0 * i as f64;
            let noise = 10_000.0 * (rng.next_u64() as f64 / u64::MAX as f64 - 0.5);
            let z = (offset + rate * tau + noise).round();
            // Float.
            let (gb, gr) = (pbb + pbr * tau, pbr + prr * tau);
            let s = gb + gr * tau + v;
            let (kb, kr) = (gb / s, gr / s);
            let innovation = z - (ob + rb * tau);
            ob += kb * innovation;
            rb += kr * innovation;
            (pbb, pbr, prr) = (pbb - kb * gb, pbr - kb * gr, prr - kr * gr);
            // Integer: the error against its own prediction δ·τ (half to even, as the model).
            let exact = i128::from(t.delta) * (7 + 5 * i as i128);
            let mut predicted = exact >> 10;
            let rem = exact - (predicted << 10);
            if rem > 512 || (rem == 512 && predicted & 1 == 1) {
                predicted += 1;
            }
            t.learn((z as i128 - predicted) as i64, i, lambda, &fx);
            let want = rb * 1024.0;
            // One unit of the quotient's truncation, and the float recursion's own rounding.
            assert!(
                (t.delta as f64 - want).abs() <= 1.0 + want.abs() * 1e-6,
                "weight {weight} variance {v} symbol {i}: {} against {want}",
                t.delta
            );
        }
        // The case is what it says: the rate is found (37.3 units per τ; the noise leaves
        // about 0.04 of scatter, and 1367 symbols outweigh either prior).
        let found = t.delta as f64 / 1024.0;
        assert!((found - rate).abs() < 0.5, "weight {weight}: {found}");
    }
}

#[test]
fn the_tracker_in_the_receiver_is_the_float_one() {
    // The whole tracker in the receiver: the same 12-bit samples through the float receiver
    // and the integer one symbol by symbol, near the sensitivity limit and at a high SNR,
    // through a two-path channel, with a clock offset of either sign, started from nothing,
    // from the hint and fixed. Each symbol's measured slope and the drift applied agree.
    //
    // One event parts the two and is not an error of either: a pilot pair whose angle falls
    // on ±π (a pilot the noise has all but cancelled) reads +π in one receiver and −π in the
    // other when their predictions differ in the last digits. The measurement then differs by
    // a turn times the pair's coefficient. Such a symbol ends the frame's comparison and is
    // counted. With two fractional bits in the FFT word this run had one (6 Mb/s, 6 dB,
    // −40 ppm, symbol 38); with ADR 0008's one it has none, and that branch is not taken.
    use radio_phy::channel::resample;
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f31);
    let unit = 2.0 * std::f64::consts::PI / 4_194_304.0;
    let (mut worst_slope, mut worst_drift, mut compared, mut wraps) = (0f64, 0f64, 0usize, 0usize);
    for (rate, snr, ppm) in [
        (Rate::R6, 6.0, 40.0),
        (Rate::R6, 6.0, -40.0),
        (Rate::R54, 28.0, 33.0),
        (Rate::R24, 18.0, -17.0),
        (Rate::R12, 9.0, 0.0),
    ] {
        let data = psdu(800, &mut rng);
        let mut x = frame(&data, rate);
        let symbols = (x.len() - 320) / 80;
        x.extend(vec![(0.0, 0.0); 16]);
        let x = two_path(&resample(&x, ppm), (0.25, 0.3), 2);
        let x = at_level(&awgn(&x, snr, mean_power(&x), &mut rng), -12.0);
        let z = to_fixed(&x, 2048.0, 12);
        let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
        let fft_start = 192 - 6;
        let h = estimate_channel(&y[fft_start..fft_start + 128]).unwrap();
        let ch = estimate_channel_fixed(&z[fft_start..fft_start + 128], &fx).unwrap();
        let c = tracker_constants(&ch, &fx);
        let turns = [c.outer, c.inner].map(|k| k as f64 / 65_536.0 * 2.0 * std::f64::consts::PI);
        for (float, fixed) in [
            (Tracking::default(), FixedTracking::unknown(&fx)),
            (
                Tracking::from_clock_offset(ppm + 1.5),
                FixedTracking::from_clock_offset(ppm + 1.5, &fx),
            ),
            (Tracking::fixed(ppm), FixedTracking::fixed(ppm, &fx)),
        ] {
            let (mut tf, mut ti) = (float, fixed);
            for i in 0..symbols {
                let m = if i == 0 {
                    Modulation::Bpsk
                } else {
                    rate.modulation()
                };
                assert_eq!(
                    tf.shift, ti.shift,
                    "{rate:?} {ppm} symbol {i}: the windows part"
                );
                let at = (fft_start + 144 + 80 * i) as i64 + tf.shift;
                let w = at as usize..at as usize + 64;
                equalise(&y[w.clone()], &h, i, m, &cfg, &mut tf).unwrap();
                symbol_soft_fixed(&z[w], &ch, i, m, &cfg, &fx, &mut ti).unwrap();
                let slope = (ti.measured as f64 * unit - tf.measured).abs();
                if turns.iter().any(|t| (slope - t).abs() < 1e-3) {
                    wraps += 1;
                    break;
                }
                let drift = (ti.timing(&fx) - tf.timing()).abs();
                // One fractional bit in the FFT word (ADR 0008's hand-off): the largest
                // difference is 4.0e-4, in one symbol of a 6 Mb/s frame at 6 dB, and at
                // most 1.6e-4 in every other (9.6e-5 with two fractional bits).
                assert!(
                    slope < 5e-4,
                    "{rate:?} {ppm} ppm symbol {i}: measured {} against {}",
                    ti.measured as f64 * unit,
                    tf.measured
                );
                assert!(
                    drift < 0.005,
                    "{rate:?} {ppm} ppm symbol {i}: {} against {} samples",
                    ti.timing(&fx),
                    tf.timing()
                );
                (worst_slope, worst_drift) = (worst_slope.max(slope), worst_drift.max(drift));
                compared += 1;
            }
            // The case is what it says: the drift is there to follow.
            // (A start from nothing learns little in the 31 symbols of the 54 Mb/s frame.)
            assert!(
                ppm == 0.0 || ti.weight == 64_846 || tf.timing().abs() > 0.03,
                "{rate:?} {ppm}"
            );
        }
    }
    assert!(wraps <= 1, "{wraps} frames ended by a pair at ±π");
    assert!(worst_slope > 5e-5, "{worst_slope}");
    assert!(compared > 2000, "{compared}");
    println!(
        "{compared} symbols, {wraps} ended by a pair at ±π; largest difference: measured slope \
         {worst_slope:.2e} rad per subcarrier, drift {worst_drift:.5} samples"
    );
}

#[test]
fn a_weak_pilot_pair_is_not_followed() {
    // A two-path channel with its notch on the pilot at −21: x[n] − 0.9·e^(−j2π·21/64)·x[n−1]
    // leaves that pilot a gain of 0.01 against 1.46, 3.47 and 2.81 on the other three (mean
    // 1.8), so the outer pair weighs 0.01 and the measurement's variance is 5·1.8/1.12 ≈ 8
    // nominal ones: beyond the limit of 3, in both receivers. The slope is still measured and
    // reported, and neither tracker learns from it, although the clock is 40 ppm off.
    use radio_phy::channel::resample;
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let mut rng = Rng::new(0x0f32);
    let x = resample(&frame(&psdu(300, &mut rng), Rate::R6), 40.0);
    let phase = -2.0 * std::f64::consts::PI * 21.0 / 64.0;
    let x = two_path(&x, (-0.9 * phase.cos(), -0.9 * phase.sin()), 1);
    let x = at_level(&awgn(&x, 25.0, mean_power(&x), &mut rng), -12.0);
    let z = to_fixed(&x, 2048.0, 12);
    let y: Vec<Sample> = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
    let fft_start = 192 - 4;
    let h = estimate_channel(&y[fft_start..fft_start + 128]).unwrap();
    let ch = estimate_channel_fixed(&z[fft_start..fft_start + 128], &fx).unwrap();
    let c = tracker_constants(&ch, &fx);
    assert!(
        (4 * 1024..16 * 1024).contains(&c.variance),
        "variance {} / 1024",
        c.variance
    );
    let (mut tf, mut ti) = (Tracking::default(), FixedTracking::unknown(&fx));
    let unit = 2.0 * std::f64::consts::PI / 4_194_304.0;
    let mut largest = 0f64;
    for i in 0..60 {
        let w = fft_start + 144 + 80 * i..fft_start + 208 + 80 * i;
        equalise(&y[w.clone()], &h, i, Modulation::Bpsk, &cfg, &mut tf).unwrap();
        symbol_soft_fixed(&z[w], &ch, i, Modulation::Bpsk, &cfg, &fx, &mut ti).unwrap();
        assert_eq!((tf.rate, ti.delta, ti.theta), (0.0, 0, 0), "symbol {i}");
        let (a, b) = (ti.measured as f64 * unit, tf.measured);
        assert!((a - b).abs() < 2e-4, "symbol {i}: measured {a} against {b}");
        largest = largest.max(b.abs());
    }
    // The case is what it says: there was a slope to follow (40 ppm over 60 symbols is 0.19
    // samples, 0.019 rad per subcarrier).
    assert!(largest > 0.01, "{largest}");
}

#[test]
fn the_tracker_constants_at_their_edges() {
    let fx = FixedOfdmConfig::default();
    let c = |g, sum| tracker_constants(&pilots_of(g, sum), &fx);
    // A pair with both pilots gone has no weight and the other measures alone, as in the
    // float tracker. Inner pair gone, flat otherwise: D = 9·10⁶ (24 bits: shift 8 → 35 156,
    // N_o 3906); 3·3906/(14·35 156) = 1560.3/2¹⁶ (1/42), variance
    // 5·52 000·2000/(52·9·10⁶) = 1.111 → 1137. Outer pair gone: 1/14 → 4681, variance
    // 5·20 800·1600/(52·630 000) = 5.08 → 5201.
    assert_eq!(
        c([1000, 0, 0, 1000], 52_000),
        TrackerConstants {
            outer: 1560,
            inner: 0,
            variance: 1137
        }
    );
    assert_eq!(
        c([0, 700, 900, 0], 20_800),
        TrackerConstants {
            outer: 0,
            inner: 4681,
            variance: 5201
        }
    );
    // Roundings that the round cases above do not separate. a = 13 237, b = 6238,
    // N_o = 50 962 039 656, N_i = 13 264 069 665, D = 471 922 426 569 (39 bits: shift 23 →
    // 56 257, 6075, 1581): 3·6075/(14·56 257) = 1516.502/2¹⁶ → 1517 (half up; a shift of
    // 24 gives 1516), 1581/(14·56 257) = 131.55 → 132 (a shift of 24 gives 131); variance
    // 5·4 494 101/(52·56 257) = 7.681 → 7865.
    assert_eq!(
        c([12_588, 165, 6073, 649], 456_560),
        TrackerConstants {
            outer: 1517,
            inner: 132,
            variance: 7865
        }
    );
    // Used up to 3 nominal variances, 3 included; not at 0.
    let with = |variance| TrackerConstants {
        outer: 1,
        inner: 1,
        variance,
    };
    assert!(with(1).usable(&fx));
    assert!(with(3072).usable(&fx));
    assert!(!with(3073).usable(&fx));
    assert!(!with(0).usable(&fx));
    assert!(!with(-5).usable(&fx));
    // λ = weight·variance/2¹⁰, half to even: 64 846·1396/1024 = 88 403.3; ties 0.5 → 0,
    // 1.5 → 2, 2.5 → 2. None for a tracker that learns nothing.
    let mut t = FixedTracking::unknown(&fx);
    assert_eq!(t.lambda(&with(1396), &fx), 88_403);
    t.weight = 1;
    for (variance, want) in [(512, 0), (1536, 2), (2560, 2), (1537, 2), (2561, 3)] {
        assert_eq!(t.lambda(&with(variance), &fx), want, "{variance}");
    }
    assert!(FixedTracking::fixed(3.0, &fx).lambda(&with(1024), &fx) < 0);
}

#[test]
fn the_tracker_registers_saturate_and_refuse() {
    let fx = FixedOfdmConfig::default();
    // Unusable widths, a symbol beyond any frame: nothing learned, no panic.
    for bad in [
        FixedOfdmConfig {
            rate_frac: 128,
            ..fx
        },
        FixedOfdmConfig {
            rate_frac: 200,
            ..fx
        },
        FixedOfdmConfig { var_frac: 99, ..fx },
    ] {
        let mut t = FixedTracking::unknown(&fx);
        t.learn(5, 0, 64_846, &bad);
        t.learn(90_000, 1, 64_846, &bad);
        assert_eq!(t.delta, 0);
        let c = TrackerConstants {
            outer: 1,
            inner: 1,
            variance: 1024,
        };
        assert!(!c.usable(&bad) || bad.var_frac <= 16);
        let _ = t.lambda(&c, &bad);
    }
    let mut t = FixedTracking::unknown(&fx);
    t.learn(0, 0, 0, &fx);
    t.learn(1000, 1367, 0, &fx);
    assert_eq!(t.delta, 0);
    // z saturates at 20 bits: with no prior (λ = 0) two measurements give the line through
    // them, δ = 2¹⁰·(z₁ − z₀)/5. An error of 600 000 counts as 524 287: δ = 107 374 029.9…
    // → 107 373 977 (1024·524 287/5 = 107 373 977.6, toward zero).
    let mut t = FixedTracking::unknown(&fx);
    t.learn(0, 0, 0, &fx);
    t.learn(600_000, 1, 0, &fx);
    assert_eq!(t.delta, 107_373_977);
    let mut t = FixedTracking::unknown(&fx);
    t.learn(0, 0, 0, &fx);
    t.learn(-600_000, 1, 0, &fx);
    // −524 288·1024/5 = −107 374 182.4 → −107 374 182.
    assert_eq!(t.delta, -107_374_182);
    // δ saturates at 28 bits: from −2¹⁹ to 2¹⁹ − 1 in one step of 5 is
    // 1024·1 048 575/5 = 214 748 160 → 2²⁷ − 1, and the mirror image −2²⁷.
    let mut t = FixedTracking::unknown(&fx);
    t.learn(-600_000, 0, 0, &fx);
    t.learn(600_000, 1, 0, &fx);
    assert_eq!(t.delta, (1 << 27) - 1);
    let mut t = FixedTracking::unknown(&fx);
    t.learn(600_000, 0, 0, &fx);
    t.learn(-600_000, 1, 0, &fx);
    assert_eq!(t.delta, -(1 << 27));
    // And stays in its 28 bits whatever is fed, for a whole frame and back.
    let mut t = FixedTracking::unknown(&fx);
    for i in 0..1367 {
        t.learn(i64::MAX, i, 0, &fx);
        assert!(t.delta.abs() < 1 << 27, "symbol {i}: {}", t.delta);
    }
    for i in 0..1367 {
        t.learn(i64::MIN, 1366 - i, 0, &fx);
        assert!(t.delta.abs() <= 1 << 27, "symbol {i}: {}", t.delta);
    }
}

#[test]
fn the_window_moves_beyond_half_a_sample_only() {
    // A tracker that applies a fixed rate: at symbol 5 (τ = 32) a rate of 2²⁰/2¹⁰ units per
    // τ is a drift of exactly 2¹⁵, half a sample: the window stays. One unit more and it
    // moves one sample earlier; the mirror image moves it later.
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0x0f33);
    let x = at_level(&frame(&psdu(60, &mut rng), Rate::R12), -12.0);
    let z = to_fixed(&x, 2048.0, 12);
    let ch = estimate_channel_fixed(&z[188..316], &fx).unwrap();
    let w = 188 + 144 + 400..188 + 208 + 400;
    for (rate0, theta, shift) in [
        (1 << 20, 32_768, 0),
        ((1 << 20) + 32, 32_769, -1),
        (-(1 << 20), -32_768, 0),
        (-(1 << 20) - 32, -32_769, 1),
    ] {
        let mut t = FixedTracking::fixed(0.0, &fx);
        t.rate0 = rate0;
        symbol_soft_fixed(
            &z[w.clone()],
            &ch,
            5,
            Modulation::Qpsk,
            &RxOfdmConfig::default(),
            &fx,
            &mut t,
        )
        .unwrap();
        assert_eq!((t.theta, t.shift), (theta, shift), "rate {rate0}");
    }
}

#[test]
fn the_default_input_is_the_hand_off_of_adr_0008() {
    // ADR 0008, decision 1: the decoder is handed the rotator's 13 bits (the ADC's LSB kept)
    // with one fractional bit in the FFT word.
    let fx = FixedOfdmConfig::default();
    assert_eq!((fx.in_bits, fx.in_shift), (Y_BITS, 1));
    assert_eq!(fx.in_bits + fx.in_shift, 14);
    assert!(fx.is_valid());

    // The thirteenth bit is used: a frame whose mean power is the 12-bit full scale squared
    // passes at 54 Mb/s with the default input, and not when the input saturates at 12 bits
    // (a sixth of its components would).
    let mut rng = Rng::new(0x13b1);
    let p = psdu(200, &mut rng);
    let mut x = vec![(0.0, 0.0); 192];
    x.extend(frame(&p, Rate::R54));
    let k = (1.0 / mean_power(&x[192..])).sqrt();
    let z = to_fixed(&x, 2048.0 * k, 13);
    assert!(z.iter().any(|v| v.0.abs() > 2048 || v.1.abs() > 2048));
    let cfg = RxOfdmConfig::default();
    let d = decode_ppdu_fixed(&z, 192 + 160 + 32, &cfg, &fx, unknown(&fx)).unwrap();
    assert_eq!(d.psdu, p);
    let twelve = FixedOfdmConfig { in_bits: 12, ..fx };
    let d = decode_ppdu_fixed(&z, 192 + 160 + 32, &cfg, &twelve, unknown(&twelve));
    assert!(d.map_or(true, |d| d.psdu != p));
}

#[test]
fn the_tracker_starts_from_the_cfo_word_by_hand() {
    // ADR 0008, decision 7. The register: fs/fc with 20 fractional bits in 16.
    // 2^20·20/5800 = 3615.78, 2^20·20/2412 = 8694.66, 2^20·2/903 = 2322.43.
    assert_eq!(clock_ratio(20e6, 5.8e9), Some(3616));
    assert_eq!(clock_ratio(20e6, 2.412e9), Some(8695));
    assert_eq!(clock_ratio(2e6, 903e6), Some(2322));
    // The largest: 65535.5/2^20 rounds to 65536 and is refused, as are no ratio at all and
    // values that are not numbers.
    assert_eq!(clock_ratio(65_535.0, 1_048_576.0), Some(65_535));
    assert_eq!(clock_ratio(65_535.5, 1_048_576.0), None);
    assert_eq!(clock_ratio(0.4, 1_048_576.0), None);
    assert_eq!(clock_ratio(0.5, 1_048_576.0), Some(1));
    assert_eq!(clock_ratio(1.0, 1_048_576.0), Some(1));
    assert_eq!(clock_ratio(160e6, 2.6e9), Some(64_528));
    assert_eq!(clock_ratio(160e6, 2.56e9), None);
    assert_eq!(clock_ratio(20e6, 0.0), None);
    assert_eq!(clock_ratio(-20e6, 5.8e9), None);
    assert_eq!(clock_ratio(f64::NAN, 5.8e9), None);
    assert_eq!(clock_ratio(20e6, f64::INFINITY), None);

    // The rate: a clock offset ε = word·(fs/fc)/2^P drifts 16·ε samples in a fifth of a
    // symbol, 2^(P−6) units of slope each, with R fractional bits: word·ratio/2^(22−R)
    // whatever P is. R = 10: word·ratio/4096, half to even.
    let fx = FixedOfdmConfig::default();
    assert_eq!(fx.rate_frac, 10);
    let rate = |word: i64, ratio: u16| FixedTracking::from_cfo_word(word, ratio, &fx).rate0;
    assert_eq!(rate(4096, 3616), 3616);
    assert_eq!(rate(-4096, 3616), -3616);
    assert_eq!(rate(1, 2048), 0);
    assert_eq!(rate(3, 2048), 2);
    assert_eq!(rate(5, 2048), 2);
    assert_eq!(rate(-1, 2048), 0);
    assert_eq!(rate(-3, 2048), -2);
    assert_eq!(rate(1, 2049), 1);
    // 232 kHz at 20 MHz is the word 48 654 (40 ppm at 5.8 GHz): 48654·3616/4096 = 42952.4,
    // against 40·16·10⁻⁶·2^16·2^10 = 42949.7 from the offset itself.
    assert_eq!(rate(48_654, 3616), 42_952);
    assert_eq!(FixedTracking::from_clock_offset(40.0, &fx).rate0, 42_950);
    // Other R: 2^(22−R) is 4 at R = 20 and 2^22 at R = 0.
    let at = |r: u32, word: i64| {
        let fx = FixedOfdmConfig { rate_frac: r, ..fx };
        FixedTracking::from_cfo_word(word, 3616, &fx).rate0
    };
    assert_eq!(at(20, 3), 2712);
    assert_eq!(at(0, 48_654), 42);
    assert_eq!(at(0, 579), 0);
    assert_eq!(at(0, 580), 1);
    // A word of the accumulator's whole range and the largest ratio stay inside the rate's
    // signed register of P − 6 + R = 26 bits (below 2^25); a word no accumulator holds
    // saturates at its ends, whether or not its product fits 64 bits.
    assert_eq!(rate((1 << 21) - 1, u16::MAX), 33_553_904);
    assert_eq!(rate(-(1 << 21), u16::MAX), -33_553_920);
    assert_eq!(rate(i64::MAX, u16::MAX), (1 << 25) - 1);
    assert_eq!(rate(i64::MIN, u16::MAX), -(1 << 25));
    assert_eq!(rate(1 << 50, u16::MAX), (1 << 25) - 1);
    assert_eq!(rate(-(1 << 50), u16::MAX), -(1 << 25));
    // At P = 30 the register is 34 bits and a word of 30 bits passes whole:
    // (2^29 − 1)·65535/4096 = 8 589 803 504.0002.
    let p30 = FixedOfdmConfig {
        front: FixedRxConfig {
            nco_phase_bits: 30,
            ..fx.front
        },
        ..fx
    };
    assert!(p30.is_valid());
    let rate30 = |word: i64| FixedTracking::from_cfo_word(word, u16::MAX, &p30).rate0;
    assert_eq!(rate30((1 << 29) - 1), 8_589_803_504);
    assert_eq!(rate30(i64::MAX), (1 << 33) - 1);
    assert_eq!(rate30(i64::MIN), -(1 << 33));
    // Widths `is_valid` refuses are not a panic: R is taken as at most 20 and P as 8 to 40.
    let odd = |r: u32, p: u32| {
        let fx = FixedOfdmConfig {
            rate_frac: r,
            front: FixedRxConfig {
                nco_phase_bits: p,
                ..fx.front
            },
            ..fx
        };
        FixedTracking::from_cfo_word(3, 3616, &fx).rate0
    };
    assert_eq!(odd(30, 22), 2712);
    assert_eq!(odd(u32::MAX, 22), 2712);
    assert_eq!(odd(20, 3), 2712);
    assert_eq!(odd(20, 60), 2712);

    // Everything else is the start from a clock offset known to 2 ppm: at a word of 0 the
    // two are one tracker, and the weight is that start's constant:
    // 25·(0.016/(2π/64·80·10⁻⁶·2))² = 25·1018.59² = 25 938 2xx.
    assert_eq!(
        FixedTracking::from_cfo_word(0, 3616, &fx),
        FixedTracking::from_clock_offset(0.0, &fx)
    );
    assert_eq!(
        FixedTracking::from_cfo_word(777, 3616, &fx).weight,
        25_938_223
    );
    assert_eq!(
        FixedTracking::from_clock_offset(1.0, &fx).weight,
        25_938_223
    );
}

#[test]
fn the_start_from_the_cfo_word_is_the_start_from_the_offset() {
    // Against the float conversion the tools used until now: ppm = word·fs/2^P/fc·10⁶, then
    // `from_clock_offset`. The two rates differ by their two roundings (half a unit each)
    // and by the register's rounding of fs/fc (half of 2^−20), which the word multiplies:
    // 1 + |word|·2^(R−23).
    let mut rng = Rng::new(0x0cf0);
    let mut closest = 0f64;
    for (fs, fc) in [
        (20e6, 5.8e9),
        (20e6, 2.412e9),
        (20e6, 5.18e9),
        (20e6, 7.115e9),
        (2e6, 903e6),
    ] {
        let ratio = clock_ratio(fs, fc).unwrap();
        for (r, p) in [(10u32, 22u32), (6, 22), (20, 22), (10, 30)] {
            let fx = FixedOfdmConfig {
                rate_frac: r,
                front: FixedRxConfig {
                    nco_phase_bits: p,
                    ..FixedRxConfig::default()
                },
                ..FixedOfdmConfig::default()
            };
            assert!(fx.is_valid());
            for _ in 0..200 {
                // Up to ±60 ppm of the carrier.
                let ppm_true = (rng.uniform() * 2.0 - 1.0) * 60.0;
                let word = (ppm_true * 1e-6 * fc / fs * 2f64.powi(p as i32)).round() as i64;
                let ppm = word as f64 * fs / 2f64.powi(p as i32) / fc * 1e6;
                let a = FixedTracking::from_cfo_word(word, ratio, &fx);
                let b = FixedTracking::from_clock_offset(ppm, &fx);
                let bound = 1.0 + word.abs() as f64 * 2f64.powi(r as i32 - 23);
                let d = (a.rate0 - b.rate0).abs() as f64;
                assert!(
                    d <= bound,
                    "{fs} {fc} R {r} P {p} word {word}: {d} > {bound}"
                );
                closest = closest.max(d / bound);
                assert_eq!((a.weight, a.delta, a.shift), (b.weight, b.delta, b.shift));
            }
        }
    }
    // The bound is not slack: some draw comes within a third of it.
    assert!(closest > 0.67, "{closest}");
    println!("largest difference over its bound: {closest:.2}");
}

/// The PSDU from decoded DATA bits, as the receiver makes it: the seed from the first seven
/// SERVICE bits (IEEE 802.11-2020 §17.3.5.5), the descrambling, the octets. `None` when no
/// seed gives seven zeros.
fn psdu_of(bits: &[u8], length: usize) -> Option<Vec<u8>> {
    use radio_phy::scrambler::Scrambler;
    let seed = (1u8..128).find(|&s| {
        Scrambler::new(s)
            .unwrap()
            .scramble(&bits[..7])
            .iter()
            .all(|b| *b == 0)
    })?;
    let clear = Scrambler::new(seed).unwrap().scramble(bits);
    Some(
        clear[16..16 + 8 * length]
            .chunks(8)
            .map(|c| c.iter().rev().fold(0u8, |a, &b| (a << 1) | b))
            .collect(),
    )
}

#[test]
fn the_data_is_decoded_with_the_hardwares_traceback_window() {
    use radio_phy::ppdu::data_symbols;
    use radio_phy::viterbi::{decode, decode_windowed, Termination, TRACEBACK_DEPTH};
    // ADR 0010, decision 6.1a: the receiver decodes as the hardware will, each bit decided
    // 96 steps later from the best state then (`viterbi::decode_windowed`); a window of 0 is
    // the whole block's traceback, which every figure before 2026-11-23 was measured with.
    let fx = FixedOfdmConfig::default();
    assert_eq!((fx.traceback, TRACEBACK_DEPTH), (96, 96));
    let whole = FixedOfdmConfig { traceback: 0, ..fx };
    let long = FixedOfdmConfig {
        traceback: 1 << 20,
        ..fx
    };
    let cfg = RxOfdmConfig::default();
    // 54 Mb/s (rate 3/4, where the window costs most) near the edge: some frames decode,
    // some do not, and in some the window decides a bit otherwise than the whole block.
    let (rate, length) = (Rate::R54, 300);
    let symbols = data_symbols(rate, length);
    let tail = 16 + 8 * length + 6;
    let mut rng = Rng::new(0x0064);
    let (mut differ, mut same, mut right, mut wrong) = (0, 0, 0, 0);
    for k in 0..60 {
        let data = psdu(length, &mut rng);
        let mut x = frame(&data, rate);
        x.extend(vec![(0.0, 0.0); 8]);
        let x = at_level(&awgn(&x, 17.5, mean_power(&x), &mut rng), -12.0);
        let z = to_fixed(&x, 2048.0, 12);
        let Ok(windowed) = decode_ppdu_fixed(&z, 192 - 6, &cfg, &fx, unknown(&fx)) else {
            continue;
        };
        if windowed.signal.rate != rate || windowed.signal.length != length {
            continue;
        }
        // The soft values of the data symbols, as the receiver's own steps give them (SIGNAL's
        // 48 and each symbol's common phase taken out of `all_soft`'s list).
        let all = all_soft(&z, 192 - 6, rate, &cfg, &fx, unknown(&fx));
        let per = 48 * rate.modulation().bits_per_subcarrier() + 1;
        let soft: Vec<i32> = all[49..]
            .chunks(per)
            .take(symbols)
            .flat_map(|c| c[..per - 1].iter().map(|&v| v as i32))
            .collect();
        assert_eq!(soft.len(), symbols * (per - 1), "frame {k}");
        let end = Termination::ZeroAfter(tail);
        let by_window = decode_windowed(&soft, rate.code_rate(), end, 96).unwrap();
        let by_block = decode(&soft, rate.code_rate(), end).unwrap();
        assert_eq!(
            Some(&windowed.psdu),
            psdu_of(&by_window, length).as_ref(),
            "frame {k}"
        );
        let block = decode_ppdu_fixed(&z, 192 - 6, &cfg, &whole, unknown(&whole)).unwrap();
        assert_eq!(
            Some(&block.psdu),
            psdu_of(&by_block, length).as_ref(),
            "frame {k}"
        );
        // A window longer than the frame is the whole block.
        let longer = decode_ppdu_fixed(&z, 192 - 6, &cfg, &long, unknown(&long)).unwrap();
        assert_eq!(longer.psdu, block.psdu, "frame {k}");
        differ += usize::from(windowed.psdu != block.psdu);
        same += usize::from(windowed.psdu == block.psdu);
        right += usize::from(windowed.psdu == data);
        wrong += usize::from(windowed.psdu != data);
    }
    assert!(
        differ >= 2 && same >= 10 && right >= 5 && wrong >= 5,
        "{differ} {same} {right} {wrong}"
    );
}
