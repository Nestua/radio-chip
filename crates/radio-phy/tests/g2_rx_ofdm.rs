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
    decode_ppdu, equalise, estimate_channel, soft_bits, RxOfdmConfig, Tracking,
};
use radio_phy::rx_ofdm_fixed::{
    decode_ppdu_fixed, estimate_channel_fixed, fft64_fixed, symbol_soft_fixed, to_fixed,
    FixedOfdmConfig,
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
