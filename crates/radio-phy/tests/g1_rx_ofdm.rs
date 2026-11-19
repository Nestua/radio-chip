//! G1 gate of the OFDM receiver after acquisition (R0-09, `radio_phy::rx_ofdm`).
//!
//! References we did not write:
//! 1. The transmit chain, itself gated against the standard's worked example and an outside
//!    receiver (`g1_annex_i1`, `g1_ppdu_openofdm_decode`): every PSDU it encodes must come back.
//! 2. The max-log soft metric computed by brute force from the constellation (its definition),
//!    against the receiver's closed-form demapper.
//! 3. Textbook error rates in AWGN (`ser_theory`, as in `g1_channel`), and the textbook loss of
//!    a least-squares channel estimate from two training symbols: the estimate's noise has half
//!    the variance of a data symbol's, so the equalised points' mean squared error is 1.5 times
//!    the noise, 1.76 dB.
//! 4. CommPy's soft-decision Viterbi bit error rates (`fixtures/viterbi_ber_commpy.txt`, a black
//!    box, ADR 0005) for the whole demapper → deinterleaver → depuncturer → decoder chain.
//! 5. The standard's own waveform: the 881 printed samples of Annex I.1 decode to its PSDU
//!    (`g1_annex_i1::annex_waveform_decodes`, local fixture).

// Helpers outside #[test] build known-good inputs; a failure there should just fail the test.
#![allow(clippy::unwrap_used)]

use std::f64::consts::PI;

use radio_phy::channel::{
    awgn, erfc, fractional_delay, frequency_offset, ifft, mean_power, resample, tgn_taps, Doppler,
    Multipath, Rng, TgnModel,
};
use radio_phy::mapper::{map_group, mean_energy};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{fft_bin, modulate_symbol_f64, IfftConfig};
use radio_phy::ppdu::{build_ppdu, data_bits, long_training_sequence, Rate};
use radio_phy::rx_front::{acquire, correct, RxConfig};
use radio_phy::rx_ofdm::{
    decode_data_bits, decode_ppdu, decode_ppdu_from, decode_signal, demap, equalise,
    estimate_channel, soft_bits, ChannelEstimate, Equalised, RxOfdmConfig, RxOfdmError, Signal,
    SymbolLog, Tracking,
};

type C = (f64, f64);

const FS: f64 = 20e6;
const T1: usize = 192;
const MODULATIONS: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

fn frame(psdu: &[u8], rate: Rate, seed: u8) -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

fn random_psdu(len: usize, rng: &mut Rng) -> Vec<u8> {
    (0..len).map(|_| (rng.next_u64() & 0xff) as u8).collect()
}

fn bits(group: usize, n: usize) -> Vec<u8> {
    (0..n).map(|i| ((group >> (n - 1 - i)) & 1) as u8).collect()
}

/// All points of a constellation with their bits (b0 first), on the unnormalised grid.
fn constellation(m: Modulation) -> Vec<(Vec<u8>, C)> {
    let n = m.bits_per_subcarrier();
    (0..1usize << n)
        .map(|g| {
            let b = bits(g, n);
            let p = map_group(&b, m).unwrap();
            (b, (f64::from(p.i), f64::from(p.q)))
        })
        .collect()
}

/// The max-log metric by its definition: (min over points with the bit 1 of d² − min over
/// points with the bit 0 of d²) / 4, positive for bit 0, 1 for a noiseless BPSK point.
fn max_log(v: C, m: Modulation) -> Vec<f64> {
    let pts = constellation(m);
    (0..m.bits_per_subcarrier())
        .map(|k| {
            let best = |bit: u8| {
                pts.iter()
                    .filter(|(b, _)| b[k] == bit)
                    .map(|(_, p)| (v.0 - p.0).powi(2) + (v.1 - p.1).powi(2))
                    .fold(f64::INFINITY, f64::min)
            };
            (best(1) - best(0)) / 4.0
        })
        .collect()
}

#[test]
fn demapper_inverts_the_mapper() {
    for m in MODULATIONS {
        for (b, p) in constellation(m) {
            for (dx, dy) in [
                (0.0, 0.0),
                (0.9, -0.9),
                (-0.9, 0.9),
                (0.9, 0.9),
                (-0.9, -0.9),
            ] {
                // BPSK has no information on Q: any Q offset must be ignored.
                let v = (
                    p.0 + dx,
                    p.1 + if m == Modulation::Bpsk { 5.0 * dy } else { dy },
                );
                let soft = demap(v, m);
                assert_eq!(soft.len(), b.len(), "{m:?}");
                let hard: Vec<u8> = soft.iter().map(|s| u8::from(*s < 0.0)).collect();
                assert_eq!(hard, b, "{m:?} point {p:?} + ({dx}, {dy}): soft {soft:?}");
            }
            // A noiseless point is at least one unit from every decision boundary.
            assert!(
                demap(p, m).iter().all(|s| s.abs() >= 1.0 - 1e-12),
                "{m:?} {p:?}"
            );
        }
    }
}

#[test]
fn demapper_against_brute_force_max_log() {
    for m in MODULATIONS {
        let (mut exact, mut total) = (0usize, 0usize);
        for ix in -72..=72 {
            for iq in -72..=72 {
                let v = (f64::from(ix) * 0.125 + 0.01, f64::from(iq) * 0.125 - 0.02);
                let ours = demap(v, m);
                let want = max_log(v, m);
                assert_eq!(ours.len(), want.len(), "{m:?}");
                for (k, (a, b)) in ours.iter().zip(&want).enumerate() {
                    // Same decision everywhere; never more confident than the exact metric;
                    // and equal to it wherever the two nearest candidates are neighbours.
                    assert_eq!(a.signum(), b.signum(), "{m:?} {v:?} bit {k}: {a} vs {b}");
                    assert!(a.abs() <= b.abs() + 1e-9, "{m:?} {v:?} bit {k}: {a} vs {b}");
                    total += 1;
                    exact += usize::from((a - b).abs() < 1e-9);
                }
            }
        }
        println!("{m:?}: closed form equals max-log on {exact} of {total} grid metrics");
        // BPSK and QPSK are exact; the higher orders are exact near the boundaries.
        if matches!(m, Modulation::Bpsk | Modulation::Qpsk) {
            assert_eq!(exact, total);
        }
        assert!(
            exact * 4 >= total,
            "{m:?}: exact on only {exact} of {total}"
        );
    }
}

#[test]
fn every_rate_and_length_comes_back() {
    let mut rng = Rng::new(0x0f09_0001);
    let cfg = RxOfdmConfig::default();
    let mut frames = 0;
    for rate in Rate::ALL {
        for len in [1usize, 2, 13, 100, 257, 1500] {
            let psdu = random_psdu(len, &mut rng);
            let seed = 1 + (rng.next_u64() % 127) as u8;
            let mut y = vec![(0.0, 0.0); 50];
            y.extend(frame(&psdu, rate, seed));
            // Any window start inside the cyclic prefix decodes a clean frame.
            for backoff in [0usize, 4, 9, 15] {
                let got = decode_ppdu(&y, 50 + T1 - backoff, &cfg)
                    .unwrap_or_else(|e| panic!("{rate:?} len {len} back-off {backoff}: {e}"));
                assert_eq!(got.signal.rate, rate);
                assert_eq!(got.signal.length, len);
                assert_eq!(got.scrambler_seed, seed);
                assert_eq!(got.psdu, psdu, "{rate:?} len {len} back-off {backoff}");
                frames += 1;
            }
        }
    }
    // The longest PSDU at the lowest and the highest rate.
    for rate in [Rate::R6, Rate::R54] {
        let psdu = random_psdu(4095, &mut rng);
        let y = frame(&psdu, rate, 0x5d);
        assert_eq!(decode_ppdu(&y, T1 - 4, &cfg).unwrap().psdu, psdu);
        frames += 1;
    }
    println!("{frames} clean frames decoded");
}

#[test]
fn signal_errors_are_reported() {
    let cfg = RxOfdmConfig::default();
    let y = frame(&[0x11; 40], Rate::R24, 0x35);
    let h = estimate_channel(&y[T1 - 4..T1 - 4 + 128]).unwrap();
    let s = decode_signal(&y, T1 - 4, &h, &cfg).unwrap();
    assert_eq!((s.rate, s.length), (Rate::R24, 40));
    // Truncated in SIGNAL, and in DATA.
    assert!(matches!(
        decode_ppdu(&y[..T1 + 128 + 40], T1 - 4, &cfg),
        Err(RxOfdmError::Truncated { .. })
    ));
    assert!(matches!(
        decode_ppdu(&y[..y.len() - 81], T1 - 4, &cfg),
        Err(RxOfdmError::Truncated { .. })
    ));
    assert!(matches!(
        decode_ppdu(&y[..T1 + 60], T1 - 4, &cfg),
        Err(RxOfdmError::Truncated { .. })
    ));
    // Noise where SIGNAL should be: over many draws parity fails about half the time, and what
    // passes parity is rejected by the rate or length checks or decodes to garbage — never a panic.
    let mut rng = Rng::new(0x0f09_0002);
    let (mut parity, mut other) = (0, 0);
    for _ in 0..400 {
        let mut z = y[..T1 + 124].to_vec();
        z.extend(awgn(&vec![(0.0, 0.0); 400], 0.0, mean_power(&y), &mut rng));
        match decode_ppdu(&z, T1 - 4, &cfg) {
            Err(RxOfdmError::SignalParity) => parity += 1,
            _ => other += 1,
        }
    }
    println!("noise in place of SIGNAL: {parity} parity failures, {other} other outcomes");
    assert!(
        (140..=260).contains(&parity),
        "parity failures: {parity} of 400"
    );
}

/// A long training field (GI2 + T1 + T2) and `n` data symbols of random points, in exact float.
fn uncoded_stream(m: Modulation, n: usize, rng: &mut Rng) -> (Vec<C>, Vec<Vec<usize>>) {
    let mut bins = [(0.0, 0.0); 64];
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        bins[fft_bin(k)] = (f64::from(l), 0.0);
    }
    let t = ifft(&bins).unwrap();
    let mut x: Vec<C> = t[32..].to_vec();
    x.extend(&t);
    x.extend(&t);
    let nb = m.bits_per_subcarrier();
    let mut sent = Vec::with_capacity(n);
    for i in 0..n {
        let groups: Vec<usize> = (0..48)
            .map(|_| (rng.next_u64() % (1 << nb)) as usize)
            .collect();
        let points: Vec<_> = groups
            .iter()
            .map(|&g| map_group(&bits(g, nb), m).unwrap())
            .collect();
        x.extend(modulate_symbol_f64(&points, m, i + 1).unwrap());
        sent.push(groups);
    }
    (x, sent)
}

fn ser_theory(m: Modulation, es_n0: f64) -> f64 {
    let q = |x: f64| 0.5 * erfc(x / 2f64.sqrt());
    match m {
        Modulation::Bpsk => q((2.0 * es_n0).sqrt()),
        _ => {
            let big_m = f64::from(1u32 << m.bits_per_subcarrier());
            let a = 2.0 * (1.0 - 1.0 / big_m.sqrt()) * q((3.0 * es_n0 / (big_m - 1.0)).sqrt());
            1.0 - (1.0 - a).powi(2)
        }
    }
}

/// Symbol errors of the equaliser's hard decisions at a per-subcarrier Es/N0, with the true
/// channel (`genie`) or the receiver's own estimate. The channel is a constant complex gain.
/// `streams` independent training fields, each followed by `symbols` data symbols.
fn symbol_errors(
    m: Modulation,
    es_n0_db: f64,
    genie: bool,
    streams: usize,
    symbols: usize,
    rng: &mut Rng,
) -> (usize, usize, f64) {
    let cfg = RxOfdmConfig {
        pilot_tracking: false,
        ..RxOfdmConfig::default()
    };
    let g = (0.6, -0.8);
    let mut truth = [(0.0, 0.0); 64];
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        if l != 0 {
            truth[fft_bin(k)] = g;
        }
    }
    let pts = constellation(m);
    let nb = m.bits_per_subcarrier();
    // Unnormalised FFT: a bin of amplitude 1 against noise of 64 σ² per bin.
    let sigma2 = 1.0 / (64.0 * 10f64.powf(es_n0_db / 10.0));
    let mut errors = 0;
    let mut sq = 0.0;
    let energy = f64::from(mean_energy(m));
    for _ in 0..streams {
        let (x, sent) = uncoded_stream(m, symbols, rng);
        let x: Vec<C> = x
            .iter()
            .map(|s| (s.0 * g.0 - s.1 * g.1, s.0 * g.1 + s.1 * g.0))
            .collect();
        let y = awgn(&x, 0.0, sigma2, rng);
        let h: ChannelEstimate = if genie {
            truth
        } else {
            estimate_channel(&y[32..160]).unwrap()
        };
        for (i, groups) in sent.iter().enumerate() {
            let start = 160 + 80 * i + 16;
            let eq = equalise(
                &y[start..start + 64],
                &h,
                i + 1,
                m,
                &cfg,
                &mut Tracking::default(),
            )
            .unwrap();
            assert_eq!(eq.points.len(), 48);
            for (v, &gidx) in eq.points.iter().zip(groups) {
                let d = |p: &C| (p.0 - v.0).powi(2) + (p.1 - v.1).powi(2);
                let best = pts
                    .iter()
                    .min_by(|a, b| d(&a.1).total_cmp(&d(&b.1)))
                    .unwrap();
                errors += usize::from(best.0 != bits(gidx, nb));
                sq += d(&pts.iter().find(|p| p.0 == bits(gidx, nb)).unwrap().1) / energy;
            }
        }
    }
    let n = 48 * symbols * streams;
    (errors, n, sq / n as f64)
}

#[test]
fn equaliser_error_rate_with_the_true_channel_is_the_textbook_one() {
    let mut rng = Rng::new(0x0f09_0003);
    for (m, db) in [
        (Modulation::Bpsk, 6.0),
        (Modulation::Qpsk, 9.0),
        (Modulation::Qam16, 16.0),
        (Modulation::Qam64, 22.0),
    ] {
        let p = ser_theory(m, 10f64.powf(db / 10.0));
        let symbols = ((3000.0 / p) as usize / 48).clamp(500, 20_000);
        let (e, n, _) = symbol_errors(m, db, true, symbols.div_ceil(50), 50, &mut rng);
        let mean = n as f64 * p;
        let sd = (mean * (1.0 - p)).sqrt();
        println!(
            "{m:?} {db} dB, true channel: {e} errors / {n}, theory {mean:.0} ({:+.2} σ)",
            (e as f64 - mean) / sd
        );
        assert!(
            (e as f64 - mean).abs() < 5.0 * sd,
            "{m:?}: {e} vs {mean:.0}"
        );
    }
}

#[test]
fn least_squares_estimate_raises_the_error_vector_by_1_76_db() {
    // Equalised point ≈ s + (n − e·s)/H: the estimate's error e has half the variance of the
    // noise n, so the mean squared error is σ²(1 + E|s|²/2) = 1.5 σ² for unit mean energy.
    // First order in e: measured at a high Es/N0. (Hard decisions of BPSK do not see this
    // loss: an amplitude error of the estimate cannot move a point across the only boundary.
    // An earlier form of this test assumed they would, and the receiver proved it wrong.)
    let mut rng = Rng::new(0x0f09_0004);
    for m in MODULATIONS {
        let db = 28.0;
        let noise = 10f64.powf(-db / 10.0);
        let (_, _, genie) = symbol_errors(m, db, true, 1500, 2, &mut rng);
        let (_, _, own) = symbol_errors(m, db, false, 1500, 2, &mut rng);
        println!(
            "{m:?}: error vector with the true channel {:.4} σ², with the estimate {:.4} σ² ({:+.2} dB)",
            genie / noise,
            own / noise,
            10.0 * (own / genie).log10()
        );
        assert!(
            (genie / noise - 1.0).abs() < 0.02,
            "{m:?}: {}",
            genie / noise
        );
        assert!((own / noise - 1.5).abs() < 0.03, "{m:?}: {}", own / noise);
    }
}

const COMMPY: &str = include_str!("fixtures/viterbi_ber_commpy.txt");

#[test]
fn coded_bit_error_rate_matches_commpy() {
    // Fine soft values: the CommPy runs used an unquantised metric.
    let cfg = RxOfdmConfig {
        soft_max: 255,
        clip: 4.0,
        pilot_tracking: false,
        timing_tracking: false,
    };
    let mut rng = Rng::new(0x0f09_0005);
    let mut h = [(0.0, 0.0); 64];
    for (k, &l) in (-26i8..=26).zip(long_training_sequence().iter()) {
        if l != 0 {
            h[fft_bin(k)] = (1.0, 0.0);
        }
    }
    let mut checked = 0;
    for line in COMMPY
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        let (rate, code) = match f[0] {
            "1/2" => (Rate::R6, 0.5),
            "3/4" => (Rate::R9, 0.75),
            _ => continue, // 2/3 exists only with 64-QAM
        };
        let eb_n0_db: f64 = f[1].parse().unwrap();
        let (want, se): (f64, f64) = (f[4].parse().unwrap(), f[5].parse().unwrap());
        // BPSK: Es/N0 per subcarrier = code rate · Eb/N0.
        let es_n0 = code * 10f64.powf(eb_n0_db / 10.0);
        let sigma2 = 1.0 / (64.0 * es_n0);
        let (mut errors, mut total, mut lost) = (0usize, 0usize, 0usize);
        let blocks = 600;
        for _ in 0..blocks {
            let psdu = random_psdu(125, &mut rng);
            let seed = 1 + (rng.next_u64() % 127) as u8;
            let x = frame(&psdu, rate, seed);
            let y = awgn(&x, 0.0, sigma2, &mut rng);
            // The decoder's output before the descrambler against the transmitter's scrambled
            // bits (SERVICE and PSDU): a wrong seed would turn one error into a lost frame.
            let sent = data_bits(&psdu, rate, seed).unwrap();
            let signal = Signal { rate, length: 125 };
            match decode_signal(&y, T1, &h, &cfg) {
                Ok(s) if s == signal => {
                    let got = decode_data_bits(
                        &y,
                        T1,
                        &h,
                        signal,
                        &cfg,
                        &mut Tracking::default(),
                        &mut SymbolLog::default(),
                    )
                    .unwrap();
                    errors += got[..1016]
                        .iter()
                        .zip(&sent[..1016])
                        .filter(|(a, b)| a != b)
                        .count();
                    total += 1016;
                }
                // SIGNAL (rate 1/2 BPSK at the same Es/N0) was lost: a receiver would not
                // decode the frame at all.
                _ => lost += 1,
            }
        }
        let ber = errors as f64 / total.max(1) as f64;
        println!("{} Eb/N0 {eb_n0_db} dB: BER {ber:.3e} ({errors} / {total}), CommPy {want:.3e} ± {se:.1e}; {lost} of {blocks} SIGNAL fields lost", f[0]);
        // Our standard error is about CommPy's × √(1000/600); block lengths differ slightly.
        let tol = 5.0 * se * (1.0f64 + 1000.0 / blocks as f64).sqrt() + 0.10 * want;
        assert!(
            (ber - want).abs() < tol,
            "{} at {eb_n0_db} dB: {ber:e} vs {want:e}",
            f[0]
        );
        assert!(lost * 5 < blocks, "{lost} frames lost");
        checked += 1;
    }
    assert_eq!(checked, 4);
}

#[test]
fn through_the_front_and_a_channel() {
    // Acquisition (float front), CFO correction, then this receiver. Loose bounds: the figures
    // go to the journal; the gate is that the chain holds together at every rate.
    let mut rng = Rng::new(0x0f09_0006);
    let rx = RxConfig::new(FS);
    let cfg = RxOfdmConfig::default();
    for (rate, snr_db, tgn, max_lost) in [
        (Rate::R6, 8.0, false, 2),
        (Rate::R12, 11.0, false, 2),
        (Rate::R24, 17.0, false, 2),
        (Rate::R54, 28.0, false, 3),
        (Rate::R6, 20.0, true, 4),
        (Rate::R24, 30.0, true, 6),
    ] {
        let (mut lost, frames) = (0, 40);
        for _ in 0..frames {
            let psdu = random_psdu(100, &mut rng);
            let f = frame(&psdu, rate, 0x5d);
            let mut x = vec![(0.0, 0.0); 300];
            x.extend_from_slice(&f);
            x.extend(vec![(0.0, 0.0); 200]);
            let x = if tgn {
                Multipath::new(
                    tgn_taps(TgnModel::D, false),
                    Doppler::Static,
                    1,
                    FS,
                    &mut rng,
                )
                .apply(&x)
            } else {
                fractional_delay(&x, rng.uniform())
            };
            let cfo = (rng.uniform() * 2.0 - 1.0) * 200e3;
            let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
            let x = awgn(&x, snr_db, mean_power(&f), &mut rng);
            let ok = acquire(&x, 0, &rx)
                .and_then(|a| decode_ppdu(&correct(&x, a.cfo_hz, FS), a.fft_start, &cfg).ok())
                .is_some_and(|d| d.psdu == psdu);
            lost += usize::from(!ok);
        }
        println!(
            "{rate:?} at {snr_db} dB{}: {lost} of {frames} frames lost",
            if tgn { ", TGn D" } else { "" }
        );
        assert!(
            lost <= max_lost,
            "{rate:?} at {snr_db} dB: {lost} of {frames} lost"
        );
    }
}

#[test]
fn soft_value_contract() {
    // docs/gates/viterbi.md: a matched-filter level of 1 (a noiseless BPSK point on a subcarrier
    // of mean gain) is soft_max/clip = 7/1.5 → 5; 1.5 and beyond is 7; the weight scales it.
    let cfg = RxOfdmConfig::default();
    let eq = |v: f64, w: f64| Equalised {
        points: vec![(v, 0.3); 48],
        weights: vec![w; 48],
        common_phase: 0.0,
    };
    let all = |v: f64, w: f64| {
        let s = soft_bits(&eq(v, w), Modulation::Bpsk, &cfg);
        assert_eq!(s.len(), 48);
        assert!(s.iter().all(|x| *x == s[0]));
        s[0]
    };
    assert_eq!(all(1.0, 1.0), -5); // bit 1 is +1 on air, negative soft
    assert_eq!(all(-1.0, 1.0), 5);
    assert_eq!(all(-1.5, 1.0), 7);
    assert_eq!(all(-9.0, 1.0), 7);
    assert_eq!(all(9.0, 1.0), -7);
    assert_eq!(all(-1.0, 0.5), 2); // 2.33 → 2
    assert_eq!(all(-1.0, 0.0), 0);
    assert_eq!(all(-0.1, 1.0), 0); // 0.47 → 0: an erasure
    assert_eq!(all(-0.11, 1.0), 1); // 0.513 → 1
                                    // Order: the soft values come back in coded order. A symbol whose points encode the
                                    // interleaved pattern of a known coded sequence gives that sequence's signs.
    for m in MODULATIONS {
        let n = 48 * m.bits_per_subcarrier();
        let coded: Vec<u8> = (0..n).map(|k| ((k * 7 + k / 5) % 3 == 0) as u8).collect();
        let on_air = radio_phy::interleaver::interleave(&coded, m).unwrap();
        let points: Vec<C> = on_air
            .chunks(m.bits_per_subcarrier())
            .map(|g| {
                let p = map_group(g, m).unwrap();
                (f64::from(p.i), f64::from(p.q))
            })
            .collect();
        let e = Equalised {
            points,
            weights: vec![1.0; 48],
            common_phase: 0.0,
        };
        let hard: Vec<u8> = soft_bits(&e, m, &cfg)
            .iter()
            .map(|s| u8::from(*s < 0))
            .collect();
        assert_eq!(hard, coded, "{m:?}");
    }
}

#[test]
fn channel_state_weights_carry_a_notched_channel() {
    // Two equal paths two samples apart: H(k) = 1 + e^(−j2πk·2/64), nulls near k = ±16. The
    // subcarriers in the notch are noise; weighted by |H|² they cost the code little, unweighted
    // they poison it. The bound is what the receiver does (a consistency gate); that weighting
    // is right is the max-log metric's definition, LLR ∝ |H|²·distance.
    let cfg = RxOfdmConfig::default();
    // Without the weights 130 of 150 frames are lost at 7 dB (mutation, docs/gates/rx_ofdm.md).
    for (snr_db, max_lost) in [(7.0, 8)] {
        let mut rng = Rng::new(0x0f09_0007);
        let (mut lost, frames) = (0, 150);
        for _ in 0..frames {
            let psdu = random_psdu(60, &mut rng);
            let f = frame(&psdu, Rate::R12, 0x4b);
            let mut x = vec![(0.0, 0.0); f.len() + 2];
            for (n, s) in f.iter().enumerate() {
                x[n].0 += s.0;
                x[n].1 += s.1;
                x[n + 2].0 += s.0;
                x[n + 2].1 += s.1;
            }
            let y = awgn(&x, snr_db, mean_power(&f), &mut rng);
            let ok = decode_ppdu(&y, T1 - 1, &cfg).is_ok_and(|d| d.psdu == psdu);
            lost += usize::from(!ok);
        }
        println!("notched channel, 12 Mb/s at {snr_db} dB: {lost} of {frames} frames lost");
        assert!(lost <= max_lost, "{snr_db} dB: {lost} of {frames} lost");
    }
}

#[test]
fn pilots_track_a_residual_frequency_offset() {
    // A residual CFO f turns symbol i by 2π·f·Δn/fs against the channel estimate, Δn from the
    // middle of the two training symbols to the middle of the symbol's window: the reference
    // for the reported common phase. 54 Mb/s over 57 symbols does not survive it untracked
    // (700 Hz turns the last symbol by 58°).
    let psdu = random_psdu(1500, &mut Rng::new(0x0f09_0008));
    let f = frame(&psdu, Rate::R54, 0x2a);
    let cfg = RxOfdmConfig::default();
    for cfo in [700.0, -1900.0] {
        let y = frequency_offset(&f, cfo, FS, 0.4);
        let d = decode_ppdu(&y, T1 - 4, &cfg).unwrap();
        assert_eq!(d.psdu, psdu, "cfo {cfo}");
        assert_eq!(d.common_phase.len(), 1 + 56);
        let mut worst = 0.0f64;
        for (i, got) in d.common_phase.iter().enumerate() {
            let dn = (128 + 16 + 80 * i + 32) as f64 - 64.0;
            let want = 2.0 * PI * cfo * dn / FS;
            let err = (got - want + PI).rem_euclid(2.0 * PI) - PI;
            worst = worst.max(err.abs());
        }
        println!(
            "residual CFO {cfo} Hz: worst common-phase error {worst:.4} rad over {} symbols",
            d.common_phase.len()
        );
        // The offset also smears each subcarrier a little (ICI); 0.02 rad covers it.
        assert!(worst < 0.02, "cfo {cfo}: {worst}");
        let untracked = RxOfdmConfig {
            pilot_tracking: false,
            ..cfg
        };
        assert!(decode_ppdu(&y, T1 - 4, &untracked).map(|d| d.psdu) != Ok(psdu.clone()));
    }
}

#[test]
fn pilots_track_a_sample_clock_offset() {
    // A transmitter clock fast by p ppm (`resample`'s p) makes the receiver's windows
    // p·1e-6·Δn samples late Δn samples after the training field: the reference for the
    // tracked timing. ±40 ppm is the standard's worst
    // case (±20 ppm at each end, IEEE 802.11-2020 §17.3.9.5–6); the longest frames drift by more
    // than four samples, past any fixed window.
    let mut rng = Rng::new(0x0f09_0009);
    let cfg = RxOfdmConfig::default();
    for (rate, octets) in [
        (Rate::R6, 4095usize),
        (Rate::R24, 4095),
        (Rate::R54, 4095),
        (Rate::R54, 300),
    ] {
        let psdu = random_psdu(octets, &mut rng);
        let mut x = frame(&psdu, rate, 0x61);
        x.extend(vec![(0.0, 0.0); 16]);
        for ppm in [40.0, -40.0, 13.0] {
            let y = resample(&x, ppm);
            // The loop starts from the clock offset, as it does from the front's CFO (0.2 ppm
            // off here, a 1 kHz CFO error at 5.8 GHz).
            let start = Tracking::from_clock_offset(ppm + 0.2);
            let d = decode_ppdu_from(&y, T1 - 4, &cfg, start)
                .unwrap_or_else(|e| panic!("{rate:?} {octets} octets at {ppm} ppm: {e}"));
            assert_eq!(d.psdu, psdu, "{rate:?} {octets} octets at {ppm} ppm");
            let mut worst = 0.0f64;
            for (i, got) in d.timing.iter().enumerate() {
                let dn = (128 + 16 + 80 * i + 32) as f64 - 64.0;
                worst = worst.max((got - ppm * 1e-6 * dn).abs());
            }
            let last = d.timing.last().copied().unwrap_or(0.0);
            println!(
                "{rate:?} {octets} octets at {ppm:+} ppm: {} symbols, drift {last:+.2} samples, worst tracking error {worst:.3} samples",
                d.timing.len()
            );
            assert!(worst < 0.004, "{rate:?} at {ppm} ppm: {worst}");
            let untracked = RxOfdmConfig {
                timing_tracking: false,
                ..cfg
            };
            if octets == 4095 {
                assert!(decode_ppdu(&y, T1 - 4, &untracked).map(|d| d.psdu) != Ok(psdu.clone()));
            }
            // Started from zero the loop still pulls in where the constellation leaves room.
            if rate != Rate::R54 {
                assert_eq!(
                    decode_ppdu(&y, T1 - 4, &cfg).map(|d| d.psdu),
                    Ok(psdu.clone())
                );
            }
        }
    }
}

#[test]
fn timing_loop_from_zero_has_no_standing_error() {
    // Started from zero at +40 ppm the drift is a ramp of 0.0032 samples per symbol, which the
    // tracker has to learn from the frame: its estimate must converge on the ramp, with no
    // standing error.
    let psdu = random_psdu(4095, &mut Rng::new(0x0f09_000a));
    let mut x = frame(&psdu, Rate::R6, 0x33);
    x.extend(vec![(0.0, 0.0); 16]);
    let y = resample(&x, 40.0);
    let d = decode_ppdu(&y, T1 - 4, &RxOfdmConfig::default()).unwrap();
    assert_eq!(d.psdu, psdu);
    let err = |i: usize| {
        let dn = (128 + 16 + 80 * i + 32) as f64 - 64.0;
        (d.timing[i] - 40e-6 * dn).abs()
    };
    let early = (0..200).map(err).fold(0.0, f64::max);
    let late = (700..d.timing.len()).map(err).fold(0.0, f64::max);
    println!("from zero at +40 ppm: worst error {early:.3} samples in the first 200 symbols, {late:.4} after 700");
    assert!(late < 0.01, "standing error {late}");
    assert!(early < 0.2, "pull-in error {early}");
}

#[test]
fn window_follows_a_drift_longer_than_the_guard_interval() {
    // Within the standard's ±40 ppm and 4095 octets the drift stays under 4.4 samples, which a
    // window with a back-off survives without moving (with some inter-symbol interference at
    // the end). To gate the move itself: ten times the worst case, ±400 ppm over 5.5 ms, a
    // drift of 44 samples, almost three guard intervals. Only a window that walks decodes it.
    let mut rng = Rng::new(0x0f09_000b);
    let cfg = RxOfdmConfig::default();
    let psdu = random_psdu(4095, &mut rng);
    let mut x = frame(&psdu, Rate::R6, 0x1c);
    x.extend(vec![(0.0, 0.0); 80]);
    for ppm in [400.0, -400.0] {
        let y = resample(&x, ppm);
        let d = decode_ppdu_from(&y, T1 - 4, &cfg, Tracking::from_clock_offset(ppm)).unwrap();
        assert_eq!(d.psdu, psdu, "{ppm} ppm");
        let last = d.timing.last().copied().unwrap();
        let dn = (128 + 16 + 80 * (d.timing.len() - 1) + 32) as f64 - 64.0;
        println!("{ppm:+} ppm: drift tracked to {last:+.2} samples");
        assert!((last - ppm * 1e-6 * dn).abs() < 0.02, "{ppm} ppm: {last}");
        assert!(last.abs() > 43.0);
    }
}

#[test]
fn slope_and_phase_on_unequal_pilots() {
    // One symbol built in the frequency domain: Z(k) = H(k)·X(k)·e^(j(φ + kθ)) with a known
    // common phase φ and slope θ (a window 0.3 samples late), a channel whose pilots at ±21 are
    // in a null (and carry garbage) and whose pilots at ±7 differ fivefold in gain. The
    // measurement must be θ; with θ removed, the phase is φ and the points are the sent ones.
    let mut rng = Rng::new(0x0f09_000c);
    let m = Modulation::Qam16;
    let groups: Vec<usize> = (0..48).map(|_| (rng.next_u64() % 16) as usize).collect();
    let points: Vec<_> = groups
        .iter()
        .map(|&g| map_group(&bits(g, 4), m).unwrap())
        .collect();
    let clean = modulate_symbol_f64(&points, m, 5).unwrap();
    let x = radio_phy::channel::fft(&clean[16..]).unwrap();
    let mut h: ChannelEstimate = [(0.0, 0.0); 64];
    for k in (-26i8..=26).filter(|k| *k != 0) {
        let (mag, ph) = (0.3 + 1.2 * rng.uniform(), 2.0 * PI * rng.uniform());
        h[fft_bin(k)] = (mag * ph.cos(), mag * ph.sin());
    }
    h[fft_bin(21)] = (1e-4, 2e-4);
    h[fft_bin(-21)] = (-2e-4, 1e-4);
    h[fft_bin(7)] = (1.5, 0.0);
    h[fft_bin(-7)] = (0.0, 0.3);
    let (phi, theta) = (0.4, 0.3 * 2.0 * PI / 64.0);
    let mut z = vec![(0.0, 0.0); 64];
    for k in (-26i8..=26).filter(|k| *k != 0) {
        let b = fft_bin(k);
        let a = phi + f64::from(k) * theta;
        let v = (
            x[b].0 * h[b].0 - x[b].1 * h[b].1,
            x[b].0 * h[b].1 + x[b].1 * h[b].0,
        );
        z[b] = (v.0 * a.cos() - v.1 * a.sin(), v.0 * a.sin() + v.1 * a.cos());
    }
    z[fft_bin(21)] = (0.02, -0.05);
    z[fft_bin(-21)] = (-0.04, 0.01);
    let window = ifft(&z).unwrap();
    let cfg = RxOfdmConfig::default();
    // The measurement: a tracker that knows nothing sees θ.
    let mut tracking = Tracking::default();
    equalise(&window, &h, 5, m, &cfg, &mut tracking).unwrap();
    println!("measured slope {:.6} (true {theta:.6})", tracking.measured);
    assert!((tracking.measured - theta).abs() < 1e-4);
    // The correction: a tracker that knows the drift (θ at this symbol's time, 6.4 symbols
    // after the training field) removes it, and then the phase and the points are exact.
    let ppm = 0.3 / (6.4 * 80.0) * 1e6;
    let mut tracking = Tracking::fixed(ppm);
    let eq = equalise(&window, &h, 5, m, &cfg, &mut tracking).unwrap();
    println!(
        "removed {:.6}, left {:.6}, common phase {:.6} (true {phi})",
        tracking.slope, tracking.measured, eq.common_phase
    );
    assert!((tracking.slope - theta).abs() < 1e-9);
    assert!(tracking.measured.abs() < 1e-4);
    assert!((tracking.timing() - 0.3).abs() < 1e-9);
    assert!((eq.common_phase - phi).abs() < 1e-3);
    assert_eq!(tracking.shift, 0);
    for (v, p) in eq.points.iter().zip(&points) {
        assert!((v.0 - f64::from(p.i)).abs() < 0.02 && (v.1 - f64::from(p.q)).abs() < 0.02);
    }
}

#[test]
fn clock_offset_and_carrier_offset_have_one_sign() {
    // The rule the timing loop's start rests on, from physics rather than from our own
    // scenario: `resample(x, p)` scales every frequency of x by 1 + p·1e-6 (a tone at f comes
    // out at f·(1 + p·1e-6)). A transmitter whose one oscillator is p ppm fast therefore also
    // sends its carrier p·1e-6·fc high: the CFO the front measures is +p·1e-6·fc, and
    // `Tracking::from_clock_offset` takes p = +CFO/carrier·1e6. (The first version of this
    // receiver and of `rxper` had the opposite sign on both sides, which no test could see.)
    let (f, p) = (1.0e6, 1000.0);
    let x: Vec<C> = (0..4000)
        .map(|n| {
            let a = 2.0 * PI * f * n as f64 / FS;
            (a.cos(), a.sin())
        })
        .collect();
    let y = resample(&x, p);
    // Mean phase advance per sample over the middle of the stream.
    let adv: f64 = (1000..3000)
        .map(|n| {
            let (a, b) = (y[n], y[n + 1]);
            (b.1 * a.0 - b.0 * a.1).atan2(b.0 * a.0 + b.1 * a.1)
        })
        .sum::<f64>()
        / 2000.0;
    let measured = adv * FS / (2.0 * PI);
    println!("1 MHz through resample(+1000 ppm): {measured:.1} Hz");
    assert!((measured - f * (1.0 + p * 1e-6)).abs() < 1.0, "{measured}");
    // And the loop's start for +p is the drift that resample(+p) makes: a 54 Mb/s frame whose
    // tracker is fixed (it learns nothing) decodes only if the start has the right sign.
    let frozen = RxOfdmConfig::default();
    let psdu = random_psdu(1500, &mut Rng::new(0x0f09_000d));
    let mut x = frame(&psdu, Rate::R54, 0x47);
    x.extend(vec![(0.0, 0.0); 16]);
    let y = resample(&x, 40.0);
    let right = decode_ppdu_from(&y, T1 - 4, &frozen, Tracking::fixed(40.0));
    let wrong = decode_ppdu_from(&y, T1 - 4, &frozen, Tracking::fixed(-40.0));
    // The same through the start a receiver uses, with the clock offset the CFO of such a
    // transmitter implies (+40 ppm of the carrier): `from_clock_offset` has the sign too.
    let cfo = 40e-6 * 5.8e9;
    let hinted = decode_ppdu_from(
        &y,
        T1 - 4,
        &frozen,
        Tracking::from_clock_offset(cfo / 5.8e9 * 1e6),
    );
    assert_eq!(hinted.map(|d| d.psdu), Ok(psdu.clone()));
    let against = decode_ppdu_from(
        &y,
        T1 - 4,
        &frozen,
        Tracking::from_clock_offset(-cfo / 5.8e9 * 1e6),
    );
    assert!(against.map(|d| d.psdu) != Ok(psdu.clone()));
    assert_eq!(right.map(|d| d.psdu), Ok(psdu.clone()));
    assert!(wrong.map(|d| d.psdu) != Ok(psdu));
}

#[test]
fn hostile_arguments_do_not_panic() {
    let cfg = RxOfdmConfig::default();
    let y = frame(&[1, 2, 3], Rate::R6, 0x11);
    let h = estimate_channel(&y[T1..T1 + 128]).unwrap();
    // Starts and shifts beyond any stream.
    for start in [usize::MAX, usize::MAX - 100, y.len(), y.len() - 1] {
        assert!(matches!(
            decode_ppdu(&y, start, &cfg),
            Err(RxOfdmError::Truncated { .. })
        ));
        assert!(matches!(
            decode_signal(&y, start, &h, &cfg),
            Err(RxOfdmError::Truncated { .. })
        ));
    }
    for shift in [i64::MAX, i64::MIN, -100_000, 100_000] {
        let t = Tracking {
            shift,
            ..Tracking::default()
        };
        assert!(matches!(
            decode_ppdu_from(&y, T1, &cfg, t),
            Err(RxOfdmError::Truncated { .. })
        ));
    }
    // A window the loop has moved: `needed` names the window actually read.
    let t = Tracking {
        shift: 5000,
        ..Tracking::default()
    };
    assert_eq!(
        decode_ppdu_from(&y, T1, &cfg, t),
        Err(RxOfdmError::Truncated {
            needed: T1 + 128 + 16 + 5000 + 64,
            have: y.len()
        })
    );
    // A length no SIGNAL can carry.
    for length in [0, 4096, usize::MAX / 8, usize::MAX] {
        let r = decode_data_bits(
            &y,
            T1,
            &h,
            Signal {
                rate: Rate::R54,
                length,
            },
            &cfg,
            &mut Tracking::default(),
            &mut SymbolLog::default(),
        );
        assert_eq!(r, Err(RxOfdmError::SignalLength), "{length}");
    }
    // Soft values from a symbol of the wrong size, and with a senseless configuration.
    let short = Equalised {
        points: vec![(1.0, 0.0); 7],
        weights: vec![1.0; 3],
        common_phase: 0.0,
    };
    assert_eq!(soft_bits(&short, Modulation::Qam64, &cfg).len(), 18);
    for (soft_max, clip) in [
        (-1, 1.5),
        (0, 1.5),
        (7, 0.0),
        (7, f64::NAN),
        (i32::MAX, 1e-300),
    ] {
        let c = RxOfdmConfig {
            soft_max,
            clip,
            ..cfg
        };
        let _ = decode_ppdu(&y, T1, &c);
    }
}

#[test]
fn a_slope_only_the_pilots_have_is_not_applied_to_the_data() {
    // The tracker's reason to exist in this form. The channel estimate is exact except on the
    // four pilots, which are tilted as by 0.3 samples: every symbol's pilots then measure that
    // slope, constant in time, and the data have none. A tracker that follows the measurement
    // turns the outer data subcarriers by 0.77 rad and loses a 54 Mb/s frame; one that fits
    // offset + rate·t puts it in the offset and leaves the data alone. (This is what the
    // estimate's noise on the pilots does at low SNR, made large and noise-free.)
    let psdu = random_psdu(1000, &mut Rng::new(0x0f09_000e));
    let y = frame(&psdu, Rate::R54, 0x2f);
    let mut h = estimate_channel(&y[T1..T1 + 128]).unwrap();
    let tilt = 0.3 * 2.0 * PI / 64.0;
    for k in [-21i8, -7, 7, 21] {
        // P = Z·H*: turning H(k) by −k·tilt turns the pilot by +k·tilt.
        let a = -f64::from(k) * tilt;
        let v = h[fft_bin(k)];
        h[fft_bin(k)] = (v.0 * a.cos() - v.1 * a.sin(), v.0 * a.sin() + v.1 * a.cos());
    }
    let cfg = RxOfdmConfig::default();
    for (name, start) in [
        ("nothing known", Tracking::default()),
        ("from a clock offset of 0", Tracking::from_clock_offset(0.0)),
    ] {
        let mut tracking = start;
        let signal = decode_signal(&y, T1, &h, &cfg).unwrap();
        let mut log = SymbolLog::default();
        let bits = decode_data_bits(&y, T1, &h, signal, &cfg, &mut tracking, &mut log).unwrap();
        let sent = data_bits(&psdu, Rate::R54, 0x2f).unwrap();
        let errors = bits.iter().zip(&sent).filter(|(a, b)| a != b).count();
        let worst = log.timing.iter().fold(0.0f64, |m, t| m.max(t.abs()));
        println!(
            "{name}: offset {:.5} (tilt {tilt:.5}), worst drift applied {worst:.4} samples, {errors} bit errors",
            tracking.offset
        );
        assert!((tracking.offset - tilt).abs() < 1e-3, "{name}");
        assert!(worst < 0.01, "{name}: {worst}");
        assert_eq!(errors, 0, "{name}");
    }
    // The prior on the rate is what tells the two starts apart: 2 ppm against 40.
    let ratio = Tracking::from_clock_offset(7.0).covariance[2] / Tracking::default().covariance[2];
    assert!((ratio - (2.0f64 / 40.0).powi(2)).abs() < 1e-12, "{ratio}");
}

#[test]
fn a_weak_pilot_makes_its_pair_worthless() {
    // A channel with nulls near two pilots, one of each pair (+7 and −21): both slope
    // measurements are noise, and a tracker that trusted them would lose frames that a receiver
    // without a tracker decodes (the pre-PR reviewer measured 458 of 600 lost against 0). The
    // measurement's variance comes from the channel gains at the pilots, and a measurement more
    // than three times worse than nominal is not used.
    let mut rng = Rng::new(0x0f09_000f);
    // Three taps whose transfer function has zeros towards subcarriers +7 and −21.
    let zero = |k: f64| {
        let a = 2.0 * PI * k / 64.0;
        (0.97 * a.cos(), 0.97 * a.sin())
    };
    let (z1, z2) = (zero(7.0), zero(-21.0));
    // (1 − z1·D)(1 − z2·D) = 1 − (z1 + z2)·D + z1·z2·D².
    let taps = [
        (1.0, 0.0),
        (-(z1.0 + z2.0), -(z1.1 + z2.1)),
        (z1.0 * z2.0 - z1.1 * z2.1, z1.0 * z2.1 + z1.1 * z2.0),
    ];
    let cfg = RxOfdmConfig::default();
    let off = RxOfdmConfig {
        timing_tracking: false,
        ..cfg
    };
    let (mut lost, mut lost_off, frames) = ([0usize; 2], 0usize, 120);
    for _ in 0..frames {
        let psdu = random_psdu(300, &mut rng);
        let f = frame(&psdu, Rate::R6, 0x39);
        let mut x = vec![(0.0, 0.0); f.len() + 2];
        for (n, s) in f.iter().enumerate() {
            for (d, t) in taps.iter().enumerate() {
                x[n + d].0 += s.0 * t.0 - s.1 * t.1;
                x[n + d].1 += s.0 * t.1 + s.1 * t.0;
            }
        }
        let y = awgn(&x, 8.0, mean_power(&x), &mut rng);
        let ok = |c: &RxOfdmConfig, t: Tracking| {
            decode_ppdu_from(&y, T1 - 2, c, t).is_ok_and(|d| d.psdu == psdu)
        };
        lost[0] += usize::from(!ok(&cfg, Tracking::from_clock_offset(0.0)));
        lost[1] += usize::from(!ok(&cfg, Tracking::default()));
        lost_off += usize::from(!ok(&off, Tracking::default()));
    }
    println!(
        "nulls at two pilots, 6 Mb/s at 8 dB: {} lost from the CFO, {} from zero, {lost_off} without the tracker, of {frames}",
        lost[0], lost[1]
    );
    // Started from the CFO (the designed mode) the tracker must cost nothing here. Started from
    // zero it still loses a few frames: the estimate's own noise hides a null from the
    // gain-based variance (docs/gates/rx_ofdm.md, open questions). Bounded, not excused.
    assert!(lost[0] <= lost_off + 3, "{lost:?} against {lost_off}");
    assert!(lost[1] <= lost_off + 12, "{lost:?} against {lost_off}");
}
