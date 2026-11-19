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
        ..RxOfdmConfig::default()
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
    // A receiver clock fast by p ppm reads the signal p·1e-6·Δn samples early Δn samples after
    // the training field: the reference for the tracked timing. ±40 ppm is the standard's worst
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
            assert!(worst < 0.05, "{rate:?} at {ppm} ppm: {worst}");
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
