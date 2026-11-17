//! G1 gate for the soft-decision Viterbi decoder (R0-10).
//!
//! References we did not write:
//! 1. The published free distances of the 802.11 code: d_free = 10 at rate 1/2, 6 at 2/3, 5 at
//!    3/4 (K = 7 (133,171)₈ and its punctured forms; e.g. Proakis, Digital Communications,
//!    Table 8.2-1, and the RCPC tables of Hagenauer 1988). A maximum-likelihood decoder of the
//!    terminated code must correct every pattern of ⌊(d_free − 1)/2⌋ hard errors.
//! 2. The definition of maximum likelihood itself: for 12-bit blocks every one of the 4096
//!    codewords is scored by brute force; the decoder must return the best (both terminations).
//! 3. CommPy's Viterbi decoder run as a black box (`tools/ref/viterbi_commpy.py` →
//!    `tests/fixtures/viterbi_ber_commpy.txt`): BER at six (rate, Eb/N0) points.
//! 4. OpenOFDM's receiver output on our frames (`tests/fixtures/ppdu_decode_openofdm.txt`,
//!    R0-06): every frame must decode, including the 9 Mb/s one-octet frame OpenOFDM got wrong —
//!    and forcing OpenOFDM's end-of-frame zero state must reproduce its wrong octet exactly.

use radio_phy::channel::Rng;
use radio_phy::convolutional::{encode, CodeRate, ConvolutionalEncoder};
use radio_phy::ppdu::{data_bits, Rate};
use radio_phy::scrambler::Scrambler;
use radio_phy::viterbi::{decode, depuncture, Termination};

const COMMPY: &str = include_str!("fixtures/viterbi_ber_commpy.txt");
const FRAMES: &str = include_str!("fixtures/ppdu_decode_openofdm.txt");

/// Hard soft values: bit 0 ↦ +s, bit 1 ↦ −s.
fn soft(bits: &[u8], s: i32) -> Vec<i32> {
    bits.iter().map(|&b| if b == 0 { s } else { -s }).collect()
}

fn hex_bytes(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap_or_else(|e| panic!("{s}: {e}")))
        .collect()
}

#[test]
fn depuncture_inserts_erasures_where_bits_were_stolen() {
    let bits: Vec<u8> = (0..12).map(|i| (i % 2) as u8).collect();
    let tx = encode(&bits, CodeRate::R3_4).unwrap();
    let back = depuncture(&soft(&tx, 5), CodeRate::R3_4).unwrap();
    let full = ConvolutionalEncoder::new().encode(&bits);
    assert_eq!(back.len(), full.len());
    for (i, (&s, &c)) in back.iter().zip(&full).enumerate() {
        // Pattern A0 B0 A1 (B1) (A2) B2: positions 3 and 4 of each 6 are stolen.
        if matches!(i % 6, 3 | 4) {
            assert_eq!(s, 0, "position {i}");
        } else {
            assert_eq!(s, if c == 0 { 5 } else { -5 }, "position {i}");
        }
    }
    assert!(depuncture(&[1, 2, 3], CodeRate::R3_4).is_err());
}

#[test]
fn noise_free_frames_decode_at_every_rate() {
    let mut r = Rng::new(20);
    for rate in Rate::ALL {
        for len in [1usize, 17, 100, 400] {
            let psdu: Vec<u8> = (0..len).map(|_| r.next_u64() as u8).collect();
            let seed = (r.next_u64() % 127 + 1) as u8;
            let bits = data_bits(&psdu, rate, seed).unwrap();
            let tx = encode(&bits, rate.code_rate()).unwrap();
            let tail_end = 16 + 8 * len + 6;
            let got = decode(
                &soft(&tx, 64),
                rate.code_rate(),
                Termination::ZeroAfter(tail_end),
            )
            .unwrap();
            assert_eq!(
                got[..tail_end],
                bits[..tail_end],
                "{} Mb/s, {len} octets",
                rate.mbps()
            );
        }
    }
}

#[test]
fn corrects_every_pattern_below_half_the_free_distance() {
    let mut r = Rng::new(21);
    for (rate, d_free) in [
        (CodeRate::R1_2, 10),
        (CodeRate::R2_3, 6),
        (CodeRate::R3_4, 5),
    ] {
        let t = (d_free - 1) / 2;
        for trial in 0..1500 {
            // 294 bits + 6 tail: 600 coded bits, whole periods of both puncturing patterns.
            let mut bits: Vec<u8> = (0..294).map(|_| (r.next_u64() & 1) as u8).collect();
            bits.extend([0; 6]);
            let tx = encode(&bits, rate).unwrap();
            let mut rx = soft(&tx, 1);
            let mut flipped = Vec::new();
            while flipped.len() < t {
                let p = (r.next_u64() % rx.len() as u64) as usize;
                if !flipped.contains(&p) {
                    flipped.push(p);
                    rx[p] = -rx[p];
                }
            }
            let got = decode(&rx, rate, Termination::ZeroAfter(300)).unwrap();
            assert_eq!(got, bits, "{rate:?}, trial {trial}, errors at {flipped:?}");
        }
    }
}

/// Correlation metric Σ s·(1 − 2c) of a transmitted soft stream against coded bits.
fn correlation(rx: &[i32], coded: &[u8]) -> i64 {
    rx.iter()
        .zip(coded)
        .map(|(&s, &c)| i64::from(s) * (1 - 2 * i64::from(c)))
        .sum()
}

#[test]
fn equals_brute_force_maximum_likelihood() {
    let mut r = Rng::new(23);
    for rate in [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4] {
        for terminated in [true, false] {
            // 12 data bits (+ 6 tail zeros when terminated): 24 or 36 coded bits, whole periods.
            let with_tail = |m: u32| {
                let mut b: Vec<u8> = (0..12).map(|i| ((m >> i) & 1) as u8).collect();
                if terminated {
                    b.extend([0; 6]);
                }
                b
            };
            let book: Vec<Vec<u8>> = (0..1u32 << 12)
                .map(|m| encode(&with_tail(m), rate).unwrap())
                .collect();
            let (mut checked, mut ties, mut ml_wrong) = (0, 0, 0);
            for _ in 0..200 {
                let m = (r.next_u64() & 0xfff) as u32;
                // σ = 1.4 (Es/N0 ≈ −2.9 dB): ML itself is often wrong, so agreement is informative.
                let rx: Vec<i32> = book[m as usize]
                    .iter()
                    .map(|&c| {
                        ((if c == 0 { 1.0 } else { -1.0 } + 1.4 * r.gaussian()) * 4096.0).round()
                            as i32
                    })
                    .collect();
                let scores: Vec<i64> = book.iter().map(|cw| correlation(&rx, cw)).collect();
                let best = *scores.iter().max().unwrap();
                let winners: Vec<usize> =
                    (0..scores.len()).filter(|&i| scores[i] == best).collect();
                if winners.len() > 1 {
                    ties += 1;
                    continue;
                }
                if winners[0] != m as usize {
                    ml_wrong += 1;
                }
                let term = if terminated {
                    Termination::ZeroAfter(18)
                } else {
                    Termination::Free
                };
                let got = decode(&rx, rate, term).unwrap();
                assert_eq!(
                    got,
                    with_tail(winners[0] as u32),
                    "{rate:?} terminated={terminated}"
                );
                checked += 1;
            }
            println!("{rate:?} terminated={terminated}: {checked} blocks equal to brute-force ML ({ties} ties skipped, {ml_wrong} where ML ≠ sent)");
            assert!(checked >= 190);
            // The channel must be bad enough that ML itself errs, or the test proves little.
            assert!(
                ml_wrong >= 20,
                "only {ml_wrong} blocks where ML differs from the truth"
            );
        }
    }
}

#[test]
fn the_free_distance_is_reached() {
    // The bound above is tight: the impulse response of the rate-1/2 code has weight 10, so
    // 5 errors on it make the all-zero and the impulse path tie, and 6 flip the decision.
    let mut bits = vec![0u8; 60];
    bits[10] = 1;
    let tx = ConvolutionalEncoder::new().encode(&bits);
    let ones: Vec<usize> = (0..tx.len()).filter(|&i| tx[i] == 1).collect();
    assert_eq!(ones.len(), 10);
    let mut rx = soft(&vec![0u8; tx.len()], 1);
    for &p in &ones[..6] {
        rx[p] = -1;
    }
    let got = decode(&rx, CodeRate::R1_2, Termination::ZeroAfter(60)).unwrap();
    assert_eq!(
        got, bits,
        "6 of the 10 positions flipped must decode as the impulse"
    );
}

#[test]
fn ber_matches_commpy() {
    let mut r = Rng::new(22);
    let points: Vec<&str> = COMMPY.lines().filter(|l| !l.starts_with('#')).collect();
    assert_eq!(points.len(), 6, "fixture must hold six points");
    for line in points {
        let f: Vec<&str> = line.split_whitespace().collect();
        let (rate, r_code) = match f[0] {
            "1/2" => (CodeRate::R1_2, 0.5),
            "2/3" => (CodeRate::R2_3, 2.0 / 3.0),
            "3/4" => (CodeRate::R3_4, 0.75),
            other => panic!("{other}"),
        };
        let ebn0_db: f64 = f[1].parse().unwrap();
        let blocks: usize = f[2].parse().unwrap();
        let n: usize = f[3].parse().unwrap();
        let (their_mean, their_se): (f64, f64) = (f[4].parse().unwrap(), f[5].parse().unwrap());
        let sigma = (1.0 / (2.0 * r_code * 10f64.powf(ebn0_db / 10.0))).sqrt();
        let mut bers = Vec::with_capacity(blocks);
        for _ in 0..blocks {
            let mut bits: Vec<u8> = (0..n).map(|_| (r.next_u64() & 1) as u8).collect();
            bits.extend([0; 6]);
            let tx = encode(&bits, rate).unwrap();
            // Fine quantisation (2^-12): this gate is about the algorithm, not the soft width.
            let rx: Vec<i32> = tx
                .iter()
                .map(|&b| {
                    let x = if b == 0 { 1.0 } else { -1.0 } + sigma * r.gaussian();
                    (x * 4096.0).round() as i32
                })
                .collect();
            let got = decode(&rx, rate, Termination::ZeroAfter(n + 6)).unwrap();
            let errors = got[..n]
                .iter()
                .zip(&bits[..n])
                .filter(|(a, b)| a != b)
                .count();
            bers.push(errors as f64 / n as f64);
        }
        let m = bers.iter().sum::<f64>() / blocks as f64;
        let se = (bers.iter().map(|b| (b - m).powi(2)).sum::<f64>()
            / (blocks - 1) as f64
            / blocks as f64)
            .sqrt();
        let z = (m - their_mean) / (se * se + their_se * their_se).sqrt();
        println!(
            "{} at {ebn0_db} dB: ours {m:.3e} ± {se:.1e}, CommPy {their_mean:.3e} ± {their_se:.1e} (z = {z:+.2})",
            f[0]
        );
        assert!(z.abs() < 5.0, "{} at {ebn0_db} dB: z = {z}", f[0]);
    }
}

fn descramble_psdu(bits: &[u8], seed: u8, len: usize) -> Vec<u8> {
    let clear = Scrambler::new(seed)
        .unwrap_or_else(|e| panic!("{e}"))
        .scramble(bits);
    clear[16..16 + 8 * len]
        .chunks(8)
        .map(|c| c.iter().rev().fold(0u8, |a, &b| (a << 1) | b))
        .collect()
}

#[test]
fn decodes_every_frame_openofdm_saw() {
    let mut n = 0;
    for line in FRAMES
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let (ours, _) = line.split_once(" | ").unwrap();
        let f: Vec<&str> = ours.split_whitespace().collect();
        let mbps: u32 = f[0].parse().unwrap();
        let rate = Rate::ALL.into_iter().find(|r| r.mbps() == mbps).unwrap();
        let seed = u8::from_str_radix(f[1], 16).unwrap();
        let psdu = hex_bytes(f[2]);
        let tx = encode(&data_bits(&psdu, rate, seed).unwrap(), rate.code_rate()).unwrap();
        let tail_end = 16 + 8 * psdu.len() + 6;
        let got = decode(
            &soft(&tx, 64),
            rate.code_rate(),
            Termination::ZeroAfter(tail_end),
        )
        .unwrap();
        assert_eq!(descramble_psdu(&got, seed, psdu.len()), psdu, "{mbps} Mb/s");
        n += 1;
    }
    assert_eq!(n, 24);
}

#[test]
fn openofdm_end_of_frame_assumption_reproduces_its_wrong_octet() {
    // The frame exempted in g1_ppdu_openofdm_decode: 9 Mb/s, seed 0x6e, PSDU e8; OpenOFDM
    // returned 28. Its Viterbi assumes state 0 at the end of the frame, after the scrambled pad.
    let (rate, seed, psdu) = (Rate::R9, 0x6e, vec![0xe8u8]);
    let bits = data_bits(&psdu, rate, seed).unwrap();
    let tx = encode(&bits, rate.code_rate()).unwrap();
    let rx = soft(&tx, 64);
    let right = decode(&rx, rate.code_rate(), Termination::ZeroAfter(16 + 8 + 6)).unwrap();
    assert_eq!(descramble_psdu(&right, seed, 1), psdu);
    let wrong = decode(&rx, rate.code_rate(), Termination::ZeroAfter(bits.len())).unwrap();
    assert_eq!(
        descramble_psdu(&wrong, seed, 1),
        vec![0x28],
        "OpenOFDM's output"
    );
}

#[test]
fn ties_resolve_as_specified() {
    // All erasures: every path ties. The spec (docs/gates/viterbi.md) keeps the predecessor whose
    // dropped bit is 0 and, at a free end, the lowest state — so the answer is all zeros. The RTL
    // must make the same choice, so the rule is pinned here.
    for rate in [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4] {
        let n_tx =
            36 * rate.pattern().iter().filter(|&&k| k == 1).count() / rate.pattern().len() * 2;
        let rx = vec![0i32; n_tx];
        for term in [Termination::Free, Termination::ZeroAfter(10)] {
            let got = decode(&rx, rate, term).unwrap();
            assert_eq!(got, vec![0u8; 36], "{rate:?} {term:?}");
        }
    }
}
