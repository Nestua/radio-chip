//! G1 gate of the HT LDPC decoders (R0-12, `radio_phy::ldpc_decoder`).
//!
//! References we did not write:
//! 1. The check rules against their definition by enumeration: the exact rule is the
//!    probability that the other bits of a check have even parity, computed over all their
//!    2^(d−1) values; min-sum is the same with the best even and the best odd assignment
//!    (max-log). Probability theory, no decoder of ours involved.
//! 2. An outside decoder as a black box: CommPy's `ldpc_bp_decode` (sum-product and
//!    min-sum, flooding), on matrices expanded from the outside copy of the prototypes
//!    (`tools/ref/ldpc_ber_commpy.py` → `fixtures/ldpc_ber_commpy.txt`): bit and block error
//!    rates at ten points of three codes. Ours must agree within the statistics of both.
//! 3. The encoder of R0-12 part 1 (held by the standard's worked examples): what it encodes
//!    must come back, through puncturing, shortening and repetition.
//!
//! The layered schedule and the fixed-point decoder have no outside reference of their own:
//! they are held to the float flooding decoder, which has one (and to their G2 budget, in
//! the journal).

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::ldpc::{LdpcCode, LdpcError, LdpcLength, LdpcPlan, LdpcRate};
use radio_phy::ldpc_decoder::{
    check_min_sum, check_sum_product, CheckRule, LdpcDecoder, MinSumConfig, Schedule,
};

fn random_bits(n: usize, rng: &mut Rng) -> Vec<u8> {
    (0..n).map(|_| u8::from(rng.uniform() < 0.5)).collect()
}

/// BPSK over AWGN at Eb/N0 for a code of rate `rate`: the LLRs of `bits` (0 ↦ +1).
fn channel_llr(bits: &[u8], rate: f64, ebn0_db: f64, rng: &mut Rng) -> Vec<f64> {
    let sigma2 = 1.0 / (2.0 * rate * 10f64.powf(ebn0_db / 10.0));
    bits.iter()
        .map(|&b| {
            let y = 1.0 - 2.0 * f64::from(b) + sigma2.sqrt() * rng.gaussian();
            2.0 * y / sigma2
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------
// The check rules against enumeration
// ---------------------------------------------------------------------------------------------

/// For bit `i` of a check with a-priori LLRs `m`: (exact, max-log) LLR that the other bits
/// have even parity, by enumeration.
fn enumerated(m: &[f64], i: usize) -> (f64, f64) {
    let others: Vec<f64> = m
        .iter()
        .enumerate()
        .filter(|(j, _)| *j != i)
        .map(|(_, &x)| x)
        .collect();
    let (mut p, mut best) = ([0.0f64; 2], [f64::NEG_INFINITY; 2]);
    for assignment in 0u32..1 << others.len() {
        // P(x = 0) = 1/(1 + e^−m); the log-probability of an assignment, up to a constant
        // common to all, is Σ ±m/2.
        let (mut prob, mut metric) = (1.0, 0.0);
        for (j, &x) in others.iter().enumerate() {
            let one = assignment >> j & 1 == 1;
            let p0 = 1.0 / (1.0 + (-x).exp());
            prob *= if one { 1.0 - p0 } else { p0 };
            metric += if one { -x / 2.0 } else { x / 2.0 };
        }
        let parity = (assignment.count_ones() & 1) as usize;
        p[parity] += prob;
        best[parity] = best[parity].max(metric);
    }
    ((p[0] / p[1]).ln(), best[0] - best[1])
}

#[test]
fn check_rules_are_the_enumerated_ones() {
    let mut rng = Rng::new(0x1dc0);
    for trial in 0..400 {
        let d = 2 + trial % 9;
        let mut m: Vec<f64> = (0..d).map(|_| 3.0 * rng.gaussian()).collect();
        if trial % 7 == 0 {
            m[trial % d] = 0.0; // an erasure
        }
        let (exact, min_sum) = (check_sum_product(&m), check_min_sum(&m, 1.0));
        let scaled = check_min_sum(&m, 0.75);
        for i in 0..d {
            let (want_exact, want_max_log) = enumerated(&m, i);
            assert!(
                (exact[i] - want_exact).abs() < 1e-9 * (1.0 + want_exact.abs()),
                "{m:?} bit {i}: {} vs {want_exact}",
                exact[i]
            );
            assert!(
                (min_sum[i] - want_max_log).abs() < 1e-9,
                "{m:?} bit {i}: {} vs {want_max_log}",
                min_sum[i]
            );
            assert!((scaled[i] - 0.75 * want_max_log).abs() < 1e-9);
            // Min-sum never says less than the exact rule, and never the other sign.
            assert!(min_sum[i].abs() >= exact[i].abs() - 1e-9);
            assert!(min_sum[i] * exact[i] >= 0.0);
        }
    }
    // Sure inputs stay finite.
    assert!(check_sum_product(&[1e3, -1e3, 1e3])
        .iter()
        .all(|x| x.is_finite()));
}

// ---------------------------------------------------------------------------------------------
// Error rates against the outside decoder
// ---------------------------------------------------------------------------------------------

struct Point {
    code: LdpcCode,
    rate: f64,
    rule: CheckRule,
    ebn0_db: f64,
    blocks: f64,
    ber: f64,
    ber_se: f64,
    block_errors: f64,
}

fn commpy_points() -> Vec<Point> {
    include_str!("fixtures/ldpc_ber_commpy.txt")
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let f: Vec<&str> = line.split_whitespace().collect();
            let length = match f[0] {
                "648" => LdpcLength::N648,
                "1296" => LdpcLength::N1296,
                _ => LdpcLength::N1944,
            };
            let (rate, r) = match f[1] {
                "1/2" => (LdpcRate::R1_2, 0.5),
                "2/3" => (LdpcRate::R2_3, 2.0 / 3.0),
                "3/4" => (LdpcRate::R3_4, 0.75),
                _ => (LdpcRate::R5_6, 5.0 / 6.0),
            };
            Point {
                code: LdpcCode::new(length, rate).unwrap(),
                rate: r,
                rule: if f[2] == "SPA" {
                    CheckRule::SumProduct
                } else {
                    CheckRule::MinSum { scale: 1.0 }
                },
                ebn0_db: f[3].parse().unwrap(),
                blocks: f[4].parse().unwrap(),
                ber: f[5].parse().unwrap(),
                ber_se: f[6].parse().unwrap(),
                block_errors: f[7].parse().unwrap(),
            }
        })
        .collect()
}

/// (mean BER, its standard error, block error rate) of `blocks` random codewords.
fn error_rates(
    code: &LdpcCode,
    rate: f64,
    ebn0_db: f64,
    blocks: usize,
    rng: &mut Rng,
    decode: impl Fn(&LdpcDecoder, &[f64]) -> Vec<u8>,
) -> (f64, f64, f64) {
    let dec = LdpcDecoder::new(code);
    let (mut sum, mut sum2, mut bad) = (0.0, 0.0, 0usize);
    for _ in 0..blocks {
        let word = code.encode(&random_bits(code.k(), rng)).unwrap();
        let got = decode(&dec, &channel_llr(&word, rate, ebn0_db, rng));
        let e = got.iter().zip(&word).filter(|(a, b)| a != b).count() as f64 / code.n() as f64;
        sum += e;
        sum2 += e * e;
        bad += usize::from(e > 0.0);
    }
    let n = blocks as f64;
    let mean = sum / n;
    let var = (sum2 / n - mean * mean).max(0.0) * n / (n - 1.0);
    (mean, (var / n).sqrt(), bad as f64 / n)
}

#[test]
fn flooding_error_rates_match_commpy() {
    let points = commpy_points();
    assert_eq!(points.len(), 10);
    let mut rng = Rng::new(0x1dc1);
    let mut compared = 0;
    for p in &points {
        let their_bler = p.block_errors / p.blocks;
        // Enough blocks to expect some twenty block errors where the reference has few.
        // LDPC_GATE_BLOCKS sets the same count at every point (the journal's table: 5000).
        let blocks = match std::env::var("LDPC_GATE_BLOCKS") {
            Ok(v) => v.parse().unwrap(),
            Err(_) => ((20.0 / their_bler) as usize).clamp(400, 4000),
        };
        let (ber, se, bler) =
            error_rates(&p.code, p.rate, p.ebn0_db, blocks, &mut rng, |d, llr| {
                d.decode_float(llr, p.rule, Schedule::Flooding, 50)
                    .unwrap()
                    .bits
            });
        // Block errors are binomial on both sides.
        let bler_se =
            (their_bler * (1.0 - their_bler) * (1.0 / blocks as f64 + 1.0 / p.blocks)).sqrt();
        println!(
            "({}, {:.3}) {:?} {} dB: BER {ber:.3e} ± {se:.1e} (CommPy {:.3e} ± {:.1e}), BLER {bler:.3} (CommPy {their_bler:.3})",
            p.code.n(), p.rate, p.rule, p.ebn0_db, p.ber, p.ber_se
        );
        // The bit error rate comes in bursts, one per wrong block: its standard error means
        // something only with enough of them.
        if bler * blocks as f64 >= 10.0 {
            assert!(
                (ber - p.ber).abs() < 4.0 * (se * se + p.ber_se * p.ber_se).sqrt(),
                "BER {ber:e} vs {:e}",
                p.ber
            );
            compared += 1;
        }
        assert!(
            (bler - their_bler).abs() < 4.0 * bler_se + 1e-9,
            "BLER {bler} vs {their_bler}"
        );
        // The points are in the waterfall: an agreement at 0 or at 1 would show nothing.
        assert!(their_bler > 0.003 && their_bler < 0.99, "{their_bler}");
    }
    assert!(
        compared >= 8,
        "bit error rates compared at {compared} points only"
    );
}

// ---------------------------------------------------------------------------------------------
// Schedules, the fixed decoder, the PPDU
// ---------------------------------------------------------------------------------------------

/// How many more blocks the later decoder of a pair may lose alone than the earlier one
/// loses alone, of 400 at 1.75 dB (measured: 1 against 2 and 0 against 0; a layered
/// decoder cut to 12 iterations 22 against 1, an unnormalised one 59 against 3, a fixed
/// one fed 4 units per unit of LLR 20 against 2 of 1000 at 2 dB).
const DISCORDANT_SLACK: usize = 4;

#[test]
fn layered_and_fixed_decoders_are_held_to_the_flooding_one() {
    let code = LdpcCode::new(LdpcLength::N648, LdpcRate::R1_2).unwrap();
    let dec = LdpcDecoder::new(&code);
    let mut rng = Rng::new(0x1dc2);
    let (blocks, ebn0) = (400, 1.75);
    let scale = CheckRule::MinSum { scale: 0.75 };
    // [flooding 50, layered 25, layered 50, fixed 25]: block errors, iterations of the
    // converged blocks.
    let mut bad = [0usize; 4];
    let mut iters = [0usize; 4];
    // The four decoders see the same noise, so most blocks fail in all or in none; what
    // tells two decoders apart is the blocks only one of them gets wrong.
    // only[a][b]: blocks decoder a gets wrong and decoder b right.
    let mut only = [[0usize; 4]; 4];
    for _ in 0..blocks {
        let word = code.encode(&random_bits(code.k(), &mut rng)).unwrap();
        let llr = channel_llr(&word, 0.5, ebn0, &mut rng);
        // 64 units per unit of LLR for the fixed decoder: fine enough for its rounding not
        // to matter. At 8 units it already does (the scaled minimum is rounded down, and
        // the decoder loses blocks the float one keeps): G2's question, in the journal.
        let fixed_in: Vec<i32> = llr.iter().map(|x| (64.0 * x).round() as i32).collect();
        let results = [
            dec.decode_float(&llr, scale, Schedule::Flooding, 50)
                .unwrap(),
            dec.decode_float(&llr, scale, Schedule::Layered, 25)
                .unwrap(),
            dec.decode_float(&llr, scale, Schedule::Layered, 50)
                .unwrap(),
            dec.decode_fixed(
                &fixed_in,
                &MinSumConfig {
                    max_iter: 25,
                    msg_bits: 16,
                    app_bits: 18,
                    ..MinSumConfig::default()
                },
            )
            .unwrap(),
        ];
        for (i, r) in results.iter().enumerate() {
            assert_eq!(r.converged, code.is_codeword(&r.bits));
            let wrong = r.bits != word;
            bad[i] += usize::from(wrong);
            if !wrong {
                iters[i] += r.iterations;
            }
            // A converged decoder may still be wrong (another codeword), but never reports
            // convergence on a non-codeword; an unconverged one is wrong.
            assert!(r.converged || wrong);
        }
        for a in 0..4 {
            for b in 0..4 {
                only[a][b] += usize::from(results[a].bits != word && results[b].bits == word);
            }
        }
    }
    let mean = |i: usize| iters[i] as f64 / (blocks - bad[i]) as f64;
    println!(
        "block errors of {blocks}: flooding/50 {}, layered/25 {}, layered/50 {}, fixed/25 {}; mean iterations {:.1}, {:.1}, {:.1}, {:.1}",
        bad[0], bad[1], bad[2], bad[3], mean(0), mean(1), mean(2), mean(3)
    );
    println!(
        "wrong in one only: layered/25 {} against flooding/50 {}; fixed/25 {} against layered/25 {}",
        only[1][0], only[0][1], only[3][1], only[1][3]
    );
    // Layered needs about half the iterations of flooding, so 25 of it lose no more blocks
    // than 50 of flooding, and the fixed decoder follows: of the blocks only one of a pair
    // gets wrong, the later decoder may have DISCORDANT_SLACK more than the earlier.
    assert!(only[1][0] <= only[0][1] + DISCORDANT_SLACK, "{only:?}");
    assert!(only[3][1] <= only[1][3] + DISCORDANT_SLACK, "{only:?}");
    assert!(
        mean(1) < 0.7 * mean(0),
        "layered {} flooding {}",
        mean(1),
        mean(0)
    );
    assert!((mean(3) - mean(1)).abs() < 0.15 * mean(1));
    assert!(
        bad[0] < blocks / 4,
        "the point is past the waterfall's start"
    );
}

/// The fixed decoder as its definition reads (journal, G0), written a second time without
/// any of the decoder's shortcuts: every message recomputed from all the others.
fn fixed_by_definition(code: &LdpcCode, llr: &[i32], cfg: &MinSumConfig) -> (Vec<u8>, usize) {
    let app_max = (1 << (cfg.app_bits - 1)) - 1;
    let msg_max = (1 << (cfg.msg_bits - 1)) - 1;
    let sat = |x: i64, m: i64| x.max(-m).min(m);
    let checks = code.checks();
    let mut total: Vec<i64> = llr.iter().map(|&x| sat(i64::from(x), app_max)).collect();
    let mut last: Vec<Vec<i64>> = checks.iter().map(|c| vec![0; c.len()]).collect();
    let bits = |t: &[i64]| -> Vec<u8> { t.iter().map(|&x| u8::from(x < 0)).collect() };
    let mut passes = 0;
    while passes < cfg.max_iter && !code.is_codeword(&bits(&total)) {
        for (c, check) in checks.iter().enumerate() {
            let msg: Vec<i64> = check
                .iter()
                .enumerate()
                .map(|(i, &b)| sat(total[b] - last[c][i], app_max))
                .collect();
            for (i, &b) in check.iter().enumerate() {
                let others = || {
                    msg.iter()
                        .enumerate()
                        .filter(move |(j, _)| *j != i)
                        .map(|(_, &m)| m)
                };
                let min = others().map(i64::abs).min().unwrap();
                let minus = others().filter(|&m| m < 0).count() % 2 == 1;
                let mag = ((min * i64::from(cfg.scale_num)) >> cfg.scale_shift).min(msg_max);
                let new = if minus { -mag } else { mag };
                total[b] = sat(msg[i] + new, app_max);
                last[c][i] = new;
            }
        }
        passes += 1;
    }
    (bits(&total), passes)
}

#[test]
fn fixed_decoder_is_its_written_definition() {
    let mut rng = Rng::new(0x1dc4);
    let configs = [
        MinSumConfig::default(),
        // Narrow widths: both saturations work all the time.
        MinSumConfig {
            max_iter: 6,
            scale_num: 3,
            scale_shift: 2,
            msg_bits: 3,
            app_bits: 4,
        },
        MinSumConfig {
            max_iter: 6,
            scale_num: 13,
            scale_shift: 4,
            msg_bits: 4,
            app_bits: 4,
        },
        MinSumConfig {
            max_iter: 4,
            scale_num: 1,
            scale_shift: 0,
            msg_bits: 2,
            app_bits: 6,
        },
        MinSumConfig {
            max_iter: 5,
            scale_num: 7,
            scale_shift: 3,
            msg_bits: 5,
            app_bits: 6,
        },
    ];
    let (mut saturated, mut unconverged) = (0, 0);
    for (l, r) in [
        (LdpcLength::N648, LdpcRate::R1_2),
        (LdpcLength::N648, LdpcRate::R5_6),
        (LdpcLength::N1296, LdpcRate::R2_3),
    ] {
        let code = LdpcCode::new(l, r).unwrap();
        let dec = LdpcDecoder::new(&code);
        let (num, den) = r.fraction();
        for cfg in &configs {
            for trial in 0..6 {
                let word = code.encode(&random_bits(code.k(), &mut rng)).unwrap();
                // Around the waterfall and below it, at several input scales.
                let ebn0 = [0.5, 1.5, 2.5, 3.5][trial % 4];
                let gain = [0.5, 1.0, 3.0][trial % 3];
                let llr: Vec<i32> = channel_llr(&word, num as f64 / den as f64, ebn0, &mut rng)
                    .iter()
                    .map(|x| (gain * x).round() as i32)
                    .collect();
                let app_max = (1 << (cfg.app_bits - 1)) - 1;
                saturated += llr.iter().filter(|x| x.abs() > app_max).count();
                let got = dec.decode_fixed(&llr, cfg).unwrap();
                let (bits, passes) = fixed_by_definition(&code, &llr, cfg);
                assert_eq!(
                    (got.bits, got.iterations),
                    (bits, passes),
                    "{l:?} {r:?} {cfg:?}"
                );
                unconverged += usize::from(!got.converged);
            }
        }
    }
    assert!(
        saturated > 1000 && unconverged > 5,
        "{saturated} {unconverged}"
    );
}

#[test]
fn a_ppdu_comes_back_through_shortening_puncturing_and_repetition() {
    let mut rng = Rng::new(0x1dc3);
    let cases = [
        (816, 208, LdpcRate::R3_4, 1),  // the standard's example 1: punctured
        (1136, 216, LdpcRate::R1_2, 2), // example 2: two codewords, repeated
        (232, 52, LdpcRate::R1_2, 1),   // the extra symbol
        (8, 648, LdpcRate::R5_6, 1),    // repetition round the codeword
        (16 + 8 * 1500, 312, LdpcRate::R5_6, 1), // many 1944 codewords
        (16 + 8 * 700, 648, LdpcRate::R2_3, 2),
    ];
    for (n_pld, n_cbps, rate, m) in cases {
        let plan = LdpcPlan::new(n_pld, n_cbps, rate, m).unwrap();
        let code = LdpcCode::new(plan.length, rate).unwrap();
        let dec = LdpcDecoder::new(&code);
        let payload = random_bits(n_pld, &mut rng);
        let sent = plan.encode(&payload).unwrap();
        let (num, den) = rate.fraction();
        // 3 dB above the worst waterfall of the unpunctured code: the puncturing costs some.
        let llr = channel_llr(&sent, num as f64 / den as f64, 6.0, &mut rng);
        // Float: sum-product, flooding.
        let soft = plan.soft_codewords(&llr, 100.0).unwrap();
        assert_eq!(soft.len(), plan.n_cw * code.n());
        let mut words = Vec::new();
        for w in soft.chunks(code.n()) {
            let d = dec
                .decode_float(w, CheckRule::SumProduct, Schedule::Flooding, 50)
                .unwrap();
            assert!(d.converged, "{n_pld} {n_cbps} {rate:?}");
            words.extend(d.bits);
        }
        assert_eq!(words, plan.codewords(&payload).unwrap());
        assert_eq!(plan.payload(&words).unwrap(), payload);
        // Fixed: the same through the integer decoder.
        let fixed: Vec<i32> = llr.iter().map(|x| (2.0 * x).round() as i32).collect();
        let soft = plan.soft_codewords(&fixed, 511).unwrap();
        let mut words = Vec::new();
        for w in soft.chunks(code.n()) {
            words.extend(dec.decode_fixed(w, &MinSumConfig::default()).unwrap().bits);
        }
        assert_eq!(
            plan.payload(&words).unwrap(),
            payload,
            "fixed {n_pld} {n_cbps} {rate:?}"
        );
    }
}

#[test]
fn soft_codewords_put_every_value_in_its_place() {
    // Example 2's shape: two (1296, 1/2) codewords, 80 shortened and 80 repeated bits each.
    let plan = LdpcPlan::new(1136, 216, LdpcRate::R1_2, 2).unwrap();
    // Each sent position carries its own index + 1, so the mapping is visible.
    let soft: Vec<i32> = (1..=plan.n_avbits as i32).collect();
    let words = plan.soft_codewords(&soft, -7).unwrap();
    assert_eq!(words.len(), 2592);
    for cw in 0..2 {
        let (w, base) = (&words[cw * 1296..(cw + 1) * 1296], (cw * 1296) as i32);
        // 568 data, 648 parity, 80 repeats of the first 80 data bits: sent positions
        // base + 1 …; the repeat of bit i is at base + 568 + 648 + i + 1.
        for (i, &v) in w.iter().enumerate().take(568) {
            let sent = base + i as i32 + 1;
            let repeat = if i < 80 {
                base + 1216 + i as i32 + 1
            } else {
                0
            };
            assert_eq!(v, sent + repeat, "data bit {i}");
        }
        assert!(w[568..648].iter().all(|&v| v == -7), "shortened bits");
        for i in 0..648 {
            assert_eq!(w[648 + i], base + 568 + i as i32 + 1);
        }
    }
    // Example 1's shape: punctured parity is an erasure at the end of the codeword.
    let plan = LdpcPlan::new(816, 208, LdpcRate::R3_4, 1).unwrap();
    let soft: Vec<i32> = (1..=1248).collect();
    let w = plan.soft_codewords(&soft, -7).unwrap();
    assert_eq!(
        (w[815], w[816], w[1457], w[1458], w[1889], w[1890], w[1943]),
        (816, -7, -7, 817, 1248, 0, 0)
    );
}

#[test]
fn refusals() {
    let code = LdpcCode::new(LdpcLength::N648, LdpcRate::R1_2).unwrap();
    let dec = LdpcDecoder::new(&code);
    let rule = CheckRule::SumProduct;
    assert_eq!(
        dec.decode_float(&[0.0; 647], rule, Schedule::Flooding, 5),
        Err(LdpcError::SoftLength {
            got: 647,
            want: 648
        })
    );
    let mut llr = vec![1.0; 648];
    llr[3] = f64::NAN;
    assert_eq!(
        dec.decode_float(&llr, rule, Schedule::Layered, 5),
        Err(LdpcError::NotANumber)
    );
    for scale in [0.0, -0.75, 1.5, f64::NAN, f64::INFINITY] {
        assert_eq!(
            dec.decode_float(
                &[1.0; 648],
                CheckRule::MinSum { scale },
                Schedule::Layered,
                5
            ),
            Err(LdpcError::DecoderConfig),
            "scale {scale}"
        );
    }
    assert_eq!(
        dec.decode_fixed(&[0; 647], &MinSumConfig::default()),
        Err(LdpcError::SoftLength {
            got: 647,
            want: 648
        })
    );
    for bad in [
        MinSumConfig {
            msg_bits: 1,
            ..MinSumConfig::default()
        },
        MinSumConfig {
            app_bits: 7,
            ..MinSumConfig::default()
        },
        MinSumConfig {
            app_bits: 25,
            ..MinSumConfig::default()
        },
        MinSumConfig {
            scale_num: 0,
            ..MinSumConfig::default()
        },
        MinSumConfig {
            scale_num: 5,
            ..MinSumConfig::default()
        },
        MinSumConfig {
            scale_shift: 9,
            ..MinSumConfig::default()
        },
    ] {
        assert!(!bad.is_valid());
        assert_eq!(
            dec.decode_fixed(&[1; 648], &bad),
            Err(LdpcError::DecoderConfig)
        );
    }
    // Extreme integers saturate; all-erasure input ends without convergence trouble.
    let d = dec
        .decode_fixed(&[i32::MIN; 648], &MinSumConfig::default())
        .unwrap();
    assert_eq!(d.bits, vec![1; 648]);
    let d = dec
        .decode_fixed(&[0; 648], &MinSumConfig::default())
        .unwrap();
    assert!(d.converged && d.iterations == 0 && d.bits == vec![0; 648]);
    // The plan's mirrors.
    let plan = LdpcPlan::new(816, 208, LdpcRate::R3_4, 1).unwrap();
    assert_eq!(
        plan.soft_codewords(&[0i32; 1247], 1),
        Err(LdpcError::SoftLength {
            got: 1247,
            want: 1248
        })
    );
    assert_eq!(
        plan.payload(&[0; 1943]),
        Err(LdpcError::SoftLength {
            got: 1943,
            want: 1944
        })
    );
    let broken = LdpcPlan { n_cw: 0, ..plan };
    assert_eq!(
        broken.soft_codewords(&[0i32; 1248], 1),
        Err(LdpcError::Parameters)
    );
    assert_eq!(broken.payload(&[0; 1944]), Err(LdpcError::Parameters));
    // Hand-made plans whose pieces do not add up to N_avbits are refused before anything
    // is allocated or indexed, however large their numbers.
    for broken in [
        LdpcPlan {
            n_avbits: 1247,
            ..plan
        },
        LdpcPlan {
            n_punc: plan.n_punc + 1,
            ..plan
        },
        LdpcPlan { n_rep: 1, ..plan },
        LdpcPlan {
            n_cw: usize::MAX / 1944,
            n_shrt: (usize::MAX / 1944) * 1458 - 816,
            ..plan
        },
        LdpcPlan {
            n_cw: usize::MAX,
            ..plan
        },
    ] {
        let soft = vec![0i32; broken.n_avbits];
        assert_eq!(
            broken.soft_codewords(&soft, 1),
            Err(LdpcError::Parameters),
            "{broken:?}"
        );
        assert_eq!(
            broken.payload(&[0; 1944]),
            Err(LdpcError::Parameters),
            "{broken:?}"
        );
        assert_eq!(
            broken.encode(&[0; 816]),
            Err(LdpcError::Parameters),
            "{broken:?}"
        );
    }
}

#[test]
fn known_bits_may_be_infinite() {
    // A codeword with three bits in ten erased and the rest known for sure: every rule and
    // schedule must fill the erasures in. Infinite values inside the decoder would meet as
    // inf − inf (min-sum once answered all-zero and "converged" here).
    let code = LdpcCode::new(LdpcLength::N648, LdpcRate::R1_2).unwrap();
    let dec = LdpcDecoder::new(&code);
    let mut rng = Rng::new(0x1dc5);
    for trial in 0..8 {
        let word = code.encode(&random_bits(code.k(), &mut rng)).unwrap();
        let sure = if trial % 2 == 0 { f64::INFINITY } else { 1e300 };
        let llr: Vec<f64> = word
            .iter()
            .map(|&b| {
                if rng.uniform() < 0.3 {
                    0.0
                } else if b == 0 {
                    sure
                } else {
                    -sure
                }
            })
            .collect();
        assert!(llr.iter().filter(|x| **x == 0.0).count() > 150);
        for rule in [
            CheckRule::SumProduct,
            CheckRule::MinSum { scale: 1.0 },
            CheckRule::MinSum { scale: 0.75 },
        ] {
            for schedule in [Schedule::Flooding, Schedule::Layered] {
                let d = dec.decode_float(&llr, rule, schedule, 50).unwrap();
                assert!(d.converged && d.iterations > 0, "{rule:?} {schedule:?}");
                assert_eq!(d.bits, word, "{rule:?} {schedule:?} trial {trial}");
            }
        }
    }
}

#[test]
fn repeated_values_saturate_and_never_wrap() {
    // 8 payload bits in a 648-bit symbol: 224 bits of a 1296 codeword, each sent two or
    // three times.
    let plan = LdpcPlan::new(8, 648, LdpcRate::R5_6, 1).unwrap();
    assert!(plan.n_rep >= plan.n_avbits - plan.n_rep);
    for (value, zero) in [(i32::MAX, i32::MAX), (i32::MIN, i32::MAX), (1 << 30, 5)] {
        let words = plan
            .soft_codewords(&vec![value; plan.n_avbits], zero)
            .unwrap();
        let want = if value > 0 { i32::MAX } else { i32::MIN };
        let (sent, shortened, punctured) = words.iter().fold((0, 0, 0), |(s, z, p), &v| {
            (
                s + usize::from(v == want),
                z + usize::from(v == zero && zero != want),
                p + usize::from(v == 0),
            )
        });
        assert_eq!(punctured, plan.n_punc);
        if zero == want {
            assert_eq!(sent, plan.length.bits() - plan.n_punc);
        } else {
            assert_eq!((sent, shortened), (plan.n_avbits - plan.n_rep, plan.n_shrt));
        }
    }
}
