//! G1 gate of the HT LDPC encoder and PPDU encoding process (R0-12, `radio_phy::ldpc`).
//!
//! References we did not write:
//! 1. IEEE Std 802.11-2024 Annex I.2 and I.3, the standard's two LDPC worked examples: the
//!    bits after scrambling, after shortening, after encoding and after puncturing or
//!    repetition, and the derived parameters printed with them. In CI by the SHA-512 of each
//!    table (`fixtures/annex_ldpc_hashes.txt`, made by `tools/ref/annex_ldpc_hash.py` with
//!    CPython's hashlib from a licensed copy; ADR 0005, addendum 2026-11-19); value by value
//!    in the `#[ignore]`d test, which needs the local fixture `target/ext/annex_ldpc.fixture`.
//!    They exercise two of the twelve codes: (1944, 3/4) and (1296, 1/2).
//! 2. The definition itself for all twelve: a codeword is what H checks (§19.3.11.7.3). The
//!    test builds H from the prototypes by a route that shares nothing with the encoder's
//!    back-substitution.
//!
//! 3. An outside copy of the twelve matrix prototypes (github.com/tavildar/LDPC, MIT, read as
//!    data by `tools/ref/ldpc_matrices_outside.py`), by hash: the encoder and the checker of
//!    reference 2 are built from one table, so a transcription error outside the two codes
//!    of the worked examples would pass both. This holds our transcription of Annex F to
//!    someone else's.
//!
//! The rules of Table 19-16 and of step d) that the two examples do not reach are held by
//! `plan_cases_worked_by_hand`: our own second reading of the clause, not an outside one.

#![allow(clippy::unwrap_used)]

use radio_capture::sha512::{hex, sha512};
use radio_phy::channel::Rng;
use radio_phy::ldpc::{LdpcCode, LdpcError, LdpcLength, LdpcPlan, LdpcRate};
use radio_phy::scrambler::Scrambler;
use std::collections::BTreeMap;

const LENGTHS: [LdpcLength; 3] = [LdpcLength::N648, LdpcLength::N1296, LdpcLength::N1944];
const RATES: [LdpcRate; 4] = [
    LdpcRate::R1_2,
    LdpcRate::R2_3,
    LdpcRate::R3_4,
    LdpcRate::R5_6,
];

fn random_bits(n: usize, rng: &mut Rng) -> Vec<u8> {
    (0..n).map(|_| u8::from(rng.uniform() < 0.5)).collect()
}

#[test]
fn twelve_codes_with_the_standard_dimensions() {
    // Table 19-15: information block sizes.
    let k = [
        [324, 432, 486, 540],
        [648, 864, 972, 1080],
        [972, 1296, 1458, 1620],
    ];
    for (li, &l) in LENGTHS.iter().enumerate() {
        for (ri, &r) in RATES.iter().enumerate() {
            let c = LdpcCode::new(l, r).unwrap();
            assert_eq!((c.n(), c.k(), c.z()), (l.bits(), k[li][ri], l.bits() / 24));
            let checks = c.checks();
            assert_eq!(checks.len(), c.n() - c.k());
            // Every bit is in a check; a block column of H holds each of its bits once per
            // non-empty block, so the degrees of the Z bits of a column are equal.
            let mut degree = vec![0usize; c.n()];
            for check in &checks {
                assert!(
                    check.windows(2).all(|w| w[0] < w[1]),
                    "a bit twice in a check"
                );
                for &i in check {
                    degree[i] += 1;
                }
            }
            for block in degree.chunks(c.z()) {
                assert!(block[0] >= 1 && block.iter().all(|&d| d == block[0]));
            }
            // The last parity block is in two checks only: the end of the double diagonal.
            assert!(degree[c.n() - c.z()..].iter().all(|&d| d == 2));
        }
    }
}

#[test]
fn encoder_output_is_a_codeword_of_every_code() {
    let mut rng = Rng::new(0x1d9c);
    for &l in &LENGTHS {
        for &r in &RATES {
            let c = LdpcCode::new(l, r).unwrap();
            assert!(c.is_codeword(&vec![0; c.n()]));
            let mut sum = vec![0u8; c.n()];
            for _ in 0..20 {
                let info = random_bits(c.k(), &mut rng);
                let word = c.encode(&info).unwrap();
                assert_eq!(&word[..c.k()], &info[..], "systematic");
                assert!(c.is_codeword(&word), "{l:?} {r:?}");
                // One wrong bit anywhere is seen by the checks.
                let mut bad = word.clone();
                bad[(rng.uniform() * c.n() as f64) as usize % c.n()] ^= 1;
                assert!(!c.is_codeword(&bad));
                sum.iter_mut().zip(&word).for_each(|(s, w)| *s ^= w);
            }
            // Linear: the sum of codewords is a codeword, and the code of the sum.
            assert!(c.is_codeword(&sum));
            assert_eq!(c.encode(&sum[..c.k()]).unwrap(), sum);
            // Each single information bit alone gives a codeword too (every column of the
            // information part takes part).
            for i in (0..c.k()).step_by(c.z() / 3 + 1) {
                let mut info = vec![0u8; c.k()];
                info[i] = 1;
                assert!(
                    c.is_codeword(&c.encode(&info).unwrap()),
                    "{l:?} {r:?} bit {i}"
                );
            }
        }
    }
}

#[test]
fn prototypes_match_an_outside_copy() {
    let mut seen = 0;
    for line in include_str!("fixtures/ldpc_matrices_outside.txt")
        .lines()
        .filter(|l| !l.starts_with('#'))
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        let l = LENGTHS[[648, 1296, 1944]
            .iter()
            .position(|n| n.to_string() == f[0])
            .unwrap()];
        let r = RATES[["1/2", "2/3", "3/4", "5/6"]
            .iter()
            .position(|r| *r == f[1])
            .unwrap()];
        let c = LdpcCode::new(l, r).unwrap();
        assert_eq!(c.prototype().len().to_string(), f[2], "{l:?} {r:?}: rows");
        let text: String = c
            .prototype()
            .iter()
            .map(|row| {
                let v: Vec<String> = row.iter().map(|s| s.to_string()).collect();
                v.join(" ") + "\n"
            })
            .collect();
        assert_eq!(
            hex(&sha512(text.as_bytes())),
            f[3],
            "{l:?} {r:?}: our prototype is not the outside copy's"
        );
        seen += 1;
    }
    assert_eq!(seen, 12);
}

/// Cases worked by hand from §19.3.11.7.5 for the rules the standard's two examples do not
/// reach. (n_pld, N_CBPS, rate, m_STBC) → (N_CW, L, N_avbits, N_shrt, N_punc, N_rep).
#[test]
fn plan_cases_worked_by_hand() {
    use LdpcLength::{N1296, N1944, N648};
    let cases = [
        // MCS 0, 27 octets: N_avbits = 52·⌈232/26⌉ = 468 ≤ 648 and 468 < 232 + 912/2, so one
        // 648 codeword; N_shrt = 324 − 232 = 92; N_punc = 648 − 468 − 92 = 88 > 0.1·324 and
        // 92 < 1.2·88·1 = 105.6: one more symbol, N_avbits = 520, N_punc = 36.
        ((232, 52, LdpcRate::R1_2, 1), (1, N648, 520, 92, 36, 0)),
        // MCS 0, 8 octets: N_avbits = 208, N_shrt = 244, N_punc = 196. 244 ≥ 1.2·196, so the
        // first condition fails, but 196 > 0.3·324: one more symbol, 260 and 144.
        ((80, 52, LdpcRate::R1_2, 1), (1, N648, 260, 244, 144, 0)),
        // MCS 0, 30 octets: N_avbits = 52·⌈256/26⌉ = 520, N_shrt = 68, N_punc = 60: more than
        // 0.1·324 = 32.4 (but not than 0.2·324) and 68 < 1.2·60 = 72: 572 and 8.
        ((256, 52, LdpcRate::R1_2, 1), (1, N648, 572, 68, 8, 0)),
        // MCS 0, 18 octets: N_avbits = 52·⌈160/26⌉ = 364, N_shrt = 164, N_punc = 120.
        // 164 ≥ 1.2·120 = 144, and 120 > 0.3·324 = 97.2 (but not 0.4·324): 416 and 68.
        ((160, 52, LdpcRate::R1_2, 1), (1, N648, 416, 164, 68, 0)),
        // N_avbits = 312 ≤ 648 at rate 2/3 with 10 payload bits: 312 < 10 + 912/3 = 314, so
        // 648 and not 1296; N_shrt = 432 − 10 = 422, N_rep = 312 − 216 − 10 = 86.
        ((10, 312, LdpcRate::R2_3, 1), (1, N648, 312, 422, 0, 86)),
        // The bounds of Table 19-16's ranges belong to the lower range. N_avbits = 648 exactly
        // (648 < 200 + 456): one 648 codeword, N_shrt = 124, N_rep = 648 − 324 − 200 = 124.
        ((200, 648, LdpcRate::R1_2, 1), (1, N648, 648, 124, 0, 124)),
        // N_avbits = 648·⌈1200/486⌉ = 1944 exactly: one 1944 codeword, not two; N_shrt =
        // 1458 − 1200 = 258, N_rep = 1944 − 486 − 1200 = 258.
        (
            (1200, 648, LdpcRate::R3_4, 1),
            (1, N1944, 1944, 258, 0, 258),
        ),
        // "≥" in the table: N_avbits = 600 = 144 + 912/2 exactly, so 1296. N_shrt = 504,
        // N_punc = 1296 − 600 − 504 = 192: not above 0.3·648 = 194.4, and 504 ≥ 1.2·192.
        ((144, 600, LdpcRate::R1_2, 1), (1, N1296, 600, 504, 192, 0)),
        // 648 < N_avbits = 1296 ≤ 1296 at rate 2/3 with 810 payload bits (no whole number of
        // octets, but the rule is on bits): 1296 < 810 + 1464/3 = 1298, so 1296 and not 1944;
        // N_shrt = 864 − 810 = 54, nothing punctured, N_rep = 1296 − 432 − 810 = 54.
        ((810, 648, LdpcRate::R2_3, 2), (1, N1296, 1296, 54, 0, 54)),
    ];
    for ((n_pld, n_cbps, rate, m), want) in cases {
        let p = LdpcPlan::new(n_pld, n_cbps, rate, m).unwrap();
        assert_eq!(
            (p.n_cw, p.length, p.n_avbits, p.n_shrt, p.n_punc, p.n_rep),
            want,
            "{n_pld} {n_cbps} {rate:?} {m}"
        );
    }
}

#[test]
fn refusals() {
    let c = LdpcCode::new(LdpcLength::N648, LdpcRate::R1_2).unwrap();
    assert_eq!(
        c.encode(&[0; 323]),
        Err(LdpcError::InfoLength {
            got: 323,
            want: 324
        })
    );
    let mut info = vec![0u8; 324];
    info[7] = 2;
    assert_eq!(c.encode(&info), Err(LdpcError::NotABit));
    assert!(!c.is_codeword(&[0; 647]));
    for (pld, cbps, m) in [
        (0, 52, 1),
        (816, 0, 1),
        (816, 52, 0),
        (816, 52, 3),
        (816, 53, 1),
    ] {
        assert_eq!(
            LdpcPlan::new(pld, cbps, LdpcRate::R1_2, m),
            Err(LdpcError::Parameters),
            "{pld} {cbps} {m}"
        );
    }
    // Sizes no PPDU has are refused before any arithmetic on them (no overflow).
    for (pld, cbps, rate) in [
        (usize::MAX, 52, LdpcRate::R1_2),
        (816, usize::MAX, LdpcRate::R2_3),
        (816, usize::MAX / 2 + 2, LdpcRate::R3_4),
        ((1 << 24) + 1, 52, LdpcRate::R1_2),
        (816, (1 << 20) + 2, LdpcRate::R1_2),
    ] {
        assert_eq!(
            LdpcPlan::new(pld, cbps, rate, 1),
            Err(LdpcError::Parameters)
        );
    }
    // A plan put together by hand that does not fit its code is refused, not trusted.
    let good = LdpcPlan::new(816, 208, LdpcRate::R3_4, 1).unwrap();
    for bad in [
        LdpcPlan { n_cw: 0, ..good },
        LdpcPlan {
            n_shrt: good.n_shrt + 1,
            ..good
        },
        LdpcPlan { n_cw: 2, ..good },
        LdpcPlan {
            n_punc: 486,
            ..good
        },
        LdpcPlan {
            length: LdpcLength::N648,
            ..good
        },
    ] {
        assert_eq!(bad.encode(&[0; 816]), Err(LdpcError::Parameters), "{bad:?}");
    }
    let plan = LdpcPlan::new(816, 208, LdpcRate::R3_4, 1).unwrap();
    assert_eq!(
        plan.encode(&[0; 815]),
        Err(LdpcError::PayloadLength {
            got: 815,
            want: 816
        })
    );
}

/// The plan's own arithmetic over many PPDUs: the pieces add up to the symbols, shortening
/// and the choice of code follow Table 19-16, and puncturing and repetition exclude each other.
#[test]
fn plan_invariants() {
    // N_CBPS of the HT MCSs 0…7 at 20 and 40 MHz, one stream, with their rates.
    let mcs = [
        (52, LdpcRate::R1_2),
        (104, LdpcRate::R1_2),
        (104, LdpcRate::R3_4),
        (208, LdpcRate::R1_2),
        (208, LdpcRate::R3_4),
        (312, LdpcRate::R2_3),
        (312, LdpcRate::R3_4),
        (312, LdpcRate::R5_6),
        (108, LdpcRate::R1_2),
        (216, LdpcRate::R1_2),
        (216, LdpcRate::R3_4),
        (432, LdpcRate::R1_2),
        (432, LdpcRate::R3_4),
        (648, LdpcRate::R2_3),
        (648, LdpcRate::R3_4),
        (648, LdpcRate::R5_6),
    ];
    let mut rng = Rng::new(0x1d9d);
    let (mut punctured, mut repeated, mut incremented) = (0, 0, 0);
    for &(n_cbps, rate) in &mcs {
        for m in [1, 2] {
            for length in (1..=400).chain([1000, 1500, 2304, 4095, 65535]) {
                let n_pld = 16 + 8 * length;
                let p = LdpcPlan::new(n_pld, n_cbps, rate, m).unwrap();
                let (num, den) = rate.fraction();
                let (l, k) = (p.length.bits(), p.length.bits() * num / den);
                assert_eq!(p.n_avbits % (n_cbps * m), 0);
                assert_eq!(p.n_sym * n_cbps, p.n_avbits);
                assert_eq!(
                    p.n_cw * k,
                    n_pld + p.n_shrt,
                    "shortening fills the codewords"
                );
                assert!(p.n_punc == 0 || p.n_rep == 0);
                // What is sent is the symbols exactly.
                assert_eq!(p.n_cw * l - p.n_shrt - p.n_punc + p.n_rep, p.n_avbits);
                assert!(p.n_punc < p.n_cw * (l - k), "parity left after puncturing");
                let minimum = n_cbps * m * (n_pld * den).div_ceil(n_cbps * m * num);
                assert!(p.n_avbits == minimum || p.n_avbits == minimum + n_cbps * m);
                incremented += usize::from(p.n_avbits != minimum);
                punctured += usize::from(p.n_punc > 0);
                repeated += usize::from(p.n_rep > 0);
                if length <= 400 && length % 37 == 0 {
                    let payload = random_bits(n_pld, &mut rng);
                    let sent = p.encode(&payload).unwrap();
                    assert_eq!(sent.len(), p.n_avbits);
                    let code = LdpcCode::new(p.length, rate).unwrap();
                    let words = p.codewords(&payload).unwrap();
                    assert!(words.chunks(l).all(|w| code.is_codeword(w)));
                    // The payload is in the sent bits, in order, codeword by codeword.
                    let mut got = Vec::new();
                    let mut at = 0;
                    for cw in 0..p.n_cw {
                        let share = |t: usize| t / p.n_cw + usize::from(cw < t % p.n_cw);
                        let data = k - share(p.n_shrt);
                        got.extend_from_slice(&sent[at..at + data]);
                        at += data + (l - k) - share(p.n_punc) + share(p.n_rep);
                    }
                    assert_eq!(at, sent.len());
                    assert_eq!(got, payload);
                }
            }
        }
    }
    println!("plans with puncturing {punctured}, with repetition {repeated}, with the extra symbol {incremented}");
    assert!(punctured > 1000 && repeated > 1000 && incremented > 100);
}

/// Step e) beyond what example 2 reaches (it repeats 80 of 568 data bits): the repeated bits
/// run on from the data into the parity and, when one pass over the codeword is not enough,
/// start again. Our reading of the clause, with the content checked.
#[test]
fn repetition_goes_through_the_parity_and_round_again() {
    // 8 payload bits in one 648-bit symbol at rate 5/6: N_avbits = 648 ≥ 8 + 912/6, so a 1296
    // codeword; N_shrt = 1080 − 8, nothing punctured, N_rep = 648 − 216 − 8 = 424: more than
    // the 224 bits of the codeword that are sent.
    let p = LdpcPlan::new(8, 648, LdpcRate::R5_6, 1).unwrap();
    assert_eq!(
        (p.n_cw, p.length, p.n_shrt, p.n_punc, p.n_rep),
        (1, LdpcLength::N1296, 1072, 0, 424)
    );
    let payload = [1, 0, 1, 1, 0, 0, 1, 0];
    let word = p.codewords(&payload).unwrap();
    let mut once = payload.to_vec();
    once.extend_from_slice(&word[1080..]);
    assert_eq!(once.len(), 224);
    assert!(
        once[8..].contains(&1),
        "a parity part of zeros shows nothing"
    );
    let want: Vec<u8> = once.iter().cycle().take(224 + 424).copied().collect();
    assert_eq!(p.encode(&payload).unwrap(), want);
}

// ---------------------------------------------------------------------------------------------
// The standard's worked examples
// ---------------------------------------------------------------------------------------------

struct Annex {
    psdu: [Vec<u8>; 2],
    /// Table → (bits, SHA-512 of its canonical text).
    tables: BTreeMap<String, (usize, String)>,
}

fn load() -> Annex {
    let mut a = Annex {
        psdu: [Vec::new(), Vec::new()],
        tables: BTreeMap::new(),
    };
    for line in include_str!("fixtures/annex_ldpc_hashes.txt")
        .lines()
        .filter(|l| !l.starts_with('#'))
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        if let Some(n) = f[0].strip_prefix("psdu") {
            a.psdu[n.parse::<usize>().unwrap() - 1] = (0..f[1].len() / 2)
                .map(|i| u8::from_str_radix(&f[1][2 * i..2 * i + 2], 16).unwrap())
                .collect();
        } else {
            a.tables
                .insert(f[0].to_string(), (f[1].parse().unwrap(), f[2].to_string()));
        }
    }
    assert_eq!(
        (a.psdu[0].len(), a.psdu[1].len(), a.tables.len()),
        (100, 140, 10)
    );
    a
}

fn digest(bits: &[u8]) -> String {
    let text: String = bits
        .iter()
        .enumerate()
        .map(|(n, b)| format!("{n} {b}\n"))
        .collect();
    hex(&sha512(text.as_bytes()))
}

/// The local, uncommitted bits of the tables, when present (`ANNEX_LDPC_FIXTURE` or
/// `target/ext/annex_ldpc.fixture`).
fn local() -> Option<BTreeMap<String, Vec<u8>>> {
    let path = std::env::var("ANNEX_LDPC_FIXTURE").unwrap_or_else(|_| {
        format!(
            "{}/../../target/ext/annex_ldpc.fixture",
            env!("CARGO_MANIFEST_DIR")
        )
    });
    let text = std::fs::read_to_string(path).ok()?;
    Some(
        text.lines()
            .map(|l| {
                let (t, bits) = l.split_once(' ').unwrap();
                (t.to_string(), bits.bytes().map(|c| c - b'0').collect())
            })
            .collect(),
    )
}

/// SERVICE (16 zeros) and the PSDU, LSB of each octet first.
fn clear_bits(psdu: &[u8]) -> Vec<u8> {
    let mut v = vec![0u8; 16];
    v.extend(psdu.iter().flat_map(|b| (0..8).map(move |i| (b >> i) & 1)));
    v
}

fn scramble(bits: &[u8], seed: u8) -> Vec<u8> {
    let mut s = Scrambler::new(seed).unwrap();
    bits.iter().map(|b| b ^ s.next_bit()).collect()
}

/// The five tables of one example from the model: clear, scrambled, shortened, encoded, sent.
fn example(a: &Annex, which: usize, plan: &LdpcPlan, names: [&str; 5]) -> Vec<(String, Vec<u8>)> {
    let clear = clear_bits(&a.psdu[which]);
    // The scrambler seed is not among the printed parameters: it is the one whose output is
    // the printed table, and exactly one of the 127 must be.
    let seeds: Vec<u8> = (1..=0x7f)
        .filter(|&s| digest(&scramble(&clear, s)) == a.tables[names[1]].1)
        .collect();
    assert_eq!(seeds.len(), 1, "{}: seeds {seeds:?}", names[1]);
    let scrambled = scramble(&clear, seeds[0]);
    let tables = [
        clear,
        scrambled.clone(),
        plan.shortened(&scrambled).unwrap(),
        plan.codewords(&scrambled).unwrap(),
        plan.encode(&scrambled).unwrap(),
    ];
    names.iter().map(|n| n.to_string()).zip(tables).collect()
}

fn plans() -> [(LdpcPlan, [&'static str; 5]); 2] {
    // I.2: MCS 4 at 20 MHz (16-QAM 3/4, N_CBPS = 208), 100 octets, no STBC.
    // I.3: MCS 1 at 40 MHz (QPSK 1/2, N_CBPS = 216), 140 octets, STBC.
    [
        (
            LdpcPlan::new(816, 208, LdpcRate::R3_4, 1).unwrap(),
            ["I-32", "I-33", "I-34", "I-35", "I-36"],
        ),
        (
            LdpcPlan::new(1136, 216, LdpcRate::R1_2, 2).unwrap(),
            ["I-38", "I-39", "I-40", "I-41", "I-42"],
        ),
    ]
}

#[test]
fn derived_parameters_of_the_worked_examples() {
    let [(p1, _), (p2, _)] = plans();
    // Printed in I.2.5 and I.3.5.
    assert_eq!(
        (
            p1.n_cw,
            p1.length,
            p1.n_avbits,
            p1.n_shrt,
            p1.n_punc,
            p1.n_sym,
            p1.n_rep
        ),
        (1, LdpcLength::N1944, 1248, 642, 54, 6, 0)
    );
    assert_eq!(
        (
            p2.n_cw,
            p2.length,
            p2.n_avbits,
            p2.n_shrt,
            p2.n_punc,
            p2.n_sym,
            p2.n_rep
        ),
        (2, LdpcLength::N1296, 2592, 160, 0, 12, 160)
    );
}

#[test]
fn worked_examples_by_hash() {
    let a = load();
    for (which, (plan, names)) in plans().iter().enumerate() {
        for (name, bits) in example(&a, which, plan, *names) {
            let (len, sha) = &a.tables[&name];
            assert_eq!(bits.len(), *len, "{name}: length");
            assert_eq!(
                &digest(&bits),
                sha,
                "{name}: not the annex's table (for the first differing bit run \
                 `cargo test -p radio-phy --test g1_ldpc -- --ignored` on a local copy)"
            );
        }
    }
}

#[test]
#[ignore = "needs the locally extracted Annex I.2/I.3 fixture (tools/ref/annex_ldpc_hash.py)"]
fn worked_examples_bit_by_bit() {
    let a = load();
    let annex = local().expect("target/ext/annex_ldpc.fixture");
    for (which, (plan, names)) in plans().iter().enumerate() {
        for (name, bits) in example(&a, which, plan, *names) {
            let want = &annex[&name];
            let first = bits.iter().zip(want).position(|(x, y)| x != y);
            assert_eq!(first, None, "{name}: first differing bit");
            assert_eq!(bits.len(), want.len(), "{name}");
        }
    }
}
