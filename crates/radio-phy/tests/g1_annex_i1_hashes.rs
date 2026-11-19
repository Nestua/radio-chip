//! G1 gate against IEEE Std 802.11-2024 Annex I.1 (Example 1 — BCC encoding) that runs in CI:
//! the standard's worked example by the hashes of its tables.
//!
//! The annex's tables are not in the repository (ADR 0005, addendum 2026-11-19). The fixture
//! `fixtures/annex_i1_hashes.txt` holds the SHA-512 of a canonical text of each table, made
//! from a licensed copy by `tools/ref/annex_i1_extract.py` and `annex_i1_hash.py` (CPython's
//! hashlib), and the example's input, the 100-octet PSDU of Table I-1. Here the model
//! computes every table from that PSDU, writes the same canonical text and must arrive at the
//! same hash: equal to the print in every bit and in every third decimal.
//!
//! A hash says "different", not where: for that, extract the annex locally and run
//! `cargo test -p radio-phy --test g1_annex_i1 -- --ignored`, which compares value by value
//! (and also holds the 13-bit transmitter against the print within its tolerance; a hash can
//! only hold the 24-bit one, whose arithmetic error is far below the print's rounding).

#![allow(clippy::unwrap_used)]

use radio_capture::sha512::{hex, sha512};
use radio_phy::convolutional::{encode, CodeRate};
use radio_phy::interleaver::{interleave, interleaved_index};
use radio_phy::mapper::{map_bits, mean_energy};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{
    data_subcarriers, pilot_polarity, IfftConfig, PILOT_BASE, PILOT_SUBCARRIERS,
};
use radio_phy::ppdu::{build_ppdu, data_bits, signal_bits, Rate};
use radio_phy::rx_ofdm::{decode_ppdu, RxOfdmConfig};
use radio_phy::scrambler::Scrambler;
use std::collections::BTreeMap;

/// What the fixture says of one table: entries, first index, hash.
struct Table {
    entries: usize,
    first: i64,
    sha512: String,
}

struct Annex {
    psdu: Vec<u8>,
    tables: BTreeMap<String, Table>,
}

fn load() -> Annex {
    let text = include_str!("fixtures/annex_i1_hashes.txt");
    let mut a = Annex {
        psdu: Vec::new(),
        tables: BTreeMap::new(),
    };
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f[0] == "psdu" {
            a.psdu = (0..f[1].len() / 2)
                .map(|i| u8::from_str_radix(&f[1][2 * i..2 * i + 2], 16).unwrap())
                .collect();
        } else {
            a.tables.insert(
                f[0].to_string(),
                Table {
                    entries: f[1].parse().unwrap(),
                    first: f[2].parse().unwrap(),
                    sha512: f[3].to_string(),
                },
            );
        }
    }
    assert_eq!(a.psdu.len(), 100);
    assert_eq!(a.tables.len(), 21);
    a
}

/// The canonical text of a table: one line per entry, the index and then the values.
fn text<I: std::fmt::Display, V: AsRef<str>>(entries: impl IntoIterator<Item = (I, V)>) -> String {
    entries
        .into_iter()
        .map(|(i, v)| format!("{i} {}\n", v.as_ref()))
        .collect()
}

fn bits_text(bits: &[u8]) -> String {
    text(bits.iter().enumerate().map(|(i, b)| (i, b.to_string())))
}

/// Three decimals as the annex prints them; a value that rounds to zero has no sign. A
/// value exactly between two prints (odd/16) goes to the even digit, as `{:.3}` does on the
/// exact binary value: the annex's hashes hold with that rule (12 such values).
fn f3(v: f64) -> String {
    let s = format!("{v:.3}");
    if s == "-0.000" {
        "0.000".into()
    } else {
        s
    }
}

fn iq_text(first: i64, values: &[Option<(f64, f64)>]) -> String {
    text(values.iter().enumerate().map(|(i, v)| {
        (
            first + i as i64,
            match v {
                Some((re, im)) => format!("{} {}", f3(*re), f3(*im)),
                None => "X X".into(),
            },
        )
    }))
}

#[track_caller]
fn check(a: &Annex, table: &str, ours: &str) {
    let t = &a.tables[table];
    assert_eq!(ours.lines().count(), t.entries, "{table}: entries");
    assert_eq!(
        hex(&sha512(ours.as_bytes())),
        t.sha512,
        "{table}: the model's table is not the annex's (compare value by value with \
         `cargo test -p radio-phy --test g1_annex_i1 -- --ignored` on a local copy)"
    );
}

/// Subcarriers −32…31 of one symbol after mapping; pilots empty (`None`) or with the
/// polarity of symbol `polarity`.
fn freq(bits: &[u8], m: Modulation, polarity: Option<usize>) -> Vec<Option<(f64, f64)>> {
    let k = 1.0 / f64::from(mean_energy(m)).sqrt();
    let mut v: BTreeMap<i64, Option<(f64, f64)>> =
        (-32..32).map(|c| (c, Some((0.0, 0.0)))).collect();
    for (&c, p) in data_subcarriers().iter().zip(map_bits(bits, m).unwrap()) {
        v.insert(c.into(), Some((f64::from(p.i) * k, f64::from(p.q) * k)));
    }
    for (&c, &base) in PILOT_SUBCARRIERS.iter().zip(&PILOT_BASE) {
        v.insert(
            c.into(),
            polarity.map(|n| (f64::from(pilot_polarity(n) * base), 0.0)),
        );
    }
    v.into_values().collect()
}

/// The seed whose 127-bit sequence is Table I-14: exactly one of the 127 must be.
fn seed(a: &Annex) -> u8 {
    let seeds: Vec<u8> = (1..=0x7f)
        .filter(|&s| {
            let mut sc = Scrambler::new(s).unwrap();
            let seq: Vec<u8> = (0..127).map(|_| sc.next_bit()).collect();
            hex(&sha512(bits_text(&seq).as_bytes())) == a.tables["I-14"].sha512
        })
        .collect();
    assert_eq!(seeds.len(), 1, "I-14: seeds giving the sequence: {seeds:?}");
    seeds[0]
}

#[test]
fn signal_field() {
    let a = load();
    let sig = signal_bits(Rate::R36, 100).unwrap();
    check(&a, "I-7", &bits_text(&sig));
    let coded = encode(&sig, CodeRate::R1_2).unwrap();
    check(&a, "I-8", &bits_text(&coded));
    let inter = interleave(&coded, Modulation::Bpsk).unwrap();
    check(&a, "I-9", &bits_text(&inter));
    check(
        &a,
        "I-10",
        &iq_text(-32, &freq(&inter, Modulation::Bpsk, None)),
    );
    check(
        &a,
        "I-11",
        &iq_text(-32, &freq(&inter, Modulation::Bpsk, Some(0))),
    );
}

#[test]
fn data_bits_and_scrambler() {
    let a = load();
    // I-13: SERVICE (16 zeros), PSDU LSB first, 6 tail zeros, pad zeros to 6 × 144 bits.
    let mut clear = vec![0u8; 16];
    clear.extend(
        a.psdu
            .iter()
            .flat_map(|b| (0..8).map(move |i| (b >> i) & 1)),
    );
    clear.resize(864, 0);
    check(&a, "I-13", &bits_text(&clear));

    let seed = seed(&a);
    assert_eq!(seed, 0x5d);
    let scrambled = data_bits(&a.psdu, Rate::R36, seed).unwrap();
    check(&a, "I-15", &bits_text(&scrambled));
    let coded = encode(&scrambled, CodeRate::R3_4).unwrap();
    check(&a, "I-16", &bits_text(&coded));

    // I-17 and I-18 are the two permutations; the model has their composition.
    let perm = text((0..192).map(|k| (k, interleaved_index(k, Modulation::Qam16).to_string())));
    check(&a, "I-17+I-18", &perm);
    let inter = interleave(&coded[..192], Modulation::Qam16).unwrap();
    check(&a, "I-19", &bits_text(&inter));
    check(
        &a,
        "I-20",
        &iq_text(-32, &freq(&inter, Modulation::Qam16, Some(1))),
    );
}

/// The annex windows each field (W = ½ at both ends) and overlaps the extra sample with the
/// next field: a boundary sample b is ½·(x[b] + x[b − 64]) (every field is periodic in 64 at
/// its end). Sample 0 and the trailing sample 880 have one side only.
fn windowed(frame: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let boundaries = [0usize, 160, 320, 400, 480, 560, 640, 720, 800, 880];
    let n = frame.len();
    (0..=n)
        .map(|i| {
            if !boundaries.contains(&i) {
                return frame[i];
            }
            let own = if i < n { frame[i] } else { (0.0, 0.0) };
            let prev = if i >= 64 { frame[i - 64] } else { (0.0, 0.0) };
            (0.5 * (own.0 + prev.0), 0.5 * (own.1 + prev.1))
        })
        .collect()
}

/// The whole PPDU from the 24-bit transmitter, windowed as the annex does: 881 samples.
fn waveform(a: &Annex) -> Vec<(f64, f64)> {
    let cfg = IfftConfig::with_word_bits(24).unwrap();
    let lsb = cfg.data.lsb();
    let frame: Vec<(f64, f64)> = build_ppdu(&a.psdu, Rate::R36, seed(a), &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * lsb, s.im as f64 * lsb))
        .collect();
    assert_eq!(frame.len(), 880);
    windowed(&frame)
}

/// The model's 881 samples, after their hashes are found to be those of Tables I-22 … I-30.
fn printed_waveform(a: &Annex) -> Vec<(f64, f64)> {
    let ours = waveform(a);
    let mut covered = 0usize;
    for n in 22..=30 {
        let name = format!("I-{n}");
        let t = &a.tables[&name];
        assert_eq!(
            t.first as usize, covered,
            "{name}: tables follow each other"
        );
        let part: Vec<_> = ours[covered..covered + t.entries]
            .iter()
            .map(|&v| Some(v))
            .collect();
        check(a, &name, &iq_text(t.first, &part));
        covered += t.entries;
    }
    assert_eq!(covered, 881);
    ours
}

#[test]
fn whole_ppdu_is_the_printed_waveform() {
    let a = load();
    let ours = printed_waveform(&a);
    // How far the values are from a rounding boundary of the print (…5 in the fourth
    // decimal). The transmitter is integer arithmetic, so the same on every machine; the
    // margin says how much a change of that arithmetic may move a sample before the hash
    // notices (the local value-by-value gate then says whether the print still holds).
    // Values exactly on a boundary are dyadic (odd multiples of 1/16, such as 0.0625).
    let (mut on_boundary, mut nearest) = (0, f64::MAX);
    for v in &ours {
        for x in [v.0, v.1] {
            let d = ((x * 1000.0).rem_euclid(1.0) - 0.5).abs() / 1000.0;
            if d == 0.0 {
                on_boundary += 1;
            } else {
                nearest = nearest.min(d);
            }
        }
    }
    println!("{on_boundary} of 1762 values exactly on a rounding boundary of the print");
    println!("nearest other value to a boundary: {nearest:.2e}");
    assert_eq!(on_boundary, 12);
    assert!(nearest > 2e-7, "{nearest:e}");
}

/// The receive half: the printed waveform (the model's, checked here against the hashes of
/// Tables I-22 … I-30 and rounded to the printed digits) decodes to the annex's PSDU.
#[test]
fn printed_waveform_decodes() {
    let a = load();
    let y: Vec<(f64, f64)> = printed_waveform(&a)
        .iter()
        .map(|v| (f3(v.0).parse().unwrap(), f3(v.1).parse().unwrap()))
        .collect();
    for backoff in [0usize, 2, 4, 8, 15] {
        let d = decode_ppdu(&y, 192 - backoff, &RxOfdmConfig::default()).unwrap();
        assert_eq!((d.signal.rate, d.signal.length), (Rate::R36, 100));
        assert_eq!(d.psdu, a.psdu, "back-off {backoff}");
        assert_eq!(d.scrambler_seed, 0x5d, "back-off {backoff}");
    }
}
