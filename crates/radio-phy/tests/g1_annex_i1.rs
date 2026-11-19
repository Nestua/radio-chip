//! G1 gate against IEEE Std 802.11-2024 Annex I.1 (Example 1 — BCC encoding): the standard's own
//! worked example, 36 Mb/s, the 100-octet "Ode to Joy" PSDU, every intermediate table.
//!
//! The annex is under the IEEE personal-use licence, so its numbers are not committed. The tables
//! are extracted from your own copy of the standard by `tools/ref/annex_i1_extract.py` into
//! `target/ext/annex_i1.fixture` (gitignored); point `ANNEX_I1_FIXTURE` elsewhere if needed.
//! The tests are `#[ignore]`d so CI (which has no copy) stays green; run them with
//! `cargo test -p radio-phy --test g1_annex_i1 -- --ignored`. Evidence: `docs/gates/ppdu.md`.
//! CI holds the same example by the hashes of its tables (`g1_annex_i1_hashes.rs`); this gate
//! compares value by value and says where a difference is.

// Helpers outside #[test] parse a fixture we control; a malformed line should just fail the test.
#![allow(clippy::unwrap_used)]

use radio_phy::convolutional::{encode, CodeRate};
use radio_phy::interleaver::{interleave, interleaved_index};
use radio_phy::mapper::{map_bits, mean_energy};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{
    data_subcarriers, pilot_polarity, IfftConfig, PILOT_BASE, PILOT_SUBCARRIERS,
};
use radio_phy::ppdu::{build_ppdu, data_bits, signal_bits, Rate};
use radio_phy::scrambler::Scrambler;
use std::collections::BTreeMap;

/// A printed complex value; `None` where the annex prints X (pilot slot not yet filled).
type Printed = (Option<f64>, Option<f64>);

struct Annex {
    pairs: BTreeMap<String, Vec<(i64, i64)>>,
    octets: BTreeMap<String, Vec<String>>,
    headers: BTreeMap<String, String>,
    iq: BTreeMap<String, BTreeMap<i64, Printed>>,
}

fn load() -> Annex {
    let path = std::env::var("ANNEX_I1_FIXTURE").unwrap_or_else(|_| {
        format!(
            "{}/../../target/ext/annex_i1.fixture",
            env!("CARGO_MANIFEST_DIR")
        )
    });
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut a = Annex {
        pairs: BTreeMap::new(),
        octets: BTreeMap::new(),
        headers: BTreeMap::new(),
        iq: BTreeMap::new(),
    };
    let num = |s: &str| (s != "X").then(|| s.parse::<f64>().unwrap());
    for line in text.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        let t = f[0].to_string();
        match f[1] {
            "pair" => a
                .pairs
                .entry(t)
                .or_default()
                .push((f[2].parse().unwrap(), f[3].parse().unwrap())),
            "octet" => a.octets.entry(t).or_default().push(f[2].to_string()),
            "header" => {
                a.headers.insert(t, f[2..].join(" "));
            }
            "iq" => {
                a.iq.entry(t)
                    .or_default()
                    .insert(f[2].parse().unwrap(), (num(f[3]), num(f[4])));
            }
            k => panic!("unknown kind {k}"),
        }
    }
    a
}

/// A bit table as a vector indexed by its `##` column (every index exactly once).
fn bit_table(a: &Annex, t: &str) -> Vec<u8> {
    let p = &a.pairs[t];
    let mut out = vec![u8::MAX; p.len()];
    for &(i, b) in p {
        assert!(out[i as usize] == u8::MAX, "{t}: index {i} twice");
        out[i as usize] = b as u8;
    }
    assert!(out.iter().all(|&b| b <= 1), "{t}: gaps");
    out
}

/// An octet table in transmission order. `msb_first` tables print B7…B0.
fn octet_bits(a: &Annex, t: &str) -> Vec<u8> {
    let msb_first = a.headers[t].starts_with("7-0");
    a.octets[t]
        .iter()
        .flat_map(|o| {
            let mut v: Vec<u8> = o.bytes().map(|c| c - b'0').collect();
            if msb_first {
                v.reverse();
            }
            v
        })
        .collect()
}

fn psdu(a: &Annex) -> Vec<u8> {
    // Table I-1 has no machine-friendly layout in the fixture; I-13 holds SERVICE + PSDU in
    // clear (bits 16 … 16 + 800), cross-checked against Table I-1 by hand.
    let bits = octet_bits(a, "I-13");
    bits[16..16 + 800]
        .chunks(8)
        .map(|c| c.iter().rev().fold(0u8, |acc, &b| (acc << 1) | b))
        .collect()
}

/// Annex values carry three decimals.
const PRINT_TOL: f64 = 0.0005 + 1e-9;

fn close(t: &str, idx: i64, ours: (f64, f64), theirs: Printed, tol: f64) {
    let (Some(re), Some(im)) = theirs else {
        panic!("{t} #{idx}: unexpected X");
    };
    assert!(
        (ours.0 - re).abs() <= tol && (ours.1 - im).abs() <= tol,
        "{t} #{idx}: ours ({:.4}, {:.4}) annex ({re:.3}, {im:.3})",
        ours.0,
        ours.1
    );
}

#[test]
#[ignore = "needs the locally extracted IEEE 802.11-2024 Annex I.1 fixture"]
fn signal_field() {
    let a = load();
    let sig = signal_bits(Rate::R36, 100).unwrap();
    assert_eq!(sig.to_vec(), bit_table(&a, "I-7"), "I-7 SIGNAL bits");
    let coded = encode(&sig, CodeRate::R1_2).unwrap();
    assert_eq!(coded, bit_table(&a, "I-8"), "I-8 SIGNAL coded");
    let inter = interleave(&coded, Modulation::Bpsk).unwrap();
    assert_eq!(inter, bit_table(&a, "I-9"), "I-9 SIGNAL interleaved");
    check_freq(&a, "I-10", &inter, Modulation::Bpsk, None);
    check_freq(&a, "I-11", &inter, Modulation::Bpsk, Some(0));
}

fn check_freq(a: &Annex, t: &str, bits: &[u8], m: Modulation, polarity: Option<usize>) {
    let k = 1.0 / f64::from(mean_energy(m)).sqrt();
    let mut ours: BTreeMap<i64, (f64, f64)> = (-32..32).map(|c| (c, (0.0, 0.0))).collect();
    for (&c, p) in data_subcarriers().iter().zip(map_bits(bits, m).unwrap()) {
        ours.insert(c.into(), (f64::from(p.i) * k, f64::from(p.q) * k));
    }
    let table = &a.iq[t];
    assert_eq!(table.len(), 64, "{t}: 64 subcarriers");
    for (&c, &base) in PILOT_SUBCARRIERS.iter().zip(&PILOT_BASE) {
        match polarity {
            None => {
                assert_eq!(
                    table[&c.into()],
                    (None, None),
                    "{t} #{c}: pilot slot left empty"
                );
                ours.remove(&c.into());
            }
            Some(n) => {
                ours.insert(c.into(), (f64::from(pilot_polarity(n) * base), 0.0));
            }
        }
    }
    for (c, v) in ours {
        close(t, c, v, table[&c], PRINT_TOL);
    }
}

#[test]
#[ignore = "needs the locally extracted IEEE 802.11-2024 Annex I.1 fixture"]
fn data_bits_and_scrambler() {
    let a = load();
    let psdu = psdu(&a);
    assert_eq!(psdu.len(), 100);
    // I-13: SERVICE (16 zeros), PSDU LSB first, 6 tail zeros, pad zeros to 6 × 144 bits.
    let mut clear = vec![0u8; 16];
    clear.extend(psdu.iter().flat_map(|b| (0..8).map(move |i| (b >> i) & 1)));
    clear.resize(864, 0);
    assert_eq!(
        clear,
        octet_bits(&a, "I-13"),
        "I-13 DATA bits before scrambling"
    );

    // I-14: which of our seeds produces the printed 127-bit sequence? It must be exactly one.
    let seq = bit_table(&a, "I-14");
    let seeds: Vec<u8> = (1..=0x7f)
        .filter(|&s| {
            let mut sc = Scrambler::new(s).unwrap();
            (0..127).map(|_| sc.next_bit()).collect::<Vec<_>>() == seq
        })
        .collect();
    assert_eq!(
        seeds.len(),
        1,
        "I-14: seeds reproducing the sequence: {seeds:?}"
    );
    let seed = seeds[0];
    println!("I-14 scrambling sequence = our Scrambler::new({seed:#04x}) ({seed:07b})");

    let scrambled = data_bits(&psdu, Rate::R36, seed).unwrap();
    assert_eq!(
        scrambled,
        octet_bits(&a, "I-15"),
        "I-15 DATA bits after scrambling"
    );

    let coded = encode(&scrambled, CodeRate::R3_4).unwrap();
    assert_eq!(coded, octet_bits(&a, "I-16"), "I-16 BCC coded DATA bits");

    // I-17 / I-18: the two permutations; our interleaved_index is their composition.
    let first: BTreeMap<i64, i64> = a.pairs["I-17"].iter().copied().collect();
    let second: BTreeMap<i64, i64> = a.pairs["I-18"].iter().copied().collect();
    assert_eq!((first.len(), second.len()), (192, 192));
    for k in 0..192 {
        let j = second[&first[&(k as i64)]];
        assert_eq!(
            interleaved_index(k, Modulation::Qam16) as i64,
            j,
            "I-17/I-18 k = {k}"
        );
    }
    let inter = interleave(&coded[..192], Modulation::Qam16).unwrap();
    assert_eq!(
        inter,
        bit_table(&a, "I-19"),
        "I-19 first DATA symbol interleaved"
    );
    check_freq(&a, "I-20", &inter, Modulation::Qam16, Some(1));
}

/// The annex windows each field (W = ½ at both ends) and overlaps the extra sample with the next
/// field, so a boundary sample b is ½·x[b] + ½·(previous field's cyclic extension) = ½·(x[b] +
/// x[b−64]) (every field is periodic in 64 at its end). Sample 0 and the trailing sample 880 have
/// one side only.
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

fn whole_ppdu(word_bits: u32, tol: f64) {
    let a = load();
    let cfg = IfftConfig::with_word_bits(word_bits).unwrap();
    let lsb = cfg.data.lsb();
    let seed = {
        let seq = bit_table(&a, "I-14");
        (1..=0x7fu8)
            .find(|&s| {
                let mut sc = Scrambler::new(s).unwrap();
                (0..127).map(|_| sc.next_bit()).collect::<Vec<_>>() == seq
            })
            .unwrap()
    };
    let frame: Vec<(f64, f64)> = build_ppdu(&psdu(&a), Rate::R36, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * lsb, s.im as f64 * lsb))
        .collect();
    assert_eq!(frame.len(), 880);
    let ours = windowed(&frame);
    let mut annex: BTreeMap<i64, Printed> = BTreeMap::new();
    for t in [
        "I-22", "I-23", "I-24", "I-25", "I-26", "I-27", "I-28", "I-29", "I-30",
    ] {
        annex.extend(a.iq[t].iter().map(|(k, v)| (*k, *v)));
    }
    assert_eq!(annex.len(), 881, "I-22…I-30: samples 0…880");
    let mut worst = 0.0f64;
    for (&i, &v) in &annex {
        close("I-22…I-30", i, ours[i as usize], v, tol);
        let (re, im) = (v.0.unwrap(), v.1.unwrap());
        worst = worst.max(
            (ours[i as usize].0 - re)
                .abs()
                .max((ours[i as usize].1 - im).abs()),
        );
    }
    println!("W = {word_bits}: worst |ours − annex| = {worst:.5} over 881 samples");
}

#[test]
#[ignore = "needs the locally extracted IEEE 802.11-2024 Annex I.1 fixture"]
fn whole_ppdu_near_float() {
    // 24-bit IFFT: arithmetic error ≪ the annex's print rounding.
    whole_ppdu(24, PRINT_TOL + 1e-5);
}

#[test]
#[ignore = "needs the locally extracted IEEE 802.11-2024 Annex I.1 fixture"]
fn whole_ppdu_tx_default() {
    // W = 13: G2 budget allows ~2^-11 per stage; allow print rounding + a few LSB.
    whole_ppdu(13, PRINT_TOL + 4.0 / 2048.0);
}

/// The receive half of the R0 exit gate: the annex's own printed waveform (Tables I-22…I-30,
/// three decimals, windowed) through the receiver gives the annex's PSDU, rate and length.
#[test]
#[ignore = "needs the locally extracted IEEE 802.11-2024 Annex I.1 fixture"]
fn annex_waveform_decodes() {
    use radio_phy::rx_ofdm::{decode_ppdu, RxOfdmConfig};
    let a = load();
    let mut annex: BTreeMap<i64, Printed> = BTreeMap::new();
    for t in [
        "I-22", "I-23", "I-24", "I-25", "I-26", "I-27", "I-28", "I-29", "I-30",
    ] {
        annex.extend(a.iq[t].iter().map(|(k, v)| (*k, *v)));
    }
    let y: Vec<(f64, f64)> = (0..881)
        .map(|i| {
            let v = annex[&i];
            (v.0.unwrap(), v.1.unwrap())
        })
        .collect();
    let want = psdu(&a);
    // The seed whose sequence is the annex's Table I-14.
    let seed = {
        let seq = bit_table(&a, "I-14");
        (1..=0x7fu8)
            .find(|&s| {
                let mut sc = Scrambler::new(s).unwrap();
                (0..127).map(|_| sc.next_bit()).collect::<Vec<_>>() == seq
            })
            .unwrap()
    };
    // T1 starts at sample 192; every window start inside the guard interval must do.
    for backoff in [0usize, 2, 4, 8, 15] {
        let d = decode_ppdu(&y, 192 - backoff, &RxOfdmConfig::default()).unwrap();
        assert_eq!(d.signal.rate, Rate::R36);
        assert_eq!(d.signal.length, 100);
        assert_eq!(d.psdu, want, "back-off {backoff}");
        assert_eq!(d.scrambler_seed, seed, "back-off {backoff}");
    }
    println!(
        "Annex I.1 waveform: {} octets decoded, \"{}…\"",
        want.len(),
        String::from_utf8_lossy(&want[24..52])
    );
}
