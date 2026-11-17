//! G1 gate for the PPDU builder: references we did not write.
//!
//! 1. OpenOFDM's receiver tables (`scripts/decode.py`, Apache-2.0, commit ea638eb), printed as a
//!    black box (`tests/fixtures/preamble_openofdm.txt`): the 160 long-training samples and one
//!    16-sample short-training period in the 3-decimal format of the standard's annex, and the
//!    long training sequence LTS_REF.
//! 2. Structural rules of IEEE 802.11-2020 §17.3.4–17.3.5 (SIGNAL layout and parity, N_SYM, tail
//!    reset after scrambling, field lengths).
//! 3. OpenOFDM decoding whole frames is a separate test (`g1_ppdu_openofdm_decode.rs`).

use radio_phy::ofdm::{IfftConfig, SYMBOL_LEN};
use radio_phy::ppdu::{
    build_ppdu, data_bits, data_symbols, long_training_field, long_training_sequence,
    short_training_field, signal_bits, PpduError, Rate, TRAINING_FIELD_LEN,
};
use radio_phy::scrambler::Scrambler;

const PREAMBLE: &str = include_str!("fixtures/preamble_openofdm.txt");

fn table(key: &str) -> Vec<&str> {
    PREAMBLE
        .lines()
        .find(|l| l.split_whitespace().next() == Some(key))
        .unwrap_or_else(|| panic!("fixture has no {key} line"))
        .split_whitespace()
        .skip(1)
        .collect()
}

fn complex_table(key: &str) -> Vec<(f64, f64)> {
    table(key)
        .iter()
        .map(|s| {
            let (a, b) = s.split_once(',').unwrap_or_else(|| panic!("{s}"));
            (
                a.parse().unwrap_or_else(|e| panic!("{a}: {e}")),
                b.parse().unwrap_or_else(|e| panic!("{b}: {e}")),
            )
        })
        .collect()
}

/// Least-squares real scale s with annex ≈ s · ours, and the residual EVM in percent.
fn fit(annex: &[(f64, f64)], ours: &[(f64, f64)]) -> (f64, f64) {
    let num: f64 = annex
        .iter()
        .zip(ours)
        .map(|(a, o)| a.0 * o.0 + a.1 * o.1)
        .sum();
    let den: f64 = ours.iter().map(|o| o.0 * o.0 + o.1 * o.1).sum();
    let s = num / den;
    let err: f64 = annex
        .iter()
        .zip(ours)
        .map(|(a, o)| (a.0 - s * o.0).powi(2) + (a.1 - s * o.1).powi(2))
        .sum();
    let pow: f64 = annex.iter().map(|a| a.0 * a.0 + a.1 * a.1).sum();
    (s, 100.0 * (err / pow).sqrt())
}

fn to_f64(samples: &[radio_phy::ofdm::Iq], cfg: &IfftConfig) -> Vec<(f64, f64)> {
    let lsb = cfg.data.lsb();
    samples
        .iter()
        .map(|s| (s.re as f64 * lsb, s.im as f64 * lsb))
        .collect()
}

#[test]
fn training_fields_match_the_annex_samples() {
    let cfg = IfftConfig::tx_default();
    let stf = to_f64(&short_training_field(&cfg).unwrap(), &cfg);
    let ltf = to_f64(&long_training_field(&cfg).unwrap(), &cfg);
    assert_eq!(
        (stf.len(), ltf.len()),
        (TRAINING_FIELD_LEN, TRAINING_FIELD_LEN)
    );
    // Short: one clean period (annex samples 16..31). Long: skip sample 0, which the annex shows
    // with the transition window applied; we do not window.
    let (s_short, evm_short) = fit(&complex_table("short"), &stf[16..32]);
    let (s_long, evm_long) = fit(&complex_table("long")[1..], &ltf[1..]);
    eprintln!("STF: scale {s_short:.4}, residual {evm_short:.2} %; LTF: scale {s_long:.4}, residual {evm_long:.2} %");
    assert!(evm_short < 1.5, "STF residual {evm_short:.2} %");
    assert!(evm_long < 1.5, "LTF residual {evm_long:.2} %");
    // One normalisation for both fields: STF and LTF carry the same power.
    assert!(
        (s_short / s_long - 1.0).abs() < 0.01,
        "scales {s_short} vs {s_long}"
    );
}

#[test]
fn long_training_sequence_matches_openofdm() {
    let ours = long_training_sequence();
    for entry in table("lts_ref") {
        let (c, v) = entry.split_once(':').unwrap();
        let (c, v): (i64, i64) = (c.parse().unwrap(), v.parse().unwrap());
        assert_eq!(i64::from(ours[(c + 26) as usize]), v, "L_{c}");
    }
    assert_eq!(ours[26], 0, "no DC");
}

#[test]
fn signal_field_layout() {
    for rate in Rate::ALL {
        for length in [1usize, 100, 1500, 4095] {
            let bits = signal_bits(rate, length).unwrap();
            assert_eq!(bits[..4], rate.signal_rate_bits());
            assert_eq!(bits[4], 0, "reserved");
            let len: usize = (0..12).map(|i| usize::from(bits[5 + i]) << i).sum();
            assert_eq!(len, length, "LENGTH LSB first");
            assert_eq!(
                bits[..18].iter().map(|&b| u32::from(b)).sum::<u32>() % 2,
                0,
                "even parity"
            );
            assert_eq!(bits[18..], [0; 6], "tail");
        }
    }
    assert_eq!(signal_bits(Rate::R6, 0), Err(PpduError::PsduLength(0)));
    assert_eq!(
        signal_bits(Rate::R6, 4096),
        Err(PpduError::PsduLength(4096))
    );
}

#[test]
fn data_stream_structure() {
    let psdu: Vec<u8> = (0..100u32).map(|i| (i * 37 + 5) as u8).collect();
    for rate in Rate::ALL {
        let n_sym = data_symbols(rate, psdu.len());
        assert_eq!(
            n_sym,
            (16 + 8 * psdu.len() + 6).div_ceil(rate.data_bits_per_symbol())
        );
        let bits = data_bits(&psdu, rate, 0x5d).unwrap();
        assert_eq!(bits.len(), n_sym * rate.data_bits_per_symbol());
        // SERVICE and PSDU are the scrambled clear bits; the tail is reset to zero afterwards.
        let mut clear = vec![0u8; 16];
        for byte in &psdu {
            clear.extend((0..8).map(|i| (byte >> i) & 1));
        }
        let mut s = Scrambler::new(0x5d).unwrap();
        let scrambled = s.scramble(&clear);
        assert_eq!(bits[..clear.len()], scrambled[..], "{rate:?}");
        assert_eq!(bits[clear.len()..clear.len() + 6], [0; 6], "{rate:?} tail");
    }
    assert_eq!(
        data_bits(&psdu, Rate::R6, 0),
        Err(PpduError::ScramblerSeed(0))
    );
}

#[test]
fn frame_length_and_order() {
    let cfg = IfftConfig::tx_default();
    let psdu = [0xA5u8; 57];
    for rate in Rate::ALL {
        let frame = build_ppdu(&psdu, rate, 0x5d, &cfg).unwrap();
        let n_sym = data_symbols(rate, psdu.len());
        assert_eq!(
            frame.len(),
            2 * TRAINING_FIELD_LEN + SYMBOL_LEN * (1 + n_sym),
            "{rate:?}"
        );
        assert_eq!(frame[..160], short_training_field(&cfg).unwrap()[..]);
        assert_eq!(frame[160..320], long_training_field(&cfg).unwrap()[..]);
    }
}

#[test]
fn rate_table_is_consistent() {
    use radio_phy::convolutional::CodeRate;
    for rate in Rate::ALL {
        let n_cbps = rate.modulation().bits_per_symbol();
        let (num, den) = match rate.code_rate() {
            CodeRate::R1_2 => (1, 2),
            CodeRate::R2_3 => (2, 3),
            CodeRate::R3_4 => (3, 4),
        };
        assert_eq!(rate.data_bits_per_symbol() * den, n_cbps * num, "{rate:?}");
        assert_eq!(
            rate.data_bits_per_symbol() as u32 * 250,
            rate.mbps() * 1000,
            "{rate:?}"
        );
    }
}
