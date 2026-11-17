//! G1 gate for the OFDM modulator: references we did not write.
//!
//! 1. OpenOFDM's receiver tables (`scripts/decode.py`, Apache-2.0, commit ea638eb), printed as a
//!    black box: the 127-entry pilot polarity sequence, the pilot and occupied subcarriers and
//!    their FFT bins (`tests/fixtures/ofdm_constants_openofdm.txt`).
//! 2. A floating-point inverse DFT written as the textbook sum (independent of the radix-2
//!    datapath): the fixed-point symbol must stay within the G2 budget of it.
//! 3. OpenOFDM's receiver decoding our symbols is a separate test (`g1_ofdm_openofdm_rx.rs`).

use radio_phy::fixed::{sqnr_db, QFormat, Rounding};
use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{
    data_subcarriers, fft_bin, ifft64, modulate_symbol, modulate_symbol_f64, pilot_polarity,
    IfftConfig, Iq, OfdmError, CP_LEN, FFT_SIZE, PILOT_SUBCARRIERS, SYMBOL_LEN,
};

const CONSTANTS: &str = include_str!("fixtures/ofdm_constants_openofdm.txt");
const ALL: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];
/// G2 budget: the IFFT's quantisation noise must sit at least 45 dB below the signal
/// (802.11a needs −25 dB EVM at 54 Mb/s; this leaves 20 dB for everything else).
const SQNR_BUDGET_DB: f64 = 45.0;

fn table(key: &str) -> Vec<i64> {
    let line = CONSTANTS
        .lines()
        .find(|l| l.split_whitespace().next() == Some(key))
        .unwrap_or_else(|| panic!("fixture has no {key} line"));
    line.split_whitespace()
        .skip(1)
        .map(|v| v.parse().unwrap_or_else(|e| panic!("{v}: {e}")))
        .collect()
}

fn random_points(m: Modulation, seed: u32) -> Vec<radio_phy::mapper::Point> {
    let mut x = seed;
    let bits: Vec<u8> = (0..48 * m.bits_per_subcarrier())
        .map(|_| {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            (x & 1) as u8
        })
        .collect();
    map_bits(&bits, m).unwrap_or_else(|e| panic!("{e}"))
}

#[test]
fn pilot_polarity_matches_openofdm() {
    let theirs = table("polarity");
    assert_eq!(theirs.len(), 127);
    for (n, &p) in theirs.iter().enumerate() {
        assert_eq!(i64::from(pilot_polarity(n)), p, "p_{n}");
        assert_eq!(
            pilot_polarity(n + 127),
            pilot_polarity(n),
            "period 127 at {n}"
        );
    }
}

#[test]
fn subcarrier_layout_matches_openofdm() {
    let pilots: Vec<i64> = PILOT_SUBCARRIERS.iter().map(|&c| i64::from(c)).collect();
    assert_eq!(pilots, table("pilots"));
    let occupied = table("subcarriers");
    let data: Vec<i64> = occupied
        .iter()
        .copied()
        .filter(|c| !pilots.contains(c))
        .collect();
    let ours: Vec<i64> = data_subcarriers().iter().map(|&c| i64::from(c)).collect();
    assert_eq!(ours, data);
    let bins: Vec<i64> = occupied.iter().map(|&c| fft_bin(c as i8) as i64).collect();
    assert_eq!(bins, table("fft_bins"));
}

#[test]
fn symbol_has_cyclic_prefix() {
    let cfg = IfftConfig::with_word_bits(16).unwrap();
    let s = modulate_symbol(
        &random_points(Modulation::Qam16, 7),
        Modulation::Qam16,
        3,
        &cfg,
    )
    .unwrap();
    assert_eq!(s.len(), SYMBOL_LEN);
    assert_eq!(s[..CP_LEN], s[FFT_SIZE..]);
}

#[test]
fn fixed_point_symbol_tracks_the_float_reference() {
    let cfg = IfftConfig::tx_default();
    let scale = cfg.data.lsb();
    for m in ALL {
        for seed in 1..=20u32 {
            let points = random_points(m, seed * 7919);
            let fixed = modulate_symbol(&points, m, seed as usize, &cfg).unwrap();
            let float = modulate_symbol_f64(&points, m, seed as usize).unwrap();
            let re_ref: Vec<f64> = float.iter().flat_map(|&(r, i)| [r, i]).collect();
            let re_fix: Vec<f64> = fixed
                .iter()
                .flat_map(|s| [s.re as f64 * scale, s.im as f64 * scale])
                .collect();
            let sqnr = sqnr_db(&re_ref, &re_fix).unwrap();
            assert!(sqnr >= SQNR_BUDGET_DB, "{m:?} seed {seed}: {sqnr:.1} dB");
        }
    }
}

#[test]
fn single_bin_gives_a_scaled_complex_exponential() {
    // IDFT of a unit impulse at bin k is e^(+j2πkn/64)/64: checks direction and the 1/64.
    let cfg = IfftConfig::with_word_bits(20).unwrap();
    let one = 1i64 << cfg.data.frac_bits();
    for k in [1usize, 5, 31, 32, 63] {
        let mut bins = [Iq::default(); FFT_SIZE];
        bins[k] = Iq { re: one, im: 0 };
        let out = ifft64(&bins, &cfg);
        for (n, s) in out.iter().enumerate() {
            let phase = 2.0 * std::f64::consts::PI * (k * n) as f64 / 64.0;
            let (want_re, want_im) = (phase.cos() / 64.0, phase.sin() / 64.0);
            let lsb = cfg.data.lsb();
            assert!(
                (s.re as f64 * lsb - want_re).abs() < 8.0 * lsb,
                "k={k} n={n} re"
            );
            assert!(
                (s.im as f64 * lsb - want_im).abs() < 8.0 * lsb,
                "k={k} n={n} im"
            );
        }
    }
}

#[test]
fn bad_inputs_are_rejected() {
    let cfg = IfftConfig::with_word_bits(16).unwrap();
    let points = random_points(Modulation::Bpsk, 1);
    assert_eq!(
        modulate_symbol(&points[..47], Modulation::Bpsk, 0, &cfg),
        Err(OfdmError::DataLength(47))
    );
    assert!(IfftConfig::with_word_bits(3).is_err());
    assert!(IfftConfig::with_word_bits(25).is_err());
    let _ = (QFormat::new(16, 14), Rounding::HalfUp);
}
