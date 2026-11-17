//! G1 gate for the interleaver: the golden model against references we did not write.
//!
//! 1. A black-box run of OpenOFDM's receiver (`scripts/decode.py`, Decoder.deinterleave,
//!    Apache-2.0, commit ea638eb) over all eight 802.11a/g rates:
//!    `tests/fixtures/interleaver_openofdm.txt`, produced by `tools/ref/interleaver_openofdm.py`.
//! 2. The two design properties IEEE 802.11-2020 §17.3.5.7 states for the permutations: adjacent
//!    coded bits go to non-adjacent subcarriers, and alternately to more and less significant
//!    constellation bits.

use radio_phy::interleaver::{
    deinterleave, interleave, interleaved_index, Modulation, PartialSymbol,
};

const ALL: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

const OPENOFDM: &str = include_str!("fixtures/interleaver_openofdm.txt");

fn parse_bits(s: &str) -> Vec<u8> {
    s.bytes().map(|c| u8::from(c == b'1')).collect()
}

#[test]
fn matches_openofdm_at_every_rate() {
    let mut rates = Vec::new();
    for line in OPENOFDM
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(f.len(), 5, "malformed fixture line: {line}");
        let n_bpsc: usize = f[1].parse().unwrap();
        let n_cbps: usize = f[2].parse().unwrap();
        let m = Modulation::from_bits_per_subcarrier(n_bpsc).unwrap();
        assert_eq!(m.bits_per_symbol(), n_cbps, "N_CBPS for {m:?}");
        let (interleaved, original) = (parse_bits(f[3]), parse_bits(f[4]));
        assert_eq!(
            interleave(&original, m).unwrap(),
            interleaved,
            "{} Mb/s",
            f[0]
        );
        assert_eq!(
            deinterleave(&interleaved, m).unwrap(),
            original,
            "{} Mb/s",
            f[0]
        );
        rates.push(f[0].to_string());
    }
    rates.dedup();
    assert_eq!(
        rates.len(),
        8,
        "fixture must cover all eight rates: {rates:?}"
    );
}

#[test]
fn index_map_is_a_permutation() {
    for m in ALL {
        let n = m.bits_per_symbol();
        let mut seen = vec![false; n];
        for k in 0..n {
            let j = interleaved_index(k, m);
            assert!(j < n && !seen[j], "{m:?}: k={k} -> j={j}");
            seen[j] = true;
        }
    }
}

#[test]
fn adjacent_coded_bits_land_on_non_adjacent_subcarriers() {
    for m in ALL {
        let n_bpsc = m.bits_per_subcarrier();
        for k in 0..m.bits_per_symbol() - 1 {
            let sc0 = interleaved_index(k, m) / n_bpsc;
            let sc1 = interleaved_index(k + 1, m) / n_bpsc;
            assert!(
                sc0.abs_diff(sc1) > 1,
                "{m:?}: k={k} on subcarriers {sc0}, {sc1}"
            );
        }
    }
}

#[test]
fn adjacent_coded_bits_alternate_bit_significance() {
    // Within a subcarrier, positions s*q .. s*q+s-1 (s = N_BPSC/2) form one significance class
    // (I and Q halves each hold the same classes). Adjacent coded bits must not stay in the same
    // class all the time: over 16 consecutive bits (one row) every class must be visited.
    for m in [Modulation::Qam16, Modulation::Qam64] {
        let n_bpsc = m.bits_per_subcarrier();
        let s = n_bpsc / 2;
        for row in 0..m.bits_per_symbol() / 16 {
            let mut classes = vec![false; s];
            for k in row * 16..row * 16 + 16 {
                classes[(interleaved_index(k, m) % n_bpsc) % s] = true;
            }
            assert!(
                classes.iter().all(|&c| c),
                "{m:?}: row {row} misses a class"
            );
        }
    }
}

#[test]
fn roundtrip_over_several_symbols() {
    for m in ALL {
        let bits: Vec<u8> = (0..3 * m.bits_per_symbol() as u32)
            .map(|i| ((i * 37 + 11) % 7 % 2) as u8)
            .collect();
        let there = interleave(&bits, m).unwrap();
        assert_ne!(there, bits, "{m:?}: interleaving must move bits");
        assert_eq!(deinterleave(&there, m).unwrap(), bits);
    }
}

#[test]
fn partial_symbol_is_rejected() {
    assert_eq!(
        interleave(&[0; 50], Modulation::Bpsk),
        Err(PartialSymbol {
            len: 50,
            n_cbps: 48
        })
    );
    assert!(deinterleave(&[0; 95], Modulation::Qpsk).is_err());
}
