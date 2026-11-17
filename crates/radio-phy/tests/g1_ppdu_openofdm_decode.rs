//! G1 gate for the PPDU builder: OpenOFDM decodes our whole frames.
//!
//! `tests/fixtures/ppdu_decode_openofdm.txt` lists frames produced by `rvec ppdu-frames` (three
//! per rate, random PSDUs and scrambler seeds) and what OpenOFDM's `Decoder.decode_next`
//! (Apache-2.0, commit ea638eb, run unmodified; `tools/ref/ppdu_decode_openofdm.py`) made of
//! them: packet detection on our preamble, SIGNAL, and the PSDU after its own demodulation,
//! deinterleaving, depuncturing, Viterbi decoding and descrambling.
//!
//! 1. Today's model must rebuild each recorded frame exactly (FNV-1a hash of every sample), so
//!    the verdict covers the current code.
//! 2. OpenOFDM read the right rate and length with good parity, and returned our PSDU octet for
//!    octet — at every rate, which also exercises our puncturing patterns against its receiver.
//!
//! Known receiver limitation (see `docs/gates/ppdu.md`): OpenOFDM's Viterbi appends zeros and
//! traces back from the end of the frame as if the encoder ended in the zero state. When fewer
//! than 12 scrambled pad bits follow the tail at a punctured rate, it can miscorrect the last
//! data bits; whether it does depends only on the pad values (seed experiment in the journal).
//! For such frames only SIGNAL is required to decode.

use radio_phy::convolutional::CodeRate;
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, data_symbols, frame_hash, Rate};

/// Pad bits after the tail below which OpenOFDM's end-of-frame handling is not trusted.
const MIN_PAD_FOR_PUNCTURED: usize = 12;

const DECODED: &str = include_str!("fixtures/ppdu_decode_openofdm.txt");

fn hex_bytes(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap_or_else(|e| panic!("{s}: {e}")))
        .collect()
}

#[test]
fn openofdm_decodes_our_frames_at_every_rate() {
    let cfg = IfftConfig::tx_default();
    let mut per_rate = std::collections::BTreeMap::new();
    let mut exempt = 0;
    for line in DECODED
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let (ours, theirs) = line.split_once(" | ").unwrap_or_else(|| panic!("{line}"));
        let f: Vec<&str> = ours.split_whitespace().collect();
        let mbps: u32 = f[0].parse().unwrap();
        let rate = Rate::ALL.into_iter().find(|r| r.mbps() == mbps).unwrap();
        let seed = u8::from_str_radix(f[1], 16).unwrap();
        let psdu = hex_bytes(f[2]);
        let recorded_hash = u64::from_str_radix(f[3], 16).unwrap();

        let frame = build_ppdu(&psdu, rate, seed, &cfg).unwrap();
        assert_eq!(
            frame_hash(&frame),
            recorded_hash,
            "{mbps} Mb/s, {} octets: frame changed",
            psdu.len()
        );

        let t: Vec<&str> = theirs.split_whitespace().collect();
        assert_eq!(
            t.first(),
            Some(&"rate"),
            "OpenOFDM failed on {mbps} Mb/s, {} octets: {theirs}",
            psdu.len()
        );
        assert_eq!(t[1].parse::<u32>().unwrap(), mbps, "SIGNAL rate");
        assert_eq!(t[3].parse::<usize>().unwrap(), psdu.len(), "SIGNAL length");
        assert_eq!(t[5], "1", "SIGNAL parity");
        let pad =
            data_symbols(rate, psdu.len()) * rate.data_bits_per_symbol() - (22 + 8 * psdu.len());
        let short_pad = rate.code_rate() != CodeRate::R1_2 && pad < MIN_PAD_FOR_PUNCTURED;
        if short_pad && hex_bytes(t[7]) != psdu {
            exempt += 1;
        } else {
            assert_eq!(
                hex_bytes(t[7]),
                psdu,
                "{mbps} Mb/s, {} octets: PSDU",
                psdu.len()
            );
        }
        *per_rate.entry(mbps).or_insert(0) += 1;
    }
    assert_eq!(per_rate.len(), 8, "every rate: {per_rate:?}");
    assert!(
        per_rate.values().all(|&n| n == 3),
        "three frames per rate: {per_rate:?}"
    );
    assert!(
        exempt <= 1,
        "only the 9 Mb/s one-octet frame may be exempt, got {exempt}"
    );
}
