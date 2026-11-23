//! `WindowedViterbi`, the streaming form of `decode_windowed` that the streaming data path
//! (`RxDataFixed`, ADR 0010 decision 6.1b) runs symbol by symbol: fed the same soft values
//! in pieces of whole puncture periods, it must give the bits of `decode_windowed` up to the
//! termination point, each as soon as the window allows, and nothing more.
//!
//! Not a G1 gate (both sides are our model; `decode_windowed` has its own gates in
//! `g1_viterbi.rs` and `docs/gates/viterbi.md`): it shows that the streaming form is the same
//! decoder.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::convolutional::CodeRate;
use radio_phy::viterbi::{decode_windowed, Termination, WindowedViterbi, SOFT_MAX};

/// Coded values per 12: a whole number of puncture periods at every rate (2, 3 and 4).
const PERIODS: usize = 12;

/// Rate-1/2 steps in `coded` transmitted values.
fn steps(coded: usize, rate: CodeRate) -> usize {
    match rate {
        CodeRate::R1_2 => coded / 2,
        CodeRate::R2_3 => coded / 3 * 2,
        CodeRate::R3_4 => coded / 4 * 3,
    }
}

#[test]
fn the_streaming_decoder_is_decode_windowed() {
    let mut rng = Rng::new(0x5717_EA11);
    let mut cases = 0;
    let mut early = 0usize; // bits handed out before the stream reached the anchor
    for rate in [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4] {
        for depth in [1usize, 2, 7, 64, 96, 300, usize::MAX] {
            for _ in 0..12 {
                let coded = PERIODS * (1 + (rng.next_u64() % 90) as usize);
                let len = steps(coded, rate);
                let anchor = (rng.next_u64() % (len as u64 + 1)) as usize;
                // Soft values with a bias, so that the paths are not all ties, and clipped.
                let soft: Vec<i32> = (0..coded)
                    .map(|_| {
                        let r = rng.next_u64();
                        let v = (r % (2 * SOFT_MAX as u64 + 1)) as i32 - SOFT_MAX;
                        if (r >> 40).is_multiple_of(5) {
                            0
                        } else {
                            v
                        }
                    })
                    .collect();
                let want =
                    decode_windowed(&soft, rate, Termination::ZeroAfter(anchor), depth).unwrap();
                let mut dec = WindowedViterbi::new(rate, depth, anchor);
                let mut got = Vec::new();
                let mut at = 0;
                while at < coded {
                    let n = PERIODS * (1 + (rng.next_u64() % 8) as usize);
                    let end = (at + n).min(coded);
                    let bits = dec.push(&soft[at..end]).unwrap();
                    if !dec.is_done() {
                        early += bits.len();
                    }
                    got.extend(bits);
                    at = end;
                }
                assert!(
                    dec.is_done(),
                    "{rate:?} depth {depth} anchor {anchor} len {len}"
                );
                assert_eq!(
                    got,
                    want[..anchor],
                    "{rate:?} depth {depth} anchor {anchor} len {len}"
                );
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 3 * 7 * 12);
    assert!(early > 10_000, "{early}");
}

/// A bit is handed out in the push that brings the stream `depth` steps past it, not later.
#[test]
fn a_bit_comes_out_depth_steps_after_it() {
    let depth: usize = 96;
    let anchor: usize = 1000;
    let mut dec = WindowedViterbi::new(CodeRate::R1_2, depth, anchor);
    let mut out = 0;
    for t in 0..anchor {
        out += dec.push(&[20, -20]).unwrap().len();
        // After t + 1 steps, bits 0 … t + 1 − depth are decided (while short of the flush).
        let want = if t + 1 >= anchor {
            anchor
        } else {
            (t + 2).saturating_sub(depth).min(anchor - depth)
        };
        assert_eq!(out, want, "step {t}");
    }
    assert!(dec.is_done());
    assert!(dec.push(&[1, 1]).unwrap().is_empty());
}

#[test]
fn a_piece_that_is_not_whole_puncture_periods_is_refused() {
    let mut dec = WindowedViterbi::new(CodeRate::R3_4, 96, 100);
    assert!(dec.push(&[1, 2, 3]).is_err());
}
