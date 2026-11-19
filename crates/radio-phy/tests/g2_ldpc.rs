//! G2 gate of the hardware LDPC decoder (R0-12): `MinSumConfig::HARDWARE` on its 6-bit
//! input against float layered min-sum on exact LLRs, the same iteration limit, the same
//! noisy blocks. The budget (0.1 dB at a block error rate of 10 %) is measured by the
//! sweep `g2 ldpc` (`docs/gates/reports/g2_ldpc.txt`); this test holds the choice in CI by
//! the blocks only one of the two decoders gets wrong.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::ldpc::{LdpcCode, LdpcLength, LdpcRate};
use radio_phy::ldpc_decoder::{
    CheckRule, LdpcDecoder, MinSumConfig, Schedule, LDPC_SOFT_BITS, LDPC_SOFT_CLIP,
};

/// (blocks only the hardware decoder gets wrong, blocks only the float one gets wrong,
/// blocks the float one gets wrong).
fn discordant(
    length: LdpcLength,
    rate: LdpcRate,
    ebn0_db: f64,
    blocks: usize,
    in_bits: u32,
    cfg: &MinSumConfig,
) -> (usize, usize, usize) {
    let code = LdpcCode::new(length, rate).unwrap();
    let dec = LdpcDecoder::new(&code);
    let (num, den) = rate.fraction();
    let sigma2 = den as f64 / (2.0 * num as f64 * 10f64.powf(ebn0_db / 10.0));
    let in_max = f64::from((1i32 << (in_bits - 1)) - 1);
    let mut rng = Rng::new(0x62_1dc0);
    let (mut hw_only, mut float_only, mut float_bad) = (0, 0, 0);
    for _ in 0..blocks {
        let info: Vec<u8> = (0..code.k())
            .map(|_| u8::from(rng.uniform() < 0.5))
            .collect();
        let word = code.encode(&info).unwrap();
        let y: Vec<f64> = word
            .iter()
            .map(|&b| 1.0 - 2.0 * f64::from(b) + sigma2.sqrt() * rng.gaussian())
            .collect();
        let llr: Vec<f64> = y.iter().map(|v| 2.0 * v / sigma2).collect();
        let soft: Vec<i32> = y
            .iter()
            .map(|v| (v * in_max / LDPC_SOFT_CLIP).round().clamp(-in_max, in_max) as i32)
            .collect();
        let float = dec
            .decode_float(
                &llr,
                CheckRule::MinSum { scale: 0.8125 },
                Schedule::Layered,
                cfg.max_iter,
            )
            .unwrap();
        let hw = dec.decode_fixed(&soft, cfg).unwrap();
        let (f, h) = (float.bits != word, hw.bits != word);
        hw_only += usize::from(h && !f);
        float_only += usize::from(f && !h);
        float_bad += usize::from(f);
    }
    (hw_only, float_only, float_bad)
}

#[test]
fn hardware_decoder_follows_the_float_one() {
    // Rate 1/2 is where min-sum and narrow words lose most; 5/6 is where the clip bites.
    for (length, rate, ebn0, blocks) in [
        (LdpcLength::N648, LdpcRate::R1_2, 1.5, 400),
        (LdpcLength::N1944, LdpcRate::R1_2, 1.25, 150),
        (LdpcLength::N648, LdpcRate::R5_6, 3.5, 400),
    ] {
        let (hw, float, bad) = discordant(
            length,
            rate,
            ebn0,
            blocks,
            LDPC_SOFT_BITS,
            &MinSumConfig::HARDWARE,
        );
        println!("{length:?} {rate:?} {ebn0} dB, {blocks} blocks: hardware alone {hw}, float alone {float}, float wrong {bad}");
        assert!(bad >= 10, "the point is in the waterfall: {bad}");
        assert!(
            hw <= float + SLACK,
            "hardware alone {hw}, float alone {float}"
        );
    }
}

/// How many more blocks the hardware decoder may lose alone than the float one loses
/// alone, at each of the three points.
const SLACK: usize = 10;

#[test]
fn the_gate_sees_a_narrower_decoder() {
    // What the sweep rejects at a block error rate of 10 % must fail here too: messages
    // of 5 bits, totals of 7, an input of 4 bits, a factor of 3/4. (One bit less than
    // the choice, 6 and 8, costs nothing here: it shows as a tail below 1 %, which only
    // the sweep's block counts see.)
    let hw = MinSumConfig::HARDWARE;
    let fails = |in_bits, cfg: MinSumConfig| {
        let (h, f, _) = discordant(LdpcLength::N648, LdpcRate::R1_2, 1.5, 400, in_bits, &cfg);
        println!("in {in_bits} {cfg:?}: hardware alone {h}, float alone {f}");
        h > f + SLACK
    };
    assert!(fails(LDPC_SOFT_BITS, MinSumConfig { msg_bits: 5, ..hw }));
    assert!(fails(
        LDPC_SOFT_BITS,
        MinSumConfig {
            msg_bits: 6,
            app_bits: 7,
            ..hw
        }
    ));
    assert!(fails(4, hw));
    assert!(fails(
        LDPC_SOFT_BITS,
        MinSumConfig {
            scale_num: 3,
            scale_shift: 2,
            ..hw
        }
    ));
}
