//! Gate for SIGNAL's check count (R0-08d): the number of SIGNAL's 48 coded bits whose hard
//! decision disagrees with the decoded field encoded again.
//!
//! Reference: the transmitter's SIGNAL (`signal_bits`, `encode` — held to IEEE Std
//! 802.11-2024 Annex I.1, Tables I-7 and I-8, by `g1_annex_i1`) with a number of coded bits
//! chosen here turned over; the count must be that number. The code's free distance is 10
//! (§17.3.5.6), so up to four errors are always corrected.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, mean_power, Rng};
use radio_phy::convolutional::{encode, CodeRate};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, signal_bits, Rate};
use radio_phy::rx_ofdm::{signal_checked, RxOfdmConfig, RxOfdmError, Signal};
use radio_phy::rx_ofdm_fixed::{
    decode_signal_checked_fixed, decode_signal_fixed, FixedOfdmConfig, FixedTracking,
};

/// Soft values of coded bits: positive is a 0 (`docs/gates/viterbi.md`).
fn soft(coded: &[u8], magnitude: i32) -> Vec<i32> {
    coded
        .iter()
        .map(|&b| if b == 0 { magnitude } else { -magnitude })
        .collect()
}

#[test]
fn the_count_is_the_number_of_coded_bits_turned_over() {
    let mut rng = Rng::new(0x51c);
    for rate in Rate::ALL {
        for _ in 0..4 {
            let length = 1 + (rng.next_u64() % 4095) as usize;
            let coded = encode(&signal_bits(rate, length).unwrap(), CodeRate::R1_2).unwrap();
            assert_eq!(coded.len(), 48);
            let want = Signal { rate, length };
            let clean = soft(&coded, 100);
            assert_eq!(signal_checked(&clean), Ok((want, 0)), "{rate:?} {length}");
            let sets: [&[usize]; 7] = [
                &[5],
                &[47],
                &[0, 30],
                &[11, 12],
                &[0, 17, 40],
                &[3, 15, 27, 44],
                &[20, 21, 22, 23],
            ];
            for set in sets {
                let mut s = clean.clone();
                for &k in set {
                    s[k] = -s[k];
                }
                assert_eq!(
                    signal_checked(&s),
                    Ok((want, set.len())),
                    "{rate:?} {length} {set:?}"
                );
            }
            // The magnitude does not count, the sign does; an erasure is not a disagreement.
            let mut s = clean.clone();
            s[7] = -s[7].signum();
            s[33] = -127 * s[33].signum();
            s[9] = 0;
            s[10] = 0;
            assert_eq!(signal_checked(&s), Ok((want, 2)), "{rate:?} {length}");
        }
    }
}

#[test]
fn the_reserved_bit_set_is_encoded_as_it_was_decoded() {
    // A receiver does not refuse the reserved bit (§17.3.4.3); the count is against the
    // bits decoded, not against a field built from the rate and the length.
    let mut bits = signal_bits(Rate::R12, 1234).unwrap();
    bits[4] = 1;
    bits[17] ^= 1;
    let coded = encode(&bits, CodeRate::R1_2).unwrap();
    let want = Signal {
        rate: Rate::R12,
        length: 1234,
    };
    assert_eq!(signal_checked(&soft(&coded, 64)), Ok((want, 0)));
    let mut s = soft(&coded, 64);
    s[2] = -s[2];
    s[25] = -s[25];
    s[46] = -s[46];
    assert_eq!(signal_checked(&s), Ok((want, 3)));
}

#[test]
fn a_field_that_fails_has_no_count_and_noise_that_passes_has_a_large_one() {
    let coded = encode(&signal_bits(Rate::R6, 100).unwrap(), CodeRate::R1_2).unwrap();
    assert!(signal_checked(&soft(&coded, 100)[..47]).is_err());
    // Parity: the field with one information bit turned before encoding.
    let mut bits = signal_bits(Rate::R6, 100).unwrap();
    bits[6] ^= 1;
    let coded = encode(&bits, CodeRate::R1_2).unwrap();
    assert_eq!(
        signal_checked(&soft(&coded, 100)),
        Err(RxOfdmError::SignalParity)
    );

    // Noise: of the blocks that pass, hardly any is within four disagreements (a block of
    // random signs is on average 24 of 48 away from any one codeword; the decoder picks
    // the nearest of 2^18).
    let mut rng = Rng::new(0xbead);
    let (mut passed, mut near, mut sum) = (0usize, 0usize, 0usize);
    for _ in 0..4000 {
        let s: Vec<i32> = (0..48)
            .map(|_| ((rng.gaussian() * 40.0) as i32).clamp(-127, 127))
            .collect();
        if let Ok((_, n)) = signal_checked(&s) {
            assert!(n <= 48);
            passed += 1;
            sum += n;
            near += usize::from(n <= 4);
        }
    }
    // A quarter pass: one parity bit and half the rate patterns.
    assert!((800..1200).contains(&passed), "{passed}");
    assert!(near * 50 < passed, "{near} of {passed}");
    assert!(sum > 8 * passed, "mean {}", sum as f64 / passed as f64);
}

#[test]
fn the_integer_decoder_reports_the_count_with_its_signal() {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let tx = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << tx.data.frac_bits());
    let mut rng = Rng::new(0x0f1c);
    let t1 = 192 - 6;
    let run = |rate: Rate, snr_db: f64, rng: &mut Rng| {
        let len = 1 + (rng.next_u64() % 4095) as usize;
        let data: Vec<u8> = (0..len).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let x: Vec<(f64, f64)> = build_ppdu(&data, rate, 0x5d, &tx)
            .unwrap()
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let p = mean_power(&x);
        let x = awgn(&x[..t1 + 208], snr_db, p, rng);
        // −12 dBFS in 12 bits.
        let k = 2048.0 * (10f64.powf(-1.2) / p).sqrt();
        let z: Vec<(i64, i64)> = x
            .iter()
            .map(|s| ((s.0 * k).round() as i64, (s.1 * k).round() as i64))
            .collect();
        let start = FixedTracking::unknown(&fx);
        let checked = decode_signal_checked_fixed(&z, t1, &cfg, &fx, start);
        // The plain function is the checked one without its count.
        assert_eq!(
            decode_signal_fixed(&z, t1, &cfg, &fx, start),
            checked.map(|c| c.0)
        );
        (checked, Signal { rate, length: len })
    };
    // At 30 dB: the field, and no disagreement.
    for rate in Rate::ALL {
        let (got, sent) = run(rate, 30.0, &mut rng);
        assert_eq!(got, Ok((sent, 0)), "{rate:?}");
    }
    // At 2 dB the true field still decodes most of the time, with disagreements.
    let (mut right, mut sum) = (0usize, 0usize);
    for _ in 0..200 {
        let (got, sent) = run(Rate::R6, 2.0, &mut rng);
        if let Ok((s, n)) = got {
            assert!(n <= 48);
            if s == sent {
                right += 1;
                sum += n;
            }
        }
    }
    assert!(right > 150, "{right}");
    // Raw BPSK errors at 2 dB over the band are several per 48.
    assert!(sum > right && sum < 12 * right, "{sum} over {right}");
}
