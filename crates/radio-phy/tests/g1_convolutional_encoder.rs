//! G1 gate for the convolutional encoder: the golden model against references we did not write.
//!
//! 1. The code definition in IEEE 802.11-2020 §17.3.5.6: generators 133₈/171₈, MSB on the current
//!    input (Figure 17-8), so the impulse responses are the generator bits themselves.
//! 2. A worked example on Figure 17-8 published by UBC (ELEX 7860, 2022, midterm 1 solutions,
//!    Q1): input 1, 0, 1 from the zero state gives output B = 1, 1, 0.
//! 3. A black-box run of CommPy 0.8.0 (BSD-3-Clause) over all three rates:
//!    `tests/fixtures/convolutional_commpy.txt`, produced by `tools/ref/convenc_commpy.py`.
//!
//! The puncturing patterns (Figure 17-9) are not yet checked against the standard's own text; see
//! `docs/gates/convolutional_encoder.md`.

use radio_phy::convolutional::{encode, puncture, CodeRate, ConvolutionalEncoder, PartialPeriod};

const COMMPY: &str = include_str!("fixtures/convolutional_commpy.txt");

fn parse_bits(s: &str) -> Vec<u8> {
    s.bytes().map(|c| u8::from(c == b'1')).collect()
}

#[test]
fn impulse_responses_are_the_generators_msb_first() {
    let mut input = vec![1u8];
    input.extend([0; 6]);
    let out = ConvolutionalEncoder::new().encode(&input);
    let a: Vec<u8> = out.iter().step_by(2).copied().collect();
    let b: Vec<u8> = out.iter().skip(1).step_by(2).copied().collect();
    assert_eq!(
        a,
        parse_bits("1011011"),
        "output A must be 133 octal, MSB first"
    );
    assert_eq!(
        b,
        parse_bits("1111001"),
        "output B must be 171 octal, MSB first"
    );
}

#[test]
fn ubc_worked_example_output_b() {
    let out = ConvolutionalEncoder::new().encode(&[1, 0, 1]);
    let b: Vec<u8> = out.iter().skip(1).step_by(2).copied().collect();
    assert_eq!(b, vec![1, 1, 0]);
}

#[test]
fn matches_commpy_at_every_rate() {
    let mut checked = [0usize; 3];
    for line in COMMPY
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let fields: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(fields.len(), 3, "malformed fixture line: {line}");
        let (rate, idx) = match fields[0] {
            "1/2" => (CodeRate::R1_2, 0),
            "2/3" => (CodeRate::R2_3, 1),
            "3/4" => (CodeRate::R3_4, 2),
            other => panic!("unknown rate {other}"),
        };
        let ours = encode(&parse_bits(fields[1]), rate).unwrap();
        assert_eq!(
            ours,
            parse_bits(fields[2]),
            "rate {} input {}",
            fields[0],
            fields[1]
        );
        checked[idx] += 1;
    }
    assert!(
        checked.iter().all(|&n| n >= 16),
        "fixture coverage too small: {checked:?}"
    );
}

#[test]
fn encoder_is_linear_over_gf2() {
    let x: Vec<u8> = (0..96u32).map(|i| ((i * 13 + 5) % 7 % 2) as u8).collect();
    let y: Vec<u8> = (0..96u32).map(|i| ((i * 11 + 2) % 5 % 2) as u8).collect();
    let xy: Vec<u8> = x.iter().zip(&y).map(|(a, b)| a ^ b).collect();
    let ex = ConvolutionalEncoder::new().encode(&x);
    let ey = ConvolutionalEncoder::new().encode(&y);
    let exy = ConvolutionalEncoder::new().encode(&xy);
    let sum: Vec<u8> = ex.iter().zip(&ey).map(|(a, b)| a ^ b).collect();
    assert_eq!(exy, sum);
}

#[test]
fn output_lengths_follow_the_rate() {
    let bits = vec![1u8; 216];
    assert_eq!(encode(&bits, CodeRate::R1_2).unwrap().len(), 432);
    assert_eq!(encode(&bits, CodeRate::R2_3).unwrap().len(), 324);
    assert_eq!(encode(&bits, CodeRate::R3_4).unwrap().len(), 288);
}

#[test]
fn partial_puncturing_period_is_rejected() {
    let coded = vec![0u8; 8];
    assert_eq!(
        puncture(&coded, CodeRate::R3_4),
        Err(PartialPeriod { len: 8, period: 6 })
    );
    assert!(puncture(&coded, CodeRate::R2_3).is_ok());
}
