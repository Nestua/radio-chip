//! SIGNAL and the verdict's G0 (`docs/gates/rx_signal.md`): the architecture chosen for
//! `hw/rtl/rx_signal.sv` gives the model's field, failure, check, verdict and data block, bit
//! for bit.
//!
//! The architecture (the founder chose the stream from the decoder's flush, 2026-11-24):
//! - SIGNAL's 24 depunctured pairs (rate 1/2, nothing stolen) are kept as they pass to the
//!   decoder: per coded value its sign and whether it is 0 (an erasure).
//! - The decoder's 24 bits come out of its flush a bit a clock (`viterbi.sv`: the window of 96 is
//!   longer than the block, so they are `decode`'s). Each is encoded again as it comes
//!   (`conv_encoder.sv`) and its (A, B) compared with the kept pair: a nonzero value agrees when
//!   its sign is the encoded bit's (negative for 1). Bits 0 … 17 go to the field and its parity.
//! - At the last bit: the parity, then the rate (eight patterns of bits 0 … 3), then the length
//!   (bits 5 … 16, LSB first; 0 fails), in the model's order. The data block's steps are the
//!   anchor 16 + 8·LENGTH + 6; its symbols ⌈anchor / N_DBPS⌉ without a divider: N_DBPS = 2^a·t
//!   with t ∈ {3, 9, 27}, so the count is ((anchor + N_DBPS − 1) >> a)·M_t >> s_t; patient at
//!   6, 9 and 12 Mb/s with at least 40 agreeing.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::convolutional::{CodeRate, ConvolutionalEncoder};
use radio_phy::ppdu::{data_symbols, Rate, MAX_PSDU_LEN};
use radio_phy::rx_front_fixed::{HandoffConfig, SignalVerdict, MAX_DATA_SYMBOLS};
use radio_phy::rx_ofdm::{signal_checked, RxOfdmError, Signal, SignalCheck};
use radio_phy::viterbi::{decode, Termination, SOFT_MAX};

/// `rx_signal.sv`'s widths of z = (anchor + N_DBPS − 1) >> a and of z·M.
const Z_BITS: u32 = 14;
const ZM_BITS: u32 = 26;

/// The reciprocal constants by t: (t, M, s). `RECIPROCALS[i]` is exact for z ≤ `Z_MAX[i]`.
const RECIPROCALS: [(usize, usize, u32); 3] = [(3, 2731, 13), (9, 3641, 15), (27, 4855, 17)];

/// The RTL's rate table: SIGNAL's four rate bits (R1 first) → (rate, N_DBPS's a, t's index).
fn rate_of(bits: [u8; 4]) -> Option<(Rate, u32, usize)> {
    // IEEE 802.11-2020 Table 17-6; N_DBPS = 24, 36, 48, 72, 96, 144, 192, 216.
    let r = match bits {
        [1, 1, 0, 1] => (Rate::R6, 3, 0),
        [1, 1, 1, 1] => (Rate::R9, 2, 1),
        [0, 1, 0, 1] => (Rate::R12, 4, 0),
        [0, 1, 1, 1] => (Rate::R18, 3, 1),
        [1, 0, 0, 1] => (Rate::R24, 5, 0),
        [1, 0, 1, 1] => (Rate::R36, 4, 1),
        [0, 0, 0, 1] => (Rate::R48, 6, 0),
        [0, 0, 1, 1] => (Rate::R54, 3, 2),
        _ => return None,
    };
    Some(r)
}

/// The symbols of a block of `anchor` steps by the RTL's shift and product.
fn symbols_by_product(anchor: usize, a: u32, ti: usize) -> usize {
    let (t, m, s) = RECIPROCALS[ti];
    let n_dbps = (1usize << a) * t;
    let z = (anchor + n_dbps - 1) >> a;
    // The RTL's registers: z 14 bits, its product with M 26 bits.
    assert!(z < 1 << Z_BITS, "z {z} needs more than {Z_BITS} bits");
    assert!(
        z * m < 1 << ZM_BITS,
        "z·M {} needs more than {ZM_BITS} bits",
        z * m
    );
    (z * m) >> s
}

#[derive(Debug, PartialEq)]
struct Out {
    result: Result<(Signal, SignalCheck), RxOfdmError>,
    verdict: Option<SignalVerdict>,
    steps: usize,
}

/// The architecture: the kept pairs and the decoder's bits in, what the block puts out.
fn architecture(
    soft: &[i32],
    bits: &[u8],
) -> (Result<(Signal, u32, usize), RxOfdmError>, SignalCheck) {
    assert_eq!((soft.len(), bits.len()), (48, 24));
    let mut enc = ConvolutionalEncoder::new();
    let (mut agree, mut disagree) = (0, 0);
    let mut parity = 0u8;
    let mut field = [0u8; 18];
    for (t, &b) in bits.iter().enumerate() {
        let (ca, cb) = enc.encode_bit(b);
        for (c, v) in [(ca, soft[2 * t]), (cb, soft[2 * t + 1])] {
            if v != 0 {
                if (v < 0) == (c == 1) {
                    agree += 1;
                } else {
                    disagree += 1;
                }
            }
        }
        if t < 18 {
            field[t] = b;
            parity ^= b;
        }
    }
    let check = SignalCheck { agree, disagree };
    if parity != 0 {
        return (Err(RxOfdmError::SignalParity), check);
    }
    let rb = [field[0], field[1], field[2], field[3]];
    let Some((rate, a, ti)) = rate_of(rb) else {
        return (Err(RxOfdmError::SignalRate(rb)), check);
    };
    let length = (5..17)
        .rev()
        .fold(0usize, |acc, i| (acc << 1) | usize::from(field[i]));
    if length == 0 {
        return (Err(RxOfdmError::SignalLength), check);
    }
    (Ok((Signal { rate, length }, a, ti)), check)
}

fn run(soft: &[i32]) -> (Out, Out) {
    let bits = decode(soft, CodeRate::R1_2, Termination::ZeroAfter(24)).unwrap();
    let pace = HandoffConfig::default();
    // The model.
    let result = signal_checked(soft);
    let model = Out {
        verdict: result
            .as_ref()
            .ok()
            .map(|&(s, c)| SignalVerdict::of(s, c, &pace)),
        steps: result.as_ref().map_or(0, |(s, _)| 16 + 8 * s.length + 6),
        result,
    };
    // The architecture.
    let (r, check) = architecture(soft, &bits);
    let ours = match r {
        Ok((signal, a, ti)) => {
            let anchor = 16 + 8 * signal.length + 6;
            let symbols = symbols_by_product(anchor, a, ti);
            let patient =
                matches!(signal.rate, Rate::R6 | Rate::R9 | Rate::R12) && check.agree >= 40;
            Out {
                result: Ok((signal, check)),
                verdict: Some(SignalVerdict {
                    data_symbols: symbols,
                    patient,
                }),
                steps: anchor,
            }
        }
        Err(e) => Out {
            result: Err(e),
            verdict: None,
            steps: 0,
        },
    };
    (model, ours)
}

/// SIGNAL's 48 coded bits of a field, as soft values: magnitude random, `flips` signs turned,
/// `zeros` values erased.
fn soft_of(field: &[u8; 24], rng: &mut Rng, flips: usize, zeros: usize) -> Vec<i32> {
    let coded = ConvolutionalEncoder::new().encode(field);
    let mut soft: Vec<i32> = coded
        .iter()
        .map(|&c| {
            let m = 1 + (rng.next_u64() % SOFT_MAX as u64) as i32;
            if c == 1 {
                -m
            } else {
                m
            }
        })
        .collect();
    for _ in 0..flips {
        let i = (rng.next_u64() % 48) as usize;
        soft[i] = -soft[i];
    }
    for _ in 0..zeros {
        soft[(rng.next_u64() % 48) as usize] = 0;
    }
    soft
}

fn field_of(rate_bits: [u8; 4], length: usize, parity_ok: bool, rng: &mut Rng) -> [u8; 24] {
    let mut f = [0u8; 24];
    f[..4].copy_from_slice(&rate_bits);
    f[4] = (rng.next_u64() & 1) as u8; // reserved: not checked (§17.3.4.3)
    for i in 0..12 {
        f[5 + i] = ((length >> i) & 1) as u8;
    }
    let p = f[..17].iter().fold(0u8, |p, b| p ^ b);
    f[17] = if parity_ok { p } else { p ^ 1 };
    f
}

/// The symbol count by shift and product is `data_symbols` at every rate and length, and never
/// above the front's limit.
#[test]
fn symbols_without_a_divider() {
    for rate in Rate::ALL {
        let (r, a, ti) = rate_of(rate.signal_rate_bits()).unwrap();
        assert_eq!(r, rate);
        assert_eq!(
            (1usize << a) * RECIPROCALS[ti].0,
            rate.data_bits_per_symbol()
        );
        for length in 0..=MAX_PSDU_LEN {
            let n = symbols_by_product(16 + 8 * length + 6, a, ti);
            assert_eq!(n, data_symbols(rate, length), "{rate:?} {length}");
            assert!(n <= MAX_DATA_SYMBOLS);
        }
    }
}

/// Valid fields at every rate, noisy and erased values, bad parity, random rate patterns,
/// length 0, and noise: the architecture is the model.
#[test]
fn field_check_verdict() {
    let mut rng = Rng::new(0x51_6e41);
    let mut cases = [0usize; 4]; // ok, parity, rate, length
    let mut patient = [0usize; 2];
    for n in 0..20_000 {
        let soft = match n % 5 {
            // Noise alone.
            0 => (0..48)
                .map(|_| (rng.next_u64() % (2 * SOFT_MAX as u64 + 1)) as i32 - SOFT_MAX)
                .collect(),
            _ => {
                let rb = if n % 7 == 0 {
                    [0, 1, 2, 3].map(|_| (rng.next_u64() & 1) as u8)
                } else {
                    Rate::ALL[(rng.next_u64() % 8) as usize].signal_rate_bits()
                };
                let length = match n % 11 {
                    0 => 0,
                    1 => MAX_PSDU_LEN,
                    _ => (rng.next_u64() % 4096) as usize,
                };
                let f = field_of(rb, length, n % 13 != 0, &mut rng);
                let flips = (rng.next_u64() % 12) as usize;
                let zeros = (rng.next_u64() % 10) as usize;
                soft_of(&f, &mut rng, flips, zeros)
            }
        };
        let (model, ours) = run(&soft);
        assert_eq!(ours, model, "case {n}");
        let k = match &model.result {
            Ok(_) => 0,
            Err(RxOfdmError::SignalParity) => 1,
            Err(RxOfdmError::SignalRate(_)) => 2,
            Err(_) => 3,
        };
        cases[k] += 1;
        if let Some(v) = model.verdict {
            patient[usize::from(v.patient)] += 1;
        }
    }
    assert!(cases.iter().all(|&c| c > 300), "{cases:?}");
    assert!(patient.iter().all(|&c| c > 300), "{patient:?}");
}
