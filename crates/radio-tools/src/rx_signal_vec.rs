//! `rvec rx_signal`: SIGNAL and the verdict's vectors (`hw/rtl/rx_signal.sv`,
//! `docs/gates/rx_signal.md`), from `signal_checked`, `SignalVerdict::of` and `data_symbols`.
//!
//! A case is SIGNAL's 48 deinterleaved soft values (as `rx_deint` hands them to the decoder):
//! valid fields at every rate and length with sign flips and erasures, bad parity, random rate
//! patterns, length 0, the longest PSDU, and noise alone.

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::convolutional::{CodeRate, ConvolutionalEncoder};
use radio_phy::modulation::Modulation;
use radio_phy::ppdu::{Rate, MAX_PSDU_LEN};
use radio_phy::rx_front_fixed::{HandoffConfig, SignalVerdict};
use radio_phy::rx_ofdm::{signal_field, RxOfdmError};
use radio_phy::viterbi::{decode, Termination, SOFT_MAX};

fn field(rate_bits: [u8; 4], length: usize, parity_ok: bool, rng: &mut Rng) -> [u8; 24] {
    let mut f = [0u8; 24];
    f[..4].copy_from_slice(&rate_bits);
    f[4] = (rng.next_u64() & 1) as u8;
    for i in 0..12 {
        f[5 + i] = ((length >> i) & 1) as u8;
    }
    let p = f[..17].iter().fold(0u8, |p, b| p ^ b);
    f[17] = if parity_ok { p } else { p ^ 1 };
    // The tail: zeros, or not (the receiver does not look at it).
    if rng.next_u64().is_multiple_of(8) {
        for b in f.iter_mut().skip(18) {
            *b = (rng.next_u64() & 1) as u8;
        }
    }
    f
}

fn soft(f: &[u8; 24], rng: &mut Rng, flips: usize, zeros: usize) -> Vec<i32> {
    let coded = ConvolutionalEncoder::new().encode(f);
    let mut s: Vec<i32> = coded
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
        s[i] = -s[i];
    }
    for _ in 0..zeros {
        s[(rng.next_u64() % 48) as usize] = 0;
    }
    s
}

fn mod_code(m: Modulation) -> usize {
    match m {
        Modulation::Bpsk => 0,
        Modulation::Qpsk => 1,
        Modulation::Qam16 => 2,
        Modulation::Qam64 => 3,
    }
}

fn rate_code(r: CodeRate) -> usize {
    match r {
        CodeRate::R1_2 => 0,
        CodeRate::R2_3 => 1,
        CodeRate::R3_4 => 2,
    }
}

/// The vectors.
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec rx_signal — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# <48 soft values, two hex digits each> <24 decoded bits> <why: 0 ok, 1 parity, 2 rate, 3 length>"
    )?;
    writeln!(
        out,
        "#   <rate 0..7 (6 … 54 Mb/s)> <length> <agree> <disagree> <data symbols> <patient> <steps> <code rate> <modulation>"
    )?;
    writeln!(
        out,
        "# A failed SIGNAL has 0 in every field after <why> but <agree> and <disagree> (signal_field)."
    )?;
    let pace = HandoffConfig::default();
    let mut rng = Rng::new(0x5167_0009);
    for n in 0..3000usize {
        let s: Vec<i32> = if n % 6 == 0 {
            (0..48)
                .map(|_| (rng.next_u64() % (2 * SOFT_MAX as u64 + 1)) as i32 - SOFT_MAX)
                .collect()
        } else {
            let rb = if n % 7 == 0 {
                [0, 1, 2, 3].map(|_| (rng.next_u64() & 1) as u8)
            } else {
                Rate::ALL[n % 8].signal_rate_bits()
            };
            let length = match n % 11 {
                0 => 0,
                1 => MAX_PSDU_LEN,
                2 => 1,
                _ => (rng.next_u64() % 4096) as usize,
            };
            let f = field(rb, length, n % 13 != 0, &mut rng);
            let flips = (rng.next_u64() % 12) as usize;
            let zeros = (rng.next_u64() % 10) as usize;
            soft(&f, &mut rng, flips, zeros)
        };
        let bits =
            decode(&s, CodeRate::R1_2, Termination::ZeroAfter(24)).map_err(io::Error::other)?;
        let hex: String = s
            .iter()
            .map(|&v| format!("{:02x}", (v & 0xff) as u8))
            .collect();
        let b: String = bits
            .iter()
            .map(|&x| if x == 1 { '1' } else { '0' })
            .collect();
        let (field, check) = signal_field(&s).map_err(|e| io::Error::other(format!("{e}")))?;
        let (why, rest) = match field {
            Ok(sig) => {
                let v = SignalVerdict::of(sig, check, &pace);
                let ri = Rate::ALL.iter().position(|&r| r == sig.rate).unwrap_or(0);
                (
                    0,
                    format!(
                        "{} {} {} {} {} {} {} {} {}",
                        ri,
                        sig.length,
                        check.agree,
                        check.disagree,
                        v.data_symbols,
                        u8::from(v.patient),
                        16 + 8 * sig.length + 6,
                        rate_code(sig.rate.code_rate()),
                        mod_code(sig.rate.modulation())
                    ),
                )
            }
            Err(e) => {
                let why = match e {
                    RxOfdmError::SignalParity => 1,
                    RxOfdmError::SignalRate(_) => 2,
                    RxOfdmError::SignalLength => 3,
                    other => return Err(io::Error::other(format!("{other}"))),
                };
                (
                    why,
                    format!("0 0 {} {} 0 0 0 0 0", check.agree, check.disagree),
                )
            }
        };
        writeln!(out, "{hex} {b} {why} {rest}")?;
    }
    Ok(())
}
