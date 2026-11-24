//! `rvec rx_deint`: the deinterleaver and depuncturer's vectors (`hw/rtl/rx_deint.sv`,
//! `docs/gates/rx_deint.md`), from `deinterleave_soft` and `viterbi::depuncture`.
//!
//! A packet is SIGNAL's block (one BPSK symbol at rate 1/2, 24 steps) and the data's block
//! (the rate's symbols, steps up to the anchor 16 + 8·LENGTH + 6). The soft values are random
//! in ±SOFT_MAX: the block moves values, it does not look at them.

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::convolutional::CodeRate;
use radio_phy::modulation::Modulation;
use radio_phy::ppdu::Rate;
use radio_phy::rx_ofdm_fixed::deinterleave_soft;
use radio_phy::viterbi::{depuncture, SOFT_MAX};

const DATA: usize = 48;
const RATES: [Rate; 8] = [
    Rate::R6,
    Rate::R9,
    Rate::R12,
    Rate::R18,
    Rate::R24,
    Rate::R36,
    Rate::R48,
    Rate::R54,
];

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

fn byte(v: i32) -> String {
    format!("{:02x}", (v & 0xff) as u8)
}

/// One block: its header, its symbols in on-air order, the pairs the decoder must get.
fn block(
    out: &mut impl Write,
    rng: &mut Rng,
    m: Modulation,
    code: CodeRate,
    steps: usize,
    n_dbps: usize,
) -> io::Result<()> {
    let nb = m.bits_per_subcarrier();
    let count = steps.div_ceil(n_dbps);
    writeln!(
        out,
        "B {} {} {} {}",
        rate_code(code),
        mod_code(m),
        steps,
        count
    )?;
    let mut coded = Vec::with_capacity(count * DATA * nb);
    for _ in 0..count {
        let sym: Vec<i32> = (0..DATA * nb)
            .map(|_| (rng.next_u64() % (2 * SOFT_MAX as u64 + 1)) as i32 - SOFT_MAX)
            .collect();
        let line: String = sym.iter().map(|&v| byte(v)).collect();
        writeln!(out, "S {line}")?;
        coded.extend(deinterleave_soft(&sym, m));
    }
    let full = depuncture(&coded, code).map_err(io::Error::other)?;
    let pairs: String = full
        .chunks(2)
        .take(steps)
        .map(|p| format!("{}{}", byte(p[0]), byte(p.get(1).copied().unwrap_or(0))))
        .collect();
    writeln!(out, "E {pairs}")
}

/// Lengths whose anchor (16 + 8·LENGTH + 6, always 6 modulo 8) is nearest a symbol's first
/// step and nearest its last: the fewest and the most steps read from the last symbol.
pub fn edge_lengths(n_dbps: usize) -> Vec<usize> {
    let used = |len: usize| (16 + 8 * len + 6 - 1) % n_dbps + 1;
    let range = 0..2 * n_dbps;
    let lo = range.clone().map(used).min().unwrap_or(1);
    let hi = range.clone().map(used).max().unwrap_or(1);
    let mut out: Vec<usize> = range.clone().filter(|&l| used(l) == lo).take(2).collect();
    out.extend(range.filter(|&l| used(l) == hi).take(2));
    out
}

/// The vectors: packets of every rate, lengths whose anchor falls anywhere in the last symbol
/// (on its first and last step among them), the longest PSDU once (the step counter's width).
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec rx_deint — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# B <code rate: 0 1/2, 1 2/3, 2 3/4> <modulation 0..3> <steps> <symbols>"
    )?;
    writeln!(
        out,
        "# S <48 subcarriers × N_BPSC soft values in on-air order, two hex digits each>"
    )?;
    writeln!(
        out,
        "# E <the steps' depunctured pairs: A then B, two hex digits each>"
    )?;
    writeln!(out, "# A packet is SIGNAL's block, then the data's.")?;
    let mut rng = Rng::new(0x0de1_0009);
    for rate in RATES {
        let n_dbps = rate.data_bits_per_symbol();
        let mut lengths: Vec<usize> = vec![0, 1, 2, 3, 23, 24, 100];
        lengths.extend(edge_lengths(n_dbps));
        for _ in 0..8 {
            lengths.push((rng.next_u64() % 300) as usize);
        }
        if rate == Rate::R54 {
            lengths.push(4095);
        }
        for len in lengths {
            block(out, &mut rng, Modulation::Bpsk, CodeRate::R1_2, 24, 24)?;
            let anchor = 16 + 8 * len + 6;
            block(
                out,
                &mut rng,
                rate.modulation(),
                rate.code_rate(),
                anchor,
                n_dbps,
            )?;
        }
    }
    Ok(())
}
