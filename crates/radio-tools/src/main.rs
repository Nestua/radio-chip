//! `rvec` — writes the committed test vectors that the RTL gates (G3) replay.
//!
//! Vectors are generated, never hand-edited. CI regenerates every file and fails on any diff.
//!
//! Usage: `cargo run -q -p radio-tools --bin rvec -- <block>` writes the vectors to stdout.
//!
//! `ofdm-symbols` is not an RTL vector file: it feeds the OpenOFDM receiver harness
//! (`tools/ref/ofdm_rx_openofdm.py`) and is written to the ignored `target/ext/`.

use std::io::{self, Write};
use std::process::ExitCode;

use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, IfftConfig};
use radio_phy::scrambler::Scrambler;

/// Deterministic xorshift32 so vectors are reproducible without a `rand` dependency.
struct XorShift32(u32);

impl XorShift32 {
    fn next_bit(&mut self) -> u8 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x & 1) as u8
    }
}

fn bits_to_string(bits: &[u8]) -> String {
    bits.iter()
        .map(|b| if *b == 1 { '1' } else { '0' })
        .collect()
}

/// One line per vector: `<seed hex> <input bits> <scrambled bits>`, every non-zero seed once.
fn scrambler_vectors(out: &mut impl Write) -> io::Result<()> {
    const BITS_PER_VECTOR: usize = 256;
    writeln!(
        out,
        "# rvec scrambler — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(out, "# <seed hex> <input bits> <scrambled bits>")?;
    let mut rng = XorShift32(0x1234_5678);
    for seed in 1u8..=0x7f {
        let input: Vec<u8> = (0..BITS_PER_VECTOR).map(|_| rng.next_bit()).collect();
        let mut s = Scrambler::new(seed).map_err(io::Error::other)?;
        let output = s.scramble(&input);
        writeln!(
            out,
            "{seed:02x} {} {}",
            bits_to_string(&input),
            bits_to_string(&output)
        )?;
    }
    Ok(())
}

/// OFDM symbols for the OpenOFDM receiver harness: for each modulation, 8 consecutive symbols
/// with polarity indices 0..8 (a fresh receiver starts at p_0). One line per symbol:
/// `<n_bpsc> <polarity index> <frac bits> <48 points i,q> <80 samples re,im>` (raw integers).
fn ofdm_symbols(out: &mut impl Write) -> io::Result<()> {
    const SYMBOLS_PER_MODULATION: usize = 8;
    let cfg = IfftConfig::tx_default();
    writeln!(
        out,
        "# rvec ofdm-symbols — input to tools/ref/ofdm_rx_openofdm.py, tx_default IFFT"
    )?;
    let mut rng = XorShift32(0x0FD_0005);
    for m in [
        Modulation::Bpsk,
        Modulation::Qpsk,
        Modulation::Qam16,
        Modulation::Qam64,
    ] {
        for idx in 0..SYMBOLS_PER_MODULATION {
            let bits: Vec<u8> = (0..48 * m.bits_per_subcarrier())
                .map(|_| rng.next_bit())
                .collect();
            let points = map_bits(&bits, m).map_err(io::Error::other)?;
            let samples = modulate_symbol(&points, m, idx, &cfg).map_err(io::Error::other)?;
            let pts: Vec<String> = points.iter().map(|p| format!("{},{}", p.i, p.q)).collect();
            let smp: Vec<String> = samples
                .iter()
                .map(|s| format!("{},{}", s.re, s.im))
                .collect();
            writeln!(
                out,
                "{} {idx} {} {} {}",
                m.bits_per_subcarrier(),
                cfg.data.frac_bits(),
                pts.join(" "),
                smp.join(" ")
            )?;
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let block = std::env::args().nth(1).unwrap_or_default();
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let result = match block.as_str() {
        "scrambler" => scrambler_vectors(&mut out),
        "ofdm-symbols" => ofdm_symbols(&mut out),
        _ => {
            eprintln!("usage: rvec <block>\nblocks: scrambler, ofdm-symbols");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rvec: {e}");
            ExitCode::FAILURE
        }
    }
}
