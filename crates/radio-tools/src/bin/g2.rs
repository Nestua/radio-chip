//! `g2` — word-length sweeps for gate G2 (fixed point against the float model).
//!
//! Usage: `cargo run -q --release -p radio-tools --bin g2 -- ofdm`
//! Prints, per word length, the minimum and mean SQNR over random symbols of every modulation.
//! The result goes into the block's gate journal; it is not a generated file.

use std::process::ExitCode;

use radio_phy::fixed::sqnr_db;
use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, modulate_symbol_f64, IfftConfig};

const SYMBOLS: u32 = 200;
const ALL: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

fn ofdm() -> Result<(), String> {
    println!(
        "# G2 sweep: fixed-point OFDM symbol vs float reference, \
         {SYMBOLS} random symbols per modulation"
    );
    println!("#  W | min SQNR dB: BPSK   QPSK  16QAM  64QAM | worst |  mean");
    for w in 8..=20 {
        let cfg = IfftConfig::with_word_bits(w).map_err(|e| e.to_string())?;
        let scale = cfg.data.lsb();
        let mut mins = Vec::new();
        let mut all = Vec::new();
        for m in ALL {
            let mut x: u32 = 0x5EED_0000 ^ w;
            let mut min = f64::INFINITY;
            for s in 0..SYMBOLS {
                let bits: Vec<u8> = (0..48 * m.bits_per_subcarrier())
                    .map(|_| {
                        x ^= x << 13;
                        x ^= x >> 17;
                        x ^= x << 5;
                        (x & 1) as u8
                    })
                    .collect();
                let points = map_bits(&bits, m).map_err(|e| e.to_string())?;
                let fixed =
                    modulate_symbol(&points, m, s as usize, &cfg).map_err(|e| e.to_string())?;
                let float =
                    modulate_symbol_f64(&points, m, s as usize).map_err(|e| e.to_string())?;
                let r: Vec<f64> = float.iter().flat_map(|&(a, b)| [a, b]).collect();
                let f: Vec<f64> = fixed
                    .iter()
                    .flat_map(|v| [v.re as f64 * scale, v.im as f64 * scale])
                    .collect();
                let q = sqnr_db(&r, &f).ok_or("empty symbol")?;
                min = min.min(q);
                all.push(q);
            }
            mins.push(min);
        }
        let worst = mins.iter().copied().fold(f64::INFINITY, f64::min);
        let mean = all.iter().sum::<f64>() / all.len() as f64;
        println!(
            "{w:4} |             {:6.1} {:6.1} {:6.1} {:6.1} | {worst:5.1} | {mean:5.1}",
            mins[0], mins[1], mins[2], mins[3]
        );
    }
    Ok(())
}

fn main() -> ExitCode {
    let block = std::env::args().nth(1).unwrap_or_default();
    let result = match block.as_str() {
        "ofdm" => ofdm(),
        _ => {
            eprintln!("usage: g2 <block>\nblocks: ofdm");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("g2: {e}");
            ExitCode::FAILURE
        }
    }
}
