//! `g2` — word-length sweeps for gate G2 (fixed point against the float model).
//!
//! Usage: `cargo run -q --release -p radio-tools --bin g2 -- ofdm|viterbi`
//! `ofdm`: per word length, the minimum and mean SQNR over random symbols of every modulation.
//! `viterbi [blocks]` (default 3000 per point): Eb/N0 loss at BER 1e-4 of soft-input widths, clip levels and traceback windows
//! against the full-traceback decoder on finely quantised soft values (same noise for all).
//! The result goes into the block's gate journal; it is not a generated file.

use std::process::ExitCode;

use radio_phy::channel::Rng;
use radio_phy::convolutional::{encode, CodeRate};
use radio_phy::fixed::sqnr_db;
use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, modulate_symbol_f64, IfftConfig};
use radio_phy::viterbi::{decode, decode_windowed, metric_spread, Termination};

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

/// One decoder configuration of the Viterbi sweep.
#[derive(Clone, Copy, Debug)]
enum Config {
    /// 2^-12 soft resolution, full traceback: the reference.
    Reference,
    /// `bits`-bit symmetric soft values, saturating at ±clip (signal amplitude 1), full traceback.
    Soft { bits: u32, clip: f64 },
    /// Reference soft values, traceback window `depth`.
    Window { depth: usize },
}

impl Config {
    fn label(self) -> String {
        match self {
            Config::Reference => "reference".into(),
            Config::Soft { bits, clip } => format!("soft {bits} bit, clip {clip:.1}"),
            Config::Window { depth } => format!("window {depth}"),
        }
    }
}

const VIT_BLOCK: usize = 1002; // + 6 tail = 1008: whole periods of every puncturing pattern
const VIT_TARGET: f64 = 1e-4;

fn soft_quantise(y: f64, bits: u32, clip: f64) -> i32 {
    let levels = (1i32 << (bits - 1)) - 1;
    let step = clip / f64::from(levels);
    ((y / step).round() as i32).clamp(-levels, levels)
}

/// Bit errors per configuration for `blocks` blocks at one (rate, Eb/N0), plus the largest
/// metric spread seen per soft configuration.
fn viterbi_point(
    rate: CodeRate,
    code_rate: f64,
    ebn0_db: f64,
    configs: &[Config],
    seeds: std::ops::Range<u64>,
) -> (Vec<u64>, Vec<i64>) {
    let sigma = (1.0 / (2.0 * code_rate * 10f64.powf(ebn0_db / 10.0))).sqrt();
    let mut errors = vec![0u64; configs.len()];
    let mut spread = vec![0i64; configs.len()];
    for seed in seeds {
        let mut r = Rng::new(seed);
        let mut bits: Vec<u8> = (0..VIT_BLOCK).map(|_| (r.next_u64() & 1) as u8).collect();
        bits.extend([0; 6]);
        let Ok(tx) = encode(&bits, rate) else {
            continue;
        };
        let y: Vec<f64> = tx
            .iter()
            .map(|&c| if c == 0 { 1.0 } else { -1.0 } + sigma * r.gaussian())
            .collect();
        let fine: Vec<i32> = y.iter().map(|v| (v * 4096.0).round() as i32).collect();
        let term = Termination::ZeroAfter(VIT_BLOCK + 6);
        for (k, cfg) in configs.iter().enumerate() {
            let got = match *cfg {
                Config::Reference => decode(&fine, rate, term),
                Config::Soft { bits: b, clip } => {
                    let q: Vec<i32> = y.iter().map(|&v| soft_quantise(v, b, clip)).collect();
                    if let Ok(s) = metric_spread(&q, rate) {
                        spread[k] = spread[k].max(s);
                    }
                    decode(&q, rate, term)
                }
                Config::Window { depth } => decode_windowed(&fine, rate, term, depth),
            };
            if let Ok(got) = got {
                errors[k] += got[..VIT_BLOCK]
                    .iter()
                    .zip(&bits[..VIT_BLOCK])
                    .filter(|(a, b)| a != b)
                    .count() as u64;
            }
        }
    }
    (errors, spread)
}

/// Eb/N0 where the BER crosses `target`, by log-linear interpolation on the grid.
fn crossing(grid: &[f64], ber: &[f64], target: f64) -> Option<f64> {
    grid.windows(2).zip(ber.windows(2)).find_map(|(g, b)| {
        if b[0] >= target && b[1] < target && b[1] > 0.0 {
            let t = (b[0].log10() - target.log10()) / (b[0].log10() - b[1].log10());
            Some(g[0] + t * (g[1] - g[0]))
        } else {
            None
        }
    })
}

fn viterbi(blocks: u64) -> Result<(), String> {
    let mut configs = vec![Config::Reference];
    for bits in [3, 4, 5, 6] {
        for clip in [1.0, 1.5, 2.0, 2.5] {
            configs.push(Config::Soft { bits, clip });
        }
    }
    for depth in [24, 32, 48, 64, 96] {
        configs.push(Config::Window { depth });
    }
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()) as u64;
    println!(
        "# G2 sweep: Viterbi, {blocks} blocks x {VIT_BLOCK} bits per point, BPSK/AWGN, \
         same noise for every configuration; loss = Eb/N0 at BER {VIT_TARGET:e} minus the reference's"
    );
    for (rate, name, code_rate, grid) in [
        (CodeRate::R1_2, "1/2", 0.5f64, [2.5, 3.0, 3.5, 4.0, 4.5]),
        (CodeRate::R2_3, "2/3", 2.0 / 3.0, [3.5, 4.0, 4.5, 5.0, 5.5]),
        (CodeRate::R3_4, "3/4", 0.75, [4.0, 4.5, 5.0, 5.5, 6.0]),
    ] {
        let mut ber = vec![vec![0.0; grid.len()]; configs.len()];
        let mut spread = vec![0i64; configs.len()];
        for (gi, &ebn0) in grid.iter().enumerate() {
            let per = blocks.div_ceil(threads);
            let base = (u64::from(name.as_bytes()[2]) << 40) | ((gi as u64) << 32);
            let results: Vec<(Vec<u64>, Vec<i64>)> = std::thread::scope(|scope| {
                let handles: Vec<_> = (0..threads)
                    .map(|t| {
                        let lo = base + t * per;
                        let hi = base + ((t + 1) * per).min(blocks);
                        let configs = &configs;
                        scope.spawn(move || viterbi_point(rate, code_rate, ebn0, configs, lo..hi))
                    })
                    .collect();
                handles.into_iter().filter_map(|h| h.join().ok()).collect()
            });
            for (errs, spr) in results {
                for k in 0..configs.len() {
                    ber[k][gi] += errs[k] as f64 / (blocks as f64 * VIT_BLOCK as f64);
                    spread[k] = spread[k].max(spr[k]);
                }
            }
        }
        let reference =
            crossing(&grid, &ber[0], VIT_TARGET).ok_or("reference misses the target")?;
        println!("\n## rate {name}: reference reaches BER {VIT_TARGET:e} at {reference:.3} dB");
        print!("# {:<24}", "config");
        for g in grid {
            print!(" {g:>8.1}");
        }
        println!(" | loss dB | max spread");
        for (k, cfg) in configs.iter().enumerate() {
            print!("  {:<24}", cfg.label());
            for b in &ber[k] {
                print!(" {b:>8.1e}");
            }
            let loss = crossing(&grid, &ber[k], VIT_TARGET)
                .map_or("   n/a".to_string(), |x| format!("{:+7.3}", x - reference));
            let spr = if spread[k] > 0 {
                spread[k].to_string()
            } else {
                "-".into()
            };
            println!(" | {loss} | {spr}");
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let block = std::env::args().nth(1).unwrap_or_default();
    let result = match block.as_str() {
        "ofdm" => ofdm(),
        "viterbi" => viterbi(
            std::env::args()
                .nth(2)
                .and_then(|b| b.parse().ok())
                .unwrap_or(3000),
        ),
        _ => {
            eprintln!("usage: g2 <block>\nblocks: ofdm, viterbi");
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
