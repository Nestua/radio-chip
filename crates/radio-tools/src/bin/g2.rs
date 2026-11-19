//! `g2` — word-length sweeps for gate G2 (fixed point against the float model).
//!
//! Usage: `cargo run -q --release -p radio-tools --bin g2 -- ofdm|viterbi`
//! `ofdm`: per word length, the minimum and mean SQNR over random symbols of every modulation.
//! `viterbi [blocks]` (default 3000 per point): Eb/N0 loss at BER 1e-4 of soft-input widths, clip levels and traceback windows
//! against the full-traceback decoder on finely quantised soft values (same noise for all).
//! `ldpc [blocks]` (default 1000 per point): Eb/N0 loss at a block error rate of 10 % of the
//! integer layered min-sum decoder (input width and clip, message and total widths,
//! normalisation, iterations) against float sum-product (same noise for all).
//! The result goes into the block's gate journal; it is not a generated file.

use std::process::ExitCode;

use radio_phy::channel::Rng;
use radio_phy::convolutional::{encode, CodeRate};
use radio_phy::fixed::sqnr_db;
use radio_phy::ldpc::{LdpcCode, LdpcLength, LdpcRate};
use radio_phy::ldpc_decoder::{
    CheckRule, LdpcDecoder, MinSumConfig, Schedule, LDPC_SOFT_BITS, LDPC_SOFT_CLIP,
};
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
    /// Quantised soft values and a traceback window together: the hardware decoder.
    Both { bits: u32, clip: f64, depth: usize },
}

impl Config {
    fn label(self) -> String {
        match self {
            Config::Reference => "reference".into(),
            Config::Soft { bits, clip } => format!("soft {bits} bit, clip {clip:.1}"),
            Config::Window { depth } => format!("window {depth}"),
            Config::Both { bits, clip, depth } => format!("{bits} bit/{clip:.1} + window {depth}"),
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
                Config::Both {
                    bits: b,
                    clip,
                    depth,
                } => {
                    let q: Vec<i32> = y.iter().map(|&v| soft_quantise(v, b, clip)).collect();
                    decode_windowed(&q, rate, term, depth)
                }
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
    // The choice (docs/gates/viterbi.md): both losses together, not just each alone.
    configs.push(Config::Both {
        bits: 4,
        clip: 1.5,
        depth: 64,
    });
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

// ---------------------------------------------------------------------------------------------
// LDPC
// ---------------------------------------------------------------------------------------------

/// Block error rates the loss is read at.
const LDPC_TARGETS: [f64; 2] = [0.1, 0.01];

#[derive(Clone, Copy, PartialEq)]
enum LdpcCfg {
    /// Float sum-product, flooding, 50 iterations, exact LLRs.
    Reference,
    /// Float layered normalised min-sum, exact LLRs.
    Float { scale: f64, iters: usize },
    /// The integer decoder. Its input is the matched-filter output y (signal amplitude 1)
    /// as round(y · in_max / clip), saturated to ±in_max = ±(2^(in_bits−1) − 1): no noise
    /// variance is needed, min-sum does not depend on the input's scale.
    Fixed {
        in_bits: u32,
        clip: f64,
        cfg: MinSumConfig,
    },
}

impl LdpcCfg {
    fn label(&self) -> String {
        match self {
            LdpcCfg::Reference => "reference: sum-product".into(),
            LdpcCfg::Float { scale, iters } => format!("float layered x{scale}, {iters} it"),
            LdpcCfg::Fixed { in_bits, clip, cfg } => format!(
                "in {in_bits}b clip {clip} msg {} tot {} x{}/{}{} {} it",
                cfg.msg_bits,
                cfg.app_bits,
                cfg.scale_num,
                1 << cfg.scale_shift,
                if cfg.scale_round { "r" } else { "" },
                cfg.max_iter
            ),
        }
    }
}

/// The chosen integer decoder (docs/gates/ldpc.md); the sweep moves one parameter at a time
/// around it.
const LDPC_CHOICE: (u32, f64, MinSumConfig) =
    (LDPC_SOFT_BITS, LDPC_SOFT_CLIP, MinSumConfig::HARDWARE);

fn ldpc_configs() -> Vec<LdpcCfg> {
    let (in_bits, clip, cfg) = LDPC_CHOICE;
    let fixed = |in_bits, clip, cfg| LdpcCfg::Fixed { in_bits, clip, cfg };
    let mut v = vec![LdpcCfg::Reference];
    // The algorithm's own cost, in floating point: the scale, then the iterations.
    for scale in [1.0, 0.875, 0.8125, 0.75] {
        v.push(LdpcCfg::Float { scale, iters: 50 });
    }
    for iters in [10, 15, 20, 30] {
        v.push(LdpcCfg::Float {
            scale: 0.8125,
            iters,
        });
    }
    v.push(fixed(in_bits, clip, cfg));
    // Input: width and clip, with messages and totals wide enough not to interfere.
    let wide = MinSumConfig {
        msg_bits: 10,
        app_bits: 12,
        ..cfg
    };
    for b in [4, 5, 6, 7] {
        for c in [1.0, 1.5, 2.0, 2.5, 3.0, 4.0] {
            v.push(fixed(b, c, wide));
        }
    }
    for msg_bits in [4, 5, 6, 7] {
        for extra in [0, 1, 2, 3] {
            let c = MinSumConfig {
                msg_bits,
                app_bits: msg_bits + extra,
                ..cfg
            };
            if c != cfg {
                v.push(fixed(in_bits, clip, c));
            }
        }
    }
    // The same with one bit less at the input, at the clip that suits it at the high rates
    // (1.5): narrower messages may then do.
    for msg_bits in [4, 5, 6] {
        for extra in [1, 2, 3] {
            let c = MinSumConfig {
                msg_bits,
                app_bits: msg_bits + extra,
                ..cfg
            };
            v.push(fixed(in_bits - 1, 1.5, c));
        }
    }
    // The factor, rounded down and to nearest.
    for (scale_num, scale_shift) in [(1, 0), (15, 4), (7, 3), (13, 4), (3, 2), (11, 4)] {
        for scale_round in [false, true] {
            let c = MinSumConfig {
                scale_num,
                scale_shift,
                scale_round,
                ..cfg
            };
            if c != cfg && c.is_valid() {
                v.push(fixed(in_bits, clip, c));
            }
        }
    }
    for max_iter in [6, 8, 10, 12, 15, 20, 25, 30, 50] {
        let c = MinSumConfig { max_iter, ..cfg };
        if c != cfg {
            v.push(fixed(in_bits, clip, c));
        }
    }
    v
}

/// (block errors, iterations summed over the blocks) per configuration.
fn ldpc_point(
    code: &LdpcCode,
    rate: f64,
    ebn0_db: f64,
    configs: &[LdpcCfg],
    seeds: std::ops::Range<u64>,
) -> Result<Vec<(u64, u64)>, String> {
    let dec = LdpcDecoder::new(code);
    let sigma2 = 1.0 / (2.0 * rate * 10f64.powf(ebn0_db / 10.0));
    let mut out = vec![(0u64, 0u64); configs.len()];
    for seed in seeds {
        let mut rng = Rng::new(seed);
        let info: Vec<u8> = (0..code.k())
            .map(|_| u8::from(rng.uniform() < 0.5))
            .collect();
        let word = code.encode(&info).map_err(|e| e.to_string())?;
        let y: Vec<f64> = word
            .iter()
            .map(|&b| 1.0 - 2.0 * f64::from(b) + sigma2.sqrt() * rng.gaussian())
            .collect();
        let llr: Vec<f64> = y.iter().map(|v| 2.0 * v / sigma2).collect();
        for (k, cfg) in configs.iter().enumerate() {
            let got = match *cfg {
                LdpcCfg::Reference => {
                    dec.decode_float(&llr, CheckRule::SumProduct, Schedule::Flooding, 50)
                }
                LdpcCfg::Float { scale, iters } => {
                    dec.decode_float(&llr, CheckRule::MinSum { scale }, Schedule::Layered, iters)
                }
                LdpcCfg::Fixed { in_bits, clip, cfg } => {
                    let max = f64::from((1i32 << (in_bits - 1)) - 1);
                    let q: Vec<i32> = y
                        .iter()
                        .map(|v| (v * max / clip).round().clamp(-max, max) as i32)
                        .collect();
                    dec.decode_fixed(&q, &cfg)
                }
            };
            // A refused block would otherwise count as a correct one.
            let d = got.map_err(|e| format!("{}: {e}", cfg.label()))?;
            out[k].0 += u64::from(d.bits != word);
            out[k].1 += d.iterations as u64;
        }
    }
    Ok(out)
}

fn ldpc(blocks: u64) -> Result<(), String> {
    let configs = ldpc_configs();
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()) as u64;
    println!(
        "# G2 sweep: LDPC, {blocks} blocks per point, BPSK/AWGN, same noise for every \
         configuration; loss = Eb/N0 at a block error rate of 10 % (and of 1 %) minus the reference's; \
         a factor marked r is rounded to nearest"
    );
    println!(
        "# columns: block error rate per Eb/N0; mean iterations at the grid point where the \
         configuration's own block error rate is nearest 10 %"
    );
    for (length, rate, name, r, start) in [
        (LdpcLength::N648, LdpcRate::R1_2, "(648, 1/2)", 0.5f64, 1.0),
        (LdpcLength::N1944, LdpcRate::R1_2, "(1944, 1/2)", 0.5, 0.5),
        (LdpcLength::N1944, LdpcRate::R3_4, "(1944, 3/4)", 0.75, 1.75),
        (
            LdpcLength::N648,
            LdpcRate::R5_6,
            "(648, 5/6)",
            5.0 / 6.0,
            2.75,
        ),
        (
            LdpcLength::N1944,
            LdpcRate::R5_6,
            "(1944, 5/6)",
            5.0 / 6.0,
            2.5,
        ),
    ] {
        let code = LdpcCode::new(length, rate).ok_or("no such code")?;
        let grid: Vec<f64> = (0..8).map(|i| start + 0.25 * f64::from(i)).collect();
        let mut bler = vec![vec![0.0; grid.len()]; configs.len()];
        let mut iters = vec![vec![0.0; grid.len()]; configs.len()];
        for (gi, &ebn0) in grid.iter().enumerate() {
            let per = blocks.div_ceil(threads);
            let base = ((code.n() as u64) << 44) | ((code.k() as u64) << 32) | ((gi as u64) << 24);
            type Part = Option<Result<Vec<(u64, u64)>, String>>;
            let results: Vec<Part> = std::thread::scope(|scope| {
                let handles: Vec<_> = (0..threads)
                    .map(|t| {
                        let lo = base + (t * per).min(blocks);
                        let hi = base + ((t + 1) * per).min(blocks);
                        let (configs, code) = (&configs, &code);
                        scope.spawn(move || ldpc_point(code, r, ebn0, configs, lo..hi))
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().ok()).collect()
            });
            for part in results {
                let part = part.ok_or("a worker thread panicked")??;
                for (k, (bad, it)) in part.iter().enumerate() {
                    bler[k][gi] += *bad as f64 / blocks as f64;
                    iters[k][gi] += *it as f64 / blocks as f64;
                }
            }
        }
        // The 10 % crossing must exist; the 1 % one may be missed by a thin run.
        let reference = LDPC_TARGETS.map(|t| crossing(&grid, &bler[0], t));
        let at = |x: Option<f64>| x.map_or("n/a".to_string(), |x| format!("{x:.3} dB"));
        if reference[0].is_none() {
            return Err("reference misses the target".into());
        }
        println!(
            "\n## {name}: reference reaches a block error rate of 10 % at {}, of 1 % at {}",
            at(reference[0]),
            at(reference[1])
        );
        print!("# {:<40}", "config");
        for g in &grid {
            print!(" {g:>7.2}");
        }
        println!(" | loss 10 % |     1 % | iter");
        for (k, cfg) in configs.iter().enumerate() {
            print!("  {:<40}", cfg.label());
            for b in &bler[k] {
                print!(" {b:>7.4}");
            }
            let loss = |i: usize| {
                crossing(&grid, &bler[k], LDPC_TARGETS[i])
                    .zip(reference[i])
                    .map_or("    n/a".to_string(), |(x, r)| format!("{:+7.3}", x - r))
            };
            // Iterations where this configuration's block error rate is nearest 10 %.
            let near = (0..grid.len())
                .min_by(|&a, &b| {
                    (bler[k][a] - LDPC_TARGETS[0])
                        .abs()
                        .total_cmp(&(bler[k][b] - LDPC_TARGETS[0]).abs())
                })
                .unwrap_or(0);
            println!(" |    {} | {} | {:>4.1}", loss(0), loss(1), iters[k][near]);
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
        "ldpc" => ldpc(
            std::env::args()
                .nth(2)
                .and_then(|b| b.parse().ok())
                .unwrap_or(1000),
        ),
        _ => {
            eprintln!("usage: g2 <block>\nblocks: ofdm, viterbi, ldpc");
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
