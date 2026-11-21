//! `rxagc` — frames lost by the whole receiver behind the gain-control model (R0-09 G2,
//! stage B, second part; G0 and results in `docs/gates/rx_ofdm.md`). Multi-threaded.
//!
//! Usage: `cargo run -q --release -p radio-tools --bin rxagc -- <command> [frames per point]`
//! - `per`: frames lost against SNR, AWGN and TGn D, the noise at −30 dBFS at the largest gain.
//! - `level`: frames lost against the noise level at the largest gain (the lower edge of the
//!   input level), near 10 % PER.
//! - `dc`: the DC offsets of `rxfront agcdc` through the integer chain.
//! - `pole`: what the DC blocker's pole costs 54 Mb/s frames with no offset to remove.
//! - `cfo`: that cost against the size of the clock offset (ADR 0008).
//! - `cand`: what the decoder could be handed instead, under the DC offsets of `dc` (ADR 0008).
//!
//! The scene: a 1000-octet frame, a clock offset uniform in ±40 ppm that sets CFO and
//! sample-clock offset at 5.8 GHz (as `rxper`), noise, then `channel::FastAttackAgc` (power
//! over 16 samples, the lock left on a large overload, the gain held from the first lock
//! inside the packet), a DC offset per gain state, a phase jump per gain change, a 12-bit ADC.
//!
//! Every frame's random stream is drawn from the row and the frame's number: a report does
//! not depend on the number of threads, and the columns of a row are the same frames.
//!
//! SNR is over the whole sampled band, as everywhere in the simulator.

use std::f64::consts::PI;
use std::process::ExitCode;

use radio_phy::channel::{
    agc_transient, awgn, fractional_delay, frequency_offset, mean_power, resample, tgn_taps,
    Doppler, FastAttackAgc, GainDc, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, correct, dc_block, ltf_dc_offset, RxConfig};
use radio_phy::rx_front_fixed::{
    acquire_fixed, correct_fixed, dc_block_fixed, ltf_dc_offset_fixed, remove_dc_fixed, to_adc,
    FixedRxConfig, ADC_FULL_SCALE, IQ,
};
use radio_phy::rx_ofdm::{decode_ppdu_from, RxOfdmConfig, Tracking};
use radio_phy::rx_ofdm_fixed::{decode_ppdu_fixed, FixedOfdmConfig, FixedTracking};

type C = (f64, f64);

const FS: f64 = 20e6;
/// Carrier of the scenarios: the two ends' oscillators set both the CFO and the sample-clock
/// offset (IEEE 802.11-2020 §17.3.9.5–6), ±40 ppm between them at most.
const CARRIER_HZ: f64 = 5.8e9;
const OCTETS: usize = 1000;
/// Phase jump per gain change, uniform within ± this (assumed, as in `rxfront agcdc`).
const JUMP_DEG: f64 = 30.0;
/// Level of the frame at the ADC when the gain is the one that knows the frame (`rxper`'s).
const KNOWN_DBFS: f64 = -12.0;
/// Pole of both DC blockers unless a column says otherwise, 2^(−7).
const DC_SHIFT: u32 = 7;

fn threads() -> usize {
    std::thread::available_parallelism().map_or(4, |n| n.get())
}

/// Runs `f(thread index)` on every core and collects the results.
fn par<T: Send>(f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    let n = threads();
    std::thread::scope(|s| {
        let handles: Vec<_> = (0..n)
            .map(|i| {
                s.spawn({
                    let f = &f;
                    move || f(i)
                })
            })
            .collect();
        handles.into_iter().filter_map(|h| h.join().ok()).collect()
    })
}

fn frame(psdu: &[u8], rate: Rate) -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, 0x5d, &cfg)
        .map(|f| {
            f.iter()
                .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
                .collect()
        })
        .unwrap_or_default()
}

#[derive(Clone, Copy)]
enum Channel {
    Awgn,
    Tgn(TgnModel),
}

/// One operating point. Levels in dBFS at the ADC (0 dBFS: a mean |v|² of full scale squared).
#[derive(Clone, Copy)]
struct Point {
    /// The frames of two points with the same `scene` are the same up to the gain control.
    scene: u64,
    rate: Rate,
    snr_db: f64,
    ch: Channel,
    /// Noise level at the largest gain.
    idle_dbfs: f64,
    /// `GainDc::per_index`, −∞ for none.
    per_index_dbfs: f64,
    /// `GainDc::before_gain` at the largest gain, −∞ for none.
    before_dbfs: f64,
    /// The clock offset's size drawn in this band of ppm instead of 0 to 40 (the sign and
    /// every other draw of the frame as without it).
    ppm_band: Option<(f64, f64)>,
}

/// A receiver chain on the ADC's samples.
#[derive(Clone, Copy, PartialEq)]
enum Chain {
    /// Float DC blocker, float front, `rx_ofdm` at its defaults.
    Float,
    /// `dc_block_fixed`, `RxFrontFixed`, `correct_fixed` to `out_bits`, `decode_ppdu_fixed`
    /// with `in_bits = out_bits` and this `in_shift`.
    Int { out_bits: u32, in_shift: u32 },
}

#[derive(Clone, Copy)]
struct Column {
    name: &'static str,
    chain: Chain,
    /// The ADC behind a gain that knows the frame (it at −12 dBFS, no DC offset) instead of
    /// the gain control.
    known: bool,
    /// The decoder is handed the samples before the DC blocker (the acquisition is not).
    raw: bool,
    /// The DC blocker's pole, 2^(−shift).
    shift: u32,
    /// The decoder is handed the samples before the DC blocker less the offset estimated from
    /// the two long training symbols (`ltf_dc_offset`, `ltf_dc_offset_fixed`).
    ltf: bool,
}

const fn col(name: &'static str, chain: Chain) -> Column {
    Column {
        name,
        chain,
        known: false,
        raw: false,
        shift: DC_SHIFT,
        ltf: false,
    }
}

const INT_12_2: Chain = Chain::Int {
    out_bits: 12,
    in_shift: 2,
};
const INT_13_1: Chain = Chain::Int {
    out_bits: 13,
    in_shift: 1,
};
const INT_13_2: Chain = Chain::Int {
    out_bits: 13,
    in_shift: 2,
};

/// SplitMix64's output function: the frame's seed from the row and the frame's number.
fn mix(a: u64, b: u64) -> u64 {
    let mut z = a
        .wrapping_mul(0x9e37_79b9_7f4a_7c15)
        .wrapping_add(b)
        .wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// What one frame did.
struct Outcome {
    /// Per column: the PSDU came back.
    ok: Vec<bool>,
    /// The gain would have changed again before the packet's end without the hold.
    held: bool,
    /// Mean power of the frame's span at the ADC behind the gain control, full scale² = 1.
    level: f64,
    /// Gain changes of the control up to the hold.
    changes: usize,
}

fn one(pt: &Point, columns: &[Column], agc: &FastAttackAgc, number: u64) -> Outcome {
    let mut rng = Rng::new(mix(pt.scene, number));
    let full = f64::from(ADC_FULL_SCALE);
    let cfg_float = RxConfig {
        dc_shift: None,
        ..RxConfig::new(FS)
    };
    let fx = FixedRxConfig::default();
    let ofdm = RxOfdmConfig::default();

    let psdu: Vec<u8> = (0..OCTETS).map(|_| (rng.next_u64() & 0xff) as u8).collect();
    let f = frame(&psdu, pt.rate);
    let lead = 600 + (rng.uniform() * 50.0) as usize;
    let mut x = vec![(0.0, 0.0); lead];
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); 200]);
    let x = match pt.ch {
        Channel::Awgn => fractional_delay(&x, rng.uniform()),
        Channel::Tgn(m) => {
            Multipath::new(tgn_taps(m, false), Doppler::Static, 1, FS, &mut rng).apply(&x)
        }
    };
    // The transmitter's oscillator fast by p ppm against ours (as `rxper`).
    let v = rng.uniform() * 2.0 - 1.0;
    let ppm = match pt.ppm_band {
        None => v * 40.0,
        Some((lo, hi)) => (lo + (hi - lo) * v.abs()).copysign(v),
    };
    let x = resample(&x, ppm);
    let x = frequency_offset(&x, ppm * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
    let p = mean_power(&f);
    let x = awgn(&x, pt.snr_db, p, &mut rng);
    let end = (lead + f.len()).min(x.len());
    let span = &x[lead.min(end)..end];

    // Everything drawn from here on belongs to the gain control: two points of one scene
    // that differ in their levels share the frame, the channel and the noise above.
    let k = (10f64.powf((pt.idle_dbfs - agc.max_gain_db + pt.snr_db) / 10.0) / p).sqrt();
    let xs: Vec<C> = x.iter().map(|s| (s.0 * k, s.1 * k)).collect();
    let a = 10f64.powf((pt.before_dbfs - agc.max_gain_db) / 20.0);
    let th0 = 2.0 * PI * rng.uniform();
    let dc = GainDc {
        per_index: 10f64.powf(pt.per_index_dbfs / 20.0),
        seed: rng.next_u64(),
        before_gain: (a * th0.cos(), a * th0.sin()),
    };
    // The control's detectors see the offset with the signal.
    let mut t = agc.run_dc(&xs, Some(&dc));
    // The gain is held from the first lock inside the packet (assumed: a hold driven by the
    // receiver, ADR 0008's question); without it a peak of the data unlocks it again.
    let mut held = false;
    if let Some(&at) = t.locks.iter().find(|l| **l > lead) {
        held = t.changes.iter().any(|c| c.0 > at && c.0 < end);
        t.changes.retain(|c| c.0 <= at);
        t.locks.retain(|l| *l <= at);
        t.unlocks.retain(|u| *u < at);
    }
    let changes = t.changes.len().saturating_sub(1);
    let (settled, states) = t.gain_states_dc(JUMP_DEG, &dc, &mut rng);
    let g = 10f64.powf(settled / 20.0);
    let at_settled: Vec<C> = xs.iter().map(|s| (s.0 * g, s.1 * g)).collect();
    let adc_agc = to_adc(&agc_transient(&at_settled, &states), full);
    let level = mean_power(span) * k * k * g * g;

    let adc_known = columns.iter().any(|c| c.known).then(|| {
        let gain = full * (10f64.powf(KNOWN_DBFS / 10.0) / mean_power(span)).sqrt();
        to_adc(&x, gain)
    });

    // One front per kind, ADC stream and pole, made when a column first asks for it.
    type FloatFront = (Vec<C>, Vec<C>, Option<(usize, f64)>);
    type IntFront = (Vec<IQ>, Vec<IQ>, Option<(usize, i64, f64)>);
    let mut float_fronts: Vec<((bool, u32), FloatFront)> = Vec::new();
    let mut int_fronts: Vec<((bool, u32), IntFront)> = Vec::new();
    let empty: Vec<(i16, i16)> = Vec::new();
    let ok = columns
        .iter()
        .map(|c| {
            let adc = if c.known {
                adc_known.as_ref().unwrap_or(&empty)
            } else {
                &adc_agc
            };
            let key = (c.known, c.shift);
            match c.chain {
                Chain::Float => {
                    if float_fronts.iter().all(|f| f.0 != key) {
                        let raw: Vec<C> = adc
                            .iter()
                            .map(|s| (f64::from(s.0) / full, f64::from(s.1) / full))
                            .collect();
                        let blocked = dc_block(&raw, c.shift);
                        let acq = acquire(&blocked, 0, &cfg_float).map(|a| (a.fft_start, a.cfo_hz));
                        float_fronts.push((key, (raw, blocked, acq)));
                    }
                    let Some((_, (raw, blocked, acq))) = float_fronts.iter().find(|f| f.0 == key)
                    else {
                        return false;
                    };
                    acq.is_some_and(|(fft_start, cfo_hz)| {
                        let less: Vec<C>;
                        let handed = if c.ltf {
                            let d = ltf_dc_offset(raw, fft_start, cfo_hz, FS).unwrap_or((0.0, 0.0));
                            less = raw.iter().map(|s| (s.0 - d.0, s.1 - d.1)).collect();
                            &less
                        } else if c.raw {
                            raw
                        } else {
                            blocked
                        };
                        let y = correct(handed, cfo_hz, FS);
                        let start = Tracking::from_clock_offset(cfo_hz / CARRIER_HZ * 1e6);
                        decode_ppdu_from(&y, fft_start, &ofdm, start).is_ok_and(|d| d.psdu == psdu)
                    })
                }
                Chain::Int { out_bits, in_shift } => {
                    if int_fronts.iter().all(|f| f.0 != key) {
                        let cfg = RxConfig {
                            dc_shift: Some(c.shift),
                            ..RxConfig::new(FS)
                        };
                        let raw: Vec<IQ> = adc
                            .iter()
                            .map(|s| (i64::from(s.0), i64::from(s.1)))
                            .collect();
                        let blocked = dc_block_fixed(adc, &cfg, &fx);
                        let acq = acquire_fixed(adc, &cfg, &fx)
                            .map(|a| (a.fft_start, a.cfo_phase, a.cfo_hz(&fx, FS)));
                        int_fronts.push((key, (raw, blocked, acq)));
                    }
                    let Some((_, (raw, blocked, acq))) = int_fronts.iter().find(|f| f.0 == key)
                    else {
                        return false;
                    };
                    acq.is_some_and(|(fft_start, cfo_phase, cfo_hz)| {
                        let fo = FixedOfdmConfig {
                            in_bits: out_bits,
                            in_shift,
                            ..FixedOfdmConfig::default()
                        };
                        let less: Vec<IQ>;
                        let handed = if c.ltf {
                            let d = ltf_dc_offset_fixed(adc, fft_start, cfo_phase, &fx)
                                .unwrap_or((0, 0));
                            less = remove_dc_fixed(adc, d);
                            &less
                        } else if c.raw {
                            raw
                        } else {
                            blocked
                        };
                        let z = correct_fixed(handed, cfo_phase, out_bits, &fx);
                        let start =
                            FixedTracking::from_clock_offset(cfo_hz / CARRIER_HZ * 1e6, &fo);
                        decode_ppdu_fixed(&z, fft_start, &ofdm, &fo, start)
                            .is_ok_and(|d| d.psdu == psdu)
                    })
                }
            }
        })
        .collect();
    Outcome {
        ok,
        held,
        level,
        changes,
    }
}

/// A row's sums over its frames.
#[derive(Clone)]
struct Tally {
    frames: usize,
    lost: Vec<usize>,
    /// Per column against the reference column: frames lost by the column alone, and by the
    /// reference alone.
    only: Vec<(usize, usize)>,
    held: usize,
    level: f64,
    changes: usize,
}

fn run(pt: &Point, columns: &[Column], reference: usize, frames: usize) -> Tally {
    let agc = FastAttackAgc {
        power_samples: 16,
        unlock_on_overload: true,
        ..FastAttackAgc::ad9361_reset(2)
    };
    let n = threads();
    let zero = Tally {
        frames: 0,
        lost: vec![0; columns.len()],
        only: vec![(0, 0); columns.len()],
        held: 0,
        level: 0.0,
        changes: 0,
    };
    let parts = par(|th| {
        let mut t = zero.clone();
        for number in (th..frames).step_by(n) {
            let o = one(pt, columns, &agc, number as u64);
            let r = o.ok.get(reference).copied().unwrap_or(true);
            for (c, &ok) in o.ok.iter().enumerate() {
                t.lost[c] += usize::from(!ok);
                t.only[c].0 += usize::from(!ok && r);
                t.only[c].1 += usize::from(ok && !r);
            }
            t.frames += 1;
            t.held += usize::from(o.held);
            t.level += o.level;
            t.changes += o.changes;
        }
        t
    });
    parts.into_iter().fold(zero, |mut a, t| {
        for c in 0..a.lost.len() {
            a.lost[c] += t.lost[c];
            a.only[c].0 += t.only[c].0;
            a.only[c].1 += t.only[c].1;
        }
        a.frames += t.frames;
        a.held += t.held;
        a.level += t.level;
        a.changes += t.changes;
        a
    })
}

fn header(columns: &[Column], reference: usize) {
    println!(
        "# Each cell: frames lost (not decoded to the PSDU), then +a−b: lost by this column alone and\n\
         # by the column '{}' alone (the same frames). A lost frame is one not acquired, acquired\n\
         # wrongly (the first acquisition of the stream is the only one tried) or not decoded.\n\
         # level: signal and noise of the frame's span at the settled gain, before the ADC clips and\n\
         # without the DC offset, dBFS, the mean of the powers over the frames. chg: gain changes per\n\
         # frame up to the hold (with no lock inside the packet: all of the stream's). held: frames in\n\
         # which the gain would have changed again before the packet's end without the hold.",
        columns.get(reference).map_or("", |c| c.name)
    );
}

fn head_line(first: &str, columns: &[Column]) -> String {
    format!(
        "# {first:<44} {:>6} {:>6} {:>5} {:>5} {}",
        "frames",
        "level",
        "chg",
        "held",
        columns
            .iter()
            .map(|c| format!("{:>16}", c.name))
            .collect::<Vec<_>>()
            .join(" ")
    )
}

fn row_line(name: &str, t: &Tally, reference: usize) -> String {
    let f = t.frames.max(1) as f64;
    format!(
        "  {name:<44} {:>6} {:>6.1} {:>5.2} {:>5} {}",
        t.frames,
        10.0 * (t.level / f).log10(),
        t.changes as f64 / f,
        t.held,
        (0..t.lost.len())
            .map(|c| if c == reference {
                format!("{:>16}", t.lost[c])
            } else {
                format!(
                    "{:>16}",
                    format!("{} +{}−{}", t.lost[c], t.only[c].0, t.only[c].1)
                )
            })
            .collect::<Vec<_>>()
            .join(" ")
    )
}

/// SNR at which the PER crosses 10 %, by linear interpolation of log10(PER) between the two
/// grid points around it; `None` if it does not cross inside the grid.
fn crossing(points: &[(f64, f64)]) -> Option<f64> {
    points.windows(2).find_map(|w| {
        let ((s0, p0), (s1, p1)) = (w[0], w[1]);
        (p0 > 0.1 && p1 <= 0.1).then(|| {
            let (l0, l1) = (p0.log10(), p1.max(1e-4).log10());
            s0 + (s1 - s0) * (l0 + 1.0) / (l0 - l1)
        })
    })
}

fn preamble_text(what: &str, frames: usize) {
    println!(
        "# rxagc {what}: {frames} frames per point of {OCTETS} octets, a new PSDU per frame, clock offset\n\
         # uniform in ±40 ppm (CFO and sample clock, carrier 5.8 GHz), channel::FastAttackAgc (power\n\
         # over 16 samples, lock left on a large overload, 2 detector clocks per sample, lock level\n\
         # −10 dBFS, largest gain 73 dB), the gain held from the first lock inside the packet\n\
         # (assumed), phase jump ±{JUMP_DEG}° per gain change (assumed), 12-bit ADC.\n\
         # Chains on the ADC's samples:\n\
         #   float     = float DC blocker (pole 2^-{DC_SHIFT}), float front, rx_ofdm (8 bits, clip 3);\n\
         #   int O/s   = dc_block_fixed (pole 2^-{DC_SHIFT}), RxFrontFixed, correct_fixed to O bits,\n\
         #               decode_ppdu_fixed with in_bits O and in_shift s, tracker started from the\n\
         #               CFO word (its conversion to ppm is the one float between the ADC and\n\
         #               the PSDU); the int columns of a row share one acquisition;\n\
         #   known     = the same chain behind a gain that knows the frame (it at {KNOWN_DBFS} dBFS at\n\
         #               the ADC, no gain control, no DC offset);\n\
         #   raw       = the decoder handed the samples before the DC blocker.\n\
         # A frame's random stream depends on the row and the frame's number only."
    );
}

fn none() -> f64 {
    f64::NEG_INFINITY
}

/// Frames lost against SNR through the gain control: the grids of `rxper fixed` around the
/// 10 % crossings, then points where the noise loses nothing and the gain moves.
fn per(frames: usize) {
    let columns = [
        Column {
            known: true,
            ..col("float known", Chain::Float)
        },
        Column {
            known: true,
            ..col("int 12/2 known", INT_12_2)
        },
        col("float", Chain::Float),
        col("int 12/2", INT_12_2),
        col("int 13/1", INT_13_1),
        col("int 13/2", INT_13_2),
    ];
    let reference = 2;
    const IDLE: f64 = -30.0;
    preamble_text("per", frames);
    println!("# Noise at {IDLE} dBFS at the largest gain; no DC offset.");
    header(&columns, reference);
    let grids: [(&str, Channel, [f64; 8], f64, usize); 2] = [
        (
            "AWGN",
            Channel::Awgn,
            [2.5, 3.5, 5.5, 7.5, 10.5, 13.5, 17.5, 18.5],
            0.5,
            6,
        ),
        (
            "TGn D, static",
            Channel::Tgn(TgnModel::D),
            [6.0, 10.0, 10.0, 13.5, 15.0, 19.0, 22.0, 24.5],
            1.0,
            6,
        ),
    ];
    for (gi, (name, ch, start, step, points)) in grids.into_iter().enumerate() {
        println!("\n## {name}: around 10 % PER");
        println!("{}", head_line("rate, SNR", &columns));
        let mut summary = Vec::new();
        for (ri, rate) in Rate::ALL.into_iter().enumerate() {
            let mut curves = vec![Vec::new(); columns.len()];
            for k in 0..points {
                let snr = start[ri] + step * k as f64;
                let pt = Point {
                    scene: (1000 + gi * 100 + ri * 10 + k) as u64,
                    rate,
                    snr_db: snr,
                    ch,
                    idle_dbfs: IDLE,
                    per_index_dbfs: none(),
                    before_dbfs: none(),
                    ppm_band: None,
                };
                let t = run(&pt, &columns, reference, frames);
                for (c, curve) in curves.iter_mut().enumerate() {
                    curve.push((snr, t.lost[c] as f64 / t.frames.max(1) as f64));
                }
                println!(
                    "{}",
                    row_line(
                        &format!("{:>2} Mb/s {snr:>4.1} dB", rate.mbps()),
                        &t,
                        reference
                    )
                );
            }
            summary.push((rate, curves.iter().map(|c| crossing(c)).collect::<Vec<_>>()));
        }
        println!(
            "# SNR at 10 % PER (dB), and the difference to '{}':",
            columns[0].name
        );
        for (rate, cross) in summary {
            let mut line = format!("#   {:>2} Mb/s:", rate.mbps());
            for (k, c) in cross.iter().enumerate() {
                line += &match (c, cross[0]) {
                    (Some(c), Some(first)) if k > 0 => {
                        format!("  {} {c:.2} ({:+.2})", columns[k].name, c - first)
                    }
                    (Some(c), _) => format!("  {} {c:.2}", columns[k].name),
                    _ => format!("  {} —", columns[k].name),
                };
            }
            println!("{line}");
        }
    }
    for (gi, (name, ch)) in [
        ("AWGN", Channel::Awgn),
        ("TGn D, static", Channel::Tgn(TgnModel::D)),
    ]
    .into_iter()
    .enumerate()
    {
        println!("\n## {name}: above the noise's reach");
        println!("{}", head_line("rate, SNR", &columns));
        for (ri, rate) in [Rate::R6, Rate::R24, Rate::R54].into_iter().enumerate() {
            for (si, snr) in [30.0, 35.0, 40.0, 50.0, 60.0].into_iter().enumerate() {
                let pt = Point {
                    scene: (2000 + gi * 100 + ri * 10 + si) as u64,
                    rate,
                    snr_db: snr,
                    ch,
                    idle_dbfs: IDLE,
                    per_index_dbfs: none(),
                    before_dbfs: none(),
                    ppm_band: None,
                };
                let t = run(&pt, &columns, reference, frames);
                println!(
                    "{}",
                    row_line(
                        &format!("{:>2} Mb/s {snr:>4.1} dB", rate.mbps()),
                        &t,
                        reference
                    )
                );
            }
        }
    }
}

/// The lower edge of the input level: the noise at the largest gain from −24 to −60 dBFS at
/// the ADC, near 10 % PER, where the gain control has nothing left to add.
fn level(frames: usize) {
    let columns = [
        Column {
            known: true,
            ..col("float known", Chain::Float)
        },
        col("float", Chain::Float),
        col("int 12/2", INT_12_2),
        col("int 13/1", INT_13_1),
        col("int 13/2", INT_13_2),
    ];
    let reference = 1;
    preamble_text("level", frames);
    println!(
        "# No DC offset. The rows of one rate are the same frames, channels and noise at every level."
    );
    header(&columns, reference);
    let points: [(&str, Rate, f64, Channel); 5] = [
        ("AWGN  6 Mb/s  3.5 dB", Rate::R6, 3.5, Channel::Awgn),
        ("AWGN 24 Mb/s 11.5 dB", Rate::R24, 11.5, Channel::Awgn),
        ("AWGN 54 Mb/s 20.0 dB", Rate::R54, 20.0, Channel::Awgn),
        (
            "TGn D  6 Mb/s  8.5 dB",
            Rate::R6,
            8.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn D 54 Mb/s 26.5 dB",
            Rate::R54,
            26.5,
            Channel::Tgn(TgnModel::D),
        ),
    ];
    for (pi, (name, rate, snr, ch)) in points.into_iter().enumerate() {
        println!("\n## {name}");
        println!("{}", head_line("noise at the largest gain", &columns));
        for idle in [-24.0, -30.0, -36.0, -42.0, -48.0, -54.0, -60.0] {
            let pt = Point {
                scene: 3000 + pi as u64,
                rate,
                snr_db: snr,
                ch,
                idle_dbfs: idle,
                per_index_dbfs: none(),
                before_dbfs: none(),
                ppm_band: None,
            };
            let t = run(&pt, &columns, reference, frames);
            println!("{}", row_line(&format!("{idle:>5.0} dBFS"), &t, reference));
        }
    }
}

/// The DC offsets of `rxfront agcdc` through the integer chain, the blocker as it is.
fn dc(frames: usize) {
    let columns = [
        col("float", Chain::Float),
        Column {
            raw: true,
            ..col("float raw", Chain::Float)
        },
        col("int 12/2", INT_12_2),
        col("int 13/2", INT_13_2),
        Column {
            raw: true,
            ..col("int 13/2 raw", INT_13_2)
        },
    ];
    let reference = 0;
    const IDLE: f64 = -30.0;
    preamble_text("dc", frames);
    println!(
        "# Noise at {IDLE} dBFS at the largest gain, AWGN. DC per gain state: channel::GainDc, levels in\n\
         # dBFS at the ADC; 'per index' has that size at every gain with a direction per index,\n\
         # 'before the gain' has that size at the largest gain (73 dB) and falls with the gain. The\n\
         # gain control's detectors see the offset with the signal. Both blockers are told nothing.\n\
         # The rows of one rate and SNR are the same frames at every offset."
    );
    header(&columns, reference);
    let cases: [(&str, f64, f64); 7] = [
        ("no DC", none(), none()),
        ("per index −40 dBFS", -40.0, none()),
        ("per index −30 dBFS", -30.0, none()),
        ("per index −20 dBFS", -20.0, none()),
        ("before the gain, −30 dBFS at the largest", none(), -30.0),
        ("before the gain, −20 dBFS at the largest", none(), -20.0),
        ("before the gain, −10 dBFS at the largest", none(), -10.0),
    ];
    for (ri, rate) in [Rate::R54, Rate::R6].into_iter().enumerate() {
        let snrs: &[f64] = if ri == 0 {
            &[25.0, 30.0, 40.0, 60.0]
        } else {
            &[10.0, 30.0, 60.0]
        };
        println!("\n## {} Mb/s", rate.mbps());
        println!("{}", head_line("DC, SNR", &columns));
        for (name, per_index, before) in cases {
            for (si, &snr) in snrs.iter().enumerate() {
                let pt = Point {
                    scene: (4000 + ri * 10 + si) as u64,
                    rate,
                    snr_db: snr,
                    ch: Channel::Awgn,
                    idle_dbfs: IDLE,
                    per_index_dbfs: per_index,
                    before_dbfs: before,
                    ppm_band: None,
                };
                let t = run(&pt, &columns, reference, frames);
                println!("{}", row_line(&format!("{name}, {snr} dB"), &t, reference));
            }
        }
    }
}

/// What the DC blocker's own pole costs with no offset to remove: the no-DC rows of `dc` at
/// 54 Mb/s (the same frames), then points near 10 % PER and through TGn D, with slower poles
/// in both blockers. The integer blocker keeps
/// F = 10 fractional bits, which a pole of 2^(−9) still leaves a bit above.
fn pole(frames: usize) {
    let with = |name: &'static str, chain: Chain, shift: u32| Column {
        shift,
        ..col(name, chain)
    };
    let columns = [
        Column {
            raw: true,
            ..col("float raw", Chain::Float)
        },
        col("float 2^-7", Chain::Float),
        with("float 2^-8", Chain::Float, 8),
        with("float 2^-9", Chain::Float, 9),
        with("float 2^-11", Chain::Float, 11),
        col("int 13/2 2^-7", INT_13_2),
        with("int 13/2 2^-8", INT_13_2, 8),
        with("int 13/2 2^-9", INT_13_2, 9),
    ];
    let reference = 0;
    const IDLE: f64 = -30.0;
    preamble_text("pole", frames);
    println!(
        "# Noise at {IDLE} dBFS at the largest gain, AWGN, no DC offset; the frames of `dc`'s rows without\n\
         # an offset. Columns: the pole of the DC blocker, 2^-k, in the acquisition's path and the\n\
         # decoder's alike; 'float raw' is the blocker at 2^-7 before the acquisition only."
    );
    header(&columns, reference);
    println!("\n## 54 Mb/s, AWGN: the frames of `dc`");
    println!("{}", head_line("SNR", &columns));
    for (si, snr) in [25.0, 30.0, 40.0, 60.0].into_iter().enumerate() {
        let pt = Point {
            scene: 4000 + si as u64,
            rate: Rate::R54,
            snr_db: snr,
            ch: Channel::Awgn,
            idle_dbfs: IDLE,
            per_index_dbfs: none(),
            before_dbfs: none(),
            ppm_band: None,
        };
        let t = run(&pt, &columns, reference, frames);
        println!("{}", row_line(&format!("{snr} dB"), &t, reference));
    }
    // Near the sensitivity and through a fading channel, where `per` and `level` stand
    // above `rxper fixed`, whose decoder is handed the samples before the blocker.
    println!("\n## Near 10 % PER and through TGn D (frames of their own)");
    println!("{}", head_line("channel, rate, SNR", &columns));
    let d = Channel::Tgn(TgnModel::D);
    let rows: [(&str, Channel, Rate, f64); 10] = [
        ("AWGN 24 Mb/s 11.5 dB", Channel::Awgn, Rate::R24, 11.5),
        ("AWGN 36 Mb/s 14.5 dB", Channel::Awgn, Rate::R36, 14.5),
        ("AWGN 48 Mb/s 18.5 dB", Channel::Awgn, Rate::R48, 18.5),
        ("AWGN 54 Mb/s 20.0 dB", Channel::Awgn, Rate::R54, 20.0),
        ("TGn D  6 Mb/s  8.5 dB", d, Rate::R6, 8.5),
        ("TGn D 24 Mb/s 17.0 dB", d, Rate::R24, 17.0),
        ("TGn D 36 Mb/s 21.5 dB", d, Rate::R36, 21.5),
        ("TGn D 54 Mb/s 26.5 dB", d, Rate::R54, 26.5),
        ("TGn D 54 Mb/s 40.0 dB", d, Rate::R54, 40.0),
        ("TGn D 54 Mb/s 60.0 dB", d, Rate::R54, 60.0),
    ];
    for (k, (name, ch, rate, snr)) in rows.into_iter().enumerate() {
        let pt = Point {
            scene: 5000 + k as u64,
            rate,
            snr_db: snr,
            ch,
            idle_dbfs: IDLE,
            per_index_dbfs: none(),
            before_dbfs: none(),
            ppm_band: None,
        };
        let t = run(&pt, &columns, reference, frames);
        println!("{}", row_line(name, &t, reference));
    }
}

/// Is the blocker's cost the CFO's? The clock offset's size in bands, no DC offset.
fn cfo(frames: usize) {
    let columns = [
        Column {
            raw: true,
            ..col("float raw", Chain::Float)
        },
        col("float 2^-7", Chain::Float),
        Column {
            ltf: true,
            ..col("float ltf", Chain::Float)
        },
        col("int 13/2 2^-7", INT_13_2),
        Column {
            ltf: true,
            ..col("int 13/2 ltf", INT_13_2)
        },
    ];
    let reference = 0;
    const IDLE: f64 = -30.0;
    preamble_text("cfo", frames);
    println!(
        "# Noise at {IDLE} dBFS at the largest gain, no DC offset. The clock offset's size uniform in\n\
         # the band of the row (40 ppm: 232 kHz of CFO, 0.74 of the subcarrier spacing), either sign.\n\
         # The rows of a point are the same frames but for the clock offset.\n\
         #   2^-7 = the decoder handed the blocker's output (the chain as it is);\n\
         #   ltf  = the decoder handed the samples before the blocker less the offset estimated\n\
         #          from the two long training symbols (ltf_dc_offset, ltf_dc_offset_fixed)."
    );
    header(&columns, reference);
    let d = Channel::Tgn(TgnModel::D);
    let points: [(&str, Channel, f64); 4] = [
        ("AWGN 54 Mb/s 20.0 dB", Channel::Awgn, 20.0),
        ("AWGN 54 Mb/s 25.0 dB", Channel::Awgn, 25.0),
        ("TGn D 54 Mb/s 26.5 dB", d, 26.5),
        ("TGn D 54 Mb/s 40.0 dB", d, 40.0),
    ];
    for (pi, (name, ch, snr)) in points.into_iter().enumerate() {
        println!("\n## {name}");
        println!("{}", head_line("clock offset", &columns));
        for (lo, hi) in [
            (0.0, 2.0),
            (2.0, 10.0),
            (10.0, 20.0),
            (20.0, 30.0),
            (30.0, 40.0),
        ] {
            let pt = Point {
                scene: 6000 + pi as u64,
                rate: Rate::R54,
                snr_db: snr,
                ch,
                idle_dbfs: IDLE,
                per_index_dbfs: none(),
                before_dbfs: none(),
                ppm_band: Some((lo, hi)),
            };
            let t = run(&pt, &columns, reference, frames);
            println!(
                "{}",
                row_line(&format!("{lo:>2.0} to {hi:>2.0} ppm"), &t, reference)
            );
        }
    }
}

/// Candidates for what the decoder is handed (ADR 0008), under the DC offsets of `dc`.
fn cand(frames: usize) {
    let ltf = |name: &'static str, chain: Chain| Column {
        ltf: true,
        ..col(name, chain)
    };
    let columns = [
        Column {
            raw: true,
            ..col("float raw", Chain::Float)
        },
        col("float 2^-7", Chain::Float),
        ltf("float ltf", Chain::Float),
        col("int 13/2 2^-7", INT_13_2),
        ltf("int 12/2 ltf", INT_12_2),
        ltf("int 13/2 ltf", INT_13_2),
    ];
    let reference = 2;
    const IDLE: f64 = -30.0;
    preamble_text("cand", frames);
    println!(
        "# Noise at {IDLE} dBFS at the largest gain. DC per gain state as in `dc`; the gain control's\n\
         # detectors see the offset with the signal; the blocker (in every column's acquisition) is\n\
         # told nothing. The rows of one point are the same frames at every offset.\n\
         #   2^-7 = the decoder handed the blocker's output (the chain as it is);\n\
         #   ltf  = the decoder handed the samples before the blocker less the offset estimated\n\
         #          from the two long training symbols (ltf_dc_offset; in the int columns\n\
         #          ltf_dc_offset_fixed, whole ADC LSB, and remove_dc_fixed)."
    );
    header(&columns, reference);
    let d = Channel::Tgn(TgnModel::D);
    let all: [(&str, f64, f64); 7] = [
        ("no DC", none(), none()),
        ("per index −40 dBFS", -40.0, none()),
        ("per index −30 dBFS", -30.0, none()),
        ("per index −20 dBFS", -20.0, none()),
        ("before the gain −30 dBFS", none(), -30.0),
        ("before the gain −20 dBFS", none(), -20.0),
        ("before the gain −10 dBFS", none(), -10.0),
    ];
    // (title, channel, rate, SNR, scene): the scenes of `dc` and `pole` where they have one.
    let points: [(&str, Channel, Rate, f64, u64); 11] = [
        ("AWGN 54 Mb/s 25 dB", Channel::Awgn, Rate::R54, 25.0, 4000),
        ("AWGN 54 Mb/s 30 dB", Channel::Awgn, Rate::R54, 30.0, 4001),
        ("AWGN 54 Mb/s 40 dB", Channel::Awgn, Rate::R54, 40.0, 4002),
        ("AWGN 54 Mb/s 60 dB", Channel::Awgn, Rate::R54, 60.0, 4003),
        ("AWGN 6 Mb/s 10 dB", Channel::Awgn, Rate::R6, 10.0, 4010),
        ("AWGN 54 Mb/s 20 dB", Channel::Awgn, Rate::R54, 20.0, 5003),
        ("AWGN 6 Mb/s 3.5 dB", Channel::Awgn, Rate::R6, 3.5, 7000),
        ("TGn D 6 Mb/s 8.5 dB", d, Rate::R6, 8.5, 5004),
        ("TGn D 54 Mb/s 26.5 dB", d, Rate::R54, 26.5, 5007),
        ("TGn D 54 Mb/s 40 dB", d, Rate::R54, 40.0, 5008),
        ("TGn D 54 Mb/s 60 dB", d, Rate::R54, 60.0, 5009),
    ];
    for (name, ch, rate, snr, scene) in points {
        println!("\n## {name}");
        println!("{}", head_line("DC offset", &columns));
        for (case, per_index, before) in all {
            let pt = Point {
                scene,
                rate,
                snr_db: snr,
                ch,
                idle_dbfs: IDLE,
                per_index_dbfs: per_index,
                before_dbfs: before,
                ppm_band: None,
            };
            let t = run(&pt, &columns, reference, frames);
            println!("{}", row_line(case, &t, reference));
        }
    }
    // The lower edge of the level (`level`'s frames): the estimate is whole ADC LSB.
    let low: [(&str, Channel, Rate, f64, u64); 2] = [
        ("AWGN 6 Mb/s 3.5 dB", Channel::Awgn, Rate::R6, 3.5, 3000),
        ("AWGN 54 Mb/s 20 dB", Channel::Awgn, Rate::R54, 20.0, 3002),
    ];
    for (name, ch, rate, snr, scene) in low {
        println!("\n## {name}: the noise at the largest gain lower");
        println!(
            "{}",
            head_line("noise at the largest gain, DC offset", &columns)
        );
        for idle in [-42.0, -48.0, -54.0] {
            for (case, per_index) in [("no DC", none()), ("per index −50 dBFS", -50.0)] {
                let pt = Point {
                    scene,
                    rate,
                    snr_db: snr,
                    ch,
                    idle_dbfs: idle,
                    per_index_dbfs: per_index,
                    before_dbfs: none(),
                    ppm_band: None,
                };
                let t = run(&pt, &columns, reference, frames);
                println!(
                    "{}",
                    row_line(&format!("{idle:.0} dBFS, {case}"), &t, reference)
                );
            }
        }
    }
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let cmd = args.next().unwrap_or_default();
    let n: Option<usize> = args.next().and_then(|v| v.parse().ok());
    match cmd.as_str() {
        "per" => per(n.unwrap_or(2000)),
        "level" => level(n.unwrap_or(2000)),
        "dc" => dc(n.unwrap_or(2000)),
        "pole" => pole(n.unwrap_or(2000)),
        "cfo" => cfo(n.unwrap_or(2000)),
        "cand" => cand(n.unwrap_or(2000)),
        _ => {
            eprintln!("usage: rxagc per|level|dc|pole|cfo|cand [frames]");
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
