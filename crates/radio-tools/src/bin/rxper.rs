//! `rxper` — packet error rate of the whole float receiver (R0-08a front + R0-09 OFDM receiver)
//! through the channel simulator. Measurements for `docs/gates/rx_ofdm.md`; multi-threaded.
//!
//! Usage: `cargo run -q --release -p radio-tools --bin rxper -- <command> [frames per point]`
//! - `awgn`: PER against SNR for the eight rates, 1000-octet PSDUs (the frame of the standard's
//!   sensitivity requirement, IEEE 802.11-2020 §17.3.10.2), with the decoder's 4-bit soft values
//!   and with fine ones.
//! - `tgn`: the same through static TGn B and D channels, a new channel per frame.
//! - `sfo`: frames lost against the sample-clock offset and the frame's duration.
//! - `timing`: what the timing tracker costs and saves near 10 % PER.
//! - `soft`, `soft2`: PER against the soft values' width and clip level, AWGN and TGn B and D
//!   (a wide survey, then the candidates around a step of 0.1).
//! - `fixed`, `fixedw`: G2 of the integer data path (`rx_ofdm_fixed`): the loss against the
//!   float receiver on the same 12-bit samples over the grids of `softg2`, and the frames lost
//!   near 10 % PER with one width changed at a time.
//! - `softg2`: the G2 sweep of the soft-value format (R0-09): the candidates on the same
//!   frames, fine grids around the 10 % crossings, AWGN and TGn B, D and E.
//!
//! Every frame has a clock offset between the two ends (uniform in ±40 ppm unless stated) that
//! sets both its CFO and its sample-clock offset, at a 5.8 GHz carrier.
//!
//! SNR is over the whole sampled band, as everywhere in the simulator.

use std::f64::consts::PI;
use std::process::ExitCode;

use radio_phy::channel::{
    awgn, fractional_delay, frequency_offset, mean_power, resample, tgn_taps, Doppler, Multipath,
    Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, correct, RxConfig};
use radio_phy::rx_front_fixed::FixedRxConfig;
use radio_phy::rx_ofdm::{decode_ppdu_from, RxOfdmConfig, Tracking};
use radio_phy::rx_ofdm_fixed::{decode_ppdu_fixed, to_fixed, FixedOfdmConfig};

type C = (f64, f64);

const FS: f64 = 20e6;

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

/// Carrier of the scenarios: the two ends' oscillators set both the CFO and the sample-clock
/// offset (IEEE 802.11-2020 §17.3.9.5–6), ±40 ppm between them at most.
const CARRIER_HZ: f64 = 5.8e9;

/// A receiver under test: its settings and whether the timing loop starts from the clock
/// offset the front's CFO implies.
#[derive(Clone, Copy)]
struct Receiver {
    name: &'static str,
    cfg: RxOfdmConfig,
    hint: bool,
    /// The integer data path with these widths instead of the float one.
    fixed: Option<FixedOfdmConfig>,
}

fn receiver(name: &'static str, hint: bool, cfg: RxOfdmConfig) -> Receiver {
    Receiver {
        name,
        cfg,
        hint,
        fixed: None,
    }
}

/// Level of the frame in the 12-bit samples both kinds of receiver are handed when any of
/// them is the integer one, dBFS (0 dBFS: a mean |v|² of full scale squared).
const ADC_LEVEL_DBFS: f64 = -12.0;
const ADC_FULL: f64 = 2048.0;

/// Frames lost by each receiver out of `frames` (rounded up to the thread count).
/// One operating point: `ppm` is the clock offset between the two ends, `None` draws it
/// uniformly in ±40 per frame.
#[derive(Clone, Copy)]
struct Scene {
    rate: Rate,
    octets: usize,
    snr_db: f64,
    ch: Channel,
    ppm: Option<f64>,
}

fn lost(scene: Scene, receivers: &[Receiver], frames: usize, seed: u64) -> (Vec<usize>, usize) {
    let Scene {
        rate,
        octets,
        snr_db,
        ch,
        ppm,
    } = scene;
    let per_thread = frames.div_ceil(threads());
    let rx = RxConfig::new(FS);
    let r = par(|t| {
        let mut rng = Rng::new(seed * 4096 + t as u64);
        let mut lost = vec![0usize; receivers.len()];
        for _ in 0..per_thread {
            let psdu: Vec<u8> = (0..octets).map(|_| (rng.next_u64() & 0xff) as u8).collect();
            let f = frame(&psdu, rate);
            let mut x = vec![(0.0, 0.0); 300];
            x.extend_from_slice(&f);
            x.extend(vec![(0.0, 0.0); 200]);
            let x = match ch {
                Channel::Awgn => fractional_delay(&x, rng.uniform()),
                Channel::Tgn(m) => {
                    Multipath::new(tgn_taps(m, false), Doppler::Static, 1, FS, &mut rng).apply(&x)
                }
            };
            // The transmitter's oscillator fast by p ppm against ours: its symbols come p·1e-6
            // faster and its carrier is p·1e-6·carrier above our local oscillator.
            let p = ppm.unwrap_or_else(|| (rng.uniform() * 2.0 - 1.0) * 40.0);
            let x = resample(&x, p);
            let x = frequency_offset(&x, p * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
            let x = awgn(&x, snr_db, mean_power(&f), &mut rng);
            let Some(a) = acquire(&x, 0, &rx) else {
                lost.iter_mut().for_each(|l| *l += 1);
                continue;
            };
            let mut y = correct(&x, a.cfo_hz, FS);
            // With an integer receiver in the set every receiver sees the corrected stream
            // as 12-bit samples, the frame's span at the level a gain control would leave it.
            let z = receivers.iter().any(|r| r.fixed.is_some()).then(|| {
                let span = &y[300.min(y.len())..(300 + f.len()).min(y.len())];
                let k = (10f64.powf(ADC_LEVEL_DBFS / 10.0) / mean_power(span)).sqrt();
                to_fixed(&y, ADC_FULL * k, 12)
            });
            if let Some(z) = &z {
                y = z.iter().map(|v| (v.0 as f64, v.1 as f64)).collect();
            }
            for (k, r) in receivers.iter().enumerate() {
                let start = if r.hint {
                    Tracking::from_clock_offset(a.cfo_hz / CARRIER_HZ * 1e6)
                } else {
                    Tracking::default()
                };
                let ok = match (&r.fixed, &z) {
                    (Some(fx), Some(z)) => decode_ppdu_fixed(z, a.fft_start, &r.cfg, fx, start)
                        .is_ok_and(|d| d.psdu == psdu),
                    _ => decode_ppdu_from(&y, a.fft_start, &r.cfg, start)
                        .is_ok_and(|d| d.psdu == psdu),
                };
                lost[k] += usize::from(!ok);
            }
        }
        lost
    });
    let total = per_thread * threads();
    let mut sum = vec![0usize; receivers.len()];
    for v in &r {
        for (s, l) in sum.iter_mut().zip(v) {
            *s += l;
        }
    }
    (sum, total)
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

fn sweep(
    name: &str,
    ch: Channel,
    receivers: &[Receiver],
    start_db: [f64; 8],
    step: f64,
    points: usize,
    frames: usize,
) {
    let names: Vec<&str> = receivers.iter().map(|r| r.name).collect();
    println!(
        "\n## {name}: PER of 1000-octet frames, {frames} frames per point, clock offset uniform in ±40 ppm\n# receivers: {}",
        names.join(" / ")
    );
    let mut summary = Vec::new();
    for (ri, rate) in Rate::ALL.into_iter().enumerate() {
        let mut curve = vec![Vec::new(); receivers.len()];
        let mut line = format!("{:>2} Mb/s:", rate.mbps());
        for k in 0..points {
            let snr = start_db[ri] + step * k as f64;
            let (l, total) = lost(
                Scene {
                    rate,
                    octets: 1000,
                    snr_db: snr,
                    ch,
                    ppm: None,
                },
                receivers,
                frames,
                (ri * 100 + k) as u64 + 1,
            );
            let mut cell = Vec::new();
            for (v, c) in curve.iter_mut().enumerate() {
                c.push((snr, l[v] as f64 / total as f64));
                cell.push(format!("{:.3}", l[v] as f64 / total as f64));
            }
            line += &format!("  {snr:>4.1} dB {}", cell.join("/"));
        }
        println!("{line}");
        summary.push((rate, curve.iter().map(|c| crossing(c)).collect::<Vec<_>>()));
    }
    println!("# SNR at 10 % PER (dB), and the difference to the first receiver:");
    for (rate, cross) in summary {
        let mut line = format!("#   {:>2} Mb/s:", rate.mbps());
        for (k, c) in cross.iter().enumerate() {
            line += &match (c, cross[0]) {
                (Some(c), Some(first)) if k > 0 => {
                    format!("  {} {c:.1} ({:+.2})", names[k], c - first)
                }
                (Some(c), _) => format!("  {} {c:.1}", names[k]),
                _ => format!("  {} —", names[k]),
            };
        }
        println!("{line}");
    }
}

fn base() -> RxOfdmConfig {
    RxOfdmConfig::default()
}

fn no_loop() -> RxOfdmConfig {
    RxOfdmConfig {
        timing_tracking: false,
        ..base()
    }
}

fn fine() -> RxOfdmConfig {
    RxOfdmConfig {
        soft_max: 255,
        clip: 6.0,
        ..base()
    }
}

/// The working soft format (the founder, 2026-11-19): six bits, clip 2.
fn soft6() -> RxOfdmConfig {
    RxOfdmConfig {
        soft_max: 31,
        clip: 2.0,
        ..base()
    }
}

/// The soft-format sweep: a fine reference and every candidate on the same frames, on grids
/// of 0.5 dB (AWGN) and 1 dB (fading) around the 10 % crossings that `soft2` found.
fn soft_formats(tag: &str, formats: &[(&'static str, i32, f64)], frames: usize) {
    let mut rx = vec![receiver("fine", true, fine())];
    for &(name, soft_max, clip) in formats {
        rx.push(receiver(
            name,
            true,
            RxOfdmConfig {
                soft_max,
                clip,
                ..base()
            },
        ));
    }
    sweep(
        &format!("AWGN, soft-value formats ({tag})"),
        Channel::Awgn,
        &rx,
        [2.5, 3.5, 5.5, 7.5, 10.5, 13.5, 17.5, 18.5],
        0.5,
        7,
        frames,
    );
    for (name, m, start, points) in [
        (
            format!("TGn B, static, soft-value formats ({tag})"),
            TgnModel::B,
            [7.5, 12.0, 11.0, 15.5, 16.5, 20.5, 23.5, 26.0],
            7,
        ),
        (
            format!("TGn D, static, soft-value formats ({tag})"),
            TgnModel::D,
            [6.0, 10.0, 10.0, 13.5, 15.0, 19.0, 22.0, 24.5],
            7,
        ),
        (
            format!("TGn E, static, soft-value formats ({tag})"),
            TgnModel::E,
            [5.0, 9.0, 9.0, 12.5, 14.0, 18.0, 21.0, 23.5],
            9,
        ),
    ] {
        sweep(&name, Channel::Tgn(m), &rx, start, 1.0, points, frames);
    }
}

/// The width sweep of the integer data path: frames lost of the same frames near 10 % PER,
/// the float receiver first, then the default widths, then one width changed at a time.
fn widths(frames: usize) {
    let d = FixedOfdmConfig::default();
    let front = |f: FixedRxConfig| FixedOfdmConfig { front: f, ..d };
    let variants: Vec<(&'static str, FixedOfdmConfig)> = vec![
        (
            "wide",
            FixedOfdmConfig {
                in_shift: 8,
                fft_bits: 28,
                twiddle_bits: 24,
                prod_bits: 40,
                kappa_bits: 20,
                scale_frac: 8,
                front: FixedRxConfig {
                    cordic_iters: 24,
                    cordic_bits: 30,
                    angle_bits: 24,
                    nco_phase_bits: 30,
                    nco_lut_bits: 16,
                    nco_amp_bits: 20,
                    ..d.front
                },
                ..d
            },
        ),
        (
            "front wide",
            front(FixedRxConfig {
                cordic_iters: 24,
                cordic_bits: 30,
                angle_bits: 24,
                nco_phase_bits: 30,
                nco_lut_bits: 16,
                nco_amp_bits: 20,
                ..d.front
            }),
        ),
        (
            "data wide",
            FixedOfdmConfig {
                in_shift: 8,
                fft_bits: 28,
                twiddle_bits: 24,
                prod_bits: 40,
                kappa_bits: 20,
                scale_frac: 8,
                ..d
            },
        ),
        (
            "L11 Q14",
            front(FixedRxConfig {
                nco_lut_bits: 11,
                nco_amp_bits: 14,
                ..d.front
            }),
        ),
        (
            "cordic wide",
            front(FixedRxConfig {
                cordic_iters: 24,
                cordic_bits: 30,
                angle_bits: 22,
                ..d.front
            }),
        ),
        (
            "C14",
            FixedOfdmConfig {
                kappa_bits: 14,
                ..d
            },
        ),
        ("F8", FixedOfdmConfig { scale_frac: 8, ..d }),
        ("default", d),
        (
            "W14 s0",
            FixedOfdmConfig {
                fft_bits: 14,
                in_shift: 0,
                ..d
            },
        ),
        ("W14 s2", FixedOfdmConfig { fft_bits: 14, ..d }),
        ("W15 s2", FixedOfdmConfig { fft_bits: 15, ..d }),
        ("W16 s0", FixedOfdmConfig { in_shift: 0, ..d }),
        ("W16 s4", FixedOfdmConfig { in_shift: 4, ..d }),
        (
            "W18 s4",
            FixedOfdmConfig {
                fft_bits: 18,
                in_shift: 4,
                ..d
            },
        ),
        (
            "T8",
            FixedOfdmConfig {
                twiddle_bits: 8,
                ..d
            },
        ),
        (
            "T10",
            FixedOfdmConfig {
                twiddle_bits: 10,
                ..d
            },
        ),
        (
            "T14",
            FixedOfdmConfig {
                twiddle_bits: 14,
                ..d
            },
        ),
        ("U10", FixedOfdmConfig { prod_bits: 10, ..d }),
        ("U12", FixedOfdmConfig { prod_bits: 12, ..d }),
        ("U14", FixedOfdmConfig { prod_bits: 14, ..d }),
        ("U20", FixedOfdmConfig { prod_bits: 20, ..d }),
        ("C6", FixedOfdmConfig { kappa_bits: 6, ..d }),
        ("C8", FixedOfdmConfig { kappa_bits: 8, ..d }),
        ("F0", FixedOfdmConfig { scale_frac: 0, ..d }),
        (
            "L7",
            front(FixedRxConfig {
                nco_lut_bits: 7,
                ..d.front
            }),
        ),
        (
            "L8",
            front(FixedRxConfig {
                nco_lut_bits: 8,
                ..d.front
            }),
        ),
        (
            "L11",
            front(FixedRxConfig {
                nco_lut_bits: 11,
                ..d.front
            }),
        ),
        (
            "Q8",
            front(FixedRxConfig {
                nco_amp_bits: 8,
                ..d.front
            }),
        ),
        (
            "Q9",
            front(FixedRxConfig {
                nco_amp_bits: 9,
                ..d.front
            }),
        ),
        (
            "N10",
            front(FixedRxConfig {
                cordic_iters: 10,
                ..d.front
            }),
        ),
        (
            "N12",
            front(FixedRxConfig {
                cordic_iters: 12,
                ..d.front
            }),
        ),
    ];
    let mut rx = vec![receiver("float", true, soft6())];
    for (name, fx) in &variants {
        if !fx.is_valid() {
            println!("# {name}: not a usable set of widths, left out");
            continue;
        }
        rx.push(Receiver {
            fixed: Some(*fx),
            ..receiver(name, true, soft6())
        });
    }
    println!(
        "\n## widths of the integer data path: frames lost of {frames} (rounded up), 1000 octets, the same frames for every column,\n\
         # 12-bit samples at {ADC_LEVEL_DBFS} dBFS, six-bit soft values with a clip of 2, clock offset uniform in ±40 ppm.\n\
         # default = W16 s2 T12 U16 C10 F4, rotator L9 Q11, CORDIC N16; each other column changes what its name says;\n\
         # wide = every width far beyond need (W28 s8 T24 U40 C20 F8, L16 Q20, CORDIC N24 A24): what is left is not a width's.\n\
         # columns: {}",
        rx.iter().map(|r| r.name).collect::<Vec<_>>().join(" | ")
    );
    for (label, rate, snr, ch) in [
        ("AWGN  6 Mb/s  3.5 dB", Rate::R6, 3.5, Channel::Awgn),
        ("AWGN 54 Mb/s 20.0 dB", Rate::R54, 20.0, Channel::Awgn),
        ("AWGN 54 Mb/s 35.0 dB", Rate::R54, 35.0, Channel::Awgn),
        (
            "TGn D 18 Mb/s 15.5 dB",
            Rate::R18,
            15.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn D 54 Mb/s 26.5 dB",
            Rate::R54,
            26.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn E 54 Mb/s 26.5 dB",
            Rate::R54,
            26.5,
            Channel::Tgn(TgnModel::E),
        ),
        (
            "TGn E 54 Mb/s 45.0 dB",
            Rate::R54,
            45.0,
            Channel::Tgn(TgnModel::E),
        ),
    ] {
        let (l, total) = lost(
            Scene {
                rate,
                octets: 1000,
                snr_db: snr,
                ch,
                ppm: None,
            },
            &rx,
            frames,
            9000 + rate.mbps() as u64 + snr as u64,
        );
        println!(
            "{label:<22} | {}  of {total}",
            l.iter()
                .map(|x| format!("{x:>4}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
}

/// The receivers of the PER curves.
fn standard() -> Vec<Receiver> {
    vec![
        receiver("4-bit", true, base()),
        receiver("fine soft", true, fine()),
    ]
}

/// What the timing tracker costs and what it saves, near 10 % PER. Two runs per point, each
/// with every receiver on the same frames: with the clock offset drawn per frame, and at 0 ppm
/// against a receiver without the tracker.
fn timing(frames: usize) {
    let rx = [
        receiver("from the CFO", true, base()),
        receiver("from zero", false, base()),
        receiver("no tracker", false, no_loop()),
    ];
    println!("\n## timing tracker: frames lost of {frames} (rounded up), 1000 octets");
    println!("# columns: clock offset uniform in ±40 ppm: started from the CFO / from zero / no tracker || 0 ppm, the same three");
    for (label, rate, snr, ch) in [
        ("AWGN  6 Mb/s  3.5 dB", Rate::R6, 3.5, Channel::Awgn),
        ("AWGN 24 Mb/s 11.5 dB", Rate::R24, 11.5, Channel::Awgn),
        ("AWGN 54 Mb/s 20.0 dB", Rate::R54, 20.0, Channel::Awgn),
        ("AWGN 54 Mb/s 35.0 dB", Rate::R54, 35.0, Channel::Awgn),
        (
            "TGn D  6 Mb/s  8.5 dB",
            Rate::R6,
            8.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn D 12 Mb/s 12.5 dB",
            Rate::R12,
            12.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn D 48 Mb/s 24.5 dB",
            Rate::R48,
            24.5,
            Channel::Tgn(TgnModel::D),
        ),
        (
            "TGn D 54 Mb/s 28.5 dB",
            Rate::R54,
            28.5,
            Channel::Tgn(TgnModel::D),
        ),
    ] {
        let seed = 7000 + rate.mbps() as u64 + snr as u64;
        let scene = |ppm| Scene {
            rate,
            octets: 1000,
            snr_db: snr,
            ch,
            ppm,
        };
        let (l, total) = lost(scene(None), &rx, frames, seed);
        let (y, _) = lost(scene(Some(0.0)), &rx, frames, seed);
        let f = |v: &[usize]| {
            v.iter()
                .map(|x| format!("{x:>4}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        println!("{label:<22} | {} || {}  of {total}", f(&l), f(&y));
    }
}

fn sfo(frames: usize) {
    let rx = [
        receiver("from the CFO", true, base()),
        receiver("from zero", false, base()),
        receiver("no loop", false, no_loop()),
    ];
    println!("\n## sample-clock offset: frames lost of {frames} (rounded up), high SNR: loop started from the CFO / from zero / no loop");
    println!(
        "# rate, octets (duration)    ppm:   0          10          20          40         -40"
    );
    for (rate, snr) in [(Rate::R6, 20.0), (Rate::R24, 28.0), (Rate::R54, 37.0)] {
        for octets in [100usize, 1000, 4095] {
            let symbols = radio_phy::ppdu::data_symbols(rate, octets);
            let ms = (320 + 80 * (symbols + 1)) as f64 / 20e3;
            let mut line = format!("{:>2} Mb/s {octets:>5} ({ms:5.2} ms)     ", rate.mbps());
            for (k, ppm) in [0.0, 10.0, 20.0, 40.0, -40.0].into_iter().enumerate() {
                let (l, _) = lost(
                    Scene {
                        rate,
                        octets,
                        snr_db: snr,
                        ch: Channel::Awgn,
                        ppm: Some(ppm),
                    },
                    &rx,
                    frames,
                    9000 + k as u64,
                );
                line += &format!(" {:>3}/{:>2}/{:<3}", l[0], l[1], l[2]);
            }
            println!("{line}");
        }
    }
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let cmd = args.next().unwrap_or_default();
    let n: Option<usize> = args.next().and_then(|v| v.parse().ok());
    println!("# rxper {cmd}: float front (acquire, correct) + rx_ofdm::decode_ppdu_from, carrier 5.8 GHz");
    match cmd.as_str() {
        "awgn" => sweep(
            "AWGN, random fractional delay",
            Channel::Awgn,
            &standard(),
            [1.0, 4.0, 4.0, 7.0, 10.0, 13.5, 18.0, 19.5],
            1.0,
            7,
            n.unwrap_or(200),
        ),
        "tgn" => {
            for (name, m) in [
                ("TGn B, static", TgnModel::B),
                ("TGn D, static", TgnModel::D),
            ] {
                sweep(
                    name,
                    Channel::Tgn(m),
                    &standard(),
                    [4.0, 8.0, 7.0, 11.0, 13.0, 18.0, 22.0, 24.0],
                    3.0,
                    7,
                    n.unwrap_or(200),
                );
            }
        }
        "soft" | "soft2" => {
            // Soft-value width and clip: what the Viterbi decoder is handed. Fine soft first.
            // `soft` is the wide survey; `soft2` the candidates around a step of 0.1.
            let wide: &[(&'static str, i32, f64)] = &[
                ("4b/1.5", 7, 1.5),
                ("4b/3", 7, 3.0),
                ("4b/6", 7, 6.0),
                ("5b/1.5", 15, 1.5),
                ("5b/3", 15, 3.0),
                ("5b/6", 15, 6.0),
                ("6b/3", 31, 3.0),
                ("6b/6", 31, 6.0),
            ];
            let near: &[(&'static str, i32, f64)] = &[
                ("4b/0.75", 7, 0.75),
                ("4b/1", 7, 1.0),
                ("5b/1", 15, 1.0),
                ("5b/1.5", 15, 1.5),
                ("5b/2", 15, 2.0),
                ("6b/1.5", 31, 1.5),
                ("6b/2", 31, 2.0),
                ("6b/3", 31, 3.0),
            ];
            let mut rx = vec![receiver("fine", true, fine())];
            for &(name, soft_max, clip) in if cmd == "soft" { wide } else { near } {
                rx.push(receiver(
                    name,
                    true,
                    RxOfdmConfig {
                        soft_max,
                        clip,
                        ..base()
                    },
                ));
            }
            sweep(
                "AWGN, soft-value formats",
                Channel::Awgn,
                &rx,
                [1.0, 4.0, 4.0, 7.0, 10.0, 13.5, 18.0, 19.5],
                1.0,
                7,
                n.unwrap_or(200),
            );
            for (name, m) in [
                ("TGn B, static, soft-value formats", TgnModel::B),
                ("TGn D, static, soft-value formats", TgnModel::D),
            ] {
                sweep(
                    name,
                    Channel::Tgn(m),
                    &rx,
                    [4.0, 8.0, 7.0, 11.0, 13.0, 18.0, 22.0, 24.0],
                    3.0,
                    7,
                    n.unwrap_or(200),
                );
            }
        }
        "softg2" => {
            // The G2 sweep of the soft-value format: every candidate on the same frames, on
            // grids of 0.5 dB (AWGN) and 1 dB (fading) around the 10 % crossings that
            // `soft2` found.
            let formats: &[(&'static str, i32, f64)] = &[
                ("4b/1", 7, 1.0),
                ("4b/1.5", 7, 1.5),
                ("5b/1", 15, 1.0),
                ("5b/1.5", 15, 1.5),
                ("5b/2", 15, 2.0),
                ("6b/1.5", 31, 1.5),
                ("6b/2", 31, 2.0),
                ("6b/3", 31, 3.0),
            ];
            soft_formats("G2", formats, n.unwrap_or(2000));
        }
        "softg3" => {
            // The same grids with seven- and eight-bit candidates, after `softg2` showed that
            // no six-bit format holds 0.2 dB in both AWGN (wants a larger clip) and TGn B
            // (wants a finer step). 6b/2 is repeated as the paired reference.
            let formats: &[(&'static str, i32, f64)] = &[
                ("6b/2", 31, 2.0),
                ("7b/2", 63, 2.0),
                ("7b/3", 63, 3.0),
                ("7b/4", 63, 4.0),
                ("8b/3", 127, 3.0),
                ("8b/4", 127, 4.0),
            ];
            soft_formats("G2, wider", formats, n.unwrap_or(2000));
        }
        "softg4" => {
            // The quantiser without a zero level against the rounding one, word for word:
            // "6m/2" is six bits (32 magnitudes and a sign), clip 2, mid-rise.
            let mut rx = vec![receiver("fine", true, fine())];
            for (name, soft_max, clip, mid_rise) in [
                ("6b/2", 31, 2.0, false),
                ("5m/2", 15, 2.0, true),
                ("6m/2", 31, 2.0, true),
                ("6m/3", 31, 3.0, true),
                ("7m/3", 63, 3.0, true),
            ] {
                rx.push(receiver(
                    name,
                    true,
                    RxOfdmConfig {
                        soft_max,
                        clip,
                        mid_rise,
                        ..base()
                    },
                ));
            }
            let frames = n.unwrap_or(2000);
            sweep(
                "AWGN, mid-rise soft values (G2)",
                Channel::Awgn,
                &rx,
                [2.5, 3.5, 5.5, 7.5, 10.5, 13.5, 17.5, 18.5],
                0.5,
                7,
                frames,
            );
            sweep(
                "TGn B, static, mid-rise soft values (G2)",
                Channel::Tgn(TgnModel::B),
                &rx,
                [7.5, 12.0, 11.0, 15.5, 16.5, 20.5, 23.5, 26.0],
                1.0,
                7,
                frames,
            );
            sweep(
                "TGn E, static, mid-rise soft values (G2)",
                Channel::Tgn(TgnModel::E),
                &rx,
                [5.0, 9.0, 9.0, 12.5, 14.0, 18.0, 21.0, 23.5],
                1.0,
                9,
                frames,
            );
        }
        "fixed" => {
            // G2 of the integer data path: float and integer receiver on the same 12-bit
            // samples, six-bit soft values with a clip of 2, the grids of `softg2`.
            let rx = [
                receiver("float", true, soft6()),
                Receiver {
                    fixed: Some(FixedOfdmConfig::default()),
                    ..receiver("integer", true, soft6())
                },
                // The one width `fixedw` shows a cost for: the rotator's table.
                Receiver {
                    fixed: Some(FixedOfdmConfig {
                        front: FixedRxConfig {
                            nco_lut_bits: 11,
                            nco_amp_bits: 14,
                            ..FixedRxConfig::default()
                        },
                        ..FixedOfdmConfig::default()
                    }),
                    ..receiver("integer L11 Q14", true, soft6())
                },
            ];
            let frames = n.unwrap_or(2000);
            sweep(
                "AWGN, float against integer data path (G2)",
                Channel::Awgn,
                &rx,
                [2.5, 3.5, 5.5, 7.5, 10.5, 13.5, 17.5, 18.5],
                0.5,
                7,
                frames,
            );
            for (name, m, start, points) in [
                (
                    "TGn B, static, float against integer data path (G2)",
                    TgnModel::B,
                    [7.5, 12.0, 11.0, 15.5, 16.5, 20.5, 23.5, 26.0],
                    7,
                ),
                (
                    "TGn D, static, float against integer data path (G2)",
                    TgnModel::D,
                    [6.0, 10.0, 10.0, 13.5, 15.0, 19.0, 22.0, 24.5],
                    7,
                ),
                (
                    "TGn E, static, float against integer data path (G2)",
                    TgnModel::E,
                    [5.0, 9.0, 9.0, 12.5, 14.0, 18.0, 21.0, 23.5],
                    9,
                ),
            ] {
                sweep(name, Channel::Tgn(m), &rx, start, 1.0, points, frames);
            }
        }
        "fixedw" => widths(n.unwrap_or(2000)),
        "sfo" => sfo(n.unwrap_or(40)),
        "timing" => timing(n.unwrap_or(300)),
        _ => {
            eprintln!(
                "usage: rxper awgn|tgn|sfo|timing|soft|soft2|softg2|softg3|softg4|fixed|fixedw [frames]"
            );
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
