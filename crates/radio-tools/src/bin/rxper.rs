//! `rxper` — packet error rate of the whole float receiver (R0-08a front + R0-09 OFDM receiver)
//! through the channel simulator. Measurements for `docs/gates/rx_ofdm.md`; multi-threaded.
//!
//! Usage: `cargo run -q --release -p radio-tools --bin rxper -- <command> [frames per point]`
//! - `awgn`: PER against SNR for the eight rates, 1000-octet PSDUs (the frame of the standard's
//!   sensitivity requirement, IEEE 802.11-2020 §17.3.10.2), with the decoder's 4-bit soft values
//!   and with fine ones.
//! - `tgn`: the same through static TGn B and D channels, a new channel per frame.
//! - `sfo`: frames lost against the sample-clock offset and the frame's duration.
//! - `timing`: candidates for the timing loop's gains near 10 % PER.
//! - `soft`: PER against the soft values' width and clip level, AWGN and TGn B and D.
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
use radio_phy::rx_ofdm::{decode_ppdu_from, RxOfdmConfig, Tracking};

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
}

fn receiver(name: &'static str, hint: bool, cfg: RxOfdmConfig) -> Receiver {
    Receiver { name, cfg, hint }
}

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
            // A receiver clock fast by p ppm: its samples run ahead by p·1e-6 per sample and
            // its local oscillator is high, so the signal appears at −p·1e-6·carrier.
            let p = ppm.unwrap_or_else(|| (rng.uniform() * 2.0 - 1.0) * 40.0);
            let x = resample(&x, p);
            let x = frequency_offset(&x, -p * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
            let x = awgn(&x, snr_db, mean_power(&f), &mut rng);
            let Some(a) = acquire(&x, 0, &rx) else {
                lost.iter_mut().for_each(|l| *l += 1);
                continue;
            };
            let y = correct(&x, a.cfo_hz, FS);
            for (k, r) in receivers.iter().enumerate() {
                let start = if r.hint {
                    Tracking::from_clock_offset(-a.cfo_hz / CARRIER_HZ * 1e6)
                } else {
                    Tracking::default()
                };
                let ok =
                    decode_ppdu_from(&y, a.fft_start, &r.cfg, start).is_ok_and(|d| d.psdu == psdu);
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

fn gains(g: f64, r: f64) -> RxOfdmConfig {
    RxOfdmConfig {
        timing_gain: g,
        timing_rate_gain: r,
        ..base()
    }
}

/// The receivers of the PER curves.
fn standard() -> Vec<Receiver> {
    vec![
        receiver("4-bit", true, base()),
        receiver("fine soft", true, fine()),
    ]
}

/// Timing-loop candidates at a few operating points near 10 % PER, against a receiver that
/// knows the clock offset exactly is not available: the yardstick is zero offset without a loop.
fn timing(frames: usize) {
    let candidates = [
        ("1/8, 1/250", 0.125, 0.004),
        ("1/16, 1/1000", 0.0625, 0.001),
        ("1/32, 1/4000", 0.03125, 0.00025),
        ("1/64, 1/16000", 0.015625, 0.0000625),
    ];
    let mut rx = Vec::new();
    for (name, g, r) in candidates {
        rx.push(receiver(name, true, gains(g, r)));
    }
    let n = rx.len();
    for (name, g, r) in candidates {
        rx.push(receiver(name, false, gains(g, r)));
    }
    println!("\n## timing loop: frames lost of {frames} (rounded up), 1000 octets, clock offset uniform in ±40 ppm");
    println!("# columns: gains (proportional, integral) started from the CFO's clock offset | the same started from zero | at 0 ppm: no loop, the slowest loop, no pilot tracking at all");
    let names: Vec<&str> = rx.iter().take(n).map(|r| r.name).collect();
    println!("# gains: {}", names.join(" ; "));
    for (label, rate, snr, ch) in [
        ("AWGN  6 Mb/s  3.5 dB", Rate::R6, 3.5, Channel::Awgn),
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
            seed,
        );
        let slow = gains(0.015625, 0.0000625);
        let (y, _) = lost(
            Scene {
                rate,
                octets: 1000,
                snr_db: snr,
                ch,
                ppm: Some(0.0),
            },
            &[
                receiver("no loop", false, no_loop()),
                receiver("slowest loop", false, slow),
                receiver(
                    "no pilots at all",
                    false,
                    RxOfdmConfig {
                        pilot_tracking: false,
                        ..no_loop()
                    },
                ),
            ],
            frames,
            seed,
        );
        let f = |v: &[usize]| {
            v.iter()
                .map(|x| format!("{x:>4}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        println!(
            "{label:<22} | {} | {} | {}  of {total}",
            f(&l[..n]),
            f(&l[n..]),
            f(&y)
        );
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
        "soft" => {
            // Soft-value width and clip: what the Viterbi decoder is handed. Fine soft first.
            let mut rx = vec![receiver("fine", true, fine())];
            for (name, soft_max, clip) in [
                ("4b/1.5", 7, 1.5),
                ("4b/3", 7, 3.0),
                ("4b/6", 7, 6.0),
                ("5b/1.5", 15, 1.5),
                ("5b/3", 15, 3.0),
                ("5b/6", 15, 6.0),
                ("6b/3", 31, 3.0),
                ("6b/6", 31, 6.0),
            ] {
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
        "sfo" => sfo(n.unwrap_or(40)),
        "timing" => timing(n.unwrap_or(300)),
        _ => {
            eprintln!("usage: rxper awgn|tgn|sfo|timing|soft [frames]");
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
