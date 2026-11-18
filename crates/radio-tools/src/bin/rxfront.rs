//! `rxfront` — Monte-Carlo measurements behind the R0-08a detector choice
//! (`docs/gates/rx_front.md`). Multi-threaded; long runs go to the build server.
//!
//! Usage: `cargo run -q --release -p radio-tools --bin rxfront -- <command> [args]`
//! - `tail <million samples>`: detection events per sample on complex white noise, per
//!   detector, threshold and run length — the false-alarm law the thresholds are chosen from.
//! - `pd <frames per point>`: detection probability within 4 µs, acquisition rate and CFO RMS
//!   versus SNR, per detector, in AWGN and TGn B/D/E, CFO uniform in ±235 kHz.
//! - `robust <million samples>`: detections on noise plus a CW tone or DC offset at several
//!   levels relative to the noise, and on noise with AGC gain steps.
//! - `frames <per SNR>`: impaired 6 Mb/s frames as int16 IQ for the outside-receiver black box
//!   (`tools/ref/rx_front_openofdm.py`); `tests/g1_rx_front_openofdm.rs` rebuilds them from the
//!   same recipe and checks the hash.
//!
//! The output goes into the gate journal / `docs/gates/reports/`; it is not a generated file.

use std::f64::consts::PI;
use std::process::ExitCode;

use radio_phy::channel::{
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, tgn_taps, Doppler,
    Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{
    acquire, acquisitions, backoff_samples, detect, detections, detector_metric, front, Detector,
    RxConfig,
};

type C = (f64, f64);

const FS: f64 = 20e6;

/// An impairment applied to a noise stream.
type Impairment = Box<dyn Fn(&[C]) -> Vec<C> + Sync>;
const DETECTORS: [Detector; 3] = [
    Detector::Autocorrelation,
    Detector::MatchedFilter,
    Detector::MatchedExact,
];
const T1: usize = 192;

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

fn noise(len: usize, rng: &mut Rng) -> Vec<C> {
    awgn(&vec![(0.0, 0.0); len], 0.0, 1.0, rng)
}

fn tail(million: usize) {
    let thetas: [&[f64]; 3] = [
        &[0.45, 0.475, 0.5, 0.525, 0.55, 0.575, 0.6],
        &[0.25, 0.275, 0.3, 0.325, 0.35, 0.375, 0.4],
        &[0.25, 0.275, 0.3, 0.325, 0.35, 0.375, 0.4],
    ];
    const RUNS: usize = 8;
    let per_thread = million.div_ceil(threads());
    println!("# rxfront tail: detection events per sample on complex white noise");
    println!(
        "# {} threads x {per_thread} M samples; columns: run length 1..{RUNS}",
        threads()
    );
    for (d, detector) in DETECTORS.iter().enumerate() {
        let cfg = RxConfig::new(FS).with_detector(*detector);
        let counts = par(|t| {
            let mut rng = Rng::new(0x7a11_0000 + (d * 1000 + t) as u64);
            let mut c = vec![[0u64; RUNS]; thetas[d].len()];
            let mut n = 0u64;
            for _ in 0..per_thread {
                let x = noise(1_000_000, &mut rng);
                let m = detector_metric(&front(&x, &cfg), &cfg);
                let valid = &m[200..];
                n += valid.len() as u64;
                for (i, th) in thetas[d].iter().enumerate() {
                    let mut run = 0usize;
                    for v in valid {
                        if v >= th {
                            run += 1;
                            for (r, cnt) in c[i].iter_mut().enumerate() {
                                if run.is_multiple_of(r + 1) {
                                    *cnt += 1;
                                }
                            }
                        } else {
                            run = 0;
                        }
                    }
                }
            }
            (c, n)
        });
        let n: u64 = counts.iter().map(|c| c.1).sum();
        println!("\n## {detector:?}: {n} samples");
        println!("# θ      events per sample for run length 1..{RUNS}");
        for (i, th) in thetas[d].iter().enumerate() {
            let line: Vec<String> = (0..RUNS)
                .map(|r| {
                    let k: u64 = counts.iter().map(|c| c.0[i][r]).sum();
                    format!("{:8.1e}", k as f64 / n as f64)
                })
                .collect();
            println!("  {th:.3} {}", line.join(" "));
        }
    }
}

fn preamble() -> Vec<C> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
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

fn pd(frames: usize) {
    let f = preamble();
    let snrs = [0.0, 2.0, 4.0, 6.0, 9.0, 12.0];
    let channels = [
        ("AWGN", Channel::Awgn),
        ("TGn B", Channel::Tgn(TgnModel::B)),
        ("TGn D", Channel::Tgn(TgnModel::D)),
        ("TGn E", Channel::Tgn(TgnModel::E)),
    ];
    let per_thread = frames.div_ceil(threads());
    println!(
        "# rxfront pd: {} frames per point, CFO uniform ±235 kHz, random start and",
        per_thread * threads()
    );
    println!("# fractional delay, static TGn realisation per frame (NLOS).");
    println!("# Pd = detected within 80 samples (4 µs) of the STF start; acq = acquired with");
    println!(
        "# the FFT window within ±2 samples of first path − back-off (AWGN) / acquired (TGn);"
    );
    println!("# CFO = RMS error of acquired frames in Hz.");
    for (cname, ch) in channels {
        println!("\n## {cname}");
        println!(
            "# detector          {}",
            snrs.iter()
                .map(|s| format!("{s:>5.0} dB: Pd    acq   CFO"))
                .collect::<Vec<_>>()
                .join(" |")
        );
        for detector in DETECTORS {
            let cfg = RxConfig::new(FS).with_detector(detector);
            let b = backoff_samples(&cfg) as f64;
            let mut cells = Vec::new();
            for (si, &snr) in snrs.iter().enumerate() {
                let res = par(|t| {
                    let mut rng = Rng::new(0x9d00_0000 + (si * 1000 + t) as u64);
                    let (mut det, mut acq, mut e2, mut na) = (0usize, 0usize, 0.0, 0usize);
                    for _ in 0..per_thread {
                        let lead = 200 + (rng.uniform() * 200.0) as usize;
                        let delay = rng.uniform();
                        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
                        let mut x = vec![(0.0, 0.0); lead];
                        x.extend_from_slice(&f);
                        x.extend(vec![(0.0, 0.0); 64]);
                        let x = match ch {
                            Channel::Awgn => fractional_delay(&x, delay),
                            Channel::Tgn(m) => {
                                Multipath::new(tgn_taps(m, false), Doppler::Static, 1, FS, &mut rng)
                                    .apply(&x)
                            }
                        };
                        let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                        let x = awgn(&x, snr, mean_power(&f), &mut rng);
                        if let Some(d) = detect(&x, 0, &cfg) {
                            if d >= lead && d < lead + 80 {
                                det += 1;
                            }
                        }
                        if let Some(a) = acquire(&x, 0, &cfg) {
                            let first = (lead + T1) as f64
                                + if matches!(ch, Channel::Awgn) {
                                    delay
                                } else {
                                    0.0
                                };
                            let ok = match ch {
                                Channel::Awgn => (a.fft_start as f64 - (first - b)).abs() <= 2.0,
                                Channel::Tgn(_) => (a.fft_start as f64 - first).abs() <= 16.0,
                            };
                            if ok {
                                acq += 1;
                                e2 += (a.cfo_hz - cfo).powi(2);
                                na += 1;
                            }
                        }
                    }
                    (det, acq, e2, na)
                });
                let n = (per_thread * threads()) as f64;
                let det: usize = res.iter().map(|r| r.0).sum();
                let acq: usize = res.iter().map(|r| r.1).sum();
                let e2: f64 = res.iter().map(|r| r.2).sum();
                let na: usize = res.iter().map(|r| r.3).sum();
                let rms = if na > 0 {
                    (e2 / na as f64).sqrt()
                } else {
                    f64::NAN
                };
                cells.push(format!(
                    "{:>9.4} {:.4} {:5.0}",
                    det as f64 / n,
                    acq as f64 / n,
                    rms
                ));
            }
            println!("  {:<16} {}", format!("{detector:?}"), cells.join(" |"));
        }
    }
}

fn robust(million: usize) {
    let per_thread = million.div_ceil(threads()).max(1);
    let total = per_thread * threads();
    println!(
        "# rxfront robust: detection events in {total} M samples of unit noise plus an impairment"
    );
    let cases: Vec<(String, Impairment)> = {
        let mut v: Vec<(String, Impairment)> = Vec::new();
        for db in [-10.0, -3.0, 0.0, 3.0, 10.0, 20.0, 30.0] {
            v.push((
                format!("DC {db:+} dB"),
                Box::new(move |x: &[C]| add_tone(x, 10f64.powf(db / 10.0), 0.0, FS, 0.4)),
            ));
        }
        for f in [0.3e6, 1.25e6, 3.1e6] {
            for db in [-10.0, -3.0, 0.0, 3.0, 10.0, 20.0] {
                v.push((
                    format!("CW {:.2} MHz {db:+} dB", f / 1e6),
                    Box::new(move |x: &[C]| add_tone(x, 10f64.powf(db / 10.0), f, FS, 0.0)),
                ));
            }
        }
        for db in [10.0, 20.0, 30.0] {
            v.push((
                format!("gain steps ±{db} dB every 0.1 M, DC 0 dB"),
                Box::new(move |x: &[C]| {
                    let mut y = add_tone(x, 1.0, 0.0, FS, 0.3);
                    for (i, k) in (100_000..x.len()).step_by(100_000).enumerate() {
                        y = gain_step(&y, k, if i % 2 == 0 { db } else { -db });
                    }
                    y
                }),
            ));
        }
        v
    };
    println!("# cells: detections / false acquisitions (detections the LTF stage confirmed)");
    println!(
        "# case                            {}",
        DETECTORS
            .map(|d| format!("{:>18}", format!("{d:?}")))
            .join("")
    );
    for (ci, (name, imp)) in cases.iter().enumerate() {
        let mut row = Vec::new();
        for detector in DETECTORS {
            let cfg = RxConfig::new(FS).with_detector(detector);
            let r = par(|t| {
                let mut rng = Rng::new(0xb0b0_0000 + (ci * 1000 + t) as u64);
                let (mut d, mut a) = (0usize, 0usize);
                for _ in 0..per_thread {
                    let x = imp(&noise(1_000_000, &mut rng));
                    d += detections(&x, &cfg).len();
                    a += acquisitions(&x, &cfg).len();
                }
                (d, a)
            });
            let d: usize = r.iter().map(|v| v.0).sum();
            let a: usize = r.iter().map(|v| v.1).sum();
            row.push(format!("{:>18}", format!("{d} / {a}")));
        }
        println!("  {name:<32}{}", row.join(""));
    }
    // Packets under the impairment: 10 dB SNR frames (noise power 1), CFO ±235 kHz.
    let f = preamble();
    let frames = 100 * threads();
    println!("\n# acquisition rate of 10 dB frames under the impairment ({frames} frames)");
    for (ci, (name, imp)) in cases.iter().enumerate() {
        let mut row = Vec::new();
        for detector in DETECTORS {
            let cfg = RxConfig::new(FS).with_detector(detector);
            let ok: usize = par(|t| {
                let mut rng = Rng::new(0xacc0_0000 + (ci * 1000 + t) as u64);
                let mut ok = 0;
                for _ in 0..100 {
                    let lead = 300 + (rng.uniform() * 200.0) as usize;
                    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
                    let mut x = vec![(0.0, 0.0); lead];
                    x.extend_from_slice(&f);
                    x.extend(vec![(0.0, 0.0); 64]);
                    let g = (10.0 / mean_power(&f)).sqrt(); // signal power 10 = 10 dB over noise
                    let x: Vec<C> = x.iter().map(|s| (g * s.0, g * s.1)).collect();
                    let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                    let x = awgn(&x, 10.0, 10.0, &mut rng);
                    let x = imp(&x);
                    if let Some(a) = acquire(&x, 0, &cfg) {
                        if (a.fft_start as f64 - (lead + T1) as f64 + 4.0).abs() <= 2.0 {
                            ok += 1;
                        }
                    }
                }
                ok
            })
            .iter()
            .sum();
            row.push(format!("{:>18.3}", ok as f64 / frames as f64));
        }
        println!("  {name:<32}{}", row.join(""));
    }
}

/// SNRs of the black-box frames.
const BB_SNRS: [f64; 4] = [2.0, 4.0, 6.0, 9.0];
const BB_LEAD: usize = 437;
const BB_TRAIL: usize = 400;
const BB_SCALE: f64 = 20000.0;

/// Frame `k` of the black-box set (the recipe `g1_rx_front_openofdm.rs` repeats): zeros, a
/// 6 Mb/s frame with a random 20-octet PSDU, fractional delay, CFO, noise over the frame only
/// (OpenOFDM's absolute power trigger cannot work in continuous noise), int16 at ×20000.
fn bb_frame(k: usize, per_snr: usize) -> (f64, f64, Vec<(i16, i16)>) {
    let snr = BB_SNRS[(k / per_snr).min(BB_SNRS.len() - 1)];
    let mut rng = Rng::new(0x0fd0_0000 + k as u64);
    let psdu: Vec<u8> = (0..20).map(|_| (rng.next_u64() & 0xff) as u8).collect();
    let seed = 1 + (rng.next_u64() % 127) as u8;
    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
    let delay = rng.uniform();
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let f: Vec<C> = build_ppdu(&psdu, Rate::R6, seed, &cfg)
        .map(|f| {
            f.iter()
                .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
                .collect()
        })
        .unwrap_or_default();
    let mut x = vec![(0.0, 0.0); BB_LEAD];
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); BB_TRAIL]);
    let x = fractional_delay(&x, delay);
    let mut x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
    let span = BB_LEAD..BB_LEAD + f.len() + 32;
    let noisy = awgn(&x[span.clone()], snr, mean_power(&f), &mut rng);
    x[span].copy_from_slice(&noisy);
    let q = |v: f64| (v * BB_SCALE).round().clamp(-32768.0, 32767.0) as i16;
    (snr, cfo, x.iter().map(|s| (q(s.0), q(s.1))).collect())
}

fn frames(per_snr: usize) {
    println!(
        "# rxfront frames: {per_snr} per SNR {BB_SNRS:?}, 6 Mb/s, 20-octet PSDU, int16 x{BB_SCALE}"
    );
    println!("# <k> <snr dB> <cfo Hz> <i,q>...");
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for k in 0..per_snr * BB_SNRS.len() {
        let (snr, cfo, iq) = bb_frame(k, per_snr);
        for (i, q) in &iq {
            for b in i.to_le_bytes().iter().chain(q.to_le_bytes().iter()) {
                h ^= u64::from(*b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        let body: Vec<String> = iq.iter().map(|(i, q)| format!("{i},{q}")).collect();
        println!("{k} {snr} {cfo:.3} {}", body.join(" "));
    }
    // FNV-1a over every frame's int16 samples (i then q, little-endian), in order.
    println!(
        "# frames {} per-snr {per_snr} fnv {h:016x}",
        per_snr * BB_SNRS.len()
    );
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let cmd = args.next().unwrap_or_default();
    let n: Option<usize> = args.next().and_then(|v| v.parse().ok());
    match cmd.as_str() {
        "tail" => tail(n.unwrap_or(20)),
        "pd" => pd(n.unwrap_or(2000)),
        "robust" => robust(n.unwrap_or(4)),
        "frames" => frames(n.unwrap_or(200)),
        _ => {
            eprintln!("usage: rxfront tail|pd|robust|frames [n]");
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
