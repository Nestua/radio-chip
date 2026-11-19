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
//! - `g2 <frames per case>`: G2 of the fixed-point model (R0-08b): one-at-a-time word-length
//!   sweep against the float model on paired ADC frames, with detector false alarms on noise.
//! - `g2joint <frames per case>`: the chosen widths and a generous set on many paired frames,
//!   listing every FFT-start disagreement.
//! - `tailfx <million>`: false alarms of the fixed detector on noise at five input levels.
//! - `latency <streams per kind>`: open question 13, the acquisition decision applied L samples
//!   after acceptance — differences from L = 0 on clean, CW-tone, AGC-step, two-packet and
//!   noise-only streams, float and fixed.
//!
//! - `agc <frames per case>`: the fixed receiver on packets whose start goes through an AGC
//!   settling transient (clipping, gain steps with phase jumps and DC changes inside the STF):
//!   acquisition, timing, and the error of the CFO handed on (output) and of the coarse and
//!   fine estimates.
//! - `tailbw <million>`: detector events and acquisitions of the fixed receiver on noise that
//!   went through a channel-select low-pass filter (band-limited noise is correlated at lag 16).
//! - `agcmodel <frames per point>`: the same, with the gain steps placed by the gain control
//!   model (`channel::FastAttackAgc`) instead of by hand: when the gain is still, where the
//!   packet's level ends, and what the fixed receiver makes of it.
//! - `level <frames per point>`: float and fixed receiver on the same ADC samples against the
//!   packet's level in the ADC, at three SNRs.
//!
//! The output goes into the gate journal / `docs/gates/reports/`; it is not a generated file.

use std::f64::consts::PI;
use std::process::ExitCode;

use radio_phy::channel::{
    add_tone, agc_transient, awgn, fractional_delay, frequency_offset, gain_step, mean_power,
    tgn_taps, Doppler, FastAttackAgc, GainState, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{
    acquire, acquisitions, backoff_samples, detect, detections, detector_metric, front, Detector,
    RxConfig,
};
use radio_phy::rx_front_fixed::{
    acquire_fixed, acquisitions_fixed, detections_fixed, to_adc, FixedRxConfig, ADC_FULL_SCALE,
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
    println!("\n# acquisition rate of 10 dB frames under the impairment ({frames} frames):");
    println!("# FFT start within ±2 samples and CFO within 2 % of the subcarrier spacing");
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
                        // Timing within ±2 samples and CFO within 2 % of the subcarrier spacing.
                        if (a.fft_start as f64 - (lead + T1) as f64 + 4.0).abs() <= 2.0
                            && (a.cfo_hz - cfo).abs() <= 0.02 * FS / 64.0
                        {
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

/// One paired G2 frame: ADC samples, STF start, true CFO and the float model's outcome.
struct G2Frame {
    case: usize,
    adc: Vec<(i16, i16)>,
    stf: usize,
    cfo: f64,
    det: bool,
    acq: Option<(usize, f64)>,
}

/// A fixed-model outcome: frame index, detected in time, (FFT start, CFO Hz) if acquired.
type G2Row = (usize, bool, Option<(usize, f64)>);

/// The G2 cases: name, SNR, TGn D or AWGN.
const G2_CASES: [(&str, f64, bool); 4] = [
    ("AWGN 4 dB", 4.0, false),
    ("AWGN 10 dB", 10.0, false),
    ("AWGN 25 dB", 25.0, false),
    ("TGn D 9 dB", 9.0, true),
];

fn g2_frames(per_case: usize) -> Vec<G2Frame> {
    let f = preamble();
    let cfg = RxConfig::new(FS);
    let d_taps = tgn_taps(TgnModel::D, false);
    let per_thread = per_case.div_ceil(threads());
    par(|t| {
        let mut rng = Rng::new(0x9200_0000 + t as u64);
        let mut out = Vec::new();
        for (case, &(_, snr, tgn)) in G2_CASES.iter().enumerate() {
            for _ in 0..per_thread {
                let lead = 200 + (rng.uniform() * 300.0) as usize;
                let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
                let mut x = vec![(0.0, 0.0); lead];
                x.extend_from_slice(&f);
                x.extend(vec![(0.0, 0.0); 400]);
                let x = if tgn {
                    Multipath::new(d_taps.clone(), Doppler::Static, 1, FS, &mut rng).apply(&x)
                } else {
                    fractional_delay(&x, rng.uniform())
                };
                let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                let p = mean_power(&f);
                let x = awgn(&x, snr, p, &mut rng);
                let rms = (p * (1.0 + 10f64.powf(-snr / 10.0))).sqrt();
                let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / rms;
                let adc = to_adc(&x, gain);
                let xf: Vec<C> = adc
                    .iter()
                    .map(|&(a, b)| (f64::from(a), f64::from(b)))
                    .collect();
                let det = detections(&xf, &cfg)
                    .iter()
                    .any(|&d| d >= lead && d < lead + 80);
                let acq = acquire(&xf, 0, &cfg).map(|a| (a.fft_start, a.cfo_hz));
                out.push(G2Frame {
                    case,
                    adc,
                    stf: lead,
                    cfo,
                    det,
                    acq,
                });
            }
        }
        out
    })
    .into_iter()
    .flatten()
    .collect()
}

/// Budget check of one configuration on the paired frames: per case Pd and acquisition
/// differences, CFO RMS ratio; overall timing mismatches. Returns (pass, summary).
fn g2_eval(frames: &[G2Frame], fx: &FixedRxConfig) -> (bool, String) {
    let cfg = RxConfig::new(FS);
    let chunk = frames.len().div_ceil(threads());
    let rows: Vec<G2Row> = par(|t| {
        frames
            .iter()
            .enumerate()
            .skip(t * chunk)
            .take(chunk)
            .map(|(i, fr)| {
                let det = detections_fixed(&fr.adc, &cfg, fx)
                    .iter()
                    .any(|&d| d >= fr.stf && d < fr.stf + 80);
                let acq = acquire_fixed(&fr.adc, &cfg, fx).map(|a| (a.fft_start, a.cfo_hz(fx, FS)));
                (i, det, acq)
            })
            .collect::<Vec<_>>()
    })
    .into_iter()
    .flatten()
    .collect();
    let mut pass = true;
    let mut parts = Vec::new();
    let (mut both_all, mut mism_all) = (0usize, 0usize);
    for case in 0..G2_CASES.len() {
        let (mut dpf, mut dpx, mut ddis, mut of, mut ox, mut both, mut mism) =
            (0i64, 0i64, 0i64, 0i64, 0i64, 0, 0);
        let (mut ef, mut ex) = (0.0, 0.0);
        let mut n = 0;
        for &(i, det, acq) in &rows {
            let fr = &frames[i];
            if fr.case != case {
                continue;
            }
            n += 1;
            dpf += i64::from(fr.det);
            ddis += i64::from(fr.det != det);
            dpx += i64::from(det);
            match (fr.acq, acq) {
                (Some(a), Some(b)) => {
                    both += 1;
                    if a.0 != b.0 {
                        mism += 1;
                    }
                    ef += (a.1 - fr.cfo).powi(2);
                    ex += (b.1 - fr.cfo).powi(2);
                }
                (Some(_), None) => of += 1,
                (None, Some(_)) => ox += 1,
                (None, None) => {}
            }
        }
        let tol = n as f64 * 0.01 + 3.0 * (n as f64 * 0.01).sqrt();
        let ratio = (ex / ef.max(1e-30)).sqrt();
        pass &= (ddis as f64) <= tol && ((of + ox) as f64) <= tol && ratio <= 1.10;
        both_all += both;
        mism_all += mism;
        parts.push(format!(
            "Pd{:+} (≠{ddis}) acq≠{} cfo{:+.1}%",
            dpx - dpf,
            of + ox,
            (ratio - 1.0) * 100.0
        ));
    }
    pass &= mism_all * 1000 <= both_all;
    (
        pass,
        format!("timing≠ {mism_all}/{both_all} | {}", parts.join(" | ")),
    )
}

/// False alarms of the fixed detector on `million` M noise samples per level.
fn g2_noise(fx: &FixedRxConfig, million: usize) -> usize {
    let cfg = RxConfig::new(FS);
    let levels = [-15.0, -30.0, -45.0, -55.0, -65.0];
    let per_thread = (million * levels.len()).div_ceil(threads());
    par(|t| {
        let mut rng = Rng::new(0x9300_0000 + t as u64);
        let mut n = 0;
        for k in 0..per_thread {
            let level: f64 = levels[(t * per_thread + k) % levels.len()];
            let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(level / 20.0);
            n += detections_fixed(&to_adc(&noise(1_000_000, &mut rng), gain), &cfg, fx).len();
        }
        n
    })
    .iter()
    .sum()
}

/// `g2 <frames per case>`: one-at-a-time word-length sweep of the fixed-point receiver front
/// against the float model on paired frames (all other widths at their defaults).
fn g2(per_case: usize) {
    let frames = g2_frames(per_case);
    let base = FixedRxConfig::default();
    println!(
        "# rxfront g2: fixed-point receiver front vs float on {} paired frames",
        frames.len()
    );
    println!(
        "# ({per_case} per case: {}), packet at -15 dBFS, CFO ±235 kHz",
        G2_CASES.map(|c| c.0).join(", ")
    );
    println!(
        "# budget: frames where detection or acquisition disagree ≤ 1 % + 3σ per case, CFO RMS"
    );
    println!(
        "# ≤ +10 % per case, FFT start identical in ≥ 99.9 % overall; FA = detector events on"
    );
    println!(
        "# 5 × 3 M noise samples at -15/-30/-45/-55/-65 dBFS (detector widths only; must be 0)"
    );
    let (ok, s) = g2_eval(&frames, &base);
    println!(
        "\n## defaults {base:?}\n  {} {s}",
        if ok { "PASS" } else { "FAIL" }
    );
    type Set = fn(&mut FixedRxConfig, u32);
    let sweeps: [(&str, std::ops::RangeInclusive<u32>, Set, bool); 12] = [
        ("dc_frac F", 2..=12, |f, v| f.dc_frac = v, true),
        ("det_bits D", 6..=13, |f, v| f.det_bits = v, true),
        ("cordic_iters N", 6..=16, |f, v| f.cordic_iters = v, false),
        ("cordic_bits W_c", 8..=20, |f, v| f.cordic_bits = v, false),
        (
            "angle_bits A (N = min(16, A))",
            8..=16,
            |f, v| {
                f.angle_bits = v;
                f.cordic_iters = v.min(16)
            },
            false,
        ),
        (
            "nco_phase_bits P",
            22..=28,
            |f, v| f.nco_phase_bits = v,
            false,
        ),
        ("nco_lut_bits L", 4..=12, |f, v| f.nco_lut_bits = v, false),
        ("nco_amp_bits Q", 4..=14, |f, v| f.nco_amp_bits = v, false),
        ("z_bits Z", 3..=12, |f, v| f.z_bits = v, false),
        ("tap_bits T", 1..=8, |f, v| f.tap_bits = v, false),
        ("corr_bits K", 6..=18, |f, v| f.corr_bits = v, false),
        ("est_bits E", 6..=13, |f, v| f.est_bits = v, false),
    ];
    for (name, range, set, noise_check) in sweeps {
        println!("\n## {name}");
        for v in range {
            let mut fx = base;
            set(&mut fx, v);
            let (ok, s) = g2_eval(&frames, &fx);
            let fa = noise_check.then(|| g2_noise(&fx, 3));
            let ok = ok && fa.is_none_or(|n| n == 0);
            let fa = fa.map_or(String::new(), |n| format!(" | FA {n}"));
            println!("  {v:>2} {} {s}{fa}", if ok { "PASS" } else { "FAIL" });
        }
    }
}

/// `tailfx <million>`: detection events of the fixed detector (defaults) on noise, per level.
fn tailfx(million: usize) {
    let fx = FixedRxConfig::default();
    let n = g2_noise(&fx, million);
    println!("# rxfront tailfx: fixed detector (defaults), {million} M noise samples at each of -15/-30/-45/-55/-65 dBFS: {n} events");
}

/// `g2joint <frames per case>`: the chosen widths and a generous set on many paired frames,
/// with every FFT-start disagreement listed (fixed − float, in samples).
fn g2joint(per_case: usize) {
    let frames = g2_frames(per_case);
    let cfg = RxConfig::new(FS);
    let generous = FixedRxConfig {
        dc_frac: 14,
        cordic_iters: 18,
        cordic_bits: 24,
        angle_bits: 18,
        nco_phase_bits: 28,
        nco_lut_bits: 14,
        nco_amp_bits: 16,
        z_bits: 13,
        tap_bits: 10,
        corr_bits: 24,
        est_bits: 13,
        ..FixedRxConfig::default()
    };
    println!(
        "# rxfront g2joint: {} paired frames ({per_case} per case)",
        frames.len()
    );
    let c = FixedRxConfig::default();
    let first = FixedRxConfig {
        angle_bits: 14,
        cordic_iters: 14,
        cordic_bits: 15,
        z_bits: 10,
        nco_amp_bits: 9,
        corr_bits: 17,
        ..c
    };
    let variants = [
        ("chosen", c),
        ("generous", generous),
        ("first joint set (A14 N14 W_c15 Z10 Q9 K17)", first),
    ];
    for (name, fx) in variants {
        let (ok, s) = g2_eval(&frames, &fx);
        let chunk = frames.len().div_ceil(threads());
        let offs: Vec<(usize, i64)> = par(|t| {
            frames
                .iter()
                .skip(t * chunk)
                .take(chunk)
                .filter_map(|fr| {
                    let a = fr.acq?;
                    let b = acquire_fixed(&fr.adc, &cfg, &fx)?;
                    (a.0 != b.fft_start).then_some((fr.case, b.fft_start as i64 - a.0 as i64))
                })
                .collect::<Vec<_>>()
        })
        .into_iter()
        .flatten()
        .collect();
        println!(
            "## {name}: {} {s}\n   disagreements (case, fixed − float): {offs:?}",
            if ok { "PASS" } else { "FAIL" }
        );
    }
}

/// Decision latencies swept by `latency` (samples at 20 MHz).
const LATENCIES: [usize; 8] = [0, 16, 32, 48, 63, 64, 79, 128];

/// One `latency` trial: a stream exercising the decision path. Kinds: 0 a clean packet
/// (AWGN or static TGn D), 1 a CW tone over the whole stream (failing candidates before the
/// packet), 2 an AGC gain step down before the packet (the DC transient), 3 two packets with
/// a gap of 0–400 samples (re-arm after an acquisition), 4 noise only (false acquisitions),
/// 5 two packets at 20 dB under a CW tone at −6…+6 dB relative to the signal (the detector
/// keeps firing after an acquisition).
fn latency_stream(kind: usize, t: u64) -> Vec<C> {
    let mut rng = Rng::new(0x1a7e_0000 + 1_000_000 * kind as u64 + t);
    let snr = if kind == 5 {
        20.0
    } else {
        [0.0, 3.0, 6.0, 12.0][(rng.next_u64() % 4) as usize]
    };
    let packet = |rng: &mut Rng| -> Vec<C> {
        let psdu: Vec<u8> = (0..20).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let seed = 1 + (rng.next_u64() % 127) as u8;
        let cfg = IfftConfig::tx_default();
        let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
        build_ppdu(&psdu, Rate::R6, seed, &cfg)
            .map(|f| {
                f.iter()
                    .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
                    .collect()
            })
            .unwrap_or_default()
    };
    let f = packet(&mut rng);
    let p = mean_power(&f);
    let lead = 300 + (rng.next_u64() % 300) as usize;
    let mut x = vec![(0.0, 0.0); lead];
    if kind != 4 {
        x.extend_from_slice(&f);
    }
    if kind == 3 || kind == 5 {
        let gap = (rng.next_u64() % 401) as usize;
        x.extend(vec![(0.0, 0.0); gap]);
        x.extend(packet(&mut rng));
    }
    x.extend(vec![(0.0, 0.0); 400]);
    if kind == 4 {
        x.extend(vec![(0.0, 0.0); 4000]);
    }
    let x = if kind == 0 && rng.next_u64() % 2 == 1 {
        Multipath::new(
            tgn_taps(TgnModel::D, false),
            Doppler::Static,
            1,
            FS,
            &mut rng,
        )
        .apply(&x)
    } else {
        fractional_delay(&x, rng.uniform())
    };
    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
    let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
    let x = awgn(&x, snr, p, &mut rng);
    match kind {
        1 => {
            let noise_p = p * 10f64.powf(-snr / 10.0);
            let rel = [0.0, 6.0, 12.0][(rng.next_u64() % 3) as usize];
            let fhz = (rng.uniform() * 2.0 - 1.0) * 5e6;
            add_tone(
                &x,
                noise_p * 10f64.powf(rel / 10.0),
                fhz,
                FS,
                2.0 * PI * rng.uniform(),
            )
        }
        5 => {
            let rel = [-6.0, 0.0, 6.0][(rng.next_u64() % 3) as usize];
            let fhz = (rng.uniform() * 2.0 - 1.0) * 5e6;
            add_tone(
                &x,
                p * 10f64.powf(rel / 10.0),
                fhz,
                FS,
                2.0 * PI * rng.uniform(),
            )
        }
        2 => gain_step(
            &x,
            lead.saturating_sub(40 + (rng.next_u64() % 200) as usize),
            -20.0,
        ),
        _ => x,
    }
}

/// `latency <trials per kind>`: the explicit decision latency of open question 13. Each
/// stream runs through the float and the fixed receiver at every latency in [`LATENCIES`].
/// Counted per stream and model against latency 0: `grid` — the acquisitions differ in
/// anything but `detected_at` and `decided_at` (count, timing, CFO, metric); `det` — they
/// differ in `detected_at` only; `late` — the decision comes more than L after the latency-0
/// one (most extra samples in brackets).
fn latency(trials: usize) {
    const KINDS: [&str; 6] = [
        "clean",
        "CW tone",
        "AGC step",
        "two packets",
        "noise only",
        "tone+2 pkts",
    ];
    let per_thread = trials.div_ceil(threads());
    let fx = FixedRxConfig::default();
    println!(
        "# rxfront latency: {} streams per kind, SNR 0/3/6/12 dB (tone+2 pkts: 20 dB), CFO ±235 kHz, 20 MHz",
        per_thread * threads()
    );
    println!("# per L, float / fixed: streams whose acquisitions differ from L = 0 in timing/CFO/");
    println!("# metric/count (grid), in detected_at only (det), decided more than L later (late,");
    println!("# most extra samples). Last column: acquisitions at L = 0, float / fixed.");
    print!("# {:<14}", "kind");
    for l in LATENCIES.iter().skip(1) {
        print!(" {:>13}", format!("L={l}"));
    }
    println!(" {:>13}", "acq at L=0");
    // [model][latency] -> (grid, det, late, max extra)
    type Cell = (usize, usize, usize, usize);
    type Tally = [[Cell; LATENCIES.len()]; 2];
    for (kind, name) in KINDS.iter().enumerate() {
        let res = par(|t| {
            let mut tally: Tally = [[(0, 0, 0, 0); LATENCIES.len()]; 2];
            let mut acq = [0usize; 2];
            for k in 0..per_thread {
                let x = latency_stream(kind, (t * per_thread + k) as u64);
                let rms = mean_power(&x).sqrt().max(1e-12);
                let adc = to_adc(
                    &x,
                    f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / rms,
                );
                let xf: Vec<C> = adc
                    .iter()
                    .map(|&(a, b)| (f64::from(a), f64::from(b)))
                    .collect();
                // (key without detected_at/decided_at, detected_at, decided_at) per acquisition
                type Row = (Vec<u64>, usize, usize);
                let run = |l: usize| -> [Vec<Row>; 2] {
                    let cfg = RxConfig {
                        decision_latency: l,
                        ..RxConfig::new(FS)
                    };
                    let f = acquisitions(&xf, &cfg)
                        .iter()
                        .map(|a| {
                            let key = vec![
                                a.ltf_start as u64,
                                a.fft_start as u64,
                                a.cfo_hz.to_bits(),
                                a.coarse_cfo_hz.to_bits(),
                                a.fine_cfo_hz.to_bits(),
                                a.ltf_metric.to_bits(),
                            ];
                            (key, a.detected_at, a.decided_at)
                        })
                        .collect();
                    let x = acquisitions_fixed(&adc, &cfg, &fx)
                        .iter()
                        .map(|a| {
                            let key = vec![
                                a.ltf_start as u64,
                                a.fft_start as u64,
                                a.cfo_phase as u64,
                                a.coarse_phase as u64,
                                a.fine_phase as u64,
                            ];
                            (key, a.detected_at, a.decided_at)
                        })
                        .collect();
                    [f, x]
                };
                let base = run(0);
                acq[0] += base[0].len();
                acq[1] += base[1].len();
                for (li, &l) in LATENCIES.iter().enumerate().skip(1) {
                    let got = run(l);
                    for m in 0..2 {
                        let (g, b) = (&got[m], &base[m]);
                        let tl = &mut tally[m][li];
                        if g.len() != b.len() || g.iter().zip(b).any(|(x, y)| x.0 != y.0) {
                            tl.0 += 1;
                            continue;
                        }
                        if g.iter().zip(b).any(|(x, y)| x.1 != y.1) {
                            tl.1 += 1;
                        }
                        let extra = g
                            .iter()
                            .zip(b)
                            .map(|(x, y)| x.2 as i64 - y.2 as i64 - l as i64)
                            .max()
                            .unwrap_or(0);
                        if extra > 0 {
                            tl.2 += 1;
                            tl.3 = tl.3.max(extra as usize);
                        }
                    }
                }
            }
            (tally, acq)
        });
        let mut tally: Tally = [[(0, 0, 0, 0); LATENCIES.len()]; 2];
        let mut acq = [0usize; 2];
        for (tr, ar) in &res {
            for m in 0..2 {
                acq[m] += ar[m];
                for (d, v) in tally[m].iter_mut().zip(&tr[m]) {
                    *d = (d.0 + v.0, d.1 + v.1, d.2 + v.2, d.3.max(v.3));
                }
            }
        }
        let row = |label: &str, f: &dyn Fn(&Cell) -> String| {
            print!("  {label:<14}");
            for (a, b) in tally[0].iter().zip(&tally[1]).skip(1) {
                print!(" {:>13}", format!("{}/{}", f(a), f(b)));
            }
        };
        println!("  {name}");
        row("   grid", &|t| t.0.to_string());
        println!(" {:>13}", format!("{}/{}", acq[0], acq[1]));
        row("   det", &|t| t.1.to_string());
        println!();
        row("   late", &|t| {
            if t.2 == 0 {
                "0".into()
            } else {
                format!("{}({})", t.2, t.3)
            }
        });
        println!();
    }
}

/// One `agc` case: gain steps as (offset from the STF's first sample, dB above the settled gain
/// before it), a phase jump uniform in ±`jump_deg` at each step, a DC offset per state at
/// `dc_dbfs` with a random phase, and the level the packet settles at.
struct AgcCase {
    name: String,
    steps: Vec<(usize, f64)>,
    jump_deg: f64,
    dc_dbfs: Option<f64>,
    settled_dbfs: f64,
}

/// The fixed receiver on packets through an AGC settling transient. Before the first step the
/// chain is at a higher gain (the idle state on noise); at −15 dBFS settled a +20 dB start
/// clips in the ADC, at −40 dBFS it does not.
///
/// The phase jumps are an assumption, not a measurement: no capture of a real AGC is behind
/// them yet (R1). The cases separate the causes so that the result can be reread when one is.
fn agc(frames: usize) {
    const SNR_DB: f64 = 20.0;
    let cfg = RxConfig::new(FS);
    let fx = FixedRxConfig::default();
    let frame = preamble();
    let p = mean_power(&frame);
    let full = f64::from(ADC_FULL_SCALE);
    let unit = FS / 2f64.powi(fx.nco_phase_bits as i32);
    let b = backoff_samples(&cfg);
    let mut cases: Vec<(&str, Vec<AgcCase>)> = Vec::new();
    let case =
        |name: String, steps: &[(usize, f64)], jump: f64, dc: Option<f64>, lvl: f64| AgcCase {
            name,
            steps: steps.to_vec(),
            jump_deg: jump,
            dc_dbfs: dc,
            settled_dbfs: lvl,
        };
    cases.push((
        "causes apart, one event at STF sample 48",
        vec![
            case("no gain change".into(), &[], 0.0, None, -15.0),
            case(
                "no gain change, settled at −40 dBFS".into(),
                &[],
                0.0,
                None,
                -40.0,
            ),
            case(
                "−6 dB, no clipping, no jump".into(),
                &[(48, 6.0)],
                0.0,
                None,
                -15.0,
            ),
            case(
                "−20 dB, no clipping (−40 dBFS), no jump".into(),
                &[(48, 20.0)],
                0.0,
                None,
                -40.0,
            ),
            case(
                "−20 dB, clipping before, no jump".into(),
                &[(48, 20.0)],
                0.0,
                None,
                -15.0,
            ),
            case(
                "no gain change, jump ±10°".into(),
                &[(48, 0.0)],
                10.0,
                None,
                -15.0,
            ),
            case(
                "no gain change, jump ±30°".into(),
                &[(48, 0.0)],
                30.0,
                None,
                -15.0,
            ),
            case(
                "no gain change, jump ±90°".into(),
                &[(48, 0.0)],
                90.0,
                None,
                -15.0,
            ),
        ],
    ));
    let mut combined = Vec::new();
    for jump in [10.0, 30.0, 90.0] {
        combined.push(case(
            format!("−20 dB at 48, clipping, jump ±{jump}°"),
            &[(48, 20.0)],
            jump,
            None,
            -15.0,
        ));
    }
    for jump in [0.0, 30.0] {
        combined.push(case(
            format!("−20, −10 dB at 32, 64, DC −30 dBFS, jump ±{jump}°"),
            &[(32, 30.0), (64, 10.0)],
            jump,
            Some(-30.0),
            -15.0,
        ));
        combined.push(case(
            format!("−20, −10 dB at 48, 96, DC −30 dBFS, jump ±{jump}°"),
            &[(48, 30.0), (96, 10.0)],
            jump,
            Some(-30.0),
            -15.0,
        ));
    }
    cases.push(("causes together", combined));
    let mut sweep = Vec::new();
    // STF 0…159, GI2 160…191, T1 192…255, T2 256…319.
    for at in [16, 32, 48, 64, 80, 96, 112, 128, 144, 176, 208, 272] {
        sweep.push(case(
            format!("−20 dB at {at}, clipping, jump ±30°"),
            &[(at, 20.0)],
            30.0,
            None,
            -15.0,
        ));
    }
    cases.push(("position of a −20 dB step", sweep));
    let mut trim = Vec::new();
    for at in [100, 144, 176, 208, 272] {
        for jump in [0.0, 10.0, 30.0] {
            trim.push(case(
                format!("−2 dB trim at {at}, jump ±{jump}°"),
                &[(at, 2.0)],
                jump,
                None,
                -15.0,
            ));
        }
    }
    cases.push(("position of a −2 dB trim", trim));
    println!(
        "# rxfront agc: RxFrontFixed, {frames} frames per case, SNR {SNR_DB} dB, CFO uniform in ±235 kHz.\n\
         # Steps are offsets from the STF's first sample (STF 0…159, GI2 160…191, T1 192…255, T2\n\
         # 256…319); the gain before a step is that many dB above the settled gain. Settled at\n\
         # −15 dBFS unless stated. The phase jumps are assumed, not measured on a real AGC.\n\
         # timing = FFT start exact; ±64 = off by one whole long training symbol.\n\
         # RMS errors in Hz over the frames with exact timing; slips = of those, fine estimate off\n\
         # by more than fs/256 (its lag-64 ambiguity resolved wrongly), left out of the fine RMS."
    );
    let mut ci = 0u64;
    for (title, group) in &cases {
        println!(
            "\n## {title}\n# case                                          acquired  timing    ±64     output     coarse       fine  slips"
        );
        for c in group {
            ci += 1;
            let settled = full * 10f64.powf(c.settled_dbfs / 20.0) / p.sqrt();
            let mut rng = Rng::new(0xa6c0 + ci);
            let (mut acq, mut tim, mut sym, mut slips) = (0usize, 0usize, 0usize, 0usize);
            let (mut se, mut sc, mut sf) = (0.0f64, 0.0f64, 0.0f64);
            for _ in 0..frames {
                let lead = 600 + (rng.uniform() * 50.0) as usize;
                let mut x = vec![(0.0, 0.0); lead];
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 300]);
                let cfo = (2.0 * rng.uniform() - 1.0) * 235e3;
                let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                let x = awgn(&x, SNR_DB, p, &mut rng);
                let mut phase = 0.0;
                let mut states: Vec<GainState> = Vec::new();
                // Phases accumulate backwards from the settled chain (phase 0 after the last
                // step).
                for &(at, db) in c.steps.iter().rev() {
                    phase += (2.0 * rng.uniform() - 1.0) * c.jump_deg.to_radians();
                    let a = c
                        .dc_dbfs
                        .map_or(0.0, |d| full * 10f64.powf(d / 20.0) / settled);
                    let t = 2.0 * PI * rng.uniform();
                    states.push(GainState {
                        until: lead + at,
                        gain_db: db,
                        phase,
                        dc: (a * t.cos(), a * t.sin()),
                    });
                }
                states.reverse();
                let adc = to_adc(&agc_transient(&x, &states), settled);
                if let Some(a) = acquire_fixed(&adc, &cfg, &fx) {
                    acq += 1;
                    let want = lead + T1 - b;
                    sym += usize::from(a.fft_start.abs_diff(want) == 64);
                    if a.fft_start != want {
                        continue;
                    }
                    tim += 1;
                    let e = a.cfo_hz(&fx, FS) - cfo;
                    let ec = a.coarse_phase as f64 * unit - cfo;
                    let ef = a.fine_phase as f64 * unit - cfo;
                    se += e * e;
                    sc += ec * ec;
                    if ef.abs() > FS / 256.0 {
                        slips += 1;
                    } else {
                        sf += ef * ef;
                    }
                }
            }
            let rms = |s: f64, n: usize| (s / n.max(1) as f64).sqrt();
            println!(
                "{:<48} {acq:>8} {tim:>7} {sym:>6} {:>10.0} {:>10.0} {:>10.0} {slips:>6}",
                c.name,
                rms(se, tim),
                rms(sc, tim),
                rms(sf, tim - slips)
            );
        }
    }
}

/// Packets through the gain control model. The noise sits at `IDLE_DBFS` at the largest gain
/// (where the control idles); a packet `snr` dB above it arrives after 600 to 650 samples of
/// noise. Reported per point: when the gain last changed relative to the STF's first sample,
/// the packet's level after it, and the fixed receiver on the result (a phase jump uniform
/// in ±30° at each gain change: an assumption, as in `agc`).
fn agcmodel(frames: usize) {
    const IDLE_DBFS: f64 = -30.0;
    const JUMP_DEG: f64 = 30.0;
    let cfg = RxConfig::new(FS);
    let fx = FixedRxConfig::default();
    let frame = preamble();
    let p = mean_power(&frame);
    let full = f64::from(ADC_FULL_SCALE);
    let b = backoff_samples(&cfg);
    let reset = FastAttackAgc::ad9361_reset(2);
    let variants: [(&str, FastAttackAgc); 4] = [
        ("reset values (unlock by power)", reset),
        (
            "reset values, unlock on a large overload",
            FastAttackAgc {
                unlock_on_overload: true,
                ..reset
            },
        ),
        (
            "lock window 16, exit count 2, unlock on a large overload",
            FastAttackAgc {
                unlock_on_overload: true,
                lock_window: 16,
                exit_count: 2,
                ..reset
            },
        ),
        (
            "the same, stronger-signal threshold 6 dB",
            FastAttackAgc {
                unlock_on_overload: true,
                lock_window: 16,
                exit_count: 2,
                stronger_db: 6.0,
                ..reset
            },
        ),
    ];
    println!(
        "# rxfront agcmodel: channel::FastAttackAgc (AD9361 reset values at 2 detector clocks per\n\
         # sample unless stated), {frames} frames per point, noise at {IDLE_DBFS} dBFS at the largest\n\
         # gain, CFO uniform in ±235 kHz, RxFrontFixed behind a 12-bit ADC, phase jump ±{JUMP_DEG}° per\n\
         # gain change (assumed). Samples count from the STF's first (STF 0…159, LTF 160…319).\n\
         # moved = frames whose gain changed during the packet; last change: median / 95th\n\
         # percentile / latest, over those; ≤159 = frames whose gain is still from the end of the\n\
         # STF (unmoved frames count); level = the packet's RMS after the last change, dBFS,\n\
         # lowest / mean / highest; window = frames inside −15 … −3 dBFS; early = frames whose\n\
         # gain changed in the 400 samples of noise before the packet."
    );
    for (vi, (name, agc)) in variants.iter().enumerate() {
        println!(
            "\n## {name}\n#  SNR  moved   last change (med/95%/max)  ≤159   level dBFS (min/mean/max)  window  early  acquired  timing   fine Hz"
        );
        for (si, snr) in [5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0, 60.0]
            .into_iter()
            .enumerate()
        {
            let mut rng = Rng::new(0xa6c1_0000 + (vi * 100 + si) as u64);
            // Noise power p / 10^(snr/10) is IDLE_DBFS at the largest gain.
            let k = (10f64.powf((IDLE_DBFS - agc.max_gain_db + snr) / 10.0) / p).sqrt();
            let mut last: Vec<usize> = Vec::new();
            let (mut still, mut window, mut early) = (0usize, 0usize, 0usize);
            let (mut lo, mut hi, mut sum) = (f64::INFINITY, f64::NEG_INFINITY, 0.0);
            let (mut acq, mut tim) = (0usize, 0usize);
            let mut sf = 0.0f64;
            for _ in 0..frames {
                let lead = 600 + (rng.uniform() * 50.0) as usize;
                let mut x = vec![(0.0, 0.0); lead];
                x.extend_from_slice(&frame);
                let cfo = (2.0 * rng.uniform() - 1.0) * 235e3;
                let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                let x: Vec<C> = awgn(&x, snr, p, &mut rng)
                    .iter()
                    .map(|s| (s.0 * k, s.1 * k))
                    .collect();
                let t = agc.run(&x);
                early += usize::from(t.changes.iter().any(|c| c.0 + 400 >= lead && c.0 < lead));
                let at = t
                    .changes
                    .iter()
                    .rev()
                    .find(|c| c.0 >= lead)
                    .map(|c| c.0 - lead);
                if let Some(at) = at {
                    last.push(at);
                }
                still += usize::from(at.is_none_or(|a| a <= 159));
                let (settled, states) = t.gain_states(JUMP_DEG, &mut rng);
                let level = 10.0 * (p * k * k).log10() + settled;
                lo = lo.min(level);
                hi = hi.max(level);
                sum += level;
                window += usize::from((-15.0..=-3.0).contains(&level));
                let g = 10f64.powf(settled / 20.0);
                let at_settled: Vec<C> = x.iter().map(|s| (s.0 * g, s.1 * g)).collect();
                let adc = to_adc(&agc_transient(&at_settled, &states), full);
                if let Some(a) = acquire_fixed(&adc, &cfg, &fx) {
                    acq += 1;
                    if a.fft_start == lead + T1 - b {
                        tim += 1;
                        let e = a.cfo_hz(&fx, FS) - cfo;
                        sf += e * e;
                    }
                }
            }
            last.sort_unstable();
            let pick = |q: f64| {
                last.get(((last.len() as f64 - 1.0) * q).round() as usize)
                    .map_or("-".to_string(), |v| v.to_string())
            };
            println!(
                "  {snr:>4} {:>6} {:>12} {:>6} {:>6} {:>6} {:>10.1} {:>7.1} {:>7.1} {window:>7} {early:>6} {acq:>9} {tim:>7} {:>9.0}",
                last.len(),
                pick(0.5),
                pick(0.95),
                pick(1.0),
                still,
                lo,
                sum / frames as f64,
                hi,
                (sf / tim.max(1) as f64).sqrt()
            );
        }
    }
}

/// A Hamming-windowed sinc low-pass filter, cutoff in cycles per sample, unit DC gain.
fn lowpass(cutoff: f64, taps: usize) -> Vec<f64> {
    let mid = (taps - 1) as f64 / 2.0;
    let h: Vec<f64> = (0..taps)
        .map(|i| {
            let t = i as f64 - mid;
            let sinc = if t == 0.0 {
                2.0 * cutoff
            } else {
                (2.0 * PI * cutoff * t).sin() / (PI * t)
            };
            sinc * (0.54 - 0.46 * (2.0 * PI * i as f64 / (taps - 1) as f64).cos())
        })
        .collect();
    let sum: f64 = h.iter().sum();
    h.iter().map(|v| v / sum).collect()
}

fn fir(x: &[C], h: &[f64]) -> Vec<C> {
    (h.len()..x.len())
        .map(|n| {
            h.iter().enumerate().fold((0.0, 0.0), |a, (k, c)| {
                (a.0 + c * x[n - k].0, a.1 + c * x[n - k].1)
            })
        })
        .collect()
}

/// False alarms of the fixed receiver on band-limited noise at −30 dBFS: the white-noise law
/// (`tail`, `tailfx`) does not hold behind a channel filter. An excursion is a group of events
/// less than 1000 samples apart (events of one excursion are not independent).
fn tailbw(million: usize) {
    let cfg = RxConfig::new(FS);
    let fx = FixedRxConfig::default();
    let per_thread = million.div_ceil(threads()).max(1);
    println!(
        "# rxfront tailbw: RxFrontFixed on noise at −30 dBFS behind a 65-tap low-pass filter,\n\
         # {} M samples per case ({} threads x {per_thread} M; the streams depend on the thread count).\n\
         # Budget: 5e-8 detector events per sample (1 per second at 20 MS/s).\n\
         # The 802.11 signal occupies ±8.3 MHz: the two narrowest filters cut into it and are not\n\
         # usable channel filters; they show the trend.\n\
         # passband        detector events  excursions   per sample   acquisitions",
        per_thread * threads(),
        threads()
    );
    for (ci, (name, cutoff)) in [
        ("white", 0.0),
        ("±9.0 MHz", 0.45),
        ("±8.3 MHz", 0.415),
        ("±7.5 MHz (cuts)", 0.375),
        ("±6.0 MHz (cuts)", 0.30),
    ]
    .into_iter()
    .enumerate()
    {
        let h = lowpass(cutoff, 65);
        let r = par(|t| {
            let mut rng = Rng::new(0xfa00_0000 + ((ci as u64) << 20) + t as u64);
            let (mut d, mut e, mut a, mut n) = (0usize, 0usize, 0usize, 0usize);
            for _ in 0..per_thread {
                let x = noise(1_000_000, &mut rng);
                let x = if cutoff > 0.0 { fir(&x, &h) } else { x };
                // Unit power before the filter; the level is set on what the ADC sees.
                let gain =
                    f64::from(ADC_FULL_SCALE) * 10f64.powf(-30.0 / 20.0) / mean_power(&x).sqrt();
                let adc = to_adc(&x, gain);
                let det = detections_fixed(&adc, &cfg, &fx);
                d += det.len();
                e += det
                    .iter()
                    .zip(std::iter::once(&usize::MAX).chain(det.iter()))
                    .filter(|(now, before)| **before == usize::MAX || **now - **before >= 1000)
                    .count();
                a += acquisitions_fixed(&adc, &cfg, &fx).len();
                n += adc.len();
            }
            (d, e, a, n)
        });
        let (d, e, a, n) = r.iter().fold((0, 0, 0, 0), |s, v| {
            (s.0 + v.0, s.1 + v.1, s.2 + v.2, s.3 + v.3)
        });
        println!(
            "{name:<16} {d:>16} {e:>11}   {:>10.1e}   {a:>12}",
            d as f64 / n as f64
        );
    }
}

/// Float and fixed receiver on the same ADC samples against the packet's level in the ADC.
fn level(frames: usize) {
    let cfg = RxConfig::new(FS);
    let fx = FixedRxConfig::default();
    let frame = preamble();
    let p = mean_power(&frame);
    let b = backoff_samples(&cfg);
    println!(
        "# rxfront level: {frames} frames per point, CFO uniform in ±235 kHz, random fractional delay;\n\
         # both receivers on the same 12-bit ADC samples. timing = FFT start within ±1 sample.\n\
         # SNR  level     float: acquired timing CFO RMS   fixed: acquired timing CFO RMS   fixed/float"
    );
    for snr in [4.0, 10.0, 25.0] {
        for lvl in [-3.0, -9.0, -15.0, -30.0, -45.0, -55.0, -60.0] {
            let mut rng = Rng::new(0x1e7e1);
            let (mut af, mut ax, mut tf, mut tx) = (0usize, 0usize, 0usize, 0usize);
            let (mut sf, mut sx) = (0.0f64, 0.0f64);
            for _ in 0..frames {
                let lead = 600;
                let mut x = vec![(0.0, 0.0); lead];
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 300]);
                let x = fractional_delay(&x, rng.uniform());
                let cfo = (2.0 * rng.uniform() - 1.0) * 235e3;
                let x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
                let x = awgn(&x, snr, p, &mut rng);
                let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(lvl / 20.0) / p.sqrt();
                let adc = to_adc(&x, gain);
                let xf: Vec<C> = adc
                    .iter()
                    .map(|s| (f64::from(s.0), f64::from(s.1)))
                    .collect();
                let ok = |start: usize| start.abs_diff(lead + T1 - b) <= 1;
                if let Some(a) = acquire(&xf, 0, &cfg) {
                    af += 1;
                    tf += usize::from(ok(a.fft_start));
                    sf += (a.cfo_hz - cfo).powi(2);
                }
                if let Some(a) = acquire_fixed(&adc, &cfg, &fx) {
                    ax += 1;
                    tx += usize::from(ok(a.fft_start));
                    sx += (a.cfo_hz(&fx, FS) - cfo).powi(2);
                }
            }
            let (rf, rx) = (
                (sf / af.max(1) as f64).sqrt(),
                (sx / ax.max(1) as f64).sqrt(),
            );
            println!(
                "{snr:>5} {lvl:>6} dBFS {af:>12} {tf:>6} {rf:>7.0} Hz {ax:>12} {tx:>6} {rx:>7.0} Hz   {:>6.2}",
                rx / rf.max(1e-9)
            );
        }
    }
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
        "g2" => g2(n.unwrap_or(300)),
        "tailfx" => tailfx(n.unwrap_or(100)),
        "g2joint" => g2joint(n.unwrap_or(3000)),
        "latency" => latency(n.unwrap_or(500)),
        "agc" => agc(n.unwrap_or(500)),
        "tailbw" => tailbw(n.unwrap_or(400)),
        "agcmodel" => agcmodel(n.unwrap_or(400)),
        "level" => level(n.unwrap_or(400)),
        _ => {
            eprintln!("usage: rxfront tail|pd|robust|frames|g2|g2joint|tailfx|latency|agc|agcmodel|tailbw|level [n]");
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
