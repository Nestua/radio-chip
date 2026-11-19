//! `rcap` — IQ capture files (SigMF) for the SDR and field stages (R0-13, `docs/gates/capture.md`).
//!
//! Usage: `cargo run -q --release -p radio-tools --bin rcap -- <command> <file.sigmf-meta> [args]`
//! - `info`: the metadata, the number of samples and the duration.
//! - `verify`: reads the dataset through and checks its length and SHA-512.
//! - `synth [frames] [snr dB]`: writes a simulated capture (6 Mb/s frames through the channel
//!   simulator into the 12-bit ADC, `ci16_le`), with an annotation at each frame: a known
//!   capture for testing whatever reads captures.
//! - `replay [shift]`: runs the fixed-point receiver front over a `ci16_le` capture and lists
//!   its acquisitions. `shift` (default 0) drops that many low bits first: an SDR's 16-bit
//!   samples to the model's 12-bit ADC is 4.

use std::path::Path;
use std::process::ExitCode;

use radio_capture::sigmf::{Annotation, Datatype, Meta, Reader, Recording};
use radio_phy::channel::{awgn, fractional_delay, frequency_offset, mean_power, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{to_adc, FixedRxConfig, RxFrontFixed, ADC_FULL_SCALE};

const FS: f64 = 20e6;

fn info(path: &Path) -> Result<(), String> {
    let reader = Reader::open(path).map_err(|e| e.to_string())?;
    let m = &reader.meta;
    let data = radio_capture::sigmf::data_path(path).map_err(|e| e.to_string())?;
    let bytes = std::fs::metadata(data).map_err(|e| e.to_string())?.len();
    let samples = bytes / m.datatype.sample_bytes() as u64;
    println!("datatype     {}", m.datatype.name());
    println!("sample rate  {} Hz", m.sample_rate_hz);
    println!(
        "samples      {samples} ({:.6} s)",
        samples as f64 / m.sample_rate_hz
    );
    println!("version      {}", m.version);
    for (name, v) in [
        ("description", &m.description),
        ("hw", &m.hw),
        ("author", &m.author),
    ] {
        if let Some(v) = v {
            println!("{name:<12} {v}");
        }
    }
    println!("sha512       {}", m.sha512.as_deref().unwrap_or("(none)"));
    for s in &m.captures {
        println!(
            "capture      from sample {}: {} Hz, {}",
            s.sample_start,
            s.frequency_hz.map_or("?".into(), |f| f.to_string()),
            s.datetime.as_deref().unwrap_or("no time")
        );
    }
    println!("annotations  {}", m.annotations.len());
    Ok(())
}

fn verify(path: &Path) -> Result<(), String> {
    let mut reader = Reader::open(path).map_err(|e| e.to_string())?;
    let mut samples = 0usize;
    loop {
        let chunk = reader.next_chunk(1 << 16).map_err(|e| e.to_string())?;
        if chunk.is_empty() {
            break;
        }
        samples += chunk.len();
    }
    let checked = if reader.meta.sha512.is_some() {
        "SHA-512 matches"
    } else {
        "no core:sha512 to check"
    };
    println!("{samples} samples, whole; {checked}");
    Ok(())
}

fn synth(path: &Path, frames: usize, snr_db: f64) -> Result<(), String> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let mut rng = Rng::new(0xca97_0001);
    let mut x: Vec<(f64, f64)> = Vec::new();
    let mut meta = Meta::new(Datatype::Ci16Le, FS, Some(5.805e9));
    meta.description = Some(format!(
        "rcap synth: {frames} simulated 6 Mb/s frames, SNR {snr_db} dB, packets at -15 dBFS of a 12-bit ADC"
    ));
    meta.hw = Some("none (radio_phy channel simulator)".into());
    let mut power = 1.0;
    for k in 0..frames {
        let psdu: Vec<u8> = (0..40 + 7 * k).map(|i| (i * 29 + k) as u8).collect();
        let frame: Vec<(f64, f64)> = build_ppdu(&psdu, Rate::R6, 0x5d, &cfg)
            .map_err(|e| e.to_string())?
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        power = mean_power(&frame);
        let lead = 400 + (rng.uniform() * 300.0) as usize;
        let mut burst = vec![(0.0, 0.0); lead];
        burst.extend_from_slice(&frame);
        let burst = fractional_delay(&burst, rng.uniform());
        let cfo = (rng.uniform() * 2.0 - 1.0) * 100e3;
        meta.annotations.push(Annotation {
            sample_start: (x.len() + lead) as u64,
            sample_count: Some(frame.len() as u64),
            label: Some("ppdu".into()),
            comment: Some(format!("6 Mb/s, {} octets, CFO {cfo:.0} Hz", psdu.len())),
            other: Vec::new(),
        });
        x.extend(frequency_offset(
            &burst,
            cfo,
            FS,
            rng.uniform() * std::f64::consts::TAU,
        ));
    }
    x.extend(vec![(0.0, 0.0); 400]);
    let x = awgn(&x, snr_db, power, &mut rng);
    let rms = (power * (1.0 + 10f64.powf(-snr_db / 10.0))).sqrt();
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / rms;
    let rec = Recording::from_i16(meta, &to_adc(&x, gain));
    rec.write(path).map_err(|e| e.to_string())?;
    println!(
        "{} samples, {frames} frames written to {}",
        rec.len(),
        path.display()
    );
    Ok(())
}

fn replay(path: &Path, shift: u32) -> Result<(), String> {
    let mut reader = Reader::open(path).map_err(|e| e.to_string())?;
    if reader.meta.datatype != Datatype::Ci16Le {
        return Err(format!(
            "replay reads ci16_le, not {}",
            reader.meta.datatype.name()
        ));
    }
    let fx = FixedRxConfig::default();
    let rate = reader.meta.sample_rate_hz;
    let mut rx = RxFrontFixed::new(RxConfig::new(rate), fx);
    let limit = f64::from(ADC_FULL_SCALE);
    let (mut n, mut found, mut clipped) = (0usize, 0usize, 0usize);
    println!("# decided_at  fft_start  CFO Hz   (sample rate {rate} Hz, {shift} low bits dropped)");
    loop {
        let chunk = reader.next_chunk(1 << 16).map_err(|e| e.to_string())?;
        if chunk.is_empty() {
            break;
        }
        for (i, q) in chunk {
            let s = |v: f64| {
                let v = (v / f64::from(1u32 << shift)).floor();
                (
                    v.clamp(-limit, limit - 1.0) as i16,
                    v < -limit || v >= limit,
                )
            };
            let ((i, ci), (q, cq)) = (s(i), s(q));
            clipped += usize::from(ci || cq);
            if let Some(a) = rx.push((i, q)) {
                found += 1;
                println!(
                    "{:>12} {:>10} {:>8.0}",
                    a.decided_at,
                    a.fft_start,
                    a.cfo_hz(&fx, rate)
                );
            }
            n += 1;
        }
    }
    println!("# {n} samples, {found} acquisitions, {clipped} samples beyond the 12-bit ADC");
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(cmd), Some(file)) = (args.first(), args.get(1)) else {
        eprintln!("usage: rcap info|verify|synth|replay <file.sigmf-meta> [args]");
        return ExitCode::from(2);
    };
    let path = Path::new(file);
    // An argument that is given must be a number in range: no silent defaults.
    let arg = |k: usize, name: &str, default: f64, range: std::ops::RangeInclusive<f64>| match args
        .get(k)
    {
        None => Ok(default),
        Some(t) => t
            .parse::<f64>()
            .ok()
            .filter(|v| range.contains(v) && v.fract() == 0.0 || name == "snr" && range.contains(v))
            .ok_or(format!("{name} must be a number in {range:?}, not {t:?}")),
    };
    let result = match cmd.as_str() {
        "info" => info(path),
        "verify" => verify(path),
        "synth" => arg(2, "frames", 3.0, 1.0..=10_000.0).and_then(|frames| {
            arg(3, "snr", 20.0, -20.0..=100.0).and_then(|snr| synth(path, frames as usize, snr))
        }),
        "replay" => arg(2, "shift", 0.0, 0.0..=15.0).and_then(|shift| replay(path, shift as u32)),
        _ => Err("unknown command".into()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rcap {cmd}: {e}");
            ExitCode::FAILURE
        }
    }
}
