//! G1 gate for the receiver front: an outside receiver as a black box (R0-08a, reference 3 of
//! `docs/gates/rx_front.md`).
//!
//! `tests/fixtures/rx_front_openofdm.txt` holds what OpenOFDM's Python decoder (Apache-2.0,
//! commit ea638eb, run unmodified; `tools/ref/rx_front_openofdm.py`) made of 800 impaired
//! 6 Mb/s frames from `rxfront frames 200`: the packet start, its coarse frequency offset, the
//! SIGNAL field and the PSDU. This test rebuilds the same frames (same recipe, checked by the
//! FNV hash the generator printed), runs our front end + the shared 6 Mb/s decoder on exactly
//! the same int16 samples, and requires, per SNR: SIGNAL and PSDU recovered at least as often
//! as OpenOFDM (SIGNAL needs good parity on both sides), and a CFO error no larger — a low
//! bar, since OpenOFDM's fine estimate is disabled in that commit (coarse only), and the RMS is
//! over the frames each receiver located. OpenOFDM's absolute power trigger gets noise only
//! over the frame, a genie gate in its favour — and the same input goes to ours.

#![allow(clippy::unwrap_used)]

mod common;
use common::decode_6mbps;

use radio_phy::channel::{awgn, fractional_delay, frequency_offset, mean_power, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, correct, RxConfig};
use std::collections::BTreeMap;
use std::f64::consts::PI;

type C = (f64, f64);

const FIXTURE: &str = include_str!("fixtures/rx_front_openofdm.txt");
const FS: f64 = 20e6;
const SNRS: [f64; 4] = [2.0, 4.0, 6.0, 9.0];
const LEAD: usize = 437;
const TRAIL: usize = 400;
const SCALE: f64 = 20000.0;

/// The `rxfront frames` recipe (crates/radio-tools/src/bin/rxfront.rs, `bb_frame`).
fn frame(k: usize, per_snr: usize) -> (f64, f64, f64, Vec<u8>, Vec<(i16, i16)>) {
    let snr = SNRS[(k / per_snr).min(SNRS.len() - 1)];
    let mut rng = Rng::new(0x0fd0_0000 + k as u64);
    let psdu: Vec<u8> = (0..20).map(|_| (rng.next_u64() & 0xff) as u8).collect();
    let seed = 1 + (rng.next_u64() % 127) as u8;
    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
    let delay = rng.uniform();
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let f: Vec<C> = build_ppdu(&psdu, Rate::R6, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect();
    let mut x = vec![(0.0, 0.0); LEAD];
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); TRAIL]);
    let x = fractional_delay(&x, delay);
    let mut x = frequency_offset(&x, cfo, FS, 2.0 * PI * rng.uniform());
    let span = LEAD..LEAD + f.len() + 32;
    let noisy = awgn(&x[span.clone()], snr, mean_power(&f), &mut rng);
    x[span].copy_from_slice(&noisy);
    let q = |v: f64| (v * SCALE).round().clamp(-32768.0, 32767.0) as i16;
    (
        snr,
        cfo,
        delay,
        psdu,
        x.iter().map(|s| (q(s.0), q(s.1))).collect(),
    )
}

#[derive(Default)]
struct Tally {
    frames: usize,
    signal: usize,
    psdu: usize,
    cfo_err2: Vec<f64>,
}

impl Tally {
    fn cfo_rms(&self) -> f64 {
        (self.cfo_err2.iter().sum::<f64>() / self.cfo_err2.len().max(1) as f64).sqrt()
    }
}

#[test]
fn front_end_is_no_worse_than_openofdm() {
    let header = FIXTURE
        .lines()
        .find(|l| l.starts_with("# frames "))
        .expect("fixture names the frame set");
    let h: Vec<&str> = header.split_whitespace().collect();
    let (count, per_snr): (usize, usize) = (h[2].parse().unwrap(), h[4].parse().unwrap());
    let want_fnv = u64::from_str_radix(h[6], 16).unwrap();

    let theirs_lines: BTreeMap<usize, &str> = FIXTURE
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .map(|l| (l.split_whitespace().next().unwrap().parse().unwrap(), l))
        .collect();
    assert_eq!(theirs_lines.len(), count, "one verdict per frame");

    let cfg = RxConfig::new(FS);
    let mut ours: BTreeMap<u64, Tally> = BTreeMap::new();
    let mut theirs: BTreeMap<u64, Tally> = BTreeMap::new();
    let mut fnv: u64 = 0xcbf2_9ce4_8422_2325;
    for k in 0..count {
        let (snr, cfo, delay, psdu, iq) = frame(k, per_snr);
        for (i, q) in &iq {
            for b in i.to_le_bytes().iter().chain(q.to_le_bytes().iter()) {
                fnv ^= u64::from(*b);
                fnv = fnv.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        let key = snr as u64;
        let first_path = LEAD as f64 + delay;

        // OpenOFDM's verdict: "k | start S phase P signal R L PAR | psdu HEX".
        let f: Vec<&str> = theirs_lines[&k].split_whitespace().collect();
        // A SIGNAL it never decoded is a single "none".
        let at = |name: &str| f.iter().position(|w| *w == name).unwrap();
        let (sig, ps) = (at("signal"), at("psdu"));
        let t = theirs.entry(key).or_default();
        t.frames += 1;
        if f[sig + 1..ps - 1] == ["6", "20", "1"] {
            t.signal += 1;
        }
        let want_hex: String = psdu.iter().map(|b| format!("{b:02x}")).collect();
        if f[ps + 1] == want_hex {
            t.psdu += 1;
        }
        if let (Ok(start), Ok(phase)) = (f[3].parse::<f64>(), f[5].parse::<f64>()) {
            // Only frames it located (start within the guard of the truth) count for CFO.
            if (start - first_path).abs() <= 16.0 {
                let est = -phase * FS / (2.0 * PI);
                t.cfo_err2.push((est - cfo).powi(2));
            }
        }

        // Ours, on the same int16 samples.
        let x: Vec<C> = iq
            .iter()
            .map(|&(i, q)| (f64::from(i) / SCALE, f64::from(q) / SCALE))
            .collect();
        let o = ours.entry(key).or_default();
        o.frames += 1;
        if let Some(acq) = acquire(&x, 0, &cfg) {
            if (acq.ltf_start as f64 + 32.0 - (first_path + 192.0)).abs() <= 16.0 {
                o.cfo_err2.push((acq.cfo_hz - cfo).powi(2));
            }
            let y = correct(&x, acq.cfo_hz, FS);
            if let Some((rate, len, got)) = decode_6mbps(&y, acq.fft_start) {
                if rate == Rate::R6.signal_rate_bits() && len == 20 {
                    o.signal += 1;
                }
                if got == psdu {
                    o.psdu += 1;
                }
            }
        }
    }
    assert_eq!(fnv, want_fnv, "the frames differ from those OpenOFDM saw");

    for (snr, t) in &theirs {
        let o = &ours[snr];
        let n = t.frames as f64;
        println!(
            "{snr} dB: SIGNAL ours {:.3} OpenOFDM {:.3} | PSDU ours {:.3} OpenOFDM {:.3} | \
             CFO rms ours {:.0} Hz ({} frames), OpenOFDM coarse-only {:.0} Hz ({} frames)",
            o.signal as f64 / n,
            t.signal as f64 / n,
            o.psdu as f64 / n,
            t.psdu as f64 / n,
            o.cfo_rms(),
            o.cfo_err2.len(),
            t.cfo_rms(),
            t.cfo_err2.len()
        );
        assert!(o.signal >= t.signal, "{snr} dB: SIGNAL");
        assert!(o.psdu >= t.psdu, "{snr} dB: PSDU");
        assert!(o.cfo_rms() <= t.cfo_rms(), "{snr} dB: CFO");
    }
}
