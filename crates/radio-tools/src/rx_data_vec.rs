//! `rvec rx_data`: the data path's top level's vectors (`hw/rtl/rx_data.sv`,
//! `docs/gates/rx_data.md`), from `radio_phy::rx_data::RxDataSchedule`: the hand-off's streams
//! (the front's model, `RxHandoffFixed`, on frames through a channel), every beat's take cycle
//! and every event's cycle at the block's pins.
//!
//! A run: the block reset, then cycle 0, 1, … Beat i is offered (valid) from cycle
//! max(its `avail`, the cycle after beat i − 1 was taken) and held until taken; the schedule's
//! reader takes it in the cycle the vectors give. Events are what the block's outputs show in
//! the cycle given; no other cycle shows one.

use std::f64::consts::PI;
use std::io::{self, Write};

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_data::{DataTiming, RxDataSchedule};
use radio_phy::rx_data_fixed::{RxDataEnd, RxDataEvent, RxDataFixed};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    to_adc, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd, HandoffStart, RxHandoffFixed,
    ADC_FULL_SCALE,
};
use radio_phy::rx_ofdm::{RxOfdmConfig, RxOfdmError};
use radio_phy::rx_ofdm_fixed::{clock_ratio, FixedOfdmConfig};

const FS: f64 = 20e6;
const CARRIER_HZ: f64 = 5.8e9;
/// The CFO word's bits (rx_front_rom_pkg's NCO_PHASE_BITS).
const CFO_BITS: u32 = 22;
const Y_BITS: u32 = 13;

const RATES: [Rate; 8] = [
    Rate::R6,
    Rate::R9,
    Rate::R12,
    Rate::R18,
    Rate::R24,
    Rate::R36,
    Rate::R48,
    Rate::R54,
];

fn rate_code(r: Rate) -> usize {
    RATES.iter().position(|&x| x == r).unwrap_or(0)
}

fn frame(psdu: &[u8], rate: Rate, seed: u8) -> io::Result<Vec<(f64, f64)>> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    Ok(build_ppdu(psdu, rate, seed, &cfg)
        .map_err(|e| io::Error::other(format!("{e:?}")))?
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect())
}

/// One frame (rate, octets, SNR in dB, clock offset in ppm) after 400 samples of silence, at
/// −15 dBFS, through the ADC.
fn adc(rate: Rate, octets: usize, snr: f64, ppm: f64, seed: u64) -> io::Result<Vec<(i16, i16)>> {
    let mut rng = Rng::new(seed);
    let psdu: Vec<u8> = (0..octets).map(|_| (rng.next_u64() & 0xff) as u8).collect();
    let f = frame(&psdu, rate, 1 + (rng.next_u64() % 127) as u8)?;
    let p = mean_power(&f);
    let mut x = vec![(0.0, 0.0); 400];
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); 600]);
    let x = if ppm == 0.0 { x } else { resample(&x, ppm) };
    let x = frequency_offset(&x, ppm * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
    let x = awgn(&x, snr, p, &mut rng);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p.sqrt();
    Ok(to_adc(&x, gain))
}

/// The first packet's beats and its mark, as the hand-off hands them on with no limit of pace.
/// Every stream ends with a mark, as the hand-off's always does.
fn first_stream(adc: &[(i16, i16)]) -> Vec<HandoffBeat> {
    let free = HandoffConfig {
        rate: usize::MAX,
        ..HandoffConfig::default()
    };
    let mut rx = RxHandoffFixed::new(RxConfig::new(FS), FixedRxConfig::default(), free);
    let mut beats = Vec::new();
    for &x in adc {
        for b in rx.push(x, usize::MAX).beats {
            let end = matches!(b, HandoffBeat::End(_));
            beats.push(b);
            if end {
                return beats;
            }
        }
    }
    // The ADC's samples ran out before the hand-off's end mark (it reads on to the frame's
    // longest end): the mark it would give.
    beats.push(HandoffBeat::End(HandoffEnd::Packet));
    beats
}

/// A stream of noise with a first beat: SIGNAL fails; then the mark `end`.
fn noise_stream(rng: &mut Rng, n: usize, tag: u8, end: HandoffEnd) -> Vec<HandoffBeat> {
    let mut v: Vec<HandoffBeat> = (0..n)
        .map(|i| HandoffBeat::Sample {
            z: (
                (rng.next_u64() % 1024) as i64 - 512,
                (rng.next_u64() % 1024) as i64 - 512,
            ),
            first: (i == 0).then_some(HandoffStart {
                cfo_phase: 0,
                packet: tag,
            }),
        })
        .collect();
    v.push(HandoffBeat::End(end));
    v
}

/// A noise stream whose SIGNAL fails one of its checks (parity, rate or length), by the model.
fn failing_noise(rng: &mut Rng, tag: u8) -> io::Result<Vec<HandoffBeat>> {
    for _ in 0..64 {
        let beats = noise_stream(rng, 600, tag, HandoffEnd::Stopped);
        let mut m = RxDataFixed::new(
            RxOfdmConfig::default(),
            FixedOfdmConfig::default(),
            HandoffConfig::default(),
            clock_ratio(FS, CARRIER_HZ),
        )
        .ok_or_else(|| io::Error::other("model"))?;
        let fails = beats.iter().any(|b| {
            m.push(*b)
                .iter()
                .any(|e| matches!(e, RxDataEvent::End(RxDataEnd::Signal(x)) if why(*x) != 0))
        });
        if fails {
            return Ok(beats);
        }
    }
    Err(io::Error::other("no failing noise in 64 tries"))
}

fn hex(v: i64, bits: u32) -> String {
    format!("{:x}", (v as u64) & ((1u64 << bits) - 1))
}

fn kind(e: HandoffEnd) -> usize {
    match e {
        HandoffEnd::Packet => 0,
        HandoffEnd::Stopped => 1,
        HandoffEnd::Acquisition => 2,
        HandoffEnd::Overrun => 3,
    }
}

fn why(e: RxOfdmError) -> usize {
    match e {
        RxOfdmError::SignalParity => 1,
        RxOfdmError::SignalRate(_) => 2,
        RxOfdmError::SignalLength => 3,
        _ => 0,
    }
}

/// Counts for the generator's own checks.
#[derive(Debug, Default)]
pub struct Cover {
    runs: usize,
    beats: usize,
    ends: [usize; 5],
    octets: usize,
    waits: usize,
    /// Windows (SIGNAL's and data) placed early and late by the tracker.
    early: usize,
    late: usize,
    /// Beats taken in the cycle of a failed SIGNAL's end (the drop's own cycle).
    drop_takes: usize,
}

/// One run: the beats with their availability, through a fresh schedule; written out.
fn run(
    out: &mut impl Write,
    cover: &mut Cover,
    label: &str,
    ratio: Option<u16>,
    beats: &[HandoffBeat],
    pace: usize,
    offset: usize,
) -> io::Result<()> {
    let mut s = RxDataSchedule::new(
        RxOfdmConfig::default(),
        FixedOfdmConfig::default(),
        HandoffConfig::default(),
        ratio,
        DataTiming::default(),
    )
    .ok_or_else(|| io::Error::other("schedule"))?;
    writeln!(out, "R {label} {}", ratio.unwrap_or(0))?;
    let mut clock = 0usize;
    let mut lines = Vec::with_capacity(beats.len());
    for (i, b) in beats.iter().enumerate() {
        let avail = pace * i + offset;
        clock = clock.max(avail);
        let from = clock;
        while !s.ready_at(clock) {
            clock += 1;
            if clock > from + 1_000_000 {
                return Err(io::Error::other(format!("{label}: stalled at beat {i}")));
            }
        }
        if clock > from {
            cover.waits += 1;
        }
        s.take(clock, *b);
        let what = match *b {
            HandoffBeat::Sample { z, first: None } => {
                format!("S {} {}", hex(z.0, Y_BITS), hex(z.1, Y_BITS))
            }
            HandoffBeat::Sample { z, first: Some(f) } => format!(
                "F {} {} {} {:x}",
                hex(z.0, Y_BITS),
                hex(z.1, Y_BITS),
                hex(f.cfo_phase, CFO_BITS),
                f.packet
            ),
            HandoffBeat::End(e) => format!("M {}", kind(e)),
        };
        lines.push(format!("B {avail} {clock} {what}"));
        clock += 1;
    }
    // Which beat each window started on, against its nominal place in its packet.
    let mut first_beat = 0usize;
    let mut beat_of = std::collections::HashMap::new();
    let mut taken_at = Vec::with_capacity(beats.len());
    for l in &lines {
        let f: Vec<&str> = l.split_whitespace().collect();
        taken_at.push(f[2].parse::<usize>().unwrap_or(0));
    }
    for (i, b) in beats.iter().enumerate() {
        if matches!(b, HandoffBeat::Sample { first: Some(_), .. }) {
            first_beat = i;
        }
        beat_of.insert(taken_at[i], i - first_beat);
    }
    for w in s.windows() {
        if let radio_phy::rx_data::Window::Symbol(k) = w.window {
            match beat_of.get(&w.first).map(|&b| b.cmp(&(144 + 80 * k))) {
                Some(std::cmp::Ordering::Less) => cover.early += 1,
                Some(std::cmp::Ordering::Greater) => cover.late += 1,
                _ => {}
            }
        }
    }
    let takes: std::collections::HashSet<usize> = taken_at.iter().copied().collect();
    let hits = s
        .events()
        .iter()
        .filter(|(c, e)| matches!(e, RxDataEvent::End(RxDataEnd::Signal(_))) && takes.contains(c))
        .count();
    cover.drop_takes += hits;
    if !s.violations().is_empty() {
        return Err(io::Error::other(format!(
            "{label}: violations {:?}",
            s.violations()
        )));
    }
    for l in lines {
        writeln!(out, "{l}")?;
    }
    for &(c, e) in s.events() {
        match e {
            RxDataEvent::Verdict {
                packet, verdict, ..
            } => {
                let (fail, sym, pat) =
                    verdict.map_or((1, 0, 0), |v| (0, v.data_symbols, usize::from(v.patient)));
                writeln!(out, "V {c} {packet:x} {fail} {sym} {pat}")?;
            }
            RxDataEvent::Start { signal, .. } => {
                writeln!(out, "T {c} {} {}", rate_code(signal.rate), signal.length)?;
            }
            RxDataEvent::Octet { octet, first } => {
                cover.octets += 1;
                let (f, seed, service) = first.map_or((0, 0, 0), |f| (1, f.seed, f.service));
                // oct_last: the frame's last octet comes with its Decoded end (checked by
                // the bench against the end in the same cycle).
                writeln!(out, "O {c} {octet:02x} {f} {seed:02x} {service:04x}")?;
            }
            RxDataEvent::End(end) => {
                let (status, info) = match end {
                    RxDataEnd::Decoded => (0, 0),
                    RxDataEnd::Seed => (1, 0),
                    RxDataEnd::Signal(e) => (2, why(e)),
                    RxDataEnd::Truncated => (3, 0),
                    RxDataEnd::Aborted(k) => (4, kind(k)),
                };
                cover.ends[status] += 1;
                writeln!(out, "E {c} {status} {info}")?;
            }
        }
    }
    let last = s.last_event().max(clock) + 100;
    writeln!(out, "Z {last}")?;
    cover.runs += 1;
    cover.beats += beats.len();
    Ok(())
}

/// The vectors; returns what they cover.
pub fn vectors(out: &mut impl Write) -> io::Result<Cover> {
    writeln!(
        out,
        "# rvec rx_data — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(out, "# R <label> <ratio>: reset, a run from cycle 0")?;
    writeln!(
        out,
        "# B <avail> <taken> S <re> <im> | F <re> <im> <cfo> <tag> | M <kind>: a beat (hex values)"
    )?;
    writeln!(
        out,
        "# V <cycle> <tag> <fail> <symbols> <patient>; T <cycle> <rate> <length>;"
    )?;
    writeln!(
        out,
        "# O <cycle> <octet> <first> <seed> <service>; E <cycle> <status> <info>; Z <last cycle>"
    )?;
    let mut c = Cover::default();
    let ratio = clock_ratio(FS, CARRIER_HZ);
    let mut seed = 1u64;
    let mut stream = |rate, octets, snr, ppm| -> io::Result<Vec<HandoffBeat>> {
        seed += 1;
        Ok(first_stream(&adc(rate, octets, snr, ppm, seed)?))
    };

    // ADR 0010's table: one data symbol at 54 Mb/s, a beat every cycle.
    let b = stream(Rate::R54, 24, 30.0, 0.0)?;
    run(out, &mut c, "table", ratio, &b, 0, 0)?;
    // Every rate: short and longer frames a beat a cycle, one at the antenna's pace.
    for rate in RATES {
        let r = rate_code(rate);
        for octets in [1usize, 100] {
            let b = stream(rate, octets, 30.0, 0.0)?;
            run(out, &mut c, &format!("r{r}-{octets}"), ratio, &b, 0, 0)?;
        }
        let b = stream(rate, 24, 30.0, 20.0)?;
        run(out, &mut c, &format!("r{r}-24-pace8"), ratio, &b, 8, 0)?;
    }
    // Long frames with a clock offset either way: the windows move early and late.
    for (rate, ppm) in [
        (Rate::R6, 40.0),
        (Rate::R6, -40.0),
        (Rate::R54, -40.0),
        (Rate::R24, 40.0),
    ] {
        let b = stream(rate, 1500, 30.0, ppm)?;
        let label = format!(
            "long-r{}{}",
            rate_code(rate),
            if ppm < 0.0 { "-neg" } else { "" }
        );
        run(out, &mut c, &label, ratio, &b, 0, 0)?;
    }
    // No ratio written.
    let b = stream(Rate::R12, 50, 30.0, 0.0)?;
    run(out, &mut c, "no-ratio", None, &b, 0, 0)?;
    // The CFO word forged far from the frame's: SIGNAL's θ moves its window, either way.
    for (label, word) in [("cfo-high", (1i64 << 21) - 1), ("cfo-low", -(1i64 << 21))] {
        let mut b = stream(Rate::R18, 40, 30.0, 0.0)?;
        if let Some(HandoffBeat::Sample { first: Some(f), .. }) = b.first_mut() {
            f.cfo_phase = word;
        }
        run(out, &mut c, label, ratio, &b, 0, 0)?;
    }

    // Marks, each followed by a frame that decodes.
    let a = stream(Rate::R6, 300, 30.0, 0.0)?;
    let next = stream(Rate::R54, 100, 30.0, 0.0)?;
    for (label, cut, mark, pace) in [
        ("cut-data", 144 + 80 * 6 + 64, HandoffEnd::Acquisition, 0),
        ("cut-before-verdict", 218, HandoffEnd::Acquisition, 0),
        ("cut-in-signal", 180, HandoffEnd::Overrun, 0),
        ("cut-in-t2", 100, HandoffEnd::Acquisition, 0),
        ("cut-data-packet", 144 + 80 * 3 + 30, HandoffEnd::Packet, 0),
        ("cut-data-pace8", 144 + 80 * 4 + 64, HandoffEnd::Overrun, 8),
        ("cut-before-verdict-pace8", 230, HandoffEnd::Acquisition, 8),
    ] {
        let mut beats: Vec<HandoffBeat> = a[..cut].to_vec();
        beats.push(HandoffBeat::End(mark));
        beats.extend_from_slice(&next);
        run(out, &mut c, label, ratio, &beats, pace, 0)?;
    }
    // A mark between the frame's last window and its end: no end by the mark, the frame's
    // own end. Before the verdict (one symbol, a beat a cycle) and after it.
    for (label, rate, octets, cut, pace) in [
        (
            "mark-after-last-before-verdict",
            Rate::R54,
            24,
            144 + 80 + 64 + 5,
            0,
        ),
        (
            "mark-after-last-after-verdict",
            Rate::R54,
            100,
            144 + 80 * 4 + 64 + 2,
            0,
        ),
        (
            "mark-after-last-pace8",
            Rate::R12,
            30,
            144 + 80 * 3 + 64 + 1,
            8,
        ),
    ] {
        let mut beats: Vec<HandoffBeat> = stream(rate, octets, 30.0, 0.0)?[..cut].to_vec();
        beats.push(HandoffBeat::End(HandoffEnd::Packet));
        beats.extend_from_slice(&next);
        run(out, &mut c, label, ratio, &beats, pace, 0)?;
    }
    // A failed SIGNAL (noise), stopped; then a frame.
    let mut rng = Rng::new(0xda7a);
    for pace in [0usize, 8] {
        let mut beats = noise_stream(&mut rng, 600, 7, HandoffEnd::Stopped);
        beats.extend_from_slice(&next);
        let label = format!("signal-fails-pace{pace}");
        run(out, &mut c, &label, ratio, &beats, pace, 0)?;
    }
    // At the antenna's pace the verdict comes while data window 1 fills: with each phase of
    // the pace, one run takes a sample in the drop's own cycle (it must not reach the FFT).
    // Paces from 5 to 12 clocks a sample until one does (the generator requires it).
    let fails = failing_noise(&mut rng, 9)?;
    for pace in 5..=12 {
        let mut beats = fails.clone();
        beats.extend_from_slice(&next);
        let before = c.drop_takes;
        run(
            out,
            &mut c,
            &format!("signal-fails-pace{pace}x"),
            ratio,
            &beats,
            pace,
            0,
        )?;
        if c.drop_takes > before {
            break;
        }
    }
    // A sub-GHz carrier (HaLow's ratio) and a forged CFO word: SIGNAL's θ moves its window.
    let halow = clock_ratio(FS, 0.9e9);
    for (label, word) in [
        ("halow-cfo-high", (1i64 << 21) - 1),
        ("halow-cfo-low", -(1i64 << 21)),
    ] {
        let mut b = stream(Rate::R12, 40, 30.0, 0.0)?;
        if let Some(HandoffBeat::Sample { first: Some(f), .. }) = b.first_mut() {
            f.cfo_phase = word;
        }
        run(out, &mut c, label, halow, &b, 0, 0)?;
    }
    // Two frames back to back, each with its mark.
    let mut beats = stream(Rate::R36, 60, 30.0, 0.0)?;
    beats.extend(stream(Rate::R9, 40, 30.0, 0.0)?);
    run(out, &mut c, "two-frames", ratio, &beats, 0, 0)?;
    Ok(c)
}

/// `rvec rx_data`, with its cover on standard error.
pub fn write(out: &mut impl Write) -> io::Result<()> {
    let c = vectors(out)?;
    eprintln!("rvec rx_data: {c:?}");
    if c.drop_takes == 0 || c.early == 0 || c.late == 0 {
        return Err(io::Error::other(format!("a case not covered: {c:?}")));
    }
    if c.ends
        .iter()
        .take(5)
        .enumerate()
        .any(|(i, &n)| n == 0 && i != 1)
    {
        return Err(io::Error::other(format!(
            "an end status not covered: {c:?}"
        )));
    }
    Ok(())
}
