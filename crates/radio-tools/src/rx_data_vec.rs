//! `rvec rx_data`: the data path's top level's vectors (`hw/rtl/rx_data.sv`,
//! `docs/gates/rx_data.md`), from `radio_phy::rx_data::RxDataSchedule`: the hand-off's streams
//! (the front's model, `RxHandoffFixed`, on frames through a channel), every beat's take cycle
//! and every event's cycle at the block's pins.
//!
//! A run: the block reset, then cycle 0, 1, … Beat i is offered (valid) from cycle
//! max(its `avail`, the cycle after beat i − 1 was taken) and held until taken; the schedule's
//! reader takes it in the cycle the vectors give. Events are what the block's outputs show in
//! the cycle given; no other cycle shows one.
//!
//! The age's inputs (rx_handoff's, here made up): `adc_count` in cycle c is
//! (`count0` + ⌊c / 8⌋) mod 2^16, a sample every 8 clocks from the run's `count0`; `end_stamp`
//! the latest stamp line's value from its cycle on. A stamp follows each verdict with a
//! length, some way behind the count so that the age varies and wraps; rx_end's age is
//! `rx_end_age` of the end, the count and the stamp in its cycle.

use std::f64::consts::PI;
use std::io::{self, Write};

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, build_ppdu_bits, data_symbols, Rate};
use radio_phy::rx_data::{rx_end_age, DataTiming, RxDataSchedule};
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

/// A frame sent with a seed of 0 (SERVICE and the PSDU clear: the receiver finds no seed),
/// after 400 samples of silence, at −15 dBFS and 30 dB, through the ADC.
fn adc_seed_zero(rate: Rate, octets: usize, seed: u64) -> io::Result<Vec<(i16, i16)>> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let mut rng = Rng::new(seed);
    let mut clear = vec![0u8; 16];
    for _ in 0..octets {
        let byte = (rng.next_u64() & 0xff) as u8;
        clear.extend((0..8).map(|i| (byte >> i) & 1));
    }
    clear.resize(data_symbols(rate, octets) * rate.data_bits_per_symbol(), 0);
    let f: Vec<(f64, f64)> = build_ppdu_bits(&clear, rate, octets, &cfg)
        .map_err(|e| io::Error::other(format!("{e:?}")))?
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect();
    let p = mean_power(&f);
    let mut x = vec![(0.0, 0.0); 400];
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); 600]);
    let x = awgn(&x, 30.0, p, &mut rng);
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
    /// Data window 1 complete in the verdict's own cycle.
    window_at_verdict: usize,
    /// θ of data symbol 1 before a verdict of the run (a frame that does not measure its
    /// timing; in a run of two packets also the first's against the second's: the search
    /// that needs it counts its own hits).
    theta_before_verdict: usize,
    /// Ends with an age, and those whose subtraction wraps.
    ages: usize,
    age_wraps: usize,
}

/// `adc_count` in cycle `c` of a run that starts from `count0` (see the module's text).
fn count_at(count0: u16, c: usize) -> u16 {
    count0.wrapping_add(u16::try_from((c / 8) & 0xffff).unwrap_or(0))
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
    run_split(out, cover, label, ratio, beats, pace, offset, None)
}

/// [`run`] with the pace changed from beat `later.0` on to `later.1`, after a gap of `later.2`.
#[allow(clippy::too_many_arguments)]
fn run_split(
    out: &mut impl Write,
    cover: &mut Cover,
    label: &str,
    ratio: Option<u16>,
    beats: &[HandoffBeat],
    pace: usize,
    offset: usize,
    later: Option<(usize, usize, usize)>,
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
    // Runs start the count near its wrap, or at 0 (a stamp behind a count of 0 wraps), in turn.
    let count0 = if cover.runs.is_multiple_of(3) {
        0
    } else {
        u16::try_from((cover.runs * 7919 + 64_900) % 65_536).unwrap_or(0)
    };
    writeln!(out, "C {count0}")?;
    let mut clock = 0usize;
    let mut lines = Vec::with_capacity(beats.len());
    for (i, b) in beats.iter().enumerate() {
        let avail = match later {
            Some((from, q, gap)) if i >= from => pace * from + offset + gap + q * (i - from),
            _ => pace * i + offset,
        };
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
    let verdicts: Vec<usize> = s
        .events()
        .iter()
        .filter(|(_, e)| {
            matches!(
                e,
                RxDataEvent::Verdict {
                    verdict: Some(_),
                    ..
                }
            )
        })
        .map(|(c, _)| *c)
        .collect();
    cover.theta_before_verdict += s
        .symbols()
        .iter()
        .filter(|r| r.symbol == 1 && verdicts.iter().any(|&v| r.theta + 1 < v))
        .count();
    cover.window_at_verdict += s
        .windows()
        .iter()
        .filter(|w| w.window == radio_phy::rx_data::Window::Symbol(1) && verdicts.contains(&w.last))
        .count();
    if !s.violations().is_empty() {
        return Err(io::Error::other(format!(
            "{label}: violations {:?}",
            s.violations()
        )));
    }
    for l in lines {
        writeln!(out, "{l}")?;
    }
    let mut stamp = 0u16;
    for &(c, e) in s.events() {
        match e {
            RxDataEvent::Verdict {
                packet, verdict, ..
            } => {
                let (fail, sym, pat) =
                    verdict.map_or((1, 0, 0), |v| (0, v.data_symbols, usize::from(v.patient)));
                writeln!(out, "V {c} {packet:x} {fail} {sym} {pat}")?;
                if verdict.is_some() {
                    // rx_handoff takes the stamp at its next tick; any value will do here.
                    let behind = u16::try_from((c * 31 + 100) % 900).unwrap_or(0);
                    stamp = count_at(count0, c).wrapping_sub(behind);
                    writeln!(out, "P {} {stamp}", c + 3)?;
                }
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
                let count = count_at(count0, c);
                let (valid, age) = rx_end_age(end, count, stamp).map_or((0, 0), |a| (1, a));
                cover.ages += valid;
                cover.age_wraps += usize::from(valid == 1 && count < stamp);
                writeln!(out, "E {c} {status} {info} {valid} {age}")?;
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
        "# O <cycle> <octet> <first> <seed> <service>; E <cycle> <status> <info> <age valid> <age>;"
    )?;
    writeln!(
        out,
        "# C <count0>: adc_count = count0 + cycle / 8; P <cycle> <end_stamp>: from that cycle; Z <last cycle>"
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
    // A seed of 0: the seed-zero end, with its age.
    for (k, rate) in [Rate::R6, Rate::R54].into_iter().enumerate() {
        let b = first_stream(&adc_seed_zero(rate, 24, 0x5EED + k as u64)?);
        run(
            out,
            &mut c,
            &format!("seed0-r{}", rate_code(rate)),
            ratio,
            &b,
            0,
            0,
        )?;
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
    // Frames that do not measure their timing (noise near 0 dB at 6 Mb/s): θ of data symbol
    // 1 comes before the verdict, and data window 2 waits for the verdict itself.
    // A search over SNR and seeds; the generator requires a hit.
    let mut found = 0;
    for k in 0..200 {
        let snr = [0.5, 1.0, 1.5, 2.0][k % 4];
        let b = stream(Rate::R6, 24, snr, 0.0)?;
        let mut trial = Cover::default();
        run(&mut Vec::new(), &mut trial, "probe", ratio, &b, 0, 0)?;
        if trial.theta_before_verdict > 0 {
            run(out, &mut c, &format!("no-measure-{k}"), ratio, &b, 0, 0)?;
            found += 1;
            if found == 2 {
                break;
            }
        }
    }
    // Counted by this search alone (the cover's count also sees other packets' verdicts).
    if found == 0 {
        return Err(io::Error::other(
            "no frame that does not measure its timing",
        ));
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
        // After data window 1, before the verdict (the review's deadlock, 2026-11-25).
        ("cut-after-window1", 295, HandoffEnd::Overrun, 0),
        ("cut-after-window1-pace2", 290, HandoffEnd::Acquisition, 2),
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
    // Data window 1 complete in the verdict's own cycle, then a mark inside the data: the
    // steps expected at the mark count that window (a pace search; the generator requires a
    // hit). Each try is written to a buffer and kept only when it hits.
    let mut hit = false;
    let mut beats: Vec<HandoffBeat> = a[..144 + 80 * 4 + 10].to_vec();
    beats.push(HandoffBeat::End(HandoffEnd::Acquisition));
    beats.extend_from_slice(&next);
    'search: for pace in [1usize, 8] {
        for after in 2..=5 {
            for gap in 0..80 {
                let later = Some((208, after, gap));
                let mut trial = Cover::default();
                let mut sink = Vec::new();
                run_split(
                    &mut sink, &mut trial, "probe", ratio, &beats, pace, 0, later,
                )?;
                if trial.window_at_verdict > 0 {
                    let label = format!("window-at-verdict-{pace}-{after}-{gap}");
                    run_split(out, &mut c, &label, ratio, &beats, pace, 0, later)?;
                    hit = true;
                    break 'search;
                }
            }
        }
    }
    if !hit {
        return Err(io::Error::other(
            "no pace puts data window 1 in the verdict's cycle",
        ));
    }
    // A sub-GHz carrier (HaLow's ratio) and a forged CFO word: SIGNAL's θ moves its window.
    let halow = clock_ratio(FS, 0.9e9);
    for (label, word) in [
        ("halow-cfo-high", (1i64 << 21) - 1),
        // The shift runs to tens of samples (σ's width).
        ("halow-cfo-drift-high", 1i64 << 20),
        ("halow-cfo-drift-low", -(1i64 << 20)),
        ("halow-cfo-low", -(1i64 << 21)),
    ] {
        // The drift runs: 100 symbols, the windows moved a sample a symbol at most.
        let (rate, octets) = if label.contains("drift") {
            (Rate::R6, 300)
        } else {
            (Rate::R12, 40)
        };
        let mut b = stream(rate, octets, 30.0, 0.0)?;
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
    if c.drop_takes == 0
        || c.early == 0
        || c.late == 0
        || c.window_at_verdict == 0
        || c.theta_before_verdict == 0
        || c.ages == 0
        || c.age_wraps == 0
    {
        return Err(io::Error::other(format!("a case not covered: {c:?}")));
    }
    if c.ends.contains(&0) {
        return Err(io::Error::other(format!(
            "an end status not covered: {c:?}"
        )));
    }
    Ok(())
}
