//! Gate for the streaming data path (ADR 0010, decision 6.1b): `RxDataFixed` takes the
//! hand-off's beats one at a time, holds one FFT window, and must decide what the
//! whole-buffer decoder decides on the same samples, frame for frame.
//!
//! References (our model, so this is the data path's G2-side consistency, not G1): SIGNAL's
//! verdict of `decode_signal_checked_fixed` on the stream's first 208 samples, at the same
//! beat; the PSDU, the seed or the error of `decode_ppdu_fixed` on the whole stream, which
//! has its own G1 and G2 gates (`g1_rx_ofdm.rs`, `g2_rx_ofdm.rs`). The settled cases of ADR
//! 0010's Context 3 are the statuses compared here (`docs/gates/rx_ofdm.md`, 2026-11-23).

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{awgn, frequency_offset, mean_power, resample, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, data_symbols, Rate};
use radio_phy::rx_data_fixed::{RxDataEnd, RxDataEvent, RxDataFixed};
use radio_phy::rx_front::RxConfig;
use radio_phy::rx_front_fixed::{
    acquire_fixed, handoff_fixed, to_adc, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd,
    HandoffStart, RxHandoffFixed, SignalVerdict, ADC_FULL_SCALE, IQ,
};
use radio_phy::rx_ofdm::{RxOfdmConfig, RxOfdmError, SignalCheck};
use radio_phy::rx_ofdm_fixed::{
    clock_ratio, decode_ppdu_fixed, decode_signal_checked_fixed, FixedOfdmConfig, FixedTracking,
};
use std::f64::consts::PI;

const FS: f64 = 20e6;
const CARRIER_HZ: f64 = 5.8e9;
/// T1, T2 and SIGNAL's window: the samples SIGNAL's verdict is taken from.
const SIGNAL_END: usize = 128 + 80;
/// Periods from the beat that completes SIGNAL to the verdict's strobe (as `rxper stream`).
const VERDICT_LATENCY: usize = 50;

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

fn frame(psdu: &[u8], rate: Rate, seed: u8) -> Vec<(f64, f64)> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(psdu, rate, seed, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

/// A train of frames (rate, octets, SNR in dB, clock offset in ppm) with leads of noise and
/// gaps of a SIFS or more, at −15 dBFS; `burst`: the second frame starts inside the first's
/// data (an acquisition aborts the first stream).
fn train(
    frames: &[(Rate, usize, f64, f64)],
    burst: bool,
    seed: u64,
) -> (Vec<(i16, i16)>, Vec<Vec<u8>>) {
    let mut rng = Rng::new(seed);
    let mut x: Vec<(f64, f64)> = Vec::new();
    let mut psdus = Vec::new();
    let mut p_ref = 0.0;
    for (k, &(rate, octets, snr, ppm)) in frames.iter().enumerate() {
        let psdu: Vec<u8> = (0..octets).map(|_| (rng.next_u64() & 0xff) as u8).collect();
        let f = frame(&psdu, rate, 1 + (rng.next_u64() % 127) as u8);
        let p = mean_power(&f);
        if k == 0 {
            p_ref = p;
        }
        let mut part = vec![(0.0, 0.0); 320 + (rng.uniform() * 300.0) as usize];
        part.extend_from_slice(&f);
        part.extend(vec![(0.0, 0.0); 200]);
        let part = resample(&part, ppm);
        let part = frequency_offset(&part, ppm * 1e-6 * CARRIER_HZ, FS, 2.0 * PI * rng.uniform());
        let part = awgn(&part, snr, p, &mut rng);
        if burst && k == 1 {
            // Over the first frame's data, 10 dB stronger.
            let at = x.len() - 5000;
            for (v, s) in x.iter_mut().skip(at).zip(&part) {
                *v = (v.0 + 3.16 * s.0, v.1 + 3.16 * s.1);
            }
            x.extend(
                part.iter()
                    .skip(x.len() - at)
                    .map(|s| (3.16 * s.0, 3.16 * s.1)),
            );
        } else {
            x.extend(part);
        }
        psdus.push(psdu);
    }
    x.extend(vec![(0.0, 0.0); 400]);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-15.0 / 20.0) / p_ref.sqrt();
    (to_adc(&x, gain), psdus)
}

/// The verdict's pace, off its default so that a data path on the default would differ.
fn pace() -> HandoffConfig {
    HandoffConfig {
        patient_agree: 44,
        ..HandoffConfig::default()
    }
}

/// Samples the hand-off sends after a frame's last one before its end mark: the back-off
/// (4 at the default) and `HANDOFF_TAIL` (8).
const STREAM_TAIL: usize = 12;

fn ofdm() -> RxOfdmConfig {
    RxOfdmConfig::default()
}

fn start(word: i64) -> FixedTracking {
    let fo = FixedOfdmConfig::default();
    match clock_ratio(FS, CARRIER_HZ) {
        Some(r) => FixedTracking::from_cfo_word(word, r, &fo),
        None => FixedTracking::unknown(&fo),
    }
}

/// What the streaming data path gave for one stream.
#[derive(Default, Debug)]
struct Got {
    verdict_at: Option<usize>,
    verdict: Option<Option<SignalVerdict>>,
    check: Option<Option<SignalCheck>>,
    start: Option<radio_phy::rx_ofdm::Signal>,
    frame_end: Option<usize>,
    octets: Vec<u8>,
    seed: Option<u8>,
    service: Option<u16>,
    end: Option<RxDataEnd>,
}

impl Got {
    fn take(&mut self, ev: &[RxDataEvent], beat: usize) {
        for e in ev {
            match *e {
                RxDataEvent::Verdict { verdict, check, .. } => {
                    assert_eq!(
                        check.is_some(),
                        verdict.is_some(),
                        "a check with every verdict"
                    );
                    assert!(self.verdict.is_none(), "two verdicts");
                    self.verdict = Some(verdict);
                    self.check = Some(check);
                    self.verdict_at = Some(beat);
                }
                RxDataEvent::Start { signal, frame_end } => {
                    self.start = Some(signal);
                    self.frame_end = Some(frame_end);
                }
                RxDataEvent::Octet { octet, first } => {
                    assert_eq!(first.is_some(), self.octets.is_empty(), "first-octet flag");
                    if let Some(f) = first {
                        self.seed = Some(f.seed);
                        self.service = Some(f.service);
                    }
                    self.octets.push(octet);
                }
                RxDataEvent::End(end) => {
                    assert!(self.end.is_none(), "two ends");
                    self.end = Some(end);
                }
            }
        }
    }
}

/// Totals over a gate's streams.
#[derive(Default, Debug)]
struct Tally {
    /// The scenes are clean (SERVICE must decode to zero on a right frame).
    clean: bool,
    decoded: usize,
    right: usize,
    signal: usize,
    seed: usize,
    truncated: usize,
    aborted: usize,
    moves: usize,
    /// Frame ends compared with the hand-off's end mark.
    ends: usize,
}

/// One stream checked against the whole-buffer decoder on its samples `z`, ended by `mark`.
fn check(
    z: &[IQ],
    word: i64,
    mark: HandoffEnd,
    through: bool,
    got: &Got,
    psdus: &[Vec<u8>],
    tally: &mut Tally,
) {
    let (cfg, fo, pace) = (ofdm(), FixedOfdmConfig::default(), pace());
    if z.len() >= SIGNAL_END {
        let signal = decode_signal_checked_fixed(&z[..SIGNAL_END], 0, &cfg, &fo, start(word)).ok();
        let want = signal.map(|(s, c)| SignalVerdict::of(s, c, &pace));
        assert_eq!(got.verdict, Some(want), "verdict");
        assert_eq!(got.check, Some(signal.map(|(_, c)| c)), "SIGNAL's check");
        assert_eq!(got.verdict_at, Some(SIGNAL_END - 1), "verdict's beat");
    }
    match mark {
        HandoffEnd::Acquisition | HandoffEnd::Overrun
            if got.end == Some(RxDataEnd::Aborted(mark)) =>
        {
            tally.aborted += 1;
            return;
        }
        _ => {}
    }
    // A seed of zero is decided from the first data symbol, before the stream is whole; the
    // buffer's decoder wants the whole stream first, so it is run on it padded with zeros
    // (later samples cannot change bits already decided by the window).
    let padded: Vec<IQ>;
    let z = match got.start {
        Some(sig) if got.end == Some(RxDataEnd::Seed) => {
            let need = SIGNAL_END + 80 * data_symbols(sig.rate, sig.length) + 64;
            padded = z
                .iter()
                .copied()
                .chain(std::iter::repeat((0, 0)))
                .take(need.max(z.len()))
                .collect();
            &padded[..]
        }
        _ => z,
    };
    match decode_ppdu_fixed(z, 0, &cfg, &fo, start(word)) {
        Ok(d) => {
            assert_eq!(
                got.end,
                Some(RxDataEnd::Decoded),
                "end of a frame the buffer decodes"
            );
            assert_eq!(got.start, Some(d.signal), "start");
            assert_eq!(got.seed, Some(d.scrambler_seed), "seed");
            assert_eq!(got.octets, d.psdu, "octets");
            if mark == HandoffEnd::Packet && through {
                // The frame's end at the antenna, against where the hand-off ends the stream
                // (the back-off and HANDOFF_TAIL later), once the verdict came in time: a short
                // frame's stream runs on until the verdict.
                let end = got.frame_end.unwrap_or(0);
                if end > 600 {
                    assert_eq!(end, z.len() - STREAM_TAIL, "frame end");
                    tally.ends += 1;
                } else {
                    assert!(end + STREAM_TAIL <= z.len(), "frame end past the stream");
                }
            }
            tally.decoded += 1;
            if psdus.contains(&d.psdu) {
                // §17.3.5.2: SERVICE is transmitted as zeros (its first seven bits find the
                // seed, the other nine are reserved); in marginal noise its reserved bits may
                // decode wrong while the PSDU is right.
                if tally.clean {
                    assert_eq!(got.service, Some(0), "SERVICE of a right frame");
                }
                tally.right += 1;
            }
        }
        Err(RxOfdmError::ScramblerSeed) => {
            assert_eq!(got.end, Some(RxDataEnd::Seed), "seed zero");
            assert!(got.octets.is_empty(), "no octet before a seed of zero");
            tally.seed += 1;
        }
        Err(RxOfdmError::Truncated { .. }) => {
            assert_eq!(got.end, Some(RxDataEnd::Truncated), "truncated");
            tally.truncated += 1;
        }
        Err(e) => {
            assert_eq!(got.end, Some(RxDataEnd::Signal(e)), "SIGNAL's error");
            assert!(got.start.is_none() && got.octets.is_empty());
            tally.signal += 1;
        }
    }
}

/// The streaming front with `RxDataFixed` as its reader (every beat taken at once, the
/// verdict back after `VERDICT_LATENCY` periods), each stream checked when its mark comes.
fn through_front(adc: &[(i16, i16)], psdus: &[Vec<u8>], tally: &mut Tally) {
    let rx = RxConfig::new(FS);
    let pace = pace();
    let mut hand = RxHandoffFixed::new(rx, FixedRxConfig::default(), pace);
    let mut data = RxDataFixed::new(
        ofdm(),
        FixedOfdmConfig::default(),
        pace,
        clock_ratio(FS, CARRIER_HZ),
    )
    .unwrap();
    let mut z: Vec<IQ> = Vec::new();
    let mut word = 0i64;
    let mut got = Got::default();
    let mut due: Option<(usize, u8, Option<SignalVerdict>)> = None;
    for (n, &s) in adc.iter().enumerate() {
        for beat in hand.push(s, usize::MAX).beats {
            if let HandoffBeat::Sample { z: v, first } = beat {
                if let Some(HandoffStart { cfo_phase, .. }) = first {
                    z.clear();
                    word = cfo_phase;
                    got = Got::default();
                }
                z.push(v);
            }
            let before = data.shift();
            let ev = data.push(beat);
            tally.moves += usize::from(data.shift() != before && data.busy());
            got.take(&ev, z.len().saturating_sub(1));
            for e in &ev {
                if let RxDataEvent::Verdict {
                    packet, verdict, ..
                } = *e
                {
                    due = Some((n + VERDICT_LATENCY, packet, verdict));
                }
            }
            if let HandoffBeat::End(mark) = beat {
                check(&z, word, mark, true, &got, psdus, tally);
                z.clear();
                got = Got::default();
            }
        }
        if let Some((_, tag, verdict)) = due.filter(|d| d.0 == n) {
            due = None;
            hand.signal(tag, verdict);
        }
    }
}

#[test]
fn the_stream_decodes_what_the_buffer_decodes() {
    let mut tally = Tally {
        clean: true,
        ..Tally::default()
    };
    let mut seed = 0xDA7A_0000u64;
    // Every rate, short and long, clean, with clock offsets.
    for (k, &rate) in RATES.iter().enumerate() {
        let ppm = [-40.0, 40.0, -20.0, 25.0][k % 4];
        let frames: Vec<_> = [1usize, 24, 300]
            .iter()
            .map(|&o| (rate, o, 28.0, ppm))
            .collect();
        seed += 1;
        let (adc, psdus) = train(&frames, false, seed);
        through_front(&adc, &psdus, &mut tally);
    }
    // Long frames at 6 Mb/s and ±40 ppm: the tracker moves the window.
    for ppm in [-40.0, 40.0] {
        seed += 1;
        let (adc, psdus) = train(&[(Rate::R6, 2000, 25.0, ppm)], false, seed);
        through_front(&adc, &psdus, &mut tally);
    }
    tally.clean = false;
    // Marginal noise: SIGNAL fails on some, the data on others, and some of those lose the
    // scrambler's seed (seven zero bits, about one in 128).
    for k in 0..40 {
        seed += 1;
        let frames: Vec<_> = (0..12)
            .map(|j| {
                (
                    RATES[(k + j) % 8],
                    4 + j % 3,
                    -1.0 + (j % 6) as f64 * 1.5,
                    0.0,
                )
            })
            .collect();
        let (adc, psdus) = train(&frames, false, seed);
        through_front(&adc, &psdus, &mut tally);
    }
    tally.clean = true;
    // A frame inside another: the first stream aborted by the second's acquisition.
    for k in 0..3 {
        seed += 1;
        let (adc, psdus) = train(
            &[(Rate::R12, 800, 25.0, 0.0), (RATES[k + 4], 40, 25.0, 0.0)],
            true,
            seed,
        );
        through_front(&adc, &psdus, &mut tally);
    }
    println!("{tally:?}");
    assert!(tally.right >= 26, "{tally:?}");
    assert!(
        tally.decoded > tally.right,
        "frames decoded wrong: none, {tally:?}"
    );
    assert!(
        tally.signal > 0 && tally.seed > 0 && tally.aborted > 0,
        "{tally:?}"
    );
    assert!(tally.moves >= 2 && tally.ends >= 12, "{tally:?}");
}

/// Streams cut short and closed by an end mark, fed straight from the hand-off's buffer: a
/// window past the stream's end is `Truncated`, as the buffer's decoder says, wherever the
/// cut falls; a cut after the last window changes nothing.
#[test]
fn a_stream_cut_short_is_truncated_as_the_buffer_says() {
    let rx = RxConfig::new(FS);
    let front = FixedRxConfig::default();
    let pace = pace();
    let mut tally = Tally::default();
    for (k, &rate) in RATES.iter().enumerate() {
        let (adc, psdus) = train(
            &[(rate, 60, 30.0, if k % 2 == 0 { 40.0 } else { -40.0 })],
            false,
            0xC07 + k as u64,
        );
        let a = acquire_fixed(&adc, &rx, &front).unwrap();
        let full = handoff_fixed(&adc, a.fft_start, a.cfo_phase, &front).unwrap();
        let z: Vec<IQ> = full[a.fft_start..].to_vec();
        let frame_len = decode_ppdu_fixed(
            &z,
            0,
            &ofdm(),
            &FixedOfdmConfig::default(),
            start(a.cfo_phase),
        )
        .map(|d| d.psdu.len())
        .unwrap();
        assert_eq!(frame_len, 60);
        let last = z.len().min(2000);
        for cut in (100..last).step_by(37).chain([last]) {
            let mut data = RxDataFixed::new(
                ofdm(),
                FixedOfdmConfig::default(),
                pace,
                clock_ratio(FS, CARRIER_HZ),
            )
            .unwrap();
            let mut got = Got::default();
            for (i, &v) in z[..cut].iter().enumerate() {
                let first = (i == 0).then_some(HandoffStart {
                    cfo_phase: a.cfo_phase,
                    packet: 7,
                });
                got.take(&data.push(HandoffBeat::Sample { z: v, first }), i);
            }
            got.take(&data.push(HandoffBeat::End(HandoffEnd::Packet)), cut);
            check(
                &z[..cut],
                a.cfo_phase,
                HandoffEnd::Packet,
                false,
                &got,
                &psdus,
                &mut tally,
            );
        }
    }
    println!("{tally:?}");
    assert!(tally.truncated > 100 && tally.decoded >= 8, "{tally:?}");
}
