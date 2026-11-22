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
//! - `hold`: the gain held from the control's first lock or from the latest sample at which
//!   the receiver can have declared its acquisition (ADR 0008).
//! - `edge`: the 13 / 1 hand-off under an offset, small clock offsets, the acquisition's share.
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

use std::collections::VecDeque;
use std::f64::consts::PI;
use std::process::ExitCode;

use radio_phy::channel::{
    agc_transient, awgn, fractional_delay, frequency_offset, mean_power, resample, tgn_taps,
    Doppler, FastAttackAgc, GainDc, GainState, Multipath, Rng, TgnModel,
};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{acquire, correct, dc_block, ltf_dc_offset, RxConfig};
use radio_phy::rx_front_fixed::{
    acquire_fixed, correct_fixed, dc_block_fixed, ltf_dc_offset_fixed, remove_dc_fixed, to_adc,
    FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd, RxHandoffFixed, SignalVerdict,
    ADC_FULL_SCALE, IQ,
};
use radio_phy::rx_ofdm::{decode_ppdu_from, RxOfdmConfig, Tracking};
use radio_phy::rx_ofdm_fixed::{
    clock_ratio, decode_ppdu_fixed, decode_signal_checked_fixed, FixedOfdmConfig, FixedTracking,
};

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
    /// The gain is held from this many samples after the packet's first sample instead of
    /// from the first lock inside the packet (never before that lock).
    hold_after: Option<usize>,
    /// What is sent before the frame.
    burst: Burst,
}

#[derive(Clone, Copy, PartialEq)]
enum Burst {
    None,
    /// The frame's first 320 samples (its two training fields and nothing after them) at
    /// the stream's start, 280 to 330 samples before the frame itself.
    Preamble,
    /// A whole frame of 100 octets of zeros at the same rate and level, 650 samples into
    /// the stream (after the timer of the control's first lock on noise has run out), and
    /// the frame itself a SIFS (320 samples) after its last sample.
    Frame,
    /// A weaker frame of the same length (zeros) at the frame's rate or at `weak`, `db` below
    /// the frame, 600 to 650 samples into the stream; the frame itself begins [`UNDER_AT`] samples into it, in its
    /// data.
    Under {
        db: f64,
        weak: Option<Rate>,
    },
    /// White noise added to the frame over `len` samples, [`HIT_AT`] samples into it (in its
    /// data), nominally `db` above the frame's mean power (one draw of `len` samples: a few
    /// dB either way for a short one), before the channel: an interferer's burst.
    Hit {
        len: usize,
        db: f64,
    },
}

/// Where the frame begins inside the weaker one of [`Burst::Under`].
const UNDER_AT: usize = 1200;
/// Where the burst of [`Burst::Hit`] begins inside the frame.
const HIT_AT: usize = 1000;

/// A receiver chain on the ADC's samples.
#[derive(Clone, Copy, PartialEq)]
enum Chain {
    /// Float DC blocker, float front, `rx_ofdm` at its defaults.
    Float,
    /// `dc_block_fixed`, `RxFrontFixed`, `correct_fixed` to `out_bits`, `decode_ppdu_fixed`
    /// with `in_bits = out_bits` and this `in_shift`.
    Int { out_bits: u32, in_shift: u32 },
    /// The receiver as ADR 0008 has it, in a closed loop with the gain control ([`keep`]
    /// with the front's hold and its release, no timer): the column's other fields do not
    /// apply, nor does the row's assumed hold.
    Stream,
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
    /// The column counts the acquisition alone: lost is a frame with none, or with an FFT
    /// start more than 16 samples from the frame's own T1.
    acq_only: bool,
}

const fn col(name: &'static str, chain: Chain) -> Column {
    Column {
        name,
        chain,
        known: false,
        raw: false,
        shift: DC_SHIFT,
        ltf: false,
        acq_only: false,
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

/// One frame of a point as the gain control meets it.
struct Stream {
    psdu: Vec<u8>,
    rate: Rate,
    /// The channel's samples: the frame behind `lead` samples of noise, 200 after it.
    x: Vec<C>,
    /// The same at the antenna's level (the point's noise level at the largest gain).
    xs: Vec<C>,
    dc: GainDc,
    lead: usize,
    /// One past the frame's last sample.
    end: usize,
    /// Mean power of the frame's span in `xs`.
    span_power: f64,
    /// The generator after the frame's own draws: what follows belongs to the gain control.
    rng: Rng,
}

fn stream(pt: &Point, agc: &FastAttackAgc, number: u64) -> Stream {
    let mut rng = Rng::new(mix(pt.scene, number));
    let psdu: Vec<u8> = (0..OCTETS).map(|_| (rng.next_u64() & 0xff) as u8).collect();
    let f = frame(&psdu, pt.rate);
    let mut lead = 600 + (rng.uniform() * 50.0) as usize;
    let mut x = vec![(0.0, 0.0); lead];
    let mut under = None;
    match pt.burst {
        Burst::None => {}
        Burst::Preamble => x[..320].copy_from_slice(&f[..320]),
        Burst::Frame => {
            x.truncate(650);
            x.extend(frame(&[0u8; 100], pt.rate));
            x.extend(vec![(0.0, 0.0); 320]);
            lead = x.len();
        }
        Burst::Under { .. } => {
            // The weaker frame takes the frame's place and the frame moves into it.
            under = Some(lead);
            x.extend(vec![(0.0, 0.0); UNDER_AT]);
            lead = x.len();
        }
        Burst::Hit { .. } => {}
    }
    x.extend_from_slice(&f);
    x.extend(vec![(0.0, 0.0); 200]);
    match pt.burst {
        Burst::Under { db, weak } => {
            let a = 10f64.powf(-db / 20.0);
            let weak = frame(&[0u8; OCTETS], weak.unwrap_or(pt.rate));
            // The frame ends UNDER_AT samples after the weaker one: `x` holds both.
            for (o, w) in x[under.unwrap_or(0)..].iter_mut().zip(&weak) {
                *o = (o.0 + a * w.0, o.1 + a * w.1);
            }
        }
        Burst::Hit { len, db } => {
            // Its own generator: the frame's own draws do not depend on the burst.
            let mut hit = Rng::new(mix(mix(pt.scene, number), 0x6869_7421));
            let at = lead + HIT_AT;
            let noise = awgn(&x[at..at + len], -db, mean_power(&f), &mut hit);
            x[at..at + len].copy_from_slice(&noise);
        }
        _ => {}
    }
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
    let span_power = mean_power(span) * k * k;
    Stream {
        psdu,
        rate: pt.rate,
        x,
        xs,
        dc,
        lead,
        end,
        span_power,
        rng,
    }
}

fn one(pt: &Point, columns: &[Column], agc: &FastAttackAgc, number: u64) -> Outcome {
    let st = stream(pt, agc, number);
    let streamed = columns.iter().any(|c| c.chain == Chain::Stream).then(|| {
        let front = Keeper {
            name: "",
            lock_hold: 0,
            at_max: true,
            hold: Hold::Front { delay: 0 },
            clip_release: true,
            by_rate: true,
            trial: None,
        };
        // The loop's phase jumps are drawn apart from the frame's and the other columns'.
        keep(&st, agc, &front, mix(mix(pt.scene, number), 0x6a75_6d70)).ok
    });
    let Stream {
        psdu,
        x,
        xs,
        dc,
        lead,
        end,
        span_power,
        mut rng,
        ..
    } = st;
    let full = f64::from(ADC_FULL_SCALE);
    let cfg_float = RxConfig {
        dc_shift: None,
        ..RxConfig::new(FS)
    };
    let fx = FixedRxConfig::default();
    let ofdm = RxOfdmConfig::default();
    let span = &x[lead.min(end)..end];
    // The control's detectors see the offset with the signal.
    let mut t = agc.run_dc(&xs, Some(&dc));
    // The gain is held from the first lock inside the packet (assumed: a hold driven by the
    // receiver, ADR 0008's question); without it a peak of the data unlocks it again.
    let mut held = false;
    if let Some(&lock) = t.locks.iter().find(|l| **l > lead) {
        let at = pt.hold_after.map_or(lock, |after| lock.max(lead + after));
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
    let level = span_power * g * g;

    let adc_known = columns.iter().any(|c| c.known).then(|| {
        let gain = full * (10f64.powf(KNOWN_DBFS / 10.0) / mean_power(span)).sqrt();
        to_adc(&x, gain)
    });

    // One front per kind, ADC stream and pole, made when a column first asks for it.
    type FloatFront = (Vec<C>, Vec<C>, Option<(usize, f64)>);
    type IntFront = (Vec<IQ>, Vec<IQ>, Option<(usize, i64)>);
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
                Chain::Stream => streamed.unwrap_or(false),
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
                    if c.acq_only {
                        return acq
                            .is_some_and(|(fft_start, _)| fft_start.abs_diff(lead + 192) <= 16);
                    }
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
                        let acq = acquire_fixed(adc, &cfg, &fx).map(|a| (a.fft_start, a.cfo_phase));
                        int_fronts.push((key, (raw, blocked, acq)));
                    }
                    let Some((_, (raw, blocked, acq))) = int_fronts.iter().find(|f| f.0 == key)
                    else {
                        return false;
                    };
                    if c.acq_only {
                        return acq
                            .is_some_and(|(fft_start, _)| fft_start.abs_diff(lead + 192) <= 16);
                    }
                    acq.is_some_and(|(fft_start, cfo_phase)| {
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
                        let start = match clock_ratio(FS, CARRIER_HZ) {
                            Some(ratio) => FixedTracking::from_cfo_word(cfo_phase, ratio, &fo),
                            None => FixedTracking::unknown(&fo),
                        };
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

/// Who keeps the gain still in a column of `loop` (ADR 0008, decision 2).
#[derive(Clone, Copy)]
struct Keeper {
    name: &'static str,
    /// The control's own timer: samples a lock is kept ([`FastAttackAgc::lock_hold`]).
    lock_hold: usize,
    /// Whether the timer also keeps a lock taken at the largest gain
    /// ([`FastAttackAgc::lock_hold_at_max`]).
    at_max: bool,
    /// How the hold is driven.
    hold: Hold,
    /// The front gives up its hold on a clipping ADC (ADR 0008, decision 2 as amended
    /// 2026-11-22: [`HandoffConfig::clip_count`] of the last [`HandoffConfig::clip_window`]
    /// samples on a rail). Off: the hold as it was before that amendment.
    clip_release: bool,
    /// With the release, the model's rule by the rate (the fourth amendment, the same day:
    /// [`SignalVerdict::of`], the patient rule and its inheritance). Off: 2 of 4 all
    /// through, the third amendment as it was.
    by_rate: bool,
    /// A release tried in this tool only, in place of the model's (`rxagc release`).
    trial: Option<Trial>,
}

/// A release that is in this tool only: the front's hold stops acting on the control once
/// `count` of the last `window` ADC samples sit on a rail, until the front declares again.
#[derive(Clone, Copy, PartialEq)]
struct Trial {
    /// (count, window) until the decoder's verdict on SIGNAL names a robust rate, and all
    /// through when `robust` is `None`.
    fast: (usize, usize),
    /// What holds once SIGNAL has decoded to 6, 9 or 12 Mb/s (a frame that survives a few
    /// clipped samples under a still gain): `None` is `fast` all through, `Some(None)` no
    /// release at all, `Some(Some(rule))` that rule.
    robust: Option<Option<(usize, usize)>>,
    /// The fewest agreeing coded bits of SIGNAL (`SignalCheck::agree`) at which its rate is
    /// believed.
    trust: usize,
    /// A declaration under a hold that is up and patient (the stream it replaces had named
    /// a robust rate, and nothing was released) leaves the hold patient until its own
    /// SIGNAL's verdict, instead of going back to `fast` at once.
    inherit: bool,
}

/// The longest window a [`Trial`] looks back over.
const TRIAL_WINDOW: usize = 32;

#[derive(Clone, Copy, PartialEq)]
enum Hold {
    /// None: the control is free all through.
    Free,
    /// From the control's first lock inside the packet to the stream's end: the assumption
    /// of every other command, which knows where the packet is.
    FromLock,
    /// The front's own ([`RxHandoffFixed::hold`]): what it shows after a sample holds the
    /// control from the next sample, this many samples later.
    Front { delay: usize },
}

/// What one keeper did with one frame.
#[derive(Clone, Copy, Default)]
struct Kept {
    ok: bool,
    /// Acquisitions the front declared in the stream.
    declared: usize,
    /// Streams stopped because SIGNAL did not decode.
    stopped: usize,
    /// Streams ended by a new declaration or by the history running over.
    aborted: usize,
    /// The gain changed after the packet's short training field (160 samples) and more than
    /// 16 samples before its end.
    moved: bool,
    /// SIGNALs that decoded, by the coded bits that agree with them (`SignalCheck::agree`,
    /// in fours: 0 to 3, 4 to 7, … 44 to 47, 48): the frame's own (its rate, and its length
    /// or the 100 octets of the frame before it) and the others.
    own: [u32; CHECK_BINS],
    other: [u32; CHECK_BINS],
}

/// Bins of SIGNAL's agreeing bits in `release`'s second table.
const CHECK_BINS: usize = 13;

/// Samples from the FFT window of T1 to the end of SIGNAL's window, and the periods the
/// decoder takes from the beat that completes them to its verdict (as `rxper stream`; ADR
/// 0008, decision 3).
const SIGNAL_END: usize = 128 + 80;
const VERDICT_LATENCY: usize = 50;

/// One frame through the closed loop of one keeper, a sample at a time: the gain control
/// (with its timer), the ADC, the streaming front ([`RxHandoffFixed`]) and the integer
/// decoder as its reader, which sees the beats alone, gives SIGNAL's verdict
/// [`VERDICT_LATENCY`] periods after the beat that completes SIGNAL's window and decodes a
/// stream when its end mark comes. The front's hold after sample n − 1 − `delay` is the
/// control's hold of sample n ([`Hold::Front`]). Every stretch of constant gain has its own
/// offset, and the carrier phase turns at each change by an angle uniform in ±[`JUMP_DEG`]
/// drawn from `jumps` and the change's own sample. All integer from the ADC on.
fn keep(st: &Stream, agc: &FastAttackAgc, k: &Keeper, jumps: u64) -> Kept {
    let agc = FastAttackAgc {
        lock_hold: k.lock_hold,
        lock_hold_at_max: k.at_max,
        ..*agc
    };
    let fo = FixedOfdmConfig::default();
    let ofdm = RxOfdmConfig::default();
    let start = |word: i64| match clock_ratio(FS, CARRIER_HZ) {
        Some(ratio) => FixedTracking::from_cfo_word(word, ratio, &fo),
        None => FixedTracking::unknown(&fo),
    };
    // The tool's own hold: it knows where the packet is.
    let from_lock = (k.hold == Hold::FromLock)
        .then(|| {
            let free = agc.run_held(&st.xs, Some(&st.dc), &[]);
            free.locks.into_iter().find(|l| *l > st.lead)
        })
        .flatten();
    let delay = match k.hold {
        Hold::Front { delay } => Some(delay),
        _ => None,
    };
    let pace = HandoffConfig::default();
    let mut control = agc.start(Some(st.dc));
    let mut hand = RxHandoffFixed::new(
        RxConfig::new(FS),
        FixedRxConfig::default(),
        HandoffConfig {
            clip_count: if k.clip_release && k.trial.is_none() {
                pace.clip_count
            } else {
                0
            },
            ..pace
        },
    );
    // The front's hold after each sample not yet acted on, oldest first.
    let mut line: VecDeque<bool> = VecDeque::new();
    // The trial release: the last ADC samples' rail flags (the newest last), whether the
    // hold in force has been given up, whether SIGNAL of the open stream named a robust
    // rate, and whether a declaration still waits for its stream's first beat.
    let mut rails: VecDeque<bool> = VecDeque::new();
    let (mut given_up, mut robust, mut unopened) = (false, false, false);
    // The front's hold after the sample before.
    let mut held_on = false;
    let (mut gain, mut phase) = (f64::NAN, 0.0);
    let mut z: Vec<IQ> = Vec::new();
    let (mut word, mut tag) = (0i64, 0u8);
    // The verdict that is due: its period, the stream's tag and what SIGNAL said.
    let mut due: Option<(usize, u8, Option<SignalVerdict>, bool)> = None;
    let mut out = Kept::default();
    for (n, &s) in st.xs.iter().enumerate() {
        let held = match delay {
            Some(d) if line.len() > d => line.pop_front().unwrap_or(false),
            Some(_) => false,
            None => from_lock.is_some_and(|lock| n >= lock),
        };
        let step = control.push(s, held);
        if step.gain_db != gain {
            if n > 0 {
                let u = Rng::new(mix(jumps, n as u64)).uniform();
                phase += (2.0 * u - 1.0) * JUMP_DEG.to_radians();
                out.moved |= n > st.lead + 160 && n + 16 < st.end;
            }
            gain = step.gain_db;
        }
        let state = GainState {
            until: usize::MAX,
            gain_db: gain,
            phase,
            dc: st.dc.at(gain),
        };
        let at_adc = to_adc(&agc_transient(&[s], &[state]), f64::from(ADC_FULL_SCALE));
        let Some(&sample) = at_adc.first() else {
            break;
        };
        let got = hand.push(sample, usize::MAX);
        out.declared += usize::from(got.acquired.is_some());
        let rail = |v: i16| i32::from(v) <= -ADC_FULL_SCALE || i32::from(v) >= ADC_FULL_SCALE - 1;
        rails.push_back(rail(sample.0) || rail(sample.1));
        if rails.len() > TRIAL_WINDOW {
            rails.pop_front();
        }
        if got.acquired.is_some() {
            let keep = k.trial.is_some_and(|t| t.inherit) && robust && held_on && !given_up;
            (given_up, robust, unopened) = (false, keep, true);
        }
        for beat in got.beats {
            match beat {
                HandoffBeat::Sample { z: v, first } => {
                    if let Some(f) = first {
                        z.clear();
                        (word, tag) = (f.cfo_phase, f.packet);
                        unopened = false;
                    }
                    z.push(v);
                    if z.len() == SIGNAL_END {
                        let signal =
                            decode_signal_checked_fixed(&z, 0, &ofdm, &fo, start(word)).ok();
                        if let Some((s, check)) = signal {
                            let own =
                                s.rate == st.rate && (s.length == st.psdu.len() || s.length == 100);
                            let bins = if own { &mut out.own } else { &mut out.other };
                            bins[(check.agree / 4).min(CHECK_BINS - 1)] += 1;
                        }
                        let slow = signal.is_some_and(|(s, check)| {
                            matches!(s.rate, Rate::R6 | Rate::R9 | Rate::R12)
                                && k.trial.is_none_or(|t| check.agree >= t.trust)
                        });
                        // The model's patience only where the column has it.
                        let verdict = signal.map(|(s, check)| {
                            let v = SignalVerdict::of(s, check, &pace);
                            SignalVerdict {
                                patient: v.patient && k.by_rate && k.trial.is_none(),
                                ..v
                            }
                        });
                        due = Some((n + VERDICT_LATENCY, tag, verdict, slow));
                    }
                }
                HandoffBeat::End(end) => {
                    match end {
                        HandoffEnd::Packet => {
                            out.ok |= decode_ppdu_fixed(&z, 0, &ofdm, &fo, start(word))
                                .is_ok_and(|d| d.psdu == st.psdu);
                        }
                        HandoffEnd::Stopped => out.stopped += 1,
                        HandoffEnd::Acquisition | HandoffEnd::Overrun => out.aborted += 1,
                    }
                    z.clear();
                }
            }
        }
        if let Some((_, of, verdict, slow)) = due.filter(|d| d.0 == n) {
            due = None;
            hand.signal(of, verdict);
            // The verdict is the open stream's: no declaration since its first beat.
            if of == tag && !unopened {
                robust = slow;
            }
        }
        if delay.is_some() {
            let hold = hand.hold();
            if let Some(t) = k.trial {
                let rule = match t.robust {
                    Some(r) if robust => r,
                    _ => Some(t.fast),
                };
                given_up &= hold;
                given_up |= hold
                    && rule.is_some_and(|(c, w)| {
                        rails.iter().rev().take(w).filter(|r| **r).count() >= c
                    });
            }
            held_on = hold;
            line.push_back(hold && !given_up);
        }
    }
    out
}

/// `loop`'s row: per keeper the frames lost, the acquisitions declared, the streams stopped
/// on a failed SIGNAL, the streams aborted and the frames whose gain moved after the short
/// training field.
#[derive(Clone)]
struct LoopTally {
    frames: usize,
    lost: Vec<usize>,
    only: Vec<(usize, usize)>,
    declared: Vec<usize>,
    stopped: Vec<usize>,
    aborted: Vec<usize>,
    moved: Vec<usize>,
    own: Vec<[u64; CHECK_BINS]>,
    other: Vec<[u64; CHECK_BINS]>,
}

fn run_loop(pt: &Point, keepers: &[Keeper], frames: usize) -> LoopTally {
    let agc = FastAttackAgc {
        power_samples: 16,
        unlock_on_overload: true,
        ..FastAttackAgc::ad9361_reset(2)
    };
    let n = threads();
    let zero = LoopTally {
        frames: 0,
        lost: vec![0; keepers.len()],
        only: vec![(0, 0); keepers.len()],
        declared: vec![0; keepers.len()],
        stopped: vec![0; keepers.len()],
        aborted: vec![0; keepers.len()],
        moved: vec![0; keepers.len()],
        own: vec![[0; CHECK_BINS]; keepers.len()],
        other: vec![[0; CHECK_BINS]; keepers.len()],
    };
    let parts = par(|th| {
        let mut t = zero.clone();
        for number in (th..frames).step_by(n) {
            let mut st = stream(pt, &agc, number as u64);
            let jumps = st.rng.next_u64();
            let r: Vec<Kept> = keepers.iter().map(|k| keep(&st, &agc, k, jumps)).collect();
            let reference = r.first().is_none_or(|k| k.ok);
            for (c, k) in r.iter().enumerate() {
                t.lost[c] += usize::from(!k.ok);
                t.only[c].0 += usize::from(!k.ok && reference);
                t.only[c].1 += usize::from(k.ok && !reference);
                t.declared[c] += k.declared;
                t.stopped[c] += k.stopped;
                t.aborted[c] += k.aborted;
                t.moved[c] += usize::from(k.moved);
                for b in 0..CHECK_BINS {
                    t.own[c][b] += u64::from(k.own[b]);
                    t.other[c][b] += u64::from(k.other[b]);
                }
            }
            t.frames += 1;
        }
        t
    });
    parts.into_iter().fold(zero, |mut a, t| {
        for c in 0..a.lost.len() {
            a.lost[c] += t.lost[c];
            a.only[c].0 += t.only[c].0;
            a.only[c].1 += t.only[c].1;
            a.declared[c] += t.declared[c];
            a.stopped[c] += t.stopped[c];
            a.aborted[c] += t.aborted[c];
            a.moved[c] += t.moved[c];
            for b in 0..CHECK_BINS {
                a.own[c][b] += t.own[c][b];
                a.other[c][b] += t.other[c][b];
            }
        }
        a.frames += t.frames;
        a
    })
}

/// Decision 2 of ADR 0008 as a closed loop: the control's timer and the front's own hold
/// against the hold the earlier commands assumed.
fn hold_loop(frames: usize, trials: bool) {
    let keeper = |name: &'static str, lock_hold: usize, at_max: bool, hold: Hold| Keeper {
        name,
        lock_hold,
        at_max,
        hold,
        clip_release: false,
        by_rate: false,
        trial: None,
    };
    let front = Hold::Front { delay: 0 };
    let keepers = [
        keeper("from the lock", 0, true, Hold::FromLock),
        keeper("timer + front", 512, true, front),
        keeper("timer<max + front", 512, false, front),
        keeper("front alone", 0, true, front),
        keeper("timer<max alone", 512, false, Hold::Free),
        keeper("nothing", 0, true, Hold::Free),
        keeper("t<max + front +40", 512, false, Hold::Front { delay: 40 }),
        keeper("t<max + front +400", 512, false, Hold::Front { delay: 400 }),
        Keeper {
            clip_release: true,
            ..keeper("front + release", 0, true, front)
        },
        Keeper {
            clip_release: true,
            ..keeper("front + release +40", 0, true, Hold::Front { delay: 40 })
        },
        Keeper {
            clip_release: true,
            by_rate: true,
            ..keeper("front + release by rate", 0, true, front)
        },
    ];
    // `release`: the model's rule against rules that are in this tool only.
    let trial = |name: &'static str, fast, robust, trust| Keeper {
        trial: Some(Trial {
            fast,
            robust,
            trust,
            inherit: false,
        }),
        ..keeper(name, 0, true, front)
    };
    let slow = Some(Some((24, 32)));
    let tried = [
        keeper("from the lock", 0, true, Hold::FromLock),
        keeper("front alone", 0, true, front),
        Keeper {
            clip_release: true,
            ..keeper("front + release", 0, true, front)
        },
        trial("4 of 8", (4, 8), None, 0),
        trial("8 of 16", (8, 16), None, 0),
        trial("24 of 32", (24, 32), None, 0),
        trial("2/4; slow: none", (2, 4), Some(None), 0),
        trial("2/4; slow: 24/32", (2, 4), slow, 0),
        trial("slow 24/32, agree>=24", (2, 4), slow, 24),
        trial("slow 24/32, agree>=32", (2, 4), slow, 32),
        trial("slow 24/32, agree>=40", (2, 4), slow, 40),
        trial("slow 24/32, agree>=44", (2, 4), slow, 44),
        trial("slow none, agree>=40", (2, 4), Some(None), 40),
        Keeper {
            trial: Some(Trial {
                fast: (2, 4),
                robust: slow,
                trust: 40,
                inherit: true,
            }),
            ..keeper("24/32, >=40, inherited", 0, true, front)
        },
        Keeper {
            trial: Some(Trial {
                fast: (2, 4),
                robust: Some(None),
                trust: 40,
                inherit: true,
            }),
            ..keeper("none, >=40, inherited", 0, true, front)
        },
        Keeper {
            clip_release: true,
            by_rate: true,
            ..keeper("the model", 0, true, front)
        },
    ];
    let keepers: &[Keeper] = if trials { &tried } else { &keepers };
    const IDLE: f64 = -30.0;
    const LOUD: f64 = -5.0;
    // The rows `release` runs: indices into `points`.
    const TRIED: [usize; 27] = [
        2, 4, 6, 8, 9, 10, 11, 12, 15, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
        32, 33, 34,
    ];
    let HandoffConfig {
        clip_window,
        clip_count,
        patient_window,
        patient_count,
        patient_agree,
        ..
    } = HandoffConfig::default();
    println!(
        "# rxagc loop: ADR 0008, decision 2, as a closed loop a sample at a time. {frames} frames of {OCTETS} octets per row, the\n\
         # frames of `hold` (the same scenes); noise at {IDLE} dBFS at the largest gain unless the row says, no DC offset unless the\n\
         # row says; clock offset uniform in ±40 ppm, carrier 5.8 GHz. Gain control: the AD9361's reset values with 16\n\
         # samples of power and the overload exit on (as every command), stepped with the receiver (FastAttackAgc::start).\n\
         # Receiver, all in integers, in every column: each ADC sample through the streaming front (RxHandoffFixed: 8 beats\n\
         # a period, the replay 20 periods after the declaration), the decoder as its reader: it sees the beats alone, gives\n\
         # SIGNAL's verdict (decode_signal_fixed on the stream's first {SIGNAL_END} samples, with the stream's tag) {VERDICT_LATENCY} periods\n\
         # after the beat that completes them and decodes a stream (decode_ppdu_fixed, the tracker started from the CFO\n\
         # word) when its end mark comes. The front searches all through; a frame is received if any stream decodes to it.\n\
         # Phase jump per gain change uniform in ±{JUMP_DEG}°.\n\
         # Who keeps the gain still:\n\
         #   from the lock    = held from the control's first lock inside the packet to the stream's end\n\
         #                      (the tool knows where the packet is: the assumption of the other commands' columns);\n\
         #   timer + front    = the control keeps every lock 512 samples, and the front's own hold (RxHandoffFixed::hold:\n\
         #                      from the declaration to the end SIGNAL's length gives, to the verdict when SIGNAL fails,\n\
         #                      to the longest PPDU's end with no verdict) holds the control from the sample after;\n\
         #   timer<max + front = the same, the timer keeping only a lock taken below the largest gain\n\
         #                      (a lock on noise is left at the first overload);\n\
         #   front alone      = the front's hold without any timer and without the release (decision 2 before 2026-11-22);\n\
         #   timer<max alone  = that timer and no hold;   nothing = neither;\n\
         #   t<max + front +40/+400 = timer<max + front with the hold acting 40 and 400 samples (2 and 20 µs) later, at\n\
         #                      both its edges: the control stays held that long past the hold's end;\n\
         #   front + release  = decision 2's third amendment (2026-11-22): front alone, and the front gives up its\n\
         #                      hold when {clip_count} of the last {clip_window} ADC samples sit on a rail (RxHandoffFixed with no\n\
         #                      verdict ever patient; the stream goes on, only a new declaration raises the hold again);\n\
         #   front + release +40 = the same with the hold acting 40 samples (2 µs) later, its release with it;\n\
         #   front + release by rate = the fourth amendment, the model's default: once SIGNAL has decoded to 6, 9 or\n\
         #                      12 Mb/s with {patient_agree} or more of its 48 coded bits agreeing (SignalVerdict::of), {patient_count} of the\n\
         #                      last {patient_window} in place of {clip_count} of {clip_window}; a declaration under a hold that is up and so patient\n\
         #                      stays patient until its own verdict.\n\
         # The release is off in every column but the last three.\n\
         # A released control returns to the largest gain and searches (an assumption).\n\
         # Rows 'preamble +': the frame's two training fields alone (320 samples, nothing after them)\n\
         # are sent first, 280 to 330 samples before the frame: an acquisition whose SIGNAL is noise.\n\
         # Rows 'frame + SIFS +': a whole frame of 100 octets at the same rate and level first, the\n\
         # frame counted 320 samples after its end.\n\
         # Rows 'noise {LOUD}': the noise at {LOUD} dBFS at the largest gain, above the lock level: the\n\
         # control locks on it below the largest gain.\n\
         # Rows 'DC per index -20': an offset of -20 dBFS at every gain, its direction per gain index; rows 'DC before\n\
         # gain -10': an offset of -10 dBFS at the largest gain that falls with the gain, one direction (channel::GainDc).\n\
         # Rows 'under -D dB': a weaker frame of the same rate and length (rows '6 Mb/s under': at 6 Mb/s), D dB below\n\
         # the frame, is being received\n\
         # (from 600 to 650 samples into the stream) when the frame begins {UNDER_AT} samples into it; the frame is the\n\
         # one counted, and its SNR is the row's.\n\
         # Rows 'hit L +D dB': white noise added to the frame over L samples, {HIT_AT} samples into it, nominally D dB above\n\
         # the frame's mean power (one draw: a few dB either way at L = 4), before the channel's delay and clock offset.\n\
         # Each cell: frames lost, +a−b against 'from the lock' (lost by this column alone, by that one\n\
         # alone), then acquisitions declared / streams stopped on a failed SIGNAL / streams ended by a new\n\
         # declaration or an overrun of the history / frames whose gain changed after the short training\n\
         # field and more than 16 samples before the frame's end."
    );
    if trials {
        println!(
            "# rxagc release: the rows of `loop` named below, the same frames, with other columns. Releases tried in\n\
             # this tool only (not in the model, not in ADR 0008), each in place of the model's, on front alone:\n\
             #   C of W           = the hold stops acting on the control once C of the last W ADC samples sit on a rail,\n\
             #                      until the front declares again ('2 of 4' is the model's: the column front + release);\n\
             #   2/4; slow: X     = 2 of 4, until the decoder's verdict on SIGNAL of the open stream names 6, 9 or\n\
             #                      12 Mb/s; from then on X (none: no release; C/W: that rule);\n\
             #   slow X, agree>=N = the same, the rate believed only when N or more of SIGNAL's 48 coded bits agree\n\
             #                      with the decoded field encoded again (rx_ofdm::SignalCheck; a soft value of 0 agrees\n\
             #                      with nothing).\n\
             #   X, >=N, inherited = slow X, agree>=N, and a declaration under a hold that is up and patient (nothing\n\
             #                      released since a believed slow rate) leaves it patient until its own verdict.\n\
             #   the model        = RxHandoffFixed at its defaults (`loop`'s front + release by rate).\n\
             # Under each row: the SIGNALs that decoded in the column front + release, by agreeing bits in fours (0 to 3,\n\
             # 4 to 7, ... 44 to 47, 48): 'own' has the frame's rate and its length (or the 100 octets of the frame\n\
             # before it).\n\
             # The text above is `loop`'s; its columns that are not below were not run."
        );
    }
    println!(
        "# {:<38} {:>6} {}",
        "point",
        "frames",
        keepers
            .iter()
            .map(|k| format!("{:>32}", k.name))
            .collect::<Vec<_>>()
            .join(" ")
    );
    let d = Channel::Tgn(TgnModel::D);
    let (n, p, f) = (Burst::None, Burst::Preamble, Burst::Frame);
    let (awgn, r54, r6) = (Channel::Awgn, Rate::R54, Rate::R6);
    let under = |db| Burst::Under { db, weak: None };
    let over6 = |db| Burst::Under {
        db,
        weak: Some(Rate::R6),
    };
    let (r12, r24) = (Rate::R12, Rate::R24);
    let hit = |len, db| Burst::Hit { len, db };
    let points: [(&str, Channel, Rate, f64, Burst, f64); 35] = [
        ("AWGN 54 Mb/s 25 dB", awgn, r54, 25.0, n, IDLE),
        ("AWGN 54 Mb/s 30 dB", awgn, r54, 30.0, n, IDLE),
        ("AWGN 54 Mb/s 40 dB", awgn, r54, 40.0, n, IDLE),
        ("AWGN 54 Mb/s 60 dB", awgn, r54, 60.0, n, IDLE),
        ("AWGN 6 Mb/s 30 dB", awgn, r6, 30.0, n, IDLE),
        ("AWGN 6 Mb/s 60 dB", awgn, r6, 60.0, n, IDLE),
        ("TGn D 54 Mb/s 40 dB", d, r54, 40.0, n, IDLE),
        ("TGn D 54 Mb/s 60 dB", d, r54, 60.0, n, IDLE),
        ("TGn D 6 Mb/s 30 dB", d, r6, 30.0, n, IDLE),
        ("AWGN 6 Mb/s 5 dB", awgn, r6, 5.0, n, IDLE),
        ("preamble + AWGN 54 Mb/s 30 dB", awgn, r54, 30.0, p, IDLE),
        ("preamble + AWGN 6 Mb/s 30 dB", awgn, r6, 30.0, p, IDLE),
        (
            "frame + SIFS + AWGN 54 Mb/s 30 dB",
            awgn,
            r54,
            30.0,
            f,
            IDLE,
        ),
        ("frame + SIFS + AWGN 6 Mb/s 30 dB", awgn, r6, 30.0, f, IDLE),
        ("noise -5: AWGN 54 Mb/s 30 dB", awgn, r54, 30.0, n, LOUD),
        (
            "noise -5: preamble + 54 Mb/s 30 dB",
            awgn,
            r54,
            30.0,
            p,
            LOUD,
        ),
        (
            "noise -5: frame + SIFS + 54 Mb/s 30 dB",
            awgn,
            r54,
            30.0,
            f,
            LOUD,
        ),
        (
            "under -25 dB: AWGN 54 Mb/s 50 dB",
            awgn,
            r54,
            50.0,
            under(25.0),
            IDLE,
        ),
        (
            "under -25 dB: AWGN 6 Mb/s 50 dB",
            awgn,
            r6,
            50.0,
            under(25.0),
            IDLE,
        ),
        (
            "under -10 dB: AWGN 6 Mb/s 50 dB",
            awgn,
            r6,
            50.0,
            under(10.0),
            IDLE,
        ),
        (
            "hit 4 +10 dB: AWGN 54 Mb/s 30 dB",
            awgn,
            r54,
            30.0,
            hit(4, 10.0),
            IDLE,
        ),
        (
            "hit 4 +20 dB: AWGN 6 Mb/s 30 dB",
            awgn,
            r6,
            30.0,
            hit(4, 20.0),
            IDLE,
        ),
        (
            "hit 8 +20 dB: AWGN 6 Mb/s 30 dB",
            awgn,
            r6,
            30.0,
            hit(8, 20.0),
            IDLE,
        ),
        (
            "hit 8 +10 dB: AWGN 6 Mb/s 30 dB",
            awgn,
            r6,
            30.0,
            hit(8, 10.0),
            IDLE,
        ),
        (
            "hit 16 +10 dB: AWGN 6 Mb/s 30 dB",
            awgn,
            r6,
            30.0,
            hit(16, 10.0),
            IDLE,
        ),
        (
            "6 Mb/s under -25 dB: 54 Mb/s 50 dB",
            awgn,
            r54,
            50.0,
            over6(25.0),
            IDLE,
        ),
        (
            "6 Mb/s under -10 dB: 54 Mb/s 50 dB",
            awgn,
            r54,
            50.0,
            over6(10.0),
            IDLE,
        ),
        (
            "under -25 dB: AWGN 12 Mb/s 50 dB",
            awgn,
            r12,
            50.0,
            under(25.0),
            IDLE,
        ),
        (
            "under -10 dB: AWGN 12 Mb/s 50 dB",
            awgn,
            r12,
            50.0,
            under(10.0),
            IDLE,
        ),
        (
            "hit 8 +20 dB: AWGN 12 Mb/s 30 dB",
            awgn,
            r12,
            30.0,
            hit(8, 20.0),
            IDLE,
        ),
        (
            "hit 8 +20 dB: AWGN 24 Mb/s 30 dB",
            awgn,
            r24,
            30.0,
            hit(8, 20.0),
            IDLE,
        ),
        ("TGn D: preamble + 54 Mb/s 40 dB", d, r54, 40.0, p, IDLE),
        (
            "TGn D: under -25 dB: 54 Mb/s 50 dB",
            d,
            r54,
            50.0,
            under(25.0),
            IDLE,
        ),
        (
            "TGn D: hit 8 +20 dB: 6 Mb/s 30 dB",
            d,
            r6,
            30.0,
            hit(8, 20.0),
            IDLE,
        ),
        ("TGn D 6 Mb/s 50 dB", d, r6, 50.0, n, IDLE),
    ];
    // The offsets of `dc` on the frames of the 2nd, 11th and 13th rows: (name, index of the
    // row in `points`, per index, before the gain at the largest).
    let offsets: [(&str, usize, f64, f64); 6] = [
        ("DC per index -20: AWGN 54 Mb/s 30 dB", 1, -20.0, none()),
        ("DC per index -20: preamble + 54 Mb/s", 10, -20.0, none()),
        ("DC per index -20: frame + SIFS + 54", 12, -20.0, none()),
        ("DC before gain -10: AWGN 54 Mb/s 30 dB", 1, none(), -10.0),
        ("DC before gain -10: preamble + 54 Mb/s", 10, none(), -10.0),
        ("DC before gain -10: frame + SIFS + 54", 12, none(), -10.0),
    ];
    let rows = points
        .iter()
        .enumerate()
        .map(|(pi, p)| (p.0, pi, none(), none()))
        .chain(offsets)
        .filter(|r| !trials || (r.2.is_infinite() && r.3.is_infinite() && TRIED.contains(&r.1)));
    for (name, pi, per_index, before) in rows {
        let (_, ch, rate, snr, burst, idle) = points[pi];
        let pt = Point {
            scene: 8000 + pi as u64,
            rate,
            snr_db: snr,
            ch,
            idle_dbfs: idle,
            per_index_dbfs: per_index,
            before_dbfs: before,
            ppm_band: None,
            hold_after: None,
            burst,
        };
        let t = run_loop(&pt, keepers, frames);
        println!(
            "  {name:<38} {:>6} {}",
            t.frames,
            (0..keepers.len())
                .map(|c| format!(
                    "{:>32}",
                    format!(
                        "{} +{}−{} {}/{}/{}/{}",
                        t.lost[c],
                        t.only[c].0,
                        t.only[c].1,
                        t.declared[c],
                        t.stopped[c],
                        t.aborted[c],
                        t.moved[c]
                    )
                ))
                .collect::<Vec<_>>()
                .join(" ")
        );
        if trials {
            let line = |b: &[u64; CHECK_BINS]| {
                b.iter()
                    .map(|v| v.to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            println!(
                "#     own: {}   other: {}",
                line(&t.own[2]),
                line(&t.other[2])
            );
        }
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
         # wrongly (the first acquisition of the stream is the only one tried; a column 'stream' tries\n\
         # every one) or not decoded. level, chg and held describe the assumed hold of the other\n\
         # columns, not the loop of a column 'stream', which also ignores a row's own hold.\n\
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
         # (assumed; `hold` has rows that hold it later), phase jump ±{JUMP_DEG}° per gain change (assumed), 12-bit ADC.\n\
         # Chains on the ADC's samples:\n\
         #   float     = float DC blocker (pole 2^-{DC_SHIFT}), float front, rx_ofdm (8 bits, clip 3);\n\
         #   int O/s   = dc_block_fixed (pole 2^-{DC_SHIFT}), RxFrontFixed, correct_fixed to O bits,\n\
         #               decode_ppdu_fixed with in_bits O and in_shift s, tracker started from the\n\
         #               CFO word and the register fs/fc (FixedTracking::from_cfo_word): no float\n\
         #               between the ADC and the PSDU; the int columns of a row share one acquisition;\n\
         #   stream    = where a command has it: the receiver of ADR 0008 in a closed loop with the\n\
         #               gain control, a sample at a time (the column `front + release` of `loop`): the\n\
         #               streaming front (RxHandoffFixed), its own hold in place of the assumed one,\n\
         #               the offset of the two long training symbols removed, 13 bits, the decoder\n\
         #               at its defaults reading the beats; the front searches all through and the\n\
         #               frame is received if any stream decodes to it; its phase jumps are its own draws;\n\
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
        col("stream", Chain::Stream),
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
                    hold_after: None,
                    burst: Burst::None,
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
                    hold_after: None,
                    burst: Burst::None,
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
        col("stream", Chain::Stream),
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
                hold_after: None,
                burst: Burst::None,
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
        col("stream", Chain::Stream),
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
                    hold_after: None,
                    burst: Burst::None,
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
            hold_after: None,
            burst: Burst::None,
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
            hold_after: None,
            burst: Burst::None,
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
                hold_after: None,
                burst: Burst::None,
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
                hold_after: None,
                burst: Burst::None,
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
                    hold_after: None,
                    burst: Burst::None,
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

/// Who holds the gain: the control's first lock inside the packet (the assumption of every
/// other command) against the receiver, from the latest sample at which its acquisition can
/// be declared.
fn hold(frames: usize) {
    let ltf = |name: &'static str, chain: Chain| Column {
        ltf: true,
        ..col(name, chain)
    };
    let columns = [
        ltf("float ltf", Chain::Float),
        col("float 2^-7", Chain::Float),
        ltf("int 13/1 ltf", INT_13_1),
    ];
    let reference = 0;
    const IDLE: f64 = -30.0;
    // First sample of T1 (192) + the latest declaration (223, the doc of
    // `rx_front::HISTORY`) + the decision latency (64).
    const DECLARED: usize = 192 + 223 + 64;
    preamble_text("hold", frames);
    println!(
        "# Noise at {IDLE} dBFS at the largest gain, no DC offset. Rows 'lock': the gain held from the\n\
         # control's first lock inside the packet (as everywhere else). Rows 'declared': held from\n\
         # {DECLARED} samples after the packet's first, the latest the acquisition is declared (T1 + 223\n\
         # + 64), or from the first lock if that is later: until then the control is free. That\n\
         # sample is fixed by the frame's position, not taken from the chain's own acquisition,\n\
         # and the hold lasts to the stream's end whether or not the frame was acquired. The\n\
         # two rows of a point are the same frames; they differ only in frames whose gain\n\
         # changes between the first lock and that sample (a handful per row).\n\
         #   2^-7 = the decoder handed the blocker's output (the chain as it is);\n\
         #   ltf  = the decoder handed the samples before the blocker less the offset estimated\n\
         #          from the two long training symbols."
    );
    header(&columns, reference);
    let d = Channel::Tgn(TgnModel::D);
    let points: [(&str, Channel, Rate, f64); 9] = [
        ("AWGN 54 Mb/s 25 dB", Channel::Awgn, Rate::R54, 25.0),
        ("AWGN 54 Mb/s 30 dB", Channel::Awgn, Rate::R54, 30.0),
        ("AWGN 54 Mb/s 40 dB", Channel::Awgn, Rate::R54, 40.0),
        ("AWGN 54 Mb/s 60 dB", Channel::Awgn, Rate::R54, 60.0),
        ("AWGN 6 Mb/s 30 dB", Channel::Awgn, Rate::R6, 30.0),
        ("AWGN 6 Mb/s 60 dB", Channel::Awgn, Rate::R6, 60.0),
        ("TGn D 54 Mb/s 40 dB", d, Rate::R54, 40.0),
        ("TGn D 54 Mb/s 60 dB", d, Rate::R54, 60.0),
        ("TGn D 6 Mb/s 30 dB", d, Rate::R6, 30.0),
    ];
    println!("{}", head_line("point, the gain held from", &columns));
    for (pi, (name, ch, rate, snr)) in points.into_iter().enumerate() {
        for (what, hold_after) in [("lock", None), ("declared", Some(DECLARED))] {
            let pt = Point {
                scene: 8000 + pi as u64,
                rate,
                snr_db: snr,
                ch,
                idle_dbfs: IDLE,
                per_index_dbfs: none(),
                before_dbfs: none(),
                ppm_band: None,
                hold_after,
                burst: Burst::None,
            };
            let t = run(&pt, &columns, reference, frames);
            println!("{}", row_line(&format!("{name}, {what}"), &t, reference));
        }
    }
}

/// What `cand` left out (ADR 0008's review): the 13 / 1 hand-off under an offset and at the
/// lower edge, the clock offsets below the estimate's guard, and how many of the frames lost
/// were lost by the acquisition.
fn edge(frames: usize) {
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
        Column {
            ltf: true,
            ..col("int 13/1 ltf", INT_13_1)
        },
        Column {
            acq_only: true,
            ..col("float acq", Chain::Float)
        },
        Column {
            acq_only: true,
            ..col("int acq", INT_13_1)
        },
    ];
    let reference = 2;
    preamble_text("edge", frames);
    println!(
        "# DC per gain state as in `dc`; the blocker (in every column's acquisition) is told nothing.\n\
         #   2^-7 = the decoder handed the blocker's output (the chain as it is);\n\
         #   ltf  = the decoder handed the samples before the blocker less the offset estimated\n\
         #          from the two long training symbols;\n\
         #   acq  = the acquisition alone: lost is a frame with none, or with an FFT start more\n\
         #          than 16 samples from the frame's own T1 (the +a−b of these two columns\n\
         #          compare unlike things and mean nothing)."
    );
    header(&columns, reference);
    // (title, channel, rate, SNR, scene, noise at the largest gain, clock offset's band)
    type Row = (
        &'static str,
        Channel,
        Rate,
        f64,
        u64,
        f64,
        Option<(f64, f64)>,
    );
    let a = Channel::Awgn;
    type Case = (&'static str, f64, f64);
    let sections: [(&str, Vec<Row>, Vec<Case>); 3] = [
        (
            "Where `cand` lost frames under an offset (its frames)",
            vec![
                ("AWGN 54 Mb/s 25 dB", a, Rate::R54, 25.0, 4000, -30.0, None),
                ("AWGN 54 Mb/s 20 dB", a, Rate::R54, 20.0, 5003, -30.0, None),
                ("AWGN 54 Mb/s 40 dB", a, Rate::R54, 40.0, 4002, -30.0, None),
            ],
            vec![
                ("no DC", none(), none()),
                ("per index −30 dBFS", -30.0, none()),
                ("per index −20 dBFS", -20.0, none()),
                ("before the gain −20 dBFS", none(), -20.0),
                ("before the gain −10 dBFS", none(), -10.0),
            ],
        ),
        (
            "Clock offset 0 to 2 ppm: at and below the estimate's guard (0.54 ppm)",
            vec![
                (
                    "AWGN 54 Mb/s 40 dB",
                    a,
                    Rate::R54,
                    40.0,
                    9000,
                    -30.0,
                    Some((0.0, 2.0)),
                ),
                (
                    "AWGN 54 Mb/s 20 dB",
                    a,
                    Rate::R54,
                    20.0,
                    9001,
                    -30.0,
                    Some((0.0, 2.0)),
                ),
                (
                    "AWGN 6 Mb/s 3.5 dB",
                    a,
                    Rate::R6,
                    3.5,
                    9002,
                    -30.0,
                    Some((0.0, 2.0)),
                ),
            ],
            vec![
                ("no DC", none(), none()),
                ("per index −30 dBFS", -30.0, none()),
                ("per index −20 dBFS", -20.0, none()),
            ],
        ),
        (
            "The lower edge of the level (`level`'s frames)",
            vec![
                (
                    "AWGN 54 Mb/s 20 dB, noise −48 dBFS",
                    a,
                    Rate::R54,
                    20.0,
                    3002,
                    -48.0,
                    None,
                ),
                (
                    "AWGN 54 Mb/s 20 dB, noise −54 dBFS",
                    a,
                    Rate::R54,
                    20.0,
                    3002,
                    -54.0,
                    None,
                ),
                (
                    "AWGN 6 Mb/s 3.5 dB, noise −48 dBFS",
                    a,
                    Rate::R6,
                    3.5,
                    3000,
                    -48.0,
                    None,
                ),
                (
                    "AWGN 6 Mb/s 3.5 dB, noise −54 dBFS",
                    a,
                    Rate::R6,
                    3.5,
                    3000,
                    -54.0,
                    None,
                ),
            ],
            vec![
                ("no DC", none(), none()),
                ("per index −50 dBFS", -50.0, none()),
            ],
        ),
    ];
    for (title, rows, cases) in sections {
        println!("\n## {title}");
        println!("{}", head_line("point, DC offset", &columns));
        for (name, ch, rate, snr, scene, idle, band) in rows {
            for &(case, per_index, before) in &cases {
                let pt = Point {
                    scene,
                    rate,
                    snr_db: snr,
                    ch,
                    idle_dbfs: idle,
                    per_index_dbfs: per_index,
                    before_dbfs: before,
                    ppm_band: band,
                    hold_after: None,
                    burst: Burst::None,
                };
                let t = run(&pt, &columns, reference, frames);
                println!("{}", row_line(&format!("{name}, {case}"), &t, reference));
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
        "hold" => hold(n.unwrap_or(2000)),
        "loop" => hold_loop(n.unwrap_or(2000), false),
        "release" => hold_loop(n.unwrap_or(2000), true),
        "edge" => edge(n.unwrap_or(2000)),
        _ => {
            eprintln!("usage: rxagc per|level|dc|pole|cfo|cand|hold|edge|loop [frames]");
            return ExitCode::from(2);
        }
    }
    ExitCode::SUCCESS
}
