//! The receiver's data path as a stream (ADR 0010, decision 6.1b): the hand-off's beats in,
//! SIGNAL's verdict, the frame's start, its octets and its end out — the decoder of
//! [`crate::rx_ofdm_fixed::decode_ppdu_fixed`] taken apart so that each sample is seen once,
//! in order, and only the current symbol's FFT window is held.
//!
//! The arithmetic is the whole-buffer decoder's own (`channel_with`, `symbol_with`,
//! `signal_checked`, the scrambler seed's search), and the Viterbi decoder is
//! [`WindowedViterbi`], the streaming form of `decode_windowed`. One symbol after another
//! (decision 2): symbol i + 1's window is placed when symbol i's tracker step has ended, by
//! the tracker's shift, so 15, 16 or 17 samples pass between two windows.
//!
//! What the whole-buffer decoder leaves open, settled here (ADR 0010, Context 3; the journal
//! `docs/gates/rx_ofdm.md`, 2026-11-23):
//! - **a scrambler seed of zero** (the first seven SERVICE bits all zero) is known before the
//!   first octet: no octet is handed on, the end is [`RxDataEnd::Seed`];
//! - **a window that needs samples past the stream's end** (the end mark comes first): the
//!   end is [`RxDataEnd::Truncated`], after whatever octets were already out;
//! - **the number of data symbols** for the verdict is [`SignalVerdict::of`]'s;
//! - **a mark that aborts the packet** after some octets: the end is
//!   [`RxDataEnd::Aborted`] with the mark's kind; the frame check is the MAC's (decision 5);
//! - **a new packet's first beat while the last one is still being decoded**: the last one
//!   is finished first and the new stream waits in the hand-off's history. In this model
//!   nothing takes time, so a frame is finished at the beat that completes its last window;
//!   that the wait fits the history is the schedule's (decision 6.1c). A first beat that
//!   comes without a mark before it ends the packet in flight as aborted by an acquisition;
//! - **the frame's end at the antenna**: given with the start, in samples from the stream's
//!   first one (one past the last data symbol's last sample, by the rate and length, not by
//!   the tracker); the top level adds the stream's place.

use crate::convolutional::CodeRate;
use crate::modulation::Modulation;
use crate::ppdu::data_symbols;
use crate::rx_front_fixed::{HandoffBeat, HandoffConfig, HandoffEnd, SignalVerdict, IQ};
use crate::rx_ofdm::{
    scrambler_seed_of, signal_checked, symbol_start, RxOfdmConfig, RxOfdmError, Signal, SignalCheck,
};
use crate::rx_ofdm_fixed::{
    channel_with, symbol_with, tables, FixedChannel, FixedOfdmConfig, FixedTracking, Tables,
};
use crate::scrambler::Scrambler;
use crate::viterbi::WindowedViterbi;

/// Samples of an FFT window.
const FFT_SIZE: usize = 64;
/// The two long training symbols' windows, from the stream's first sample.
const LTF_LEN: usize = 2 * FFT_SIZE;

/// How a frame's decoding ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxDataEnd {
    /// Every octet of the PSDU has been handed on (right or wrong: the MAC checks).
    Decoded,
    /// SIGNAL did not decode (parity, rate or length); no start was given.
    Signal(RxOfdmError),
    /// The first seven SERVICE bits were all zero: no scrambler seed; no octet was given.
    Seed,
    /// The stream's end mark came before the frame's last window.
    Truncated,
    /// An abort mark (or a first beat with no mark before it) ended the packet.
    Aborted(HandoffEnd),
}

/// What the first octet carries besides itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirstOctet {
    /// The transmitter's scrambler seed.
    pub seed: u8,
    /// The SERVICE field's 16 bits, descrambled, first transmitted bit in bit 0.
    pub service: u16,
}

/// What the data path gives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxDataEvent {
    /// SIGNAL's verdict for the front's hold (`RxHandoffFixed::signal`), with the stream's
    /// tag; `None`: SIGNAL failed.
    Verdict {
        /// The tag of the stream's first beat.
        packet: u8,
        /// The verdict.
        verdict: Option<SignalVerdict>,
        /// SIGNAL's check, what the verdict's patience was taken from (`None`: failed).
        check: Option<SignalCheck>,
    },
    /// `rx_start`: SIGNAL decoded.
    Start {
        /// The SIGNAL field.
        signal: Signal,
        /// The frame's end at the antenna, in samples from the stream's first one.
        frame_end: usize,
    },
    /// One octet of the PSDU, descrambled, in order.
    Octet {
        /// The octet (first received bit in bit 0).
        octet: u8,
        /// On the first octet only.
        first: Option<FirstOctet>,
    },
    /// `rx_end`.
    End(RxDataEnd),
}

/// The decoding of one frame after SIGNAL.
#[derive(Debug, Clone)]
struct Frame {
    ch: FixedChannel,
    signal: Signal,
    symbols: usize,
    /// The next data symbol (1-based: 0 is SIGNAL).
    next: usize,
    viterbi: WindowedViterbi,
    /// Decoded bits so far, and the scrambler once the seed is known.
    bits: usize,
    head: Vec<u8>,
    scrambler: Option<(u8, Scrambler)>,
    service: u16,
    octet: u8,
    octets: usize,
}

#[derive(Debug, Clone)]
enum Phase {
    /// No stream: beats are dropped until a first beat.
    Idle,
    /// Before SIGNAL's verdict: the long training field is kept whole for the channel.
    Training { ltf: Vec<IQ> },
    /// Data symbols.
    Data(Box<Frame>),
}

/// The streaming data path; see the module's documentation.
#[derive(Debug, Clone)]
pub struct RxDataFixed {
    cfg: RxOfdmConfig,
    fx: FixedOfdmConfig,
    pace: HandoffConfig,
    ratio: Option<u16>,
    t: Tables,
    phase: Phase,
    packet: u8,
    /// Samples of the stream so far.
    n: usize,
    tracking: FixedTracking,
    window: Vec<IQ>,
}

impl RxDataFixed {
    /// A data path decoding with `cfg` and the widths `fx`, giving verdicts by `pace`, the
    /// tracker started from the first beat's CFO word and the fs/fc register `ratio`
    /// ([`crate::rx_ofdm_fixed::clock_ratio`]; `None`: the tracker's prior alone). `None`
    /// for widths that cannot decode.
    pub fn new(
        cfg: RxOfdmConfig,
        fx: FixedOfdmConfig,
        pace: HandoffConfig,
        ratio: Option<u16>,
    ) -> Option<Self> {
        Some(Self {
            cfg,
            t: tables(&fx)?,
            fx,
            pace,
            ratio,
            phase: Phase::Idle,
            packet: 0,
            n: 0,
            tracking: FixedTracking::unknown(&fx),
            window: Vec::with_capacity(FFT_SIZE),
        })
    }

    /// Whether a frame is being decoded (a first beat seen, no end given yet).
    pub fn busy(&self) -> bool {
        !matches!(self.phase, Phase::Idle)
    }

    /// The tracker's shift of the windows, in samples (what the gates count the moves by).
    pub fn shift(&self) -> i64 {
        self.tracking.shift
    }

    /// The next beat of the hand-off's stream; returns what it decided, in order.
    pub fn push(&mut self, beat: HandoffBeat) -> Vec<RxDataEvent> {
        let mut out = Vec::new();
        match beat {
            HandoffBeat::End(mark) => {
                if self.busy() {
                    out.push(RxDataEvent::End(match mark {
                        HandoffEnd::Packet | HandoffEnd::Stopped => RxDataEnd::Truncated,
                        _ => RxDataEnd::Aborted(mark),
                    }));
                }
                self.phase = Phase::Idle;
            }
            HandoffBeat::Sample { z, first } => {
                if let Some(f) = first {
                    if self.busy() {
                        out.push(RxDataEvent::End(RxDataEnd::Aborted(
                            HandoffEnd::Acquisition,
                        )));
                    }
                    self.packet = f.packet;
                    self.n = 0;
                    self.tracking = match self.ratio {
                        Some(r) => FixedTracking::from_cfo_word(f.cfo_phase, r, &self.fx),
                        None => FixedTracking::unknown(&self.fx),
                    };
                    self.window.clear();
                    self.phase = Phase::Training {
                        ltf: Vec::with_capacity(LTF_LEN),
                    };
                }
                if self.busy() {
                    self.sample(z, &mut out);
                    self.n += 1;
                }
            }
        }
        out
    }

    /// The window of symbol `i` (0 = SIGNAL) starts here, as the tracker has moved it.
    fn window_start(&self, i: usize) -> Option<usize> {
        i64::try_from(symbol_start(0, i))
            .ok()
            .and_then(|s| s.checked_add(self.tracking.shift))
            .and_then(|s| usize::try_from(s).ok())
    }

    fn sample(&mut self, z: IQ, out: &mut Vec<RxDataEvent>) {
        let n = self.n;
        // The symbol whose window is being filled: SIGNAL while training, else the next.
        let i = match &mut self.phase {
            Phase::Idle => return,
            Phase::Training { ltf } => {
                if ltf.len() < LTF_LEN {
                    ltf.push(z);
                }
                0
            }
            Phase::Data(f) => {
                if f.next > f.symbols {
                    // Past the frame's last window: the stream's tail, dropped.
                    return;
                }
                f.next
            }
        };
        let Some(start) = self.window_start(i) else {
            return;
        };
        if n < start || n >= start + FFT_SIZE {
            return;
        }
        self.window.push(z);
        if self.window.len() < FFT_SIZE {
            return;
        }
        let window = std::mem::take(&mut self.window);
        self.window.reserve(FFT_SIZE);
        let phase = std::mem::replace(&mut self.phase, Phase::Idle);
        self.phase = match phase {
            Phase::Training { ltf } => self.signal(&ltf, &window, out),
            Phase::Data(mut f) => {
                if self.symbol(&mut f, &window, out) {
                    Phase::Data(f)
                } else {
                    Phase::Idle
                }
            }
            Phase::Idle => Phase::Idle,
        };
    }

    /// The channel and SIGNAL: the verdict, and the start or the end.
    fn signal(&mut self, ltf: &[IQ], window: &[IQ], out: &mut Vec<RxDataEvent>) -> Phase {
        let packet = self.packet;
        let failed = |e: RxOfdmError, out: &mut Vec<RxDataEvent>| {
            out.push(RxDataEvent::Verdict {
                packet,
                verdict: None,
                check: None,
            });
            out.push(RxDataEvent::End(RxDataEnd::Signal(e)));
            Phase::Idle
        };
        let short = RxOfdmError::Truncated {
            needed: LTF_LEN,
            have: ltf.len(),
        };
        let Some(ch) = channel_with(ltf, &self.fx, &self.t) else {
            return failed(short, out);
        };
        let Some((soft, _)) = symbol_with(
            window,
            &ch,
            0,
            Modulation::Bpsk,
            &self.cfg,
            &self.fx,
            &mut self.tracking,
            &self.t,
        ) else {
            return failed(short, out);
        };
        let (signal, check) = match signal_checked(&soft) {
            Ok(s) => s,
            Err(e) => return failed(e, out),
        };
        out.push(RxDataEvent::Verdict {
            packet,
            verdict: Some(SignalVerdict::of(signal, check, &self.pace)),
            check: Some(check),
        });
        let symbols = data_symbols(signal.rate, signal.length);
        out.push(RxDataEvent::Start {
            signal,
            frame_end: symbol_start(0, symbols).saturating_add(FFT_SIZE),
        });
        // §17.3.5.3: the encoder is back in state 0 after SERVICE, the PSDU and the tail.
        let anchor = 16 + 8 * signal.length + 6;
        let depth = match self.fx.traceback {
            0 => usize::MAX,
            d => d,
        };
        let code: CodeRate = signal.rate.code_rate();
        Phase::Data(Box::new(Frame {
            ch,
            signal,
            symbols,
            next: 1,
            viterbi: WindowedViterbi::new(code, depth, anchor),
            bits: 0,
            head: Vec::with_capacity(7),
            scrambler: None,
            service: 0,
            octet: 0,
            octets: 0,
        }))
    }

    /// One data symbol; whether the frame goes on.
    fn symbol(&mut self, f: &mut Frame, window: &[IQ], out: &mut Vec<RxDataEvent>) -> bool {
        let modulation = f.signal.rate.modulation();
        let soft = symbol_with(
            window,
            &f.ch,
            f.next,
            modulation,
            &self.cfg,
            &self.fx,
            &mut self.tracking,
            &self.t,
        );
        f.next += 1;
        // A whole symbol is whole puncture periods: the decoder takes it.
        let bits = soft.and_then(|(s, _)| f.viterbi.push(&s).ok());
        let Some(bits) = bits else {
            out.push(RxDataEvent::End(RxDataEnd::Truncated));
            return false;
        };
        for b in bits {
            if !Self::bit(f, b, out) {
                return false;
            }
        }
        true
    }

    /// One decoded bit: the seed, SERVICE, then the PSDU's octets; whether the frame goes on.
    fn bit(f: &mut Frame, b: u8, out: &mut Vec<RxDataEvent>) -> bool {
        let k = f.bits;
        f.bits += 1;
        if f.scrambler.is_none() {
            f.head.push(b);
            if f.head.len() < 7 {
                return true;
            }
            let Some((seed, sc)) =
                scrambler_seed_of(&f.head).and_then(|s| Scrambler::new(s).ok().map(|sc| (s, sc)))
            else {
                out.push(RxDataEvent::End(RxDataEnd::Seed));
                return false;
            };
            let mut sc = sc;
            for (j, &h) in f.head.iter().enumerate() {
                let c = sc.scramble(&[h]).first().copied().unwrap_or(0);
                f.service |= u16::from(c) << j;
            }
            f.scrambler = Some((seed, sc));
            return true;
        }
        let Some((seed, sc)) = f.scrambler.as_mut() else {
            return false;
        };
        let c = sc.scramble(&[b]).first().copied().unwrap_or(0);
        if k < 16 {
            f.service |= u16::from(c) << k;
            return true;
        }
        let at = (k - 16) % 8;
        f.octet |= c << at;
        if at < 7 {
            return true;
        }
        out.push(RxDataEvent::Octet {
            octet: f.octet,
            first: (f.octets == 0).then_some(FirstOctet {
                seed: *seed,
                service: f.service,
            }),
        });
        f.octet = 0;
        f.octets += 1;
        if f.octets == f.signal.length {
            out.push(RxDataEvent::End(RxDataEnd::Decoded));
            return false;
        }
        true
    }
}
