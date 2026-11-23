//! `fft64`'s G0 (`docs/gates/fft64.md`): the candidate architecture "radix-2 single-path delay
//! feedback, decimation in time, samples in natural order" computes `fft64_fixed` bit for bit.
//!
//! The model's transform (`radio_phy::rx_ofdm_fixed::fft64_fixed`) is radix 2, decimation in
//! time, in place on a bit-reversed array: stage s pairs positions p and p + 2^s. Label each
//! position by its bit reverse, l = rev6(p): the input x[l] then sits at label l, stage s
//! pairs labels l and l + 32/2^s, and bin k leaves at label rev6(k). So a stream in natural
//! order meets, at stage s, the two inputs of each butterfly 32/2^s samples apart: a delay
//! line of 32/2^s words per stage, the twiddle on the later of the two, the bins out in
//! bit-reversed order. The arithmetic of each butterfly is the model's (the same operands, the
//! same twiddle, the same narrowing); only where the values wait differs. This file is that
//! claim run, not argued: a sample-by-sample model of the six stages against `fft64_fixed`.
//!
//! The stream is in frames of 64 samples. A stage drains the last frame's differences while
//! the next frame's first half comes in, so after a window either the next window follows at
//! once or a frame of zeros flushes it (the RTL's choice; both are run here).

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::fixed::{quantize, Overflow, QFormat, Rounding};
use radio_phy::rx_front_fixed::IQ;
use radio_phy::rx_ofdm_fixed::{fft64_fixed, FixedOfdmConfig};
use std::collections::VecDeque;

const N: usize = 64;
const STAGES: usize = 6;

fn rev6(x: usize) -> usize {
    (x as u8).reverse_bits() as usize >> 2
}

/// v / 2^s, half to even (the model's `rsh_even`, restated from its doc).
fn rsh_even(v: i128, s: u32) -> i128 {
    if s == 0 {
        return v;
    }
    let floor = v >> s;
    let rem = v - (floor << s);
    let half = 1i128 << (s - 1);
    if rem > half || (rem == half && floor & 1 == 1) {
        floor + 1
    } else {
        floor
    }
}

fn sat(v: i128, bits: u32) -> i64 {
    let top = (1i128 << (bits - 1)) - 1;
    v.clamp(-top - 1, top) as i64
}

/// e^(−j2πk/64), k < 32, Q(T, T − 2), half to even (the model's ROM, from its doc).
fn rom(fx: &FixedOfdmConfig) -> Vec<IQ> {
    let f = QFormat::new(fx.twiddle_bits, fx.twiddle_bits - 2).unwrap();
    let q = |v: f64| quantize(v, f, Rounding::HalfEven, Overflow::Saturate).unwrap();
    (0..N / 2)
        .map(|k| {
            let ph = 2.0 * std::f64::consts::PI * k as f64 / N as f64;
            (q(ph.cos()), q(-ph.sin()))
        })
        .collect()
}

/// The twiddle index of the butterfly whose later input has label `l` at stage `s`: the
/// model's j·step with p = rev6(l), j = p mod 2^s, step = 32 / 2^s.
fn twiddle_index(l: usize, s: usize) -> usize {
    (rev6(l) & ((1 << s) - 1)) * (32 >> s)
}

/// One stage of the delay-feedback pipeline: D = 32 / 2^s words of delay.
struct Stage {
    s: usize,
    d: usize,
    fifo: VecDeque<IQ>,
    count: usize,
}

impl Stage {
    fn new(s: usize) -> Self {
        let d = 32 >> s;
        Self {
            s,
            d,
            fifo: std::iter::repeat_n((0, 0), d).collect(),
            // The stages before delay the labels by 32 + 16 + … = 64 − 2D samples.
            count: 2 * d,
        }
    }

    /// One sample in, one out (D samples late, in the same label order).
    fn step(&mut self, x: IQ, fx: &FixedOfdmConfig, rom: &[IQ]) -> IQ {
        let label = self.count % N;
        self.count += 1;
        let old = self.fifo.pop_front().unwrap();
        if label & self.d == 0 {
            // First half of a group of 2D: store the input, pass on last group's difference.
            self.fifo.push_back(x);
            return old;
        }
        let ft = fx.twiddle_bits - 2;
        let down = ft + u32::from(fx.fft_halve >> self.s & 1);
        let narrow = |t: i128| sat(rsh_even(t, down), fx.fft_bits);
        let w = rom[twiddle_index(label, self.s)];
        let (wr, wi) = (i128::from(w.0), i128::from(w.1));
        let (br, bi) = (i128::from(x.0), i128::from(x.1));
        let (tr, ti) = (wr * br - wi * bi, wr * bi + wi * br);
        let (ar, ai) = (i128::from(old.0) << ft, i128::from(old.1) << ft);
        self.fifo.push_back((narrow(ar - tr), narrow(ai - ti)));
        (narrow(ar + tr), narrow(ai + ti))
    }
}

/// The pipeline run over `frames` of 64 samples (a window, or `None` for a frame of zeros):
/// the bins of every window, in natural order.
fn sdf(frames: &[Option<Vec<IQ>>], fx: &FixedOfdmConfig) -> Vec<[IQ; N]> {
    let mut stages: Vec<Stage> = (0..STAGES).map(Stage::new).collect();
    sdf_on(&mut stages, frames, fx)
}

/// [`sdf`] on stages in whatever state they are (their counters at a frame's start).
fn sdf_on(stages: &mut [Stage], frames: &[Option<Vec<IQ>>], fx: &FixedOfdmConfig) -> Vec<[IQ; N]> {
    let rom = rom(fx);
    let mut out = Vec::new();
    let delay = N - 1;
    let mut t = 0usize;
    let mut bins = [(0i64, 0i64); N];
    for frame in frames.iter().chain(std::iter::once(&None)) {
        for i in 0..N {
            let x = frame.as_ref().map_or((0, 0), |w| {
                (
                    sat(i128::from(w[i].0), fx.fft_bits),
                    sat(i128::from(w[i].1), fx.fft_bits),
                )
            });
            let y = stages.iter_mut().fold(x, |v, st| st.step(v, fx, &rom));
            if t >= delay {
                let label = (t - delay) % N;
                bins[rev6(label)] = y;
                if label == N - 1 {
                    let src = (t - delay) / N;
                    if frames[src].is_some() {
                        out.push(bins);
                    }
                }
            }
            t += 1;
        }
    }
    out
}

fn window(rng: &mut Rng, bits: u32) -> Vec<IQ> {
    let m = 1i64 << (bits - 1);
    let r = |rng: &mut Rng| (rng.next_u64() % (2 * m as u64)) as i64 - m;
    (0..N).map(|_| (r(rng), r(rng))).collect()
}

#[test]
fn the_delay_feedback_pipeline_is_the_models_transform_bit_for_bit() {
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0xf764);
    // Windows at the hand-off's width shifted into the word (the receiver's input), and at
    // the word's full range (every stage saturating somewhere).
    for &bits in &[fx.in_bits + fx.in_shift, fx.fft_bits] {
        let ws: Vec<Vec<IQ>> = (0..400).map(|_| window(&mut rng, bits)).collect();
        // Back to back, and each window followed by a frame of zeros.
        let joined: Vec<Option<Vec<IQ>>> = ws.iter().cloned().map(Some).collect();
        let flushed: Vec<Option<Vec<IQ>>> =
            ws.iter().flat_map(|w| [Some(w.clone()), None]).collect();
        for frames in [&joined, &flushed] {
            let got = sdf(frames, &fx);
            assert_eq!(got.len(), ws.len());
            for (w, b) in ws.iter().zip(&got) {
                assert_eq!(&fft64_fixed(w, &fx).unwrap(), b, "{bits} bits");
            }
        }
    }
}

#[test]
fn an_abort_mid_window_needs_only_the_counters_reset() {
    // ADR 0010, decision 2: an abort drops what is in flight. Cut a window after c samples,
    // put every stage's label counter back to a frame's start and leave the delay lines as
    // they are: the next windows' bins are still the model's.
    let fx = FixedOfdmConfig::default();
    let mut rng = Rng::new(0xab07);
    let bits = fx.in_bits + fx.in_shift;
    for cut in 1..N {
        let mut stages: Vec<Stage> = (0..STAGES).map(Stage::new).collect();
        let first = window(&mut rng, bits);
        let rom = rom(&fx);
        for x in first.iter().take(cut) {
            stages.iter_mut().fold(*x, |v, st| st.step(v, &fx, &rom));
        }
        for st in stages.iter_mut() {
            st.count = 2 * st.d;
        }
        let ws: Vec<Vec<IQ>> = (0..3).map(|_| window(&mut rng, bits)).collect();
        let frames: Vec<Option<Vec<IQ>>> = vec![
            Some(ws[0].clone()),
            None,
            Some(ws[1].clone()),
            Some(ws[2].clone()),
        ];
        let got = sdf_on(&mut stages, &frames, &fx);
        assert_eq!(got.len(), 3, "cut {cut}");
        for (w, b) in ws.iter().zip(&got) {
            assert_eq!(&fft64_fixed(w, &fx).unwrap(), b, "cut {cut}");
        }
    }
}

#[test]
fn the_twiddles_each_stage_needs() {
    // What the multipliers have to be: stages 0 and 1 multiply by 1 and −j only (no
    // multiplier), stage 2 by W⁰, W⁸, W¹⁶, W²⁴ (one constant, cos π/4), stage 3 by the eight
    // multiples of 4 (two constants and cos π/4), stages 4 and 5 by a general ROM.
    let expect: [&[usize]; STAGES] = [
        &[0],
        &[0, 16],
        &[0, 8, 16, 24],
        &[0, 4, 8, 12, 16, 20, 24, 28],
        &(0..32).step_by(2).collect::<Vec<_>>(),
        &(0..32).collect::<Vec<_>>(),
    ];
    for (s, e) in expect.iter().enumerate() {
        let d = 32 >> s;
        let mut used: Vec<usize> = (0..N)
            .filter(|l| l & d != 0)
            .map(|l| twiddle_index(l, s))
            .collect();
        used.sort_unstable();
        used.dedup();
        assert_eq!(&used, e, "stage {s}");
    }
    let fx = FixedOfdmConfig::default();
    let r = rom(&fx);
    // 1 and −j are exact in Q(12, 10): their products are shifts, no rounding is lost.
    assert_eq!(r[0], (1024, 0));
    assert_eq!(r[16], (0, -1024));
    assert_eq!(r[8], (724, -724));
}
