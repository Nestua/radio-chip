//! Channel simulator: a seeded RNG, AWGN, CFO, phase noise, front-end impairments (DC, CW tone,
//! gain step), fractional delay, SFO, fading and TGn multipath.
//!
//! Simulation only: this is the air between our transmitter and receiver models, never
//! hardware, so it is plain `f64` and has no fixed-point counterpart. Samples are complex
//! `(re, im)` pairs. Definitions (SNR over the full sampled band, ppm offsets, the Wiener phase
//! model) are in `docs/gates/channel.md`.

use std::f64::consts::PI;

/// A complex baseband sample.
pub type Sample = (f64, f64);

/// Deterministic pseudo-random generator: xoshiro256** seeded through SplitMix64.
///
/// One seed gives one stream on every platform, so every simulation is reproducible.
/// Not cryptographic.
#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

impl Rng {
    /// A generator whose whole state is derived from `seed` (SplitMix64, so any seed, including
    /// 0, gives a well-mixed non-zero state).
    pub fn new(seed: u64) -> Self {
        let mut x = seed;
        let mut split = || {
            x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut z = x;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            z ^ (z >> 31)
        };
        Self {
            s: [split(), split(), split(), split()],
            spare: None,
        }
    }

    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let out = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        out
    }

    /// Uniform in [0, 1), 53 random bits.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal N(0, 1), Box–Muller (two per pair of uniforms; the second is kept).
    pub fn gaussian(&mut self) -> f64 {
        if let Some(g) = self.spare.take() {
            return g;
        }
        // 1 − u lies in (0, 1], so the logarithm is finite.
        let r = (-2.0 * (1.0 - self.uniform()).ln()).sqrt();
        let a = 2.0 * PI * self.uniform();
        self.spare = Some(r * a.sin());
        r * a.cos()
    }
}

/// Complementary error function, relative error ≲ 1e-14 over the whole real line.
///
/// Below 1.5: 1 − erf with erf from its all-positive series
/// erf(x) = 2/√π·e^(−x²)·Σ 2ⁿx^(2n+1)/(1·3·…·(2n+1)). From 1.5 up: the continued fraction
/// erfc(x) = e^(−x²)/√π · 1/(x + ½/(x + 1/(x + 3/2/(x + …)))) by the modified Lentz method.
pub fn erfc(x: f64) -> f64 {
    if x.is_nan() {
        return f64::NAN;
    }
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    if x < 1.5 {
        let x2 = x * x;
        let (mut term, mut sum, mut n) = (x, x, 0.0);
        while term > sum * 1e-17 {
            n += 1.0;
            term *= 2.0 * x2 / (2.0 * n + 1.0);
            sum += term;
        }
        return 1.0 - 2.0 / PI.sqrt() * exp_neg_square(x) * sum;
    }
    if x > 27.3 {
        return 0.0; // below the smallest subnormal
    }
    // Modified Lentz for b0 + a1/(b1 + a2/(b2 + …)), all b = x, a_n = n/2.
    let tiny = 1e-300;
    let mut f = x;
    let (mut c, mut d) = (f, 0.0);
    for n in 1..10_000 {
        let a = f64::from(n) / 2.0;
        d = x + a * d;
        d = if d.abs() < tiny { tiny } else { d };
        c = x + a / c;
        c = if c.abs() < tiny { tiny } else { c };
        d = 1.0 / d;
        let delta = c * d;
        f *= delta;
        if (delta - 1.0).abs() < 1e-16 {
            break;
        }
    }
    exp_neg_square(x) / (PI.sqrt() * f)
}

/// e^(−x²) without the rounding error of forming x² (which costs ~x²·ε relative for large x):
/// split x = h + l with h on a 1/16 grid, so h² is exact, and e^(−x²) = e^(−h²)·e^(−l(x+h)).
fn exp_neg_square(x: f64) -> f64 {
    let h = (x * 16.0).trunc() / 16.0;
    let l = x - h;
    (-h * h).exp() * (-l * (x + h)).exp()
}

/// Gaussian tail probability Q(x) = P(N(0,1) > x) = erfc(x/√2)/2.
pub fn q(x: f64) -> f64 {
    0.5 * erfc(x / std::f64::consts::SQRT_2)
}

/// Mean power of a slice, (1/n)·Σ|x|². 0 for an empty slice.
pub fn mean_power(x: &[Sample]) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    x.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / x.len() as f64
}

/// Adds complex white Gaussian noise of power `signal_power / 10^(snr_db/10)` (half per axis).
/// SNR is over the whole sampled band; see `docs/gates/channel.md` for the per-subcarrier SNR.
pub fn awgn(x: &[Sample], snr_db: f64, signal_power: f64, rng: &mut Rng) -> Vec<Sample> {
    let sigma = (signal_power / 10f64.powf(snr_db / 10.0) / 2.0).sqrt();
    x.iter()
        .map(|s| (s.0 + sigma * rng.gaussian(), s.1 + sigma * rng.gaussian()))
        .collect()
}

/// Rotates the signal by a carrier frequency offset: x[n]·e^(j(2π·Δf·n/fs + φ0)).
pub fn frequency_offset(
    x: &[Sample],
    offset_hz: f64,
    sample_rate_hz: f64,
    phase0: f64,
) -> Vec<Sample> {
    let step = 2.0 * PI * offset_hz / sample_rate_hz;
    x.iter()
        .enumerate()
        // Phase from n directly (not accumulated), so long frames do not drift by rounding.
        .map(|(n, s)| rotate(*s, phase0 + step * n as f64))
        .collect()
}

fn rotate(s: Sample, phase: f64) -> Sample {
    let (sin, cos) = phase.sin_cos();
    (s.0 * cos - s.1 * sin, s.0 * sin + s.1 * cos)
}

/// A frequency error in ppm of a carrier, in Hz. IEEE 802.11-2024 §17.3.9.5 allows ±20 ppm per
/// station at 20 and 10 MHz, ±10 ppm at 5 MHz.
pub fn ppm_to_hz(ppm: f64, carrier_hz: f64) -> f64 {
    ppm * 1e-6 * carrier_hz
}

/// 3 dB linewidth β of a Wiener-phase oscillator whose single-sideband phase noise is
/// `dbc_per_hz` at `offset_hz` from the carrier: L(f) = β/(2πf²), so β = 2π·f²·10^(L/10).
pub fn linewidth_from_dbc(dbc_per_hz: f64, offset_hz: f64) -> f64 {
    2.0 * PI * offset_hz * offset_hz * 10f64.powf(dbc_per_hz / 10.0)
}

/// A Wiener phase path of `len` samples: φ[n] = φ[n−1] + N(0, 2πβ/fs), φ[−1] = 0, so φ[n]
/// has variance 2πβ·(n+1)/fs.
pub fn wiener_phase(len: usize, linewidth_hz: f64, sample_rate_hz: f64, rng: &mut Rng) -> Vec<f64> {
    let sigma = (2.0 * PI * linewidth_hz / sample_rate_hz).sqrt();
    let mut phi = 0.0;
    (0..len)
        .map(|_| {
            phi += sigma * rng.gaussian();
            phi
        })
        .collect()
}

/// Multiplies the signal by e^(jφ[n]) with φ a fresh Wiener phase path of linewidth β.
pub fn phase_noise(
    x: &[Sample],
    linewidth_hz: f64,
    sample_rate_hz: f64,
    rng: &mut Rng,
) -> Vec<Sample> {
    let phi = wiener_phase(x.len(), linewidth_hz, sample_rate_hz, rng);
    x.iter().zip(&phi).map(|(s, p)| rotate(*s, *p)).collect()
}

// ---------------------------------------------------------------------------------------------
// Front-end impairments (R0-07c): what a zero-IF receiver adds before the digital front end.
// ---------------------------------------------------------------------------------------------

/// Adds a continuous-wave tone of mean power `power` at `freq_hz`:
/// x[n] + √power·e^(j(2π·f·n/fs + φ0)).
///
/// At 0 Hz this is a DC offset (LO leakage, self-mixing in a zero-IF receiver) of value
/// √power·e^(jφ0); elsewhere a spur or a narrowband interferer.
pub fn add_tone(
    x: &[Sample],
    power: f64,
    freq_hz: f64,
    sample_rate_hz: f64,
    phase0: f64,
) -> Vec<Sample> {
    let amp = power.sqrt();
    let step = 2.0 * PI * freq_hz / sample_rate_hz;
    x.iter()
        .enumerate()
        .map(|(n, s)| {
            let (sin, cos) = (phase0 + step * n as f64).sin_cos();
            (s.0 + amp * cos, s.1 + amp * sin)
        })
        .collect()
}

/// An AGC gain change: samples from index `at` on are scaled by `gain_db` (amplitude
/// 10^(g/20)); earlier samples are unchanged. `at ≥ len` leaves the signal as it is.
///
/// The step is instantaneous — the worst case for the estimators that must be frozen across
/// it (`docs/gates/rx_front.md`). Compose with `add_tone` to model a DC offset that changes
/// with gain.
pub fn gain_step(x: &[Sample], at: usize, gain_db: f64) -> Vec<Sample> {
    let g = 10f64.powf(gain_db / 20.0);
    x.iter()
        .enumerate()
        .map(|(n, s)| if n < at { *s } else { (s.0 * g, s.1 * g) })
        .collect()
}

/// One state of a receive chain while its gain control settles: the state holds for the
/// samples before `until`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GainState {
    /// First sample of the next state.
    pub until: usize,
    /// Gain relative to the settled chain, dB (amplitude 10^(g/20)).
    pub gain_db: f64,
    /// Carrier phase relative to the settled chain, radians (a gain stage switched in or out
    /// changes the phase of the path).
    pub phase: f64,
    /// DC offset added after the gain (it changes with the gain state in a zero-IF receiver).
    pub dc: Sample,
}

/// An AGC settling transient: sample n takes the first state with `n < until`; samples after the
/// last state are unchanged (the settled chain). `states` must be ordered by `until`. Steps are
/// instantaneous. Clipping is the ADC's: feed the result to a saturating quantiser.
///
/// What a fast-attack AGC does to the start of a packet: it sits at high gain on noise, the
/// packet clips, and one to three gain steps follow inside the short training field, each with
/// its own phase and DC offset.
pub fn agc_transient(x: &[Sample], states: &[GainState]) -> Vec<Sample> {
    let mut k = 0;
    x.iter()
        .enumerate()
        .map(|(n, s)| {
            while k < states.len() && n >= states[k].until {
                k += 1;
            }
            match states.get(k) {
                None => *s,
                Some(st) => {
                    let g = 10f64.powf(st.gain_db / 20.0);
                    let (sin, cos) = st.phase.sin_cos();
                    (
                        g * (s.0 * cos - s.1 * sin) + st.dc.0,
                        g * (s.0 * sin + s.1 * cos) + st.dc.1,
                    )
                }
            }
        })
        .collect()
}

/// A fast-attack gain control as the AD9361's is published (UG-570, "Fast Attack AGC Mode";
/// the table of what is taken from it and what is defined here is in
/// `docs/gates/channel.md`, R0-07e). Simulation only. Times are in samples, levels in dB with
/// 0 dBFS a mean |v|² of 1 after the gain, peaks as max(|I|, |Q|) with 1.0 the ADC's full
/// scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FastAttackAgc {
    /// Largest gain, dB: where the control starts and returns to.
    pub max_gain_db: f64,
    /// Smallest gain, dB.
    pub min_gain_db: f64,
    /// Peak at or above which both overload detectors fire (the analogue one too).
    pub both_peak: f64,
    /// Peak at or above which the large overload fires.
    pub large_peak: f64,
    /// Peak at or above which the small overload fires.
    pub small_peak: f64,
    /// Decrease when both detectors fire, dB (Table 20, case 1).
    pub step_both: f64,
    /// Decrease on a large overload, dB (case 2).
    pub step_large: f64,
    /// Decrease on a small overload, dB (case 3).
    pub step_small: f64,
    /// Samples after a gain change in which the peak detectors are held in reset.
    pub peak_wait: usize,
    /// Samples without overload after which peak detection ends.
    pub energy_detect: usize,
    /// Samples from the last change to the start of the power measurement.
    pub settling_delay: usize,
    /// Samples of the power measurement.
    pub power_samples: usize,
    /// Level the lock change aims at, dBFS.
    pub lock_dbfs: f64,
    /// Samples of the power measurement while locked: the power is compared once per
    /// window, counted from the lock.
    pub lock_window: usize,
    /// Samples a power comparison must stand to unlock the gain.
    pub exit_count: usize,
    /// Rise of the power that unlocks, dB.
    pub stronger_db: f64,
    /// Fall of the power that unlocks, dB.
    pub lost_db: f64,
    /// Whether a large overload unlocks at once.
    pub unlock_on_overload: bool,
    /// Whether an unlocked gain returns to the maximum (or stays).
    pub unlock_to_max: bool,
}

/// What [`FastAttackAgc::run`] did.
#[derive(Debug, Clone, PartialEq)]
pub struct AgcTrace {
    /// The gain applied to each sample, dB.
    pub gain_db: Vec<f64>,
    /// (first sample, gain) of every stretch of constant gain; the first is (0, maximum).
    pub changes: Vec<(usize, f64)>,
    /// First samples of the locked stretches.
    pub locks: Vec<usize>,
    /// Samples at which the lock was left.
    pub unlocks: Vec<usize>,
}

impl FastAttackAgc {
    /// The AD9361's register reset values (UG-672) at `per_sample` detector clocks (ClkRF)
    /// per sample, counts rounded up: 4, 8 and 20 clocks, 512 samples of power (64 while
    /// locked), lock level −10 dBFS, 10 dB to unlock over 16 clocks, steps of 5, 2 and 2 dB.
    /// The peak thresholds are the model's: full scale, 1 dB below it, and 6 dB above it for
    /// both detectors. The overload exit is off (the documents are not clear on its reset
    /// state, `docs/gates/channel.md`). A receiver of short packets shortens the power
    /// measurement: `FastAttackAgc { power_samples: 16, ..FastAttackAgc::ad9361_reset(2) }`.
    pub fn ad9361_reset(per_sample: usize) -> Self {
        let per = per_sample.max(1);
        Self {
            max_gain_db: 73.0,
            min_gain_db: 0.0,
            both_peak: 2.0,
            large_peak: 1.0,
            small_peak: 0.891,
            step_both: 5.0,
            step_large: 2.0,
            step_small: 2.0,
            peak_wait: 4usize.div_ceil(per),
            energy_detect: 8usize.div_ceil(per),
            settling_delay: 20usize.div_ceil(per),
            power_samples: 512,
            lock_dbfs: -10.0,
            lock_window: 64,
            exit_count: 16usize.div_ceil(per),
            stronger_db: 10.0,
            lost_db: 10.0,
            unlock_on_overload: false,
            unlock_to_max: true,
        }
    }

    /// The gain the control gives each sample of `x` (the signal before the gain), starting
    /// in peak detection at the maximum gain. A decision at sample n acts from n + 1.
    pub fn run(&self, x: &[Sample]) -> AgcTrace {
        self.run_dc(x, None)
    }

    /// [`FastAttackAgc::run`] with the chain's DC offset at each gain added after the gain,
    /// where the detectors measure: an offset is a signal to them like any other.
    pub fn run_dc(&self, x: &[Sample], dc: Option<&GainDc>) -> AgcTrace {
        enum State {
            /// Peak detection and power measurement; `since` is the first sample at the
            /// present gain (or of the state), `look` the first sample the peak detectors
            /// examine (they are held in reset after a change), `sum` the power so far.
            Seeking { since: usize, look: usize, sum: f64 },
            /// Locked from `since`, aiming at `level` (power); `stands` is the last power
            /// comparison (1 stronger, −1 lost, 0 neither), `stood` the samples it has stood.
            Locked {
                since: usize,
                level: f64,
                stands: i64,
                stood: usize,
            },
        }
        let mut gain = self.max_gain_db;
        let mut t = AgcTrace {
            gain_db: Vec::with_capacity(x.len()),
            changes: vec![(0, gain)],
            locks: Vec::new(),
            unlocks: Vec::new(),
        };
        let mut state = State::Seeking {
            since: 0,
            look: 0,
            sum: 0.0,
        };
        // Power after the gain, per sample, for the locked state's window.
        let mut powers: Vec<f64> = Vec::with_capacity(x.len());
        let mut offset = dc.map_or((0.0, 0.0), |d| d.at(gain));
        for (n, s) in x.iter().enumerate() {
            t.gain_db.push(gain);
            let g = 10f64.powf(gain / 20.0);
            // A sample that is not a number is no signal.
            let finite = |v: f64| if v.is_finite() { v } else { 0.0 };
            let (re, im) = (finite(g * s.0 + offset.0), finite(g * s.1 + offset.1));
            let peak = re.abs().max(im.abs());
            let power = re * re + im * im;
            powers.push(power);
            let mut next = gain;
            match &mut state {
                State::Seeking { since, look, sum } => {
                    let step = if n < *look {
                        None
                    } else if peak >= self.both_peak {
                        Some(self.step_both)
                    } else if peak >= self.large_peak {
                        Some(self.step_large)
                    } else if peak >= self.small_peak {
                        Some(self.step_small)
                    } else {
                        None
                    };
                    let measure_from = *since + self.settling_delay.max(self.energy_detect);
                    if let Some(step) = step {
                        // An overload at any point of the search starts it again.
                        next = (gain - step).max(self.min_gain_db);
                        state = State::Seeking {
                            since: n + 1,
                            look: n + 1 + self.peak_wait,
                            sum: 0.0,
                        };
                    } else if n >= measure_from {
                        *sum += power;
                        if n + 1 == measure_from + self.power_samples.max(1) {
                            let mean = *sum / self.power_samples.max(1) as f64;
                            let change = (self.lock_dbfs - 10.0 * mean.log10()).round();
                            // Not `clamp`: it panics on a range given the wrong way round.
                            next = (gain + change).min(self.max_gain_db).max(self.min_gain_db);
                            t.locks.push(n + 1);
                            state = State::Locked {
                                since: n + 1,
                                level: mean * 10f64.powf((next - gain) / 10.0),
                                stands: 0,
                                stood: 0,
                            };
                        }
                    }
                }
                State::Locked {
                    since,
                    level,
                    stands,
                    stood,
                } => {
                    let w = self.lock_window.max(1);
                    let mut unlock = self.unlock_on_overload && peak >= self.large_peak;
                    // The power value updates once per window, counted from the lock; the
                    // comparison stands until the next update.
                    if (n + 1 - *since).is_multiple_of(w) {
                        let mean = powers[n + 1 - w..=n].iter().sum::<f64>() / w as f64;
                        // A lock on nothing (level 0) is left by any power, never by less.
                        let dir = if mean > 0.0
                            && mean >= *level * 10f64.powf(self.stronger_db / 10.0)
                        {
                            1
                        } else if *level > 0.0 && mean <= *level * 10f64.powf(-self.lost_db / 10.0)
                        {
                            -1
                        } else {
                            0
                        };
                        if dir != *stands {
                            (*stands, *stood) = (dir, 0);
                        }
                    }
                    if *stands != 0 {
                        // Samples the comparison has stood, the update's own included.
                        *stood += 1;
                        unlock |= *stood >= self.exit_count.max(1);
                    }
                    if unlock {
                        t.unlocks.push(n);
                        if self.unlock_to_max {
                            next = self.max_gain_db;
                        }
                        state = State::Seeking {
                            since: n + 1,
                            look: n + 1,
                            sum: 0.0,
                        };
                    }
                }
            }
            if next != gain {
                gain = next;
                offset = dc.map_or((0.0, 0.0), |d| d.at(gain));
                t.changes.push((n + 1, gain));
            }
        }
        // A change decided at the last sample has no sample to act on.
        t.changes.retain(|c| c.0 < x.len().max(1));
        t.locks.retain(|l| *l < x.len());
        t
    }
}

impl AgcTrace {
    /// The trace as [`agc_transient`] takes it: the gain the chain ends at, dB, and one
    /// state per stretch before the last change, its gain relative to that. Each change
    /// turns the carrier phase by an angle uniform in ±`jump_deg` (an assumption: nothing is
    /// published); the phases are relative to the settled chain.
    pub fn gain_states(&self, jump_deg: f64, rng: &mut Rng) -> (f64, Vec<GainState>) {
        let settled = self.changes.last().map_or(0.0, |c| c.1);
        let mut phase = 0.0;
        let mut states: Vec<GainState> = self
            .changes
            .windows(2)
            .rev()
            .map(|w| {
                phase += (2.0 * rng.uniform() - 1.0) * jump_deg.to_radians();
                GainState {
                    until: w[1].0,
                    gain_db: w[0].1 - settled,
                    phase,
                    dc: (0.0, 0.0),
                }
            })
            .collect();
        states.reverse();
        (settled, states)
    }

    /// [`AgcTrace::gain_states`] with the DC offset of every stretch, the settled one
    /// included: a last state without end carries it (gain 0 dB, phase 0). The offsets are
    /// in the units of the signal after the gain, as [`agc_transient`] adds them.
    pub fn gain_states_dc(
        &self,
        jump_deg: f64,
        dc: &GainDc,
        rng: &mut Rng,
    ) -> (f64, Vec<GainState>) {
        let (settled, mut states) = self.gain_states(jump_deg, rng);
        for s in &mut states {
            s.dc = dc.at(settled + s.gain_db);
        }
        states.push(GainState {
            until: usize::MAX,
            gain_db: 0.0,
            phase: 0.0,
            dc: dc.at(settled),
        });
        (settled, states)
    }
}

/// The DC offset of a zero-IF receive chain as a function of its gain (R0-07e; what is
/// published and what is assumed: `docs/gates/channel.md`). Two parts, after the gain, in the
/// units of the signal there (1.0 the ADC's full scale where the caller says so).
///
/// The AD9361 stores a correction per gain index and applies it when the index is used
/// (UG-570 Rev. A, pp. 11–12), so what a calibrated chain leaves is a property of the
/// index: `per_index`. What nothing corrects and arises before the gain grows with it:
/// `before_gain`. No level of either is published.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GainDc {
    /// Magnitude of the offset each gain index leaves; its direction is drawn once per
    /// index (of 1 dB) from `seed`, uniform.
    pub per_index: f64,
    /// The table of directions.
    pub seed: u64,
    /// An offset at the input of the gain: 10^(g/20) times this after it.
    pub before_gain: Sample,
}

impl GainDc {
    /// The offset after a gain of `gain_db`.
    pub fn at(&self, gain_db: f64) -> Sample {
        // Indices of 1 dB, as the chip's gain table; a gain that is not a number is index 0.
        let index = if gain_db.is_finite() {
            gain_db.round() as i64
        } else {
            0
        };
        let mut rng = Rng::new(self.seed ^ (index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15));
        let (sin, cos) = (2.0 * PI * rng.uniform()).sin_cos();
        let g = 10f64.powf(gain_db / 20.0);
        let before = if g.is_finite() {
            (self.before_gain.0 * g, self.before_gain.1 * g)
        } else {
            (0.0, 0.0)
        };
        (
            self.per_index * cos + before.0,
            self.per_index * sin + before.1,
        )
    }
}

// ---------------------------------------------------------------------------------------------
// Part II (R0-07b): FFT, fractional delay, sample-clock offset, fading taps, TGn multipath.
// ---------------------------------------------------------------------------------------------

/// The FFT length is not a power of two.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotPowerOfTwo(pub usize);

impl std::fmt::Display for NotPowerOfTwo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FFT length {} is not a power of two", self.0)
    }
}

impl std::error::Error for NotPowerOfTwo {}

/// Forward DFT, X_k = Σ x_n·e^(−j2πkn/N), radix-2 (N a power of two). Simulation tool only.
pub fn fft(x: &[Sample]) -> Result<Vec<Sample>, NotPowerOfTwo> {
    transform(x, -1.0)
}

/// Inverse DFT with the 1/N factor, so `ifft(fft(x)) = x`.
pub fn ifft(x: &[Sample]) -> Result<Vec<Sample>, NotPowerOfTwo> {
    let n = x.len() as f64;
    Ok(transform(x, 1.0)?
        .into_iter()
        .map(|s| (s.0 / n, s.1 / n))
        .collect())
}

fn transform(x: &[Sample], sign: f64) -> Result<Vec<Sample>, NotPowerOfTwo> {
    let n = x.len();
    if !n.is_power_of_two() {
        return Err(NotPowerOfTwo(n));
    }
    let bits = n.trailing_zeros();
    let mut a: Vec<Sample> = (0..n)
        .map(|i| {
            x[if bits == 0 {
                0
            } else {
                i.reverse_bits() >> (usize::BITS - bits)
            }]
        })
        .collect();
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        for k in 0..half {
            // Twiddles from sin/cos directly, not by recurrence, to keep rounding at ~ε.
            let w = rotate((1.0, 0.0), sign * 2.0 * PI * k as f64 / len as f64);
            for start in (0..n).step_by(len) {
                let (u, v) = (a[start + k], a[start + k + half]);
                let t = (v.0 * w.0 - v.1 * w.1, v.0 * w.1 + v.1 * w.0);
                a[start + k] = (u.0 + t.0, u.1 + t.1);
                a[start + k + half] = (u.0 - t.0, u.1 - t.1);
            }
        }
        len *= 2;
    }
    Ok(a)
}

/// Half-length of the interpolation kernel (taps on each side).
pub const INTERP_HALF: usize = 24;
const KAISER_BETA: f64 = 8.0;

/// Modified Bessel function I0 by its power series (converges fast for the β used here).
fn bessel_i0(x: f64) -> f64 {
    let (mut term, mut sum) = (1.0f64, 1.0f64);
    let q = x * x / 4.0;
    for k in 1..60 {
        term *= q / f64::from(k * k);
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

/// Value of the band-limited signal behind `x` at real time `t` (samples), zero outside `x`:
/// Kaiser-windowed sinc over ±[`INTERP_HALF`] samples.
fn interpolate(x: &[Sample], t: f64) -> Sample {
    let base = t.floor();
    let frac = t - base;
    let base = base as i64;
    if frac == 0.0 {
        return usize::try_from(base)
            .ok()
            .and_then(|i| x.get(i).copied())
            .unwrap_or((0.0, 0.0));
    }
    let half = INTERP_HALF as i64;
    let norm = bessel_i0(KAISER_BETA);
    let mut acc = (0.0, 0.0);
    for j in (base - half + 1)..=(base + half) {
        let Some(s) = usize::try_from(j).ok().and_then(|i| x.get(i)) else {
            continue;
        };
        let u = t - j as f64; // in (−half, half)
        let r = u / half as f64;
        let w = bessel_i0(KAISER_BETA * (1.0 - r * r).max(0.0).sqrt()) / norm;
        let c = (PI * u).sin() / (PI * u) * w;
        acc = (acc.0 + c * s.0, acc.1 + c * s.1);
    }
    acc
}

/// Delays a band-limited signal by `delay` samples (any real number): y[n] = x(n − delay), zero
/// before the signal starts. Same length as `x`.
pub fn fractional_delay(x: &[Sample], delay: f64) -> Vec<Sample> {
    (0..x.len())
        .map(|n| interpolate(x, n as f64 - delay))
        .collect()
}

/// What a receiver reads when the transmitter's clock is `ppm` fast against its own (or its
/// own is slow): y[n] = x(n·(1 + ppm·1e-6)), every frequency of the signal scaled up by the
/// same factor. Same length as `x` (zero past its end). Use with [`frequency_offset`] at
/// +ppm of the carrier, since one oscillator drives both (IEEE 802.11-2024 §17.3.9.5–6).
/// (This comment used to say "a receiver whose sample clock is ppm fast": the opposite.)
pub fn resample(x: &[Sample], ppm: f64) -> Vec<Sample> {
    let ratio = 1.0 + ppm * 1e-6;
    (0..x.len())
        .map(|n| interpolate(x, n as f64 * ratio))
        .collect()
}

/// Doppler power spectrum of a fading tap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Doppler {
    /// No time variation: one Gaussian draw per tap.
    Static,
    /// TGn "Bell" spectrum S(f) = 1/(1 + 9·(f/fd)²), |f| ≤ 5·fd (IEEE 802.11-03/940r4 §4.7.1).
    Bell {
        /// Doppler spread fd in Hz (TGn: v₀/λ, v₀ = 1.2 km/h).
        fd_hz: f64,
    },
    /// Clarke/Jakes spectrum S(f) = 1/(π·fd·√(1 − (f/fd)²)), |f| < fd: a moving terminal with
    /// uniform scattering; autocorrelation J0(2π·fd·τ).
    Jakes {
        /// Maximum Doppler shift fd = v/λ in Hz.
        fd_hz: f64,
    },
}

impl Doppler {
    /// Highest frequency in the spectrum (0 for `Static`).
    fn max_hz(self) -> f64 {
        match self {
            Doppler::Static => 0.0,
            Doppler::Bell { fd_hz } => 5.0 * fd_hz,
            Doppler::Jakes { fd_hz } => fd_hz,
        }
    }

    /// Normalised cumulative spectrum: the fraction of the power below `f`.
    fn cdf(self, f: f64) -> f64 {
        match self {
            Doppler::Static => {
                if f < 0.0 {
                    0.0
                } else {
                    1.0
                }
            }
            Doppler::Bell { fd_hz } => {
                let edge = 5.0 * fd_hz;
                let f = f.clamp(-edge, edge);
                // ∫ 1/(1 + 9(f/fd)²) df = (fd/3)·atan(3f/fd)
                ((3.0 * f / fd_hz).atan() + 15f64.atan()) / (2.0 * 15f64.atan())
            }
            Doppler::Jakes { fd_hz } => {
                let f = f.clamp(-fd_hz, fd_hz);
                0.5 + (f / fd_hz).asin() / PI
            }
        }
    }
}

/// Maximum Doppler shift fd = v·fc/c for a terminal moving at `speed_mps`.
pub fn doppler_hz(speed_mps: f64, carrier_hz: f64) -> f64 {
    speed_mps * carrier_hz / 299_792_458.0
}

/// One tap of a tapped-delay-line channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tap {
    /// Excess delay in seconds.
    pub delay_s: f64,
    /// Mean power (linear; a profile's taps usually sum to 1).
    pub power: f64,
    /// Rician K-factor, linear (0 = Rayleigh).
    pub k_factor: f64,
    /// Doppler shift of the line-of-sight component in Hz (ignored when K = 0).
    pub los_doppler_hz: f64,
}

/// The TGn channel models (IEEE 802.11-03/940r4), SISO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TgnModel {
    /// Flat fading, one tap (optional model, stress test only).
    A,
    /// Residential, 15 ns rms delay spread.
    B,
    /// Small office, 30 ns.
    C,
    /// Typical office, 50 ns.
    D,
    /// Large office, 100 ns.
    E,
    /// Large open space, 150 ns.
    F,
}

/// A cluster of a TGn profile: index of its first tap and its powers in dB.
type Cluster = (usize, &'static [f64]);

// IEEE 802.11-03/940r4 Appendix C: tap delays (ns) and per-cluster powers (dB) with the index of
// each cluster's first tap. Checked against the document by `g1_channel_fading` through
// `tools/ref/tgn_pdp_extract.py`.
const TGN_B_DELAYS_NS: [f64; 9] = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0];
const TGN_B_CLUSTERS: [Cluster; 2] = [
    (0, &[0.0, -5.4, -10.8, -16.2, -21.7]),
    (2, &[-3.2, -6.3, -9.4, -12.5, -15.6, -18.7, -21.8]),
];
const TGN_C_DELAYS_NS: [f64; 14] = [
    0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 110.0, 140.0, 170.0, 200.0,
];
const TGN_C_CLUSTERS: [Cluster; 2] = [
    (
        0,
        &[
            0.0, -2.1, -4.3, -6.5, -8.6, -10.8, -13.0, -15.2, -17.3, -19.5,
        ],
    ),
    (6, &[-5.0, -7.2, -9.3, -11.5, -13.7, -15.8, -18.0, -20.2]),
];
const TGN_D_DELAYS_NS: [f64; 18] = [
    0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 110.0, 140.0, 170.0, 200.0, 240.0,
    290.0, 340.0, 390.0,
];
const TGN_D_CLUSTERS: [Cluster; 3] = [
    (
        0,
        &[
            0.0, -0.9, -1.7, -2.6, -3.5, -4.3, -5.2, -6.1, -6.9, -7.8, -9.0, -11.1, -13.7, -16.3,
            -19.3, -23.2,
        ],
    ),
    (10, &[-6.6, -9.5, -12.1, -14.7, -17.4, -21.9, -25.5]),
    (14, &[-18.8, -23.2, -25.2, -26.7]),
];
const TGN_E_DELAYS_NS: [f64; 18] = [
    0.0, 10.0, 20.0, 30.0, 50.0, 80.0, 110.0, 140.0, 180.0, 230.0, 280.0, 330.0, 380.0, 430.0,
    490.0, 560.0, 640.0, 730.0,
];
const TGN_E_CLUSTERS: [Cluster; 4] = [
    (
        0,
        &[
            -2.6, -3.0, -3.5, -3.9, -4.5, -5.6, -6.9, -8.2, -9.8, -11.7, -13.9, -16.1, -18.3,
            -20.5, -22.9,
        ],
    ),
    (
        4,
        &[
            -1.8, -3.2, -4.5, -5.8, -7.1, -9.9, -10.3, -14.3, -14.7, -18.7, -19.9, -22.4,
        ],
    ),
    (8, &[-7.9, -9.6, -14.2, -13.8, -18.6, -18.1, -22.8]),
    (14, &[-20.6, -20.5, -20.7, -24.6]),
];
const TGN_F_DELAYS_NS: [f64; 18] = [
    0.0, 10.0, 20.0, 30.0, 50.0, 80.0, 110.0, 140.0, 180.0, 230.0, 280.0, 330.0, 400.0, 490.0,
    600.0, 730.0, 880.0, 1050.0,
];
const TGN_F_CLUSTERS: [Cluster; 6] = [
    (
        0,
        &[
            -3.3, -3.6, -3.9, -4.2, -4.6, -5.3, -6.2, -7.1, -8.2, -9.5, -11.0, -12.5, -14.3, -16.7,
            -19.9,
        ],
    ),
    (
        4,
        &[
            -1.8, -2.8, -3.5, -4.4, -5.3, -7.4, -7.0, -10.3, -10.4, -13.8, -15.7, -19.9,
        ],
    ),
    (8, &[-5.7, -6.7, -10.4, -9.6, -14.1, -12.7, -18.5]),
    (12, &[-8.8, -13.3, -18.7]),
    (14, &[-12.9, -14.2]),
    (16, &[-16.3, -21.2]),
];

/// The SISO power-delay profile of a TGn model, total power 1. Clusters on the same tap add in
/// linear power. With `los`, the first tap gets the Table II K-factor (A, B, C: 0 dB; D: 3 dB;
/// E, F: 6 dB) and a zero-Doppler LOS component; every other tap stays Rayleigh.
pub fn tgn_taps(model: TgnModel, los: bool) -> Vec<Tap> {
    let (delays, clusters, k_db): (&[f64], &[Cluster], f64) = match model {
        TgnModel::A => (&[0.0], &[(0, &[0.0])], 0.0),
        TgnModel::B => (&TGN_B_DELAYS_NS, &TGN_B_CLUSTERS, 0.0),
        TgnModel::C => (&TGN_C_DELAYS_NS, &TGN_C_CLUSTERS, 0.0),
        TgnModel::D => (&TGN_D_DELAYS_NS, &TGN_D_CLUSTERS, 3.0),
        TgnModel::E => (&TGN_E_DELAYS_NS, &TGN_E_CLUSTERS, 6.0),
        TgnModel::F => (&TGN_F_DELAYS_NS, &TGN_F_CLUSTERS, 6.0),
    };
    let mut power = vec![0.0; delays.len()];
    for &(start, dbs) in clusters {
        for (p, db) in power.iter_mut().skip(start).zip(dbs) {
            *p += 10f64.powf(db / 10.0);
        }
    }
    let total: f64 = power.iter().sum();
    delays
        .iter()
        .zip(&power)
        .enumerate()
        .map(|(i, (&d, &p))| Tap {
            delay_s: d * 1e-9,
            power: p / total,
            k_factor: if los && i == 0 {
                10f64.powf(k_db / 10.0)
            } else {
                0.0
            },
            los_doppler_hz: 0.0,
        })
        .collect()
}

/// Unit-power complex Gaussian process with the given Doppler spectrum, `len` samples at
/// `sample_rate_hz`. Shaped in the frequency domain (each bin carries the exact integral of the
/// spectrum over it) at 64× the highest Doppler frequency, then linearly interpolated.
fn scatter_process(
    doppler: Doppler,
    len: usize,
    sample_rate_hz: f64,
    rng: &mut Rng,
) -> Vec<Sample> {
    let f_max = doppler.max_hz();
    let cn = |rng: &mut Rng| {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        (s * rng.gaussian(), s * rng.gaussian())
    };
    if f_max <= 0.0 || len == 0 {
        let g = cn(rng);
        return vec![g; len];
    }
    let fu = 64.0 * f_max;
    let duration_updates = (len as f64 / sample_rate_hz * fu).ceil() as usize + 2;
    // Resolution fd/32 at least, so the autocorrelation holds over many coherence times.
    let fd = match doppler {
        Doppler::Bell { fd_hz } | Doppler::Jakes { fd_hz } => fd_hz,
        Doppler::Static => f_max,
    };
    let min_bins = (32.0 * fu / fd).ceil() as usize;
    let m = duration_updates.max(min_bins).next_power_of_two();
    let df = fu / m as f64;
    let mut bins = vec![(0.0, 0.0); m];
    for (k, bin) in bins.iter_mut().enumerate() {
        // Bin k holds frequency (k − m for the upper half)·df.
        let f = if k < m / 2 {
            k as f64
        } else {
            k as f64 - m as f64
        } * df;
        let w = doppler.cdf(f + df / 2.0) - doppler.cdf(f - df / 2.0);
        if w > 0.0 {
            let g = cn(rng);
            *bin = (w.sqrt() * g.0, w.sqrt() * g.1);
        }
    }
    // Σ_k G_k·e^(j2πkm/M): the unscaled inverse transform, so E|g|² = Σ w_k = 1.
    let Ok(series) = transform(&bins, 1.0) else {
        return vec![(0.0, 0.0); len]; // unreachable: m is a power of two
    };
    (0..len)
        .map(|n| {
            let t = n as f64 / sample_rate_hz * fu;
            let i = t.floor();
            let a = t - i;
            let i = i as usize % m;
            let (p, q) = (series[i], series[(i + 1) % m]);
            (p.0 + a * (q.0 - p.0), p.1 + a * (q.1 - p.1))
        })
        .collect()
}

/// A time-varying tapped-delay-line channel: per-tap fading gains for `len` samples, and the
/// operator that applies them with fractional tap delays.
#[derive(Debug, Clone)]
pub struct Multipath {
    taps: Vec<Tap>,
    gains: Vec<Vec<Sample>>,
    sample_rate_hz: f64,
}

impl Multipath {
    /// Draws the gains of every tap: g = √P·(√(K/(K+1))·e^(j(2π·f_LOS·t + θ)) + √(1/(K+1))·s(t)),
    /// θ uniform, s a unit-power process with the `doppler` spectrum. `len` ≥ 1 samples of gain
    /// are kept; past the end the last gain holds.
    pub fn new(
        taps: Vec<Tap>,
        doppler: Doppler,
        len: usize,
        sample_rate_hz: f64,
        rng: &mut Rng,
    ) -> Self {
        let len = len.max(1);
        let gains = taps
            .iter()
            .map(|t| {
                let k = t.k_factor.max(0.0);
                let los_amp = (k / (k + 1.0)).sqrt();
                let nlos_amp = (1.0 / (k + 1.0)).sqrt();
                let theta = 2.0 * PI * rng.uniform();
                let scatter = scatter_process(doppler, len, sample_rate_hz, rng);
                let amp = t.power.sqrt();
                scatter
                    .iter()
                    .enumerate()
                    .map(|(n, s)| {
                        let los = rotate(
                            (los_amp, 0.0),
                            theta + 2.0 * PI * t.los_doppler_hz * n as f64 / sample_rate_hz,
                        );
                        (
                            amp * (los.0 + nlos_amp * s.0),
                            amp * (los.1 + nlos_amp * s.1),
                        )
                    })
                    .collect()
            })
            .collect();
        Self {
            taps,
            gains,
            sample_rate_hz,
        }
    }

    /// Gain of tap `tap` at sample `n` ((0, 0) for a tap that does not exist).
    pub fn gain(&self, tap: usize, n: usize) -> Sample {
        self.gains
            .get(tap)
            .and_then(|g| g.get(n.min(g.len() - 1)))
            .copied()
            .unwrap_or((0.0, 0.0))
    }

    /// The channel output y[n] = Σ_k g_k[n]·x(n − τ_k·fs), long enough to hold the delayed tail.
    pub fn apply(&self, x: &[Sample]) -> Vec<Sample> {
        let max_delay = self
            .taps
            .iter()
            .map(|t| t.delay_s * self.sample_rate_hz)
            .fold(0.0, f64::max);
        let out_len = x.len() + max_delay.ceil() as usize + INTERP_HALF;
        let mut y = vec![(0.0, 0.0); out_len];
        for (k, t) in self.taps.iter().enumerate() {
            let d = t.delay_s * self.sample_rate_hz;
            for (n, out) in y.iter_mut().enumerate() {
                let s = interpolate(x, n as f64 - d);
                let g = self.gain(k, n);
                out.0 += g.0 * s.0 - g.1 * s.1;
                out.1 += g.0 * s.1 + g.1 * s.0;
            }
        }
        y
    }
}
