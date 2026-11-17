//! Channel simulator, part I: a seeded RNG, AWGN, carrier frequency offset and phase noise.
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
