//! Fixed-point framework: signed Q formats, rounding and overflow modes, and the G2 metrics.
//!
//! Hardware blocks compute on integers. A value is a raw `i64` together with a [`QFormat`]
//! (word length W, fractional bits F): real value = raw · 2^-F, range [-2^(W-1), 2^(W-1) - 1].
//! Models keep raw integers and call [`requantize`] wherever the hardware narrows a word, so the
//! model rounds exactly like the RTL. [`quantize`] turns real reference values (twiddle factors,
//! float-model outputs) into raw integers; [`sqnr_db`] and [`evm_percent`] measure what a word
//! length costs (gate G2).

/// Signed two's-complement fixed-point format: W word bits, F fractional bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QFormat {
    word_bits: u32,
    frac_bits: u32,
}

/// Formats are limited so every intermediate fits comfortably in `i128` and doubles stay exact.
pub const MAX_WORD_BITS: u32 = 48;
/// See [`MAX_WORD_BITS`].
pub const MAX_FRAC_BITS: u32 = 60;

/// Errors from the fixed-point framework.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixedError {
    /// Word length outside 1..=[`MAX_WORD_BITS`] or fractional bits above [`MAX_FRAC_BITS`].
    InvalidFormat {
        /// Requested word bits.
        word_bits: u32,
        /// Requested fractional bits.
        frac_bits: u32,
    },
    /// The input is NaN or infinite.
    NotFinite,
    /// The input is so large that it cannot be scaled exactly (|x · 2^F| ≥ 2^52).
    OutOfRange,
}

impl std::fmt::Display for FixedError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FixedError::InvalidFormat {
                word_bits,
                frac_bits,
            } => write!(
                f,
                "invalid Q format: {word_bits} word bits, {frac_bits} fractional"
            ),
            FixedError::NotFinite => write!(f, "value is not finite"),
            FixedError::OutOfRange => write!(f, "value too large to quantise exactly"),
        }
    }
}

impl std::error::Error for FixedError {}

/// How a value between two representable steps is resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rounding {
    /// Toward minus infinity: drop the low bits (plain truncation in hardware).
    Floor,
    /// Add half an LSB, then floor: ties go toward plus infinity.
    HalfUp,
    /// Round to nearest, ties to the even step (convergent rounding).
    HalfEven,
}

/// What happens to a value outside the format's range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Overflow {
    /// Clamp to the nearest representable extreme.
    Saturate,
    /// Keep the low W bits (two's-complement wrap-around).
    Wrap,
}

impl QFormat {
    /// The narrowest valid format, Q(1, 0): a placeholder for unreachable fallbacks.
    pub const MIN_SIGNED: QFormat = QFormat {
        word_bits: 1,
        frac_bits: 0,
    };

    /// A signed format with `word_bits` bits in total, `frac_bits` of them fractional.
    pub fn new(word_bits: u32, frac_bits: u32) -> Result<Self, FixedError> {
        if word_bits == 0 || word_bits > MAX_WORD_BITS || frac_bits > MAX_FRAC_BITS {
            return Err(FixedError::InvalidFormat {
                word_bits,
                frac_bits,
            });
        }
        Ok(Self {
            word_bits,
            frac_bits,
        })
    }

    /// Word length W.
    pub fn word_bits(self) -> u32 {
        self.word_bits
    }

    /// Fractional bits F.
    pub fn frac_bits(self) -> u32 {
        self.frac_bits
    }

    /// Smallest raw value, -2^(W-1).
    pub fn min_raw(self) -> i64 {
        -(1i64 << (self.word_bits - 1))
    }

    /// Largest raw value, 2^(W-1) - 1.
    pub fn max_raw(self) -> i64 {
        (1i64 << (self.word_bits - 1)) - 1
    }

    /// Real value of one LSB, 2^-F.
    pub fn lsb(self) -> f64 {
        pow2(-(self.frac_bits as i32))
    }

    /// Real value of `raw` in this format.
    pub fn to_f64(self, raw: i64) -> f64 {
        raw as f64 * self.lsb()
    }

    /// Applies the overflow mode to an unbounded raw value.
    fn bound(self, v: i128, overflow: Overflow) -> i64 {
        match overflow {
            Overflow::Saturate => {
                v.clamp(i128::from(self.min_raw()), i128::from(self.max_raw())) as i64
            }
            Overflow::Wrap => {
                // Keep the low W bits and sign-extend, as a W-bit register does.
                let unused = 128 - self.word_bits;
                ((v << unused) >> unused) as i64
            }
        }
    }
}

/// Quantises a real value into `format`.
pub fn quantize(
    x: f64,
    format: QFormat,
    rounding: Rounding,
    overflow: Overflow,
) -> Result<i64, FixedError> {
    if !x.is_finite() {
        return Err(FixedError::NotFinite);
    }
    // Scaling by a power of two is exact; below 2^52 the rounding steps below are exact too.
    let y = x * pow2(format.frac_bits as i32);
    if y.abs() >= pow2(52) {
        return Err(FixedError::OutOfRange);
    }
    let rounded = match rounding {
        Rounding::Floor => y.floor(),
        Rounding::HalfUp => (y + 0.5).floor(),
        Rounding::HalfEven => y.round_ties_even(),
    };
    Ok(format.bound(rounded as i128, overflow))
}

/// Re-expresses `raw` (with `from_frac` fractional bits) in `to`: the hardware narrowing step.
pub fn requantize(
    raw: i64,
    from_frac: u32,
    to: QFormat,
    rounding: Rounding,
    overflow: Overflow,
) -> i64 {
    let v = i128::from(raw);
    let unbounded = if to.frac_bits >= from_frac {
        v << (to.frac_bits - from_frac)
    } else {
        let s = from_frac - to.frac_bits;
        let half = 1i128 << (s - 1);
        match rounding {
            Rounding::Floor => v >> s,
            Rounding::HalfUp => (v + half) >> s,
            Rounding::HalfEven => {
                let floor = v >> s;
                let rem = v & ((1i128 << s) - 1);
                if rem > half || (rem == half && floor & 1 == 1) {
                    floor + 1
                } else {
                    floor
                }
            }
        }
    };
    to.bound(unbounded, overflow)
}

/// Signal-to-quantisation-noise ratio in dB of `approx` against `reference`.
///
/// `None` if the lengths differ, the slices are empty, or the reference has no power.
/// Returns `f64::INFINITY` when the two are identical.
pub fn sqnr_db(reference: &[f64], approx: &[f64]) -> Option<f64> {
    if reference.is_empty() || reference.len() != approx.len() {
        return None;
    }
    let signal: f64 = reference.iter().map(|r| r * r).sum();
    let noise: f64 = reference
        .iter()
        .zip(approx)
        .map(|(r, a)| (r - a) * (r - a))
        .sum();
    if signal == 0.0 {
        return None;
    }
    Some(10.0 * (signal / noise).log10())
}

/// RMS error vector magnitude of complex `approx` against `reference`, in percent of the
/// reference RMS. `None` under the same conditions as [`sqnr_db`].
pub fn evm_percent(reference: &[(f64, f64)], approx: &[(f64, f64)]) -> Option<f64> {
    if reference.is_empty() || reference.len() != approx.len() {
        return None;
    }
    let power: f64 = reference.iter().map(|(i, q)| i * i + q * q).sum();
    let error: f64 = reference
        .iter()
        .zip(approx)
        .map(|((ri, rq), (ai, aq))| (ri - ai).powi(2) + (rq - aq).powi(2))
        .sum();
    if power == 0.0 {
        return None;
    }
    Some(100.0 * (error / power).sqrt())
}

/// 2^e as an exact double (|e| well inside the exponent range for our formats).
fn pow2(e: i32) -> f64 {
    f64::powi(2.0, e)
}

/// Evaluates `metric` for every word length in `words`: the G2 sweep.
pub fn sweep(words: std::ops::RangeInclusive<u32>, metric: impl Fn(u32) -> f64) -> Vec<(u32, f64)> {
    words.map(|w| (w, metric(w))).collect()
}
