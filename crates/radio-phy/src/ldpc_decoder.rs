//! Decoders of the HT LDPC codes (IEEE 802.11-2020 §19.3.11.7; the standard defines the codes,
//! not the decoder).
//!
//! Two decoders over the same parity checks:
//! - [`LdpcDecoder::decode_float`], belief propagation in floating point with the exact
//!   check rule (sum-product) or the min-sum approximation, on a flooding or a layered
//!   schedule: the reference the hardware algorithm is measured against;
//! - [`LdpcDecoder::decode_fixed`], layered normalised min-sum in integers: the algorithm of
//!   the hardware, every rounding and saturation defined here.
//!
//! A soft value is a log-likelihood ratio ln(P(bit = 0)/P(bit = 1)): positive says 0, the
//! magnitude says how surely, 0 is an erasure (a punctured bit).

use crate::ldpc::{LdpcCode, LdpcError, LdpcPlan};

/// The float decoder's input is limited to ±this: surer than any channel makes a bit, and
/// finite, so that an infinite input (a known bit) cannot turn into inf − inf inside.
pub const LLR_LIMIT: f64 = 1.0e6;

/// A soft value the PPDU side can combine: how the values of a bit sent more than once
/// (repetition, §19.3.11.7.5 e) become one.
pub trait Soft: Copy + Default {
    /// The value of a bit received as `self` and again as `other`.
    fn combined(self, other: Self) -> Self;
}

impl Soft for f64 {
    /// LLRs of independent observations add.
    fn combined(self, other: Self) -> Self {
        self + other
    }
}

impl Soft for i32 {
    /// The sum, saturating at the ends of `i32` (never wrapping). The decoder then
    /// saturates its input to the width of its totals.
    fn combined(self, other: Self) -> Self {
        self.saturating_add(other)
    }
}

/// How a check combines the messages of its other bits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CheckRule {
    /// The exact rule: 2·atanh(Π tanh(m/2)).
    SumProduct,
    /// The sign product and the smallest magnitude, times `scale` (1 = plain min-sum;
    /// below 1 corrects its overestimate: normalised min-sum).
    MinSum {
        /// The normalisation factor.
        scale: f64,
    },
}

/// The order in which messages are updated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Schedule {
    /// All checks from the previous iteration's bit messages, then all bits.
    Flooding,
    /// Check after check, each using the bit values the checks before it have just updated.
    Layered,
}

/// The fixed-point layered min-sum decoder's parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MinSumConfig {
    /// Most iterations run (the decoder stops earlier once every check holds).
    pub max_iter: usize,
    /// The normalisation factor is `scale_num / 2^scale_shift` (3 and 2: 0.75).
    pub scale_num: i32,
    /// See `scale_num`.
    pub scale_shift: u32,
    /// Whether the scaled minimum is rounded to nearest (half up: 2^(scale_shift−1) is
    /// added before the shift) or down. Rounding down takes 3/8 of a unit from a message
    /// on average at ×3/4 and turns a minimum of 1 into 0.
    pub scale_round: bool,
    /// Width of a check's message, sign included: magnitudes saturate at 2^(bits−1) − 1.
    pub msg_bits: u32,
    /// Width of a bit's running total (and of the input after saturation), sign included.
    pub app_bits: u32,
}

impl Default for MinSumConfig {
    /// Wide enough to behave like the float decoder on inputs scaled to a few units per
    /// unit of LLR; the hardware's widths are the G2 result (`docs/gates/ldpc.md`).
    fn default() -> Self {
        Self {
            max_iter: 20,
            scale_num: 3,
            scale_shift: 2,
            scale_round: false,
            msg_bits: 8,
            app_bits: 10,
        }
    }
}

impl MinSumConfig {
    /// Whether the parameters describe a decoder: widths of 2 to 24 bits, the total at
    /// least as wide as a message, a factor in (0, 1] (below 1 if it is rounded).
    pub fn is_valid(&self) -> bool {
        (2..=24).contains(&self.msg_bits)
            && (self.msg_bits..=24).contains(&self.app_bits)
            && self.scale_shift <= 8
            && self.scale_num >= 1
            && self.scale_num <= 1 << self.scale_shift
            && (!self.scale_round || self.scale_shift >= 1)
    }
}

/// What a decoder returns.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Decoded {
    /// The n hard decisions (the first k are the information bits).
    pub bits: Vec<u8>,
    /// Iterations run.
    pub iterations: usize,
    /// Whether every parity check holds on `bits`.
    pub converged: bool,
}

/// The exact check rule: for each of the messages, what the others say together.
/// `out[i] = 2·atanh(Π_{j≠i} tanh(m[j]/2))`.
pub fn check_sum_product(m: &[f64]) -> Vec<f64> {
    // Products of the others by a prefix and a suffix pass: no division, so a zero message
    // (an erasure) needs no special case.
    let t: Vec<f64> = m.iter().map(|&x| (x / 2.0).tanh()).collect();
    let n = t.len();
    let mut prefix = vec![1.0; n + 1];
    for i in 0..n {
        prefix[i + 1] = prefix[i] * t[i];
    }
    let mut suffix = 1.0;
    let mut out = vec![0.0; n];
    for i in (0..n).rev() {
        // Kept inside (−1, 1): atanh of ±1 is infinite. 2·atanh(1 − 1e−15) ≈ 35.
        let p = (prefix[i] * suffix).clamp(-1.0 + 1e-15, 1.0 - 1e-15);
        out[i] = 2.0 * p.atanh();
        suffix *= t[i];
    }
    out
}

/// The min-sum check rule: `out[i] = scale · Π_{j≠i} sign(m[j]) · min_{j≠i} |m[j]|`. A zero
/// message counts as positive (its magnitude makes the result zero anyway).
pub fn check_min_sum(m: &[f64], scale: f64) -> Vec<f64> {
    let (mut min1, mut min2, mut at) = (f64::INFINITY, f64::INFINITY, 0);
    let mut negative = false;
    for (i, &x) in m.iter().enumerate() {
        let a = x.abs();
        if a < min1 {
            (min2, min1, at) = (min1, a, i);
        } else if a < min2 {
            min2 = a;
        }
        negative ^= x < 0.0;
    }
    m.iter()
        .enumerate()
        .map(|(i, &x)| {
            let mag = scale * if i == at { min2 } else { min1 };
            if negative ^ (x < 0.0) {
                -mag
            } else {
                mag
            }
        })
        .collect()
}

/// A decoder for one code.
#[derive(Debug, Clone)]
pub struct LdpcDecoder {
    n: usize,
    /// For each check, the bits it adds.
    checks: Vec<Vec<usize>>,
}

impl LdpcDecoder {
    /// The decoder of `code`.
    pub fn new(code: &LdpcCode) -> Self {
        Self {
            n: code.n(),
            checks: code.checks(),
        }
    }

    fn converged(&self, bits: &[u8]) -> bool {
        self.checks
            .iter()
            .all(|c| c.iter().fold(0, |acc, &i| acc ^ bits[i]) == 0)
    }

    /// Belief propagation in floating point. `llr` holds n values (positive = 0), limited
    /// to ±[`LLR_LIMIT`] on entry (so ±∞ is a known bit); a NaN is refused, and so is a
    /// min-sum scale outside (0, 1]. At most `max_iter` iterations, stopping once every
    /// check holds (0 iterations if the input's own hard decisions already are a codeword).
    pub fn decode_float(
        &self,
        llr: &[f64],
        rule: CheckRule,
        schedule: Schedule,
        max_iter: usize,
    ) -> Result<Decoded, LdpcError> {
        if llr.len() != self.n {
            return Err(LdpcError::SoftLength {
                got: llr.len(),
                want: self.n,
            });
        }
        if llr.iter().any(|x| x.is_nan()) {
            return Err(LdpcError::NotANumber);
        }
        if let CheckRule::MinSum { scale } = rule {
            // Written so that a NaN scale is refused too.
            if !(scale > 0.0 && scale <= 1.0) {
                return Err(LdpcError::DecoderConfig);
            }
        }
        let update = |m: &[f64]| match rule {
            CheckRule::SumProduct => check_sum_product(m),
            CheckRule::MinSum { scale } => check_min_sum(m, scale),
        };
        let hard =
            |total: &[f64]| -> Vec<u8> { total.iter().map(|&x| u8::from(x < 0.0)).collect() };
        // total[b] = llr[b] + every check's message to b; c2v[c][i] = check c's message to
        // its i-th bit.
        let mut total: Vec<f64> = llr.iter().map(|x| x.clamp(-LLR_LIMIT, LLR_LIMIT)).collect();
        let mut c2v: Vec<Vec<f64>> = self.checks.iter().map(|c| vec![0.0; c.len()]).collect();
        let mut bits = hard(&total);
        let mut iterations = 0;
        while iterations < max_iter && !self.converged(&bits) {
            match schedule {
                Schedule::Flooding => {
                    let before = total.clone();
                    for (check, old) in self.checks.iter().zip(c2v.iter_mut()) {
                        let v2c: Vec<f64> = check
                            .iter()
                            .zip(old.iter())
                            .map(|(&b, &o)| before[b] - o)
                            .collect();
                        let new = update(&v2c);
                        for ((&b, o), n) in check.iter().zip(old.iter_mut()).zip(new) {
                            total[b] += n - *o;
                            *o = n;
                        }
                    }
                }
                Schedule::Layered => {
                    for (check, old) in self.checks.iter().zip(c2v.iter_mut()) {
                        let v2c: Vec<f64> = check
                            .iter()
                            .zip(old.iter())
                            .map(|(&b, &o)| total[b] - o)
                            .collect();
                        let new = update(&v2c);
                        for (((&b, o), n), v) in check.iter().zip(old.iter_mut()).zip(new).zip(v2c)
                        {
                            total[b] = v + n;
                            *o = n;
                        }
                    }
                }
            }
            bits = hard(&total);
            iterations += 1;
        }
        let converged = self.converged(&bits);
        Ok(Decoded {
            bits,
            iterations,
            converged,
        })
    }

    /// Layered normalised min-sum in integers: the hardware's algorithm.
    ///
    /// - The input is saturated to the total's range, ±(2^(app_bits−1) − 1).
    /// - Checks are taken in the order of [`LdpcCode::checks`] (block row after block row;
    ///   the Z checks of a block row share no bit, so hardware may take them together).
    /// - For a check: each bit's message is its total minus this check's last message to
    ///   it, saturated to the total's range. The check's new message to a bit is the sign
    ///   product of the other bits' messages (0 counts as positive) and
    ///   `(min · scale_num + r) >> scale_shift` of their smallest magnitude (r =
    ///   2^(scale_shift−1) with `scale_round`, else 0), saturated to the message range.
    ///   The bit's new total is its message plus the new one, saturated.
    /// - After each pass over all checks: bit = 1 where the total is negative; stop when
    ///   every check holds or after `max_iter` passes.
    pub fn decode_fixed(&self, llr: &[i32], cfg: &MinSumConfig) -> Result<Decoded, LdpcError> {
        if llr.len() != self.n {
            return Err(LdpcError::SoftLength {
                got: llr.len(),
                want: self.n,
            });
        }
        if !cfg.is_valid() {
            return Err(LdpcError::DecoderConfig);
        }
        let app_max = (1i32 << (cfg.app_bits - 1)) - 1;
        let msg_max = (1i32 << (cfg.msg_bits - 1)) - 1;
        let mut total: Vec<i32> = llr.iter().map(|&x| x.clamp(-app_max, app_max)).collect();
        let mut c2v: Vec<Vec<i32>> = self.checks.iter().map(|c| vec![0; c.len()]).collect();
        let hard = |total: &[i32]| -> Vec<u8> { total.iter().map(|&x| u8::from(x < 0)).collect() };
        let mut bits = hard(&total);
        let mut iterations = 0;
        let mut v2c = Vec::new();
        let half = if cfg.scale_round {
            1i64 << (cfg.scale_shift - 1)
        } else {
            0
        };
        while iterations < cfg.max_iter && !self.converged(&bits) {
            for (check, old) in self.checks.iter().zip(c2v.iter_mut()) {
                v2c.clear();
                let (mut min1, mut min2, mut at) = (i32::MAX, i32::MAX, 0);
                let mut negative = false;
                for (i, (&b, &o)) in check.iter().zip(old.iter()).enumerate() {
                    let v = (total[b] - o).clamp(-app_max, app_max);
                    let a = v.abs();
                    if a < min1 {
                        (min2, min1, at) = (min1, a, i);
                    } else if a < min2 {
                        min2 = a;
                    }
                    negative ^= v < 0;
                    v2c.push(v);
                }
                for (i, ((&b, o), &v)) in check.iter().zip(old.iter_mut()).zip(&v2c).enumerate() {
                    let min = if i == at { min2 } else { min1 };
                    // min ≤ 2^23 and scale_num ≤ 2^8: the product stays inside i64 easily.
                    let scaled =
                        (i64::from(min) * i64::from(cfg.scale_num) + half) >> cfg.scale_shift;
                    let mag = scaled.min(i64::from(msg_max)) as i32;
                    let new = if negative ^ (v < 0) { -mag } else { mag };
                    total[b] = (v + new).clamp(-app_max, app_max);
                    *o = new;
                }
            }
            bits = hard(&total);
            iterations += 1;
        }
        let converged = self.converged(&bits);
        Ok(Decoded {
            bits,
            iterations,
            converged,
        })
    }
}

impl LdpcPlan {
    /// The soft values of the Data field put back into whole codewords, the mirror of
    /// [`LdpcPlan::encode`]: shortened bits get `known_zero` (a sure 0), punctured bits 0
    /// (nothing known), and the soft values of repeated bits are combined with their
    /// originals ([`Soft::combined`]: added; integers saturate). `soft` holds N_avbits
    /// values; the result N_CW·L.
    pub fn soft_codewords<T: Soft>(&self, soft: &[T], known_zero: T) -> Result<Vec<T>, LdpcError> {
        // Consistency first: it ties N_CW·L to N_avbits, so the length check below bounds
        // everything allocated here.
        if !self.is_consistent() {
            return Err(LdpcError::Parameters);
        }
        if soft.len() != self.n_avbits {
            return Err(LdpcError::SoftLength {
                got: soft.len(),
                want: self.n_avbits,
            });
        }
        let (num, den) = self.rate.fraction();
        let l = self.length.bits();
        let k = l * num / den;
        let mut out = Vec::with_capacity(self.n_cw * l);
        let mut at = 0;
        for cw in 0..self.n_cw {
            let data = k - self.share_of(self.n_shrt, cw);
            let parity = l - k - self.share_of(self.n_punc, cw);
            let rep = self.share_of(self.n_rep, cw);
            let sent = data + parity;
            if at + sent + rep > soft.len() {
                return Err(LdpcError::Parameters);
            }
            let mut word: Vec<T> = soft[at..at + sent].to_vec();
            for i in 0..rep {
                word[i % sent] = word[i % sent].combined(soft[at + sent + i]);
            }
            out.extend_from_slice(&word[..data]);
            out.resize(out.len() + (k - data), known_zero);
            out.extend_from_slice(&word[data..]);
            out.resize((cw + 1) * l, T::default());
            at += sent + rep;
        }
        if at != soft.len() {
            return Err(LdpcError::Parameters);
        }
        Ok(out)
    }

    /// The payload out of decoded codewords (N_CW·L bits): the information bits of each
    /// codeword without its shortening bits. The mirror of [`LdpcPlan::shortened`].
    pub fn payload(&self, codewords: &[u8]) -> Result<Vec<u8>, LdpcError> {
        if !self.is_consistent() {
            return Err(LdpcError::Parameters);
        }
        let (num, den) = self.rate.fraction();
        let l = self.length.bits();
        let k = l * num / den;
        if codewords.len() != self.n_cw * l {
            return Err(LdpcError::SoftLength {
                got: codewords.len(),
                want: self.n_cw * l,
            });
        }
        let mut out = Vec::with_capacity(self.n_pld);
        for (cw, word) in codewords.chunks(l).enumerate() {
            out.extend_from_slice(&word[..k - self.share_of(self.n_shrt, cw)]);
        }
        Ok(out)
    }
}
