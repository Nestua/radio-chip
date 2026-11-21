//! Receiver front, fixed point (R0-08b part 2): the streaming receiver of [`crate::rx_front`]
//! step by step in integer arithmetic — the model the RTL must match bit for bit.
//!
//! Every width is a field of [`FixedRxConfig`]; the defaults are the widths the G2 sweep chose
//! (`docs/gates/rx_front.md`, `docs/gates/reports/rx_front_g2.txt`). The control flow (detector
//! run counter, search window, running maximum, latest start, drop-and-restart) is the float
//! model's; only the arithmetic differs, and every sum of products is an exact integer, so
//! re-summing a window from the history buffer equals a running accumulator read through a
//! delay line (the RTL may use either).
//!
//! Integer conventions: a value with `b` bits is a two's-complement integer in
//! [−2^(b−1), 2^(b−1)); narrowing rounds half to even and saturates (R0-11 `requantize`). Angles are
//! in units of 2π/2^A, NCO phases and CFO in units of 2π/2^P per sample (fs·2^−P Hz).

use crate::fixed::{requantize, Overflow, QFormat, Rounding};
use crate::rx_front::{
    backoff_samples, long_training_symbol, Pending, Ring, RxConfig, AUTOCORR_LEN,
    COARSE_END_BEFORE_T1, COARSE_LEN, FINE_GUARD, FINE_LEN, GI2_LEN, GI_LEN, HISTORY, LATCH_LAG,
    LTF_SYMBOL_LEN, MAX_DECISION_LATENCY, PAIR_LEN, SEARCH_LEN, SEARCH_OFFSET, STF_PERIOD,
};
use std::f64::consts::PI;

/// ADC full scale: a 12-bit converter gives integers in [−2048, 2048).
pub const ADC_FULL_SCALE: i32 = 2048;
/// Bits of the ADC samples.
pub const ADC_BITS: u32 = 12;
/// Bits of the DC blocker's output (one more than the ADC: x − m spans twice the range).
pub const Y_BITS: u32 = ADC_BITS + 1;

/// An integer complex sample.
pub type IQ = (i64, i64);

/// Word lengths and constants of the fixed-point receiver front.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedRxConfig {
    /// Fractional bits F of the DC blocker's accumulator (m·2^F).
    pub dc_frac: u32,
    /// Bits D of the samples entering detector A and the provisional latch.
    pub det_bits: u32,
    /// Detector threshold θ² as `theta_num / 2^theta_shift` (must be ≥ 0.575²).
    pub theta_num: i64,
    /// See [`FixedRxConfig::theta_num`].
    pub theta_shift: u32,
    /// CORDIC micro-rotations N.
    pub cordic_iters: u32,
    /// Width W_c the CORDIC input is block-normalised to.
    pub cordic_bits: u32,
    /// Angle resolution A: angles in units of 2π/2^A.
    pub angle_bits: u32,
    /// NCO phase accumulator width P (CFO resolution fs·2^−P).
    pub nco_phase_bits: u32,
    /// NCO table address bits L (the phase's top L bits).
    pub nco_lut_bits: u32,
    /// NCO table amplitude bits Q (signed; full scale 2^(Q−1) − 1).
    pub nco_amp_bits: u32,
    /// Bits Z of the rotated samples entering the LTF correlators.
    pub z_bits: u32,
    /// Bits T of the LTF taps (1: sign only, ±1 ± j).
    pub tap_bits: u32,
    /// Bits K the search correlator's output keeps before |c|².
    pub corr_bits: u32,
    /// Bits E of the samples entering the final coarse and fine CFO correlations.
    pub est_bits: u32,
    /// Weight w of the coarse CFO estimate in the CFO handed on, in 64ths (0..=64):
    /// `cfo = coarse + round((64 − w)·(fine − coarse)/64)`. 0 by decision (ADR 0007): the
    /// fine estimate, exactly. 15 is the AWGN optimum (`rx_front::FINE_WEIGHT` ≈ 49/64).
    pub coarse_weight_64: i64,
}

impl Default for FixedRxConfig {
    /// The widths chosen by the G2 sweep.
    fn default() -> Self {
        Self {
            dc_frac: 10,
            det_bits: 13,
            theta_num: 21668,
            theta_shift: 16,
            cordic_iters: 16,
            cordic_bits: 18,
            angle_bits: 16,
            nco_phase_bits: 22,
            nco_lut_bits: 9,
            nco_amp_bits: 11,
            z_bits: 12,
            tap_bits: 8,
            corr_bits: 20,
            est_bits: 12,
            coarse_weight_64: 0,
        }
    }
}

impl FixedRxConfig {
    /// Whether the widths are usable: `P ≥ A + 6` (the lag-64 step is an exact shift), table
    /// and register widths in range. The receiver functions do nothing on an invalid set.
    pub fn is_valid(&self) -> bool {
        (2..=24).contains(&self.nco_lut_bits)
            && self.nco_lut_bits <= self.nco_phase_bits
            && (2..=30).contains(&self.nco_amp_bits)
            && (2..=30).contains(&self.angle_bits)
            && self.nco_phase_bits <= 40
            && self.nco_phase_bits >= self.angle_bits + 6
            && (1..=Y_BITS).contains(&self.det_bits)
            && (1..=Y_BITS).contains(&self.est_bits)
            && (2..=30).contains(&self.z_bits)
            && (1..=16).contains(&self.tap_bits)
            && (2..=40).contains(&self.corr_bits)
            && (4..=40).contains(&self.cordic_bits)
            && self.cordic_iters <= 30
            && (1..=30).contains(&self.dc_frac)
            && (0..=64).contains(&self.coarse_weight_64)
            && self.theta_shift <= 16
    }
}

/// Narrows a `from`-bit integer to `to` bits: drops the low bits with convergent rounding
/// (half to even) and saturates; widening shifts left, exactly. Convergent, because half-up
/// (or floor) adds a DC bias of up to ½ LSB, and on a quiet input that DC correlates perfectly
/// at lag 16 and trips the detector (G2 sweep, `docs/gates/rx_front.md`).
fn narrow(v: i64, from: u32, to: u32) -> i64 {
    match QFormat::new(to, to.saturating_sub(1)) {
        Ok(q) => requantize(
            v,
            from.saturating_sub(1),
            q,
            Rounding::HalfEven,
            Overflow::Saturate,
        ),
        Err(_) => 0,
    }
}

fn narrow_iq(v: IQ, from: u32, to: u32) -> IQ {
    (narrow(v.0, from, to), narrow(v.1, from, to))
}

fn imul(a: IQ, b: IQ) -> IQ {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn iconj(a: IQ) -> IQ {
    (a.0, -a.1)
}

fn iadd(a: IQ, b: IQ) -> IQ {
    (a.0 + b.0, a.1 + b.1)
}

fn iabs2(a: IQ) -> i128 {
    i128::from(a.0) * i128::from(a.0) + i128::from(a.1) * i128::from(a.1)
}

/// The low `bits` bits of `v`, sign-extended (a `bits`-wide register).
fn wrap(v: i64, bits: u32) -> i64 {
    let unused = 64 - bits.clamp(1, 64);
    (v << unused) >> unused
}

/// Bits needed for |v| (0 for 0).
fn bit_len(v: i64) -> u32 {
    u64::BITS - v.unsigned_abs().leading_zeros()
}

/// The CORDIC arctangent table: round(atan(2^−i)·2^A/(2π)), i < N. A constant ROM in the RTL.
pub fn cordic_table(fx: &FixedRxConfig) -> Vec<i64> {
    let scale = f64::from(1u32 << fx.angle_bits.min(30)) / (2.0 * PI);
    (0..fx.cordic_iters)
        .map(|i| (2f64.powi(-(i as i32)).atan() * scale).round() as i64)
        .collect()
}

/// ∠(re + j·im) in units of 2π/2^A, in [−2^(A−1), 2^(A−1)), by CORDIC vectoring: block
/// normalisation to W_c bits (a leading-zero count and a shift), a half-turn when re < 0, then N
/// shift-and-add micro-rotations driving im to zero. 0 for a zero input.
pub fn cordic_angle(re: i64, im: i64, fx: &FixedRxConfig) -> i64 {
    cordic_with(re, im, fx, &cordic_table(fx))
}

pub(crate) fn cordic_with(re: i64, im: i64, fx: &FixedRxConfig, table: &[i64]) -> i64 {
    if re == 0 && im == 0 {
        return 0;
    }
    // Components keep W_c − 3 magnitude bits: the vector can grow by √2 (both components at
    // full scale) times the CORDIC gain (≈ 1.65) < 4, so x and y fit W_c-bit registers.
    let target = fx.cordic_bits.saturating_sub(3).max(2);
    let len = bit_len(re).max(bit_len(im));
    let (mut x, mut y) = if len > target {
        let s = len - target;
        (re >> s, im >> s)
    } else {
        let s = target - len;
        (re << s, im << s)
    };
    let half_turn = 1i64 << (fx.angle_bits - 1);
    let mut z = 0i64;
    if x < 0 {
        x = -x;
        y = -y;
        z = half_turn;
    }
    for (i, a) in table.iter().enumerate() {
        let (dx, dy) = (y >> i, x >> i);
        if y > 0 {
            x += dx;
            y -= dy;
            z += a;
        } else {
            x -= dx;
            y += dy;
            z -= a;
        }
        // The x/y registers are W_c bits wide (two's complement): with W_c − 3 magnitude bits
        // in, they never wrap; the model wraps as the RTL would if they did.
        (x, y) = (wrap(x, fx.cordic_bits), wrap(y, fx.cordic_bits));
    }
    let full = 1i64 << fx.angle_bits;
    (z + half_turn).rem_euclid(full) - half_turn
}

/// e^(jφ) for a P-bit phase φ (units 2π/2^P): the top L bits address a quarter-wave table of
/// round((2^(Q−1) − 1)·cos/sin(2π·r/2^L)), r < 2^(L−2), folded by quadrant (exact negations
/// and swaps).
pub fn nco_rotator(phase: i64, fx: &FixedRxConfig) -> IQ {
    rotate_with(phase, fx, &nco_table(fx))
}

/// The NCO's quarter-wave table: (cos, sin) for r < 2^(L−2). A constant ROM in the RTL.
pub fn nco_table(fx: &FixedRxConfig) -> Vec<IQ> {
    let l = fx.nco_lut_bits;
    let amp = f64::from((1u32 << (fx.nco_amp_bits - 1)) - 1);
    (0..1u64 << (l - 2))
        .map(|r| {
            let a = 2.0 * PI * r as f64 / f64::from(1u32 << l);
            (
                (amp * a.cos()).round() as i64,
                (amp * a.sin()).round() as i64,
            )
        })
        .collect()
}

/// The NCO mixer of the receiver's step 2: z = y·e^(jφ) for a [`Y_BITS`]-bit sample y and a
/// P-bit phase φ, the product narrowed (half to even, saturating) to Z bits. `None` for an
/// invalid configuration.
pub fn nco_mix(y: IQ, phase: i64, fx: &FixedRxConfig) -> Option<IQ> {
    fx.is_valid()
        .then(|| nco_mix_with(y, phase, fx, &nco_table(fx)))
}

fn nco_mix_with(y: IQ, phase: i64, fx: &FixedRxConfig, table: &[IQ]) -> IQ {
    let prod = imul(y, rotate_with(phase, fx, table));
    narrow_iq(prod, Y_BITS + fx.nco_amp_bits - 1, fx.z_bits)
}

/// The CFO taken out of the decoder's samples (R0-09 G2, stage B; G0 in
/// `docs/gates/rx_ofdm.md`): z(n) = y(n)·e^(−jφ·n) for the DC blocker's [`Y_BITS`]-bit output
/// y and the acquisition's CFO word φ ([`FixedAcquisition::cfo_phase`], 2π/2^P per sample),
/// with the front's own rotator ([`nco_mix`]: a P-bit accumulator that wraps, the table
/// addressed by the phase's top L bits) and its narrowing (half to even, saturating) to
/// `out_bits`. At `out_bits` = Z this is the mixer of the receiver's step 2; one bit more
/// keeps the ADC's last bit. Sample 0 is the stream's first: a constant phase is the channel
/// estimate's to absorb. Nothing (an empty stream) for an invalid configuration or an
/// `out_bits` outside 2..=30.
///
/// Where the hardware forms this stream and how wide it is handed on is ADR 0008's decision;
/// this is the candidate the measurements are made with.
pub fn correct_fixed(y: &[IQ], cfo_phase: i64, out_bits: u32, fx: &FixedRxConfig) -> Vec<IQ> {
    if !fx.is_valid() || !(2..=30).contains(&out_bits) {
        return Vec::new();
    }
    let table = nco_table(fx);
    let p = fx.nco_phase_bits;
    let step = cfo_phase.rem_euclid(1i64 << p);
    let mut phase = 0i64;
    y.iter()
        .map(|&v| {
            let prod = imul(v, rotate_with(phase, fx, &table));
            phase = (phase - step).rem_euclid(1i64 << p);
            narrow_iq(prod, Y_BITS + fx.nco_amp_bits - 1, out_bits)
        })
        .collect()
}

/// n / d for d > 0, rounded half to even.
fn div_half_even(n: i64, d: i64) -> i64 {
    let (q, r) = (n.div_euclid(d), n.rem_euclid(d));
    match (2 * r).cmp(&d) {
        std::cmp::Ordering::Greater => q + 1,
        std::cmp::Ordering::Equal => q + (q & 1),
        std::cmp::Ordering::Less => q,
    }
}

/// The DC offset of the ADC's samples `x` in ADC LSB, estimated from the two long training
/// symbols whose FFT windows start at `fft_start` and 64 samples on; the integer counterpart
/// of [`crate::rx_front::ltf_dc_offset`] (candidate for ADR 0008's hand-off; G0 in
/// `docs/gates/rx_front.md`). With e the front's rotator at the phase 64·`cfo_phase`
/// (rounded to the table's address), A its amplitude and d = A − e:
/// S = Σ (x[n + 64]·A − e·x[n]) over the 64 pairs, exactly, and D = S·d*/(64·|d|²) rounded
/// half to even and saturated to the ADC's 12 bits. One division per packet.
///
/// Zero where 256·|d|² < A² (the float model's [`crate::rx_front::DC_MIN_TURN`]): the offset
/// is then within a hundredth of a subcarrier spacing of the unused subcarrier. `None` when
/// the stream does not hold the 128 samples or the configuration is invalid.
pub fn ltf_dc_offset_fixed(
    x: &[(i16, i16)],
    fft_start: usize,
    cfo_phase: i64,
    fx: &FixedRxConfig,
) -> Option<IQ> {
    if !fx.is_valid() {
        return None;
    }
    let field = x.get(fft_start..fft_start.checked_add(PAIR_LEN)?)?;
    let table = nco_table(fx);
    let (p, l) = (fx.nco_phase_bits, fx.nco_lut_bits);
    // Half a table address, added before the phase is cut to the address (a tie goes up).
    let half_address = p.checked_sub(l + 1).map_or(0, |k| 1i64 << k);
    let turn = cfo_phase.rem_euclid(1i64 << p) * LTF_SYMBOL_LEN as i64 + half_address;
    let e = rotate_with(turn, fx, &table);
    let a = rotate_with(0, fx, &table).0;
    let d = (a - e.0, -e.1);
    let d2 = d.0 * d.0 + d.1 * d.1;
    if 256 * d2 < a * a {
        return Some((0, 0));
    }
    let (first, second) = field.split_at(LTF_SYMBOL_LEN);
    let s = first.iter().zip(second).fold((0i64, 0i64), |s, (&u, &v)| {
        let turned = imul(e, (i64::from(u.0), i64::from(u.1)));
        (
            s.0 + i64::from(v.0) * a - turned.0,
            s.1 + i64::from(v.1) * a - turned.1,
        )
    });
    // |S| < 2^29 and |d| < 2^12 with the widest rotator the configuration allows in
    // practice (Q = 11); the products stay far inside 64 bits.
    let num = imul(s, iconj(d));
    let den = LTF_SYMBOL_LEN as i64 * d2;
    let (lo, hi) = (-(1i64 << (ADC_BITS - 1)), (1i64 << (ADC_BITS - 1)) - 1);
    Some((
        div_half_even(num.0, den).clamp(lo, hi),
        div_half_even(num.1, den).clamp(lo, hi),
    ))
}

/// The ADC's samples less a DC offset in ADC LSB, saturated to the [`Y_BITS`] of the
/// rotator's input: what [`correct_fixed`] is handed when the offset comes from
/// [`ltf_dc_offset_fixed`] instead of the DC blocker. Two 12-bit values never reach the limit.
pub fn remove_dc_fixed(x: &[(i16, i16)], dc: IQ) -> Vec<IQ> {
    let (lo, hi) = (-(1i64 << (Y_BITS - 1)), (1i64 << (Y_BITS - 1)) - 1);
    x.iter()
        .map(|&(a, b)| {
            (
                (i64::from(a) - dc.0).clamp(lo, hi),
                (i64::from(b) - dc.1).clamp(lo, hi),
            )
        })
        .collect()
}

pub(crate) fn rotate_with(phase: i64, fx: &FixedRxConfig, table: &[IQ]) -> IQ {
    let l = fx.nco_lut_bits;
    let p = fx.nco_phase_bits;
    let addr = (phase.rem_euclid(1i64 << p) >> (p - l)) as usize;
    let quarter = 1usize << (l - 2);
    let (q, r) = (addr / quarter, addr % quarter);
    let (c, s) = table.get(r).copied().unwrap_or((0, 0));
    match q {
        0 => (c, s),
        1 => (-s, c),
        2 => (-c, -s),
        _ => (s, -c),
    }
}

/// The LTF symbol's taps in T bits, scaled to its largest component (T = 1: signs, ±1 ± j).
pub fn ltf_taps(fx: &FixedRxConfig) -> [IQ; LTF_SYMBOL_LEN] {
    let ltf = long_training_symbol();
    let peak = ltf
        .iter()
        .flat_map(|s| [s.0.abs(), s.1.abs()])
        .fold(0.0f64, f64::max);
    let mut out = [(0, 0); LTF_SYMBOL_LEN];
    for (o, s) in out.iter_mut().zip(ltf.iter()) {
        *o = if fx.tap_bits <= 1 {
            let sgn = |v: f64| if v < 0.0 { -1 } else { 1 };
            (sgn(s.0), sgn(s.1))
        } else {
            let full = f64::from((1u32 << (fx.tap_bits - 1)) - 1);
            let q = |v: f64| (v / peak * full).round() as i64;
            (q(s.0), q(s.1))
        };
    }
    out
}

/// The LTF search correlator: c = Σ_{k<128} z(k)·conj(tap(k mod 64)) over a window of 128
/// Z-bit samples (oldest first, T1 T2), narrowed to K bits (half to even, saturating).
/// |c| ≤ 128·2^(Z−1)·2^(T−1)·2, so the full sum is a (Z + T + 8)-bit value whose top K bits are
/// kept. `None` when `fx` is not valid.
pub fn ltf_corr(window: &[IQ; PAIR_LEN], fx: &FixedRxConfig) -> Option<IQ> {
    fx.is_valid()
        .then(|| ltf_corr_with(window, fx, &ltf_taps(fx)))
}

fn ltf_corr_with(window: &[IQ; PAIR_LEN], fx: &FixedRxConfig, taps: &[IQ; LTF_SYMBOL_LEN]) -> IQ {
    let c = window.iter().enumerate().fold((0, 0), |a, (k, z)| {
        iadd(a, imul(*z, iconj(taps[k % LTF_SYMBOL_LEN])))
    });
    let full = fx.z_bits + fx.tap_bits + 8;
    narrow_iq(c, full, fx.corr_bits)
}

/// Scales float samples by `gain` into the 12-bit ADC: round, saturate.
pub fn to_adc(x: &[(f64, f64)], gain: f64) -> Vec<(i16, i16)> {
    let q = |v: f64| {
        (v * gain)
            .round()
            .clamp(-f64::from(ADC_FULL_SCALE), f64::from(ADC_FULL_SCALE - 1)) as i16
    };
    x.iter().map(|&(a, b)| (q(a), q(b))).collect()
}

/// Causal DC blocker in integers: M = m·2^F, e = x·2^F − M, M += e >> k (k the gear shift of
/// [`crate::rx_front::DcBlocker`]), y = e·2^−F rounded half to even, saturated to [`Y_BITS`].
/// F must exceed the steady-state shift: the floored e >> k has a dead zone of 2^k LSB of M,
/// i.e. a residual DC up to 2^(k−F) LSB that trips the detector on a quiet input.
#[derive(Debug, Clone)]
struct DcFixed {
    shift: u32,
    frac: u32,
    m: IQ,
    n: u64,
}

impl DcFixed {
    fn push(&mut self, x: IQ) -> IQ {
        let k = (u64::BITS - 1 - self.n.saturating_add(1).leading_zeros()).min(self.shift);
        let e = ((x.0 << self.frac) - self.m.0, (x.1 << self.frac) - self.m.1);
        self.m = (self.m.0 + (e.0 >> k), self.m.1 + (e.1 >> k));
        self.n = self.n.saturating_add(1);
        (
            narrow(e.0, Y_BITS + self.frac, Y_BITS),
            narrow(e.1, Y_BITS + self.frac, Y_BITS),
        )
    }
}

/// Detector A in integers: D-bit inputs, exact running sums over [`AUTOCORR_LEN`] lag-16
/// products, fire when 2^s·|C|² ≥ num·P₁·P₂.
#[derive(Debug, Clone)]
struct DetFixed {
    hist: Ring<IQ>,
    n: usize,
    c: IQ,
    p1: i64,
    p2: i64,
}

impl DetFixed {
    fn new() -> Self {
        Self {
            hist: Ring::new(AUTOCORR_LEN + STF_PERIOD + 1, (0, 0)),
            n: 0,
            c: (0, 0),
            p1: 0,
            p2: 0,
        }
    }

    /// Whether the metric is at or above the threshold at this sample.
    fn push(&mut self, s: IQ, fx: &FixedRxConfig) -> bool {
        let (d, len, n) = (STF_PERIOD, AUTOCORR_LEN, self.n);
        self.hist.put(n, s);
        self.n += 1;
        if n < d {
            return false;
        }
        let x = |j: usize| self.hist.get(j);
        let k = n - d;
        let e2 = |v: IQ| v.0 * v.0 + v.1 * v.1;
        self.c = iadd(self.c, imul(x(k), iconj(x(n))));
        self.p1 += e2(x(k));
        self.p2 += e2(x(n));
        if k >= len {
            let old = k - len;
            let p = imul(x(old), iconj(x(old + d)));
            self.c = (self.c.0 - p.0, self.c.1 - p.1);
            self.p1 -= e2(x(old));
            self.p2 -= e2(x(old + d));
        }
        if k + 1 < len || self.p1 <= 0 || self.p2 <= 0 {
            return false;
        }
        // Exact: |C|, P₁, P₂ < 48·2·4096² < 2^32, so 2^16·|C|² < 2^81 and 21668·P₁·P₂ < 2^79
        // fit i128.
        // (A common normalising shift sized on max(P₁, P₂) zeroed the smaller sum at a power
        // onset and fired on it — pre-PR review; an RTL may normalise each term separately only
        // if the model changes with it.)
        (iabs2(self.c) << fx.theta_shift)
            >= i128::from(fx.theta_num) * i128::from(self.p1) * i128::from(self.p2)
    }
}

/// One sample of the fixed-point detection path (the RTL gate's per-sample values).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DetectorTrace {
    /// DC blocker output, [`Y_BITS`] bits.
    pub y: IQ,
    /// Detector A's metric at or above the threshold at this sample.
    pub fire: bool,
    /// A detection event: the run counter reached `run` (it restarts after each event).
    pub event: bool,
}

/// The fixed-point detection path per sample: DC blocker, detector A, run counter restarting
/// after each event — the receiver's step 1 without the acquisition's counter reset. Empty for
/// an invalid configuration.
pub fn detector_trace_fixed(
    x: &[(i16, i16)],
    cfg: &RxConfig,
    fx: &FixedRxConfig,
) -> Vec<DetectorTrace> {
    if !fx.is_valid() {
        return Vec::new();
    }
    let mut dc = dc_fixed(cfg, fx);
    let mut det = DetFixed::new();
    let run = cfg.run.max(1);
    let mut count = 0;
    x.iter()
        .map(|&(a, b)| {
            let y = dc_out(&mut dc, (i64::from(a), i64::from(b)));
            let fire = det.push(narrow_iq(y, Y_BITS, fx.det_bits), fx);
            let mut event = false;
            if fire {
                count += 1;
                if count >= run {
                    event = true;
                    count = 0;
                }
            } else {
                count = 0;
            }
            DetectorTrace { y, fire, event }
        })
        .collect()
}

/// Every detection event of the fixed-point detector (DC blocker, detector A, run counter
/// restarting after each event).
pub fn detections_fixed(x: &[(i16, i16)], cfg: &RxConfig, fx: &FixedRxConfig) -> Vec<usize> {
    detector_trace_fixed(x, cfg, fx)
        .iter()
        .enumerate()
        .filter_map(|(n, t)| t.event.then_some(n))
        .collect()
}

/// The fixed-point DC blocker over a whole stream (a copy when it is disabled).
pub fn dc_block_fixed(x: &[(i16, i16)], cfg: &RxConfig, fx: &FixedRxConfig) -> Vec<IQ> {
    let mut dc = dc_fixed(cfg, fx);
    x.iter()
        .map(|&(a, b)| dc_out(&mut dc, (i64::from(a), i64::from(b))))
        .collect()
}

fn dc_fixed(cfg: &RxConfig, fx: &FixedRxConfig) -> Option<DcFixed> {
    cfg.dc_shift.map(|shift| DcFixed {
        shift: shift.min(30),
        frac: fx.dc_frac.max(1),
        m: (0, 0),
        n: 0,
    })
}

fn dc_out(dc: &mut Option<DcFixed>, x: IQ) -> IQ {
    match dc {
        Some(d) => d.push(x),
        None => x,
    }
}

/// An acquisition by the fixed-point receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedAcquisition {
    /// The sample at which the acquisition was declared.
    pub decided_at: usize,
    /// The detection that opened the LTF search.
    pub detected_at: usize,
    /// First sample of GI2 on the first-path grid.
    pub ltf_start: usize,
    /// First sample of the FFT window of T1 (first path − back-off).
    pub fft_start: usize,
    /// The CFO handed on (the fine estimate unless `coarse_weight_64` is set), in units of
    /// 2π/2^P per sample: the downstream NCO's step is its negative.
    pub cfo_phase: i64,
    /// Coarse (STF) estimate alone, same units.
    pub coarse_phase: i64,
    /// Fine estimate alone (coarse + resolved LTF residual), same units.
    pub fine_phase: i64,
}

impl FixedAcquisition {
    /// The CFO handed on, in Hz.
    pub fn cfo_hz(&self, fx: &FixedRxConfig, sample_rate_hz: f64) -> f64 {
        self.cfo_phase as f64 * sample_rate_hz / 2f64.powi(fx.nco_phase_bits as i32)
    }
}

#[derive(Debug, Clone, Copy)]
struct Search {
    opened_by: usize,
    first: usize,
    deadline: usize,
    best: Option<(usize, i128)>,
}

/// The fixed-point streaming receiver front; the float counterpart is
/// [`crate::rx_front::RxFront`] (same control flow, see its documentation). Always detector A
/// with the threshold `theta_num/2^theta_shift`: `RxConfig::detector` and `threshold` are not
/// used (its sample rate, run length, DC shift and back-off are). Does nothing (never
/// acquires) when `fx` is not [`FixedRxConfig::is_valid`].
#[derive(Debug, Clone)]
pub struct RxFrontFixed {
    cfg: RxConfig,
    fx: FixedRxConfig,
    w: usize,
    b: usize,
    n: usize,
    dc: Option<DcFixed>,
    det: DetFixed,
    count: usize,
    y: Ring<IQ>,
    z: Ring<IQ>,
    pair: Ring<i128>,
    energy: Ring<i128>,
    step: i64,
    phase: i64,
    taps: [IQ; LTF_SYMBOL_LEN],
    tap_energy: i128,
    cordic: Vec<i64>,
    nco: Vec<IQ>,
    search: Option<Search>,
    pending: Option<Pending<FixedAcquisition>>,
    trace: SampleTrace,
}

/// The intermediate values of one sample of [`RxFrontFixed`] (what the RTL gates compare).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SampleTrace {
    /// DC blocker output, [`Y_BITS`] bits.
    pub y: IQ,
    /// NCO phase that rotated this sample, P bits (units 2π/2^P), in [0, 2^P).
    pub phase: i64,
    /// NCO step in effect at this sample (after this sample's latch, if any): the next
    /// sample's phase is (phase + step) mod 2^P.
    pub step: i64,
    /// NCO-rotated sample entering the LTF correlator, Z bits.
    pub z: IQ,
    /// LTF search correlator output (K bits) of the window ending at this sample; `None` before
    /// the first full window.
    pub corr: Option<IQ>,
    /// Whether detector A's run counter produced an event at this sample (step 1).
    pub event: bool,
}

impl RxFrontFixed {
    /// A receiver in the idle state.
    pub fn new(cfg: RxConfig, fx: FixedRxConfig) -> Self {
        let b = backoff_samples(&cfg);
        let w = GI_LEN.saturating_sub(b);
        let taps = ltf_taps(&fx);
        Self {
            dc: dc_fixed(&cfg, &fx),
            det: DetFixed::new(),
            cfg,
            fx,
            w,
            b,
            n: 0,
            count: 0,
            y: Ring::new(HISTORY, (0, 0)),
            z: Ring::new(PAIR_LEN, (0, 0)),
            pair: Ring::new(GI_LEN, 0),
            energy: Ring::new(LTF_SYMBOL_LEN + GI_LEN + 1, 0),
            step: 0,
            phase: 0,
            tap_energy: taps.iter().map(|t| iabs2(*t)).sum(),
            taps,
            cordic: cordic_table(&fx),
            nco: nco_table(&fx),
            search: None,
            pending: None,
            trace: SampleTrace::default(),
        }
    }

    /// Whether an LTF search is open.
    pub fn searching(&self) -> bool {
        self.search.is_some()
    }

    /// Σ_{k<len} v(start+k)·v*(start+k+lag) over the history narrowed to `bits`.
    fn lag_sum(&self, start: usize, len: usize, lag: usize, bits: u32) -> IQ {
        let v = |k: usize| narrow_iq(self.y.get(k), Y_BITS, bits);
        (start..start + len).fold((0, 0), |a, k| iadd(a, imul(v(k), iconj(v(k + lag)))))
    }

    /// An angle (2π/2^A) as a P-bit phase per sample of a lag-`lag` correlation, rounded.
    fn per_sample(&self, angle: i64, lag: u32) -> i64 {
        let up = self.fx.nco_phase_bits - self.fx.angle_bits;
        let v = i128::from(angle) << up;
        let s = lag.trailing_zeros();
        ((v + (1i128 << (s - 1))) >> s) as i64
    }

    /// Processes the next ADC sample; returns the acquisition decided at it, if any.
    pub fn push(&mut self, x: (i16, i16)) -> Option<FixedAcquisition> {
        self.trace = SampleTrace::default();
        self.step_sample(x)
    }

    /// [`RxFrontFixed::push`], also returning the intermediate values of this sample (the
    /// internal signals the RTL gates compare, `rvec`).
    pub fn push_traced(&mut self, x: (i16, i16)) -> (Option<FixedAcquisition>, SampleTrace) {
        let acq = self.push(x);
        (acq, self.trace)
    }

    fn step_sample(&mut self, x: (i16, i16)) -> Option<FixedAcquisition> {
        let fx = self.fx;
        if !fx.is_valid() {
            return None;
        }
        let n = self.n;
        self.n += 1;
        let yv = dc_out(&mut self.dc, (i64::from(x.0), i64::from(x.1)));
        self.y.put(n, yv);

        // 1. Detector, run counter, search control, provisional latch.
        let mut event = false;
        if self.det.push(narrow_iq(yv, Y_BITS, fx.det_bits), &fx) {
            self.count += 1;
            if self.count >= self.cfg.run.max(1) {
                self.count = 0;
                event = true;
            }
        } else {
            self.count = 0;
        }
        if event {
            let deadline = n + SEARCH_OFFSET + SEARCH_LEN;
            match &mut self.search {
                Some(s) => s.deadline = deadline,
                None => {
                    self.search = Some(Search {
                        opened_by: n,
                        first: n + SEARCH_OFFSET,
                        deadline,
                        best: None,
                    })
                }
            }
            if let Some(k0) = (n + 1).checked_sub(LATCH_LAG + STF_PERIOD + COARSE_LEN) {
                let c = self.lag_sum(k0, COARSE_LEN, STF_PERIOD, fx.det_bits);
                // The correction step is +∠/16 per sample (rx_front: step = −2π·cfo/fs).
                let angle = cordic_with(c.0, c.1, &fx, &self.cordic);
                self.step = self.per_sample(angle, STF_PERIOD as u32);
            }
        }

        // 2. NCO: z = y·e^(jφ), rounded to Z bits.
        let zv = nco_mix_with(yv, self.phase, &fx, &self.nco);
        self.z.put(n, zv);
        self.trace = SampleTrace {
            y: yv,
            phase: self.phase,
            step: self.step,
            z: zv,
            corr: None,
            event,
        };
        self.phase = (self.phase + self.step).rem_euclid(1i64 << fx.nco_phase_bits);

        // 3. LTF search; 4. the result of an accepted candidate, `decision_latency` later.
        self.search_step(n);
        self.apply_pending(n)
    }

    /// See [`crate::rx_front::RxFront`]: the same pending decision, bit for bit.
    fn apply_pending(&mut self, n: usize) -> Option<FixedAcquisition> {
        let p = self.pending.filter(|p| p.due == n)?;
        self.pending = None;
        let mut a = p.result?;
        a.decided_at = n;
        self.search = None;
        self.count = 0;
        Some(a)
    }

    fn search_step(&mut self, n: usize) -> Option<()> {
        let fx = self.fx;
        let i = n.checked_sub(PAIR_LEN - 1)?;
        let window: [IQ; PAIR_LEN] = std::array::from_fn(|k| self.z.get(i + k));
        let cq = ltf_corr_with(&window, &fx, &self.taps);
        self.trace.corr = Some(cq);
        self.pair.put(i, iabs2(cq));
        if self.w == 0 {
            return None;
        }
        let j = (i + 1).checked_sub(self.w)?;
        let e: i128 = (j..j + self.w).map(|p| self.pair.get(p)).sum();
        self.energy.put(j, e);
        let mut s = self.search?;
        if j < s.first {
            return None;
        }
        if j >= s.deadline {
            self.search = None;
            return None;
        }
        if s.best.is_none_or(|(_, emax)| e > emax) {
            s.best = Some((j, e));
        }
        if let Some((imax, _)) = s.best {
            if self.pending.is_none() && j >= imax + LTF_SYMBOL_LEN + self.w - 1 {
                self.pending = Some(Pending {
                    due: n + self.cfg.decision_latency.min(MAX_DECISION_LATENCY),
                    result: self.decide(&s, imax, n),
                });
                s.best = None;
            }
        }
        if j + 1 >= s.deadline {
            self.search = None;
        } else {
            self.search = Some(s);
        }
        None
    }

    fn decide(&self, s: &Search, imax: usize, n: usize) -> Option<FixedAcquisition> {
        let fx = self.fx;
        let w = self.w;
        let emax = self.energy.get(imax);
        // Latest start within GI − B of the maximum keeping 7/8 of its energy.
        let keep = emax - (emax >> 3);
        let sym = (imax..=imax + w)
            .rev()
            .find(|&p| p < s.deadline && self.energy.get(p) >= keep)?;

        // Final coarse CFO over the STF located from the timing.
        let kc = sym.checked_sub(COARSE_END_BEFORE_T1 + COARSE_LEN)?;
        if kc + HISTORY <= n {
            return None;
        }
        let cc = self.lag_sum(kc, COARSE_LEN, STF_PERIOD, fx.est_bits);
        let angle_c = cordic_with(cc.0, cc.1, &fx, &self.cordic);

        // Confirmation on the history rotated by the final coarse angle (+∠/16 per sample).
        let step = self.per_sample(angle_c, STF_PERIOD as u32);
        let rot: Vec<IQ> = (0..PAIR_LEN + w)
            .map(|m| {
                let ph = (step as i128 * m as i128).rem_euclid(1i128 << fx.nco_phase_bits) as i64;
                nco_mix_with(self.y.get(sym + m), ph, &fx, &self.nco)
            })
            .collect();
        let xc = |o: usize| {
            (0..LTF_SYMBOL_LEN).fold((0, 0), |a, k| {
                iadd(a, imul(rot[o + k], iconj(self.taps[k])))
            })
        };
        let (conf, coh) = (0..w).fold((0i128, 0i128), |(nc, ch), o| {
            let (a, b) = (xc(o), xc(o + LTF_SYMBOL_LEN));
            (nc + iabs2(a) + iabs2(b), ch + iabs2(iadd(a, b)))
        });
        let rx_energy: i128 = rot[..PAIR_LEN].iter().map(|v| iabs2(*v)).sum();
        if rx_energy <= 0 {
            return None;
        }
        // Float rules: conf/(E_L·rx) ≥ 1/2 and coh/(2·E_L·rx) ≥ 1/4, i.e. both 2·x ≥ E_L·rx.
        let rhs = self.tap_energy * rx_energy;
        if 2 * conf < rhs || 2 * coh < rhs {
            return None;
        }

        // Fine CFO, lag 64, resolved against the coarse one.
        let fine_start = (sym + FINE_GUARD).checked_sub(GI2_LEN)?;
        let cf = self.lag_sum(fine_start, FINE_LEN, LTF_SYMBOL_LEN, fx.est_bits);
        let coarse = -self.per_sample(angle_c, STF_PERIOD as u32);
        let raw = -self.per_sample(
            cordic_with(cf.0, cf.1, &fx, &self.cordic),
            LTF_SYMBOL_LEN as u32,
        );
        let span = 1i64 << (fx.nco_phase_bits - 6);
        let half = span >> 1;
        let d = (raw - coarse + half).rem_euclid(span) - half;
        let fine = coarse + d;
        // The CFO handed on: coarse + (64 − w)/64 of the resolved residual, rounded. With
        // w = 0 (ADR 0007) this is `fine` exactly and the hardware needs no multiplier; w = 15
        // is the AWGN optimum (49/64 for 0.771).
        let cfo = coarse + (((64 - fx.coarse_weight_64) * d + 32) >> 6);

        Some(FixedAcquisition {
            decided_at: n,
            detected_at: s.opened_by,
            ltf_start: sym - GI2_LEN,
            fft_start: sym.checked_sub(self.b)?,
            cfo_phase: cfo,
            coarse_phase: coarse,
            fine_phase: fine,
        })
    }
}

/// The first acquisition of the fixed-point receiver in the stream.
pub fn acquire_fixed(
    x: &[(i16, i16)],
    cfg: &RxConfig,
    fx: &FixedRxConfig,
) -> Option<FixedAcquisition> {
    let mut rx = RxFrontFixed::new(cfg.clone(), *fx);
    x.iter().find_map(|&s| rx.push(s))
}

/// Every acquisition of the fixed-point receiver in the stream.
pub fn acquisitions_fixed(
    x: &[(i16, i16)],
    cfg: &RxConfig,
    fx: &FixedRxConfig,
) -> Vec<FixedAcquisition> {
    let mut rx = RxFrontFixed::new(cfg.clone(), *fx);
    x.iter().filter_map(|&s| rx.push(s)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The RTL correlator folds tap pairs (k, 64 − k): it relies on the quantised taps keeping
    /// the LTF's Hermitian symmetry t(64 − k) = conj(t(k)) with t(0), t(32) real (the LTF is
    /// real in frequency, IEEE 802.11-2020 §17.3.3).
    #[test]
    fn ltf_taps_are_hermitian() {
        // Multi-bit taps only: sign-only taps (T = 1) map 0 to +1 and are not symmetric.
        for tap_bits in 2..=16 {
            let fx = FixedRxConfig {
                tap_bits,
                ..FixedRxConfig::default()
            };
            let t = ltf_taps(&fx);
            assert_eq!(t[0].1, 0, "T {tap_bits}");
            assert_eq!(t[32].1, 0, "T {tap_bits}");
            for k in 1..LTF_SYMBOL_LEN {
                assert_eq!(t[LTF_SYMBOL_LEN - k], iconj(t[k]), "T {tap_bits} k {k}");
            }
        }
    }
}
