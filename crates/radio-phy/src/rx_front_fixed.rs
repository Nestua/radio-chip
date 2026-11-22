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
use crate::ppdu::{data_symbols, Rate, MAX_PSDU_LEN};
use crate::rx_front::{
    backoff_samples, long_training_symbol, Pending, Ring, RxConfig, AUTOCORR_LEN,
    COARSE_END_BEFORE_T1, COARSE_LEN, FINE_GUARD, FINE_LEN, GI2_LEN, GI_LEN, HISTORY, LATCH_LAG,
    LTF_SYMBOL_LEN, MAX_DECISION_LATENCY, PAIR_LEN, SEARCH_LEN, SEARCH_OFFSET, STF_PERIOD,
};
use crate::rx_ofdm::{Signal, SignalCheck};
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

pub(crate) fn narrow_iq(v: IQ, from: u32, to: u32) -> IQ {
    (narrow(v.0, from, to), narrow(v.1, from, to))
}

pub(crate) fn imul(a: IQ, b: IQ) -> IQ {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

pub(crate) fn iconj(a: IQ) -> IQ {
    (a.0, -a.1)
}

pub(crate) fn iadd(a: IQ, b: IQ) -> IQ {
    (a.0 + b.0, a.1 + b.1)
}

pub(crate) fn iabs2(a: IQ) -> i128 {
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
/// Empty for an invalid configuration.
pub fn cordic_table(fx: &FixedRxConfig) -> Vec<i64> {
    if !fx.is_valid() {
        return Vec::new();
    }
    let scale = f64::from(1u32 << fx.angle_bits.min(30)) / (2.0 * PI);
    (0..fx.cordic_iters)
        .map(|i| (2f64.powi(-(i as i32)).atan() * scale).round() as i64)
        .collect()
}

/// ∠(re + j·im) in units of 2π/2^A, in [−2^(A−1), 2^(A−1)), by CORDIC vectoring: block
/// normalisation to W_c bits (a leading-zero count and a shift), a half-turn when re < 0, then N
/// shift-and-add micro-rotations driving im to zero. 0 for a zero input and for an invalid
/// configuration.
pub fn cordic_angle(re: i64, im: i64, fx: &FixedRxConfig) -> i64 {
    if !fx.is_valid() {
        return 0;
    }
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
    if !fx.is_valid() {
        return (0, 0);
    }
    rotate_with(phase, fx, &nco_table(fx))
}

/// The NCO's quarter-wave table: (cos, sin) for r < 2^(L−2). A constant ROM in the RTL.
/// Empty for an invalid configuration.
pub fn nco_table(fx: &FixedRxConfig) -> Vec<IQ> {
    if !fx.is_valid() {
        return Vec::new();
    }
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

pub(crate) fn nco_mix_with(y: IQ, phase: i64, fx: &FixedRxConfig, table: &[IQ]) -> IQ {
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
    let mut phase = 0i64;
    y.iter()
        .map(|&v| turn(v, &mut phase, cfo_phase, out_bits, fx, &table))
        .collect()
}

/// One sample of [`correct_fixed`]: `v` turned by `phase`, which then steps back by the CFO
/// word.
fn turn(
    v: IQ,
    phase: &mut i64,
    cfo_phase: i64,
    out_bits: u32,
    fx: &FixedRxConfig,
    table: &[IQ],
) -> IQ {
    let p = fx.nco_phase_bits;
    let prod = imul(v, rotate_with(*phase, fx, table));
    *phase = (*phase - cfo_phase.rem_euclid(1i64 << p)).rem_euclid(1i64 << p);
    narrow_iq(prod, Y_BITS + fx.nco_amp_bits - 1, out_bits)
}

/// n / d for d > 0, rounded half to even.
fn div_half_even(n: i128, d: i128) -> i128 {
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
/// Zero where 256·|d|² < A² (the float model's [`crate::rx_front::DC_MIN_TURN`], decided on
/// the table's entry: with the default table from a CFO word of 704, 3.36 kHz at 20 MHz,
/// where the float model's edge is 3.11 kHz): the offset is then within about a hundredth
/// of a subcarrier spacing of the unused subcarrier. `None` when the stream does not hold
/// the 128 samples (whatever the CFO) or the configuration is invalid.
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
    let d2 = iabs2(d);
    if 256 * d2 < i128::from(a) * i128::from(a) {
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
    // With the default rotator (Q = 11) and 12-bit samples |S| < 2^29, the numerator has 42
    // bits and the divisor 28: the hardware's widths. The model keeps the products in 128
    // bits, so that no rotator the configuration allows (Q up to 30) and no 16-bit sample
    // overflows them.
    let wide = |v: i64| i128::from(v);
    let num = (
        wide(s.0) * wide(d.0) + wide(s.1) * wide(d.1),
        wide(s.1) * wide(d.0) - wide(s.0) * wide(d.1),
    );
    let den = LTF_SYMBOL_LEN as i128 * d2;
    let (lo, hi) = (-(1i128 << (ADC_BITS - 1)), (1i128 << (ADC_BITS - 1)) - 1);
    let q = |n: i128| div_half_even(n, den).clamp(lo, hi) as i64;
    Some((q(num.0), q(num.1)))
}

/// The ADC's samples less a DC offset in ADC LSB, saturated to the [`Y_BITS`] of the
/// rotator's input: what [`correct_fixed`] is handed when the offset comes from
/// [`ltf_dc_offset_fixed`] instead of the DC blocker. Two 12-bit values never exceed the limit.
pub fn remove_dc_fixed(x: &[(i16, i16)], dc: IQ) -> Vec<IQ> {
    x.iter().map(|&v| remove_dc(v, dc)).collect()
}

/// One sample of [`remove_dc_fixed`].
fn remove_dc(x: (i16, i16), dc: IQ) -> IQ {
    let (lo, hi) = (-(1i64 << (Y_BITS - 1)), (1i64 << (Y_BITS - 1)) - 1);
    (
        (i64::from(x.0) - dc.0).clamp(lo, hi),
        (i64::from(x.1) - dc.1).clamp(lo, hi),
    )
}

/// The stream the decoder is handed (ADR 0008, decision 1): the ADC's samples `x` less the
/// offset [`ltf_dc_offset_fixed`] finds in the two long training symbols at `fft_start`
/// ([`remove_dc_fixed`]), with the CFO word `cfo_phase` taken out by the front's rotator
/// ([`correct_fixed`]) and narrowed to [`Y_BITS`]: the ADC's LSB is kept, and
/// `rx_ofdm_fixed`'s default input is this width. The whole stream is rotated from its
/// sample 0 with the one offset of this packet; `None` when the stream does not hold the two
/// symbols or the configuration is invalid.
///
/// Where the hardware's replay starts, and so the phase its first sample is rotated by, is
/// the streaming model's to fix (`RxFrontFixed`'s second history, before the hand-off's RTL).
pub fn handoff_fixed(
    x: &[(i16, i16)],
    fft_start: usize,
    cfo_phase: i64,
    fx: &FixedRxConfig,
) -> Option<Vec<IQ>> {
    let dc = ltf_dc_offset_fixed(x, fft_start, cfo_phase, fx)?;
    Some(correct_fixed(
        &remove_dc_fixed(x, dc),
        cfo_phase,
        Y_BITS,
        fx,
    ))
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
/// Zeros for an invalid configuration.
pub fn ltf_taps(fx: &FixedRxConfig) -> [IQ; LTF_SYMBOL_LEN] {
    if !fx.is_valid() {
        return [(0, 0); LTF_SYMBOL_LEN];
    }
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

/// An angle (2π/2^A) as a P-bit phase per sample of a lag-`lag` correlation, rounded.
pub(crate) fn phase_per_sample(angle: i64, lag: u32, fx: &FixedRxConfig) -> i64 {
    let up = fx.nco_phase_bits - fx.angle_bits;
    let v = i128::from(angle) << up;
    let s = lag.trailing_zeros();
    ((v + (1i128 << (s - 1))) >> s) as i64
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
    /// The search's energy metric completed at this sample: the sum of GI − B squared
    /// correlator outputs from position n + 1 − [`PAIR_LEN`] − (GI − B) on. `None` before
    /// the first one.
    pub energy: Option<i128>,
    /// The candidate accepted at this sample, whose decision is applied
    /// `decision_latency` samples later.
    pub accepted: Option<Accepted>,
}

/// What the search hands the decision when it accepts a candidate (the inputs of the
/// decision besides the memories; `rx_decision`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Accepted {
    /// Position of the running maximum of the energy metric.
    pub imax: usize,
    /// That maximum.
    pub emax: i128,
    /// The search's deadline at the acceptance: positions at or beyond it are not taken.
    pub deadline: usize,
    /// The detection that opened the search.
    pub opened_by: usize,
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

    /// See [`phase_per_sample`].
    fn per_sample(&self, angle: i64, lag: u32) -> i64 {
        phase_per_sample(angle, lag, &self.fx)
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
            energy: None,
            accepted: None,
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
        self.trace.energy = Some(e);
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
        if let Some((imax, emax)) = s.best {
            if self.pending.is_none() && j >= imax + LTF_SYMBOL_LEN + self.w - 1 {
                self.trace.accepted = Some(Accepted {
                    imax,
                    emax,
                    deadline: s.deadline,
                    opened_by: s.opened_by,
                });
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

/// Depth of the second history, of the ADC's own samples (ADR 0008, decision 1): as deep as
/// the history the decision reads ([`HISTORY`]). The decision refuses a candidate whose
/// short training field has left that history, which keeps T1's window in this one when
/// the acquisition is declared (274 to 277 samples after the window began in the gate's
/// scenes). The first beat must follow while it is still there: a `start_delay` beyond
/// some 230 periods always loses the stream.
pub const ADC_HISTORY: usize = HISTORY;
/// Samples the stream runs on after the hold's end, the frame's nominal last sample: the
/// decoder's timing tracker moves its windows with the transmitter's clock, four samples
/// late at the end of the longest PPDU from a clock 40 ppm slow (the frame is 4.4 samples
/// longer). The default back-off of four samples just holds that window; a smaller one does
/// not.
pub const HANDOFF_TAIL: usize = 8;
/// Data symbols of the longest PPDU, 4095 octets at 6 Mb/s (IEEE 802.11-2020 §17.3.5.4,
/// Table 17-4: 16 + 8·4095 + 6 bits in symbols of 24): the watchdog of the hold and of the
/// stream, 5.47 ms after T1.
pub const MAX_DATA_SYMBOLS: usize = 1366;
/// Samples from the FFT window of T1 to the end of SIGNAL's window.
const SIGNAL_END: usize = PAIR_LEN + GI_LEN + LTF_SYMBOL_LEN;
/// Samples of one data symbol.
const SYMBOL: usize = GI_LEN + LTF_SYMBOL_LEN;

/// The pace of the replay (ADR 0008, decision 3) and the release of the hold (decision 2 as
/// amended 2026-11-22). No number here changes a sample of the stream: the pace decides
/// when a slow reader loses it, the release when the hold stops keeping the gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HandoffConfig {
    /// Beats the stream can hand on in one sample period: the clocks per sample (8 at
    /// 160 MHz and 20 MS/s).
    pub rate: usize,
    /// Sample periods from the declaration to the first beat: the DC estimate's 128 reads,
    /// multiplication and division (≈ 160 clocks, 20 periods).
    pub start_delay: usize,
    /// The ADC samples the release looks back over, the newest included. A number above 64
    /// is 64; 0 sees no sample.
    pub clip_window: usize,
    /// The hold, while it is not patient, is given up when this many samples of the window
    /// sit on a rail of the ADC
    /// (I or Q at or beyond −[`ADC_FULL_SCALE`] or [`ADC_FULL_SCALE`] − 1). 0, or a count
    /// above the window: never.
    pub clip_count: usize,
    /// The window and the count of the release while the hold is patient
    /// ([`SignalVerdict::patient`]); as `clip_window` and `clip_count`.
    pub patient_window: usize,
    /// See [`HandoffConfig::patient_window`].
    pub patient_count: usize,
    /// The fewest agreeing coded bits of SIGNAL ([`SignalCheck::agree`]) at which a slow
    /// rate makes the hold patient ([`SignalVerdict::of`]).
    pub patient_agree: usize,
}

/// The decoder's verdict on a SIGNAL that decoded, as the front needs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SignalVerdict {
    /// The frame's number of data symbols.
    pub data_symbols: usize,
    /// The frame is one that survives a few clipped samples under a still gain, and its
    /// SIGNAL is believed: the hold is given up by the patient rule from now on.
    pub patient: bool,
}

impl SignalVerdict {
    /// The verdict for a decoded SIGNAL and its check: patient at 6, 9 and 12 Mb/s (BPSK
    /// and QPSK at rate 1/2 and BPSK at 3/4) when at least `patient_agree` of the 48 coded
    /// bits agree.
    pub fn of(signal: Signal, check: SignalCheck, pace: &HandoffConfig) -> Self {
        Self {
            // A decoded length is at most 4095; a larger one is the watchdog's.
            data_symbols: data_symbols(signal.rate, signal.length.min(MAX_PSDU_LEN))
                .min(MAX_DATA_SYMBOLS),
            patient: matches!(signal.rate, Rate::R6 | Rate::R9 | Rate::R12)
                && check.agree >= pace.patient_agree,
        }
    }
}

impl Default for HandoffConfig {
    fn default() -> Self {
        Self {
            rate: 8,
            start_delay: 20,
            clip_window: 4,
            clip_count: 2,
            patient_window: 32,
            patient_count: 24,
            patient_agree: 40,
        }
    }
}

/// How a stream ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandoffEnd {
    /// The end mark: the packet's last sample (by the decoder's verdict, or the watchdog's
    /// longest PPDU) has been handed on.
    Packet,
    /// The end mark after the decoder said SIGNAL did not decode.
    Stopped,
    /// The abort mark: the front declared a new acquisition; that packet's stream follows.
    Acquisition,
    /// The abort mark: the reader fell a whole history behind and the next sample is gone.
    Overrun,
}

/// What a stream's first beat carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HandoffStart {
    /// The acquisition's CFO word ([`FixedAcquisition::cfo_phase`]).
    pub cfo_phase: i64,
    /// The packet's tag: a counter of the packets opened, modulo 256. The decoder gives it
    /// back with its verdict ([`RxHandoffFixed::signal`]), so that a verdict on a packet
    /// that a new acquisition has replaced meanwhile is not taken for the new one.
    pub packet: u8,
}

/// One beat of the hand-off's stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandoffBeat {
    /// A sample of [`handoff_fixed`]'s stream, [`Y_BITS`] wide; `first` is set on the first
    /// sample of T1's FFT window, which is rotated by the phase zero.
    Sample {
        /// The sample.
        z: IQ,
        /// The packet's CFO word and tag, on a stream's first sample only.
        first: Option<HandoffStart>,
    },
    /// The end or abort mark; a beat of its own.
    End(HandoffEnd),
}

/// What one sample period of [`RxHandoffFixed`] gave.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HandoffStep {
    /// The acquisition declared at this sample, as [`RxFrontFixed::push`] returns it.
    pub acquired: Option<FixedAcquisition>,
    /// The hold on the gain control after this sample.
    pub hold: bool,
    /// The beats handed on in this period, in order.
    pub beats: Vec<HandoffBeat>,
}

/// A declared packet whose samples are being handed on.
#[derive(Debug, Clone, Copy)]
struct Replay {
    packet: u8,
    fft_start: usize,
    cfo_phase: i64,
    dc: IQ,
    /// The period of the first beat.
    open_at: usize,
    /// The next sample to hand on, and the phase it is rotated by.
    read: usize,
    phase: i64,
    /// The sample at which the hold is down, and one past the stream's last.
    hold_end: usize,
    last: usize,
    /// Whether the decoder has given its verdict.
    verdict: bool,
}

/// The integer front with ADR 0008's hand-off: [`RxFrontFixed`], the second history of the
/// ADC's samples, the replay and the hold on the gain control (decisions 1 to 3).
///
/// An acquisition raises the hold and opens a stream: the offset of
/// [`ltf_dc_offset_fixed`] is taken from the two long symbols in the history, and from
/// `start_delay` periods on the samples from T1's FFT window are read back, the offset
/// subtracted and the CFO word turned out, at up to `rate` beats a period until the reader is
/// at the antenna and one a period from then on. The samples are [`handoff_fixed`] of the
/// ADC's stream from T1's window, whatever the pace.
///
/// The hold lasts to the frame's last sample: [`RxHandoffFixed::signal`] tells it the
/// length; without it the watchdog ends it at the longest PPDU. The stream runs
/// [`HANDOFF_TAIL`] samples further and closes with an end mark. A new acquisition while a
/// packet is open ends it with an abort mark and opens its own (the hold stays up, unless
/// the release takes it in the same period); a reader
/// that falls [`ADC_HISTORY`] samples behind loses the stream and the hold. The hold is
/// also given up, and the stream left as it is, when `clip_count` of the last `clip_window`
/// ADC samples sit on a rail: the gain it keeps is not the gain of what is arriving (a
/// declaration on a burst that has ended, a stronger packet over the one being read). Only a
/// new acquisition raises it again. Once the decoder's verdict says the frame is one that
/// survives a few clipped samples ([`SignalVerdict::patient`]) the rule is
/// `patient_count` of `patient_window` instead, so that a short burst does not move the
/// gain under the frame; a verdict that does not say so puts the first rule back. A new
/// acquisition under a hold that is up and patient stays patient until its own verdict: the
/// gain has not moved, and going back to the quick rule in the middle of what is arriving
/// would release there. Every packet
/// opened ends in one mark, also one whose first sample was never taken; a mark that finds
/// the one before it still waiting for the reader is not sent. A new acquisition and an
/// overrun in the same period give the acquisition's mark. A packet whose last sample was
/// taken ends as a packet whatever comes before its mark is taken.
///
/// The front searches on while a packet is read; what it should search for then is not
/// decided (ADR 0008, decision 1). The clock-exact schedule is the RTL's.
#[derive(Debug, Clone)]
pub struct RxHandoffFixed {
    front: RxFrontFixed,
    pace: HandoffConfig,
    x: Ring<(i16, i16)>,
    replay: Option<Replay>,
    hold: bool,
    /// Whether the hold is given up by the patient rule. It means something only while
    /// the hold is up: every raising of the hold sets it, and nothing reads it otherwise.
    patient: bool,
    /// Which of the last 64 ADC samples sat on a rail, the newest in bit 0.
    rails: u64,
    mark: Option<HandoffEnd>,
    opened: u8,
}

impl RxHandoffFixed {
    /// A receiver in the idle state.
    pub fn new(cfg: RxConfig, fx: FixedRxConfig, pace: HandoffConfig) -> Self {
        Self {
            front: RxFrontFixed::new(cfg, fx),
            pace,
            x: Ring::new(ADC_HISTORY, (0, 0)),
            replay: None,
            hold: false,
            patient: false,
            rails: 0,
            mark: None,
            opened: 0,
        }
    }

    /// The hold on the gain control as it stands.
    pub fn hold(&self) -> bool {
        self.hold
    }

    /// The decoder's verdict on SIGNAL of the packet tagged `packet` (the tag of its first
    /// beat): its number of data symbols and whether the hold is patient from now on, or
    /// `None` when it did not decode. The first
    /// verdict for a packet counts; one for a packet that is not the open one is ignored.
    /// The hold covers the samples to the back-off after the last symbol's window: it is
    /// down at once if the first sample after them has been pushed, or SIGNAL failed.
    pub fn signal(&mut self, packet: u8, verdict: Option<SignalVerdict>) {
        let Some(r) = self
            .replay
            .as_mut()
            .filter(|r| r.packet == packet && !r.verdict)
        else {
            return;
        };
        match verdict {
            Some(SignalVerdict {
                data_symbols: symbols,
                patient,
            }) => {
                r.verdict = true;
                self.patient = patient;
                r.hold_end = packet_end(r.fft_start, symbols, self.front.b);
                r.last = r.hold_end.saturating_add(HANDOFF_TAIL);
                // As in `push`: the hold is down with sample `hold_end`, not before it.
                if self.front.n > r.hold_end {
                    self.hold = false;
                }
            }
            None => self.close(HandoffEnd::Stopped),
        }
    }

    /// Ends the open packet with `mark` and drops the hold.
    fn close(&mut self, mark: HandoffEnd) {
        self.replay = None;
        self.hold = false;
        self.mark.get_or_insert(mark);
    }

    /// Processes the next ADC sample; the reader takes at most `room` beats in its period.
    pub fn push(&mut self, x: (i16, i16), room: usize) -> HandoffStep {
        let fx = self.front.fx;
        if !fx.is_valid() {
            return HandoffStep::default();
        }
        let n = self.front.n;
        self.x.put(n, x);
        let acquired = self.front.push(x);
        if self.replay.is_some_and(|r| n >= r.hold_end) {
            self.hold = false;
        }
        if let Some(a) = acquired {
            self.open(&a, n);
        }
        // The release (decision 2 as amended): an ADC that clips this much is not receiving
        // the packet the hold was raised for. The window runs whatever the hold is, so a
        // hold raised by this sample can be given up by it.
        let full = i32::from(x.0).min(i32::from(x.1)) <= -ADC_FULL_SCALE
            || i32::from(x.0).max(i32::from(x.1)) >= ADC_FULL_SCALE - 1;
        self.rails = self.rails << 1 | u64::from(full);
        let (window, count) = if self.patient {
            (self.pace.patient_window, self.pace.patient_count)
        } else {
            (self.pace.clip_window, self.pace.clip_count)
        };
        let window = match window {
            w if w >= 64 => u64::MAX,
            w => (1u64 << w) - 1,
        };
        let clipped = (self.rails & window).count_ones() as usize;
        if count > 0 && clipped >= count {
            self.hold = false;
        }
        // The oldest sample the history still holds is n + 1 − ADC_HISTORY.
        if self
            .replay
            .is_some_and(|r| r.read < r.last && r.read + ADC_HISTORY <= n)
        {
            self.close(HandoffEnd::Overrun);
        }

        let mut beats = Vec::new();
        let mut budget = room.min(self.pace.rate);
        if budget > 0 {
            if let Some(m) = self.mark.take() {
                beats.push(HandoffBeat::End(m));
                budget -= 1;
            }
        }
        while budget > 0 {
            let Some(r) = self.replay.as_mut().filter(|r| n >= r.open_at) else {
                break;
            };
            if r.read >= r.last {
                // The hold went down HANDOFF_TAIL samples before the stream's last.
                beats.push(HandoffBeat::End(HandoffEnd::Packet));
                self.replay = None;
                break;
            }
            if r.read > n {
                break;
            }
            let first = (r.read == r.fft_start).then_some(HandoffStart {
                cfo_phase: r.cfo_phase,
                packet: r.packet,
            });
            let v = remove_dc(self.x.get(r.read), r.dc);
            let z = turn(v, &mut r.phase, r.cfo_phase, Y_BITS, &fx, &self.front.nco);
            r.read += 1;
            budget -= 1;
            beats.push(HandoffBeat::Sample { z, first });
        }
        HandoffStep {
            acquired,
            hold: self.hold,
            beats,
        }
    }

    /// Opens the packet of an acquisition declared at sample `n`.
    fn open(&mut self, a: &FixedAcquisition, n: usize) {
        // A packet whose last sample has been taken ends as a packet, even if its end mark
        // was still waiting for the reader.
        // A declaration under a hold that is up and patient inherits the patience until its
        // own verdict: the gain has not moved, and what was being read survives a few
        // clipped samples.
        let patient = self.patient && self.hold;
        if let Some(old) = self.replay {
            self.close(if old.read >= old.last {
                HandoffEnd::Packet
            } else {
                HandoffEnd::Acquisition
            });
        }
        // Both long symbols are in the history: a declaration follows them and comes within
        // the history's depth of T1 (see [`ADC_HISTORY`]). The estimate is there for every
        // field of 128 samples and a valid configuration.
        let field: Vec<(i16, i16)> = (0..PAIR_LEN).map(|k| self.x.get(a.fft_start + k)).collect();
        let dc = ltf_dc_offset_fixed(&field, 0, a.cfo_phase, &self.front.fx).unwrap_or_default();
        let hold_end = packet_end(a.fft_start, MAX_DATA_SYMBOLS, self.front.b);
        self.opened = self.opened.wrapping_add(1);
        self.replay = Some(Replay {
            packet: self.opened,
            fft_start: a.fft_start,
            cfo_phase: a.cfo_phase,
            dc,
            open_at: n.saturating_add(self.pace.start_delay),
            read: a.fft_start,
            phase: 0,
            hold_end,
            last: hold_end.saturating_add(HANDOFF_TAIL),
            verdict: false,
        });
        self.hold = true;
        self.patient = patient;
    }
}

/// The frame's last sample and one: the back-off after the window of the last of `symbols`
/// data symbols (at most [`MAX_DATA_SYMBOLS`]) of the packet whose T1 window starts at
/// `fft_start`.
fn packet_end(fft_start: usize, symbols: usize, backoff: usize) -> usize {
    fft_start
        .saturating_add(SIGNAL_END + SYMBOL * symbols.min(MAX_DATA_SYMBOLS))
        .saturating_add(backoff)
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
