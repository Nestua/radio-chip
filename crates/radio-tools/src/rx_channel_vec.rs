//! `rvec rx_channel` and `rvec rx-channel-pkg`: the channel block's vectors and constants
//! (`hw/rtl/rx_channel.sv`, `docs/gates/rx_channel.md`), from `channel_from_bins`,
//! `tracker_constants` and `soft_scale`.

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{fft_bin, PILOT_SUBCARRIERS};
use radio_phy::ppdu::long_training_sequence;
use radio_phy::rx_front_fixed::IQ;
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{
    channel_from_bins, scale_constant, soft_scale, tracker_constants, FixedOfdmConfig,
};

const N: usize = 64;
const MODS: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

type Case = ([IQ; N], [IQ; N]);

fn bit_len(v: i64) -> u32 {
    i64::BITS - v.unsigned_abs().leading_zeros()
}

/// L(k) per FFT bin: ±1 on the 52 used subcarriers, 0 elsewhere (IEEE 802.11-2020 §17.3.3).
fn ltf_signs() -> [i64; N] {
    let mut l = [0i64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = i64::from(v);
    }
    l
}

fn random_cases(rng: &mut Rng, count: usize, hi: i64) -> Vec<Case> {
    let lo = -hi - 1;
    let span = (hi - lo + 1) as u64;
    (0..count)
        .map(|_| {
            let mut t = [[(0i64, 0i64); N]; 2];
            for w in &mut t {
                for v in w.iter_mut() {
                    *v = (
                        (rng.next_u64() % span) as i64 + lo,
                        (rng.next_u64() % span) as i64 + lo,
                    );
                }
            }
            (t[0], t[1])
        })
        .collect()
}

/// A channel of one to eight taps at amplitude 2^level, noise 0 to 30 dB below it, on L(k);
/// the unused bins carry noise (the block must ignore them).
fn channel_cases(rng: &mut Rng, per_level: usize, hi: i64) -> Vec<Case> {
    let lo = -hi - 1;
    let l = ltf_signs();
    let mut out = Vec::new();
    for level in 0..=16 {
        for _ in 0..per_level {
            let amp = 2f64.powi(level) * (0.5 + rng.uniform());
            let taps: Vec<(f64, f64)> = (0..1 + (rng.next_u64() % 8) as usize)
                .map(|_| (rng.gaussian(), rng.gaussian()))
                .collect();
            let noise = amp * 10f64.powf(-rng.uniform() * 1.5);
            let mut t = [[(0i64, 0i64); N]; 2];
            for (b, &lb) in l.iter().enumerate() {
                let k = if b < 32 { b as f64 } else { b as f64 - 64.0 };
                let (mut hr, mut hi_) = (0.0, 0.0);
                for (n, &(tr, ti)) in taps.iter().enumerate() {
                    let ph = -2.0 * std::f64::consts::PI * k * n as f64 / 64.0;
                    hr += tr * ph.cos() - ti * ph.sin();
                    hi_ += tr * ph.sin() + ti * ph.cos();
                }
                let lb = lb as f64;
                for w in &mut t {
                    let q = |v: f64| (v.round() as i64).clamp(lo, hi);
                    w[b] = (
                        q(amp * lb * hr + noise * rng.gaussian()),
                        q(amp * lb * hi_ + noise * rng.gaussian()),
                    );
                }
            }
            out.push((t[0], t[1]));
        }
    }
    out
}

/// A pair with re² + im² in [2^m − 2^(m−16), 2^m): the largest gain rounds to 2^15 at the
/// shift m − 15 and saturates.
fn rounding_corner(m: u32, limit: i64) -> Option<(i64, i64)> {
    let top = 1i128 << m;
    let low = top - (1i128 << (m - 16));
    let mut re = ((top - 1) as f64).sqrt().floor() as i64;
    while re > 0 {
        let rest = low - i128::from(re) * i128::from(re);
        let mut im = if rest <= 0 {
            0
        } else {
            (rest as f64).sqrt().ceil() as i64
        };
        while i128::from(im) * i128::from(im) < rest {
            im += 1;
        }
        let full = i128::from(re) * i128::from(re) + i128::from(im) * i128::from(im);
        if full >= low && full < top && re <= limit && im <= limit {
            return Some((re, im));
        }
        re -= 1;
    }
    None
}

/// The corners of G0: silence, the rails, pilots removed, the variance's saturation, a zero
/// shift, the gain's rounding to 2^15.
fn corner_cases(hi: i64) -> Vec<Case> {
    let lo = -hi - 1;
    let mut out = Vec::new();
    let zero = [(0i64, 0i64); N];
    out.push((zero, zero));
    out.push(([(lo, lo); N], [(lo, lo); N]));
    out.push(([(hi, lo); N], [(lo, hi); N]));
    out.push(([(hi, hi); N], [(hi, hi); N]));
    let flat = [(1000i64, -700i64); N];
    out.push((flat, flat));
    for gone in [
        &[0usize][..],
        &[0, 3],
        &[1, 2],
        &[0, 1],
        &[0, 1, 2, 3],
        &[1],
        &[3],
    ] {
        let mut t = flat;
        for &p in gone {
            t[fft_bin(PILOT_SUBCARRIERS[p])] = (0, 0);
        }
        out.push((t, t));
    }
    let mut t = [(20000i64, 0i64); N];
    for &c in &PILOT_SUBCARRIERS {
        t[fft_bin(c)] = (1, 0);
    }
    out.push((t, t));
    for v in [1i64, 7, 50, 90] {
        out.push(([(v, -v); N], [(v, v); N]));
    }
    for m in 16u32..=33 {
        if let Some((re, im)) = rounding_corner(m, 2 * hi) {
            let (mut a, mut b) = (flat, flat);
            if re <= hi && im <= hi {
                a[1] = (re, im);
                b[1] = (0, 0);
            } else {
                a[1] = (re / 2, im / 2);
                b[1] = (re - re / 2, im - im / 2);
            }
            out.push((a, b));
        }
    }
    out.extend(variance_edges());
    out
}

/// The variance's saturation at its edge: pilots of gain 1 (Hs = 1, a = b = 2, D = 20, no
/// shift) and Σg = 208 make 5·Σg·a·b = 208·D, a quotient of exactly 2^12 (saturated by the
/// check before the division, which a strict comparison would miss); Σg = 207 and 209 beside
/// it. Hs = (2, 1) has gain 5, (2, 0) gain 4: twelve and thirty-six of them make 204.
fn variance_edges() -> Vec<Case> {
    let l = ltf_signs();
    let mut out = Vec::new();
    for fives in [11usize, 12, 13] {
        let mut t1 = [(0i64, 0i64); N];
        let mut data = 0;
        for b in 0..N {
            if l[b] == 0 {
                continue;
            }
            let pilot = PILOT_SUBCARRIERS.iter().any(|&c| fft_bin(c) == b);
            let hs = if pilot {
                (1, 0)
            } else {
                data += 1;
                if data <= fives {
                    (2, 1)
                } else {
                    (2, 0)
                }
            };
            t1[b] = (l[b] * hs.0, l[b] * hs.1);
        }
        out.push((t1, [(0i64, 0i64); N]));
    }
    out
}

fn pairs(v: &[IQ]) -> String {
    v.iter()
        .map(|p| format!("{},{}", p.0, p.1))
        .collect::<Vec<_>>()
        .join(" ")
}

/// One line a packet:
/// `<T1: 64 bins re,im> <T2: 64 bins re,im> <shift> <n> <Σg> <scale BPSK> <QPSK> <16-QAM>
/// <64-QAM> <outer> <inner> <variance (saturated by the model)> <Hs: 64 re,im> <g: 64>`, bins in bin order.
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let bad = || io::Error::other("unusable widths");
    let hi = (1i64 << (fx.fft_bits - 1)) - 1;
    writeln!(
        out,
        "# rvec rx_channel — generated, do not edit. Regenerate: make -C hw vectors\n\
         # radio_phy::rx_ofdm_fixed::{{channel_from_bins, tracker_constants, soft_scale}}, \
         FixedOfdmConfig::default(), RxOfdmConfig::default() (soft {} / clip {}).\n\
         # <T1 64 re,im> <T2 64 re,im> <shift> <n> <gsum> <q bpsk> <q qpsk> <q 16qam> \
         <q 64qam> <outer> <inner> <variance> <Hs 64 re,im> <g 64>",
        cfg.soft_max, cfg.clip
    )?;
    let mut rng = Rng::new(0xc4a7_0062);
    let mut cases = corner_cases(hi);
    cases.extend(channel_cases(&mut rng, 12, hi));
    cases.extend(random_cases(&mut rng, 100, hi));
    for (t1, t2) in &cases {
        let ch = channel_from_bins(t1, t2, &fx).ok_or_else(bad)?;
        let c = tracker_constants(&ch, &fx);
        let mut q = Vec::new();
        let mut n = 0;
        for m in MODS {
            let (v, nn) = soft_scale(ch.gain_sum, m, &cfg, &fx);
            q.push(v.to_string());
            n = nn;
        }
        let g: Vec<String> = ch.gain.iter().map(|v| v.to_string()).collect();
        writeln!(
            out,
            "{} {} {} {} {} {} {} {} {} {} {}",
            pairs(t1),
            pairs(t2),
            ch.shift,
            n,
            ch.gain_sum,
            q.join(" "),
            c.outer,
            c.inner,
            c.variance,
            pairs(&ch.hs),
            g.join(" ")
        )?;
    }
    Ok(())
}

/// `rx_channel_pkg`: the block's widths, L(k) and the soft scale's constants.
pub fn package(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let l = ltf_signs();
    let (mut used, mut neg) = (0u64, 0u64);
    for (b, &v) in l.iter().enumerate() {
        if v != 0 {
            used |= 1 << b;
        }
        if v < 0 {
            neg |= 1 << b;
        }
    }
    let pilot_bins: Vec<String> = PILOT_SUBCARRIERS
        .iter()
        .map(|&c| fft_bin(c).to_string())
        .collect();
    writeln!(
        out,
        "// rvec rx-channel-pkg — generated from radio_phy::rx_ofdm_fixed, do not edit.\n\
         // Regenerate: make -C hw vectors\n\
         \n\
         `default_nettype none\n\
         \n\
         package rx_channel_pkg;\n\
         \x20   // Widths of FixedOfdmConfig::default() (docs/gates/rx_channel.md, G0).\n\
         \x20   localparam int W_BITS    = {};  // bins\n\
         \x20   localparam int U_BITS    = {};  // gains, signed word (U - 1 magnitude bits)\n\
         \x20   localparam int M_BITS    = {};  // the tracker's denominator after its shift\n\
         \x20   localparam int K_FRAC    = {};  // the coefficients' fractional bits\n\
         \x20   localparam int V_FRAC    = {};  // the variance's fractional bits\n\
         \x20   localparam int HS_BITS   = {};  // Hs's parts: W + 2\n\
         \x20   localparam int FULL_BITS = {};  // |Hs|^2, unsigned\n\
         \x20   localparam int GSUM_BITS = {};  // sum of 52 gains\n\
         \x20   localparam int COEF_Q    = {};  // the coefficients' quotient bits: K - 2\n\
         \x20   localparam int VAR_Q     = {};  // the variance's, saturating: V + 2 (variance_max)\n\
         \x20   // IEEE 802.11-2020 §17.3.3: the 52 used bins and those where L(k) = -1 (bit b: bin b).\n\
         \x20   localparam logic [63:0] L_USED = 64'h{:016x};\n\
         \x20   localparam logic [63:0] L_NEG  = 64'h{:016x};\n\
         \x20   // The pilots' bins, subcarriers -21, -7, 7, 21 (§17.3.5.9).\n\
         \x20   localparam int PILOT_BIN [4] = '{{{}}};",
        fx.fft_bits,
        fx.prod_bits,
        fx.track_bits,
        fx.coef_frac,
        fx.var_frac,
        fx.fft_bits + 2,
        2 * (fx.fft_bits + 1),
        bit_len(52 * ((1i64 << (fx.prod_bits - 1)) - 1)),
        fx.coef_frac - 2,
        fx.var_frac + 2,
        used,
        neg,
        pilot_bins.join(", ")
    )?;
    let a: Vec<i64> = MODS.iter().map(|&m| scale_constant(m, &cfg, &fx)).collect();
    let a_bits = a.iter().map(|&v| bit_len(v)).max().unwrap_or(1);
    writeln!(
        out,
        "\x20   // The soft scale's constants A = scale_constant(m) (BPSK, QPSK, 16-QAM, 64-QAM; soft {} / clip {}),\n\
         \x20   // and the quotient bits of A·2^n / Σg: bit length of A + 1.\n\
         \x20   localparam int A_BITS = {a_bits};\n\
         \x20   localparam logic [A_BITS-1:0] SCALE_A [4] = '{{{}}};\n\
         \x20   localparam int SCALE_Q [4] = '{{{}}};\n\
         endpackage\n\n`default_nettype wire",
        cfg.soft_max,
        cfg.clip,
        a.iter()
            .map(|v| format!("{a_bits}'d{v}"))
            .collect::<Vec<_>>()
            .join(", "),
        a.iter()
            .map(|&v| (bit_len(v) + 1).to_string())
            .collect::<Vec<_>>()
            .join(", ")
    )
}
