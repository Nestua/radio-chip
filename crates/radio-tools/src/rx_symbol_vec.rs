//! `rvec rx_symbol` and `rvec rx-symbol-pkg`: the symbol block's vectors and constants
//! (`hw/rtl/rx_symbol.sv`, `docs/gates/rx_symbol.md`), from `symbol_pilots` and
//! `symbol_from_bins`.

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{data_subcarriers, fft_bin, pilot_polarity, PILOT_BASE, PILOT_SUBCARRIERS};
use radio_phy::ppdu::long_training_sequence;
use radio_phy::rx_front_fixed::IQ;
use radio_phy::rx_ofdm::RxOfdmConfig;
use radio_phy::rx_ofdm_fixed::{
    channel_from_bins, soft_scale, symbol_from_bins, symbol_pilots, threshold_constant,
    FixedChannel, FixedOfdmConfig,
};

const N: usize = 64;
const MODS: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

fn rand_in(rng: &mut Rng, bits: u32) -> i64 {
    (rng.next_u64() % (1u64 << bits)) as i64 - (1i64 << (bits - 1))
}

/// A channel of one to eight taps at amplitude 2^level, through the model's estimate.
fn channel_case(rng: &mut Rng, level: i32, fx: &FixedOfdmConfig) -> Option<FixedChannel> {
    let hi = (1i64 << (fx.fft_bits - 1)) - 1;
    let mut l = [0i64; N];
    for (k, &v) in (-26i8..=26).zip(long_training_sequence().iter()) {
        l[fft_bin(k)] = i64::from(v);
    }
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
        for w in &mut t {
            let q = |v: f64| (v.round() as i64).clamp(-hi - 1, hi);
            w[b] = (
                q(amp * lb as f64 * hr + noise * rng.gaussian()),
                q(amp * lb as f64 * hi_ + noise * rng.gaussian()),
            );
        }
    }
    channel_from_bins(&t[0], &t[1], fx).filter(|c| c.gain_sum > 0)
}

/// A channel at the inputs' limits: Hs, g, the shift and Σg anywhere in their formats (the
/// block takes them as they come). Off the 52 used bins Hs and g are 0, as the channel
/// block's stores never answer there.
fn hand_channel(rng: &mut Rng, shift: u32, g_max: i64, gsum: i64) -> FixedChannel {
    let mut hs = [(0i64, 0i64); N];
    let mut gain = [0i64; N];
    for b in (1..=26).chain(38..64) {
        hs[b] = (rand_in(rng, 18), rand_in(rng, 18));
        gain[b] = (rng.next_u64() % (g_max as u64 + 1)) as i64;
    }
    FixedChannel {
        hs,
        shift,
        gain,
        gain_sum: gsum,
    }
}

/// Bins of a symbol on `ch`: 64-QAM-like points through it, a common phase and a slope, noise.
fn symbol_bins(rng: &mut Rng, ch: &FixedChannel, level: f64, noise: f64) -> [IQ; N] {
    let hi = (1i64 << 15) - 1;
    let peak = ch
        .hs
        .iter()
        .map(|h| ((h.0 * h.0 + h.1 * h.1) as f64).sqrt())
        .fold(1.0, f64::max);
    let ph0 = rng.uniform() * 2.0 * std::f64::consts::PI;
    let slope = (rng.uniform() - 0.5) * 0.1;
    let mut z = [(0i64, 0i64); N];
    for (b, v) in z.iter_mut().enumerate() {
        let k = if b < 32 { b as f64 } else { b as f64 - 64.0 };
        let (xr, xi) = (
            (rng.next_u64() % 8) as f64 - 3.5,
            (rng.next_u64() % 8) as f64 - 3.5,
        );
        let (hr, hi_) = (ch.hs[b].0 as f64 / peak, ch.hs[b].1 as f64 / peak);
        let (yr, yi) = (hr * xr - hi_ * xi, hr * xi + hi_ * xr);
        let a = ph0 + slope * k;
        let (cr, ci) = (a.cos(), a.sin());
        let q = |x: f64| (x.round() as i64).clamp(-hi - 1, hi);
        *v = (
            q(level * (yr * cr - yi * ci) + noise * rng.gaussian()),
            q(level * (yr * ci + yi * cr) + noise * rng.gaussian()),
        );
    }
    z
}

fn random_bins(rng: &mut Rng) -> [IQ; N] {
    let mut z = [(0i64, 0i64); N];
    for v in z.iter_mut() {
        *v = (rand_in(rng, 16), rand_in(rng, 16));
    }
    z
}

/// How a packet's symbols are made.
#[derive(Clone, Copy)]
enum Bins {
    Channel,
    Random,
    Rails(IQ),
}

struct Packet {
    ch: FixedChannel,
    data_mod: Modulation,
    symbols: usize,
    bins: Bins,
    theta: Option<i64>,
}

fn pairs(v: &[IQ]) -> String {
    v.iter()
        .map(|p| format!("{},{}", p.0, p.1))
        .collect::<Vec<_>>()
        .join(" ")
}

fn mod_code(m: Modulation) -> usize {
    MODS.iter().position(|&x| x == m).unwrap_or(0)
}

fn packets(fx: &FixedOfdmConfig) -> Vec<Packet> {
    let mut rng = Rng::new(0x5e3b_0062);
    let mut out = Vec::new();
    // Channels through the model's estimate, every level, every modulation.
    for level in 0..=16 {
        for i in 0..4 {
            if let Some(ch) = channel_case(&mut rng, level, fx) {
                out.push(Packet {
                    ch,
                    data_mod: MODS[(level as usize + i) % 4],
                    symbols: 1 + (rng.next_u64() % 4) as usize,
                    bins: Bins::Channel,
                    theta: None,
                });
            }
        }
    }
    // The inputs' limits.
    for i in 0..48usize {
        let shift = (rng.next_u64() % 20) as u32;
        let g_max = if i % 3 == 0 {
            32767
        } else {
            1 + (rng.next_u64() % 32767) as i64
        };
        let gsum = if i % 5 == 0 {
            1
        } else {
            1 + (rng.next_u64() % ((1 << 21) - 1)) as i64
        };
        let theta = match i % 4 {
            0 => Some((1 << 22) - 1),
            1 => Some(-(1 << 22)),
            _ => None,
        };
        out.push(Packet {
            ch: hand_channel(&mut rng, shift, g_max, gsum),
            data_mod: MODS[i % 4],
            symbols: 1 + (rng.next_u64() % 3) as usize,
            bins: Bins::Random,
            theta,
        });
    }
    // The rails: products saturating at shift 0, Σg of 1 and the largest.
    let lo_z = -(1i64 << 15);
    let lo_h = -(1i64 << 17);
    for (i, &(z, h)) in [
        ((lo_z, lo_z), (lo_h, lo_h)),
        ((lo_z, -lo_z - 1), (lo_h, -lo_h - 1)),
        ((-lo_z - 1, -lo_z - 1), (-lo_h - 1, -lo_h - 1)),
        ((lo_z, 0), (lo_h, 0)),
        ((0, 0), (0, 0)),
    ]
    .iter()
    .enumerate()
    {
        for (j, &gsum) in [1i64, (1 << 21) - 1].iter().enumerate() {
            let mut hs = [(0i64, 0i64); N];
            let mut gain = [0i64; N];
            for b in (1..=26).chain(38..64) {
                hs[b] = h;
                gain[b] = 32767;
            }
            out.push(Packet {
                ch: FixedChannel {
                    hs,
                    shift: if j == 0 { 0 } else { 19 },
                    gain,
                    gain_sum: gsum,
                },
                data_mod: MODS[(i + j) % 4],
                symbols: 2,
                bins: Bins::Rails(z),
                theta: Some(if i % 2 == 0 { 0 } else { (1 << 22) - 1 }),
            });
        }
    }
    // A packet past the polarity sequence's period (127 symbols).
    if let Some(ch) = channel_case(&mut rng, 12, fx) {
        out.push(Packet {
            ch,
            data_mod: Modulation::Qam16,
            symbols: 131,
            bins: Bins::Channel,
            theta: None,
        });
    }
    out
}

/// Two kinds of line. A packet: `P <shift> <n> <q BPSK> <q QPSK> <q 16-QAM> <q 64-QAM>
/// <data modulation 0..3> <symbols> <Hs 64 re,im> <g 64>`; then for each symbol (0 SIGNAL, in
/// BPSK; then the data symbols, polarity index = the symbol's number):
/// `S <θ> <Z 64 re,im> <pilots 4 re,im> <φ> <soft values, 48·N_BPSC, on-air order>`.
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let bad = || io::Error::other("unusable widths");
    writeln!(
        out,
        "# rvec rx_symbol — generated, do not edit. Regenerate: make -C hw vectors\n\
         # radio_phy::rx_ofdm_fixed::{{symbol_pilots, symbol_from_bins}}, FixedOfdmConfig::default(), \
         RxOfdmConfig::default() (soft {} / clip {}).\n\
         # P <shift> <n> <q bpsk> <q qpsk> <q 16qam> <q 64qam> <data mod> <symbols> <Hs 64 re,im> <g 64>\n\
         # S <theta> <Z 64 re,im> <pilots 4 re,im> <phi> <soft 48*N_BPSC>",
        cfg.soft_max, cfg.clip
    )?;
    let mut rng = Rng::new(0x5e3b_0063);
    for p in packets(&fx) {
        let mut q = Vec::new();
        let mut n = 0;
        for m in MODS {
            let (v, nn) = soft_scale(p.ch.gain_sum, m, &cfg, &fx);
            q.push(v.to_string());
            n = nn;
        }
        let g: Vec<String> = p.ch.gain.iter().map(|v| v.to_string()).collect();
        writeln!(
            out,
            "P {} {} {} {} {} {} {}",
            p.ch.shift,
            n,
            q.join(" "),
            mod_code(p.data_mod),
            p.symbols,
            pairs(&p.ch.hs),
            g.join(" ")
        )?;
        for s in 0..p.symbols {
            let m = if s == 0 { Modulation::Bpsk } else { p.data_mod };
            let theta = p.theta.unwrap_or_else(|| rand_in(&mut rng, 23));
            let z = match p.bins {
                Bins::Channel => {
                    let lvl = 2f64.powf(4.0 + rng.uniform() * 11.0);
                    let noise = lvl * 0.05 * rng.uniform();
                    symbol_bins(&mut rng, &p.ch, lvl, noise)
                }
                Bins::Random => random_bins(&mut rng),
                Bins::Rails(v) => [v; N],
            };
            let pilots = symbol_pilots(&z, &p.ch, s, &fx).ok_or_else(bad)?;
            let (soft, phi) =
                symbol_from_bins(&z, &p.ch, s, m, theta, &cfg, &fx).ok_or_else(bad)?;
            let soft: Vec<String> = soft.iter().map(|v| v.to_string()).collect();
            writeln!(
                out,
                "S {} {} {} {} {}",
                theta,
                pairs(&z),
                pairs(&pilots),
                phi,
                soft.join(" ")
            )?;
        }
    }
    Ok(())
}

/// `rx_symbol_pkg`: the data subcarriers' bins and numbers, the pilots' numbers and signs, the
/// polarity sequence, the demapper's constants, the soft format.
pub fn package(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let cfg = RxOfdmConfig::default();
    let data = data_subcarriers();
    let bins: Vec<String> = data.iter().map(|&c| fft_bin(c).to_string()).collect();
    let nums: Vec<String> = data.iter().map(|&c| c.to_string()).collect();
    let pilots: Vec<String> = PILOT_SUBCARRIERS.iter().map(|c| c.to_string()).collect();
    let mut base_neg = 0u32;
    for (i, &b) in PILOT_BASE.iter().enumerate() {
        if b < 0 {
            base_neg |= 1 << i;
        }
    }
    let mut pol = 0u128;
    for n in 0..127 {
        if pilot_polarity(n) < 0 {
            pol |= 1 << n;
        }
    }
    let k = |c: u32, m: Modulation| threshold_constant(c, m, &fx);
    let k_hi = [0, 0, k(2, Modulation::Qam16), k(4, Modulation::Qam64)];
    let k_lo = [0, 0, 0, k(2, Modulation::Qam64)];
    let list = |v: &[i64]| {
        v.iter()
            .map(|x| x.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    };
    writeln!(
        out,
        "// rvec rx-symbol-pkg — generated from radio_phy, do not edit.\n\
         // Regenerate: make -C hw vectors\n\
         \n\
         `default_nettype none\n\
         \n\
         package rx_symbol_pkg;\n\
         \x20   // Widths of FixedOfdmConfig::default() (docs/gates/rx_symbol.md, G0).\n\
         \x20   localparam int PILOT_BITS = {};  // a pilot with its sign removed: U + 1\n\
         \x20   localparam int KAPPA_BITS = {};  // the demapper's constants, C = {} fractional bits\n\
         \x20   localparam int KAPPA_FRAC = {};\n\
         \x20   localparam int S_FRAC     = {};  // the soft scale's fractional bits F\n\
         \x20   localparam int SOFT_MAX   = {};  // soft values clip at ±SOFT_MAX\n\
         \x20   localparam int SOFT_BITS  = 8;\n\
         \x20   localparam int SQ_BITS    = 20;  // the soft scale q < 2·A (64-QAM's A has 19 bits)\n\
         \x20   // The 48 data subcarriers from -26 to 26 (IEEE 802.11-2020 §17.3.5.10): bin, number.\n\
         \x20   localparam int DATA_BIN [48] = '{{{}}};\n\
         \x20   localparam int DATA_C   [48] = '{{{}}};\n\
         \x20   // The pilots -21, -7, 7, 21 and their base signs (bit i: -1), §17.3.5.9.\n\
         \x20   localparam int PILOT_C [4] = '{{{}}};\n\
         \x20   localparam logic [3:0] PILOT_BASE_NEG = 4'b{:04b};\n\
         \x20   // The polarity sequence p_n, n = 0 .. 126 (bit n: -1), §17.3.5.10.\n\
         \x20   localparam logic [126:0] POLARITY_NEG = 127'h{:032x};\n\
         \x20   // The thresholds' constants per modulation (BPSK, QPSK, 16-QAM, 64-QAM): kappa of\n\
         \x20   // the first fold (16-QAM c = 2, 64-QAM c = 4) and of the second (64-QAM c = 2).\n\
         \x20   localparam int K_HI [4] = '{{{}}};\n\
         \x20   localparam int K_LO [4] = '{{{}}};\n\
         endpackage\n\n`default_nettype wire",
        fx.prod_bits + 1,
        k_hi.iter().chain(&k_lo).map(|&v| 64 - (v as u64).leading_zeros()).max().unwrap_or(1),
        fx.kappa_bits,
        fx.kappa_bits,
        fx.scale_frac,
        cfg.soft_max,
        bins.join(", "),
        nums.join(", "),
        pilots.join(", "),
        base_neg,
        pol,
        list(&k_hi),
        list(&k_lo),
    )
}
