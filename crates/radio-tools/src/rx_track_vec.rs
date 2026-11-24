//! `rvec rx_track` and `rvec rx-track-pkg`: the tracker block's vectors and constants
//! (`hw/rtl/rx_track.sv`, `docs/gates/rx_track.md`), from `FixedTracking::step` and
//! `FixedTracking::move_window`.

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::rx_front_fixed::{cordic_angle, IQ};
use radio_phy::rx_ofdm_fixed::{
    tracker_constants, FixedChannel, FixedOfdmConfig, FixedTracking, TrackerConstants,
    CLOCK_RATIO_FRAC,
};

/// The phase width P, as `FixedOfdmConfig::default()`.
const P: u32 = 22;
/// The last symbol index of any frame (SIGNAL 0, DATA k at k + 1; 1366 data symbols at most).
const LAST_SYMBOL: usize = 1366;
/// The CORDIC's input width (`rx_front_rom_pkg::CORDIC_IN_BITS`).
const CORDIC_IN_BITS: u32 = 34;
/// The fit's quotient bits and the registers that follow from them (G0,
/// `tests/g0_rx_track.rs`, `the_fit_never_saturates`).
const FIT_QBITS: u32 = 26;
/// The divider's operands for the fit: |num|·2^R and det (G0).
const FIT_NUM_BITS: u32 = 64;
const FIT_DEN_BITS: u32 = 47;

struct Packet {
    cfo: i64,
    ratio: u16,
    c: TrackerConstants,
    /// The pilots' true drift rate (R fractional bits per τ).
    rate: f64,
    amp: [f64; 4],
    noise: f64,
    wild: bool,
    symbols: usize,
}

/// A clock offset of `ppm` as the pilots' drift rate, and the front's CFO word for `seen` ppm
/// at 5.8 GHz and 20 MHz (ratio 3616).
fn offset(ppm: f64, seen: f64) -> (f64, i64) {
    let rate = ppm * 1e-6 * 16.0 * 65536.0 * 1024.0;
    let cfo = (seen * 1e-6 * 5.8e9 / 20e6 * f64::from(1u32 << P)).round() as i64;
    (rate, cfo.clamp(-(1 << (P - 1)), (1 << (P - 1)) - 1))
}

fn constants_of(gains: [i64; 4], gsum: i64, fx: &FixedOfdmConfig) -> TrackerConstants {
    let mut gain = [0i64; 64];
    for (&b, &g) in [43usize, 57, 7, 21].iter().zip(&gains) {
        gain[b] = g;
    }
    tracker_constants(
        &FixedChannel {
            hs: [(0, 0); 64],
            shift: 0,
            gain,
            gain_sum: gsum,
        },
        fx,
    )
}

/// A packet on a channel: pilot gains at one level (one lost now and then), a clock offset
/// within ±`ppm_max`, the CFO word near it or no ratio register.
fn real_packet(rng: &mut Rng, fx: &FixedOfdmConfig, symbols: usize, ppm_max: f64) -> Packet {
    let level = 1i64 << (4 + rng.next_u64() % 11);
    let mut gains = [0i64; 4];
    for g in gains.iter_mut() {
        *g = (((level as f64) * (0.05 + 1.5 * rng.uniform())) as i64).clamp(0, 32767);
    }
    if rng.next_u64().is_multiple_of(8) {
        gains[(rng.next_u64() % 4) as usize] = 0;
    }
    let gsum = gains.iter().sum::<i64>() * 13 + (rng.next_u64() % 1000) as i64;
    let ppm = (rng.uniform() - 0.5) * 2.0 * ppm_max;
    let ratio: u16 = if rng.next_u64().is_multiple_of(4) {
        0
    } else {
        3616
    };
    let (rate, cfo) = offset(ppm, ppm + 3.0 * rng.gaussian());
    Packet {
        cfo,
        ratio,
        c: constants_of(gains, gsum, fx),
        rate,
        amp: gains.map(|g| ((g as f64).sqrt() * 180.0).min(30000.0)),
        noise: 30.0 + 2000.0 * rng.uniform(),
        wild: false,
        symbols,
    }
}

/// The block's inputs anywhere in their formats: coefficients and variance at and inside their
/// ends, the CFO word and the ratio at theirs, pilots on the rails or anywhere.
fn limit_packet(rng: &mut Rng, case: usize) -> Packet {
    let pick = |rng: &mut Rng, bits: u32| -> i64 {
        match rng.next_u64() % 4 {
            0 => 0,
            1 => (1 << bits) - 1,
            _ => (rng.next_u64() % (1 << bits)) as i64,
        }
    };
    let c = TrackerConstants {
        outer: pick(rng, 14),
        inner: pick(rng, 14),
        variance: match case % 3 {
            0 => 1 + (rng.next_u64() % 3072) as i64,
            1 => [1, 3072, 3073, 4095, 0][(rng.next_u64() % 5) as usize],
            _ => pick(rng, 12),
        },
    };
    let lim = 1i64 << (P - 1);
    let cfo = match rng.next_u64() % 3 {
        0 => -lim,
        1 => lim - 1,
        _ => (rng.next_u64() % (2 * lim as u64)) as i64 - lim,
    };
    let ratio = match rng.next_u64() % 4 {
        0 => 0,
        1 => u16::MAX,
        _ => rng.next_u64() as u16,
    };
    Packet {
        cfo,
        ratio,
        c,
        rate: (rng.uniform() - 0.5) * 4.0e6,
        amp: [0.0; 4].map(|_: f64| 32768.0 * rng.uniform()),
        noise: 20000.0 * rng.uniform(),
        wild: case.is_multiple_of(2),
        symbols: 1 + (rng.next_u64() % 40) as usize,
    }
}

fn pilots(rng: &mut Rng, pk: &Packet, slope: f64) -> [IQ; 4] {
    let lim = 1i64 << 15;
    let mut p = [(0i64, 0i64); 4];
    if pk.wild {
        for v in p.iter_mut() {
            let mut pick = || match rng.next_u64() % 4 {
                0 => lim,
                1 => -lim,
                _ => (rng.next_u64() % (2 * lim as u64 + 1)) as i64 - lim,
            };
            *v = (pick(), pick());
        }
        return p;
    }
    let phi = rng.uniform() * 2.0 * std::f64::consts::PI;
    for ((v, &c), &a) in p.iter_mut().zip(&[-21.0f64, -7.0, 7.0, 21.0]).zip(&pk.amp) {
        let ph = phi + c * slope * 2.0 * std::f64::consts::PI / f64::from(1u32 << P);
        let q = |x: f64| (x.round() as i64).clamp(-lim, lim);
        *v = (
            q(a * ph.cos() + pk.noise * rng.gaussian()),
            q(a * ph.sin() + pk.noise * rng.gaussian()),
        );
    }
    p
}

fn packets(rng: &mut Rng, fx: &FixedOfdmConfig) -> Vec<Packet> {
    let mut v = Vec::new();
    for k in 0..110 {
        let n = 1 + (rng.next_u64() % if k % 5 == 0 { 300 } else { 60 }) as usize;
        v.push(real_packet(rng, fx, n, 300.0));
    }
    for case in 0..40 {
        v.push(limit_packet(rng, case));
    }
    // The longest frame: the sums at their largest, the drift into its saturation (64
    // samples) and the shift after it; the smallest λ and the largest.
    for k in 0..3 {
        let mut pk = real_packet(rng, fx, LAST_SYMBOL + 1, 0.0);
        pk.c.variance = [1, 3072, 7][k];
        let ppm = [1200.0, -700.0, 200.0][k];
        (pk.rate, pk.cfo) = offset(ppm, ppm);
        pk.ratio = 3616;
        pk.noise = [10.0, 10.0, 3000.0][k];
        pk.amp = [30000.0; 4];
        v.push(pk);
    }
    v.extend(threshold_packets(fx));
    v
}

/// Packets whose θ lands exactly on a move's threshold, ±2^(P−7) (θ equal to it moves
/// nothing): SIGNAL's from the CFO word, and a data symbol's in a packet that measures nothing
/// (its θ is the prior's drift whatever the pilots), found by a search over the word.
fn threshold_packets(fx: &FixedOfdmConfig) -> Vec<Packet> {
    let half = 1i64 << (P - 7);
    let none = TrackerConstants {
        outer: 0,
        inner: 0,
        variance: 0,
    };
    let packet = |cfo: i64, symbols: usize| Packet {
        cfo,
        ratio: 16384,
        c: none,
        rate: 0.0,
        amp: [0.0; 4],
        noise: 0.0,
        wild: true,
        symbols,
    };
    let mut v = vec![packet(1_198_373, 4), packet(-1_198_373, 4)];
    let pil = [(1i64, 0i64); 4];
    for sign in [1i64, -1] {
        'search: for k in 0..200_000i64 {
            let cfo = sign * (20_000 + k);
            let mut tr = FixedTracking::from_cfo_word(cfo, 16384, fx);
            for i in 0..40 {
                let Some(theta) = tr.step(&pil, &none, i, fx) else {
                    break;
                };
                tr.move_window(fx);
                if i > 0 && theta.abs() == half {
                    v.push(packet(cfo, i + 2));
                    break 'search;
                }
            }
        }
    }
    v
}

fn bad() -> io::Error {
    io::Error::other("rx_track: the model refused a case")
}

/// `rvec rx_track`: per packet its inputs, per symbol the pilots and what the model makes of
/// them; then CORDIC cases for the bench's other requester.
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    writeln!(
        out,
        "# rvec rx_track — generated, do not edit. Regenerate: make -C hw vectors\n\
         # radio_phy::rx_ofdm_fixed::FixedTracking::{{from_cfo_word, unknown, step, move_window}}, \
         FixedOfdmConfig::default().\n\
         # P <cfo word> <ratio> <outer> <inner> <variance> <symbols>\n\
         # S <pilots -21 -7 7 21, re im> <theta> <delta> <shift> (SIGNAL first: its theta is the start's)\n\
         # C <re> <im> <angle> (cordic_angle, for the bench's other requester)"
    )?;
    let mut rng = Rng::new(0x7_2ac5);
    for pk in packets(&mut rng, &fx) {
        writeln!(
            out,
            "P {} {} {} {} {} {}",
            pk.cfo, pk.ratio, pk.c.outer, pk.c.inner, pk.c.variance, pk.symbols
        )?;
        let mut tr = if pk.ratio == 0 {
            FixedTracking::unknown(&fx)
        } else {
            FixedTracking::from_cfo_word(pk.cfo, pk.ratio, &fx)
        };
        for i in 0..pk.symbols {
            let tau = 7.0 + 5.0 * i as f64;
            let slope = pk.rate * tau / 1024.0 + (tr.shift as f64) * 65536.0;
            let p = pilots(&mut rng, &pk, slope);
            let theta = tr.step(&p, &pk.c, i, &fx).ok_or_else(bad)?;
            tr.move_window(&fx);
            let ps: Vec<String> = p.iter().map(|v| format!("{} {}", v.0, v.1)).collect();
            writeln!(
                out,
                "S {} {} {} {}",
                ps.join(" "),
                theta,
                tr.delta,
                tr.shift
            )?;
        }
    }
    let hi = 1i64 << (CORDIC_IN_BITS - 1);
    for k in 0..400 {
        let pick = |rng: &mut Rng| -> i64 {
            let bits = 1 + rng.next_u64() % u64::from(CORDIC_IN_BITS - 1);
            let v = (rng.next_u64() % (1u64 << bits)) as i64;
            if rng.next_u64().is_multiple_of(2) {
                v
            } else {
                -v
            }
        };
        let (re, im) = match k {
            0 => (0, 0),
            1 => (-hi, -hi),
            2 => (hi - 1, -hi),
            _ => (pick(&mut rng), pick(&mut rng)),
        };
        writeln!(out, "C {re} {im} {}", cordic_angle(re, im, &fx.front))?;
    }
    Ok(())
}

/// `rx_track_pkg`: the prior's two weights and the widths G0 fixed.
pub fn package(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let known = FixedTracking::from_cfo_word(0, 1, &fx).weight;
    let unknown = FixedTracking::unknown(&fx).weight;
    let wbits = 64 - (known.max(unknown).max(1) as u64).leading_zeros();
    writeln!(
        out,
        "// rvec rx-track-pkg — generated from radio_phy, do not edit.\n\
         // Regenerate: make -C hw vectors\n\
         \n\
         `default_nettype none\n\
         \n\
         package rx_track_pkg;\n\
         \x20   // Widths of FixedOfdmConfig::default() (docs/gates/rx_track.md, G0).\n\
         \x20   localparam int RATE_FRAC   = {};  // R: the rates' fractional bits\n\
         \x20   localparam int RATE0_BITS  = {};  // rate0: P - 6 + R\n\
         \x20   localparam int FIT_QBITS   = {};  // the fit's quotient bits (|delta| < 2^FIT_QBITS)\n\
         \x20   localparam int FIT_NUM_BITS = {};  // the divider's operands: |num| * 2^R\n\
         \x20   localparam int FIT_DEN_BITS = {};  // and det\n\
         \x20   localparam int RATIO_SHIFT = {};  // rate0 = word * ratio >> (20 + 2 - R)\n\
         \x20   localparam int LAST_SYMBOL = {};  // SIGNAL 0, DATA k at k + 1\n\
         \x20   // The prior's weight: a clock offset known from the CFO word (TIMING_HINT_PPM) and an\n\
         \x20   // unknown one (TIMING_PRIOR_PPM), FixedTracking::{{from_cfo_word, unknown}}.\n\
         \x20   localparam int WEIGHT_BITS = {};\n\
         \x20   localparam logic [WEIGHT_BITS-1:0] WEIGHT_KNOWN   = {}'d{};\n\
         \x20   localparam logic [WEIGHT_BITS-1:0] WEIGHT_UNKNOWN = {}'d{};\n\
         endpackage\n\n`default_nettype wire",
        fx.rate_frac,
        fx.front.nco_phase_bits - 6 + fx.rate_frac,
        FIT_QBITS,
        FIT_NUM_BITS,
        FIT_DEN_BITS,
        CLOCK_RATIO_FRAC + 2 - fx.rate_frac,
        LAST_SYMBOL,
        wbits,
        wbits,
        known,
        wbits,
        unknown
    )
}
