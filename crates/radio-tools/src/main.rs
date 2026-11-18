//! `rvec` — writes the committed test vectors that the RTL gates (G3) replay.
//!
//! Vectors are generated, never hand-edited. CI regenerates every file and fails on any diff.
//!
//! Usage: `cargo run -q -p radio-tools --bin rvec -- <block>` writes the vectors to stdout.
//!
//! `ofdm-symbols` and `ppdu-frames` are not RTL vector files: they feed the OpenOFDM receiver
//! harnesses (`tools/ref/ofdm_rx_openofdm.py`, `tools/ref/ppdu_decode_openofdm.py`) and are
//! written to the ignored `target/ext/`.

use std::io::{self, Write};
use std::process::ExitCode;

use radio_phy::channel::{
    awgn, fractional_delay, frequency_offset, mean_power, tgn_taps, Doppler, Multipath, Rng,
    TgnModel,
};
use radio_phy::fixed::{requantize, Overflow, QFormat, Rounding};
use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, IfftConfig};
use radio_phy::ppdu::{build_ppdu, frame_hash, Rate};
use radio_phy::rx_front::{RxConfig, LTF_SYMBOL_LEN, PAIR_LEN};
use radio_phy::rx_front_fixed::{
    cordic_angle, cordic_table, ltf_corr, ltf_taps, nco_mix, nco_table, to_adc, FixedRxConfig,
    RxFrontFixed, ADC_FULL_SCALE, IQ, Y_BITS,
};
use radio_phy::scrambler::Scrambler;

/// Deterministic xorshift32 so vectors are reproducible without a `rand` dependency.
struct XorShift32(u32);

impl XorShift32 {
    fn next_bit(&mut self) -> u8 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x & 1) as u8
    }
}

fn bits_to_string(bits: &[u8]) -> String {
    bits.iter()
        .map(|b| if *b == 1 { '1' } else { '0' })
        .collect()
}

/// One line per vector: `<seed hex> <input bits> <scrambled bits>`, every non-zero seed once.
fn scrambler_vectors(out: &mut impl Write) -> io::Result<()> {
    const BITS_PER_VECTOR: usize = 256;
    writeln!(
        out,
        "# rvec scrambler — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(out, "# <seed hex> <input bits> <scrambled bits>")?;
    let mut rng = XorShift32(0x1234_5678);
    for seed in 1u8..=0x7f {
        let input: Vec<u8> = (0..BITS_PER_VECTOR).map(|_| rng.next_bit()).collect();
        let mut s = Scrambler::new(seed).map_err(io::Error::other)?;
        let output = s.scramble(&input);
        writeln!(
            out,
            "{seed:02x} {} {}",
            bits_to_string(&input),
            bits_to_string(&output)
        )?;
    }
    Ok(())
}

/// OFDM symbols for the OpenOFDM receiver harness: for each modulation, 8 consecutive symbols
/// with polarity indices 0..8 (a fresh receiver starts at p_0). One line per symbol:
/// `<n_bpsc> <polarity index> <frac bits> <48 points i,q> <80 samples re,im>` (raw integers).
fn ofdm_symbols(out: &mut impl Write) -> io::Result<()> {
    const SYMBOLS_PER_MODULATION: usize = 8;
    let cfg = IfftConfig::tx_default();
    writeln!(
        out,
        "# rvec ofdm-symbols — input to tools/ref/ofdm_rx_openofdm.py, tx_default IFFT"
    )?;
    let mut rng = XorShift32(0x0FD_0005);
    for m in [
        Modulation::Bpsk,
        Modulation::Qpsk,
        Modulation::Qam16,
        Modulation::Qam64,
    ] {
        for idx in 0..SYMBOLS_PER_MODULATION {
            let bits: Vec<u8> = (0..48 * m.bits_per_subcarrier())
                .map(|_| rng.next_bit())
                .collect();
            let points = map_bits(&bits, m).map_err(io::Error::other)?;
            let samples = modulate_symbol(&points, m, idx, &cfg).map_err(io::Error::other)?;
            let pts: Vec<String> = points.iter().map(|p| format!("{},{}", p.i, p.q)).collect();
            let smp: Vec<String> = samples
                .iter()
                .map(|s| format!("{},{}", s.re, s.im))
                .collect();
            writeln!(
                out,
                "{} {idx} {} {} {}",
                m.bits_per_subcarrier(),
                cfg.data.frac_bits(),
                pts.join(" "),
                smp.join(" ")
            )?;
        }
    }
    Ok(())
}

/// Whole PPDUs for the OpenOFDM decoder harness: three frames per rate with random PSDUs
/// (1 octet, tens of octets, hundreds of octets) and random scrambler seeds. One line per frame:
/// `<Mb/s> <seed hex> <psdu hex> <frac bits> <frame hash hex> <samples re,im ...>`.
fn ppdu_frames(out: &mut impl Write) -> io::Result<()> {
    let cfg = IfftConfig::tx_default();
    writeln!(
        out,
        "# rvec ppdu-frames — input to tools/ref/ppdu_decode_openofdm.py, tx_default IFFT"
    )?;
    let mut rng = XorShift32(0x0FD_0006);
    let mut word = || (0..32).fold(0u32, |acc, _| (acc << 1) | u32::from(rng.next_bit()));
    for rate in Rate::ALL {
        for size in [
            1usize,
            20 + (word() % 60) as usize,
            200 + (word() % 300) as usize,
        ] {
            let seed = (word() % 127 + 1) as u8;
            let psdu: Vec<u8> = (0..size).map(|_| word() as u8).collect();
            let frame = build_ppdu(&psdu, rate, seed, &cfg).map_err(io::Error::other)?;
            let hex: String = psdu.iter().map(|b| format!("{b:02x}")).collect();
            let smp: Vec<String> = frame.iter().map(|s| format!("{},{}", s.re, s.im)).collect();
            writeln!(
                out,
                "{} {seed:02x} {hex} {} {:016x} {}",
                rate.mbps(),
                cfg.data.frac_bits(),
                frame_hash(&frame),
                smp.join(" ")
            )?;
        }
    }
    Ok(())
}

/// A preamble + short PPDU through the channel simulator into the 12-bit ADC (packet RMS at
/// `dbfs`), with a random lead, CFO and phase, as in the G2 gate.
fn rx_scene(snr_db: f64, dbfs: f64, tgn_d: bool, seed: u64) -> io::Result<Vec<(i16, i16)>> {
    const FS: f64 = 20e6;
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let frame: Vec<(f64, f64)> = build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
        .map_err(io::Error::other)?
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect();
    let mut rng = Rng::new(seed);
    let lead = 200 + (rng.uniform() * 300.0) as usize;
    let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
    let mut x = vec![(0.0, 0.0); lead];
    x.extend_from_slice(&frame);
    x.extend(vec![(0.0, 0.0); 200]);
    let x = if tgn_d {
        let ch = Multipath::new(
            tgn_taps(TgnModel::D, false),
            Doppler::Static,
            1,
            FS,
            &mut rng,
        );
        ch.apply(&x)
    } else {
        fractional_delay(&x, rng.uniform())
    };
    let x = frequency_offset(&x, cfo, FS, 2.0 * std::f64::consts::PI * rng.uniform());
    let p = mean_power(&frame);
    let x = awgn(&x, snr_db, p, &mut rng);
    let rms = (p * (1.0 + 10f64.powf(-snr_db / 10.0))).sqrt();
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(dbfs / 20.0) / rms;
    Ok(to_adc(&x, gain))
}

/// One ltf_corr segment: a `reset` line, then per input sample `<z re> <z im> <c re> <c im>`
/// with `- -` until the first full 128-sample window.
fn ltf_corr_segment(
    out: &mut impl Write,
    label: &str,
    z: &[IQ],
    corr: &[Option<IQ>],
) -> io::Result<()> {
    writeln!(out, "reset {label}")?;
    for (zv, c) in z.iter().zip(corr) {
        match c {
            Some(c) => writeln!(out, "{} {} {} {}", zv.0, zv.1, c.0, c.1)?,
            None => writeln!(out, "{} {} - -", zv.0, zv.1)?,
        }
    }
    Ok(())
}

/// The model's correlator over a stream of Z-bit samples, one output per full window.
fn ltf_corr_stream(z: &[IQ], fx: &FixedRxConfig) -> io::Result<Vec<Option<IQ>>> {
    (0..z.len())
        .map(|n| match (n + 1).checked_sub(PAIR_LEN) {
            None => Ok(None),
            Some(i) => {
                let w: [IQ; PAIR_LEN] = std::array::from_fn(|k| z[i + k]);
                ltf_corr(&w, fx)
                    .map(Some)
                    .ok_or_else(|| io::Error::other("invalid FixedRxConfig"))
            }
        })
        .collect()
}

/// Vectors for `hw/rtl/ltf_corr.sv` (gate G3: `make -C hw ltf_corr`). Segments:
/// 1. received frames through `RxFrontFixed` (its own traced z and correlator outputs), AWGN
///    25/4 dB, TGn D 9 dB, a hot −3 dBFS frame;
/// 2. uniformly random full-scale z;
/// 3. extremes: z matched to the taps' signs at full scale (largest |c|), its negation, the
///    most negative value everywhere, alternating extremes.
fn ltf_corr_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    writeln!(
        out,
        "# rvec ltf_corr — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# Z {} bits, taps {} bits, output K {} bits. reset <label> starts a segment;",
        fx.z_bits, fx.tap_bits, fx.corr_bits
    )?;
    writeln!(
        out,
        "# then per sample: <z re> <z im> <c re> <c im> (- - before the first full window)"
    )?;
    let scenes = [
        ("frame_awgn_25dB", 25.0, -15.0, false, 0x6301u64),
        ("frame_awgn_4dB", 4.0, -15.0, false, 0x6302),
        ("frame_tgn_d_9dB", 9.0, -15.0, true, 0x6303),
        ("frame_hot_-3dBFS", 30.0, -3.0, false, 0x6304),
    ];
    for (label, snr, dbfs, tgn, seed) in scenes {
        let adc = rx_scene(snr, dbfs, tgn, seed)?;
        let mut rx = RxFrontFixed::new(RxConfig::new(20e6), fx);
        let (mut z, mut c) = (Vec::new(), Vec::new());
        let mut acquired = false;
        for s in adc {
            let (acq, t) = rx.push_traced(s);
            acquired |= acq.is_some();
            z.push(t.z);
            c.push(t.corr);
        }
        if !acquired {
            return Err(io::Error::other(format!("{label}: no acquisition")));
        }
        ltf_corr_segment(out, label, &z, &c)?;
    }

    let zmax = (1i64 << (fx.z_bits - 1)) - 1;
    let zmin = -(1i64 << (fx.z_bits - 1));
    let mut rng = XorShift32(0x0C0_4401);
    let mut word = || (0..32).fold(0u32, |acc, _| (acc << 1) | u32::from(rng.next_bit()));
    let span = 1u32 << fx.z_bits;
    let random: Vec<IQ> = (0..2000)
        .map(|_| {
            (
                i64::from(word() % span) + zmin,
                i64::from(word() % span) + zmin,
            )
        })
        .collect();
    ltf_corr_segment(out, "random", &random, &ltf_corr_stream(&random, &fx)?)?;

    let taps = ltf_taps(&fx);
    let sgn = |v: i64, hi: i64, lo: i64| if v < 0 { lo } else { hi };
    // conj(tap)·z maximal when z has the tap's component signs.
    let matched: Vec<IQ> = (0..3 * PAIR_LEN)
        .map(|k| {
            let t = taps[k % LTF_SYMBOL_LEN];
            (sgn(t.0, zmax, zmin), sgn(t.1, zmax, zmin))
        })
        .collect();
    ltf_corr_segment(out, "matched", &matched, &ltf_corr_stream(&matched, &fx)?)?;
    let anti: Vec<IQ> = matched
        .iter()
        .map(|&(a, b)| {
            (
                if a == zmax { zmin } else { zmax },
                if b == zmax { zmin } else { zmax },
            )
        })
        .collect();
    ltf_corr_segment(out, "anti_matched", &anti, &ltf_corr_stream(&anti, &fx)?)?;
    let most_negative = vec![(zmin, zmin); PAIR_LEN + 16];
    ltf_corr_segment(
        out,
        "most_negative",
        &most_negative,
        &ltf_corr_stream(&most_negative, &fx)?,
    )?;
    let alternating: Vec<IQ> = (0..PAIR_LEN + 64)
        .map(|k| match k % 4 {
            0 => (zmax, zmin),
            1 => (zmin, zmin),
            2 => (zmin, zmax),
            _ => (zmax, zmax),
        })
        .collect();
    ltf_corr_segment(
        out,
        "alternating",
        &alternating,
        &ltf_corr_stream(&alternating, &fx)?,
    )
}

/// The receiver front's constant ROMs as a SystemVerilog package (`hw/rtl/gen/`), generated
/// from the model so the RTL cannot drift from it. The correlator folds tap pairs (k, 64 − k),
/// so the generator refuses taps that are not Hermitian.
fn rx_front_rom(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    let taps = ltf_taps(&fx);
    let hermitian = taps[0].1 == 0
        && taps[32].1 == 0
        && (1..LTF_SYMBOL_LEN).all(|k| taps[LTF_SYMBOL_LEN - k] == (taps[k].0, -taps[k].1));
    if !hermitian {
        return Err(io::Error::other("LTF taps are not Hermitian"));
    }
    let t = fx.tap_bits;
    writeln!(
        out,
        "// rvec rx-front-rom — generated from radio_phy::rx_front_fixed, do not edit.\n\
         // Regenerate: make -C hw vectors\n\
         \n\
         `default_nettype none\n\
         \n\
         package rx_front_rom_pkg;\n\
         \x20   // Formats of FixedRxConfig::default().\n\
         \x20   localparam int Z_BITS    = {};\n\
         \x20   localparam int TAP_BITS  = {t};\n\
         \x20   localparam int CORR_BITS = {};\n\
         \x20   localparam int Y_BITS    = {};\n\
         \x20   localparam int CORDIC_IN_BITS = {CORDIC_IN_BITS};\n\
         \x20   localparam int CORDIC_ITERS   = {};\n\
         \x20   localparam int CORDIC_BITS    = {};\n\
         \x20   localparam int ANGLE_BITS     = {};\n\
         \x20   localparam int NCO_PHASE_BITS = {};\n\
         \x20   localparam int NCO_LUT_BITS   = {};\n\
         \x20   localparam int NCO_AMP_BITS   = {};\n\
         \n\
         \x20   // LTF taps t(k), k = 0..32 (t(64 - k) = conj(t(k)); t(0), t(32) real): ltf_taps().",
        fx.z_bits,
        fx.corr_bits,
        Y_BITS,
        fx.cordic_iters,
        fx.cordic_bits,
        fx.angle_bits,
        fx.nco_phase_bits,
        fx.nco_lut_bits,
        fx.nco_amp_bits
    )?;
    for (name, part) in [("ltf_tap_re", 0usize), ("ltf_tap_im", 1)] {
        writeln!(
            out,
            "    function automatic logic signed [{}:0] {name}(input int k);\n        case (k)",
            t - 1
        )?;
        for (k, tap) in taps.iter().enumerate().take(LTF_SYMBOL_LEN / 2 + 1) {
            let v = if part == 0 { tap.0 } else { tap.1 };
            let lit = if v < 0 {
                format!("-{t}'sd{}", -v)
            } else {
                format!("{t}'sd{v}")
            };
            writeln!(out, "            {k:2}: {name} = {lit};")?;
        }
        writeln!(
            out,
            "            default: {name} = '0;\n        endcase\n    endfunction"
        )?;
    }
    let atan = cordic_table(&fx);
    rom_function(
        out,
        "cordic_atan",
        "CORDIC arctangent table: round(atan(2^-i) 2^A / 2pi), cordic_table()",
        fx.angle_bits,
        &atan,
    )?;
    let nco = nco_table(&fx);
    let cos: Vec<i64> = nco.iter().map(|v| v.0).collect();
    let sin: Vec<i64> = nco.iter().map(|v| v.1).collect();
    rom_function(
        out,
        "nco_cos",
        "NCO quarter-wave table, cos part: nco_table()",
        fx.nco_amp_bits,
        &cos,
    )?;
    rom_function(
        out,
        "nco_sin",
        "NCO quarter-wave table, sin part: nco_table()",
        fx.nco_amp_bits,
        &sin,
    )?;
    writeln!(out, "endpackage\n\n`default_nettype wire")
}

/// One constant ROM of `rx_front_rom_pkg`: `function automatic logic signed [w-1:0] name(int i)`.
fn rom_function(
    out: &mut impl Write,
    name: &str,
    doc: &str,
    w: u32,
    values: &[i64],
) -> io::Result<()> {
    writeln!(
        out,
        "\n    // {doc}.\n    function automatic logic signed [{}:0] {name}(input int i);\n        case (i)",
        w - 1
    )?;
    for (i, &v) in values.iter().enumerate() {
        let lit = if v < 0 {
            format!("-{w}'sd{}", -v)
        } else {
            format!("{w}'sd{v}")
        };
        writeln!(out, "            {i:3}: {name} = {lit};")?;
    }
    writeln!(
        out,
        "            default: {name} = '0;\n        endcase\n    endfunction"
    )
}

/// Input width of `hw/rtl/cordic.sv`: the receiver's widest angle input, the provisional
/// latch's lag-16 sum of 96 products of D-bit samples (|re|, |im| < 96 · 2 · 2^(2D−2) < 2^33
/// at D = 13), as a signed register.
const CORDIC_IN_BITS: u32 = 34;

/// Vectors for `hw/rtl/cordic.sv` (gate G3: `make -C hw cordic`): `<re> <im> <angle>` per
/// line, angle = `cordic_angle(re, im)`. Edges (zero, the axes, the extremes of the input
/// width, the negative real axis where the half-turn applies), then random vectors with re
/// and im at independent magnitudes from 1 to 34 bits (the block normalisation shifts left or
/// right).
fn cordic_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    let hi = (1i64 << (CORDIC_IN_BITS - 1)) - 1;
    let lo = -(1i64 << (CORDIC_IN_BITS - 1));
    writeln!(
        out,
        "# rvec cordic — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# input {CORDIC_IN_BITS} bits signed, N = {}, W_c = {}, A = {}: <re> <im> <angle>",
        fx.cordic_iters, fx.cordic_bits, fx.angle_bits
    )?;
    let mut cases: Vec<(i64, i64)> = Vec::new();
    let edges = [
        0,
        1,
        -1,
        2,
        -2,
        3,
        hi,
        lo,
        hi - 1,
        lo + 1,
        1 << 14,
        -(1 << 14),
        (1 << 15) - 1,
        -(1 << 15),
    ];
    for &a in &edges {
        for &b in &edges {
            cases.push((a, b));
        }
    }
    let mut rng = XorShift32(0xC0_D1C0);
    let mut word = || (0..32).fold(0u64, |acc, _| (acc << 1) | u64::from(rng.next_bit()));
    for k in 0..6000u64 {
        // re and im at independent bit lengths: one large and one tiny component too
        let n = u64::from(CORDIC_IN_BITS);
        let mut draw = |bits: u32| {
            let span = 1u64 << bits;
            (((word() << 32 | word()) % span) as i64 - (span / 2) as i64).clamp(lo, hi)
        };
        let a = draw(1 + (k % n) as u32);
        let b = draw(1 + ((k / n + 7 * k) % n) as u32);
        cases.push((a, b));
    }
    for (re, im) in cases {
        writeln!(out, "{re} {im} {}", cordic_angle(re, im, &fx))?;
    }
    Ok(())
}

/// One nco segment: `reset <label>`, then per sample `<y re> <y im> <step> <z re> <z im>`. The
/// phase starts at 0 after reset and advances by the sample's step after the sample.
fn nco_segment(out: &mut impl Write, label: &str, y: &[IQ], step: &[i64]) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    let full = 1i64 << fx.nco_phase_bits;
    writeln!(out, "reset {label}")?;
    let mut phase = 0i64;
    for (&yv, &st) in y.iter().zip(step) {
        let z = nco_mix(yv, phase, &fx).ok_or_else(|| io::Error::other("invalid FixedRxConfig"))?;
        writeln!(out, "{} {} {st} {} {}", yv.0, yv.1, z.0, z.1)?;
        phase = (phase + st).rem_euclid(full);
    }
    Ok(())
}

/// Vectors for `hw/rtl/nco.sv` (gate G3: `make -C hw nco`). Segments:
/// 1. the receiver's own y and step per sample (`RxFrontFixed` traces; the generator checks its
///    phase sequence is the accumulation of its steps): AWGN 25/4 dB, TGn D 9 dB, hot −3 dBFS;
/// 2. random full-scale y and random steps (any P-bit value);
/// 3. a table sweep: one table address per sample over two turns, y at both extremes;
/// 4. the most negative y at quadrant boundaries.
fn nco_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    let p = fx.nco_phase_bits;
    writeln!(
        out,
        "# rvec nco — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# y {Y_BITS} bits, phase P {p} bits, table L {} bits, amplitude Q {}, z {} bits.",
        fx.nco_lut_bits, fx.nco_amp_bits, fx.z_bits
    )?;
    writeln!(
        out,
        "# reset <label> starts a segment (phase 0); then <y re> <y im> <step> <z re> <z im>"
    )?;
    let scenes = [
        ("frame_awgn_25dB", 25.0, -15.0, false, 0x6301u64),
        ("frame_awgn_4dB", 4.0, -15.0, false, 0x6302),
        ("frame_tgn_d_9dB", 9.0, -15.0, true, 0x6303),
        ("frame_hot_-3dBFS", 30.0, -3.0, false, 0x6304),
    ];
    let full = 1i64 << p;
    for (label, snr, dbfs, tgn, seed) in scenes {
        let adc = rx_scene(snr, dbfs, tgn, seed)?;
        let mut rx = RxFrontFixed::new(RxConfig::new(20e6), fx);
        let (mut y, mut step) = (Vec::new(), Vec::new());
        let mut phase = 0i64;
        let mut moved = false;
        for s in adc {
            let (_, t) = rx.push_traced(s);
            if t.phase != phase {
                return Err(io::Error::other(format!(
                    "{label}: phase is not the step sum"
                )));
            }
            moved |= t.step != 0;
            y.push(t.y);
            step.push(t.step);
            phase = (phase + t.step).rem_euclid(full);
        }
        if !moved {
            return Err(io::Error::other(format!("{label}: the NCO never moved")));
        }
        nco_segment(out, label, &y, &step)?;
    }

    let ymax = (1i64 << (Y_BITS - 1)) - 1;
    let ymin = -(1i64 << (Y_BITS - 1));
    let mut rng = XorShift32(0x0C0_0A11);
    let mut word = || (0..32).fold(0u64, |acc, _| (acc << 1) | u64::from(rng.next_bit()));
    let yspan = 1u64 << Y_BITS;
    let (mut y, mut step) = (Vec::new(), Vec::new());
    for _ in 0..3000 {
        y.push((
            (word() % yspan) as i64 + ymin,
            (word() % yspan) as i64 + ymin,
        ));
        step.push((word() % (full as u64)) as i64 - full / 2);
    }
    nco_segment(out, "random", &y, &step)?;

    let addr_step = 1i64 << (p - fx.nco_lut_bits);
    let n = 2usize << fx.nco_lut_bits;
    let y: Vec<IQ> = (0..n)
        .map(|k| {
            if k % 2 == 0 {
                (ymax, ymax)
            } else {
                (ymin, ymax)
            }
        })
        .collect();
    nco_segment(out, "table_sweep", &y, &vec![addr_step; n])?;
    let y = vec![(ymin, ymin); 16];
    nco_segment(out, "quadrants", &y, &[full / 4 - 1; 16])
}

/// (SHIFT, OUT_W) of the round_sat instances in `hw/tb/round_sat_bank.sv`, 12-bit input.
const ROUND_SAT_SETS: [(u32, u32); 8] = [
    (1, 12),
    (1, 10),
    (4, 9),
    (4, 8),
    (4, 6),
    (8, 4),
    (11, 2),
    (11, 1),
];

/// Vectors for `hw/rtl/round_sat.sv` (gate G3: `make -C hw round_sat`): every 12-bit input,
/// narrowed by `requantize(HalfEven, Saturate)` from `SHIFT + OUT_W − 1` to `OUT_W − 1`
/// fractional bits: drop SHIFT bits, keep OUT_W (`rx_front_fixed::narrow` with a value that may
/// exceed its nominal width, as the DC blocker's does).
fn round_sat_vectors(out: &mut impl Write) -> io::Result<()> {
    const IN_W: u32 = 12;
    writeln!(
        out,
        "# rvec round_sat — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    let sets: Vec<String> = ROUND_SAT_SETS
        .iter()
        .map(|(s, w)| format!("s{s}w{w}"))
        .collect();
    writeln!(
        out,
        "# <12-bit input> then one output per set: {}",
        sets.join(" ")
    )?;
    for x in -(1i64 << (IN_W - 1))..(1i64 << (IN_W - 1)) {
        let mut fields = vec![x.to_string()];
        for (shift, w) in ROUND_SAT_SETS {
            let q = QFormat::new(w, w - 1).map_err(io::Error::other)?;
            let v = requantize(
                x,
                shift + (w - 1),
                q,
                Rounding::HalfEven,
                Overflow::Saturate,
            );
            fields.push(v.to_string());
        }
        writeln!(out, "{}", fields.join(" "))?;
    }
    Ok(())
}

fn main() -> ExitCode {
    let block = std::env::args().nth(1).unwrap_or_default();
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let result = match block.as_str() {
        "scrambler" => scrambler_vectors(&mut out),
        "ofdm-symbols" => ofdm_symbols(&mut out),
        "ppdu-frames" => ppdu_frames(&mut out),
        "ltf_corr" => ltf_corr_vectors(&mut out),
        "round_sat" => round_sat_vectors(&mut out),
        "rx-front-rom" => rx_front_rom(&mut out),
        "cordic" => cordic_vectors(&mut out),
        "nco" => nco_vectors(&mut out),
        _ => {
            eprintln!(
                "usage: rvec <block>\nblocks: scrambler, round_sat, ltf_corr, cordic, nco, rx-front-rom, ofdm-symbols, ppdu-frames"
            );
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rvec: {e}");
            ExitCode::FAILURE
        }
    }
}
