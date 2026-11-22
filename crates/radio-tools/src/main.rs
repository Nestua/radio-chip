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
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, tgn_taps, Doppler,
    Multipath, Rng, TgnModel,
};
use radio_phy::convolutional::ConvolutionalEncoder;
use radio_phy::fixed::{requantize, Overflow, QFormat, Rounding};
use radio_phy::mapper::map_bits;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, IfftConfig};
use radio_phy::ppdu::{build_ppdu, frame_hash, Rate};
use radio_phy::rx_decision::{run_schedule, DecisionTiming};
use radio_phy::rx_front::{
    backoff_samples, RxConfig, AUTOCORR_LEN, COARSE_LEN, DECISION_LATENCY, FINE_GUARD, FINE_LEN,
    GI2_LEN, GI_LEN, HISTORY, LATCH_LAG, LTF_SYMBOL_LEN, PAIR_LEN, SEARCH_LEN, SEARCH_OFFSET,
    STF_PERIOD,
};
use radio_phy::rx_front_fixed::{
    cordic_angle, cordic_table, detector_trace_fixed, ltf_corr, ltf_taps, nco_mix, nco_table,
    to_adc, FixedRxConfig, RxFrontFixed, ADC_FULL_SCALE, IQ, Y_BITS,
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
    let rx = RxConfig::new(20e6);
    let dc_shift = rx
        .dc_shift
        .ok_or_else(|| io::Error::other("the default receiver has no DC blocker"))?;
    let det_run = rx.run;
    let search_w = GI_LEN.saturating_sub(backoff_samples(&rx));
    let decision_latency = rx.decision_latency;
    let est_bits = fx.est_bits;
    let tap_energy: i64 = ltf_taps(&fx).iter().map(|t| t.0 * t.0 + t.1 * t.1).sum();
    if fx.coarse_weight_64 != 0 || fx.nco_phase_bits != fx.angle_bits + 6 {
        return Err(io::Error::other(
            "the decision's RTL has no coarse weight and takes the fine angle as the step",
        ));
    }
    if decision_latency != DECISION_LATENCY {
        return Err(io::Error::other("the default decision latency moved"));
    }
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
         \x20   localparam int DET_BITS       = {};\n\
         \x20   localparam int DC_FRAC        = {};\n\
         \x20   localparam int DC_SHIFT       = {dc_shift};\n\
         \x20   localparam int STF_PERIOD     = {STF_PERIOD};\n\
         \x20   localparam int AUTOCORR_LEN   = {AUTOCORR_LEN};\n\
         \x20   localparam int DET_RUN        = {det_run};\n\
         \x20   localparam int THETA_NUM      = {};\n\
         \x20   localparam int THETA_SHIFT    = {};\n\
         \x20   // Provisional latch: COARSE_LEN lag-16 products ending LATCH_LAG samples back.\n\
         \x20   localparam int LATCH_LAG      = {LATCH_LAG};\n\
         \x20   localparam int COARSE_LEN     = {COARSE_LEN};\n\
         \x20   // LTF search: positions from SEARCH_OFFSET after the opening event to SEARCH_LEN\n\
         \x20   // after the latest; SEARCH_W = GI_LEN - back-off energy positions (RxConfig::new);\n\
         \x20   // a decision's result DECISION_LATENCY samples after its acceptance.\n\
         \x20   localparam int LTF_SYMBOL_LEN   = {LTF_SYMBOL_LEN};\n\
         \x20   localparam int GI_LEN           = {GI_LEN};\n\
         \x20   localparam int SEARCH_OFFSET    = {SEARCH_OFFSET};\n\
         \x20   localparam int SEARCH_LEN       = {SEARCH_LEN};\n\
         \x20   localparam int SEARCH_W         = {search_w};\n\
         \x20   localparam int DECISION_LATENCY = {decision_latency};\n\
         \x20   localparam int HISTORY          = {HISTORY};\n\
         \x20   // The decision: samples of the coarse and fine sums at EST_BITS; the fine sum's\n\
         \x20   // FINE_LEN lag-64 products from FINE_GUARD into GI2; the taps' energy E_L.\n\
         \x20   localparam int EST_BITS         = {est_bits};\n\
         \x20   localparam int GI2_LEN          = {GI2_LEN};\n\
         \x20   localparam int FINE_GUARD       = {FINE_GUARD};\n\
         \x20   localparam int FINE_LEN         = {FINE_LEN};\n\
         \x20   localparam int LTF_TAP_ENERGY   = {tap_energy};\n\
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
        fx.nco_amp_bits,
        fx.det_bits,
        fx.dc_frac,
        fx.theta_num,
        fx.theta_shift
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

/// One det segment: `reset <label>`, then per sample `<x re> <x im> <y re> <y im> <fire> <event>`.
fn det_segment(out: &mut impl Write, label: &str, x: &[(i16, i16)]) -> io::Result<usize> {
    let tr = detector_trace_fixed(x, &RxConfig::new(20e6), &FixedRxConfig::default());
    if tr.len() != x.len() {
        return Err(io::Error::other("invalid FixedRxConfig"));
    }
    writeln!(out, "reset {label}")?;
    for (s, t) in x.iter().zip(&tr) {
        writeln!(
            out,
            "{} {} {} {} {} {}",
            s.0,
            s.1,
            t.y.0,
            t.y.1,
            u8::from(t.fire),
            u8::from(t.event)
        )?;
    }
    Ok(tr.iter().filter(|t| t.event).count())
}

/// Vectors for `hw/rtl/dc_block.sv` + `hw/rtl/det_a.sv` (gate G3: `make -C hw det`): the
/// receiver's step 1 per sample (`detector_trace_fixed`). Segments: four received frames (as
/// for `ltf_corr`); noise alone at −30 dBFS; noise with a CW tone at the noise level and 10 dB
/// above it; a −20 dB AGC step before a frame (the DC transient); a large DC offset under a
/// frame; full-scale random samples; full-scale steps (the DC blocker's largest error, y at its
/// saturation bound).
fn det_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    writeln!(
        out,
        "# rvec det — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# x 12-bit ADC, y {Y_BITS} bits (DC blocker, F = {}), detector A D = {} bits, θ² = {}/2^{}.",
        fx.dc_frac, fx.det_bits, fx.theta_num, fx.theta_shift
    )?;
    writeln!(
        out,
        "# reset <label> starts a segment; then <x re> <x im> <y re> <y im> <fire> <event>"
    )?;
    let scenes = [
        ("frame_awgn_25dB", 25.0, -15.0, false, 0x6301u64),
        ("frame_awgn_4dB", 4.0, -15.0, false, 0x6302),
        ("frame_tgn_d_9dB", 9.0, -15.0, true, 0x6303),
        ("frame_hot_-3dBFS", 30.0, -3.0, false, 0x6304),
    ];
    let mut events = 0;
    for (label, snr, dbfs, tgn, seed) in scenes {
        events += det_segment(out, label, &rx_scene(snr, dbfs, tgn, seed)?)?;
    }
    if events == 0 {
        return Err(io::Error::other("no detection event in the frame segments"));
    }
    let fs = 20e6;
    let full = f64::from(ADC_FULL_SCALE);
    let mut rng = Rng::new(0xDE7_0001);
    let noise = |n: usize, dbfs: f64, rng: &mut Rng| -> Vec<(f64, f64)> {
        let p = (full * 10f64.powf(dbfs / 20.0)).powi(2);
        awgn(&vec![(0.0, 0.0); n], 0.0, p, rng)
    };
    let q = |x: &[(f64, f64)]| to_adc(x, 1.0);
    det_segment(out, "noise_-30dBFS", &q(&noise(3000, -30.0, &mut rng)))?;
    let pn = (full * 10f64.powf(-30.0 / 20.0)).powi(2);
    let n1 = noise(3000, -30.0, &mut rng);
    det_segment(out, "tone_0dB", &q(&add_tone(&n1, pn, 1.3e6, fs, 0.4)))?;
    let n2 = noise(3000, -30.0, &mut rng);
    det_segment(
        out,
        "tone_10dB",
        &q(&add_tone(&n2, 10.0 * pn, -0.7e6, fs, 1.1)),
    )?;
    let frame = rx_scene(15.0, -5.0, false, 0x6305)?;
    let xf: Vec<(f64, f64)> = frame
        .iter()
        .map(|&(a, b)| (f64::from(a), f64::from(b)))
        .collect();
    det_segment(out, "agc_step", &q(&gain_step(&xf, 150, -20.0)))?;
    let dc: Vec<(f64, f64)> = rx_scene(15.0, -20.0, false, 0x6306)?
        .iter()
        .map(|&(a, b)| (f64::from(a) + 1500.0, f64::from(b) - 1200.0))
        .collect();
    det_segment(out, "dc_offset", &q(&dc))?;
    let mut bits = XorShift32(0xDE7_0002);
    let mut word = || (0..32).fold(0u32, |acc, _| (acc << 1) | u32::from(bits.next_bit()));
    let lo = -ADC_FULL_SCALE;
    let random: Vec<(i16, i16)> = (0..2000)
        .map(|_| {
            let v = |w: u32| (i64::from(w % 4096) + i64::from(lo)) as i16;
            (v(word()), v(word()))
        })
        .collect();
    det_segment(out, "random_full_scale", &random)?;
    let hi = (ADC_FULL_SCALE - 1) as i16;
    let steps: Vec<(i16, i16)> = (0..1200)
        .map(|k| match (k / 150) % 4 {
            0 => (hi, -hi - 1),
            1 => (-hi - 1, hi),
            2 => (hi, hi),
            _ => (-hi - 1, -hi - 1),
        })
        .collect();
    det_segment(out, "full_scale_steps", &steps)?;
    Ok(())
}

/// One latch segment: `reset <label>`, then per sample
/// `<x re> <x im> <acq> <event> <step> <z re> <z im>` from `RxFrontFixed::push_traced`. `acq` is
/// 1 at the sample an acquisition is applied (the RTL drives `det_a.clear` from it until the
/// decision engine exists, part d step 3). Returns (events, latched events, acquisitions).
fn latch_segment(
    out: &mut dyn Write,
    label: &str,
    x: &[(i16, i16)],
) -> io::Result<(usize, usize, usize)> {
    let mut rx = RxFrontFixed::new(RxConfig::new(20e6), FixedRxConfig::default());
    let gate = LATCH_LAG + STF_PERIOD + COARSE_LEN;
    let (mut events, mut latched, mut acqs) = (0, 0, 0);
    writeln!(out, "reset {label}")?;
    for (n, s) in x.iter().enumerate() {
        let (acq, t) = rx.push_traced(*s);
        events += usize::from(t.event);
        latched += usize::from(t.event && n + 1 >= gate);
        acqs += usize::from(acq.is_some());
        writeln!(
            out,
            "{} {} {} {} {} {} {}",
            s.0,
            s.1,
            u8::from(acq.is_some()),
            u8::from(t.event),
            t.step,
            t.z.0,
            t.z.1
        )?;
    }
    Ok((events, latched, acqs))
}

/// A labelled stream of ADC samples.
type Scene = (String, Vec<(i16, i16)>);

/// The scenes of the latch gate (and, with others, of the search gate): see [`latch_vectors`].
fn latch_scenes(fx: &FixedRxConfig) -> io::Result<Vec<Scene>> {
    let fx = *fx;
    let mut scenes = Vec::new();
    let frames = [
        ("frame_awgn_25dB", 25.0, -15.0, false, 0x6301u64),
        ("frame_awgn_4dB", 4.0, -15.0, false, 0x6302),
        ("frame_tgn_d_9dB", 9.0, -15.0, true, 0x6303),
        ("frame_hot_-3dBFS", 30.0, -3.0, false, 0x6304),
    ];
    for (label, snr, dbfs, tgn, seed) in frames {
        scenes.push((label.to_string(), rx_scene(snr, dbfs, tgn, seed)?));
    }
    let fs = 20e6;
    let full = f64::from(ADC_FULL_SCALE);
    let mut rng = Rng::new(0x1A7C_0001);
    let pn = (full * 10f64.powf(-30.0 / 20.0)).powi(2);
    let noise = awgn(&vec![(0.0, 0.0); 3000], 0.0, pn, &mut rng);
    scenes.push((
        "tone_10dB_from_0".to_string(),
        to_adc(&add_tone(&noise, 10.0 * pn, 0.9e6, fs, 0.3), 1.0),
    ));
    // An event at exactly the first latching sample (n = gate − 1) and one just before it. A
    // clean tone fires from the first full window whatever its start, with events every
    // DET_RUN samples; eight full-scale samples break the run, and their position sets the
    // phase of the events after them.
    let gate = LATCH_LAG + STF_PERIOD + COARSE_LEN;
    let burst = [
        (2047i16, -2048i16),
        (-2048, 2047),
        (2047, 2047),
        (-2048, -2048),
    ];
    for (label, at) in [
        ("event_at_first_latch", gate - 1),
        ("event_before_first_latch", gate - 2),
    ] {
        let tone = to_adc(
            &add_tone(&vec![(0.0, 0.0); 400], 0.01 * full * full, 0.9e6, fs, 0.3),
            1.0,
        );
        let scene = (20..20 + 2 * STF_PERIOD).find_map(|pos| {
            let mut x = tone.clone();
            for (k, v) in x.iter_mut().skip(pos).take(8).enumerate() {
                *v = burst[(k / 2) % 4];
            }
            let mut rx = RxFrontFixed::new(RxConfig::new(fs), fx);
            let mut hit = false;
            for (n, s) in x.iter().enumerate() {
                hit |= rx.push_traced(*s).1.event && n == at;
            }
            hit.then_some(x)
        });
        let x = scene
            .ok_or_else(|| io::Error::other(format!("no burst position puts an event at {at}")))?;
        scenes.push((label.to_string(), x));
    }
    let mut two: Vec<(f64, f64)> = Vec::new();
    for seed in [0x1A7C_0002u64, 0x1A7C_0003] {
        two.extend(
            rx_scene(20.0, -20.0, false, seed)?
                .iter()
                .map(|&(a, b)| (f64::from(a), f64::from(b))),
        );
    }
    let p_two = mean_power(&two);
    scenes.push((
        "two_frames_under_tone".to_string(),
        to_adc(&add_tone(&two, 0.05 * p_two, -1.7e6, fs, 2.0), 1.0),
    ));
    // An acquisition in the middle of a run of fires: a tone 3 dB over the frame keeps detector
    // A firing through the preamble (5.1 MHz is near a multiple of fs/16, so the search still
    // finds the LTF). The counter reset at the acquisition then moves the later events: this
    // is the gate of `det_a.clear` and its alignment.
    {
        let cfg = IfftConfig::tx_default();
        let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
        let frame: Vec<(f64, f64)> = build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
            .map_err(io::Error::other)?
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        let p = mean_power(&frame);
        let mut x = vec![(0.0, 0.0); 300];
        x.extend_from_slice(&frame);
        x.extend(vec![(0.0, 0.0); 300]);
        let mut rng = Rng::new(0x1A7C_0005);
        let x = awgn(&add_tone(&x, 2.0 * p, 5.1e6, fs, 0.7), 25.0, p, &mut rng);
        let x = to_adc(&x, 0.1 * full / (3.0 * p).sqrt());
        let mut rx = RxFrontFixed::new(RxConfig::new(fs), fx);
        let with_clear: Vec<bool> = x.iter().map(|s| rx.push_traced(*s).1.event).collect();
        let without = detector_trace_fixed(&x, &RxConfig::new(fs), &fx);
        if with_clear.iter().zip(&without).all(|(a, b)| *a == b.event) {
            return Err(io::Error::other(
                "the acquisition's counter reset changes no event in frame_under_strong_tone",
            ));
        }
        scenes.push(("frame_under_strong_tone".to_string(), x));
    }
    let mut bits = XorShift32(0x1A7C_0004);
    let mut word = || (0..32).fold(0u32, |acc, _| (acc << 1) | u32::from(bits.next_bit()));
    let lo = -ADC_FULL_SCALE;
    let random: Vec<(i16, i16)> = (0..2000)
        .map(|_| {
            let v = |w: u32| (i64::from(w % 4096) + i64::from(lo)) as i16;
            (v(word()), v(word()))
        })
        .collect();
    scenes.push(("random_full_scale".to_string(), random));
    let hi = (ADC_FULL_SCALE - 1) as i16;
    let steps: Vec<(i16, i16)> = (0..1200)
        .map(|k| match (k / 150) % 4 {
            0 => (hi, -hi - 1),
            1 => (-hi - 1, hi),
            2 => (hi, hi),
            _ => (-hi - 1, -hi - 1),
        })
        .collect();
    scenes.push(("full_scale_steps".to_string(), steps));
    Ok(scenes)
}

/// Vectors for the provisional latch and the NCO path (gate G3: `make -C hw latch`):
/// `dc_block` → `det_a` → `rx_latch` → `nco` against `RxFrontFixed` sample by sample (the
/// NCO step after each sample's latch, and the rotated sample z). Segments: the four received
/// frames of `det`; a CW tone 10 dB over the noise from sample 0 (events before the latch's
/// first sample 135, which must not latch, and a step that keeps changing); clean tones timed to
/// put an event exactly at sample 135 (the first that latches) and at 134 (the last that does
/// not); two frames under a tone (acquisitions, then events again on the tone); a frame under a
/// tone 3 dB above it (an acquisition inside a run of fires: the counter reset moves the events
/// after it); full-scale random samples and full-scale steps (the widest C).
fn latch_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    writeln!(
        out,
        "# rvec latch — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# RxFrontFixed (default widths, D = {}, A = {}, P = {}, Z = {}) per sample.",
        fx.det_bits, fx.angle_bits, fx.nco_phase_bits, fx.z_bits
    )?;
    writeln!(
        out,
        "# reset <label> starts a segment; then <x re> <x im> <acq> <event> <step> <z re> <z im>"
    )?;
    let mut total = (0, 0, 0);
    let mut early = 0;
    for (label, x) in latch_scenes(&fx)? {
        let (e, l, a) = latch_segment(out, &label, &x)?;
        total = (total.0 + e, total.1 + l, total.2 + a);
        early += e - l;
    }
    // The gate needs every case it claims to cover.
    if total.1 == 0 || early == 0 || total.2 == 0 {
        return Err(io::Error::other(format!(
            "latch vectors miss a case: {} events, {} latched, {early} before the latch start, {} acquisitions",
            total.0, total.1, total.2
        )));
    }
    writeln!(
        out,
        "# totals: {} events, {} latched, {early} before the latch start, {} acquisitions",
        total.0, total.1, total.2
    )?;
    Ok(())
}

/// What a search segment exercises (the cases `rvec search` refuses to do without).
#[derive(Debug, Default, Clone, Copy)]
struct SearchCover {
    samples: usize,
    events: usize,
    /// Events before the first correlator output.
    early_events: usize,
    /// Events while a search is open (the deadline moves).
    extensions: usize,
    acceptances: usize,
    acquisitions: usize,
    /// Searches closed by their deadline.
    deadline_closes: usize,
    /// An event at the very sample the deadline would have closed the search.
    event_at_last_position: usize,
    /// An event at the sample after a deadline's close.
    event_after_close: usize,
    /// An event at the sample an acquisition is applied.
    event_at_acquisition: usize,
    /// A search opened by an event at the very sample an acquisition is applied (and closed
    /// by it).
    opened_at_acquisition: usize,
    /// An accepted maximum at a position below 2^9 and one at or above (the hand-over's
    /// numbers are modulo 2^9, with a flag).
    maximum_before_wrap: usize,
    maximum_after_wrap: usize,
    /// An acceptance at the search's last position.
    acceptance_at_last_position: usize,
    /// An accepted maximum at the search's first position.
    maximum_at_first: usize,
    /// A larger energy just before the first position than the first accepted maximum.
    larger_before_first: usize,
    /// An accepted maximum with an equal energy after it (the first of equals is kept).
    ties: usize,
    /// An acceptance in a search that already had one (the restart after a candidate).
    restarts: usize,
    /// The largest energy, in bits.
    energy_bits: u32,
}

impl SearchCover {
    fn add(&mut self, o: &Self) {
        self.samples += o.samples;
        self.events += o.events;
        self.early_events += o.early_events;
        self.extensions += o.extensions;
        self.acceptances += o.acceptances;
        self.acquisitions += o.acquisitions;
        self.deadline_closes += o.deadline_closes;
        self.event_at_last_position += o.event_at_last_position;
        self.event_after_close += o.event_after_close;
        self.event_at_acquisition += o.event_at_acquisition;
        self.opened_at_acquisition += o.opened_at_acquisition;
        self.maximum_before_wrap += o.maximum_before_wrap;
        self.maximum_after_wrap += o.maximum_after_wrap;
        self.acceptance_at_last_position += o.acceptance_at_last_position;
        self.maximum_at_first += o.maximum_at_first;
        self.larger_before_first += o.larger_before_first;
        self.ties += o.ties;
        self.restarts += o.restarts;
        self.energy_bits = self.energy_bits.max(o.energy_bits);
    }

    /// The cases every build's segments must hold.
    fn missing(&self) -> Vec<&'static str> {
        [
            (
                self.early_events,
                "an event before the first correlator output",
            ),
            (self.extensions, "an event in an open search"),
            (self.acquisitions, "an acquisition"),
            (
                self.acceptances.saturating_sub(self.acquisitions),
                "a failed candidate",
            ),
            (self.deadline_closes, "a search closed by its deadline"),
            (
                self.event_at_last_position,
                "an event at a search's last position",
            ),
            (self.event_after_close, "an event just after a close"),
            (
                self.event_at_acquisition,
                "an event at the sample of an acquisition",
            ),
            (
                self.opened_at_acquisition,
                "a search opened at the sample of an acquisition",
            ),
            (self.maximum_before_wrap, "a maximum below position 512"),
            (
                self.maximum_after_wrap,
                "a maximum at or above position 512",
            ),
            (
                self.acceptance_at_last_position,
                "an acceptance at a search's last position",
            ),
            (
                self.maximum_at_first,
                "a maximum at a search's first position",
            ),
            (
                self.larger_before_first,
                "a larger energy just before a search's first position",
            ),
            (self.ties, "equal energies after a maximum"),
            (self.restarts, "a second acceptance in one search"),
        ]
        .iter()
        .filter_map(|&(n, what)| (n == 0).then_some(what))
        .collect()
    }
}

/// The receiver configuration of a search segment: GI − B = `w` positions in the energy sum.
fn search_config(w: usize) -> RxConfig {
    let mut cfg = RxConfig::new(20e6);
    cfg.backoff_ns = (GI_LEN - w) as f64 * 50.0;
    cfg
}

/// The lines of one search segment (see [`search_vectors`]) and what it exercises.
fn search_trace(x: &[(i16, i16)], w: usize) -> (Vec<String>, SearchCover) {
    let cfg = search_config(w);
    let mut rx = RxFrontFixed::new(cfg, FixedRxConfig::default());
    let mut c = SearchCover {
        samples: x.len(),
        ..SearchCover::default()
    };
    let mut lines = Vec::with_capacity(x.len());
    let mut energy: Vec<i128> = Vec::new();
    let mut was_open = false;
    let mut last_event: Option<usize> = None;
    let mut closed_at: Option<usize> = None;
    // Of the open search: the opening event, and its acceptances so far.
    let mut accepted_in_search = 0;
    let last_position = PAIR_LEN - 2 + w + SEARCH_OFFSET + SEARCH_LEN - 1;
    for (n, s) in x.iter().enumerate() {
        let (acq, t) = rx.push_traced(*s);
        let open = rx.searching();
        if let Some(e) = t.energy {
            energy.push(e);
            c.energy_bits = c.energy_bits.max(128 - e.leading_zeros());
        }
        if t.event {
            c.events += 1;
            c.early_events += usize::from(n + 1 < PAIR_LEN);
            c.extensions += usize::from(was_open);
            c.event_at_last_position +=
                usize::from(was_open && last_event.is_some_and(|l| n - l == last_position));
            c.event_after_close += usize::from(closed_at.is_some_and(|k| n == k + 1));
            c.event_at_acquisition += usize::from(acq.is_some());
            c.opened_at_acquisition += usize::from(acq.is_some() && !was_open);
            if !was_open {
                accepted_in_search = 0;
            }
            last_event = Some(n);
        }
        if let Some(a) = t.accepted {
            let j = energy.len() - 1;
            c.acceptances += 1;
            c.acceptance_at_last_position += usize::from(!open && acq.is_none());
            let first = a.opened_by + SEARCH_OFFSET;
            c.maximum_at_first += usize::from(a.imax == first);
            c.maximum_before_wrap += usize::from(a.imax < HISTORY);
            c.maximum_after_wrap += usize::from(a.imax >= HISTORY);
            c.larger_before_first +=
                usize::from(accepted_in_search == 0 && first > 0 && energy[first - 1] > a.emax);
            c.ties += usize::from(energy[a.imax + 1..=j].contains(&a.emax));
            c.restarts += usize::from(accepted_in_search > 0);
            accepted_in_search += 1;
        }
        c.acquisitions += usize::from(acq.is_some());
        if (was_open || t.event) && !open && acq.is_none() {
            c.deadline_closes += 1;
            closed_at = Some(n);
        }
        was_open = open;
        let e = t.energy.map_or("-".to_string(), |e| e.to_string());
        let a = t.accepted.map_or("- - -".to_string(), |a| {
            format!("{} {} {}", a.imax, a.emax, a.opened_by)
        });
        lines.push(format!(
            "{} {} {} {} {} {e} {a}",
            s.0,
            s.1,
            u8::from(acq.is_some()),
            u8::from(t.event),
            u8::from(open)
        ));
    }
    (lines, c)
}

/// A burst of a clean tone (`len` samples from `at`) over noise 40 dB below full scale: a
/// handful of detection events and then a search over noise alone.
fn tone_burst_scene(total: usize, bursts: &[(usize, usize)], seed: u64) -> Vec<(i16, i16)> {
    let fs = 20e6;
    let full = f64::from(ADC_FULL_SCALE);
    let mut rng = Rng::new(seed);
    let pn = (full * 10f64.powf(-40.0 / 20.0)).powi(2);
    let mut x = awgn(&vec![(0.0, 0.0); total], 0.0, pn, &mut rng);
    for &(at, len) in bursts {
        let tone = add_tone(&vec![(0.0, 0.0); len], 0.01 * full * full, 0.9e6, fs, 0.3);
        for (v, t) in x.iter_mut().skip(at).zip(tone) {
            *v = (v.0 + t.0, v.1 + t.1);
        }
    }
    to_adc(&x, 1.0)
}

/// Vectors for the LTF search (gate G3: `make -C hw search`): dc_block → det_a → rx_latch →
/// nco → ltf_corr → rx_search against `RxFrontFixed` sample by sample — whether a search is
/// open after the sample, the energy completed at it, and the candidate accepted at it. A
/// segment is for one GI − B (`w`, in its `reset` line); the testbench of a build runs the
/// segments of its `W`. For w = 12 (the default back-off), 16 (none) and 8 (400 ns): the
/// latch gate's scenes; a tone burst, then silence (equal energies); two bursts placed so
/// that an event falls on a search's last position, and one sample later; lone bursts chosen
/// by their seed for an acceptance at a search's last position, a maximum at its first, and
/// a larger energy just before the first; a frame with a strong tone beginning in its data,
/// placed for an event at the sample of the acquisition; a bare long training field between
/// two tones, placed so that the acquisition closes a search opened at its very sample; the long symbol's signs at full scale (the largest energy
/// the chain reaches).
fn search_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    writeln!(
        out,
        "# rvec search — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# RxFrontFixed (default widths, K = {}, decision latency {}) per sample.",
        fx.corr_bits,
        RxConfig::new(20e6).decision_latency
    )?;
    writeln!(
        out,
        "# reset <label> <w> starts a segment at GI - B = w; then <x re> <x im> <acq> <event>"
    )?;
    writeln!(
        out,
        "# <searching> <energy | -> <imax> <emax> <opened by> (- - - without an acceptance)"
    )?;
    let base = latch_scenes(&fx)?;
    for w in [12usize, 16, 8] {
        let mut scenes: Vec<Scene> = base.clone();
        // Silence after a burst: the energies are equal (zero) once the burst has left the
        // correlator's window.
        let mut quiet = tone_burst_scene(900, &[(150, 120)], 0x5EA2_0001);
        for v in quiet.iter_mut().skip(270) {
            *v = (0, 0);
        }
        scenes.push(("burst_then_silence".to_string(), quiet));
        // Two bursts: the first event of the second on the last position of the first's
        // search, and on the sample after its close.
        let last_position = PAIR_LEN - 2 + w + SEARCH_OFFSET + SEARCH_LEN - 1;
        for (label, want_last) in [
            ("event_at_last_position", true),
            ("event_after_close", false),
        ] {
            // (The last event of a burst comes some 40 samples after its end, the first some
            // 25 after its start.)
            // The noise moves the events by a sample or two: seeds as well as shifts.
            let scene = (0..16 * STF_PERIOD).find_map(|k| {
                let x = tone_burst_scene(
                    1500,
                    &[(150, 100), (150 + last_position + 96 + k % 32, 100)],
                    0x5EA2_0002 + (k / 32) as u64,
                );
                let c = search_trace(&x, w).1;
                let hit = if want_last {
                    c.event_at_last_position
                } else {
                    c.event_after_close
                };
                (hit > 0).then_some(x)
            });
            let x = scene.ok_or_else(|| io::Error::other(format!("w = {w}: no scene {label}")))?;
            scenes.push((label.to_string(), x));
        }
        type Want = fn(&SearchCover) -> usize;
        let lone: [(&str, Want); 3] = [
            ("acceptance_at_last_position", |c| {
                c.acceptance_at_last_position
            }),
            ("maximum_at_first_position", |c| c.maximum_at_first),
            ("larger_before_first_position", |c| c.larger_before_first),
        ];
        for (label, want) in lone {
            let scene = (0..4000u64).find_map(|k| {
                let x = tone_burst_scene(760, &[(150, 100)], 0x5EA2_1000 + k);
                (want(&search_trace(&x, w).1) > 0).then_some(x)
            });
            let x = scene.ok_or_else(|| io::Error::other(format!("w = {w}: no scene {label}")))?;
            scenes.push((label.to_string(), x));
        }
        // A frame, and a tone 10 dB above it that begins in its data: the decision is taken
        // from samples before the tone, and the events the tone causes fall around the sample
        // the acquisition is applied at; a start and a seed that put one exactly there.
        {
            let cfg = IfftConfig::tx_default();
            let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
            let frame: Vec<(f64, f64)> = build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
                .map_err(io::Error::other)?
                .iter()
                .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
                .collect();
            let p = mean_power(&frame);
            let full = f64::from(ADC_FULL_SCALE);
            let lead = 300;
            let scene = (0..640usize).find_map(|k| {
                let mut x = vec![(0.0, 0.0); lead];
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 300]);
                let mut rng = Rng::new(0x5EA2_2000 + (k / 64) as u64);
                let mut x = awgn(&x, 25.0, p, &mut rng);
                let start = lead + 192 + 190 + k % 64;
                let tone = add_tone(&vec![(0.0, 0.0); 200], 10.0 * p, 5.1e6, 20e6, 0.7);
                for (v, t) in x.iter_mut().skip(start).zip(tone) {
                    *v = (v.0 + t.0, v.1 + t.1);
                }
                let x = to_adc(&x, 0.1 * full / (11.0 * p).sqrt());
                (search_trace(&x, w).1.event_at_acquisition > 0).then_some(x)
            });
            let x = scene
                .ok_or_else(|| io::Error::other(format!("w = {w}: no event at an acquisition")))?;
            scenes.push(("event_at_acquisition".to_string(), x));
        }
        // An acquisition that closes a search opened at its very sample (the independent
        // review's scene): a tone burst opens a search; a frame follows whose short training
        // field is 3 dB under the noise (no event) and whose long one is 17 dB over it; the
        // search accepts it and closes by its deadline while the decision is in flight; a
        // tone then begins, placed so that its first event falls on the sample the
        // acquisition is applied at.
        {
            let cfg = IfftConfig::tx_default();
            let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
            let frame: Vec<(f64, f64)> = build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
                .map_err(io::Error::other)?
                .iter()
                .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
                .collect();
            let p = mean_power(&frame);
            let full = f64::from(ADC_FULL_SCALE);
            let sigma2 = (0.1 * full).powi(2) * 10f64.powf(-17.0 / 10.0);
            let amp = 0.1 * full / p.sqrt();
            let amp_stf = (sigma2 * 10f64.powf(-3.0 / 10.0) / p).sqrt();
            // The acceptance is at about T1 + 189 + 2·w and the tone's first event some 27
            // samples after its start: the long field's place g must put the first search's
            // deadline between the acceptance and its result, the tone's start t the event
            // on the result's sample. Seeds, since the noise decides the confirmation.
            let scene = (0..70 * 13 * 12usize).filter(|_| w != 8).find_map(|k| {
                let (g, t) = (130 + k % 70, 92 + 2 * w + (k / 70) % 13);
                let mut rng = Rng::new(0x5EA2_3000 + k as u64);
                let mut x = awgn(&vec![(0.0, 0.0); 1500], 0.0, sigma2, &mut rng);
                let mut add = |at: usize, y: &[(f64, f64)], gain: f64| {
                    for (v, q) in x.iter_mut().skip(at).zip(y) {
                        *v = (v.0 + q.0 * gain, v.1 + q.1 * gain);
                    }
                };
                let tone = |len: usize| {
                    add_tone(&vec![(0.0, 0.0); len], 0.01 * full * full, 0.9e6, 20e6, 0.3)
                };
                let ltf = 250 + g;
                add(150, &tone(100), 1.0);
                add(ltf - 160, &frame[..160], amp_stf);
                add(ltf, &frame[160..320], amp);
                let start = ltf + 160 + t;
                add(start, &tone(1500 - start), 1.0);
                let x = to_adc(&x, 1.0);
                (search_trace(&x, w).1.opened_at_acquisition > 0).then_some(x)
            });
            // With a back-off of 400 ns none of these placements acquires (the candidate is
            // taken ten positions early and w = 8 does not reach T1), so they are not tried:
            // the case is required of the other two builds.
            match scene {
                Some(x) => scenes.push(("opened_at_acquisition".to_string(), x)),
                None if w == 8 => {}
                None => {
                    return Err(io::Error::other(format!(
                        "w = {w}: no search opened at an acquisition"
                    )))
                }
            }
        }
        // The long symbol's component signs at full scale: the largest energy the chain
        // reaches (32 bits of the 44: the detector's events turn the NCO, and the match is
        // lost).
        let taps = ltf_taps(&fx);
        let hi = (ADC_FULL_SCALE - 1) as i16;
        let lo = -ADC_FULL_SCALE as i16;
        let matched: Vec<(i16, i16)> = (0..6 * LTF_SYMBOL_LEN)
            .map(|k| {
                let t = taps[k % LTF_SYMBOL_LEN];
                (if t.0 < 0 { lo } else { hi }, if t.1 < 0 { lo } else { hi })
            })
            .collect();
        scenes.push(("long_symbol_signs_full_scale".to_string(), matched));

        let mut total = SearchCover::default();
        for (label, x) in &scenes {
            let (lines, c) = search_trace(x, w);
            total.add(&c);
            writeln!(out, "reset {label} {w}")?;
            for l in lines {
                writeln!(out, "{l}")?;
            }
        }
        let mut missing = total.missing();
        if w == 8 {
            missing.retain(|m| *m != "a search opened at the sample of an acquisition");
        }
        if !missing.is_empty() {
            return Err(io::Error::other(format!(
                "search vectors at w = {w} miss: {}",
                missing.join("; ")
            )));
        }
        writeln!(out, "# w = {w}: {total:?}")?;
    }
    Ok(())
}

/// The short frame of the receiver scenes (three octets at 6 Mb/s), as float samples.
fn short_frame() -> io::Result<Vec<(f64, f64)>> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    Ok(build_ppdu(&[0x5a; 3], Rate::R6, 0x5d, &cfg)
        .map_err(io::Error::other)?
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect())
}

/// The idle clocks the RTL testbenches put before sample n beyond R − 1: one after every
/// fifth sample and two after every thirteenth (`tb_decide.cpp`, `feed`).
fn tb_idle(n: usize) -> usize {
    match n.checked_sub(1) {
        Some(k) => usize::from(k % 5 == 4) + 2 * usize::from(k % 13 == 12),
        None => 0,
    }
}

/// What the decision segments exercise.
#[derive(Debug, Default, Clone, Copy)]
struct DecideCover {
    samples: usize,
    decisions: usize,
    acquisitions: usize,
    /// Candidates the rules reject (the sums are there, the acquisition is not).
    rejected: usize,
    /// Candidates whose first symbol is less than 160 samples into the stream.
    no_coarse_window: usize,
    /// Decisions whose grid moved T1 off the maximum.
    grid_moved: usize,
    /// Cycles decisions waited for the CORDIC behind a latch, with the coarse and the fine
    /// sum.
    coarse_waits: usize,
    fine_waits: usize,
    /// Decisions whose result's cycle is given.
    timed: usize,
    /// Decisions on silence: the energy of the rotated samples is zero.
    zero_energy: usize,
    /// A decision with T1 exactly 160 samples into the stream (the first that has its
    /// coarse window), and a candidate at 159 (the last that has not).
    t1_at_160: usize,
    t1_at_159: usize,
    /// Candidates that hold the confirmation rule and fail the coherence rule.
    coherence_alone: usize,
    /// Acquisitions whose residual (the word handed on less the coarse one) is 2^14 or
    /// more, and −2^14 or less: the top bit of its 16.
    residual_high: usize,
    residual_low: usize,
    /// Whether the acquisitions' clear of the run counter moves a later event.
    clear_moves_events: usize,
    /// The largest confirmation, coarse sum component and energy, in bits.
    confirmation_bits: u32,
    coarse_bits: u32,
    energy_bits: u32,
    /// Decisions with a grid energy between 7/8 of the maximum rounded down and the
    /// model's emax − (emax >> 3): the rounding decides T1.
    keep_rounding: usize,
}

/// One decision segment (see [`decide_vectors`]).
fn decide_segment(
    out: &mut dyn Write,
    label: &str,
    x: &[(i16, i16)],
    w: usize,
    c: &mut DecideCover,
) -> io::Result<()> {
    let cfg = search_config(w);
    let fx = FixedRxConfig::default();
    let timing = DecisionTiming::default();
    let runs = run_schedule(x, &cfg, &fx, &timing, &tb_idle, &|_| false)
        .map_err(|e| io::Error::other(format!("{label} at w = {w}: {e:?}")))?;
    let tap_energy: i128 = ltf_taps(&fx)
        .iter()
        .map(|t| i128::from(t.0 * t.0 + t.1 * t.1))
        .sum();
    let unclear = detector_trace_fixed(x, &cfg, &fx);
    let mut rx = RxFrontFixed::new(cfg, fx);
    writeln!(out, "reset {label} {w}")?;
    c.samples += x.len();
    let mut moved = false;
    let mut energies: Vec<i128> = Vec::new();
    for (n, s) in x.iter().enumerate() {
        let (acq, t) = rx.push_traced(*s);
        moved |= unclear.get(n).is_some_and(|u| u.event != t.event);
        energies.extend(t.energy);
        writeln!(out, "{} {} {}", s.0, s.1, u8::from(t.event))?;
        if let Some(a) = t.accepted {
            let run = runs.iter().find(|r| r.accepted_at == n);
            let t1 = t.t1.map_or("-".to_string(), |v| v.to_string());
            let sums = t.sums.map_or("- - - - -".to_string(), |m| {
                format!(
                    "{} {} {} {} {}",
                    m.coarse.0, m.coarse.1, m.confirmation, m.coherence, m.energy
                )
            });
            let fine = t
                .sums
                .and_then(|m| m.fine)
                .map_or("- -".to_string(), |f| format!("{} {}", f.0, f.1));
            // The result's cycle after the strobe, for a decision that runs its whole
            // schedule and whose result falls inside the segment.
            let done = run
                .filter(|r| r.sums.is_some())
                .map_or("- - -".to_string(), |r| {
                    format!("{} {} {}", r.done, r.coarse_wait, r.fine_wait)
                });
            writeln!(out, "A {t1} {sums} {fine} {done}")?;
            if let Some(m) = t.sums {
                let rhs = tap_energy * m.energy;
                c.coherence_alone +=
                    usize::from(2 * m.confirmation >= rhs && 2 * m.coherence < rhs);
                c.confirmation_bits = c
                    .confirmation_bits
                    .max(128 - m.confirmation.leading_zeros());
                let widest = m.coarse.0.unsigned_abs().max(m.coarse.1.unsigned_abs());
                c.coarse_bits = c.coarse_bits.max(64 - widest.leading_zeros());
                c.energy_bits = c.energy_bits.max(128 - m.energy.leading_zeros());
            }
            c.decisions += 1;
            c.rejected += usize::from(t.sums.is_some_and(|m| m.fine.is_none()));
            c.no_coarse_window += usize::from(t.t1.is_some() && t.sums.is_none());
            c.grid_moved += usize::from(t.t1.is_some_and(|v| v != a.imax));
            let (down, model) = ((7 * a.emax) >> 3, a.emax - (a.emax >> 3));
            c.keep_rounding += usize::from(
                energies[a.imax..=a.imax + w]
                    .iter()
                    .any(|&e| e >= down && e < model),
            );
            c.zero_energy += usize::from(t.sums.is_some_and(|m| m.energy == 0));
            c.t1_at_160 += usize::from(t.t1 == Some(160) && t.sums.is_some());
            c.t1_at_159 += usize::from(t.t1 == Some(159) && t.sums.is_none());
            if let Some(r) = run {
                if r.sums != t.sums {
                    return Err(io::Error::other(format!(
                        "{label} at w = {w}: the schedule's sums are not the model's"
                    )));
                }
                c.coarse_waits += r.coarse_wait;
                c.fine_waits += r.fine_wait;
                c.timed += usize::from(r.sums.is_some());
            }
        }
        if let Some(q) = acq {
            if q.fft_start + GI_LEN != q.ltf_start + GI2_LEN + w {
                return Err(io::Error::other("the acquisition's two starts disagree"));
            }
            writeln!(
                out,
                "Q {} {} {} {}",
                q.ltf_start + GI2_LEN,
                q.cfo_phase,
                q.coarse_phase,
                q.fine_phase
            )?;
            c.acquisitions += 1;
            c.residual_high += usize::from(q.cfo_phase - q.coarse_phase >= 1 << 14);
            c.residual_low += usize::from(q.cfo_phase - q.coarse_phase <= -(1 << 14));
        }
    }
    c.clear_moves_events += usize::from(moved);
    Ok(())
}

/// Vectors for the decision's datapath (gate G3: `make -C hw decide`): the chain of the
/// search's gate with `rx_decide` behind it and the loop closed. Per sample `<x re> <x im>
/// <event>` (the detector's event: with the loop closed, the gate of the run counter's clear);
/// after the sample of an acceptance a line `A <T1> <coarse re> <coarse im> <confirmation>
/// <coherence> <energy> <fine re> <fine im> <done>` (`SampleTrace::t1` and `sums`; `-` for
/// what the model did not make; `done` the result's cycle after the strobe by
/// `rx_decision::run_schedule` on the testbench's clock pattern at R = 8, D = 4); after the
/// sample an acquisition is applied at, `Q <T1> <cfo> <coarse> <fine>` (the three phase
/// words of `FixedAcquisition`). Segments, for w = 12, 16, 8 and 4: the four frames; two
/// frames under a weak tone; a frame under a strong one; a tone alone (rejected candidates);
/// frames under a tone of their own power; a stream that begins 40 samples into a frame
/// (T1 less than 160 samples in: no decision), and streams cut to put T1 at sample 160 and
/// at 159; a tone burst and then silence (candidates of zero energy). For w = 4, where the
/// fine sum's branch of the schedule is the longer one, a subset. Under the tones the
/// stream's own latches meet a decision's requests of the CORDIC (at w = 12 with this clock
/// pattern).
fn decide_vectors(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedRxConfig::default();
    writeln!(
        out,
        "# rvec decide — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# reset <label> <w>; <x re> <x im> <event> per sample; after an acceptance's sample:"
    )?;
    writeln!(
        out,
        "# A <T1> <coarse re im> <confirmation> <coherence> <energy> <fine re im> <done> <waits>;"
    )?;
    writeln!(
        out,
        "# after an acquisition's sample: Q <T1> <cfo> <coarse> <fine> (- where the model has none)"
    )?;
    let frame = short_frame()?;
    let p = mean_power(&frame);
    let full = f64::from(ADC_FULL_SCALE);
    // Frames under a tone of their own power: the detector keeps firing while decisions
    // are made.
    let under_tone = {
        let mut rng = Rng::new(0xDEC1_0001);
        let mut x = Vec::new();
        for _ in 0..5 {
            x.extend(vec![(0.0, 0.0); 400]);
            x.extend_from_slice(&frame);
        }
        x.extend(vec![(0.0, 0.0); 400]);
        let x = awgn(&add_tone(&x, p, 1.3e6, 20e6, 0.3), 25.0, p, &mut rng);
        to_adc(&x, full * 10f64.powf(-18.0 / 20.0) / (2.0 * p).sqrt())
    };
    // A stream that begins 40 samples into a frame, and a whole frame after it.
    let cut = {
        let mut rng = Rng::new(0xDEC1_0002);
        let mut x: Vec<(f64, f64)> = frame[40..].to_vec();
        x.extend(vec![(0.0, 0.0); 300]);
        x.extend_from_slice(&frame);
        x.extend(vec![(0.0, 0.0); 300]);
        to_adc(&awgn(&x, 25.0, p, &mut rng), 0.1 * full / p.sqrt())
    };
    let latch = latch_scenes(&fx)?;
    let wanted = [
        "frame_awgn_25dB",
        "frame_awgn_4dB",
        "frame_tgn_d_9dB",
        "frame_hot_-3dBFS",
        "two_frames_under_tone",
        "frame_under_strong_tone",
        "tone_10dB_from_0",
    ];
    // A tone burst, then silence: candidates of zero energy (every position keeps 7/8 of
    // nothing; the rules hold on 0 ≥ 0 and the energy test alone rejects).
    let mut silence = tone_burst_scene(900, &[(150, 120)], 0x5EA2_0001);
    for v in silence.iter_mut().skip(270) {
        *v = (0, 0);
    }
    // Streams that begin 32 and 33 samples into a frame: T1 at sample 160, the first with a
    // coarse window in the stream, and at 159.
    let cut_at = |cut: usize, seed: u64| {
        let mut rng = Rng::new(seed);
        let mut x: Vec<(f64, f64)> = frame[cut..].to_vec();
        x.extend(vec![(0.0, 0.0); 300]);
        to_adc(&awgn(&x, 30.0, p, &mut rng), 0.1 * full / p.sqrt())
    };
    // A frame whose frequency steps between the short and the long training field (156
    // samples in, phase-continuous): the coarse estimate is the short field's, so the long
    // field's residual is the step. Up to some 94 kHz the candidate is acquired with a
    // residual in the top half of its 16 bits; beyond, the two long symbols no longer add
    // up after the rotation by the coarse angle, and the coherence rule alone fails (the
    // independent review's construction).
    let stepped = |step_hz: f64, seed: u64| {
        let mut rng = Rng::new(seed);
        let mut x = vec![(0.0, 0.0); 230];
        x.extend_from_slice(&frame);
        x.extend(vec![(0.0, 0.0); 200]);
        let x = frequency_offset(&x, (rng.uniform() - 0.5) * 20e3, 20e6, 0.0);
        let x: Vec<(f64, f64)> = x
            .iter()
            .enumerate()
            .map(|(n, &(a, b))| {
                let turns = step_hz * n.saturating_sub(386) as f64 / 20e6;
                let ph = 2.0 * std::f64::consts::PI * turns;
                (a * ph.cos() - b * ph.sin(), a * ph.sin() + b * ph.cos())
            })
            .collect();
        to_adc(&awgn(&x, 25.0, p, &mut rng), 0.3 * full / p.sqrt())
    };
    // A frame 18 dB over full scale: the ADC clips nearly every sample, the widest sums the
    // stream reaches.
    let clipping = {
        let mut rng = Rng::new(0xDEC1_0003);
        let mut x = vec![(0.0, 0.0); 250];
        x.extend_from_slice(&frame);
        x.extend(vec![(0.0, 0.0); 250]);
        to_adc(&awgn(&x, 30.0, p, &mut rng), 8.0 * full / p.sqrt())
    };
    // A tone burst and then noise of a few LSB: small integer energies, where 7/8 of the
    // maximum is rounded as the model rounds it or not at all.
    let faint = |seed: u64| {
        let mut x = tone_burst_scene(900, &[(150, 120)], 0x5EA2_0001);
        let mut rng = Rng::new(seed);
        let span = 3.0 + (seed % 5) as f64 * 2.0;
        for v in x.iter_mut().skip(270) {
            *v = (
                (rng.uniform() * span) as i16 - (span as i16) / 2,
                (rng.uniform() * span) as i16 - (span as i16) / 2,
            );
        }
        x
    };
    let mut total = DecideCover::default();
    for w in [12usize, 16, 8, 4] {
        let mut c = DecideCover::default();
        for (label, x) in latch.iter().filter(|(l, _)| wanted.contains(&l.as_str())) {
            // At w = 4 (a back-off of 600 ns, where the fine sum's branch of the schedule is
            // the longer one) the two AWGN frames, the two frames under a weak tone and the
            // tone alone (the other three `frame_` scenes are left out).
            if w != 4 || !label.starts_with("frame_") || label.contains("awgn") {
                decide_segment(out, label, x, w, &mut c)?;
            }
        }
        decide_segment(out, "burst_then_silence", &silence, w, &mut c)?;
        for (label, cut) in [("t1_at_160", 32usize), ("t1_at_159", 33)] {
            let want = |c: &DecideCover| if cut == 32 { c.t1_at_160 } else { c.t1_at_159 };
            let scene = (0..200u64).find_map(|k| {
                let x = cut_at(cut, 0xDEC1_1000 + k);
                let mut probe = DecideCover::default();
                decide_segment(&mut io::sink(), label, &x, w, &mut probe).ok()?;
                (want(&probe) > 0).then_some(x)
            });
            let x = scene.ok_or_else(|| io::Error::other(format!("w = {w}: no scene {label}")))?;
            decide_segment(out, label, &x, w, &mut c)?;
        }
        decide_segment(out, "stream_cut_into_a_frame", &cut, w, &mut c)?;
        decide_segment(out, "frame_clipping", &clipping, w, &mut c)?;
        {
            let scene = (0..300u64).find_map(|k| {
                let x = faint(0xDEC1_4000 + k);
                let mut probe = DecideCover::default();
                decide_segment(&mut io::sink(), "probe", &x, w, &mut probe).ok()?;
                (probe.keep_rounding > 0).then_some(x)
            });
            let x = scene
                .ok_or_else(|| io::Error::other(format!("w = {w}: no rounding of 7/8 decides")))?;
            decide_segment(out, "burst_then_faint_noise", &x, w, &mut c)?;
        }
        type Want = fn(&DecideCover) -> usize;
        let steps: [(&str, f64, f64, Want); 3] = [
            ("residual_high", 78e3, 94e3, |c| c.residual_high),
            ("residual_low", 78e3, 94e3, |c| c.residual_low),
            ("coherence_alone", 94e3, 138e3, |c| c.coherence_alone),
        ];
        for (label, lo, hi, want) in steps {
            let scene = (0..400u64).find_map(|k| {
                // Both signs of the step in turn, across the band.
                let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
                let f = sign * (lo + (hi - lo) * ((k / 2) % 20) as f64 / 20.0);
                let x = stepped(f, 0xDEC1_2000 + k);
                let mut probe = DecideCover::default();
                decide_segment(&mut io::sink(), label, &x, w, &mut probe).ok()?;
                (want(&probe) > 0).then_some(x)
            });
            let x = scene.ok_or_else(|| io::Error::other(format!("w = {w}: no scene {label}")))?;
            decide_segment(out, label, &x, w, &mut c)?;
        }
        // A frame, and a tone 10 dB above it that begins in its data: the detector is in a
        // run of fires when the acquisition is applied, and the clear of its counter moves
        // the events that follow (the gate of `det_a.clear` from the decision).
        {
            let scene = (0..64usize).find_map(|k| {
                let mut rng = Rng::new(0xDEC1_3000 + k as u64);
                let mut x = vec![(0.0, 0.0); 300];
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 300]);
                let mut x = awgn(&x, 25.0, p, &mut rng);
                let tone = add_tone(&vec![(0.0, 0.0); 260], 10.0 * p, 5.1e6, 20e6, 0.7);
                for (v, t) in x.iter_mut().skip(300 + 192 + 180 + k).zip(tone) {
                    *v = (v.0 + t.0, v.1 + t.1);
                }
                let x = to_adc(&x, 0.1 * full / (11.0 * p).sqrt());
                let mut probe = DecideCover::default();
                decide_segment(&mut io::sink(), "probe", &x, w, &mut probe).ok()?;
                (probe.clear_moves_events > 0 && probe.acquisitions > 0).then_some(x)
            });
            let x =
                scene.ok_or_else(|| io::Error::other(format!("w = {w}: no clear inside a run")))?;
            decide_segment(out, "clear_inside_a_run", &x, w, &mut c)?;
        }
        decide_segment(out, "frames_under_own_power_tone", &under_tone, w, &mut c)?;
        writeln!(out, "# w = {w}: {c:?}")?;
        let missing: Vec<&str> = [
            (c.timed, "a timed decision"),
            (c.acquisitions, "an acquisition"),
            (c.no_coarse_window, "a first symbol too early"),
            (c.rejected, "a rejected candidate"),
            (c.zero_energy, "a candidate of zero energy"),
            (c.t1_at_160, "a decision with T1 at sample 160"),
            (c.t1_at_159, "a candidate with T1 at sample 159"),
            (
                c.coherence_alone,
                "a candidate failing the coherence rule alone",
            ),
            (
                c.residual_high,
                "an acquisition with a residual of 2^14 or more",
            ),
            (
                c.residual_low,
                "an acquisition with a residual of -2^14 or less",
            ),
            (
                c.clear_moves_events,
                "an acquisition whose clear moves a later event",
            ),
            (c.keep_rounding, "a grid decided by the rounding of 7/8"),
            (
                // (The energy's own bound is 31 bits; a stream without a DC offset of half the
                // scale does not come nearer than this.)
                usize::from(c.coarse_bits >= 28 && c.energy_bits >= 28),
                "a coarse sum and an energy of 28 bits",
            ),
            // A wait moves the result's cycle only on the schedule's longer branch: the
            // coarse sum's from w = 5 up, the fine sum's below.
            (
                usize::from(w != 12) + c.coarse_waits,
                "a coarse wait at w = 12",
            ),
            (usize::from(w != 4) + c.fine_waits, "a fine wait at w = 4"),
        ]
        .iter()
        .filter_map(|&(n, what)| (n == 0).then_some(what))
        .collect();
        if !missing.is_empty() {
            return Err(io::Error::other(format!(
                "decide vectors at w = {w} miss: {}",
                missing.join("; ")
            )));
        }
        total.grid_moved += c.grid_moved;
    }
    if total.grid_moved == 0 {
        return Err(io::Error::other("no decision moves T1 off the maximum"));
    }
    Ok(())
}

/// What the front's segments exercise.
#[derive(Debug, Default, Clone, Copy)]
struct FrontCover {
    samples: usize,
    acquisitions: usize,
    /// Acquisitions less than 900 samples after the one before (a frame of 560 samples a
    /// SIFS, 320 samples, or less after another).
    close_pairs: usize,
    /// Acquisitions whose T1 is the last position of the grid (the maximum's and GI − B
    /// more): a decision that looks at one position fewer places it elsewhere.
    grid_last: usize,
    /// Acquisitions whose FFT window starts in the lap of the history before T1's (T1 modulo
    /// the history's depth less than the back-off's GI − w): the address borrows.
    fft_borrow: usize,
    /// The sample of the last acquisition.
    last_acquisition: usize,
}

/// One segment of the front's gate (see [`front_vectors`]).
fn front_segment(
    out: &mut dyn Write,
    label: &str,
    x: &[(i16, i16)],
    w: usize,
    c: &mut FrontCover,
) -> io::Result<()> {
    let mut rx = RxFrontFixed::new(search_config(w), FixedRxConfig::default());
    writeln!(out, "reset {label} {w}")?;
    let mut last: Option<usize> = None;
    let mut at_last = false;
    for (n, s) in x.iter().enumerate() {
        writeln!(out, "{} {}", s.0, s.1)?;
        let (acq, t) = rx.push_traced(*s);
        if let Some(a) = t.accepted {
            at_last = t.t1 == Some(a.imax + w);
        }
        if let Some(q) = acq {
            let t1 = q.ltf_start + GI2_LEN;
            if q.decided_at != n || q.fft_start + GI_LEN != t1 + w {
                return Err(io::Error::other(format!(
                    "{label} at w = {w}: an acquisition off its sample or its window"
                )));
            }
            // What `rx_front.sv` promises the hand-off: an acquisition is applied 189 + w + L
            // to 189 + 2w + L samples after its T1 (the maximum's position imax is at most w
            // before T1 and the acceptance 189 + 2w after imax), so the window's first sample
            // is at most 285 samples old in a history of 512.
            let age = n - t1;
            if age < 189 + w + DECISION_LATENCY || age > 189 + 2 * w + DECISION_LATENCY {
                return Err(io::Error::other(format!(
                    "{label} at w = {w}: an acquisition {age} samples after its T1"
                )));
            }
            c.grid_last += usize::from(at_last);
            writeln!(
                out,
                "Q {} {t1} {} {} {}",
                q.fft_start, q.cfo_phase, q.coarse_phase, q.fine_phase
            )?;
            c.acquisitions += 1;
            c.close_pairs += usize::from(last.is_some_and(|p| n - p < 900));
            c.fft_borrow += usize::from(t1 % HISTORY < GI_LEN - w);
            c.last_acquisition = c.last_acquisition.max(n);
            last = Some(n);
        }
    }
    c.samples += x.len();
    Ok(())
}

/// A stream of `frames` short frames over a noise floor 46 dB below full scale, nothing
/// reset between them: each at its own level (−40 to −4 dBFS, every seventh 6 dB over full
/// scale), CFO (±235 kHz) and phase; every sixth through a TGn D channel, the others
/// delayed by a fraction of a sample; the gaps from a frame's last sample to the next one's
/// first a SIFS (320 samples), 60 to 120 samples, or 150 to 900; a tone burst in some gaps
/// and a tone of a frame's own power over some frames.
fn front_stream(frames: usize, seed: u64) -> io::Result<Vec<(i16, i16)>> {
    const FS: f64 = 20e6;
    let frame = short_frame()?;
    let p = mean_power(&frame);
    let full = f64::from(ADC_FULL_SCALE);
    let mut rng = Rng::new(seed);
    let mut x: Vec<(f64, f64)> = vec![(0.0, 0.0); 200];
    // The sample after the last frame's last (the channel's tail of 40 samples aside).
    let mut end = 200;
    let add = |x: &mut Vec<(f64, f64)>, at: usize, piece: &[(f64, f64)]| {
        if x.len() < at + piece.len() {
            x.resize(at + piece.len(), (0.0, 0.0));
        }
        for (v, s) in x.iter_mut().skip(at).zip(piece) {
            *v = (v.0 + s.0, v.1 + s.1);
        }
    };
    for k in 0..frames {
        let gap = match k % 4 {
            0 => 320,
            1 => 60 + (rng.uniform() * 60.0) as usize,
            _ => 150 + (rng.uniform() * 750.0) as usize,
        };
        let at = end + gap;
        let dbfs = if k % 7 == 6 {
            6.0
        } else {
            -40.0 + 36.0 * rng.uniform()
        };
        let g = full * 10f64.powf(dbfs / 20.0) / p.sqrt();
        let mut f = frame.clone();
        f.extend(vec![(0.0, 0.0); 40]);
        let f = if k % 6 == 5 {
            let taps = tgn_taps(TgnModel::D, false);
            Multipath::new(taps, Doppler::Static, 1, FS, &mut rng).apply(&f)
        } else {
            fractional_delay(&f, rng.uniform())
        };
        let cfo = (rng.uniform() * 2.0 - 1.0) * 235e3;
        let f = frequency_offset(&f, cfo, FS, 2.0 * std::f64::consts::PI * rng.uniform());
        let f: Vec<(f64, f64)> = f.iter().map(|s| (s.0 * g, s.1 * g)).collect();
        add(&mut x, at, &f);
        let tone_hz = (rng.uniform() * 2.0 - 1.0) * 8e6;
        if k % 5 == 2 && gap > 140 {
            let a = full * 10f64.powf((-30.0 + 20.0 * rng.uniform()) / 20.0);
            let tone = add_tone(&vec![(0.0, 0.0); gap - 40], a * a, tone_hz, FS, 0.3);
            add(&mut x, end + 10, &tone);
        }
        if k % 9 == 4 {
            let tone = add_tone(&vec![(0.0, 0.0); 300], g * g * p, tone_hz, FS, 0.7);
            add(&mut x, at + 100, &tone);
        }
        end = at + frame.len();
    }
    x.extend(vec![(0.0, 0.0); 300]);
    let pn = (full * 10f64.powf(-46.0 / 20.0)).powi(2);
    Ok(to_adc(&awgn(&x, 0.0, pn, &mut rng), 1.0))
}

/// Vectors for the front's top level (gate G3: `make -C hw front`): `hw/rtl/rx_front.sv`
/// against `RxFrontFixed::push` at its pins, nothing but the ADC's samples in and the
/// acquisitions out. Per sample `<x re> <x im>`; after the sample an acquisition is applied
/// at, `Q <fft start> <T1> <cfo> <coarse> <fine>` (the `FixedAcquisition`; the starts as full
/// sample numbers, the hardware gives them modulo the history's depth). Segments, for
/// w = 12, 16, 8 and 4: a long stream of frames with nothing reset between them
/// ([`front_stream`]; at w = 12 past sample 2^16), and for w below 16 a frame placed by
/// its lead so that its FFT window starts in the lap of the history before T1's; frames
/// with a second path GI − B or one sample more behind the first and nearly as strong
/// (0.94 to 0.99 of it), chosen by seed for a T1 at the grid's last position (the
/// independent review's construction). What the
/// front does inside (the detector's events, the acceptances, the decision's sums and
/// cycles) is the gate `decide`'s, which runs on the same top level.
fn front_vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec front — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# reset <label> <w>; <x re> <x im> per sample; after an acquisition's sample:"
    )?;
    writeln!(out, "# Q <fft start> <T1> <cfo> <coarse> <fine>")?;
    let frame = short_frame()?;
    let p = mean_power(&frame);
    let full = f64::from(ADC_FULL_SCALE);
    for w in [12usize, 16, 8, 4] {
        let frames = if w == 12 { 76 } else { 20 };
        let mut long = FrontCover::default();
        let x = front_stream(frames, 0xF207_0000 + w as u64)?;
        front_segment(out, "long_stream", &x, w, &mut long)?;
        let mut c = long;
        if w < GI_LEN {
            let scene = (0..80usize).find_map(|k| {
                let mut rng = Rng::new(0xF207_1000 + k as u64);
                let mut x = vec![(0.0, 0.0); 290 + k];
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 400]);
                x.extend_from_slice(&frame);
                x.extend(vec![(0.0, 0.0); 200]);
                let x = to_adc(&awgn(&x, 25.0, p, &mut rng), 0.1 * full / p.sqrt());
                let mut probe = FrontCover::default();
                front_segment(&mut io::sink(), "probe", &x, w, &mut probe).ok()?;
                (probe.fft_borrow > 0 && probe.acquisitions == 2).then_some(x)
            });
            let x = scene.ok_or_else(|| {
                io::Error::other(format!("w = {w}: no window before the history's wrap"))
            })?;
            front_segment(out, "window_before_the_wrap", &x, w, &mut c)?;
        }
        {
            let scene = (0..40u64).find_map(|k| {
                let mut rng = Rng::new(0xF207_2000 + k);
                let mut x: Vec<(f64, f64)> = vec![(0.0, 0.0); 300];
                for i in 0..6usize {
                    let a = [0.94, 0.97, 0.99][i % 3];
                    let d = w + (i / 3) % 2;
                    let ph = 2.0 * std::f64::consts::PI * rng.uniform();
                    let (re, im) = (a * ph.cos(), a * ph.sin());
                    let mut f = vec![(0.0, 0.0); frame.len() + d];
                    for (j, v) in frame.iter().enumerate() {
                        f[j] = (f[j].0 + v.0, f[j].1 + v.1);
                        let e = f[j + d];
                        f[j + d] = (e.0 + re * v.0 - im * v.1, e.1 + re * v.1 + im * v.0);
                    }
                    let cfo = (rng.uniform() * 2.0 - 1.0) * 100e3;
                    x.extend(frequency_offset(&f, cfo, 20e6, 0.0));
                    x.extend(vec![(0.0, 0.0); 500 + 37 * i]);
                }
                let x = to_adc(&awgn(&x, 30.0, p, &mut rng), 0.1 * full / p.sqrt());
                let mut probe = FrontCover::default();
                front_segment(&mut io::sink(), "probe", &x, w, &mut probe).ok()?;
                (probe.grid_last > 0).then_some(x)
            });
            let x = scene.ok_or_else(|| {
                io::Error::other(format!("w = {w}: no T1 at the grid's last position"))
            })?;
            let before = c.grid_last;
            front_segment(out, "two_paths", &x, w, &mut c)?;
            if c.grid_last == before {
                return Err(io::Error::other("the two-path scene lost its T1"));
            }
        }
        writeln!(out, "# w = {w}: {c:?}")?;
        let missing: Vec<&str> = [
            (
                usize::from(2 * long.acquisitions >= frames),
                "an acquisition for half the long stream's frames",
            ),
            (
                long.close_pairs,
                "an acquisition less than 900 samples after another",
            ),
            (
                usize::from(w == GI_LEN) + c.fft_borrow,
                "a window that starts before the history's wrap",
            ),
            (
                usize::from(w != 12 || long.last_acquisition > 1 << 16),
                "an acquisition past sample 2^16",
            ),
        ]
        .iter()
        .filter_map(|&(n, what)| (n == 0).then_some(what))
        .collect();
        if !missing.is_empty() {
            return Err(io::Error::other(format!(
                "front vectors at w = {w} miss: {}",
                missing.join("; ")
            )));
        }
    }
    Ok(())
}

/// Vectors for `hw/rtl/conv_encoder.sv` (gate G3: `make -C hw conv_encoder`): frames of input
/// bits with the encoder's A and B outputs (`ConvolutionalEncoder::encode_bit`), each frame
/// from the all-zero state. Frames: all zeros, all ones, a single one (the impulse response:
/// the generators themselves), alternating bits, and random frames of 1 to 600 bits.
fn conv_encoder_vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec conv_encoder — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# K = 7, g0 = 133, g1 = 171 (octal), rate 1/2, each line from state 0."
    )?;
    writeln!(out, "# <input bits> <A bits> <B bits>")?;
    let mut rng = XorShift32(0xc0de_0001);
    let mut frames: Vec<Vec<u8>> = vec![
        vec![0; 40],
        vec![1; 40],
        std::iter::once(1)
            .chain(std::iter::repeat_n(0, 15))
            .collect(),
        (0..64).map(|i| (i % 2) as u8).collect(),
        vec![1],
    ];
    for k in 0..60usize {
        let len = 1 + (k * k * 7 + k * 13) % 600;
        frames.push((0..len).map(|_| rng.next_bit()).collect());
    }
    for bits in frames {
        let mut enc = ConvolutionalEncoder::new();
        let (a, b): (Vec<u8>, Vec<u8>) = bits.iter().map(|&x| enc.encode_bit(x)).unzip();
        writeln!(
            out,
            "{} {} {}",
            bits_to_string(&bits),
            bits_to_string(&a),
            bits_to_string(&b)
        )?;
    }
    Ok(())
}

fn main() -> ExitCode {
    let block = std::env::args().nth(1).unwrap_or_default();
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let result = match block.as_str() {
        "scrambler" => scrambler_vectors(&mut out),
        "conv_encoder" => conv_encoder_vectors(&mut out),
        "ofdm-symbols" => ofdm_symbols(&mut out),
        "ppdu-frames" => ppdu_frames(&mut out),
        "ltf_corr" => ltf_corr_vectors(&mut out),
        "round_sat" => round_sat_vectors(&mut out),
        "rx-front-rom" => rx_front_rom(&mut out),
        "cordic" => cordic_vectors(&mut out),
        "nco" => nco_vectors(&mut out),
        "det" => det_vectors(&mut out),
        "latch" => latch_vectors(&mut out),
        "search" => search_vectors(&mut out),
        "decide" => decide_vectors(&mut out),
        "front" => front_vectors(&mut out),
        _ => {
            eprintln!(
                "usage: rvec <block>\nblocks: scrambler, conv_encoder, round_sat, ltf_corr, cordic, nco, det, latch, search, decide, front, rx-front-rom, ofdm-symbols, ppdu-frames"
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
