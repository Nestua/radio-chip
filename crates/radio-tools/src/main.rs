//! `rvec` — writes the committed test vectors that the RTL gates (G3) replay.
//!
//! Vectors are generated, never hand-edited. CI regenerates every file and fails on any diff.
//!
//! Usage: `cargo run -q -p radio-tools --bin rvec -- <block>` writes the vectors to stdout.
//!
//! `ofdm-symbols` and `ppdu-frames` are not RTL vector files: they feed the OpenOFDM receiver
//! harnesses (`tools/ref/ofdm_rx_openofdm.py`, `tools/ref/ppdu_decode_openofdm.py`) and are
//! written to the ignored `target/ext/`.

mod rx_channel_vec;
mod rx_deint_vec;
mod rx_symbol_vec;
mod rx_track_vec;

use std::io::{self, Write};
use std::process::ExitCode;

use radio_phy::channel::{
    add_tone, awgn, fractional_delay, frequency_offset, gain_step, mean_power, tgn_taps, Doppler,
    Multipath, Rng, TgnModel,
};
use radio_phy::convolutional::{CodeRate, ConvolutionalEncoder, G0, G1};
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
    cordic_angle, cordic_table, detector_trace_fixed, latest_start, ltf_corr, ltf_taps, nco_mix,
    nco_table, to_adc, FixedAcquisition, FixedRxConfig, HandoffBeat, HandoffConfig, HandoffEnd,
    RxFrontFixed, RxHandoffFixed, SearchAloneFixed, SignalVerdict, ADC_FULL_SCALE, IQ, Y_BITS,
};
use radio_phy::rx_handoff::{dc_offset_of_sums, run_handoff, HandoffReader, HandoffTiming};
use radio_phy::rx_ofdm_fixed::{fft64_twiddles, fft64_window_fixed, FixedOfdmConfig};
use radio_phy::scrambler::Scrambler;
use radio_phy::viterbi::{
    decode_windowed, metric_spread, Termination, SOFT_CLIP, SOFT_MAX, TRACEBACK_DEPTH,
};

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

/// Vectors for the decision's choice of T1 on energies the chain cannot produce (gate G3:
/// `make -C hw decide_t1`): `rx_decide` behind its arbiter, the candidate and the energy ring
/// driven by the testbench, against `latest_start`, the rule `RxFrontFixed::decide` calls. One
/// line per candidate: `<w> <imax mod 512> <lapped> <ring address of e(imax)> <emax> <e(imax)>
/// … <e(imax + w)> <T1 mod 512>`. As the search hands them over: e(imax) = emax, the later
/// words at most emax, every grid position before the deadline (the search accepts at
/// imax + 63 + w, before it). The words are drawn about the threshold keep = emax − ⌊emax/8⌋
/// (equal, one off, 2^32 off, the low word's carry either way), the maxima log-uniform up to
/// the bound w · 2^39; `rvec` refuses a w without the bound, a tie at the threshold, a word
/// above 32 bits at T1, a T1 that the low 32 bits would place elsewhere, and both T1 < 160 (no
/// decision) and T1 ≥ 160 with `lapped` 0.
fn decide_t1_vectors(out: &mut impl Write) -> io::Result<()> {
    const M32: i128 = 0xffff_ffff;
    let k = FixedRxConfig::default().corr_bits;
    writeln!(
        out,
        "# rvec decide_t1 — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# latest_start (RxFrontFixed::decide's T1). <w> <imax mod 512> <lapped> <ring of e(imax)>"
    )?;
    writeln!(out, "# <emax> <e(imax)> … <e(imax + w)> <T1 mod 512>")?;
    let mut rng = Rng::new(0xDEC1_0001);
    for w in [16usize, 12, 8, 1] {
        let bound = (w as i128) << (2 * k - 1);
        let (mut at_bound, mut tie, mut wide_t1, mut other_32, mut early, mut late0) =
            (0, 0, 0, 0, 0, 0);
        let mut cases = 0;
        for c in 0..240usize {
            let r = rng.next_u64();
            // The maximum: the bound, the bound − 1, a keep with a large low word, or
            // log-uniform over 1 … bound.
            let emax = match c % 8 {
                0 => bound,
                1 => bound - 1,
                2 => ((1i128 << 42) + (1 << 34)).min(bound),
                _ => {
                    let bits = 1 + (r % u64::from(2 * k + 3)) as u32;
                    let v = ((i128::from(rng.next_u64()) << 64) | i128::from(rng.next_u64()))
                        & ((1i128 << bits) - 1);
                    v.clamp(1, bound)
                }
            };
            let keep = emax - (emax >> 3);
            let mut grid = vec![emax];
            for _ in 0..w {
                let q = rng.next_u64();
                let e = match q % 12 {
                    0 => keep,
                    1 => keep - 1,
                    2 => keep + 1,
                    3 => keep - (1 << 32),
                    4 => keep + (1 << 32),
                    // The low word's carry: just below keep with the low word above keep's,
                    // and just above with it below.
                    5 => (keep & !M32) - 1,
                    6 => (keep | M32) + 1,
                    7 => emax,
                    8 => keep + (((emax - keep) * i128::from((q >> 40) & 0xff)) >> 8),
                    _ => (keep * i128::from((q >> 40) & 0xffff)) >> 16,
                };
                grid.push(e.clamp(0, emax));
            }
            let Some(off) = latest_start(emax, &grid) else {
                return Err(io::Error::other("decide_t1: no T1 (e(imax) keeps itself)"));
            };
            // T1 with the threshold made of the low 32 bits, and with the comparison at 32.
            let low_keep = (emax & M32) - ((emax & M32) >> 3);
            let at_32 = grid.iter().rposition(|&e| (e & M32) >= low_keep);
            let cmp_32 = grid.iter().rposition(|&e| (e & M32) >= (keep & M32));
            // Lapped or not; unlapped positions below 160 give no decision.
            let lapped = c % 3 == 0;
            let imax = if lapped {
                (r >> 20) as usize % HISTORY
            } else if c % 3 == 1 {
                (r >> 20) as usize % 160
            } else {
                160 + (r >> 20) as usize % (HISTORY - 160 - w)
            };
            let t1 = imax + off;
            let ring = (r >> 40) as usize % (LTF_SYMBOL_LEN + GI_LEN + 1);
            at_bound += usize::from(emax == bound);
            tie += usize::from(grid[off] == keep);
            wide_t1 += usize::from(grid[off] >= 1 << 32);
            other_32 += usize::from(at_32 != Some(off) || cmp_32 != Some(off));
            early += usize::from(!lapped && t1 < 160);
            late0 += usize::from(!lapped && t1 >= 160);
            cases += 1;
            let words: Vec<String> = grid.iter().map(|e| e.to_string()).collect();
            writeln!(
                out,
                "{w} {imax} {} {ring} {emax} {} {}",
                u8::from(lapped),
                words.join(" "),
                t1 % HISTORY
            )?;
        }
        let missing: Vec<&str> = [
            (at_bound > 0, "the bound"),
            (tie > 0, "a tie at the threshold"),
            (wide_t1 > 0, "a word above 32 bits at T1"),
            (other_32 > 0, "a T1 the low 32 bits would place elsewhere"),
            (early > 0, "T1 < 160 unlapped"),
            (late0 > 0, "T1 >= 160 unlapped"),
        ]
        .into_iter()
        .filter(|(ok, _)| !ok)
        .map(|(_, m)| m)
        .collect();
        if !missing.is_empty() {
            return Err(io::Error::other(format!(
                "decide_t1 w = {w}: missing {}",
                missing.join(", ")
            )));
        }
        writeln!(
            out,
            "# w = {w}: {cases} candidates; {at_bound} at the bound, {tie} T1 at a tie with keep, \
             {wide_t1} T1 words above 32 bits, {other_32} placed elsewhere by the low 32 bits, \
             {early} with no decision (T1 < 160)"
        )?;
    }
    Ok(())
}

/// One sample at the search's own inputs (the bench of `rx_search` alone): the detection
/// event, the correlator's output (ignored before sample [`PAIR_LEN`] − 1), and the result of
/// a decision on a candidate accepted at this sample.
type AloneInput = (bool, IQ, bool);

/// What the vectors of the search alone exercise (the cases `rvec search_alone` refuses to do
/// without).
#[derive(Debug, Default, Clone, Copy)]
struct AloneCover {
    /// The widest energy, in bits.
    energy_bits: u32,
    /// Energies of more than 32 bits (the chain reaches 32).
    wide_energies: usize,
    /// Accepted maxima of more than 32 bits.
    wide_acceptances: usize,
    /// Acceptances where a comparison of the energies' low 32 bits would have chosen another
    /// maximum among the same positions.
    other_at_32: usize,
    /// Acceptances with an energy equal to the maximum after it (the first is kept).
    ties: usize,
    acceptances: usize,
    acquisitions: usize,
    failed: usize,
    deadline_closes: usize,
    extensions: usize,
    /// Searches opened by an event at the very sample an acquisition closes them.
    opened_at_acquisition: usize,
}

impl AloneCover {
    fn missing(&self, w: usize) -> Vec<&'static str> {
        let k = FixedRxConfig::default().corr_bits;
        // The bound: w squares of 2·2^(2K−2), at full scale for both components.
        let top = 128 - ((w as i128) << (2 * k - 1)).leading_zeros();
        [
            (self.energy_bits == top, "the energy's bound for this w"),
            (self.wide_energies > 0, "an energy above 32 bits"),
            (
                self.wide_acceptances > 0,
                "an accepted maximum above 32 bits",
            ),
            (
                self.other_at_32 > 0,
                "a maximum the low 32 bits would not choose",
            ),
            (self.ties > 0, "a tie with the maximum"),
            (self.acquisitions > 0, "an acquisition"),
            (self.failed > 0, "a failed decision (restart)"),
            (self.deadline_closes > 0, "a search closed by its deadline"),
            (self.extensions > 0, "an event in an open search"),
            (
                self.opened_at_acquisition > 0,
                "a search opened at an acquisition",
            ),
        ]
        .into_iter()
        .filter(|(ok, _)| !ok)
        .map(|(_, m)| m)
        .collect()
    }

    fn add(&mut self, o: &Self) {
        self.energy_bits = self.energy_bits.max(o.energy_bits);
        self.wide_energies += o.wide_energies;
        self.wide_acceptances += o.wide_acceptances;
        self.other_at_32 += o.other_at_32;
        self.ties += o.ties;
        self.acceptances += o.acceptances;
        self.acquisitions += o.acquisitions;
        self.failed += o.failed;
        self.deadline_closes += o.deadline_closes;
        self.extensions += o.extensions;
        self.opened_at_acquisition += o.opened_at_acquisition;
    }
}

/// The lines of one segment of the search alone, what it exercises, and the samples at which
/// an acquisition was applied.
fn search_alone_trace(
    x: &[AloneInput],
    w: usize,
    latency: usize,
) -> io::Result<(Vec<String>, AloneCover, Vec<usize>)> {
    let mut s = SearchAloneFixed::new(w, latency);
    let mut c = AloneCover::default();
    let mut lines = Vec::with_capacity(x.len());
    let mut acquired_at = Vec::new();
    let mut energy: Vec<i128> = Vec::new();
    let mut was_open = false;
    // The first position the running maximum may take since the last acceptance.
    let mut restart = 0usize;
    let low = |e: i128| e & 0xffff_ffff;
    for (n, &(event, corr, acquire)) in x.iter().enumerate() {
        let t = s.push(event, corr, acquire);
        if let Some(e) = t.energy {
            energy.push(e);
            c.energy_bits = c.energy_bits.max(128 - e.leading_zeros());
            c.wide_energies += usize::from(e >= 1 << 32);
        }
        if event {
            c.extensions += usize::from(was_open);
            c.opened_at_acquisition += usize::from(!was_open && t.acquired);
            if !was_open {
                restart = 0;
            }
        }
        if let Some(a) = t.accepted {
            let j = energy.len() - 1;
            let start = restart.max(a.opened_by + SEARCH_OFFSET);
            let window = energy.get(start..=j).unwrap_or_default();
            // The first maximum, at full width and at 32 bits.
            let first_max = |key: &dyn Fn(i128) -> i128| {
                window
                    .iter()
                    .enumerate()
                    .fold(None, |b: Option<(usize, i128)>, (p, &e)| match b {
                        Some((_, m)) if key(e) <= m => b,
                        _ => Some((p, key(e))),
                    })
                    .map(|(p, _)| start + p)
            };
            if first_max(&|e| e) != Some(a.imax) {
                return Err(io::Error::other(format!(
                    "search_alone: the maximum at {} is not the window's ({start}..={j})",
                    a.imax
                )));
            }
            c.other_at_32 += usize::from(first_max(&low) != Some(a.imax));
            c.ties += usize::from(energy[a.imax + 1..=j].contains(&a.emax));
            c.acceptances += 1;
            c.wide_acceptances += usize::from(a.emax >= 1 << 32);
            c.failed += usize::from(!acquire);
            restart = j + 1;
        }
        if t.acquired {
            c.acquisitions += 1;
            acquired_at.push(n);
        }
        if (was_open || event) && !t.searching && !t.acquired {
            c.deadline_closes += 1;
        }
        was_open = t.searching;
        let cq = if n + 1 >= PAIR_LEN {
            format!("{} {}", corr.0, corr.1)
        } else {
            "- -".to_string()
        };
        let e = t.energy.map_or("-".to_string(), |e| e.to_string());
        let a = t.accepted.map_or("- - -".to_string(), |a| {
            format!("{} {} {}", a.imax, a.emax, a.opened_by)
        });
        lines.push(format!(
            "{} {cq} {} {} {} {e} {a}",
            u8::from(event),
            u8::from(acquire),
            u8::from(t.acquired),
            u8::from(t.searching)
        ));
    }
    Ok((lines, c, acquired_at))
}

/// A correlator output with heavy tails: each component's magnitude below 2^b, b uniform in
/// 0 … K − 1, a random sign; one in sixteen at a rail (−2^(K−1) or 2^(K−1) − 1).
fn heavy_corr(rng: &mut Rng, k: u32) -> IQ {
    let mut comp = || {
        let r = rng.next_u64();
        let lo = -(1i64 << (k - 1));
        match r & 0xf {
            0 => lo,
            1 => -lo - 1,
            _ => {
                let b = ((r >> 4) % u64::from(k)) as u32;
                let m = ((r >> 16) & ((1u64 << b) - 1)) as i64;
                if (r >> 63) == 1 {
                    -m
                } else {
                    m
                }
            }
        }
    };
    (comp(), comp())
}

/// Vectors for the LTF search alone (gate G3: `make -C hw search_alone`): `rx_search` driven at
/// its own inputs against `SearchAloneFixed`, the search of `RxFrontFixed`, sample by sample —
/// the energy, whether a search is open, the candidate accepted, at values the chain cannot
/// produce (it reaches 32 of the energy's 44 bits; the founder's decision of 2026-11-23,
/// `docs/gates/rx_front.md`). A segment is for one (GI − B, decision latency), in its `reset`
/// line; the testbench of a build runs the segments of its own. Scenes: every correlator
/// output at the negative rail (the bound; equal energies); the rails' combinations in turn;
/// heavy-tailed outputs under random events with random decisions, chosen by their seed for
/// maxima that the energies' low 32 bits would not choose; a peak late in a search window, its
/// acquisition due after the deadline, and an event on the acquisition's sample.
fn search_alone_vectors(out: &mut impl Write) -> io::Result<()> {
    let k = FixedRxConfig::default().corr_bits;
    let lo = -(1i64 << (k - 1));
    let hi = -lo - 1;
    writeln!(
        out,
        "# rvec search_alone — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# SearchAloneFixed (the search of RxFrontFixed at its correlator's output, K = {k})."
    )?;
    writeln!(
        out,
        "# reset <label> <w> <decision latency> starts a segment; then <event> <c re> <c im>"
    )?;
    writeln!(
        out,
        "# (- - before the first correlator output) <acquire> <acquired> <searching> <energy | ->"
    )?;
    writeln!(
        out,
        "# <imax> <emax> <opened by> (- - - without an acceptance). <acquire>: the decision's"
    )?;
    writeln!(
        out,
        "# result for a candidate accepted at this sample; <acquired>: an acquisition applied here."
    )?;
    // (w, decision latency): the builds of `make -C hw search_alone`.
    for (w, latency) in [(16usize, 64usize), (12, 64), (8, 20), (4, 67), (1, 64)] {
        let mut scenes: Vec<(String, Vec<AloneInput>)> = Vec::new();
        // The negative rail everywhere: the bound, every energy equal. A search acquired, then
        // one whose decision fails and that closes by its deadline.
        scenes.push((
            "full_scale".to_string(),
            (0..1100)
                .map(|n| (n == 3 || n == 600, (lo, lo), n < 600))
                .collect(),
        ));
        // The rails' combinations in turn: energies near the bound that differ in their low
        // bits only.
        let rails = [
            (lo, lo),
            (hi, hi),
            (lo, hi),
            (hi, lo),
            (lo, 0),
            (0, hi),
            (hi, 1),
            (-1, lo),
        ];
        scenes.push((
            "rails".to_string(),
            (0..900)
                .map(|n| {
                    (
                        n % 300 == 7,
                        rails[(n * 5 + n / 7) % rails.len()],
                        n % 2 == 0,
                    )
                })
                .collect(),
        ));
        // Heavy tails under random events and decisions: seeds until both segments together
        // have maxima the low 32 bits would not choose.
        let mut found = 0;
        for seed in 0..200u64 {
            let mut rng = Rng::new(0x5EA1_0000 + seed * 31 + w as u64);
            let x: Vec<AloneInput> = (0..1400)
                .map(|n| {
                    let r = rng.next_u64();
                    let event = n < 1100 && r.is_multiple_of(97) || n == 20;
                    (event, heavy_corr(&mut rng, k), !(r >> 20).is_multiple_of(3))
                })
                .collect();
            let (_, c, _) = search_alone_trace(&x, w, latency)?;
            if c.other_at_32 > 0 && c.acceptances >= 3 {
                scenes.push((format!("heavy_tails_{seed}"), x));
                found += 1;
                if found == 2 {
                    break;
                }
            }
        }
        // A peak late in the window: the search closes by its deadline while the decision is
        // in flight, and an event on the acquisition's sample opens a search it closes.
        let peak = (0..64usize).find_map(|back| {
            let mut rng = Rng::new(0x5EA1_9000 + back as u64);
            let ev = 10usize;
            let p = ev + SEARCH_OFFSET + SEARCH_LEN - LTF_SYMBOL_LEN - w - back;
            let mut x: Vec<AloneInput> = (0..1000)
                .map(|n| {
                    let c = heavy_corr(&mut rng, 11);
                    (n == ev, c, true)
                })
                .collect();
            for v in x.iter_mut().skip(p + PAIR_LEN - 1).take(w) {
                v.1 = (hi, lo);
            }
            let (_, _, acq) = search_alone_trace(&x, w, latency).ok()?;
            let q = *acq.first()?;
            x[q].0 = true;
            let (_, c, _) = search_alone_trace(&x, w, latency).ok()?;
            (c.opened_at_acquisition > 0).then_some(x)
        });
        let x = peak.ok_or_else(|| {
            io::Error::other(format!("w = {w}: no search opened at an acquisition"))
        })?;
        scenes.push(("opened_at_acquisition".to_string(), x));

        let mut total = AloneCover::default();
        for (label, x) in &scenes {
            let (lines, c, _) = search_alone_trace(x, w, latency)?;
            total.add(&c);
            writeln!(out, "reset {label} {w} {latency}")?;
            for l in lines {
                writeln!(out, "{l}")?;
            }
        }
        let missing = total.missing(w);
        if !missing.is_empty() {
            return Err(io::Error::other(format!(
                "search_alone w = {w}: missing {}",
                missing.join(", ")
            )));
        }
        writeln!(out, "# w = {w}, L = {latency}: {total:?}")?;
    }
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

/// Which of the 32 first-level pairs (2j, 2j + 1) of the hardware's best-state tree decide an
/// output through a tie in a rate-1/2 soft block decoded with window `depth`: bit j is set
/// when, at some step whose decision is output (depth − 1 ≤ t < steps − 1), state 2j is the
/// lowest best state, 2j + 1 has the same metric, and their decided bits differ. Only picks
/// vectors; the expected bits are `decode_windowed`'s. Metrics, ties and survivors as in
/// `radio_phy::viterbi` (start in state 0, a tie keeps the dropped bit 0).
fn tree_pair_ties(soft: &[i32], depth: usize) -> u32 {
    const S: usize = 64;
    let steps = soft.len() / 2;
    let parity = |w: usize, g: u8| i64::from(((w as u8) & g).count_ones() & 1);
    let mut metric = [i64::MIN / 4; S];
    metric[0] = 0;
    let mut survivor: Vec<[u8; S]> = Vec::with_capacity(steps);
    let mut pairs = 0u32;
    for t in 0..steps {
        let (sa, sb) = (i64::from(soft[2 * t]), i64::from(soft[2 * t + 1]));
        let mut next = [0i64; S];
        let mut surv = [0u8; S];
        for (s, slot) in next.iter_mut().enumerate() {
            let u = s >> 5;
            let cand = |p: usize| {
                let w = (u << 6) | p;
                metric[p] + sa * (1 - 2 * parity(w, G0)) + sb * (1 - 2 * parity(w, G1))
            };
            let p0 = (s << 1) & (S - 1);
            let (m0, m1) = (cand(p0), cand(p0 | 1));
            if m1 > m0 {
                *slot = m1;
                surv[s] = 1;
            } else {
                *slot = m0;
            }
        }
        metric = next;
        survivor.push(surv);
        if t + 1 < depth || t + 1 >= steps {
            continue;
        }
        let best = metric.iter().copied().max().unwrap_or(0);
        let Some(low) = metric.iter().position(|&m| m == best) else {
            continue;
        };
        if low % 2 == 1 || metric[low + 1] != best {
            continue;
        }
        let decided = |mut state: usize| {
            for back in survivor[t + 2 - depth..=t].iter().rev() {
                state = ((state << 1) & (S - 1)) | usize::from(back[state]);
            }
            state >> 5
        };
        if decided(low) != decided(low + 1) {
            pairs |= 1 << (low / 2);
        }
    }
    pairs
}

/// Vectors for `hw/rtl/viterbi.sv` (gate G3: `make -C hw viterbi`): blocks of depunctured
/// soft pairs (A, B) with the bits `viterbi::decode_windowed` gives for them, window
/// [`TRACEBACK_DEPTH`], the encoder in state 0 after the block's last step
/// (`Termination::ZeroAfter(steps)`, as the data path feeds the decoder up to the tail's end).
/// A line is `<soft> <bits>`: the soft values as two hex digits each (two's complement, A then
/// B per step) and one bit per step.
///
/// Blocks: every length near the window and the forced start (1 to 8, 23 to 25, 90 to 98,
/// 189 to 193), codewords of random bits in noise at rates 1/2, 2/3 and 3/4 (the stolen
/// values erased), some with the tail and some ending in another state, SIGNAL's 24 steps,
/// all erasures (every comparison a tie), every value at a rail, uniform random values (no
/// codeword: wrong decisions and a best state that jumps), noise so strong that most
/// decisions are wrong, one block of 12 022 steps (1500 octets), 52 short blocks of values
/// from three or five small levels (ties in the best-state tree) or uniform, blocks found by
/// search until each of the tree's 32 first-level pairs has a tie that decides an output
/// ([`tree_pair_ties`]), and four found by search whose last step's own decision differs from
/// the flush's first bit. The tool refuses to write a file whose largest path-metric spread is
/// not above 2047 (so that 12 bits of metric would be wrong somewhere), without any of those
/// searched blocks, or without a block shorter than the window, one of exactly the window and
/// one longer than twice the window.
fn viterbi_vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec viterbi — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# radio_phy::viterbi::decode_windowed, window {TRACEBACK_DEPTH}, ZeroAfter(steps); soft ±{SOFT_MAX}, 0 = erasure."
    )?;
    writeln!(
        out,
        "# <soft: 4 hex digits a step, A then B, two's complement> <decoded bits, one a step>"
    )?;
    let mut rng = Rng::new(0x7E7B_0001);
    let quantise = |y: f64| -> i32 {
        let v = (y * f64::from(SOFT_MAX) / SOFT_CLIP).round();
        v.clamp(-f64::from(SOFT_MAX), f64::from(SOFT_MAX)) as i32
    };
    // A codeword of `bits` through white noise of deviation `sigma` (signal amplitude 1),
    // quantised as the demapper's contract says, with the rate's stolen values erased.
    let noisy = |bits: &[u8], sigma: f64, rate: CodeRate, rng: &mut Rng| -> Vec<i32> {
        let mut enc = ConvolutionalEncoder::new();
        let pattern = rate.pattern();
        let mut soft = Vec::with_capacity(2 * bits.len());
        for (k, &b) in bits.iter().enumerate() {
            let (a, c) = enc.encode_bit(b);
            for (j, x) in [a, c].into_iter().enumerate() {
                let kept = pattern[(2 * k + j) % pattern.len()] == 1;
                let y = 1.0 - 2.0 * f64::from(x) + sigma * rng.gaussian();
                soft.push(if kept { quantise(y) } else { 0 });
            }
        }
        soft
    };
    let message = |n: usize, tail: bool, rng: &mut Rng| -> Vec<u8> {
        (0..n)
            .map(|i| {
                if tail && i + 6 >= n {
                    0
                } else {
                    (rng.next_u64() & 1) as u8
                }
            })
            .collect()
    };
    let max = SOFT_MAX;
    let mut blocks: Vec<Vec<i32>> = Vec::new();
    for n in (1..=8).chain(23..=25).chain(90..=98).chain(189..=193) {
        let bits = message(n, n % 2 == 0, &mut rng);
        blocks.push(noisy(&bits, 0.8, CodeRate::R1_2, &mut rng));
    }
    for (rate, sigma) in [
        (CodeRate::R1_2, 0.6),
        (CodeRate::R1_2, 1.0),
        (CodeRate::R2_3, 0.5),
        (CodeRate::R2_3, 0.8),
        (CodeRate::R3_4, 0.4),
        (CodeRate::R3_4, 0.7),
    ] {
        for n in [150, 400, 1000] {
            let bits = message(n, true, &mut rng);
            blocks.push(noisy(&bits, sigma, rate, &mut rng));
        }
    }
    for sigma in [0.0, 0.5, 1.0, 2.0] {
        let bits = message(24, true, &mut rng);
        blocks.push(noisy(&bits, sigma, CodeRate::R1_2, &mut rng));
    }
    blocks.push(vec![0; 2 * 60]);
    blocks.push(vec![0; 2 * 200]);
    blocks.push(vec![max; 2 * 200]);
    blocks.push(vec![-max; 2 * 200]);
    let bits = message(300, true, &mut rng);
    blocks.push(noisy(&bits, 0.0, CodeRate::R1_2, &mut rng));
    for n in [300, 2000] {
        let uniform = (0..2 * n)
            .map(|_| (rng.next_u64() % (2 * max as u64 + 1)) as i32 - max)
            .collect();
        blocks.push(uniform);
        let rails = (0..2 * n)
            .map(|_| if rng.next_u64() & 1 == 1 { max } else { -max })
            .collect();
        blocks.push(rails);
    }
    for rate in [CodeRate::R1_2, CodeRate::R3_4] {
        let bits = message(1000, true, &mut rng);
        blocks.push(noisy(&bits, 1.6, rate, &mut rng));
    }
    let bits = message(16 + 8 * 1500 + 6, true, &mut rng);
    blocks.push(noisy(&bits, 0.55, CodeRate::R3_4, &mut rng));
    // Ties in the best-state tree (the independent review found three tie mutants of the
    // tree that the blocks above miss): short blocks of values from three levels, where equal
    // metrics are common, and of uniform values, at lengths around and above the window.
    let pick =
        |levels: &[i32], rng: &mut Rng| levels[(rng.next_u64() % levels.len() as u64) as usize];
    for (k, n) in (97..=200).step_by(4).enumerate() {
        let levels: &[i32] = match k % 3 {
            0 => &[-1, 0, 1],
            1 => &[-max, 0, max],
            _ => &[-2, -1, 0, 1, 2],
        };
        blocks.push((0..2 * n).map(|_| pick(levels, &mut rng)).collect());
        let uniform = (0..2 * n)
            .map(|_| (rng.next_u64() % (2 * max as u64 + 1)) as i32 - max)
            .collect();
        blocks.push(uniform);
    }
    // For each of the 32 pairs (2j, 2j + 1) that the tree's first level compares, a block with
    // a decision where 2j is the lowest best state and 2j + 1 is as good with another decided
    // bit: a tie taken the wrong way in that one pair changes an output (the review's mutants
    // of one pair or one group of four lived through the blocks above).
    let mut covered = 0u32;
    for _ in 0..200_000 {
        if covered == u32::MAX {
            break;
        }
        let n = 97 + (rng.next_u64() % 104) as usize;
        let levels: &[i32] = match rng.next_u64() % 3 {
            0 => &[-1, 0, 1],
            1 => &[-max, 0, max],
            _ => &[-2, -1, 0, 1, 2],
        };
        let soft: Vec<i32> = (0..2 * n).map(|_| pick(levels, &mut rng)).collect();
        let pairs = tree_pair_ties(&soft, TRACEBACK_DEPTH);
        if pairs & !covered != 0 {
            covered |= pairs;
            blocks.push(soft);
        }
    }
    if covered != u32::MAX {
        return Err(io::Error::other(format!(
            "viterbi vectors: tree pairs without a deciding tie: {:#010x}",
            !covered
        )));
    }
    // Blocks whose last step's own decision (the best state's oldest bit) differs from the
    // flush's first bit (state 0's): the flush must take that output slot. Searched among
    // short blocks of strong noise that do not end in the tail.
    let mut found = 0;
    for _ in 0..20_000 {
        if found == 4 {
            break;
        }
        let n = 100 + (rng.next_u64() % 60) as usize;
        let bits = message(n, false, &mut rng);
        let soft = noisy(&bits, 1.4, CodeRate::R1_2, &mut rng);
        let depth = TRACEBACK_DEPTH;
        let free = decode_windowed(&soft, CodeRate::R1_2, Termination::Free, depth)
            .map_err(io::Error::other)?;
        let zero = decode_windowed(&soft, CodeRate::R1_2, Termination::ZeroAfter(n), depth)
            .map_err(io::Error::other)?;
        if free[n - depth] != zero[n - depth] {
            blocks.push(soft);
            found += 1;
        }
    }
    if found < 4 {
        return Err(io::Error::other(format!(
            "viterbi vectors: only {found} blocks whose last decision differs from the flush"
        )));
    }

    let (mut widest, mut short, mut exact, mut long) = (0i64, false, false, false);
    for soft in &blocks {
        let steps = soft.len() / 2;
        let bits = decode_windowed(
            soft,
            CodeRate::R1_2,
            Termination::ZeroAfter(steps),
            TRACEBACK_DEPTH,
        )
        .map_err(io::Error::other)?;
        widest = widest.max(metric_spread(soft, CodeRate::R1_2).map_err(io::Error::other)?);
        short |= steps < TRACEBACK_DEPTH;
        exact |= steps == TRACEBACK_DEPTH;
        long |= steps > 2 * TRACEBACK_DEPTH;
        let hex: String = soft.iter().map(|&v| format!("{:02x}", v as u8)).collect();
        writeln!(out, "{hex} {}", bits_to_string(&bits))?;
    }
    if widest <= 2047 || !(short && exact && long) {
        return Err(io::Error::other(format!(
            "viterbi vectors miss: spread {widest} (needs > 2047), short {short}, window {exact}, long {long}"
        )));
    }
    writeln!(out, "# largest path-metric spread: {widest}")?;
    Ok(())
}

/// Vectors for the offset's arithmetic (gate G3: `make -C hw dc_estimate`):
/// `hw/rtl/dc_estimate.sv` against `rx_handoff::dc_offset_of_sums`. A line is
/// `<cfo word> <Σ₁ re> <Σ₁ im> <Σ₂ re> <Σ₂ im> <offset re> <offset im>`. The sums: of
/// fields of noise about an offset, of full-scale noise and of two symbols at opposite
/// rails; equal sums 64·c (a constant: its own offset) and 64·c + 32 (a tie, to the even
/// number); −2^17 in a component (a quotient of exactly −2048: the dividers' early test at
/// its edge); and any 18-bit numbers. The CFO words: none, the edge of the skipped division
/// and its neighbours, half a turn in 64 samples (the largest divisor), a whole turn, and
/// random ones. The tool refuses to write a file without both rails, both kinds of tie,
/// skipped divisions and quotients at the edges of the early test.
fn dc_estimate_vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec dc_estimate — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# <cfo> <s1 re> <s1 im> <s2 re> <s2 im> <dc re> <dc im>"
    )?;
    let fx = FixedRxConfig::default();
    let mut rng = Rng::new(0xDC0F_F5E7);
    let mut draw = |span: i64| (rng.next_u64() % (2 * span as u64 + 1)) as i64 - span;
    let mut words: Vec<i64> = vec![0, 1, -1, 703, 704, 705, -703, -704, -705, 5000, -123_456];
    words.extend([1 << 15, -(1 << 15), (1 << 15) + 77, 1 << 16, (1 << 21) - 1]);
    words.extend([-(1 << 21), 9000, -9000, 77_777, 200_000, -310_000]);
    for _ in 0..40 {
        words.push(draw(1 << 21).clamp(-(1 << 21), (1 << 21) - 1));
    }
    let (mut high, mut low, mut skipped, mut even_ties, mut odd_ties) = (0, 0, 0, 0, 0);
    let (mut edge, mut lines) = (0, 0);
    let line = |out: &mut dyn Write, word: i64, s1: IQ, s2: IQ| -> io::Result<IQ> {
        let dc = dc_offset_of_sums(s1, s2, word, &fx)
            .ok_or_else(|| io::Error::other("the default front is not valid"))?;
        writeln!(
            out,
            "{word} {} {} {} {} {} {}",
            s1.0, s1.1, s2.0, s2.1, dc.0, dc.1
        )?;
        Ok(dc)
    };
    let field_sum = |mid: i64, amp: i64, draw: &mut dyn FnMut(i64) -> i64| {
        (0..64).fold(0i64, |s, _| s + (mid + draw(amp)).clamp(-2048, 2047))
    };
    for &word in &words {
        for k in 0..24 {
            let (s1, s2) = match k % 6 {
                0 => {
                    let f = |d: &mut dyn FnMut(i64) -> i64| {
                        (field_sum(300, 600, d), field_sum(-150, 600, d))
                    };
                    (f(&mut draw), f(&mut draw))
                }
                1 => {
                    let f = |d: &mut dyn FnMut(i64) -> i64| {
                        (field_sum(0, 2048, d), field_sum(0, 2048, d))
                    };
                    (f(&mut draw), f(&mut draw))
                }
                // Two symbols at opposite rails: no offset explains them.
                2 => (
                    (64 * -2040 + draw(300), 64 * 2040 - draw(300)),
                    (64 * 2040 - draw(300), 64 * -2040 + draw(300)),
                ),
                // A constant c, and c + 1/2: a tie.
                3 => {
                    let (a, b) = (draw(2047), draw(2047));
                    ((64 * a, 64 * b), (64 * a, 64 * b))
                }
                4 => {
                    let (a, b) = (draw(2040), draw(2040));
                    ((64 * a + 32, 64 * b - 32), (64 * a + 32, 64 * b - 32))
                }
                _ => (
                    (draw(131_071), draw(131_071)),
                    (draw(131_071), draw(131_071)),
                ),
            };
            let dc = line(out, word, s1, s2)?;
            lines += 1;
            let nonzero = dc != (0, 0);
            high += usize::from(dc.0 == 2047 || dc.1 == 2047);
            low += usize::from(dc.0 == -2048 || dc.1 == -2048);
            skipped += usize::from(!nonzero && s1 != (0, 0) && k % 6 != 4);
            if k % 6 == 4 && nonzero {
                // The tie's quotient is a + 1/2 and b − 1/2: to the even neighbour.
                let a = (s1.0 - 32) / 64;
                even_ties += usize::from(a % 2 == 0 && dc.0 == a);
                odd_ties += usize::from(a % 2 != 0 && dc.0 == a + 1);
            }
        }
        // The early test's edge: a quotient of exactly −2048, of −2048 + 1/64 and of 2047
        // and a half (which rounds to 2048: a rail without the early test).
        for (s, t) in [
            (-131_072i64, 131_071i64),
            (-131_071, 131_040),
            (131_040, -131_072),
            (-131_072, -131_072),
        ] {
            let dc = line(out, word, (s, t), (s, t))?;
            lines += 1;
            edge += usize::from(dc.0 == -2048 || dc.0 == 2047);
        }
    }
    if high < 50 || low < 50 || skipped < 50 || even_ties < 50 || odd_ties < 50 || edge < 100 {
        return Err(io::Error::other(format!(
            "the offset's vectors leave a case out: {high} {low} {skipped} {even_ties} {odd_ties} {edge}"
        )));
    }
    eprintln!(
        "rvec dc_estimate: {lines} lines, {high} at the upper rail, {low} at the lower, {skipped} skipped, \
         ties to {even_ties} even and {odd_ties} odd, {edge} at the early test's edge"
    );
    Ok(())
}

/// Whether the hand-off's reader is ready in cycle `clock`, by the pattern `id`; the same
/// rule, in 32-bit arithmetic, is in `hw/tb/tb_handoff.cpp`.
fn handoff_ready(id: u32, clock: usize) -> bool {
    let c = clock as u32;
    let hash = (c.wrapping_mul(0x9e37_79b9).wrapping_add(c >> 3) >> 7) & 0xff;
    match id {
        0 => true,
        1 => c.is_multiple_of(2),
        2 => hash < 128,
        3 => hash < 24,
        // Away 700 cycles in 2100.
        4 => (c / 700) % 3 != 1,
        // The first cycle after a tick only (samples 8 cycles apart, no idle clocks apart).
        _ => c % 8 == 7,
    }
}

/// What the hand-off's segments exercise.
#[derive(Debug, Default, Clone, Copy)]
struct HandoffCover {
    samples: usize,
    acquisitions: usize,
    beats: usize,
    /// Marks by kind: packet, stopped, acquisition, overrun.
    marks: [usize; 4],
    /// Verdict strobes: a length, a failure, a patient length, one with another tag.
    lengths: usize,
    failures: usize,
    patient: usize,
    foreign: usize,
    /// Samples at which the hold went down before the frame's last one, the stream left
    /// open (the release on a clipping ADC).
    releases: usize,
    /// Offsets that are not zero, and the largest magnitude of a component.
    offsets: usize,
    offset_max: i64,
    /// The fewest words of the history to spare at a read, and the fullest buffer.
    spare: usize,
    buffered: usize,
    /// Streams whose reader caught up with the antenna, and periods with more than 8 beats.
    long_periods: usize,
}

/// The reader of the hand-off's vectors: ready by a pattern of the cycle, and for the k-th
/// stream it reads a verdict by k, some cycles after the beat that completes SIGNAL's
/// window.
struct VectorReader {
    pattern: u32,
    /// The data symbols of the scene's frames.
    symbols: usize,
    due: Vec<(usize, u8, Option<SignalVerdict>)>,
    streams: usize,
    taken: usize,
    tag: u8,
}

impl HandoffReader for VectorReader {
    fn ready(&mut self, clock: usize) -> bool {
        handoff_ready(self.pattern, clock)
    }

    fn beat(&mut self, clock: usize, beat: &HandoffBeat) {
        let HandoffBeat::Sample { first, .. } = beat else {
            return;
        };
        if let Some(s) = first {
            self.tag = s.packet;
            self.taken = 0;
            self.streams += 1;
        }
        self.taken += 1;
        if self.taken != PAIR_LEN + GI_LEN + LTF_SYMBOL_LEN {
            return;
        }
        let told = |data_symbols, patient| {
            Some(SignalVerdict {
                data_symbols,
                patient,
            })
        };
        // By the stream's number: its length; nothing; a failure; a patient length; a
        // length with another packet's tag; a length above the longest; the length in the
        // very next cycle; a patient length far too long.
        let (after, tag, verdict) = match self.streams % 8 {
            0 => (300, self.tag, told(self.symbols, false)),
            1 => return,
            2 => (250, self.tag, None),
            3 => (400, self.tag, told(self.symbols, true)),
            4 => (300, self.tag.wrapping_add(3), told(self.symbols, false)),
            5 => (300, self.tag, told(5000, false)),
            6 => (1, self.tag, told(self.symbols, false)),
            _ => (37, self.tag, told(900, true)),
        };
        self.due.push((clock + after, tag, verdict));
    }

    fn verdict(&mut self, clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        let i = self.due.iter().position(|d| d.0 == clock)?;
        let d = self.due.remove(i);
        Some((d.1, d.2))
    }
}

/// The digest of a segment's beats, for segments too long to list them: over each beat's
/// cycle and fields in order (64-bit wrapping arithmetic, the same in the testbench).
fn handoff_digest(h: u64, cycle: usize, fields: [i64; 5]) -> u64 {
    let mut h = h;
    for v in std::iter::once(cycle as i64).chain(fields) {
        h = (h ^ v as u64).wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// The fields of a beat as the vectors list them: kind (0 a sample, 1 a stream's first
/// sample, 2 a mark), then I and Q and for a first sample the CFO word and the tag, or the
/// mark's kind.
fn handoff_fields(beat: &HandoffBeat) -> [i64; 5] {
    match *beat {
        HandoffBeat::Sample { z, first: None } => [0, z.0, z.1, 0, 0],
        HandoffBeat::Sample { z, first: Some(s) } => {
            [1, z.0, z.1, s.cfo_phase, i64::from(s.packet)]
        }
        HandoffBeat::End(e) => {
            let kind = match e {
                HandoffEnd::Packet => 0,
                HandoffEnd::Stopped => 1,
                HandoffEnd::Acquisition => 2,
                HandoffEnd::Overrun => 3,
            };
            [2, kind, 0, 0, 0]
        }
    }
}

/// A reader by script: ready by a pattern of the cycle but for the intervals it is away,
/// and verdict strobes at given cycles.
struct ScriptReader {
    pattern: u32,
    away: Vec<(usize, usize)>,
    strobes: Vec<(usize, u8, Option<SignalVerdict>)>,
}

impl HandoffReader for ScriptReader {
    fn ready(&mut self, clock: usize) -> bool {
        handoff_ready(self.pattern, clock)
            && !self.away.iter().any(|a| a.0 <= clock && clock <= a.1)
    }

    fn verdict(&mut self, clock: usize) -> Option<(u8, Option<SignalVerdict>)> {
        let i = self.strobes.iter().position(|d| d.0 == clock)?;
        let d = self.strobes.remove(i);
        Some((d.1, d.2))
    }
}

/// What a segment gave, for the scenes that are made for one outcome.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct HandoffOutcome {
    acquired: Vec<FixedAcquisition>,
    /// The hold as (first, one past the last) sample of each stretch it was up.
    holds: Vec<(usize, usize)>,
    /// The marks: the sample in whose period each was taken, and its kind (0 packet,
    /// 1 stopped, 2 acquisition, 3 overrun).
    marks: Vec<(usize, i64)>,
}

/// The cycle of sample n's tick in the testbench's clock pattern.
fn tb_tick(n: usize, period: usize) -> usize {
    (0..=n).map(|k| period + tb_idle(k)).sum::<usize>() + HandoffTiming::default().tick()
}

/// One segment of the hand-off's gate (see [`handoff_vectors`]); with no `out`, the run
/// alone.
#[allow(clippy::too_many_arguments)]
fn handoff_segment(
    out: Option<&mut dyn Write>,
    label: &str,
    x: &[(i16, i16)],
    w: usize,
    period: usize,
    pattern: u32,
    away: &[(usize, usize)],
    reader: &mut dyn HandoffReader,
    digest: bool,
    c: &mut HandoffCover,
) -> io::Result<HandoffOutcome> {
    let cfg = search_config(w);
    let fx = FixedRxConfig::default();
    let pace = HandoffConfig::default();
    let timing = HandoffTiming {
        clocks_per_sample: period,
        ..HandoffTiming::default()
    };
    let run = run_handoff(x, &cfg, &fx, &pace, &timing, &tb_idle, reader)
        .map_err(|e| io::Error::other(format!("{label}: the schedule refuses: {e:?}")))?;

    // The stream is the model's: every period's beats, the hold and the acquisition.
    let free = HandoffConfig {
        rate: usize::MAX,
        ..pace
    };
    let mut model = RxHandoffFixed::new(cfg, fx, free);
    let (mut b, mut v) = (0, 0);
    let mismatch = |what: &str, n: usize| {
        io::Error::other(format!(
            "{label}: the clocks and the model differ, {what} at {n}"
        ))
    };
    let mut outcome = HandoffOutcome::default();
    let (mut held, mut from_hold, mut releases, mut long_periods) = (false, 0, 0, 0);
    for (n, &s) in x.iter().enumerate() {
        while let Some(strobe) = run.verdicts.get(v).filter(|strobe| strobe.before <= n) {
            model.signal(strobe.packet, strobe.verdict);
            v += 1;
        }
        let step = model.push(s, run.room[n]);
        let end = run.ticks.get(n + 1).copied().unwrap_or(run.end);
        let from = b;
        while run.beats.get(b).is_some_and(|beat| beat.0 <= end) {
            b += 1;
        }
        let got: Vec<HandoffBeat> = run.beats[from..b].iter().map(|beat| beat.1).collect();
        if got != step.beats {
            return Err(mismatch("beats", n));
        }
        if run.hold[n] != step.hold || run.acquired[n] != step.acquired {
            return Err(mismatch("hold or acquisition", n));
        }
        let mut marked = false;
        for beat in &got {
            let f = handoff_fields(beat);
            if f[0] == 2 {
                outcome.marks.push((n, f[1]));
                marked = true;
            }
        }
        releases += usize::from(held && !step.hold && !marked && step.acquired.is_none());
        long_periods += usize::from(got.len() > 8);
        match (held, step.hold) {
            (false, true) => from_hold = n,
            (true, false) => outcome.holds.push((from_hold, n)),
            _ => {}
        }
        held = step.hold;
    }
    if held {
        outcome.holds.push((from_hold, x.len()));
    }
    outcome.acquired = run.acquired.iter().flatten().copied().collect();
    let Some(out) = out else {
        return Ok(outcome);
    };
    c.releases += releases;
    c.long_periods += long_periods;

    writeln!(
        out,
        "reset {label} {w} {period} {pattern} {}",
        u8::from(digest)
    )?;
    let mut k = 0;
    while k < x.len() {
        let same = |j: usize| x[j] == x[k] && run.hold[j] == run.hold[k];
        let more = (k + 1..x.len()).take_while(|&j| same(j)).count();
        writeln!(out, "x {} {} {}", x[k].0, x[k].1, u8::from(run.hold[k]))?;
        if more > 0 {
            writeln!(out, "r {more}")?;
        }
        k += 1 + more;
    }
    for a in away {
        writeln!(out, "A {} {}", a.0, a.1)?;
    }
    for e in &run.estimates {
        writeln!(out, "D {} {} {}", e.done, e.dc.0, e.dc.1)?;
    }
    for strobe in &run.verdicts {
        let (failed, symbols, patient) = match strobe.verdict {
            Some(verdict) => (0, verdict.data_symbols.min(2047), u8::from(verdict.patient)),
            None => (1, 0, 0),
        };
        writeln!(
            out,
            "V {} {} {failed} {symbols} {patient}",
            strobe.clock, strobe.packet
        )?;
        match strobe.verdict {
            None => c.failures += 1,
            Some(verdict) if verdict.patient => c.patient += 1,
            Some(_) => c.lengths += 1,
        }
    }
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for (cycle, beat) in &run.beats {
        let f = handoff_fields(beat);
        h = handoff_digest(h, *cycle, f);
        if !digest {
            match f[0] {
                0 => writeln!(out, "B {cycle} 0 {} {}", f[1], f[2])?,
                1 => writeln!(out, "B {cycle} 1 {} {} {} {}", f[1], f[2], f[3], f[4])?,
                _ => writeln!(out, "B {cycle} 2 {}", f[1])?,
            }
        }
        if f[0] == 2 {
            c.marks[f[1] as usize] += 1;
        }
    }
    writeln!(out, "end {} {} {h}", run.end, run.beats.len())?;

    c.samples += x.len();
    c.acquisitions += outcome.acquired.len();
    c.beats += run.beats.len();
    let tags: Vec<u8> = run
        .beats
        .iter()
        .filter_map(|beat| match beat.1 {
            HandoffBeat::Sample { first: Some(s), .. } => Some(s.packet),
            _ => None,
        })
        .collect();
    c.foreign += run
        .verdicts
        .iter()
        .filter(|strobe| !tags.contains(&strobe.packet))
        .count();
    for e in &run.estimates {
        c.offsets += usize::from(e.dc != (0, 0));
        c.offset_max = c.offset_max.max(e.dc.0.abs()).max(e.dc.1.abs());
    }
    c.spare = c.spare.min(run.history_spare);
    c.buffered = c.buffered.max(run.buffered);
    Ok(outcome)
}

/// Frames at 6 Mb/s of `octets` octets each behind `lead` samples, `gap` samples after
/// each, at −12 dBFS and 30 dB, with a CFO of 71 kHz, on a DC offset of −20 dBFS. A frame of
/// n octets has ⌈(22 + 8·n) / 24⌉ data symbols (IEEE 802.11-2020 §17.3.5.4).
fn handoff_scene(
    octets: &[usize],
    lead: usize,
    gap: usize,
    seed: u64,
) -> io::Result<Vec<(i16, i16)>> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    let mut rng = Rng::new(seed);
    let mut x = vec![(0.0, 0.0); lead];
    let mut p = 1.0;
    for &n in octets {
        let f: Vec<(f64, f64)> = build_ppdu(&vec![0xa7; n], Rate::R6, 0x5d, &cfg)
            .map_err(io::Error::other)?
            .iter()
            .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
            .collect();
        p = mean_power(&f);
        x.extend_from_slice(&f);
        x.extend(vec![(0.0, 0.0); gap]);
    }
    let x = frequency_offset(&x, 71e3, 20e6, 1.0);
    let x = awgn(&x, 30.0, p, &mut rng);
    let gain = f64::from(ADC_FULL_SCALE) * 10f64.powf(-12.0 / 20.0) / p.sqrt();
    let dc = 10f64.powf(-20.0 / 20.0) / gain * f64::from(ADC_FULL_SCALE);
    let x: Vec<(f64, f64)> = x.iter().map(|s| (s.0 - 0.8 * dc, s.1 + 0.6 * dc)).collect();
    Ok(to_adc(&x, gain))
}

/// `x` with `count` samples from `from` on a rail: I on the upper, Q on the lower, I on the
/// lower and Q on the upper in turn.
fn handoff_rails(x: &[(i16, i16)], from: usize, count: usize) -> Vec<(i16, i16)> {
    let mut out = x.to_vec();
    for (i, v) in out[from..from + count].iter_mut().enumerate() {
        *v = match i % 4 {
            0 => (2047, v.1),
            1 => (v.0, -2048),
            2 => (-2048, v.1),
            _ => (v.0, 2047),
        };
    }
    out
}

/// The scripted segments of the hand-off's gate: scenes made for one outcome each, which
/// is checked here before the segment is written (the outcomes are `g2_rx_handoff_schedule`'s,
/// on this tool's clock pattern).
fn handoff_scripted(out: &mut dyn Write, c: &mut HandoffCover) -> io::Result<()> {
    const SIGNAL_END: usize = PAIR_LEN + GI_LEN + LTF_SYMBOL_LEN;
    const B: usize = 4;
    // 24 octets: 9 data symbols; 3 octets: 2.
    const SYMBOLS: usize = 9;
    let told = |data_symbols: usize, patient: bool| {
        Some(SignalVerdict {
            data_symbols,
            patient,
        })
    };
    let tick = |n: usize| tb_tick(n, 8);
    type Strobes = Vec<(usize, u8, Option<SignalVerdict>)>;
    let mut number = 0;
    // One scripted segment: written, and its outcome held to the expected one.
    let mut segment = |out: &mut dyn Write,
                       c: &mut HandoffCover,
                       what: &str,
                       x: &[(i16, i16)],
                       away: Vec<(usize, usize)>,
                       strobes: Strobes,
                       holds: Option<Vec<(usize, usize)>>,
                       marks: Option<Vec<(usize, i64)>>|
     -> io::Result<()> {
        number += 1;
        let label = format!("script_{number:02}_{what}");
        let mut reader = ScriptReader {
            pattern: 0,
            away: away.clone(),
            strobes,
        };
        let got = handoff_segment(Some(out), &label, x, 12, 8, 0, &away, &mut reader, false, c)?;
        if holds.as_ref().is_some_and(|h| *h != got.holds)
            || marks.as_ref().is_some_and(|m| *m != got.marks)
        {
            return Err(io::Error::other(format!(
                "{label}: expected the hold {holds:?} and the marks {marks:?}, got {got:?}"
            )));
        }
        Ok(())
    };
    // The acquisitions of a scene, with a reader that takes everything and no verdict.
    let probe = |x: &[(i16, i16)]| -> io::Result<Vec<FixedAcquisition>> {
        let mut reader = ScriptReader {
            pattern: 0,
            away: Vec::new(),
            strobes: Vec::new(),
        };
        let mut none = HandoffCover::default();
        Ok(handoff_segment(
            None,
            "probe",
            x,
            12,
            8,
            0,
            &[],
            &mut reader,
            false,
            &mut none,
        )?
        .acquired)
    };
    let one = |x: &[(i16, i16)]| -> io::Result<FixedAcquisition> {
        match probe(x)?[..] {
            [a] => Ok(a),
            ref other => Err(io::Error::other(format!(
                "one acquisition expected: {other:?}"
            ))),
        }
    };
    let two = |x: &[(i16, i16)]| -> io::Result<(FixedAcquisition, FixedAcquisition)> {
        match probe(x)?[..] {
            [a, b] => Ok((a, b)),
            ref other => Err(io::Error::other(format!(
                "two acquisitions expected: {other:?}"
            ))),
        }
    };

    // ---- the release's two rules and their windows, on one frame
    let x = handoff_scene(&[24], 500, 500, 0x4A0D_5001)?;
    let a = one(&x)?;
    let (d, len) = (a.decided_at, x.len());
    let end = a.fft_start + SIGNAL_END + 80 * SYMBOLS + B;
    let last = end + 8;
    let at = a.fft_start + SIGNAL_END + 200;
    let length = |patient: bool| -> Strobes { vec![(tick(d + 60), 1, told(SYMBOLS, patient))] };
    let apart = |gap: usize| handoff_rails(&handoff_rails(&x, at, 1), at + gap, 1);
    let spread = |gap: usize| handoff_rails(&handoff_rails(&x, at, 1), at + 1 + gap, 23);
    let packet = Some(vec![(last - 1, 0)]);
    segment(
        out,
        c,
        "two_in_four",
        &apart(3),
        vec![],
        length(false),
        Some(vec![(d, at + 3)]),
        packet.clone(),
    )?;
    segment(
        out,
        c,
        "two_in_five",
        &apart(4),
        vec![],
        length(false),
        Some(vec![(d, end)]),
        packet.clone(),
    )?;
    segment(
        out,
        c,
        "patient_two",
        &apart(1),
        vec![],
        length(true),
        Some(vec![(d, end)]),
        packet.clone(),
    )?;
    segment(
        out,
        c,
        "patient_24_in_32",
        &spread(8),
        vec![],
        length(true),
        Some(vec![(d, at + 31)]),
        packet.clone(),
    )?;
    segment(
        out,
        c,
        "patient_24_in_33",
        &spread(9),
        vec![],
        length(true),
        Some(vec![(d, end)]),
        packet.clone(),
    )?;
    let rails_23 = handoff_rails(&x, at, 23);
    segment(
        out,
        c,
        "patient_23",
        &rails_23,
        vec![],
        length(true),
        Some(vec![(d, end)]),
        packet.clone(),
    )?;
    // A unit inside each rail is not on it.
    let mut near = x.clone();
    near[at] = (2046, -2047);
    near[at + 1] = (-2047, 2046);
    near[at + 2] = (2046, 2046);
    near[at + 3] = (-2047, -2047);
    segment(
        out,
        c,
        "a_unit_inside_the_rails",
        &near,
        vec![],
        length(false),
        Some(vec![(d, end)]),
        packet.clone(),
    )?;
    // A sample on a rail before the declaration's and the declaration's own: the hold it
    // raises is given up by it.
    let at_d = handoff_rails(&x, d - 1, 2);
    segment(
        out,
        c,
        "release_at_the_declaration",
        &at_d,
        vec![],
        vec![],
        Some(vec![]),
        None,
    )?;

    // ---- verdict strobes between ticks
    let m = d + 100;
    let t = tick(m);
    let short = told(3, false);
    let long = told(SYMBOLS, false);
    let short_end = a.fft_start + SIGNAL_END + 80 * 3 + B;
    let cases: [(&str, Strobes, usize); 9] = [
        ("the_cycle_before_a_tick", vec![(t - 1, 1, long)], end),
        (
            "another_tag_then_its_own",
            vec![(t, 7, short), (t + 2, 1, long)],
            end,
        ),
        (
            "two_of_its_own",
            vec![(t + 1, 1, long), (t + 2, 1, short)],
            end,
        ),
        (
            "its_own_then_a_failure",
            vec![(t + 3, 1, long), (t + 7, 1, None)],
            end,
        ),
        (
            "in_a_tick_then_after_it",
            vec![(t, 1, long), (t + 3, 1, None)],
            end,
        ),
        (
            "a_second_periods_later",
            vec![(t, 1, long), (t + 80, 1, short)],
            end,
        ),
        (
            "a_shorter_second",
            vec![(t, 1, short), (t + 80, 1, long)],
            short_end,
        ),
        (
            "failures_of_other_tags",
            vec![(t, 0, None), (t + 100, 2, None)],
            len,
        ),
        (
            "a_failure_before_any_packet",
            vec![(tick(100), 0, None), (tick(300) + 3, 1, None)],
            len,
        ),
    ];
    for (what, strobes, hold_end) in cases {
        segment(
            out,
            c,
            what,
            &x,
            vec![],
            strobes,
            Some(vec![(d, hold_end.min(len))]),
            None,
        )?;
    }
    // A failure in the estimate; and a second failure with the same tag once the stream
    // is stopped: one mark.
    let stopped = Some(vec![(d + 7, 1)]);
    let twice: Strobes = vec![(tick(d) + 50, 1, None), (tick(d + 200), 1, None)];
    segment(
        out,
        c,
        "a_failure_twice",
        &x,
        vec![],
        twice,
        Some(vec![(d, d + 7)]),
        stopped,
    )?;
    // A late length ends hold and stream at the tick that takes it.
    let late: Strobes = vec![(tick(end + 40), 1, long)];
    segment(
        out,
        c,
        "a_late_length",
        &x,
        vec![],
        late,
        Some(vec![(d, end + 41)]),
        Some(vec![(end + 41, 0)]),
    )?;
    // An end mark that waits 600 sample periods is not an overrun.
    let x_long = handoff_scene(&[24], 500, 1200, 0x4A0D_5001)?;
    let asleep = vec![(tick(last - 1) + 2, tick(last + 600))];
    segment(
        out,
        c,
        "asleep_at_the_end_mark",
        &x_long,
        asleep,
        length(false),
        Some(vec![(d, end)]),
        Some(vec![(last + 600, 0)]),
    )?;
    // A reader 512 behind at a tick but for the beat it takes in that tick's own cycle:
    // not lost; away a cycle longer: lost.
    let stop = d + 300;
    for (more, marks) in [(0usize, vec![]), (1, vec![(stop + 513, 3)])] {
        let away = vec![(tick(stop) + 2, tick(stop + 513) - 1 + more)];
        let what = if more == 0 {
            "kept_by_the_beat_of_the_tick"
        } else {
            "lost_a_cycle_later"
        };
        segment(out, c, what, &x_long, away, vec![], None, Some(marks))?;
    }

    // A length that comes at the very tick the reader first stands 512 behind, and ends
    // the stream before where the reader is: a packet's end, not a loss.
    {
        let stop = d + 300;
        let lost = stop + 513;
        let away = vec![(tick(stop) + 2, tick(lost) + 20)];
        let strobes: Strobes = vec![(tick(lost) - 1, 1, short)];
        segment(
            out,
            c,
            "a_length_at_the_tick_of_the_loss",
            &x_long,
            away,
            strobes,
            Some(vec![(d, lost)]),
            Some(vec![(lost + 2, 0)]),
        )?;
    }

    // ---- two frames a SIFS apart
    let x = handoff_scene(&[24, 24], 500, 320, 0x4A0D_5002)?;
    let (a, b) = two(&x)?;
    let len = x.len();
    let end = a.fft_start + SIGNAL_END + 80 * SYMBOLS + B;
    let t2 = tick(b.decided_at);
    // Two frames 30 samples apart: the second is declared less than a history after the
    // first one's last sample. The first read to its end mark, the mark not taken: a
    // packet's mark at the second declaration. With its last sample taken in the declaring
    // tick's own cycle: the same. A cycle later: an abort.
    {
        let x = handoff_scene(&[24, 24], 500, 30, 0x4A0D_5004)?;
        let (a, b) = two(&x)?;
        let last = a.fft_start + SIGNAL_END + 80 * SYMBOLS + B + 8;
        if b.decided_at <= last || b.decided_at >= last + 505 {
            return Err(io::Error::other(format!(
                "the second frame is declared {} samples after the first one's last",
                b.decided_at as i64 - last as i64
            )));
        }
        let t2 = tick(b.decided_at);
        let length: Strobes = vec![(tick(a.decided_at + 60), 1, told(SYMBOLS, false))];
        let cases = [
            (
                "the_mark_waits_for_the_declaration",
                tick(last - 1) + 2,
                t2 + 20,
                0,
                b.decided_at + 2,
            ),
            (
                "the_last_sample_in_the_declaring_tick",
                tick(last - 1) + 1,
                t2 - 1,
                0,
                b.decided_at,
            ),
            (
                "the_last_sample_after_the_declaring_tick",
                tick(last - 1) + 1,
                t2,
                2,
                b.decided_at,
            ),
        ];
        // No verdict, and a reader that stops so that its stream is lost at the very
        // sample of the second declaration: the declaration's mark, an abort. A sample
        // earlier: the overrun's.
        for (short, kind) in [(0usize, 2i64), (1, 3)] {
            let stop = b.decided_at - 513 - short;
            let away = vec![(tick(stop) + 2, t2 + 20)];
            let what = if short == 0 {
                "lost_at_the_declaration"
            } else {
                "lost_a_sample_before_the_declaration"
            };
            let marks = Some(vec![(b.decided_at + 2, kind)]);
            segment(out, c, what, &x, away, vec![], None, marks)?;
        }
        for (what, from, to, kind, taken) in cases {
            let marks = Some(vec![(taken, kind)]);
            segment(
                out,
                c,
                what,
                &x,
                vec![(from, to)],
                length.clone(),
                None,
                marks,
            )?;
        }
    }
    // No verdict: the second declaration aborts the first stream; the second runs past
    // where the first would have ended (its own watchdog, not the first one's length).
    segment(
        out,
        c,
        "an_abort",
        &x,
        vec![],
        vec![],
        Some(vec![(a.decided_at, len)]),
        Some(vec![(b.decided_at, 2)]),
    )?;
    let second_runs: Strobes = vec![(tick(a.decided_at + 60), 1, told(1, false))];
    // (Told a symbol 60 samples after the declaration, which is past that frame's end:
    // the hold and the stream end at the tick that takes it.)
    let first_end = a.decided_at + 61;
    segment(
        out,
        c,
        "a_short_first_and_a_second_without_verdict",
        &x,
        vec![],
        second_runs,
        Some(vec![(a.decided_at, first_end), (b.decided_at, len)]),
        Some(vec![(first_end, 0)]),
    )?;
    // The first frame's length in the cycle before the declaring tick: taken before the
    // declaration, the first stream ends as a packet. In the tick's own cycle: it is the
    // next tick's, with a tag that is no longer open.
    let before: Strobes = vec![(t2 - 1, 1, told(SYMBOLS, false))];
    segment(
        out,
        c,
        "a_length_before_the_declaring_tick",
        &x,
        vec![],
        before,
        None,
        Some(vec![(b.decided_at, 0)]),
    )?;
    let in_tick: Strobes = vec![(t2, 1, None)];
    segment(
        out,
        c,
        "the_old_tag_in_the_declaring_tick",
        &x,
        vec![],
        in_tick,
        Some(vec![(a.decided_at, len)]),
        Some(vec![(b.decided_at, 2)]),
    )?;
    // The new packet's tag in the declaring tick's own cycle counts for the new packet, also
    // after a strobe for the old one in the same period.
    let both: Strobes = vec![(t2 - 3, 1, told(SYMBOLS, false)), (t2, 2, None)];
    segment(
        out,
        c,
        "old_and_new_tags_around_the_declaring_tick",
        &x,
        vec![],
        both,
        Some(vec![(a.decided_at, b.decided_at + 1)]),
        Some(vec![(b.decided_at, 0), (b.decided_at + 1, 1)]),
    )?;
    // Inheritance: the first told patient and far too long, its hold up at the second
    // declaration: two samples on a rail do nothing. Told its true length: the hold is
    // down before, the second is not patient, and they take its hold.
    let clipped = handoff_rails(&x, b.decided_at + 30, 2);
    let patient_long: Strobes = vec![(tick(a.decided_at + 60), 1, told(1366, true))];
    segment(
        out,
        c,
        "patience_inherited",
        &clipped,
        vec![],
        patient_long,
        Some(vec![(a.decided_at, len)]),
        None,
    )?;
    let patient_true: Strobes = vec![(tick(a.decided_at + 60), 1, told(SYMBOLS, true))];
    segment(
        out,
        c,
        "patience_not_inherited",
        &clipped,
        vec![],
        patient_true,
        Some(vec![(a.decided_at, end), (b.decided_at, b.decided_at + 31)]),
        None,
    )?;
    // The hold that ends at the very sample of the second declaration: down before the
    // declaration looks at it. The gap is searched for a length that ends it there.
    let mut found = None;
    for gap in 320..520 {
        let x = handoff_scene(&[24, 24], 500, gap, 0x4A0D_5003)?;
        if let Ok((a, b)) = two(&x) {
            let span = b.decided_at - a.fft_start - SIGNAL_END - B;
            if span.is_multiple_of(80) {
                found = Some((x, a, b, span / 80));
                break;
            }
        }
    }
    let (x, a, b, symbols) =
        found.ok_or_else(|| io::Error::other("no gap ends a hold at the declaration"))?;
    let clipped = handoff_rails(&x, b.decided_at + 30, 2);
    for (more, holds) in [
        (0usize, vec![(a.decided_at, b.decided_at + 31)]),
        (1, vec![(a.decided_at, x.len())]),
    ] {
        let strobes: Strobes = vec![(tick(a.decided_at + 60), 1, told(symbols + more, true))];
        let what = if more == 0 {
            "the_hold_ends_at_the_declaration"
        } else {
            "the_hold_ends_a_symbol_later"
        };
        segment(out, c, what, &clipped, vec![], strobes, Some(holds), None)?;
    }
    Ok(())
}

/// Vectors for the hand-off (gate G3: `make -C hw handoff`): `hw/rtl/rx_handoff.sv` behind
/// `rx_front.sv` against `rx_handoff::run_handoff`, a cycle at a time, which this tool
/// first holds to `RxHandoffFixed::push` period by period (it refuses to write otherwise).
///
/// A segment is `reset <label> <w> <R> <ready pattern> <digest>`, then per ADC sample
/// `x <re> <im> <hold after its tick>` (`r <count>`: the line before that many times
/// more), then `A <from> <to>` (cycles the reader is away, whatever its pattern),
/// `D <cycle> <re> <im>` (the first cycle an offset is registered in, and the offset: a
/// probe inside the block), the verdict strobes
/// `V <cycle> <tag> <failed> <data symbols> <patient>` and the beats
/// `B <cycle> <kind> …` (kind 0 a sample: I, Q; 1 a stream's first: I, Q, the CFO word, the
/// tag; 2 a mark: its kind 0 packet, 1 stopped, 2 acquisition, 3 overrun), then
/// `end <last cycle> <beats> <digest of the beats>`. A segment with digest 1 lists no
/// beats. Cycles count from the reset's release; sample n is at the pins in cycle
/// X(n) = X(n − 1) + R + the testbenches' idle clocks ([`tb_idle`]), X(−1) = 0, and the
/// reader is ready by [`handoff_ready`].
fn handoff_vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec handoff — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# reset <label> <w> <R> <ready pattern> <digest>; x <re> <im> <hold>; r <count>;"
    )?;
    writeln!(
        out,
        "# A <from> <to>; D <cycle> <dc re> <dc im>; V <cycle> <tag> <failed> <symbols> <patient>;"
    )?;
    writeln!(
        out,
        "# B <cycle> <kind> ...; end <last cycle> <beats> <digest>"
    )?;
    let mut c = HandoffCover {
        spare: usize::MAX,
        ..HandoffCover::default()
    };
    let by_number = |pattern: u32, symbols: usize, first_stream: usize| VectorReader {
        pattern,
        symbols,
        due: Vec::new(),
        streams: first_stream,
        taken: 0,
        tag: 0,
    };
    // The front's long stream of short frames (two data symbols), every pattern of the
    // reader at w = 12 and R = 8; other back-offs and clocks with one pattern each.
    for (pattern, frames) in [(0u32, 12usize), (1, 8), (2, 10), (3, 8), (4, 8), (5, 8)] {
        let x = front_stream(frames, 0x4A0D_0000 + u64::from(pattern))?;
        let label = format!("long_stream_p{pattern}");
        let mut reader = by_number(pattern, 2, 0);
        handoff_segment(
            Some(out),
            &label,
            &x,
            12,
            8,
            pattern,
            &[],
            &mut reader,
            pattern % 2 == 1,
            &mut c,
        )?;
    }
    for (w, period, pattern) in [
        (16usize, 8usize, 2u32),
        (8, 8, 0),
        (4, 8, 2),
        (12, 11, 2),
        (12, 16, 0),
    ] {
        let x = front_stream(7, 0x4A0D_1000 + (w * 100 + period) as u64)?;
        let label = format!("long_stream_w{w}_r{period}");
        let mut reader = by_number(pattern, 2, 0);
        handoff_segment(
            Some(out),
            &label,
            &x,
            w,
            period,
            pattern,
            &[],
            &mut reader,
            false,
            &mut c,
        )?;
    }
    // A frame of 35 data symbols on a DC offset: an offset that is not zero, a reader at
    // the antenna for a long stream, each of the eight verdicts (the reader's count of
    // streams starts so that the frame's is the k-th kind), and readers slow enough to
    // lose it.
    for (k, pattern) in [0u32, 1, 2, 3, 0, 3, 1, 2].into_iter().enumerate() {
        let x = handoff_scene(&[100], 500 + 37 * k, 700, 0x4A0D_2000 + k as u64)?;
        let label = format!("frame_{k}");
        let mut reader = by_number(pattern, 35, k + 7);
        handoff_segment(
            Some(out),
            &label,
            &x,
            12,
            8,
            pattern,
            &[],
            &mut reader,
            k >= 2,
            &mut c,
        )?;
    }
    handoff_scripted(out, &mut c)?;
    // The watchdog: a frame, then silence to the end of the longest PPDU's 5.5 ms, told a
    // length above the longest (the fifth kind of verdict). Its beats as a digest.
    let mut x = handoff_scene(&[100], 500, 700, 0x4A0D_3000)?;
    x.extend(vec![(0, 0); 108_000]);
    let mut reader = by_number(0, 35, 4);
    let got = handoff_segment(
        Some(out),
        "watchdog",
        &x,
        12,
        8,
        0,
        &[],
        &mut reader,
        true,
        &mut c,
    )?;
    let watchdog = PAIR_LEN + GI_LEN + LTF_SYMBOL_LEN + 80 * 1366 + 4;
    let ends = got
        .acquired
        .first()
        .map(|a| (a.decided_at, a.fft_start + watchdog));
    if got.holds.first().copied() != ends || got.marks.len() != 1 {
        return Err(io::Error::other(format!(
            "the watchdog's segment does not end by the watchdog: {:?} {:?}",
            got.holds, got.marks
        )));
    }

    if c.marks.contains(&0)
        || c.lengths == 0
        || c.failures == 0
        || c.patient == 0
        || c.foreign == 0
        || c.releases == 0
        || c.offsets == 0
        || c.long_periods == 0
        || c.buffered != 7
    {
        return Err(io::Error::other(format!(
            "the hand-off's vectors leave a case out: {c:?}"
        )));
    }
    eprintln!("rvec handoff: {c:?}");
    Ok(())
}

/// `fft64`'s windows: one line a window, `<64 samples re,im> <64 bins re,im>`, the samples
/// at the hand-off's 13 bits in time order, the bins in natural order as
/// `fft64_window_fixed` gives them. Random full-scale windows, then windows that saturate
/// (full-scale tones on every bin, constants and impulses at the rails, alternating signs).
fn fft64_vectors(out: &mut impl Write) -> io::Result<()> {
    const RANDOM: usize = 160;
    let fx = FixedOfdmConfig::default();
    let bad = || io::Error::other("unusable FFT widths");
    let hi = (1i64 << (fx.in_bits - 1)) - 1;
    let lo = -hi - 1;
    writeln!(
        out,
        "# rvec fft64 — generated, do not edit. Regenerate: make -C hw vectors\n\
         # radio_phy::rx_ofdm_fixed::fft64_window_fixed, FixedOfdmConfig::default() ({} bits in, \
         shift {}, W {}, T {}, halve {:#08b}).\n\
         # <64 samples re,im in time order> <64 bins re,im in bin order>",
        fx.in_bits, fx.in_shift, fx.fft_bits, fx.twiddle_bits, fx.fft_halve
    )?;
    let mut rng = Rng::new(0xff7_0064);
    let mut windows: Vec<Vec<IQ>> = Vec::new();
    let span = (hi - lo + 1) as u64;
    for _ in 0..RANDOM {
        windows.push(
            (0..64)
                .map(|_| {
                    let r = (rng.next_u64() % span) as i64 + lo;
                    let i = (rng.next_u64() % span) as i64 + lo;
                    (r, i)
                })
                .collect(),
        );
    }
    // A full-scale tone on bin k: every bin's sum is coherent, the late stages saturate.
    for k in 0..64 {
        let ph = |n: usize| 2.0 * std::f64::consts::PI * (k * n) as f64 / 64.0;
        windows.push(
            (0..64)
                .map(|n| {
                    let q = |v: f64| ((v * hi as f64).round() as i64).clamp(lo, hi);
                    (q(ph(n).cos()), q(ph(n).sin()))
                })
                .collect(),
        );
    }
    for &(r, i) in &[(hi, hi), (lo, lo), (hi, lo), (lo, hi)] {
        windows.push(vec![(r, i); 64]);
        let mut imp = vec![(0, 0); 64];
        imp[0] = (r, i);
        windows.push(imp);
        windows.push(
            (0..64)
                .map(|n| {
                    if n % 2 == 0 {
                        (r, i)
                    } else {
                        ((-r).clamp(lo, hi), (-i).clamp(lo, hi))
                    }
                })
                .collect(),
        );
    }
    for w in &windows {
        let bins = fft64_window_fixed(w, &fx).ok_or_else(bad)?;
        let s: Vec<String> = w.iter().map(|v| format!("{},{}", v.0, v.1)).collect();
        let b: Vec<String> = bins.iter().map(|v| format!("{},{}", v.0, v.1)).collect();
        writeln!(out, "{} {}", s.join(" "), b.join(" "))?;
    }
    Ok(())
}

/// `fft64_rom_pkg`: the FFT's widths and twiddle ROM (`fft64_twiddles`).
fn fft64_rom(out: &mut impl Write) -> io::Result<()> {
    let fx = FixedOfdmConfig::default();
    let rom = fft64_twiddles(&fx).ok_or_else(|| io::Error::other("unusable FFT widths"))?;
    writeln!(
        out,
        "// rvec fft64-rom — generated from radio_phy::rx_ofdm_fixed, do not edit.\n\
         // Regenerate: make -C hw vectors\n\
         \n\
         `default_nettype none\n\
         \n\
         package fft64_rom_pkg;\n\
         \x20   // Widths of FixedOfdmConfig::default().\n\
         \x20   localparam int IN_BITS  = {};\n\
         \x20   localparam int IN_SHIFT = {};\n\
         \x20   localparam int FFT_W_BITS = {};\n\
         \x20   localparam int T_BITS   = {};\n\
         \x20   // Bit s set: stage s (0 first) halves its outputs.\n\
         \x20   localparam logic [5:0] HALVE = 6'b{:06b};",
        fx.in_bits, fx.in_shift, fx.fft_bits, fx.twiddle_bits, fx.fft_halve
    )?;
    let re: Vec<i64> = rom.iter().map(|v| v.0).collect();
    let im: Vec<i64> = rom.iter().map(|v| v.1).collect();
    rom_function(
        out,
        "tw_re",
        "Twiddle e^(-j2pi k/64), k < 32, real part, Q(T, T - 2): fft64_twiddles()",
        fx.twiddle_bits,
        &re,
    )?;
    rom_function(
        out,
        "tw_im",
        "Twiddle e^(-j2pi k/64), k < 32, imaginary part, Q(T, T - 2): fft64_twiddles()",
        fx.twiddle_bits,
        &im,
    )?;
    writeln!(out, "endpackage\n\n`default_nettype wire")
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
        "search_alone" => search_alone_vectors(&mut out),
        "decide_t1" => decide_t1_vectors(&mut out),
        "decide" => decide_vectors(&mut out),
        "front" => front_vectors(&mut out),
        "handoff" => handoff_vectors(&mut out),
        "dc_estimate" => dc_estimate_vectors(&mut out),
        "viterbi" => viterbi_vectors(&mut out),
        "fft64" => fft64_vectors(&mut out),
        "fft64-rom" => fft64_rom(&mut out),
        "rx_channel" => rx_channel_vec::vectors(&mut out),
        "rx-channel-pkg" => rx_channel_vec::package(&mut out),
        "rx_symbol" => rx_symbol_vec::vectors(&mut out),
        "rx-symbol-pkg" => rx_symbol_vec::package(&mut out),
        "rx_track" => rx_track_vec::vectors(&mut out),
        "rx_deint" => rx_deint_vec::vectors(&mut out),
        "rx-track-pkg" => rx_track_vec::package(&mut out),
        _ => {
            eprintln!(
                "usage: rvec <block>\nblocks: scrambler, conv_encoder, viterbi, round_sat, ltf_corr, cordic, nco, det, latch, search, search_alone, decide, decide_t1, front, dc_estimate, handoff, fft64, rx_channel, rx_symbol, rx_track, rx_deint, rx-front-rom, fft64-rom, rx-channel-pkg, rx-symbol-pkg, rx-track-pkg, ofdm-symbols, ppdu-frames"
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The names a generated package declares (localparams and functions), each with its
    /// declaration up to the end of the line.
    fn names(text: &str) -> Vec<(String, String)> {
        let mut out = Vec::new();
        for line in text.lines() {
            let line = line.trim();
            let head = if let Some(rest) = line.strip_prefix("localparam ") {
                rest.split('=').next().unwrap_or("")
            } else if let Some(rest) = line.strip_prefix("function automatic ") {
                rest.split('(').next().unwrap_or("")
            } else {
                continue;
            };
            // The name is the last identifier before `=` or `(`, outside brackets.
            let mut depth = 0;
            let outside: String = head
                .chars()
                .map(|c| {
                    match c {
                        '[' => depth += 1,
                        ']' => depth -= 1,
                        _ => {}
                    }
                    if depth > 0 || c == ']' {
                        ' '
                    } else {
                        c
                    }
                })
                .collect();
            if let Some(name) = outside.split_whitespace().last() {
                let body = line
                    .split("//")
                    .next()
                    .unwrap_or("")
                    .split_whitespace()
                    .collect::<Vec<_>>()
                    .join(" ");
                out.push((name.to_string(), body));
            }
        }
        out
    }

    /// Every generated package can be imported beside every other (`rx_symbol` imports three).
    /// Verilator resolves a name two imported packages declare to the first import's without a
    /// word: the demapper's K_FRAC took the channel's 16 instead of 10 (2026-11-24). A name may
    /// appear twice only with the same declaration (the same value).
    #[test]
    fn generated_packages_declare_distinct_names() {
        let mut all: Vec<(String, String, &str)> = Vec::new();
        let mut gen = |pkg: &'static str, f: &dyn Fn(&mut Vec<u8>) -> io::Result<()>| {
            let mut buf = Vec::new();
            f(&mut buf).unwrap();
            let found = names(&String::from_utf8(buf).unwrap());
            assert!(found.len() > 3, "{pkg}: {} names found", found.len());
            for (n, body) in found {
                all.push((n, body, pkg));
            }
        };
        gen("rx-front-rom", &|b| rx_front_rom(b));
        gen("fft64-rom", &|b| fft64_rom(b));
        gen("rx-channel-pkg", &|b| rx_channel_vec::package(b));
        gen("rx-symbol-pkg", &|b| rx_symbol_vec::package(b));
        gen("rx-track-pkg", &|b| rx_track_vec::package(b));
        all.sort();
        for w in all.windows(2) {
            if w[0].0 == w[1].0 {
                assert_eq!(
                    w[0].1, w[1].1,
                    "{} differs in {} and {}",
                    w[0].0, w[0].2, w[1].2
                );
            }
        }
        assert!(all.iter().any(|(n, _, _)| n == "KAPPA_FRAC"));
        assert!(all.iter().any(|(n, _, _)| n == "nco_cos"));
    }
}
