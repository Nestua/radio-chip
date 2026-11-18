//! Shared test code for the receiver-front gates: a minimal 6 Mb/s decoder standing in for
//! R0-09 (least-squares channel from the LTF, pilot common phase, BPSK soft bits) feeding the
//! R0-10 Viterbi decoder. Test code only.

#![allow(dead_code, clippy::unwrap_used)]

use radio_phy::channel::fft;
use radio_phy::convolutional::CodeRate;
use radio_phy::interleaver::interleaved_index;
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{data_subcarriers, fft_bin, pilot_polarity, PILOT_BASE, PILOT_SUBCARRIERS};
use radio_phy::ppdu::{long_training_sequence, Rate};
use radio_phy::scrambler::Scrambler;
use radio_phy::viterbi::{decode, Termination};

type C = (f64, f64);

fn cmul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn conj(a: C) -> C {
    (a.0, -a.1)
}

fn abs2(a: C) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

fn fft64(x: &[C]) -> Vec<C> {
    fft(x).unwrap()
}

/// Decodes a 6 Mb/s PPDU from an acquired, CFO-corrected stream: least-squares channel from the
/// two LTF symbols, common phase from the pilots, BPSK soft bits, deinterleaving, Viterbi.
/// Returns (SIGNAL rate bits, LENGTH, PSDU); `None` if SIGNAL fails parity. A stand-in for R0-09, test code only.
pub fn decode_6mbps(y: &[C], fft_start: usize) -> Option<([u8; 4], usize, Vec<u8>)> {
    let ltf = long_training_sequence();
    let t1 = fft64(&y[fft_start..fft_start + 64]);
    let t2 = fft64(&y[fft_start + 64..fft_start + 128]);
    let mut h = [(0.0, 0.0); 64];
    for (k, &l) in (-26i8..=26).zip(ltf.iter()) {
        if l != 0 {
            let b = fft_bin(k);
            let l = f64::from(l);
            h[b] = (
                (t1[b].0 + t2[b].0) / (2.0 * l),
                (t1[b].1 + t2[b].1) / (2.0 * l),
            );
        }
    }
    let symbol = |i: usize| -> Vec<i32> {
        // Symbol i after the LTF (0 = SIGNAL); its FFT window keeps the same back-off.
        let start = fft_start + 128 + 16 + 80 * i;
        let z = fft64(&y[start..start + 64]);
        // Common phase error from the four pilots.
        let pol = f64::from(pilot_polarity(i));
        let cpe =
            PILOT_SUBCARRIERS
                .iter()
                .zip(PILOT_BASE.iter())
                .fold((0.0, 0.0), |a, (&c, &p)| {
                    let b = fft_bin(c);
                    let t = cmul(z[b], conj(h[b]));
                    let s = pol * f64::from(p);
                    (a.0 + s * t.0, a.1 + s * t.1)
                });
        let rot = conj((cpe.0 / abs2(cpe).sqrt(), cpe.1 / abs2(cpe).sqrt()));
        let rx: Vec<f64> = data_subcarriers()
            .iter()
            .map(|&c| {
                let b = fft_bin(c);
                // Re(z·h*) weighs by |h|²: the CSI-scaled matched filter. Bit 1 ↦ +1 on air,
                // the decoder wants positive = 0.
                -cmul(cmul(z[b], conj(h[b])), rot).0
            })
            .collect();
        let scale = 3.0 / (rx.iter().map(|v| v.abs()).sum::<f64>() / 48.0);
        let mut soft = vec![0i32; 48];
        for (k, s) in soft.iter_mut().enumerate() {
            let v = rx[interleaved_index(k, Modulation::Bpsk)] * scale;
            *s = v.round().clamp(-7.0, 7.0) as i32;
        }
        soft
    };
    if y.len() < fft_start + 128 + 16 + 80 {
        return None;
    }
    let sig = decode(&symbol(0), CodeRate::R1_2, Termination::ZeroAfter(24)).ok()?;
    let rate = [sig[0], sig[1], sig[2], sig[3]];
    let length = (5..17)
        .rev()
        .fold(0usize, |a, i| (a << 1) | sig[i] as usize);
    let parity = sig[..17].iter().fold(0u8, |a, b| a ^ b);
    // A SIGNAL with bad parity is no SIGNAL (OpenOFDM's verdict requires parity too).
    if parity != sig[17] {
        return None;
    }
    if rate != Rate::R6.signal_rate_bits() {
        return Some((rate, length, vec![]));
    }
    let n_sym = (16 + 8 * length + 6).div_ceil(24);
    if y.len() < fft_start + 128 + 16 + 80 * (n_sym + 1) {
        return None;
    }
    let soft: Vec<i32> = (1..=n_sym).flat_map(symbol).collect();
    let bits = decode(
        &soft,
        CodeRate::R1_2,
        Termination::ZeroAfter(16 + 8 * length + 6),
    )
    .ok()?;
    // The scrambler seed is whatever turns the first 7 SERVICE bits into zeros.
    let seed = (1u8..128).find(|&s| {
        Scrambler::new(s)
            .map(|mut sc| sc.scramble(&bits[..7]).iter().all(|b| *b == 0))
            .unwrap_or(false)
    })?;
    let clear = Scrambler::new(seed).ok()?.scramble(&bits);
    let psdu = clear[16..16 + 8 * length]
        .chunks(8)
        .map(|c| c.iter().rev().fold(0u8, |a, &b| (a << 1) | b))
        .collect();
    Some((rate, length, psdu))
}
