//! `SearchAloneFixed` is the search of `RxFrontFixed`, driven at the correlator's output: the
//! reference of the `rx_search` bench alone (`make -C hw search_alone`, R0-08d; the
//! founder's decision of 2026-11-23 in `docs/gates/rx_front.md`).
//!
//! Not a G1 gate (both sides are our model): it shows that the driver the bench's vectors come
//! from runs the very arithmetic and control the chain runs. Fed with the events and
//! correlator outputs of `RxFrontFixed::push_traced`, and at each acceptance with the
//! decision's result the front applied `decision_latency` samples later, it must give every
//! energy, acceptance and search state of the front, sample for sample.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::{add_tone, awgn, fractional_delay, frequency_offset, mean_power, Rng};
use radio_phy::ofdm::IfftConfig;
use radio_phy::ppdu::{build_ppdu, Rate};
use radio_phy::rx_front::{RxConfig, GI_LEN};
use radio_phy::rx_front_fixed::{
    to_adc, FixedRxConfig, RxFrontFixed, SearchAloneFixed, ADC_FULL_SCALE,
};
use std::f64::consts::PI;

const FS: f64 = 20e6;

fn frame(octets: usize) -> Vec<(f64, f64)> {
    let cfg = IfftConfig::tx_default();
    let scale = 1.0 / f64::from(1u32 << cfg.data.frac_bits());
    build_ppdu(&vec![0x5a; octets], Rate::R6, 0x5d, &cfg)
        .unwrap()
        .iter()
        .map(|s| (s.re as f64 * scale, s.im as f64 * scale))
        .collect()
}

/// Frames behind random leads with their own CFO, a strong STF fragment with a noise burst
/// before some (a candidate that fails), and a tone over the second half (a search that stays
/// open, candidates rejected).
fn scene(seed: u64) -> Vec<(i16, i16)> {
    let mut rng = Rng::new(seed);
    let f = frame(20);
    let p = mean_power(&f);
    let mut x = Vec::new();
    for k in 0..10 {
        let mut part = vec![(0.0, 0.0); 150 + (rng.uniform() * 300.0) as usize];
        if k % 3 == 1 {
            part.extend(f[..160].iter().map(|&(a, b)| (a * 3.16, b * 3.16)));
            part.extend(awgn(&vec![(0.0, 0.0); 120], 0.0, 10.0 * p, &mut rng));
            part.extend(vec![(0.0, 0.0); 30]);
        }
        part.extend_from_slice(&f);
        part.extend(vec![(0.0, 0.0); 200]);
        let part = fractional_delay(&part, rng.uniform());
        let cfo = (rng.uniform() * 2.0 - 1.0) * 200e3;
        x.extend(frequency_offset(&part, cfo, FS, 2.0 * PI * rng.uniform()));
    }
    let half = x.len() / 2;
    let toned = add_tone(&x[half..], p, 1.3e6, FS, 0.3);
    x.truncate(half);
    x.extend(toned);
    let x = awgn(&x, 20.0, p, &mut rng);
    to_adc(
        &x,
        f64::from(ADC_FULL_SCALE) * 10f64.powf(-18.0 / 20.0) / (2.0 * p).sqrt(),
    )
}

#[test]
fn the_search_alone_is_the_fronts_search() {
    let mut totals = (0usize, 0usize, 0usize, 0usize); // energies, acceptances, acquired, failed
    for (w, latency, seed) in [
        (12usize, None, 0x5A1_0001u64),
        (16, None, 0x5A1_0002),
        (8, None, 0x5A1_0003),
        (1, None, 0x5A1_0004),
        (12, Some(20usize), 0x5A1_0005),
        (16, Some(79), 0x5A1_0006),
        // L > 63 + W: an acceptance due while a decision is in flight is held back.
        (4, Some(79), 0x5A1_0007),
    ] {
        let mut cfg = RxConfig::new(FS);
        cfg.backoff_ns = (GI_LEN - w) as f64 * 50.0;
        if let Some(l) = latency {
            cfg.decision_latency = l;
        }
        let lat = cfg.decision_latency;
        let x = scene(seed);
        let mut rx = RxFrontFixed::new(cfg, FixedRxConfig::default());
        let traced: Vec<_> = x
            .iter()
            .map(|s| (rx.push_traced(*s), rx.searching()))
            .collect();
        let mut alone = SearchAloneFixed::new(w, lat);
        for (n, ((acq, t), open)) in traced.iter().enumerate() {
            // The decision's result for a candidate accepted here: what the front applied.
            let acquire =
                t.accepted.is_some() && traced.get(n + lat).is_some_and(|((a, _), _)| a.is_some());
            let s = alone.push(t.event, t.corr.unwrap_or((0, 0)), acquire);
            assert_eq!(s.energy, t.energy, "w {w} L {lat} sample {n}: energy");
            assert_eq!(
                s.accepted, t.accepted,
                "w {w} L {lat} sample {n}: acceptance"
            );
            assert_eq!(
                s.acquired,
                acq.is_some(),
                "w {w} L {lat} sample {n}: acquired"
            );
            assert_eq!(s.searching, *open, "w {w} L {lat} sample {n}: searching");
            totals.0 += usize::from(s.energy.is_some());
            totals.1 += usize::from(s.accepted.is_some());
            totals.2 += usize::from(s.acquired);
            totals.3 += usize::from(s.accepted.is_some() && !acquire);
        }
    }
    let (energies, accepted, acquired, failed) = totals;
    assert!(
        energies > 50_000 && acquired >= 30 && failed >= 10,
        "{totals:?}"
    );
    assert!(accepted >= acquired + failed - 1, "{totals:?}");
}

/// The energy's bound (G0 of the bench): both components of every correlator output at
/// −2^(K−1) give e = 16 · 2·2^(2K−2) = 2^(2K+3) at W = 16, the 44th bit of 2K + 4 = 44.
#[test]
fn full_scale_correlator_outputs_reach_the_energys_top_bit() {
    let k = FixedRxConfig::default().corr_bits;
    let lo = -(1i64 << (k - 1));
    let mut alone = SearchAloneFixed::new(16, 64);
    let mut top = 0;
    for _ in 0..400 {
        if let Some(e) = alone.push(false, (lo, lo), false).energy {
            top = top.max(e);
        }
    }
    assert_eq!(top, 1i128 << (2 * k + 3));
    assert_eq!(128 - top.leading_zeros(), 2 * k + 4);
}
