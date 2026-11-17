//! G1 gate for the OFDM modulator: OpenOFDM's receiver decodes our symbols.
//!
//! `tests/fixtures/ofdm_rx_openofdm.txt` holds symbols produced by `rvec ofdm-symbols` with the
//! transmit IFFT configuration, and what OpenOFDM's `ChannelEstimator.next_symbol` (Apache-2.0,
//! commit ea638eb, run unmodified; `tools/ref/ofdm_rx_openofdm.py`) made of them: the residual
//! phase β it estimated from our pilots, and the 48 data carriers after its numpy FFT.
//!
//! 1. Today's model must still produce exactly the recorded samples, so the verdict applies to
//!    the current code (any change to the datapath means regenerating the fixture).
//! 2. β ≈ 0: our pilot positions, base values and polarity agree with the receiver.
//! 3. The decoded carriers equal our points times K_MOD within the fixed-point EVM budget.

use radio_phy::mapper::{mean_energy, Point};
use radio_phy::modulation::Modulation;
use radio_phy::ofdm::{modulate_symbol, IfftConfig, Iq, SYMBOL_LEN};

const RX: &str = include_str!("fixtures/ofdm_rx_openofdm.txt");
/// EVM budget for the decoded carriers: 1 % (−40 dB), consistent with the 45 dB SQNR budget.
const EVM_BUDGET_PERCENT: f64 = 1.0;
const BETA_BUDGET_RAD: f64 = 0.01;

fn pair<T: std::str::FromStr>(s: &str) -> (T, T)
where
    T::Err: std::fmt::Debug,
{
    let (a, b) = s
        .split_once(',')
        .unwrap_or_else(|| panic!("not a pair: {s}"));
    (
        a.parse().unwrap_or_else(|e| panic!("{a}: {e:?}")),
        b.parse().unwrap_or_else(|e| panic!("{b}: {e:?}")),
    )
}

#[test]
fn openofdm_decodes_our_symbols() {
    let cfg = IfftConfig::tx_default();
    let mut symbols = 0;
    let mut per_mod = [0usize; 7];
    for line in RX.lines().filter(|l| !l.starts_with('#') && !l.is_empty()) {
        let f: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(f.len(), 3 + 48 + SYMBOL_LEN + 2 + 48, "malformed line");
        let n_bpsc: usize = f[0].parse().unwrap();
        let m = Modulation::from_bits_per_subcarrier(n_bpsc).unwrap();
        let idx: usize = f[1].parse().unwrap();
        assert_eq!(
            f[2].parse::<u32>().unwrap(),
            cfg.data.frac_bits(),
            "fixture is for another config"
        );
        let points: Vec<Point> = f[3..51]
            .iter()
            .map(|s| {
                let (i, q) = pair::<i8>(s);
                Point { i, q }
            })
            .collect();
        let recorded: Vec<Iq> = f[51..51 + SYMBOL_LEN]
            .iter()
            .map(|s| {
                let (re, im) = pair::<i64>(s);
                Iq { re, im }
            })
            .collect();
        assert_eq!(f[51 + SYMBOL_LEN], "beta");
        let beta: f64 = f[52 + SYMBOL_LEN].parse().unwrap();
        let carriers: Vec<(f64, f64)> = f[53 + SYMBOL_LEN..]
            .iter()
            .map(|s| pair::<f64>(s))
            .collect();

        // 1. The current model reproduces the decoded symbol bit for bit.
        assert_eq!(
            modulate_symbol(&points, m, idx, &cfg).unwrap(),
            recorded,
            "{m:?} #{idx}"
        );
        // 2. Pilots agree with the receiver.
        assert!(beta.abs() < BETA_BUDGET_RAD, "{m:?} #{idx}: beta {beta}");
        // 3. Carriers come back as K_MOD · point.
        let k_mod = 1.0 / f64::from(mean_energy(m)).sqrt();
        let (mut err, mut pow) = (0.0, 0.0);
        for (p, &(re, im)) in points.iter().zip(&carriers) {
            let (wr, wi) = (f64::from(p.i) * k_mod, f64::from(p.q) * k_mod);
            err += (re - wr).powi(2) + (im - wi).powi(2);
            pow += wr * wr + wi * wi;
        }
        let evm = 100.0 * (err / pow).sqrt();
        assert!(evm < EVM_BUDGET_PERCENT, "{m:?} #{idx}: EVM {evm:.3} %");
        symbols += 1;
        per_mod[n_bpsc] += 1;
    }
    assert_eq!(symbols, 32);
    assert_eq!(
        [per_mod[1], per_mod[2], per_mod[4], per_mod[6]],
        [8, 8, 8, 8]
    );
}
