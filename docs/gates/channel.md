# Gate journal: channel simulator

Block: the simulated air between transmitter and receiver (R0-07). Model:
`crates/radio-phy/src/channel.rs`. Simulation only — nothing here becomes hardware, so it is
written in `f64` and has no G2/G3. Its job is to be a trustworthy test bench for the receiver
(R0-08…R0-10) and, later, for the drone link budget.

## 2026-11-17 · G0 · R0-07a
- Samples are complex `(f64, f64)` at `sample_rate_hz` (20 MHz for 802.11a/g, 5/10 MHz for the
  long-range profile).
- **RNG**: xoshiro256** seeded through SplitMix64 from a `u64`; one seed, one stream, on every
  platform. Normals by Box–Muller. Not cryptographic.
- **AWGN**: complex white Gaussian noise of power N = P / 10^(SNR/10), N/2 per I/Q axis, where P
  is the signal power the caller states (or `mean_power` of the slice). SNR is over the whole
  sampled band. For 802.11a/g with 52 of 64 bins occupied the per-subcarrier SNR is
  SNR + 10·log10(64/52) = SNR + 0.90 dB.
- **CFO**: x[n] · e^(j(2π·Δf·n/fs + φ0)). Transmit centre frequency and symbol clock are each
  ±20 ppm at 20/10 MHz and ±10 ppm at 5 MHz, from one oscillator (IEEE 802.11-2024 §17.3.9.5,
  §17.3.9.6); between two stations that is up to 40 ppm, 232 kHz at 5.8 GHz. `ppm_to_hz` converts.
  The matching sample-clock offset (SFO) comes with the fractional-delay filter in R0-07b.
- **Phase noise**: Wiener (free-running oscillator) model, φ[n] = φ[n−1] + N(0, 2πβ/fs), β the
  3 dB full linewidth. Its single-sideband level far from the carrier is L(f) = β/(2πf²), so a
  datasheet figure L(f0) dBc/Hz gives β = 2π·f0²·10^(L/10) (`linewidth_from_dbc`). Real
  synthesizers are flat inside the PLL loop bandwidth; the Wiener model is the worst case
  outside it and the usual first model (a PLL-shaped profile can follow when R3 hardware is known).

## 2026-11-17 · G1 · R0-07a first commit
Command: `cargo test -p radio-phy --test g1_channel -- --nocapture` (rustc 1.98.1, 1.1 s)
References:
1. CPython 3.9.6 `math.erfc`, black box (`tools/ref/erfc_python.py` →
   `tests/fixtures/erfc_python.txt`, 697 points on −3…26.5, densest near the series/continued-
   fraction switch). Our `erfc`: **worst relative error 1.1e-14** (gate: 1e-13). The e^(−x²)
   factor is split (h on a 1/16 grid) — without it x² rounding alone costs ~1.5e-13 at x = 26.
2. Exact SER in AWGN (Proakis §5.2; formulas live in the test, not the library), Monte Carlo
   on our own mapper's normalised constellations, brute-force nearest-point decisions, seed 4,
   ±5σ binomial bound. All 12 points within 1.9σ:

   | Mod | Es/N0 (dB) | errors / symbols | theory | σ |
   |---|---|---|---|---|
   | BPSK | 0 / 4 / 7 | 2078/25 429 · 2074/159 989 · 298/400 000 | 2000 · 2000 · 309.1 | +1.82 · +1.67 · −0.63 |
   | QPSK | 3 / 7 / 10 | 3040/20 000 · 2071/79 950 · 610/400 000 | 3031.3 · 2000 · 625.9 | +0.17 · +1.61 · −0.64 |
   | 16-QAM | 10 / 14 / 17 | 4430/20 000 · 1989/53 834 · 909/400 000 | 4440.6 · 2000 · 926.7 | −0.18 · −0.25 · −0.58 |
   | 64-QAM | 16 / 20 / 23 | 5430/20 000 · 2078/39 784 · 1409/400 000 | 5464.4 · 2000 · 1436.0 | −0.55 · +1.79 · −0.71 |
3. DFT shift theorem: a 64-sample tone moved by m·fs/64 (m = −7, 3, 11) lands in exactly one bin
   (|X| = 64 ± 1e-9, leakage < 1e-9); power preserved to 1e-12; ppm_to_hz(20, 5.8 GHz) = 116 kHz.
4. Wiener phase: variance after 1000 and 2000 steps = 2πβ·n/fs within ±5σ over 4000 runs.
   Lorentzian spectrum (exact sampled form, ρ = e^(−πβ/fs)) for −80 dBc/Hz at 100 kHz
   (β = 628.3 Hz), Welch with Hann windows, 300 × 4096: 103 kHz −80.11 vs −80.22; 200 kHz −85.90
   vs −86.03; 400 kHz −92.22 vs −92.04; 801 kHz −98.05 vs −98.05 dBc/Hz (gate ±0.6 dB).
5. SNR convention in OFDM: 1500 random 16-QAM symbols of our float modulator, SNR 12 dB over the
   band → per-subcarrier SNR measured 12.917 dB, expected 12 + 10·log10(64/52) = 12.902 dB.
Also: RNG determinism, uniform mean/variance/χ² (16 bins), Gaussian mean/variance/skew/kurtosis,
lag-1 whiteness and the 3σ tail count.
Result: 9/9 pass. Test-first: all 9 failed against the `todo!()` stub.
Mutation check (each restored with `cp` + `touch`):
| Mutation | Fails |
|---|---|
| AWGN noise power doubled (no ½ per axis) | awgn, SER, OFDM SNR |
| erfc continued fraction a_n = n instead of n/2 | erfc, SER |
| Wiener step variance πβ/fs instead of 2πβ/fs | variance, Lorentzian |
| CFO rotation sign flipped | shift theorem |
| linewidth_from_dbc off by 2 | Lorentzian |
| Box–Muller second output reuses cos (correlated pair) | Gaussian, awgn, SER, Wiener, Lorentzian |

## Open questions
1. RNG has no bit-level external reference (xoshiro256**'s reference outputs are not run here);
   its quality rests on the statistical checks and on every downstream curve matching theory.
   Acceptable for a simulator; revisit if a capture replay ever needs a shared stream.
2. Phase noise is Wiener only. For R3 hardware (AD9361) a PLL-shaped profile (flat inside the
   loop bandwidth) is more realistic; add it when the receiver's CPE tracking is tuned.
3. R0-07b: multipath (TGn B–F), Doppler, drone air-to-ground channel, SFO with a fractional
   delay (same oscillator as CFO, §17.3.9.6).
