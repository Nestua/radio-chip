# Gate journal: receiver front (packet detection, CFO, timing)

Block: the first receiver stage (R0-08): find an 802.11a/g PPDU in the sample stream, estimate
and remove its carrier frequency offset, and place the FFT window of the first long training
symbol. Model: `crates/radio-phy/src/rx_front.rs` (float reference in R0-08a, fixed-point model in
R0-08b). Downstream: channel estimation and equalisation (R0-09), SIGNAL decode, Viterbi (R0-10).

## 2026-11-17 · G0 · design, revised after independent review (PHY architecture decision)

**Founder decisions, 2026-11-18**: build both detectors and choose by measured data; targets
split by mode as below; our boards use a **TCXO ≤ ±2 ppm**.

An independent reviewer (fresh agent, given the draft and the standard, not our conclusions)
checked the first draft. Confirmed: the 9 dB sensitivity SNR (also at 10 and 5 MHz), the CFO
tolerances, the coarse range, the preamble facts, Doppler ≈ 0.6 kHz at 30 m/s. Found and fixed
below: (B1) θ ≈ 0.7 cannot reach 99 % at 4 dB with an acceptable false-alarm rate — the metric
|C|/P tends to SNR/(1+SNR) = 0.716 at 4 dB, while ≤ 1 false alarm/s needs θ ≈ 0.59 (Pfa per sample
= e^(−θ²L) on noise); (B2) the CFO variance formula held only for L ≤ D and high SNR; plus the
fine range per bandwidth, combining the STF and LTF estimates, latency budget, AGC/DC, timing
details, a stronger detector alternative, an air-to-ground channel, and stronger tests.

### Requirements
- **Standard (IEEE 802.11-2024 §17.3.10.6.1)**: a PPDU at minimum sensitivity must be detected
  with probability > 90 % within 4 µs at 20 MHz (8 µs at 10, 16 µs at 5 MHz). With the
  standard's assumptions (noise figure 10 dB, §17.3.10.2) that is **SNR 9 dB** at every
  bandwidth (−82 vs −91 dBm, −85 vs −94, −88 vs −97). The 4 µs budget (80 samples) includes
  correlator length, run length, filter and pipeline delay and AGC settling — all counted.
- **Ours — 802.11 mode**: meet the standard with margin; **Pd ≥ 90 % at 4 dB**, false alarms
  ≤ 1 per second on noise (5·10⁻⁸ per sample at 20 MHz).
- **Ours — TDMA long-range mode (ADR 0001)**: the receiver knows its slot, so detection is
  **gated to the expected slot window**; the false-alarm budget relaxes by orders of magnitude and
  **Pd ≥ 99 % at 4 dB** becomes realistic. A longer TDMA preamble is a protocol decision left to
  the MAC design (open question 1).
- **CFO range**: ±40 ppm between two stations (§17.3.9.5) at up to 5.885 GHz = **±235 kHz**.
  Coarse (STF, lag 16) is unambiguous to ±fs/32: 625 / 312 / 156 kHz at 20 / 10 / 5 MHz; fine
  (LTF, lag 64) to ±fs/128: 156 / 78 / 39 kHz. At 5 MHz a ±25 ppm crystal pair (±290 kHz) exceeds
  the coarse range, so **our boards need a TCXO** (≤ ±2 ppm proposed; the standard's ±10 ppm at
  5 MHz is the floor) — a hardware requirement for R3/R4, **decided: ≤ ±2 ppm**.
- **Timing**: the FFT window must start inside the cyclic prefix after the channel's delay
  spread. Sample-clock drift (40 ppm over a 5.5 ms maximum 6 Mb/s PSDU ≈ 4.4 samples) exceeds any
  fixed back-off, so R0-09 tracks it with the pilots and slips samples; TDMA frames are capped.

### Algorithms (textbook, none proprietary) — two detectors compared before committing
1. **Detector A — normalised delay-and-correlate on the STF** (Schmidl & Cox 1997):
   C(n) = Σ_{k<L} r(n+k)·r*(n+k+16), normalised by √(P₁P₂) (both windows' energies, so the
   metric ≤ 1); detect when |C|² ≥ θ²·P₁P₂ for N consecutive samples. Immune to CFO and channel.
   Weak at low SNR because of the noise×noise term.
2. **Detector B — sign-quantised 16-tap STF matched filter** (±1 ± j coefficients, adders only),
   magnitudes summed non-coherently over STF periods, normalised by energy. Loses only ~0.5 dB
   to a 232 kHz CFO at 20 MHz (more at 5 MHz — another reason for the TCXO) and avoids the
   noise×noise term, so it is expected to win at low SNR. Its product between periods also gives
   a coarse CFO.
   **R0-08a implements both and picks by measured Pd at the false-alarm budget** (and by
   robustness to DC, a CW tone and a gain step). Either may be followed by a confirmation stage
   (the LTF cross-correlation below) before the acquisition is declared.
3. **DC blocker** in front of the detector (a DC offset or CW spur correlates perfectly at lag 16
   and would trigger constantly).
4. **CFO — both estimates, combined**: coarse from lag 16 over the STF after detection, fine from
   lag 64 over GI2 + both LTF symbols (L = 96), the fine one resolved against the coarse one, the
   two combined by inverse variance. Accumulation stops before GI2 for the STF part.
5. **Timing — cross-correlation with the 64-sample long training symbol** on the coarse-corrected
   signal, searched in a window derived from the detection-plateau uncertainty (~96 samples); the
   symbol grid is set by the **maximum energy of |xcorr|² over a sliding window of GI − B
   samples** (captures the multipath cluster, not just the strongest tap), then the FFT window
   **backs off B into the cyclic prefix, B specified in ns** (≈ 200 ns: 4 / 2 / 1 samples at
   20 / 10 / 5 MHz). The linear phase this adds is absorbed by channel estimation (R0-09).
6. **Correction**: an NCO rotates every following sample by the combined CFO. Residual CFO and
   clock drift: pilots, R0-09.
7. **AGC interaction** (AGC itself is R1/R3): the CFO accumulators are frozen until gain has
   settled; the channel simulator gains DC offset, CW tone and gain-step impairments to test it.

### CFO estimator variance (the theory the G1 gate checks; SNR = Ps/σ², σ² complex noise)
var(Δf̂) = var(φ̂)/(2π·D/fs)², φ̂ the angle of a lag-D correlation over L samples:
- L ≤ D (no overlap): var(φ̂) = (1/(L·SNR))·(1 + 1/(2·SNR));
- L ≥ D (overlapping; both the STF and the LTF case — corrected 2026-11-18, see G1): var(φ̂) =
  (D·SNR + L/2)/(L²·SNR²);
both reduce at high SNR to 1/(L·SNR) and D/(L²·SNR). Gate: Monte-Carlo variance within ±20 % of
these, unbiased, at 6–25 dB; also not below the Cramér–Rao bound.

### Interfaces
- In: complex baseband samples at fs (20/10/5 MHz) from a 12-bit ADC (AD9361 in R3);
  R0-08a takes `(f64, f64)`, R0-08b fixes the formats.
- Out: `Acquisition { detected_at, ltf_start, fft_start, cfo_hz }` and the CFO-corrected samples.

### Fixed point (R0-08b, after the float reference passes G1)
Correlators, the product comparison, a CORDIC for ∠, an NCO, the sign-quantised filters. Budget:
Pd/Pfa unchanged within Monte-Carlo noise, CFO RMS within +10 % of float, timing identical in
≥ 99.9 % of frames.

### Gates
- **G1 (R0-08a)**, references we did not write:
  1. Ground truth: `build_ppdu` frames (G1 against Annex I) through the channel simulator (G1
     against theory) with known CFO, delay and first-path delay.
  2. Theory: the variance expressions above and the Cramér–Rao bound.
  3. An outside receiver as a black box (gr-ieee802-11 under ADR 0005, or OpenOFDM's Python
     receiver): Pd and CFO-RMS curves on the same impaired frames — ours must be no worse.
  4. End to end: front end + genie channel knowledge + R0-10 decode the frames the front end
     accepted.
  Required tests: noise only ≥ 6·10⁷ samples with zero false alarms (≤ 1/s at 95 % confidence);
  DC, CW-tone and gain-step cases; CFO at ±235 kHz, at the ±fs/32 edge and at the 5 MHz fine
  edge; drift over a maximum-length frame; known first-path delay; a truncated STF without LTF
  must not acquire; mutations flipping a conjugate or a lag sign.
  The standard's CCA figure is a requirement checked by these runs, not a reference.
- Channels: AWGN, TGn B/D/E (E's 730 ns is inside the 800 ns guard; only F exceeds it) and an
  air-to-ground Rician case (two-ray, K from R0-07d) for the drone link.

### Open questions
1. TDMA preamble: slot gating is assumed; a longer preamble (doubled STF) is a protocol choice
   for the MAC design.
2. ~~TCXO tolerance~~ — decided: ≤ ±2 ppm on our boards (R3/R4 hardware requirement).
3. First-path vs energy-window timing in TGn F (delay spread > guard): measure after R0-09.

## 2026-11-18 · G1 · R0-08a float reference: detector chosen by measurement
Model: `crates/radio-phy/src/rx_front.rs`. Gates: `tests/g1_rx_front.rs` (15 tests, ~9 s),
`tests/g1_rx_front_openofdm.rs` (outside receiver). Measurements: `radio-tools` bin `rxfront`
(`tail`, `pd`, `robust`, `frames`); full output in `docs/gates/reports/rx_front_detectors.txt`.
Command: `cargo test --release -p radio-phy --test g1_rx_front --test g1_rx_front_openofdm`
(rustc 1.98.1, base 2ad2f79). Test-first: the gate was written against the planned API and
failed to compile; the first run of the model then failed 9 of 14 tests (findings below).

### Decision (founder delegated "pick by data")
**Detector A — normalised lag-16 autocorrelation, L = 48, θ = 0.575, 4 consecutive samples.**
Pd within 4 µs, CFO uniform ±235 kHz, 4000 frames per point:

| Pd | AWGN 2 dB | AWGN 4 dB | TGn B 9 dB | TGn D 9 dB | TGn E 9 dB |
|---|---|---|---|---|---|
| A autocorrelation | 0.825 | **0.999** | **0.936** | **0.988** | **0.998** |
| B sign-quantised matched filter | 0.301 | 0.471 | 0.536 | 0.367 | 0.272 |
| B exact-tap matched filter | **0.966** | **0.999** | 0.948 | 0.886 | 0.720 |

(TGn is a static Rayleigh realisation per frame, so its Pd includes deep fades.) B with exact
taps wins only at ≤ 2 dB in AWGN; any frequency-selective channel distorts the STF it is
matched to. B sign-quantised loses 2.6 dB to its taps alone (below). False-alarm budget,
`rxfront tail` on 10⁹ noise samples: A at θ = 0.575, run 4 had **no event** (< 10⁻⁹ per sample;
budget 5·10⁻⁸ = 1/s); B (both) at θ = 0.35 ≈ 10⁻⁸. Robustness (`rxfront robust`, 10⁷ samples
per case): **no false acquisition by any detector** under DC to +30 dB, CW tones −10…+20 dB
and ±10…30 dB gain steps. A *detects* on CW ≥ −3 dB over the noise and on DC transients after
gain steps; the LTF stage rejects all of it. Packets at 10 dB SNR under CW: A acquires
≥ 98 % up to a spur +3 dB over the noise (7 dB under the signal), like exact-tap B; a spur as
strong as the signal defeats all three.

### Findings while building the gate (each fixed, the gate catches each)
1. **The fine CFO window overlaps** (L = 96 > D = 64), so the G0 text's "L ≤ D, the LTF case" was
   wrong: both estimators follow the overlapping form. Derivation (now the gate's theory): a
   noise sample entering both as n_m and n*_m contributes only to the real part; the angle
   variance is [σ²/2·Σ_m |α_m − β_m*|² + L·σ⁴/2]/|A|², which for constant envelope gives
   (D·SNR + L/2)/(L²·SNR²) for L ≥ D and (1/(L·SNR))(1 + 1/(2SNR)) for L ≤ D. Combination weight
   recomputed from the constants (below).
2. **Sign-quantised taps cost 2.6 dB**: ρ² = |⟨q, s⟩|²/(‖q‖²‖s‖²) = 0.553 for the STF period
   — not in the G0 review. Exact-tap B added to the comparison as the bound.
3. **Search window**: detection can fire as early as STF + 15 (noise-free); the LTF search now
   spans [d + 24, d + 264) and a maximum in its last 64 + (GI − B) positions is not trusted
   (the next detection retries) — GI2 is half an LTF symbol, so T1 − 64 matches partly and
   won when the true T1 lay beyond the window, which a tone triggering A early provoked.
4. **LTF correlation is coherent over T1 T2** (128 samples): the position 64 early keeps a
   quarter of the peak instead of half. With (3) this took packets under a +3 dB spur on the
   STF grid (1.25 MHz) from 0.72 to 1.00. A peak-to-average confirmation tried on the way
   proved redundant once (3) and (4) were in and was removed.
5. **DC blocker start-up**: m[−1] = x[0] made a ringing DC estimate that triggered A at stream
   start; it is now the mean of the first 2^shift samples.
6. **Fine-window guard**: our frames are not windowed; through the sinc interpolator a
   half-sample delay smears STF→GI2 and T2→SIGNAL, biasing the fine estimate by −300 Hz at
   20 dB. 4 samples are kept clear at each end (L = 88): −70 Hz left (σ/6 at 20 dB), std +9 %;
   8 samples would remove it at +20 % std. The AWGN gate bounds the bias at 5σ_mean + σ/4.

### Model as merged
- DC blocker μ = 2^−7; detector A as above; coarse CFO lag 16 over products [d − 40, d + 56);
  LTF timing by coherent 128-sample cross-correlation, energy over a GI − B window, latest start
  within 7/8 of the maximum; confirmation by normalised window energy ≥ 0.5 (≈ SNR/(1 + SNR) for
  a packet, (GI − B)/128 for noise); fine CFO lag 64 over GI2 + T1 vs T1 + T2 less the guard;
  combined Δf = coarse + w·residual, **w = 0.771** = v_c/(v_c + v_f) from v ∝ 1/(L²·D); back-off
  200 ns (4/2/1 samples); `Acquisition { detected_at, ltf_start, fft_start, cfo_hz, … }`.

### Evidence
1. **Theory, CFO** (`lag_correlation_cfo_is_unbiased_with_the_theoretical_variance`, 4000 trials
   per point): STF (L 96, D 16) and LTF (L 96, D 64) at 6/10/15/25 dB — std within 3 % of the
   first-order theory (e.g. STF 10 dB 2967 vs 2988 Hz, LTF 25 dB 233.5 vs 233.4 Hz), unbiased,
   above the known-waveform CRB (efficiency 0.6–0.9).
2. **Theory, false alarms**: with L = D the two windows are independent and
   P(|ρ̂| ≥ θ) = (1 − θ²)^(L−1): measured 0.2423/0.0334/0.00132 vs 0.2430/0.0336/0.00124.
3. **Noise only**: 6·10⁷ samples, zero detections (chosen detector).
4. **Pd**: 0.999 at 4 dB (target ≥ 0.90), 1.000 at 9 dB (standard: > 0.90).
5. **Ground truth** at 20/10/5 MHz, 10 and 20 dB, 1000 frames each, random start, fractional
   delay and CFO (±235 kHz; ±25 kHz at 5 MHz, the TCXO pair): FFT start within ±1.5 samples of
   first path − B every time; CFO RMS 5–11 % above the combined theory (1399 vs 1330 Hz at
   20 MHz 10 dB; 110 vs 100 Hz at 5 MHz 20 dB). CFO edges ±235 kHz, ±600 kHz (fs/32 = 625),
   5 MHz ±150 kHz and the fine edge ±39/−45 kHz: within 1 % of the subcarrier spacing.
   ±40 ppm SFO + CFO: timing and CFO right. Truncated STF without LTF: detected 200/200,
   acquired 0/200. Two-path (0.3 at 0, 0.7 at 6 samples): FFT start B before the first path.
   TGn B and D: ISI-free in 100 % of realisations; E: 0.5 % ISI-free (its tail exceeds
   GI − B), mean ISI power 1.9·10⁻³ (−27 dB).
6. **End to end**: front end + LS channel from the LTF + pilot phase + R0-10 Viterbi decode
   100/100 6 Mb/s 100-octet frames at 8 dB, CFO ±235 kHz.
7. **Outside receiver**: OpenOFDM's Python decoder (Apache-2.0, ea638eb, unmodified, black box,
   `tools/ref/rx_front_openofdm.py`; its input file wrapped to raise at EOF because
   `decode_next` spins forever on a frame it cannot find) on 800 frames (`rxfront frames 200`,
   FNV f1ee656c89b4b18c), noise over the frame only (its absolute power trigger), same int16
   samples into ours:

   | SNR | SIGNAL ours / OpenOFDM | PSDU ours / OpenOFDM | CFO RMS ours / OpenOFDM |
   |---|---|---|---|
   | 2 dB | 0.995 / 0.695 | 0.935 / 0.445 | 5257 / 16012 Hz |
   | 4 dB | 1.000 / 0.730 | 1.000 / 0.710 | 3337 / 10429 Hz |
   | 6 dB | 1.000 / 0.840 | 1.000 / 0.825 | 2555 / 7469 Hz |
   | 9 dB | 1.000 / 0.905 | 1.000 / 0.890 | 1876 / 4839 Hz |

   (Its fine CFO estimate is disabled in that commit; its timing is the two strongest LTF
   correlation peaks.) Required: ours no worse at every SNR — met with wide margin.

Mutation check (each restored with `cp` + `touch`; 13 mutations, all caught):
| Mutation | Fails |
|---|---|
| lag-correlation conjugate flipped | CFO theory, ground truth, edges, SFO, DC, CW, gain, e2e |
| CFO sign flipped | same 8 |
| detector lag 15 instead of 16 | 9 tests incl. truncated STF, multipath |
| coarse correction sign | ground truth, edges, SFO, DC, CW, gain, e2e |
| energy window keeps the earliest start | ground truth, edges, SFO, multipath, DC, CW, gain |
| combination weight 0.5 | ground truth (CFO RMS vs theory), SFO, gain |
| DC blocker frozen after start-up | DC (offset switched on mid-stream) |
| back-off ignored | ground truth, edges, SFO, multipath, DC, CW, gain |
| single-symbol LTF correlation | 8 tests |
| edge-peak rejection removed | CW (+3 dB spur on the STF grid) |
| LTF confirmation off | truncated STF, CW, gain |
| detector threshold 0.45 | noise only, DC, gain |
Two mutations (blocker frozen, edge rejection) first survived; the DC-mid-stream and the
randomised 1.25 MHz spur cases were added for them. A conjugate flip inside the detector's
own |C| is an equivalent mutation (|C| unchanged), so the lag was mutated instead.

### Open questions
4. **Interference above the noise**: a CW spur ≥ 0 dB over the noise keeps A firing (harmless
   for acquisitions, but in hardware the LTF stage then runs continuously) and biases both CFO
   estimators toward the spur (≈ −0.5 kHz at −10 dB INR, 10 dB SNR). Candidates: switch to
   exact-tap B while A is saturated, or a spur notch. Decide with R3 hardware spur data.
5. **AGC interface**: on a gain step the DC estimate should be rescaled by the same step (the
   AGC knows it) and detection blanked while gain settles; today the LTF stage absorbs the
   transient detections. Defined with the AGC (R1/R3).
6. **TDMA mode** (slot-gated, relaxed false-alarm budget): exact-tap B is 0.83 vs 0.23 at 0 dB
   AWGN; with a gated window either detector's threshold can drop. Re-measure in the MAC design.
7. **Fixed point (R0-08b)**: the 128-tap complex LTF correlator over a 240-position search is
   the largest block; sign- or 3-bit-quantised LTF taps and the combination weight as a shift
   sum (0.771 ≈ 0.75 + 1/64) go to G2.
