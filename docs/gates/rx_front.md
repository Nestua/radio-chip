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

### Decision (founder delegated "pick by data"; confirmed 2026-11-18: "берем А", with a follow-up
to tune it to the maximum — BACKLOG R0-08c)
**Detector A — normalised lag-16 autocorrelation, L = 48, θ = 0.575, 4 consecutive samples.**
Pd within 4 µs, CFO uniform ±235 kHz, 4000 frames per point:

| Pd | AWGN 2 dB | AWGN 4 dB | TGn B 9 dB | TGn D 9 dB | TGn E 9 dB |
|---|---|---|---|---|---|
| A autocorrelation | 0.825 | **0.999** | **0.936** | **0.988** | **0.998** |
| B sign-quantised matched filter | 0.301 | 0.471 | 0.536 | 0.367 | 0.272 |
| B exact-tap matched filter | **0.966** | **0.999** | 0.948 | 0.886 | 0.720 |

(TGn is a static Rayleigh realisation per frame, so its Pd includes deep fades.) B with exact
taps wins only at ≤ 2 dB in AWGN; any frequency-selective channel, and even a half-sample
delay (noise-free peak metric 0.49–0.56), distorts the STF it is matched to. B sign-quantised
has a ceiling, not just a loss: its taps cost 2.6 dB (below) and at a half-sample delay with
±235 kHz CFO its noise-free peak metric is 0.23–0.37 against the 0.35 threshold, so some frames
are never detected (Pd stops at 0.73 at 12 dB). False-alarm budget,
`rxfront tail` on 10⁹ noise samples: A at θ = 0.575, run 4 had **no event** (< 10⁻⁹ per sample;
budget 5·10⁻⁸ = 1/s); B (both) at θ = 0.35 ≈ 10⁻⁸. Robustness (`rxfront robust`, 10⁷ samples
per case): **no false acquisition by any detector** under DC to +30 dB, CW tones −10…+20 dB
and ±10…30 dB gain steps. A *detects* on CW ≥ −3 dB over the noise and on DC transients after
gain steps; the LTF stage rejects all of it. Packets at 10 dB SNR under CW, counted only with
the FFT start within ±2 samples *and* the CFO within 2 % of the subcarrier spacing: ≥ 97 % up
to a spur 3 dB under the noise; 74–88 % at 0 dB; 42–60 % at +3 dB (timing still exact: the
spur, 7 dB under the signal, pulls both lag correlations toward its own frequency); nothing at
+10 dB. The same for every detector — the CFO estimators are shared — so CW above the noise is
an estimator limit (open question 4), not a detector property.

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
4. **LTF correlation is coherent over T1 T2** (128 samples): 64 late (T2 + SIGNAL) now keeps
   0.23 of the peak instead of 0.5 (the T2 ambiguity), while 64 early keeps 0.59 (GI2 is half a
   symbol), a margin of only 2.3 dB that the edge-peak rule (3) covers. With (3) this took packets under a +3 dB spur on the
   STF grid (1.25 MHz) from 0.72 to 1.00. A peak-to-average confirmation tried on the way
   proved redundant once (3) and (4) were in and was removed.
5. **DC blocker start-up**: m[−1] = x[0] made a ringing DC estimate that triggered A at stream
   start; it is now the mean of the first 2^shift samples.
6. **Fine-window guard**: our frames are not windowed; through the sinc interpolator a
   half-sample delay smears STF→GI2 and T2→SIGNAL, biasing the fine estimate by −300 Hz at
   20 dB. 4 samples are kept clear at each end (L = 88): −70 Hz left (σ/6 at 20 dB), std +9 %;
   8 samples would remove it at +20 % std. The AWGN gate bounds the bias at 5σ_mean + σ/4.

7. **Coarse CFO re-anchored on the timing** (independent review, below): anchored on the
   detection, the coarse window began before the STF at ≥ 10 dB (A fires at STF + 26…34) and ran
   into GI2 at 0–2 dB; under a spur that keeps A firing it sat on the spur (coarse error RMS
   46 kHz, 23 % of it leaking into the result). Now a provisional detection-anchored estimate
   serves only the LTF search, and the final coarse estimate uses products [T1 − 160, T1 − 64)
   of the STF located from the timing. CFO RMS against the combined theory went from +5…11 %
   to −0.4…+2 %.

### Independent review (risk point), 2026-11-18
A fresh reviewer, given the code and the standard but not our conclusions, confirmed the sign
conventions (including OpenOFDM's), the combination weight and the independence of the two
estimates, the index arithmetic of every window, the statistical basis of the threshold (0 in
6·10⁷ ⇒ ≤ 5·10⁻⁸ at 95 %) and licence hygiene; it agreed the detector choice follows from the
data. Its findings and what was done:
1. Coarse window mis-anchored under a spur, CW claims checked timing only → finding 7, CFO now
   part of the robustness criterion and the claims above restated.
2. Coarse window not inside the STF → finding 7.
3. Timing is a block search with retries over overlapping data (decision needs samples to
   d + 404) → scoped into R0-08b: restate as a streaming rule first (open question 8).
4. DC-blocker start-up averages *future* samples → scoped into R0-08b (open question 9).
5. "64 early keeps a quarter" was the single-symbol figure → finding 4 corrected.
6. Sign-tap B has a ceiling, not only a 2.6 dB loss → decision text corrected.
7. OpenOFDM comparison: our SIGNAL ignored parity; its CFO is coarse-only → parity now required
   on our side, the CFO row labelled; nits: the false-alarm law is checked at L = D = 16 where it
   is exact — the deployed L = 48 windows overlap, so its threshold rests on measurement alone.

### Model as merged
- DC blocker μ = 2^−7; detector A as above; provisional coarse CFO lag 16 over products
  [d − 40, d + 56) for the LTF search only;
  LTF timing by coherent 128-sample cross-correlation, energy over a GI − B window, latest start
  within 7/8 of the maximum; confirmation by normalised window energy ≥ 0.5 (≈ SNR/(1 + SNR) for
  a packet, (GI − B)/128 for noise); fine CFO lag 64 over GI2 + T1 vs T1 + T2 less the guard;
  final coarse CFO lag 16 over STF products [T1 − 160, T1 − 64);
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
   first path − B every time; CFO RMS within −0.4…+2.2 % of the combined theory (gate ±8 %;
   1339 vs 1330 Hz at 20 MHz 10 dB, 408 vs 399 at 20 dB, 99.4 vs 99.8 Hz at 5 MHz 20 dB), with
   the combination weight derived independently in the test. CFO edges ±235 kHz, ±600 kHz (fs/32 = 625),
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
   | 2 dB | 0.995 / 0.695 | 0.935 / 0.445 | 4424 / 16012 Hz |
   | 4 dB | 1.000 / 0.730 | 1.000 / 0.710 | 3090 / 10429 Hz |
   | 6 dB | 1.000 / 0.840 | 1.000 / 0.825 | 2420 / 7469 Hz |
   | 9 dB | 1.000 / 0.905 | 1.000 / 0.890 | 1613 / 4839 Hz |

   SIGNAL requires good parity on both sides. Its CFO is coarse-only (the fine estimate is
   disabled in that commit) and each RMS is over the frames that receiver located, so the CFO
   row is a low bar; its timing is the two strongest LTF correlation peaks. Required: ours no
   worse at every SNR — met with wide margin.

Mutation check (each restored with `cp` + `touch`; 14 mutations, all caught):
| Mutation | Fails |
|---|---|
| lag-correlation conjugate flipped | CFO theory, ground truth, edges, SFO, DC, CW, gain, e2e |
| CFO sign flipped | same 8 |
| detector lag 15 instead of 16 | 9 tests incl. truncated STF, multipath |
| coarse correction sign | ground truth, edges, SFO, DC, CW, gain, e2e |
| energy window keeps the earliest start | ground truth, edges, SFO, multipath, DC, CW, gain |
| combination weight 0.5 | ground truth (weight and CFO RMS vs independently derived theory) |
| final coarse anchored on the detection again | ground truth (CFO RMS vs theory, ±8 %) |
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
   estimators toward the spur (≈ −0.5 kHz at −10 dB INR; 7 kHz RMS at +3 dB, 10 dB SNR). Candidates: switch to
   exact-tap B while A is saturated, or a spur notch. Decide with R3 hardware spur data.
5. **AGC interface** (+ R0-08b G2: idle noise must stay ≥ ≈ 1 LSB rms, −65 dBFS, or the ADC
   makes it sparse and the detector false-alarms): on a gain step the DC estimate should be rescaled by the same step (the
   AGC knows it) and detection blanked while gain settles; today the LTF stage absorbs the
   transient detections. Defined with the AGC (R1/R3). Measured on 2026-11-19 (the entry
   "the DC blocker at a gain change" below): a rescale or a restart matters only for large offsets.
6. **TDMA mode** (slot-gated, relaxed false-alarm budget): exact-tap B is 0.83 vs 0.23 at 0 dB
   AWGN; with a gated window either detector's threshold can drop. Re-measure in the MAC design.
7. **Fixed point (R0-08b)**: the 128-tap complex LTF correlator over a 240-position search is
   the largest block; sign- or 3-bit-quantised LTF taps and the combination weight as a shift
   sum (0.771 ≈ 0.75 + 1/64) go to G2.
8. ~~**Streaming timing**~~ — done in R0-08b part 1 (below). Was: restate the block search with retries as
   a causal rule (running maximum of the energy metric, accepted once 64 + (GI − B) positions
   pass without a larger one) and size the sample buffer from it; the final coarse estimate
   also needs the STF products kept until the timing is known (≈ 250 samples of buffering).
9. ~~**Causal DC start-up**~~ — done in R0-08b part 1 (gear-shifted μ). Was: the model's m[−1] averages the first 2^shift samples ahead;
   the RTL needs a causal equivalent (delayed enable or a gear-shifted μ) and the model must
   match it. Packets starting 10–130 samples into the stream still acquire 100/100 at 20 dB.

## 2026-11-18 · G1 · R0-08b part 1: streaming receiver, causal DC blocker (open questions 8, 9)
Model: `RxFront` and `DcBlocker` in `crates/radio-phy/src/rx_front.rs`; `acquire`/`acquisitions`
run it sample by sample. Command: `cargo test --release -p radio-phy --test g1_rx_front
--test g1_rx_front_openofdm` (rustc 1.98.1, base 3e65564); measurements `rxfront pd 4000`,
`rxfront robust 10` (report updated; the R0-08a block-model figures are in its git history).
Test-first: the new gates were written against the planned API; on a shim around the block
model `dc_blocker_is_causal` and `streaming_decisions_are_causal_and_bounded` failed
(future-sample start-up; decisions up to d + 404); the spur gate (below) failed on the first
streaming draft (261/400).

### What changed (fixed point and RTL follow this form)
1. **DC blocker, causal (OQ 9)**: m[−1] = 0 and a gear-shifted pole μ_n = 2^(−min(7, ⌊log₂(n+1)⌋)):
   a power-of-two running mean that is the steady-state 2^−7 pole from n = 127 (an 8-bit
   saturating counter and a barrel shifter). A stream opening on DC loses it at n = 0 (μ₀ = 1).
2. **Streaming timing (OQ 8)**, one sample in, at most one acquisition out, no sample processed
   twice, one search at a time:
   - The detector (same arithmetic, now a streaming state also behind `detect`) opens an LTF
     search at its first event (positions from d + 24); every later event moves the window end
     to event + 264; with no event for that long the search closes (power: the RTL gates the
     128-tap correlator while idle).
   - **Provisional CFO latched at every event** from the lag-16 sum over the 96 products whose
     later sample is ≤ n − 24 (`LATCH_LAG`: the last event on a packet falls ≈ 10–25 samples
     into GI2), driving an NCO on the stream the correlator sees. It replaces the old window
     that reached 56 samples *after* the detection, and the retries: under a spur that keeps A
     firing, the latch follows the STF once it arrives.
   - Correlator |Σ z(i+k)·[L L]*(k)|² for i = n − 127, energy over GI − B positions, **running
     maximum accepted once 64 + (GI − B) later positions inside the window are no larger** (the
     old edge rule, stated causally); latest start ≥ 7/8 of the maximum. A candidate that fails
     confirmation is dropped and the running maximum restarts at the next position.
   - **Confirmation, two rules, on the history corrected by the final coarse CFO**: the
     non-coherent energy Σ|xc(T1)|² + |xc(T2)|² (64-sample correlations) normalised ≥ 0.5,
     tolerant of a CFO error (sinc²(64δ/fs) instead of sinc²(128δ/fs)); and the coherent
     |xc(T1) + xc(T2)|² normalised ≥ 0.25 (`LTF_COHERENCE`; noise ≈ 0.09): the LTF repeats.
   - Fine CFO from the lag-64 sum on the DC-blocked stream (independent of the NCO), resolved
     against the final coarse estimate (±fs/128); final coarse as before, [T1 − 160, T1 − 64).
     All read a 512-sample history: oldest sample T1 − 160, decision by T1 + 64 + 2(GI − B) +
     127, i.e. ≤ 381 samples back.

### Independent review (PHY architecture, risk point), 2026-11-18
A fresh reviewer, given the diff, the old model and the standard but not our conclusions,
verified every ring read against its capacity, the latency bound at 20/10/5 MHz, the index
arithmetic against the block model, the gear shift and no livelock under spurs (400/400 at
+3…+10 dB). No must-fix. Findings and what was done:
1. **Spur at the noise level at the 4 dB target: 256/400 placed vs the block model's 319/400**
   (its retries gave several provisional estimates a chance). Diagnosis: the true T1 was found
   but failed the coherent confirmation — a spur at the noise level biases even the final coarse
   estimate by ≈ 50–60 kHz (open question 4), costing the 128-sample coherent correlation
   ≈ 2 dB at a metric already ≈ SNR/(1+SNR+INR). Latch ending before GI2 + confirmation on the
   final coarse: 294/400; **non-coherent confirmation: 390/400** (block model on the same gate
   332/400). Gate `spur_at_the_noise_level_at_the_4_db_target` (≥ 360 = the 0.90 target).
   The non-coherent rule alone then confirmed one DC transient after a −30 dB AGC step in
   `rxfront robust` (normalised 0.62: a small correlation at every window position, coherent
   0.007), hence the coherence rule, and that case is now gated (`gain_step_robustness`).
2. The drop-and-restart comment claimed no T1 can be lost: wrong — a weaker true T1 within
   63 + (GI − B) positions after a stronger failing candidate (a strong burst ending just before
   a packet) is lost; the block model retried. Accepted and documented; not observed.
3. Gate gaps: bound and causality now also at 10 and 5 MHz; the "+ (GI − B)" in the acceptance
   rule survives mutation (kept: it is the old rule, and the latest start can move the grid
   GI − B past the maximum); the DC gear schedule one sample early survives (G3 vectors pin it).
4. `acquire(x, from)` filters on the opening detection: documented. Nits: the history guard is
   kept (the HISTORY = 256 mutation shows it bites); latches during T1T2 happen only with a spur
   stronger than the signal (none observed below it) — noted for open question 4.
5. Hardware realism, for the fixed-point step: the model re-sums 96 products per latch and
   96 + 88 + 2·w·64 at the decision; the RTL needs running accumulators (lag 16 delayed, lag 64)
   or a stated multi-cycle decision; the float `rem_euclid` phase becomes a wrapping integer
   phase with a LUT/CORDIC; the recompute-every-4096 exists only for float drift.

### Evidence
1. All R0-08a gates and the OpenOFDM comparison pass; printed figures equal the block model's
   to within 0.1 Hz of CFO RMS (e.g. 1338.8 vs 1339, 408.0, 99.4 Hz; OpenOFDM set unchanged:
   ours SIGNAL 0.995/1/1/1, PSDU 0.935/1/1/1 at 2/4/6/9 dB). +3 dB spur on the STF grid 200/200
   placed, CFO RMS 7138 Hz (as before). End to end 100/100.
2. New gates (6): causal DC prefix property; stream opening on DC 10 dB over noise: no
   acquisition, residual DC < 0.05 over samples 128..384; packets 10–130 samples into the stream
   50/50 each exact; decisions unchanged when the stream is cut after `decided_at` or its tail
   replaced, `decided_at` ≤ T1 + 64 + 2(GI − B) + 127 (500 frames, 4–25 dB, 20/10/5 MHz); a
   search without a packet closes in bounded time and stays open before; 20 back-to-back frames;
   the spur at the noise level at 4 dB (390/400).
3. `rxfront pd 4000`, detector A, acquisition rate streaming vs block: AWGN 0 dB 0.559 vs 0.513,
   2 dB 0.992 vs 0.990, ≥ 4 dB 1.000 both; TGn B/D/E higher or equal at every SNR (B 2 dB 0.586
   vs 0.570, D 4 dB 0.876 vs 0.865, E 0 dB 0.464 vs 0.434). Pd within ±0.0008 (DC start-up).
   Exact-tap B also gains (AWGN 0 dB 0.938 vs 0.918).
4. `rxfront robust 10`: **no false acquisition** in any case or detector; acquisition rates
   under DC/CW/gain steps within ±0.006 of the block model.

Mutation check on the final model (each restored with `cp` + `touch`):
| Mutation | Fails |
|---|---|
| accept the running maximum at once | 14 tests |
| latch the provisional CFO only at the first event | 12 tests |
| NCO sign flipped | 11 tests |
| window end not moved by later events | search-closes, spur 4 dB, CW |
| search never closes (both close rules) | search-closes |
| no gear shift (μ = 2^−7 from m = 0) | start-up, DC |
| DC estimate frozen after start-up | start-up, DC |
| fine not resolved against coarse | edges, ground truth, SFO, e2e |
| earliest start instead of latest | 10 tests |
| HISTORY 256 instead of 512 | 11 tests |
| lag-correlation conjugate flipped | 11 tests |
| detector lag 15 | 15 tests |
| weight 0.5 | ground truth |
| back-off ignored | 10 tests |
| single-symbol search template | 12 tests |
| detector threshold 0.45 | noise only, DC |
| final coarse anchored on the opening detection | 8 tests |
| confirmation without the coarse correction | 8 tests |
| coherent-only confirmation | spur 4 dB |
| non-coherent rule off | truncated STF, gain step, CW, spur 4 dB |
| coherence rule off | gain step (±30 dB) |
Survivors: `LATCH_LAG` 0 (measured 382 vs 390/400 on the spur gate, ~1σ: a real but small
gain below the gate's resolution); acceptance without the "+ (GI − B)" (above); restarting the
running maximum over the stored positions instead of dropping them (simplest kept, item 2).

### Open questions
10. **Fixed point (R0-08b part 2)**: running accumulators instead of re-sums at latch/decision
    (review item 5), integer NCO phase, CORDIC for both angles, the 128-tap search correlator
    (open question 7), the non-coherent/coherent confirmation (2·w 64-tap correlations at the
    decision, multi-cycle), weight 0.771 as a shift sum; G2 budget as in the G0 entry.

## 2026-11-18 · G0 · R0-08b part 2: fixed-point datapath and G2 method
The fixed-point model `RxFrontFixed` (`crates/radio-phy/src/rx_front_fixed.rs`) repeats
`RxFront` step by step in integer arithmetic; every width below is a parameter of
`FixedRxConfig`, chosen by the G2 sweep and then frozen as the default.

### Datapath (all integer; rounding stated per step)
- **In**: 12-bit two's-complement I/Q from the ADC (AD9361, R3), `(i16, i16)`. The ADC is common
  to both models: the float reference runs on the same integers as `f64`, so G2 measures the
  receiver's arithmetic, not the ADC.
- **DC blocker**: accumulator M = m·2^F (F fractional bits); error e = x·2^F − M (exact);
  M += e >> k (arithmetic shift, floor), k the gear shift; y = e·2^−F rounded (G2: half to
  even, not half up — finding 1 below), 13 bits.
- **Detector A**: y reduced to D bits (round, saturate); C, P₁, P₂ exact running integer sums
  (no recompute: integers do not drift); fire when den·|C|² ≥ num·P₁·P₂ with num/den = 85/256
  ≥ 0.575² (θ_eff = 0.5762, so the false-alarm rate cannot rise from the threshold itself).
  (G2 changed this: D = 13, exact comparison, num/den = 21668/2^16 — see the G2 entry.)
- **Angles**: CORDIC vectoring (N iterations, angles in units of 2π/2^A, input block-normalised
  to W_c bits by a leading-zero shift). Used for the provisional latch, final coarse and fine.
- **NCO**: P-bit phase accumulator; lag 16 makes the step the latched angle shifted right by 4;
  e^(jφ) from a quarter-wave LUT (L address bits, Q-bit amplitude); z = y·e^(jφ) rounded to Z
  bits.
- **Search correlator**: 128 taps [L L]* quantised to T bits (scaled to the LTF's peak);
  output c rounded to K bits before |c|²; energy sum, running maximum, latest-start rule as in
  the float model (the 7/8 factor as a shift: e ≥ e_max − e_max/8).
- **Confirmation**: the history (y) rotated by the final coarse angle through the same NCO/LUT
  design; 2·w correlations of 64 taps; both rules as integer comparisons with the quantised
  template's own energy in the normalisation.
- **CFO**: in phase units per sample, 2π/2^P. ω_c = angle_c·2^(P−A)/16, ω_f = angle_f·2^(P−A)/64
  resolved modulo 2^P/64 into ±2^P/128 around ω_c; combined ω_c + d·(1/2 + 1/4 + 1/64)
  (0.765625 for 0.771: the variance penalty of the weight error is < 0.1 %; G2: one rounded
  product (49·d + 32) >> 6, since three floored shifts bias by −1.5 LSB).

### G2 budget (from the G0 entry) and method
Pd within Monte-Carlo noise of the float model; false alarms: zero in 6·10⁷ noise samples at
several input levels, plus a 10⁹-sample tail for the chosen widths; CFO RMS ≤ +10 % of the
float model's; timing (fft_start) identical in ≥ 99.9 % of frames both acquire. Frames: the
R0-08a scenes scaled so the packet's RMS sits at −15 dBFS (AGC settled; PAPR headroom ≈ 15 dB),
AWGN 4/10/25 dB and TGn D 9 dB, CFO ±235 kHz, fractional delay. Each width is swept with the
others generous, the smallest meeting the budget taken, then the joint set verified by the gate
`tests/g2_rx_front.rs`. The sweep goes into `docs/gates/reports/rx_front_g2.txt`.

## 2026-11-18 · G2 · R0-08b part 2: fixed-point receiver front
Model: `RxFrontFixed` in `crates/radio-phy/src/rx_front_fixed.rs` (control flow of `RxFront`,
integer arithmetic). Gate: `tests/g2_rx_front.rs` (7 tests, ~6 s). Sweep: `rxfront g2 300`,
`g2joint 3000`, `tailfx 200` → `docs/gates/reports/rx_front_g2.txt`. Command:
`cargo test --release -p radio-phy --test g2_rx_front -- --nocapture` (rustc 1.98.1, base
8e4a71c). Test-first: the gate was written against the planned API (failed to compile); the
onset/high-SNR test was written for the review's finding and failed (743 vs 0 events) before
the fix.

### Chosen formats (`FixedRxConfig::default()`)
| Stage | Format | Smallest passing alone (sweep) |
|---|---|---|
| ADC in | 12 bits | — |
| DC blocker | accumulator F = 10 fractional bits; output y 13 bits, convergent rounding | F ≥ 8 (below: dead zone ⇒ residual DC ⇒ 10⁵–10⁶ false alarms) |
| Detector A | full 13-bit y (D = 13), exact sums, exact comparison 2^16·\|C\|² ≥ 21668·P₁P₂ (θ_eff 0.575002) | D 8–12 false-alarm; D 6–7 pass only because the five sampled levels miss their sparse level (finding 2) |
| CORDIC | N = 16 micro-rotations, input normalised to W_c − 3 magnitude bits so x/y fit W_c = 18-bit registers (modelled as wrapping), angles A = 16 bits | N ≥ 10, W_c ≥ 14, A ≥ 12 |
| NCO | phase P = 22 bits (CFO step 4.8 Hz at 20 MHz), quarter-wave table L = 9 address bits, Q = 11-bit amplitude | P 22 is the floor tried, L ≥ 7, Q ≥ 6 |
| Search correlator | z Z = 12 bits, taps T = 8 bits, output K = 20 bits before \|c\|² | Z ≥ 10, T ≥ 6, K ≥ 15 |
| Coarse/fine CFO | E = 12-bit inputs to the lag-16/lag-64 sums | E ≥ 9 (+6 % at 25 dB); 11: +1.6 %, 12: +0.7 % |
| Combination | (49·d + 32) >> 6 = 0.765625·d, rounded | — |
The joint set sits above the one-at-a-time minima on the angle path (A/N/W_c) and the
correlator path (Z/Q/K): every perturbation flips the same rare near-ties of the energy window,
so their effects add. On 12 000 paired frames (`g2joint`): the chosen set disagrees on the FFT
start in 5 (0.042 %; budget 0.1 %; Poisson 95 % upper bound ≈ 10.5, 0.088 %), all ±1 sample,
in AWGN 25 dB and TGn D; a lower joint set (A14 N14 W_c15 Z10 Q9 K17) in 12 (0.100 %, fails);
a generous set (all widths +2…+6) in 0 — precision, not an irreducible floor.

### Findings
1. **Rounding must be unbiased.** Narrowing with half-up (or floor) adds up to ½ LSB of DC; on a
   quiet input that DC correlates perfectly at lag 16. All narrowing is convergent (half to
   even, R0-11's `requantize`).
2. **The detector must see the full 13 bits.** Narrowing to D < 13 makes the idle noise sparse
   at some input level — D = 8 at −45 dBFS (1562 false alarms in 3·10⁶), D = 10 at −55, D = 12
   at −65 — and the normalised correlation of sparse sequences has a heavy tail.
3. **AGC floor requirement (R1/R3)**: below ≈ 1 LSB rms (−75 dBFS, σ ≈ 0.36 LSB) the ADC itself
   makes the noise sparse and *both* models false-alarm (float 1205, fixed 919 in 3·10⁶). The
   AGC must keep idle noise at or above ≈ −65 dBFS. Added to open question 5.
4. The DC accumulator needs F > 7 (the steady-state shift): with F ≤ 7 the floored e >> 7
   freezes errors below 2^(7−F) LSB.

### Pre-PR review (fresh agent, code only), 2026-11-18
Confirmed control-flow parity line by line, the integer identities (7/8 keep, both
confirmation rules as 2x ≥ E_L·rx), signs, wraps, no runtime float on the data path. Findings:
1. **Must-fix — detector normalisation**: a common shift sized on max(P₁, P₂) (block floating
   point, 10 bits) zeroed the smaller sum at a power onset and fired unconditionally: 743
   events vs 0 (float) within 64 samples of 200 −65 → −15 dBFS onsets; `detected_at` off in all
   frames at 40/50 dB. Fixed: exact comparison in i128 (|C|, P₁, P₂ < 2^32). An RTL may
   normalise each term separately only with the model. Gate: onsets 0 vs 0; same detection as
   float in 197/200 (40 dB) and 199/200 (50 dB).
   On the way: with θ² = 85/256 (θ_eff 0.5762) the detection came one sample late in 15 % of
   40 dB frames (steep ramp); now 21668/2^16 (θ_eff 0.575002).
2. CORDIC x/y need headroom for √2 × gain 1.65: normalised to W_c − 3 bits (was W_c − 2,
   which overflows an 18-bit register at full-scale diagonals); the register is modelled
   wrapping, so the old margin now fails the CORDIC test (mutation).
3. The gate netted float-only against fixed-only frames; now per-frame disagreements (both
   ways) ≤ 1 % + 3σ, in the gate and in `rxfront g2`.
4. Docs said half-up; the first joint-set and float spur numbers were not in the report — now
   `g2joint` runs the lower set and the gate measures float and fixed on the same spur frames.
5. `RxConfig::detector/threshold` are ignored by the fixed model (documented);
   `FixedRxConfig::is_valid` guards widths (the receiver does nothing on an invalid set).
   Nits taken: CORDIC bound called "derived (loose)", tables computed once per receiver,
   combination rounded; NCO top-L-bit truncation (−π/2^L constant phase) left as is.

### Evidence (budget from the G0 entries)
1. **Paired frames** (gate, 1500 frames: AWGN 4/10/25 dB, TGn D 9 dB, −15 dBFS, CFO ±235 kHz):
   Pd 398/400/400/295 both models, 0 frames disagreeing on detection or acquisition; FFT start
   identical 1499/1499; CFO RMS fixed vs float 2936/2923, 1349/1346, 228/228, 1819/1813 Hz
   (≤ +0.5 %; budget +10 %). `g2joint` 12 000 frames: 0 detection/acquisition disagreements,
   CFO −0.1…+0.3 %, FFT start 5 disagreements.
2. **False alarms**: gate 6·10⁷ noise samples at −15/−30/−45/−55/−65 dBFS: 0; `tailfx 200`:
   10⁹ samples over the same five levels: 0 (budget 5·10⁻⁸ per sample).
3. **Robustness through the ADC** (gate): no acquisition under DC 10 dB, CW +3 dB, ±30 dB AGC
   steps (the `rxfront robust` seeds); spur at the noise level, 4 dB: fixed 390/400, float
   390/400 on the same frames; stream opening on DC: residual DC at the noise level by sample
   128, no acquisition; power onsets and 40/50 dB detections as float (review item 1).
4. **Primitives against references we did not write**: CORDIC vs `f64::atan2` on 20 000 inputs
   (magnitudes 1…2⁴⁰, all quadrants, axes): worst 3.8·10⁻⁴ rad, derived (loose) bound
   2.4·10⁻³; NCO table vs `f64::sin_cos` over 2¹⁶ phases: worst 1.27·10⁻² (bound 1.30·10⁻²).

Mutation check (each restored with `cp` + `touch`):
| Mutation | Fails |
|---|---|
| NCO step sign | frames, robustness, onsets/high SNR |
| coarse CFO sign | frames |
| CORDIC rotation direction | CORDIC, frames, robustness |
| CORDIC normalised to W_c − 2 (old margin) | CORDIC, frames |
| NCO quadrant fold | NCO, frames, robustness |
| common normalising shift in the detector (pre-review) | onsets/high SNR |
| threshold 85/256 | onsets/high SNR |
| threshold 70/256 | false alarms |
| DC accumulator F = 6 | false alarms |
| detector input narrowed to 10 bits | false alarms |
| non-coherent rule at 1.0 | frames, robustness |
| coherence rule off | robustness (±30 dB steps) |
| weight 1/2 | frames (CFO RMS) |
| fine not resolved against coarse | frames |
| sign-only taps | frames, robustness |
| confirmation without the coarse rotation | frames, robustness |
| earliest start instead of latest | frames, robustness |
| keep 1/2 instead of 7/8 | frames, robustness |
| no gear shift in the DC blocker | DC start-up (added for it; first survived) |
| latch lag 0 | frames |
Survivor: weight 48/64 instead of 49/64 — equivalent within the budget by design (+0.3 % var).
(The rows above the review were run before its fixes; the review's rows and the weight rows
after them, on the final model.)

### Open questions
11. **G3 (RTL)**: `hw/rtl/rx_front.sv` against vectors from `rvec` (the fixed model's
    acquisitions and, for the arithmetic blocks, the CORDIC/NCO/correlator outputs). The
    decision-time sums (96 + 88 products, 2·w 64-tap correlations, rotation of 128 + w history
    samples) are multi-cycle in hardware or running accumulators with delay lines — integer
    sums are exact, so both are bit-exact with the model. The detector's exact comparison
    needs 33-bit squares/products (or per-term normalisation, modelled first).
12. The 128-tap complex correlator (12 × 8-bit products) is the largest block; 4-bit taps cost
    6/1200 timing disagreements alone — revisit with an area estimate.

## 2026-11-18 · G3 · R0-08b part 3a: LTF search correlator RTL (open question 12)

The receiver front's RTL is split into blocks, one PR each, all bit-exact with `RxFrontFixed`:
(a) the LTF search correlator (this entry); (b) CORDIC and NCO; (c) DC blocker, detector A
and run counter; (d) the top level: provisional latch, |c|², energy window, search control and
the acquisition decision. The model now exposes the correlator as `ltf_corr` (used by
`RxFrontFixed::push`) and a per-sample trace (`push_traced`: y, z, correlator output) for the
vectors of later blocks.

### Design (`hw/rtl/ltf_corr.sv`)
- One sample per `in_valid`, never stalls; output 5 clocks after the clock edge that takes the
  sample, from the 128th sample after reset on. Window = 128 × 2 × 12-bit flops (all 128 taps
  are read every sample, so a memory would not save the storage).
- Two exact folds (open question 12): T1 = T2 gives u(k) = z(k) + z(k+64); the LTF is real in
  frequency, so the quantised taps are Hermitian, t(64−k) = conj(t(k)), t(0) and t(32) real
  (unit test `ltf_taps_are_hermitian` for T = 2…16; `rvec` refuses non-Hermitian taps). Pairs
  (k, 64−k) then need 4 real products instead of 8: **128 constant 14 × 8-bit multiplications
  instead of 512 general 12 × 8** (plus 128 + 124 adders before them), all exact integer sums.
- Narrowing to K = 20 by `hw/rtl/round_sat.sv` (half to even, saturating), the generic form of
  the model's `narrow`, reused by later blocks.
- Taps come from a generated package, `hw/rtl/gen/rx_front_rom_pkg.sv` (`rvec rx-front-rom`,
  `make -C hw vectors`); `scripts/check.sh` now diffs `hw/rtl/gen` as it does `hw/vectors`.
  Later blocks add the CORDIC and NCO tables to it.

### Evidence
Command: `make -C hw ltf_corr` (Verilator 5.054, `-Wall` clean via `make -C hw lint`).
Vectors `hw/vectors/ltf_corr.txt` (`rvec ltf_corr`): four received frames through
`RxFrontFixed` (its own z and correlator trace: AWGN 25 and 4 dB, TGn D 9 dB, a −3 dBFS frame),
2000 uniformly random full-scale samples, and extremes (z matched to the tap signs at full scale,
its negation, all −2048, alternating corners). The testbench inserts idle cycles (after every
5th and 13th sample) so the window must advance on `in_valid` only.
Result: 9 segments, 7618 samples, 6475 outputs, **0 mismatches**. Stub RTL first: 6475/6475
mismatches.

Mutation check (each restored with `cp` + `touch`):
| Mutation | Mismatches |
|---|---|
| Hermitian fold: sign of the b·(ur_m − ur_k) term | 6371 |
| T1/T2 fold index off by one | 6425 |
| tap index off by one | 6475 |
| tap re/im swapped | 6475 |
| rounding ties up (half-up) | 25 |
| rounding ties down | 34 |
| out_valid one sample early | 6467 |
| window shifts every clock instead of on in_valid | 6458 |

`round_sat` has its own gate (pre-PR review): `make -C hw round_sat` drives every 12-bit input
through eight parameter sets (`hw/tb/round_sat_bank.sv`: SHIFT 1/4/8/11, OUT_W at its bound
IN_W − SHIFT + 1 and below, saturating and not) against `requantize(HalfEven, Saturate)`
(`hw/vectors/round_sat.txt`, `rvec round_sat`): 4096 inputs, 32 768 checks, 0 mismatches.
Mutations: no positive / no negative saturation 3714 mismatches each, ties up 1833, ties down
1829, sticky missing bit 0 1829; an out-of-range OUT_W stops elaboration (`$error`). In
`ltf_corr` itself the saturation branch is unreachable (|c| < 2^26, narrowed < 2^18).

### Open questions
13. **Acquisition decision in RTL (PHY architecture, risk point — independent review and
    founder decision before part d).** `decide()` re-reads the history at the decision sample:
    final coarse sum (96 products), rotation of 128 + w samples, 2·w 64-tap correlations and the
    fine sum (88) — about 1900 complex MACs — and its result feeds back at once (search closed
    or restarted, run counter reset). Either the hardware does it within one sample (≈ 240
    MACs per clock at 8 clocks/sample), or the model gains an explicit decision latency
    (search frozen for L samples, result applied at n + L) and G1/G2 are rerun. The latter is
    the likely choice; measure the Pd/timing effect first.
14. The provisional latch at an event is C(n − 24) + C(n − 72) of the detector's own lag-16
    sum (windows [n−87, n−40] and [n−135, n−88]): a 72-deep delay line of C instead of a second
    accumulator. The CORDIC latency then delays the NCO path: run NCO/correlator/search a fixed
    L samples behind the detector (bit-exact, a delay line on y), part b/d.

## 2026-11-18 · G1/G2 · R0-08b part 3b: explicit decision latency (open question 13)

**Founder decision (2026-11-18):** the model gains an explicit decision latency, fixed once its
effect is measured. **Risk point (PHY architecture):** independent review done (below); founder
approval before merge.

### Change
- `RxConfig::decision_latency` (default `DECISION_LATENCY` = 64, clamped to
  `MAX_DECISION_LATENCY` = 128) applies to the float and the fixed model alike.
  - At the acceptance sample n, `decide()` runs on the history as of n.
  - Its result is applied at n + L (`Pending`, `apply_pending`).
- While a decision is pending:
  - the search keeps a fresh running maximum, which now restarts on every acceptance;
  - the detector keeps counting;
  - a second acceptance waits for the first result.
- An acquisition closes the search and resets the run counter when it is applied.
- **What L ≤ 63 + (GI − B) leaves unchanged (75 samples at 20 MHz, 200 ns back-off):** every
  acquisition except `decided_at`, unless its search opened within L samples after the previous
  acquisition. The next acceptance needs 64 + (GI − B) positions after the restarted maximum, so
  it cannot come before the pending result.
- **What can change:** a search opened in that interval. This needs the detector to keep firing
  after an acquisition, e.g. under a tone near the signal level. The run counter is reset at the
  result rather than at the acceptance, so the search can open at another sample with another
  provisional latch:
  - its `detected_at` differs in ≈ 5 % of such streams;
  - rarely its grid or CFO as well: in the fixed model, 1 or 2 streams in 2000, with the same
    acquisition count.
- **Clock budget:** a decision is ≈ 2000 complex operations at GI − B = 12 and ≈ 2400 at B = 0.
  - The 2000 break down as: final coarse sum 96, 2·(GI − B) 64-tap correlations 1536, fine
    sum 88, energy 128, 140 rotations.
  - The chain is partly serial: coarse sum → angle → rotation → correlations → angle.
  - At an assumed 8 clocks per sample at 20 MHz, 64 samples are 512 clocks: 4 complex
    multipliers leave no slack, 5–6 do. The clock rate is not a decided target yet.
  - At 10 and 5 MHz the budget doubles and quadruples.
- **History:** the `y` ring (`HISTORY` = 512) covers reading until the result is due
  (T1 − 160 … T1 + 223 + 64, 448 samples).
  - The energy ring (81 positions) does not. The w + 1 energies that choose the grid are already
    63 + w old at acceptance, so the hardware must choose the grid, or latch those energies,
    within a few samples of acceptance.
  - Part d must respect this; the `HISTORY` doc comment says so.

### Evidence
- **Float** (`g1_rx_front::decision_latency_only_delays_the_result`):
  - 150 scenes: clean packets at 4 and 10 dB, CW spur at the noise level and +6 dB, an AGC step
    of −20 dB, two packets 0–40 samples apart.
  - At L ∈ {1, 17, 64, 75} the acquisitions equal those at L = 0, with `decided_at` + L.
  - Two packets at 10 dB are both acquired, each from its own detection.
  - 60 scenes of a tone at the signal level with two packets at 20 dB: the first acquisition is
    unchanged, and `detected_at` of later ones moves in fewer than all.
- **Fixed** (`g2_rx_front::fixed_decision_latency_only_delays_the_result`): 80 ADC scenes, bit
  for bit at L ∈ {1, 64, 75}.
- **Measurement** (`rxfront latency 2000`, report `reports/rx_front_latency.txt`):
  - 2000 streams per kind, float and fixed counted separately.
  - Kinds: clean, CW tone, AGC step, two packets, noise only, and tone + two packets
    (20 dB, tone −6…+6 dB relative to the signal).
  - Up to L = 64 the first five kinds show no difference of any sort.
  - L = 79: one CW-tone stream decides 4 samples later.
  - L = 128: one CW-tone stream differs.
  - Tone + two packets at L = 64: `detected_at` differs in 99 / 98 streams (float / fixed) of
    2000. The grid differs in 0 / 1.
- **Equivalence at L = 0, side by side with main** (independent reviewer, harness outside the
  repo): 1500 streams, SNR 0–30 dB, tones, AGC steps, two packets, 1295 acquisitions. No
  difference in the full output, float and fixed.
- **Existing gates at the new default:**
  - The causality and bound test has its bound raised by L.
  - The G2 report does not involve `decided_at`.
  - `hw/vectors` regenerate identically.

### Mutations
| Mutation | Result |
|---|---|
| `decided_at` not moved to the apply sample | caught (2 tests) |
| result applied one sample early | caught |
| search not closed at apply | survives, equivalent on legal streams |
| run counter not reset at apply | survives |
| pending dropped when the search window closes | survives, unreachable |
| running maximum not restarted after acceptance | survives (open question 15) |

- **Search not closed at apply.** The window ends ≈ T1 + 392 samples, before the first detection
  of the shortest legal next packet.
- **Pending dropped when the window closes.** Acceptance comes ≈ 180 positions before the window
  ends.
- **Why the survivors survive.** Each of these mutations changes L = 0 and L > 0 alike, so the
  latency tests cannot see them. They test the pre-existing acquisition semantics.

### Independent review (fresh agent, code only), 2026-11-18
- **HIGH:** "only `decided_at` changes" was false. The run-counter reset moves `detected_at` of a
  search opened after an acquisition under a tone (counterexample at 20 dB with a tone at the
  signal level). Fixed by restating the claim, adding the tone + two-packet scenes to G1 and to
  `rxfront latency`, and measuring it (above).
- **MEDIUM:** the energy ring does not last the latency → documented (History, above).
- **LOW–MEDIUM:** the MAC budget was undercounted (≈ 1700) → restated as ≈ 2000–2400 with the
  clock assumption.
- **LOW:** the latency was unbounded → `MAX_DECISION_LATENCY` = 128, and the sweep stops at 128.
- **Minor:**
  - The tool counts float and fixed separately, with full keys and a signed extra delay.
  - The `push` and `RxFront` docs now describe the latency.
- **Confirmed:** L = 0 is bit-identical to main, and the fixed model is the exact counterpart.

### Open questions
13. *Resolved, pending founder approval:* a decision latency of 64 samples, with the caveat
    above for searches opened within L after an acquisition.
15. *Resolved 2026-11-18:* the restart of the running maximum now has a gate (entry below).
16. The number of multipliers and the clock rate for `decide()` in part d: 5–6 complex
    multipliers at 160 MHz, or a faster clock.

## 2026-11-18 · G1/G2 · R0-08b: gate for the restart after a failed candidate (open question 15)

**Scene.** The setup is meant to produce a candidate that fails confirmation, followed by a
weaker true T1 that comes after the acceptance point:
- An STF fragment 10 dB above the packet opens a search.
- A 120-sample noise burst at +10 dB then makes the running maximum, which fails confirmation.
- 20–60 samples of silence follow, then the real packet: 15 dB SNR, 50 kHz CFO.

**Gates:**
- `g1_rx_front::a_failed_candidate_does_not_block_a_weaker_packet_after_it`: float model,
  40 scenes, ≥ 38 must be acquired with the grid within ±1 sample.
- `g2_rx_front::fixed_failed_candidate_does_not_block_a_weaker_packet_after_it`: fixed model,
  packet at −25 dBFS so the burst stays inside the ADC range.
- Both pass: 40/40 at commit base `67a2932`.

**Mutation.** Removing the running maximum's restart after acceptance makes the failing burst
stay the maximum, and 0/40 packets are acquired, in the float model and in the fixed model.

**Still open:** a true T1 that is weaker than a failing candidate and lies within the 63 + w
positions after it is still lost. This is unchanged from the streaming design (journal,
R0-08b) and is not a gate.

## 2026-11-18 · G3 · R0-08b part 3c: CORDIC and NCO RTL

**Blocks**
- **`hw/rtl/cordic.sv`** is the vectoring CORDIC (golden `cordic_angle`).
  - Input: 34 bits signed, the provisional latch's lag-16 sum (|·| < 2^33 at D = 13). The same
    block serves the final coarse and fine CFO.
  - Datapath, as in the model:
    - block normalisation to T = W_c − 3 = 15 magnitude bits (leading-one detect on
      |re| | |im|, arithmetic shift right or left);
    - half turn when x < 0;
    - 16 micro-rotations in 18-bit registers, angle wrapping at 16 bits;
    - an explicit zero input gives 0.
  - Fully pipelined, one stage per step: out_valid is seen 17 clock edges after the edge that
    takes the input, as gated by the testbench. An iterative (folded) version is a part-d area
    option.
- **`hw/rtl/nco.sv`** is the phase accumulator plus mixer (golden `nco_mix`, new public function
  that `RxFrontFixed` now calls, so the receiver and the vectors share one implementation).
  - 22-bit phase, advanced on `in_valid` by the sample's step.
  - Quarter-wave table of 128 × (cos, sin) at 11 bits, folded by quadrant.
  - Exact complex product of 24 bits, then `round_sat` to 12 bits (drop 11, half to even,
    saturate).
  - Latency 3 clock edges, gated.
- **Shared support:**
  - `rx_front_rom_pkg` gains the formats and the `cordic_atan`, `nco_cos`, `nco_sin` ROMs
    (generated, `rvec rx-front-rom`).
  - `hw/lint.vlt` waives UNUSEDPARAM for the generated package only.
  - `SampleTrace` gains the NCO `phase` and `step` per sample (behaviour-preserving; the
    G1/G2 gates are unchanged).

**Vectors** (`rvec cordic`, `rvec nco`)
- **cordic:**
  - edges in all pairwise combinations: 0, ±1, ±2, 3, 2^33 − 1, −2^33 and their
    neighbours, ±2^14, 2^15 − 1, −2^15;
  - 6000 random inputs, re and im at independent magnitudes from 1 to 34 bits.
- **nco:**
  - the receiver's own y and step traces of four frames (AWGN 25/4 dB, TGn D 9 dB,
    −3 dBFS); the generator checks that the traced phase is the accumulation of the steps;
  - 3000 random full-scale samples with random 22-bit steps;
  - a table sweep over two turns, one address per sample, y at the extremes;
  - the most negative y at quadrant steps.

**Evidence**
- `make -C hw cordic`: 6196 cases, 0 mismatches.
  - Idle cycles between inputs.
  - Every output must arrive exactly 17 edges after its input.
  - Five inputs are in flight at a mid-file reset, and none may come out.
- `make -C hw nco`: 7 segments, 8554 samples, 0 mismatches.
  - Idle cycles with garbage on `in_step`.
  - Latency exactly 3 edges.
  - Three junk samples are in flight at each segment's reset, and none may come out.
- Stub RTL fails both: 6126 and 7685 mismatches.

**Mutations, CORDIC**
| Mutation | Mismatches |
|---|---|
| `y > 0` becomes `y ≥ 0` | 713 |
| no half turn | 713 |
| logical shift in the x update, y ≤ 0 branch | 6109 |
| normalise to 14 bits | 1262 |
| zero not special | 62 |
| right shift one too far | 3129 |

Two further CORDIC mutations, for the timing checks:

| Mutation | Mismatches |
|---|---|
| valid pipeline not cleared by reset | 5877 |
| out_valid one stage early | 11856 |

The same logical-shift mutation in the `y > 0` branch is equivalent: y is positive there.

**Mutations, NCO**
| Mutation | Mismatches |
|---|---|
| quadrants 1 and 3 swapped | 1698 |
| phase moves every clock | 6487 |
| one bit too few dropped | 8545 |
| table address bit 0 flipped | 8545 |
| conjugate rotation | 8430 |
| valid not cleared by reset | 700 |

"NCO out_valid one stage early" leaves a bit unused and is stopped by lint (-Wall), so it never
builds.

**Note on the method.** The first run of the "normalise to 14 bits" mutation reported 0
mismatches. A rerun gave 1262, and the model confirms that 14 and 15 bits differ in 21 % of
random inputs. The first run reused a stale Verilator build after the previous restore (the
mtime gotcha in CLAUDE.md). Survivors are therefore re-run before they are trusted.

**Pre-PR review (fresh agent, code only), 2026-11-18:** nothing significant.
- The reviewer's own stress run reached 0 mismatches:
  - cordic: 2.4 M cases, including every input in [−300, 300]² and every ±2^b ± 1 pair;
  - nco: 3.0 M samples, including all 512 table addresses × 36 extreme y.
- Fixed after the review:
  - independent re/im magnitudes in the vectors;
  - the latency and reset-flush checks above (the module headers had the latency one too
    high);
  - the confirmation path now also calls `nco_mix_with`, so the receiver has one mixer;
  - explicit sign-extended operands in the NCO products, so they do not depend on a tool's
    cast semantics;
  - the |sh| ≤ 2^T comment.
- Noted, not changed: `CORDIC_IN_BITS` = 34 is set in `rvec`, not derived from `det_bits`.

**Not yet:** a Vivado OOC timing run. The NCO's 13 × 11 products and the CORDIC's 18-bit adders
are each registered, so they are not expected to be critical; it will be run with the top level.

## 2026-11-18 · G3 · R0-08b part 3d: DC blocker and detector A RTL; corrected mutation records

**Blocks**
- **`hw/rtl/dc_block.sv`** is the causal DC blocker (golden `DcFixed`).
  - Accumulator M = m·2^10 in 24 bits; e = x·2^F − M; M += e >>> k.
  - k = min(⌊log2(n + 1)⌋, 7), from a sample counter that stops once k = 7.
  - y = e·2^−F through `round_sat` to 13 bits. Saturation is unreachable: |e|·2^−F < 4095.5.
  - y is registered at the edge that takes x (latency 0 in the testbench's count).
- **`hw/rtl/det_a.sv`** is detector A with its run counter (golden `DetFixed` and the counter of
  the new public `detector_trace_fixed`, which `detections_fixed` now uses).
  - A 64-deep history and exact running sums: C 32 bits, P1 and P2 31 bits.
  - The comparison 2^16·|C|² ≥ 21668·P1·P2 is done in 80 bits, with no normalisation.
  - The model's add (n ≥ 16), subtract (n ≥ 64) and full-window (n ≥ 63) rules are
    reproduced by a saturating sample counter, so the history needs no reset.
  - Run counter to 4, with a `clear` input for the top level's acquisition.
  - 4 pipeline stages; fire/event come 4 edges after the sample's x edge.
- `hw/tb/det_chain.sv` wires the two as the top level will.

**Vectors** (`rvec det`): x, y, fire and event per sample.
- Four received frames.
- Noise alone at −30 dBFS: no events.
- A CW tone at the noise level and at +10 dB.
- A −20 dB AGC step before a frame.
- A DC offset of (+1500, −1200) under a frame.
- 2000 full-scale random samples.
- Full-scale steps.
- Totals: 18 943 samples, 5302 fires, 1306 events.

**Evidence**
- `make -C hw det`: 0 mismatches in y, fire and event.
- Exact latencies: y 0, fire/event 4.
- Junk samples in flight at each reset never come out.
- Idle cycles carry garbage inputs.
- The stub fails with 67 024 mismatches.

**Mutations, `det`** (each built from scratch)
| Mutation | Mismatches |
|---|---|
| no start-up gear | 6686 |
| logical shift in the M update | 32 071 |
| sample counter wraps | 16 563 |
| y off by one LSB of e | 23 |
| subtract one sample early | 4226 |
| add one sample early | 5094 |
| threshold halved | 3294 |
| no P > 0 check | 103 |
| run length 3 | 1315 |
| conjugate on the wrong factor | 9930 |
| first window one sample late | 1469 |

- "P2 subtracts the wrong term" is stopped by lint, because a term becomes unused.
- `clear` is driven 0 here, so its gate is the top level's.

**Correction to parts 3a and 3c: stale builds.** Two mutations in a row reported the same count
(713/713 and 8545/8545 in 3c, 6475/6475 in 3a). Verilator's incremental build had reused the
model of the previous mutation after the file was restored, so a result could belong to the
previous mutation. Rerun with clean builds:
- **3c, CORDIC:**

  | Mutation | Mismatches |
  |---|---|
  | `y ≥ 0` | 554 |
  | no half turn | 2208 |
  | logical shift, y ≤ 0 branch | 6170 |
  | normalise to 14 bits | 1321 |
  | right shift one too far | 2397 |
  | zero not special | 4 |
  | valid not cleared | 5877 |
  | out_valid early | 11 856 |

- **3c, NCO:**

  | Mutation | Mismatches |
  |---|---|
  | one bit too few dropped | 8545 |
  | table address bit 0 | 5900 |
  | quadrants 1/3 | 1698 |
  | phase every clock | 6487 |
  | conjugate | 8430 |
  | valid not cleared | 700 |

- **3a, `ltf_corr`:** tap index (pair 31 uses tap 1) 6458; re/im swap 6418.

Every mutation is still caught; the counts in the 3a and 3c tables above are superseded.

**Pre-PR review (fresh agent, code only), 2026-11-18:** nothing blocking.
- Its stress harness ran ≈ 4.9 M samples with 0 mismatches. The streams covered:
  - a 300 k-sample segment;
  - cuts at n = 15, 16, 62, 63 and 64;
  - P1 and P2 returning to 0 after being positive;
  - the largest DC step;
  - period-16 full-scale patterns.
- The harness bites: with no P > 0 check it gives 15 132 mismatches, and with CMP_W = 72
  135 941.
- `clear` aligned to stage 4 matched the model's reset at `apply_pending` (0 mismatches), and
  misaligned versions failed. The header now states this contract; its gate is the top level's.
- Fixed after the review:
  - the counter width is derived from the parameters;
  - the stage-3 multiplies use natural widths;
  - idle garbage is on both inputs.
- Open:
  - the `g_narrow` path (DET_BITS < Y_BITS) has no G3 evidence while D = Y_BITS;
  - the top bits of C, P and M are unreachable through the DC blocker (by analysis), so a
    mutation that shrinks them by one bit is not caught.
`hw/Makefile` now builds every gate from scratch, so the hazard is gone for gates and mutation
runs alike.

## 2026-11-19 · G3 · R0-08b part d, step 1: provisional latch and the delayed NCO path

**Blocks**
- **`hw/rtl/rx_latch.sv`** (golden: `RxFrontFixed::push` steps 1–2, `SampleTrace.step`).
  - The latch sum is C(n − 24) + C(n − 72) from a 73-deep delay line of `det_a`'s C
    (`ram_1w2r`, 2 × 32 bits; written with C(n) and read at n − 24 and n − 72 in the same
    cycle, never the written address). ADR 0006 said "72-deep"; reading n − 72 while writing n
    needs 73.
  - Events before sample 135 do not latch (a saturating sample counter, the model's
    `checked_sub`).
  - CORDIC, then the step: angle·2^(P−A−S). The model's rounding in `per_sample` is a no-op at
    P − A ≥ S (an elaboration check), so the RTL has no rounding adder; the "no rounding"
    mutation was equivalent, which is how this was found.
  - The NCO path runs `D_SAMPLES` behind: sample k leaves when sample k + D enters, with the
    step in effect at k. Latch results carry the tag of their sample and wait in a small FIFO,
    so the output does not depend on when the CORDIC answers, only on the contract: the
    result is back within D samples. Measured: 23 clocks from the sample's x edge, so
    D·R ≥ 23: D = 3 at R = 8, 4 at R = 6, 5 at R = 5. These are ADR 0006's examples, but its
    formula "⌈23/R⌉ + 1" has one sample more than they do; the examples are right, and the
    margin is 1–2 clocks, not a sample. A register added on the CORDIC path (step 3's
    arbiter) makes D = 4 at R = 8.
  - A simulation check (`CHECK_TIMING`) stops the run when the contract is broken. Hardware
    has no guard: a late result is applied late, and one later than its sample blocks the
    following latches for up to 256 samples (the module header says so).
- **`det_a`** now exports C of the sample in stage 2 (`c_valid`, `c_re`, `c_im`).
- `hw/tb/latch_chain.sv` wires dc_block → det_a → rx_latch → nco as the top level will. `clear`
  comes from the vectors (the sample an acquisition is applied at) until the decision engine
  exists, delayed to det_a's stage 4.
- Model: `SampleTrace.event` added (trace only). The ROM package gains `LATCH_LAG` and
  `COARSE_LEN`.

**Vectors** (`rvec latch`): x, acq, event, step and z per sample from `push_traced`; 11 segments,
14 787 samples, 1794 events (75 before the latch start), 1641 step changes, 7 acquisitions.
- The four frames of `det`.
- A tone 10 dB over the noise from sample 0.
- Clean tones with an event exactly at sample 135 (the first that latches) and at 134.
- Two frames under a weak tone.
- A frame under a tone 3 dB above it: the acquisition falls inside a run of fires, so the
  counter reset moves later events. `rvec` refuses to write the file unless it does.
- Full-scale random samples and full-scale steps.

**Evidence** (`make -C hw latch`: three builds, four runs; branch commit 3e17fc9 and its review
fixes, squashed into the PR of this entry)
| Build | Run | Mismatches (event, step, z) |
|---|---|---|
| D = 3 | R = 8 | 0 |
| D = 3 | R = 16 | 0 |
| D = 4 | R = 6 | 0 |
| D = 5 | R = 5 | 0 |

- Samples come every R clocks plus an idle clock after every 5th and two after every 13th; idle
  inputs carry garbage (including `in_clear` = 1).
- Exact latencies: event 4 edges after its sample; step 1 and z 5 edges after sample k + D.
- Before each segment: a one-clock reset with a latch in flight (the segment up to its first
  latching event, then 8 clocks: the CORDIC is mid-way and the FIFO holds the entry). 10 such
  resets per run. Nothing may reach the run that follows.
- Three junk samples in the detector's pipeline at each segment's end reset: nothing comes out.
- Registers without a reset and the delay line start from random values
  (`+verilator+rand+reset+2`).
- D = 3 at R = 6 (contract broken) stops with the `CHECK_TIMING` error.
- The first run failed as expected for a wrong assumption of mine: z latency is 5, not 4.

**Mutations** (each built from scratch, three builds each; the counts were equal across builds)
| Mutation | Mismatches |
|---|---|
| old window read at n − 73 | 16 185 |
| lag window read at n − 25 | 16 556 |
| no start gate | 6979 |
| start gate one sample early | 6146 |
| start gate one sample late | 268 |
| one window only | 19 999 |
| im of the old window replaced by its re | 20 389 |
| step shifted one bit too far | 20 018 |
| step one sample late | 11 013 |
| step one sample early | stopped by `CHECK_TIMING` |
| NCO path delay D − 1 | 14 531 |
| result written at the issue pointer | 19 694 |
| step register not updated | 18 344 |
| `clear` one stage early | 111 |
| `clear` never | 111 |
| C tapped one stage late | 16 825 |
| CORDIC valid chain not reset | 18 711 |
| FIFO pointers not reset | stopped by `CHECK_TIMING` ("a latch result was skipped") |
| step register not reset | 19 075 |
| emission tag not reset | stopped by `CHECK_TIMING` |

Three mutations survived the first vector set and each led to a change:
- "start gate one sample late": no event fell on sample 135. Added the two clean-tone segments.
- "`clear` early" and "`clear` never": no acquisition fell inside a run of fires. Added the
  strong-tone segment. This is the gate of the `clear` contract left open in part 3d.
- "no rounding in `per_sample`": equivalent; the adder was removed.

**Pre-PR review (fresh agent, code only), 2026-11-19:** no RTL bug.
- Its stress harness: 3 × 40 segments of ≈ 42 000 samples from `push_traced` (tones at the ±π
  angle edge and at zero CFO, hops, bursts, DC), random idle gaps, resets after random prefixes;
  D/R from 3/8 to 30/1, including back-to-back samples. 0 mismatches. D = 22 at R = 1 and
  D = 3 at R = 7 trip the check, D = 23 at R = 1 passes: the bound is exactly D·R ≥ 23.
- Fixed after the review:
  - a reset with a latch in flight was not exercised (the old run drained 40 clocks first):
    "CORDIC valid chain not reset" passed. The in-flight reset above was added; the four reset
    mutations in the table are its evidence;
  - the contract in the module header (the formula, the zero margin, the silent failure in
    hardware);
  - random initial values in the gate's runs.
- Noted, not changed: the resets of the delay line's write address and of the read-valid flag
  are redundant (removing either changes nothing: the start gate covers stale contents). They
  stay, for a defined state after reset.
- Not covered by the gate: R < 5, and jitter that removes clocks (the pattern only adds them).
  The reviewer's harness covered both.

**Open**
- The CORDIC is inside `rx_latch`. Sharing it with the decision engine (ADR 0006, latch
  priority) moves it out; that is step 3's change and step 3's gate.
- With events at least `DET_RUN` = 4 samples apart, the FIFO holds two results only at D = 5;
  D ≤ 4 never has more than one.
## 2026-11-19 · measurements · the phase review's findings rerun (AGC transient, band-limited noise, input level)

An independent review of everything through R0-08b (`docs/reviews/2026-11-19-r0-phase-review.md`)
made three claims about the receiver front that can be measured. Each was rerun with a committed
tool, and the reruns were reviewed by a second fresh reviewer (its corrections are in the text).
No model behaviour changed; the decisions are in ADR 0007 (proposed, in its own pull request).

**New in the model:** `channel::agc_transient` (gain, phase and DC per state while the gain
control settles; the ADC clips). Test: `g1_channel_impairments::agc_transient_states`, against
its definition and against `gain_step` and `add_tone`, which it generalises (`channel.md`).

**1. AGC transient** — `rxfront agc 500`, `reports/rx_front_agc.txt` (this run is kept as
`rx_front_agc_combined.txt` since R0-08d). Fixed receiver, 20 dB SNR.
The phase jumps in these cases are assumed (±10°, ±30°, ±90° per gain change); no capture of a
real AGC is behind them yet. The cases separate the causes.

One event at STF sample 48, inside the coarse window (STF samples 32…144):

| Case | Combined | Coarse | Fine |
|---|---|---|---|
| no gain change | 422 Hz | 866 Hz | 477 Hz |
| −6 dB step, no clipping, no jump | 444 Hz | 1.2 kHz | 461 Hz |
| −20 dB step, no clipping, no jump | 923 Hz | 3.5 kHz | 520 Hz |
| −20 dB step, the start clipped (+5 dBFS), no jump | 3.2 kHz | 13.4 kHz | 448 Hz |
| no gain change, phase jump ±10° | 928 Hz | 3.5 kHz | 457 Hz |
| no gain change, phase jump ±30° | 2.3 kHz | 9.8 kHz | 454 Hz |
| no gain change, phase jump ±90° | 6.1 kHz | 25.9 kHz | 467 Hz |

- What damages the coarse estimate is the clipped stretch and the phase jump. A gain step
  alone costs little (−6 dB: nothing measurable; −20 dB: 2×).
- The damage reaches the output through the 0.771/0.229 combination, about a quarter of it.
- A step at sample 16 or 32 costs nothing measurable (407 and 458 Hz against 422): the coarse
  window starts at 32, as designed.
- The fine estimate is untouched by anything before the long training field, and damaged in
  the same way by anything inside it: a −2 dB trim with a ±30° jump at sample 176, 208 or 272
  gives 2.4, 8.3 and 7.0 kHz fine and leaves the coarse estimate at 0.8 kHz. Handing on the
  fine estimate alone moves the deadline for the AGC from STF sample 32 to the end of the STF;
  it does not remove it.
- Timing: a −20 dB step at sample 144 (the last STF period) puts 214 of 500 frames one whole
  training symbol off (±64 samples); at 176, 419 of 500. The timing search has no protection
  against a gain step near the LTF. A −2 dB trim anywhere leaves the timing exact.
- With a clipped −20 dB step and ±90° jumps, 316 of 500 frames are acquired and 39 of those
  are 64 samples off: the confirmation rotates the LTF by the damaged coarse estimate. The
  coarse RMS of that row is over the survivors only.
- "Slips" are 0 in every row, but a damaged coarse estimate fails the confirmation before it
  can resolve the fine ambiguity wrongly, so this says little.
- The existing gate `gain_step_robustness` steps at sample 24, before the coarse window, with
  no clipping and no phase jump: it could not see any of this.

**2. Band-limited noise** — `rxfront tailbw 2000`, `reports/rx_front_tailbw.txt`. Fixed
receiver, 2·10⁹ samples per passband (10 threads), noise at −30 dBFS behind a 65-tap low-pass
filter. An excursion is a group of events less than 1000 samples apart.

| Noise passband | Detector events | Excursions | Per sample | Acquisitions |
|---|---|---|---|---|
| white | 0 | 0 | — | 0 |
| ±9.0 MHz | 12 | 8 | 6.0·10⁻⁹ | 0 |
| ±8.3 MHz (the occupied band) | 28 | 18 | 1.4·10⁻⁸ | 0 |
| ±7.5 MHz (cuts into the signal) | 129 | 85 | 6.5·10⁻⁸ | 0 |
| ±6.0 MHz (cuts into the signal) | 3248 | 2085 | 1.6·10⁻⁶ | 0 |

- The budget is 5·10⁻⁸ events per sample. Behind a filter that keeps the signal's own
  subcarriers the detector stays under it by a factor of 3.5 to 8, not the fifty of white
  noise. The two narrower filters are not usable channel filters and show the trend.
- No acquisition in 10¹⁰ samples (about 2200 searches opened).
- So "0 false alarms in 10⁹" (part 2, G2) is a white-noise statement about detector events.
  With a channel filter the rate depends on the filter, and the search duty cycle with it.
  R0-08c measures with this, and with an adjacent-channel STF, which is not tested yet.

**3. Input level** — `rxfront level 400`, `reports/rx_front_level.txt`. Float and fixed on the
same ADC samples.
- −3 to −15 dBFS: fixed CFO RMS equal to float within 0.5 % at 4, 10 and 25 dB SNR.
- −30 dBFS: +1 %, 0 %, +6 %. −45 dBFS: +2 %, +3 %, +62 %. −55 dBFS: +17 %, +33 %, 2.6×.
  −60 dBFS: up to 5.4×.
- Acquisition and timing: 400 of 400 in every cell.
- The G2 claim "CFO ≤ +0.5 %" was measured at −15 dBFS and holds from −3 to −15 dBFS. The
  loss at low levels is largest at high SNR, where the quantisation is the only noise.
- This is the CFO estimate only. A packet at −3 dBFS RMS clips its peaks, which the data
  symbols will not tolerate: the AGC's target window comes from R0-09's measurements, with
  this as its lower edge.

**4. Timing probe** — Vivado 2024.2, out of context, xck26-sfvc784-2LV-c (KV260), one clock at
6.25 ns (160 MHz), synthesis to routing with default settings, sources at 464d270. Script:
`tools/syn/ooc_probe.tcl`; report excerpts: `reports/rx_front_timing_ooc.txt`. Run by hand on
the build server; no gate reruns it.

| Block | WNS | LUTs | Registers | DSPs |
|---|---|---|---|---|
| `dc_block` | +3.41 ns | 189 | 83 | 0 |
| `nco` | +2.58 ns | 96 | 52 | 4 |
| `cordic` | +3.72 ns | 1212 | 779 | 0 |
| `rx_latch` (with its CORDIC and delay line) | +0.84 ns | 1758 | 1111 | 0 |
| `det_a` | −0.64 ns (53 endpoints) | 642 | 300 | 30 |
| `ltf_corr` | −1.89 ns (729 endpoints) | 1932 | 3320 | 124 |

- `det_a` fails in stage 3, as the review predicted: P1·P2 into the 15-bit constant multiply,
  6.85 ns through three DSP stages and ten carry blocks. One more register there closes it and
  moves fire/event from 4 to 5 edges; `rx_latch` and the `clear` alignment take their
  latencies from that. Not done here: R0-08c retunes this block, and the compare is a
  candidate for a narrower form in the model first (ADR 0007).
- `ltf_corr` is the known −1.9 ns (part 3a): its adder tree is not pipelined.
- `rx_latch`'s worst path is the latch sum into the CORDIC's normalisation (14 levels).
- The 73 × 64-bit delay line maps to distributed RAM (33 LUTs), no block RAM. The NCO's
  quarter-wave table became a block RAM (a ROM in logic for the ASIC).
- This is the first timing evidence for these blocks; "not expected to be critical" in part 3c
  holds for the CORDIC and the NCO.

**Open questions**
17. Which CFO the acquisition hands on (ADR 0007, decision 1).
18. What a real AGC does to the phase and for how long it clips (AD9361 fast attack first,
    from R1 captures). Every conclusion of item 1 scales with it.
20. The timing search under a gain step in the last STF period or in the LTF (whole-symbol
    errors).
19. The hand-off of samples to the rest of the receiver and the latency budget to SIGNAL and
    to the ACK (ADR 0007, decision 2).

## 2026-11-19 · note · the same blocks on the open ASIC flow

`docs/gates/silicon_probe.md` has the receiver-front blocks through OpenROAD-flow-scripts on
IHP SG13G2 at the same 6.25 ns: `det_a` fails there too (−3.2 ns), `ltf_corr` passes at the
global-route estimate, and the blocks together are about 3 mm² of cells. The simulation
checks of `rx_latch` and `ram_1w1r` moved inside `` `ifndef SYNTHESIS `` because the open
flow's front end rejects procedural `$error`; the `latch` and `ram` gates are unchanged.

## 2026-11-19 · G1/G2 · R0-08d(1): the acquisition hands on the fine CFO estimate (ADR 0007)

ADR 0007, decision 1, accepted by the founder on 2026-11-19. Both models change; no RTL and no
vector does (`make -C hw vectors` leaves `hw/vectors/` untouched: no vector carries the CFO
that is handed on).

- **Float** (`rx_front.rs`): `cfo_hz = fine + coarse_weight·(coarse − fine)`, with
  `RxConfig::coarse_weight` 0 by default. `FINE_WEIGHT` stays as the AWGN optimum:
  `coarse_weight = 1 − FINE_WEIGHT` is the rule used until today (the same value up to
  float rounding: the expression is reordered).
- **Fixed** (`rx_front_fixed.rs`): `cfo = coarse + round((64 − w)·d/64)`, `w =
  FixedRxConfig::coarse_weight_64`, 0 by default: `fine` exactly, so the decision engine
  needs no multiplier for it. `w = 15` is the old 49/64 rule bit for bit. `is_valid` refuses
  a weight outside 0…64.
- The two models carry the weight separately (`RxConfig::coarse_weight` is ignored by the
  fixed model, which reads `FixedRxConfig::coarse_weight_64`): a paired run with a
  non-default weight must set both, and 15/64 is not 1 − 0.771 exactly, as before.
- The coarse estimate still resolves the fine one's ambiguity and still rotates the LTF for
  the confirmation: what ADR 0007 lists as not solved stays not solved.

**Gates (written first, seen failing against the old default)**
| Gate | What it holds |
|---|---|
| `g1_rx_front::the_acquisition_hands_on_the_fine_estimate` | default output = fine estimate; a set weight gives `fine + w·(coarse − fine)`; with a 30° carrier jump at STF sample 48 (20 dB, 300 frames) the output's RMS error is 428 Hz, the lag-64 theory 454 Hz (gate ±15 %), where the AWGN weight gives 3947 Hz (gate: more than 4×) |
| `g1_rx_front::acquires_timing_and_cfo_in_awgn` | the default against the fine estimate's own variance theory at 20, 10 and 5 MHz, 10 and 20 dB: RMS/theory 1.01 to 1.03 (gate ±8 %); the AWGN weight on the same frames still against the combined theory |
| `g2_rx_front::fixed_acquisition_hands_on_the_fine_estimate` | fixed default `cfo_phase == fine_phase` on 200 frames; `w = 15` equals the old rule exactly; weights −1 and 65 are invalid |

**Mutations** (each fails the gates named; sources restored with `cp` + `touch`)
| Mutation | Fails |
|---|---|
| float hands on the old combination | hands-on gate, AWGN theory gate |
| float: sign of the weight | hands-on gate, AWGN theory gate |
| float hands on the coarse estimate | those two, `sample_clock_offset_does_not_disturb_acquisition`, `gain_step_robustness` |
| fixed: old rule (49/64) regardless of the weight | fixed hands-on gate |
| fixed: no rounding | fixed hands-on gate |
| fixed: weight ignored (always 64/64) | fixed hands-on gate |
| fixed: weight not validated | survived at first; the gate now asserts `is_valid` and fails |

**Reruns** (`docs/gates/reports/`, replaced: `rx_front_agc.txt`, `rx_front_level.txt`, the
`g2` and `g2joint` parts of `rx_front_g2.txt`; `rxfront latency 2000` is identical line for
line and was left; `tailfx` is the detector alone)
- **G2 holds.** Chosen widths: FFT start differs in 0 of 1200 frames, fixed against float CFO
  RMS +0.0 to +0.2 % per case (was +0.2 to +0.4 %); joint run 5 of 11 983, CFO within 0.05 %.
  The one-at-a-time sweep gives the same PASS/FAIL verdict in every row but one: W_c = 12
  now passes (CFO +8.8 % at 25 dB against the +10 % budget, was +23.4 %; W_c = 13 still
  fails on timing). The coarse angle no longer reaches the output, so its width matters
  less there. The chosen widths are not changed on this: W_c is 18, and "they can only
  shrink" is a question for G2 restated as PER loss (ADR 0007, order of work).
- **What it buys** (`rxfront agc`, 20 dB, 500 frames per case, the phase jumps assumed): with
  a gain change inside the STF the output's error is 430 to 540 Hz in every row where the
  timing is right; it was 380 Hz to 8.5 kHz (13.5 kHz in the ±90° row, where the timing is
  exact in 277 of 500 frames). A −20 dB clipped step at sample 48 with ±30°: 8133 Hz →
  478 Hz. The gain is confined to events inside the coarse window (samples 32 to 144); in
  the seven rows where the old output was already 380 to 458 Hz (a step before sample 32, a
  −6 dB step, a trim without a jump or at sample 144) the new one is 4 to 16 % worse, the
  AWGN cost below. The old run is kept as `rx_front_agc_combined.txt`; the report's first
  error column is now called "output".
- **What it costs.** With a still front end: 477 Hz instead of 422 Hz at 20 dB (+13 %; in
  the G1 run 465 against 454 Hz of theory at 20 MHz). Over SNR (`rxfront level`, float,
  −15 dBFS, against the run before): +6 % at 4 dB (3110 against 2925 Hz), +14 % at 10 dB
  (1420 against 1249 Hz), +16 % at 25 dB (250 against 215 Hz; the high-SNR limit of the
  theory is +13.9 %). And a jump inside the LTF now reaches the output whole instead of
  0.77 of it: a −2 dB trim with ±30° at sample 176, 2402 Hz (was 1853); at 208, 8311 Hz
  (was 6360); at 272, 6978 Hz (was 5345). The requirement on the AGC is the one ADR 0007 states: still from the end
  of the STF.
- **Level** (`rxfront level`): the fixed model's CFO error against float is within 1 % from
  −3 to −15 dBFS at every SNR and +7 % at −30 dBFS at 25 dB (was +6 %); the picture below
  −45 dBFS is as before (quantisation of the estimate's input).
- Not rerun: `rx_ofdm_per.txt` and the soft-format reports of R0-09 (their frames start the
  timing tracker from the CFO; the estimate they see is 6 to 16 % noisier in AWGN, most at the high SNR of
  the fast modes). They are
  rerun with R0-09 G2, which changes the receiver they measure anyway.

**Evidence:** `cargo test -p radio-phy --test g1_rx_front --test g2_rx_front` — 24 + 10 pass;
`scripts/check.sh` green; this entry's pull request.

**Pre-PR review (fresh agent, the diff only), 2026-11-19:** no defect in either model (the
fixed rule checked for every sign of the residual and for overflow; `w = 15` is the old
expression literally). Corrected after it, all in this entry or in labels: the "was" range
of the AGC rows (it had been stated as 0.8 to 13.5 kHz, ignoring seven rows that were
already below 460 Hz and are now slightly worse), the AWGN cost given per SNR instead of
one point, the report column still called "combined", the old report kept for ADR 0007's
figures, the two separate weight parameters stated. Not verified by the reviewer: the
mutation table and `scripts/check.sh` (both run here).

## 2026-11-19 · G1 + measurements · R0-07e, second half: the DC blocker at a gain change

**Question** (ADR 0007's backlog item, open question 5 above, the phase review's finding 8):
should the DC blocker be told about a gain change, and what does a DC offset that changes
with the gain cost a packet? Until now nothing measured it: `rxper` hands the decoder the
samples before the blocker, and no scene carried a DC offset into the data symbols.

**New in the float model:** three controls on `rx_front::DcBlocker`, candidates for ADR 0008.
The fixed-point blocker and `hw/rtl/dc_block.sv` are unchanged (no vector changes).
- `hold(on)`: the estimate and the gear do not move; y = x − m still.
- `rescale(gain_db)`: the estimate times 10^(g/20); acts while held too.
- `restart(k)`: the gear restarted from the present estimate at 2^(−k); a hold stays.

**Gate:** `cargo test -p radio-phy --test g1_rx_front dc_blocker` — 6 pass (4 new; the
first three were seen failing before the controls existed: they did not compile). The
reference is the definition worked by hand on inputs of a few samples: a consistency gate
of the model, not an outside reference. **Mutations** (10, each fails its test): the gear
advancing while held; the hold ignored; the rescale as a power ratio; its guard against a
step that is not a number removed; the restart one sample on; its cap removed; the estimate
cleared by it; and after the review a step of −∞ zeroing the estimate (the guard had let
it through), the rescale ignored while held, the restart releasing the hold.

**Measurement:** `rxfront agcdc 2000` → `reports/rx_front_agcdc.txt`. Packets of 1000
random octets through `FastAttackAgc` (power over 16 samples, the lock left on a large
overload), a DC offset per gain state from `channel::GainDc` (`channel.md`) which the gain
control's detectors see with the signal (`run_dc`), a phase jump within ±30° per gain
change (assumed), a 12-bit ADC, the **float** receiver (`RxFront` and `rx_ofdm` with its
defaults). Noise at −30 dBFS at the largest gain. Without an offset the packet settles at
−10 dBFS at every SNR; at 25 dB that is one change of −5 dB at the lock, on average 74
samples into the STF (the reviewer's count on the first run), from 30 dB up several steps.
The gain is held from the first lock inside the packet (see the last finding).
Frames lost of 2000, 54 Mb/s, the four SNRs (25 / 30 / 40 / 60 dB):

| DC offset | no blocker | blocker as it is | hold | rescale | restart at 2⁻⁴ | frozen after acquisition |
|---|---|---|---|---|---|---|
| none | 0 / 0 / 0 / 0 | 8 / 1 / 2 / 0 | 8 / 0 / 2 / 2 | 8 / 1 / 1 / 1 | 13 / 1 / 2 / 1 | 35 / 15 / 9 / 13 |
| per index, −40 dBFS | 0 / 0 / 0 / 0 | 5 / 0 / 1 / 1 | 5 / 2 / 0 / 1 | 5 / 1 / 0 / 2 | 13 / 3 / 1 / 3 | 47 / 23 / 11 / 13 |
| per index, −30 dBFS | 198 / 86 / 56 / 76 | 13 / 3 / 2 / 1 | 13 / 5 / 4 / 1 | 12 / 3 / 1 / 2 | 18 / 3 / 1 / 3 | 59 / 23 / 13 / 13 |
| per index, −20 dBFS | 1600 / 1519 / 1526 / 1477 | 88 / 18 / 16 / 22 | 88 / 21 / 35 / 29 | 51 / 9 / 10 / 13 | 17 / 6 / 1 / 4 | 135 / 55 / 54 / 34 |
| before the gain, −30 dBFS at the largest gain | 8 / 0 / 0 / 0 | 6 / 4 / 0 / 2 | 6 / 2 / 3 / 3 | 6 / 4 / 0 / 2 | 11 / 4 / 2 / 3 | 43 / 21 / 12 / 16 |
| before the gain, −20 dBFS | 1000 / 92 / 0 / 0 | 12 / 2 / 1 / 6 | 12 / 3 / 3 / 10 | 5 / 3 / 0 / 2 | 13 / 1 / 0 / 2 | 51 / 18 / 14 / 21 |
| before the gain, −10 dBFS | 1901 / 1486 / 74 / 0 | 810 / 49 / 40 / 42 | 851 / 87 / 117 / 200 | 693 / 0 / 1 / 2 | 707 / 2 / 2 / 2 | 869 / 109 / 89 / 99 |

("Blocker as it is" is the column `free`: the decoder is handed the blocker's output. At
25 dB the hold is the blocker as it is by construction: the one gain change is the lock's.
The report also has −50 dBFS rows, hold followed by a restart, a restart with freezing,
and `free/raw`, the blocker before the acquisition only, which at 54 Mb/s loses what "no
blocker" loses.)

- **The decoder must be handed samples without the DC offset.** With none removed, an
  offset of −30 dBFS (20 dB below the packet) loses 3 to 10 % of the 54 Mb/s frames and one
  of −20 dBFS three quarters. 6 Mb/s tolerates everything but an offset above the packet
  (10 dB SNR, the packet at −20 dBFS: half the frames at −20 dBFS, where the acquisition
  itself fails without a blocker; with the blocker before the acquisition only, none are
  lost there, and 1715 of 2000 under the largest offset). What the hand-off delivers is
  ADR 0008's to decide; this is its first number.
- **The blocker in the decoder's path is not free:** with no offset at all it loses 8 of
  2000 frames at 25 dB where the samples before it lose none, and 0 to 2 at 30 dB and above.
  Its estimate follows the signal (pole 2⁻⁷). Not measured: a slower pole once the packet
  is acquired.
- **Freezing the estimate once the packet is acquired is worse, not better** (35 of 2000
  at 25 dB against 8, 9 to 15 against 0 to 2 above): the frozen value keeps the error it
  had at that sample for the whole packet. I expected the opposite.
- **Told nothing, the blocker shows no loss up to a per-index offset of −30 dBFS and an
  offset before the gain of −30 dBFS at the largest gain.** At −20 dBFS before the gain the
  totals are still small (12 / 2 / 1 / 6), but the columns are paired (the same frames),
  and on the first run the reviewer's count of frames lost by one column only already
  separated them there (rescale against the blocker as it is 5 : 19 over the four SNRs;
  hold worse at 40 dB, 7 : 0). The paired counts were not redone for this run.
- **Beyond that a restart at each gain change is the control that works for both kinds**
  (per index −20 dBFS: 17 / 6 / 1 / 4 against 88 / 18 / 16 / 22; before the gain −10 dBFS
  from 30 dB up: 2 against 40 to 49). At 25 dB without an offset it loses 13 against 8, and
  11 to 15 against 5 to 7 in the other small-offset rows: there it is one restart at 2⁻⁴
  some 74 samples into the STF, which is the late case, not a general cost; no pair is
  significant alone.
- **Rescaling is exact for the offset before the gain and half a cure for the per-index
  one** (51 / 9 / 10 / 13), and costs nothing measurable without an offset. It needs the
  size of the step from the gain control, a restart only the fact.
- **Holding the estimate while the gain searches never helps and hurts with a large
  offset** of either kind (before the gain −10 dBFS: 200 lost at 60 dB against 42; per
  index −20 dBFS: 35 against 16 at 40 dB): the estimate stays at the idle gain's offset
  and has the whole change to work off after the lock.
- **A large offset blinds the gain control itself.** Before the gain at −10 dBFS and a
  packet at 25 dB (−5 dBFS at the largest gain): about 450 of 2000 frames get a timing one
  long symbol early in every column, and 700 to 870 are lost whatever the blocker does.
  The control is locked on the offset, the packet is not 10 dB above it, and nothing
  unlocks the gain until a peak overloads: in the frames probed (100, the rescale column)
  the first gain change came at samples 169 to 243, inside the long training field, which
  is the case ADR 0007 already names (a step there puts the timing a symbol off). A probe of
  the control alone (8 frames, no CFO, the offset on I) had its first change at sample 6:
  whether the peak comes early depends on how signal and offset add. The power
  detector of a real control must not see the offset, or the lock must be left on less
  than 10 dB; the published state machine does neither by itself.
- **Without a hold on the gain the model's control leaves the lock inside the packet** on a
  peak of the data: in 2 to 4 % of the 54 Mb/s frames (3400 samples; 8 to 9 % with a
  per-index offset of −20 dBFS) and 37 to 60 % of the 6 Mb/s ones at 30 and 60 dB (27 000
  samples; none at 10 dB, where the gain does not move). The measurement holds the gain
  from the first lock inside the packet, which no receiver here does yet (`channel.md`,
  R0-07e, open item 3; the column `held` of the report). The fixed 0x5a payload of
  `rxfront agcmodel` did not show it.

**What this is not.** No level of the AD9361's residual offset is published (`channel.md`),
so the rows are a tolerance, not a prediction: which row R1's captures land in decides
whether the blocker needs a control at all. The receiver is the float one; the fixed
blocker rounds and the fixed OFDM receiver (R0-09 G2) does not exist yet. The phase jumps
and the hold on the gain are assumptions. One seed per cell. The columns of a row are the
same frames, so a difference inside a row can be real where the totals look alike; between
rows a difference of a few frames is noise. `dc_block_told` and `dc_block_frozen` in the
tool have no test of their own (the reviewer read their indexing against the trace).

**For ADR 0008** (inputs, nothing decided here): (1) the decoder's samples come after a DC
removal; (2) if the gain control tells the blocker anything, "the gain changed" (restart)
covers both kinds of offset and "by this much" (rescale) is the cheaper one for the offset
before the gain; a hold is not worth a wire; (3) the receiver must hold the gain once it
has acquired; (4) the gain control's power measurement needs the offset removed before it;
(5) the cost of the blocker's own pole on 64-QAM near its sensitivity (8 of 2000 at 25 dB)
wants a look with R0-09 G2.

**Pre-PR review (fresh agent, the diff and the report), 2026-11-19**, on the first run,
in which the gain control did not see the offset. It reran the tool instrumented (same
seeds, identical output) and found no defect in the indexing, the units, the random
streams or the accounting, and the table equal to the report. Found and corrected: the
control ran on the signal without the offset (now `FastAttackAgc::run_dc`; the run above
is the new one, and the finding on the blinded control exists because of it); "at 25 dB
the gain barely moves" was wrong (one −5 dB change at the lock, about sample 74); "less
than the run can resolve" ignored that the columns are paired; the hold also hurts with a
per-index offset; two cells, not one, had timing misses; `free/raw` equals "no blocker"
only at 54 Mb/s; the 38 % at 6 Mb/s was for 30 and 60 dB; `rescale(−∞)` zeroed the
estimate against its own comment; five mutations of the reviewer's survived (tests added,
`channel.md`); the usage line lacked the command; hold-then-restart restarted only when
the lock itself changed the gain. The five UG-570 quotes in `channel.md` matched the
document's text and pages. Not checked by the reviewer: the 6 Mb/s rows beyond the first,
`scripts/check.sh`, the listed mutations, the register remarks.

**Open**
- The same through the fixed receiver, with R0-09 G2 (`BACKLOG.md`, R0-07e).
- Paired counts printed by the tool; a slower pole after acquisition; a restart at 2⁻⁵ or
  2⁻⁶ at each change (only after a hold was tried here).
- The chip's own tracking in front of ours (R1): its transient at a gain change is what the
  per-index rows stand in for.

## 2026-11-20 · note · the same through the fixed receiver

Done with R0-09 G2, stage B, second part: `docs/gates/rx_ofdm.md` (2026-11-20, `rxagc`,
`reports/rx_ofdm_agc_*.txt`). For this journal's open items: the integer chain follows the
float one under every DC offset measured; the blocker's own pole in the decoder's path
("8 of 2000 at 25 dB" above) is far larger with a sample-clock offset in the scene, near the
sensitivity and through fading (54 Mb/s: 403 frames lost against 167 in AWGN at 20 dB, a
floor of 13 % through TGn D), and a slower pole does not cure it. `restart` and `rescale`
were not rerun in integers.

## 2026-11-21 · G0 · R0-08d (2): what the decoder is handed (for ADR 0008)

**Question.** `rx_ofdm.md` (2026-11-20) found that the decoder can be handed neither the DC
blocker's output (it costs 54 Mb/s 0.9 dB and a 13 % floor through TGn D with nothing to
remove) nor the samples before it (an offset 20 dB below the packet loses a fifth of the
frames), and left the cause as a guess. Two things to settle before ADR 0008: is the
blocker's cost the CFO's, and is there a removal that costs nothing when there is nothing
to remove.

**The candidate.** Behind a CFO of ω per sample the long training field repeats after 64
samples turned by φ = 64·ω; a DC offset D of the receiver repeats unturned. For every n of
the first long symbol's FFT window

  x[n + 64] − e^(jφ)·x[n] = D·(1 − e^(jφ)),

whatever the channel (the field stays periodic through any linear channel shorter than
GI2) and whatever the transmitter leaks at its own carrier (that is part of the periodic
field). The estimate is the mean over the 64 pairs divided by 1 − e^(jφ). Once the CFO is
out the offset is a tone φ/2π subcarrier spacings from the unused subcarrier: where
|1 − e^(jφ)| is small the division magnifies the noise, and there the tone does no harm.
Below |1 − e^(jφ)| = 1/16 (a CFO of 3.1 kHz at 20 MHz, the tone within 0.01 of a spacing)
zero is returned. The error of the estimate, as a tone, reaches subcarrier k with a power
of 1/(2π²(k − δ)²) of that subcarrier's noise, δ = φ/2π: bounded however small the divisor
(0.2 of the noise on the nearest subcarrier at δ = 0.5, 0.75 at δ = 0.74), a few per cent
on the next ones.

**Interface and formats.**
- Float: `rx_front::ltf_dc_offset(x, fft_start, cfo_hz, fs) -> Option<Sample>`;
  `DC_MIN_TURN` = 1/16. `None` when the stream does not hold the 128 samples.
- Integers: `rx_front_fixed::ltf_dc_offset_fixed(x, fft_start, cfo_phase, fx) -> Option<IQ>`
  on the ADC's 12-bit samples. e = the front's rotator (`nco_table`, L = 9, Q = 11,
  amplitude A = 1023) at the phase 64·`cfo_phase` with half a table address added before it
  is cut to the address; d = A − e; S = Σ (x[n + 64]·A − e·x[n]) exactly (below 2²⁹);
  D = S·d*/(64·|d|²), each component rounded half to even and saturated to 12 bits, in ADC
  LSB. Zero where 256·|d|² < A². One division per packet (a 42-bit by a 28-bit number).
- `rx_front_fixed::remove_dc_fixed(x, D) -> Vec<IQ>`: x − D, saturated to the 13 bits of
  the rotator's input (two 12-bit numbers never reach the limit). `correct_fixed` then
  turns it as it turns the blocker's output today.
- The acquisition (detector, CFO, timing) still runs on the blocker's output. Nothing of
  `RxFrontFixed`, `hw/rtl` or `hw/vectors` changes.

**References.** No outside reference exists for this estimator; the gates are the
definition worked by hand on single samples and the identity above on fields that are not
ours (random 64-periodic sequences behind random CFOs), which is a consistency gate of the
model and is recorded as such. Its worth is measured as packet error rate
(`rxagc cfo`, `cand`).

**Budget (G2).** The integer estimate against the float one on the same ADC samples:
within half an LSB (its rounding) plus the table's address rounding (π/512 of phase) times
the field's own level over |d|. Through the receiver: frames lost by the integer chain with
this estimate against the float chain with the float estimate, paired, as stage B's.

## 2026-11-21 · G1/G2 + measurements · R0-08d (2): the DC offset from the long training field

**Tests first.** The eight tests below were written against the planned API and failed (the
test files did not compile: `ltf_dc_offset`, `ltf_dc_offset_fixed`, `remove_dc_fixed`
unresolved) before the functions existed. One expectation of mine was wrong and was
corrected with the implementation: a phase exactly between two table addresses goes up
(half an address is added, then the phase is cut), so −5.5 addresses is −5, inside the
guard, where I had written it symmetric.

**Gates** (`cargo test -p radio-phy --test g1_rx_front --test g2_rx_front`, rustc 1.98.1,
commit 2ff20a4: 31 and 19 pass; after the review, b65c6d1: 31 and 20):
- `g1_rx_front`: `ltf_dc_offset_worked_by_hand` (single samples at φ = π and ±π/2, the
  window's four edges), `..._is_left_alone_where_the_cfo_is_too_small_to_divide_by` (both
  sides of 3109.0 Hz, a constant stream as its own offset, the stream's length),
  `..._is_exact_behind_a_repeating_field` (200 random fields, CFOs and offsets, to 10⁻¹¹).
- `g2_rx_front`: `ltf_dc_offset_fixed_worked_by_hand` (the same by hand in integers, ties
  to even of either sign, the CFO word negative and wrapped), `..._guard_and_limits`
  (addresses 5 and 6 of the table by hand, the tie, full scale, saturation, the stream's
  length, an invalid configuration), `remove_dc_fixed_worked_by_hand`,
  `ltf_dc_offset_fixed_follows_the_float_one` (600 ADC streams of the preamble, AWGN and
  TGn D at 10, 20 and 35 dB, offsets up to ±300 LSB: against the float estimate within the
  budget above and against the offset put in within five standard deviations of the
  noise's reach; both bounds are approached to within a factor of twenty),
  `the_ltf_estimate_takes_a_dc_offset_out_of_the_decoders_samples` (frames at 54 and
  6 Mb/s, ±31 / 27 / 20.7 ppm at 5.8 GHz, an offset 8 dB below the frame: decoded at the
  three hand-off widths with the estimate taken out, not decoded at 54 Mb/s with it left
  in).

**Mutations** (30, each run against the two test files, the source restored with a copy and
a touch; all fail a test): float — the sign of φ; the guard as |d|² < 1/16; the guard
removed; the window one sample late; one sample short; 63 in the divisor; d in place of
its conjugate; the turned sample added; the guard at 1/32. Integers — no half address; the
guard at 128 and at 512; half up and half down in the division; the upper limit 2048; the
lower −2047; 63 samples of phase; d's imaginary part unsigned; the turned sample added
(each component); 32 in the divisor; the window one sample late; d in place of its
conjugate; `remove_dc_fixed` adding; its limits 4096 and −4095; the Q offset taken from I;
the configuration's check removed; the rotator's phase negated; A + 1.
Not killed because equivalent: the CFO word reduced modulo 2^P before the multiplication by
64 (the rotator reduces again; the reduction only keeps the product inside 64 bits).

**Measurements** (`rxagc`, 2000 frames of 1000 octets per row, the scene of `rx_ofdm.md`'s
stage B part 2: the gain-control model, ±40 ppm at 5.8 GHz, 12-bit ADC; a frame's random
stream from the row and the frame's number).

**1. The blocker's cost is the CFO's** (`rxagc cfo` → `reports/rx_front_dc_cfo.txt`, binary
of 2ff20a4, run locally; no DC offset; 54 Mb/s; the clock offset's size in the band of the
row; frames lost of 2000):

| Clock offset | AWGN 20 dB: before the blocker / blocker 2⁻⁷ / LTF estimate (float) | TGn D 26.5 dB | TGn D 40 dB |
|---|---|---|---|
| 0 to 2 ppm | 189 / 190 / 188 | 213 / 213 / 215 | 0 / 0 / 0 |
| 2 to 10 ppm | 186 / 188 / 192 | 236 / 231 / 235 | 0 / 0 / 0 |
| 10 to 20 ppm | 189 / 208 / 192 | 231 / 241 / 237 | 0 / 3 / 0 |
| 20 to 30 ppm | 178 / 268 / 179 | 235 / 483 / 237 | 0 / 195 / 0 |
| 30 to 40 ppm | 163 / 919 / 166 | 223 / 1134 / 227 | 0 / 912 / 0 |

- Below 10 ppm (58 kHz) the blocker in the decoder's path costs nothing; from 20 ppm on it
  is the largest loss in the receiver. AWGN at 25 dB: 0 / 0 / 0 / 1 / 112 lost by band,
  none before the blocker. The integer chain's columns say the same (report).
- The guess of `rx_ofdm.md` holds as far as this test goes: the dependence on the CFO is
  measured, the mechanism (the subcarrier next to DC on the skirt of a high-pass whose
  memory is longer than a symbol) is the explanation that fits and was not isolated further.
- The LTF estimate with nothing to remove is the samples before the blocker within the
  run's noise in every band (paired: at most +17 −14).

**2. The estimate from the long training field removes the offset and costs nothing**
(`rxagc cand` → `reports/rx_front_dc_cand.txt`, binary of 2ff20a4 built and run on the build
server; frames lost of 2000; float before the blocker / integer chain as it is (13 / 2,
blocker 2⁻⁷) / float with the LTF estimate / integers 12 / 2 with it / integers 13 / 2 with it):

| DC offset | AWGN 54 Mb/s 40 dB | AWGN 54 Mb/s 20 dB | AWGN 6 Mb/s 3.5 dB | TGn D 54 Mb/s 26.5 dB | TGn D 54 Mb/s 40 dB |
|---|---|---|---|---|---|
| none | 0 / 12 / 0 / 0 / 0 | 167 / 412 / 162 / 167 / 170 | 213 / 210 / 222 / 220 / 217 | 226 / 517 / 232 / 231 / 230 | 0 / 251 / 0 / 0 / 0 |
| per index −40 dBFS | 0 / 11 / 0 / 0 / 0 | 231 / 416 / 163 / 177 / 179 | 256 / 217 / 219 / 223 / 222 | 334 / 515 / 230 / 228 / 231 | 42 / 250 / 0 / 0 / 0 |
| per index −30 dBFS | 377 / 19 / 0 / 0 / 0 | 1207 / 403 / 162 / 172 / 170 | 1001 / 215 / 221 / 214 / 220 | 1231 / 535 / 229 / 231 / 228 | 978 / 264 / 0 / 0 / 0 |
| per index −20 dBFS | 1636 / 98 / 0 / 0 / 0 | 1844 / 808 / 453 / 460 / 458 | 1801 / 215 / 219 / 224 / 223 | 1838 / 771 / 235 / 235 / 241 | 1744 / 418 / 0 / 0 / 0 |
| before the gain −20 dBFS at the largest | 0 / 36 / 0 / 0 / 0 | 1834 / 636 / 417 / 427 / 434 | 1810 / 216 / 219 / 216 / 220 | 1540 / 544 / 238 / 235 / 238 | 125 / 283 / 0 / 0 / 0 |
| before the gain −10 dBFS | 386 / 215 / 0 / 0 / 0 | 1989 / 1778 / 1743 / 1741 / 1741 | 1927 / 212 / 220 / 220 / 220 | 1905 / 1027 / 451 / 457 / 457 | 1047 / 642 / 0 / 0 / 0 |

- Under every offset of the table the chain with the LTF estimate loses what it loses with
  no offset, except where the gain control itself is disturbed: AWGN 20 dB under the
  offsets at −20 dBFS (453 and 417 against 162: the gain changes in 0.39 of the frames
  where it otherwise does not, report column `chg`), and the offset before the gain at
  −10 dBFS at 20 and 26.5 dB, the blinded control of the 2026-11-19 entry (at 25 dB: 842 of
  2000 in every LTF column). Which of these the acquisition loses is in 4 below (I first
  wrote "lost before the decoder" here without having separated them; at 20 dB that was
  wrong).
- The integer estimate (whole ADC LSB) follows the float one: paired differences of the
  13 / 2 column against the float LTF column are at most +23 −13 in the rows above and in
  the size of stage B's own integer-against-float differences.
- With the noise at the largest gain lowered to −48 and −54 dBFS (report, last two
  sections) the 13-bit column with the estimate loses 214 and 254 at 6 Mb/s against the
  float column's 214 and 238, and 196 and 224 against 186 and 199 at 54 Mb/s: the lower
  edge of `rx_ofdm_agc_level.txt`, not moved by an estimate in whole LSB. The 12-bit
  hand-off loses 240 and 336, 211 and 272 there.
- 6 Mb/s at 10 dB loses no frame in any column but the unremoved one.

**3. The gain held by the receiver instead of by the control's first lock**
(`rxagc hold` → `reports/rx_front_dc_hold.txt`, binary of b65c6d1, local; first run with 6349c95): held from 479
samples after the packet's first (T1 + 223 + 64, the latest declaration) with the control
free until then, the float chain with the LTF estimate loses 2, 1, 5 and 0 frames of 2000
at 54 Mb/s in AWGN at 25, 30, 40 and 60 dB, 2 and 4 through TGn D at 40 and 60 dB, where
it loses none with the assumed hold; none at 6 Mb/s. The integer 13 / 1 column is the same
frames. 530 to 546 of the 2000 6 Mb/s frames would leave the lock later without any hold.

**4. What `cand` left out** (`rxagc edge` → `reports/rx_front_dc_edge.txt`, binary of
a562eb1, local; asked for by the ADR's reviewer; float before the blocker / float blocker /
float LTF / integers 13 / 1 LTF, then frames the float and the integer acquisition lost: none,
or an FFT start more than 16 samples off):

| Row | lost | not acquired |
|---|---|---|
| AWGN 54 Mb/s 20 dB, no offset | 167 / 403 / 162 / 172 | 0 / 0 |
| …, per index −20 dBFS | 1844 / 801 / 453 / 460 | 0 / 0 |
| …, before the gain −20 dBFS | 1834 / 626 / 417 / 435 | 0 / 0 |
| …, before the gain −10 dBFS | 1989 / 1778 / 1743 / 1743 | 0 / 0 |
| AWGN 54 Mb/s 25 dB, before the gain −10 dBFS | 1933 / 1115 / 842 / 843 | 508 / 508 |
| AWGN 54 Mb/s 40 dB, per index −30 / −20 dBFS | 377 / 19 / 0 / 0 and 1636 / 99 / 0 / 0 | 0 / 0 |
| clock offset 0 to 2 ppm, 54 Mb/s 40 dB, none / −30 / −20 dBFS | 0 / 0 / 0 / 0 in each | 0 / 0 |
| …, 54 Mb/s 20 dB | 174 / 173 / 178 / 179, 181 / 175 / 178 / 176, 499 / 509 / 470 / 471 | 0 / 0 |
| …, 6 Mb/s 3.5 dB | 187 / 188 / 190 / 195, 191 / 187 / 189 / 187, 225 / 192 / 190 / 193 | 0 / 0 |
| 54 Mb/s 20 dB, noise at −48 / −54 dBFS, no offset | 179 / 402 / 186 / 196 and 200 / 406 / 199 / 227 | 0 / 0 |
| 6 Mb/s 3.5 dB, noise at −48 / −54 dBFS, no offset | 212 / 219 / 214 / 221 and 233 / 230 / 238 / 276 | 0 / 0 |

- The 13 / 1 hand-off with the estimate is the 13 / 2 one of `cand` within a few frames,
  under an offset and at the lower edge.
- **The frames lost under the −20 dBFS offsets at 20 dB are not lost by the acquisition**
  (none of 2000 not acquired or mistimed, in either chain). At 20 dB the control has no
  lock inside the packet, so no hold starts, and the offset provokes gain changes (`chg`
  0.39 to 0.92 against 0.00) that fall somewhere in the frame; which of them cost the frame
  was not separated. Only the blinded control at 25 dB loses acquisitions (508 of the 842).
  So nothing here asks for a control on the DC blocker in the acquisition's path.
- With the clock offset within 2 ppm, where the estimate is mostly or wholly left out, the
  offset does no harm even unremoved at 40 dB, and the chain with the estimate is level
  with the float one.

**Reviews (two fresh agents, 2026-11-21).**
*The code and G0* (not the results or the ADR), on 6349c95. Confirmed by rederivation: the
identity, the sign against `correct_fixed` and `cfo_phase`, the rounding, the widths at the
default configuration, every hand-worked number, the bound 1/(2π²(k − δ)²) within ±40 ppm,
the tool's pairing of frames, `ppm_band`, `hold_after`. Found and corrected:
- **a defect:** the integer estimate's products overflowed 64 bits for rotators the
  configuration allows (Q from 24 with 12-bit samples, from 22 with full 16-bit ones): a
  panic in library code. The products are 128 bits wide now; the default's results are
  the same integers (reports unaffected); test `..._off_the_default_configuration`;
- six surviving mutants of its twenty: the float guard on the angle instead of the chord,
  and zero above 300 kHz (no test went near a full turn; G0's "the tone does no harm" was
  false there, and the doc now says the offset is left in on a subcarrier at a CFO of one
  spacing, outside the standard's range); the guard's constant at 200 and the half address
  hard-coded (nothing tested another table: L = 12 by hand and L = P now); the guard
  before the length check in either model (a short stream with no CFO now tested). All six
  fail a test now, with two of mine (the widening cut to 32 bits; R1 to R7 of the rerun);
  one of mine survives, half an address of 1 added at L = P, a phase unit of 2⁻²² of a
  turn that no test stream can see;
- the bound of `ltf_dc_offset_fixed_follows_the_float_one` was loose (worst 0.44 of it, a
  2 LSB error would have passed) and had a term G0 did not: replaced by the reviewer's
  derivation from the samples, 0.5 + 1.1·(π/512 + √2/1023)·|mean(T2) − mean(T1)|/|d|²,
  approached to within a factor of two;
- wording: the integer guard's edge is the table's (CFO word 704, 3.36 kHz) against the
  float 3.11 kHz, so the two chains decide differently in about 0.1 % of frames;
  `remove_dc_fixed`'s "never reach"; `hold`'s header (the hold is at a fixed sample, not
  the chain's own acquisition; the general header said "held from the first lock" above
  rows that are not; rerun with the corrected text, same numbers).
- Its instrumented `hold` run: see ADR 0008, decision 2.
Not checked by it: the `cfo` and `cand` reports as reruns, the TGn D rows of `hold`,
release-mode behaviour of the overflow, the RTL feasibility of the division.

*ADR 0008's first draft*, on a4589ad: recorded at the end of the ADR.

**What this is not.** One seed per row; `cand` was run once (on the server), `cfo`, `edge`
and `hold` locally (`hold` twice, identical rows). The offset is constant through a packet at a held gain by
construction of the model; a real front end's drift and its transient at a gain change are
not in it. The estimate uses the true model of the field (two identical symbols): a gain
change or a phase jump inside the long training field breaks it and was not separated
here. The hand-off measured with the estimate is 12 / 2 and 13 / 2 in `cand` and 13 / 1 in
`hold`; ADR 0008 proposes 13 / 1 on `rx_ofdm_agc_level.txt`'s evidence that the two 13-bit
ones do not differ. TGn B and E and other frame lengths were not run. The streaming front
(`RxFrontFixed`) does not hold the second history or the replay yet.

**Open**
- The founder's decisions on ADR 0008.
- `rxper` and `FixedOfdmConfig`'s defaults moved to the proposed hand-off, with the G2 rerun.
- The estimate and the replay inside `RxFrontFixed` as a streaming model, then vectors.

## 2026-11-21 · G2 · ADR 0008's hand-off as one function of the model

Commit: the one carrying this entry, on 613d358 (ADR 0008 accepted). Command:
`cargo test -p radio-phy --test g2_rx_front` (23 pass). First of the model follow-ups the
ADR names; the decoder's side (its default input, `rxper`, the G2 rerun) is in
`rx_ofdm.md` under the same date.

**What was added.** `handoff_fixed(x, fft_start, cfo_phase, fx)`: decision 1's stream as one
function, `correct_fixed(remove_dc_fixed(x, ltf_dc_offset_fixed(x, fft_start, cfo_phase)),
cfo_phase, Y_BITS)` — the ADC's samples less the offset estimated from the two long training
symbols, the CFO word taken out by the front's rotator, 13 bits with the ADC's LSB kept.
`None` when the stream does not hold the two symbols or the widths are unusable. It rotates
the whole stream from its sample 0; where the hardware's replay starts (and so the first
sample's phase) belongs to the streaming model and is not fixed here.

**Tests, written before the function** (the import failed to compile, then):
- `handoff_fixed_worked_by_hand`: a CFO word of 0 (inside the estimate's guard, the rotator
  at (1023, 0)): v·1023/1024 half to even at 1, 511, 512 (the tie), 513, 2047 and −2048,
  zeros elsewhere; a word of 2¹⁸ + 2¹⁵ (half a turn between the two symbols, so the estimate
  is the offset exactly): a stream that is the offset (100, −50) alone comes out as zeros,
  one sample 1024 above it at index 32 as (0, −1023) and at index 192 as (−1023, 0); the two
  symbols taken eight samples early see that sample and no longer return zeros; a stream one
  sample short, and an invalid table, return nothing.
- `handoff_fixed_is_the_estimate_the_subtraction_and_the_rotation`: the three steps one by
  one on seven words (both sides of the guard's edge, ±40 ppm at 5.8 GHz, beyond half a
  turn) and samples over the ADC's whole range, with outputs beyond 12 bits where an offset
  was taken off.
- `the_default_receiver_decodes_the_hand_off`: `acquire_fixed`, `handoff_fixed` and
  `decode_ppdu_fixed` at their defaults on 54, 36 and 6 Mb/s frames (300 octets, 30 dB,
  +33, −12.5 and −38 ppm, a DC offset of −20 dBFS at the ADC). This one passed as soon as
  the function existed, at the old default input too (a 12-bit input clips nothing at
  −12 dBFS): it shows the chain whole, not the width. With decision 7's start
  (`rx_ofdm.md`, same date) the tracker is started from the acquisition's CFO word and
  nothing between the ADC and the PSDU is in floating point; the test then also holds the
  sign to the clock: the last symbol's tracked timing is ε·(112 + 80·i) samples within
  0.03 (−0.31 at the end of the 6 Mb/s frame), and with the word negated it is not.

**Mutations** (each built clean, the file restored by `cp` and `touch`; all six fail):
no offset removed (the three tests); 12 bits out (by hand, steps); the rotation the other
way (the three); `fft_start + 1` in the estimate (by hand, steps); the estimate with the
word negated (steps, the chain); a short stream not refused (by hand).

**Open**
- The second history and the replay inside `RxFrontFixed`, with the phase of the replay's
  first sample; then the hand-off's vectors.
- The gain control's timer and hold input (`FastAttackAgc`).

## 2026-11-21 · measurements · decision 2 as a closed loop: the control's timer and the front's hold

Commit: the report from the commit before this entry's (the tool as reviewed). Command:
`rxagc loop 2000` → `reports/rx_front_agc_loop.txt` (locally, 10 threads, about 15
minutes). The model's side (`lock_hold`, `lock_hold_at_max`, `run_held`) is in `channel.md`
under the same date.

**What the loop is.** Per frame and per column: the gain control runs on the stream; the
ADC's samples are formed from its trace; the integer front (`RxFrontFixed`) declares an
acquisition; a hold is raised at that sample (or 40 or 400 samples later) and the control
is run again with it; the decoder (`handoff_fixed`, `decode_ppdu_fixed`, tracker from the
CFO word) reads SIGNAL. A SIGNAL that decodes drops the hold at the end its length gives
(4 samples of back-off after the last FFT window), and a frame counts only if it decodes
from the stream with the hold dropped there; a SIGNAL that fails drops the hold 180
samples after its symbol (decision 3's 9 µs) and the front searches again from there. A
hold that would act only after its own end is not raised. A released control returns to
the largest gain and searches. Gains and phase jumps are absolute and a jump is drawn from
its change's own sample, so the samples before a decision do not depend on what is decided
after it (the first version of the tool referred them to the last gain and did not have
this property; the review found it, below). The first nine rows are the frames of `hold`;
the others are new.

**Results** (frames lost of 2000; in brackets, where not 0, the frames whose gain changed
after the short training field and more than 16 samples before the frame's end):

| Row | from the lock (the old assumption) | timer on every lock + front | timer below the largest gain + front | front alone | that timer alone | nothing | timer below + front, hold 40 late | 400 late |
|---|---|---|---|---|---|---|---|---|
| AWGN 54 Mb/s 25 dB | 0 | 0 | 0 | 0 | 61 (61) | 67 (67) | 0 | 5 (5) |
| AWGN 54 Mb/s 30 dB | 0 | 0 | 0 | 1 (1) | 68 (68) | 70 (70) | 0 | 7 (7) |
| AWGN 54 Mb/s 40 dB | 0 | 0 | 0 | 4 (4) | 55 (55) | 61 (61) | 0 | 8 (8) |
| AWGN 54 Mb/s 60 dB | 0 | 0 | 0 | 0 | 60 (60) | 60 (60) | 0 | 14 (14) |
| AWGN 6 Mb/s 30 dB | 0 | 0 | 0 | 0 (2) | 0 (528) | 0 (530) | 0 | 0 (6) |
| AWGN 6 Mb/s 60 dB | 0 | 0 | 0 | 0 (2) | 26 (543) | 24 (546) | 0 | 0 (3) |
| TGn D 54 Mb/s 40 dB | 0 | 0 | 0 | 2 (2) | 85 (85) | 87 (87) | 0 | 12 (12) |
| TGn D 54 Mb/s 60 dB | 0 | 0 | 0 | 3 (3) | 64 (64) | 68 (68) | 0 | 8 (8) |
| TGn D 6 Mb/s 30 dB | 0 | 0 | 0 | 0 | 1 (536) | 1 (539) | 0 | 0 (9) |
| AWGN 6 Mb/s 5 dB | 2 | 2 | 2 | 2 | 2 | 2 | 2 | 2 |
| preamble alone, then 54 Mb/s 30 dB | — | 1991 | 478 | 479 | — | — | 478 | 483 |
| preamble alone, then 6 Mb/s 30 dB | — | 503 | 503 | 503 | — | — | 503 | 503 |
| frame, SIFS, 54 Mb/s 30 dB | — | **2000** | 0 | 3 (3) | — | — | 0 | 2 (2) |
| frame, SIFS, 6 Mb/s 30 dB | — | **2000** | 0 | 0 (2) | — | — | 0 | 0 (4) |
| noise −5 dBFS: 54 Mb/s 30 dB | 0 | 20 | 19 | 2 (2) | 65 | 51 | 19 | 22 |
| noise −5 dBFS: preamble alone, then 54 Mb/s | — | 1577 | 1577 | 499 | — | — | 1576 | 1725 |
| noise −5 dBFS: frame, SIFS, 54 Mb/s | — | **2000** | **2000** | 1 (1) | — | — | 2000 | 1978 |

("—": these columns try the stream's first acquisition only and lose every such frame by
construction. "preamble alone": the frame's two training fields, nothing behind them, 280
to 330 samples before the frame. "frame, SIFS": a whole frame of 100 octets at the same
rate and level, the counted frame 320 samples after its end. "noise −5 dBFS": the noise at
−5 dBFS at the largest gain, above the lock level of −10, so the control locks on it below
the largest gain; elsewhere it is at −30 dBFS.)

1. **A timer on every lock (ADR 0008's first wording) loses every frame that follows
   another after a SIFS.** The hold is dropped at the first frame's end, the control
   returns to the largest gain and locks on noise 26 samples later; the next frame comes
   320 samples after the end, 218 samples before the timer runs out, and its overload is
   not answered until then: 2000 of 2000 lost at 54 and at 6 Mb/s. After a preamble alone
   the same happens within a few µs of the drop (1991 at 54 Mb/s; 6 Mb/s survives the
   clipping of its first samples there).
2. **A timer that spares a lock at the largest gain cures that only while the idle noise
   locks the control at the largest gain.** At −30 dBFS: 0 lost after a SIFS and 478 after
   a preamble alone (finding 4). At −5 dBFS the lock on noise is below the largest gain,
   is kept, and the rows are those of the first timer: 2000 and 1577; even the ordinary
   row loses 19 against 2.
3. **The front's hold alone, without any timer, is the one column that holds in every
   row:** 0 to 4 of 2000 at 54 Mb/s in the ordinary rows, each a frame whose gain changed
   between the control's lock and the declaration (it was 0 to 5 with a fixed sample in
   `hold`), none at 6 Mb/s; 3 and 0 after a SIFS; 2, 499 and 1 at the loud noise.
   **The founder decided on this (2026-11-21, a second amendment the same day): no timer;
   ADR 0008's decision 2 is its step 2 alone.**
4. **After a preamble with nothing behind it a quarter of the frames are lost in every
   column:** 478 to 503 of 2000. The SIGNAL read there is noise; about 1500 fail (parity,
   rate or length) and drop the hold, the rest pass and hold the gain and the front for
   the length they claim. One parity bit and eight valid rates of sixteen give
   2000/4 = 500 ± 19. The decoder does not check SIGNAL's reserved bit (IEEE 802.11-2020 §17.3.4 says it
   is 0): checking it would halve this. A receiver that restarts on a stronger packet
   would remove it; not designed.
5. **Without the front's hold** 55 to 87 of 2000 are lost at 54 Mb/s — every frame whose
   gain changed inside the data. At 6 Mb/s a quarter of the frames have such a change and
   nearly all survive it (0 to 26 lost).
6. **How late the hold may act** (measured with the timer of finding 2, at −30 dBFS):
   40 samples (2 µs) late costs nothing; 400 (20 µs) late 5 to 14 of 2000 at 54 Mb/s.
   Without a timer it was not run: the cost of a late hold there is at least finding 3's
   and grows with the delay.
7. The ten ordinary rows could not show finding 1: the control's first lock on noise is at
   sample 26 and the packet comes after 600, when its timer has run out.

**Independent review (fresh agent, the code and the G0 text, not the conclusions),
2026-11-21,** on the first version of the tool and model (815eec2). The model matched G0
at every boundary it probed and the hand-worked numbers were right. Found and corrected:
- *the tool's ADC stream was not causal* (gains and phases referred to the last gain: a
  later hold rotated and rescaled every earlier sample, so the stream that was decoded was
  not the one the acquisition was found on; it measured 150 of 206 re-acquisitions
  differing). Absolute gains and phases now; the columns that share the physics of a row
  agree exactly where the first report had 473 / 477 / 490 / 500;
- the first ADR amendment said the frame came 15 µs after a dropped hold (it was 1.2 to
  3.7 µs) and extended the finding to "every ACK and every next frame of a burst" without
  the case having been run: the "frame, SIFS" rows are that case, run;
- "a lock at the largest gain" is "a lock on noise" at one noise level only: the "noise
  −5 dBFS" rows, which changed the decision;
- the hold's end from SIGNAL was never applied to a frame that counted, and was 4 samples
  (the back-off) early: applied now, with the back-off;
- "nothing here reduces it": the reserved bit (finding 4);
- model: a release that coincides with the stream's end was not recorded; no test with a
  timer without end, with an offset per gain across a release, with an overload on an
  unkept lock's first sample, or with a quiet signal after a hold from sample 0 (five of
  its twenty mutants survived; all fail now, `channel.md`).
It also confirmed by instrumenting single frames that the loss in the preamble rows is the
kept lock on noise (the whole frame received at the largest gain, a third of its samples
clipped), that the columns do not share a code path, and that `hold`, `cfo` and `edge` give
byte-identical output before and after `stream()` was taken out of `one()`.
Not checked by it: runs above 300 frames, the TGn rows under instrumentation, the other
commands' outputs, the AD9361's documents.

**What this is not.** One seed per row, one run. The timer and the release's behaviour (to
the largest gain, the search from scratch) are assumptions about a radio, not the AD9361's
documented behaviour. The watchdog at the longest PPDU never acts: a length SIGNAL can
carry ends before it. The front between a drop and the next search is the streaming
model's whole-stream run filtered by the detection's sample, not a front restarted at the
drop. No DC offset; two noise levels; frames of 1000 octets; TGn B and E not run; the
bursts are one gap and the frame's own level each. The first frame of a "frame, SIFS" row
is not counted. The float chain is not in the loop. `rxagc`'s other commands still hold
from the control's lock and start the tracker through the float conversion; their reports
were not rerun.

**Open**
- The second history and the replay inside `RxFrontFixed`; the hold's raise and drop as
  outputs of the streaming front (they are the tool's here).
- SIGNAL's reserved bit in the decoder (float first, then integer): its own gate.
- At R1: how the radio's hold pin acts and how fast; a late hold without a timer.
- A restart of the receiver on a stronger packet while it holds on a false SIGNAL.
- A timer that ignores only the data's own peaks and still answers a large overload was
  not modelled; the 0 to 4 frames of 2000 are what it could win back.

## 2026-11-21 · G0 + model gate + measurement · the streaming hand-off: second history, replay, hold

Commits cae398e (first version) and the one carrying this entry (after the review below),
on 128c559 (after #57 and #58). Commands:
`cargo test -p radio-phy --test g2_rx_handoff` (12 pass), `--test g2_rx_ofdm` (27 pass),
`--test g2_rx_front` (23 pass);
`cargo run -q --release -p radio-tools --bin rxper -- stream 2000` →
`reports/rx_front_stream.txt` (2.5 min on 10 cores).

**G0 — `RxHandoffFixed`** (ADR 0008, decisions 1 to 3; what the ADR left to "the streaming
model" is fixed here and marked *new*). `RxFrontFixed` is inside it unchanged: its
acquisitions, traces and G3 vectors are the same.

- **Second history:** the ADC's samples as they come, 512 deep (`ADC_HISTORY`), written
  every sample.
- **Opening:** at the sample an acquisition is declared the offset is taken from the 128
  samples of the two long symbols in that history (`ltf_dc_offset_fixed`, with the CFO word
  handed on), the hold is raised and a packet is opened with a tag (*new*: a counter of
  the packets opened, modulo 256). Its first beat comes `start_delay` periods later (20:
  the ≈ 160 clocks of the estimate at 8 clocks a sample; beyond some 230 periods T1's
  window has left the history and the stream is always lost).
- **Stream:** beats of one 13-bit sample each, `handoff_fixed` of the ADC's stream cut at
  T1's FFT window: *new* — the first sample is rotated by the phase zero, the phase steps
  back by the CFO word per sample. The first beat carries the CFO word and the tag. At most
  `rate` beats a sample period (8) and at most what the reader takes (`room`), never a
  sample the antenna has not given: eight a period until the reader has caught up, one a
  period after. Neither number changes a sample.
- **Verdict:** the decoder calls `signal(tag, Some(data symbols))` or `signal(tag, None)`
  (`decode_signal_fixed` gives it from the stream's first 208 samples). *New:* it counts
  only for the open packet of that tag, and only the first time: the verdict comes beside
  the stream and may arrive after a new acquisition has replaced its packet.
- **Hold:** covers the samples from the declaration's to the frame's nominal last, the one
  before T1's window + 208 + 80·N + the back-off (`end`). The level reported with a sample
  is the hold over that sample: it is down with sample `end`, whether the verdict came long
  before or in the last sample's period; a verdict that comes after sample `end` was pushed
  drops it at once, as `signal(tag, None)` does. Without a verdict the watchdog is
  N = 1366 (4095 octets at 6 Mb/s, 5.47 ms). The radio's delay is its own (`rxagc loop`:
  2 µs late costs nothing).
- **End:** *new* — the stream runs `HANDOFF_TAIL` = 8 samples past the hold's end and
  closes with an end mark, a beat of its own. The tail is for the decoder's timing
  tracker: from a transmitter 40 ppm slow the longest frame is 4.4 samples longer and the
  last window is moved 4 late; the default back-off of 4 just holds it, a front with none
  does not.
- **Marks:** end (`Packet`; `Stopped` after `signal(tag, None)`) and abort (`Acquisition`:
  a new declaration while a packet is open — its own stream follows and the hold stays up;
  `Overrun`: the next sample to hand on is 512 behind the newest — the hold drops).
  *New:* every opened packet ends in one mark, also one whose first sample was never
  taken, with these rules: a mark waits for a beat of room, and one that finds the mark
  before it still waiting is not sent (one register; the first stays: it closes the stream
  the reader was in); a packet whose last sample has been taken ends as `Packet` even if a
  new declaration comes before its mark is taken; a new declaration and an overrun in the
  same period give `Acquisition`. With `rate` 0 nothing is ever sent.
- The front goes on searching while a packet is open, as `RxFrontFixed` does; what it
  should search for then is still not decided (ADR 0008, decision 1).
- Timing is in sample periods; the clock-exact schedule is the RTL's.

**Gate.** The reference for the samples is `handoff_fixed` (gated by hand and against the
float chain above); pace, hold and marks are worked by hand from the rules. The tests were
written before the model and failed to compile; 7 of 10 passed on the first run of the
model, 3 failed on the tests' own expectations (the declaration is 274 to 277 samples
after T1's window in these scenes, not "some 290"; a fast transmitter moves the windows
early, a slow one late; an invalid configuration panicked, below). The tests for the
review's findings were written after it, with the corrections. Scenes: frames at −12 dBFS,
30 dB, ±12.5 to ±40 ppm, a DC offset of −20 dBFS.
- samples = `handoff_fixed(&adc[fft_start..], 0, word)` to the sample, the word and tag on
  the first beat only, the hold exactly [declaration, T1 + 208 + 80·N + 4), the mark in the
  last sample's period (three scenes); the same with a coarse weight of 15/64, where the
  word handed on is neither estimate, and at 6 dB in the frames where the offset's estimate
  differs between the two words;
- catching up by hand: with L samples waiting at the first push, eight beats for
  ⌊(L − 8)/7⌋ + 1 periods, then the rest, then one a period: 43 periods at L = 295
  (ADR 0008, decision 3, said 43 for 300); also at 3 beats a period;
- the same beats whatever the reader's pace (5 beats every third period; 1 a period with
  a start delay of 100);
- `signal(None)`: hold down with the call, `Stopped` alone in the next period, the next
  frame its own stream; a late verdict ends hold and stream at once; a verdict in the last
  sample's period and one a period later, to the sample; a second verdict changes nothing;
  a length of no symbols ends the packet at SIGNAL's window as a packet; the watchdog at
  N = 1366 (also for a verdict above it);
- a verdict with the tag of a packet a new acquisition has replaced, as a failure and as a
  length: the new packet's hold, stream and own verdict are untouched;
- a reader that takes nothing loses hold and stream 512 periods after T1; at half the
  antenna's pace the loss is at the period the hand count gives, to the sample; an end
  mark waiting 700 periods for room is not an overrun;
- two frames without a verdict: `Acquisition` in the period of the second declaration,
  the second stream 20 periods later with its own word and tag, one hold across both; a
  delivered first packet whose mark waited ends as `Packet`; a loss and a declaration in
  one period give `Acquisition`, a loss one period earlier `Overrun`; two streams lost to
  a stalled reader give one mark, and it is the first one's;
- the longest frame (4095 octets, 6 Mb/s, −40 ppm) decodes from the stream's samples with
  the same timing log as from the whole buffer; cut at the hold's end it decodes with the
  default back-off and not with none.

**Mutations** (cp + touch; `g2_rx_handoff` and `g2_rx_front` after each). First version:
32 of 32 failed. After the review, on the corrected model: 42 mutants, 41 failed and one
survived (the offset estimated with the fine word instead of the one handed on: no scene
told the two apart), which fails now with the 6 dB scenes — both comparisons of the hold's
end in `push` and in `signal`, the overrun's bound and its `read < last`, the stream's end
test, `read > n`, the first beat, the first phase, the tail (in the verdict and at
opening), the start delay, the rate, a free mark, the last mark winning, a close that
keeps the hold, the back-off, the symbol's 80, the clamp at 1366, the verdict's flag, the
tag ignored, the tag never counted, a delivered packet aborted, no abort at all, the word
and the offset from the fine estimate, no symbols as a stop, the overrun tested before the
declaration, `open_at`, the word's sign, a history of 256, SIGNAL's end at 200, a hold not
raised, the rotator's output width, five configuration guards. Not killable and left: no
mark for a replaced stream that was never read (no true frames declare twice within the
history's depth), and the validity guard in `push` (the front inside does nothing either).

**Also changed in the model.**
- `decode_signal_fixed`: SIGNAL alone from the two long symbols and its window, the head
  of `decode_ppdu_fixed` taken out as one function (`signal_with`; the length's second
  check there was a copy of `signal_from_soft`'s and is gone). Gate
  (`signal_alone_is_the_frames_signal_from_its_first_208_samples`): rate and length as
  transmitted for the eight rates and random lengths, equal to the whole decoder's; one
  sample short is `Truncated`; the caller's tracker places the window (moved one sample it
  needs 209); on noise its errors are the whole decoder's. Two of three mutants of the
  moved head survived at first (SIGNAL decoded as symbol 1; the tracker not stepped over
  SIGNAL) — a gap older than this change: nothing pinned the first entries of the timing
  log. `the_timing_log_starts_at_signal` does (ε·(112 + 80·i) by hand for a tracker that
  applies 40 ppm); 4 of 4 fail now (the fourth: the caller's tracker ignored).
- **A panic found by the new gate:** `RxFrontFixed::new`, `nco_table`, `nco_rotator`,
  `cordic_table`, `cordic_angle` and `ltf_taps` panicked (or would have built a table of
  2^28 entries) on an invalid `FixedRxConfig`, against `RxFrontFixed`'s own documentation.
  They return empty tables, zeros and a front that never acquires now. No caller used an
  invalid configuration on purpose (`rvec` uses the default; `tables()` already required a
  valid front).
- `correct_fixed` and `remove_dc_fixed` share their per-sample step with the replay (their
  by-hand gates unchanged).

**Measurement — `rxper stream 2000`** (the seven points, frames and receiver of `fixedw`;
frames lost of 2000, 1000 octets, clock offset uniform in ±40 ppm, the frame at −12 dBFS;
`rxper fixedw 40` is byte-identical before and after the tool's change).
`whole` is the hand-off rotated from the buffer's first sample (`fixedw`'s column
`word start`, reproduced to the frame); `slice` is rotated from T1's window; `stream` is
every sample through `RxHandoffFixed` with the integer decoder as reader: it sees the
beats alone, takes each at once, gives SIGNAL's verdict with the stream's tag 50 periods
after the beat that completes SIGNAL's window, and decodes at the end mark.

| Point | whole | slice | stream | lost by whole only / slice only | by stream only / slice only | declared again / aborted / stopped |
|---|---|---|---|---|---|---|
| AWGN 6 Mb/s 3.5 dB | 220 | 219 | 219 | 3 / 2 | 0 / 0 | 0 / 0 / 0 |
| AWGN 54 Mb/s 20 dB | 191 | 189 | 189 | 19 / 17 | 0 / 0 | 0 / 0 / 0 |
| AWGN 54 Mb/s 35 dB | 0 | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 / 0 |
| TGn D 18 Mb/s 15.5 dB | 181 | 179 | 179 | 2 / 0 | 0 / 0 | 0 / 0 / 0 |
| TGn D 54 Mb/s 26.5 dB | 221 | 217 | 217 | 9 / 5 | 0 / 0 | 0 / 0 / 0 |
| TGn E 54 Mb/s 26.5 dB | 143 | 145 | 145 | 8 / 10 | 0 / 0 | 0 / 0 / 0 |
| TGn E 54 Mb/s 45 dB | 0 | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 / 0 |

1. **The stream loses the frames the slice loses, frame for frame**, at every point: the
   replay, the verdict from the stream and the end from SIGNAL's length change nothing
   with a reader that always has room.
2. **Where the rotation starts decides single frames near the edge:** up to 19 frames of
   2000 are lost by one start only and 17 by the other at 54 Mb/s; the counts differ by
   −2 to +4. G2 was measured with `whole`; the hardware's stream is `slice`. The full G2
   grid was not rerun with it. (That the cause is the rotator's table being read at other
   addresses is reasoned, not measured.)
3. **The front did not declare a second time in any of 14 000 frames** of 1000 octets, so
   no stream was aborted by its own packet's data; none was stopped on a failed SIGNAL
   either (at 6 Mb/s and 3.5 dB the frames are lost in the data). These two columns are
   zero by measurement; a loss to the reader cannot occur in this tool and has no column.

**Independent review (fresh agent; the code, the tests and ADR 0008's clauses, not the
conclusions), 2026-11-21, on cae398e.** It confirmed by its own runs and reading: the
samples' identity with `handoff_fixed` (also after a new declaration, after a stop and on
a wrapped history), no stuck state, no panic on hostile pace numbers, `decode_ppdu_fixed`
unchanged in the order of its checks and errors, `fixedw 40` byte-identical with `main`,
the hand arithmetic, and that the tool's paths for stopped, aborted and lost streams work
when forced. Found and corrected:
- *a verdict had no packet identity:* one arriving after a new acquisition had replaced
  its packet stopped the new packet (a failure) or set its length and blocked its own
  verdict (a length). The tag above;
- *a delivered packet got an abort mark* when a new declaration came before its end mark
  was taken: `Packet` now;
- *the hold's end was two comparisons,* a period apart between a timely and a last-period
  verdict: one rule now, stated above;
- *the tool's reader used the acquisition's position* to time its verdict, and its
  `overrun` column was zero by construction: the verdict is timed from the beats now and
  the column is gone; the report was regenerated (same counts);
- the first journal text said the rotation's start "moves 1 to 4 frames": those were
  differences of counts; per frame it is up to 19 and 17, now a column;
- the ADR's paragraph said every packet ends in one mark without the exceptions; the
  history's bound in the constant's documentation was not derived (restated as what the
  decision guarantees and what the scenes measure); the start delay's limit was unstated;
- ten surviving mutants: eight fail now (above), two are the unkillable ones named above;
  one more it listed, the `needed` of a later `Truncated` in `decode_ppdu_fixed`, is older
  than this change and is left.
Not checked by it: the 2000-frame run, the first 35 mutants, `scripts/check.sh`, the
clock-level buildability of a write and a read of the history in one period.

**What this is not.** A model in sample periods with two pace numbers, not the hardware's
schedule. A still gain (no gain control in `rxper`): `rxagc loop` still models the hold in
the tool and is to be moved onto this model. One frame per buffer in the measurement, a
reader that always has room; frames after a SIFS, a false SIGNAL with a long length, a
stale verdict and an overrun are in the unit tests only. The verdict's latency is one
number (50 periods after SIGNAL's window is read). No vectors for the RTL yet: they come
with the hand-off's RTL, the last step of part d.

**Open**
- What the front searches for while a packet is open; a second declaration aborts the
  stream as the ADR says, and none was seen on true frames.
- `rxagc`'s commands onto `RxHandoffFixed` (the hold, the release) and their reruns.
- The G2 grid with the replay's phase.
- A hold on a phantom SIGNAL (the entry above, finding 4): see the correction below.

**Correction to the entry above (decision 2 as a closed loop), finding 4 and its "Open".**
It proposed checking SIGNAL's reserved bit to halve the holds on a SIGNAL decoded from
noise. The standard forbids rejecting a frame for it: "Bit 4 is reserved. It shall be set
to 0 on transmit and ignored on receive" (IEEE Std 802.11-2024 §17.3.4.4, read in the local
copy on 2026-11-21; `rx_ofdm.rs` already says so beside `decode_signal`). A receiver that
checks it would drop frames of a future amendment that sets it. So that item is withdrawn
as written; what else can end a hold on a phantom length (the level falling by the packet's
height, a restart on a stronger packet) is a question for the founder, not yet asked.

## 2026-11-21 · measurements · `rxagc` on the streaming hand-off: decision 2 as a loop a sample at a time

Commits: 171a3f3 (the control stepped), 94bf573 and 2785e4c (the tool). The reports are
from the binary of 2785e4c (`loop`) and of 94bf573 (the others; the commits between them
change `loop`'s extra column, texts and a test), built and run on the build server (Linux
x86-64, 32 threads, `rust:1-slim`), 2000 frames per row.

**What changed in the tool.**
- *The loop is the model's.* Until now `loop` formed the ADC's stream from a batch run of
  the control, took the acquisitions of a whole-stream run, raised and dropped a hold by
  the tool's own rules and ran everything again after each decision. Now, in every
  column, a frame goes a sample at a time through the control
  (`FastAttackAgc::start`, `AgcRun::push`: the same control stepped, `channel.md`), the
  ADC, the streaming front (`RxHandoffFixed`) and the decoder as its reader, which sees
  the beats alone: SIGNAL's verdict (`decode_signal_fixed` on the stream's first 208
  samples, with the stream's tag) 50 periods after the beat that completes them, the
  stream decoded when its end mark comes. The front's `hold()` after sample n − 1 holds
  the control at sample n (40 or 400 samples later in two columns). The hold's rise, its
  end from SIGNAL's length, its fall on a failed SIGNAL and the watchdog are
  `RxHandoffFixed`'s, not the tool's.
- *The front searches all through,* in every column: a frame is received if any stream
  decodes to it. The columns without the front's hold therefore no longer lose the rows
  with something before the frame by construction, and a SIGNAL that passes on noise no
  longer loses the frame behind it by construction (the old tool held the front for the
  length it claimed): a second declaration aborts that stream and opens the frame's.
- *Rows with a DC offset* (per index −20 dBFS; before the gain, −10 dBFS at the largest),
  on the frames of three rows: the loop had none.
- *A column `stream`* in `per`, `level` and `dc`: the loop's "front alone" on the row's
  frame. The integer chains of the other columns start their tracker from the CFO word
  and the register (`FixedTracking::from_cfo_word`), as `rxper` does; no float is left
  between the ADC and the PSDU. Those reports: the entry after this one.

**Results of `loop`** (`reports/rx_front_agc_loop.txt`; frames lost of 2000):

| Row | from the lock (the tool knows the packet) | timer on every lock + front | timer below the largest gain + front | **front alone** | that timer alone | nothing | front alone, clip release (tool only) |
|---|---|---|---|---|---|---|---|
| AWGN 54 Mb/s 25 / 30 / 40 / 60 dB | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 1 / 1 / 4 / 0 | 61 / 68 / 55 / 60 | 67 / 70 / 61 / 60 | 1 / 1 / 4 / 0 |
| AWGN 6 Mb/s 30 / 60 dB | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 28 | 0 / 26 | 0 / 0 |
| TGn D 54 Mb/s 40 / 60 dB | 0 / 0 | 0 / 0 | 0 / 0 | 2 / 3 | 85 / 64 | 87 / 68 | 4 / 3 |
| TGn D 6 Mb/s 30 dB; AWGN 6 Mb/s 5 dB | 0; 2 | 0; 2 | 0; 2 | 0; 2 | 1; 2 | 1; 2 | 0; 2 |
| preamble alone, then 54 / 6 Mb/s 30 dB | 0 / 0 | 1525 / 0 | 6 / 0 | **477 / 3** | 64 / 0 | 70 / 0 | 1 / 0 |
| frame, SIFS, then 54 / 6 Mb/s | 0 / 0 | 2000 / 2000 | 0 / 0 | 3 / 0 | 62 / 0 | 67 / 0 | 3 / 0 |
| noise −5 dBFS: 54 Mb/s; after a preamble; after a SIFS | 0; 0; 0 | 20; 1409; 2000 | 19; 1408; 2000 | 2; **508**; 1 | 65; 1643; 1979 | 51; 63; 56 | 2; 2; 1 |
| DC per index −20 dBFS: 54 Mb/s; after a preamble; after a SIFS | 0; 19; 14 | 0; 1520; 2000 | 0; 10; 1 | 1; **375**; 6 | 155; 158; 152 | 160; 163; 161 | 1; 5; 6 |
| DC before the gain −10 dBFS: the same three | 0; 0; 0 | 0; 1538; 2000 | 0; 7; 0 | 3; **472**; 5 | 133; 123; 113 | 139; 125; 119 | 3; 4; 5 |

(The two columns with the hold 40 and 400 samples late are in the report. They are on the
timer below the largest gain and share its loud-noise rows; elsewhere 40 late loses 0 to
41 and 400 late 0 to 813. The delay now moves both edges of the hold.)

1. **The ordinary rows, the SIFS rows and the loud-noise rows are the old report's,** to
   the frame in the columns with the front's hold: "front alone" 1, 1, 4, 0 at AWGN
   54 Mb/s, 2 and 3 through TGn D, 3 and 0 after a SIFS, 1 after a SIFS in loud noise. What
   the tool modelled by hand and what `RxHandoffFixed` does agree where both were right.
   The amendment of decision 2 stands on the same numbers: a timer on every lock loses
   every frame after a SIFS; the timer below the largest gain loses every one in loud
   noise.
2. **A SIGNAL read from noise does not by itself lose the frame behind it.** After a
   preamble alone the front declares twice per frame (4000 declarations); about 1510 of
   the first streams are stopped on a failed SIGNAL and about 470 are ended by the frame's
   own declaration: those are the SIGNALs that passed on noise (2000/4 by parity and the
   valid rates). With the gain right (from the lock; the timer below the largest gain
   that happens to keep the preamble's lock) the frame is then received: 0 and 6 lost of
   2000, where the old tool lost 478 in every column because its front did not search
   under a hold. The old finding 4 ("a quarter are lost in every column") was the tool's.
3. **What does lose them is the gain the hold freezes.** With the front's hold alone the
   preamble has ended (sample 320) before the front declares it; the control has left its
   lock on the lost power and is back at the largest gain when the hold is raised. When
   SIGNAL then passes on noise the hold stays, and the frame arrives at the largest gain:
   **477 of 2000 at 54 Mb/s (24 %), 508 in loud noise, 375 and 472 with an offset;** 3 at
   6 Mb/s, which survives the clipping. This is the open question of the entry before last
   (what ends a hold on a phantom SIGNAL) with its mechanism measured: a hold whose packet
   is not there, at a gain that was not taken for the packet that comes.
4. **A candidate, in the tool only (not in the model, not in ADR 0008): the hold stops
   acting on the control when 2 of the last 4 ADC samples under it sit on a rail,** until
   the front declares again or its hold falls. It takes the four cells of finding 3 to 1,
   2, 5 and 4 of 2000, and leaves every other cell as "front alone" but TGn D 54 Mb/s
   40 dB (4 against 2); frames whose gain moved inside the data rise at 6 Mb/s (5, 5 and
   14 of 2000 against 2, 2 and 0), none of them lost. Other settings were tried at 200
   frames only and were worse in the preamble rows (8 of 16: 30 and 21 lost of 200 at 54
   and 6 Mb/s; 4 of 8: 10 and 6; 2 of 2: 21 and 8; 2 of 4: 0 and 0): the release has to
   come as fast as the control's own overload detector or the gain settles too late in the
   short training field. One setting, one scene: an input for the founder's question, not
   a design.
5. **With an offset the tool's own assumption is not the best column:** "from the lock"
   loses 19 after a preamble and 14 after a SIFS at −20 dBFS per index (it holds from the
   control's first lock inside the packet, which the offset moves); the front's hold loses
   6 after the SIFS.
6. **Without the front's hold** the rows are as before (55 to 87 of 2000 at 54 Mb/s), and
   with an offset twice that (113 to 163): the offset is one more thing the detectors see.

**Independent review (fresh agent, the diff and ADR 0008, not the conclusions).** No
defect found in the stepped control or in the loop: a randomized differential test of
`run_held` against the old function (300 000 cases: holds from sample 0, overlapping,
reversed, to the stream's end and beyond, NaN and infinite samples, offsets) differed in
none with finite parameters; with a largest gain that is not a number the list of changes
gets entries on held samples (gains, locks and unlocks the same; not changed). It traced
the hold's line, the verdict's period, the sample a phase jump is drawn at, the offset per
gain, and that the reader uses the beats, the stream's word and its tag only. Found and
corrected:
- *the stepped control's new test could not fail:* both of its sides ran on the stepper
  (five mutants passed it; the older hand-worked tests caught them). It now carries the
  first case worked by hand through two holds (locks at 45, 150 and 255, releases reported
  at 105 and 210, the gains around them); four of those five mutants fail it and the
  fifth (a held sample still decides: nothing to decide at a quiet lock) fails
  `a_hold_freezes_the_gain…` as before;
- the header of `loop` described both kinds of offset as "per index"; the header of
  `per`/`level`/`dc` said every column tries the first acquisition only and did not say
  that `level`, `chg` and `held` describe the assumed hold, not the `stream` column;
- the delay columns' text did not say that both edges move.
It also read finding 3 off a 40-frame run before the journal said it.
Not checked by it: `RxHandoffFixed` beyond `push`/`signal`/`open`, `from_cfo_word`'s
numerics, `per` and `level`, any 2000-frame number.

**What this is not.** One seed per row, one run. The release's behaviour (to the largest
gain, the search from scratch) is an assumption about a radio. Frames of 1000 octets; one
gap after a preamble (280 to 330 samples) and one level; the first frame of a "frame,
SIFS" row is not counted. The reader always has room. The verdict's latency is one number
(50 periods). The float chain is not in the loop.

**Open**
- What ends a hold on a phantom SIGNAL: for the founder, with findings 3 and 4.
- The clip release, if chosen, belongs in `RxHandoffFixed` with its own gate, and needs
  the scenes it has not seen (a stronger packet inside a true one; an interferer).

## 2026-11-21 · measurements · `rxagc`'s other reports on the word start, with the `stream` column

Commit: 94bf573 (the binary; the build and the run as in the entry before this one, 2000
frames per row). Commands: `rxagc per`, `level`, `dc`, `cand`, `hold`, `cfo`, `pole`,
`edge`, each to its report (`reports/rx_ofdm_agc_{per,level,dc,pole}.txt`,
`reports/rx_front_dc_{cand,hold,cfo,edge}.txt`). Two things changed against the committed
reports: the integer chains start their tracker from the CFO word and the register
(`FixedTracking::from_cfo_word`), and `per`, `level` and `dc` carry the column `stream`
(the loop's "front alone" on the row's frame).

**The old columns, cell by cell against the reports they replace** (a script over the data
rows, the new column left out):

| Report | cells | changed | largest change of a count |
|---|---|---|---|
| `per` | 1407 | 10 | 1 frame |
| `level` | 350 | 7 | 1 |
| `cand` | 965 | 8 | 2 |
| `edge` | 352 | 1 | 1 |
| `dc` | 476 | 0 | |
| `hold`, `cfo`, `pole` | 138, 204, 182 | 0 | |

Every changed cell is in an integer column; no float cell moved. The start from the word
in place of the float conversion costs or gains a frame in 26 cells of 3674, and `per`'s
lines of the SNR at 10 % PER are the old report's to the printed digit in every old column.

**The `stream` column.** It is the first place the receiver of ADR 0008 as a whole (the
streaming front, its own hold, the offset of the two long training symbols removed, no DC
blocker before the decoder, 13 bits) stands next to the chains with the blocker on the
same frames.

1. **`per`, around 10 % PER: the same as "float known" up to 24 Mb/s, better above it.**
   SNR at 10 % PER against "float known" (a gain that knows the frame, no gain control):

   | | 6 | 9 | 12 | 18 | 24 | 36 | 48 | 54 Mb/s |
   |---|---|---|---|---|---|---|---|---|
   | AWGN | +0.00 | +0.01 | +0.00 | −0.01 | +0.03 | −0.10 | −0.39 | −0.84 dB |
   | TGn D, static | +0.05 | +0.00 | −0.01 | +0.01 | −0.10 | −0.84 | −2.12 | (26.79 dB; no other column reaches 10 %) |

   The gain at the high rates is not the stream's merit over an ideal chain: every other
   column, "known" included, has the 2⁻⁷ blocker in the decoder's path, and `rx_ofdm.md`
   (2026-11-20, finding 3) measured that it more than doubles the frames lost at 54 Mb/s.
   The stream is what that entry's "raw" column was, now with the offset handled.
2. **`per`, above the noise's reach** (30 to 60 dB; frames lost of 10 000 per rate): at
   54 Mb/s the stream loses 12 in AWGN and 49 through TGn D where float loses 70 and
   1437; at 6 Mb/s 0 and 1 (float 0 and 0). **At 24 Mb/s in AWGN it loses 12 of 10 000
   (1, 2, 4, 4, 1 over the five SNRs) where every other column loses none**: a small
   floor of the stream's own, not looked into (the frames whose gain moves under the
   front's later hold are the first suspect: `chg` is 3 to 10 per frame there).
3. **`level`** (the noise from −24 to −60 dBFS at the largest gain, at the SNR of about
   10 % PER): at 6 Mb/s the stream follows the 13-bit integer columns down the level
   (487 lost at −60 dBFS; int 13/1 470, float 302); at 54 Mb/s it loses about half of
   every other column at every level (185 to 374 against 387 to 736 for int 13/1), again
   the blocker's absence.
4. **`dc`** (54 Mb/s; frames lost of 2000): 1 to 4 in every row but one, where the chains
   with the blocker lose 5 to 239 and the decoder handed the raw samples up to 1666. The
   one row: an offset before the gain of −10 dBFS at the largest gain, at 25 dB: 850
   (float 1115, raw 1933); 3, 2 and 1 at 30, 40 and 60 dB. At 6 Mb/s the stream loses
   none in any row.

**What this is not.** One seed per row, one run. The `stream` column's phase jumps are its
own draws, so a frame's gain history differs from the other columns': the pairs +a−b
against "float" are wider than between the other columns (for example 6 Mb/s 2.5 dB:
+39−32 around 1024 against 1017) with the totals equal. `level`, `chg` and `held` describe
the assumed hold, not the stream's. No independent review of these numbers beyond the
review of the tool recorded in the entry before this one.

**Open**
- The stream's 12 of 10 000 at 24 Mb/s in AWGN above 30 dB.
- The row of `dc` at −10 dBFS before the gain and 25 dB (850 of 2000): whether it is the
  offset estimate or the ADC's range.

## 2026-11-21 · G3 · the search correlator folded over 8 clocks (ADR 0008, decision 4)

**What changed.** `hw/rtl/ltf_corr.sv` makes the same sums (the two exact folds of part 3a:
T1 + T2, then the Hermitian pairs) with 16 multipliers of 14 × 8 bits over the 8 clocks of
a sample, in place of 128 constant multiplications in one. A slot is one pair k = 1 … 31
(four products) or the two real taps t(0), t(32) together; clock p of a sample takes slots
4p … 4p + 3, their window entries through 8-to-1 multiplexers and their taps from the
generated table. The window of 128 samples stays a shift register. The model is untouched
and so are the vectors (`hw/vectors/ltf_corr.txt`): the sums are the same integers.

**Interface (G0).** One sample per `in_valid`, at most one in 8 clocks; no `ready`. The
output is registered **14 clocks** after the clock edge that takes the sample (was 5): 1
select, 1 fold, 1 product, 2 of sums, the accumulator after the 8th clock, 1 of narrowing.
A sample taken sooner than 8 clocks after the one before abandons that one's correlation
(no `out_valid` for it) and is itself correlated. The decision's schedule (part d, step 3)
takes the 14 from here; ADR 0008 counted "a sample later", 8 clocks, and this is 9.

**Gate.** `make -C hw ltf_corr` (Verilator 5.054; `make -C hw lint` clean), with the
review's additions below. The testbench feeds a sample every 8th clock with one more idle
clock after every 5th and two after every 13th, other values on the data inputs between
samples; it requires every output exactly 14 clocks after its sample's edge and equal to
the model; it replays the first segment with the sample after every 97th coming after 7, 5
and 1 clocks (no output for the abandoned one, the model's for the early one); and it
resets the block 1 to 13 clocks after a sample with a full window and requires that
nothing comes out. The simulation starts flip-flops without a reset at random values
(`+verilator+rand+reset+2`).
Result: 9 segments, 7618 samples, 9517 outputs, 13 resets in flight, **0 mismatches**. Seen
failing first: the unfolded RTL against the first version of the new testbench, 13 190
mismatches of 7489 outputs (its outputs come at 5 clocks: every one is counted as early and
as missing).

Mutations (each restored with `cp` + `touch`; mismatches of the 7489 outputs of the
testbench before the review's additions, timing errors counted too):
| Mutation | Mismatches |
|---|---|
| tap of the neighbouring pair | 7489 |
| the real tap table for the b products | 7472 |
| sign of the b term of the real part / of the imaginary part | 7398 / 7383 |
| sign of a product in the imaginary sum | 7480 |
| the accumulator never restarts | 7485 |
| t(0) and t(32) swapped / slot 0 treated as a pair / its second entry off by one | 7485 / 7483 / 7292 |
| the T2 half of the window off by one | 7472 |
| slots of a clock in the other order | 7472 |
| the last clock's mark a clock early | 22 465 |
| the first clock's mark a clock late | 7489 |
| an output before the window is full | 7491 |
| an early sample does not restart the clocks | 7507 (4043 outputs) |
| the window shifts once more in a sample's 8 clocks | 7472 |
One survives and is equivalent: the accumulator also clocked outside a sample's clocks (its
enable is there to keep it still, not for the value: the first clock's mark reloads it).

**Independent review (fresh agent, the diff and decision 4).** No defect found in the RTL:
it derived the worst cases of every width by hand (sums within 13 and 14 bits, products
within 2^20 of 22 bits, the accumulator within 2^26 of 28; the saturation unreachable),
rebuilt the correlator from the tap table in a model of its own, and ran a bench of its own
(spacings 1 to 12 at random, bursts of three samples on consecutive clocks, other data
between samples, resets in flight, random initial state: nothing wrong in 20 runs). Found
in the gate and corrected:
- *the marks' reset was not tested:* the reset branch of the marks removed passed (the
  testbench reset only an idle block, and the simulation started every flip-flop at 0). The
  resets in flight and the random start are in the gate now: that mutant gives 1036
  mismatches;
- *the early sample was tried at 5 clocks only:* the last clock's mark taken from the
  delayed phase passed and let out a sample abandoned 7 clocks after it was taken. Spacings
  7, 5 and 1 now: 968 mismatches;
- the data inputs held the last sample between samples: other values now (a window that
  takes its input on another clock: 9517 of 9517).
It also confirmed the 14 clocks stage by stage and that the surviving mutant is equivalent.
Not checked by it: the synthesis tools, the journal's mutation counts, timing and area.

**Timing and area** (probes run by hand on the build server; no gate reruns them):
- KV260 part, out of context, 6.25 ns (`tools/syn/ooc_probe.tcl`, Vivado 2024.2; excerpt
  appended to `reports/rx_front_timing_ooc.txt`): **+1.80 ns** (was −1.89 ns, 729 failing
  endpoints), 1370 LUTs (1932), 3611 registers (3320), **20 DSP blocks (124)**. The worst
  path is the sum of a clock's four slots, which Vivado put into DSP blocks.
- IHP SG13G2, 6.25 ns: `docs/gates/silicon_probe.md` (2026-11-21).

**What this is not.** No enable from the search control yet (ADR 0008: the correlators'
arithmetic runs only while a search is open): it comes with the top level. Nothing checks
the block beyond its vectors (G4 is not started); the early-sample rule is exercised at one
spacing (5 clocks). The decision's own correlator (a window of 64 folded over 4) is a
separate block of part d.

## 2026-11-22 · G0 + model gate + measurement · the hold given up on a clipping ADC (ADR 0008, decision 2, third amendment)

Decision: the founder, 2026-11-22, on findings 3 and 4 of the entry of 2026-11-21 ("`rxagc`
on the streaming hand-off"): the release goes into the model and into ADR 0008, with its own
gate and the scenes the candidate had not seen.

**G0 (the rule; ADR 0008, decision 2, amendment of 2026-11-22).**
- A *rail sample* is an ADC sample with I or Q at −2048 or 2047 (`ADC_FULL_SCALE`).
- The front keeps the rail flags of the last `clip_window` = 4 ADC samples (sample n − 3
  to n), from reset on and whatever the hold is.
- After the push of a sample that makes `clip_count` = 2 of them set, the hold is down. In
  the order of one push: the hold's end by length, a new declaration (raises the hold), the
  release. So a declaration with two rail samples in its window shows no hold at all.
- Nothing else changes: the stream, its marks, the verdict's handling, the watchdog. A
  verdict after the release leaves the hold down. Only a new declaration raises it.
- `clip_count` = 0: no release. A count above the window: never reached.
- Interface: `HandoffConfig::clip_window`, `clip_count`; the hold is still
  `HandoffStep::hold` / `RxHandoffFixed::hold`.

**Gate** (`crates/radio-phy/tests/g2_rx_handoff.rs`, three tests; reference: the rule
worked by hand on streams with chosen samples replaced by rail values).
- `two_of_four_samples_on_a_rail_give_up_the_hold_and_the_stream_goes_on`: twelve
  placements (one rail sample; two that never share a window; two at distances 1, 2, 3;
  each rail of each part; one count short of a rail; the hold's first sample its last),
  the hold's stretch exact in each, the beats in the same periods and equal to
  `handoff_fixed` of the altered samples, no end mark; a count of 0 with 40 rail samples;
  3 of 8; a count the window cannot reach.
- `a_hold_given_up_stays_down_through_the_verdict_and_a_new_declaration_raises_it`.
- `rail_samples_before_the_declaration_count_in_the_window`.

Seen to fail first: with the two fields in `HandoffConfig` and no rule, all three fail
(the hold's stretch `[(1162, 4420)]` against `[(1162, 1265)]` in the first).
Command: `cargo test -p radio-phy --test g2_rx_handoff` — 15 passed (commit 58fe414).

**Mutations** (each applied to `rx_front_fixed.rs`, the gate run, the file restored with
`cp` and `touch`; all eleven fail the gate):

| Mutant | Fails |
|---|---|
| window of 3 | test 1 |
| count + 1 | all three |
| upper rail one count lower | test 1 |
| lower rail never | all three |
| Q not looked at, upper rail; lower rail | test 1; tests 1 and 3 |
| the window starts with the hold | test 3 |
| the window cleared at a declaration | test 3 |
| a count of 0 does not switch it off | test 1 |
| the release ends the stream | tests 1 and 2 |
| the verdict raises the hold again | test 2 |

**The tool on the model's rule.** `rxagc loop`'s candidate column was the tool's own
code; it is `RxHandoffFixed`'s now ("front + release"; "front alone" runs the model with
`clip_count` 0). Check: the binary before the change against the binary after it, `loop`
at 100 frames, the 23 rows of the old report: every data row identical in every column.
The `stream` column of `per`, `level` and `dc` runs the model's default, the release on.

**Review's additions to the gate** (below): a fourth test,
`the_release_acts_anywhere_under_the_hold_and_for_any_window` — the release 1000 and
20 000 samples after a verdict with a length and two samples before the hold's end, 20 000
samples into the watchdog's hold, samples beyond the rails (the input is 16 bits wide),
windows of 16, 63, 64, 65, 1000 and `usize::MAX` at the distance that just fits and the one
that just does not, windows of 1 and 0, 64 of 64. 16 passed (commit ffc8af5).

**Measurement: `rxagc loop`** (commit ffc8af5, the build server, 2000 frames per row;
`reports/rx_front_agc_loop.txt`). New rows: a weaker frame of the same rate and length
being received when the frame begins 1200 samples into it ("under"), and white noise
added over L samples 1000 samples into the frame ("hit"). New column: the release acting
40 samples (2 µs) late with its hold. Frames lost of 2000:

| Row | from the lock | front alone (no release) | **front + release** | release 2 µs late | nothing |
|---|---|---|---|---|---|
| the 23 rows of the report of 2026-11-21 | as there | as there | as the candidate there | | as there |
| preamble alone, then 54 Mb/s 30 dB (idle; loud noise; DC per index; DC before the gain) | 0; 0; 19; 0 | 477; 508; 375; 472 | **1; 2; 5; 4** | 3; 85; 6; 8 | 70; 63; 163; 125 |
| under −25 dB: 54 Mb/s 50 dB | 0 | **2000** | **2** | 279 | 59 |
| under −25 dB: 6 Mb/s 50 dB | 0 | 43 | 1 | 246 | 11 |
| under −10 dB: 6 Mb/s 50 dB | 0 | 0 | 9 | 188 | 62 |
| hit 4, +10 dB: 54 Mb/s 30 dB | 2000 | 2000 | 2000 | 2000 | 2000 |
| hit 4, +20 dB: 6 Mb/s 30 dB | 91 | 91 | 94 | 91 | 141 |
| hit 8, +20 dB: 6 Mb/s 30 dB | 710 | 710 | **1010** | 709 | 1249 |
| hit 8, +10 dB: 6 Mb/s 30 dB | 21 | 21 | **54** | 21 | 116 |
| hit 16, +10 dB: 6 Mb/s 30 dB | 384 | 383 | **766** | 384 | 950 |

1. **The model's rule is the candidate, to the frame.** All 23 rows of the old report are
   identical in all nine old columns at 2000 frames (and at 100 frames against the binary
   before the change): the plain rows, the SIFS rows and the loud-noise rows are untouched
   by the release, TGn D 54 Mb/s 40 dB is 4 against 2 as before, and the four rows after a
   preamble alone are 1, 2, 5 and 4 of 2000 against 477, 508, 375 and 472.
2. **A stronger frame inside a weaker one is the same failure, and complete.** With the
   hold alone every one of 2000 frames at 54 Mb/s is lost: the gain is held for a frame
   25 dB weaker. With the release 2 are lost; the front declares the stronger frame
   (4000 declarations, 2000 streams aborted by the second). At 6 Mb/s the frame mostly
   survives the clipping (43 lost) and the release still helps (1). With the weaker frame
   only 10 dB below, at 6 Mb/s, the hold alone loses none and the release 9: the gain
   moves in 97 frames and the weaker frame is then interference at 10 dB.
3. **The cost: a clipping burst inside a 6 Mb/s frame.** A frame at 6 Mb/s often survives
   a short burst under a still gain; with the release the gain moves under it (in 1299 to
   2000 of the 2000 frames) and with it the carrier's phase and the offset. 8 samples
   20 dB above the frame: 1010 lost against 710; 8 samples at +10 dB: 54 against 21; 16
   samples at +10 dB: 766 against 383; 4 samples at +20 dB: 94 against 91. Of the frames
   that the burst alone spares, the release loses up to a quarter (300 of 1290; 383 of
   1617).
   At 54 Mb/s the burst loses the frame in every column, with and without a hold.
   No hold at all is worse than either in every one of these rows (116 to 1249).
4. **A release that acts 2 µs late is a different rule.** It costs nothing in the burst
   rows (the burst is over when it acts; the gain still moves in as many frames, and why
   that then costs no frame was not looked into) — and it loses 279, 246 and
   188 of 2000 in the "under" rows and 85 after a preamble in loud noise, because the
   stronger frame's short training field is half over before the control may answer. So
   the release wants the same speed as the control's own overload detector for what it is
   for, and a burst is what that speed costs. Both cannot be had by a delay.
5. **Where it does not matter:** every row without a burst, a preamble before it or a
   frame under it differs by single frames between the three columns with the front's
   hold.

**Reading for the decision.** The release trades frames lost for certain (a quarter of
those after an aborted preamble, all of the stronger-inside-weaker ones at 54 Mb/s)
against a higher loss among 6 Mb/s frames that a clipping burst hits. How often each
happens on air is not something this model knows: capture effect (a stronger frame over a
weaker one) is ordinary in a busy channel; a burst of 0.4 to 0.8 µs that clips the ADC
without being a frame is radar or an impulsive interferer. The founder decided for the
release before these rows existed; the rows are for R1 to weigh against captures.

**Independent review (fresh agent, the diff of 27b2e30..58fe414 and ADR 0008, not the
conclusions).** No defect found in the rule: order inside `push`, `signal`, `close`, the
watchdog, an overrun, windows of 0, 1, 64 and above, no panic path; a randomized
differential test of its own (600 trials, 102 with a release; slow and stalled readers,
all four verdicts, windows and counts from 0 to `usize::MAX`) found the beats and the
acquisitions identical to a run with the release off in every trial and the hold equal to
an independent oracle. Old binary against new, `loop` at 40 frames: the 23 old rows
identical. Found and corrected:
- *holes in the gate:* of its 32 mutants 8 survived, 7 of them real — no release once the
  verdict is in; none later than 200 samples after the stream's opening; `==` on the
  rails; a window of 64 or more seeing nothing; windows from 16 up treated as 64; a rail
  register of 8 bits; a shift by 64 at a window of 64; a window of 0 treated as 1. The
  fourth test (above) was added and all eight now fail it; the ninth survivor (the release
  only while a stream is open) is equivalent;
- the ADR's amendment was cited by the code and not yet written (it is in this PR);
- "under": the frame began 1152 to 1197 samples into the weaker one, not 1200 (the weaker
  frame was placed at a fixed sample): placed at the frame's own lead now; a dead branch
  removed;
- "hit": the text did not say the burst is added, that its power is one draw (2.6 to
  11.9 dB measured for "4, +10 dB" over 6 frames) and that it goes through the channel's
  delay and clock offset;
- the header did not say the release is off in the timer columns; stale comments
  (`Chain::Stream`, "the hold stays up", the window's limit).
Noted and left: the row "hit 4 +10 dB, 54 Mb/s" is saturated in every column (kept: it
says a clipping burst costs the 54 Mb/s frame whatever holds the gain); a burst row has
no row with the same frames and no burst; `Burst::Frame`'s "650 samples into the stream"
is 600 to 649.
Not checked by it: any 2000-frame number, `per`/`level`/`dc`, how far a burst spreads
after the fractional delay, `rxper stream` before and after (argued from the code: it
never reads the hold), `scripts/check.sh`.

**What this is not.** One seed per row, one run. The numbers 4 and 2 were chosen on the
preamble rows at 200 frames; nothing here searched for better ones against the burst rows.
The released control returns to the largest gain and searches (an assumption about a
radio). The rails are the ADC's: a radio that clips before its ADC needs its own flag. The
weaker frame under the frame has the same rate, length and channel; the burst is white
noise through the frame's channel. No float chain in the loop.

**The `stream` column with the release** (`rxagc dc`, `level`, `per` at commit 5a9a9fa,
whose `stream` path is that of ffc8af5; 2000 frames per row; the three reports replaced).
The old columns are identical cell for cell. The `stream` column, against the run with the
hold alone: `level` no cell of 35 changed; `dc` one of 49, by one frame; `per` 12 of 126,
every one upward: one frame in seven cells, and through TGn D above the noise's reach
6 Mb/s 50 and 60 dB 5 and 2 of 2000 (0 and 0 before), 24 Mb/s 30, 50 and 60 dB 3, 2 and 4
(1, 0, 1). So in a fading channel at a high level the release costs a few frames of 2000
that the hold alone kept (a frame's own peaks at a gain held a little high). The SNR at
10 % PER moves in one line, by 0.01 dB.

**Rules in between, in the tool only** (`rxagc release`, commit of the tool after
ffc8af5, 2000 frames per row on `loop`'s frames; `reports/rx_front_agc_release.txt`). The
founder asked for something between the hold alone and the release as it is. Tried: other
counts and windows, and a rule by the rate — 2 of 4 until the verdict on SIGNAL of the open
stream names 6, 9 or 12 Mb/s, another rule from then on. Frames lost of 2000:

| Row | hold alone | **2 of 4 (the model)** | 4 of 8 | 8 of 16 | 24 of 32 | by rate: none | by rate: 8/16 | by rate: 24/32 |
|---|---|---|---|---|---|---|---|---|
| preamble alone, then 54 Mb/s | 477 | 1 | 100 | 290 | 477 | 175 | 101 | 175 |
| preamble alone, then 6 Mb/s | 3 | 0 | 41 | 136 | 3 | 1 | 48 | 1 |
| the same at 54 Mb/s in loud noise | 508 | 2 | 2 | 2 | 5 | 189 | 2 | 3 |
| under −25 dB, 54 Mb/s | 2000 | 2 | 2 | 2 | 4 | 2 | 2 | 2 |
| under −25 dB, 6 Mb/s | 43 | 1 | 1 | 1 | 2 | 23 | 1 | 2 |
| under −10 dB, 6 Mb/s | 0 | 9 | 19 | 40 | 8 | 62 | 39 | 69 |
| hit 4, +20 dB, 6 Mb/s | 91 | 94 | 67 | 91 | 91 | 91 | 91 | 91 |
| hit 8, +20 dB, 6 Mb/s | 710 | 1010 | 795 | 529 | 710 | 711 | 530 | 711 |
| hit 8, +10 dB, 6 Mb/s | 21 | 54 | 19 | 21 | 21 | 21 | 21 | 21 |
| hit 16, +10 dB, 6 Mb/s | 383 | 766 | 496 | 383 | 383 | 384 | 383 | 384 |

(6 of 8 and 12 of 16 are in the report: 477 after a preamble, 530 and 303 in "under
−10 dB". The four rows without a burst, a preamble or a frame under are 0 to 4 in every
column; "hit 4, +10 dB, 54 Mb/s" is 2000 in every column.)

6. **No rule tried has both ends.** The preamble row wants the release within a few
   samples (the gain stands at its largest and the control needs the whole short training
   field to come down): 2 of 4 loses 1, 4 of 8 loses 100, 8 of 16 loses 290, 24 of 32
   all 477. The burst rows want patience: 24 of 32 costs nothing there, 4 of 8 a quarter
   of what 2 of 4 costs.
7. **The rule by the rate removes the burst's cost and keeps the stronger-frame case, and
   pays 175 of 2000 after a preamble alone:** the SIGNALs read from noise that name 6, 9
   or 12 Mb/s (three rates of eight, of the 470 that pass) put the front into its patient
   rule just when the frame arrives at the largest gain. What it would need is fewer
   noise SIGNALs believed: a quality threshold on SIGNAL's decoding. Not built.
8. **A frame 10 dB under at 6 Mb/s is lost more by every slower rule** (19 to 530
   against 9): a release that comes during the frame's training fields is worse than one
   at its first samples or none.
9. A release some samples after an 8-sample burst (6 of 8: 485; 8 of 16: 529) loses
   fewer frames than no release (710). Not looked into.

Nothing of this is in the model: it runs 2 of 4, as decided.

## 2026-11-22 · model gate + measurement · SIGNAL's check, and a release by the rate (tool only)

Asked by the founder the same day: something between the hold alone and the release as it
is, and then the rule by the rate with a threshold on SIGNAL's quality.

**SIGNAL's check (model).** `rx_ofdm::signal_checked` and
`rx_ofdm_fixed::decode_signal_checked_fixed` give, with the field, `SignalCheck`: of
SIGNAL's 48 coded bits, how many soft values agree in sign with the decoded field encoded
again (`agree`) and how many have the other sign (`disagree`); a soft value of 0 is in
neither. The decoded 24 bits are encoded as they were decoded, the reserved bit included
(§17.3.4.3: it is not refused). `decode_signal_fixed` is the same decoding without the
check; nothing that existed changes its result.

Gate: `crates/radio-phy/tests/g1_signal_check.rs`, four tests. Reference: the
transmitter's SIGNAL (`signal_bits`, `encode`, held to Annex I.1 Tables I-7 and I-8 by
`g1_annex_i1`) with coded bits chosen in the test turned over (one to four, the code's
free distance being 10) or erased: the counts must be those numbers, for every rate and
four lengths each; the reserved bit set; a failed field has no check; the integer decoder
reports 48 and 0 at 30 dB for every rate and its plain form the same field. Seen to fail
first with a count that was always 0 (all four tests). Mutants, all nine failing the gate:
the sign the other way; an erasure counted; the tail not encoded; the soft values read
backwards; the reserved bit taken as 0; every other coded bit only; the integer decoder
reporting a fixed check; the two counts swapped; `agree` as 48 less `disagree`.
Command: `cargo test -p radio-phy --test g1_signal_check` — 4 passed.

**What the two counts are worth** (`rxagc release`, 2000 frames per row, the column
"front + release"; the report's second table):
- `disagree` does not tell a noise SIGNAL behind a true preamble from a true one: there
  the symbol at SIGNAL's place is near silence against the channel estimate, the 4-bit
  soft values are mostly 0, and the count is 1 to 5 (seen at 100 frames with the first
  form of the check, which had this count alone).
- `agree` does. The frame's own SIGNAL: 48 in every one of 2000 in AWGN at 30 dB and
  more; 44 or more in 1999 of 2000 at 6 Mb/s 5 dB; through TGn D 40 or more in 1999 of
  2000 at 6 Mb/s 30 dB (one at 36 to 39) and in all at 54 Mb/s 40 dB. The others (SIGNALs
  read behind a preamble alone): 16 to 35 in quiet noise (488 and 495 of them), 12 to 43
  in loud noise (4 of 518 at 40 to 43, none above).

**A release by the rate, in the tool only** (`reports/rx_front_agc_release.txt`, commit
a480af1; frames lost of 2000). The rule: 2 of 4 as the model has it; once SIGNAL of the
open stream has named 6, 9 or 12 Mb/s with `agree` ≥ 40, 24 of 32 from then on
("patient"); and a declaration under a hold that is up and patient, with nothing released,
leaves it patient until its own SIGNAL's verdict ("inherited").

| Row | hold alone | 2 of 4 (the model) | by rate, no threshold | by rate, agree ≥ 40 | **the same, inherited** |
|---|---|---|---|---|---|
| preamble alone, then 54 Mb/s | 477 | 1 | 175 | 1 | **1** |
| the same in loud noise | 508 | 2 | 3 | 2 | **2** |
| preamble alone, then 6 Mb/s | 3 | 0 | 1 | 0 | **0** |
| under −25 dB, 54 Mb/s | 2000 | 2 | 2 | 2 | **2** |
| under −25 dB, 6 Mb/s | 43 | 1 | 2 | 2 | **2** |
| under −10 dB, 6 Mb/s | 0 | 9 | 69 | 69 | **8** |
| hit 4, +20 dB, 6 Mb/s | 91 | 94 | 91 | 91 | **91** |
| hit 8, +20 dB, 6 Mb/s | 710 | 1010 | 711 | 711 | **711** |
| hit 8, +10 dB, 6 Mb/s | 21 | 54 | 21 | 21 | **21** |
| hit 16, +10 dB, 6 Mb/s | 383 | 766 | 384 | 384 | **384** |
| TGn D 54 Mb/s 40 dB | 2 | 4 | 4 | 4 | **4** |

(The other rows are the same in every column. Thresholds of 24, 32 and 44 are in the
report: 94, 2 and 1 in the first row; 40 and 44 agree in every row.)

1. **With the threshold the rule has both ends:** what 2 of 4 gains after a preamble and
   for a stronger frame at 54 Mb/s, and what the hold alone keeps of the 6 Mb/s frames a
   burst hits. The 175 of the rule without a threshold were noise SIGNALs naming a slow
   rate; none has 40 agreeing bits in quiet noise.
2. **Without the inheritance one row is worse than both** (69): the weaker frame's true
   SIGNAL makes the hold patient, the frame 10 dB above it clips too little for 24 of 32,
   and the front's declaration of that frame put the rule back to 2 of 4 — which then
   released in the middle of the frame. Inherited: 8, with the model's 9 and the hold
   alone's 0.
3. With "no release at all" as the patient rule, inherited: 43 in "under −25 dB, 6 Mb/s"
   (the hold alone's), 4 in the loud preamble row. 24 of 32 is the better patient rule.
4. The gain moves inside far fewer frames: at 6 Mb/s 30 dB in 2 of 2000 against 5, through
   TGn D in 1 against 14.

**What this is not.** In the tool, not in the model; one seed per row; the same scenes the
rule was shaped on (the inheritance was added after the 2000-frame run showed row 6, and
run again on the same frames). The slow rates are 6, 9 and 12 Mb/s by judgement; 18 Mb/s
and above were not tried as slow. The burst is white noise; the weaker frame has the
frame's rate. A noise SIGNAL in loud noise reached 43 agreeing bits: at a threshold of 40
four of 518 would be believed there (and cost nothing in that row). No independent review
yet: it belongs with the change to the model.

**Open**
- Whether the rule goes into `RxHandoffFixed` and ADR 0008 (the founder).
- If it does: `signal()` has to be told the rate and the check, and the gate needs scenes
  the rule has not seen (other rates under, an interferer that is a frame, TGn channels).

## 2026-11-22 · G0 + model gate + measurement · the release by the rate in the model (ADR 0008, decision 2, fourth amendment)

Decision: the founder, 2026-11-22, on the entry before this one: the rule by the rate with
the threshold on SIGNAL's agreeing bits and the inheritance goes into `RxHandoffFixed` and
ADR 0008 in place of plain 2 of 4, with its gate, rows it has not seen and a review.

**G0 (the rule; ADR 0008, decision 2, fourth amendment).**
- The decoder's verdict on SIGNAL is `SignalVerdict { data_symbols, patient }`.
  `SignalVerdict::of(signal, check, pace)`: `patient` when the rate is 6, 9 or 12 Mb/s
  and `check.agree` ≥ `patient_agree` = 40 (`rx_ofdm::SignalCheck`, the entry before).
- The front has one bit more, *patient*. The first verdict on the open stream sets it to
  the verdict's (a failed SIGNAL ends the stream and the hold as before). A second
  verdict, or one for a stream that is not the open one, changes nothing.
- The release: with the bit clear, `clip_count` = 2 of the last `clip_window` = 4 ADC
  samples on a rail, as the third amendment has it; with it set, `patient_count` = 24 of
  the last `patient_window` = 32. One history of rail flags serves both (64 samples, from
  reset on): a verdict changes which window is read from the next push on, not what is
  in it.
- A declaration raises the hold; it starts patient when the hold was up and patient at
  that push (after the hold's end by length has been applied, before this sample's
  release) and not otherwise, and stays so until its own first verdict. This passes
  through any number of declarations that get no verdict.
- The bit is only read while the hold is up. Nothing in the stream, its marks or the
  acquisitions depends on it.
- A count of 0 switches that rule's release off; a window of 64 or more is 64.
- Interface: `HandoffConfig::patient_window`, `patient_count`, `patient_agree`;
  `RxHandoffFixed::signal(packet, Option<SignalVerdict>)`.

**Gate** (`crates/radio-phy/tests/g2_rx_handoff.rs`, eight tests of the rule in its 23;
reference: the rule worked by hand on streams with chosen samples replaced by rail
values and verdicts given at chosen samples).
- `a_patient_hold_is_given_up_by_24_of_32_and_not_by_2_of_4`
- `a_declaration_under_a_patient_hold_inherits_…`
- `the_verdict_is_patient_for_a_slow_rate_with_forty_agreeing_bits` (every rate; 39, 40
  and 48 agreeing; 30 agreeing with none against; another threshold)
- `the_patience_is_the_open_streams_first_verdict_and_nothing_elses`
- `the_rail_flags_are_one_history_for_both_rules`
- `the_inheritance_lasts_passes_on_and_needs_the_hold_up_at_the_declaration` (three
  declarations with a verdict on the first alone; a declaration at the very sample of the
  hold's end by length; a verdict that is not patient with rail samples just before it)
- `a_verdict_of_any_length_field_does_not_panic`
- and the older tests on `SignalVerdict` in place of a bare length.

Seen to fail first: against the model with the fields and a verdict that is never patient
and no second rule, the first three fail (the hold's stretch `[(1162, 2277)]` against
`[(1162, 27900)]` in the first). Command:
`cargo test -p radio-phy --test g2_rx_handoff` — 23 passed (commit 0b63cb3).

**Mutations** (each applied to `rx_front_fixed.rs`, the gate run, the file restored with
`cp` and `touch`). Sixteen of the author's: the patient rule never used; its window one
short; its count one more; its window with the fast count; the verdict never patient;
every verdict patient; a verdict that cannot end the patience; no inheritance; inherited
also with the hold down; every declaration patient; 18 Mb/s slow; 12 Mb/s not slow;
`agree` strictly above the threshold; a count of 0 that releases — fourteen failed at
once; the threshold put on `disagree` passed, a case was added (30 agreeing, none
against) and it fails; `close` not clearing the bit passed and is equivalent (the bit is
read only under a hold, and every hold is raised by a declaration that sets it): that
line was removed. Seven of the reviewer's that the gate of 19 tests let through, all
failing the four tests added for them: the rail history cleared at a patient verdict; a
second verdict setting the bit; a verdict for another stream setting it; inherited
patience running out 30 periods after the opening; inherited only from a stream that had
a verdict; the hold judged before its end by length at a declaration; a verdict judging
the window at once instead of at the next push.

**The tools on the model's rule.** `rxagc` and `rxper stream` give the model
`SignalVerdict::of` of `decode_signal_checked_fixed`. `rxagc loop` has a column "front +
release by rate" (the model's default) beside "front + release" (2 of 4: the model with no
verdict ever patient) and "front alone"; `rxagc release` a column "the model" beside the
tool's own trial of the same rule. The `stream` column of `per`, `level` and `dc` runs the
model's default.

**Measurement: `rxagc release`** (commit 0b63cb3, the build server, 2000 frames per row;
`reports/rx_front_agc_release.txt`).
1. **The model is the trial, to the frame:** in all 27 rows the column "the model" equals
   "24/32, >=40, inherited" cell for cell (frames lost, declarations, stopped streams,
   frames with the gain moved). The 17 rows and 15 columns of the report of the entry
   before are unchanged.
2. **Ten rows the rule had not seen** (added with the model, after the numbers 40, 24 and
   32 were fixed; frames lost of 2000):

| Row | from the lock | hold alone | 2 of 4 | by rate, not inherited | **the model** |
|---|---|---|---|---|---|
| 6 Mb/s under −25 dB: 54 Mb/s 50 dB | 0 | 2000 | 3 | 4 | **4** |
| 6 Mb/s under −10 dB: 54 Mb/s 50 dB | 2000 | 2000 | 2000 | 2000 | **2000** |
| under −25 dB: 12 Mb/s 50 dB | 0 | 1793 | 0 | 0 | **0** |
| under −10 dB: 12 Mb/s 50 dB | 0 | 12 | 29 | 483 | **12** |
| hit 8, +20 dB: 12 Mb/s 30 dB | 1911 | 1910 | 1958 | 1910 | **1910** |
| hit 8, +20 dB: 24 Mb/s 30 dB | 2000 | 2000 | 2000 | 2000 | **2000** |
| TGn D: preamble alone, then 54 Mb/s 40 dB | 0 | 188 | 6 | 6 | **6** |
| TGn D: under −25 dB: 54 Mb/s 50 dB | 3 | 2000 | 1 | 1 | **1** |
| TGn D: hit 8, +20 dB: 6 Mb/s 30 dB | 1375 | 1374 | 1644 | 1375 | **1375** |
| TGn D 6 Mb/s 50 dB | 1 | 1 | 3 | 1 | **1** |

   In every one the model is the better of the hold alone and 2 of 4, or within a frame
   of it. The case the rule was built for, the other way round — a 54 Mb/s frame over a
   true 6 Mb/s one that made the hold patient — is released by 24 of 32 (4 lost, 3 with
   2 of 4). A 54 Mb/s frame 10 dB above a 6 Mb/s one is lost in every column, the gain
   held from the lock included: that is the interference, not the hold. The burst at
   24 Mb/s loses the frame whatever holds the gain, so that 24 Mb/s is not a slow rate
   costs nothing in this row. Without the inheritance the 12 Mb/s row is 483.
3. **SIGNAL's agreeing bits in the new rows.** The frame's own SIGNAL through TGn D: 36 to
   39 in 1 of 2000 (6 Mb/s 30 dB), 2 of 2000 (6 Mb/s 50 dB) and 3 of 4000 (54 Mb/s
   50 dB), 40 or more in the rest. Read behind a preamble alone through TGn D: 0 to 35 in
   all 217.

**Measurement: `rxagc loop`** (commit 0b63cb3, the build server, 2000 frames per row;
`reports/rx_front_agc_loop.txt`: 35 rows and the 6 with a DC offset, the ten new rows
among them, and the new column). Every cell of the 31 rows and ten columns of the report
before is unchanged. Frames lost of 2000, and in brackets the frames inside which the gain
moved:

| Row | from the lock | front alone | front + release (2 of 4) | **front + release by rate** |
|---|---|---|---|---|
| preamble alone, then 54 Mb/s 30 dB (idle; loud noise; DC per index; DC before the gain) | 0; 0; 19; 0 | 477; 508; 375; 472 | 1; 2; 5; 4 | **1; 2; 5; 4** |
| TGn D: preamble alone, then 54 Mb/s 40 dB | 0 | 188 | 6 | **6** |
| under −25 dB: 54 Mb/s 50 dB (AWGN; TGn D) | 0; 3 | 2000; 2000 | 2; 1 | **2; 1** |
| under −25 dB: 6 Mb/s; 12 Mb/s | 0; 0 | 43; 1793 | 1; 0 | **2; 0** |
| under −10 dB: 6 Mb/s; 12 Mb/s | 0; 0 | 0; 12 | 9 (97); 29 (48) | **8 (44); 12 (3)** |
| hit 4, +20 dB: 6 Mb/s 30 dB | 91 | 91 | 94 (1993) | **91 (4)** |
| hit 8, +20 dB: 6 Mb/s 30 dB (AWGN; TGn D) | 710; 1375 | 710; 1374 | 1010 (2000); 1644 (1999) | **711 (1); 1375 (2)** |
| hit 8, +10 dB: 6 Mb/s 30 dB | 21 | 21 | 54 (1299) | **21 (3)** |
| hit 16, +10 dB: 6 Mb/s 30 dB | 384 | 383 | 766 (1813) | **384 (2)** |
| hit 8, +20 dB: 12 Mb/s 30 dB | 1911 | 1910 | 1958 (2000) | **1910 (1)** |
| TGn D 6 Mb/s 30 dB; 50 dB | 0; 1 | 0; 1 | 0 (14); 3 (12) | **0 (1); 1 (1)** |
| TGn D 54 Mb/s 40 dB | 0 | 2 | 4 | **4** |

4. **The rule has both ends at 2000 frames, in the model.** Where 2 of 4 gained (after a
   preamble alone, a stronger frame inside a weaker one) the model's rule has 2 of 4's
   count, but for one frame in "under −25 dB, 6 Mb/s". Where 2 of 4 cost (a burst on
   a frame at 6 or 12 Mb/s) it has the hold alone's count, within a frame: the gain moves inside 1 to
   4 frames of 2000 where it moved inside 1299 to 2000.
5. **The other rows lose the same number of frames** in the two release columns (plain,
   SIFS, loud noise, DC), but TGn D 6 Mb/s 50 dB: 1 against 3. At 6 Mb/s the gain moves
   inside fewer frames (2 of 2000 against 5 to 8; through TGn D 1 against 12 to 14).
6. What remains of 2 of 4's cost is at the rates that are not slow: TGn D 54 Mb/s 40 dB
   is 4 against the hold alone's 2, as before.

**The `stream` column with the rule** (`rxagc dc`, `level`, `per` at commit ca2a97d, whose
receiver and `stream` path are those of 0b63cb3; the build server, 2000 frames per row;
the three reports replaced). Against the reports of the third amendment: every other
column identical cell for cell; `stream` unchanged in all of `dc` (49 rows) and `level`
(35), and in `per` (126 rows) two cells changed, both through TGn D at 6 Mb/s: 50 dB 0 of
2000 against 5, 60 dB 0 against 2 — the frames 2 of 4 had cost there. The cells that the
third amendment raised at 24 Mb/s through TGn D (3, 2 and 4 of 2000) stay: 24 Mb/s is not
a slow rate.
One line of these three reports' header was written after the run: the column was named
"the column `front + release` of `loop`" by the binary of ca2a97d and is "`loop`'s column
`front + release by rate`" in the tool now; the reports carry the tool's present line
(checked: `rxagc level 2` of the present tool against the report, the header lines differ
in the number of frames alone). `rxper stream` at 0b63cb3
(`reports/rx_front_stream.txt`): every number as before, one header line names
`decode_signal_checked_fixed`.

**Independent review (fresh agent in its own worktree, the diff at ca2a97d and ADR 0008,
not the conclusions).** No defect found in the rule: the order inside `push`, verdicts
after a release, after the stream's end, repeated, with another tag; windows and counts
of 0 and above 64; no panic path; `patient` and `hold` never read on the beat path. Its
own oracle against the model on 6000 random scenes (2 to 4 frames at four rates, rail
bursts, windows up to 99, counts of 0, verdicts at random times with current, stale and
random tags, five reader paces with stalls): the hold equal at every push and after every
verdict, beats and acquisitions identical to a run with both releases off (16 215
acquisitions, 1994 inherited, 4719 fast and 4415 patient releases). `rxagc release` at 40
frames: "the model" equal to the tool's trial in all 27 rows. `rxper stream` at 100
frames from main and from the branch: identical. Found and corrected (commit 0b63cb3):
- *holes in the gate:* of its 25 mutants of the hand-off 7 survived that are not
  equivalent (listed above); four tests added, all seven fail them. Four more survivors
  are equivalent (the bit cleared or kept at `close`, at a release, at the hold's end by
  length, at a failed SIGNAL): the bit is read only under a hold. Of its 11 mutants of
  `signal_checked` 10 fail `g1_signal_check`; the one that passes (the parity bit forced
  even before encoding again) is equivalent;
- `SignalVerdict::of` could overflow on a `Signal` built by hand with a length the
  decoder cannot give: the length is clamped to the longest PSDU, with a test;
- doc comments of `RxHandoffFixed`, `signal` and `clip_count` that still described 2 of 4
  alone; the tools' headers naming `decode_signal_fixed`; "'2 of 4' is the model's"; the
  rows "6 Mb/s under" did not say the weaker frame of 1000 octets outlasts the stream;
  the usage line without `release`.
Noted by it: two places where the tool's trial and the model can differ by construction
(the trial switches rule in the verdict's own period, the model at the next push; a
declaration at the very sample of the hold's end) — not seen at 40 frames, and not at
2000 (item 1).
Not checked by it: any 2000-frame number, `loop`, `per`, `level`, `dc`; the journal's
numbers; `signal_checked` against a reference outside the crate's own encoder; the tag's
wrap at 256 streams (as before this change); whether the rule is good for a radio.

**What this is not.** One seed per row, one run. 40, 24 and 32 and the three slow rates
were chosen on the rows of the entry before; the ten new rows were run after and nothing
was retuned on them, but they are the same kinds of scene (a frame under, a white-noise
burst, a preamble alone) at other rates and through TGn D — not an interferer that is a
frame of another network, not radar's pulse shape, no adjacent channel. A patient hold
waits 24 samples (1.2 µs) of clipping, so a stronger frame over a believed slow one that
clips less than three samples in four is held through: "under −10 dB" at 6 and 12 Mb/s is
8 and 12 of 2000, and the gain is then right only by the frame's luck. In loud noise a
noise SIGNAL reached 40 to 43 agreeing bits in 4 of 518 (the entry before): it would be
believed if it also named a slow rate. The released control returns to the largest gain
(an assumption about a radio); the rails are the ADC's. No float chain in the loop.

**Open**
- The cycle-exact schedule of the decision, now with the verdict's second bit: when
  `signal()` may arrive in clocks, and that the rule changes at the next sample.
- R1: the rule against captures (how often a clipping burst that is not a frame, how
  often a stronger frame over a slow one).
- As before: the stream's 12 of 10 000 at 24 Mb/s in AWGN above 30 dB; the `dc` row
  −10 dBFS before the gain at 25 dB.

## 2026-11-22 · G0 + model gate · the decision's schedule in clocks (part d step 3, first deliverable)

ADR 0008, decision 4: "the cycle-exact schedule, with the energy ring's margin and
`det_a`'s latency in it, is the first deliverable of part d step 3, before any RTL of the
decision engine". This entry is that schedule. No RTL, and no change to what the model
decides.

**What it is.** `radio_phy::rx_decision::run_schedule` makes every decision of
`RxFrontFixed` again from memories *as they stand in the clock each word is read*: the
history (512 words, one write port, two read ports) and the energy ring (81 words) are
written a sample at a time on the clock grid, and the schedule says in which cycle each
word is read. A word read after it was overwritten or in the cycle that overwrites it, a
read port used twice in a cycle, or a result registered after its deadline is an error.
The gate holds its decisions and their sums to the model's, bit for bit.

**G0: the clocks.** Counts are from cycle to cycle: a block with latency k and its input
in cycle X shows its registered output in cycle X + k. (The RTL headers count clock edges
after the edge that takes the input, one less: `ltf_corr`'s 14 is 15 here, the CORDIC's
17 is 18, the NCO's 3 is 4.) Cycle Y(n) is the one in which sample n of the DC blocker's
output is at the history's write port and at detector A's input; samples are at least R
cycles apart. With the NCO path D samples behind (ADR 0006, decision 3):
- the acceptance at model sample n is a registered strobe in cycle
  a = Y(n + D) + 5 (`rx_latch` and `nco`, as `tb_latch` requires) + 15 (`ltf_corr`) + 3
  (the search: square, running sum, comparison — an allowance for step 2);
- the result is applied at model sample n + L: the run counter of `det_a` is cleared in
  the cycle sample n + L is in its last stage, so the result is due as a register's output
  in cycle Y(n + L) + 4 − 1, and in any case inside that sample's R cycles (the hand-off
  and the gain's hold start with it: a slower, serial `det_a` does not move the deadline
  past the sample);
- with samples exactly R apart: (L − D)·R − 23 + 3 cycles. **460 at R = 8, D = 4,
  L = 64.** D = 4: `rx_latch`'s contract is D·R ≥ 24 with a register for the CORDIC's
  arbiter, which D = 3 meets with nothing to spare.

**G0: the schedule** (cycles after the strobe; w = GI − B, F the fold of the decision's
correlator):

| Cycles | What | Reads |
|---|---|---|
| 0 … w | the grid: the energies at imax … imax + w, oldest first; T1 is the last that keeps 7/8 of the maximum | ring, one a cycle |
| w + 2 | T1 registered (the word, the comparison) | |
| c = w + 3 … c + 95 | final coarse sum: product i from y(T1 − 160 + i) and y(T1 − 144 + i) at E bits, one a cycle on one complex multiplier | ports 0 and 1 |
| c + 99 | the sum registered (the words 1, narrowed 1, the product 1, the accumulator 1), the CORDIC asked; a latch asking in the same cycle goes first and the decision asks again in the next | |
| g + 1; g + 19; g + 20 | the sum at the CORDIC's input (g: the cycle of the grant; the arbiter's register); the coarse angle; the step | |
| r = g + 20 … | y(T1 + m), m = 0 … 126 + w, through a second NCO into the decision's correlator, one a cycle; after each sample that completes a window to be correlated (m = 63 … 62 + w, 127 … 126 + w) the stream waits F − 1 cycles | port 0 |
| f = c + 96 … f + 175 | fine sum on the same multiplier: product i from y(T1 − 28 + i) in cycle f + 2i and y(T1 + 36 + i) in the next | port 1 |
| f + 179; + 19; + 2 | the fine sum registered and the CORDIC asked; its angle; the resolved CFO | |
| last read + 5 | the last sample out of the NCO, at the correlator (memory 1, NCO 4) | |
| + F + 7 | its correlation (`ltf_corr`'s stages with F in place of 8) | |
| + 3 + 1 + 2 | the squares, the sums, the two comparisons | |
| + 1 | **the result registered** | |

Without a wait at the CORDIC: **8·w + 268 cycles at F = 4** (the fine sum's branch ends at
w + 300 and is the longer one only below w = 5). Words read: port 0 96 + 127 + w, port 1
96 + 176, the ring w + 1.

| | w = 8 (back-off 400 ns) | **w = 12 (200 ns, the default)** | w = 16 (none) |
|---|---|---|---|
| cycles, F = 4 | 332 | **364** | 396 |
| spare of 460 | 128 (28 %) | **96 (21 %)** | 64 (14 %) |
| cycles, F = 8 | 396 | 460: none to spare | 524: does not fit |
| history words to spare (of 512), at least | 138 | 130 | 121 |
| ring words to spare (of 81) | 9 | 5 | 1 |

A latch that asks for the CORDIC in the cycle the coarse sum does costs one cycle; one
that meets the fine sum costs nothing above w = 4 (that branch has 52 cycles to spare at
w = 12). On a grid of exactly 8 cycles a sample the stream's own latches (5 cycles after
their sample) can only meet a decision's requests when w is a multiple of 8.
*(Corrected 2026-11-22, step 3a's review: the latch asks 4 cycles after its sample, and
meets a decision's requests when w is 7 more than a multiple of 8; see that entry.)*

**Which counts are whose.** The stages of blocks that exist are theirs: the memory (1),
`cordic` (18), `nco` (4), and the correlator's are `ltf_corr`'s (F + 7). The others are
allowances for RTL not yet written, none through synthesis: the search's 3, the
multiplier's 3 with its accumulator (`nco`'s product alone, from flip-flops, routed with
0.9 ns to spare), the arbiter's 1, the squares' 3, the sums' 1, the comparisons' 2. A
stage more in any of them is a cycle more; 96 cycles are what absorbs that at w = 12.

**What the schedule found against the ADRs' estimates.**
1. **The write does not take a read slot.** ADR 0006's formula stretched every read by
   R/(R − 1) for the history's write. The wrapper as built and gated (`ram_1w2r`,
   `memory.md`) has its own write port: a read is only illegal at the address being
   written, and the decision reads words at least 60 samples old. ADR 0008's "about 406
   clocks, 13 % of slack" by that formula is 364 of 460, 21 %.
2. **The rotation is 127 + w samples, not 128 + w.** The model rotates one more
   (`rot` has 128 + w entries) and no sum reads it.
3. **Two tests of the model are always true at an acceptance** and need no hardware: the
   grid's `p < deadline` (the acceptance is at a position j < deadline with
   imax + w ≤ j − 63) and `kc + HISTORY > n` (n − kc ≤ 349 + 2·w). The model keeps both.
   Of the `checked_sub` cases only T1 < 160 (a stream that begins inside a frame) can
   happen; the hardware needs a saturating sample count for it.
4. **The ring is read oldest first.** At w = 16 it has one word to spare; read newest
   first (the order the model's search runs in) the oldest energy is gone before its
   turn.
5. **F = 8 (16 multipliers in place of 32) has no cycle to spare at w = 12 and does not
   fit at w = 16.** ADR 0008's choice of F = 4 stands on the count as well as on its
   formula.
6. A decision that ends early (no coarse window in the stream, a failed rule) is given
   its whole schedule here; the RTL may answer sooner.
7. **ADR 0008 counted D = 5 and a budget of 464;** with `det_a` as it is the budget is
   460 at D = 4 and 452 at D = 5. A serial `det_a` (R0-08c, then ADR 0008's option C)
   needs D = 5 if its event comes more than 12 cycles after its sample (the latch's
   contract D·R ≥ that latency + 20).

**Gate.** `crates/radio-phy/tests/g2_rx_decision.rs`, eight tests, and four unit tests
(`rx_decision::tests`, `rx_front_fixed::tests`). It is a model gate, as `g2_rx_handoff`
is: no reference from outside the project. References: the decisions and their sums are
`RxFrontFixed::push`'s (`SampleTrace::accepted`, `sums`, new; `decide` is gated by
`g2_rx_front`); the cycle counts, the budget and the memories' margins are worked by hand
from the table above; the trace's energy is checked against the sum of the squared
correlator outputs of the same trace, and an acceptance's deadline against the events.
- `the_schedule_makes_the_models_decisions`: twelve scenes (frames at 30, 6 and 2 dB; a
  strong short-training fragment and a noise burst before a weaker frame; frames under a
  tone of their own power; no back-off, with and without failed candidates and under the
  tone; back-offs of 400 and 50 ns; frames whose second long symbol is negated; a stream
  cut 40 samples into a frame): every acquisition and the five sums of every decision.
- `each_way_out_of_a_decision_is_taken` (no coarse window in the stream; both rules
  failed; the confirmation alone; accepted — no scene fails the coherence alone, and the
  rules' comparison has its own unit test), `a_decision_ends_at_the_clock_the_schedule_says`
  (the counts above, the words read on each port),
  `the_latch_goes_first_at_the_cordic_and_costs_a_clock` (a latch every 4 samples at each
  of its 32 phases; the stream's own latches under a tone, met without back-off and not
  with the default; w = 4, where the fine sum's wait counts; no cycle to spare and one
  wait: late; a latch in every cycle: late),
  `the_memories_hold_what_the_schedule_reads` (the margins; one word fewer and the read is
  stale, in the memory named; a read in the cycle that overwrites),
  `idle_clocks_between_samples_only_add_slack` (the strobe's cycle and the budget with
  the testbenches' idle pattern), `what_does_not_fit_is_refused` (F = 8 with one wait;
  R = 6; D = 17; L = 51; a serial detector's deadline; a decision that ends early),
  `the_models_trace_names_the_energy_and_the_accepted_maximum`.

Seen to fail first: against a module that returned no decision and a length of 0, five of
the first six tests failed and the sixth passed on nothing (it now requires decisions).
The first real model then failed the hand count by 3 cycles: the hand count had the
rotation's last sample correlated, the model had one sample more rotated than any sum
reads (finding 2).
Command: `cargo test -p radio-phy --test g2_rx_decision` — 8 passed;
`cargo test -p radio-phy --lib` — 5 passed.

**Mutations** (each applied to `rx_decision.rs`, the gate and the unit tests run, the
file restored with `cp` and `touch`): 61, of which 59 fail. They cover the grid (order,
first or last, the rule, the reads' cycles and port), each sum's window, lag, port and
read cycles, the rotation's start, waits and phase, the CORDIC's priority and each stage
count, the stream's own latches a cycle either way or left out, the strobe and the
deadline, the memory's rule in the overwriting cycle, the ports' check, and each field of
the result. The two that pass: the ring written in the strobe's cycle instead of the one
before (the margin does not depend on it: nothing to observe), and a fallback for a
decision with a coarse window and no fine one, which cannot happen (removed). Seven more
on the model's side (`confirmed`, the trace's energy and maximum): all fail.

**Independent review (fresh agent in its own worktree; the code, the ADRs and the RTL
headers, not the conclusions).** The schedule is sound in structure: a simulation of its
own (memories of 512 and 81 words addressed modulo their depth, reads at the cycles of
the table) agreed with `run_schedule` and the model in 126 configurations (three scenes,
w from 1 to 16, D = 3 to 5, with and without idle clocks): every word, sum, acquisition,
end, wait, budget and least depth. `scripts/check.sh` green on the change, the vectors
regenerated without a difference. It confirmed that `ram_1w2r` has no write slot to wait
for (finding 1), that the rotation's last sample and the two tests of finding 3 are
dead, the ring's and the history's margins, and the deadline against `det_a` as built.
Found and corrected:
- *the counts mixed two conventions.* The first version took `ltf_corr`'s 14, the
  CORDIC's 17 and the NCO's 3 as cycles; they are edges after the taking edge, one less.
  With that, one register for the arbiter and a stage more for the multiplier (its
  judgement from `nco`'s routed slack) the count went from 359 of 461 to 364 of 460, and
  "F = 8 fits by 6 cycles" to "none to spare". Every count is from cycle to cycle now and
  the module says so;
- *the read cycles were not tied to the result's:* a read could be moved by hundreds of
  cycles and the reported end stayed. Every time is now taken from the memories' own
  record of the reads;
- *the stream's own latches never met a decision* in the gate (their cycle and the
  decision's are 4 apart modulo 8 at w = 12): the tone scene without back-off, where
  they do;
- the ring had no port and its reads no cycle that mattered; the error did not say
  which memory; the history's bound in the test was one short at w = 7 and 15 (a scene
  at w = 15 added); the deadline let a slow detector push the result past its sample; a
  comment said no scene fails one rule alone (the confirmation alone does); the trace's
  deadline was not checked.
Of its 121 mutants 32 passed the first gate; the corrected one fails all it listed as
not equivalent but the ring's write cycle (above).
Noted by it and left: `hw/Makefile`'s latch gate does not run D = 4 at R = 8;
`DecisionTiming` does not check the latch's contract; the file is named `g2_` and is not
a fixed-point budget.
Not checked by it: synthesis or timing of any stage; the macro's access time; R other
than 8 beyond the probes; `FixedRxConfig` off its defaults; whether three stages are
enough for the search.

**What this is not.** Not RTL and not a proof of timing: the allowances are stage counts,
and the squares (54 bits), the comparisons (about 60) and a 12 × 12 complex product behind
a memory's output have not been synthesised. One clock (R = 8) and samples never closer
than R. The schedule of the hand-off (the DC estimate, the replay, when the decoder's
verdict may arrive) is not in it.

**Open (for the RTL of steps 2 and 3)**
- Detection events delayed into the search's timeline, tagged as the latch's results
  are: where `opened_by` and the deadline live.
- A saturating sample count for T1 < 160; the w correlations of the first symbol kept
  64 samples for the coherent sum (about 16 × 54 bits); a reset in the middle of a
  decision (the memories keep their contents).
- L above 63 + w is not lossless in the model, and by the reviewer's probe the ring has
  no word to spare at L = 80.
- The latch gate at D = 4, R = 8; the hand-off's own schedule and the verdict's.

## 2026-11-22 · G0 + G3 · R0-08d part d, step 2: the LTF search (`rx_search`)

**Block.** `hw/rtl/rx_search.sv`; golden: `RxFrontFixed::search_step` with the search control
of `push` step 1 (open, extend) and of `apply_pending` (the acquisition closes the search).
Gate: `make -C hw search` on the chain dc_block → det_a → rx_latch → nco → ltf_corr →
rx_search (`hw/tb/search_chain.sv`), as the top level will wire it. The decision engine
(step 3) does not exist: the vectors say at which sample an acquisition is applied, as they
do for `det_a.clear` in the latch gate.

**G0: what it computes** (w = GI − B, the parameter `W`, 1 … 16; L the decision latency)
- |c|² of each correlator output (K = 20 bits a component, so 2·K bits unsigned), the
  energy e(j) = Σ of the w squares from position j on (2·K + 4 = 44 bits), as a running
  sum: plus the new square, minus the one w back. Exact, so equal to the model's fresh sum.
- The energy ring: 64 + GI + 1 = 81 words of 44 bits through `ram_1w1r`, e(j) at address
  j mod 81, with a read port for the decision (the schedule's grid: imax … imax + w, oldest
  first, one a cycle from the strobe's cycle on).
- The search's state in counts of its own samples, no absolute numbers in any comparison:
  `wait` (150 + w at the opening event, down to 0: position j has reached `first`), `left`
  (389 + w at every event; 0: j + 1 has reached the deadline, the search closes after this
  sample), the running maximum with its age, the decision in flight (L samples from the
  acceptance, counted here).
- Per sample, in the model's order: the event (open, or the deadline moved); the position
  (maximum, acceptance when the maximum is 63 + w positions old); the end of a decision
  in flight (an acquisition closes the search, also one opened at this very sample).

**G0: the timeline.** The search lives on the NCO path's samples, `D_SAMPLES` behind the
detector. `rx_latch` gains `out_event`: the event flag of the sample it emits, kept by the
sample's tag (the detector's flag is there 4 clocks after the sample, the emission D
samples after it: inside the latch's own contract). From the emission the flag and the
sample's tick go down a shift register of `PATH_DELAY` = 19 clocks (`nco` 4, `ltf_corr`
15, cycle to cycle) and meet the correlator's output; a simulation check stops the run if a
correlator output and a tick do not coincide. Events before the first correlator output
(the first 127 samples) open the search as in the model: the tick does not depend on the
correlator.

**G0: clocks** (the schedule's allowance of 3, cycle to cycle from the correlator's
output in cycle C): the squares registered for C + 1; the energy for C + 2, at the ring's
write port in that cycle; the comparison and the state for C + 3, the cycle of the
acceptance strobe: a = Y(n + D) + 5 + 15 + 3, as `rx_decision` assumes. Nothing of it is
through synthesis yet: two 20 × 20 squares and their sum in one clock, a 44-bit sum of
three, a 44-bit comparison with the control behind it.

**G0: the interface to the decision (step 3).** Strobe `acc_valid` with: `acc_n`, the
sample of the acceptance, and `acc_imax`, both sample numbers modulo 2^9 (the history's
depth: its addresses); `acc_lapped`, whether the maximum's position is 512 or more;
`acc_ring`, the ring address of e(imax); `acc_emax`; `acc_opened`, the opening event's
sample modulo 2^9 (a search under a tone stays open without bound: the number is a tag,
not an age). Back: `acquired`, a strobe from the cycle of `acc_valid` to the cycle before
the one in which the search takes its own sample n + L (in the detector's timeline the
result is due D samples and 20 clocks earlier than that); no strobe: not acquired.
- **T1 < 160 is the decision's test, from `acc_lapped`** (added after the review). The
  model gives no decision when the first symbol is less than 160 samples into the stream
  (`checked_sub`; the schedule's open item "a saturating sample count"). While
  `acc_lapped` is 0, `acc_imax` is the position itself and T1 = imax + k can be compared
  with 160; step 3 owns the comparison.
- **The deadline is not handed over.** `Accepted::deadline` has no use in hardware (the
  schedule's finding 3); it lives in `left` and is gated through `searching` per sample.
- **`pending.is_none()` needs no hardware when L ≤ 63 + w.** After an acceptance the next
  maximum starts a position later and cannot be 63 + w old before sample n + 64 + w, which
  is after n + L. An elaboration check requires L ≤ 63 + W; a simulation check stops on an
  acceptance with a decision in flight. (Beyond that bound the model itself reads energies
  that its ring has dropped: the open item of the schedule's entry.) The maximum is
  therefore always exactly 63 + w old at an acceptance, which is what keeps e(imax) in the
  81 words.

**Changed besides.** The generated package gains the search's constants (`SEARCH_OFFSET`,
`SEARCH_LEN`, `SEARCH_W`, `DECISION_LATENCY`, `HISTORY`, `GI_LEN`, `LTF_SYMBOL_LEN`).
`rvec`'s latch scenes moved into a function the search shares; `hw/vectors/latch.txt` is
byte for byte the same. The latch gate now also runs D = 4 at R = 8, the schedule's
default (0 mismatches; the item noted in the schedule's review).

**Vectors** (`rvec search`, `hw/vectors/search.txt`): per sample x, the acquisition applied,
the event, `searching()` after the sample, the energy completed at it and the accepted
candidate (imax, emax, opened by), from `push_traced`; 20 segments and 24 011 samples for
w = 12 and 16, 19 and 22 511 for w = 8 (a segment names its w; a build runs those of its
`W`).
- The eleven scenes of the latch gate.
- A tone burst, then silence: equal (zero) energies after a maximum.
- Two bursts placed, by shift and seed, so that an event falls on a search's last position
  (the deadline moves instead of closing), and on the sample after its close.
- Lone bursts over noise chosen by seed: an acceptance at a search's last position; a
  maximum at its first; a larger energy just before the first.
- A frame with a tone 10 dB above it beginning in its data, placed so that an event falls on
  the sample the acquisition is applied at. (The latch gate's frame under a tone of +3 dB
  does not acquire at a back-off of 400 ns.)
- The reviewer's scene (below): a bare long training field between two tones, placed so
  that the acquisition closes a search opened by an event at its very sample. Not at
  w = 8, where no placement tried acquires.
- The long symbol's signs at full scale.
`rvec` counts sixteen cases per w and refuses to write the file if one is missing (the
reviewer's scene is required at w = 12 and 16): events before the first correlator output
(65), events in an open search, acquisitions (9 / 10 / 7 at w = 12 / 16 / 8), failed
candidates (67 / 62 / 64), searches closed by their deadline (7 / 7 / 6), the placed cases
above (1 to 5 each), maxima below and above position 512 (33 and 43 / 30 and 42 / 30 and
41), second acceptances in one search (53 / 48 / 51).

**Evidence** (`make -C hw search`: five builds, fifteen runs; commit c0c64aa)
| Build | Runs | Compared in each | Mismatches |
|---|---|---|---|
| D = 4, W = 12 | R = 8, 11, 16 | 24 011 samples, 21 251 energies, 76 acceptances, 988 ring words | 0 |
| D = 4, W = 16 | R = 8, 11, 16 | 24 011, 21 171, 72, 1224 | 0 |
| D = 4, W = 8 | R = 8, 11, 16 | 22 511, 19 965, 71, 639 | 0 |
| D = 3, W = 12 | R = 8, 11, 16 | as the first | 0 |
| D = 5, W = 12 | R = 8, 11, 16 | as the first | 0 |

- Seen to fail first: against a module with the ports and no output, every segment failed
  (0 energies and 0 strobes of 19 889 and 22 511, the first vector set). The RTL then
  passed at its first run, which is why the mutations and the review below matter more
  than usual.
- Exact latencies, in clock edges after sample n + D: the energy 22, the strobe 23 — the
  schedule's Y(n + D) + 5 + 15 + 3. (Constants of the testbench; nothing derives them from
  `DecisionTiming`.)
- Per sample: `searching`, the acceptance and its six fields (`acc_n`, `acc_imax` and
  `acc_opened` modulo 512, `acc_lapped`, `acc_ring` = imax mod 81, `acc_emax`).
- After every acceptance the testbench reads the ring as the schedule does (imax … imax + w,
  one a cycle from the strobe's cycle on) and requires the model's energies: at W = 16 the
  ring has the one word to spare the schedule counted, with the real write clock.
- Samples every R clocks plus an idle clock after every 5th and two after every 13th; idle
  inputs carry garbage.
- Before each segment two one-clock resets with state in flight: in the clocks in which the
  search takes an accepted sample (19 … 24 clocks after it, in turn; 18 a run at W = 12),
  and between an acquisition's strobe and the search's own sample of it (8 a run). The
  first version reset 8 clocks after a later sample and three reset mutations passed
  (`p_valid`, `e_valid`, `acc_valid`); the second kind came from the review.
- Registers without a reset, the squares' delay line and the ring start from random values
  (`+verilator+rand+reset+2`).
- `acquired` delayed in a copy of the chain (D = 4, W = 12, R = 8): by 50 clocks 0
  mismatches; by 51 (the cycle the decision ends) and by 52 the run stops on the RTL's
  checks.

**Mutations** (each built from scratch in a private copy of `hw/`, D = 4, R = 8, both W;
the numbers are mismatches, "stopped" a simulation check of the RTL; the table is the third
round, on the gate as committed): 63, of which 58 fail.
| Mutation | W = 12 | W = 16 |
|---|---|---|
| im square from re | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| sum never subtracts | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| sum subtracts one square early | 21727 | 21656 |
| first energy one square early | 43303 | 43378 |
| first energy one square late | 43244 | 43319 |
| energy keeps 32 bits | **passes** | **passes** |
| energy keeps 31 bits | 38 | 50 |
| ring one word short | stopped: ring read and written at one address | stopped: ring read and written at one address |
| ring address not advanced at the wrap | stopped: ring read and written at one address | stopped: ring read and written at one address |
| ring written a cycle late | 2 | 2 |
| first position one early | 27 | 32 |
| first position one late | 16 | 27 |
| deadline one early | 15 | 15 |
| deadline one late | 12 | 10 |
| every event restarts the first position | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| only the opening event sets the deadline | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| the last of equal energies | 3 | 3 |
| accepted one position early | 169 | 161 |
| accepted one position late | 157 | 153 |
| accepted only at the exact age | **passes** | **passes** |
| maximum kept after an acceptance | stopped: acceptance with a decision in flight | stopped: acceptance with a decision in flight |
| maximum kept across searches | 6 | 15 |
| decision ends a sample early | 25 | 25 |
| decision ends a sample late | 9 | 10 |
| acquisition does not close | 706 | 686 |
| every decision closes | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| an event survives the acquisition | 18 | 16 |
| an acquisition closes only a search open before the sample | 9 | 8 |
| acc_lapped is the count's wrap at the acceptance | 9 | 8 |
| wrap flag set at the first position | 33 | 30 |
| wrap flag not reset | 33 | 30 |
| acquired flag not cleared | 104 | 190 |
| deadline closes in spite of an event | 8 | 10 |
| no acceptance at the last position | 2 | 1 |
| opened_by follows every event | 75 | 72 |
| acc_imax is the current position | 76 | 72 |
| acc_ring is the write address | 76 | 72 |
| acc_emax is the current energy | 73 | 69 |
| acc_n one late | 76 | 72 |
| maximum's position one late | 76 | 72 |
| maximum's ring address one late | 76 | 72 |
| state a cycle early | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| event a cycle off the tick | **passes** | **passes** |
| PATH_DELAY 18 | stopped: sample without its correlator output | stopped: sample without its correlator output |
| ticks not reset | stopped: sample without its correlator output | stopped: sample without its correlator output |
| p_valid not reset | 10018 | 9279 |
| np not reset | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| e not reset | 22315 | 22467 |
| e_valid not reset | 9492 | 5577 |
| ring address not reset | stopped: ring read and written at one address | stopped: ring read and written at one address |
| npos not reset | 152 | 144 |
| jpos not reset | 93 | 94 |
| open not reset | 3236 | stopped: `acquired` without a decision |
| busy not reset | **passes** | **passes** |
| acq_seen not reset | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| best_v not reset | **passes** | **passes** |
| out_valid not reset | 3978 | 3061 |
| acc_valid not reset | 2 | 1 |
| latch: event read one tag late | 114 | 114 |
| latch: event written one tag late | 612 | 1181 |
| latch: event of the entering sample | stopped: `acquired` without a decision | stopped: `acquired` without a decision |
| chain: the detector's event undelayed | 180 | 182 |
| chain: acquired never | 706 | 686 |

The five that pass:
- *energy keeps 32 bits*: **a gap of the gate.** The largest energy in the vectors has 32
  bits (31 kept: mismatches). The chain cannot be driven higher: the correlator's largest
  output over full-scale inputs is 108 486 a component (`ltf_corr`'s `matched` segment),
  38 bits of energy at W = 16, and in the chain the detector's events turn the NCO and the
  match is lost. The 44 bits are the bound for K = 20-bit inputs, by argument only; the
  reviewer's masks of the squares and of the comparison at 32 bits pass as well.
- *accepted only at the exact age*, and the age's saturation (removed from the RTL after it
  passed the first round): equivalent. The age is read only while a maximum is held, and a
  maximum is accepted the sample it is 63 + W old.
- *event a cycle off the tick*: equivalent in this chain. `rx_latch` holds `out_event` from
  one emission to the next, so the flag is the same a cycle later.
- *`busy`, `best_v` not reset*: no output changes (the reviewer's analysis as well): a stale
  decision in flight only matters through a remembered `acquired`, which is reset; a stale
  maximum is dropped at the next opening. Both resets stay, for a defined state.

**Independent review (fresh agent in its own worktree; the code, the model and
`rx_decision`'s module text, not the conclusions), on d0fba2e.** No input found on which
`rx_search` or `rx_latch.out_event` differs from the model: its own derivation of the
counters and of the dead condition; a generator of its own over `push_traced` (random
bursts, frames whole and cut, frames under tones, chopped preambles, random idle clocks):
W = 1, 2, 5, 12, 16 at D = 3, 4, 5 with L = 20, 64, 71, 75, 79 in nine sets of 35 scenes,
and W = 2 … 16 at L = 64 and at L = 63 + W, all 0 mismatches; `out_event` at D = 1 to 14
(flag memories of 4, 8 and 16). Of its 100 mutants the gate of d0fba2e killed 61. Found
and corrected:
- *a reset between `acquired` and the search's sample of the result was never made.*
  "`acq_seen` not reset" passed, and I had recorded it as equivalent: it is not (the
  remembered strobe closes the next search at its first failed candidate). The second
  in-flight reset above; the mutant now stops the run.
- *"also a search opened at that very sample" was not in the vectors.* My counter took any
  event at the acquisition's sample, and in the scene the search was already open. Its
  scene and a counter for the case itself; "an acquisition closes only a search open
  before the sample" now fails (9 and 8 mismatches).
- *`acquired` in the cycle the decision ends was lost without a word* (51 clocks late in
  its sweep: 226 mismatches and no `$error`). Unreachable on the schedule; the header now
  names the last legal cycle and a check stops the simulation there.
- *T1 < 160 could not be told from numbers modulo 512* (13 of 228 acceptances in one of its
  sets had imax < 160): `acc_lapped`.
- The journal's entry had G0 only at that commit (the evidence came a commit later); the
  testbench's header overstated the junk samples in flight.
Noted and left: the energy's upper bits (above; its proposal: a bench of the block alone,
or narrower widths with a stated bound); the latencies 22 and 23 are typed constants; the
RTL refuses W = 0, L = 0 and L above 63 + W, which the model accepts; `lint` covers the
default W and D only (the gate's builds cover the others).
Not checked by it: synthesis or timing; R below 8; `FixedRxConfig` off its defaults; a
reset of `rx_search` alone; the decision engine.

**Timing probe** (Vivado 2024.2, out of context, KV260 part, 6.25 ns, `tools/syn/ooc_probe.tcl`
on the build server, sources at c0c64aa; excerpt appended to
`docs/gates/reports/rx_front_timing_ooc.txt`): **+3.05 ns**, 512 LUTs (129 as the ring's
memory), 410 registers, 2 DSP blocks; the worst path is the energy's comparison into the
maximum's enable. The script constrains no input, so that figure does not time the squares;
with an input delay of 0.5 ns on every input (a copy of the script on the server) the
squares are the worst path at **+1.82 ns**. The three cycles of the schedule's allowance
hold on this part. Not run: the open ASIC flow.

**Not covered**
- Energies above 32 bits (above).
- In the committed vectors: `L` other than 64, `W` other than 8, 12, 16 (the reviewer's
  runs covered both); `PATH_DELAY` other than 19.
- R below 8 (the correlator's limit) and clock patterns that remove clocks.
- A search opened at an acquisition's sample at W = 8.

**Open (for step 3)**
- The decision engine's own sample count in the detector's timeline must agree with
  `acc_n` (both count from reset, modulo 512): the top level's gate.
- `acc_opened` is nine bits of the opening event's sample; `FixedAcquisition::detected_at`
  as a full number needs a wider count at the top level if anything uses it.
- The energy's upper bits: a bench of `rx_search` alone driven over the full range of the
  correlator's output, or the widths narrowed to the taps' bound (38 bits), or a formal
  property (G4). The founder's choice at the review of this PR.

## 2026-11-22 · G0 + G3 · R0-08d part d, step 3a: the CORDIC's arbiter (`cordic_arb`)

The first piece of the decision engine: the front has one CORDIC, the latch and the decision
both need it, and until now it sat inside `rx_latch` (the open item of part d step 1).

**G0.** `hw/rtl/cordic_arb.sv`: the CORDIC (`cordic`, unchanged; the angles are the model's
`cordic_angle`) behind an arbiter with one register. Requester a, the latch: a pulse, always
taken (ADR 0006: the latch first). Requester b, the decision engine: holds its request and
its input until `b_grant`, which is every cycle the latch does not ask; a decision that
meets a latch asks again in the next cycle. The owner travels beside the CORDIC's stages;
`a_valid` or `b_valid` marks the angle 18 clock edges after the edge that takes the request:
the schedule's g + 19 (`rx_decision`: the arbiter's register 1, the CORDIC 18, cycle to
cycle).

**`rx_latch`** asks through it (`rq_valid`, `rq_re`, `rq_im`; `a_valid`, `a_angle`). Its
contract is now D_SAMPLES·R ≥ 24 (the sample periods between a sample and its emission
must add up to 24 clocks): D = 3 at R = 8 and D = 4 at R = 6 with nothing to spare; the
schedule's own D = 4 at R = 8 has a sample. The edge was probed with the testbench's
idle pattern at one clock a sample: D = 18 (23 clocks in the shortest span) stops on
`CHECK_TIMING`, D = 19 (24) passes.

**Gate** `make -C hw cordic_arb` (`hw/tb/tb_cordic_arb.cpp`) on `hw/vectors/cordic.txt`:
every request must get the model's angle of its own input on its own valid, exactly 18
edges later, and `b_grant` must be "b asks and a does not" in every cycle. Phases: a alone
back to back; b alone back to back; both at random for 20 000 cycles, b holding its input
while it waits; a in every cycle for 200 with b waiting, then released; a reset with
requests in the pipeline (nothing comes out), and requests after it.
- Seen to fail first: against a module with the ports and no output, 0 angles and 18 662
  mismatches. The first real version then failed 7310 times: the owner was a stage short
  of the CORDIC (an angle on the other requester's valid whenever the two alternated).
- Result (commit 8569271 and its review fixes, squashed into the PR of this entry): 7476 angles for the latch, 8805 for the decision, 4139 cycles with
  both asking, 0 mismatches.
- The latch gate through the arbiter: five runs, 0 mismatches. The search gate: fifteen
  runs, 0 mismatches.

**Mutations** (each built from scratch in a private copy of `hw/`; mismatches in the arbiter's
gate, and in the latch gate at D = 3, R = 8, where the decision never asks): 14, all fail
in at least one.

| Mutation | `cordic_arb` | latch |
|---|---|---|
| b before a | 4139 | passes |
| b's input when both ask | 3046 | passes |
| im from re's choice inverted | 14568 | 20013 |
| grant under a | 3703 | passes |
| grant never with a idle | 14085 | passes |
| valid only for a | 8805 | passes |
| valid not reset | 1 (after the testbench's change below) | passes |
| owner a stage early | 7310 | stopped by `CHECK_TIMING` |
| owner always a | 8805 | passes |
| no arbiter register (valid) | 39874 | stopped by `CHECK_TIMING` |
| latch: request a cycle late | — | stopped by `CHECK_TIMING` |
| latch: im request from re | — | 20541 |
| latch: request at every event | — | 12192 |
| chain: the latch takes b's answers | — | stopped by `CHECK_TIMING` |

- "valid not reset" passed the first testbench, whose reset came with the inputs idle: a
  request at the inputs in the reset's own cycle is now part of it.
- A mutant that never grants hung the first testbench (it waited for the grant without
  a bound); the wait is four cycles now and an error after them.
- The mutants on the decision's side pass the latch gate, as they must: nothing asks on
  that side until step 3b.

**Pre-PR review (fresh agent in its own worktree; the code, `rx_decision` and the ADRs).**
The arbiter agrees with the schedule on the decision's side (who wins, one cycle a
collision, the angle in g + 19), and the contract's 24 is exact (its probes: 23 clocks stop,
24 pass, at one and at two clocks a sample). Found and corrected:
- *the schedule model had the latch ask a cycle late.* `rx_decision` put the stream's own
  latches 5 cycles after their sample; `rx_latch` asks in the event's own cycle, 4 after
  (as it has since step 1: its request is combinational on the event). No decision's value
  depends on it, but which decisions wait a cycle does, and step 3b will be held to those
  cycles. The model follows the RTL as built: `own` is `det_latency` cycles after the
  sample. In its gate the stream's latches now meet a decision's requests at a back-off of
  50 ns (w = 15: 8 coarse and 9 fine waits in the tone scene) and at neither 0 nor 200 ns;
  the first version had them meet without back-off. `cargo test -p radio-phy --test
  g2_rx_decision`: 8 passed.
- The texts: `rx_latch`'s count of the 24 mixed two conventions; D = 4 at R = 6 has no
  clock to spare either (the header and the Makefile said otherwise); ADR 0006's "≈ 23
  clocks" has a dated note; the arbiter's header now says that a latch in every cycle
  starves the decision and that both sides share the reset.
Its mutants: owner from the decision's request 4139, owner a stage late 8246, the CORDIC's
valid chain not reset 1160; two on the reset's own cycle pass (`b_grant` gated with the
reset; the decision alone taken during a reset): the contract above, not a gate.
Not checked by it: synthesis (the path from the event through the request to `b_grant`
and the input's multiplexer); a reset longer than a clock.

**Open**
- A request of the decision in a reset cycle is not defined beyond "dropped": step 3b's
  engine shares the reset.
- The decision's side is driven by the testbench only; its first real user is step 3b.
- `docs/gates/silicon_probe.md`'s figures for `rx_latch` include the CORDIC it no longer
  holds; not rerun.
