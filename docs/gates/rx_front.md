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
   transient detections. Defined with the AGC (R1/R3).
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
