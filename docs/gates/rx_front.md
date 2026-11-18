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
- L ≤ D (no overlap; the LTF case): var(φ̂) = (1/(L·SNR))·(1 + 1/(2·SNR));
- L ≥ D (overlapping; the STF case): var(φ̂) = (D·SNR + L/2)/(L²·SNR²);
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
