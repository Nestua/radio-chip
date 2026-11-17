# Gate journal: OFDM modulator

Block: 802.11a/g OFDM symbol: 48 data + 4 pilot subcarriers, 64-point IFFT, 16-sample cyclic
prefix (IEEE 802.11-2020 §17.3.5.9–17.3.5.10). Model: `crates/radio-phy/src/ofdm.rs`.
RTL: not yet (R2). **Risk point (PHY architecture): merged only with the founder's approval.**

## 2026-11-17 · G0 · first commit
- Layout: data subcarriers −26…26 without DC and pilots, assigned in ascending order; pilots at
  −21, −7, 7, 21 with base values 1, 1, 1, −1 times polarity p_n; bin = c mod 64.
- Pilot polarity p_n: scrambler (R0-01) with the all-ones state, 0 → +1, 1 → −1, period 127.
  SIGNAL uses p_0, DATA symbol k uses p_(k+1) (the PPDU builder passes the index).
- IFFT architecture: radix-2 decimation in time, 6 stages, each butterfly output scaled by ½
  (the 1/64 of the inverse DFT; magnitudes never grow, so no overflow). One rounding per
  butterfly output: (a·2^Ft ± w·b) narrowed from Fd+Ft+1 fractional bits, half-up, saturating.
  Twiddle ROM: 32 entries e^(+j2πk/64), half-even.
- Number formats: data and twiddles Q(W, W−2) (range ±2 holds 7/√42 ≈ 1.08 and +1 exactly).
  Frequency bins: K_MOD·level quantised half-even (a small constant ROM); pilots ±1 exact.
- **W = 13 for transmit** (`TX_IFFT_WORD_BITS`), chosen by G2 below.

## 2026-11-17 · G2 · first commit
Command: `cargo run -q --release -p radio-tools --bin g2 -- ofdm` (200 random symbols per
modulation, SQNR of the 80 fixed-point samples against the float model with a direct DFT).
Budget: ≥ 45 dB (802.11a requires −25 dB EVM at 54 Mb/s; this leaves 20 dB for the rest of
the transmitter). Rule: the smallest W that clears the budget by ≥ 3 dB at worst → W = 13.

| W | BPSK | QPSK | 16-QAM | 64-QAM | worst | mean |
|---|---|---|---|---|---|---|
| 11 | 39.3 | 37.6 | 37.3 | 37.0 | 37.0 | 39.5 |
| 12 | 44.8 | 44.1 | 43.4 | 43.6 | 43.4 | 45.6 |
| **13** | 51.2 | 50.3 | 49.1 | 50.3 | **49.1** | 51.9 |
| 14 | 56.8 | 57.3 | 55.5 | 55.9 | 55.5 | 58.2 |
| 16 | 70.1 | 68.0 | 67.3 | 68.0 | 67.3 | 70.2 |

(minimum SQNR in dB per modulation; full table W = 8…20 from the command above: ≈ 6 dB/bit.)
Observation: with ½ at every stage the time samples sit ~24 dB below full scale (RMS ≈ √52/64),
so about 4 bits of the word carry no signal. Scaling only some stages would reach the same SQNR
with about W = 10; that is an RTL-area optimisation for R2 (open question 2).

## 2026-11-17 · G1 · first commit
Commands: `cargo test -p radio-phy --test g1_ofdm --test g1_ofdm_openofdm_rx`
References:
1. OpenOFDM receiver tables (Apache-2.0, commit ea638eb), printed unmodified
   (`tools/ref/ofdm_constants_openofdm.py` → `tests/fixtures/ofdm_constants_openofdm.txt`):
   127-entry polarity sequence, pilot and occupied subcarriers, FFT bins — all match.
2. Textbook inverse DFT (direct sum, not radix-2): fixed-point symbols within the 45 dB budget
   for 80 random symbols; single-bin impulses give e^(+j2πkn/64)/64 (direction and 1/64).
3. **OpenOFDM's receiver decodes our symbols**: `rvec ofdm-symbols` (8 per modulation, polarity
   indices 0–7) → `ChannelEstimator.next_symbol` run unmodified in `python:2.7-slim`
   (`--platform linux/amd64`, numpy 1.16.6 FFT; flat channel, no CFO set directly) →
   `tests/fixtures/ofdm_rx_openofdm.txt`. The test requires today's model to reproduce the
   recorded samples bit for bit, then checks the receiver's verdict:
   worst EVM 0.25 / 0.33 / 0.35 / 0.32 % (BPSK/QPSK/16/64-QAM, ≈ −49 dB), worst |β| 0.002 rad
   (a wrong pilot sign would give β ≈ π).
Result: 7/7 pass. Test-first check: the six model tests failed against the `todo!()` stub
(the receiver fixture can only exist after the model, by construction).
Mutation check (each restored with `cp`):
| Mutation | g1_ofdm | openofdm_rx |
|---|---|---|
| Pilot base 1,1,1,−1 → 1,1,1,1 | passes | fails |
| Polarity mapping inverted | 1 fails | fails |
| Twiddle sign (forward FFT) | 2 fail | fails |
| Bit reversal off by one | 2 fail | fails |
| No ½ per stage | 2 fail | fails |
| Cyclic prefix from the start | 1 fails | fails |
The pilot base values are pinned only by the OpenOFDM verdict (β on the recorded symbols) plus
the bit-exact regression; any datapath change requires regenerating that fixture.

## 2026-11-17 · G1 · the standard's worked example
Command: `cargo test -p radio-phy --test g1_annex_i1 -- --ignored` (needs the local fixture; see
`docs/gates/ppdu.md`, entry "the standard's worked example").
Reference: IEEE 802.11-2024 Annex I.1, Tables I-11 and I-20 (pilots with p_0 and p_1),
I-24…I-30 (time samples of SIGNAL and the six DATA symbols, inside the whole-frame check).
Result: see the PPDU journal — worst sample error 0.0005 at W = 24 (print rounding) and 0.0013 at
the transmit W = 13. Mutation: PILOT_BASE [1,1,1,−1] → [1,1,−1,1] fails I-11, I-20 and the samples.

## Open questions
1. Stage scaling schedule for the RTL (fewer ½ stages, narrower word) — decide with the RTL in R2
   with this G2 tool.
3. Output gain toward the DAC (AD9361 is 12-bit) belongs to the transmit front end, not here.
