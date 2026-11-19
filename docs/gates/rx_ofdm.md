# Gate journal: OFDM receiver after acquisition

Block: channel estimation, pilot tracking, equaliser, soft demapper and the decode of SIGNAL and
DATA down to the PSDU (R0-09), IEEE 802.11-2020 §17.3. Model: `crates/radio-phy/src/rx_ofdm.rs`,
float. Mirror of `ppdu.rs` (R0-06); feeds `viterbi.rs` (R0-10); fed by `rx_front.rs` (R0-08).

## 2026-11-19 · G0 · definitions

- **Input:** the stream after the front's CFO correction, and `fft_start`: the first sample of
  T1's FFT window, back-off included (`Acquisition::fft_start`). Symbol i after the long
  training field (0 = SIGNAL) has its window at `fft_start + 128 + 16 + 80·i`.
- **Channel estimate:** least squares on the 52 used subcarriers,
  H(k) = (T1(k) + T2(k)) / (2·L(k)). No smoothing across subcarriers.
- **Pilots:** P(k) = Z(k)·H*(k)·s(k) at k = −21, −7, 7, 21, with s the transmitted ±1 (pilot
  base × polarity p_i; SIGNAL uses p_0).
- **Timing loop** (sample-clock offset): the slope predicted from the earlier symbols is
  removed from the pilots; the slope error ε is the least-squares fit over the pairs ±21 and
  ±7, weighted by their magnitudes. slope ← prediction + ε/32, rate ← rate + ε/4096
  (`TIMING_GAIN`, `TIMING_RATE_GAIN`). When the slope passes half a sample the FFT window moves
  by one sample for the next symbol.
- **The loop's start:** from the clock offset the front's CFO implies, ppm = −CFO / carrier
  (`Tracking::from_clock_offset`). Both ends derive carrier and sample clock from one
  oscillator (§17.3.9.5–6 for the transmitter; our receiver by design). Started from zero
  (`decode_ppdu`) the loop still pulls in, slowly: enough for BPSK to 16-QAM at ±40 ppm, not
  for 64-QAM.
- **Common phase:** φ = ∠Σ P(k)·e^(−j·k·slope), removed from every data subcarrier.
- **Equaliser:** point(k) = Z(k)·H*(k)·e^(−j(φ + k·slope)) / (|H(k)|²·K_MOD), on the integer
  grid of the mapper (±1, ±3, ±5, ±7). Weight w(k) = |H(k)|² over the mean |H|² of the 52 used
  subcarriers.
- **Demapper:** per axis, positive = bit 0, unit = half the minimum distance. First bit −v;
  then each further bit folds the axis: |v| − 2 for 16-QAM; |v| − 4, then ||v| − 4| − 2 for
  64-QAM. This is the max-log metric near the decision boundaries and smaller in magnitude
  away from them.
- **Soft value:** round(metric · w · soft_max / clip), saturated to ±soft_max; 7 and 1.5 by
  default, the Viterbi decoder's contract (`viterbi.md`). Rounding is half away from zero.
- **Deinterleaving** per symbol, depuncturing and decoding by R0-10 with the termination at
  16 + 8·LENGTH + 6.
- **SIGNAL:** even parity over the first 17 bits; RATE must be one of the eight; LENGTH 1…4095.
  The reserved bit is not checked.
- **Scrambler seed:** from the first seven decoded bits of SERVICE (they are zero before
  scrambling). An error there loses the frame.

Not in this block yet: fixed point (G2), channel-estimate smoothing, decision-directed
tracking, a SIGNAL plausibility check beyond parity, anything for HT/HE.

## 2026-11-19 · G1 · float receiver

**References we did not write**
| # | Reference | Test (`tests/g1_rx_ofdm.rs` unless stated) |
|---|---|---|
| 1 | The standard's own waveform: the 881 printed samples of Annex I.1 (Tables I-22…I-30) decode to its PSDU, 36 Mb/s, 100 octets, at window back-offs 0, 2, 4, 8, 15 | `g1_annex_i1::annex_waveform_decodes` (local fixture, `--ignored`) |
| 2 | The transmit chain (gated against Annex I.1 and OpenOFDM): 194 clean frames, all eight rates, 1 to 4095 octets, four back-offs, random seeds | `every_rate_and_length_comes_back` |
| 3 | The max-log metric by brute force over the constellation | `demapper_against_brute_force_max_log`, `demapper_inverts_the_mapper` |
| 4 | Textbook symbol error rates in AWGN with the true channel | `equaliser_error_rate_with_the_true_channel_is_the_textbook_one` |
| 5 | Textbook loss of a two-symbol least-squares estimate: error vector 1.5 σ², 1.76 dB | `least_squares_estimate_raises_the_error_vector_by_1_76_db` |
| 6 | CommPy's soft-decision Viterbi BER (black box, ADR 0005) through demapper, deinterleaver, depuncturer and decoder | `coded_bit_error_rate_matches_commpy` |
| 7 | A known residual CFO: the common phase reported per symbol | `pilots_track_a_residual_frequency_offset` |
| 8 | A known sample-clock offset: the timing drift reported per symbol | `pilots_track_a_sample_clock_offset` |

This closes the receive half of the R0 exit gate on a clean channel: the model decodes the
standard's worked example from the standard's own samples.

**Evidence** (`cargo test --release -p radio-phy --test g1_rx_ofdm -- --nocapture`)
- Demapper: same decision as max-log on every point of a 145 × 145 grid, never more confident;
  equal to it on all metrics for BPSK and QPSK, 61 % for 16-QAM, 55 % for 64-QAM.
- Symbol errors with the true channel: +0.49, +0.63, +0.59, −0.49 σ from theory (BPSK 6 dB,
  QPSK 9 dB, 16-QAM 16 dB, 64-QAM 22 dB).
- Error vector with the receiver's estimate: 1.498, 1.509, 1.502, 1.505 σ² (theory 1.5).
- Coded BER, fine soft values, true channel:

  | Rate | Eb/N0 | Ours | CommPy |
  |---|---|---|---|
  | 1/2 | 1.5 dB | 1.506·10⁻² | 1.520·10⁻² ± 0.05 |
  | 1/2 | 2.5 dB | 1.47·10⁻³ | 1.62·10⁻³ ± 0.13 |
  | 3/4 | 3.0 dB | 6.14·10⁻³ | 6.59·10⁻³ ± 0.36 |
  | 3/4 | 4.0 dB | 2.8·10⁻⁴ | 3.9·10⁻⁴ ± 0.7 |

- Residual CFO of 700 and −1900 Hz over 57 symbols at 54 Mb/s: common phase within 0.004 and
  0.011 rad of 2π·f·Δn/fs; the same frame is lost without pilot tracking.
- Sample-clock offset of ±40 and +13 ppm, 4095 octets at 6, 24 and 54 Mb/s, the loop started
  0.2 ppm off: all decoded; the tracked drift (up to ±4.38 samples over 1367 symbols, with
  window moves) within 0.001 samples of ppm·Δn; lost without the timing loop. Started from
  zero, 6 and 24 Mb/s still decode.
- Soft-value contract: a noiseless BPSK point on a mean-gain subcarrier is ±5, 1.5 and beyond
  ±7, weight 0 gives 0 (`soft_value_contract`), and the values come back in coded order.
- Channel-state weights: two equal paths two samples apart (nulls near k = ±16), 12 Mb/s at
  7 dB: 0 of 150 frames lost; 130 of 150 without the weights (mutation).
- Through the float front (acquisition, CFO correction) at every modulation, AWGN and TGn D:
  `through_the_front_and_a_channel`.

**Two tests were wrong before the receiver was, and the receiver showed it**
- The first form of reference 5 expected the 1.76 dB in BPSK hard decisions. It is not there:
  an amplitude error of the estimate cannot move a BPSK point across its only boundary
  (measured 0.3 dB). The loss is in the error vector, where the test now looks.
- The first form of reference 6 counted errors after the descrambler. One error in the seven
  seed bits loses the whole frame, which inflated the BER by half. It now compares the decoder's
  output with the transmitter's scrambled bits (`decode_data_bits`).

## 2026-11-19 · measurements · the whole float receiver through the channel simulator

`rxper` (`crates/radio-tools/src/bin/rxper.rs`): float front (acquisition, CFO correction), then
this receiver. Each frame has a clock offset between the two ends, uniform in ±40 ppm, which
sets its CFO (±232 kHz at 5.8 GHz) and its sample-clock offset; the timing loop starts from
the front's CFO. 1000-octet PSDUs (the frame of the standard's sensitivity requirement,
§17.3.10.2). SNR over the whole sampled band. Report: `reports/rx_ofdm_per.txt`.
These are measurements of the model, not gates: nothing outside the model confirms a PER curve
yet (R1's captures and an outside receiver on the same samples will).

**SNR at 10 % PER, dB** (300 frames per point in AWGN, 200 in TGn; a new static channel per
frame)

| Rate | AWGN, 4-bit soft | AWGN, fine soft | TGn B, 4-bit | TGn B, fine | TGn D, 4-bit | TGn D, fine |
|---|---|---|---|---|---|---|
| 6 Mb/s | 3.7 | 3.6 | 11.1 | 11.0 | 8.6 | 8.4 |
| 9 Mb/s | 4.7 | 4.7 | 15.4 | 14.5 | 15.1 | 11.9 |
| 12 Mb/s | 6.8 | 6.8 | 13.9 | 13.8 | 12.7 | 12.5 |
| 18 Mb/s | 8.8 | 8.8 | 19.0 | 17.9 | 18.4 | 16.8 |
| 24 Mb/s | 11.7 | 11.5 | 19.1 | 19.0 | 17.2 | 17.0 |
| 36 Mb/s | 14.8 | 14.6 | 24.0 | 22.7 | 23.7 | 20.9 |
| 48 Mb/s | 18.8 | 18.6 | 26.0 | 25.8 | 24.6 | 24.4 |
| 54 Mb/s | 20.1 | 20.0 | 28.6 | 28.0 | 28.1 | 26.5 |

- In AWGN the Viterbi decoder's 4-bit soft values (clip 1.5) cost at most 0.2 dB, as its own
  G2 found.
- In fading they cost 1.0 to 3.2 dB at the rate-3/4 modes (9, 18, 36, 54 Mb/s) and 0.1 to 0.3 dB
  at the others. The weights |H|² spread the soft values over more than four bits hold: weak
  subcarriers round to 0 (erasures) and strong ones clip, and a rate-3/4 code has no
  redundancy to spare. At 9 Mb/s in TGn B the 4-bit curve does not reach 1 % by 20 dB. This is
  open question 1 of `viterbi.md`, answered: the format has to be chosen again on the whole
  receiver in fading. `reports/rx_ofdm_soft.txt` is the first sweep for that choice.

**Sample-clock offset** (40 frames per cell, high SNR): with the timing loop, no frame is lost
at any length, rate or offset up to ±40 ppm, started from the CFO or from zero. Without it,
every 1000-octet frame at ±40 ppm is lost at 6 and 54 Mb/s, and every 4095-octet frame from
±10 ppm.

**The timing loop's gains** (300 frames near 10 % PER; frames lost)

| Operating point | 1/8, 1/250 | 1/16, 1/1000 | 1/32, 1/4000 | 1/64, 1/16000 | 0 ppm, no loop | 0 ppm, slowest loop |
|---|---|---|---|---|---|---|
| AWGN, 6 Mb/s, 3.5 dB | 52 | 46 | 43 | 39 | 26 | 31 |
| AWGN, 54 Mb/s, 20 dB | 46 | 42 | 36 | 35 | 40 | 39 |
| TGn D, 6 Mb/s, 8.5 dB | 55 | 52 | 53 | 54 | 33 | 38 |
| TGn D, 12 Mb/s, 12.5 dB | 40 | 40 | 37 | 36 | 25 | 26 |
| TGn D, 48 Mb/s, 24.5 dB | 40 | 38 | 38 | 37 | 34 | 35 |
| TGn D, 54 Mb/s, 28.5 dB | 26 | 25 | 23 | 21 | 21 | 21 |

- Started from the CFO, the slower gains are better or equal everywhere; 1/32 and 1/4096 are
  the defaults. Started from zero the slow gains lose 64-QAM frames at ±40 ppm (95 and 132 of
  300 at 54 Mb/s, 20 dB), which is why the start from the CFO is part of the design.
- The first version of the loop (1/8, 1/250, started from zero) cost up to 1 dB in TGn D.
- At 6 and 12 Mb/s the frames with a clock offset lose more than the same receiver at 0 ppm
  (39 against 31, 54 against 38, 36 against 26), whatever the gains. The differences are
  1.4 to 2.3 standard deviations each and all in one direction. Not explained yet: open
  question 2.

**Open questions**
1. The soft-value format in fading (above). A G2 decision of the Viterbi decoder's input, so
   the founder's (it was approved at 4 bits on AWGN evidence, with this caveat recorded).
2. What a clock offset costs at the low rates beyond the loop itself, if anything.
3. The scrambler seed is a single point of failure: one error in seven bits loses the frame.
   The standard gives no protection; a receiver can at best check the result against the
   reserved SERVICE bits (9 zeros).
4. No smoothing of the channel estimate: 1.76 dB of error vector is left on the table in flat
   channels. Smoothing trades it against delay spread; a G2 question with TGn E and F.
5. The demapper's closed form is below the max-log metric away from the boundaries (16- and
   64-QAM). Its cost has not been separated from the quantisation's.
6. The reserved SIGNAL bit and implausible LENGTH values are not used to reject a false
   SIGNAL: parity alone passes half of all noise.
7. Fixed point (G2) for every step of this block.
