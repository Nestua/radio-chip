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
- **Timing tracker** (sample-clock offset). Each symbol's pilots measure the slope of the
  phase across the subcarriers left after the predicted slope is removed: a least-squares fit
  over the pairs ±21 and ±7. A pair's weight is the harmonic combination of the channel gains
  |H|² at its two pilots (one weak pilot makes the pair worthless), and the same gains give
  the measurement's variance against a flat channel of the same mean gain; a measurement more
  than three times worse than nominal is not used (`TIMING_MAX_VARIANCE`). A clock offset makes the slope grow
  linearly, slope(t) = rate·t, t in symbols from the middle of the long training field (1.4
  for SIGNAL). The measurement also has a part that does not grow: the channel estimate's own
  noise on the four pilots looks like a constant slope, about 0.1 samples at the sensitivity
  limit. The tracker therefore fits measured(t) = offset + rate·t by recursive least squares
  and removes only rate·t from the data. When the slope left in the window passes half a
  sample, the FFT window moves by one sample for the next symbol.
- **The tracker's prior:** the offset's is flat; the rate's is ±40 ppm around zero when nothing
  is known (`Tracking::default`), ±2 ppm around the clock offset the front's CFO implies when
  it is given (`Tracking::from_clock_offset`), with 0.016 rad per subcarrier assumed for one
  measurement (`TIMING_MEASUREMENT_STD`: what it is at the sensitivity limit, 0.0155 measured
  at 3.5 dB).
- **CFO to clock offset:** both ends derive carrier and sample clock from one oscillator
  (§17.3.9.5–6 for the transmitter; our receiver by design), so a transmitter p ppm fast is
  also p·10⁻⁶·fc high: ppm = +CFO / carrier · 10⁶.
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
| 8 | A known sample-clock offset: the timing drift reported per symbol | `pilots_track_a_sample_clock_offset`, `timing_loop_from_zero_has_no_standing_error`, `window_follows_a_drift_longer_than_the_guard_interval` |
| 9 | A symbol built in the frequency domain with a known slope and common phase, on a channel with two pilots in a null and two of unequal gain | `slope_and_phase_on_unequal_pilots` |
| 10 | A channel estimate tilted by 0.3 samples on the four pilots only: a constant slope the data do not have | `a_slope_only_the_pilots_have_is_not_applied_to_the_data` |
| 12 | A channel with nulls at one pilot of each pair (three taps with zeros towards subcarriers +7 and −21), against the same receiver without the tracker | `a_weak_pilot_makes_its_pair_worthless` |
| 11 | Physics of a clock offset: a tone through `resample(+p)` is p·10⁻⁶ higher, so a fast transmitter is also high in carrier | `clock_offset_and_carrier_offset_have_one_sign` |

This closes the receive half of the R0 exit gate on a clean channel: the model decodes the
standard's worked example from the standard's own samples.

**Evidence** (`cargo test --release -p radio-phy --test g1_rx_ofdm -- --nocapture`: 19 tests
pass; `g1_annex_i1 -- --ignored` with the local fixture: 5 pass; this entry's pull request is
the commit)
- Demapper: same decision as max-log on every point of a 145 × 145 grid, never more confident;
  equal to it on all metrics for BPSK and QPSK, 61 % for 16-QAM, 55 % for 64-QAM.
- Symbol errors with the true channel: +0.19, −1.15, +0.63, +0.76 σ from theory (BPSK 6 dB,
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
- Sample-clock offset of ±40 and +13 ppm, 4095 octets at 6, 24 and 54 Mb/s, the tracker
  started 0.2 ppm off: all decoded; the tracked drift (up to ±4.38 samples over 1367 symbols,
  with window moves) within 0.003 samples of ppm·Δn; lost without the tracker. Started from
  zero, 6 and 24 Mb/s still decode.
- Started from zero at +40 ppm: the estimate is off by at most 0.054 samples in the first 200
  symbols and by 0.0003 after 700.
- The window move: within the standard's limits (±40 ppm, 4095 octets) the drift stays under
  4.4 samples, and a window with a back-off decodes clean frames without moving. Its gate is
  therefore ten times the worst case: ±400 ppm over 5.5 ms, 43.7 samples of drift, decoded.
- Slope and common phase on unequal pilots: the measurement is 0.029452 for a true slope of
  0.029452; with that slope removed, the phase is 0.399999 for 0.4 and every point is within
  0.02 of the grid.
- A slope only the pilots have (reference 10): the tracker puts 0.02951 of a 0.02945 tilt
  into its offset, applies at most 0.0006 samples of drift, and the 54 Mb/s frame has no bit
  error; from a clock offset of 0, none at all.
- Nulls at one pilot of each pair (reference 12), 6 Mb/s at 8 dB, 120 frames: 0 lost started
  from the CFO, 5 from zero, 0 without the tracker.
- The sign of the CFO-to-clock-offset rule (`clock_offset_and_carrier_offset_have_one_sign`):
  a 1 MHz tone through `resample(+1000 ppm)` comes out at 1 001 000.0 Hz, so a transmitter
  that is fast is also high; a 54 Mb/s frame with a fixed tracker decodes with +40 ppm and
  not with −40.
- Hostile arguments (`hostile_arguments_do_not_panic`): window starts and shifts beyond any
  stream, lengths no SIGNAL can carry, a symbol of the wrong size, senseless soft settings:
  errors, never a panic or an overflow (under `cargo test`, as `scripts/check.sh` runs it,
  overflow checks are on; the `--release` command above has them off).
- Soft-value contract: a noiseless BPSK point on a mean-gain subcarrier is ±5, 1.5 and beyond
  ±7, weight 0 gives 0 (`soft_value_contract`), and the values come back in coded order.
- Channel-state weights: two equal paths two samples apart (nulls near k = ±16), 12 Mb/s at
  7 dB: 1 of 150 frames lost; 130 of 150 without the weights (mutation).
- Through the float front (acquisition, CFO correction) at every modulation, AWGN and TGn D:
  `through_the_front_and_a_channel`.

**Mutations** (`rx_ofdm.rs`, one at a time; every one fails the tests named)
| Mutation | Fails |
|---|---|
| estimate from T1 only | error vector |
| sign of bit 0 | 8 tests |
| last fold of 64-QAM | 5 tests |
| deinterleaving direction | 6 tests |
| pilot base signs ignored | CFO tracking, through the front |
| pilot polarity ignored | 3 tests |
| no channel-state weights | notched channel (130 of 150 frames lost) |
| weights not normalised | CommPy BER |
| symbol start without the guard interval | 5 tests |
| termination without the tail | 4 tests |
| no parity check | SIGNAL errors |
| LENGTH MSB first | 5 tests |
| octets MSB first | 3 tests |
| grid without K_MOD | true-channel error rate, all rates, through the front |
| soft scale ignores the clip | CommPy BER, soft-value contract |
| no derotation; derotation sign | CFO tracking, through the front |
| the fit's gain sign | 6 tests |
| covariance never shrinks | SFO tracking, no standing error |
| time counted from SIGNAL instead of the training field | SFO tracking, unequal pilots, long drift |
| slope not applied to the data | sign of the rule, SFO tracking, unequal pilots |
| window's move left out of the prediction; window moves the wrong way | SFO tracking, no standing error, long drift |
| no window move | long drift |
| drift per symbol computed for 64 samples instead of 80 | SFO tracking, unequal pilots, long drift |
| a fixed tracker that learns | sign of the rule |
| inner pilot pair ignored | unequal pilots |
| the offset removed from the data as well; the innovation without the offset; the offset never updated; the CFO's prior as wide as none | a slope only the pilots have |
| every measurement at the nominal variance; no rejection of worthless measurements; a pair's weight as the sum of its gains | a weak pilot |

The last row is the tracker's point, and its first mutation run left three of those four
alive: nothing tested that a slope which only the pilots have stays out of the data.
`a_slope_only_the_pilots_have_is_not_applied_to_the_data` was written for them (reference 10:
a channel estimate tilted on the pilots alone; the data must decode with no drift applied).

**What was wrong first, and what showed it**
- *A test's theory.* The first form of reference 5 expected the 1.76 dB in BPSK hard
  decisions. It is not there: an amplitude error of the estimate cannot move a BPSK point
  across its only boundary (measured 0.3 dB). The loss is in the error vector.
- *A test's bookkeeping.* The first form of reference 6 counted errors after the
  descrambler. One error in the seven seed bits loses the whole frame, which inflated the BER
  by half. It now compares the decoder's output with the transmitter's scrambled bits.
- *The sign of the clock offset.* The first version took ppm = −CFO/carrier, and `rxper`
  built its frames with the same wrong sign, so every test and report agreed. The pre-PR
  reviewer derived the sign from `resample` itself; with the physical pairing the hinted
  start lost 186 of 300 frames where a start from zero lost 92. Fixed, with a test from
  physics rather than from our own scenario. (`channel::resample`'s comment said "receiver
  clock fast" for what is a fast transmitter; corrected.)
- *The timing loop.* The first tracker was a proportional-integral loop on the measured
  slope. It lost 30 to 40 % more frames than no tracker at 0 ppm at the low rates, whatever
  its gains. The cause is not loop noise: the channel estimate's noise on the four pilots is a
  constant slope for the whole frame (0.11 samples RMS at 3.5 dB), the loop converged onto it,
  and applied it to all 48 data subcarriers. The tracker now fits offset + rate·t and removes
  only the drift. A frozen loop cost nothing, which separated the cause.
- *A measurement trusted when it was worthless.* The second review built a channel with a
  null at one pilot of each pair: the tracker lost 458 of 600 frames (6 Mb/s, 6 dB) that a
  receiver without it decoded, and in TGn D at 6 Mb/s it cost 12 % of frames. The pair
  weights were the pilots' own (noisy) magnitudes and cancelled out of the fit, so every
  measurement entered at the same variance. They are now the channel gains, and they set
  the variance.
- *Panics.* The reviewer found overflow and index panics on arguments no caller should
  pass (a start of `usize::MAX`, a symbol with 7 points, a negative `soft_max`); fixed and
  gated.

## 2026-11-19 · measurements · the whole float receiver through the channel simulator

`rxper` (`crates/radio-tools/src/bin/rxper.rs`): float front (acquisition, CFO correction), then
this receiver. Each frame has a clock offset between the two ends, uniform in ±40 ppm, which
sets its CFO (±232 kHz at 5.8 GHz) and its sample-clock offset; the timing tracker starts from
the front's CFO. 1000-octet PSDUs (the frame of the standard's sensitivity requirement,
§17.3.10.2). SNR over the whole sampled band. Report: `reports/rx_ofdm_per.txt`.
These are measurements of the model, not gates: nothing outside the model confirms a PER curve
yet (R1's captures and an outside receiver on the same samples will).

**SNR at 10 % PER, dB** (300 frames per point in AWGN, 200 in TGn; a new static channel per
frame; crossings interpolated on 1 dB and 3 dB grids, so ±0.2 dB at best)

| Rate | AWGN, 4-bit soft | AWGN, fine soft | TGn B, 4-bit | TGn B, fine | TGn D, 4-bit | TGn D, fine |
|---|---|---|---|---|---|---|
| 6 Mb/s | 3.5 | 3.4 | 10.2 | 9.7 | 7.9 | 7.9 |
| 9 Mb/s | 4.4 | 4.3 | 15.7 | 14.2 | 14.7 | 11.7 |
| 12 Mb/s | 6.8 | 6.7 | 13.3 | 13.2 | 12.1 | 11.9 |
| 18 Mb/s | 8.7 | 8.6 | 18.6 | 17.8 | 18.0 | 15.6 |
| 24 Mb/s | 11.6 | 11.3 | 19.3 | 19.2 | 17.0 | 16.9 |
| 36 Mb/s | 14.6 | 14.5 | 23.2 | 22.2 | 22.9 | 21.1 |
| 48 Mb/s | 18.9 | 18.5 | 25.9 | 25.8 | 24.7 | 24.2 |
| 54 Mb/s | 19.9 | 19.7 | 28.3 | 28.0 | 28.0 | 26.5 |

- In AWGN the Viterbi decoder's 4-bit soft values (clip 1.5) cost 0.1 to 0.4 dB.
- In fading they cost 0.8 to 3.0 dB at 9, 18 and 36 Mb/s in both channels and 1.5 dB at
  54 Mb/s in TGn D (0.3 dB in TGn B); 0.5 dB or less at the rate-1/2 and rate-2/3 modes. The
  weights |H|² spread the soft values over more than four bits hold: weak subcarriers round
  to 0 (erasures) and strong ones clip, and a rate-3/4 code has no redundancy to spare. This
  is open question 1 of `viterbi.md`, answered: the format has to be chosen again on the whole
  receiver in fading.

**Soft-value formats** (`rxper soft`, `rxper soft2`; `reports/rx_ofdm_soft.txt`,
`rx_ofdm_soft2.txt`; 200 and 300 frames per point). Loss against fine soft values (8 bits,
clip 6) at 10 % PER: the worst of the eight rates in each channel, and where a format is in
both runs, the worse of the two.

| Format (bits / clip) | Step | AWGN | TGn B | TGn D |
|---|---|---|---|---|
| 4 / 0.75 | 0.11 | 0.97 dB | 0.43 dB | 0.78 dB |
| 4 / 1 | 0.14 | 0.77 | 0.62 | 1.23 |
| 4 / 1.5 (today) | 0.21 | 0.39 | 1.40 | 3.03 |
| 4 / 3 | 0.43 | 0.52 | no crossing | no crossing |
| 5 / 1 | 0.067 | 0.72 | 0.20 | 0.30 |
| 5 / 1.5 | 0.10 | 0.35 | 0.36 | 1.03 |
| 5 / 2 | 0.13 | 0.15 | 0.62 | 0.88 |
| 5 / 3 | 0.20 | 0.16 | 0.96 | 2.53 |
| 6 / 1.5 | 0.048 | 0.37 | 0.03 | 0.26 |
| 6 / 2 | 0.065 | 0.13 | 0.23 | 0.22 |
| 6 / 3 | 0.097 | 0.14 | 0.28 | 1.04 |
| 6 / 6 | 0.19 | 0.16 | 0.83 | 2.50 |

- Fading needs a small step: the weak subcarriers' values must not round to 0. AWGN needs
  the clip at 1.5 or more for 16- and 64-QAM. Four bits cannot have both, five nearly; six
  bits with a clip of 2 is the first format within about 0.25 dB everywhere (clip 1.5:
  0.4 dB, in AWGN).
- These figures rank the formats and no more. Single cells differ by up to 0.3 dB between
  the two runs, and "losses" as low as −0.2 dB occur: that is the noise of 200 to 300
  frames per point with crossings interpolated on 1 and 3 dB grids. The G2 decision needs
  the paired method of `g2 viterbi` on this receiver.
- Six-bit soft values widen the decoder's path metrics from 9 to 11 bits (2·24·31 < 2¹¹).
- Not tried: a format with less dynamic range to cover (weights limited or compressed) at
  four or five bits.

**Sample-clock offset** (40 frames per cell, high SNR): with the tracker, started from the CFO
or from zero, no frame is lost at any length, rate or offset up to ±40 ppm. Without it, every
1000-octet frame at ±40 ppm is lost at 6 and 54 Mb/s and half of them at 24 Mb/s, and 37 to 40
of 40 of the 4095-octet frames from ±10 ppm.

**What the tracker costs and saves** (1000 frames per cell near 10 % PER; frames lost; within
each half of a row the receivers saw the same frames)

| Operating point | ±40 ppm: from the CFO | from zero | no tracker | 0 ppm: from the CFO | from zero | no tracker |
|---|---|---|---|---|---|---|
| AWGN, 6 Mb/s, 3.5 dB | 116 | 149 | 716 | 110 | 128 | 108 |
| AWGN, 24 Mb/s, 11.5 dB | 127 | 143 | 512 | 109 | 116 | 111 |
| AWGN, 54 Mb/s, 20 dB | 104 | 260 | 544 | 105 | 108 | 104 |
| AWGN, 54 Mb/s, 35 dB | 0 | 0 | 153 | 0 | 0 | 0 |
| TGn D, 6 Mb/s, 8.5 dB | 108 | 221 | 583 | 111 | 117 | 110 |
| TGn D, 12 Mb/s, 12.5 dB | 78 | 159 | 414 | 71 | 74 | 72 |
| TGn D, 48 Mb/s, 24.5 dB | 120 | 212 | 401 | 112 | 115 | 112 |
| TGn D, 54 Mb/s, 28.5 dB | 71 | 208 | 447 | 77 | 76 | 77 |

- Started from the CFO, the tracker costs nothing at 0 ppm (−2 to +2 frames in 1000 against
  no tracker), and with a clock offset it loses what the same receiver loses at 0 ppm (−6 to
  +18 frames; different frames, so within their noise).
- Started from zero with a clock offset it loses 16 to 156 frames in 1000 more than from the
  CFO: it has to learn the rate from the frame itself, and it is cautious about its
  measurements. The start from the CFO is the designed mode; the start from zero is what is
  left when the carrier is unknown, and it still saves most of what no tracker loses.
- The proportional-integral loop this replaced lost 153 where no tracker lost 108 (AWGN,
  6 Mb/s, 0 ppm, 1000 paired frames) and 142 against 110 in TGn D. Those figures are from the
  run before the change and are in no committed report; the scratch control that separated
  the cause (97 lost without a loop, 97 with the loop frozen, 145 with it running, 1000
  frames) is not committed either.
- The tracker before the second review's fix (measurements at equal variance) lost 129
  against 110 in TGn D at 6 Mb/s and 0 ppm; with the variance from the channel gains, 111.

**Open questions**
1. Answered 2026-11-19 (entry "decisions" below): six bits with a clip of 2 is the working
   format; a full G2 sweep confirms or corrects it.
2. The tracker's constants (the measurement's assumed deviation, the ±2 ppm of the CFO's
   clock offset, the rejection at three times the nominal variance) were set by reasoning and
   one test, not swept. Two known weaknesses:
   - a transmitter that does not derive carrier and clock from one oscillator breaks the
     start from the CFO, and the tracker then needs hundreds of symbols to outweigh its prior;
   - started from zero in a channel with a null at one pilot of each pair, the tracker still
     loses frames a receiver without it decodes (`a_weak_pilot_makes_its_pair_worthless`: a
     few in 120): the estimate's own noise makes a null's gain look larger than it is, so the
     variance computed for the measurement is too small. A noise estimate would fix it; the
     front has none yet. Started from the CFO it loses none there.
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

## 2026-11-19 · decisions · the founder on the soft-value format

- **Six bits with a clip of 2 is the working soft-value format** for the fixed-point receiver
  (R0-09 G2), replacing four bits with a clip of 1.5. Reason: the measurements above (0.8 to
  3.0 dB lost at 9, 18 and 36 Mb/s in TGn B and D with four bits).
- It is a working format, not a G2 result: the evidence is a ranking on 200 to 300 frames per
  point. R0-09 G2 runs the full sweep (thousands of frames, AWGN and TGn B, D, E, the eight
  rates, 4, 5 and 6 bits). If five bits hold the budget the question returns to the founder
  with those figures.
- Cost accepted: the decoder's path metrics widen from 9 to 11 bits. No decoder RTL exists
  yet, so nothing is rewritten.
- `SOFT_BITS`, `SOFT_CLIP` and `METRIC_BITS` in the model change with R0-09 G2, behind their
  own gates (the Viterbi G2 of `docs/gates/viterbi.md` is rerun at the new widths), not with
  this note.

## 2026-11-19 · G0 · R0-09 G2, stage A: the receiver's data path in integers

Model: `crates/radio-phy/src/rx_ofdm_fixed.rs` (`FixedOfdmConfig`). Stage A is the data path
from the FFT to the soft values; the timing tracker's arithmetic (a recursive least-squares
fit with a division per symbol) stays in float behind an integer interface and is stage B.
**Budget:** the whole fixed receiver within 0.2 dB of the float one at 10 % PER, every rate,
AWGN and TGn B, D, E, on the same ADC samples (`docs/testing.md`, G2).

The float receiver divides every data point by |H|² and multiplies the metric by the weight
|H|²/mean. The two cancel; the integer path never divides per subcarrier:

| Step | Arithmetic | Width (parameter) |
|---|---|---|
| Input | the front's CFO-corrected samples, `in_bits`, shifted left by `in_shift` into the FFT word | Z = 12, shift 2 |
| FFT, 64 points | radix 2, decimation in time, twiddles e^(−j2πk/64) in Q(T, T−2) (the transmit ROM conjugated), every butterfly output narrowed to W bits half to even with saturation; three of the six stages halve (`fft_halve`), so a bin is the float FFT's over 8 | W, T |
| Channel | Hs(k) = L(k)·(T1(k) + T2(k)) = 2·H(k), exact, W + 1 bits | — |
| Products | u(k) = Z(k)·Hs*(k), g(k) = \|Hs(k)\|², both shifted right by one per-packet shift chosen so that the largest g of the 52 fits U − 1 bits, half to even | U |
| Pilots | p = ±u at −21, −7, 7, 21 (base × polarity) | U |
| Tracker (stage B: float) | takes the four p and their g; gives the slope θ as a P-bit phase per subcarrier and the window's move | P of the front's NCO |
| Common phase | φ = angle of Σ p·e^(−j·k·θ) by the front's CORDIC (A bits, widened to P) | the front's N, W_c, A |
| Rotation | v(k) = u(k)·e^(−j(φ + k·θ)) with the front's rotator (table of 2^L entries, Q-bit amplitude), narrowed back to U bits | the front's L, Q |
| Demapper | per axis on v: first bit −v; then \|v\| − t₄, \|\|v\| − t₄\| − t₂ with the thresholds t_c = κ_c·g(k), κ_c = c/(2·√E) (E the constellation's mean energy: 1/√10 for 16-QAM; 4/(2√42) and half of it for 64-QAM), κ in C fractional bits | C |
| Scale | once per packet and modulation: n = bit length of Σg, q = ⌊A·2ⁿ / Σg⌋ with A = round(2·√E·52·soft_max/clip·2^F); soft = sign·⌊(\|m\|·q + half) / 2^(n+F)⌋, saturated to ±soft_max (half away from zero, the float contract) | F |
| Deinterleave, depuncture, decode | as the float receiver (R0-10) | soft format |

Why it is the same receiver: with Hs = 2H the float point is 2√E·u/g and its weight g/ḡ,
ḡ = Σg/52, so metric × weight × soft_max/clip = (2√E·52·soft_max/clip)·m/Σg with m the
demapper's value on v and the thresholds scaled by g. One division per packet and
modulation (two per frame: SIGNAL's BPSK and the data's) replaces 48 per symbol.

**Gates:** `tests/g2_rx_ofdm.rs`. (1) The integer FFT against the float FFT of the same
samples, and exact on inputs whose transform is known by hand. (2) With wide words the soft
values equal the float receiver's. (3) With the default words every rate decodes, through a
notched channel too, and the standard's printed waveform decodes where the local fixture
is present. (4) The loss at 10 % PER and the sweep that chose each width:
`rxper fixed`, a report.

**Not in stage A:** the tracker in integers; channel-estimate smoothing; the DC removal in
front of the decoder (`rx_front.md`, 2026-11-19: its first numbers); the Viterbi decoder at
the new soft format (R0-10's G2 is rerun when `softg2` has confirmed it).

## 2026-11-19 · G2 · stage A: the integer data path against the float receiver

Commits 3965195 (model, gate), f37f56a and the one carrying this entry (measurements).
Commands: `cargo test -p radio-phy --test g2_rx_ofdm` (14 pass);
`cargo test -p radio-phy --test g1_annex_i1 -- --ignored` (the standard's printed waveform,
local fixture: decodes through `decode_ppdu_fixed` to the annex's message);
`rxper fixedw 2000` → `reports/rx_ofdm_fixedw.txt`; `rxper fixed 2000` →
`reports/rx_ofdm_fixed.txt`.

**Outside references.** The FFT is checked on inputs whose transform is known by hand (a
constant, an impulse, the alternating sequence, (−j)ⁿ → bin 48, ties, saturation) and against
the float transform; the receiver against the float receiver (itself G1 against the
standard's Annex I waveform) and, where the fixture is present, against that waveform
directly.

**The gate bites.** 31 mutations of `rx_ofdm_fixed.rs`, one at a time, each restored with
`cp` and `touch`; every one fails at least one test of `g2_rx_ofdm`. They are: twiddle sign; no stage halved; ties
rounded up; ties rounded down; FFT word not saturated; L(k) ignored; T2 not used; u not
conjugated; the packet shift one short; no packet shift; pilot polarity ignored; pilot base
ignored; slope sign on the pilots; common phase not removed; slope not removed from the
data; angle not widened to the phase word; threshold twice too large; centre threshold not
halved; fold without magnitude; first-bit sign; scale over 48 instead of 52; soft value
truncated; soft value not saturated; not deinterleaved; window moves the wrong way; window
never moves; input shift ignored; rotation not narrowed back; Q of BPSK demapped; the gain
of bin 0 used for every threshold; window moves one way only. One survived at first — "the
window never moves" — because no test ran long enough for the window to walk; the test
`the_window_walks_over_a_drift_longer_than_the_guard_interval` was added and kills it, and
"the window moves one way only" after it.

*After the review (2026-11-20).* Those 31 were faults of the architecture; the reviewer's
own 33 were faults of the bits — a rounding, a saturation, a default width — and 20 of them
passed, because the gate compared with float at wide words and only decoded at the default
ones. Added: `the_default_words_bit_for_bit` (a pin: hashes of the soft values and phases
of five frames, two soft formats, one frame with its data four times louder than its
training field), `the_constants_worked_by_hand` (κ and A against values worked outside the
model), `wide_words_with_a_slope_on_unequal_pilots_and_with_the_loops_off`,
`samples_outside_the_input_width_saturate`, `the_common_phase_log_is_the_float_receivers`,
`the_window_stays_when_the_timing_loop_is_off`, `a_channel_with_an_impossible_shift_is_refused`
(14 tests). Of the reviewer's survivors rerun, with the four default widths and two guards
(27 mutations), 24 now fail a test. Three do not, and are recorded as not pinned:
"gain not saturated" (the packet shift leaves the largest gain inside the word; only a
round-up to exactly 2^(U−1) could reach the saturation); "rotator output not saturated" (a
value that large saturates the soft value either way, and the model's mutant does not wrap
as hardware without the saturation would); "soft value half up on the signed value" (differs
from half away from zero only on an exact tie, one in 2^(n+F) values). The RTL's G3 on whole
vectors is where these three get pinned or shown unreachable.

**Constants for the ROM** (C = 10, F = 4; `threshold_constant`, `scale_constant`):
κ: 16-QAM c = 2 → 324; 64-QAM c = 4 → 316, c = 2 → 158. A at six bits, clip 2: BPSK 25792,
QPSK 36475, 16-QAM 81561, 64-QAM 167151 (at four bits, clip 1.5: 7765, 10982, 24556, 50325).

**Open, found by the review, for stage B:**

- The rotator's table is addressed by the phase's top L bits, truncated (the front's NCO,
  whose RTL and vectors exist, so it is not changed here). In the data path that is a mean
  turn of −π/512 on the data and, once θ is not zero, a common phase read π/512 = 0.006 rad
  low (the phase-log test shows exactly this step and allows for it). Half an address added
  to the phase before the table would remove both; whether it is the few frames that
  L11 Q14 recovers is to be measured when stage B reruns the reports.
- The slope's conversion to θ and the tracker call libm (`cos`, `sin`, `atan2`): a
  last-bit difference between platforms could move θ by one unit, which would matter for
  vectors regenerated in CI. Stage B puts the tracker in integers.
- A `FixedChannel` built by hand with a shift no estimate can produce panicked in a debug
  build (a shift wider than the word); refused now, with a test.
- A clip that is not a positive number gives a scale of 0, every soft value an erasure
  (the float receiver saturates instead); documented on `scale_constant`.

**Measurement set-up.** The same frames for every receiver: the float front acquires and
corrects, the corrected stream is scaled to −12 dBFS RMS and quantised to 12 bits
(full scale 2048), and *every* receiver, the float one too, decodes those samples; soft
values six bits with a clip of 2 (the working format). 1000-octet frames, 2000 per point,
clock offset uniform in ±40 ppm.

**Loss at 10 % PER, integer data path − float** (`reports/rx_ofdm_fixed.txt`; default widths
W16 s2 T12 U16 C10 F4, the front's rotator L9 Q11 and CORDIC N16; dB, + is a loss):

| Rate, Mb/s | AWGN | TGn B | TGn D | TGn E |
|---|---|---|---|---|
| 6 | −0.00 | +0.00 | −0.01 | −0.00 |
| 9 | −0.00 | +0.00 | +0.01 | −0.00 |
| 12 | −0.00 | +0.00 | −0.01 | −0.01 |
| 18 | −0.01 | +0.04 | +0.00 | +0.01 |
| 24 | −0.01 | +0.00 | +0.01 | −0.00 |
| 36 | +0.02 | +0.10 | +0.03 | +0.01 |
| 48 | +0.03 | +0.05 | +0.01 | +0.02 |
| 54 | +0.01 | +0.02 | +0.01 | +0.00 |

The worst cell is +0.10 dB (TGn B, 36 Mb/s); the budget is 0.2 dB. **Stage A's data path
adds no loss that 2000 frames per point resolve.** How far that goes:

- A single cell is not resolved below about 0.1 dB in fading. At the worst cell the PERs at
  23.5 dB are 0.099 and 0.100 — two frames — on a curve of about 30 frames per dB there, so
  the whole budget is about six frames; the report prints PER to three decimals and no
  counts of the frames on which the receivers disagree, so a paired test per cell cannot be
  made from it. The sum can: frames lost, integer minus float, over all the grid points of a
  channel: AWGN +72 of 112 000, TGn B +46 of 112 000, TGn D +42 of 112 000, TGn E +2 of
  144 000. A mean cost of a few hundredths of a dB at most.
- **This is the data path's share, not G2 passed.** The float receiver it is compared with
  already has the six-bit soft values with a clip of 2; G0's budget is for the whole fixed
  receiver against float with fine soft values, and the format's own loss (next entry: up
  to +0.32 dB) adds to this one.
- One input level, −12 dBFS, set from each frame's own power. The level test of the gate
  (−3 to −40 dBFS) has no noise. The reviewer's paired check (AWGN, 54 Mb/s, 19 dB, 600 to
  2000 frames per level, not a committed report) found no difference from −5 to −42 dBFS
  and the integer path losing on its own below: 1461 against 1514 frames decoded of 2000 at
  −45 dBFS, 1147 against 1233 at −48. The lower edge is to be measured with the gain
  control in front (stage B, R0-07e's open item).
- Not covered: the tracker's arithmetic and the slope's conversion to θ (float here, stage
  B); the front in integers on the same frames (the front here is the float one; its own
  G2 is in `rx_front.md`).

**Widths** (`reports/rx_ofdm_fixedw.txt`: frames lost of the same 2000 at seven scenes, one
width changed per column).

- With every width far beyond need the integer path loses what the float receiver loses
  (186/186, 188/189, 233/233, 195/195, 141/141 frames; 0 at 35 and 45 dB). The
  architecture — no division per subcarrier, thresholds scaled by g, one division per
  packet — costs nothing measurable.
- The default's few extra frames at 54 Mb/s (196 against 189 in AWGN, 203 against 195 in
  TGn D, 144 against 141 in TGn E) go with the rotator table: with L11 Q14 the counts are
  193, 195, 141. In dB at 10 % PER the wider table changes nothing that the grid resolves
  (third column of `rx_ofdm_fixed.txt`: within 0.03 dB of the default except TGn B 48 Mb/s,
  −0.03 against +0.05; TGn B 36 Mb/s stays +0.10). **The shared table L9 Q11 stays**; a
  second table for the data path is not bought by these numbers.
- Too narrow: products of 10 bits (U10: 270 against 203 in TGn D, 255 against 144 in TGn E,
  34 frames lost at 45 dB); an FFT word with less than two bits above the input after the
  shift (W14 s2 and W16 s4: 32–33 frames lost at 45 dB in TGn E — saturation on a strong
  signal; the default leaves two); C6 (216 against 196 in AWGN); L7 (229).
- No difference found, so candidates for narrowing when area is counted (ADR 0008): W15,
  T10, U14, F0, N10–N12. Not narrowed now: each is within the noise of 2000 frames at these
  scenes, and that is not evidence that it is free everywhere.
- Not candidates, though close: U12 (one frame lost at 45 dB in TGn E, where every
  adequate width loses none and U10 loses 34; 149 against 144 at 26.5 dB) and C8 (202
  against 196 in AWGN at 54 Mb/s, on the trend C6 216, C8 202, C10 196, C14 196).
- "CORDIC wide alone" is not a valid set of widths (the angle word must stay six bits
  below the phase word) and has no column.

## 2026-11-20 · G2 · the soft-value format: the sweep

Commands: `rxper softg2 2000` → `reports/rx_ofdm_softg2.txt` (float receiver, a fine
reference of soft_max 255 clip 6, eight candidates on the same frames; grids of 0.5 dB in
AWGN and 1 dB in TGn B, D, E). Worst loss at 10 % PER against the fine reference, dB (rate
in Mb/s where it happens):

| Format (bits / clip) | AWGN | TGn B | TGn D | TGn E |
|---|---|---|---|---|
| 4 / 1 | +0.68 (48) | +0.99 (36) | +0.90 (18) | +0.98 (18) |
| 4 / 1.5 (R0-10's contract) | +0.31 (48) | +1.96 (9) | +2.74 (18) | +2.76 (18) |
| 5 / 1 | +0.63 (48) | +0.29 (36) | +0.21 (18) | +0.21 (36) |
| 5 / 1.5 | +0.28 (48) | +0.65 (36) | +0.39 (18) | +0.46 (36) |
| 5 / 2 | +0.10 (24) | +0.89 (36) | +0.70 (18) | +0.84 (18) |
| 6 / 1.5 | +0.24 (48) | +0.21 (36) | +0.06 (18) | +0.13 (36) |
| 6 / 2 (working format) | +0.10 (24) | +0.32 (36) | +0.15 (18) | +0.21 (36) |
| 6 / 3 | +0.02 (48) | +0.53 (36) | +0.32 (18) | +0.44 (36) |

Findings:

- **No format of six bits or fewer holds 0.2 dB everywhere.** The working format 6 / 2
  misses in TGn B (36 Mb/s +0.32, 9 Mb/s +0.23) and in TGn E (+0.21); 6 / 1.5 misses in
  AWGN, on a cell the count resolves (48 Mb/s at 18.5 dB: PER 0.134 against 0.091, some
  86 frames), and in TGn B (+0.21). Single fading cells within a few hundredths of the
  limit are not resolved (see the entry above); the conclusion does not rest on them.
- Two different things cost. In AWGN it is the clip: 5 / 2 and 6 / 2 lose the same
  (+0.10 at 24 Mb/s, +0.09 at 48), 6 / 3 nothing; the rates hit are 24 and 48 Mb/s. In
  fading it is the step, clip over levels: at one width the smaller clip is the better
  (5 / 1 beats 5 / 2, 6 / 1.5 beats 6 / 3), and every rate hit has the 3/4 code (9, 18, 36,
  54 Mb/s); in fading 6, 12, 24, 48 Mb/s lose under 0.1 dB with any six-bit format.
- A reading, not measured: a weak subcarrier's soft values, already scaled down by its
  gain, round to zero, and a zero is what a punctured bit is; the 3/4 code has the least
  distance to spare. The quantiser without a zero level (`softg4`) tests it.
- Five bits are out: the question "do five bits hold the budget" has the answer no.

## 2026-11-20 · G2 · the soft-value format: seven and eight bits, and a quantiser without zero

Commands: `rxper softg3 2000` → `reports/rx_ofdm_softg3.txt` (the same frames and grids as
`softg2`; 6 / 2 repeated as the paired reference — its column is `softg2`'s, cell for cell);
`rxper softg4 2000` → `reports/rx_ofdm_softg4.txt` (AWGN, TGn B, TGn E).

Worst loss at 10 % PER against the fine reference, dB (rate), and — the steadier figure —
the frames lost beyond the fine reference's, summed over all grid points of the rates with
the 3/4 code in fading (TGn B / D / E, of 56 000 / 56 000 / 72 000) and of the other rates
in AWGN (of 56 000):

| Format | AWGN | TGn B | TGn D | TGn E | extra frames, 3/4 code, B / D / E | extra frames, AWGN, other rates |
|---|---|---|---|---|---|---|
| 6 / 2 | +0.10 (24) | +0.32 (36) | +0.15 (18) | +0.21 (36) | +184 / +266 / +376 | +310 |
| 7 / 2 | +0.09 (24) | +0.02 (12) | +0.04 (36) | +0.07 (9) | +10 / +40 / +86 | +288 |
| 7 / 3 | +0.00 (24) | +0.28 (36) | +0.06 (18) | +0.16 (36) | +76 / +134 / +184 | +8 |
| 7 / 4 | +0.02 (48) | +0.35 (36) | +0.17 (18) | +0.20 (36) | +194 / +258 / +352 | +18 |
| 8 / 3 | +0.01 (48) | +0.01 (54) | −0.00 (48) | +0.02 (36) | +0 / −2 / +2 | +16 |
| 8 / 4 | +0.01 (9) | +0.02 (24) | +0.04 (36) | +0.07 (36) | +14 / +34 / +86 | +2 |

Findings:

- **The two costs are separate and each has one cause.** The step (clip over levels)
  sets the loss of the 3/4-code rates in fading: 7 / 2 and 8 / 4 have one step and one
  loss (+10 / +40 / +86 and +14 / +34 / +86 frames), as have 6 / 2 and 7 / 4. The clip sets
  the loss in AWGN at 24 and 48 Mb/s: about +300 frames with a clip of 2 at six or seven
  bits, under +20 with 3 or 4.
- **7 / 2 holds 0.2 dB everywhere** (worst +0.09 dB, in AWGN, the clip's). **8 / 3 loses
  nothing that the measurement resolves** (worst +0.02 dB). 7 / 3 and 7 / 4 miss in TGn B.
- The quantiser without a zero level (`softg4`, the `m` columns) is not the remedy the
  earlier entry guessed at: 6m / 2 loses +126 frames at the 3/4 rates in TGn B where 6 / 2
  loses +184 — about what half a step finer buys, no more — and the same as 6 / 2 in AWGN.
  A weak bit rounding to zero is not the mechanism; the step is. (`mid_rise` stays in the
  float receiver's config as the experiment's record and is off by default; the integer
  path does not have it.)
- What the format costs downstream is for ADR 0008's count: a bit more per soft value in
  the deinterleaver's 288 words and in the Viterbi's branch and path metrics per bit added.
- Open: the LDPC decoder's input was fixed at six bits with a clip of 2 by its own G2
  (R0-12), in AWGN. Whether fading asks the same of it as of the Viterbi path here has not
  been measured.

**Decision (the founder, 2026-11-20): eight bits, clip 3** — soft values of ±127, 3 half
minimum distances on a subcarrier of mean gain mapping to 127. It replaces the working
format of 2026-11-19 (six bits, clip 2). Reason: it is the one candidate that leaves the
0.2 dB budget to the data path, the tracker and the front; 7 / 2 holds the budget alone but
spends half of it.

Follows, each behind its own gate and not in this change: `SOFT_BITS` 4 → 8, `SOFT_CLIP`
1.5 → 3 and `METRIC_BITS` (2^M > 2·24·127, so 13) in `viterbi.rs` with R0-10's G1 and G2
rerun; `rxper fixed` and `fixedw` rerun at the new format (they were measured at 6 / 2), the
pin of `g2_rx_ofdm` and the constants A regenerated; the LDPC input's question above.

## 2026-11-20 · G2 · the data path at eight bits, clip 3

Commit: the one carrying this entry, on ec240fb (`SOFT_BITS` 8, `SOFT_CLIP` 3,
`METRIC_BITS` 13 in `viterbi.rs`; the decoder's own G1 and G2 at the format are in
`docs/gates/viterbi.md`). Commands: `cargo test -p radio-phy --test g2_rx_ofdm` (14 pass;
the bit-for-bit pin and the constants A regenerated for the format: 70443, 99621, 222759,
456521); `rxper fixed 2000` → `reports/rx_ofdm_fixed.txt`; `rxper fixedw 2000` →
`reports/rx_ofdm_fixedw.txt`. Both reports replace the ones measured at 6 / 2.

`rxper fixed` now runs three receivers on the same frames: the float receiver with fine
soft values (the reference the budget is counted from), the float receiver at 8 / 3, and
the integer data path at 8 / 3. 2000 frames of 1000 octets per point, clock offset uniform
in ±40 ppm, the front in float.

SNR at 10 % PER against the fine float receiver, worst rate of each channel (Mb/s), and
frames over all points of the channel:

| Channel | float 8 / 3, worst | integer 8 / 3, worst | frames lost, fine | float 8 / 3 − fine | integer − float 8 / 3 |
|---|---|---|---|---|---|
| AWGN | +0.01 dB (36) | +0.02 dB (48) | 14958 | +6 | +36 |
| TGn B | +0.02 dB (24) | +0.04 dB (48) | 10868 | +2 | +32 |
| TGn D | +0.01 dB (36) | +0.03 dB (36, 48, 54) | 10492 | +6 | +18 |
| TGn E | +0.02 dB (9) | +0.03 dB (36) | 11562 | +0 | +22 |

Findings:

- **The format and the integer data path together cost at most +0.04 dB** in one cell
  (TGn B, 48 Mb/s) against a budget of 0.2 dB for the whole fixed receiver. The interpolated
  crossings scatter by a few hundredths either way (the most negative cell is −0.03 dB), so
  the single cells are at the resolution of the measurement; the frame sums are the
  steadier figure: the format adds 0 to 6 frames per channel, the integer arithmetic 18 to
  36 of some 11 000 to 15 000 lost.
- At 6 / 2 the same comparison gave +0.10 dB in the worst cell and +72 / +46 / +42 / +2
  frames (entry of 2026-11-19), with the format's own loss of up to 0.32 dB on top and not
  counted there. The integer path's remaining cost is the same order as before; the
  format's has gone.
- **The widths stand** (`fixedw`, frames lost of 2000, same frames in every column). The
  default (W16 s2 T12 U16 C10 F4, L9 Q11, N16) against float: 190 / 190, 188 / 184, 215 /
  215, 194 / 188, 130 / 130 at the five points with errors, and no frame lost at the two
  high-SNR points. The columns that failed at 6 / 2 fail the same way: W14 s2 and W16 s4
  leave a floor (33 frames at 45 dB in TGn E), U10 loses 254 against 130, C6 205 against
  188, L7 218 against 188 and 152 against 130. C8, which cost 6 frames at 6 / 2 in AWGN at
  54 Mb/s, shows none here (188 / 188); one point is not ground to narrow κ on, and C10
  stays.
- Where the default differs from float it is with the rotator: "data wide" equals the
  default to within 1 frame at every point, while "front wide" and L11 Q14 come back to
  float's count or near it (AWGN 54 Mb/s: 184 and 185 against 188; TGn D 54 Mb/s: 190 and
  190 against 194). That is 3 to 4 frames of 2000 at single points — the same finding as
  at 6 / 2, and not enough to widen the table (L9 Q11 stays); the rotator's phase rounding
  is on stage B's list.

Not measured here, unchanged from stage A's list: the timing tracker in integers, the
rotator's phase rounding, PER through the gain-control model and the lower edge of the
input level, DC removal before the decoder (stage B); the LDPC input's format in fading.

## 2026-11-20 · G0 · R0-09 G2, stage B: the timing tracker in integers

Model: `rx_ofdm_fixed.rs`, `FixedTracking`, `tracker_constants`. It replaces the float
tracker that stage A left behind an integer interface; between the samples and the soft
values there is no floating point now. The reference is the float tracker (`rx_ofdm.rs`,
`Tracking`: a recursive least-squares fit of measured(t) = offset + rate·t, of which only
rate·t is removed from the data).

**Units.** Phases in units of 2π/2^P per subcarrier (P the rotator's 22: one sample of
timing is 2^(P−6) = 65 536). Time τ in fifths of a symbol from the middle of the long
training field: 7 for SIGNAL (112 samples), then 5 per symbol, an integer. Rates per unit
of τ with R more fractional bits.

**Per packet** (after the channel estimate; three quotients on one denominator). With g the
gains at −21, −7, 7, 21, a = g₀ + g₃, b = g₁ + g₂, the float tracker's pair weights g₀g₃/a
and g₁g₂/b are cleared of their divisions:

| Quantity | Arithmetic | Width |
|---|---|---|
| N_o, N_i, D | g₀·g₃·b, g₁·g₂·a, 9·N_o + N_i | 46, 46, 50 bits for U = 16 |
| normalisation | D cut to M bits by a right shift (leading-zero count), the same shift on N_o, N_i and Σg·a·b, truncating | M |
| pair coefficients | c_o = 3·N_o/(14·D), c_i = N_i/(14·D), K fractional bits, rounded half up (at most 1/42 and 1/14) | K |
| variance | v = 5·Σg·a·b/(52·D), V fractional bits, truncated: the measurement's variance against a flat channel of the same mean gain | V |
| prior's weight | λ = round(w·v), w a ROM constant: 25·(σ/drift per symbol of the prior's deviation)², σ = 0.016 rad: 64 846 for the 40 ppm prior, 25 938 223 for the front's CFO hint (2 ppm) | 35 bits |

**Per symbol** i (τ = 7 + 5i):

| Step | Arithmetic |
|---|---|
| Prediction | drift = round((r₀ + δ)·τ / 2^R), half to even; θ_pred = drift + shift·2^(P−6) (what the window's moves took) |
| Measurement | the CORDIC's angle of p₃·p₀* and of p₂·p₁* (A bits, widened to P), less 42·θ_pred and 14·θ_pred modulo a turn; e = round((c_o·∠outer + c_i·∠inner)/2^K), half to even. Subtracting the predicted turn from the angle is the float tracker's rotation of the pilots before it measures, without the rotator |
| Fit | if 0 < v ≤ 3: z = e + round(δ·τ/2^R); n += 1, Στ += τ, Στ² += τ², Σz += z, Στz += τ·z; δ = 2^R·(n·Στz − Στ·Σz) / (n·(Στ² + λ) − (Στ)²), truncated toward zero |
| Applied | drift with the new δ; θ = drift + shift·2^(P−6) goes to the rotator; beyond half a sample (2^(P−7)) the window moves by one for the next symbol |

The quotient is the float recursion's rate exactly (the batch form of the same least
squares; the float tracker's prior on the offset, 10⁶ nominal variances, is taken as flat).
For the longest frame (1367 symbols) the sums need 11, 23, 35, 31 and 43 bits; the division
is 64 bits by 45, once per symbol (4 µs), so a serial divider. r₀ is the prior rate: 0, or
from the clock offset the front's CFO gives. Here that conversion is the one float at the
boundary (`FixedTracking::start`, ppm in); in hardware it is the CFO word times a constant
of the channel's centre frequency, and where that product is formed is ADR 0008's hand-off.

**Parameters** (`FixedOfdmConfig`): R = 10, M = 16, K = 16, V = 10 to start with; the sweep
(`rxper fixedw`) confirms or narrows them. `rot_round` adds half a table address to the
data path's rotation phases (the open item of stage A's review: the front's rotator
truncates).

**Gates:** `tests/g2_rx_ofdm.rs`. (1) The per-packet constants worked by hand. (2) The fit
worked by hand on three symbols, both signs, and the starts' constants. (3) The fit against
the float recursion written out in the test, 1367 symbols, two priors, three variances.
(4) In the receiver: each symbol's measured slope and applied drift against the float
receiver on the same samples, from 6 dB up, three starts. (5) A notch on a pilot: measured,
reported, not followed. (6) The bit-for-bit pin.

## 2026-11-20 · G2 · stage B: the integer tracker and the rotator's rounding

Commits 16283ae and the one carrying this entry. Commands:
`cargo test -p radio-phy --test g2_rx_ofdm` (19 pass);
`cargo test -p radio-phy --test g1_annex_i1 -- --ignored` (the standard's printed waveform
decodes through the all-integer path, local fixture);
`rxper fixedw 2000` → `reports/rx_ofdm_fixedw.txt`.

**Order of work.** The tracker's tests were written after its first version, not before
it (the rule is the other way round). What stands in for the failing run: the float
tracker is the reference and existed first, and each of the 41 mutations below was seen to
fail a test.

**Against the float tracker.** In the receiver, on the same 12-bit samples (6 Mb/s at
6 dB, 12 at 9 dB, 24 at 18 dB, 54 at 28 dB, a two-path channel, ±40 ppm and less, three
starts): over 2085 symbols the measured slope differs by at most 9.6·10⁻⁵ rad per
subcarrier and the drift applied by at most 0.00015 samples. The fit alone against the
float recursion written out in the test: within one unit of the quotient's last bit (2⁻¹⁰
of 2π/2²² per subcarrier per fifth of a symbol) over 1367 symbols, two priors, three
variances.

**A finding about both trackers.** In one of the 6 dB frames a pilot was all but cancelled
by noise and the outer pair's angle fell within 2·10⁻⁴ rad of ±π: the float receiver read
−π, the integer one +π, and the two measurements differ by a turn times the pair's
coefficient (0.12 rad per subcarrier, eight nominal deviations). Neither is right: a pair
angle near ±π is a measurement that has wrapped, and both trackers take it at face value.
The test ends that frame's comparison there and counts the event. Open: whether resolving
the outer pair's turn from the inner pair's angle (three times closer to zero), or
dropping a symbol whose two pairs disagree, is worth having at the sensitivity limit. It
would change the float receiver first.

**The gate bites.** 41 mutations of `rx_ofdm_fixed.rs`, one at a time, restored with `cp`
and `touch`; each fails at least one test. The per-packet constants: the outer
coefficient without its 3; D = N_o + N_i; N_o with a; N_i with b; coefficient truncated;
variance with 4 for 5; variance over 48; the gains of the wrong pilots; D not cut to M
bits. The fit: τ from 0; τ in steps of 4; z without the prediction; the prediction
truncated; the determinant without λ; the numerator's sign; the quotient not scaled; the
quotient floored; a fixed tracker that learns; λ without the variance; the weight without
its 25; the rate per symbol instead of per fifth. The step: τ from 0 in the drift; the
outer span 21; the inner span 7; the pairs swapped; the pair product not conjugated; the
prediction without the window's moves; θ without them; the angle not widened; the angle
saturated instead of wrapped; half a sample taken as a whole one; the window moving the
wrong way; the error one bit too large; the drift not refreshed after learning; the timing
log in half samples. Three survived the first run — the variance limit at 30, weak
measurements used, the error truncated — the first two because no test had a weak pilot
(`a_weak_pilot_pair_is_not_followed` added, kills both), the third is a last-bit rounding
that the bit-for-bit pin kills. The rotator: phase truncated; a whole address added; half
an address subtracted (the phase-log test, whose tolerance went from 8·10⁻³ to
1.5·10⁻³ rad, and the pin).

**Widths** (`fixedw`, frames lost of 2000, the same frames in every column; base = the
tracker in integers, the rotator's phase truncated):

| Point | float | base | round | round L8 | R6 | R8 | M8 | M12 | K8 | K12 | V4 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| AWGN 6 Mb/s 3.5 dB | 190 | 190 | 189 | 190 | 190 | 190 | 190 | 190 | 191 | 190 | 190 |
| AWGN 54 Mb/s 20 dB | 184 | 188 | 189 | 192 | 188 | 188 | 188 | 188 | 188 | 188 | 188 |
| TGn D 18 Mb/s 15.5 dB | 215 | 215 | 213 | 215 | 215 | 215 | 215 | 215 | 215 | 215 | 215 |
| TGn D 54 Mb/s 26.5 dB | 188 | 194 | 188 | 191 | 194 | 194 | 194 | 194 | 194 | 194 | 194 |
| TGn E 54 Mb/s 26.5 dB | 130 | 130 | 130 | 135 | 130 | 130 | 130 | 130 | 130 | 130 | 130 |

(No frame lost at the two high-SNR points in any of these columns.)

- **The integer tracker costs nothing that these points resolve:** the base column is the
  one measured with the float tracker inside (entry above: 190, 188, 215, 194, 130).
- **The rotator's phase is rounded now** (`rot_round`, default on: half a table address
  added before the table is addressed, one adder). Over the five points it gives back 7
  frames of the 9 the base loses against float; the common phase the receiver reports
  agrees with the float receiver's to 5·10⁻⁴ rad (was 6·10⁻³, the truncation's bias). With
  rounding, a table of 2⁸ entries is not enough (+9 frames over the five points against
  2⁹), so L stays 9. The front's own rotator (CFO correction, RTL and vectors exist) is
  not changed: a constant phase offset there is absorbed by the channel estimate.
- **The tracker's widths are not tight.** R 6, M 8, K 8, V 4 lose nothing here (K 8: one
  frame at one point). The defaults R 10, M 16, K 16, V 10 stay until ADR 0008 counts what
  each bit costs; narrowing them is free of risk by this table and to be confirmed on the
  full grids if it is taken.

Open: the report of `rxper fixed` at these defaults (below, when its run ends); the frames
of every `rxper` report depend on the number of threads (the seed is per thread), so a
report is reproduced only on a machine with as many cores as the one that made it — the
tool should draw each frame's seed from the frame's number.
