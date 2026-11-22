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
| normalisation | D cut to M bits by a right shift (leading-zero count), the same shift on N_o, N_i and Σg·a·b, truncating. A pair with both its pilots at zero gain has no weight; its sum (a or b) counts as 1 so that the other pair still measures | M |
| pair coefficients | c_o = 3·N_o/(14·D), c_i = N_i/(14·D), K fractional bits, rounded half up (at most 1/42 and 1/14) | K |
| variance | v = 5·Σg·a·b/(52·D), V fractional bits, truncated: the measurement's variance against a flat channel of the same mean gain | V |
| prior's weight | λ = round(w·v), w a ROM constant: 25·(σ/drift per symbol of the prior's deviation)², σ = 0.016 rad: 64 846 for the 40 ppm prior, 25 938 223 for the front's CFO hint (2 ppm); half to even | 27 bits (v ≤ 3) |

**Per symbol** i (τ = 7 + 5i):

| Step | Arithmetic |
|---|---|
| Prediction | drift = round((r₀ + δ)·τ / 2^R), half to even; θ_pred = drift + shift·2^(P−6) (what the window's moves took) |
| Measurement | the CORDIC's angle of p₃·p₀* and of p₂·p₁* (A bits, widened to P), less 42·θ_pred and 14·θ_pred modulo a turn; e = round((c_o·∠outer + c_i·∠inner)/2^K), half to even. Subtracting the predicted turn from the angle is the float tracker's rotation of the pilots before it measures, without the rotator |
| Fit | if 0 < v ≤ 3: z = e + round(δ·τ/2^R), saturated to P − 2 bits; n += 1, Στ += τ, Στ² += τ², Σz += z, Στz += τ·z; δ = 2^R·(n·Στz − Στ·Σz) / (n·(Στ² + λ) − (Στ)²), truncated toward zero, saturated to P + R − 4 bits |
| Applied | drift with the new δ; θ = drift + shift·2^(P−6) goes to the rotator; beyond half a sample (2^(P−7)) the window moves by one for the next symbol |

The quotient is the float recursion's rate exactly (the batch form of the same least
squares; the float tracker's prior on the offset, 10⁶ nominal variances, is taken as flat).

**Registers** (P = 22, R = 10). e is at most half a turn over 14: 19 bits signed. z is 20
bits signed and saturates: 8 samples of timing, which a prior on the clock offset wrong by
190 ppm reaches over the longest frame (the standard's worst case is 40 ppm). For the
longest frame (1367 symbols) n, Στ, Στ² need 11, 23 and 35 bits, and with z bounded Σz and
Στz cannot exceed 31 and 43 bits signed. δ is 28 bits signed, saturating. The division is
65 bits by 43, once per symbol (4 µs), so a serial divider. With a signal present nothing
saturates. When the signal ends inside a frame the pilots are noise, the measurement no
longer answers the prediction, and the rate runs away — in the float tracker too (the
review drove both: the float timing reached 635 samples); the integer one stops at z's
limit. What a receiver does with a frame whose signal has gone is not the tracker's
business, but the widths above hold only because z saturates.

r₀ is the prior rate: 0, or
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
and `touch`; each fails at least one test (the list is of the code before the review; the
review's additions follow it). The per-packet constants: the outer
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
  added before the table is addressed, one adder). Over the five points it gives back 8
  frames of the 10 the base loses against float; the common phase the receiver reports
  agrees with the float receiver's to 5·10⁻⁴ rad (was 6·10⁻³, the truncation's bias). With
  rounding, a table of 2⁸ entries is not enough (+14 frames over the five points against
  2⁹), so L stays 9. The front's own rotator (CFO correction, RTL and vectors exist) is
  not changed: a constant phase offset there is absorbed by the channel estimate.
- **The tracker's widths show slack at these points.** R 6, M 8, K 8, V 4 lose nothing here
  (K 8: one frame at one point). That is not yet a licence to narrow them: the frames are
  1000 octets (at most 335 symbols) and R's rounding grows with the frame's length, up to
  τ = 6837; and every column varies one width around the base with the truncating rotator,
  not around today's default. The defaults R 10, M 16, K 16, V 10 stay; ADR 0008 counts
  what each bit costs, and a narrower set is measured on the full grids and the longest
  frame before it is taken.

**After the review** (an independent reviewer on commit 37134f6, with the code and the
float reference, not this entry). It rederived the estimator from the float code and
found it the same in normal operation, recomputed every hand-worked number, and found
three faults, all fixed in commit 923fdb3:

- The register widths stated in G0 were not in the model: with the signal gone after 20
  symbols of a long frame z reached 27 bits and the numerator 67, where the model's
  128-bit integers went on and a register would have wrapped. z and δ saturate now, at
  the widths G0 states, and the other widths follow from them (G0's paragraph on the
  registers was rewritten; its "35 bits" for λ was wrong, it is 27, and the divisor is 43
  bits, not 45).
- `FixedTracking::learn` could be driven to a shift overflow with unusable widths
  (a `rate_frac` of 128); it refuses them now, and symbols beyond any frame's.
- A pair with both pilots at zero gain switched the whole tracker off, where the float
  tracker goes on with the other pair. Fixed (the pair's sum counts as 1); it needs both
  pilots some 48 dB below the strongest subcarrier, so no measured figure moves.

Of the reviewer's own 46 mutations 14 survived. Tests added
(`the_tracker_constants_at_their_edges`, `the_tracker_registers_saturate_and_refuse`,
`the_window_moves_beyond_half_a_sample_only`; 22 pass) and 20 mutations rerun, 17 killed:
the variance limit at 4, exclusive, and 0 taken as usable; λ truncated; D cut to M − 1
bits; the coefficient's rounding constant 13 for 14; the window moving at exactly half a
sample, either sign; a dead pair taking the other with it, either pair; z not saturated
and one bit wider; δ not saturated and one bit wider; symbols beyond the frame learned;
unusable widths learned. Not pinned, because no frame reaches them: the widths of Σz and
Στz (they cannot overflow once z is bounded) and the divisor's sign test (it is positive
whenever λ ≥ 0). The pin's first frame now runs z into its limit (its prior is wrong by
318 ppm) and so pins the saturation; the other four hashes did not change with the
review's fixes, which is why the report below, started before them, stands for this code:
its frames have clock offsets within ±40 ppm.

**The whole grid at these defaults** (`rxper fixed 2000` → `reports/rx_ofdm_fixed.txt`;
run on the build server, 32 threads, Linux x86-64, rustc 1.100.0, from the tree of
commit bfd619e — the code before the review's fixes, which do not touch these frames,
see above. The frames are not those of the report it replaces:
they depend on the thread count). Float with fine soft values, float at 8 / 3, and the
all-integer receiver at 8 / 3 on the same frames; SNR at 10 % PER against the first,
worst rate (Mb/s), and frames over all points of the channel:

| Channel | float 8 / 3, worst | integer 8 / 3, worst | frames lost, fine | float 8 / 3 − fine | integer − float 8 / 3 |
|---|---|---|---|---|---|
| AWGN | +0.01 dB (18) | +0.02 dB (54) | 14926 | +24 | +10 |
| TGn B | +0.03 dB (18) | +0.07 dB (54) | 10772 | +6 | +22 |
| TGn D | +0.01 dB (6) | +0.05 dB (54) | 10416 | +10 | −2 |
| TGn E | +0.00 dB (12) | +0.04 dB (48) | 11630 | +2 | +26 |

- **The receiver with nothing in floating point after the front is within 0.07 dB of the
  float receiver with fine soft values in every cell** (worst: TGn B, 54 Mb/s; the most
  negative cell is −0.05 dB, TGn E at 24 Mb/s, so single cells are at the measurement's
  resolution). Budget for the whole fixed receiver: 0.2 dB.
- In frames the integer arithmetic costs −2 to +26 per channel of some 10 000 to 15 000
  lost, 56 over the four channels; the run before it (float tracker inside, truncating
  rotator, other frames) had 108. The two runs do not share frames, so the difference
  between them is not a measurement of the tracker or of the rounding; the paired figure
  for the rounding is `fixedw`'s above.
- Not in this figure: the front in integers on the same frames (the front here is the
  float one; its own G2 is in `rx_front.md`), the gain control and DC in front, input
  levels other than −12 dBFS.

Open: the frames
of every `rxper` report depend on the number of threads (the seed is per thread), so a
report is reproduced only on a machine with as many cores as the one that made it — the
tool should draw each frame's seed from the frame's number.

## 2026-11-20 · G0 · R0-09 G2, stage B, second part: the receiver behind the gain control

**Question.** Every figure above was measured with the frame placed at −12 dBFS by a gain
that knows the frame, with the float front, and with nothing removed or added between the
ADC and the decoder. What does the all-integer receiver lose when the level is the one a
gain control leaves, the samples are the ADC's, the DC blocker is in the decoder's path and
the CFO is taken out by the integer rotator? And where is the lower edge of the input level?

**The chain measured** (all integers from the ADC to the PSDU):

| Step | Model | Format |
|---|---|---|
| gain control, DC, ADC | `channel::FastAttackAgc::run_dc` (power over 16 samples, the lock left on a large overload), `GainDc`, `agc_transient`, `rx_front_fixed::to_adc`; the gain held from the first lock inside the packet (assumed, as in `rxfront agcdc`) | 12 bits |
| DC blocker | `dc_block_fixed`, pole 2⁻⁷, as it is (told nothing) | Y = 13 bits |
| acquisition | `RxFrontFixed` on the same ADC samples: FFT start and the CFO word (2π/2^P per sample) | P = 22 |
| CFO out of the data | **new:** `rx_front_fixed::correct_fixed`: z(n) = y(n)·e^(−jφ·n), φ the CFO word, the front's own rotator (table of 2^L entries, amplitude 2^(Q−1) − 1, phase truncated to L bits) and its narrowing (half to even, saturating) to O bits | O bits |
| decoder | `decode_ppdu_fixed`, `FixedTracking::from_clock_offset` of the CFO word (the one float at the boundary, see stage B's G0) | `in_bits` = O |

**The hand-off's scale is the thing to measure.** The front's rotator output is Z = 12 bits
of a 13-bit sample times an 11-bit amplitude: z ≈ y/2, the ADC's last bit rounded away and
the frame 6 dB lower in the decoder's 12-bit word than in the ADC's. The reviewer's check in
stage A (one rate, not a committed report) put the integer data path level with the float one from −5 to −42 dBFS *of its own word*: at
O = 12 that is +1 to −36 dBFS at the ADC, and a frame near the sensitivity sits wherever the
radio's noise sits at the largest gain. Variants, the front itself unchanged (its Z stays 12):

| Name | O | `in_shift` | FFT word for an ADC sample a |
|---|---|---|---|
| 12 / 2 | 12 | 2 | 2·a, the ADC's last bit rounded away (the arithmetic of the front's rotator as it is) |
| 13 / 1 | 13 | 1 | 2·a, the rotated sample rounded to the ADC's last bit, not to twice it |
| 13 / 2 | 13 | 2 | 4·a, one bit less headroom |

**Reference.** The float receiver on the same ADC samples (float blocker, pole 2⁻⁷, float
front and `rx_ofdm` at 8 / 3), and the same frame without a gain control: placed at −12 dBFS
by a gain that knows it, the all-integer chain. For `correct_fixed` alone: the rotation
`rx_front::correct` makes, written out in the gate. (In all three variants the amplitude is
1023/1024 of the table's; in hardware the CFO word exists only once the acquisition is
declared, so the stream this function makes from sample 0 on is the hand-off's to arrange.) The budget of the whole fixed receiver stays 0.2 dB.

**Gates** (`tests/g2_rx_front.rs`): `correct_fixed` (1) worked by hand on a few samples,
(2) against the float rotation within the bound its table and rounding give, (3) equal to
the front's mixer at the phase −φ·n over streams in which that phase wraps, (4) saturating
and refusing unusable widths, (5) the whole chain bringing a frame back from ADC samples.

**Measurements** (`rxagc`, a new tool; every frame's random stream is drawn from the point
and the frame's number, so a report does not depend on the machine's thread count):
`per` — frames lost against SNR through the gain control; `level` — against the noise level
at the largest gain (the lower edge); `dc` — seven of the DC cases of `rxfront agcdc` through
the integer chain (another scene: its counts are not that report's). Clock offset uniform in ±40 ppm setting CFO and sample-clock offset at 5.8 GHz, as
in `rxper`.

## 2026-11-20 · G2 + measurements · stage B, second part: the receiver behind the gain control

Commits 0697d36 (G0, `correct_fixed`, its gates), d06cc0e (after the review), 762575d,
16875f6 and 2f7ce65 (`rxagc`). Commands:
`cargo test -p radio-phy --test g2_rx_front` (14 pass, 4 new);
`rxagc per 2000` → `reports/rx_ofdm_agc_per.txt` (build server, Linux x86-64, rustc 1.100.0,
the binary of 762575d: the header's wording was corrected after it, the arithmetic not);
`rxagc level 2000`, `rxagc dc 2000`, `rxagc pole 2000` → `reports/rx_ofdm_agc_level.txt`,
`…_dc.txt`, `…_pole.txt` (this machine, macOS arm64; `level` and `dc` were run twice, with
the binaries of 762575d and of 16875f6: every figure identical). A frame's random stream comes from
the row and the frame's number, so the reports do not depend on the thread count (the
reviewer ran 3 threads against 10: identical output); between the two machines the frames
agree only as far as their `sin` and `cos` do.

**`correct_fixed`, the gate.** The four tests were written before the function and seen
failing (they did not compile). Mutations, one at a time, restored with `cp` and `touch`,
each fails at least one test — mine (16): the step added; the phase advanced before use;
one bit fewer dropped; the width guard removed, 2..=31, 1..=30, 3..=29, 2..=24, 3..=30; the
rotator conjugated; the sample conjugated; the step halved; the output floored; the phase
starting one step in; half up at 13 bits; the negative limit −4095 at 13 bits. The reviewer's
(24, own list in the review below): 19 killed at first, the five survivors are the last
five of mine and die with the tests added after the review. Not observable, so not a
mutation: the accumulator's own wrap (the table reduces the phase again).

**1. Through the gain control the all-integer receiver is the float one**
(`per`; noise at −30 dBFS at the largest gain, no DC offset; both chains have the DC blocker
at 2⁻⁷ in the decoder's path). SNR at 10 % PER against the float chain behind a gain that
knows the frame, worst rate, and frames lost over the grid (paired against the float chain
behind the gain control: lost by the column alone / by float alone):

| Grid | float, gain control | int 12 / 2 | int 13 / 1 | int 13 / 2 | frames lost by float | int 12 / 2, paired | int 13 / 2, paired |
|---|---|---|---|---|---|---|---|
| AWGN, 8 rates × 6 points | −0.02 … +0.01 dB | −0.03 … +0.02 | −0.03 … +0.03 | −0.02 … +0.02 | 16 966 | +564 / −451 | +475 / −401 |
| TGn D, 8 rates × 6 points (54 Mb/s does not cross 10 %) | −0.01 … +0.10 dB | −0.03 … +0.11 | −0.02 … +0.16 | −0.03 … +0.15 | 13 602 | +222 / −188 | +226 / −170 |

- In AWGN no cell is further than 0.03 dB from the reference; the gain control costs the
  float chain nothing there (16 966 lost against 16 983 behind the known gain).
- The +0.10 to +0.16 dB are one rate, 36 Mb/s through TGn D, in every column including the
  integer chain behind the known gain (+0.16): its curve is shallow there (303 → 207 → 173
  frames over 2 dB), so 8 frames move the crossing by that much. In frames the integer
  chain loses 34 more than float of 13 602 over the TGn D grid, 113 of 16 966 in AWGN
  (0.7 %; the 13-bit hand-offs 74 to 83).
- 54 Mb/s through TGn D does not cross 10 % inside its grid in any column, and the AWGN
  crossings of 48 and 54 Mb/s stand at 18.9 and 20.8 dB where `rxper fixed` has 18.5 and
  19.9: that is finding 3, not the gain control.
- Where the gain moves (30 to 60 dB, 3 to 11 changes per frame, 6 / 24 / 54 Mb/s): no frame
  of 6 or 24 Mb/s is lost in AWGN by any chain, and the integer chains lose within 6 frames
  of float in every row (the TGn D block: 1442 against 1439 over its fifteen rows).
- Near the sensitivity the gain hardly moves with the noise at −30 dBFS (in AWGN the column
  `chg` is 0.00 up to 20 dB, through TGn D it reaches one change per frame from 24 dB): the
  low rates' rows measure a fixed level of −27 to −10 dBFS, not a transient.

**2. The lower edge of the input level is the hand-off's last bit** (`level`; the same
frames at every level; frames lost of 2000; "float known" is the reference behind the known
gain, 208 / 170 / 398 / 195 / 498 for the five points):

| Noise at the largest gain | AWGN 6 Mb/s 3.5 dB: float / 12-2 / 13-1 / 13-2 | AWGN 24 Mb/s 11.5 dB | AWGN 54 Mb/s 20 dB | TGn D 6 Mb/s 8.5 dB | TGn D 54 Mb/s 26.5 dB |
|---|---|---|---|---|---|
| −36 dBFS | 211 / 212 / 206 / 208 | 170 / 174 / 174 / 174 | 399 / 411 / 410 / 412 | 198 / 201 / 197 / 196 | 499 / 494 / 498 / 497 |
| −42 dBFS | 210 / 213 / 223 / 219 | 179 / 182 / 181 / 181 | 397 / 414 / 408 / 412 | 200 / 202 / 200 / 199 | 496 / 501 / 500 / 501 |
| −48 dBFS | 219 / 240 / 233 / 223 | 174 / 191 / 183 / 185 | 402 / 442 / 423 / 422 | 197 / 208 / 202 / 201 | 500 / 506 / 500 / 506 |
| −54 dBFS | 230 / 350 / 272 / 260 | 182 / 257 / 216 / 199 | 406 / 545 / 447 / 458 | 204 / 226 / 212 / 207 | 497 / 517 / 519 / 512 |
| −60 dBFS | 302 / 936 / 470 / 412 | 223 / 629 / 343 / 300 | 474 / 1029 / 736 / 683 | 218 / 316 / 242 / 233 | 510 / 607 / 560 / 558 |

- With the front's rotator as it is (12 / 2) the integer chain starts to lose against the
  float one when the noise at the largest gain is at −48 dBFS (17 to 40 frames of 2000 in
  the AWGN rows, +10 % of the losses) and has lost a third to a half as many frames again
  at −54 dBFS.
  **Requirement for the radio in front (to ADR 0008): the noise at the largest gain at or
  above −42 dBFS at the ADC**, or one more bit in the hand-off.
- One more bit (13 / 1 or 13 / 2) moves the edge down by about one row (6 dB): at −54 dBFS
  the 13-bit hand-offs lose what 12 / 2 loses at −48. 13 / 1 and 13 / 2 differ by less than
  the rows resolve (13 / 2 a little better at the lowest levels), so the loss is the
  rotator's rounding of the ADC's last bit, not the FFT word. Stage A's "level with float
  down to −42 dBFS of its own word" is the same fact seen from the decoder's side.
- The float chain on the same ADC samples loses from −54 dBFS on: there the ADC's own
  12 bits are the limit, which no hand-off recovers.
- The upper edge was not approached: the control locks at −10 dBFS and the three hand-offs
  agree there, 13 / 2 with one bit less headroom included.

**3. The DC blocker at 2⁻⁷ in the decoder's path costs the high rates more than everything
else in this entry** (`pole`; no DC offset; the blocker's pole in both paths; "raw" is the
float chain with the decoder handed the samples before the blocker; frames lost of 2000):

| Point | float raw | float 2⁻⁷ | 2⁻⁸ | 2⁻⁹ | 2⁻¹¹ | int 13 / 2: 2⁻⁷ | 2⁻⁸ | 2⁻⁹ |
|---|---|---|---|---|---|---|---|---|
| AWGN 54 Mb/s 25 / 30 / 40 / 60 dB | 0 / 0 / 0 / 0 | 32 / 16 / 12 / 8 | 2 / 1 / 0 / 1 | 0 | 0 | 31 / 16 / 12 / 5 | 3 / 1 / 0 / 1 | 0 |
| AWGN 24 Mb/s 11.5 dB | 158 | 171 | 164 | 159 | 161 | 169 | 166 | 161 |
| AWGN 36 Mb/s 14.5 dB | 214 | 251 | 227 | 225 | 212 | 252 | 234 | 233 |
| AWGN 48 Mb/s 18.5 dB | 213 | 326 | 283 | 241 | 221 | 334 | 273 | 255 |
| AWGN 54 Mb/s 20 dB | 167 | 403 | 278 | 200 | 172 | 412 | 287 | 202 |
| TGn D 6 Mb/s 8.5 dB | 182 | 183 | 182 | 182 | 183 | 183 | 181 | 183 |
| TGn D 24 Mb/s 17 dB | 208 | 216 | 208 | 209 | 207 | 216 | 212 | 211 |
| TGn D 36 Mb/s 21.5 dB | 184 | 281 | 225 | 197 | 187 | 284 | 222 | 194 |
| TGn D 54 Mb/s 26.5 dB | 226 | 511 | 415 | 304 | 259 | 517 | 416 | 310 |
| TGn D 54 Mb/s 40 dB | 0 | 251 | 153 | 55 | 14 | 251 | 152 | 61 |
| TGn D 54 Mb/s 60 dB | 0 | 267 | 175 | 67 | 29 | 267 | 177 | 68 |

- With nothing to remove, the blocker as it is more than doubles the frames lost at
  54 Mb/s near the sensitivity (403 against 167 in AWGN, 511 against 226 through TGn D) and
  leaves a floor of 13 % of the 54 Mb/s frames through TGn D at 40 and 60 dB, where the
  samples before it lose none. It is why `per`'s 48 and 54 Mb/s crossings stand 0.4 and
  0.9 dB above `rxper fixed`'s and 54 Mb/s through TGn D never reaches 10 %. 6 Mb/s does
  not feel it, 24 Mb/s hardly.
- A slower pole helps by about a factor of two per two bits and is not a cure at any pole
  an F = 10 blocker can hold: at 2⁻⁹ the fading floor is still 3 %, at 2⁻¹¹ (float only)
  1 %.
- `rx_front.md` (R0-07e, second half) had this as "8 of 2000 at 25 dB" in a scene without
  a sample-clock offset and asked for a look with R0-09 G2; this is the look. The cause is
  not established here. A guess to be tested, not a finding: the blocker is a high-pass
  with a memory of 128 samples, longer than a symbol, placed before the CFO is taken out,
  so at the largest CFOs (±232 kHz of a 312.5 kHz spacing) the subcarrier next to DC falls
  on its skirt and carries its own earlier symbols.
- **For ADR 0008: the decoder cannot be handed the output of this blocker.** What it is
  handed instead (the estimate frozen was worse in `rxfront agcdc`; an offset estimated
  over the long training field and subtracted; a notch after the CFO is out; a pole
  switched after the acquisition) is to be measured there with `rxagc`.

**4. With a DC offset the integer chain follows the float one, and the offset must be
removed** (`dc`; 54 Mb/s at 25 / 30 / 40 / 60 dB; frames lost of 2000):

| DC offset | float | float raw | int 12 / 2 | int 13 / 2 | int 13 / 2 raw |
|---|---|---|---|---|---|
| none | 32 / 16 / 12 / 8 | 0 / 0 / 0 / 0 | 32 / 16 / 11 / 6 | 31 / 16 / 12 / 5 | 0 / 0 / 0 / 0 |
| per index, −40 dBFS | 39 / 15 / 10 / 8 | 1 / 0 / 0 / 0 | 36 / 14 / 11 / 9 | 36 / 15 / 11 / 10 | 1 / 0 / 0 / 0 |
| per index, −30 dBFS | 62 / 18 / 19 / 14 | 507 / 385 / 377 / 344 | 61 / 20 / 19 / 15 | 62 / 20 / 19 / 15 | 504 / 385 / 375 / 345 |
| per index, −20 dBFS | 234 / 69 / 99 / 89 | 1664 / 1625 / 1636 / 1634 | 239 / 72 / 102 / 93 | 238 / 72 / 98 / 94 | 1666 / 1626 / 1634 / 1636 |
| before the gain, −20 dBFS at the largest | 68 / 30 / 36 / 30 | 1347 / 385 / 0 / 0 | 67 / 30 / 36 / 29 | 70 / 31 / 36 / 30 | 1342 / 385 / 0 / 0 |
| before the gain, −10 dBFS | 1115 / 199 / 218 / 209 | 1933 / 1604 / 386 / 0 | 1115 / 199 / 218 / 208 | 1119 / 199 / 215 / 207 | 1933 / 1610 / 387 / 0 |

- In every row the integer chains are within a few frames of the float chain, paired
  (at most +9 / −4). The fixed blocker and the 12-bit hand-off add nothing of their own
  under an offset.
- Without a removal an offset 20 dB below the packet (−30 dBFS) loses a fifth of the
  54 Mb/s frames and one 10 dB below it four fifths, in integers as in float. 6 Mb/s
  (report) loses no frame in any blocked column and, raw, only under an offset above the
  packet (10 dB SNR: 179 per index at −20 dBFS; before the gain 182 at −20 and 1808 at
  −10 dBFS).
- So finding 3 and this one pull against each other: the decoder needs the offset gone
  and cannot afford this blocker. With the blocker the columns of this table carry its
  own floor (the row "none").
- The row before the gain at −10 dBFS at 25 dB (1115 lost in every blocked column) is the
  blinded gain control of `rx_front.md`, unchanged in integers.

**What this is not.** One seed per row. The gain control is a model after one chip's
published state machine; the hold on the gain, the phase jumps and every DC level are
assumptions (`channel.md`). The blocker is told nothing; `restart` and `rescale` exist in
the float model only and were not rerun. A lost frame is not split into not acquired and
not decoded. Only the first acquisition of a stream is tried. TGn B and E, frames other
than 1000 octets and the upper edge of the level were not run. The front's own G2 on
these frames is not separated from the data path's (the integer columns share one
acquisition, so differences among them are the hand-off alone; float against integer
includes the front).

**Independent review (fresh agent, the code and G0, not this entry), 2026-11-20,** on
commit 762575d. No defect in `correct_fixed` or in the tool's arithmetic: it rederived the
function, recomputed every hand-worked number, found the float bound tight (worst error
0.93 to 0.98 of it), the three hand-offs to be what G0's table says, the draw order to
support "the same frames", the hold's indexing equal to `rxfront agcdc`'s, the two chains
treated alike, and the output identical at 3 and 10 threads. Found and corrected: rounding,
negative limit and the guard's inner edges not pinned at 13 bits (five surviving mutants,
tests added); the chain test started the tracker from nothing while its comment said from
the CFO word, and had no sample-clock offset (now ±31 ppm in both); the mean-phase check
passed for the slowest CFOs by the stream's length (the stream is whole table periods
now, and the tolerance says what a CFO of one unit can average); G0's "unpaired check",
"nothing rounded away", "the front's rotator as it is", "the rows of `rxfront agcdc`" and
the tool's "nothing in floating point"; the header's description of `level` and `chg`.
Left as they are: a panicking thread would show only in the `frames` column; `level` is a
float sum whose last digit could depend on the thread partition. Not checked by the
reviewer: any run above 30 frames per point, `scripts/check.sh`, the `is_valid` guard's
removal (it would allocate a 2²⁸-entry table), the `pole` command (written after it).

**Open**
- What the decoder is handed instead of the 2⁻⁷ blocker's output, and the guess about the
  cause tested (losses against |CFO|): ADR 0008, with `rxagc`.
- The hand-off's width: 12 bits with a requirement on the radio's noise level, or 13.
- `restart` / `rescale` in the fixed blocker if ADR 0008 keeps a control.
- `rxper` still seeds per thread; `rxagc`'s per-frame seed is the pattern to move it to.

## 2026-11-21 · G0 + G2 · behind ADR 0008's hand-off: the default input, the tracker's start from the CFO word, the G2 rerun

Commit: the one carrying this entry, on 613d358 (ADR 0008 accepted). Commands:
`cargo test -p radio-phy --test g2_rx_ofdm` (25 pass) and `--test g2_rx_front` (23 pass);
`rxper fixed 2000` → `reports/rx_ofdm_fixed.txt`; `rxper fixedw 2000` →
`reports/rx_ofdm_fixedw.txt` (both on the build server, 32 threads; they replace the reports
of 2026-11-20). The first two of the model follow-ups ADR 0008 names; the hand-off as a
function is in `rx_front.md` under the same date.

**G0 — what changed in the definitions.**

- *The decoder's input* (decision 1): `FixedOfdmConfig::default()` takes 13 bits
  (`in_bits` = `Y_BITS`) with one fractional bit (`in_shift` 1) in the 16-bit FFT word,
  instead of 12 and two. The sample's LSB is the ADC's; the word holds twice the ADC's full
  scale.
- *The tracker's start* (decision 7): `clock_ratio(fs, fc)` = round(fs/fc·2²⁰) as a 16-bit
  register (3616 for 20 MHz at 5.8 GHz; ratios below 1/16 fit), written by the MAC with
  the channel. `FixedTracking::from_cfo_word(word, ratio, fx)`: a clock offset
  ε = word·(fs/fc)/2^P (both ends derive carrier and sample clock from one oscillator,
  IEEE 802.11-2020 §17.3.9.5–6) drifts 16·ε samples in a fifth of a symbol, each 2^(P−6)
  units of slope, so with R fractional bits rate0 = word·ratio/2^(22−R), half to even, in
  a signed register of P − 6 + R bits (26 at the defaults): one 22 × 16-bit product per
  packet. A P-bit word and a 16-bit ratio stay below 2^(P+R−7), so the register never
  saturates in hardware; the model saturates a word no accumulator holds. The weight is the constant
  of the start from an offset known to 2 ppm (25 938 223). `from_clock_offset` (ppm as a
  float) stays for the tests and the float comparison. The model still computes the two
  weights' constants in floating point from `TIMING_HINT_PPM` and `TIMING_PRIOR_PPM`; in
  hardware they are two ROM words.

**Tests, written before the code** (each seen failing: the default's constants; the missing
functions):
- `the_default_input_is_the_hand_off_of_adr_0008`: the constants; and a 54 Mb/s frame
  whose mean power is the 12-bit full scale squared, handed at 13 bits, decodes with the
  default input and not with a 12-bit one.
- `the_tracker_starts_from_the_cfo_word_by_hand`: the register for three channels and its
  limits (0, 65535, 65535.5, not a number); the rate at R = 10 (ties to even on both sides
  of zero), R = 0 and R = 20; 48 654 (232 kHz, 40 ppm at 5.8 GHz) → 42 952 against 42 950
  from 40 ppm itself; the largest word and ratio inside the register, words no accumulator
  holds saturating at its ends (2²⁵ − 1 and −2²⁵; 2³³ − 1 and −2³³ at P = 30), a word
  whose product does not fit 64 bits among them; widths `is_valid` refuses (no panic); the
  weight.
- `the_start_from_the_cfo_word_is_the_start_from_the_offset`: against the float conversion
  the tools used (word → Hz → ppm → `from_clock_offset`) over five channels, R of 6, 10
  and 20, P of 22 and 30, offsets to ±60 ppm: the rates differ by at most
  1 + |word|·2^(R−23) (two roundings and the register's half LSB); the largest difference
  is 0.98 of that bound.
- The chain from the ADC with this start, and its sign held to the clock:
  `the_default_receiver_decodes_the_hand_off` (`rx_front.md`).

**Tests changed by the default.** Four of the older tests fed 12-bit samples to the default
words and now run with one fractional bit fewer:
- `the_default_words_bit_for_bit`: the five pins are new (they were 797987032209841876,
  2722506916840601925, 2803077607927797426, 16026798777574690161,
  12466612513770086848).
- `samples_outside_the_input_width_saturate`: overdrives 13 bits now.
- `the_tracker_in_the_receiver_is_the_float_one`: the bound on the measured slope's
  difference is 5·10⁻⁴ rad per subcarrier instead of 2·10⁻⁴. The largest difference was
  9.6·10⁻⁵ with two fractional bits and is 4.0·10⁻⁴ in one symbol (6 Mb/s, 6 dB, +40 ppm,
  symbol 184) and at most 1.6·10⁻⁴ in every other of 2316; the drift differs by at most
  0.00019 of a sample (was 0.00015). The measurement's own deviation at that SNR is two
  orders above.
- The other tests pass unchanged, the standard's own waveform among them
  (`g1_annex_i1`).

**Mutations** (clean build each, restored by `cp` + `touch`). The default: `in_bits` 12,
`in_shift` 2, `in_shift` 0 — all fail. `from_cfo_word` and `clock_ratio`, thirteen: the
shift one less and one more, floor instead of half to even, the limit halved, the sign,
the word cut to 32 bits, the unknown start's weight, the register floored, 0 accepted,
65536 accepted, 19 fractional bits, R fixed at 10, P fixed at 22. Twelve failed at once;
the last survived (no test left P = 22 with a saturating word) and fails since the P = 30
cases were added. After the review (below) seven more, all failing: R and P taken unclamped,
a 64-bit wrapping product, a register value of 1 refused, the register one bit wider and
one narrower, `handoff_fixed`'s width taken from the front's `z_bits`.

**`rxper` after ADR 0008.** With an integer receiver in the set (`fixed`, `fixedw`) every
receiver now starts from the same 12-bit ADC samples (the frame at −12 dBFS by a gain that
knows it, no DC offset in the scene) and the same acquisition, the integer front's
(`acquire_fixed`, its DC blocker in its own path). The integer receiver is handed
`handoff_fixed`'s stream and starts its tracker from the CFO word and the register; the
float receivers are handed the same samples less the float estimate (`ltf_dc_offset`),
rotated by `correct` at the word's frequency. Until now the float front acquired and
corrected the channel's samples and only then were they quantised to 12 bits: the rotator,
the offset's estimate and the 13 / 1 hand-off were outside `rxper`'s comparison (they were
measured in `rxagc`). Frames are seeded by the point and the frame's number (`rxagc`'s
pattern), so a report no longer depends on the thread count and the count of frames is
exact; the frames are therefore not those of the earlier reports. The float-only commands
(`awgn`, `tgn`, `soft*`, `timing`, `sfo`) keep the float front and get the new seeding
too: their committed reports were not rerun and are at their own commits.
`rxagc` still starts its integer chains through the float conversion; it moves to the word
with the gain control's timer and hold, when its reports are rerun.

**G2 results** (`rxper fixed`, 2000 frames of 1000 octets per point, clock offset uniform
in ±40 ppm; SNR at 10 % PER against the float receiver with fine soft values, worst rate
of each channel in Mb/s, and frames lost over all points of the channel):

| Channel | float 8 / 3, worst | integer, worst | most negative | frames lost, fine | float 8 / 3 − fine | integer − float 8 / 3 |
|---|---|---|---|---|---|---|
| AWGN | +0.02 dB (24) | +0.04 dB (48, 54) | −0.01 dB | 15076 | +12 | +84 |
| TGn B | +0.03 dB (36) | +0.03 dB (24, 36, 54) | −0.05 dB | 10840 | +8 | +32 |
| TGn D | +0.00 dB | +0.04 dB (9, 12) | −0.02 dB | 10534 | −6 | +50 |
| TGn E | +0.01 dB (9, 12) | +0.03 dB (54) | −0.03 dB | 11486 | −8 | +52 |

- **The integer receiver from the ADC on — the hand-off's rotator, the offset's estimate,
  13 / 1, the data path and the tracker started from the CFO word — costs at most
  +0.04 dB in a cell** against the budget of 0.2 dB for the whole fixed receiver. The
  crossings scatter by a few hundredths either way (the most negative cell is −0.05 dB),
  so single cells are at the measurement's resolution; the frame sums are the steadier
  figure: 32 to 84 frames more than the float receiver at the same soft format, of some
  10 500 to 15 000 lost.
- The sums are larger than in the report this one replaces (+10, +22, −2, +26 with the
  float front correcting the samples before they were quantised): the hand-off's rotator
  is inside the comparison now. They are still a few tenths of a percent of the frames
  lost.

**The widths** (`rxper fixedw`, frames lost of 2000, the same frames in every column; rows
AWGN 6 Mb/s 3.5 dB, AWGN 54 Mb/s 20 dB, TGn D 18 Mb/s 15.5 dB, TGn D 54 Mb/s 26.5 dB,
TGn E 54 Mb/s 26.5 dB; no column but U10 and U12 loses a frame at the two high-SNR points):

| Column | lost |
|---|---|
| float | 217 / 181 / 182 / 220 / 144 |
| the defaults, tracker from ppm | 220 / 191 / 181 / 221 / 143 |
| the defaults, tracker from the CFO word | 220 / 191 / 181 / 221 / 143 |
| every width far beyond need | 218 / 182 / 182 / 217 / 146 |
| the front's widths alone beyond need | 218 / 181 / 182 / 217 / 143 |
| the data path's alone | 221 / 194 / 182 / 221 / 148 |
| rotator L11 Q14 | 218 / 184 / 181 / 221 / 144 |
| rotator L8 | 223 / 206 / 181 / 229 / 154 |
| rotator L7 | 218 / 246 / 180 / 248 / 177 |
| rotator Q9 | 220 / 202 / 182 / 225 / 145 |
| hand-off 12 bits, two fractional (Z12 s2) | 221 / 191 / 183 / 216 / 146 |
| 13 bits, two fractional (W16 s2) | 221 / 189 / 182 / 220 / 144 |
| 13 bits, none (W16 s0) | 218 / 192 / 182 / 221 / 145 |
| FFT word 14 bits (W14 s1) | 221 / 191 / 182 / 224 / 145 |

- **The start from the CFO word loses exactly the frames the float conversion loses**, at
  every point.
- **13 / 1 against 12 / 2 and 13 / 2: within 8 frames either way at every point**
  (the default with the truncated phase, the base of these columns: 221 / 191 / 182 / 224 / 145), at
  this level (−12 dBFS, where the ADC's last bit is not what limits; the lower edge of the
  level is `rxagc level`'s). One fractional bit is enough, as is none at this level.
- **Where the default differs from float it is the rotator, and more than before:** AWGN
  54 Mb/s 20 dB loses 191 against 181 (it was 188 against 184 when only the data path's
  rotation was in integers); with the front's widths beyond need 181, with L11 Q14 184.
  Ten frames of 2000 at the steepest point of the grid, inside the +0.04 dB above. L8 costs
  15 more there and L7 55: L9 Q11 stays, and a wider table (L11) is the first place to
  buy back the difference if the budget is ever short.
- The floors that W14 s2 and W16 s4 left with a 12-bit input (33 frames at 45 dB in
  TGn E) are gone: those columns no longer exist (13 + 2 does not fit 14, 13 + 4 not 16),
  and W14 s1 and W16 s3 lose nothing at 45 dB. U10 still does (41), U12 one frame.
- The other columns are as on 2026-11-20 (C6, T8, U10, Q8 lose; the tracker's R6, M8, K8,
  V4 do not): the report has them.

Both reports are byte for byte the same rows on 10 and on 32 threads (`fixedw` was run on
both machines).

**Independent review (fresh agent, the code and ADR 0008's decisions 1 and 7, not this
entry), 2026-11-21,** on 6c94df0. No defect in the arithmetic: it rederived
`from_cfo_word` (units, sign, independence of P, the shift, no panic path for any widths),
recomputed every hand-worked number, confirmed the bound of the float comparison, that the
draws do not depend on the receiver set, the same samples and acquisition for float and
integer, the output identical at 3 threads, and `word_at` for the columns at P = 30.
Found and corrected:
- five surviving mutants of its twelve (R and P unclamped — a panic for widths `is_valid`
  refuses; a 64-bit wrapping product; a register value of 1 refused; `handoff_fixed`'s
  width from `z_bits`): tests added, all fail now;
- the rate's saturation was ±2^(P−6+R) inclusive, a limit no register width has: it is a
  signed register of P − 6 + R bits now (reachable values unchanged);
- wording: "ratios up to 1/16"; a stale comment on the ±π event (none in this run any
  more, so that branch of the test is not taken); `fixedw`'s header on "wide" (the
  acquisition is the default front's in every column) and on "round" (the same
  configuration as "word start");
- `rxper`: a missing offset estimate lost the frame for the integer receiver at 13 bits
  and decoded without removal elsewhere (unreachable today; the same now).
Its measurement of the tracker comparison against the fractional bits (rms 2.2·10⁻⁵ at two
and three, 3.4·10⁻⁵ at one, 8.1·10⁻⁵ at none; the maximum one heavy-tailed symbol) is why
the bound of 5·10⁻⁴ is recorded as close to that one sample and not as a margin.
Left as they are: `rxper` has no DC offset in its scenes, so the equivalence of the float
and integer offset removal is not shown by these reports (it is `rxagc`'s, `rx_front.md`),
nor is the 0.1 % of frames in which the two guards decide differently; `word_at`'s
narrowing branch is dead; a panicking thread would under-report; TGn D and E at 54 Mb/s
26.5 dB share a seed in `fixedw` (different channels).
Not checked by it: any run above 100 frames, `rxper fixed`, the journal's numbers,
`scripts/check.sh`.

**What this is not.** One seed per point. A gain that knows the frame and no DC offset:
the gain control, the level's lower edge and the offset are `rxagc`'s. The reports were
run at 6c94df0; the commits after it change the saturation of unreachable rates, tests,
texts and the tool's handling of a stream too short for the estimate, and `fixedw` was run
again on the final code with the same rows.

**Open**
- The second history and the replay in `RxFrontFixed`; the gain control's hold; then RTL.
- `rxagc` on the word start with its reruns: done 2026-11-21, `rx_front.md` ("`rxagc`'s other
  reports on the word start"): 26 integer cells of 3674 move, by one or two frames.
- Narrowing the tracker (R 6, M 8, K 8, V 4 lose nothing here either) before its RTL.
