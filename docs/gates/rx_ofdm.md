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

**Evidence** (`cargo test --release -p radio-phy --test g1_rx_ofdm -- --nocapture`: 18 tests
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
| 4 / 0.75 | 0.11 | 0.97 dB | 0.36 dB | 0.73 dB |
| 4 / 1 | 0.14 | 0.77 | 0.62 | 1.25 |
| 4 / 1.5 (today) | 0.21 | 0.43 | 1.35 | 3.03 |
| 4 / 3 | 0.43 | 0.50 | no crossing | no crossing |
| 5 / 1 | 0.067 | 0.74 | 0.20 | 0.27 |
| 5 / 1.5 | 0.10 | 0.35 | 0.43 | 0.97 |
| 5 / 2 | 0.13 | 0.13 | 0.67 | 0.94 |
| 5 / 3 | 0.20 | 0.14 | 1.08 | 2.65 |
| 6 / 1.5 | 0.048 | 0.34 | 0.08 | 0.30 |
| 6 / 2 | 0.065 | 0.19 | 0.23 | 0.34 |
| 6 / 3 | 0.097 | 0.13 | 0.28 | 0.96 |
| 6 / 6 | 0.19 | 0.16 | 0.83 | 2.50 |

- Fading needs a small step: the weak subcarriers' values must not round to 0. AWGN needs
  the clip at 1.5 or more for 16- and 64-QAM. Four bits cannot have both, five nearly; six
  bits with a clip of 1.5 to 2 is the first format within about 0.35 dB everywhere.
- These figures rank the formats and no more. Single cells differ by up to 0.3 dB between
  the two runs, and "losses" as low as −0.3 dB occur: that is the noise of 200 to 300
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
1. The soft-value format in fading (above). A G2 decision of the Viterbi decoder's input, so
   the founder's (it was approved at 4 bits on AWGN evidence, with this caveat recorded).
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
