# Gate journal: HT LDPC codes (R0-12)

Block: the twelve LDPC codes of 802.11n, their encoder, the PPDU encoding process, and the
decoders. Model: `crates/radio-phy/src/ldpc.rs`, tables `ldpc_tables.rs`, decoders
`ldpc_decoder.rs`.

## 2026-11-19 · G0 · definitions

- **Standard:** IEEE 802.11-2020 §19.3.11.7 (LDPC codes: .2 coding rates and codeword
  lengths, .3 encoder, .4 parity-check matrices, .5 PPDU encoding process); the matrix
  prototypes are Annex F, Tables F-1 to F-3. Clause and table numbers are the same in
  802.11-2024, the copy used.
- **Codes:** n = 648, 1296, 1944 bits (Z = 27, 54, 81), R = 1/2, 2/3, 3/4, 5/6; H has
  24·(1 − R) block rows and 24 block columns; a block is zero or the Z × Z identity with its
  columns cyclically shifted right by the prototype's entry: row r of a block with shift s
  checks bit (r + s) mod Z of that block column.
- **Codeword:** c = (i_0 … i_{k−1}, p_0 … p_{n−k−1}), H·cᵀ = 0, systematic. Bits are `u8`
  0/1 in transmission order.
- **Encoder:** every prototype's parity part is one column with shifts 1, 0, 1 (top, one
  row between, bottom) followed by a double diagonal of unshifted identities, so the parity
  is a back-substitution: the first parity block is the sum of the information parts of all
  block rows, the others follow row by row. `LdpcCode::new` checks that structure on the
  table and returns `None` otherwise; nothing is assumed.
- **PPDU encoding process:** `LdpcPlan::new(N_pld, N_CBPS, R, m_STBC)` gives N_CW, L_LDPC,
  N_avbits, N_shrt, N_punc, N_rep, N_SYM (steps a to e, Table 19-16), in integers: the
  comparisons with 0.1, 0.3, 1.2 and (1 − R) are multiplied through, no floating point.
  `shortened`, `codewords` and `encode` are steps c to f. The stream parser and what
  follows (step g) belong to the HT PHY, not to this block.
- **Matrices in the repository** (founder, 2026-11-19; ADR 0005 addendum): transcribed by
  `tools/ref/ldpc_matrices_extract.py` from a licensed copy into the generated
  `ldpc_tables.rs`, no digit typed by hand.

## 2026-11-19 · G1 · encoder and PPDU encoding process

**References we did not write**
| # | Reference | Test (`radio-phy/tests/g1_ldpc.rs`) |
|---|---|---|
| 1 | The standard's two worked examples, Annex I.2 and I.3: the bits before and after scrambling, after shortening, after encoding, after puncturing (example 1) or repetition (example 2), ten tables, 15 034 bits (six of them, 11 130 bits, are this block's: after shortening, after encoding, after puncturing or repetition; the two before scrambling are the input itself and the two after it test the scrambler); in CI by SHA-512 of each table (`tools/ref/annex_ldpc_hash.py`, CPython hashlib, from a licensed copy), bit by bit locally | `worked_examples_by_hash`, `worked_examples_bit_by_bit` (`#[ignore]`, local fixture) |
| 2 | The derived parameters printed with the examples (N_CW, L_LDPC, N_avbits, N_shrt, N_punc, N_SYM, N_rep) | `derived_parameters_of_the_worked_examples` |
| 3 | An outside copy of the twelve prototypes: github.com/tavildar/LDPC (MIT), `LdpcC/WiFiLDPC.h` at commit 05ee7f4, read as data by `tools/ref/ldpc_matrices_outside.py`; by SHA-512 of each matrix | `prototypes_match_an_outside_copy` |
| 4 | The definition: a codeword is what H checks. H is built from the prototype by a route that shares nothing with the back-substitution; all twelve codes, random and single-bit information blocks, linearity, one flipped bit seen | `encoder_output_is_a_codeword_of_every_code`, `twelve_codes_with_the_standard_dimensions` (k of Table 19-15) |

- The model matched all ten tables of both examples on its first run (of which six say
  something about LDPC, see the table above). Example 1 is one
  (1944, 3/4) codeword with 642 shortening bits and 54 punctured; example 2 is two
  (1296, 1/2) codewords with 80 shortening and 80 repeated bits each.
- The scrambler seed is not among the examples' printed parameters: the test takes the one
  seed of the 127 whose output hashes to the printed table (exactly one must).
- The examples' inputs are in the fixture in clear: the 100-octet PSDU of example 1 (the
  same as Annex I.1's, already decided) and the 140-octet PSDU of example 2 (Table I-37), by
  the same reasoning; the founder allowed the second one on 2026-11-19 as well, and the
  ADR 0005 addendum now names both.
- Reference 3 is what holds the ten codes the examples do not use: an encoder and a checker
  made from one table agree on a wrongly transcribed table (mutations 11 and 12 below).
  Its weakness is that it is one outside source, itself a transcription; both agree in all
  2592 entries.

**Our own second reading** (not an outside reference)
- `plan_cases_worked_by_hand`: nine cases of Table 19-16 and of step d) that the two
  examples do not reach, each worked by hand from the clause in a comment (the inputs were
  found by a throw-away search for parameters at which a wrong threshold would change the
  result): the constants 912 and 1464, the range bounds 648, 1296 and 1944, the "≥" of the
  table, the three constants of step d).
- `repetition_goes_through_the_parity_and_round_again`: step e) where example 2 does not
  go (it repeats 80 of 568 data bits): 424 repeated bits for 224 sent, content checked.
- `plan_invariants`: 16 MCS × with and without STBC × 405 lengths: the pieces add up to the
  symbols exactly, shortening fills the codewords, puncturing and repetition exclude each
  other, the payload is found in order in what is sent; 6925 plans puncture, 5905 repeat,
  1984 take the extra symbol.

**Mutations** (each fails the tests named; `cp` + `touch` to restore)
| Mutation | Fails |
|---|---|
| H: shift direction reversed | codeword test, plan invariants |
| encoder: shift direction reversed | codeword test, both example tests |
| encoder: shift of the first parity column 1 → 0 | codeword test, both example tests |
| the extra shortening bit in the last codewords instead of the first | plan invariants |
| puncture from the start of the parity instead of its end | both example tests |
| repeat from the end of the codeword instead of its start | both example tests |
| extra-symbol rule removed | plan invariants |
| N_CW rounded down | plan invariants |
| Table 19-16: 2916 → 2900 | parameters, both example tests |
| Table 19-16: 1464 → 1458; 912 → 900 | survived at first; now `plan_cases_worked_by_hand` |
| step d): 1.2 → 1.0; 0.3 → 0.4; 0.1 → 0.2 | survived at first; now `plan_cases_worked_by_hand` |
| 11. one shift changed in the (1944, 3/4) prototype | both example tests |
| 12. one shift changed in the (648, 5/6) prototype; a zero block made an identity in (1296, 2/3) | survived the codeword and example tests, as expected; `prototypes_match_an_outside_copy` |

- With whole octets no PPDU of the sixteen single-stream MCSs separates 1464 from 1458 (a
  search over lengths 1 to 3000 found none): the case that holds it has 810 payload bits.
  The rule is stated on bits and the function takes bits.

**Pre-PR review (fresh agent), 2026-11-19:** no error in the encoder or in the plan against
the clause (its own exact-fraction reading of §19.3.11.7.5 agreed on 1 027 976 plans, and
its own reconstruction of steps c to f on about 15 000 with `encode` bit for bit); the
fixtures regenerate byte for byte. Found and corrected: `LdpcPlan::new` multiplied before
its range check (an overflow panic in debug builds on a refused N_CBPS); five mutations of
its own that no test held (the range bounds 648 and 1944, "≥" against ">", repetition into
the parity) — four have tests now, the fifth (">" against "≥" in step d's 0.1 comparison)
has no case found; a hand-built plan could divide by zero (now refused); the count of bits
and of tables that bear on LDPC in this entry.

**Evidence:** `cargo test -p radio-phy --test g1_ldpc` — 9 pass, 1 ignored (it passes with
the local fixture: `ANNEX_LDPC_FIXTURE=… cargo test … -- --include-ignored`, 10 pass);
`scripts/check.sh` green; this entry's pull request.

**Open questions**
1. The decoder (normalised min-sum, layered), its G1 against published error-rate curves and
   its G2 (LLR and message widths): the next part of R0-12.
2. A second outside source of the matrices, or of the PPDU encoding process as a whole (a
   commercial chip's capture at R1 would be the strongest: an LDPC frame from a Wi-Fi card
   decoded by our model).
3. The HT PHY around it (HT-SIG, stream parser, the 52/108-subcarrier mapping) is not in
   R0's scope; the plan takes N_CBPS and m_STBC as numbers.
4. `ldpc_tables.rs` is generated from a document CI does not have, so CI cannot regenerate
   it as it does `hw/vectors/`: the outside copy's hashes are its guard there.

## 2026-11-19 · G0 · decoder definitions

The standard defines the codes, not a decoder. Ours, in `ldpc_decoder.rs`:

- **Soft value:** a log-likelihood ratio ln(P(0)/P(1)): positive says 0, 0 is an erasure.
- **Float reference** (`decode_float`): belief propagation with the exact check rule
  2·atanh(Π tanh(m/2)) or min-sum with a scale, on a flooding schedule (all checks, then
  all bits) or a layered one (check after check, each using the totals the checks before it
  updated). Stops when every check holds on the hard decisions; 0 iterations if the input
  already is a codeword. The input is limited to ±10⁶ on entry (`LLR_LIMIT`), so an
  infinite value is a known bit and nothing infinite meets inside; a NaN is refused, and
  so is a min-sum scale outside (0, 1].
- **Hardware algorithm** (`decode_fixed`, `MinSumConfig`): layered normalised min-sum in
  integers. Checks in the order of `LdpcCode::checks` (block row after block row; the Z
  checks of a block row share no bit). Per check: a bit's message is its total minus this
  check's last message, saturated to the total's width; the new message is the sign product
  of the others (0 counts as positive) and `(min · scale_num) >> scale_shift`, saturated to
  the message width; the new total is message plus new message, saturated. Hard decision 1
  where the total is negative. Two minima, the position of the first and one sign bit per
  edge are all a check needs to keep. Widths, the factor and the iteration count are
  parameters: G2 chooses.
- **PPDU side** (`LdpcPlan::soft_codewords`, `payload`): shortened bits enter as a sure 0,
  punctured bits as 0, the soft values of repeated bits are combined with their originals
  (`Soft::combined`): floats add; integers add in `i32`, saturating at its ends and never
  wrapping, and the decoder then saturates its input to the width of its totals. So the
  model's sum is exact for any receiver width, and the hardware's accumulator (how wide,
  or saturating at each addition, which is not the same when signs mix) is still to be
  defined: with the widths, at G2. A plan is refused unless its pieces add up to N_avbits
  (N_CW·L − N_shrt − N_punc + N_rep), checked before anything is allocated.
- **Notes for the hardware** (from the pre-PR review, the arithmetic checked here):
  ⌊3·min/4⌋ is `(x + 2x) >> 2`; `x − (x >> 2)` is the ceiling, a different decoder. At
  ×3/4 a minimum of 1 becomes 0: with a coarse input scale the smallest messages vanish
  (see the finding under G1). Limiting the magnitudes to a bound M before the comparison
  gives the same messages whenever `(M · scale_num) >> scale_shift` ≥ the message maximum
  (the scaling is monotonic), so the comparators need not be as wide as the totals. The
  layer order is fixed and sequential in the model; hardware that takes the Z checks of a
  block row together computes the same thing, any other order does not.

## 2026-11-19 · G1 · decoders

**References we did not write**
| # | Reference | Test (`radio-phy/tests/g1_ldpc_decoder.rs`) |
|---|---|---|
| 1 | The check rules by enumeration: the exact rule is the log-ratio of the probabilities that the other bits of a check have even and odd parity, summed over all their 2^(d−1) values; min-sum is the same with the best even and the best odd assignment. 400 random checks of 2 to 10 bits, some with an erasure | `check_rules_are_the_enumerated_ones` |
| 2 | An outside decoder as a black box: CommPy 0.8.0 `ldpc_bp_decode` (sum-product and min-sum, flooding, 50 iterations) on matrices expanded from the outside copy of the prototypes, so nothing of our model is on that side (`tools/ref/ldpc_ber_commpy.py`; all-zero codeword, BPSK, AWGN; 5000 blocks per point). Ten points of three codes, block error rates 0.4 % to 73 % | `flooding_error_rates_match_commpy` |
| 3 | The encoder (held by the standard's worked examples): six PPDUs, among them both examples' shapes, come back through puncturing, shortening and repetition, in the float and in the fixed decoder | `a_ppdu_comes_back_through_shortening_puncturing_and_repetition`, `soft_codewords_put_every_value_in_its_place` |

**Float flooding decoder against CommPy.** The test runs 400 to 4000 blocks per point (enough
for about twenty block errors where the reference has few) and asks for agreement within
four standard errors of both sides, on the block error rate at every point and on the bit
error rate where at least ten blocks were wrong. With 5000 blocks at every point
(`LDPC_GATE_BLOCKS=5000 cargo test --release -p radio-phy --test g1_ldpc_decoder flooding -- --nocapture`;
run again after the review's corrections, the same figures):

| Code | Rule | Eb/N0 | BER ours | BER CommPy | BLER ours | BLER CommPy |
|---|---|---|---|---|---|---|
| (648, 1/2) | sum-product | 1.0 dB | 3.46·10⁻² ± 0.07 | 3.44·10⁻² ± 0.07 | 0.350 | 0.348 |
| | | 1.5 dB | 6.24·10⁻³ ± 0.35 | 6.49·10⁻³ ± 0.35 | 0.072 | 0.073 |
| | | 2.0 dB | 6.4·10⁻⁴ ± 1.1 | 4.0·10⁻⁴ ± 0.8 | 0.009 | 0.006 |
| (648, 1/2) | min-sum | 1.5 dB | 5.21·10⁻² ± 0.10 | 5.29·10⁻² ± 0.10 | 0.376 | 0.378 |
| | | 2.0 dB | 8.10·10⁻³ ± 0.46 | 8.37·10⁻³ ± 0.46 | 0.067 | 0.069 |
| | | 2.5 dB | 3.4·10⁻⁴ ± 1.0 | 5.2·10⁻⁴ ± 1.2 | 0.003 | 0.004 |
| (648, 5/6) | sum-product | 3.0 dB | 1.26·10⁻² ± 0.02 | 1.30·10⁻² ± 0.02 | 0.430 | 0.443 |
| | | 3.5 dB | 2.20·10⁻³ ± 0.11 | 2.24·10⁻³ ± 0.11 | 0.084 | 0.088 |
| (1296, 3/4) | sum-product | 2.0 dB | 3.40·10⁻² ± 0.03 | 3.39·10⁻² ± 0.03 | 0.727 | 0.727 |
| | | 2.5 dB | 4.85·10⁻³ ± 0.19 | 5.13·10⁻³ ± 0.19 | 0.128 | 0.135 |

(± in units of the last power of ten.) The largest differences are about two standard
errors. On the first run with 400 blocks two points looked better than CommPy by up to 3σ
(1.5 dB sum-product, 2.0 dB min-sum); with 5000 blocks they are 0.5σ: noise, not a
difference. The reference covers three of the twelve codes; the other nine share the
decoder and differ in the matrix, which part 1 holds.

**What has no outside reference** (held to the float flooding decoder, which has one)
- The layered schedule: `layered_and_fixed_decoders_are_held_to_the_flooding_one`, (648, 1/2)
  at 1.75 dB, 400 blocks, normalised min-sum 0.75: block errors flooding/50 26, layered/25
  25, layered/50 21, fixed/25 25; mean iterations of the decoded blocks 12.4, 6.8, 7.1,
  6.8. Layered needs 0.55 of flooding's iterations for the same result. The decoders see
  the same noise, so the test compares the blocks only one of a pair gets wrong: layered/25
  alone 1, flooding/50 alone 2; fixed/25 alone 1, layered/25 alone 1. The later decoder may
  have 4 more than the earlier. (The first version compared totals with a slack of 9 on 9
  block errors, which a decoder 0.2 to 0.3 dB worse would have passed: review finding.)
  What the criterion separates, measured: a layered decoder cut to 12 iterations 22
  against 1, an unnormalised one 59 against 3; one at scale 0.875 passes (6 against 11:
  it is better, as G2's first run also says).
- The fixed decoder: the same test (input at 64 units per unit of LLR, 16-bit messages,
  18-bit totals: fine enough for rounding not to matter), and
  `fixed_decoder_is_its_written_definition`: a second writing of the G0 definition without
  the two-minima shortcut, compared bit for bit and in its iteration count on three codes ×
  five width sets (down to 2-bit messages and 4-bit totals) × six noisy blocks. Our own
  second reading, like the plan cases of part 1.
- **Finding for G2: the input scale matters more than expected.** The first version of
  the test fed the fixed decoder 8 units per unit of LLR and called that fine enough. It
  is not: of 1000 blocks at 1.5 dB the fixed decoder alone loses 43 and the float layered
  one alone 4 (176 block errors against 137); at 64 units it is 9 against 3 (143 against
  137), at 4 units 76 against 6. The widths were ample in all three, so this is the
  rounding. My reading, not isolated yet: the scaled minimum is rounded down, a loss of
  3/8 of a unit on average, which at 8 units is a twentieth of an LLR on messages that are
  often near 1. G2 must try rounding to nearest in the scaling, `(3x + 2) >> 2`, before it
  spends bits on the input.
- With the default widths and an input of 2 units per unit of LLR the fixed decoder loses
  visibly (4 block errors of 300 against 1 at 2.5 dB in a first run): the input scale and
  the widths are G2's subject, not decided here. `MinSumConfig::default()` is a wide
  decoder for tests; the hardware's is `MinSumConfig::HARDWARE` (G2, below).

**Mutations** (each fails the tests named; `cp` + `touch` to restore)
| Mutation | Fails |
|---|---|
| exact rule: tanh(x) for tanh(x/2) | enumeration, CommPy |
| float min-sum: a bit's own magnitude not excluded | enumeration, CommPy, schedules |
| fixed: own magnitude not excluded; own sign not excluded | PPDU round trip, schedules (and the definition test, added later) |
| fixed: no normalisation | schedules, refusals |
| fixed: message, total, or message to the check not saturated | survived at first; `fixed_decoder_is_its_written_definition` |
| fixed: a zero message counts as negative | definition test, schedules |
| fixed: input not saturated | survives: equivalent (a bit's total is first read by a check, which saturates it; nothing else reads it before) |
| float layered reads the input instead of the running totals | schedules |
| flooding reads the running totals (becomes layered) | schedules (iteration ratio) |
| float: the stop test ignored | schedules (iteration counts) |
| punctured bits entered as a sure zero | value placement, PPDU round trip |
| repeated values not added | value placement |
| after the review: float input not limited | `known_bits_may_be_infinite` |
| min-sum scale not validated | refusals |
| repeated integers added with wrap-around | `repeated_values_saturate_and_never_wrap` |
| a plan's pieces not checked against N_avbits | refusals |
| fixed: no normalisation (again, under the new criterion) | definition test, schedules, refusals |
| float layered reads the input instead of the running totals (again) | schedules, `known_bits_may_be_infinite` |
| in the test: layered cut to 12 iterations; fixed input at 4 units | schedules |

**Pre-PR review (fresh agent), 2026-11-19:** no error in the algorithms for finite inputs;
by its own measurement the CommPy gate fails a float decoder displaced by ±0.1 dB on 4 of
4 seeds (its floor is about 0.05 dB). Found and corrected: float min-sum with infinite
inputs answered all-zero and "converged" (inf − inf inside; the old test tried one
infinity with sum-product and looked at the length only); repeated integer values were
added with a bare `+=` (wrap-around in release builds); a hand-made plan could allocate
and index by numbers nothing had tied to the input's length; the schedule test's slack;
the 5000-block table had no committed command; error variants reused under misleading
names (now `SoftLength`, `NotANumber`, `DecoderConfig`); "two minima and the position"
left out the sign bits; timings in this entry (removed). Not done, by its suggestion: an
outside reference for the layered and the fixed decoder (open question 5), and the
founder's word on the hardware algorithm's definition, which goes to them with the G2
widths as one question.

**Evidence:** `cargo test -p radio-phy --test g1_ldpc_decoder` — 9 pass;
`scripts/check.sh` green; this entry's pull request.

**Open questions** (decoder)
1. G2: the input scale (units per LLR, what the demapper delivers), message and total
   widths, the normalisation factor, the iteration count; loss in dB against float
   sum-product at a block error rate of 10 % and 1 %, on the codes and lengths the TDMA
   profile will use. The widths are the founder's to approve, as the Viterbi's were.
2. No fading or real demapper yet: the LLRs here are exact for BPSK in AWGN. The decoder
   meets the receiver (R0-09) when the HT mapping exists or the TDMA profile is defined.
3. The stop rule tests every check after each pass. Hardware will want it cheaper (the
   checks of the last layers, or the syndrome computed on the fly): equivalent or not is to
   be decided with the RTL.
4. An error floor cannot be seen at these block counts (the lowest point has 21 wrong
   blocks of 5000).
5. The layered and the fixed decoder have no outside reference. AFF3CT (MIT licence) has a
   layered normalised min-sum decoder that could be run as a black box on these matrices.
6. The accumulator that combines repeated bits in hardware (its width, where it saturates)
   and the rounding of the scaled minimum: both to G2, and with the widths to the founder.

## 2026-11-19 · G0 · G2 budget (decoder widths)

- **What is chosen:** the integer decoder's input format (width b and clip C: soft value =
  round(y·in_max/C) saturated to ±in_max = ±(2^(b−1) − 1), y the matched-filter output
  with signal amplitude 1, the Viterbi's convention), the message and total widths, the
  normalisation factor with its rounding, and the iteration limit.
- **Two losses, kept apart.** (a) The algorithm: layered normalised min-sum at a given
  iteration limit against sum-product with 50 flooding iterations. That is a choice of
  throughput and area, not of arithmetic; it is reported, and the iteration limit is the
  founder's to set. (b) The arithmetic: the integer decoder against the float layered
  min-sum at its best factor and the same iteration limit.
- **Budget:** (b) ≤ 0.1 dB of Eb/N0 at a block error rate of 10 %, on BPSK/AWGN, on the
  worst of five codes: (648, 1/2), (1944, 1/2), (1944, 3/4), (648, 5/6), (1944, 5/6) (both
  ends of length and rate; rate 1/2 is the hardest for min-sum). Rule: the narrowest input,
  then the narrowest messages, then the narrowest totals that meet it; the factor is the
  best one that is a shift-and-add.
- **Method:** `g2 ldpc [blocks]`: every configuration decodes the same noisy blocks; the
  10 % crossing by log-linear interpolation on a 0.25 dB grid; the 1 % crossing is printed
  too, as a check that nothing bends lower down (an error floor would need far more
  blocks). One parameter at a time around the choice, so the choice is revised and the
  sweep rerun until the centre is the choice.
- The clip is relative to the signal amplitude, as the Viterbi's: it assumes soft values
  normalised by the channel gain, which R0-09's demapper owns; in fading the weights
  change the optimum, and this sweep is rerun through the receiver when the HT or TDMA
  mapping exists.

## 2026-11-19 · G2 · decoder widths

Command: `cargo run --release -p radio-tools --bin g2 -- ldpc 3000` (3000 blocks per point,
five codes × 8 points of Eb/N0 × 76 configurations, the same noise for all). Full output:
`docs/gates/reports/g2_ldpc.txt`. Three runs: the first around a first guess (×3/4, 15
iterations, 6-bit messages, 8-bit totals), the second around ×7/8 and 20 iterations, the
third, committed, around the choice; rows common to the runs are identical (seeded).

**The choice, approved by the founder on 2026-11-19** (`MinSumConfig::HARDWARE`,
`LDPC_SOFT_BITS`, `LDPC_SOFT_CLIP`): **input 6 bits at clip 2** (the receiver's working soft
format), **messages 7 bits, totals 9 bits, factor ⌊7·min/8⌋, at most 20 iterations.**

Loss in dB at a block error rate of 10 % (and of 1 %) against sum-product, 50 iterations:

| Configuration | (648, 1/2) | (1944, 1/2) | (1944, 3/4) | (648, 5/6) | (1944, 5/6) |
|---|---|---|---|---|---|
| float layered min-sum ×13/16, 50 it | 0.07 (0.02) | 0.06 (0.03) | 0.03 (−0.01) | −0.04 (−0.03) | 0.00 (−0.04) |
| the same, 20 it | 0.14 (0.07) | 0.17 (0.10) | 0.07 (0.04) | 0.01 (0.02) | 0.04 (0.00) |
| **the choice** | **0.14 (0.06)** | **0.17 (0.10)** | **0.08 (0.06)** | **0.02 (0.01)** | **0.06 (0.04)** |
| messages 6, totals 8 | 0.15 (0.11) | 0.20 (0.20) | 0.08 (0.06) | 0.02 (0.01) | 0.06 (0.04) |
| messages 7, totals 8 | 0.15 (0.09) | 0.18 (0.13) | 0.08 (0.06) | 0.02 (0.01) | 0.06 (0.04) |
| messages 10, totals 12 | 0.14 (0.06) | 0.17 (0.10) | 0.08 (0.06) | 0.02 (0.01) | 0.06 (0.04) |
| input 5 bits, clip 1.5 (wide messages) | 0.17 (0.09) | 0.20 (0.11) | 0.09 (0.06) | 0.06 (0.07) | 0.10 (0.08) |
| input 5 bits, clip 2 | 0.13 (0.07) | 0.17 (0.11) | 0.13 (0.09) | 0.16 (0.14) | 0.20 (0.18) |
| input 4 bits, clip 1.5 | 0.28 (0.20) | 0.31 (0.24) | 0.53 (0.47) | 0.76 | 0.79 (0.77) |
| factor 13/16 | 0.16 (0.10) | 0.19 (0.16) | 0.06 (0.04) | 0.02 (0.02) | 0.06 (0.05) |
| factor 3/4 | 0.33 (0.32) | 0.44 (0.52) | 0.09 (0.05) | 0.03 (0.04) | 0.08 (0.07) |
| factor 7/8 rounded to nearest | 0.24 (0.11) | 0.26 (0.17) | 0.16 (0.14) | 0.08 (0.08) | 0.11 (0.08) |
| 10 iterations | 0.36 (0.25) | 0.47 (0.41) | 0.18 (0.20) | 0.08 (0.07) | 0.13 (0.11) |
| 15 iterations | 0.20 (0.12) | 0.26 (0.18) | 0.11 (0.08) | 0.04 (0.04) | 0.08 (0.06) |
| 30 iterations | 0.09 (0.02) | 0.12 | 0.06 | 0.00 (−0.01) | 0.04 (0.03) |
| 50 iterations | 0.06 (0.00) | 0.08 (0.00) | 0.03 (0.01) | −0.02 (−0.03) | 0.02 (0.01) |

- **The budget** (arithmetic ≤ 0.1 dB at 10 % against float layered min-sum at the same 20
  iterations): the choice is at −0.00, +0.00, +0.01, +0.00 and +0.02 dB. It is the wide
  decoder: messages of 10 bits and totals of 12 give the same block error rate at every
  point of every code.
- **Why not one bit less, which also meets the budget.** Messages of 6 bits and totals of 8
  lose 0.00 to 0.03 dB at 10 %, but on (1944, 1/2) they leave a tail below 1 %: 15, 6 and 3
  wrong blocks of 3000 at 1.75, 2.0 and 2.25 dB, where the choice has 3, 0 and 0 on the
  same blocks (0.10 dB at 1 %). A 6-bit message cannot say more than a 6-bit input does.
  I recommended 6/8 from the 10 % column first, the founder approved it, the 1 % column of
  the next run showed the tail, and the question went back: 7/9 approved. The budget's
  definition at 10 % alone would have let it through; the 1 % column is in the method for
  this.
- **Narrower still does not work at a 6-bit input:** 5-bit messages or 7-bit totals do not
  reach 10 % on the rate-1/2 codes inside the grid.
- **Input.** 6 bits are flat against the clip (1.5 to 2.5: within 0.05 dB). 5 bits meet the
  budget at clip 1.5 only (0.01 to 0.06 dB) and lose 0.14 to 0.16 dB at the rate-5/6 codes at
  clip 2: the scale would have to be held to a quarter, which fading will not allow. 4 bits
  do not meet it.
- **Factor.** 7/8 rounded down is best at rate 1/2 and within 0.02 dB of the best elsewhere.
  Rounded down it behaves like the float decoder's 13/16 (at about 5.6 units per unit of
  LLR the lost fraction is that much). Rounding to nearest was tried on the G1 finding
  (`scale_round`): it shifts the effective factor up and helps only where the factor was too
  small (3/4); at the right factor it is not needed. The option stays in the model for the
  report's rows and is off in the choice. In hardware ⌊7x/8⌋ is `(8x − x) >> 3`.
- **Iterations** are throughput, not arithmetic: the founder chose 20 (0.17 dB from
  sum-product on the worst code; 15 would cost 0.08 dB more at rate 1/2, 30 would gain
  0.05). The mean at a block error rate near 10 % is 2 to 5 iterations; the limit sets the
  worst case, and ADR 0008's latency budget may revise it.
- Noise of the estimate: rows that differ only in unused width are identical; 10 % crossings
  of rows that should agree scatter by about ±0.02 dB and the 1 % column, on about 30 wrong
  blocks per point, by about ±0.05 dB (my estimate from the table, not a computed interval).

**Gate in CI:** `tests/g2_ldpc.rs`. The hardware decoder on its 6-bit input against float
layered min-sum ×13/16 on exact LLRs, 20 iterations, the same blocks, at three points
((648, 1/2) 1.5 dB, (1944, 1/2) 1.25 dB, (648, 5/6) 3.5 dB): blocks only the hardware decoder
loses 4, 3, 12; only the float one 10, 2, 6; allowed 10 more. A second test shows the gate
sees what the sweep rejects at 10 %: 5-bit messages (154 against 2), 6-bit messages with
7-bit totals (206 against 0), a 4-bit input (75 against 1), factor 3/4 (31 against 7). It
does not see 6/8 against 7/9: that difference lives below 1 %, in the sweep.

**Mutations:** the rounding term ignored → `fixed_decoder_is_its_written_definition`;
`HARDWARE` at factor 3/4 → the G2 gate; `LDPC_SOFT_BITS` 4 → the G2 gate.

**Evidence:** the report above (commit 6808132); `cargo test -p radio-phy --test g2_ldpc` —
2 pass; `scripts/check.sh` green; this entry's pull request.

**Open questions** (G2)
1. Everything here is BPSK in AWGN with exact scaling. Through the receiver (fading, the
   demapper's weights, higher-order mappings) the clip and perhaps the input width are
   measured again, when the HT or TDMA mapping exists.
2. The tail of narrow messages suggests looking lower: an error-floor run (10⁵ blocks and
   more on (1944, 1/2) and a rate-5/6 code) on the build server, before RTL.
3. The repetition accumulator (decoder open question 6) is still defined as the exact sum.
4. Early termination by the full parity check costs a pass of logic per iteration; cheaper
   stop rules are an RTL question with their own loss to measure here.
