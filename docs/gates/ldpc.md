# Gate journal: HT LDPC codes (R0-12)

Block: the twelve LDPC codes of 802.11n, their encoder, the PPDU encoding process, and (next)
the decoder. Model: `crates/radio-phy/src/ldpc.rs`, tables `ldpc_tables.rs`.

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
