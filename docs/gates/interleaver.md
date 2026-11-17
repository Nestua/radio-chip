# Gate journal: interleaver

Block: 802.11a/g OFDM block interleaver, two permutations per OFDM symbol
(IEEE 802.11-2020 §17.3.5.7). Model: `crates/radio-phy/src/interleaver.rs`. RTL: not yet (R2).

## 2026-11-17 · G0 · first commit
- Interface: `interleave(bits, modulation)` / `deinterleave(bits, modulation)` over whole OFDM
  symbols of N_CBPS = 48 × N_BPSC bits (N_BPSC = 1, 2, 4, 6); a partial symbol is rejected.
  `interleaved_index(k, modulation)` gives the position of coded bit k after both permutations.
- First permutation: i = (N_CBPS/16)(k mod 16) + ⌊k/16⌋. Second: j = s⌊i/s⌋ +
  (i + N_CBPS − ⌊16i/N_CBPS⌋) mod s, s = max(N_BPSC/2, 1). Written from memory; agrees with the
  black-box reference below for every rate.

## 2026-11-17 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_interleaver`
References:
1. OpenOFDM black box (github.com/jhshi/openofdm, Apache-2.0, commit ea638eb), the receiver's
   `Decoder.deinterleave` in `scripts/decode.py`, run unmodified under Python 2.7.18 in
   `python:2.7-slim` with no network: `tools/ref/interleaver_openofdm.py` →
   `crates/radio-phy/tests/fixtures/interleaver_openofdm.txt`. 16 vectors, all eight rates
   (6–54 Mb/s), three OFDM symbols each; our interleave and deinterleave must reproduce both
   directions. OpenOFDM writes the inverse in its own (receiver) form, so this is an independent
   implementation, not a copy of ours.
2. The design properties stated in §17.3.5.7: adjacent coded bits on non-adjacent subcarriers;
   adjacent coded bits alternate between more and less significant constellation bits (checked as:
   each row of 16 coded bits visits every significance class).
Result: 6/6 pass (also: the index map is a permutation, multi-symbol round trip, partial symbol
rejected). Test-first check: all 6 failed against the `todo!()` stub.
Mutation check (each restored afterwards):
| Mutation | Result |
|---|---|
| 16 columns → 12 | 1 test fails |
| Second permutation removed | 2 tests fail |
| Rotation sign flipped | 1 test fails |
| s = N_BPSC instead of N_BPSC/2 | 1 test fails |

Harness notes: decode.py imports numpy, wltrace and CommPy for code the deinterleaver does not
use; the harness registers empty modules for them. Docker Desktop cannot mount the agent's
temporary folder, so the OpenOFDM checkout lives in the ignored `target/ext/`.

## 2026-11-17 · G1 · the standard's worked example
Command: `cargo test -p radio-phy --test g1_annex_i1 -- --ignored` (needs the local fixture; see
`docs/gates/ppdu.md`, entry "the standard's worked example").
Reference: IEEE 802.11-2024 Annex I.1, Tables I-9 (SIGNAL, BPSK), I-17 and I-18 (both
permutations, 16-QAM, all 192 k) and I-19 (first DATA symbol).
Result: our `interleaved_index` equals I-18∘I-17 for every k; I-9 and I-19 match bit for bit.
Clause §17.3.5.7 unchanged in the 2024 text.

## Open questions
1. HT (802.11n) uses 13 columns and, for several streams, frequency rotation; OpenOFDM covers
   single-stream HT too and can extend this fixture when the HT profile starts.
