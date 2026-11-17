# Gate journal: convolutional encoder

Block: 802.11 OFDM convolutional encoder, K = 7, generators (133, 171)₈, puncturing to 2/3 and 3/4
(IEEE 802.11-2020 §17.3.5.6, Figures 17-8 and 17-9).
Model: `crates/radio-phy/src/convolutional.rs`. RTL: not yet (R2).

## 2026-11-17 · G0 · first commit
- Interface: `ConvolutionalEncoder::encode_bit(bit) -> (a, b)` from the all-zero state; serial
  output A0 B0 A1 B1 …; `puncture(coded, rate)` steals bits from that stream and rejects a stream
  that does not end on a whole puncturing period (802.11 always gives whole periods: every N_DBPS
  is a multiple of 6).
- Generator bit order: MSB taps the current input. Evidence: Figure 17-8 as reproduced in
  UBC ELEX 7860 (2022) midterm 1 solutions, Q1, where output B is "the exclusive-or of the input,
  the first three bits and the last bit in the shift register" — taps 0, 1, 2, 3, 6 = 171₈ MSB first.
- Puncturing patterns over A0 B0 A1 B1 …: 2/3 = 1110, 3/4 = 111001. Written from memory of
  Figure 17-9; **not yet confirmed against the standard text** (open question 1).

## 2026-11-17 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_convolutional_encoder`
References:
1. Code definition: impulse responses equal the generator bits, A = 1011011, B = 1111001.
2. UBC worked example on Figure 17-8: input 1, 0, 1 → B = 1, 1, 0.
3. CommPy 0.8.0 black box (BSD-3-Clause, numpy 2.5.5, Python 3.14.8):
   `tools/ref/convenc_commpy.py` → `crates/radio-phy/tests/fixtures/convolutional_commpy.txt`,
   48 vectors (16 per rate, inputs of every 802.11a N_DBPS from 24 to 216 bits).
   CommPy reads octal generators with the current input in the LSB: probing it with (133, 171)
   gave A = 1101101, B = 1001111 (the mirror code), so it is run with (155, 117)₈, which then
   reproduces references 1 and 2.
Result: 6/6 pass (also: GF(2) linearity, output lengths per rate, partial period rejected).
Test-first check: all 6 tests failed against the `todo!()` stub before the implementation.
Mutation check (each restored afterwards):
| Mutation | Result |
|---|---|
| G0 mirrored, 133 → 155 | 2 tests fail |
| G1 tap added, 171 → 173 | 2 tests fail |
| A and B outputs swapped | 3 tests fail |
| 3/4 pattern 111001 → 110110 | 1 test fails |
| 2/3 pattern 1110 → 1101 | 1 test fails |

The pattern mutations are caught only because CommPy was given the same patterns; that part of G1
is consistency, not independence, until open question 1 is closed.

## 2026-11-17 · G1 · puncturing confirmed by a receiver
OpenOFDM's receiver depunctures by inserting erasures at exactly our stolen positions (3/4:
111001, 2/3: 1110, read from its `viterbi_decode`), and its full decode of our frames returns
the PSDU octet-exact at 9, 18, 36, 48 and 54 Mb/s (`docs/gates/ppdu.md`). The patterns are no
longer resting on memory alone.

## Open questions
1. Confirm the clause numbers and check against the encoded DATA bits of the standard's annex
   example once the text is at hand.
2. G3 (RTL) belongs to R2. Puncturing makes the output rate uneven, so the RTL needs a
   valid/ready output or a documented fixed duty cycle.
