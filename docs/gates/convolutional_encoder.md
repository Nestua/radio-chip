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

## 2026-11-17 · G1 · the standard's worked example
Command: `cargo test -p radio-phy --test g1_annex_i1 -- --ignored` (needs the local fixture; see
`docs/gates/ppdu.md`, entry "the standard's worked example").
Reference: IEEE 802.11-2024 Annex I.1, Tables I-8 (SIGNAL, rate 1/2) and I-16 (DATA, rate 3/4).
Result: all 48 SIGNAL and all 1152 DATA coded bits match — the 3/4 puncturing pattern now rests
on the standard itself. Clause numbers (§17.3.5.6, Figure 17-9) unchanged in the 2024 text.
Mutation: 3/4 pattern 111001 → 110110 fails I-16 and the whole-frame samples.

## Open questions
1. G3 (RTL) belongs to R2. Puncturing makes the output rate uneven, so the RTL needs a
   valid/ready output or a documented fixed duty cycle.

## 2026-11-19 · G3 · RTL (for the silicon probe, R0-14a)

**Block:** `hw/rtl/conv_encoder.sv`, the rate-1/2 encoder (golden:
`ConvolutionalEncoder::encode_bit`). One bit per `in_valid`, the pair (A, B) one clock edge
later; `clear` returns the state to zero for the next frame. Puncturing is not in the block:
it steals output bits and belongs to the consumer.

**Vectors** (`rvec conv_encoder`): 65 frames, each from state 0: all zeros, all ones, a single
one (the impulse response, which is the generators), alternating bits, one bit, and 60 random
frames of 1 to 600 bits.

**Evidence** (`make -C hw conv_encoder`): 17 941 bits, 0 mismatches.
- An idle cycle after every 3rd bit and two after every 7th, with the inverted bit on the
  idle input; each pair exactly one edge after its input; no output on an idle cycle.
- A reset instead of `clear` before every third frame, with a bit in flight that must not come
  out.
- A frame ending in ones fed twice without `clear`: the second pass must differ from the run
  from state 0 (the first form of this check used the last random frame, which happens to
  end in six zeros, and failed for that reason).
- The stub fails with 13 330 mismatches.

**Mutations** (each built from scratch)
| Mutation | Mismatches |
|---|---|
| A reads D4 instead of D5 | 8788 |
| B without D1 | 8949 |
| A and B swapped | 8845 |
| shift direction | 13 337 |
| `clear` ignored | 177 |
| state moves on idle cycles | 13 318 |
| valid not cleared by reset | 17 |
| state not cleared by reset or `clear` | 266 |
