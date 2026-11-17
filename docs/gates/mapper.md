# Gate journal: subcarrier mapper

Block: 802.11a/g subcarrier modulation mapping, BPSK / QPSK / 16-QAM / 64-QAM, Gray coded
(IEEE 802.11-2020 §17.3.5.8). Model: `crates/radio-phy/src/mapper.rs`. RTL: not yet (R2).

## 2026-11-17 · G0 · first commit
- Interface: `map_group(bits, modulation) -> Point` for one group of N_BPSC bits (b0 first);
  `map_bits` over a stream, rejecting a partial group.
- Number format: points are on the **unnormalised odd-integer grid** (`i8`, ±1…±7), which is exact
  and is what hardware holds before scaling. The normalisation K_MOD = 1/√E is not applied here:
  `mean_energy` gives E = 1, 2, 10, 42 as an exact integer, and the fixed-point scaling is decided
  with the IFFT (R0-05) and the fixed-point framework (R0-11).
- Labels: b0…b(N/2−1) select I, the rest select Q; BPSK uses I only, Q = 0.
- `Modulation` moved from `interleaver` to its own module `modulation` (re-exported from
  `interleaver`, so existing paths still work).

## 2026-11-17 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_mapper`
References:
1. OpenOFDM black box (github.com/jhshi/openofdm, Apache-2.0, commit ea638eb): the receiver's
   `Demodulator.demodulate` in `scripts/decode.py`, run unmodified on every grid point of every
   constellation (2 + 4 + 16 + 64 points) under Python 2.7.18 + numpy 1.16.6 in
   `python:2.7-slim`, `--platform linux/amd64` (numpy has no aarch64 wheel for Python 2.7):
   `tools/ref/mapper_openofdm.py` → `crates/radio-phy/tests/fixtures/mapper_openofdm.txt`.
2. Analytic properties: every label on a distinct odd grid point; nearest neighbours (distance 2)
   differ in exactly one bit (Gray); mean |point|² equals E = 1/K_MOD² = 1, 2, 10, 42.
Result: 5/5 pass. Test-first check: all 5 failed against the `todo!()` stub.
Mutation check (each restored afterwards):
| Mutation | Result |
|---|---|
| 16-QAM natural binary instead of Gray | 2 tests fail |
| 64-QAM two Gray entries swapped | 2 tests fail |
| I and Q halves swapped | 1 test fails |
| BPSK polarity flipped | 1 test fails |
| 64-QAM energy 42 → 40 | 1 test fails |

## Open questions
1. Check against the mapped DATA symbols of the standard's annex example once the standard is at
   hand (it prints normalised values, which also pins K_MOD in context).
2. Fixed-point format of the normalised points (word length, rounding of 1/√10 and 1/√42) is a
   G2 question for R0-05/R0-11.
