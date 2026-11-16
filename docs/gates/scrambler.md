# Gate journal: scrambler

Block: 802.11 OFDM data scrambler, S(x) = x^7 + x^4 + 1 (IEEE 802.11-2020 §17.3.5.5).
Model: `crates/radio-phy/src/scrambler.rs`. RTL: `hw/rtl/scrambler.sv`.

## 2026-11-16 · G3 · first commit
Command: `make -C hw scrambler`
Result: 127 vectors (every non-zero seed), 32 512 bits, 0 mismatches. Verilator 5.054, `-Wall` clean.
Mutation check: tap changed from x7^x4 to x7^x5 in the RTL → 16 384 mismatches, gate failed as expected; restored.

## 2026-11-16 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_scrambler_standard`
Reference: the 127-bit sequence for the all-ones initial state printed in §17.3.5.5.
Result: 5/5 tests pass — sequence matches, period 127 for all seeds, scramble∘scramble = identity, invalid seeds rejected.
Test-first check: all four behavioural tests failed against the `todo!()` stub before the implementation.
