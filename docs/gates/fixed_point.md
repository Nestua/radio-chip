# Gate journal: fixed-point framework

Block: signed Q formats, rounding and overflow modes, G2 metrics (SQNR, EVM) and the word-length
sweep. Model: `crates/radio-phy/src/fixed.rs`. Used by every DSP block from R0-05 on.

## 2026-11-17 · G0 · first commit
- A value is a raw `i64` plus `QFormat { W, F }`: real = raw · 2^-F, range [-2^(W-1), 2^(W-1)-1].
  W ≤ 48, F ≤ 60, so intermediates fit `i128` and doubles stay exact.
- Rounding: `Floor` (hardware truncation), `HalfUp` (+½ LSB then floor; ties toward +∞),
  `HalfEven` (convergent). Overflow: `Saturate`, `Wrap` (low W bits, sign-extended).
- `requantize(raw, from_frac, to, …)` is the narrowing step models call where the RTL narrows a
  word; it uses shifts and masks, as hardware does. `quantize(f64, …)` converts real reference
  values (twiddles, float-model outputs) and rejects NaN/∞ and |x·2^F| ≥ 2^52.
- G2 metrics: `sqnr_db`, `evm_percent` (RMS, percent of reference RMS), `sweep` over word lengths.

## 2026-11-17 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_fixed`
References:
1. fxpmath 0.4.10 black box (MIT; numpy 2.5.5, Python 3.14.8): `tools/ref/fixedpoint_fxpmath.py`
   → `crates/radio-phy/tests/fixtures/fixedpoint_fxpmath.txt`, 2160 quantisations over nine Q
   formats (incl. F = 0 and F > W), three roundings, two overflow modes, dense in exact ties.
   Probed mapping: floor → `floor`, half-even → `around`; fxpmath's `trunc` is toward zero and
   is not used; half-up = fxpmath `floor` of x + ½ LSB (exact, values are dyadic).
2. Textbook SQNR of a full-scale sine, 6.02·W + 1.76 dB, W = 8…16, within 0.2 dB.
3. Exact-integer oracle for `requantize` (i128 Euclidean division, a different formulation from
   the shift/mask implementation), 20 000 random cases × 6 modes; error bounds (floor: −LSB < e
   ≤ 0, half modes: |e| ≤ ½ LSB) over 120 000 values; metrics on known errors.
Result: 6/6 pass. Test-first check: all 6 failed against the `todo!()` stub.
Finding while writing the gate: a sine of amplitude exactly 1.0 in Q(W, W−1) clips at the
positive peak (full scale is 1 − LSB) and measured 49.12 dB at W = 8 instead of 49.92; numpy gave
the same 49.12, so the test setup was wrong, not the code. The gate now uses amplitude 1 − LSB.
Mutation check (each restored with `cp`):
| Mutation | Result |
|---|---|
| `quantize` HalfUp acts as floor | 2 tests fail |
| `requantize` HalfEven ties always up | 1 test fails |
| Wrap without sign extension | 2 tests fail |
| `requantize` Floor rounds half up | 1 test fails |
| `max_raw` off by one | 3 tests fail |
| SQNR with 20·log10 instead of 10·log10 | 2 tests fail |

## Open questions
1. Complex fixed-point helpers (complex multiply with a chosen rounding point) arrive with the
   IFFT (R0-05), where the first real datapath decides them.
