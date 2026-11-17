# Gate journal: Viterbi decoder

Block: soft-decision Viterbi decoder for the 802.11 K = 7, (133, 171)₈ code with the rate-2/3 and
3/4 puncturing of IEEE 802.11-2024 §17.3.5.6 (R0-10). Model: `crates/radio-phy/src/viterbi.rs`.
Mirror of `convolutional.rs` (R0-02).

## 2026-11-17 · G0 · first commit
- **Input**: one signed soft value (`i32`) per *transmitted* coded bit, in transmission order.
  Positive means "0 more likely" (the sign of log P(0)/P(1)); 0 is an erasure. Magnitude is the
  confidence; the fixed-point width and scaling of the soft values is the G2 question (next PR).
- **Depuncturing**: `depuncture` re-inserts a 0 (erasure) at every stolen position of the rate's
  pattern, giving the rate-1/2 stream A0 B0 A1 B1 …
- **Trellis**: 64 states, the encoder's six delay cells exactly as in `ConvolutionalEncoder`
  (bit 5 = most recent input). Input bit u moves state s to (u << 5) | (s >> 1).
- **Branch metric**: correlation Σ sᵢ·(1 − 2cᵢ) over the pair (A, B), maximised (the integer form
  of minimum Euclidean distance for antipodal signalling).
- **Path metrics**: start in state 0 (metric 0, all others unreachable). The model keeps `i64`
  metrics without normalisation; the RTL may renormalise or use modulo arithmetic, which
  preserves every comparison as long as its width covers the metric spread — checked at G3.
- **Ties**: equal candidate metrics keep the predecessor whose dropped (oldest) bit is 0.
  Equal end metrics (free termination) pick the lowest state index.
- **Traceback**: whole block (the golden model is maximum likelihood). A hardware decoder with a
  sliding traceback window will be a separate model variant with its own G2 loss.
- **Termination**: `Termination::ZeroAfter(n)` — the path is forced through state 0 after n
  input bits. In 802.11 the receiver knows LENGTH from SIGNAL, so n = 16 + 8·LENGTH + 6 (end of
  the tail); pad bits after it follow the best end state (they are discarded anyway).
  `Termination::Free` traces back from the best end state.

## 2026-11-17 · G1 · first commit
Command: `cargo test -p radio-phy --test g1_viterbi -- --nocapture` (3.6 s; `[profile.test]`
now builds at opt-level 2 — the Monte-Carlo gates took ~40 s unoptimised)
References:
1. **Maximum likelihood by definition**: 12-bit blocks, all 4096 codewords scored by brute force,
   σ = 1.4 so ML itself is often wrong. 1200 blocks (3 rates × terminated/free × 200): the
   decoder returned the brute-force winner every time, including the 90–179 blocks per case where
   ML ≠ the sent message. (No exact ties occurred at 2^-12 soft resolution.)
2. **Published free distances** (10, 6, 5 for rates 1/2, 2/3, 3/4): 1500 random patterns of
   ⌊(d−1)/2⌋ = 4, 2, 2 hard errors per rate, all corrected; the rate-1/2 impulse codeword has
   weight 10.
3. **CommPy 0.8.0 black box** (BSD-3-Clause; numpy 2.5.5, Python 3.14.10, docker
   `python:3.14-slim` on the build server, 60 chunks in parallel; `tools/ref/viterbi_commpy.py`
   → `tests/fixtures/viterbi_ber_commpy.txt`): 1000 blocks × 1002 bits per point, unquantised
   soft decisions, traceback 120. Ours (own noise, same setup, full traceback):

   | Rate | Eb/N0 | ours | CommPy | z |
   |---|---|---|---|---|
   | 1/2 | 1.5 dB | 1.534e-2 ± 4.8e-4 | 1.520e-2 ± 4.7e-4 | +0.21 |
   | 1/2 | 2.5 dB | 1.446e-3 ± 1.1e-4 | 1.620e-3 ± 1.3e-4 | −1.04 |
   | 2/3 | 2.5 dB | 6.080e-3 ± 3.3e-4 | 7.053e-3 ± 3.7e-4 | −1.98 |
   | 2/3 | 3.5 dB | 3.463e-4 ± 6.6e-5 | 3.204e-4 ± 5.8e-5 | +0.30 |
   | 3/4 | 3.0 dB | 6.564e-3 ± 3.9e-4 | 6.594e-3 ± 3.6e-4 | −0.06 |
   | 3/4 | 4.0 dB | 3.453e-4 ± 6.5e-5 | 3.932e-4 ± 7.0e-5 | −0.50 |

   (A first run with 150 blocks per point was too thin — two points at |z| ≈ 2.5 with a handful
   of error events — and was replaced before commit.)
4. **OpenOFDM's frames** (R0-06 fixture): all 24 decode noise-free with `ZeroAfter(tail end)`,
   including the 9 Mb/s one-octet frame OpenOFDM got wrong. Forcing state 0 at the *end of the
   frame* (OpenOFDM's assumption) returns exactly its wrong octet, 0x28 — the R0-06 diagnosis is
   confirmed. That reproduction also depends on the tie rule (see the mutation table): OpenOFDM's
   decoder resolves that tie the same way ours does.
Also: depuncturing positions, the tie rule on all-erasure input (all zeros, both terminations).
Result: 9/9 pass. Test-first: all decoder tests failed against the `todo!()` stub (the CommPy test
passed vacuously on an empty fixture, so it now asserts six points).
Mutation check (each restored with `cp` + `touch`):
| Mutation | Fails |
|---|---|
| Tie rule flipped (`>=`) | ties, OpenOFDM octet |
| A/B outputs swapped | 7 tests |
| `ZeroAfter` ignored | free distance, brute-force ML, OpenOFDM octet |
| Metric minimised instead of maximised | 7 tests |
| Depuncturing inverted | 8 tests |
| Predecessor wiring wrong | 7 tests |

## Open questions
1. G2: soft-value width (3–6 bits) and scaling from the demapper, path-metric width — next PR.
2. Hardware traceback: a sliding window (depth ~5–8 K, longer for 3/4) will differ from this
   full-traceback model at low SNR; model it as its own variant with a G2 loss before the RTL.
