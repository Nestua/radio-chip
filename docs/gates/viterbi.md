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

## 2026-11-17 · G0 · G2 budget
- **What is chosen**: the soft-input width b and clip level C (soft value = round(y/Δ) saturated
  to ±(2^(b−1) − 1), Δ = C/(2^(b−1) − 1), y the matched-filter output with signal amplitude 1),
  the traceback window D of the hardware variant (`decode_windowed`), and the path-metric width.
- **Budget**: Eb/N0 loss at BER 1e-4 on BPSK/AWGN, against full traceback on 2^-12 soft values,
  at every rate: soft quantisation ≤ 0.15 dB, traceback window ≤ 0.05 dB (≤ 0.2 dB together).
  Rule: the smallest b (then D) that meets it at the worst rate.
- **Method**: `g2 viterbi [blocks]`; every configuration decodes the *same* noisy blocks, so the
  differences are paired. BER 1e-4 crossing by log-linear interpolation on a 0.5 dB grid.
- **Metric width**: spread ≤ 24·(2^(b−1) − 1) (proved in `metric_spread`'s doc); a modulo
  metric register needs 2^w > 2·spread. The sweep also reports the largest spread observed.
- The clip level is relative to the signal amplitude, so it assumes the demapper (R0-09) hands
  over soft values already normalised by the channel gain; R0-09 owns that scaling.

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

## 2026-11-17 · G2 · first commit
Command: `cargo run -q --release -p radio-tools --bin g2 -- viterbi 30000` (30 000 blocks ×
1002 bits = 3·10⁷ bits per point, 3 rates × 5 Eb/N0 × 27 configurations, same noise for all;
~15 min on the build server's 32 threads, docker `rust:1-slim`, rustc 1.100.0). Full output:
`docs/gates/reports/g2_viterbi.txt`. Two runs gave identical rows (deterministic seeds); a first
3000-block run was too thin near 1e-4 (losses wandered by ±0.1 dB, some negative) and only
served to pick the grid.
Reference (2^-12 soft, full traceback) reaches BER 1e-4 at 3.45 / 3.91 / 4.42 dB for rates
1/2, 2/3, 3/4. Loss in dB at BER 1e-4 (best clip per width shown; full grid in the report):

| Configuration | 1/2 | 2/3 | 3/4 | worst |
|---|---|---|---|---|
| 3 bit, clip 1.0 | 0.248 | 0.222 | 0.229 | 0.248 ✗ |
| **4 bit, clip 1.5** | 0.057 | 0.080 | 0.088 | **0.088** |
| 4 bit, clip 1.0 | 0.148 | 0.093 | 0.062 | 0.148 |
| 5 bit, clip 2.0 | 0.022 | 0.015 | 0.007 | 0.022 |
| 6 bit, clip 2.0 | 0.006 | 0.003 | −0.004 | 0.006 |
| window 32 | 0.080 | 0.357 | 0.573 | 0.573 ✗ |
| window 48 | 0.003 | 0.064 | 0.141 | 0.141 ✗ |
| **window 64** | 0.000 | 0.007 | 0.024 | **0.024** |
| window 96 | 0.000 | 0.000 | 0.000 | 0.000 |
| **4 bit / 1.5 + window 64** | 0.057 | 0.086 | 0.109 | **0.109** |

Decision by the G0 rule: **SOFT_BITS = 4, SOFT_CLIP = 1.5, TRACEBACK_DEPTH = 64**
(≈ 9·K; rate 3/4 needs it — 48 is enough at 1/2 only), together 0.109 dB worst ≤ 0.2 dB.
**Approved by the founder on 2026-11-17.** Alternative considered and declined: 5 bit / clip 2.0
(≈ 0.03 dB with window 64, and much flatter against the clip level — 0.007…0.048 dB over clip
1.5…2.5 versus 0.057…0.223 dB at 4 bits — at +25 % soft storage and a 10-bit metric). Context:
3-bit soft at 0.22–0.25 dB matches the classic Heller & Jacobs (1971) figure for 8-level
quantisation, an outside sanity check of the sweep.
Path metrics: spread ≤ 24·7 = 168 (observed ≤ 118 at 4 bits) → **METRIC_BITS = 9** for modulo
comparison (2^9 = 512 > 336). Noise-of-the-estimate: the 6-bit rows scatter by ±0.005 dB
around zero, so differences above ~0.02 dB are real.
New G1 checks for the model variants: `decode_windowed` equals `decode` when the window covers
the block; each windowed decision equals a free-ended full Viterbi over the first i + D steps
(the definition); the metric spread respects its bound. Mutations: window decision one step
early (caught only by the prefix-definition test — added for that reason), flush too short,
spread overstated — all caught.

## Open questions
1. The soft scaling contract (SOFT_CLIP = 1.5 × signal amplitude) assumes CSI-normalised soft
   values; R0-09's demapper must deliver that, and in fading the per-subcarrier weighting
   (|H|²) changes the optimum — re-run this sweep through the full receiver in R0-09. At 4 bits
   the loss is sensitive to that scaling (0.06 → 0.22 dB from clip 1.5 to 2.5): if R0-09 shows
   a real receiver cannot hold the scale, revisit 5 bits (the declined alternative above).
2. Loss measured at BER 1e-4 on AWGN; 802.11's sensitivity point is PER 10 % at 1000 octets
   (BER ≈ 1e-5). Re-check at 1e-5 when the full receiver exists.
3. RTL (R2): register-exchange vs traceback memory is an area choice; either must reproduce
   `decode_windowed` bit-exactly at D = 64.
