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

## 2026-11-19 · note · open question 1 answered by R0-09: the soft format does not hold in fading

The whole receiver (`docs/gates/rx_ofdm.md`, 2026-11-19, measurements) loses 0.8 to 3.0 dB at
9, 18 and 36 Mb/s in TGn B and D with SOFT_BITS = 4, SOFT_CLIP = 1.5, against fine soft values,
and 0.1 to 0.4 dB in AWGN. Six bits with a clip of 2 is the first format within about
0.25 dB in AWGN and in both channels (a ranking on 200 to 300 frames per point, not a G2
figure). SOFT_BITS, SOFT_CLIP and METRIC_BITS are unchanged: the choice was the founder's and
its revision is too, on a paired G2 run of the whole receiver.

## 2026-11-19 · decision · the founder: six bits, clip 2, as the working format

Decided on the evidence of the note above. The constants change with R0-09 G2, where the
whole receiver's sweep confirms or corrects the format and this journal's G2 (loss against
float, metric width) is rerun at the new widths. Record: `docs/gates/rx_ofdm.md`,
2026-11-19, decisions.

## 2026-11-20 · decision and G2 · eight bits, clip 3 (the founder), after R0-09's sweep

R0-09's sweep of the whole receiver (`docs/gates/rx_ofdm.md`, 2026-11-20: 2000 paired
frames per point, every rate, AWGN and TGn B, D, E) showed that no format of six bits or
fewer holds 0.2 dB at the 3/4-code rates in fading, that seven bits with a clip of 2 hold it
(worst +0.09 dB) and that eight bits with a clip of 3 lose nothing resolved (worst
+0.02 dB). **The founder chose eight bits, clip 3 on 2026-11-20**; it replaces the working
format of 2026-11-19 (six bits, clip 2) and the first choice of 2026-11-17 (four bits,
clip 1.5), which was right for the question it was asked on (BPSK in AWGN) and wrong for
the receiver in fading — open question 1 above, answered.

Constants: `SOFT_BITS = 8` (soft values ±127), `SOFT_CLIP = 3.0`, `METRIC_BITS = 13`
(spread ≤ 24·127 = 3048, so 2^13 = 8192 > 6096). `TRACEBACK_DEPTH` stays 64.

This journal's own G2 at the new format: `cargo run -q --release -p radio-tools --bin g2
-- viterbi 30000`, three rows added per rate, every earlier row identical to the run of
2026-11-17 (`docs/gates/reports/g2_viterbi.txt`). Loss at BER 1e-4 against 2^-12 soft
values and full traceback, dB:

| Configuration | 1/2 | 2/3 | 3/4 | worst | largest spread seen |
|---|---|---|---|---|---|
| 7 bit, clip 2.0 | 0.002 | 0.002 | 0.009 | 0.009 | 920 |
| **8 bit, clip 3.0** | 0.003 | 0.003 | −0.000 | 0.003 | 1366 |
| **8 bit / 3.0 + window 64** | 0.003 | 0.010 | 0.025 | **0.025** | — |

Within the budget of 2026-11-17 (0.15 dB for the soft values, 0.05 dB for the window, 0.2
together): what is left is the window's own 0.024 dB. The spread seen, 1366, is under the
bound of 3048.

G1 at the new constants: `cargo test -p radio-phy --test g1_viterbi` (12 pass; the
metric-bound test failed with `METRIC_BITS = 9`, as it should, and has a second case at the
decoder's own width, saturated inputs of ±127). `soft_value_contract` in `g1_rx_ofdm` is
rewritten for the format (level 1 → 42, 3 and beyond → 127, 0.011 → erasure, 0.012 → 1).

Cost, for ADR 0008's count: four more bits per soft value than the first choice and four
more bits per path metric (64 registers of 13 bits, add-compare-select 13 bits wide).

Open question 2 (the loss at BER 1e-5) is superseded by R0-09's sweep, which measures the
whole receiver at 10 % PER of 1000-octet frames.

## 2026-11-23 · decision · the traceback window is 96 (the founder), after the receiver's frames lost

The window of 64 chosen above (0.024 dB at a bit error rate of 10⁻⁴, BPSK in white noise)
was put into the receiver model on 2026-11-23 (ADR 0010, decision 6.1a) and measured as
frames lost for the first time: 11 to 25 % more frames of 1000 octets lost than with the
whole block at the rates coded at 3/4, in white noise and through TGn D and E
(`rx_ofdm.md` of this date, `reports/rx_ofdm_fixedw.txt`). With 96 it is 1 to 3 %, with
128 at most 3 frames of 2000. The founder chose 96. `TRACEBACK_DEPTH` = 96.

The G2 table above stands as what it measured; its budget (a bit error rate) was the wrong
yardstick for a frame of 8000 bits. Not redone here: the bit error sweep at 96 (the table
has "window 96: 0.000" at every rate).

## 2026-11-23 · G0 · the RTL (ADR 0010, decision 4: register exchange, one step a clock)

Order (the founder, 2026-11-23, `rx_ofdm.md` of this date): this decoder's RTL and its run
through the open flow first, the streaming data path's model after it.

**Interface** (`hw/rtl/viterbi.sv`):
- In: one depunctured pair a step, `in_a`, `in_b` signed 8 bits in ±127 (0 an erasure, −128
  never given; the depuncturer is a block of its own, ADR 0010 decision 1), `in_valid`,
  `in_first` on a block's first step, `in_last` on its last. A block is fed up to the end of
  its tail (SIGNAL: 24 steps; DATA: 16 + 8·LENGTH + 6); the pad is not fed. Never stalls;
  idle cycles anywhere.
- Out: `out_valid`, `out_bit`, `out_last`, a bit an edge at most, in order. The model's rule
  with `Termination::ZeroAfter(steps)`: the bit of step t − 95 decided by the best state
  after step t (ties: the lowest state) **4 edges after the edge that took step t**; at the
  last step the min(96, steps) bits left, from state 0's path, one an edge from 4 edges after
  the last step's edge, `out_last` on the final one.
- Contract: a block's first step no earlier than n + 2 edges after the previous block's last
  step, n its flushed bits (first written as n; the gate's first run showed that a block of
  one step that close loads the flush register before the previous flush is out).

**Number formats and the three questions ADR 0010 left to this G0:**
1. **The metric's width: 13 bits, no valid flags, no 14th bit.** Modulo comparison needs
   every compared difference below 2^12. The 64 metrics' spread is at most 24·127 = 3048
   from step 6 on (the bound of 2026-11-17: from the best state six steps earlier every
   state is reached); the two candidates of one state differ by at most 3048 + 2·254 = 3556.
   During the first six steps a reachable state's predecessor with dropped bit 1 is never
   reachable (the encoder starts in state 0: after k steps the low 6 − k bits of a reachable
   state are 0), so the model's choice there is always the dropped bit 0: the RTL forces it
   and compares nothing. Through those six steps every state's metric descends from state
   0's, so nothing needs clearing at a block's start: the old value is an offset common to
   all 64 (the mutation run below confirms that clearing changes nothing). State 0's metric
   alone is reset (13 flip-flops), for a four-valued simulation only: without it every metric
   descends from an X after power-up and the output is X for ever there (the independent
   review), while silicon and Verilator's two values do not care.
2. **Survivors: 64 registers of 90 bits** (window 96 less the six bits that are the state
   itself), each rewritten every step by shifting in the survivor bit of the chosen
   predecessor. The decided bit is the oldest bit of the best state's register; the flush
   reads state 0's 90 bits and six zeros.
3. **The best-state tree: 63 compare-selects in three registered stages of two levels**
   (64 → 16 → 4 → 1), the lower index kept on a tie, carrying each candidate's oldest
   register bit with its metric instead of its index, so no column of oldest bits is
   delayed beside it (ADR 0010 had expected that). 282 flip-flops of pipeline (16 and 4
   candidates of 14 bits, two valid bits).
- Branch metric: the two branches into a state come from predecessors that differ in the
  oldest bit, a tap of both generators (133₈ and 171₈ both end in 1), so their outputs are
  complements and their metrics opposite: one of ±(A + B), ±(A − B) a state, registered at
  the input.

**Reference and gate.** G3 against `decode_windowed` (`TRACEBACK_DEPTH` = 96), the model
whose G1 is the entries above (brute-force maximum likelihood, the free distances, CommPy,
OpenOFDM's frames, the windowed decision's definition). `rvec viterbi` →
`hw/vectors/viterbi.txt`; `make -C hw viterbi`.

## 2026-11-23 · G3 · `viterbi.sv` bit-exact with `decode_windowed`, every bit in its cycle

Command: `make -C hw viterbi` (Verilator 5.054, registers from random values,
`+verilator+rand+reset+2`). Result: `viterbi G3: 147 blocks, 44837 bits compared, 74 blocks
followed at once, 10 resets after a last step, 8 one-step blocks at the contract's edge,
0 mismatches`.

**Vectors** (`rvec viterbi`, 147 blocks; 63 before the independent review, below): every
length near the window and the forced start
(1 to 8, 23 to 25, 90 to 98, 189 to 193 steps); codewords in noise at rates 1/2, 2/3 and 3/4
(stolen values erased) of 150, 400 and 1000 steps; SIGNAL's 24 steps from no noise to σ = 2;
all erasures (every comparison a tie); every value at one rail, at random rails, uniform
random values; noise of σ = 1.6 (most decisions wrong); 12 022 steps (1500 octets, the
13-bit metrics wrap hundreds of times); 52 blocks of 97 to 200 steps of values from three or
five small levels ({−1, 0, 1}, {−127, 0, 127}, −2 to 2: equal metrics in the best-state
tree) or uniform; 32 blocks found by search until each of the tree's 32 first-level pairs
(2j, 2j + 1) has a decision where 2j is the lowest best state and 2j + 1 as good with
another decided bit (`tree_pair_ties`, a selector in the tool; the expected bits stay
`decode_windowed`'s); four blocks found by search whose last step's own decision differs
from the flush's first bit. The largest metric spread in them is 2286
(the tool refuses a file at 2047 or less, so 12 bits are wrong somewhere).

**Testbench** (`hw/tb/tb_viterbi.cpp`): holds every output to its edge (4 after the step's
edge; the flush from 4 after the last step's), and nothing at other edges; idle cycles
inside blocks (none, every fifth step, random), every other block followed as early as the
contract allows; resets while decisions come out, during a flush, and with their first edge
1 to 5 edges after a last step (nothing due after the reset may come out, what was due
before it must; the next block decodes); a one-step block exactly n + 2 edges after a long
block's last step, and eight one-step blocks chained at the contract's edge.

**Test first.** The gate failed on a stub (31 336 of 31 336 bits). The first RTL failed it
twice: a block of one step right behind a flush overwrote the flush register (the contract
was n edges; now n + 2, above), and the testbench still expected the outputs due during a
reset's own edges (its bug). Then it passed at the first full run, so it was not believed before mutations.

**Mutations** (44, each on a private copy of `hw/`, five in parallel; the script is not in
the repo). The first round (39) left seven alive; what they showed changed the RTL, the
vectors and the testbench:
- clearing the metrics at a block's start (two mutants removed it, both lived): equivalent,
  as G0 item 1 argues; the clearing was taken out of the RTL;
- the last step's own decision suppressed only by the flush's priority (lived): the
  suppression was taken out, the priority does it; a mutant that gives the decision the
  priority lived too, because no vector had a last decision that differs from the flush's
  first bit: four such blocks found by search are in the vectors now, and it fails;
- the flush's guard for the six state bits (lived: an index out of range reads 0 in
  simulation): the flush reads 96 bits, state 0's register over six zero bits, no guard;
- `s1_valid` and `acs_last` not reset (lived): no reset came right after a last step; the
  testbench now resets 1 to 5 edges after one, and both fail;
- the step index not reset (lived): equivalent, a block's first step carries `in_first`,
  which ignores the index; the reset was taken out;
- a mutant of the first tree level that carries an equal metric from the other side (lived:
  equal metrics, nothing changes) was replaced by one that moves the bit on a tie.

Final round, on the RTL and vectors as committed (after the independent review below,
which added three tree mutants of its own and the reset of state 0's metric): **46 of 48
fail, all on a wrong or misplaced output bit (none by a build or lint error)**; the two
alive are state 0's metric cleared at a block's start and reset to 1 instead of 0, both an
offset common to every metric, equivalent by G0 item 1.

| Mutation | Fails |
|---|---|
| metric 12 bits | yes (first wrong bit in block 50 of 63) |
| a tie taken to the predecessor with bit 1; the comparison unsigned; its sign inverted | yes |
| forced for 5 or 7 steps, or never | yes |
| the tree: a tie to the higher state (all levels; level 2 only; level 1's right-hand pairs only; level 1's bit only, in all groups or in group 7 alone); the losing metric carried; the bit of stage B or of the last level from the other side; register bit 88 read for 89 | yes |
| the flush: index + 1; a state bit 1; from state 1; one bit long; n without its + 1; `out_last` a bit early; the decision given priority over it | yes |
| the survivor bit shifted in inverted; the branch metrics' cases, A and B, a generator (133 → 131, 171 → 175), the complement of the second branch, P1 = P0 | yes |
| the index saturating at 94, stepping by 2, starting at 1; a decision tagged one step early or without a valid step; the tree's valid a stage early; the ACS stepping without a valid step | yes |
| `fstart` skipped; `acs_last` without a valid step; resets of `s1_valid`, `acs_tag`, `acs_last`, `va`, `fstart`, `factive`, `out_valid` removed | yes |
| state 0's metric cleared at a block's start; reset to 1 | no — equivalent (G0 item 1) |

**Independent review** (a fresh agent in its own worktree on 9a3c61a, given the code, the
model, the clause and this entry, not its conclusions). It found no defect in the RTL over
the interface's input space: read line by line against `decode_windowed` (branch outputs,
the complement, ties in the ACS and in the tree, the survivor shift, bit t − 95, the flush's
order for fewer than 6, 6 to 95 and 96 or more steps, the last decision's slot); the G0
bounds rebuilt; a model of its own built the RTL's way (13-bit modulo metrics, register
exchange, the same tree) equal to `decode_windowed` on 300 random blocks; 740 blocks of its
own through the gate, 0 mismatches; 36 mutants of its own. Found:
1. **Three mutants of the tree's tie rule lived** (a tie to the higher state at level 2;
   at level 1 in the right-hand pairs only; level 1's bit in one group of four only), each
   killed by its extra blocks. The tie-heavy blocks and the search for a deciding tie in
   each of the 32 first-level pairs are in the vectors now (above).
2. **A four-valued simulation would show X for ever after power-up** (no metric is reset;
   every one descends from state 0's). State 0's metric is reset now (G0 item 1).
3. **The case the contract's n + 2 was set for was never run** (a one-step block that
   close behind a long one). It is now; the reviewer showed n + 1 fails and n + 2 passes.
4. The mutation script is not in the repo, so "the mutation run confirmed" could not be
   reproduced by it; its own reasoning agreed.
It also found that every comparison inside the tree differs by at most 8·254 = 2032 (12
bits would do there; the ACS needs 13). Not checked by it: synthesis, the standard's text,
timing and area.

## 2026-11-23 · probe · the decoder through the open flow (ADR 0010, decision 4)

`silicon_probe.md` of this date: **0.96 mm² routed, +0.35 ns at 160 MHz** (IHP 130 nm,
typical corner), 7012 flip-flops, the add-compare-select loop the worst path; ADR 0010 had
estimated 0.5 to 0.7 mm². A second multiplexer per survivor and metric bit (no enable
flip-flop in the library) is most of the difference; a gated clock would remove it and
has not been tried.

**Decision (the founder, 2026-11-23, asked with these figures): register exchange stays**
(ADR 0010, decision 4, confirmed). The gated clock and a switching simulation for power
come later, with the clock-gating wrapper; the traceback memory is not built.
