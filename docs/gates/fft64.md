# Gate journal: `fft64`

Block: the receiver's 64-point forward transform (R0-09's data path, ADR 0010 decision 1).
Model: `radio_phy::rx_ofdm_fixed::fft64_fixed` (G2 done with the integer data path,
`docs/gates/rx_ofdm.md`, 2026-11-19; the outside references of its G1/G2 are there: transforms
known by hand and the float FFT, which has its own G1). RTL: `hw/rtl/fft64.sv` (to come), gate
`make -C hw fft64`.

## 2026-11-23 · G0 · the specification and the architecture

**Decided by the founder, 2026-11-23: B, R2SDF in natural order** (the recommendation, after
the review below). ADR 0010 is not amended: it left the architecture to this G0. Its area line
for `fft64` (≈ 0.25 mm²) is low; this entry's ≈ 0.4 stands until the flow measures it.

ADR 0010 left the architecture to this G0 ("in place on a register array with two butterfly
units, or a pipelined single-path transform in the model's order fed in bit-reversed order from
a 64-word store"). PHY architecture: a risk point (CLAUDE.md, 9). Reviewed by a fresh agent the
same day; what it changed is at the end.

### What the block must compute (fixed by the model; not open)

- IEEE 802.11-2020 §17.3.5.10's transform, forward: 64 complex samples in, 64 bins out, a bin
  the float FFT's over 8.
- Radix 2, decimation in time, 6 stages of 32 butterflies. A butterfly: t = w·b exact, then
  a·2^(T−2) ± t, each part narrowed to W bits: right shift by T − 2 (+1 on the stages of
  `fft_halve`), half to even, saturating.
- Widths (`FixedOfdmConfig::default()`): the input is the hand-off's 13 bits, saturated to
  `in_bits` and shifted left by `in_shift` 1 into W = 16 (`window_fft`,
  `rx_ofdm_fixed.rs`; the saturation is a no-op on the hand-off's 13-bit words); twiddles
  T = 12, Q(12, 10), the ROM e^(−j2πk/64), k < 32, half to even; stages 1, 3, 5 halve
  (`fft_halve` 0b101010). **G3's reference** is `window_fft`'s function: `fft64_fixed` on the
  input saturated to 13 bits and shifted by 1 (the block takes 13-bit samples).
- **Bit-exactness pins the butterflies, not the schedule.** Any architecture that performs the
  same 192 butterflies on the same operands with the same twiddles and narrowing is the model.
  Radix 2² or radix 4 regroup the twiddles and move the narrowing: a different transform in
  fixed point, a model change with its own G2. At 64 points it would buy nothing: R2²SDF needs
  two general multipliers and constant ones, what candidate B below already has.

### The candidates

**A — iterative, in place on a register array, B butterfly units** (ADR 0010's figure 96 is
B = 2). The window written into 64 × 32 flip-flops at bit-reversed addresses; then 6 × 32 / B
clocks of butterflies, each unit reading two words through 64:1 multiplexers and writing two
back. The array is busy for the whole transform, so T2 (which follows T1 at once) and SIGNAL
(144 … 207, during T2's transform) wait in a window store, a RAM with one read a clock: loading
the array from it is serial (≈ 64 clocks), or stage 0 runs at the store's pace (32 clocks at
least for two units, which need four operands a clock). The bins must also leave the array for
the channel (T1, T2), or the array holds them and blocks the next transform.

**A′ — the same butterflies in constant geometry (Pease)** on two 32-word rotating register
banks: stage after stage the units read the banks' heads (j, j + 32) and write (2j, 2j + 1) to
their tails. Bit-exact for the same reason as A; no 64:1 multiplexers. The fair version of A
for area; its clocks and its window store are A's.

**B — single-path delay feedback, radix 2, decimation in time, samples in natural order
(R2SDF).** Label each position of the model's in-place array by its bit reverse: the input
x[n] sits at label n, stage s pairs labels l and l + 32/2^s, bin k leaves at label rev6(k). A
stream in natural order therefore meets each butterfly's two inputs 32/2^s samples apart: six
stages with delay lines of 32, 16, 8, 4, 2, 1 words, the twiddle on the later input, bins out in
bit-reversed order (the bin store is addressed; the order costs nothing). It takes the window's
samples as they come: no window store, no bit-reversed read. Its twiddles (test below): stage 0
only 1, stage 1 1 and −j (no multiplier; both exact in Q(12, 10)), stage 2 W⁰, W⁸, W¹⁶, W²⁴ (one
constant, 724), stage 3 the multiples of W⁴ (constants 946, 392, 724), stages 4 and 5 a general
16 × 12 complex product from the ROM. A stage drains its last group while the next group enters:
after a window either the next follows at once (only T1 → T2) or a frame of 64 zeros flushes it
(SIGNAL, and every data symbol: decision 2 holds the next window until the tracker's step).

ADR 0010's variant of B (bit-reversed feed from a store) is worse than B in every respect and
is not carried. R2MDC (two paths) is bit-exact too but is built for two samples a clock; at one
it doubles the multipliers for nothing.

### Evidence for B's arithmetic

`crates/radio-phy/tests/g0_fft64_sdf.rs`: a sample-by-sample model of the six stages (delay
lines, labels, twiddle indices; the narrowing restated from the model's doc, which can only
make the test fail, since every result is compared with the real `fft64_fixed`). Command:
`cargo test --release -p radio-phy --test g0_fft64_sdf`, 3 tests green.
- 400 random windows at the input's 14 bits and 400 at the word's full 16, each run back to back
  and each followed by a zero frame: 1600 transforms, **all bit-exact**. The reviewer counted
  on an instrumented copy: at 16 bits every stage saturates (12 824, 408, 7267, 440, 4858, 231
  times), rounding ties occur at stages 1 to 5 (stage 0 has none: twiddle 1, no halving); at
  the real 14 bits saturation fires once in 400 windows (stage 4).
- **An abort mid-window**: cut after each of 1 … 63 samples, every stage's label counter put
  back to a frame's start, the delay lines left as they are: the next windows (after a zero
  frame, and back to back) are all bit-exact. An abort needs the counters reset and the
  output's valid killed, nothing more.
- The twiddle sets per stage asserted as listed above.
- Mutants, each fails: a twiddle index's low bit flipped at stage 5; the halving stages
  mirrored; the stages in the other order (delays 1 … 32, the DIF shape); the stages' label
  offset dropped (each stage's counter must start 64 − 2D late; the first version of the model
  had exactly this bug and the test caught it); the sum's imaginary sign; the abort's counters
  reset to 0. 6 of 6. The reviewer's own (half-up rounding at stage 4 or 5, saturation removed
  at stage 5, symmetric saturation at stage 1, 3 or 5) fail too.

### The comparison (estimates; nothing synthesised)

Cycles: the data path's pace is one symbol per shift-to-shift loop, 245 clocks today = the
window 1 + 64, the transform 1 + 96, the rest 83 (`g2_rx_data_schedule`). For B the transform's
cost is the flush, not the delay lines: the last bin ≈ 63 + p clocks after the window's last
sample, **p ≈ 13 pipeline registers assumed** (an input register, one per stage, two per general
multiplier, one per constant multiplier). The verdict (489 clocks after the first beat in the
schedule today) worked by hand from the schedule's rules; for B with SIGNAL's first sample held
48 clocks in the hand-off's history behind T2's zero frame: T1's bins end ≈ 139, T2's ≈ 203, the
channel 313, SIGNAL's bins ≈ 331, the verdict ≈ 454 (a hold in place of the zero frame: ≈ 453).

| | A, B = 2 (A′ the same) | A, B = 4 | **B, R2SDF** |
|---|---|---|---|
| bins after a data window's last sample | 97 (the array as the bin store until the products have read it) | ≈ 51 | ≈ 76 |
| the data path's loop (today 245) | 245 (2.6 × real time) | ≈ 199 (3.2 ×) | ≈ 224 (2.9 ×) |
| the verdict after the first beat | ≥ 505, up to ≈ 553 (T2 and SIGNAL loaded from the store; 489 was the allowance, not A) | worse in proportion: 8 reads a clock from the store | ≈ 454 |
| storage | 64 × 32 flip-flops + a window store (64 × 26, RAM) | the same | 63 × 32 flip-flops in delay lines; no window store |
| general complex multipliers | 2 | 4 | 2 (+ two constant ones) |
| butterfly adders (add, round, saturate) | 2 units | 4 | 6 (≈ 0.05 mm² more than A2) |
| multiplexers | A: 4 operands × 32 bits × 64:1 and a write path into every word; A′: 2:1 | twice A2's | 2:1 per stage as shift registers (every bit toggles each beat: power), or 32:1 and 16:1 reads as circular buffers |
| cell area, rough | A ≈ 0.45 to 0.5 mm²; A′ ≈ 0.35 + the store | ≈ 0.75 mm² | ≈ 0.4 mm² |

Area: the probe's yardsticks (`silicon_probe.md`): `nco`, one 13 × 11 complex rotation with its
ROM and 194 flip-flops, 0.075 mm² routed; a flip-flop ≈ 49 µm² placed; a general 16 × 12 complex
product ≈ 0.10 mm² by `nco` scaled with the widths. The yardstick is a routed design area (its
ROM and flip-flops in it), used here as cell area: the figures are a ranking within ± 40 %, and
**B against A′ is a wash on area**. ADR 0010's 0.25 mm² was low for all of them (it counted the
multipliers at `nco`'s size and not the storage). The routed area comes with the RTL through the
open flow.

Wider transforms (802.11ax: 256 points at 20 MHz, 1024 / 2048 at 80 / 160 MHz) will each have
their own fixed-point model and G2; nothing here carries over bit for bit. An SDF grows a stage
at a time with the long delay lines in `ram_1w1r` (a delay line is one write and one read), but
radix 2 is the SDF with the most multipliers (≈ 7 general ones at 2048 points against ≈ 4 to 5 for
radix 2²), and a 64-clock flush per window is not affordable when windows must run back to back
(ADR 0009's pool at 160 MS/s on 160 MHz); an in-place transform scales too on banked memory.
The scaling argument is therefore weak either way and is not a reason for B.

### Recommendation

**B, R2SDF in natural order**, on its own merits: it takes the stream as it is (no window
store, no load, no bit-reversed read), the verdict ≈ 50 to 100 clocks earlier than A2 and the
loop ≈ 21 clocks shorter, area level with A′ and below A2. A4 is ≈ 25 clocks faster on the
loop for ≈ 0.35 mm² more, an earlier verdict on paper but a store that has to deliver 8
operands a clock.

### What B's RTL has to settle (interface and control)

- **In**: samples of a window with an `in_valid`: guard-interval beats and dropped beats never
  reach the block, so they never advance the labels. An `in_ready`, low while a zero frame
  flushes; `out_ready` to the hand-off is a register (ADR 0010, decision 2), so a beat taken
  while `in_ready` falls needs a skid of one.
- **Gaps**: once the reader has caught up, beats come every 8 clocks. Each stage, its
  multiplier's pipeline included, advances on its own input's valid, so that the zero frame's
  64 steps also push the last ≈ 13 bins out; a block clock-enabled on beats alone would leave
  them in the pipeline.
- **Out**: a bin with its index and a valid, at the beat's pace (a clock apart only in the
  flush); never stalls (the bin store takes every bin).
- **Abort**: the counters back to a frame's start, the output's valid killed, the delay lines
  left (tested above).
- **The flush rule**: the zero frame (proposed: one rule, the labels never move) or a hold until
  the next window; the schedule model decides by the verdict's cycle (≈ 454 against ≈ 453).
- **The bin store** (ADR 0010's stores, outside the block): SIGNAL's bins are written
  (≈ 268 to 331) while the channel still reads T1's and T2's (from 203): a third slot, or T1 + T2
  accumulated read-modify-write as T2's bins come (reading bin k a cycle before writing it is
  legal under `memory.md`).
- The delay lines of 32 and 16 words: shift registers or `ram_1w1r` (a 32-deep line in it
  needs 33 words or a read a clock ahead: the wrapper forbids a read of the address being
  written). The general multipliers as three real products or four (the same integers).
- Later, not now: the pilots leave B at labels 39, 42, 53, 56, so the products could start ≈ 7
  clocks earlier; the channel's first pass could run on T2's bins as they come.

**The schedule model** (`radio_phy::rx_data`) needs new rules for B, not new numbers in
`DataTiming`: `F = max(W + 1, last bins)`, `B = F + fft` do not describe a transform that runs
while its window arrives, and T1's bins during T2's window and the zero frame's hold on the
reader have no rule yet. Restated with the RTL (ADR 0010, 6.2).

### The review (fresh agent, 2026-11-23)

It confirmed B's arithmetic and its test (with its own mutants and coverage counts, and the
abort rule, now a test here), and found the first draft unfair to A and short on control: A's
figures were the schedule's allowance, not A (its store-load cost was missing: the verdict
≥ 505, not 489); the interface lacked `in_valid`, `in_ready` with its skid, a valid per stage
and the abort; the bin store's third slot; the schedule model needs rules, not numbers; the
constant-geometry A′ and R2MDC were missing; radix 2² saves nothing at 64 points; the
butterfly adders and the shift registers' power were not in the area; the scaling argument
was overstated (now withdrawn as a reason). All taken.

## 2026-11-23 · G3 · `hw/rtl/fft64.sv`, R2SDF, against `fft64_window_fixed`

**The block** (its header has the contract): a window's 64 samples of 13 bits in time order,
`in_valid` with gaps anywhere; `in_next` with the 64th sample when the next window follows at
once (T1 → T2), otherwise a frame of 64 zero tokens flushes the window, `in_ready` low meanwhile,
with a skid of one for a reader whose valid is registered; bins out in bit-reversed order with
their index and `out_valid`, never stalling; `drop` (an abort) and `rst_n` drop every token in
flight and put the labels back to a frame's start, the delay lines left as they are. The labels
travel with the data (each stage's output is label − D); a bin is valid if its window was: label
0 leaves with its own window's 64th sample, the others with the next frame's first 63 tokens.
An input register, then per stage a twiddle register and a butterfly register: **a window's
last bin 75 edges after the edge that took its 64th sample** when the flush follows at once
(G0 estimated ≈ 76). Stages 0 and 1 multiply by shifts (1, −j), stages 2 and 3 by the ROM's
constant magnitudes (cos π/4, cos π/8, sin π/8, selected), stages 4 and 5 a general 16 × 12
complex product from the ROM. Delay lines as shift registers (the power question of the G0
stays open for the flow).

**Generated**: `rvec fft64` (236 windows: 160 random at the full 13 bits, a full-scale tone on
each of the 64 bins, constants, impulses and alternating signs at the rails; 72 bins of the
file saturate at the output, and internal saturation is exercised by the tones) and
`rvec fft64-rom` → `hw/rtl/gen/fft64_rom_pkg.sv` (widths, `HALVE`, the twiddle ROM from the new
`fft64_twiddles`). The reference is `fft64_window_fixed`, new: the receiver's own `window_fft`
(the input saturated to 13 bits and shifted by 1, then `fft64_fixed`), made public.

**Gate** (`make -C hw fft64`, `hw/tb/tb_fft64.cpp`): every window in four passes — no gaps and
a reader that looks at `in_ready`; a sample every 8 clocks, a registered reader, every fourth
window chained; random gaps, readers, chains and drops (in a window, during a flush, between
windows; random inputs, `in_valid` among them, during each drop and the reset); no gaps, a
registered reader and every window right behind the last, so that each window's first sample
goes through the skid. Every bin compared with the model's, each window's 64 exactly once in bit-reversed
order, nothing else out (no bin of a dropped window after the drop), the last bin's 75 edges
wherever a flush follows at once, `in_ready` high throughout a chained window. Registers start
random (`+verilator+rand+reset+2`). Result (this branch, after the pre-PR review): **236 windows,
57 989 bins compared, 789 latencies, 252 skids, 52 drops, 107 chains, 0 mismatches. PASS.**
(The mutants below ran on the gate's first version, three passes; the fourth pass and the
inputs during drops only add checks.)

**Mutants** (each built fresh; the two marked lint-non-fatal built with `-Wno-fatal`, since
they leave a signal unused):

| Mutant | Mismatches |
|---|---|
| half up on ties (`narrow`) | 5396 |
| no upper saturation | 381 |
| halving stages mirrored | 24 840 |
| twiddle index's low bit flipped at stage 5 | 29 533 |
| sin π/8's product by its constant + 1 | 3699 |
| −j as +j in stage 1 | 17 076 |
| skid never filled | 4190 |
| `in_next` ignored (lint non-fatal) | 113 |
| `drop` keeps the input counter | 13 874 |
| `out_valid` from the token's own frame only | 44 655 |
| labels move + D | 34 647 |
| the input not shifted into the word | 31 873 |

12 of 12. Two first attempts did not count: "sin π/8's constant negated" passed with 0
mismatches, an equivalent mutant (the constant's sign is carried into the `case` item and the
product alike), replaced by the one in the table; "`in_next` ignored" first **passed** the
gate: a flush between chained windows changes no bin, only the time. The check of `in_ready`
through a chained window was added for it, and it now fails (113 chains, each flagged).

**The pre-PR review** (fresh agent, 2026-11-24): nothing blocking. It reran the gate (numbers as
printed), counted the 72 saturated bins, and synthesised the block with the open flow's front
end (Yosys 0.67 `read_slang`): 0 errors, 0 warnings, ≈ 23.4k generic cells, ≈ 2.7k flip-flops
(2041 in the delay lines and the skid); the packed struct and the array of them are new in
`hw/rtl/` and slang takes them. Taken: `in_ready` is not AXI (a beat offered while it is low is
consumed) — said at the ports, for the data path's top level to keep the hand-off's registered
`out_ready`; the skid was hit 6 times only — the fourth pass and `skids ≥ windows / 2`, inputs
during drops and the reset; `mulc` returned 0 silently for a twiddle it does not list — a
simulation check (removing the cos π/8·sin π/8 branch now stops the gate: "stage 3 twiddle
(392, −946) not in mulc"); `skid_next` was dead (the skid holds a frame's label 0) — removed;
the Makefile's comment. Left open: the latency is checked at the last bin of a flushed window
only; stage 2's `mulc` builds the products for all three constants (Yosys keeps 12 constant
products: a stage-specific form saves a few hundred cells, for the run through the flow).

**Not covered here**
- The routed area and timing through the open flow, and the power of the shift-register delay
  lines against circular buffers (G0's open item): with the data path's next run on the build
  server.
- The flush rule against the schedule (zero frame, as proposed in G0) and the schedule model's
  new rules for a transform that runs while its window arrives (G0, "The schedule model"):
  `radio_phy::rx_data` is restated when the top level of ADR 0010 6.2 is built; its
  `DataTiming::fft` allowance (96) is now 75 after the window's last sample for this block.
- The bin store (a third slot or T1 + T2 accumulated): the channel's block — T1 + T2 accumulated (the founder, 2026-11-24; `rx_channel.md`).
