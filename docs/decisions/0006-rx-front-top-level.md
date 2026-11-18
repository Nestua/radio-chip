# ADR 0006 — Receiver front top level: decision engine, provisional latch, NCO delay

Status: **proposed** · 2026-11-18 · PHY architecture (risk point): independent review and
founder approval before part d of R0-08b is built on it.

## Context

R0-08b has the RTL blocks bit-exact with `RxFrontFixed`: `ltf_corr`, `cordic`, `nco`,
`dc_block` and `det_a` (`docs/gates/rx_front.md`, parts 3a–3d). The top level wires them and
adds four things the model computes "for free":

- the provisional CFO latch at each detection event;
- the step change of the NCO at that event;
- the acquisition decision `decide()` within `DECISION_LATENCY` = L samples (open question 13,
  PR #24, founder-approved; lossless for L ≤ 63 + (GI − B));
- the search control.

The model is not changed by this ADR. Every option below keeps the RTL bit-exact with it.

Notation:
- R is the number of clocks per sample: 160 MHz at 20 MS/s is R = 8, 120 MHz R = 6,
  100 MHz R = 5. The 10 and 5 MHz modes have 2× and 4× the R.
- w = GI − B = 12 at 20 MHz with the 200 ns back-off.
- Cycle counts are estimates from the model's loop bounds, made before any RTL. Part d's G3
  measures the real ones.

## Decision 1 — the decision engine: a dedicated streaming 64-tap correlator (1A)

`decide()` needs, from the history `y` (512 × 2 × 13 bits):

| Step | Work | Reads of the history |
|---|---|---|
| choose the grid (latest start keeping 7/8 of the energy) | w + 1 = 13 energies | the energy ring |
| final coarse sum, 12-bit inputs | 96 complex MACs | 2 per product |
| coarse angle | 1 CORDIC pass (17 clocks) | — |
| rotation of 128 + w = 140 samples | 140 complex multiplies (NCO table) | 1 per sample |
| 2w = 24 64-tap correlations, then the non-coherent and coherent sums | streaming correlator, 1 sample per clock | — (rotated stream) |
| received energy | 128 squares on the rotated stream | — |
| fine sum, 12-bit inputs, lag 64 | 88 complex MACs | 2 per product |
| fine angle | 1 CORDIC pass | — |

**The engine.** It is a second, dedicated 64-tap constant correlator, fed by one rotator at one
sample per clock. The main `ltf_corr` keeps running the shadow search while a decision is
pending (PR #24).

**Area, corrected.** The Hermitian fold of the 64 LTF taps gives the same 128 constant
multiplications as `ltf_corr`, whose T1 = T2 fold also leaves 64 unique taps. Only the window
(64 instead of 128 registers) and the pre-adders are smaller. On the KV260 this is ≈ 124 DSPs
more, out of 1248. In an ASIC, constant multiplications are shift-and-add.

**Also needed, outside the correlator:**
- one complex MAC (12-bit inputs) for the coarse and the fine sum. The detector's 13-bit C
  cannot be reused, because the decision sums run on E = 12-bit inputs;
- one squarer for the energy;
- a rotator (table + complex multiply), a second NCO instance;
- a share of the one CORDIC.

**Schedule.** The history is a 1-write, 2-read memory. The write takes one slot of port A per
sample, so the reads are stretched by about R/(R − 1). The grid is chosen first, and the
coarse sum takes both ports. The rotation (port A) and the fine sum (port B, one read per clock)
then run in parallel:

```
13 (grid) + 96·R/(R−1) (coarse) + 18 (CORDIC) + max(140·R/(R−1) + 10, 176·R/(R−1)) + 18 + 5
```

The budget for this work is (L − D)·R clocks, where D is the NCO-path delay of decision 3:

| R | Clock at 20 MS/s | D | Work (clocks) | Budget at L = 64 | Budget at L = 71 |
|---|---|---|---|---|---|
| 5 | 100 MHz | 5 | ≈ 394 | 295 ✗ | 330 ✗ |
| 6 | 120 MHz | 4 | ≈ 380 | 360 ✗ | 402 ✓ (≈ 5 %) |
| 8 | 160 MHz | 3 | ≈ 365 | 488 ✓ (≈ 25 %) | 544 ✓ |

The long pole is the fine sum: 176 single-port reads. A third read port (a copy of the history,
+13.3 kbit) runs it at 2 reads per clock, and the rotation becomes the long pole. That brings
the work to ≈ 347 at R = 6: ≈ 4 % slack at L = 64, ≈ 14 % at L = 71.

**Consequences:**
- The 20 MHz mode is comfortable only at R ≥ 8 with L = 64.
- R = 6 fits only with thin slack: either L = 71 (≈ 5 %) or a third read port (≈ 4 %), or
  ≈ 14 % with both.
- L = 71 is still lossless up to a 400 ns back-off (w ≥ 8). This needs no model change beyond
  the default of `DECISION_LATENCY`, within the bound already gated (PR #24).
- R = 5 does not fit at any lossless L. It would need a second history port pair and a
  two-sample-per-clock correlator, i.e. double the area.

**Proposal:**
- L and D are parameters of the RTL, as L already is of the model.
- Target R = 8 at L = 64.
- If the FPGA does not close 160 MHz, fall back to R = 6 with a third read port and L = 71
  (≈ 14 % slack).
- For reference, `ltf_corr` today has WNS −1.9 ns at 160 MHz before its pipelining, ≈ 120 MHz.
- The part-d G3 testbench runs at both R = 6 and R = 8, with idle cycles. The design's
  independence of R is then evidence, not a claim.

**Deferred (ASIC area).** `ltf_corr`'s multipliers are busy 1 clock in R. A second 64-sample
window and an operand multiplexer in front of its multipliers could do the decision's
correlations in the idle clocks, which saves the second array. The cost is a multiplexer on its
critical path and a new G3 of a merged block. Revisit for R5 (silicon).

## Decision 2 — the provisional latch: a delay line of the detector's C (2A)

At an event at sample n, the model's latch is the lag-16 sum of the 96 products whose later
sample is at most n − 24, on the same D = 13-bit samples as detector A. That is exactly
C(n − 24) + C(n − 72), two of `det_a`'s 48-product windows.

- **Design:** a 72-deep delay line of C (2 × 32 bits) and one adder. No multiplier; bit-exact by
  construction.
- **Memory:** 72 × 64 bits = 4.6 kbit, 1 write and 1 read per sample. In an ASIC this is a macro
  or ≈ 4600 flip-flops. Together with the history, it is the first block that needs the memory
  wrapper of `hw/rtl/lib/` (CLAUDE.md, RTL rules). The wrapper and its own gate come first in
  part d.
- **Start-up:** the model skips the latch for events before n + 1 = 24 + 16 + 96 = 136 samples
  (`checked_sub`). The RTL gates it by the same sample count, and the part-d vectors contain an
  event in the first 135 samples of a stream.

Rejected:
- a second running accumulator (+2 complex multiplies per sample);
- computing the latch from the history at each event (≈ 3 complex MACs per clock at R = 8, since
  events come every 4 samples on an STF).

## Decision 3 — CORDIC latency against the NCO step: the NCO path runs D samples behind (3A)

In the model, the step changes at the event's sample and already rotates that sample. In the
RTL, an event is known after `det_a` (4 clocks), then the latch read and add, then the CORDIC
(17 clocks): ≈ 23 clocks after the sample.

- **Design:** the NCO, the main correlator and the search run on y delayed by D samples. Then the
  model's order is preserved and the RTL stays bit-exact. Search positions, the deadline and
  `detected_at` are kept in model sample numbers.
- **D is a parameter:** D ≥ ⌈23 / R⌉ + 1 for safety, i.e. D = 5 at R = 5, 4 at R = 6, 3 at R = 8.
  It costs D × 26 bits of registers. D also shortens the decision budget, as tabled above.
- **Rejected:** changing the model so that the step acts from n + Δ. That is a behaviour change
  with G1/G2 reruns and no benefit.

## Shared resources and the rules that keep them deterministic

- **One CORDIC.** It serves the latch (at an event) and the decision (coarse, then fine angle).
  Both can present an input in the same clock, so the latch has fixed priority. A decision input
  waits at most one clock per collision, and at most 2 per decision. That is in the schedule's
  slack, and the results are unchanged because only their timing moves.
- **The energy ring** (81 positions) does not last the decision: the w + 1 energies that choose
  the grid are already 63 + w old at acceptance (journal, part 3b). The grid is therefore chosen
  first, in 13 clocks, i.e. 2–3 samples. The ring has 81 − (64 + w) = 5 samples of margin.
- **The run counter.** `det_a.clear` is asserted on the clock in which stage 4 holds the
  sample at which the acquisition is applied (decided_at). This is the contract checked in the
  part-3d review.
- **History memory ports:** port A carries one write per sample plus the coarse sum and the
  rotation; port B carries the coarse sum and the fine sum. Nothing else reads the history: the
  latch uses its own delay line, and the search uses the main correlator's stream.

## Consequences

- No model change, unless the R = 6 fallback's L = 71 is taken (`DECISION_LATENCY` 64 → 71, inside the
  gated lossless bound). G1/G2/G3 so far stay valid.
- New hardware for part d:
  - the decision correlator (≈ 128 constant multiplications) and one 12-bit complex MAC;
  - one squarer and a rotator;
  - two memories through `hw/rtl/lib/` wrappers (13.3 kbit history, 4.6 kbit C delay line);
  - a D-sample delay on y.
- The part-d gate (G3 of the whole front against `RxFrontFixed` acquisitions and traces) runs at
  R = 6 and R = 8 with idle cycles. Its vectors include an event before sample 136, CORDIC
  collisions, a decision while an event latches (tone + two packets), and the acceptance at the
  energy ring's margin.
- Timing closure at 160 MHz on the KV260 is a part-d task: pipeline `ltf_corr` and register
  every multiply and wide add, following the phy-sim Vivado notes.
