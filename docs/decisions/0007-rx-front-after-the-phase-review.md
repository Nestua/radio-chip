# ADR 0007 — Receiver front after the phase review: CFO output, hand-off, order of work

Status: **accepted** · proposed 2026-11-19, accepted by the founder 2026-11-19 · PHY architecture
(risk point).
Source: the independent review of 2026-11-19 (`docs/reviews/2026-11-19-r0-phase-review.md`) and
the reruns in `docs/gates/reports/rx_front_agc.txt`, `rx_front_tailbw.txt`, `rx_front_level.txt`
(journal: `docs/gates/rx_front.md`, 2026-11-19, measurements).

## Decided

1. **CFO output: option B.** The acquisition hands on the fine estimate; the weight of the
   coarse estimate stays a parameter of the model (default 0), so the combination can be
   measured again once a real AGC has been captured (R1).
2. **ADR 0006 is amended (as ADR 0008, with its own review) before the decision engine:**
   hand-off of samples to the rest of the receiver, the latency budget to SIGNAL and the ACK,
   multiplier folding with the area and timing figures of `docs/gates/silicon_probe.md`.
3. **Order of work: the model first, then more receiver-front RTL.** Part d steps 2–4 wait.
   Step 2 (the search) depends on none of the findings, but it is scheduled after the model
   work as well: the founder accepted the order "model, then front RTL" as a whole.

The measurements were reviewed by a second fresh reviewer, whose corrections are included.
The text below is the proposal as it was put to the founder.

## Decision 1 — which CFO the acquisition hands on

**Today:** `cfo = coarse + 0.771·(fine − coarse)`, the inverse-variance weight for clean AWGN.
The coarse estimate uses STF samples 32…144, the fine one GI2 and the two long symbols.

**Measured** (`rx_front_agc.txt`; the phase jumps are assumed, no real AGC has been captured):

| What happens | Coarse | Fine | Combined |
|---|---|---|---|
| nothing | 0.9 kHz | 477 Hz | 422 Hz |
| a gain step before STF sample 32 | 0.9–1.0 kHz | unchanged | unchanged |
| a −20 dB step at sample 48, start clipped | 13 kHz | unchanged | 3.2 kHz |
| a ±30° phase jump at sample 48 | 10 kHz | unchanged | 2.3 kHz |
| a −2 dB trim with ±30° inside the LTF | unchanged | 2.4–8.3 kHz | 1.9–6.4 kHz |

Each estimate is damaged by clipping or a phase jump inside its own window and by nothing
outside it. The combination inherits both exposures.

| Option | Clean AWGN | The AGC must be still from | Cost |
|---|---|---|---|
| A. Keep the combination | 422 Hz | STF sample 32 (1.6 µs into the packet) | none |
| B. Hand on the fine estimate; coarse resolves its ambiguity and rotates the confirmation | 477 Hz | the end of the STF (8 µs) for the output; sample 32 still for the confirmation at large jumps | model: one line; G1/G2 rerun; the decision engine loses a multiply-add |
| C. Weight by what the AGC reports (gain changes and their times) | 422 Hz | the same as B, and better where the AGC was still earlier | an AGC-to-front signal that does not exist yet |
| D. Decide after R1's captures of the AD9361's AGC | — | — | the decision engine waits for it anyway (decision 2) |

**Recommendation: B, with the weight as a parameter of the model.** A gain control cannot be
expected to finish 1.6 µs into a packet: the detector itself does not fire before sample 67.
Giving up 55 Hz at 20 dB buys the AGC the whole short training field. `FixedAcquisition`
already carries `coarse_phase` and `fine_phase`, so the RTL architecture does not change.
D is the cautious alternative and costs nothing now.

**Not solved by any option:**
- The confirmation rotates the LTF by the coarse estimate: with a clipped −20 dB step and
  jumps up to ±90°, a third of the frames are not acquired. A coarse window placed after the
  last gain change would fix it; that needs the AGC's behaviour.
- A gain step in the last STF period or in the LTF puts the timing a whole training symbol
  off (214 of 500 frames at sample 144).
- Both become requirements on the AGC ("still from the end of the STF") and gates on
  `agc_transient` once the AGC is modelled.

## Decision 2 — ADR 0006 before the decision engine is built

The review found two gaps in ADR 0006. Neither affects the blocks already at G3 or the latch
path of part d, step 1.

- **No hand-off.** The acquisition is applied up to T1 + 279 samples, 3.5 µs after SIGNAL
  ends. The rest of the receiver needs the samples from T1 on, CFO-corrected, faster than real
  time to catch up. ADR 0006 says nothing else reads the history.
- **No latency budget.** Antenna to SIGNAL-decoded, and to the ACK 16 µs after the frame ends
  (SIFS). The 64-sample decision latency was chosen for the decision engine's clock budget
  without that budget on the table.
- **Area.** A second 128-multiplier array used one clock in R. Folding one array over R, or
  sharing `ltf_corr`'s, was deferred to R5; the reviewer argues it belongs in the first
  design.

**Recommendation:** amend ADR 0006 (as ADR 0008, with its own review) before part d steps 3
and 4: a replay port on the history, the latency budget, and the multiplier folding decided
with numbers from a timing and area probe. Part d step 2 (the search: |c|², the energy
window, the running maximum, the deadline) is not touched by any finding and can go ahead.

## Decision 3 — order of work

**Today's order:** finish the receiver-front RTL (part d), then R0-09 (channel estimation).

**The review's point:** every bit-exact block makes an algorithm change cost model + vectors +
RTL + mutations, and the algorithm has not yet met an AGC, a decoded packet or real air.
R0-08c is already a planned retune of a block whose RTL exists.

**Recommendation:**
1. Part d step 2 now (independent of everything above).
2. Then the model, not RTL: R0-09 (channel estimation, equaliser, SIGNAL and DATA decode) so
   that a packet error rate exists (its float receiver and G1 gate were started on
   2026-11-19: it is on the backlog under any answer to this ADR), and an AGC model with a
   gate on `agc_transient`.
3. G2 restated as PER loss in dB once (2) exists; the word lengths chosen on "agrees with
   float" are revisited then (they can only shrink).
4. ADR 0008, then part d steps 3 and 4.
5. R0-08c's retune of `det_a` before its RTL is touched again, with band-limited noise and an
   adjacent-channel STF in its false-alarm measurements (finding 5).

**Backlog items** (added to `BACKLOG.md` with this ADR's acceptance, except where noted):
- AGC model and its gate, with its two requirements: still from the end of the STF, and a
  target window for the packet's level (the fixed model's CFO estimate keeps its budget from
  −3 to −15 dBFS and is +6 % at −30 dBFS, `rx_front_level.txt`; the upper edge comes from
  what the data symbols tolerate, R0-09).
- A hash-only Annex I.1 fixture so the strongest TX reference runs in CI (finding 7): not
  added, it waits for a licence decision (risk point).
- A check of the IHP SG13G2 memory macros against the wrapper's read/write contract: done,
  `docs/gates/memory.md` (2026-11-19).
- DC blocker hold or rescale at a gain step.

## Consequences

- One model change (decision 1, option B) with G1/G2 reruns. `hw/vectors/` do not change: no
  vector contains the combined CFO yet.
- The G3 evidence so far stays valid.
- R0-08b closes at "blocks, latch path and search at G3"; the decision engine and the top
  level move behind R0-09 and ADR 0008.
