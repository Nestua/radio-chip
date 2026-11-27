# Gate journal: `rx_channel`

Block: the channel and the per-packet quotients (R0-09's data path, ADR 0010 decisions 1 and
6.2), with the data path's serial divider. Model: `radio_phy::rx_ofdm_fixed::{channel_from_bins,
tracker_constants, soft_scale}` (G1/G2 with the integer receiver, `docs/gates/rx_ofdm.md`: the
standard's own samples decode, the integer path within 0.04 dB of float; `channel_from_bins`
and `soft_scale` are the parts of `estimate_channel_fixed` and `symbol_soft_fixed` the block
computes, split out without a change of behaviour). RTL: `hw/rtl/rx_channel.sv`,
`hw/rtl/div_serial.sv`; gate `make -C hw rx_channel`.

## 2026-11-24 · G0 · the specification and the architecture

**The bin store accumulates: decided by the founder, 2026-11-24** (the recommendation; the
open item of `fft64.md`: "a third slot, or T1 + T2 accumulated").

### What the block computes (fixed by the model)

From T1's and T2's bins (`fft64`, W = 16 bits, any order within a window, all of T1's before
T2's), on the 52 used bins (IEEE 802.11-2020 §17.3.3, L(k) = ±1):
- Hs(k) = L(k)·(T1(k) + T2(k)). **18 bits a part, not the W + 1 = 17 the model's doc said**: the
  sum of two bins at −2^15 is −2^16, which L(k) = −1 makes +2^16 (the doc is corrected; the
  model's i64 was never wrong). |Hs|² ≤ 2·2^32: 34 bits unsigned.
- shift = max(bit length of the largest |Hs|² − 15, 0) (0 to 19); g(k) = |Hs(k)|² / 2^shift,
  half to even, saturated to 2^15 − 1 (the largest rounds to 2^15 when it lies in
  [2^(15+s) − 2^(s−1), 2^(15+s)): **the saturation binds**, it is not decoration); Σg (21 bits);
  n = bit length of Σg.
- The soft scale q_m = ⌊A_m·2^n / Σg⌋, 0 for Σg = 0; A_m = `scale_constant` (8 / 3:
  70443, 99621, 222759, 456521); q_m lies in [A_m, 2A_m]: 18, 18, 19, 20 bits (ADR 0010's
  schedule: "bit length of A + 1").
- The tracker's constants from the pilots' gains g₀…g₃ (−21, −7, 7, 21): a = max(g₀ + g₃, 1),
  b = max(g₁ + g₂, 1), N_o = g₀g₃b, N_i = g₁g₂a (46 bits), D = 9N_o + N_i (50 bits),
  s = max(bit length of D − 16, 0); dn = D >> s, and with the numerators shifted alike:
  outer = ⌊(3(N_o >> s)·2^17 + 14dn) / 28dn⌋ (below 2^11), inner = ⌊((N_i >> s)·2^17 + 14dn) /
  28dn⌋ (≤ 4681): 14 quotient bits each; variance = ⌊5(Σg·a·b >> s)·2^10 / 52dn⌋, all three 0
  for D = 0.
- **The variance in V + 2 = 12 bits, saturating at 4095, in the model itself** (changed with
  this block: `tracker_constants` saturates to `variance_max`). It used to be kept whole (up to
  ~2^25 when the pilots are weak and the data strong), but it is used only when it is at most 3
  nominal ones (3072; `track_fixed`: `usable`, then `lambda`), so the saturation changes no
  decision of the receiver. The first draft saturated in the vector generator instead (the RTL
  then matched a post-processed model: the review's point 4); now the RTL is held to the model
  as it is, and the package's widths (`VAR_Q` = V + 2, `COEF_Q` = K − 2) are generated from the
  configuration and tied to it by `g0_rx_channel`'s `quotient_widths`. Three hand-worked
  expectations of `g2_rx_ofdm` above the limit (5201, 7680, 7865, and "4 to 16 nominal" for the
  notched pilot) now read 4095, their arithmetic kept in the comments; the variance's rounding
  stays separated by its cases below the limit (1137, 1396, 1493), the limit itself by
  `usable`'s cases (3072 in, 3073 out).

### The architecture

- **The bin store** (`ram_1w1r`, 64 × 36): T1's bin k written as L(k)·T1(k) as it comes; T2's
  bin k reads word k and writes Hs(k) = word + L(k)·T2(k) back a clock later. No read meets a
  write of its address: within a window every bin comes once, and a T1 bin is written in its own
  cycle, so T2's first read finds it. The store is Hs for the rest of the packet (the products'
  read port). No third slot.
- **The first pass at the bins' pace**: the sum, the two squares (18 × 18), |Hs|² and the
  running largest, pipelined (a bin a clock in the flush).
- **The second pass**, 52 reads in a row: the same squarers (their input is the store's word
  plus L·T2 in the first pass, plus 0 in the second), the rounding (floor, the guard bit, the
  sticky bits), the saturation, the gain written (`ram_1w1r`, 64 × 15), Σg, the four pilots'
  gains kept.
- **The products**: one 37 × 16 multiplier, used six times (g₀g₃, g₁g₂, ·b, ·a, Σg·a, ·b), then
  D, s, the shifts, the numerators: ten clocks, inside BPSK's division.
- **The divider** (`div_serial`, shared with the tracker's fit later, ADR 0010): unsigned,
  restoring, a quotient bit a clock, in a fixed order: BPSK's scale, outer, inner, variance; a
  data scale per request after them. Its remainder starts at ⌊num / 2^q⌋ and the numerator's
  last q bits are shifted in: 25 bits of remainder instead of a 48-bit one, legal when
  num < den·2^q — which holds for every job by the bounds above (and is asserted in simulation).
  For the variance a comparison before the division, 5(Σg·a·b >> s) ≥ 208dn, replaces the
  quotient by 4095 (and the division's numerator by 0, keeping the run's length); the strict
  `>` would miss the quotient of exactly 2^12.
- **Interfaces**: bins with valid, the window (T1 or T2) and the index, no ready (the next
  packet's bins come after this one has ended, ADR 0010 Context 3 rule 5); out `chan_valid` (one
  cycle) with shift, Σg, n; then `q_valid` with the job (0 BPSK, 1 outer, 2 inner, 3 variance,
  4 + m the scale of modulation m on `scale_req`: one outstanding, from `chan_valid` on, never
  on the edge of the next packet's first bin — each a protocol error that stops simulation);
  read ports of both stores on the 52 used bins (Hs from `chan_valid` until the edge that takes
  the next packet's first bin, g until its second pass; the unused words are never written).
  `drop` returns it to idle, the stores as they are; the divider takes the same drop.

**Evidence** (`cargo test --release -p radio-phy --test g0_rx_channel`, 3 tests green; commit
e4aaee5): a
register-for-register model of this architecture (`tests/g0_rx_channel.rs`) against the model
on 300 random full-scale packets, 680 channels of one to eight taps at amplitudes 2^0 … 2^16
with noise, and the corners (silence, the rails, pilots removed singly, in pairs and all, the
variance saturating, shifts of 0, the gain's rounding to 2^15 at every shift from 1 to 18 —
which needs a pair (re, im): one part alone cannot reach that window below 2^34 — and the
variance's edge Σg = 207, 208, 209 with pilots of gain 1, where 208 makes the quotient exactly
2^12). Every width above is asserted on every case; equal on all. Branches reached (asserted):
a zero shift, a shift ≥ 18, Σg = 0, a saturated and a usable variance, each pair alone,
Hs = +2^16, the gain's saturation in ≥ 10 cases. Mutations of the architecture model (Hs in
17 bits, no gain saturation, a truncating shift, the shift off by one, the variance's check a
power of two off, no max(·, 1), D without its 9th N_o, coefficients in 12 bits, the
remainder's start off by one, Σg = 0 not replaced): **10 of 10 fail** — after two fixes to
the test: the rounding corner was built with one part (never reached, the saturation's
mutant passed) and the emulated divider returned 0 for a zero denominator instead of the
all-ones the hardware gives.

**Not decided here**: the schedule model (`radio_phy::rx_data`) keeps its allowance of 110
clocks for the channel (the RTL needs 63 after T2's last bin, below) until it is restated with
the top level (ADR 0010, 6.2; `fft64.md` lists the same for the transform's 96 → 75). The
divider's sharing with the tracker: its arbitration comes with the tracker.

## 2026-11-24 · G3 · `hw/rtl/rx_channel.sv` and `hw/rtl/div_serial.sv`

`make -C hw rx_channel`: `channel_chain` (the block and the divider on one drop) against
`hw/vectors/rx_channel.txt` (`rvec rx_channel`: 338 packets — the G0 test's corners, 204
channels at 17 levels, 100 random; constants in the generated `rtl/gen/rx_channel_pkg.sv`,
`rvec rx-channel-pkg`). Three passes over every packet: back to back in `fft64`'s order with T2
right behind T1; a bin every 8 clocks; random gaps and orders with aborted packets (this one or
another, dropped in T1, T2, the second pass or the quotients, random inputs during the drop, a
reset in its place one time in four, the packet then at once one time in three), every seventh
packet behind a packet at the rails aborted in T2's window with one clock of drop, requests
dropped while divided, and on every T1 bin an outside read of the address it writes. Per packet:
shift, Σg, n; the four quotients; every modulation's scale on request, and a request made while
the quotients run (its edge after the variance checked); Hs and g of the 52 used bins through
the read ports after the requests, and random words while the quotients run. Registers start
random (`+verilator+rand+reset+2`); every wait bounded.

**Latencies, by construction and checked to the edge on every packet**: `chan_valid` 63 edges
after the edge that took T2's last used bin (the last |Hs|² into the largest at + 4, the shift
at + 5, the second pass's 52 reads at + 6 … + 57, the last gain at + 62); the quotients at
21, 37, 53, 67 edges after `chan_valid` (each division q + 2 clocks: asked on the edge that
brings the previous quotient); a data scale 3 + q edges after its request when the divider is
free, or after the variance's `q_valid` when it was queued. Against ADR 0010's schedule
(`rx_data`): its channel allowance is 110 after the last bin, the RTL 63; its divisions are
q + 2 each, as here.

**Evidence** (`make -C hw rx_channel`, `scripts/check.sh` all green; commit e4aaee5):
`rx_channel: 338 packets x 3 passes, 8293 quotients, 4056 requests (181 queued), 215 drops
(107 replayed at once, 40 resets, 56 during a division, 48 after a packet at the rails), 52728
store words (+33889 during the quotients), 809457 edges, 0 mismatches` — PASS. Lint clean
(`make -C hw lint`).

**Mutations** (51, each on a private copy of `hw/`, the full gate): the L(k) sign, the
accumulation's sign, the second pass adding T2, the store's word order, the largest, the
shift ±1, the rounding's two conditions, the saturation (two ways), the second pass's address
jump, T2's count, Σg not cleared, the pilots' bins, max(·, 1), D, s, the coefficient's 3 and
its 14, 52dn, the variance check's ≥, its 4095, the D = 0 and Σg = 0 replacements, the next
division asked a clock late, the request's quotient bits, the request taken only when idle, the
state, `s1_v` … `s5_v`, `cr` and `waiting` not cleared by a drop, the multiplier's operand, 5·x,
the store's external read address, the store answering outside reads only in READY, or on the
edge of a T1 write, `cr` not cleared at a new packet; in the divider the comparison, the
remainder's start, the alignment, the length, `busy` not cleared by a drop.

How the bench got there: the first run on the first bench, **34 of 37 failed**; the three that
passed showed three holes: the variance's check `>` for `≥` (no packet had the quotient of
exactly 2^12: the G0 corner Σg = 208 added), a stage register and the divider's `busy` not
cleared by a drop (the bench waited 200 quiet clocks after every drop and an aborted packet was
always followed by itself). After the review (below) the stage registers' mutants passed again
on a new random sequence: they had been killed by luck, so the packet at the rails aborted in T2
is now deterministic. Four mutants first failed only by a lint warning; each was rewritten so
that it fails in simulation. **Final: 48 of 51 fail** (five of them by lint, each with a
functional twin that fails in simulation). **Equivalent, argued and confirmed by the review**:
`s4_v` not cleared by a drop (the stale entry's update of the largest acts on the first edge
after the drop and is overwritten by the next packet's reset of the largest at its first T1
bin, on that edge — the code orders the reset after it — or later); `s5_v` not cleared (a stale
gain, Σg and pilot gain belong to the aborted packet and are rewritten — Σg cleared at the
shift, every used word and pilot in the second pass — before `chan_valid`); `cr` not cleared by
a drop (cleared at every packet's first bin, before the quotients can start).

### The review (fresh agent, 2026-11-24)

It rebuilt every bound, rounding and saturation by hand against the model, regenerated the
vectors and package byte for byte, and ran 27 000 extra fuzz packets (rails, ±1, levels 2^0 …
2^16, pilots zeroed or ±1; shifts 0 … 19, 6379 saturated and 4744 usable variances, many
D = 0) through the bench: 0 mismatches. No arithmetic defect. It found, and this entry fixed:
- **the stores were read only in READY**, while the header promises Hs from `chan_valid`
  (SIGNAL is equalised during the quotients): the mutant "answer only in READY" passed. Now
  random words are read while the quotients run, and the store refuses an outside read on the
  edge of the next packet's first T1 write (it would have collided with the write);
- **a request on the edge that starts the next packet** was carried into it (served with the
  wrong Σg), and a second request on the edge that issues the first was lost silently: both,
  and requests outside QUOT and READY, are now protocol errors that stop simulation;
- **the variance's saturation lived in the vector generator**, not the model (non-negotiable 1
  bent): moved into `tracker_constants`, the widths generated and tied to the configuration;
- the untested paths: the queued request's latency, a drop while a data scale is divided, a
  reset mid-packet — all now in the bench;
- documentation: [A, 2A] not (A, 2A]; "the last used T2 bin"; the stores' validity windows and
  the unused words; why the block needs no ready; the commit for the evidence.

**Not covered here**
- The routed area and timing (the open flow, with the data path's next run). Rough: two
  18 × 18 squarers, one 37 × 16 multiplier, the divider's 25-bit subtractor, two
  64-word memories (36 and 15 bits), barrel shifters of 50 to 56 bits by 0 … 34 for the
  tracker's operands; ADR 0010's line "channel store … 0.2 to 0.4 mm²" stands until measured.
- The divider shared with the tracker (its arbitration), and the schedule model's channel
  allowance (G0, "Not decided here").
- A packet that starts while the last one's data scale is still being divided: excluded by
  rule 5, not tested.
- The receiver's packet error figures with the variance saturated in the model: not rerun; the
  saturation changes only values above 3 nominal ones, which `usable` rejects either way, so no
  decision of `decode_ppdu_fixed` can change (`g2_rx_ofdm`, `g2_rx_data_stream` green).

## 2026-11-25 · G3 · the multiplier pipelined (the founder's choice)

The whole data path through the open flow (`silicon_probe.md`, 2026-11-25) missed 160 MHz by
1.77 ns. Its worst path was a single-cycle product in this block. The founder chose on
2026-11-25 to pipeline both products, accepting the extra latency and a restated schedule.

**What changed.** The multiplier now has three stages:
1. the operands chosen by the step, registered;
2. the products of mb's two bytes;
3. their sum.

A product issued in step k is captured in step k + 3. To fit that, the issue order is now pg0·pg3, pg1·pg2, Σg·a, then po·b, pi·a and sa·b. The tracker's constants are ready in step 13 where they were ready in step 9, so `cr` is registered on CH + 14. The first division that needs `cr` is the outer coefficient, which is asked on CH + 21. **No latency of the block moves**, and the header and the bench are unchanged.

| Gate | Command | Result |
|---|---|---|
| G3 | `make -C hw rx_channel` | 338 packets × 3 passes, 8293 quotients, 0 mismatches |

**Mutants**, each in a private copy, all killed:
- po captured a step early: 2794 mismatches;
- mph shifted by 7;
- sa captured a step late.

## 2026-11-27 · G3 · the first pass's adds in modules of their own (no change of function)

Through the open flow with the stores as SRAM macros, `Hs`'s read-add-write was the worst
path, because the flow's mapper turned the 22-bit add into a ripple-carry chain
(`silicon_probe.md`, 2026-11-27). The two adds (`hs1_re`, `hs1_im`) are now instances of
`hw/rtl/lib/add_alone.sv` (`y = a + b`, kept as a module through synthesis). Nothing else
changed: no latency, no width.

| Gate | Command | Result |
|---|---|---|
| G3 | `make -C hw rx_channel` | 338 packets × 3 passes, 8293 quotients, PASS |
| G3 | `make -C hw rx_data` | 58 runs, 590 038 cycles, 189 793 beats, 0 mismatches |
| lint | `make -C hw lint` | clean |

**Mutant:** `y = a − b` in `add_alone.sv` fails `make -C hw rx_channel`; restored by copy,
the gate passes again.
