# Gate journal: `rx_symbol`

Block: per OFDM symbol, the products, the pilots for the tracker, the common phase, and the
soft pass (rotation, demapper, scale) — R0-09's data path, ADR 0010 decisions 1 and 6.2 ("the
products, rotation, demapper and scale"), with the data path's own iterative CORDIC. Model:
`radio_phy::rx_ofdm_fixed::{symbol_pilots, symbol_from_bins}` — `symbol_soft_fixed` after its
FFT and its tracker step, split out without a change of behaviour (every `radio-phy` test as
before; G1/G2 of the integer receiver in `docs/gates/rx_ofdm.md`). RTL: `hw/rtl/rx_symbol.sv`,
`hw/rtl/cordic_iter.sv`; gates `make -C hw rx_symbol`, `make -C hw cordic_iter`.

## 2026-11-24 · G0 · the specification and the architecture

**Two slots: decided by the founder, 2026-11-24** (the recommendation). `fft64` hands its bins
on without a stop; SIGNAL's bins can come before the channel is in (the products need Hs and
the shift), and the first data symbol's while SIGNAL is still being processed (its window is
known at the first beat, ADR 0010 decision 3). The alternative, one slot with the next window
held until the soft pass has read it, saves ~2 kbit of RAM but adds a rule to the reader and
the schedule and ~50 to 80 clocks a symbol while the history catches up.

### What the block computes (fixed by the model)

Per symbol, from its 64 bins Z (W = 16 bits, bin order) and the packet's channel (Hs, W + 2 = 18
bits a part; g, 15 bits; the shift, 0 … 19; n = bit length of Σg, 1 … 21):
- u(k) = Z(k)·Hs*(k) / 2^shift, half to even, saturated to U = 16 bits (the sum of two
  products: 35 bits).
- The pilots at −21, −7, 7, 21 (IEEE 802.11-2020 §17.3.5.9): u times the base sign {1, 1, 1, −1}
  times the polarity p_n (§17.3.5.10; n = 0 for SIGNAL, k + 1 for data symbol k, modulo 127).
  **U + 1 = 17 bits**: the sign of −2^15 is not saturated (the model keeps it whole).
- With the tracker's slope θ (P + 1 = 23 bits, saturated by the tracker): each pilot rotated by
  −c·θ through the front's rotator (the phase in P = 22 bits plus half a table step, the table's
  top L = 9 bits, Q = 11-bit values, the product (28 bits) narrowed by 10, half to even,
  saturated to 16); summed (18 bits); φ = the CORDIC's angle (A = 16 bits) · 2^(P − A).
  **Only θ's low P bits matter**: the phase is kept modulo 2^P (tested: θ and θ ± 2^P give the
  same symbol).
- The soft pass, data subcarriers −26 … 26: u rotated by −(φ + c·θ) as above; the demapper of
  each axis (−v; |v| − t_hi; ||v| − t_hi| − t_lo, t = g·κ / 2^10 half to even, κ = 324 for
  16-QAM's c = 2, 316 and 158 for 64-QAM's c = 4 and 2: 24-bit products, 15-bit thresholds);
  the values are 17 bits with |m| ≤ 2^15; each times the packet's quotient q (< 2^20) — 36 bits
  — then (|m|·q + 2^(down − 1)) >> down, down = n + F = 5 … 25, clipped to 127, the sign
  restored. N_BPSC values a subcarrier in on-air order (the deinterleaver is the next block).
- Only the default configuration is hardware: pilot and timing tracking on, soft 8 bits / clip
  3 (the model's other branches are for the float comparisons).

### The architecture

- **Two slots**, each a `ram_1w1r` 64 × 32: a symbol's 52 used bins are written to the empty
  slot as they come (the others ignored); the slots alternate. `bins_ok` tells the top level
  whether the next symbol's bins may come (the slot they would go to is empty or filling); a bin
  for a slot that is not stops simulation.
- **The products engine**, once a slot is full and `chan_valid` has come: 52 reads of Z (the
  slot) and Hs (the channel block's read port, a clock later) — the pilots first, then the data
  subcarriers — four real products (16 × 18), their sums, the shift (floor, half bit, sticky,
  even), the saturation, u written back in place. The pilots, their signs removed, go to the
  tracker with `pil_valid`.
- **The rotation engine**, symbol by symbol, once a slot's products are in and its θ has come
  (one θ may wait; θ is used in the cycle it comes): the four pilots read again, re-signed,
  rotated by −c·θ (c·θ: a 6-bit by 22-bit product modulo 2^P), summed, sent to the CORDIC
  (held until taken); φ from its angle; then, once the symbol's soft scale is in (SIGNAL: BPSK's;
  data: the data modulation's, which needs `mod_valid`), the soft pass — a subcarrier a clock:
  u and g read, the phase, the table (folded by quadrant), four products (17 × 11), the
  narrowing, two threshold products (15 × 9), the demapper, six |m|·q products (16 × 20), the
  rounding and clip — and the slot is empty after its 48th read. One rotator for the pilots and
  the data; the pipeline carries a tag (pilot, subcarrier, slot).
- **The CORDIC**: `cordic_iter`, the pipelined `cordic`'s arithmetic on one set of registers, a
  micro-rotation a clock (17 edges from input to angle, as the pipelined block's; a new input in
  the cycle the angle comes out: one every 18 clocks). The data path's only CORDIC (ADR 0010:
  the tracker's two angles a symbol and the common phase); its arbitration comes with the
  tracker. Gated on the CORDIC's own vectors.
- **Interfaces**: bins with valid and index (no ready: `bins_ok` is the top level's to obey);
  `chan_valid` with the shift and n; read ports to the channel block's Hs and g stores (a word a
  clock after the read, as `rx_channel` answers); `scale_valid` with a modulation's q (from
  `rx_channel`'s quotients), `mod_valid` with the data's modulation; `pil_valid` and four
  pilots out, θ in; the CORDIC's request and answer; `phi_valid`; `soft_valid` with the
  subcarrier's number, the modulation, the last flag and six 8-bit lanes (lanes N_BPSC … 5 are
  0). No ready on the soft values (a subcarrier a clock; the deinterleaver takes them). `drop`
  returns to a packet's start, both slots empty; the top level drops at every packet's end.

**Evidence** (`cargo test --release -p radio-phy --test g0_rx_symbol`, 5 tests green): a
register-for-register model of this architecture (`tests/g0_rx_symbol.rs`) against the model
on 816 symbols on channels of one to eight taps at 2^0 … 2^16 (64-QAM-like points through the
channel, a common phase and a slope, noise; every modulation), 1500 symbols at the inputs'
limits (Hs, g, shift, Σg anywhere in their formats, random bins, θ at both rails and random),
and the rails (every bin and Hs at −2^15 / −2^17 and the other corners, shifts 0, 1 and 19, Σg
1 and 2^21 − 1, θ 0 and ±2^22, polarity +1 and −1, every modulation). Every width above is
asserted on every case; equal on all. θ modulo 2^P: 50 cases. Mutations of the architecture
model (no saturation of u, no half step, a truncating soft rounding, no clip, truncated
thresholds, the pilots' sign not removed, ||m1|| for |m1|, no saturation after the rotation,
φ widened by 5, θ kept in 21 bits, φ + c·θ for φ − c·θ, m1 = v − t for |v| − t, the polarity
index off by one, Z·Hs for Z·Hs*): **14 of 14 fail**.

**Not decided here** (with the top level, ADR 0010 6.2): the schedule model (`radio_phy::rx_data`)
keeps its allowances — products 3, rotator 4, demap 9, CORDIC 18 — until it is restated (the
RTL below, in edges after the last of each event's triggers: pilots 9, φ's request 10, φ 18
after the CORDIC takes it, the first soft value 8); the reader's rule that the next window waits for `bins_ok`
(by the schedule's margins it never waits, but nothing structural guarantees it); the tracker's
pair products (the schedule put them on the products unit; here the tracker gets the pilots);
the CORDIC's arbitration between the tracker and this block.

## 2026-11-24 · G3 · `hw/rtl/rx_symbol.sv` and `hw/rtl/cordic_iter.sv`

**`make -C hw cordic_iter`**: the CORDIC's vectors (`hw/vectors/cordic.txt`, 6196 inputs,
`cordic_angle`), one input at a time — in the cycle the previous angle comes out (every third),
or after a gap of one to three clocks — every 50th aborted at a random edge in flight (by reset
every 150th), `busy` checked high until the angle and low with it, the angle's edge checked.
`cordic_iter G3: 6196 cases, 6073 outputs, 123 aborted, 2066 back to back, 0 mismatches` — PASS.

**`make -C hw rx_symbol`**: `symbol_chain` (the block and `cordic_iter` on one drop) against
`hw/vectors/rx_symbol.txt` (`rvec rx_symbol`: 127 packets, 437 symbols — 68 channels through
the model's estimate at every level and modulation, 48 at the inputs' limits (θ at both rails in
half of them), 10 at the rails (shift 0 and 19, Σg 1 and 2^21 − 1), one of 131 symbols past the
polarity sequence's period; constants in the generated `rtl/gen/rx_symbol_pkg.sv`,
`rvec rx-symbol-pkg`, whose polarity sequence begins 1, 1, 1, 1, −1, −1, −1, 1 as §17.3.5.10
prints it). The bench is the block's surroundings: the channel's stores (a word a clock after
the read, random words on a cycle without one), the tracker (θ after the symbol's pilots, in
order; SIGNAL's at any time, also before its bins), the quotients and the modulation at random
times from `chan_valid` on (late enough that the soft pass waits, 88 times), the FFT (a symbol's
64 bins in a random order, a bin a clock, every 8 clocks or random gaps per packet; the next
symbol's first bin only on `bins_ok`, which held them 14 489 times). Two passes; packets dropped
at their end; one in five aborted at a random edge (one in four of those by reset), nothing may
come out after it, half replayed at once. Every pilot, φ and soft value (and the unused lanes'
zeros, `soft_j`, `soft_last`, `soft_mod`) against the model; **every event's edge** against the
latencies in the block's header (pilots 9, φ's request 10, φ 18 after the CORDIC took it, the
first soft value 8, then one a clock). Registers start random (`+verilator+rand+reset+2`).

The CORDIC's arbiter is the bench's too (`cordic_hold`: half the requests wait 1 to 30
clocks, as when the tracker has it), and one aborted packet in three is aborted in the third
clock a request waits.

`rx_symbol G3: 127 packets x 2 passes, 237 completed, 833 symbols, 870 pilots, 848 phases, 40577
soft subcarriers, 36 aborts (13 resets, 19 replayed), 16032 bins held, 78 scales late, 157
SIGNAL thetas before the pilots, 428 CORDIC requests held (11 aborted waiting), 273936 edges, 0
mismatches` — PASS. Lint clean (`make -C hw lint`). Commit 18fa0d3, `scripts/check.sh` all green.

**A defect the gate found in the first run**: every 16- and 64-QAM threshold was 0. Both
`rx_channel_pkg` (the tracker's K = 16) and `rx_symbol_pkg` declared `K_FRAC`; `rx_symbol`
imports both, and Verilator resolved the name to the first import's without a warning (the
standard makes it an error). The constants are now `KAPPA_BITS`/`KAPPA_FRAC`, and a test of
`rvec` (`generated_packages_declare_distinct_names`) fails when two generated packages declare
one name differently (it fails on the old name).

**Mutations** (each on a private copy of `hw/`, the full gate). `cordic_iter` (its own gate):
the iteration count, the half turn, the drop, `busy`, the sign test of y, the shift of x, the
zero flag, the counter's start — **8 of 8 fail**. `rx_symbol`, the final run on the final bench:
the products' rounding (round to odd) and saturation, the product's sign, the pilots' base sign
(as written and as re-read), the polarity's wrap at 127, the half table step, the phase's sign,
φ in the pilots' phase, the quadrant fold, the rotation's sign, the second fold, the lanes'
order, the soft rounding, the clip (at 126), n + F − 1, the scale's three waits (q of the
modulation, `mod_ok`, BPSK's q standing in), SIGNAL's modulation, θ's bypass and buffer, the
pilots' count, the sum's reset, the slot emptied, every reset of a drop (slots, stage valid,
`p_wait`, the channel, the scales, the modulation, θ, the first symbol, the fill count, the
polarity, the engine, the CORDIC request), the slot of the store's word (`rsel` for the
pipeline's tag: only the last subcarrier differs), the products' order, the Hs and g addresses,
`bins_ok`, the last product, `pil_valid` a product early, the soft pass's length, the second
threshold's constant, `soft_last`, `p_wait` never set, SIGNAL's flag, the request not cleared
when taken, the pilots' imaginary sign; and two checks of the new assertions (a bench giving θ
a cycle before its pilots' `pil_valid`; a shift bound of 2) — **53 of 54 fail**.

How the bench got there: the first run, 50 of 51 failed, seven of them only by a lint warning
(each rewritten as a twin that compiles and fails in simulation, all seven now fail). The one
that passed: **the CORDIC request not cleared by a drop** — the bench's CORDIC was always free,
so a request lived one clock and no abort met it. The bench got the arbiter (`cordic_hold`) and
the abort while a request waits; it fails now. **Equivalent, argued and found by the review**:
`th_cur` taken from θ's input whenever θ comes, instead of from the buffer when one waits —
different only when a data symbol's θ comes in the cycle the previous symbol takes its buffered
θ, which needs θ before its own pilots; the header now forbids that and an assertion stops
simulation on it (its check above fails).

### The review (fresh agent, 2026-11-24)

It reran every gate, regenerated the vectors and the package byte for byte, rebuilt every width
and bound by hand (none can overflow on the inputs' formats; the products' narrowing is exact
for every shift up to 31), the rounding at each step against the model, the phase modulo 2^P,
the polarity sequence against the scrambler and the standard, the lanes' order, the slots'
collisions, the θ buffer, every latency of the header by its own count, and killed ten mutants
of its own. **No defect in the RTL.** It found, and this entry fixed:
- the G3 evidence above was from the bench before the CORDIC's arbiter and had no commit: now
  the current output, the commit below;
- one order of θ the header allowed and the bench never drove (the equivalent mutant above):
  now forbidden and asserted;
- no G3 mutation record: above;
- "Not decided here" counted the offsets from the engines' start instead of the triggers: fixed;
- **the package test let a name appear twice with the same declaration** — still ambiguous
  under IEEE 1800, and `W_BITS` was in both `fft64_rom_pkg` and `rx_channel_pkg`: the test now
  refuses every duplicate, and the FFT's width is `FFT_W_BITS` (`fft64` gate rerun: 0
  mismatches);
- `SOFT_BITS` and `SQ_BITS` were written as numbers in the generator: now derived (soft_max,
  2·A); the input ranges (`chan_shift` ≤ 19, n in 1 … 21) are asserted; the header says the
  CORDIC's answer must be routed by the arbiter (it carries no tag).

## 2026-11-25 · G3 · the slot's and Hs's reads registered (the founder's choice)

**Why.** With the stores built as the PDK's two-port SRAM macros (`silicon_probe.md`,
2026-11-25), a macro's read takes 3.22 ns. This block multiplied the slot's and `Hs`'s words
in the cycle after the read, as `ram_1w1r`'s latency of 1 allows, and that path missed
160 MHz by 3.29 ns. The founder chose on 2026-11-25 to register those two reads here,
rather than lengthen every store's read.

**What changed.** A stage `pq` holds Z and Hs between the read and the four products. The
products pipeline is one edge longer, so **pil_valid comes 10 edges after its triggers**
(it was 9). Every other latency in the header is counted from pil_valid or from the last
product written, so it moves with it. Values are unchanged.

| Gate | Command | Result |
|---|---|---|
| G3 | `make -C hw rx_symbol` (`kPil` 10) | 127 packets × 2 passes, 821 symbols, 40 462 soft subcarriers, 0 mismatches |
| The data path | `make -C hw rx_data` on regenerated vectors | 58 runs, 0 mismatches |

**Mutants**, each in a private copy:
- The first three made a signal unused and died only at lint (`-Wall`): pq bypassed in the
  valid chain, bypassed in the index chain, and `hs_rdata_re` replaced by `_im`.
- With `UNUSEDSIGNAL` off in that copy, the gate itself kills each of these:

| Mutant | Mismatches |
|---|---|
| `pq_v` from the issue (data a cycle late) | 1 |
| `pq_k` from the issue counter | 4108 |
| Z's I and Q swapped into `pq` | 72 382 |

## 2026-11-27 · G3 · the six |m|·q products in modules of their own (no change of function)

Through the open flow with the stores as SRAM macros, once `rx_channel`'s products were
mapped by themselves, the one endpoint short of 160 MHz was this block's `r5_m` →
`r6_mag` (−0.12 ns; `silicon_probe.md`, 2026-11-27, the second entry). Stage r6's six
products are now instances of `hw/rtl/lib/absmul_alone.sv` (`y = |m| · q`, kept as a module
through synthesis); the negation's bound (|m| ≤ 2^(U−1), G0) is unchanged and stated at the
instances. Nothing else changed: no latency, no width. At branch commit 8f9cbb8:

| Gate | Command | Result |
|---|---|---|
| G3 | `make -C hw rx_symbol` | 127 packets × 2 passes, 821 symbols, 40 462 soft subcarriers, 0 mismatches |
| G3 | `make -C hw rx_data` | 58 runs, 590 038 cycles, 189 793 beats, 0 mismatches |
| lint | `make -C hw lint` | clean |

**Mutants**, each restored by copy; each fails `make -C hw rx_symbol`:
- `absmul_alone`: `~m` for `−m` (the magnitude one short for a negative value);
- `absmul_alone`: `q >> 1` for `q`;
- `rx_symbol`: the six instances fed `r5_m` in reverse order.
