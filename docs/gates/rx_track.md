# Gate journal: `rx_track`

Block: the timing tracker, R0-09's data path, ADR 0010 decisions 1 and 6.2 ("the tracker"),
with the data path's shared units: the iterative CORDIC (`cordic_iter`, now behind
`cordic_share`) and the serial divider (`div_serial`, now behind `div_share`). Model:
`radio_phy::rx_ofdm_fixed::FixedTracking::{from_cfo_word, unknown, step, move_window}`. `step`
is `track_fixed` with the packet's constants as an argument, and `move_window` is the window's
move that `symbol_with` made inline. Both were split out without a change of behaviour: every
`radio-phy` test passes as before. G1/G2 of the integer tracker are in `docs/gates/rx_ofdm.md`.
RTL: `hw/rtl/rx_track.sv`, `hw/rtl/cordic_share.sv`, `hw/rtl/div_share.sv`. Gate:
`make -C hw rx_track`.

## 2026-11-24 · G0 · the specification and the architecture

**One shared multiplier: decided by the founder, 2026-11-24** (the recommendation). The
alternatives were a multiplier for every product (some ten to fourteen of them, θ about ten
clocks sooner) and the pair products on the symbol block's products unit (the schedule model's
assumption, which would reopen `rx_symbol`).

### What the block computes (fixed by the model)

Formats: P = 22, R = 10, K = 16, V = 10, A = 16. Per packet:
- **rate0** = CFO word (P bits) × ratio (16 bits) / 2^12, half to even, with the prior's weight
  of a known offset. A ratio of 0 (no register written) gives rate0 = 0 and the weight of an
  unknown offset. The two weights are ROM constants (`rx_track_pkg`: 25 938 223 and 64 846).
  rate0 always fits its 26 bits, so the model's saturation never acts.
- **SIGNAL's θ** = rate0·7 / 2^R, half to even, saturated to P + 1 bits, together with its move.
  δ is 0 before SIGNAL and after it: with one measurement the fit's numerator is 7z − 7z (ADR
  0010, decision 3).
- **From the channel block's quotients** (the outer and inner coefficients, below 2^14; the
  variance, 12 bits): λ = weight·variance / 2^V, half to even. The packet measures when
  0 < variance ≤ 3·2^V.

Per symbol i (0 for SIGNAL, k + 1 for data symbol k, at most 1366; τ = 7 + 5i), from its four
pilots (17 bits each, signs removed: the symbol block's `pil_valid`):
- The pair products p₃·p₀* and p₂·p₁*: 34-bit parts, the CORDIC's input. Their angles are
  widened by 2^(P − A), and the predicted slope's turn (42 and 14 times the prediction) is
  subtracted modulo 2^P. **Only the prediction's low P bits matter**, so the turn is made by
  shifts and adds (42 = 32 + 8 + 2, 14 = 8 + 4 + 2).
- e = (co·∠o + ci·∠i) / 2^K, half to even. It is 38 bits before the shift; |e| < 2^(P−1), so
  no saturation is needed.
- When the packet measures:
  - z = e + δ·τ / 2^R (half to even), saturated to P − 2 bits;
  - the sums n, Στ, Στ², Σz, Στz;
  - det = n·(Στ² + λ) − (Στ)², at most 47 bits and never negative;
  - num = n·Στz − Στ·Σz, 55 bits signed;
  - δ = num·2^R / det, truncated toward zero.
- θ = drift + the window's moves·2^(P−6), saturated to P + 1 bits. The drift is
  (rate0 + δ)·τ / 2^R, half to even, saturated to P + 1 bits. The move is a sample earlier
  when θ > 2^(P−7) and later when θ < −2^(P−7).

**Three of the model's saturations never act**, and the RTL leaves them out:
- **δ never saturates** (`tests/g0_rx_track.rs`, `the_fit_never_saturates`). For n
  measurements at τ = 7, 12, …, |num| ≤ 2^19·Σ|nτ − Στ| (z is saturated) and
  det ≥ n·(Στ² + λ_min) − (Στ)². λ_min = 63 is the smaller weight times a variance of 1. The
  largest quotient is at n = 4, about 5.8·10⁷ against 2^26, so 26 quotient bits always suffice.
  The model's δ register is 28 bits and the RTL's 27.
- **The sums** (n, Στ, Στ², Σz, Στz) cannot overflow in a frame of at most 1367 measurements,
  as the model's own doc says. The emulation asserts their widths at the longest frame.
- **rate0** always fits its 26 bits (above).

The window's shift is 8 bits: the drift saturates at 2^P (64 samples), and a shift follows it
to at most 65 (the longest frame at 1200 ppm reaches 64).

### The architecture

- **One multiplier**, 24 × 44 bits signed. Its operands are registered on the issuing edge and
  its product on the next, so a product issued in cycle c is read in c + 2. All 17 products of a
  step go through it, plus two per packet:
  - eight for the pair products;
  - the predicted drift, δ·τ and τ²;
  - λ (recomputed every step, which keeps the timeline fixed);
  - n·(Στ² + λ) and (Στ)²;
  - co·∠o and ci·∠i;
  - τ·z, n·Στz and Στ·Σz;
  - the fitted drift;
  - per packet: word·ratio and 7·rate0.
- **A sequencer** of 45 states. In order, a step has:
  - eight states for the pair products, each pair's CORDIC request raised as soon as its parts
    are in;
  - the prediction, δ·τ and τ²;
  - a wait for the three quotients;
  - λ, the sums and the determinant;
  - the angles' products as the angles arrive;
  - e, z, Στz and the numerator;
  - the division;
  - the drift, then θ.
- **The pilots held**: one set of 4 × 2 × 17 bits, free once the step has made its pair
  products (8 edges after it starts). SIGNAL's register is free at most 14 edges after its
  pilots (the start's 5 edges, then the step's 9), and the symbol block's pilots are at least
  48 clocks apart. So the first data
  symbol's pilots, which can come while SIGNAL's step still waits for the quotients (its window
  is known at the first beat), find the register free. Every later symbol's come only after the
  last θ, because its window is placed by it. A first draft held two sets; the mutation run
  showed the second was never used.
- **The divider**: |num|·2^R (64 bits) / det (47 bits), 26 quotient bits. The shared
  `div_serial` is widened to 64/47 bits, from the channel block's 48/24.
- **`cordic_share`**: two requesters, each holding its request until taken. The symbol block's
  common phase (a) has priority, as in the schedule's order. The answer is routed to whoever's
  request the CORDIC took.
- **`div_share`**: the channel block (a) pulses its start as built; when the divider is busy,
  its request waits in a register and goes out on the first free edge (or in the same cycle when
  free, so the channel's latencies are unchanged). The tracker (b) holds its request until its
  grant. a has priority.

### Latencies (the RTL's header; every one checked by the bench)

All counts are in edges.
- **θ0**: 5 after start (3 without a ratio).
(Superseded 2026-11-25 by the pipelined multiplier: the latencies below are those before it;
the current ones are in the last section.)

- **A step** starts on T = max(its pilots' edge, the last step's end) + 1. Its outer request is
  up on T + 6 and its inner on T + 10.
- **λ** is asked on C = max(T + 14, the last quotient's edge + 1).
- **The angles' products** go on G0 = max(C + 7, outer + 1) and G1 = max(G0 + 1, inner + 1),
  where outer and inner are the edges the two angles are seen.
- **SIGNAL's step** ends on G1 + 2 (its packet measures nothing) or on G1 + 10.
- **A data symbol whose packet measures nothing** has θ on G1 + 3.
- **A data symbol that measures** has dv_req on G1 + 10; its quotient comes 27 after the grant
  is seen, and θ 5 after that.
- **An angle** is seen 18 edges after its take is seen.

With every unit free, a data symbol's θ comes 87 edges after its pilots. Counted from T:
- the outer angle is in on T + 25 and the inner on T + 43 (the inner waits for the outer on the
  one CORDIC);
- the division is asked on T + 54 and its quotient comes on T + 81;
- θ is out on T + 86.

The schedule model (`rx_data`, default allowances) gives about 77 for the same span. Its
allowances (`fit_in` 5, the division 30, `after_fit` 2, the pair products on the products unit)
are replaced with the top level, as with the earlier blocks.

### Evidence

`tests/g0_rx_track.rs` restates the architecture register by register: the multiplier's operand
and product widths, the divider's restoring algorithm with its bound, the turn by shifts and
adds, and every register's width. It runs this beside `FixedTracking::step` / `move_window` and
requires the same θ, δ and shift after every symbol.
- **Packets on a channel**: 400 packets of 1 to 300 symbols, ±300 ppm, the CFO word near the
  offset or no ratio, a pilot lost now and then.
- **At the limits**: 600 packets with the coefficients and the variance at and inside their
  ends, the CFO word and the ratio at theirs, and pilots on the rails or anywhere. Some reach
  |δ| > 2^25.
- **The longest frame**: six frames of 1367 symbols at ±200 to ±1200 ppm, with the smallest and
  the largest λ.
- **`the_fit_never_saturates`**: the bound above, for every n.
- **`widths_are_the_defaults`**.

Commit: 22f7d8b. Command:
`cargo test --release -p radio-phy --test g0_rx_track`. Result: 5 passed.

**Mutants of the model** (`crates/radio-phy/src/rx_ofdm_fixed.rs`): 16 of 16 fail the G0 tests:
- δ·τ's shift;
- z's width;
- Στz's τ;
- λ in the determinant;
- the outer pair's 42;
- the turn's sign;
- the coefficients swapped;
- the move's threshold (≥);
- rate0's shift;
- λ's shift;
- the drift's saturation;
- the moved shift;
- the variance's bound (<);
- Στ's τ;
- θ's saturation;
- δ's saturation narrowed by two bits.

## 2026-11-24 · G3 · `hw/rtl/rx_track.sv`, `cordic_share.sv`, `div_share.sv`

`make -C hw rx_track`: `track_chain` connects the tracker with `cordic_share` → `cordic_iter`
and `div_share` → `div_serial`, all on one drop. The bench (`hw/tb/tb_rx_track.cpp`) replays
`hw/vectors/rx_track.txt` (rvec `rx_track`, with 400 CORDIC cases for the other requester):
- 110 packets on a channel, 1 to 300 symbols at ±300 ppm;
- 40 at the limits;
- three of 1367 symbols at 1200, −700 and 200 ppm.

157 packets in all: the three groups above, plus four whose θ lands exactly on a move's
threshold (±2^15, which moves nothing): SIGNAL's from the CFO word, and a data symbol's found by
a search in a packet that measures nothing. Every θ and every move is checked against the model, each in the edge of the header's formulas.
The bench is the tracker's surroundings:
- **The quotients** come at random times, a third of them up to 400 clocks late, with the jobs
  the tracker ignores mixed in.
- **The pilots**: SIGNAL's come any time after the start, the first data symbol's at least 48
  clocks after them (often while SIGNAL's step still waits), and each later symbol's only after
  the previous θ.
- **The other CORDIC requester** has priority, sparse or back to back per packet; every answer
  is checked against `cordic_angle`.
- **The other divider requester** issues random operands within the divider's bound, sparse or
  dense per packet; every quotient is checked. Its operands carry garbage outside its start
  cycle, so a waiting request must come from `div_share`'s register.
- **Packets are dropped at their end.** One in five is aborted at a random edge or on the third
  cycle a tracker request waits; one abort in four is by reset. Nothing may come out after an
  abort, and half of the aborted packets are replayed. The bench refuses to pass if any of
  these paths is never taken.

Result (commit 22f7d8b):

> rx_track G3: 157 packets x 2 passes, 281 completed, 20321 symbols, 20404 thetas (18549 fitted,
> 1515 unmeasured; 612 moves early, 490 late), 63 SIGNAL steps unmeasured, 208 first data pilots
> held behind SIGNAL, 142 steps waiting for the quotients, 54734 noise quotients, 506444 tracker
> CORDIC waits, 64656 tracker divider waits, 36593 other angles, 40446 other quotients (7343
> behind the tracker), 60 aborts (16 resets, 29 waiting, 27 replayed), 2185694 edges,
> **0 mismatches**

Besides θ and the move, the bench compares δ after every symbol (`fit_delta`: an error of one
in δ moves θ by only τ/1024). It checks the inner CORDIC request's edge, and the other divider
requester's quotient edge: at once when the divider is free (so the channel's latencies are as
before), and on the tracker's done when it waited in `div_share`'s register (7 343 times).

**Mutants** (each on a private copy of `hw/`, the whole gate rebuilt). The first run had 72
mutants, of which 67 failed. Five of those 67 failed only by lint (a signal left unused); each
was replaced by a functional twin, and all five twins fail in simulation. The five survivors
were findings, and each was fixed:
- **Three move-threshold mutants** (`>` → `≥` and `<` → `≤`, on SIGNAL's θ and on a data
  symbol's) survived because no vector had θ exactly at ±2^15. The four threshold packets were
  added, and all three now fail.
- **The second pilot set was never used** (a mutant of its tail pointer survived), so it was
  removed. Three mutants of the single register fail.
- **The branch for det = 0 is unreachable**: λ ≥ 63 makes det > 0 for every data symbol whose
  packet measures. It was removed and replaced by an assertion.

Against the final RTL: **74 of 75 fail**. The kinds:
- rounding: the half and sticky bits, the saturation values;
- rate0's shift and SIGNAL's 7;
- every pair product's operands and signs;
- the prediction without δ, δ·τ, the move in the prediction, the turn's 42 and 14 and its sign;
- z's saturation;
- the variance's bounds, the weights swapped, the wait for the variance;
- the sums' increments, λ's shift, λ in the determinant, the determinant;
- each coefficient, the waits for each angle, e's shift;
- Σz, Στz and the numerator;
- SIGNAL's skipped division, the divider's numerator shift, δ's sign, the drift's update, the
  fitted rate;
- the moves;
- the step's end and τ's step;
- the CORDIC's request order and the answers' order;
- the grant clearing the request;
- the start's weight and its path;
- the sums' reset;
- in `div_share`: the priority, the pending request (each operand), the routing of done;
- in `cordic_share`: the priority, the owner, the routing, the busy.

Some of these fail on the blocks' simulation assertions (a third angle, a second request, an
input while busy, the divider's bound) rather than on a value. The survivor is
`d_num = pend ? p_num + 1 : …` in `div_share`: it changes floor(num/den) only when num + 1
crosses a multiple of den, rare on random operands, and is nearly equivalent. Its stronger
twins (the live operand in place of the register's; a bit flipped) fail.

### Review (independent, 2026-11-24)

**No defect in the RTL.** The reviewer reran the gate, the G0 tests and lint. It confirmed:
- the proof that δ never saturates (`the_fit_never_saturates`);
- the shift's bound, |shift| ≤ 64 even when δ jumps;
- the remaining widths;
- that one pilot register is enough;
- the header's latencies (θ on T + 86 with every unit free);
- the arbiters' owner changes in the answer's or done's cycle;
- that the model's refactor preserves behaviour.

Findings and what was done:
1. **δ was not compared.** It is now an output (`fit_delta`), checked after every symbol.
2. **A start coinciding with a drop or a quotient** would be lost or would clear the quotient.
   The header now states the contract, and both cases are asserted.
3. **The inner request's edge was not checked.** It is now. A mutant moving the request a clock
   later fails, where before it would have survived.
4. **The coverage of `div_share`'s pending register counted the wrong thing.** It now counts the
   requests that waited behind the tracker's division, and checks their edges.
5. **"The channel's latencies are as before" was untested.** The other requester's quotient
   edges are now checked; see the G3 note above.
6. **The other CORDIC requester never asks in its own answer's cycle.** Not changed: it asks on
   the next clock at the earliest, and `cordic_share` takes either requester on the answer's
   edge (the tracker does so every time it waits).
7. **Nits fixed:** the single-register argument rephrased (SIGNAL's register is free at most 14
   edges after its pilots); the Makefile's comments put back in order. `move_window` clamps P
   to 8…40 as the rest of `FixedTracking` does, which is identical for every valid
   configuration.

After the fixes the survivor `p_num + 1` in `div_share` still survives (nearly equivalent; its
stronger twins fail). The two new mutants (the inner request moved to A10; the pending request
never granted) fail.

### Open, for the top level

- The schedule model's allowances: replace `fit_in`, the division and `after_fit` with the RTL's
  timeline, and move the pair products from the products unit to the tracker.
- `div_share` in front of `rx_channel` at the top level (its gate still drives `div_serial`
  directly at 48/24), and `cordic_share` in front of `rx_symbol` (its bench's `cordic_hold`
  stands for it).
- The top level drops at every packet's end: a start is legal only while the tracker is quiet
  (asserted).
- `rx_track`, `cordic_share`, `div_share` and the widened divider through the open flow. The
  24 × 44 multiplier between two registers is the path to watch at 160 MHz.

## 2026-11-25 · G3 · the multiplier pipelined (the founder's choice)

The whole data path through the open flow (`silicon_probe.md`, 2026-11-25) missed 160 MHz by
1.77 ns. Its worst path was a single-cycle product in this block. The founder chose on
2026-11-25 to pipeline both products, accepting the extra latency and a restated schedule.

**What changed.** The operands are still registered on the issue edge. The next edge
registers the two halves of MA times MB: ma = hi·2^12 + lo, with lo unsigned. The edge after
that registers their sum. Every read of `mp` moves one state later, so a product is read three
states after its issue, where it used to be read two after. The new states are K1W, K4W, A13,
C7, C8, G4, F8, F9 and D5, plus waits where a product feeds the next operand.

The latencies, from the header and the bench (`tb_rx_track.cpp`):

| Edge | Before | After |
|---|---|---|
| θ0 | 5 | 7 |
| θ0 with no ratio | 3 | 4 |
| outer request | T + 6 | T + 7 |
| inner request | T + 10 | T + 11 |
| λ at C | T + 14 | T + 15 |
| G0 | C + 7 | C + 9 |
| SIGNAL's step ends | G1 + 2 / + 10 | G1 + 3 / + 13 |
| θ with nothing measured | G1 + 3 | G1 + 4 |
| dv_req | G1 + 10 | G1 + 13 |
| θ after dv_done | 5 | 6 |

The values are unchanged.

| Gate | Command | Result |
|---|---|---|
| G3 | `make -C hw rx_track` | 157 packets × 2 passes, 21 024 θ, 0 mismatches |
| Lint | `make -C hw lint` | clean |

**Mutants**, each in a private copy, all killed:
- skip D4's wait: 62 843 mismatches;
- the high half shifted by MAL − 1;
- the low half signed;
- skip K1W;
- skip C6's wait.

The schedule restated: `rx_data.md`, same date.
