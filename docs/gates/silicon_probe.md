# Gate journal: silicon probe (R0-14)

Block: the rehearsal of the R5 flow on the open PDK (IHP SG13G2, 130 nm): R0-14 (a) takes
G3-passed blocks through the open flow for area and timing, with no fab. This journal holds
that evidence. Nothing here is a tape-out: (b), a shuttle submission, is a risk point and has
its own ADR to come.

## 2026-11-19 · probe · the receiver-front blocks through OpenROAD-flow-scripts

**What was run.** Each block on its own through OpenROAD-flow-scripts, platform `ihp-sg13g2`
(typical corner, 1.2 V, 25 °C): yosys synthesis with its SystemVerilog front end, floorplan at
40 % utilisation, placement, clock tree, routing, final report. One clock at 6.25 ns
(160 MHz, the R = 8 of ADR 0006), inputs and outputs constrained at 20 % of the period.
- Script: `tools/syn/orfs_block.sh`. Image `openroad/orfs:latest` as found on the build server
  (image 1671deadac53, built 2026-07-16; OpenROAD 26Q3-528-g20d2d5c16e, Yosys 0.67).
  Sources: `main` at 6f4a2f6 plus this entry's change to the simulation checks.
- Run by hand on the build server (`/data/radio-chip/asic`); no gate reruns it. The server
  was shared with a long simulation, so run times mean nothing.
- No pads, no macros, no sign-off checks beyond the flow's own. Memories are flip-flops.

**Results**

| Block | Cells | Flip-flops | Area after synthesis | Design area (at 40 % utilisation) | Worst slack at 6.25 ns | Stage of the figure |
|---|---|---|---|---|---|---|
| `scrambler` | 50 | 9 | 913 µm² | 1067 µm² (fixed 160 µm die) | +4.44 ns | routed |
| `dc_block` | 1827 | 83 | 0.022 mm² | 0.022 mm² | +1.44 ns | routed |
| `nco` | 6505 | 194 | 0.074 mm² | 0.075 mm² | +0.91 ns | routed |
| `cordic` | 7742 | 797 | 0.110 mm² | 0.115 mm² | +1.13 ns | routed |
| `rx_latch` (with its CORDIC; delay line as flip-flops) | 35 654 | 6030 | 0.68 mm² | 0.71 mm² | +1.21 ns | routed |
| `det_a` | 104 426 | 2166 | 1.16 mm² | 0.81 mm² | −2.46 ns | routed |
| `ltf_corr` | 154 776 | 7932 | 1.67 mm² | 1.22 mm² | +1.42 ns | routed |

(The flow reports "design area" as the cells' area after its own optimisation; for the two
largest blocks it is below the synthesis figure, which counts the cells yosys mapped before
the flow resized and removed buffers.)

**What it says**
- The small blocks close 160 MHz on 130 nm with 0.9 to 1.4 ns to spare.
- `det_a` does not: 2.5 ns short, about 115 MHz. The path is the one Vivado named
  (stage 3: the squares and P1·P2 into the constant multiply and the 80-bit compare). The
  block is 104 000 cells for one comparison per sample.
- `ltf_corr` closes, which it does not on the FPGA at the same
  clock (−1.9 ns, `rx_front.md`): a tree of adders in gates is not a carry chain in LUTs.
- **Area is the finding.** The receiver front's blocks so far are about 3 mm² of cells at
  130 nm (`ltf_corr` 1.2 to 1.7, `det_a` 0.8 to 1.2, `rx_latch` 0.7, the rest 0.2), before
  the decision engine's second correlator (ADR 0006, another `ltf_corr`), the history memory,
  and everything after acquisition. A Tiny Tapeout tile is 0.016 mm². This is the review's
  finding 4 in square millimetres: the multipliers of `ltf_corr` and `det_a` are used one
  clock in eight. It belongs in ADR 0008 (proposed in ADR 0007, decision 2) before the
  decision engine is built.
- `rx_latch` is 6030 flip-flops, 4672 of them the 73 × 64-bit delay line. As a two-port
  macro (`memory.md`, 2026-11-19) it would be a fraction of that.

**What the open flow refused, and what was changed**
- **Procedural `$error`.** The flow's SystemVerilog front end rejects `$error` inside
  `always` blocks. The simulation checks of `ram_1w1r` (collision) and `rx_latch`
  (`CHECK_TIMING`) said "synthesis drops them"; this synthesis does not. They are now inside
  `` `ifndef SYNTHESIS ``. Verilator does not define it, so the gates keep the checks: `make
  -C hw ram latch` unchanged, and D = 3 at R = 6 still stops on the timing check.
  Elaboration-time `$error` in generate blocks (parameter checks) is accepted.
- **Memories as flip-flops.** The flow stops at 4096 bits of inferred memory; `rx_latch`
  needs 4672. For this probe the limit was raised; the real answer is the macro.
- **A block too small for the power grid.** The scrambler at 40 % utilisation is narrower
  than the platform's top-metal straps; it was given a fixed 160 µm die.

**Power** (the flow's estimate at the typical corner with its default activity, 160 MHz; a
ranking, not a budget): `ltf_corr` 650 mW, `det_a` 191 mW, `rx_latch` 125 mW, `nco` 74 mW,
`cordic` 33 mW, `dc_block` 5 mW, `scrambler` 0.1 mW. About 1.1 W for these blocks at 130 nm,
most of it in the correlator's multipliers, which do useful work one clock in eight. For a
radio that flies on a drone this is the second reason, after area, to fold them.

**Open**
- All seven figures are now from routed designs (the table first held global-route estimates
  for the three large blocks: +0.94, −3.22 and +0.95 ns).
- `conv_encoder` (merged after these runs started) and `round_sat` (combinational: it needs
  a registered wrapper) are not in the table.
- One corner, one clock, default flow settings: this ranks the blocks, it does not sign
  anything off.
- Ring oscillators and the Tiny Tapeout harness (R0-14 (a)) are not started.

## 2026-11-21 · probe · the search correlator folded over 8 clocks

**What was run.** `ltf_corr` as folded for ADR 0008's decision 4 (`rx_front.md`, the same
date; sources at 9a2512e) through the same flow, image, platform, corner, clock (6.25 ns)
and settings as the entry above (`tools/syn/orfs_block.sh`'s configuration, 40 %
utilisation). By hand on the build server (`/data/radio-chip/asic/ltf_corr_fold`), which
was running a simulation on every core at the time; no gate reruns it.

| `ltf_corr` | Cells | Flip-flops | Area after synthesis | Design area, routed | Worst slack at 6.25 ns | Power (flow's estimate) |
|---|---|---|---|---|---|---|
| unfolded (the entry above) | 154 776 | 7932 | 1.67 mm² | 1.22 mm² | +1.42 ns | 650 mW |
| folded over 8 | 38 918 | 4507 | 0.60 mm² | **0.58 mm²** | **+1.23 ns** | 110 mW |

**What it says**
- The fold halves the block, it does not take it to ADR 0008's estimate of about 0.4 mm²
  ("the window plus an eighth of today's logic plus an allowance"): 0.58 mm² routed. Of
  the cells after synthesis 0.22 mm² are flip-flops (3072 of the window and 1435 others; the
  pipeline and control declare 1386: the selected entries 384, the multiplicands and taps
  288, 16 products 352, the sums 234, accumulator and output 96, control 32; where the
  other 49 come from was not looked up) and 0.09 mm²
  multiplexers (3567 two-way, most of them the window's shift enable, and 573 four-way, the
  slots' selection); the rest, about 0.29 mm², is the 16 multipliers, the adders and the
  tap table. The estimate left out the pipeline's registers and the selection.
- It closes 160 MHz with 1.2 ns, as the unfolded block did; on the FPGA part it now closes
  too (+1.80 ns, 20 DSP blocks, `rx_front.md`).
- The power figure is the flow's default-activity estimate, a ranking as before; 44 % of it
  is in the flip-flops and 26 % in the clock tree, so the window, which shifts one clock in
  eight, is where clock gating would act. No switching simulation has been run: ADR 0008
  signs no power target before one.
- Against ADR 0008's working target (one chain's front ≤ 2.5 mm²): this block is 0.18 mm²
  above its line in the estimate; the decision's correlator and `det_a` are still estimates.

**Open**
- The window as a shift register costs a multiplexer per bit and clocks 3072 flip-flops; a
  gated clock for it, or a memory with the 8 reads a clock that the slots need, has not
  been tried.
- One corner, default settings, no sign-off, as before.

## 2026-11-23 · probe · the Viterbi decoder (register exchange, a step a clock)

**What was run.** `hw/rtl/viterbi.sv` (R0-10, `viterbi.md` of this date; ADR 0010
decision 4 asked for this run before the blocks around the decoder) through the same flow,
image (1671deadac53), platform, corner, clock (6.25 ns) and settings as above
(`tools/syn/orfs_block.sh`, 40 % utilisation at the floorplan). By hand on the build server
(`/data/radio-chip/asic/viterbi`), beside a simulation on every core; about 40 minutes. The
sources were the branch's RTL one step before the version committed: since then the step
index's reset was removed and state 0's metric reset added (7 and 13 flip-flops' reset
logic); not rerun for that.

| `viterbi` | Cells | Flip-flops | Area after synthesis | Design area, routed | Worst slack at 6.25 ns | Power (flow's estimate) |
|---|---|---|---|---|---|---|
| register exchange, window 96 | 54 432 routed (90 973 after synthesis) | 7012 | 1.43 mm² | **0.96 mm²** | **+0.35 ns** | 241 mW |

**What it says**
- **It closes 160 MHz**, by 0.35 ns. The worst path is the add-compare-select loop (from
  a registered branch metric through a state's two sums, their comparison and the
  selection into a metric register); the best-state tree's three stages are not near it.
- **Area: 0.96 mm² routed, against ADR 0010's estimate of 0.5 to 0.7 mm².** Of it
  0.34 mm² are the 7012 flip-flops (5760 survivor bits, 832 metric bits, the tree's 282,
  the flush's 96, the rest input and control), 0.52 mm² combinational cells, 0.09 mm²
  clock and repair buffers. The 0.6 mm² of buffers after synthesis (the survivor bits'
  select drives 103 multiplexers a state) were mostly removed by the flow's repair. The
  estimate counted one multiplexer per survivor bit; there are two, because the cell
  library has no flip-flop with an enable: the survivor and metric registers hold on idle
  cycles through a second multiplexer each (about 6600 of the 13 400 two-way
  multiplexers after synthesis).
- That second multiplexer is what a gated clock for the 6592 survivor and metric
  flip-flops would remove (about 0.12 mm² by the cells' size, and their share of the
  power on idle cycles); clock gating goes through a wrapper in `hw/rtl/lib/` (CLAUDE.md),
  which does not exist yet. Not tried.
- The power figure is the flow's default-activity estimate, a ranking: 54 % combinational,
  30 % flip-flops, 17 % clock tree. Every survivor bit can toggle on every step, so this
  block's real figure needs a switching simulation more than most.
- The traceback memory of ADR 0010's alternative was not built; its "perhaps a third of
  the area" is still unchecked.
- One corner, default settings, no sign-off, as before.


## 2026-11-25 · probe · the receiver's data path, whole (`rx_data`)

**What was run.** `hw/rtl/rx_data.sv` with every block under it (PR C, #86, at c61b361: the
RTL as committed after its review) through the same flow, image (1671deadac53), platform,
corner, clock (6.25 ns, 160 MHz) and settings as the entries above (`tools/syn/orfs_block.sh`,
40 % utilisation at the floorplan, memories as flip-flops). By hand on the build server
(`/data/radio-chip/asic-top2`; its reports and logs kept in `asic-top2-keep`), about 2.5 hours; nothing else ran there but a first run of the
same top on the RTL before its review (ab2c1fa: σ 4 bits, the verdict's gate; 8 flip-flops and
a gate apart), used here only as a second placement: worst slack −1.17 ns. Its reports were not
kept, so its area is not quoted.

| `rx_data` | Cells | Flip-flops | Area after synthesis | Design area, routed | Worst slack at 6.25 ns | TNS | Power (flow's estimate) |
|---|---|---|---|---|---|---|---|
| the data path, c61b361 | 384 725 after synthesis | 26 493 | 4.61 mm² | **3.77 mm²** | **−1.77 ns** | −83.9 ns | 405 mW |

Flip-flops by block (the final netlist's instance names):

| Block | Flip-flops | What most of them are |
|---|---|---|
| `viterbi` | 7010 | survivors and metrics (its own probe: 7012, 0.96 mm² routed) |
| `rx_symbol` | 5620 | two slots of 64 words (memories as flip-flops) |
| `rx_deint` | 4789 | two banks of 48 × 48 bits (memories as flip-flops) |
| `rx_channel` | 4308 | the channel's and the gains' stores (memories as flip-flops) |
| `fft64` | 2712 | the delay lines (63 words) and the pipeline |
| `rx_track` | 1137 | the shared multiplier's operands, the pilot register, the state |
| the top level's own | 257 | the reader, the packet's state, the FFT's input register |
| `rx_signal`, `div_serial`, `div_share`, `rx_octets`, `cordic_iter`, `cordic_share` | 599 | |

The rows add up to 26 432; the other 61 flip-flops were not attributed (not looked up).

**What it says**
- **Area: 3.77 mm² routed, against ADR 0010's estimate of 1.5 to 2.0 mm² (± 40 %).** About
  12 000 of the 26 493 flip-flops are memories built of flip-flops (`rx_symbol`'s slots,
  2 × 64 × 32; `rx_deint`'s banks, 2 × 48 × 48; `rx_channel`'s stores, 64 × 36 + 64 × 15;
  plus `ram_1w1r`'s registered read words, about 210), out of the three blocks' 14 717: some
  0.6 mm² of flip-flops alone at the probe's 49 µm², before their write multiplexers and read trees. The ADR left them open ("macros or
  flip-flops; the macros' sizes not read from the PDK"); this run is the flip-flop end. The
  next figure to get is the same top with those stores as the PDK's SRAM macros (IHP SG13G2
  has them; the flow places macros, this probe has not tried).
- **It does not close 160 MHz: −1.77 ns, about 125 MHz.** The worst path is `rx_track`'s
  shared 24 × 44 multiplier in one cycle (`ma` → `mp`, 9.38 ns of arrival against 7.60 ns
  required); in the first run, placed differently, it was `rx_channel`'s 37 × 16 product of
  the tracker's constants (`pg` → `po`, −1.17 ns). Both are single-cycle products of wide
  operands in blocks that had not been through the flow on their own (BACKLOG lists them).
  A register in the middle of each product is the obvious fix: one edge more on every step
  that waits for the product (`rx_track` uses its multiplier on consecutive cycles in A0–A7),
  restated in the schedule; not tried.
- **Power: 405 mW** at the flow's default activity, a ranking only: 57 % flip-flops' internal
  power, 38 % the clock tree, 5 % combinational. The memories' flip-flops hold most of the
  time; a switching simulation and clock gating of the stores (the wrapper in `hw/rtl/lib/`)
  are what would make this figure mean something.
- One corner, default settings, no sign-off, as before. 34 nets over the flow's fanout limit
  after repair (no slew or capacitance violations).

**Open**
- `rx_track`'s and `rx_channel`'s products pipelined, the schedule restated, the gates rerun,
  then the flow again.
- The stores as SRAM macros (and the area of the macros themselves from the PDK).
- `fft64`, `rx_channel`, `rx_symbol`, `rx_track`, `rx_deint` and `rx_signal` through the flow
  each on its own, to know each one's share (this run counts flip-flops only).


## 2026-11-25 · probe · `rx_data` again, with the two products pipelined

The founder chose on 2026-11-25 to pipeline the two products that missed 160 MHz. This
probe reruns the same flow, image, platform, corner, clock (6.25 ns) and settings as the
entry above, by hand on the build server. Each run took about 2.5 hours.

| `rx_data` | Flip-flops | Design area, routed | Worst slack at 6.25 ns | TNS | Power (flow's estimate) | Worst path |
|---|---|---|---|---|---|---|
| c61b361 (the entry above) | 26 493 | 3.77 mm² | −1.77 ns | −83.9 ns | 405 mW | `rx_track` 24 × 44 in one cycle |
| 070dad4 (#90: `rx_track` in halves of ma, `rx_channel` in three stages) | 26 802 | 3.79 mm² | −1.71 ns | −82.0 ns | 408 mW | `rx_track` `mb` → `mpl`: a 12 × 44 half-product per cycle |
| d4ba06d (`rx_track`'s first stage in four 12 × 22 products) | 26 827 | 3.76 mm² | **−0.30 ns** | **−0.69 ns** | 405 mW | `fft64` stage 4 `q` → stage 5 `pre`: the twiddle's ROM and complex product in one cycle |

Reports and logs are kept on the server in `/data/radio-chip/asic-pipe-keep` and
`asic-mul4-keep`.

**What it says**
- **Splitting only ma's rows bought 0.06 ns.** A 12 × 44 product still has the full 44-bit
  width and its final adder. Splitting both operands (four 12 × 22 products, then a
  four-term sum) leaves `rx_track` off the worst paths. The latency is the same as #90's,
  so the schedule does not change.
- **`rx_channel`** no longer appears in the worst paths.
- **The data path now misses 160 MHz by 0.30 ns (about 153 MHz).** TNS is −0.69 ns, so only
  a few endpoints fail, all in `fft64`'s stage 5. That stage looks up the twiddle from the
  index and multiplies, in the cycle after stage 4's register. Two fixes are open:
  - register the twiddle one stage ahead, with no latency change (not tried);
  - put a register in the complex product, which adds an edge to the FFT's latency and
    restates the schedule.
- **Area and power are unchanged** (3.76 mm², 405 mW). The stores built of flip-flops are
  still about half of the flip-flops (the entry above).

**SRAM macros**
- **What the PDK offers.** The platform in the image has only single-port macros
  (`RM_IHPSG13_1P_*`: 64×64, 256×48, 256×64, 512×64, 1024×8/16/64, 2048×64, 4096×8/16).
  The smallest, 64×64, is 784.48 × 64.36 µm, which is 0.050 mm².
- **What the stores need.** They are 1-write 1-read (`ram_1w1r`). A count over the
  `rx_data` gate's 57 runs (instrumented in a private copy) shows how often each store reads
  and writes in the same cycle:

  | Store | Same-cycle read and write |
  |---|---|
  | `rx_symbol`'s two slots | 41 520 and 43 296 of about 90 000 writes |
  | `rx_deint`'s two banks | 564 each |
  | `rx_channel`'s `Hs` | 2149 |
  | `rx_channel`'s gains | 0 |

- **The consequence.** Only the gains' store fits a single-port macro as built. The other
  five need a change of architecture (banking, or a slot/bank discipline that never reads
  and writes the same macro in one cycle) before macros can be placed.
- **The size of the prize.** With one 64×64 macro for each of the six stores, the macros
  would take about 0.30 mm² plus halos, against roughly 0.6 mm² of flip-flops alone for
  the same bits, before their multiplexers.

**Open, for the founder**
- `fft64`'s stage 5: register the twiddle a stage ahead (no latency change), or pipeline
  the product (one edge more)?
- The stores: change `rx_symbol`, `rx_deint` and `rx_channel` to single-port access so they
  can be IHP macros, or keep flip-flops and accept the area?


## 2026-11-25 · probe · `rx_data` at 160 MHz, and with the PDK's two-port SRAM macros

These are two more runs with the same flow, image, platform, corner, clock (6.25 ns) and
settings as above, by hand on the build server. Reports are kept in
`/data/radio-chip/asic-fft-keep` and `asic-sram-keep`.

| `rx_data` | Flip-flops | Design area, routed | Worst slack at 6.25 ns | TNS | Power (flow's estimate) | Worst path |
|---|---|---|---|---|---|---|
| 5687060 (#92: `fft64`'s twiddle a stage ahead), stores as flip-flops | 26 891 | 3.74 mm² | **+0.16 ns** | **0** | 437 mW | `rx_symbol` `p1_slot` → `p2_ii` |
| 5687060, stores as SRAM macros (9 macros) | 14 712 | **3.00 mm²** (macros 0.41) | −3.29 ns | −171 ns | **237 mW** | an SRAM macro's read (`rx_symbol`'s slot) into the products in the same cycle |

**What it says**
- **The data path closes 160 MHz at 130 nm** with its stores as flip-flops. This is one
  corner (typical), default settings and no sign-off, as before. It took three changes, all
  without changing a value:
  - `rx_track`'s product in two stages of four 12 × 22 products (#90, #91);
  - `rx_channel`'s product in three stages (#90);
  - `fft64`'s twiddle registered a stage ahead (#92).
- **The PDK has two-port macros, and the stores fit them with no change of architecture.**
  The open PDK (IHP-Open-PDK at e73adb4, Apache-2.0, `ihp-sg13g2/libs.ref/sg13g2_sram`) has
  `RM_IHPSG13_2P_*` with one port per side and one clock on both. That is exactly
  `ram_1w1r`'s contract (`memory.md`, 2026-11-19). The flow's image ships only the
  single-port set, so the entry above wrongly concluded that a change of architecture was
  needed; with the two-port macros none is.
- **How the macros were used.** `tools/syn/ihp_sg13g2/ram_1w1r.sv` gives `ram_1w1r` a body
  for this flow alone: macros of 64 × 32 and 64 × 22 side by side, the cheapest mix for the
  width, with port A writing and port B reading. The macros' LEF, Liberty (typ/slow/fast)
  and GDS are passed as `ADDITIONAL_LEFS`, `ADDITIONAL_TYP_LIBS` (and slow, fast) and
  `ADDITIONAL_GDS` in the run's `config.mk`. The IHP platform reads macro libraries from the
  per-corner variables, not `ADDITIONAL_LIBS`; the first attempt failed in synthesis for
  that reason. RTL and gates are unchanged.
- **What the macros buy:**
  - 0.74 mm² of area (−20 %);
  - 12 200 flip-flops fewer;
  - about 200 mW at the flow's default activity (a ranking only, as before).
- **What they cost.** The macro's read takes 3.22 ns from its clock to `B_DOUT` (typical
  corner). `rx_symbol` multiplies the words it reads from its slots and from `rx_channel`'s
  `Hs` in the next cycle, as `ram_1w1r`'s latency of 1 allows, so that path misses by
  3.29 ns. The other consumers register what they read at once (`rx_symbol`'s gains read,
  `r2_g`; `rx_deint`'s banks behind a lane multiplexer), so they are not in this miss. The
  review of this entry pointed that out.
- **The way out** is a register between those reads and `rx_symbol`'s products: one more
  edge in `rx_symbol`'s latencies, with the schedule restated. A read latency of 2 for every
  store would also do it, but it is more than this run shows to be needed. Whether the
  registered reads then close 160 MHz is a further run.
- **The run.** The configuration is committed as `tools/syn/ihp_sg13g2/rx_data_sram.mk`,
  with the steps to repeat it. The script exited 1 at its very end, after the final report
  `6_report` and the merged GDS were written; the figures are from that report.

**Open, for the founder**
- The stores as macros at 160 MHz: register `rx_symbol`'s slot and `Hs` reads (one more
  edge in `rx_symbol`, the schedule restated, then a run with macros) for 0.74 mm² and about
  200 mW, or keep flip-flops?


## 2026-11-26 · probe · `rx_data` with the SRAM macros and `rx_symbol`'s reads registered

This is the same flow, image, platform, corner, clock (6.25 ns), settings and macro
configuration (`tools/syn/ihp_sg13g2/rx_data_sram.mk`) as the entry above, at 1dd55ed (#94:
`rx_symbol`'s slot and `Hs` reads registered; `hw/rtl` is the same on main at 486311b). The
run was by hand on the build server. Its reports, the final database and the list of failing
endpoints (`failing_endpoints.txt`, from `report_checks -format end -slack_max 0` on
`6_final.odb`) are kept in `/data/radio-chip/asic-sram2-keep`.

| `rx_data`, stores as SRAM macros (9 macros) | Flip-flops | Design area, routed | Worst slack at 6.25 ns | TNS | Failing endpoints | Power (flow's estimate) |
|---|---|---|---|---|---|---|
| 5687060, before #94 (entry above) | 14 712 | 3.00 mm² (macros 0.41) | −3.29 ns | −171 ns | 253 | 237 mW |
| 1dd55ed, `rx_symbol`'s reads registered | 14 794 | **3.05 mm²** (macros 0.41) | **−1.19 ns** (about 134 MHz) | −25.2 ns | 68 | 441 mW |
| 5687060, stores as flip-flops (entry above) | 26 891 | 3.74 mm² | +0.16 ns | 0 | 0 | 437 mW |

**What it says**
- **#94 did what it was for.** `rx_symbol`'s slot and `Hs` reads no longer appear among the
  failing paths. The worst slack goes from −3.29 to −1.19 ns.
- **It still misses 160 MHz with the macros.** The 68 failing endpoints are in three groups,
  all in `rx_channel`, `rx_symbol` and `rx_track`:
  1. **`rx_channel`'s first pass, a read, an add and a write in one cycle** (worst −1.19 ns):
     `Hs`'s macro answers 3.00 ns after its clock on this path (the entry above quotes
     3.22 ns: the read's delay depends on its load). `hs1 = word + L·T2` then goes back into
     the same store (`A_DIN`, 0.35 ns setup) and into `s2_re`/`s2_im`. With flip-flops this
     read was nearly free; with the macro, about 3.4 ns are left after it for the add, the
     write multiplexer and the wires (with this run's clock skew: launch 1.26 ns, capture
     1.79 ns).
  2. **`rx_channel`'s multiplier, the 37 × 8 products `mph`/`mpl`** (to −0.80 ns), and the
     sum `d = 9·no + ni` (to −0.11 ns).
  3. **`rx_symbol`'s `r6_mag`** (to −0.58 ns), `soft_v` (−0.03 ns), and `rx_track`'s `mlh`
     (−0.01 ns).

  Groups 2 and 3 are logic between flip-flops. They passed in the run with flip-flops (worst
  +0.16 ns), so the placement around the macros cost them about 1 ns or more (at least
  0.96 ns on `mpl`). That is one placement on one seed, and it may move from run to run.
- **Area:** 3.05 mm² against 3.74 with flip-flops (−18 %). The 82 flip-flops more than the
  previous macro run are #94's registers.
- **Power:** the flow's estimate is 441 mW, against 237 mW for the previous macro run and
  437 mW with flip-flops. The switching part is 141 mW against 40. #94 adds 82 flip-flops,
  which does not explain it. The clock tree differs visibly: 3 958 clock buffers against
  2 751 in the previous macro run, 148 fanout violations against 24
  (`finish__timing__drv__max_fanout`), and a worst VDD drop of 0.103 V against 0.010 V. That may account for the switching
  part. Until a run with real activity (a VCD from the gate's stimulus),
  this estimate should not be used to compare the runs, even as a ranking.
- **The run.** The script exited 1 at its very end, after `6_report` and the merged GDS were
  written, as before; the figures are from that report.

**Open, for the founder.** With the macros, 160 MHz needs at least the following. All of
these are proposals, and none has been decided:
- **Group 1 (−1.19 ns):** register `Hs`'s read before the first pass's add. That is one more
  edge between a T2 bin and its write. The 52 used bins of a window have distinct addresses,
  so no read meets a pending write. The next packet's first T1 write comes only after the
  block is back in READY, after `chan_valid`, long after the first pass's last write. Whether `chan_valid` moves depends on the slack between the
  first pass's last write and the second pass's first read, which needs checking against the
  gate. The alternative is to keep `Hs` as flip-flops: one store of 64 × 44 bits, about
  2 800 flip-flops.
- **Groups 2 and 3 (to −0.80 ns):** a fourth stage in `rx_channel`'s multiplier (its
  latencies restated or absorbed), and the same question for `rx_symbol`'s `r6_mag`. Or
  first try floorplanning: place the macros at the edge or near their users, by hand
  (`MACRO_PLACEMENT`). That changes no RTL, and it may be enough for the logic paths.
- **Or the stores stay flip-flops for now.** 160 MHz is R = 8 clocks a sample at 20 MS/s
  (ADR 0006, 0008). A slower clock is not a setting: ADR 0006's named fallback, R = 6
  (120 MHz), needs a third read port and L = 71, and every latency in µs grows by 8/6. It
  would be an architecture decision. The flip-flop build closes
  160 MHz today. The macros can be taken up again with the ASIC stage's floorplan, so the
  0.7 mm² is deferred rather than given up.

## 2026-11-26 · probe · `rx_data` with the SRAM macros placed by hand

The founder chose on 2026-11-26 to try placing the macros by hand first, with no RTL change.
This run uses the same flow, image, platform, corner, clock (6.25 ns), settings and RTL as the
entry above (`hw/rtl` on main at ddac8ed is the same as at 486311b). The only change is
`MACRO_PLACEMENT_TCL = tools/syn/ihp_sg13g2/rx_data_macros.tcl` in the run's `config.mk`. The
run was by hand on the build server. Its reports, the final database, the placement script
and the failing endpoints are kept in `/data/radio-chip/asic-mp40-keep`.

**Why this placement.** The automatic placement of the entry above (read back from its
`6_final.odb`) put all nine macros in three rows along the bottom edge, about 80 µm apart.
All of a macro's pins lie on one long edge (526 or 703 µm), so `rx_symbol`'s two slots sat
directly in front of `Hs`'s pins. Also, `Hs`'s high half was mirrored (R180) beside its low
half, so the add's carry from bit 21 to bit 22 crossed about 600 µm. On the worst path two
`o21ai_1` cells each drove about 70 fF, 0.52 ns each (0.75 to 0.8 ns with the buffer that
follows). The script places the macros in one row on the
bottom edge (pins up) and one on the top edge (pins down), so no macro faces another:
- bottom: `Hs` low half, then `Hs` high half, in bit order, then the gains `G`, then
  `rx_deint`'s bank 0;
- top: `rx_symbol`'s two slots, then `rx_deint`'s bank 1.

The positions are computed from the core box. A first attempt put the slots on the right
edge, rotated; the flow's `place_macro` stopped with MPL-0034 ("outside of the core") at a
position that fits only with the unrotated size (that log was not kept). A second run at
`CORE_UTILIZATION = 50` was set up but dropped, because the two rows do not fit that core
without rotation.

| `rx_data`, stores as SRAM macros (9 macros) | Design area, routed | Worst slack at 6.25 ns | TNS | Failing endpoints | Power (flow's estimate) |
|---|---|---|---|---|---|
| automatic placement (entry above) | 3.05 mm² | −1.19 ns | −25.2 ns | 68 | 441 mW |
| **placed by hand** | **2.99 mm²** (macros 0.41) | **−0.92 ns** (about 140 MHz) | **−10.3 ns** | 39 | 380 mW |
| stores as flip-flops (2026-11-25) | 3.74 mm² | +0.16 ns | 0 | 0 | 437 mW |

Flip-flops: 14 794, as in the entry above.

**What it says**
- **The placement bought 0.27 ns on the worst path and more on the logic paths, but not
  160 MHz.** The 39 failing endpoints are now all in `rx_channel` (33) and `rx_symbol` (6).
  `rx_track` no longer appears.
- **`rx_channel`'s first pass is still the worst** (`s2_re` −0.92 ns, `Hs`'s `A_DIN`
  −0.80 ns), but most of the path is now the flow's own repair, not the add. The launch
  clock reaches `Hs`'s `B_CLK` at 1.31 ns, and the macro answers at 4.37 ns (3.07 ns read).
  That leaves 3.12 ns to the required time (7.49 ns). Of the 4.04 ns after the read, about
  2.4 ns are a chain of 37 `buf_1` cells in series named `rebuffer*`, all within about
  25 × 65 µm next to `s2_re[8]`. The add and the multiplexer themselves take about 1.65 ns,
  so without the chain this path would pass by about 1.4 ns. The chain is not for hold: the
  endpoint's hold slack is +0.55 ns, and the repair after global routing reports no hold
  violations. That repair (`repair_timing` in `5_1_grt`) inserted 1 864 buffers. The other
  failing endpoints checked carry such buffers too: 39 and 40 on the worst paths to
  `s2_re[8]` and `Hs`'s `A_DIN[4]`, 14 to `s2_re[17]`, 48 to `mpl[20]` (cells on the path). Why the repair builds them,
  and what the path is once they are gone, has not been found yet. The previous run's worst
  path had no such chain.
- **The logic paths moved both ways:** `rx_channel`'s `mpl`/`mph` to −0.74 ns (was −0.80),
  `rx_symbol`'s `r6_mag` to −0.14 ns (was −0.58), but `d` to −0.25 ns (was −0.11). This is one placement on one seed. With flip-flops all of them passed.
- **Power:** the estimate is 380 mW (internal 259, switching 120). As in the entry above,
  it is not comparable between runs without real activity.
- One corner, default settings, no sign-off, as before. There are 15 nets over the flow's
  fanout limit and 4 capacitance violations after repair.

**Open.** First, why the flow's repair builds the buffer chains, and a run without them
(the repair's settings, no RTL change). If the paths then still fail, the founder chooses:
register `Hs`'s read before the first pass's add (whether `chan_valid` moves is to be
checked against the gate), or keep the stores as flip-flops until the ASIC stage.

## 2026-11-27 · probe · `rx_data` with macros: the repair's chains, and `Hs`'s add mapped by itself

Four runs after the entry above and one synthesis-only experiment, all with the same flow,
image, platform, corner, clock (6.25 ns), macros and hand placement. Runs A, B and C change
only the flow's settings (RTL on main at 91c8601). Run D adds one change of RTL with no
change of function: `hw/rtl` as in this entry's PR (branch commit f4a5865; the server's copy
was synchronised from that tree by checksum; later commits of the PR change comments only).
All were by hand on the build server; each run's `config.mk`, reports, logs, final database
and failing endpoints are kept in `/data/radio-chip/asic-rep{A,B,C,D}-keep` (A has no
endpoint list). The static-timing scripts and their outputs for the synthesis comparison are
in `/data/radio-chip/mp` (`s1.tcl`, `s1-base.out`, `s1-add.out`); the experiment's own
synthesis run was not kept, D's `1_synth.odb` is the same design.

| Run | Change against the hand-placed run | Worst slack at 6.25 ns | TNS | Failing endpoints | Design area, routed |
|---|---|---|---|---|---|
| hand-placed (entry above) | — | −0.92 ns | −10.3 ns | 39 | 2.99 mm² |
| A | `SKIP_INCREMENTAL_REPAIR = 1` (no timing repair after global routing) | −1.55 ns | −13.6 ns | 71 | 2.97 mm² |
| B | `SETUP_MOVE_SEQUENCE = unbuffer,sizeup,swap,clone,split` (repair may not insert buffers) | −0.83 ns | −8.4 ns | 35 | 2.98 mm² |
| C | B and `ABC_CLOCK_PERIOD_IN_PS = 3000` | −0.83 ns | −8.4 ns | 35 | 2.98 mm² |
| **D** | B and `Hs`'s two adds each in a module of its own (`hw/rtl/lib/add_alone.sv`) | **−0.37 ns** (about 151 MHz) | **−1.2 ns** | **5** | 3.00 mm² |
| stores as flip-flops (2026-11-25) | | +0.16 ns | 0 | 0 | 3.74 mm² |

Flip-flops: 14 794 in every run.

**What the runs say**
- **The buffer chains were not the main cause (A, B).** With the repair after global routing
  skipped (A) everything is worse, and the worst path (`Hs` → `s2_re[8]`) still carries
  `rebuffer*` cells from the earlier repair steps. With buffer insertion forbidden (B) the
  worst path has 3 buffers in place of 37 and is still `Hs`'s: read at 4.34 ns, arrival at
  `Hs`'s `A_DIN[13]` at 8.69 ns against 7.85 required (−0.83 ns before rounding). The entry above said this path would
  pass by about 1.4 ns without the chain. That was wrong: it took the add's delay from a
  path whose add had been restructured by the repair.
- **The add is a ripple-carry chain, and it leaves synthesis that way.** In B and C the
  4.35 ns after the read are about 3.3 ns of carry (the group `a21oi`, `a21oi`, `a21o`,
  `o21ai`, `inv` repeated along the 18 bits) and about 1.0 ns for the sum bit, the write multiplexer and the
  way to the macro's pin. Static timing on C's `1_synth.odb` (before placement, no wires)
  shows the same chain: 5.0 ns after the read, slack −1.89 ns to `s2_im`. Yosys does read
  the flow's Kogge-Stone carry map (`lcu_kogge_stone.v` in the log); the chain is what ABC
  makes of it afterwards.
- **ABC's clock target does nothing in this flow (C).** The log shows `-D 3000` on the ABC
  command against `-D 6250` in B, and the result is B's to the last digit of the synthesised
  area (3 767 844.78 µm²), slack and TNS. No command in the flow's `abc_speed.script` uses
  the target. So C did not test the explanation it was run for.
- **Alone in a module, the add is mapped fast (the synthesis experiment, then D).** With each
  add in a module of its own, kept through synthesis, the same static timing after synthesis
  shows 13 cells and 1.19 ns after the read, slack +1.89 ns. The explanation that fits both
  results, not shown beyond them: ABC maps a module to the depth of its longest path and
  recovers area on every shorter one, and a macro's output is to it an input at time 0, so
  in the flattened design the add has "slack" up to the depth of the products. Synthesised
  area grows by 1.2 % (3 813 044 against 3 767 845 µm²), more than the two adds themselves
  (2 268 µm² each); where the rest comes from has not been looked at.
- **Routed (D): 5 failing endpoints against 35.** No endpoint behind `Hs`'s read is
  among the twelve worst paths (all others at 0.00 ns or better). The five that fail are all
  `rx_channel`'s `mpl`, the 37 × 8 product of the multiplier's second stage, flip-flop to
  flip-flop: about 40 cells, 6.5 ns, −0.37 ns at `mpl[43]`. Close behind and passing:
  `rx_symbol`'s `r6_mag` (+0.04 ns), `rx_track`'s `y0` (+0.14 ns).
- **Power:** the flow's estimate is 374 mW (A), 376 mW (B, C) and 245 mW (D; switching 45 mW
  against 119). Why D's is lower is not known; as before, it is not comparable between runs
  without real activity.
- One corner, default settings, no sign-off, as before. After D: 21 nets over the fanout
  limit, 3 capacitance violations.

**The change of RTL (D).** `hw/rtl/lib/add_alone.sv` is `y = a + b` with yosys's
`keep_hierarchy` attribute; `rx_channel` uses it for the first pass's two adds. Gates on the
changed RTL: `make -C hw rx_channel` (338 packets × 3 passes, 8293 quotients, PASS),
`make -C hw rx_data` (58 runs, 590 038 cycles, 0 mismatches), `make -C hw lint` clean. The
mutant `y = a − b` fails `rx_channel`. `tools/syn/ihp_sg13g2/rx_data_sram.mk` lists the new
file; D's `config.mk` is that file with the two lines `MACRO_PLACEMENT_TCL` and
`SETUP_MOVE_SEQUENCE` above (both now named in that file's header).

**Open.** A run E is in flight: D with the flow's default repair moves (buffers allowed
again), to see whether the repair closes `mpl` now that `Hs`'s path no longer asks for the
chains. If `mpl` still fails, the choices are the founder's: split the multiplier's second
stage further (a change of RTL, latency to be checked against the gate), or accept about
151 MHz with macros in this flow until the ASIC stage. The rule "an add or a product behind
a macro's read goes in a module of its own" may apply to other stores' reads; not surveyed.

## 2026-11-27 · probe · `rx_data` with macros closes 160 MHz: the products mapped by themselves

See 2026-11-28 below: one of four placement seeds fails this run's constraints (−1.00 ns,
the port path `end_stamp` → `rx_age`).

"Closes" here is what it was in the entries above: the typical corner (1.2 V, 25 °C), one
seed, the flow's default settings, no sign-off.

Runs E to I after the entry above, same flow, image, platform, corner, clock (6.25 ns),
macros and hand placement; from E on with the flow's default repair moves (the line
`SETUP_MOVE_SEQUENCE` of runs B to D removed). E is the RTL of main at f72fbb8. F and G were
run on scratch trees on the build server (`/data/radio-chip/src-mul`, `src-sym`: the same
logic as this entry's PR, with the new modules written inside `rx_channel.sv` and
`rx_symbol.sv` and the product's widths fixed, not parameters). **I is the RTL of this
entry's PR (branch commit 8f9cbb8; the server's copy was made from that tree and is called
`src-pr98`, a name given before the PR had a number: #98 is another PR; later commits of
the PR change comments only)**, with the source list of `tools/syn/ihp_sg13g2/rx_data_sram.mk`.
Each run's `config.mk`, reports, logs, final database and endpoint list are kept in
`/data/radio-chip/asic-rep{E,F,G}-keep` and `asic-{I,G55}-keep`; H2's and G65's directories
(`asic-H2`, `asic-G65`) are left as they stopped. G55, G65, H2 and I were started with
`tools/syn/probe_run.sh` at 8 threads each, the others by hand. H and H2 are apart from the
rest: the RTL of main at 91c8601 (before `add_alone`), with two of the flow's options.

| Run | Change | Worst slack at 6.25 ns | TNS | Failing endpoints | Design area, routed |
|---|---|---|---|---|---|
| D (entry above) | adds alone, repair without buffer moves | −0.37 ns | −1.2 ns | 5 | 3.00 mm² |
| E | D with the default repair | −0.34 ns | −1.1 ns | 4 | 3.01 mm² |
| F | E and `rx_channel`'s two 37 × 8 products each in a module | −0.12 ns | −0.12 ns | 1 | 3.01 mm² |
| G | F and `rx_symbol`'s six \|m\|·q products each in a module | **+0.33 ns** | 0 | 0 | 3.06 mm² |
| **I** | G's logic as committed (8f9cbb8) | **+0.33 ns** | 0 | 0 | 3.06 mm² |
| G55 | G with `PLACE_DENSITY = 0.55` | +0.31 ns | 0 | 0 | 3.05 mm² |
| stores as flip-flops (2026-11-25) | | +0.16 ns | 0 | 0 | 3.74 mm² |

Flip-flops: 14 794 in every run.

**What the runs say**
- **The default repair does not close `mpl` (E).** With buffers allowed again the four
  failing endpoints are all `rx_channel`'s `mpl`, as in D.
- **The product has the add's disease (F).** Static timing after synthesis, before placement
  (`/data/radio-chip/mp/s2.tcl`, `s2-base.out`, `s2-mul.out`): `mqb` → `mpl` is 8.13 ns deep
  in the flattened design (slack −2.00 ns) and 2.98 ns with each 37 × 8 product in a module
  of its own (slack +3.14 ns). Routed, no `rx_channel` endpoint fails; the one that does is
  `rx_symbol`'s `r5_m` → `r6_mag[3][7]`, the |m|·q product, −0.12 ns.
- **With the six |m|·q products in modules as well, nothing fails (G, I).** G's worst paths:
  `end_stamp` → `rx_age` +0.33 ns, `u_sym.cur_down` → `soft_v` +0.34, the Viterbi decoder's
  `metric` → `ma` +0.35 and `bm_pm` → `path` +0.38, `r5_m` → `r6_mag` +0.39. So the margin is
  no longer one path's: five groups sit within 0.1 ns of each other. I, on
  the committed RTL and at 8 threads, is G to the last digit printed: the same slack, worst
  paths, design area (3 056 077 µm²), synthesised area and power estimate.
- **Synthesised area** (µm², yosys): 3 767 845 with nothing wrapped, 3 813 044 with the adds
  (E), 3 823 782 with `rx_channel`'s products (F), 3 760 057 with all of them (G). Why the
  last step makes it smaller has not been looked at.
- **Placement density (G55, G65).** At 0.65 the estimates before routing were worse at every
  stage (−0.78, −0.14, −0.46 ns after placement, clock tree and global routing, against G's
  −0.22, +0.01, −0.16) and the run was stopped there. At 0.55 they were close to G's after the clock tree and
  global routing and 0.13 ns worse after placement (−0.35, 0.00, −0.15), and the run ends at +0.31 ns with other paths on top: `rx_track`'s `err` → `mb`
  (+0.31) and `err` → `y0`, the decoder's `bm_pp` → `path`, the CORDIC's `state` → `x`.
  The repair works until nothing fails and no further, so about +0.3 ns over many groups
  may be where it stops rather than what the design has; not looked at.
- **Power:** the flow's estimate is 245 mW (E), 280 mW (F), 239 mW (G); not comparable
  between runs without real activity, as before.
- One corner, default settings, no sign-off, as before. After G: 20 nets over the fanout
  limit, 2 capacitance violations, no slew, setup or hold violation.

**The common cause, as far as it was looked for.** The flow maps with ABC through
`abc_speed.script` (`&st; &dch; &nf`, then five rounds of `&st; &syn2; &if -g -K 6;
&synch2; &nf`, then `buffer; upsize; dnsize`). No command in it takes the clock target (run
C above), so `&nf` has no target to map to. The reading that fits the test below, not checked against
ABC's source: it maps for the best depth it finds for the module's longest path and recovers
area on every shorter one. A small test (`/data/radio-chip/abct`: `src/t.sv`,
`run.sh`, the script variants in `scripts/`, one synthesis each, static timing before
placement), without any macro:

| Module | ABC script | The 18-bit add, flip-flop to flip-flop |
|---|---|---|
| the add alone | the flow's | 9 cells, 1.14 ns |
| the add beside a 37 × 8 product | the flow's | 25 cells, 2.52 ns (the product: 29 cells, 3.15 ns) |
| the same | only the first `&nf`, no rounds | 18 cells, 1.85 ns |
| the same | the rounds without `&syn2` and `&synch2` | 18 cells, 1.91 ns |
| the same | `&nf -F 1` | 24 cells, 2.41 ns |
| the same | the flow's, with `SWAP_ARITH_OPERATORS = 1` | 11 cells, 1.24 ns |

In this one test module the add is mapped more than twice as slow beside a deeper path that
shares nothing with it. The restructuring rounds account for about 0.6 ns of that; area
recovery for at most 0.1 ns in the one variant that ran (`&nf -F 0`, none at all, ends ABC
with status 86 in this image). That fits `Hs`'s add, where the macro's read (3.07 ns) is
outside what ABC sees. **It is not shown for the products**: `mqb` → `mpl` was 101 cells and
8.13 ns after synthesis in the flattened design, and no deeper path of that design has been
named that would leave it such room (static timing on the synthesised database without
buffering puts the decoder's enables on top at over 100 ns, which says nothing). What is
measured is only that each operator, alone in a module, comes out fast.

**The flow's own option, tried once and not pursued (H, H2).** `SWAP_ARITH_OPERATORS = 1` with
`OPENROAD_HIERARCHICAL = 1` keeps every operator as a module in several architectures and
lets the repair swap them (the flow's documentation calls it in development). No change of
RTL (main at 91c8601). H stopped at macro placement: the synthesised area fell to
2.94 mm² (from 3.77), the core sized by utilisation shrank, and the hand placement's
coordinates fell outside it (`MPL-0034`). H2, with the die and core of the earlier runs
fixed (`DIE_AREA = 0 0 3122.54 3122.54`, `CORE_AREA = 17.5 17.5 3105.04 3105.04`), was
stopped after the clock tree: −7.97 ns after placement, −4.92 ns after the clock tree, 2000
or more failing endpoints (the list is cut there), the worst `fft64`'s `pre` → `fr` at 11.2 ns of logic, then the decoder's
`path` (1448 or more endpoints) — paths that pass by themselves in every other run. The repair's
log has `ODB-0453` (a module cannot be swapped: 68 ports against 54). H2 was never routed, the two
options were not tried apart, and in the small test above the first of them gives the best
add; the smaller synthesised area is unverified.

**Reading a run early.** The worst slack the flow reports after placement, the clock tree
and global routing, against the final one (ns):

| Run | placement | clock tree | global routing | final |
|---|---|---|---|---|
| A | −1.91 | −1.19 | −1.98 | −1.55 |
| B | −1.91 | −1.61 | −1.76 | −0.83 |
| D | −1.06 | −0.84 | −1.05 | −0.37 |
| E | −1.06 | −0.72 | −0.97 | −0.34 |
| F | −0.36 | −0.13 | −0.63 | −0.12 |
| G | −0.22 | +0.01 | −0.16 | +0.33 |

The estimate after global routing is 0.4 to 0.9 ns below the final slack and keeps the order
of all six; the clock tree's inverts A and B; placement's separates only changes of RTL (A
and B, D and E are the same there). They rank variants; they decide nothing. A run took 88
minutes at the server's default thread count (F: synthesis 6, floorplan and placement 18,
clock tree 3, global routing 15, detailed routing 35, the final report 11).

**The change of RTL.** `hw/rtl/lib/mul_alone.sv` (`y = a · b`, unsigned, exact) and
`hw/rtl/lib/absmul_alone.sv` (`y = |m| · q`), both with yosys's `keep_hierarchy`, as
`add_alone` is; `rx_channel` uses the first for the two byte products of its multiplier's
second stage, `rx_symbol` the second for the six of its stage r6. No change of function or
latency. Gates at branch commit 8f9cbb8: `make -C hw rx_channel` (338 packets × 3 passes, 0 mismatches,
PASS), `make -C hw rx_symbol` (127 packets × 2 passes, 0 mismatches), `make -C hw rx_data`
(58 runs, 590 038 cycles, 0 mismatches), `make -C hw lint` clean. Mutants: in `mul_alone`,
`b ^ 1` fails `rx_channel`, and `a` cast one bit narrower is equivalent (a is 37 bits, the
product is still taken in 45); the two instances' bytes swapped in `rx_channel` fail it; in
`absmul_alone`, `~m` for `−m` and `q >> 1` each fail `rx_symbol`; the six instances in
reverse order in `rx_symbol` fail it. 5 of 6, one equivalent.

**Open.** The margin is +0.33 ns at one corner with one seed, five groups of paths within
0.1 ns of each other; a denser placement (0.65) was 0.3 ns worse after global routing. Whether the rule
("an operator that must be fast goes in a module of its own in this flow") should be
applied to the next groups (the decoder's compare-select, `rx_symbol`'s rounding) is not
decided. The power estimate's drop at D and its changes since (245, 280, 239 mW), and
the synthesised area's changes, are unexplained.

## 2026-11-28 · probe · `rx_data` with macros over four placement seeds

Run I of the entry above again with the flow's
`GPL_RANDOM_SEED` at 1, 2, 3 and 4 and nothing else changed: the same RTL (the server's
`src-pr98`, which is main at f0ad070 but for comments), config, macros and hand placement,
typical corner, no sign-off. Started together with `tools/syn/probe_run.sh` at 8 threads
each (`start S<n> I /data/radio-chip/src-pr98 GPL_RANDOM_SEED=<n>`); kept in
`/data/radio-chip/asic-S{1,2,3,4}-keep`. Four at once took 173 to 201 minutes each by the
flow's own count, against about 90 for one alone.

| Seed | After placement | clock tree | global routing | Final worst slack | Failing endpoints | Worst register-to-register path |
|---|---|---|---|---|---|---|
| 1 | −0.22 | +0.01 | −0.16 | +0.33 ns | 0 | +0.34 ns, `u_sym.cur_down` → `soft_v` |
| 2 | −0.29 | 0.00 | −0.07 | +0.35 ns | 0 | +0.35 ns, `u_sym.r5_m` → `r6_mag` |
| 3 | −0.42 | −0.21 | −1.36 | **−1.00 ns** | 1 | +0.30 ns, `u_vit.bm_pp` → `path` |
| 4 | −0.46 | +0.01 | −0.09 | +0.25 ns | 0 | +0.25 ns, `u_vit.bm_pp` → `path` |

**What the runs say**
- **Seed 1 is the default:** S1 is run I to the last digit printed (slack, worst paths,
  design area 3 056 077 µm², power estimate).
- **No endpoint at a flip-flop fails in any seed** (the endpoint lists); the worst
  register-to-register path in each run's list of worst paths is +0.25 to +0.35 ns, and its
  group changes with the seed (the soft values, |m|·q, the decoder's compare-select).
- **Seed 3 fails on one path that is not between flip-flops:** the input port `end_stamp` to
  the output port `rx_age[5]`, through combinational logic only
  (`rx_age = adc_count − end_stamp`, `rx_data.sv`). The probe's constraint file (written by
  `tools/syn/orfs_block.sh`) gives every input and every output 20 % of the period as
  external delay, a placeholder chosen for the probe, not a requirement of the receiver; so
  this path has 3.75 ns, not 6.25. The path is 3.37 ns in S1 (+0.33 ns, also that run's
  worst path) and 4.68 ns in S3, where 50 lines of the path report name cells the flow
  inserted (rebuffer, split, clone, wire); how many cells that is and which step put them
  there was not counted. Only bit 5 of the sixteen fails. Why it is 1.3 ns slower than in S1
  has not been looked at: the pins' positions in S1 and S3 were not compared (the config
  constrains none). The estimate after global routing was already S3's worst by far
  (−1.36 ns); that it is the same path was not checked.
- So the earlier entry's "+0.33 ns" was this port-to-port path, whose budget is the probe's
  assumption. By the two modules' headers `adc_count` and `end_stamp` are both registers of
  `rx_handoff` on the same clock and `rx_age` is meant for the MAC; no top level joins them
  yet and the MAC does not exist, so the real budget on the output side is unknown.
- Design area 3.04 to 3.06 mm², power estimate 239 mW in all four, 19 to 21 nets over the
  fanout limit, 2 to 6 capacitance violations.

**Open (the founder's).** What `rx_age`'s path should be held to:
(a) keep the RTL and take the placeholder out of the probe: the real budget on the three
input buses (`adc_count`, `end_stamp` behind `rx_handoff`'s registers), and pin positions
for them; the output side stays a guess until there is a MAC;
(b) register `rx_age` in `rx_data`. `rx_age_valid` is combinational from `rx_end` and the
age changes every sample, so this either moves the age off `rx_end`'s cycle, against the G0
choice of 2026-11-25, or computes it a cycle early: a change of the golden model's
`rx_end_age`, not only of a latency;
(c) nothing until the block above `rx_data` goes through the flow with it.
Seeds were tried at one density and one corner only.
