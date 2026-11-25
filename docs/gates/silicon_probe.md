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
| 5687060, before #94 (entry above) | 14 712 | 3.00 mm² (macros 0.41) | −3.29 ns | −171 ns | | 237 mW |
| 1dd55ed, `rx_symbol`'s reads registered | 14 794 | **3.05 mm²** (macros 0.41) | **−1.19 ns** (about 134 MHz) | −25.2 ns | 68 | 441 mW |
| 5687060, stores as flip-flops (entry above) | 26 891 | 3.74 mm² | +0.16 ns | 0 | 0 | 437 mW |

**What it says**
- **#94 did what it was for.** `rx_symbol`'s slot and `Hs` reads no longer appear among the
  failing paths. The worst slack goes from −3.29 to −1.19 ns.
- **It still misses 160 MHz with the macros.** The 68 failing endpoints are in three groups,
  all in `rx_channel`, `rx_symbol` and `rx_track`:
  1. **`rx_channel`'s first pass, a read, an add and a write in one cycle** (worst −1.19 ns):
     `Hs`'s macro answers 3.00 ns after its clock. `hs1 = word + L·T2` then goes back into the
     same store (`A_DIN`, 0.35 ns setup) and into `s2_re`/`s2_im`. With flip-flops this read
     was nearly free; with the macro, 2.9 ns are left for the add, the write multiplexer and
     the wires.
  2. **`rx_channel`'s multiplier, the 37 × 8 products `mph`/`mpl`** (to −0.80 ns), and `d`
     (to −0.11 ns).
  3. **`rx_symbol`'s `r6_mag`** (to −0.58 ns), `soft_v` (−0.03 ns), and `rx_track`'s `mlh`
     (−0.01 ns).

  Groups 2 and 3 are logic between flip-flops. They passed in the run with flip-flops (worst
  +0.16 ns), so the placement around the macros costs them up to about 1 ns. That is one
  placement on one seed, and it may move from run to run.
- **Area:** 3.05 mm² against 3.74 with flip-flops (−18 %). The 82 flip-flops more than the
  previous macro run are #94's registers.
- **Power:** the flow's estimate is 441 mW, against 237 mW for the previous macro run and
  437 mW with flip-flops. The switching part is 141 mW against 40. #94 adds 82 flip-flops,
  which does not explain it. Until a run with real activity (a VCD from the gate's stimulus),
  this estimate should not be used to compare the runs, even as a ranking.
- **The run.** The script exited 1 at its very end, after `6_report` and the merged GDS were
  written, as before; the figures are from that report.

**Open, for the founder.** With the macros, 160 MHz needs at least the following. All of
these are proposals, and none has been decided:
- **Group 1 (−1.19 ns):** register `Hs`'s read before the first pass's add. That is one more
  edge between a T2 bin and its write. Two bins in a row have different addresses, so there
  is no read-after-write hazard. Whether `chan_valid` moves depends on the slack between the
  first pass's last write and the second pass's first read, which needs checking against the
  gate. The alternative is to keep `Hs` as flip-flops: one store of 64 × 44 bits, about
  2 800 flip-flops.
- **Groups 2 and 3 (to −0.80 ns):** a fourth stage in `rx_channel`'s multiplier (its
  latencies restated or absorbed), and the same question for `rx_symbol`'s `r6_mag`. Or
  first try floorplanning: place the macros at the edge or near their users, by hand
  (`MACRO_PLACEMENT`). That changes no RTL, and it may be enough for the logic paths.
- **Or the stores stay flip-flops for now.** 160 MHz is not a margin: it is R = 8 clocks a
  sample at 20 MS/s (ADR 0006, 0008), and the front's folding and schedules are built on it.
  A slower clock would be an architecture change, not a setting. The flip-flop build closes
  160 MHz today. The macros can be taken up again with the ASIC stage's floorplan, so the
  0.7 mm² is deferred rather than given up.
