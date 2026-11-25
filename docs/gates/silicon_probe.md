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
