#!/bin/bash
# One RTL block through the open ASIC flow (OpenROAD-flow-scripts, platform ihp-sg13g2):
# synthesis (yosys with its SystemVerilog front end), floorplan, placement, clock tree,
# routing, final report. Evidence for the gate journals (docs/gates/silicon_probe.md), not a
# tape-out flow: no pads, no macros, no sign-off.
#
#   tools/syn/orfs_block.sh <repo> <workdir> <top> <period ns> <source under hw/rtl> ...
#
# Needs docker and the image openroad/orfs (used: the one on the build server, see the
# journal for its versions). Results: <workdir>/<top>/run/{logs,reports,results}.
# Memories are synthesised as flip-flops up to 16 kbit (the flow's own limit is 4 kbit): a
# block that needs more needs a macro, which this probe does not place.
# A block too small for the platform's power grid needs DIE_AREA/CORE_AREA in its config.mk
# (the scrambler: DIE_AREA = 0 0 160 160, CORE_AREA = 30 30 130 130) instead of
# CORE_UTILIZATION.
set -euo pipefail
repo=$(realpath "$1"); work=$(realpath "$2"); top=$3; period=$4; shift 4
mkdir -p "$work/$top"
files=""
for f in "$@"; do files="$files /repo/hw/rtl/$f"; done
if [ ! -f "$work/$top/config.mk" ]; then
cat > "$work/$top/config.mk" <<EOC
export DESIGN_NAME = $top
export DESIGN_NICKNAME = $top
export PLATFORM = ihp-sg13g2
export SYNTH_HDL_FRONTEND = slang
export VERILOG_FILES =$files
export SDC_FILE = /work/$top/constraint.sdc
export CORE_UTILIZATION = 40
export PLACE_DENSITY ?= 0.60
export TNS_END_PERCENT = 100
export SYNTH_MEMORY_MAX_BITS = 16384
EOC
fi
cat > "$work/$top/constraint.sdc" <<EOC
current_design $top
set clk_period $period
create_clock -name clk -period \$clk_period [get_ports clk]
set non_clock_inputs [lsearch -inline -all -not -exact [all_inputs] [get_ports clk]]
set_input_delay  [expr \$clk_period * 0.2] -clock clk \$non_clock_inputs
set_output_delay [expr \$clk_period * 0.2] -clock clk [all_outputs]
EOC
docker run --rm -v "$repo":/repo:ro -v "$work":/work openroad/orfs:latest bash -lc \
  "source /OpenROAD-flow-scripts/env.sh >/dev/null 2>&1; cd /OpenROAD-flow-scripts/flow; \
   make DESIGN_CONFIG=/work/$top/config.mk WORK_HOME=/work/$top/run" > "$work/$top/flow.log" 2>&1
r="$work/$top/run/reports/ihp-sg13g2/$top/base"
echo "$top: $(grep 'Chip area for module' "$r/synth_stat.txt" | tail -1)"
grep 'Design area' "$work/$top/run/logs/ihp-sg13g2/$top/base/6_report.log" | tail -1
grep -A2 '^worst slack max' "$r/6_finish.rpt" | head -1
