#!/bin/bash
# The data path's blocks each alone through the open ASIC flow (docs/gates/silicon_probe.md,
# 2026-11-28, third entry): makes a set-up per block (asic-T_<block>) and starts the runs with
# probe_run.sh as asic-b_<block>, on the build server. As run, but for rx_deint: its first
# start at 40 % utilisation ended in the macro placer (a 703 µm macro with its 40 µm halo in
# a 703 µm core), so its die is given here as it was for the second.
#
#   blocks.sh <src>      <src> a tree with hw/rtl; PROBE_ROOT as for probe_run.sh
#
# The stores are the PDK's SRAM macros as in rx_data (rx_data_sram.mk): the macros' views and
# the wrapper are taken from a run of rx_data, $PROBE_ROOT/asic-$MACROS_FROM (default P1).
# Every port has the placeholder of 20 % of the period as external delay.
set -euo pipefail
R=${PROBE_ROOT:-/data/radio-chip}; SRC=$(realpath "$1"); M=$R/asic-${MACROS_FROM:-P1}
common="/repo/hw/rtl/gen/rx_front_rom_pkg.sv /repo/hw/rtl/gen/fft64_rom_pkg.sv /repo/hw/rtl/gen/rx_channel_pkg.sv /repo/hw/rtl/gen/rx_symbol_pkg.sv /repo/hw/rtl/gen/rx_track_pkg.sv /repo/hw/rtl/round_sat.sv"
mk() {  # top, uses macros (0/1), sources; EXTRA: settings for probe_run.sh
    local top=$1 mac=$2; shift 2
    local t=$R/asic-T_$top
    rm -rf "$t"; mkdir -p "$t/$top"
    local files="$common"
    for f in "$@"; do case $f in /*) files="$files $f" ;; *) files="$files /repo/hw/rtl/$f" ;; esac; done
    cat > "$t/$top/config.mk" <<EOC
export DESIGN_NAME = $top
export DESIGN_NICKNAME = $top
export PLATFORM = ihp-sg13g2
export SYNTH_HDL_FRONTEND = slang
export VERILOG_FILES = $files
export SDC_FILE = /work/$top/constraint.sdc
export CORE_UTILIZATION = 40
export PLACE_DENSITY ?= 0.60
export TNS_END_PERCENT = 100
export SYNTH_MEMORY_MAX_BITS = 16384
EOC
    if [ "$mac" = 1 ]; then
        cp -r "$M/macros" "$t/"; cp "$M/ram_1w1r_ihp.sv" "$t/"
        grep -E '^export ADDITIONAL_' "$M/rx_data/config.mk" >> "$t/$top/config.mk"
    fi
    cat > "$t/$top/constraint.sdc" <<EOC
current_design $top
set clk_period 6.25
create_clock -name clk -period \$clk_period [get_ports clk]
set non_clock_inputs [lsearch -inline -all -not -exact [all_inputs] [get_ports clk]]
set_input_delay  [expr \$clk_period * 0.2] -clock clk \$non_clock_inputs
set_output_delay [expr \$clk_period * 0.2] -clock clk [all_outputs]
EOC
    PROBE_CORES=${PROBE_CORES:-5} "$R/tools/probe_run.sh" start "b_$top" "T_$top" "$SRC" "${EXTRA[@]}"
}
EXTRA=()
mk fft64      0 fft64.sv
mk rx_channel 1 /work/ram_1w1r_ihp.sv lib/add_alone.sv lib/mul_alone.sv rx_channel.sv
mk rx_symbol  1 /work/ram_1w1r_ihp.sv lib/absmul_alone.sv rx_symbol.sv
mk rx_track   0 rx_track.sv
EXTRA=(-CORE_UTILIZATION "DIE_AREA=0 0 860 720" "CORE_AREA=18 18 842 702")
mk rx_deint   1 /work/ram_1w1r_ihp.sv rx_deint.sv
EXTRA=()
mk rx_signal  0 conv_encoder.sv rx_signal.sv
