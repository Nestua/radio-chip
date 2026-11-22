# Out-of-context timing and area probe of one RTL block (Vivado, batch mode). Evidence for the
# gate journals, not a build flow: hw/fpga/ arrives with R2.
#
#   vivado -mode batch -nojournal -source tools/syn/ooc_probe.tcl -tclargs <repo> <top> <outdir> [extra.sv ...]
#
# Reads the generated ROM package, round_sat and the memory wrappers, any extra sources, and
# hw/rtl/<top>.sv; one clock at 6.25 ns (160 MHz, R = 8 clocks per sample at 20 MS/s, ADR 0006)
# on the KV260's part; default synthesis, placement and routing. cordic_arb needs cordic.sv as
# an extra source.
set repo [lindex $argv 0]
set top  [lindex $argv 1]
set out  [lindex $argv 2]
set r $repo/hw/rtl
set src [list $r/gen/rx_front_rom_pkg.sv $r/round_sat.sv $r/lib/ram_1w1r.sv $r/lib/ram_1w2r.sv]
foreach f [lrange $argv 3 end] { lappend src $f }
lappend src $r/$top.sv
read_verilog -sv $src
synth_design -top $top -part xck26-sfvc784-2LV-c -mode out_of_context
create_clock -period 6.25 -name clk [get_ports clk]
opt_design
place_design
route_design
report_utilization -file $out/util.txt
report_timing_summary -file $out/timing.txt
report_timing -max_paths 3 -file $out/paths.txt
