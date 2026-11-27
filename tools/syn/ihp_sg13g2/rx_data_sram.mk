# The config.mk of the silicon probe's run of rx_data with its stores as IHP SG13G2 two-port
# SRAM macros (docs/gates/silicon_probe.md, 2026-11-25), as run. To repeat it, in a workdir W:
#   mkdir -p W/rx_data W/macros; cp this file W/rx_data/config.mk;
#   cp tools/syn/ihp_sg13g2/ram_1w1r.sv W/ram_1w1r_ihp.sv;
#   cp the two macros' LEF, GDS and typ/slow/fast Liberty from IHP-Open-PDK (e73adb4)
#     ihp-sg13g2/libs.ref/sg13g2_sram/{lef,gds,lib} to W/macros;
#   tools/syn/orfs_block.sh <repo> W rx_data 6.25 x.sv   (the config is kept as it is: the
#     source list argument is then unused; the script writes only constraint.sdc)
# The run with the macros placed by hand (2026-11-26) adds one line, with
# tools/syn/ihp_sg13g2/rx_data_macros.tcl copied to W/rx_data/macros.tcl:
#   export MACRO_PLACEMENT_TCL = /work/rx_data/macros.tcl
export DESIGN_NAME = rx_data
export DESIGN_NICKNAME = rx_data
export PLATFORM = ihp-sg13g2
export SYNTH_HDL_FRONTEND = slang
export VERILOG_FILES = /repo/hw/rtl/gen/rx_front_rom_pkg.sv /repo/hw/rtl/gen/fft64_rom_pkg.sv /repo/hw/rtl/gen/rx_channel_pkg.sv /repo/hw/rtl/gen/rx_symbol_pkg.sv /repo/hw/rtl/gen/rx_track_pkg.sv /repo/hw/rtl/round_sat.sv /work/ram_1w1r_ihp.sv /repo/hw/rtl/fft64.sv /repo/hw/rtl/div_serial.sv /repo/hw/rtl/div_share.sv /repo/hw/rtl/lib/add_alone.sv /repo/hw/rtl/rx_channel.sv /repo/hw/rtl/cordic_iter.sv /repo/hw/rtl/cordic_share.sv /repo/hw/rtl/rx_symbol.sv /repo/hw/rtl/rx_track.sv /repo/hw/rtl/rx_deint.sv /repo/hw/rtl/viterbi.sv /repo/hw/rtl/conv_encoder.sv /repo/hw/rtl/rx_signal.sv /repo/hw/rtl/rx_octets.sv /repo/hw/rtl/rx_data.sv
export SDC_FILE = /work/rx_data/constraint.sdc
export CORE_UTILIZATION = 40
export PLACE_DENSITY ?= 0.60
export TNS_END_PERCENT = 100
export SYNTH_MEMORY_MAX_BITS = 16384
export ADDITIONAL_LEFS = /work/macros/RM_IHPSG13_2P_64x32_c2.lef /work/macros/RM_IHPSG13_2P_64x22_c2_bm_bist.lef
export ADDITIONAL_GDS = /work/macros/RM_IHPSG13_2P_64x32_c2.gds /work/macros/RM_IHPSG13_2P_64x22_c2_bm_bist.gds
export ADDITIONAL_TYP_LIBS = /work/macros/RM_IHPSG13_2P_64x32_c2_typ_1p20V_25C.lib /work/macros/RM_IHPSG13_2P_64x22_c2_bm_bist_typ_1p20V_25C.lib
export ADDITIONAL_SLOW_LIBS = /work/macros/RM_IHPSG13_2P_64x32_c2_slow_1p08V_125C.lib /work/macros/RM_IHPSG13_2P_64x22_c2_bm_bist_slow_1p08V_125C.lib
export ADDITIONAL_FAST_LIBS = /work/macros/RM_IHPSG13_2P_64x32_c2_fast_1p32V_m55C.lib /work/macros/RM_IHPSG13_2P_64x22_c2_bm_bist_fast_1p32V_m55C.lib
