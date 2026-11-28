# rx_data's own external delays for the silicon probe (docs/gates/silicon_probe.md,
# 2026-11-28), read after the placeholder of tools/syn/orfs_block.sh (20 % of the period on
# every input and output), which stays on every port not named here.
#
# adc_count and end_stamp are registers of rx_handoff on this clock (rx_handoff.sv: the
# sample count and the frame's end in it), with no logic behind them. Their delay is a
# flip-flop's clock-to-output and the wire between the two blocks: 0.5 ns, against 0.17 to
# 0.43 ns (median 0.25) for the clock-to-output of the flip-flops on the worst paths of the
# run with seed 1. The output side of rx_age (the MAC) is still the placeholder: there is
# no MAC.
set_input_delay 0.5 -clock clk [get_ports {adc_count[*] end_stamp[*]}]
