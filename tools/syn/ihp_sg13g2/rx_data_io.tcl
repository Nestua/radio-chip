# Pin positions for rx_data in the silicon probe (IO_CONSTRAINTS; docs/gates/silicon_probe.md,
# 2026-11-28). rx_age = adc_count − end_stamp is logic from two input buses to an output
# bus with no flip-flop between, so its length is where the flow puts those 48 pins. Left to
# itself the pin placer once (GPL_RANDOM_SEED = 3) split each bus, eleven bits on the left
# edge and five on the top, and the borrow crossed 1.6 mm of the block through 49 buffers.
# Here the three buses are one group on the left edge, a bit of each side by side in bit
# order. Every other pin is left to the placer.
set age_pins {}
for {set i 0} {$i < 16} {incr i} {
  lappend age_pins "adc_count\[$i\]" "end_stamp\[$i\]" "rx_age\[$i\]"
}
set_io_pin_constraint -group -order -region left:* -pin_names $age_pins
