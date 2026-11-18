# G3 gates: RTL vs the golden model, bit for bit, under Verilator.
#
#   make              # all gates
#   make scrambler    # one block
#   make vectors      # regenerate every committed vector file (and generated ROM) from the model
#   make lint         # Verilator lint of every block's sources (-Wall, warnings are errors)
#   make clean
#
# Requires: verilator >= 5.050 (brew install verilator) and the Rust workspace.

VERILATOR ?= verilator
VFLAGS    := --cc --exe --build -Wall --x-assign unique --x-initial unique
RVEC      := cargo run -q --release -p radio-tools --bin rvec --
BLOCKS    := scrambler round_sat ltf_corr
# Generated SystemVerilog (rvec).
GEN_RTL   := rtl/gen/rx_front_rom_pkg.sv
# Sources and top module per block, packages first.
TOP_scrambler := scrambler
TOP_round_sat := round_sat_bank
TOP_ltf_corr  := ltf_corr
SRC_scrambler := rtl/scrambler.sv
SRC_round_sat := rtl/round_sat.sv tb/round_sat_bank.sv
SRC_ltf_corr  := $(GEN_RTL) rtl/round_sat.sv rtl/ltf_corr.sv

.PHONY: all lint vectors clean $(BLOCKS)

all: lint $(BLOCKS)

lint:
	$(foreach b,$(BLOCKS),$(VERILATOR) --lint-only -Wall --top-module $(TOP_$(b)) $(SRC_$(b)) &&) true

vectors:
	cd .. && for b in $(BLOCKS); do $(RVEC) $$b > hw/vectors/$$b.txt || exit 1; done
	cd .. && mkdir -p hw/rtl/gen && $(RVEC) rx-front-rom > hw/$(GEN_RTL)

scrambler: vectors/scrambler.txt
	mkdir -p obj_dir/scrambler
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/scrambler --top-module scrambler \
		$(SRC_scrambler) tb/tb_scrambler.cpp -o sim_scrambler
	./obj_dir/scrambler/sim_scrambler vectors/scrambler.txt

round_sat: vectors/round_sat.txt
	mkdir -p obj_dir/round_sat
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/round_sat --top-module round_sat_bank \
		$(SRC_round_sat) tb/tb_round_sat.cpp -o sim_round_sat
	./obj_dir/round_sat/sim_round_sat vectors/round_sat.txt

ltf_corr: vectors/ltf_corr.txt
	mkdir -p obj_dir/ltf_corr
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/ltf_corr --top-module ltf_corr \
		$(SRC_ltf_corr) tb/tb_ltf_corr.cpp -o sim_ltf_corr
	./obj_dir/ltf_corr/sim_ltf_corr vectors/ltf_corr.txt

clean:
	rm -rf obj_dir
