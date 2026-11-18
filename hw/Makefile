# G3 gates: RTL vs the golden model, bit for bit, under Verilator.
#
#   make              # all gates
#   make scrambler    # one block
#   make vectors      # regenerate every committed vector file (and generated ROM) from the model
#   make lint         # Verilator lint of every block's sources (-Wall, warnings are errors)
#   make clean
#
# Requires: verilator >= 5.050 (brew install verilator) and the Rust workspace.
#
# Every gate builds from scratch (rm -rf its obj_dir): Verilator's incremental build reused a
# stale model after a mutation was restored, and a mutation result was read off the previous
# build (docs/gates/rx_front.md, part 3d).

VERILATOR ?= verilator
VFLAGS    := --cc --exe --build -Wall --x-assign unique --x-initial unique lint.vlt
RVEC      := cargo run -q --release -p radio-tools --bin rvec --
# Blocks gated on rvec vectors; `ram` is gated against a reference model in its testbench.
VEC_BLOCKS := scrambler round_sat ltf_corr cordic nco det latch
BLOCKS    := ram $(VEC_BLOCKS)
# Generated SystemVerilog (rvec).
GEN_RTL   := rtl/gen/rx_front_rom_pkg.sv
# Sources and top module per block, packages first.
TOP_ram       := ram_bank
TOP_scrambler := scrambler
TOP_round_sat := round_sat_bank
TOP_ltf_corr  := ltf_corr
TOP_cordic    := cordic
TOP_nco       := nco
TOP_det       := det_chain
TOP_latch     := latch_chain
SRC_ram       := rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv tb/ram_bank.sv
SRC_scrambler := rtl/scrambler.sv
SRC_round_sat := rtl/round_sat.sv tb/round_sat_bank.sv
SRC_ltf_corr  := $(GEN_RTL) rtl/round_sat.sv rtl/ltf_corr.sv
SRC_cordic    := $(GEN_RTL) rtl/cordic.sv
SRC_nco       := $(GEN_RTL) rtl/round_sat.sv rtl/nco.sv
SRC_det       := $(GEN_RTL) rtl/round_sat.sv rtl/dc_block.sv rtl/det_a.sv tb/det_chain.sv
SRC_latch     := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/nco.sv rtl/rx_latch.sv tb/latch_chain.sv

.PHONY: all lint vectors clean $(BLOCKS)

all: lint $(BLOCKS)

lint:
	$(foreach b,$(BLOCKS),$(VERILATOR) --lint-only -Wall lint.vlt --top-module $(TOP_$(b)) $(SRC_$(b)) &&) true

vectors:
	cd .. && for b in $(VEC_BLOCKS); do $(RVEC) $$b > hw/vectors/$$b.txt || exit 1; done
	cd .. && mkdir -p hw/rtl/gen && $(RVEC) rx-front-rom > hw/$(GEN_RTL)

ram:
	rm -rf obj_dir/ram && mkdir -p obj_dir/ram
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/ram --top-module ram_bank \
		$(SRC_ram) tb/tb_ram.cpp -o sim_ram
	./obj_dir/ram/sim_ram

scrambler: vectors/scrambler.txt
	rm -rf obj_dir/scrambler && mkdir -p obj_dir/scrambler
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/scrambler --top-module scrambler \
		$(SRC_scrambler) tb/tb_scrambler.cpp -o sim_scrambler
	./obj_dir/scrambler/sim_scrambler vectors/scrambler.txt

round_sat: vectors/round_sat.txt
	rm -rf obj_dir/round_sat && mkdir -p obj_dir/round_sat
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/round_sat --top-module round_sat_bank \
		$(SRC_round_sat) tb/tb_round_sat.cpp -o sim_round_sat
	./obj_dir/round_sat/sim_round_sat vectors/round_sat.txt

ltf_corr: vectors/ltf_corr.txt
	rm -rf obj_dir/ltf_corr && mkdir -p obj_dir/ltf_corr
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/ltf_corr --top-module ltf_corr \
		$(SRC_ltf_corr) tb/tb_ltf_corr.cpp -o sim_ltf_corr
	./obj_dir/ltf_corr/sim_ltf_corr vectors/ltf_corr.txt

cordic: vectors/cordic.txt
	rm -rf obj_dir/cordic && mkdir -p obj_dir/cordic
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/cordic --top-module cordic \
		$(SRC_cordic) tb/tb_cordic.cpp -o sim_cordic
	./obj_dir/cordic/sim_cordic vectors/cordic.txt

nco: vectors/nco.txt
	rm -rf obj_dir/nco && mkdir -p obj_dir/nco
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/nco --top-module nco \
		$(SRC_nco) tb/tb_nco.cpp -o sim_nco
	./obj_dir/nco/sim_nco vectors/nco.txt

det: vectors/det.txt
	rm -rf obj_dir/det && mkdir -p obj_dir/det
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/det --top-module det_chain \
		$(SRC_det) tb/tb_det.cpp -o sim_det
	./obj_dir/det/sim_det vectors/det.txt

# Three builds: D_SAMPLES = 3 at R = 8 clocks per sample (and R = 16, the 10 MHz mode), D_SAMPLES
# = 4 at R = 6 (ADR 0006), and D_SAMPLES = 5 at R = 5, the only one with two latches in flight.
latch: vectors/latch.txt
	rm -rf obj_dir/latch && mkdir -p obj_dir/latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=3 --Mdir obj_dir/latch/d3 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=4 --Mdir obj_dir/latch/d4 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=5 --Mdir obj_dir/latch/d5 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	./obj_dir/latch/d3/sim_latch vectors/latch.txt 8 3
	./obj_dir/latch/d3/sim_latch vectors/latch.txt 16 3
	./obj_dir/latch/d4/sim_latch vectors/latch.txt 6 4
	./obj_dir/latch/d5/sim_latch vectors/latch.txt 5 5

clean:
	rm -rf obj_dir
