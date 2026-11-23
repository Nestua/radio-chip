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
VEC_BLOCKS := scrambler conv_encoder round_sat ltf_corr cordic nco det latch search decide front dc_estimate handoff
# `cordic_arb` replays the CORDIC's vectors.
BLOCKS    := ram cordic_arb $(VEC_BLOCKS)
# Generated SystemVerilog (rvec).
GEN_RTL   := rtl/gen/rx_front_rom_pkg.sv
# Sources and top module per block, packages first.
TOP_ram       := ram_bank
TOP_scrambler := scrambler
TOP_conv_encoder := conv_encoder
TOP_round_sat := round_sat_bank
TOP_ltf_corr  := ltf_corr
TOP_cordic    := cordic
TOP_cordic_arb := cordic_arb
TOP_nco       := nco
TOP_det       := det_chain
TOP_latch     := latch_chain
TOP_search    := search_chain
TOP_decide    := decide_chain
TOP_front     := rx_front
TOP_handoff   := handoff_chain
TOP_dc_estimate := dc_estimate
SRC_ram       := rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv tb/ram_bank.sv
SRC_scrambler := rtl/scrambler.sv
SRC_conv_encoder := rtl/conv_encoder.sv
SRC_round_sat := rtl/round_sat.sv tb/round_sat_bank.sv
SRC_ltf_corr  := $(GEN_RTL) rtl/round_sat.sv rtl/ltf_corr.sv
SRC_cordic    := $(GEN_RTL) rtl/cordic.sv
SRC_cordic_arb := $(GEN_RTL) rtl/cordic.sv rtl/cordic_arb.sv
SRC_nco       := $(GEN_RTL) rtl/round_sat.sv rtl/nco.sv
SRC_det       := $(GEN_RTL) rtl/round_sat.sv rtl/dc_block.sv rtl/det_a.sv tb/det_chain.sv
SRC_latch     := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv tb/latch_chain.sv
SRC_search    := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv rtl/ltf_corr.sv rtl/rx_search.sv \
                 tb/search_chain.sv
SRC_decide    := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv rtl/ltf_corr.sv rtl/rx_search.sv \
                 rtl/ltf_corr1.sv rtl/rx_decide.sv rtl/rx_front.sv tb/decide_chain.sv
SRC_front     := $(filter-out tb/decide_chain.sv,$(SRC_decide))
SRC_dc_estimate := $(GEN_RTL) rtl/dc_estimate.sv
SRC_handoff   := $(SRC_front) rtl/dc_estimate.sv rtl/rx_handoff.sv tb/handoff_chain.sv

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

conv_encoder: vectors/conv_encoder.txt
	rm -rf obj_dir/conv_encoder && mkdir -p obj_dir/conv_encoder
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/conv_encoder --top-module conv_encoder \
		$(SRC_conv_encoder) tb/tb_conv_encoder.cpp -o sim_conv_encoder
	./obj_dir/conv_encoder/sim_conv_encoder vectors/conv_encoder.txt

round_sat: vectors/round_sat.txt
	rm -rf obj_dir/round_sat && mkdir -p obj_dir/round_sat
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/round_sat --top-module round_sat_bank \
		$(SRC_round_sat) tb/tb_round_sat.cpp -o sim_round_sat
	./obj_dir/round_sat/sim_round_sat vectors/round_sat.txt

ltf_corr: vectors/ltf_corr.txt
	rm -rf obj_dir/ltf_corr && mkdir -p obj_dir/ltf_corr
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/ltf_corr --top-module ltf_corr \
		$(SRC_ltf_corr) tb/tb_ltf_corr.cpp -o sim_ltf_corr
	./obj_dir/ltf_corr/sim_ltf_corr vectors/ltf_corr.txt +verilator+rand+reset+2

cordic: vectors/cordic.txt
	rm -rf obj_dir/cordic && mkdir -p obj_dir/cordic
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/cordic --top-module cordic \
		$(SRC_cordic) tb/tb_cordic.cpp -o sim_cordic
	./obj_dir/cordic/sim_cordic vectors/cordic.txt

cordic_arb: vectors/cordic.txt
	rm -rf obj_dir/cordic_arb && mkdir -p obj_dir/cordic_arb
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/cordic_arb --top-module cordic_arb \
		$(SRC_cordic_arb) tb/tb_cordic_arb.cpp -o sim_cordic_arb
	./obj_dir/cordic_arb/sim_cordic_arb vectors/cordic.txt +verilator+rand+reset+2

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
# = 4 at R = 6 (ADR 0006) and at R = 8 (the decision's schedule), and D_SAMPLES = 5 at R = 5,
# the only one with two latches in flight. With the arbiter's register the contract is
# D_SAMPLES·R ≥ 24: D = 3 at R = 8 and D = 4 at R = 6 meet it with nothing to spare.
# Registers without a reset and the delay line start from random values (rand+reset+2).
latch: vectors/latch.txt
	rm -rf obj_dir/latch && mkdir -p obj_dir/latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=3 --Mdir obj_dir/latch/d3 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=4 --Mdir obj_dir/latch/d4 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	$(VERILATOR) $(VFLAGS) -GD_SAMPLES=5 --Mdir obj_dir/latch/d5 --top-module latch_chain \
		$(SRC_latch) tb/tb_latch.cpp -o sim_latch
	./obj_dir/latch/d3/sim_latch vectors/latch.txt 8 3 +verilator+rand+reset+2
	./obj_dir/latch/d3/sim_latch vectors/latch.txt 16 3 +verilator+rand+reset+2
	./obj_dir/latch/d4/sim_latch vectors/latch.txt 6 4 +verilator+rand+reset+2
	./obj_dir/latch/d4/sim_latch vectors/latch.txt 8 4 +verilator+rand+reset+2
	./obj_dir/latch/d5/sim_latch vectors/latch.txt 5 5 +verilator+rand+reset+2

# One build per (D_SAMPLES, W): the schedule's default D = 4 with GI − B = 12 (the default
# back-off), 16 (none: one ring word to spare) and 8; D = 3 and 5 at W = 12. The correlator
# takes a sample in 8 clocks at most: R = 8, 11 and 16 (the 10 MHz mode).
SEARCH_BUILDS := 4.12 4.16 4.8 3.12 5.12
search: vectors/search.txt
	rm -rf obj_dir/search && mkdir -p obj_dir/search
	set -e; for b in $(SEARCH_BUILDS); do d=$${b%.*}; w=$${b#*.}; \
		$(VERILATOR) $(VFLAGS) -GD_SAMPLES=$$d -GW=$$w --Mdir obj_dir/search/d$${d}w$$w \
			--top-module search_chain $(SRC_search) tb/tb_search.cpp -o sim_search; \
		for r in 8 11 16; do \
			./obj_dir/search/d$${d}w$$w/sim_search vectors/search.txt $$r $$d $$w +verilator+rand+reset+2; \
		done; \
	done

# The front with its loop closed. Builds as the search's and W = 4 (where the fine sum's
# branch of the schedule is the longer one); the run at D = 4, R = 8 also holds
# each result's cycle to the schedule model's (rvec gave it this testbench's clock pattern).
DECIDE_BUILDS := 4.12 4.16 4.8 4.4 3.12 5.12
decide: vectors/decide.txt
	rm -rf obj_dir/decide && mkdir -p obj_dir/decide
	set -e; for b in $(DECIDE_BUILDS); do d=$${b%.*}; w=$${b#*.}; \
		$(VERILATOR) $(VFLAGS) -GD_SAMPLES=$$d -GW=$$w --Mdir obj_dir/decide/d$${d}w$$w \
			--top-module decide_chain $(SRC_decide) tb/tb_decide.cpp -o sim_decide; \
		for r in 8 11 16; do \
			./obj_dir/decide/d$${d}w$$w/sim_decide vectors/decide.txt $$r $$d $$w +verilator+rand+reset+2; \
		done; \
	done

# The top level at its pins: only the ADC's samples in, only the acquisitions out, on long
# streams with nothing reset inside them. The builds and clock rates of `decide`.
front: vectors/front.txt
	rm -rf obj_dir/front && mkdir -p obj_dir/front
	set -e; for b in $(DECIDE_BUILDS); do d=$${b%.*}; w=$${b#*.}; \
		$(VERILATOR) $(VFLAGS) -GD_SAMPLES=$$d -GW=$$w --Mdir obj_dir/front/d$${d}w$$w \
			--top-module rx_front $(SRC_front) tb/tb_front.cpp -o sim_front; \
		for r in 8 11 16; do \
			./obj_dir/front/d$${d}w$$w/sim_front vectors/front.txt $$r $$d $$w +verilator+rand+reset+2; \
		done; \
	done

dc_estimate: vectors/dc_estimate.txt
	rm -rf obj_dir/dc_estimate && mkdir -p obj_dir/dc_estimate
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/dc_estimate --top-module dc_estimate \
		$(SRC_dc_estimate) tb/tb_dc_estimate.cpp -o sim_dc_estimate
	./obj_dir/dc_estimate/sim_dc_estimate vectors/dc_estimate.txt +verilator+rand+reset+2

# The hand-off behind the front, a cycle at a time against the schedule model: the beats in
# their cycles, the hold in every cycle. Each build runs the vectors' segments of its W at
# the clocks they were written for (W = 12 at R = 8, 11 and 16; the others at R = 8).
HANDOFF_RUNS := 12.8 12.11 12.16 16.8 8.8 4.8
handoff: vectors/handoff.txt
	rm -rf obj_dir/handoff && mkdir -p obj_dir/handoff
	set -e; for w in 12 16 8 4; do \
		$(VERILATOR) $(VFLAGS) -GD_SAMPLES=4 -GW=$$w --Mdir obj_dir/handoff/w$$w \
			--top-module handoff_chain $(SRC_handoff) tb/tb_handoff.cpp -o sim_handoff; \
	done; \
	for b in $(HANDOFF_RUNS); do w=$${b%.*}; r=$${b#*.}; \
		./obj_dir/handoff/w$$w/sim_handoff vectors/handoff.txt $$r 4 $$w +verilator+rand+reset+2; \
	done

clean:
	rm -rf obj_dir
