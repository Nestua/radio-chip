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
VEC_BLOCKS := scrambler conv_encoder viterbi round_sat ltf_corr cordic nco det latch search search_alone decide decide_t1 front dc_estimate handoff fft64 rx_channel rx_symbol rx_track rx_deint rx_signal rx_octets rx_data
# `cordic_arb` and `cordic_iter` replay the CORDIC's vectors; `handoff_forced` holds the hand-off to itself.
BLOCKS    := ram cordic_arb cordic_iter handoff_forced $(VEC_BLOCKS)
# Generated SystemVerilog (rvec).
GEN_RTL   := rtl/gen/rx_front_rom_pkg.sv
GEN_FFT   := rtl/gen/fft64_rom_pkg.sv
GEN_CHAN  := rtl/gen/rx_channel_pkg.sv
GEN_SYM   := rtl/gen/rx_symbol_pkg.sv
GEN_TRK   := rtl/gen/rx_track_pkg.sv
# Sources and top module per block, packages first.
TOP_ram       := ram_bank
TOP_scrambler := scrambler
TOP_conv_encoder := conv_encoder
TOP_viterbi   := viterbi
TOP_fft64     := fft64
TOP_rx_channel := channel_chain
TOP_rx_symbol := symbol_chain
TOP_rx_track := track_chain
TOP_rx_deint := rx_deint
TOP_rx_signal := signal_chain
TOP_rx_octets := rx_octets
TOP_rx_data   := rx_data
TOP_round_sat := round_sat_bank
TOP_ltf_corr  := ltf_corr
TOP_cordic    := cordic
TOP_cordic_arb := cordic_arb
TOP_cordic_iter := cordic_iter
TOP_nco       := nco
TOP_det       := det_chain
TOP_latch     := latch_chain
TOP_search    := search_chain
TOP_search_alone := rx_search
TOP_decide    := decide_chain
TOP_decide_t1 := decide_alone
TOP_front     := rx_front
TOP_handoff   := handoff_chain
TOP_dc_estimate := dc_estimate
TOP_handoff_forced := handoff_alone
SRC_ram       := rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv tb/ram_bank.sv
SRC_scrambler := rtl/scrambler.sv
SRC_conv_encoder := rtl/conv_encoder.sv
SRC_viterbi   := rtl/viterbi.sv
SRC_fft64     := $(GEN_FFT) rtl/fft64.sv
SRC_rx_channel := $(GEN_CHAN) rtl/lib/ram_1w1r.sv rtl/div_serial.sv rtl/rx_channel.sv tb/channel_chain.sv
SRC_rx_symbol := $(GEN_RTL) $(GEN_CHAN) $(GEN_SYM) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/cordic_iter.sv \
                 rtl/rx_symbol.sv tb/symbol_chain.sv
SRC_rx_track  := $(GEN_RTL) $(GEN_CHAN) $(GEN_SYM) $(GEN_TRK) rtl/cordic_iter.sv rtl/cordic_share.sv \
                 rtl/div_serial.sv rtl/div_share.sv rtl/rx_track.sv tb/track_chain.sv
SRC_rx_deint  := $(GEN_SYM) rtl/lib/ram_1w1r.sv rtl/rx_deint.sv
SRC_rx_signal := rtl/viterbi.sv rtl/conv_encoder.sv rtl/rx_signal.sv tb/signal_chain.sv
SRC_rx_octets := rtl/rx_octets.sv
SRC_rx_data   := $(GEN_RTL) $(GEN_FFT) $(GEN_CHAN) $(GEN_SYM) $(GEN_TRK) rtl/round_sat.sv rtl/lib/ram_1w1r.sv \
                 rtl/fft64.sv rtl/div_serial.sv rtl/div_share.sv rtl/rx_channel.sv rtl/cordic_iter.sv \
                 rtl/cordic_share.sv rtl/rx_symbol.sv rtl/rx_track.sv rtl/rx_deint.sv rtl/viterbi.sv \
                 rtl/conv_encoder.sv rtl/rx_signal.sv rtl/rx_octets.sv rtl/rx_data.sv
SRC_round_sat := rtl/round_sat.sv tb/round_sat_bank.sv
SRC_ltf_corr  := $(GEN_RTL) rtl/round_sat.sv rtl/ltf_corr.sv
SRC_cordic    := $(GEN_RTL) rtl/cordic.sv
SRC_cordic_arb := $(GEN_RTL) rtl/cordic.sv rtl/cordic_arb.sv
SRC_cordic_iter := $(GEN_RTL) rtl/cordic_iter.sv
SRC_nco       := $(GEN_RTL) rtl/round_sat.sv rtl/nco.sv
SRC_det       := $(GEN_RTL) rtl/round_sat.sv rtl/dc_block.sv rtl/det_a.sv tb/det_chain.sv
SRC_latch     := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv tb/latch_chain.sv
SRC_search    := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv rtl/ltf_corr.sv rtl/rx_search.sv \
                 tb/search_chain.sv
SRC_search_alone := $(GEN_RTL) rtl/lib/ram_1w1r.sv rtl/rx_search.sv
SRC_decide   := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/dc_block.sv \
                 rtl/det_a.sv rtl/cordic.sv rtl/cordic_arb.sv rtl/nco.sv rtl/rx_latch.sv rtl/ltf_corr.sv rtl/rx_search.sv \
                 rtl/ltf_corr1.sv rtl/rx_decide.sv rtl/rx_front.sv tb/decide_chain.sv
SRC_front     := $(filter-out tb/decide_chain.sv,$(SRC_decide))
SRC_decide_t1 := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/lib/ram_1w2r.sv rtl/cordic.sv rtl/cordic_arb.sv \
                 rtl/nco.sv rtl/ltf_corr1.sv rtl/rx_decide.sv tb/decide_alone.sv
SRC_dc_estimate := $(GEN_RTL) rtl/dc_estimate.sv
SRC_handoff_forced := $(GEN_RTL) rtl/round_sat.sv rtl/lib/ram_1w1r.sv rtl/dc_estimate.sv rtl/rx_handoff.sv \
                 tb/handoff_alone.sv
SRC_handoff   := $(SRC_front) rtl/dc_estimate.sv rtl/rx_handoff.sv tb/handoff_chain.sv

.PHONY: all lint vectors clean $(BLOCKS)

all: lint $(BLOCKS)

lint:
	$(foreach b,$(BLOCKS),$(VERILATOR) --lint-only -Wall lint.vlt --top-module $(TOP_$(b)) $(SRC_$(b)) &&) true

vectors:
	cd .. && for b in $(VEC_BLOCKS); do $(RVEC) $$b > hw/vectors/$$b.txt || exit 1; done
	cd .. && mkdir -p hw/rtl/gen && $(RVEC) rx-front-rom > hw/$(GEN_RTL)
	cd .. && $(RVEC) fft64-rom > hw/$(GEN_FFT)
	cd .. && $(RVEC) rx-channel-pkg > hw/$(GEN_CHAN)
	cd .. && $(RVEC) rx-symbol-pkg > hw/$(GEN_SYM)
	cd .. && $(RVEC) rx-track-pkg > hw/$(GEN_TRK)

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

# The FFT, R2SDF: every bin of every window against fft64_window_fixed, gaps, chains, skids,
# drops; the last bin's cycle.
fft64: vectors/fft64.txt
	rm -rf obj_dir/fft64 && mkdir -p obj_dir/fft64
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/fft64 --top-module fft64 \
		$(SRC_fft64) tb/tb_fft64.cpp -o sim_fft64
	./obj_dir/fft64/sim_fft64 vectors/fft64.txt +verilator+rand+reset+2

# The channel and the per-packet quotients (with the serial divider): every packet's shift,
# Σg, quotients and stores against the model, their edges, requests, drops.
rx_channel: vectors/rx_channel.txt
	rm -rf obj_dir/rx_channel && mkdir -p obj_dir/rx_channel
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_channel --top-module channel_chain \
		$(SRC_rx_channel) tb/tb_rx_channel.cpp -o sim_rx_channel
	./obj_dir/rx_channel/sim_rx_channel vectors/rx_channel.txt +verilator+rand+reset+2

# The symbol block (products, pilots, common phase, soft pass) with its CORDIC: every pilot,
# phase and soft value against the model, their edges, held bins, late scales, aborts.
rx_symbol: vectors/rx_symbol.txt
	rm -rf obj_dir/rx_symbol && mkdir -p obj_dir/rx_symbol
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_symbol --top-module symbol_chain \
		$(SRC_rx_symbol) tb/tb_rx_symbol.cpp -o sim_rx_symbol
	./obj_dir/rx_symbol/sim_rx_symbol vectors/rx_symbol.txt +verilator+rand+reset+2

# The decoder at one trellis step a clock, against decode_windowed: every bit in its cycle,
# idle cycles inside blocks, blocks back to back, resets with a block in flight.
viterbi: vectors/viterbi.txt
	rm -rf obj_dir/viterbi && mkdir -p obj_dir/viterbi
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/viterbi --top-module viterbi \
		$(SRC_viterbi) tb/tb_viterbi.cpp -o sim_viterbi
	./obj_dir/viterbi/sim_viterbi vectors/viterbi.txt +verilator+rand+reset+2

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

# The tracker with the shared CORDIC and divider: every θ and window move against
# FixedTracking::{step, move_window}, their edges, the other requesters' answers, drops.
rx_track: vectors/rx_track.txt
	rm -rf obj_dir/rx_track && mkdir -p obj_dir/rx_track
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_track --top-module track_chain \
		$(SRC_rx_track) tb/tb_rx_track.cpp -o sim_rx_track
	./obj_dir/rx_track/sim_rx_track vectors/rx_track.txt +verilator+rand+reset+2

# The deinterleaver and depuncturer: every step to the Viterbi decoder against deinterleave_soft
# and depuncture, its edge, slot_ok every cycle; packets back to back, drops and resets.
rx_deint: vectors/rx_deint.txt
	rm -rf obj_dir/rx_deint && mkdir -p obj_dir/rx_deint
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_deint --top-module rx_deint \
		$(SRC_rx_deint) tb/tb_rx_deint.cpp -o sim_rx_deint
	./obj_dir/rx_deint/sim_rx_deint vectors/rx_deint.txt +verilator+rand+reset+2

# SIGNAL and the verdict behind the Viterbi decoder: every verdict, its fields and its edge
# against signal_checked, SignalVerdict::of and data_symbols; data blocks, drops, resets.
rx_signal: vectors/rx_signal.txt
	rm -rf obj_dir/rx_signal && mkdir -p obj_dir/rx_signal
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_signal --top-module signal_chain \
		$(SRC_rx_signal) tb/tb_rx_signal.cpp -o sim_rx_signal
	./obj_dir/rx_signal/sim_rx_signal vectors/rx_signal.txt +verilator+rand+reset+2

rx_octets: vectors/rx_octets.txt
	rm -rf obj_dir/rx_octets && mkdir -p obj_dir/rx_octets
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/rx_octets --top-module rx_octets \
		$(SRC_rx_octets) tb/tb_rx_octets.cpp -o sim_rx_octets
	./obj_dir/rx_octets/sim_rx_octets vectors/rx_octets.txt +verilator+rand+reset+2

# The data path's top level at its pins, a cycle at a time against the schedule model.
rx_data: vectors/rx_data.txt
	rm -rf obj_dir/rx_data && mkdir -p obj_dir/rx_data
	$(VERILATOR) $(VFLAGS) -O3 --Mdir obj_dir/rx_data --top-module rx_data \
		$(SRC_rx_data) tb/tb_rx_data.cpp -o sim_rx_data
	./obj_dir/rx_data/sim_rx_data vectors/rx_data.txt +verilator+rand+reset+2

# The data path's CORDIC, one micro-rotation a clock: the CORDIC's vectors, one at a time,
# back to back and with gaps, drops and resets in flight.
cordic_iter: vectors/cordic.txt
	rm -rf obj_dir/cordic_iter && mkdir -p obj_dir/cordic_iter
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/cordic_iter --top-module cordic_iter \
		$(SRC_cordic_iter) tb/tb_cordic_iter.cpp -o sim_cordic_iter
	./obj_dir/cordic_iter/sim_cordic_iter vectors/cordic.txt +verilator+rand+reset+2

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

# The search alone, driven at its correlator's port past what the chain gives (the energy's
# upper 12 of 44 bits; the founder's decision of 2026-11-23, docs/gates/rx_front.md). One build
# per W.L.PATH_DELAY: GI − B = 16 (the bound, 2^43), 12, 8, 4 and 1; L = 64, 20 and 63 + W (the
# largest the RTL takes); PATH_DELAY = 19 (the chain's), 7 and 1. R = 8 and 13.
SEARCH_ALONE_BUILDS := 16.64.19 12.64.19 8.20.7 4.67.1 1.64.19
search_alone: vectors/search_alone.txt
	rm -rf obj_dir/search_alone && mkdir -p obj_dir/search_alone
	set -e; for b in $(SEARCH_ALONE_BUILDS); do w=$${b%%.*}; r=$${b#*.}; l=$${r%.*}; p=$${r#*.}; \
		$(VERILATOR) $(VFLAGS) -GW=$$w -GL=$$l -GPATH_DELAY=$$p --Mdir obj_dir/search_alone/w$$w \
			--top-module rx_search $(SRC_search_alone) tb/tb_search_alone.cpp -o sim_search_alone; \
		for c in 8 13; do \
			./obj_dir/search_alone/w$$w/sim_search_alone vectors/search_alone.txt $$c $$w $$l $$p +verilator+rand+reset+2; \
		done; \
	done

# The decision's T1 on the search's full-width energies (the chain's stay within 32 bits): the
# decision engine alone, its candidate and ring the testbench's. W = 16 (the bound), 12, 8, 1.
DECIDE_T1_BUILDS := 16 12 8 1
decide_t1: vectors/decide_t1.txt
	rm -rf obj_dir/decide_t1 && mkdir -p obj_dir/decide_t1
	set -e; for w in $(DECIDE_T1_BUILDS); do \
		$(VERILATOR) $(VFLAGS) -GW=$$w --Mdir obj_dir/decide_t1/w$$w \
			--top-module decide_alone $(SRC_decide_t1) tb/tb_decide_t1.cpp -o sim_decide_t1; \
		./obj_dir/decide_t1/w$$w/sim_decide_t1 vectors/decide_t1.txt $$w +verilator+rand+reset+2; \
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

# The hand-off alone with its acquisitions forced: a declaration 1 to 80 samples before
# another, which the front does not give. The reference is the second declaration alone.
handoff_forced:
	rm -rf obj_dir/handoff_forced && mkdir -p obj_dir/handoff_forced
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/handoff_forced --top-module handoff_alone \
		$(SRC_handoff_forced) tb/tb_handoff_forced.cpp -o sim_handoff_forced
	./obj_dir/handoff_forced/sim_handoff_forced +verilator+rand+reset+2

clean:
	rm -rf obj_dir
