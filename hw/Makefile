# G3 gates: RTL vs the golden model, bit for bit, under Verilator.
#
#   make              # all gates
#   make scrambler    # one block
#   make vectors      # regenerate every committed vector file from the golden model
#   make lint         # Verilator lint of all RTL (-Wall, warnings are errors)
#   make clean
#
# Requires: verilator >= 5.050 (brew install verilator) and the Rust workspace.

VERILATOR ?= verilator
VFLAGS    := --cc --exe --build -Wall --x-assign unique --x-initial unique
RVEC      := cargo run -q --release -p radio-tools --bin rvec --
BLOCKS    := scrambler

.PHONY: all lint vectors clean $(BLOCKS)

all: lint $(BLOCKS)

lint:
	$(VERILATOR) --lint-only -Wall rtl/*.sv

vectors:
	cd .. && for b in $(BLOCKS); do $(RVEC) $$b > hw/vectors/$$b.txt || exit 1; done

scrambler: vectors/scrambler.txt
	mkdir -p obj_dir/scrambler
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir/scrambler --top-module scrambler \
		rtl/scrambler.sv tb/tb_scrambler.cpp -o sim_scrambler
	./obj_dir/scrambler/sim_scrambler vectors/scrambler.txt

clean:
	rm -rf obj_dir
