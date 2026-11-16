// G3 gate for the scrambler: replays the golden vectors (hw/vectors/scrambler.txt) through the RTL
// and requires every output bit to match radio_phy::scrambler bit for bit.
// Exit code 0 = gate passed; anything else = failed.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "Vscrambler.h"
#include "verilated.h"

static void tick(Vscrambler& dut) {
    dut.clk = 0;
    dut.eval();
    dut.clk = 1;
    dut.eval();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_scrambler <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }

    Vscrambler dut;
    dut.rst_n = 0;
    dut.load = 0;
    dut.in_valid = 0;
    tick(dut);
    dut.rst_n = 1;

    std::string line;
    long vectors = 0, bits = 0, mismatches = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        std::string seed_hex, input, expected;
        if (!(fields >> seed_hex >> input >> expected) || input.size() != expected.size()) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }

        dut.load = 1;
        dut.seed = std::stoul(seed_hex, nullptr, 16);
        dut.in_valid = 0;
        tick(dut);
        dut.load = 0;

        for (size_t i = 0; i < input.size(); ++i) {
            dut.in_valid = 1;
            dut.in_bit = input[i] == '1';
            tick(dut);
            const int got = dut.out_bit;
            const int want = expected[i] == '1';
            if (!dut.out_valid || got != want) {
                if (mismatches < 10) {
                    std::fprintf(stderr, "seed %s bit %zu: rtl=%d golden=%d valid=%d\n",
                                 seed_hex.c_str(), i, got, want, dut.out_valid);
                }
                ++mismatches;
            }
            ++bits;
        }
        dut.in_valid = 0;
        ++vectors;
    }

    std::printf("scrambler G3: %ld vectors, %ld bits, %ld mismatches\n", vectors, bits, mismatches);
    if (vectors == 0) {
        std::cerr << "no vectors read — refusing to pass an empty gate\n";
        return 1;
    }
    return mismatches == 0 ? 0 : 1;
}
