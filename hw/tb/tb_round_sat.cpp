// G3 gate for round_sat: every 12-bit input through eight parameter sets (round_sat_bank.sv),
// compared with radio_phy::fixed::requantize(HalfEven, Saturate) via hw/vectors/round_sat.txt.
// Exit code 0 = gate passed.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "Vround_sat_bank.h"
#include "verilated.h"

static long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_round_sat <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    Vround_sat_bank dut;
    static const int widths[8] = {12, 10, 9, 8, 6, 4, 2, 1};
    long lines = 0, checks = 0, mismatches = 0, saturated = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        long x, want[8];
        if (!(f >> x)) continue;
        for (long& w : want) {
            if (!(f >> w)) {
                std::cerr << "malformed vector line: " << line << "\n";
                return 2;
            }
        }
        dut.in = static_cast<uint32_t>(x) & 0xfff;
        dut.eval();
        const uint32_t got_raw[8] = {dut.o_s1_w12, dut.o_s1_w10, dut.o_s4_w9, dut.o_s4_w8,
                                     dut.o_s4_w6,  dut.o_s8_w4,  dut.o_s11_w2, dut.o_s11_w1};
        for (int i = 0; i < 8; ++i) {
            const long got = sext(got_raw[i], widths[i]);
            const long lim = 1L << (widths[i] - 1);
            if (want[i] == lim - 1 || want[i] == -lim) ++saturated;
            if (got != want[i]) {
                if (mismatches < 10)
                    std::fprintf(stderr, "in %ld set %d: rtl=%ld golden=%ld\n", x, i, got, want[i]);
                ++mismatches;
            }
            ++checks;
        }
        ++lines;
    }
    std::printf("round_sat G3: %ld inputs, %ld checks (%ld at a bound), %ld mismatches\n", lines,
                checks, saturated, mismatches);
    if (lines != 4096) {
        std::cerr << "expected all 4096 inputs\n";
        return 1;
    }
    return mismatches == 0 ? 0 : 1;
}
