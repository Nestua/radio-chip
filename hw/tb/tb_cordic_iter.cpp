// G3 gate for the iterative CORDIC (hw/rtl/cordic_iter.sv): replays hw/vectors/cordic.txt and
// requires every angle to match radio_phy::rx_front_fixed::cordic_angle bit for bit.
//
// One input at a time: the next is offered as soon as busy is low (in the cycle out_valid is
// up, every third input) or after a gap of up to three clocks. Every output must come exactly
// kLatency edges after its input and busy must stay high until then. Every 50th input is
// dropped (drop) or reset (every 150th) a random number of edges after it was taken: it must
// not come out, and the next input must be served normally. Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vcordic_iter.h"
#include "verilated.h"

namespace {

constexpr int kInBits = 34;
constexpr int kAngleBits = 16;
constexpr long kLatency = 17;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Case {
    long long re, im, angle;
};

struct Gate {
    Vcordic_iter dut;
    long outputs = 0, mismatches = 0, cycle = 0, aborted = 0, back_to_back = 0;
    uint32_t rng = 0x1234567;

    uint32_t rand() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return rng;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
    }

    void error(const char* what, const Case& c) {
        if (mismatches < 10) std::fprintf(stderr, "(%lld, %lld): %s\n", c.re, c.im, what);
        ++mismatches;
    }

    // Offers c on the next edge (busy is low); returns with out_valid seen or the abort done.
    void run(const Case& c, size_t n) {
        dut.in_valid = 1;
        dut.in_re = static_cast<uint64_t>(c.re) & ((1ull << kInBits) - 1);
        dut.in_im = static_cast<uint64_t>(c.im) & ((1ull << kInBits) - 1);
        tick();
        dut.in_valid = 0;
        const bool abort = n % 50 == 49;
        const long at = abort ? static_cast<long>(rand() % kLatency) : -1;
        // State seen after edge e (e = 0: the taking edge).
        for (long e = 0; e <= kLatency + 3; ++e) {
            if (e == at) {
                const bool reset = n % 150 == 149;
                if (reset) dut.rst_n = 0; else dut.drop = 1;
                tick();
                dut.rst_n = 1;
                dut.drop = 0;
                ++aborted;
                for (int k = 0; k < kLatency + 3; ++k) {
                    if (dut.out_valid) error("output after an abort", c);
                    if (dut.busy) error("busy after an abort", c);
                    tick();
                }
                return;
            }
            if (dut.out_valid) {
                if (e != kLatency) error("latency", c);
                if (dut.busy) error("busy with the output", c);
                if (sext(dut.out_angle, kAngleBits) != c.angle) error("angle", c);
                ++outputs;
                return;
            }
            if (!dut.busy) error("busy fell before the output", c);
            tick();
        }
        error("no output", c);
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_cordic_iter <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Case> cases;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        Case c{};
        if (!(f >> c.re >> c.im >> c.angle)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        cases.push_back(c);
    }
    Gate g;
    g.dut.rst_n = 0;
    g.dut.drop = 0;
    g.dut.in_valid = 0;
    g.tick();
    g.tick();
    g.dut.rst_n = 1;
    for (size_t n = 0; n < cases.size(); ++n) {
        // In the output's cycle (back to back) every third time, otherwise a gap of 1 to 3.
        if (n % 3 != 0 || g.dut.busy) {
            const int gap = 1 + static_cast<int>(g.rand() % 3);
            for (int k = 0; k < gap; ++k) g.tick();
        } else {
            ++g.back_to_back;
        }
        g.run(cases[n], n);
    }
    for (int k = 0; k < 40; ++k) {
        g.tick();
        if (g.dut.out_valid) g.error("an output after the last", cases.back());
    }
    const long expect = static_cast<long>(cases.size()) - g.aborted;
    std::printf("cordic_iter G3: %zu cases, %ld outputs, %ld aborted, %ld back to back, "
                "%ld mismatches\n",
                cases.size(), g.outputs, g.aborted, g.back_to_back, g.mismatches);
    if (g.outputs == 0 || g.aborted == 0 || g.back_to_back == 0) {
        std::cerr << "a path was never exercised — refusing to pass\n";
        return 1;
    }
    return g.mismatches == 0 && g.outputs == expect ? 0 : 1;
}
