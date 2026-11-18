// G3 gate for the CORDIC vectoring block: replays hw/vectors/cordic.txt through the RTL and
// requires every angle to match radio_phy::rx_front_fixed::cordic_angle bit for bit.
//
// Inputs are fed back to back, with an idle cycle (in_valid = 0) after every 7th, so the gate
// also checks that results follow valid inputs only. Every output must come exactly kLatency
// clocks after its input. In the middle of the file, five inputs are still in the pipeline when
// reset arrives: none of them may come out. Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vcordic.h"
#include "verilated.h"

namespace {

constexpr int kInBits = 34;
constexpr int kAngleBits = 16;
// Clock edges from the one that takes an input to the one after which its out_valid is seen.
constexpr long kLatency = 17;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Case {
    long long re, im, angle;
    long cycle = 0;
};

struct Gate {
    Vcordic dut;
    long outputs = 0, mismatches = 0, cycle = 0;
    std::deque<Case> pending;

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (dut.out_valid) check();
    }

    void check() {
        const long long a = sext(dut.out_angle, kAngleBits);
        if (pending.empty()) {
            if (mismatches < 10) std::fprintf(stderr, "unexpected output %lld\n", a);
            ++mismatches;
            return;
        }
        const Case c = pending.front();
        pending.pop_front();
        if (cycle - c.cycle != kLatency) {
            if (mismatches < 10)
                std::fprintf(stderr, "(%lld, %lld): latency %ld, expected %ld\n", c.re, c.im,
                             cycle - c.cycle, kLatency);
            ++mismatches;
        }
        if (a != c.angle) {
            if (mismatches < 10)
                std::fprintf(stderr, "(%lld, %lld): rtl=%lld golden=%lld\n", c.re, c.im, a,
                             c.angle);
            ++mismatches;
        }
        ++outputs;
    }

    // Reset; anything still in the pipeline must be discarded (an output after this is an
    // unexpected one).
    void reset() {
        pending.clear();
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
    }

    void drain() {
        dut.in_valid = 0;
        for (int i = 0; i < 64; ++i) tick();
        if (!pending.empty()) {
            std::fprintf(stderr, "%zu results missing\n", pending.size());
            ++mismatches;
            pending.clear();
        }
    }

    void feed(const Case& c, size_t n) {
        dut.in_valid = 1;
        dut.in_re = static_cast<uint64_t>(c.re) & ((1ull << kInBits) - 1);
        dut.in_im = static_cast<uint64_t>(c.im) & ((1ull << kInBits) - 1);
        Case d = c;
        d.cycle = cycle + 1;
        pending.push_back(d);
        tick();
        dut.in_valid = 0;
        if (n % 7 == 6) tick();
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_cordic <vectors.txt>\n";
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
    g.reset();
    for (size_t n = 0; n < cases.size(); ++n) {
        g.feed(cases[n], n);
        if (n == cases.size() / 2) {
            g.drain();
            for (size_t k = 0; k < 5; ++k) g.feed(cases[k], 0);  // in flight at reset
            g.reset();
        }
    }
    g.drain();
    std::printf("cordic G3: %zu cases, %ld outputs, %ld mismatches\n", cases.size(), g.outputs,
                g.mismatches);
    if (g.outputs == 0) {
        std::cerr << "no outputs compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.outputs == static_cast<long>(cases.size()) ? 0 : 1;
}
