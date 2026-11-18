// G3 gate for the NCO (phase accumulator, quarter-wave table, mixer): replays hw/vectors/nco.txt
// through the RTL and requires every output to match radio_phy::rx_front_fixed::nco_mix with the
// phase accumulated as RxFrontFixed does, bit for bit.
//
// Each `reset` segment starts from reset (phase 0). Samples are fed with an idle cycle after
// every 5th and two after every 13th, so the gate also checks that the phase advances only on
// valid samples. Every output must come exactly kLatency clocks after its input. Each segment
// ends with three junk samples still in the pipeline when the next reset arrives: none may come
// out. Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vnco.h"
#include "verilated.h"

namespace {

constexpr int kYBits = 13;
constexpr int kPhaseBits = 22;
constexpr int kZBits = 12;
// Clock edges from the one that takes a sample to the one after which its out_valid is seen.
constexpr long kLatency = 3;

long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

struct Sample {
    long yr, yi, step, zr, zi;
    long cycle = 0;
};

struct Segment {
    std::string label;
    std::vector<Sample> s;
};

struct Gate {
    Vnco dut;
    long outputs = 0, mismatches = 0, cycle = 0;
    std::deque<Sample> pending;
    const std::string* label = nullptr;

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (dut.out_valid) check();
    }

    void check() {
        const long re = sext(dut.out_re, kZBits), im = sext(dut.out_im, kZBits);
        if (pending.empty()) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s: unexpected output %ld %ld\n", label->c_str(), re, im);
            ++mismatches;
            return;
        }
        const Sample w = pending.front();
        pending.pop_front();
        if (cycle - w.cycle != kLatency) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s output %ld: latency %ld, expected %ld\n",
                             label->c_str(), outputs, cycle - w.cycle, kLatency);
            ++mismatches;
        }
        if (re != w.zr || im != w.zi) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s output %ld: rtl=(%ld,%ld) golden=(%ld,%ld)\n",
                             label->c_str(), outputs, re, im, w.zr, w.zi);
            ++mismatches;
        }
        ++outputs;
    }

    void run(const Segment& seg) {
        label = &seg.label;
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (size_t n = 0; n < seg.s.size(); ++n) {
            const Sample& x = seg.s[n];
            dut.in_valid = 1;
            dut.in_re = static_cast<uint32_t>(x.yr) & ((1u << kYBits) - 1);
            dut.in_im = static_cast<uint32_t>(x.yi) & ((1u << kYBits) - 1);
            dut.in_step = static_cast<uint32_t>(x.step) & ((1u << kPhaseBits) - 1);
            Sample d = x;
            d.cycle = cycle + 1;
            pending.push_back(d);
            tick();
            dut.in_valid = 0;
            dut.in_step = 0x2aaaaa;  // garbage while idle: must not move the phase
            if (n % 5 == 4) tick();
            if (n % 13 == 12) {
                tick();
                tick();
            }
        }
        dut.in_valid = 0;
        for (int i = 0; i < 32; ++i) tick();
        if (!pending.empty()) {
            std::fprintf(stderr, "%s: %zu outputs missing\n", seg.label.c_str(), pending.size());
            ++mismatches;
            pending.clear();
        }
        // Junk in flight when the next segment's reset arrives: it must not come out.
        for (int k = 0; k < static_cast<int>(kLatency); ++k) {
            dut.in_valid = 1;
            dut.in_re = 0x0fff;
            dut.in_im = 0x1001;
            dut.in_step = 0x155555;
            tick();
        }
        dut.in_valid = 0;
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_nco <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Segment> segments;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string a;
        if (!(f >> a)) continue;
        if (a == "reset") {
            segments.push_back({});
            f >> segments.back().label;
            continue;
        }
        Sample x{};
        x.yr = std::stol(a);
        if (segments.empty() || !(f >> x.yi >> x.step >> x.zr >> x.zi)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        segments.back().s.push_back(x);
    }
    Gate g;
    long samples = 0;
    for (const auto& s : segments) {
        samples += static_cast<long>(s.s.size());
        g.run(s);
    }
    std::printf("nco G3: %zu segments, %ld samples, %ld outputs, %ld mismatches\n",
                segments.size(), samples, g.outputs, g.mismatches);
    if (g.outputs == 0) {
        std::cerr << "no outputs compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.outputs == samples ? 0 : 1;
}
