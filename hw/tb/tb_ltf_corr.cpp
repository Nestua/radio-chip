// G3 gate for the LTF search correlator: replays hw/vectors/ltf_corr.txt through the RTL and
// requires every output to match radio_phy::rx_front_fixed::ltf_corr bit for bit.
//
// Each `reset` segment starts from reset. Samples are fed with an idle cycle (in_valid = 0)
// after every 5th and a two-cycle gap after every 13th, so the gate also checks that the
// window advances only on valid samples. Outputs are compared in order (the latency is the
// RTL's business; the count and order are not). Exit code 0 = gate passed.

#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Vltf_corr.h"
#include "verilated.h"

namespace {

constexpr int kZBits = 12;
constexpr int kCBits = 20;

long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

struct Segment {
    std::string label;
    std::vector<std::pair<long, long>> z;
    std::deque<std::pair<long, long>> want;
};

struct Gate {
    Vltf_corr dut;
    long outputs = 0, mismatches = 0;
    std::deque<std::pair<long, long>>* want = nullptr;
    const std::string* label = nullptr;

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        if (dut.out_valid) check();
    }

    void check() {
        const long re = sext(dut.out_re, kCBits), im = sext(dut.out_im, kCBits);
        if (want->empty()) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s: unexpected output %ld %ld\n", label->c_str(), re, im);
            ++mismatches;
            return;
        }
        const auto w = want->front();
        want->pop_front();
        if (re != w.first || im != w.second) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s output %ld: rtl=(%ld,%ld) golden=(%ld,%ld)\n",
                             label->c_str(), outputs, re, im, w.first, w.second);
            ++mismatches;
        }
        ++outputs;
    }

    void run(Segment& s) {
        want = &s.want;
        label = &s.label;
        const size_t expected = s.want.size();
        const long before = outputs;
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (size_t n = 0; n < s.z.size(); ++n) {
            dut.in_valid = 1;
            dut.in_re = static_cast<uint32_t>(s.z[n].first) & ((1u << kZBits) - 1);
            dut.in_im = static_cast<uint32_t>(s.z[n].second) & ((1u << kZBits) - 1);
            tick();
            dut.in_valid = 0;
            if (n % 5 == 4) tick();
            if (n % 13 == 12) {
                tick();
                tick();
            }
        }
        dut.in_valid = 0;
        for (int i = 0; i < 64; ++i) tick();  // drain the pipeline
        if (!s.want.empty() || outputs - before != static_cast<long>(expected)) {
            std::fprintf(stderr, "%s: %ld outputs, %zu expected\n", s.label.c_str(),
                         outputs - before, expected);
            ++mismatches;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_ltf_corr <vectors.txt>\n";
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
        std::string a, b, c, d;
        if (!(f >> a)) continue;
        if (a == "reset") {
            segments.push_back({});
            f >> segments.back().label;
            continue;
        }
        if (segments.empty() || !(f >> b >> c >> d)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        Segment& s = segments.back();
        s.z.emplace_back(std::stol(a), std::stol(b));
        if (c != "-") s.want.emplace_back(std::stol(c), std::stol(d));
    }

    Gate g;
    long samples = 0;
    for (auto& s : segments) {
        samples += static_cast<long>(s.z.size());
        g.run(s);
    }
    std::printf("ltf_corr G3: %zu segments, %ld samples, %ld outputs, %ld mismatches\n",
                segments.size(), samples, g.outputs, g.mismatches);
    if (g.outputs == 0) {
        std::cerr << "no outputs compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
