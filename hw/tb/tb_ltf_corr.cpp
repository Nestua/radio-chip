// G3 gate for the LTF search correlator: replays hw/vectors/ltf_corr.txt through the RTL and
// requires every output to match radio_phy::rx_front_fixed::ltf_corr bit for bit.
//
// Each `reset` segment starts from reset. The block is folded over 8 clocks (ADR 0008,
// decision 4): a sample is fed every 8th clock, with one more idle cycle after every 5th and
// two more after every 13th, so the gate also checks that the window advances only on valid
// samples and that the arithmetic does not depend on what follows its 8 clocks. Every output
// must come exactly kLatency clocks after the clock edge that took its sample (the decision's
// schedule counts on it) and match the model. The first segment is then replayed with every
// 97th sample followed by the next after 5 clocks instead of 8: the early sample abandons the
// correlation in hand (no output for it) and its own output is the model's.
// Exit code 0 = gate passed.

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
constexpr int kPeriod = 8;    // clocks per sample
constexpr int kLatency = 14;  // clock edges from the one that takes a sample to its out_valid

long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

struct Segment {
    std::string label;
    std::vector<std::pair<long, long>> z;
    std::vector<bool> has;  // the sample completes a full window: an output is due
    std::deque<std::pair<long, long>> want;
};

struct Gate {
    Vltf_corr dut;
    long outputs = 0, mismatches = 0;
    long ticks = 0;
    std::deque<long> due;  // the tick at which each pending output must appear
    std::deque<std::pair<long, long>>* want = nullptr;
    const std::string* label = nullptr;

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++ticks;
        if (dut.out_valid) check();
        if (!due.empty() && due.front() <= ticks) {
            // Due at this edge (or earlier) and not consumed by check(): late or missing.
            if (mismatches < 10)
                std::fprintf(stderr, "%s: no output at tick %ld\n", label->c_str(), due.front());
            ++mismatches;
            due.pop_front();
        }
    }

    void check() {
        const long re = sext(dut.out_re, kCBits), im = sext(dut.out_im, kCBits);
        if (due.empty() || due.front() != ticks) {
            if (mismatches < 10)
                std::fprintf(stderr, "%s: output at tick %ld, due at %ld\n", label->c_str(), ticks,
                             due.empty() ? -1L : due.front());
            ++mismatches;
        } else {
            due.pop_front();
        }
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

    // `early`: the sample after every 97th comes 5 clocks after it, which abandons it.
    void run(Segment s, bool early = false) {
        if (early) {
            std::deque<std::pair<long, long>> kept;
            for (size_t n = 0; n < s.z.size(); ++n) {
                if (!s.has[n]) continue;
                if (n % 97 == 96 && n + 1 < s.z.size()) s.has[n] = false;
                else kept.push_back(s.want.front());
                s.want.pop_front();
            }
            s.want = kept;
            s.label += "_early";
        }
        want = &s.want;
        label = &s.label;
        const size_t expected = s.want.size();
        const long before = outputs;
        due.clear();
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (size_t n = 0; n < s.z.size(); ++n) {
            dut.in_valid = 1;
            dut.in_re = static_cast<uint32_t>(s.z[n].first) & ((1u << kZBits) - 1);
            dut.in_im = static_cast<uint32_t>(s.z[n].second) & ((1u << kZBits) - 1);
            // This tick is edge ticks + 1: the one that takes the sample.
            if (s.has[n]) due.push_back(ticks + 1 + kLatency);
            tick();
            dut.in_valid = 0;
            if (early && n % 97 == 96) {
                for (int i = 1; i < 5; ++i) tick();
                continue;
            }
            for (int i = 1; i < kPeriod; ++i) tick();
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
        s.has.push_back(c != "-");
        if (c != "-") s.want.emplace_back(std::stol(c), std::stol(d));
    }

    Gate g;
    long samples = 0;
    for (auto& s : segments) {
        samples += static_cast<long>(s.z.size());
        g.run(s);
    }
    if (!segments.empty()) g.run(segments.front(), true);
    std::printf("ltf_corr G3: %zu segments, %ld samples, %ld outputs, %ld mismatches\n",
                segments.size(), samples, g.outputs, g.mismatches);
    if (g.outputs == 0) {
        std::cerr << "no outputs compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
