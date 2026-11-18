// G3 gate for the receiver's step 1, DC blocker (hw/rtl/dc_block.sv) into detector A with its
// run counter (hw/rtl/det_a.sv): replays hw/vectors/det.txt through hw/tb/det_chain.sv and
// requires y, fire and event of every sample to match radio_phy::rx_front_fixed::
// detector_trace_fixed bit for bit.
//
// Each `reset` segment starts from reset (the blocker's start-up gear and the detector's
// windows restart). Samples are fed with an idle cycle after every 5th and two after every
// 13th; y must come exactly kYLatency clock edges after its sample and fire/event kDLatency
// edges after it. Each segment ends with three junk samples whose fire/event are still in the
// pipeline at the next reset: none may come out. Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vdet_chain.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr int kYBits = 13;
constexpr long kYLatency = 0;
constexpr long kDLatency = 4;

long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

struct Sample {
    long xr, xi, yr, yi, fire, event;
    long cycle = 0;
};

struct Segment {
    std::string label;
    std::vector<Sample> s;
};

struct Gate {
    Vdet_chain dut;
    long cycle = 0, ys = 0, ds = 0, mismatches = 0;
    std::deque<Sample> want_y, want_d;
    const std::string* label = nullptr;
    bool junk = false;  // y of junk samples comes out before the reset (latency 0): not compared

    void fail(const char* what, long idx, long rtl_a, long rtl_b, long gold_a, long gold_b) {
        if (mismatches < 12)
            std::fprintf(stderr, "%s %s %ld: rtl=(%ld,%ld) golden=(%ld,%ld)\n", label->c_str(),
                         what, idx, rtl_a, rtl_b, gold_a, gold_b);
        ++mismatches;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (dut.y_valid && !junk) {
            if (want_y.empty()) {
                fail("unexpected y", ys, 0, 0, 0, 0);
            } else {
                const Sample w = want_y.front();
                want_y.pop_front();
                const long re = sext(dut.y_re, kYBits), im = sext(dut.y_im, kYBits);
                if (cycle - w.cycle != kYLatency) fail("y latency", ys, cycle - w.cycle, 0, kYLatency, 0);
                if (re != w.yr || im != w.yi) fail("y", ys, re, im, w.yr, w.yi);
                ++ys;
            }
        }
        if (dut.d_valid) {
            if (want_d.empty()) {
                fail("unexpected fire/event", ds, 0, 0, 0, 0);
            } else {
                const Sample w = want_d.front();
                want_d.pop_front();
                if (cycle - w.cycle != kDLatency) fail("fire latency", ds, cycle - w.cycle, 0, kDLatency, 0);
                if (dut.fire != w.fire || dut.event_o != w.event)
                    fail("fire/event", ds, dut.fire, dut.event_o, w.fire, w.event);
                ++ds;
            }
        }
    }

    void run(const Segment& seg) {
        label = &seg.label;
        want_y.clear();
        want_d.clear();
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (size_t n = 0; n < seg.s.size(); ++n) {
            Sample x = seg.s[n];
            x.cycle = cycle + 1;
            dut.in_valid = 1;
            dut.in_re = static_cast<uint32_t>(x.xr) & ((1u << kXBits) - 1);
            dut.in_im = static_cast<uint32_t>(x.xi) & ((1u << kXBits) - 1);
            want_y.push_back(x);
            want_d.push_back(x);
            tick();
            dut.in_valid = 0;
            dut.in_re = 0x7ff;  // garbage while idle
            if (n % 5 == 4) tick();
            if (n % 13 == 12) {
                tick();
                tick();
            }
        }
        dut.in_valid = 0;
        for (int i = 0; i < 32; ++i) tick();
        if (!want_y.empty() || !want_d.empty()) {
            std::fprintf(stderr, "%s: %zu y and %zu fire outputs missing\n", seg.label.c_str(),
                         want_y.size(), want_d.size());
            ++mismatches;
        }
        // Junk in flight when the next segment's reset arrives: it must not come out.
        junk = true;
        for (int k = 0; k < 3; ++k) {
            dut.in_valid = 1;
            dut.in_re = 0x7ff;
            dut.in_im = 0x800;
            tick();
        }
        dut.in_valid = 0;
        dut.rst_n = 0;
        tick();
        junk = false;
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_det <vectors.txt>\n";
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
        x.xr = std::stol(a);
        if (segments.empty() || !(f >> x.xi >> x.yr >> x.yi >> x.fire >> x.event)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        segments.back().s.push_back(x);
    }
    Gate g;
    long samples = 0, fires = 0, events = 0;
    for (const auto& s : segments) {
        samples += static_cast<long>(s.s.size());
        for (const auto& x : s.s) {
            fires += x.fire;
            events += x.event;
        }
        g.run(s);
    }
    std::printf("det G3: %zu segments, %ld samples (%ld fires, %ld events), %ld y and %ld "
                "fire/event outputs, %ld mismatches\n",
                segments.size(), samples, fires, events, g.ys, g.ds, g.mismatches);
    if (g.ys == 0 || g.ds == 0) {
        std::cerr << "no outputs compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.ys == samples && g.ds == samples ? 0 : 1;
}
