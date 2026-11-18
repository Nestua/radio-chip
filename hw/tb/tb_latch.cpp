// G3 gate for the provisional latch and the delayed NCO path (hw/rtl/rx_latch.sv) in the chain
// dc_block → det_a → rx_latch → nco (hw/tb/latch_chain.sv): replays hw/vectors/latch.txt and
// requires the event, the NCO step in effect and the rotated sample z of every sample to match
// radio_phy::rx_front_fixed::RxFrontFixed::push_traced bit for bit.
//
// usage: sim_latch <vectors.txt> <R> <D>   R = clocks per sample, D = the build's D_SAMPLES.
//
// Samples come every R clocks, with one more idle clock after every 5th and two after every
// 13th (idle inputs carry garbage). Sample k leaves the latch when sample k + D enters, so each
// segment is followed by D flush samples, whose own outputs never appear. Exact latencies:
// event kDLatency edges after its sample; step and z kSLatency and kZLatency edges after sample
// k + D. Three junk samples are in flight at each reset: nothing of them may come out.
// rx_latch's own timing check ($error) aborts the run if a latch is late. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vlatch_chain.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr int kZBits = 12;
constexpr int kPBits = 22;
constexpr long kDLatency = 4;
constexpr long kSLatency = 1;
constexpr long kZLatency = 5;

long sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return static_cast<long>(v ^ m) - static_cast<long>(m);
}

struct Sample {
    long xr, xi, acq, event, step, zr, zi;
};

struct Segment {
    std::string label;
    std::vector<Sample> s;
};

struct Gate {
    Vlatch_chain dut;
    int R = 8, D = 3;
    long cycle = 0, mismatches = 0;
    long es = 0, ss = 0, zs = 0;          // totals compared
    size_t ne = 0, ns = 0, nz = 0;        // per-segment output indices
    const Segment* seg = nullptr;
    std::vector<long> fed;                // cycle of each fed sample (segment + flush)
    bool junk = false;                    // junk before a reset pushes flush samples out: not compared
    long after_reset = 0;                 // outputs seen after a reset (must stay 0)
    bool quiet = false;                   // between a reset and the next segment

    void fail(const char* what, size_t idx, long ra, long rb, long ga, long gb) {
        if (mismatches < 12)
            std::fprintf(stderr, "%s %s %zu: rtl=(%ld,%ld) golden=(%ld,%ld)\n",
                         seg->label.c_str(), what, idx, ra, rb, ga, gb);
        ++mismatches;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        const size_t n = seg->s.size();
        if (quiet || junk) {
            if (quiet) after_reset += dut.d_valid + dut.s_valid + dut.z_valid;
            return;
        }
        if (dut.d_valid) {
            if (ne >= fed.size()) {
                fail("unexpected event output", ne, 0, 0, 0, 0);
            } else {
                if (cycle - fed[ne] != kDLatency) fail("event latency", ne, cycle - fed[ne], 0, kDLatency, 0);
                if (ne < n) {
                    if (dut.event_o != seg->s[ne].event) fail("event", ne, dut.event_o, 0, seg->s[ne].event, 0);
                    ++es;
                }
            }
            ++ne;
        }
        if (dut.s_valid) {
            if (ns >= n || ns + D >= fed.size()) {
                fail("unexpected step output", ns, 0, 0, 0, 0);
            } else {
                const long lat = cycle - fed[ns + D];
                if (lat != kSLatency) fail("step latency", ns, lat, 0, kSLatency, 0);
                const long st = sext(dut.s_step, kPBits);
                if (st != seg->s[ns].step) fail("step", ns, st, 0, seg->s[ns].step, 0);
                ++ss;
            }
            ++ns;
        }
        if (dut.z_valid) {
            if (nz >= n || nz + D >= fed.size()) {
                fail("unexpected z output", nz, 0, 0, 0, 0);
            } else {
                const long lat = cycle - fed[nz + D];
                if (lat != kZLatency) fail("z latency", nz, lat, 0, kZLatency, 0);
                const long re = sext(dut.z_re, kZBits), im = sext(dut.z_im, kZBits);
                if (re != seg->s[nz].zr || im != seg->s[nz].zi) fail("z", nz, re, im, seg->s[nz].zr, seg->s[nz].zi);
                ++zs;
            }
            ++nz;
        }
    }

    void feed(long xr, long xi, long acq, size_t k) {
        fed.push_back(cycle + 1);
        dut.in_valid = 1;
        dut.in_re = static_cast<uint32_t>(xr) & ((1u << kXBits) - 1);
        dut.in_im = static_cast<uint32_t>(xi) & ((1u << kXBits) - 1);
        dut.in_clear = acq;
        tick();
        dut.in_valid = 0;
        dut.in_re = 0x7ff;  // garbage while idle
        dut.in_im = 0x800;
        dut.in_clear = 1;
        int idle = R - 1;
        if (k % 5 == 4) idle += 1;
        if (k % 13 == 12) idle += 2;
        for (int i = 0; i < idle; ++i) tick();
    }

    void run(const Segment& s) {
        seg = &s;
        fed.clear();
        ne = ns = nz = 0;
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        tick();
        dut.rst_n = 1;
        const size_t n = s.s.size();
        for (size_t k = 0; k < n; ++k) feed(s.s[k].xr, s.s[k].xi, s.s[k].acq, k);
        for (int k = 0; k < D; ++k) feed(0, 0, 0, n + k);   // flush
        for (int i = 0; i < 40; ++i) tick();
        if (ne != n + D || ns != n || nz != n) {
            std::fprintf(stderr, "%s: outputs %zu events, %zu steps, %zu z; expected %zu, %zu, %zu\n",
                         s.label.c_str(), ne, ns, nz, n + D, n, n);
            ++mismatches;
        }
        // Junk in flight when the next segment's reset arrives, fed back to back behind an
        // event-rich history: after the reset nothing may come out.
        junk = true;
        for (int k = 0; k < 3; ++k) {
            dut.in_valid = 1;
            dut.in_re = 0x7ff;
            dut.in_im = 0x800;
            dut.in_clear = 0;
            tick();
        }
        junk = false;
        quiet = true;
        dut.in_valid = 0;
        dut.rst_n = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (int i = 0; i < 40; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0, 0, 0);
            after_reset = 0;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 4) {
        std::cerr << "usage: sim_latch <vectors.txt> <R> <D>\n";
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
        if (segments.empty() || !(f >> x.xi >> x.acq >> x.event >> x.step >> x.zr >> x.zi)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        segments.back().s.push_back(x);
    }
    Gate g;
    g.R = std::atoi(argv[2]);
    g.D = std::atoi(argv[3]);
    if (g.R < 1 || g.D < 1) {
        std::cerr << "bad R or D\n";
        return 2;
    }
    long samples = 0, events = 0, steps = 0, acqs = 0;
    for (const auto& s : segments) {
        samples += static_cast<long>(s.s.size());
        long prev = 0;
        for (const auto& x : s.s) {
            events += x.event;
            acqs += x.acq;
            steps += x.step != prev;
            prev = x.step;
        }
        g.run(s);
    }
    std::printf("latch G3 (R = %d, D = %d): %zu segments, %ld samples (%ld events, %ld step changes, "
                "%ld acquisitions), %ld mismatches\n",
                g.R, g.D, segments.size(), samples, events, steps, acqs, g.mismatches);
    if (g.ss == 0 || steps == 0 || events == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.es == samples && g.ss == samples && g.zs == samples ? 0 : 1;
}
