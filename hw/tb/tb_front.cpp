// Gate G3 for the receiver front's top level (hw/rtl/rx_front.sv), at its pins: only the ADC
// samples go in, only the acquisitions come out. Required, against
// radio_phy::rx_front_fixed::RxFrontFixed::push bit for bit:
//   - an acquisition (acq_valid) at exactly the samples the model applies one at, in the
//     cycle after the kDecLatency-th clock edge from the edge that takes the sample, and in
//     no other;
//   - in that cycle the FFT window's start and T1 (modulo the history's depth) and the three
//     phase words. The data pins are not looked at in any other cycle.
// What happens inside the front is the gate `decide`'s (hw/tb/tb_decide.cpp), which runs on
// this top level through hw/tb/decide_chain.sv.
//
// usage: sim_front <vectors.txt> <R> <D> <W>   (R at least 8: ltf_corr; D is the build's
// D_SAMPLES, for the run line only)
//
// Samples come every R clocks, with one more idle clock after every 5th and two after every
// 13th (idle inputs carry garbage). A segment is one run of the model from reset, nothing
// reset inside it. Before each segment the front is reset four times on the way to the
// segment's first acquisition (the segment fed up to 1 + 37·k mod 190 samples before that
// acquisition's sample, k counting the resets of the run, and 0 to 7 clocks more; a
// one-clock reset): less than 64 samples before, the decision is in flight; more, the search
// or the preamble's detection. After each, kSilence samples of zeros are fed, more than a lap
// of the history: no acquisition may come out (none can be made of zeros; one left over from
// before the reset would be delivered at its sample's count). The RTL's own checks ($error)
// abort the run. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vrx_front.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr int kPBits = 22;
constexpr long kDecLatency = 3;    // acq_valid, edges after the sample it is applied at
constexpr long kPosMod = 512;
constexpr int kSilence = 520;      // zero samples after a reset in flight

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Acq {
    long long fft = 0, t1 = 0, cfo = 0, coarse = 0, fine = 0;
};

struct Sample {
    long xr = 0, xi = 0;
    int acq = -1;   // index into the segment's records
};

struct Segment {
    std::string label;
    int w = 0;
    std::vector<Sample> s;
    std::vector<Acq> acq;
};

struct Gate {
    Vrx_front dut;
    int R = 8;
    long cycle = 0, mismatches = 0, acqs = 0, borrows = 0, in_flight_resets = 0;
    const Segment* seg = nullptr;
    std::vector<long> fed;
    bool junk = false, quiet = false;
    long after_reset = 0;

    void fail(const char* what, size_t idx, long long r, long long g) {
        if (mismatches < 16)
            std::fprintf(stderr, "%s %s %zu: rtl=%lld golden=%lld\n", seg->label.c_str(), what, idx, r, g);
        ++mismatches;
    }

    // The sample fed at `edge`, or −1.
    long sample_at(long edge) const {
        for (size_t k = fed.size(); k-- > 0;) {
            if (fed[k] == edge) return static_cast<long>(k);
            if (fed[k] < edge) break;
        }
        return -1;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (quiet) after_reset += dut.acq_valid;
        if (quiet || junk || !dut.acq_valid) return;
        const long q = sample_at(cycle - kDecLatency);
        if (q < 0 || static_cast<size_t>(q) >= seg->s.size() || seg->s[q].acq < 0) {
            fail("acquisition the model does not make, at sample", static_cast<size_t>(q < 0 ? 0 : q), 1, 0);
            return;
        }
        const Acq& g = seg->acq[seg->s[q].acq];
        ++acqs;
        borrows += g.fft / kPosMod != g.t1 / kPosMod;
        if (dut.acq_fft != g.fft % kPosMod) fail("FFT window's start", q, dut.acq_fft, g.fft % kPosMod);
        if (dut.acq_t1 != g.t1 % kPosMod) fail("T1", q, dut.acq_t1, g.t1 % kPosMod);
        if (sext(dut.acq_cfo, kPBits) != g.cfo) fail("cfo", q, sext(dut.acq_cfo, kPBits), g.cfo);
        if (sext(dut.acq_coarse, kPBits) != g.coarse) fail("coarse phase", q, sext(dut.acq_coarse, kPBits), g.coarse);
        if (sext(dut.acq_fine, kPBits) != g.fine) fail("fine phase", q, sext(dut.acq_fine, kPBits), g.fine);
    }

    void feed(long xr, long xi, size_t k) {
        fed.push_back(cycle + 1);
        dut.in_valid = 1;
        dut.in_re = static_cast<uint32_t>(xr) & ((1u << kXBits) - 1);
        dut.in_im = static_cast<uint32_t>(xi) & ((1u << kXBits) - 1);
        tick();
        dut.in_valid = 0;
        dut.in_re = 0x7ff;  // garbage while idle
        dut.in_im = 0x800;
        int idle = R - 1;
        if (k % 5 == 4) idle += 1;
        if (k % 13 == 12) idle += 2;
        for (int i = 0; i < idle; ++i) tick();
    }

    void reset(int clocks) {
        dut.rst_n = 0;
        dut.in_valid = 0;
        for (int i = 0; i < clocks; ++i) tick();
        dut.rst_n = 1;
    }

    // A reset on the way to the segment's first acquisition: its detection, its search, its
    // decision or the result waiting for its sample, by how far before the acquisition's
    // sample the feed stops (1 to 190 samples before it, 0 to 7 clocks more).
    void reset_in_flight(const Segment& s) {
        size_t a = 0;
        while (a < s.s.size() && s.s[a].acq < 0) ++a;
        const size_t back = 1 + (in_flight_resets * 37) % 190;
        if (a >= s.s.size() || a < back) return;
        const size_t upto = a - back;
        const int more = static_cast<int>(in_flight_resets % 8);
        ++in_flight_resets;
        junk = true;
        reset(1);
        fed.clear();
        for (size_t k = 0; k <= upto; ++k) feed(s.s[k].xr, s.s[k].xi, k);
        for (int i = 0; i < more; ++i) tick();
        junk = false;
        quiet = true;
        reset(1);
        for (int k = 0; k < kSilence; ++k) feed(0, 0, static_cast<size_t>(k));
        for (int i = 0; i < 30; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("an acquisition after a reset in flight", upto, after_reset, 0);
            after_reset = 0;
        }
    }

    void run(const Segment& s) {
        seg = &s;
        for (int k = 0; k < 4; ++k) reset_in_flight(s);
        junk = true;
        reset(1);
        junk = false;
        fed.clear();
        const long before = acqs;
        const size_t n = s.s.size();
        for (size_t k = 0; k < n; ++k) feed(s.s[k].xr, s.s[k].xi, k);
        for (int i = 0; i < 40; ++i) tick();
        if (acqs - before != static_cast<long>(s.acq.size()))
            fail("acquisitions in the segment", 0, acqs - before, static_cast<long long>(s.acq.size()));
        quiet = true;
        reset(2);
        for (int i = 0; i < 60; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0);
            after_reset = 0;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 5) {
        std::cerr << "usage: sim_front <vectors.txt> <R> <D> <W>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    Gate g;
    g.R = std::atoi(argv[2]);
    const int d = std::atoi(argv[3]);
    const int w = std::atoi(argv[4]);
    if (g.R < 8 || w < 1) {
        std::cerr << "bad R or W\n";
        return 2;
    }
    std::vector<Segment> segments;
    std::string line;
    auto bad = [&line]() {
        std::cerr << "malformed vector line: " << line << "\n";
        return 2;
    };
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string a;
        if (!(f >> a)) continue;
        if (a == "reset") {
            segments.push_back({});
            if (!(f >> segments.back().label >> segments.back().w)) return bad();
            continue;
        }
        if (segments.empty()) return bad();
        Segment& s = segments.back();
        if (a == "Q") {
            Acq x;
            if (s.s.empty() || s.s.back().acq >= 0 || !(f >> x.fft >> x.t1 >> x.cfo >> x.coarse >> x.fine)) return bad();
            s.s.back().acq = static_cast<int>(s.acq.size());
            s.acq.push_back(x);
        } else {
            Sample x;
            x.xr = std::stol(a);
            if (!(f >> x.xi)) return bad();
            s.s.push_back(x);
        }
    }
    long samples = 0, run_segments = 0, want = 0;
    size_t longest = 0;
    for (const auto& s : segments) {
        if (s.w != w) continue;
        ++run_segments;
        samples += static_cast<long>(s.s.size());
        want += static_cast<long>(s.acq.size());
        if (s.s.size() > longest) longest = s.s.size();
        g.run(s);
    }
    std::printf("front G3 (R = %d, D = %d, W = %d): %ld segments, %ld samples (the longest %zu), %ld acquisitions "
                "(%ld with the window a lap before T1), %ld resets in flight, %ld mismatches\n",
                g.R, d, w, run_segments, samples, longest, g.acqs, g.borrows, g.in_flight_resets, g.mismatches);
    if (run_segments == 0 || g.acqs == 0 || g.in_flight_resets == 0 || (w < 16 && g.borrows == 0)) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.acqs == want ? 0 : 1;
}
