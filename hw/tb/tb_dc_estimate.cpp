// Gate G3 for the offset's arithmetic (hw/rtl/dc_estimate.sv) against
// radio_phy::rx_handoff::dc_offset_of_sums, bit for bit: for every vector the offset, and
// done in exactly the 17th cycle after start's and in no other.
//
// usage: sim_dc_estimate <vectors.txt>
//
// Per vector: the CFO word a cycle or more before start (the module's contract), the sums
// with start; 0 to 3 idle cycles between vectors, the inputs then garbage. Every 7th vector
// is first started with other sums and abandoned by a new start 1 to 16 cycles later: the
// abandoned estimate gives no done, the new one its own. Registers start from random values.
// Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vdc_estimate.h"
#include "verilated.h"

namespace {

constexpr int kLatency = 17;
constexpr int kXBits = 12;
constexpr int kSBits = 18;
constexpr int kPBits = 22;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Vec {
    long long cfo, s1r, s1i, s2r, s2i, dcr, dci;
};

struct Gate {
    Vdc_estimate dut;
    long mismatches = 0, dones = 0, stray = 0, abandoned = 0;

    void fail(const char* what, size_t k, long long r, long long g) {
        if (mismatches < 16) std::fprintf(stderr, "%s, vector %zu: rtl=%lld golden=%lld\n", what, k, r, g);
        ++mismatches;
    }

    // One cycle: the inputs as set, the outputs of this cycle looked at, the edge.
    bool cycle() {
        dut.clk = 0;
        dut.eval();
        const bool done = dut.done;
        dut.clk = 1;
        dut.eval();
        return done;
    }

    void sums(const Vec& v, long long twist) {
        const uint32_t m = (1u << kSBits) - 1;
        dut.s1_re = static_cast<uint32_t>(v.s1r ^ twist) & m;
        dut.s1_im = static_cast<uint32_t>(v.s1i + twist) & m;
        dut.s2_re = static_cast<uint32_t>(v.s2r - twist) & m;
        dut.s2_im = static_cast<uint32_t>(v.s2i ^ (twist << 3)) & m;
    }

    void idle(int cycles) {
        dut.start = 0;
        for (int i = 0; i < cycles; ++i) stray += cycle();
    }

    void run(const Vec& v, size_t k) {
        dut.cfo = static_cast<uint32_t>(v.cfo) & ((1u << kPBits) - 1);
        idle(1 + static_cast<int>(k % 4));
        if (k % 7 == 3) {
            // An estimate started on other sums and abandoned.
            sums(v, 0x1234 + static_cast<long long>(k));
            dut.start = 1;
            stray += cycle();
            idle(static_cast<int>(k % 16));
            ++abandoned;
        }
        sums(v, 0);
        dut.start = 1;
        stray += cycle();
        dut.start = 0;
        for (int c = 1; c <= kLatency; ++c) {
            const bool done = cycle();
            if (c < kLatency && done) fail("done early, cycles after start", k, c, kLatency);
            if (c == kLatency) {
                if (!done) fail("no done in its cycle", k, 0, 1);
                dones += done;
                const long long r = sext(dut.dc_re, kXBits), i = sext(dut.dc_im, kXBits);
                if (r != v.dcr) fail("offset's I", k, r, v.dcr);
                if (i != v.dci) fail("offset's Q", k, i, v.dci);
            }
        }
        // The offset stays; the inputs may go.
        sums(v, 0x2aaaa);
        dut.cfo = ~dut.cfo & ((1u << kPBits) - 1);
        idle(2);
        if (sext(dut.dc_re, kXBits) != v.dcr || sext(dut.dc_im, kXBits) != v.dci)
            fail("offset moved after done", k, sext(dut.dc_re, kXBits), v.dcr);
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_dc_estimate <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Vec> vecs;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        Vec v;
        if (!(f >> v.cfo >> v.s1r >> v.s1i >> v.s2r >> v.s2i >> v.dcr >> v.dci)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        vecs.push_back(v);
    }
    Gate g;
    g.dut.rst_n = 0;
    g.dut.start = 0;
    g.cycle();
    g.cycle();
    g.dut.rst_n = 1;
    for (size_t k = 0; k < vecs.size(); ++k) g.run(vecs[k], k);
    if (g.stray != 0) g.fail("done outside its cycle", 0, g.stray, 0);
    std::printf("dc_estimate G3: %zu vectors, %ld offsets compared, %ld estimates abandoned, %ld mismatches\n",
                vecs.size(), g.dones, g.abandoned, g.mismatches);
    if (vecs.empty() || g.dones == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
