// G3 gate for the decision's choice of T1 on full-width energies (hw/rtl/rx_decide.sv in
// hw/tb/decide_alone.sv): replays hw/vectors/decide_t1.txt and requires, at every result
// (res_valid), T1 (acq_t1, modulo HISTORY) and whether the decision went on to its sums
// (res_sums: not when T1 is less than 160 samples into the stream) to match
// radio_phy::rx_front_fixed::latest_start, the rule RxFrontFixed::decide calls.
//
// Why: rx_decide sees the search's 44-bit energies in two places only — keep = emax − emax/8
// from acc_emax, and ring_data ≥ keep over the grid — and the gate `decide` gives it the chain's
// energies, which stay within 32 bits. Here the candidate and the ring are the testbench's: the
// maxima up to the bound w · 2^39, the grid's words about the threshold.
//
// usage: sim_decide_t1 <vectors.txt> <W>   (the build's W; L the default)
//
// Per candidate: the energy ring (81 words) holds the grid at the candidate's ring address on,
// wrapping; every other word is 2^44 − 1 or 0 in turn (a read off the grid moves T1). The
// ring answers a read in the next cycle and shows garbage otherwise. The history is written a
// sample every 8 clocks (zeros and ones); acc_valid for one cycle where the front would give it
// (acc_n = imax + 189 + 2w, the write port 4 samples on), so the history's reads stay behind
// its write and the result is due L samples later; the CORDIC is the real one behind the
// arbiter. Only T1 and res_sums are compared (the history is not a frame: gate `decide`). The next candidate comes after dec_valid. The RTL's own checks
// ($error) abort the run. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vdecide_alone.h"
#include "verilated.h"

namespace {

constexpr int kRing = 81;
constexpr long kPosMod = 512;
constexpr int kR = 8;
constexpr uint64_t kAllOnes44 = (1ULL << 44) - 1;

struct Case {
    int w = 0;
    long imax = 0, lapped = 0, ring = 0, t1 = 0;
    long long emax = 0;
    std::vector<long long> grid;
};

struct Gate {
    Vdecide_alone dut;
    long cycle = 0, samples = 0, mismatches = 0;
    uint64_t ring[kRing] = {};
    uint32_t lfsr = 0xACE1u;

    uint32_t rnd() {
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        return lfsr;
    }

    void fail(const char* what, size_t idx, long long r, long long g) {
        if (mismatches < 12) std::fprintf(stderr, "case %zu %s: rtl=%lld golden=%lld\n", idx, what, r, g);
        ++mismatches;
    }

    // One clock: inputs for the coming edge are set; the ring answers the read it takes.
    void tick() {
        dut.y_valid = (cycle % kR) == 0;
        dut.y_re = dut.y_valid ? (rnd() & 1) : 0x1fff;
        dut.y_im = dut.y_valid ? (rnd() & 1) : 0x1000;
        dut.clk = 0;
        dut.eval();
        const bool re = dut.ring_re;
        const uint32_t ad = dut.ring_addr;
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (dut.y_valid) ++samples;   // the edge took it: the write port's count
        dut.acc_valid = 0;
        if (re && ad < kRing) {
            dut.ring_data = ring[ad];
        } else {
            dut.ring_data = ((static_cast<uint64_t>(rnd()) << 32) | rnd()) & kAllOnes44;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 3) {
        std::cerr << "usage: sim_decide_t1 <vectors.txt> <W>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    const int W = std::atoi(argv[2]);
    std::vector<Case> cases;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        Case c;
        if (!(f >> c.w >> c.imax >> c.lapped >> c.ring >> c.emax)) {
            std::cerr << "malformed line: " << line << "\n";
            return 2;
        }
        c.grid.resize(static_cast<size_t>(c.w) + 1);
        for (auto& e : c.grid)
            if (!(f >> e)) {
                std::cerr << "malformed line: " << line << "\n";
                return 2;
            }
        if (!(f >> c.t1)) {
            std::cerr << "malformed line: " << line << "\n";
            return 2;
        }
        if (c.w == W) cases.push_back(c);
    }
    Gate g;
    g.dut.rst_n = 0;
    g.dut.acc_valid = 0;
    for (int i = 0; i < 4; ++i) g.tick();
    g.dut.rst_n = 1;
    for (int i = 0; i < 20; ++i) g.tick();
    long results = 0, no_decision = 0, wide = 0;
    for (size_t i = 0; i < cases.size(); ++i) {
        const Case& c = cases[i];
        const uint64_t junk = (i % 2 == 0) ? kAllOnes44 : 0;
        for (auto& v : g.ring) v = junk;
        for (int k = 0; k <= c.w; ++k) g.ring[(c.ring + k) % kRing] = static_cast<uint64_t>(c.grid[static_cast<size_t>(k)]);
        // The candidate where the front would accept it: at sample n = imax + 63 + w (its age)
        // + 126 + w (the correlator's span), the history's write port D = 4 samples on (the
        // search runs on the NCO path), in a cycle between samples. The decision reads the
        // history from T1 − 160 to T1 + 126 + w, all behind the write port then.
        const long n = c.imax + 189 + 2L * c.w;
        while ((g.samples - n - 4) % kPosMod != 0) g.tick();
        while (g.cycle % kR != 3) g.tick();
        g.dut.acc_valid = 1;
        g.dut.acc_n = static_cast<uint32_t>(n % kPosMod);
        g.dut.acc_imax = static_cast<uint32_t>(c.imax);
        g.dut.acc_lapped = static_cast<uint8_t>(c.lapped);
        g.dut.acc_ring = static_cast<uint32_t>(c.ring);
        g.dut.acc_emax = static_cast<uint64_t>(c.emax);
        g.tick();
        g.dut.acc_emax = 0;   // held only in the strobe's cycle
        g.dut.acc_imax = 0;
        bool got = false, done = false;
        for (long t = 0; t < 2000 && !done; ++t) {
            if (g.dut.res_valid) {
                if (got) g.fail("second result", i, 1, 0);
                got = true;
                const bool early = c.lapped == 0 && c.t1 < 160;
                if (static_cast<long>(g.dut.acq_t1) != c.t1) g.fail("T1", i, g.dut.acq_t1, c.t1);
                if (g.dut.res_sums != !early) g.fail("res_sums", i, g.dut.res_sums, !early);
                ++results;
                no_decision += early;
                wide += c.grid[static_cast<size_t>((c.t1 - c.imax + kPosMod) % kPosMod)] >= (1LL << 32);
            }
            if (g.dut.dec_valid) done = true;
            g.tick();
        }
        if (!got) g.fail("no result", i, 0, 1);
        if (!done) g.fail("no delivery", i, 0, 1);
    }
    std::printf("decide_t1 G3 (W = %d): %zu candidates, %ld results (%ld with no decision, %ld T1 words above 32 "
                "bits), %ld mismatches\n",
                W, cases.size(), results, no_decision, wide, g.mismatches);
    if (cases.empty() || results == 0 || no_decision == 0 || wide == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && results == static_cast<long>(cases.size()) ? 0 : 1;
}
