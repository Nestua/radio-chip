// G3 gate for the descrambler and octets (hw/rtl/rx_octets.sv, docs/gates/rx_octets.md):
// replays hw/vectors/rx_octets.txt (rvec rx_octets, from radio_phy::rx_octets::RxOctets) a
// cycle at a time at the block's pins and requires every octet and end in the cycle the
// vectors give, with its fields, and nothing in any other cycle.
// The length input carries a random value in every cycle without a start (it must be taken
// only with the start). Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "Vrx_octets.h"
#include "verilated.h"

struct Octet {
    unsigned data, first, last, seed, service;
};
struct Segment {
    unsigned length = 0;
    std::string cycles;
    std::map<size_t, Octet> octets;
    std::map<size_t, unsigned> ends;  // cycle -> end_seed
};

static void tick(Vrx_octets& dut) {
    dut.clk = 0;
    dut.eval();
    dut.clk = 1;
    dut.eval();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_rx_octets <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Segment> segs;
    unsigned length = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string tag;
        f >> tag;
        if (tag == "L") {
            f >> std::hex >> length;
        } else if (tag == "I") {
            Segment s;
            s.length = length;
            f >> s.cycles;
            segs.push_back(s);
        } else if (tag == "O" && !segs.empty()) {
            size_t n;
            Octet o;
            f >> std::dec >> n >> std::hex >> o.data >> o.first >> o.last >> o.seed >> o.service;
            segs.back().octets[n] = o;
        } else if (tag == "E" && !segs.empty()) {
            size_t n;
            unsigned seed;
            f >> std::dec >> n >> seed;
            segs.back().ends[n] = seed;
        } else {
            std::cerr << "malformed vector line: " << line.substr(0, 80) << "\n";
            return 2;
        }
        if (f.fail()) {
            std::cerr << "malformed vector line: " << line.substr(0, 80) << "\n";
            return 2;
        }
    }

    Vrx_octets dut;
    std::mt19937 rng(7);
    dut.rst_n = 0;
    dut.start = 0;
    dut.drop = 0;
    dut.in_valid = 0;
    dut.in_bit = 0;
    dut.length = 0;
    tick(dut);
    tick(dut);
    dut.rst_n = 1;

    long cycles = 0, octets = 0, ends = 0, mismatches = 0;
    auto bad = [&](size_t seg, size_t n, const char* what, unsigned got, unsigned want) {
        if (mismatches < 20)
            std::fprintf(stderr, "segment %zu cycle %zu: %s rtl=%x golden=%x\n", seg, n, what, got,
                         want);
        ++mismatches;
    };
    for (size_t si = 0; si < segs.size(); ++si) {
        const Segment& s = segs[si];
        for (size_t n = 0; n < s.cycles.size(); ++n) {
            const char c = s.cycles[n];
            dut.start = c == 'S' || c == 's' || c == 't' || c == 'X';
            dut.drop = c == 'D' || c == 'a' || c == 'b' || c == 'X';
            dut.in_valid = c == '0' || c == '1' || c == 's' || c == 't' || c == 'a' || c == 'b';
            dut.in_bit = c == '1' || c == 't' || c == 'b';
            dut.length = dut.start ? s.length : (rng() & 0xfff);
            if (!dut.in_valid) dut.in_bit = rng() & 1;  // must not matter
            tick(dut);
            ++cycles;

            auto o = s.octets.find(n);
            if (o == s.octets.end()) {
                if (dut.oct_valid) bad(si, n, "oct_valid", 1, 0);
            } else {
                ++octets;
                const Octet& w = o->second;
                if (!dut.oct_valid) bad(si, n, "oct_valid", 0, 1);
                if (dut.oct_data != w.data) bad(si, n, "oct_data", dut.oct_data, w.data);
                if (dut.oct_first != w.first) bad(si, n, "oct_first", dut.oct_first, w.first);
                if (dut.oct_last != w.last) bad(si, n, "oct_last", dut.oct_last, w.last);
                if (w.first) {
                    if (dut.oct_seed != w.seed) bad(si, n, "oct_seed", dut.oct_seed, w.seed);
                    if (dut.oct_service != w.service)
                        bad(si, n, "oct_service", dut.oct_service, w.service);
                }
            }
            auto e = s.ends.find(n);
            if (e == s.ends.end()) {
                if (dut.end_valid) bad(si, n, "end_valid", 1, 0);
            } else {
                ++ends;
                if (!dut.end_valid) bad(si, n, "end_valid", 0, 1);
                if (dut.end_seed != e->second) bad(si, n, "end_seed", dut.end_seed, e->second);
            }
        }
    }
    dut.final();

    std::printf("rx_octets G3: %zu segments, %ld cycles, %ld octets, %ld ends, %ld mismatches\n",
                segs.size(), cycles, octets, ends, mismatches);
    if (octets == 0 || ends == 0) {
        std::cerr << "no octets or no ends read — refusing to pass an empty gate\n";
        return 1;
    }
    return mismatches == 0 ? 0 : 1;
}
