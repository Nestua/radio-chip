// G3 gate for the Viterbi decoder (hw/rtl/viterbi.sv): replays hw/vectors/viterbi.txt and
// requires every decoded bit to equal radio_phy::viterbi::decode_windowed's (window 96,
// ZeroAfter(steps)), in its cycle.
//
// The cycle contract (the module's header): a bit decided by the best state at step t comes
// out LAT edges after the edge that took step t; at a block's last step the bits not yet
// decided (min(96, steps) of them) come out of state 0's path a bit an edge from LAT edges
// after the edge that took that step, the last one with out_last. Nothing else comes out.
//
// Steps are fed with idle cycles inside the blocks (none, every fifth, or random), and the
// next block starts either at the earliest the contract allows (n + 2 edges after the last
// step, n the bits flushed) or later. Resets are pulsed with a block in flight: while its
// decisions are coming out, during its flush, and with their first edge 1 to 5 edges after its
// last step; nothing due from the reset on may come out (what was due before it must), and
// the next block must decode. Registers start from random values
// (+verilator+rand+reset+2). Exit code 0 = gate passed.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Vviterbi.h"
#include "verilated.h"

namespace {

constexpr int DEPTH = 96;
constexpr long LAT = 4;
constexpr long WATCHDOG = 4'000'000;

struct Block {
    std::vector<int> a, b;
    std::string bits;
};

struct Expect {
    int bit;
    bool last;
    size_t block;
    long index;
};

struct Gate {
    Vviterbi dut;
    long edge = 0, compared = 0, mismatches = 0;
    std::map<long, Expect> expect;
    unsigned lcg = 12345;

    unsigned rnd() {
        lcg = lcg * 1103515245u + 12345u;
        return (lcg >> 16) & 0x7fff;
    }

    void fail(const std::string& what) {
        if (mismatches < 16) std::fprintf(stderr, "edge %ld: %s\n", edge, what.c_str());
        ++mismatches;
    }

    // One clock edge, then the outputs it registered are checked against the contract.
    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++edge;
        if (edge > WATCHDOG) {
            std::fprintf(stderr, "watchdog\n");
            std::exit(1);
        }
        auto it = expect.find(edge);
        if (it == expect.end()) {
            if (dut.out_valid) fail("an output where none is due");
            return;
        }
        const Expect e = it->second;
        expect.erase(it);
        char msg[160];
        if (!dut.out_valid) {
            std::snprintf(msg, sizeof msg, "block %zu bit %ld: no output", e.block, e.index);
            fail(msg);
        } else if (dut.out_bit != e.bit || dut.out_last != e.last) {
            std::snprintf(msg, sizeof msg, "block %zu bit %ld: got %d%s, want %d%s", e.block, e.index,
                          dut.out_bit, dut.out_last ? " last" : "", e.bit, e.last ? " last" : "");
            fail(msg);
        }
        ++compared;
    }

    void idle(long n) {
        dut.in_valid = 0;
        dut.in_first = 0;
        dut.in_last = 0;
        // Idle inputs carry values that would decode differently.
        dut.in_a = static_cast<int8_t>(rnd());
        dut.in_b = static_cast<int8_t>(rnd());
        for (long k = 0; k < n; ++k) tick();
    }

    // Feeds a block's steps (all of them, or the first `upto`); returns the edge of its last
    // step. `gaps`: 0 none, 1 an idle cycle after every fifth step, 2 random idle cycles.
    long feed(const Block& blk, size_t index, int gaps, long upto = -1) {
        const long steps = static_cast<long>(blk.a.size());
        const long stop = upto < 0 ? steps : upto;
        const long n = steps < DEPTH ? steps : DEPTH;
        long last_edge = -1;
        for (long t = 0; t < stop; ++t) {
            dut.in_valid = 1;
            dut.in_first = t == 0;
            dut.in_last = t == steps - 1;
            dut.in_a = static_cast<int8_t>(blk.a[t]);
            dut.in_b = static_cast<int8_t>(blk.b[t]);
            const long taken = edge + 1;
            if (t >= DEPTH - 1 && t < steps - 1) {
                const long i = t - (DEPTH - 1);
                expect[taken + LAT] = {blk.bits[i] == '1', false, index, i};
            }
            if (t == steps - 1) {
                for (long k = 0; k < n; ++k) {
                    const long i = steps - n + k;
                    expect[taken + LAT + k] = {blk.bits[i] == '1', k == n - 1, index, i};
                }
                last_edge = taken;
            }
            tick();
            long pause = gaps == 1 ? (t % 5 == 4) : gaps == 2 ? (rnd() % 7 == 0) * (1 + rnd() % 3) : 0;
            if (t + 1 < stop && pause) idle(pause);
        }
        dut.in_valid = 0;
        dut.in_first = 0;
        dut.in_last = 0;
        return last_edge;
    }

    void reset() {
        // Whatever was due from the reset's first edge on must not come out.
        expect.erase(expect.upper_bound(edge), expect.end());
        dut.in_valid = 0;
        dut.rst_n = 0;
        tick();
        tick();
        dut.rst_n = 1;
    }

    // Waits until every due output has come out, and a little longer.
    void drain() {
        long guard = 0;
        while (!expect.empty() && guard++ < 4 * DEPTH) idle(1);
        idle(LAT + 2);
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_viterbi <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Block> blocks;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        std::string hex, bits;
        if (!(fields >> hex >> bits) || hex.size() != 4 * bits.size() || bits.empty()) {
            std::cerr << "malformed vector line\n";
            return 2;
        }
        Block b;
        for (size_t t = 0; t < bits.size(); ++t) {
            b.a.push_back(static_cast<int8_t>(std::stoi(hex.substr(4 * t, 2), nullptr, 16)));
            b.b.push_back(static_cast<int8_t>(std::stoi(hex.substr(4 * t + 2, 2), nullptr, 16)));
        }
        b.bits = bits;
        blocks.push_back(b);
    }

    Gate g;
    g.dut.rst_n = 0;
    g.idle(2);
    g.dut.rst_n = 1;
    g.idle(3);

    long tight = 0;
    for (size_t k = 0; k < blocks.size(); ++k) {
        const Block& blk = blocks[k];
        const long steps = static_cast<long>(blk.a.size());
        g.feed(blk, k, static_cast<int>(k % 3));
        const long n = steps < DEPTH ? steps : DEPTH;
        // Every other block is followed at once by the next, as early as the contract allows.
        if (k % 2 == 0) {
            g.idle(n + 1);
            ++tight;
        } else {
            g.idle(n + static_cast<long>(g.rnd() % 50));
        }
    }
    g.drain();

    // A reset while decisions are coming out, and one during a flush: nothing more of that
    // block, and the next block decodes.
    size_t longest = 0;
    for (size_t k = 0; k < blocks.size(); ++k)
        if (blocks[k].a.size() > blocks[longest].a.size()) longest = k;
    const Block& big = blocks[longest];
    g.feed(big, longest, 0, 3 * DEPTH);
    g.reset();
    g.idle(LAT + 3 * DEPTH);
    g.feed(blocks[0], 0, 0);
    g.drain();
    size_t mid = 0;
    for (size_t k = 0; k < blocks.size(); ++k)
        if (blocks[k].a.size() > static_cast<size_t>(DEPTH) && blocks[k].a.size() < 600) mid = k;
    g.feed(blocks[mid], mid, 0);
    g.idle(LAT + DEPTH / 2);
    g.reset();
    g.idle(LAT + 2 * DEPTH);
    g.feed(blocks[1], 1, 0);
    g.drain();

    // A reset whose first edge is 1 to 5 edges after the edge that took a last step (before,
    // while and after the flush is started): no flush may come out. A short block (all of it
    // flushed) and a long one (its last decisions are due before the reset and must come).
    size_t shortb = 0;
    for (size_t k = 0; k < blocks.size(); ++k)
        if (blocks[k].a.size() == 24) shortb = k;
    long resets = 0;
    for (size_t which : {shortb, mid}) {
        for (long o = 1; o <= 5; ++o) {
            g.feed(blocks[which], which, 0);
            g.idle(o - 1);
            g.reset();
            ++resets;
            g.idle(LAT + DEPTH + 2);
        }
    }
    g.feed(blocks[2], 2, 1);
    g.drain();

    // The case the contract's n + 2 is for: a block of one step exactly n + 2 edges after a
    // long block's last step, then one-step blocks each 3 edges after the one before.
    long chained = 0;
    g.feed(blocks[mid], mid, 0);
    g.idle(DEPTH + 1);
    for (int r = 0; r < 8; ++r) {
        g.feed(blocks[0], 0, 0);
        g.idle(2);
        ++chained;
    }
    g.drain();

    std::printf("viterbi G3: %zu blocks, %ld bits compared, %ld blocks followed at once, %ld resets after a "
                "last step, %ld one-step blocks at the contract's edge, %ld mismatches\n",
                blocks.size(), g.compared, tight, resets, chained, g.mismatches);
    if (!g.expect.empty()) {
        std::fprintf(stderr, "%zu expected outputs never came\n", g.expect.size());
        return 1;
    }
    if (g.compared == 0) {
        std::cerr << "no bits compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
