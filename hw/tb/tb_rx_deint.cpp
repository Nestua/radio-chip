// G3 gate for the deinterleaver and depuncturer (hw/rtl/rx_deint.sv): replays
// hw/vectors/rx_deint.txt and requires every step handed to the Viterbi decoder — (A, B), first,
// last — to equal radio_phy::rx_ofdm_fixed::deinterleave_soft and viterbi::depuncture's, in the
// edge the module's header gives it; and slot_ok in every cycle as the header says.
//
// The edges, from the header: a symbol's first step 2 edges after the latest of the edge that
// took its soft_last, the edge that took its block (the block's first symbol), and the edge
// before the previous step's; then a step an edge to the symbol's (or the block's) end. A
// symbol's slot is free from the edge before its last step's; slot_ok is high while fewer than
// two symbols are written and not freed. Nothing else comes out.
//
// The driving: symbols written as soon as slot_ok allows or after random waits, a subcarrier a
// clock or with random gaps; the unused lanes carry random values; a block announced before its
// first symbol is written, while it is, or after (the next block only once the previous one's
// last step is out, as the contract says). The packets run back to back, so the next packet's
// SIGNAL is written while the previous data block is read. soft_last is random while soft_valid
// is low (rx_symbol holds it after its 48th beat), soft_mod random but at soft_last. Drops and
// resets fall in a sixth of the packets (always the longest) at a random edge from the
// packet's last write on, some with a write beat or a block in their cycle; the packet is then
// replayed (the longest always) or skipped, and nothing due from the drop's edge on may come
// out. Registers start random (+verilator+rand+reset+2). Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Vrx_deint.h"
#include "verilated.h"

namespace {

constexpr long WATCHDOG = 20'000'000;
constexpr int DATA = 48;
constexpr int NB[4] = {1, 2, 4, 6};
// N_DBPS of (code rate, modulation): 48·N_BPSC·R.
int n_dbps(int rate, int mod) {
    const int n = DATA * NB[mod];
    return rate == 0 ? n / 2 : rate == 1 ? n * 2 / 3 : n * 3 / 4;
}

struct Block {
    int rate = 0, mod = 0;
    long steps = 0;
    std::vector<std::vector<int>> sym;   // on-air values, 48·N_BPSC a symbol
    std::vector<int> a, b;               // the pairs
};

int hex2(const std::string& s, size_t at) {
    const int v = std::stoi(s.substr(at, 2), nullptr, 16);
    return v >= 128 ? v - 256 : v;
}

std::vector<Block> load(const char* path) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", path);
        std::exit(2);
    }
    std::vector<Block> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        char tag;
        ss >> tag;
        if (tag == 'B') {
            Block b;
            long n;
            ss >> b.rate >> b.mod >> b.steps >> n;
            out.push_back(b);
        } else if (tag == 'S') {
            std::string h;
            ss >> h;
            std::vector<int> v;
            for (size_t i = 0; i < h.size(); i += 2) v.push_back(hex2(h, i));
            out.back().sym.push_back(v);
        } else if (tag == 'E') {
            std::string h;
            ss >> h;
            for (size_t i = 0; i < h.size(); i += 4) {
                out.back().a.push_back(hex2(h, i));
                out.back().b.push_back(hex2(h, i + 2));
            }
        }
    }
    return out;
}

struct Expect {
    int a, b;
    bool first, last;
    size_t block;
    long step;
};

// A symbol of the stream: its block, its steps.
struct Sym {
    size_t block, index;
    long step0, steps;
};

struct Gate {
    Vrx_deint dut;
    std::vector<Block> blocks;
    std::vector<Sym> syms;              // every symbol of every block, in order
    std::vector<size_t> first_sym;      // a block's first symbol in `syms`

    long edge = 0, compared = 0, mismatches = 0, slot_checks = 0;
    long skipped = 0, busy_drops = 0;
    long drops = 0, resets = 0, replays = 0, gaps = 0, waits = 0, late_blocks = 0;
    unsigned lcg = 0x2545f491;
    std::map<long, Expect> expect;

    // The model since the last drop.
    std::map<size_t, long> sl_edge;     // symbol → the edge that took its soft_last
    std::map<size_t, long> blk_edge;    // block → the edge that took blk_valid
    std::map<long, int> frees;          // edge → symbols freed at it
    size_t planned = 0;                 // next symbol to plan
    long last_v = -1000;                // the last planned step's edge
    std::map<size_t, long> blk_last_v;  // block → its last step's edge
    int held = 0;                       // written and not freed

    unsigned rnd() {
        lcg = lcg * 1103515245u + 12345u;
        return (lcg >> 16) & 0x7fff;
    }

    void fail(const std::string& what) {
        if (mismatches < 20) std::fprintf(stderr, "edge %ld: %s\n", edge, what.c_str());
        ++mismatches;
    }

    void reset_model(size_t from_sym) {
        sl_edge.clear();
        blk_edge.clear();
        frees.clear();
        blk_last_v.clear();
        planned = from_sym;
        last_v = -1000;
        held = 0;
    }

    // Plans every symbol whose soft_last and block are both in, in order.
    void plan() {
        while (planned < syms.size()) {
            const Sym& s = syms[planned];
            auto sl = sl_edge.find(planned);
            auto bl = blk_edge.find(s.block);
            if (sl == sl_edge.end() || bl == blk_edge.end()) return;
            long first = last_v + 1;
            if (sl->second + 2 > first) first = sl->second + 2;
            if (s.index == 0 && bl->second + 2 > first) first = bl->second + 2;
            const Block& B = blocks[s.block];
            for (long t = 0; t < s.steps; ++t) {
                const long k = s.step0 + t;
                expect[first + t] = {B.a[k], B.b[k], k == 0, k == B.steps - 1, s.block, k};
            }
            last_v = first + s.steps - 1;
            frees[last_v - 1] += 1;
            if (s.step0 + s.steps == B.steps) blk_last_v[s.block] = last_v;
            ++planned;
        }
    }

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
            if (dut.v_valid) fail("a step where none is due");
        } else {
            const Expect e = it->second;
            expect.erase(it);
            char msg[200];
            const int a = static_cast<int8_t>(dut.v_a), b = static_cast<int8_t>(dut.v_b);
            if (!dut.v_valid) {
                std::snprintf(msg, sizeof msg, "block %zu step %ld: no step", e.block, e.step);
                fail(msg);
            } else if (a != e.a || b != e.b || dut.v_first != e.first || dut.v_last != e.last) {
                std::snprintf(msg, sizeof msg,
                              "block %zu step %ld: got (%d, %d)%s%s, want (%d, %d)%s%s", e.block,
                              e.step, a, b, dut.v_first ? " first" : "", dut.v_last ? " last" : "",
                              e.a, e.b, e.first ? " first" : "", e.last ? " last" : "");
                fail(msg);
            }
            ++compared;
        }
    }

    // After an edge: the model's frees, then slot_ok against it.
    void check_slot() {
        auto f = frees.find(edge);
        if (f != frees.end()) {
            held -= f->second;
            frees.erase(f);
        }
        const bool want = held < 2;
        if (static_cast<bool>(dut.slot_ok) != want) {
            char msg[120];
            std::snprintf(msg, sizeof msg, "slot_ok %d, want %d (%d held)", dut.slot_ok, want, held);
            fail(msg);
        }
        ++slot_checks;
    }

    void quiet() {
        dut.drop = 0;
        dut.blk_valid = 0;
        dut.blk_rate = rnd() % 3;
        dut.blk_steps = rnd();
        dut.soft_valid = 0;
        dut.soft_last = rnd() % 2;   // counts only with soft_valid
        dut.soft_j = rnd() % 64;
        dut.soft_mod = rnd() % 4;
        for (int i = 0; i < 6; ++i) dut.soft_v[i] = rnd() & 0xff;
    }

    // Runs packets [p0, p1) (two blocks each), drops and resets in some of them.
    void run() {
        const size_t packets = blocks.size() / 2;
        size_t p = 0;
        // The stream's position.
        size_t ws = 0;                  // next symbol to write
        int beat = -1;                  // beat of the symbol being written; -1 none
        long wait = 0;
        size_t bs = 0;                  // next block to announce
        long blk_wait = 0;
        long drop_at = -1;              // the edge count at which to drop / reset
        bool drop_is_reset = false;
        size_t drop_packet = 0;
        size_t epoch_block = 0;          // the first block since the last drop

        auto pick_drop = [&](size_t pk) {
            drop_at = -1;
            const long len = blocks[2 * pk + 1].steps;
            if (rnd() % 6 == 0 || len > 30000) {
                drop_at = edge + 1 + rnd() % (400 + 2 * len);
                drop_is_reset = rnd() % 3 == 0;
                drop_packet = pk;
            }
        };
        reset_model(0);
        pick_drop(0);

        while (p < packets || !expect.empty()) {
            quiet();
            // A drop or a reset now?
            if (drop_at >= 0 && edge + 1 == drop_at) {
                if (rnd() % 2) {
                    dut.soft_valid = 1;
                    ++busy_drops;
                }
                if (rnd() % 3 == 0) {
                    dut.blk_valid = 1;
                    ++busy_drops;
                }
                if (drop_is_reset) {
                    dut.rst_n = 0;
                    ++resets;
                } else {
                    dut.drop = 1;
                    ++drops;
                }
                // Nothing due from the drop's edge on comes out.
                for (auto it = expect.lower_bound(edge + 1); it != expect.end();)
                    it = expect.erase(it);
                tick();
                if (drop_is_reset) {
                    const long n = 1 + rnd() % 3;
                    for (long k = 0; k < n; ++k) {
                        quiet();
                        dut.rst_n = 0;
                        tick();
                    }
                    dut.rst_n = 1;
                }
                // Replay the packet or skip it.
                size_t next = drop_packet;
                if (rnd() % 2 || blocks[2 * drop_packet + 1].steps > 30000) {
                    ++replays;
                } else {
                    next = drop_packet + 1;
                    skipped += blocks[2 * drop_packet].steps + blocks[2 * drop_packet + 1].steps;
                }
                p = next;
                bs = 2 * p;
                epoch_block = bs;
                ws = p < packets ? first_sym[bs] : syms.size();
                beat = -1;
                wait = rnd() % 4;
                blk_wait = rnd() % 3 == 0 ? 0 : rnd() % 200;
                reset_model(ws);
                drop_at = -1;
                if (p < packets && next == drop_packet + 1) pick_drop(p);
                check_slot();
                continue;
            }
            if (p >= packets && ws >= syms.size()) {
                tick();
                check_slot();
                continue;
            }
            // A block: the first since a drop any time; the next once the previous block's last
            // step is out (taken no earlier than that step's edge), after a random wait.
            bool announce = false;
            if (bs < 2 * packets) {
                bool allowed = bs == epoch_block;
                if (!allowed) {
                    auto prev = blk_last_v.find(bs - 1);
                    allowed = prev != blk_last_v.end() && edge + 1 >= prev->second;
                }
                if (allowed && blk_wait > 0) {
                    --blk_wait;
                    allowed = false;
                }
                announce = allowed;
            }
            if (announce) {
                dut.blk_valid = 1;
                dut.blk_rate = blocks[bs].rate;
                dut.blk_steps = blocks[bs].steps;
            }
            // The soft pass.
            bool wrote_last = false;
            size_t wsym = ws;
            if (beat < 0 && ws < syms.size() && dut.slot_ok) {
                if (wait > 0) {
                    --wait;
                } else {
                    beat = 0;
                }
            }
            if (beat >= 0) {
                if (rnd() % 23 == 0 && beat > 0) {
                    ++gaps;   // an idle beat
                } else {
                    const Sym& s = syms[ws];
                    const Block& B = blocks[s.block];
                    const int nb = NB[B.mod];
                    dut.soft_valid = 1;
                    dut.soft_j = beat;
                    // The modulation counts at soft_last only.
                    dut.soft_mod = beat == DATA - 1 ? B.mod : rnd() % 4;
                    dut.soft_last = beat == DATA - 1;
                    for (int i = 0; i < nb; ++i) dut.soft_v[i] = B.sym[s.index][beat * nb + i] & 0xff;
                    wrote_last = beat == DATA - 1;
                    ++beat;
                }
            }
            tick();
            if (announce) {
                blk_edge[bs] = edge;
                ++bs;
                // The next block (a data block) waits a random while after it is allowed.
                blk_wait = rnd() % 4 == 0 ? 0 : rnd() % 300;
                if (blk_wait > 150) ++late_blocks;
            }
            if (wrote_last) {
                sl_edge[wsym] = edge;
                ++held;
                beat = -1;
                ++ws;
                wait = rnd() % 3 == 0 ? 0 : rnd() % 120;
                if (wait > 0) ++waits;
                // A packet is done being driven once its last symbol is written.
                if (ws == syms.size() || syms[ws].block >= 2 * (p + 1)) {
                    ++p;
                    if (p < packets && drop_at < 0) pick_drop(p);
                }
            }
            plan();
            check_slot();
        }
        for (int k = 0; k < 50; ++k) {
            quiet();
            tick();
            check_slot();
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s vectors/rx_deint.txt\n", argv[0]);
        return 2;
    }
    Gate g;
    g.blocks = load(argv[1]);
    if (g.blocks.size() % 2 != 0 || g.blocks.empty()) {
        std::fprintf(stderr, "a packet is two blocks\n");
        return 2;
    }
    for (size_t b = 0; b < g.blocks.size(); ++b) {
        const Block& B = g.blocks[b];
        const long per = B.rate == 0 && B.mod == 0 && B.steps == 24 && b % 2 == 0
                             ? 24
                             : n_dbps(B.rate, B.mod);
        g.first_sym.push_back(g.syms.size());
        long done = 0;
        for (size_t i = 0; i < B.sym.size(); ++i) {
            const long n = B.steps - done < per ? B.steps - done : per;
            g.syms.push_back({b, i, done, n});
            done += n;
        }
        if (done != B.steps || static_cast<long>(B.a.size()) != B.steps) {
            std::fprintf(stderr, "block %zu: %ld steps in symbols, %zu pairs, %ld declared\n", b,
                         done, B.a.size(), B.steps);
            return 2;
        }
    }
    g.dut.rst_n = 0;
    g.quiet();
    for (int k = 0; k < 3; ++k) g.tick();
    g.dut.rst_n = 1;
    g.run();
    std::printf("rx_deint: %zu blocks, %ld steps compared, %ld slot_ok checks, %ld mismatches; "
                "%ld drops, %ld resets (%ld with a beat or a block), %ld replays, %ld write gaps, "
                "%ld waits, %ld late blocks\n",
                g.blocks.size(), g.compared, g.slot_checks, g.mismatches, g.drops, g.resets, g.busy_drops,
                g.replays, g.gaps, g.waits, g.late_blocks);
    long total = 0;
    for (auto& B : g.blocks) total += B.steps;
    if (g.mismatches || !g.expect.empty() || g.compared < total - g.skipped) {
        std::printf("FAIL (%zu steps still due; %ld of %ld steps)\n", g.expect.size(), g.compared,
                    total);
        return 1;
    }
    std::printf("PASS\n");
    return 0;
}
