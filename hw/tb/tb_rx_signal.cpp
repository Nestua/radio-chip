// G3 gate for SIGNAL and the verdict (hw/rtl/rx_signal.sv), behind the Viterbi decoder
// (hw/rtl/viterbi.sv) as the top level will wire them (hw/tb/signal_chain.sv): replays
// hw/vectors/rx_signal.txt and requires every verdict — decoded or which check failed, the
// agreeing and disagreeing counts, rate, length, data symbols, patience, the data block's steps,
// code rate and modulation — to equal radio_phy::rx_ofdm::signal_checked,
// SignalVerdict::of and data_symbols, in its edge: 29 edges after the edge that took SIGNAL's
// last pair (the decoder's flush: its first bit registered 4 edges after, a bit an edge; the
// verdict 1 edge after the edge that takes the 24th). The decoder's 24 bits are checked too (they are decode's: viterbi.md's G3).
// Nothing else may raise sig_valid.
//
// The driving: SIGNAL's pairs back to back or with random idle cycles; after it, in most
// packets, a data block of random pairs (1 … 300 steps) from the earliest the decoder's
// contract allows (24 + 2 edges after SIGNAL's last step) or later, whose pairs and bits
// rx_signal must ignore; the drop that ends every packet at a random edge, also inside the data
// block's flush (its bits must not count for the next SIGNAL). Some packets are cut: a drop
// among SIGNAL's pairs (some with a pair in the drop's cycle), during the flush, or in the
// verdict's own edge; or a reset among the pairs or in the flush. A cut
// packet gives no verdict and the next one must decode. Registers start random
// (+verilator+rand+reset+2). Exit code 0 = gate passed.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Vsignal_chain.h"
#include "verilated.h"

namespace {

constexpr long WATCHDOG = 20'000'000;
constexpr long VIT_LAT = 4;      // viterbi.sv: a flushed bit's first edge after the last step
constexpr long VERDICT = 29;     // the header: 4 + 23, taken an edge later, + 1

struct Case {
    std::vector<int> soft;       // 48
    std::string bits;            // 24
    int why, rate, length, agree, disagree, symbols, patient, steps, code, mod;
};

std::vector<Case> load(const char* path) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", path);
        std::exit(2);
    }
    std::vector<Case> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string hex;
        Case c;
        ss >> hex >> c.bits >> c.why >> c.rate >> c.length >> c.agree >> c.disagree >> c.symbols >>
            c.patient >> c.steps >> c.code >> c.mod;
        for (size_t i = 0; i < hex.size(); i += 2) {
            const int v = std::stoi(hex.substr(i, 2), nullptr, 16);
            c.soft.push_back(v >= 128 ? v - 256 : v);
        }
        if (c.soft.size() != 48 || c.bits.size() != 24) {
            std::fprintf(stderr, "bad line: %s\n", line.c_str());
            std::exit(2);
        }
        out.push_back(c);
    }
    return out;
}

struct Gate {
    Vsignal_chain dut;
    long edge = 0, verdicts = 0, bits_checked = 0, mismatches = 0;
    long drop_with_pair = 0, reset_pairs = 0;
    long cut_pairs = 0, cut_flush = 0, cut_verdict = 0, resets = 0, data_blocks = 0, gaps = 0,
         late_drops = 0;
    unsigned lcg = 0x1234567u;
    std::map<long, size_t> verdict_at;            // edge → case
    std::map<long, std::pair<int, bool>> bit_at;  // edge → (bit, last)
    const std::vector<Case>* cases = nullptr;

    unsigned rnd() {
        lcg = lcg * 1103515245u + 12345u;
        return (lcg >> 16) & 0x7fff;
    }

    void fail(const std::string& what) {
        if (mismatches < 20) std::fprintf(stderr, "edge %ld: %s\n", edge, what.c_str());
        ++mismatches;
    }

    void quiet() {
        dut.drop = 0;
        dut.v_valid = 0;
        dut.v_first = rnd() & 1;
        dut.v_last = rnd() & 1;
        dut.v_a = rnd() & 0xff;
        dut.v_b = rnd() & 0xff;
    }

    void check_verdict(const Case& c, size_t k) {
        char msg[300];
        const bool ok = c.why == 0;
        bool bad = dut.sig_why != c.why || dut.sig_fail != !ok || dut.sig_agree != c.agree ||
                   dut.sig_disagree != c.disagree;
        if (ok) {
            bad = bad || dut.sig_rate != c.rate || dut.sig_length != c.length ||
                  dut.sig_symbols != c.symbols || dut.sig_patient != c.patient ||
                  dut.sig_steps != c.steps || dut.sig_code != c.code || dut.sig_mod != c.mod;
        } else {
            bad = bad || dut.sig_rate || dut.sig_length || dut.sig_symbols || dut.sig_patient ||
                  dut.sig_steps || dut.sig_code || dut.sig_mod;
        }
        if (bad) {
            std::snprintf(msg, sizeof msg,
                          "case %zu: got why %d fail %d agree %d/%d rate %d len %d sym %d pat %d "
                          "steps %d code %d mod %d; want why %d agree %d/%d rate %d len %d sym %d "
                          "pat %d steps %d code %d mod %d",
                          k, dut.sig_why, dut.sig_fail, dut.sig_agree, dut.sig_disagree,
                          dut.sig_rate, dut.sig_length, dut.sig_symbols, dut.sig_patient,
                          dut.sig_steps, dut.sig_code, dut.sig_mod, c.why, c.agree, c.disagree,
                          c.rate, c.length, c.symbols, c.patient, c.steps, c.code, c.mod);
            fail(msg);
        }
        ++verdicts;
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
        auto v = verdict_at.find(edge);
        if (v != verdict_at.end()) {
            if (!dut.sig_valid) {
                fail("case " + std::to_string(v->second) + ": no verdict");
            } else {
                check_verdict((*cases)[v->second], v->second);
            }
            verdict_at.erase(v);
        } else if (dut.sig_valid) {
            fail("a verdict where none is due");
        }
        auto b = bit_at.find(edge);
        if (b != bit_at.end()) {
            if (!dut.dec_valid || dut.dec_bit != b->second.first ||
                dut.dec_last != b->second.second)
                fail("a decoded bit of SIGNAL");
            ++bits_checked;
            bit_at.erase(b);
        }
    }

    void idle(long n) {
        for (long k = 0; k < n; ++k) {
            quiet();
            tick();
        }
    }

    // One step into the decoder (and rx_signal); returns the edge that took it.
    long step(int a, int b, bool first, bool last) {
        quiet();
        dut.v_valid = 1;
        dut.v_first = first;
        dut.v_last = last;
        dut.v_a = static_cast<uint8_t>(a);
        dut.v_b = static_cast<uint8_t>(b);
        tick();
        return edge;
    }

    void drop_now() {
        quiet();
        dut.drop = 1;
        // Nothing of rx_signal's due from this edge on.
        verdict_at.erase(verdict_at.lower_bound(edge + 1), verdict_at.end());
        tick();
    }

    void reset_now() {
        quiet();
        dut.rst_n = 0;
        verdict_at.erase(verdict_at.lower_bound(edge + 1), verdict_at.end());
        bit_at.erase(bit_at.lower_bound(edge + 1), bit_at.end());
        const long n = 1 + rnd() % 3;
        for (long k = 0; k < n; ++k) tick();
        dut.rst_n = 1;
        ++resets;
    }

    void run() {
        const std::vector<Case>& cs = *cases;
        long earliest = edge + 1;   // the decoder's contract for the next block's first step
        for (size_t k = 0; k < cs.size(); ++k) {
            const Case& c = cs[k];
            while (edge + 1 < earliest) idle(1);
            if (rnd() % 4 == 0) idle(rnd() % 40);
            const int cut = rnd() % 16;   // 0 among the pairs, 1 in the flush, 2 at the verdict,
                                          // 3 a reset in the flush
            const bool with_gaps = rnd() % 3 == 0;
            long last = 0;
            const int cut_at = static_cast<int>(rnd() % 24);
            bool cut_done = false;
            for (int t = 0; t < 24; ++t) {
                if (cut == 0 && t == cut_at && t > 0) {
                    const int how = rnd() % 3;
                    if (how == 0) {
                        drop_now();
                    } else if (how == 1) {
                        // The drop with a pair in its cycle: the decoder takes the pair,
                        // rx_signal does not.
                        quiet();
                        dut.drop = 1;
                        dut.v_valid = 1;
                        dut.v_first = 0;
                        dut.v_last = 0;
                        dut.v_a = static_cast<uint8_t>(c.soft[2 * t]);
                        dut.v_b = static_cast<uint8_t>(c.soft[2 * t + 1]);
                        verdict_at.erase(verdict_at.lower_bound(edge + 1), verdict_at.end());
                        tick();
                        ++drop_with_pair;
                    } else {
                        reset_now();
                        ++reset_pairs;
                    }
                    if (how != 2) ++cut_pairs;
                    cut_done = true;
                    break;
                }
                if (with_gaps && rnd() % 5 == 0) {
                    idle(1 + rnd() % 3);
                    ++gaps;
                }
                last = step(c.soft[2 * t], c.soft[2 * t + 1], t == 0, t == 23);
            }
            if (cut_done) {
                // The decoder holds a block that never ended; the next SIGNAL starts afresh.
                earliest = edge + 1;
                continue;
            }
            for (int i = 0; i < 24; ++i)
                bit_at[last + VIT_LAT + i] = {c.bits[i] == '1', i == 23};
            verdict_at[last + VERDICT] = k;
            long flush_end = last + VIT_LAT + 23;
            if (cut == 1 || cut == 2 || cut == 3) {
                const long at = cut == 2 ? last + VERDICT : last + 1 + rnd() % (VERDICT - 1);
                while (edge + 1 < at) idle(1);
                if (cut == 3) {
                    reset_now();
                } else {
                    drop_now();
                    (cut == 2 ? cut_verdict : cut_flush) += 1;
                }
                // The flush goes on (not after a reset): the next SIGNAL waits for it.
                earliest = cut == 3 ? edge + 1 : last + 26;
                continue;
            }
            // A data block, from the earliest the contract allows or later.
            long data_last = -1, data_n = 0;
            if (rnd() % 4 != 0) {
                long start = last + 26 + (rnd() % 3 == 0 ? 0 : rnd() % 60);
                while (edge + 1 < start) idle(1);
                data_n = 1 + rnd() % 300;
                for (long t = 0; t < data_n; ++t) {
                    if (with_gaps && rnd() % 7 == 0) idle(1);
                    data_last = step(static_cast<int8_t>(rnd()), static_cast<int8_t>(rnd()),
                                     t == 0, t == data_n - 1);
                }
                ++data_blocks;
            }
            // The packet's end: a drop after the verdict, sometimes inside the data's flush.
            long end_at = last + VERDICT + 1 + rnd() % 20;
            const long nd = data_n < 96 ? data_n : 96;
            if (data_last >= 0) {
                const long flush = data_last + VIT_LAT + nd;
                if (rnd() % 2) {
                    end_at = std::max(end_at, data_last + 1 + static_cast<long>(rnd() % (nd + 4)));
                    ++late_drops;
                } else {
                    end_at = std::max(end_at, flush + 1);
                }
            }
            while (edge + 1 < end_at) idle(1);
            drop_now();
            earliest = data_last >= 0 ? data_last + nd + 2 : flush_end + 1;
            earliest = std::max(earliest, edge + 1);
        }
        idle(400);
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s vectors/rx_signal.txt\n", argv[0]);
        return 2;
    }
    const std::vector<Case> cs = load(argv[1]);
    Gate g;
    g.cases = &cs;
    g.dut.rst_n = 0;
    g.quiet();
    for (int k = 0; k < 3; ++k) g.tick();
    g.dut.rst_n = 1;
    g.run();
    long ok = 0;
    for (const Case& c : cs) ok += c.why == 0;
    std::printf("rx_signal: %zu cases (%ld decoded), %ld verdicts, %ld SIGNAL bits checked, %ld "
                "mismatches; cut among the pairs %ld, in the flush %ld, at the verdict %ld, "
                "resets %ld (%ld among the pairs); %ld drops with a pair; %ld data blocks, %ld drops "
                "inside their flush, %ld gaps\n",
                cs.size(), ok, g.verdicts, g.bits_checked, g.mismatches, g.cut_pairs, g.cut_flush,
                g.cut_verdict, g.resets, g.reset_pairs, g.drop_with_pair, g.data_blocks,
                g.late_drops, g.gaps);
    const long cut = g.cut_pairs + g.cut_flush + g.cut_verdict + g.resets;
    if (g.mismatches || !g.verdict_at.empty() || g.verdicts + cut != static_cast<long>(cs.size())) {
        std::printf("FAIL (%zu verdicts still due)\n", g.verdict_at.size());
        return 1;
    }
    std::printf("PASS\n");
    return 0;
}
