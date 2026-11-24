// G3 gate for the symbol block (hw/rtl/rx_symbol.sv with hw/rtl/cordic_iter.sv, wired by
// hw/tb/symbol_chain.sv): replays hw/vectors/rx_symbol.txt and requires every pilot, every
// common phase and every soft value to match radio_phy::rx_ofdm_fixed::{symbol_pilots,
// symbol_from_bins} bit for bit, each in the edge the block's header gives it.
//
// The testbench is the block's surroundings: the channel block's stores (a word a clock after
// the read, garbage on a cycle without one), the tracker (θ for a symbol after its pilots, in
// order; SIGNAL's at any time), the quotients and the data's modulation at random times from
// chan_valid on, the FFT (a symbol's 64 bins in a random order, at a pace per packet: a bin a
// clock, every 8 clocks, random gaps; the next symbol's first bin only while bins_ok). Packets
// are dropped at their end; one in five is aborted at a random edge (one abort in four by
// reset) and nothing may come out after it; half of those are replayed at once.
//
// Latencies, in edges after the trigger's edge (the later of the edges listed):
//   pil_valid   kPil after: the 52nd used bin, chan_valid, the previous symbol's last product
//               written (its pil_valid + 48);
//   cr_valid    kCr after: this symbol's last product written, θ's edge − 1 (θ is taken in the
//               cycle it comes), the previous soft pass's last read (its first soft + 41);
//   phi_valid   kPhi after the CORDIC took the request (cr_valid && !busy);
//   soft_valid  kSoft after: phi_valid, the scale's edge, the modulation's edge (data);
//               subcarrier j a clock after j − 1.
// Exit code 0 = gate passed.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vsymbol_chain.h"
#include "verilated.h"

namespace {

constexpr long kPil = 9;
constexpr long kCr = 10;
constexpr long kPhi = 18;
constexpr long kSoft = 8;
constexpr int kNbpsc[4] = {1, 2, 4, 6};

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}
uint32_t mask(long long v, int bits) { return static_cast<uint32_t>(v) & ((1u << bits) - 1); }

struct Symbol {
    long long theta;
    long long z[64][2];
    long long pil[4][2];
    long long phi;
    std::vector<int> soft;
};

struct Packet {
    int shift, n, dmod, nsym;
    long long q[4];
    long long hs[64][2];
    long long g[64];
    std::vector<Symbol> sym;
};

bool used(int b) { return (b >= 1 && b <= 26) || (b >= 38 && b <= 63); }

struct Rng {
    uint64_t s = 0x9e3779b97f4a7c15ull;
    uint32_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<uint32_t>(s >> 11);
    }
    int below(int n) { return static_cast<int>(next() % static_cast<uint32_t>(n)); }
};

struct Gate {
    Vsymbol_chain dut;
    Rng rng;
    long edge = 0;
    long mismatches = 0;
    // counters
    long pilots = 0, phis = 0, softs = 0, symbols = 0, packets = 0, aborts = 0, resets = 0,
         replays = 0, held = 0, late_scale = 0, theta_early = 0, cordic_held = 0,
         aborts_waiting = 0;

    void error(const char* what, long a = 0, long b = 0) {
        if (mismatches < 20) std::fprintf(stderr, "edge %ld: %s (%ld, %ld)\n", edge, what, a, b);
        ++mismatches;
    }

    // Channel stores, as rx_channel answers them.
    const Packet* store = nullptr;
    bool chan_seen = false;

    void tick() {
        dut.clk = 0;
        dut.eval();
        const bool hre = dut.hs_re, gre = dut.g_re;
        const int ha = dut.hs_raddr, ga = dut.g_raddr;
        if ((hre || gre) && (!chan_seen || store == nullptr)) error("store read before the channel");
        if (hre && !used(ha)) error("Hs read off the used bins", ha);
        if (gre && !used(ga)) error("g read off the used bins", ga);
        dut.clk = 1;
        dut.eval();
        ++edge;
        if (hre && store) {
            dut.hs_rdata_re = mask(store->hs[ha][0], 18);
            dut.hs_rdata_im = mask(store->hs[ha][1], 18);
        } else {
            dut.hs_rdata_re = rng.next() & 0x3ffff;
            dut.hs_rdata_im = rng.next() & 0x3ffff;
        }
        dut.g_rdata = (gre && store) ? static_cast<uint32_t>(store->g[ga]) : (rng.next() & 0x7fff);
        dut.eval();
    }

    void idle_inputs() {
        dut.bin_valid = 0;
        dut.chan_valid = 0;
        dut.scale_valid = 0;
        dut.mod_valid = 0;
        dut.theta_valid = 0;
        dut.drop = 0;
        dut.rst_n = 1;
        dut.cordic_hold = 0;
    }

    void quiet(int n, const char* why) {
        for (int k = 0; k < n; ++k) {
            tick();
            if (dut.pil_valid || dut.phi_valid || dut.soft_valid || dut.cr_valid)
                error(why);
        }
    }

    // Runs one packet; abort_at >= 0: abort at that edge from the packet's start (by reset if
    // by_reset); abort_at == -2: abort in the third cycle a CORDIC request waits. Returns true
    // when the packet completed.
    bool run(const Packet& p, long abort_at, bool by_reset) {
        store = &p;
        chan_seen = false;
        const long t0 = edge;
        const int pace = rng.below(3);             // 0: a bin a clock, 1: every 8, 2: random
        const long chan_at = t0 + rng.below(300);
        // the scales: BPSK's soon after the channel, the data's later, the others sometimes
        long scale_at[4];
        for (int m = 0; m < 4; ++m) scale_at[m] = -1;
        scale_at[0] = chan_at + rng.below(40);
        if (p.dmod != 0) scale_at[p.dmod] = chan_at + rng.below(600);
        for (int m = 1; m < 4; ++m)
            if (m != p.dmod && rng.below(3) == 0) scale_at[m] = chan_at + rng.below(600);
        const long mod_at = chan_at + rng.below(600);
        long scale_edge[4] = {-1, -1, -1, -1};
        long mod_edge = -1, chan_edge = -1;

        // bins
        int bsym = 0, bpos = 0;
        long bnext = t0 + rng.below(50);
        std::vector<int> order(64);
        long last_bin_edge[1024];
        // θ
        int tsym = 0;
        long tnext = t0 + rng.below(500);
        long theta_edge[1024];
        // expectations
        int pil_sym = 0, phi_sym = 0, soft_sym = 0, soft_j = 0;
        long pil_edge[1024], cr_edge[1024], take_edge[1024], phi_edge[1024], first_soft[1024];
        bool cr_seen = false;
        int hold_left = 0;       // cycles the arbiter still keeps the CORDIC from the request
        int waited = 0;          // cycles the current request has waited
        bool pending = false;    // a request not yet taken
        const int ns = p.nsym;

        for (long guard = 0; guard < 200000; ++guard) {
            if ((abort_at >= 0 && edge - t0 == abort_at) || (abort_at == -2 && waited == 3)) {
                if (abort_at == -2) ++aborts_waiting;
                idle_inputs();
                if (by_reset) dut.rst_n = 0; else dut.drop = 1;
                tick();
                idle_inputs();
                ++aborts;
                if (by_reset) ++resets;
                quiet(40, "output after an abort");
                return false;
            }
            idle_inputs();
            const long now = edge;   // inputs set now are taken by edge now + 1
            if (now >= chan_at && chan_edge < 0) {
                dut.chan_valid = 1;
                dut.chan_shift = p.shift;
                dut.chan_n = p.n;
            }
            // one scale a cycle
            for (int m = 0; m < 4; ++m) {
                if (scale_at[m] >= 0 && scale_edge[m] < 0 && now >= scale_at[m] &&
                    (chan_edge >= 0 || dut.chan_valid)) {
                    dut.scale_valid = 1;
                    dut.scale_mod = m;
                    dut.scale_q = mask(p.q[m], 20);
                    scale_edge[m] = now + 1;
                    break;
                }
            }
            if (mod_edge < 0 && now >= mod_at && (chan_edge >= 0 || dut.chan_valid)) {
                dut.mod_valid = 1;
                dut.data_mod = p.dmod;
                mod_edge = now + 1;
            }
            // θ, in order: symbol s after its pilots (SIGNAL at any time)
            if (tsym < ns && now >= tnext && (tsym == 0 || pil_sym > tsym)) {
                if (tsym == 0 && pil_sym == 0) ++theta_early;
                dut.theta_valid = 1;
                dut.theta = mask(p.sym[tsym].theta, 23);
                theta_edge[tsym] = now + 1;
                ++tsym;
                tnext = now + 1 + rng.below(60);
            }
            // bins
            if (bsym < ns && now >= bnext) {
                if (bpos == 0 && !dut.bins_ok) {
                    ++held;
                } else {
                    if (bpos == 0) {
                        for (int k = 0; k < 64; ++k) order[k] = k;
                        for (int k = 63; k > 0; --k) std::swap(order[k], order[rng.below(k + 1)]);
                    }
                    const int b = order[bpos];
                    dut.bin_valid = 1;
                    dut.bin_idx = b;
                    dut.bin_re = mask(p.sym[bsym].z[b][0], 16);
                    dut.bin_im = mask(p.sym[bsym].z[b][1], 16);
                    if (used(b)) last_bin_edge[bsym] = now + 1;
                    ++bpos;
                    bnext = now + 1 + (pace == 0 ? 0 : pace == 1 ? 7 : rng.below(4));
                    if (bpos == 64) {
                        bpos = 0;
                        ++bsym;
                        bnext += rng.below(4) == 0 ? rng.below(120) : 0;
                    }
                }
            }
            // the arbiter: half the requests wait 1 to 30 cycles (the tracker's angles)
            if (dut.cr_valid && hold_left > 0) {
                dut.cordic_hold = 1;
                --hold_left;
            }
            if (dut.cr_valid) ++waited;
            const bool chan_now = dut.chan_valid;
            dut.eval();
            const bool take_now = dut.cr_take;
            tick();
            if (chan_now) {
                chan_edge = edge;
                chan_seen = true;
            }
            if (take_now) {
                pending = false;
                waited = 0;
                if (phi_sym >= ns) error("a CORDIC request too many");
                else take_edge[phi_sym] = edge;
            }
            // outputs
            if (pending && !dut.cr_valid) error("the request fell before it was taken");
            if (dut.cr_valid && !cr_seen) {
                cr_seen = true;
                pending = true;
                waited = 0;
                hold_left = rng.below(2) == 0 ? 1 + rng.below(30) : 0;
                if (hold_left > 0) ++cordic_held;
                const int s = phi_sym;
                if (s < ns) {
                    cr_edge[s] = edge;
                    long trig = std::max(pil_edge[s] + 48, theta_edge[s] - 1);
                    if (s > 0) trig = std::max(trig, first_soft[s - 1] + 41);
                    if (edge != trig + kCr) error("cr_valid's edge", edge, trig + kCr);
                }
            }
            if (dut.pil_valid) {
                const int s = pil_sym;
                if (s >= ns) {
                    error("pilots too many");
                } else {
                    for (int k = 0; k < 4; ++k) {
                        if (sext(dut.pil_re[k], 17) != p.sym[s].pil[k][0] ||
                            sext(dut.pil_im[k], 17) != p.sym[s].pil[k][1])
                            error("pilot", s, k);
                    }
                    long trig = std::max(last_bin_edge[s], chan_edge);
                    if (s > 0) trig = std::max(trig, pil_edge[s - 1] + 48);
                    if (edge != trig + kPil) error("pil_valid's edge", edge, trig + kPil);
                    pil_edge[s] = edge;
                    ++pil_sym;
                    ++pilots;
                }
            }
            if (dut.phi_valid) {
                const int s = phi_sym;
                if (s >= ns) {
                    error("phi too many");
                } else {
                    if (sext(dut.phi, 22) != p.sym[s].phi) error("phi", s);
                    if (edge != take_edge[s] + kPhi) error("phi's edge", edge, take_edge[s] + kPhi);
                    phi_edge[s] = edge;
                    ++phi_sym;
                    ++phis;
                    cr_seen = false;
                }
            }
            if (dut.soft_valid) {
                const int s = soft_sym;
                if (s >= ns || s >= phi_sym) {
                    error("soft values too many");
                } else {
                    const int m = s == 0 ? 0 : p.dmod;
                    const int nb = kNbpsc[m];
                    if (dut.soft_j != soft_j) error("soft_j", dut.soft_j, soft_j);
                    if (dut.soft_mod != m) error("soft_mod", dut.soft_mod, m);
                    if (dut.soft_last != (soft_j == 47)) error("soft_last", soft_j);
                    for (int l = 0; l < 6; ++l) {
                        const long long got = sext(dut.soft_v[l], 8);
                        const long long want = l < nb ? p.sym[s].soft[soft_j * nb + l] : 0;
                        if (got != want) {
                            if (mismatches < 8)
                                std::fprintf(stderr, "  mod %d j %d lane %d got %lld want %lld\n",
                                             m, soft_j, l, got, want);
                            error("soft value", s, soft_j * 8 + l);
                        }
                    }
                    if (soft_j == 0) {
                        long trig = std::max(phi_edge[s], scale_edge[m]);
                        if (s > 0) trig = std::max(trig, mod_edge);
                        if (trig > phi_edge[s]) ++late_scale;
                        if (edge != trig + kSoft) error("first soft's edge", edge, trig + kSoft);
                        first_soft[s] = edge;
                    } else if (edge != first_soft[s] + soft_j) {
                        error("soft's edge", edge, first_soft[s] + soft_j);
                    }
                    ++softs;
                    if (++soft_j == 48) {
                        soft_j = 0;
                        ++soft_sym;
                    }
                }
            }
            if (soft_sym == ns) {
                // the packet's end: a few idle clocks, then the top level's drop
                idle_inputs();
                quiet(rng.below(5), "output after the packet's last");
                dut.drop = 1;
                tick();
                idle_inputs();
                ++packets;
                symbols += ns;
                return true;
            }
        }
        // A block that hangs would make every later packet wait as long: stop here.
        error("the packet never ended", soft_sym, ns);
        std::fprintf(stderr, "rx_symbol G3: stopped, %ld mismatches\n", mismatches);
        std::exit(1);
    }
};

bool parse(const char* path, std::vector<Packet>& out) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        for (char& c : line)
            if (c == ',') c = ' ';
        std::istringstream f(line);
        std::string tag;
        f >> tag;
        if (tag == "P") {
            Packet p{};
            f >> p.shift >> p.n >> p.q[0] >> p.q[1] >> p.q[2] >> p.q[3] >> p.dmod >> p.nsym;
            for (auto& h : p.hs) f >> h[0] >> h[1];
            for (auto& g : p.g) f >> g;
            if (!f) return false;
            out.push_back(p);
        } else if (tag == "S") {
            if (out.empty()) return false;
            Packet& p = out.back();
            Symbol s{};
            f >> s.theta;
            for (auto& z : s.z) f >> z[0] >> z[1];
            for (auto& v : s.pil) f >> v[0] >> v[1];
            f >> s.phi;
            const int m = p.sym.empty() ? 0 : p.dmod;
            s.soft.resize(48 * kNbpsc[m]);
            for (auto& v : s.soft) f >> v;
            if (!f) return false;
            p.sym.push_back(s);
        } else {
            return false;
        }
    }
    for (const auto& p : out)
        if (static_cast<int>(p.sym.size()) != p.nsym) return false;
    return !out.empty();
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_rx_symbol <vectors.txt>\n";
        return 2;
    }
    std::vector<Packet> packets;
    if (!parse(argv[1], packets)) {
        std::cerr << "cannot read " << argv[1] << "\n";
        return 2;
    }
    Gate g;
    g.idle_inputs();
    g.dut.rst_n = 0;
    g.tick();
    g.tick();
    g.idle_inputs();
    long expected_symbols = 0;
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 0; i < packets.size(); ++i) {
            const Packet& p = packets[i];
            if (g.rng.below(5) == 0) {
                const long at = g.rng.below(3) == 0 ? -2 : g.rng.below(800);
                const bool by_reset = g.rng.below(4) == 0;
                if (g.run(p, at, by_reset)) {
                    expected_symbols += p.nsym;
                    continue;      // ended before the abort's edge
                }
                if (g.rng.below(2) == 0) {
                    ++g.replays;
                    if (g.run(p, -1, false)) expected_symbols += p.nsym;
                }
            } else if (g.run(p, -1, false)) {
                expected_symbols += p.nsym;
            }
        }
    }
    std::printf("rx_symbol G3: %zu packets x 2 passes, %ld completed, %ld symbols, %ld pilots, "
                "%ld phases, %ld soft subcarriers, %ld aborts (%ld resets, %ld replayed), "
                "%ld bins held, %ld scales late, %ld SIGNAL thetas before the pilots, %ld CORDIC "
                "requests held (%ld aborted waiting), %ld edges, %ld mismatches\n",
                packets.size(), g.packets, g.symbols, g.pilots, g.phis, g.softs, g.aborts,
                g.resets, g.replays, g.held, g.late_scale, g.theta_early, g.cordic_held,
                g.aborts_waiting, g.edge, g.mismatches);
    if (g.symbols == 0 || g.aborts == 0 || g.resets == 0 || g.replays == 0 || g.held == 0 ||
        g.late_scale == 0 || g.theta_early == 0 || g.cordic_held == 0 || g.aborts_waiting == 0) {
        std::cerr << "a path was never exercised — refusing to pass\n";
        return 1;
    }
    if (g.symbols != expected_symbols) {
        std::cerr << "symbols " << g.symbols << " != " << expected_symbols << "\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
