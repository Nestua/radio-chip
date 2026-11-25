// G3 gate for the tracker (hw/rtl/rx_track.sv with the shared units hw/rtl/cordic_share.sv,
// hw/rtl/cordic_iter.sv, hw/rtl/div_share.sv, hw/rtl/div_serial.sv, wired by
// hw/tb/track_chain.sv): replays hw/vectors/rx_track.txt and requires every θ and every window
// move to match radio_phy::rx_ofdm_fixed::FixedTracking::{step, move_window} bit for bit, each
// in the edge the block's header gives it.
//
// The testbench is the tracker's surroundings: the packet's start with the CFO word and the
// ratio; the channel block's quotients at random times (some long after the pilots), with the
// jobs the tracker ignores among them; the symbol block's pilots (SIGNAL's any time after the
// start, the first data symbol's at least 48 clocks after it, every later one only after the
// previous θ); and the shared units' other requesters: CORDIC requests of the symbol block
// (priority; the C cases of the vectors, each answer checked) and the channel block's
// divisions (random operands within the divider's bound, each quotient checked), sparse or
// dense per packet. Packets are dropped at their end; one in five is aborted at a random edge
// or on the third cycle a tracker request waits (one abort in four by reset) and nothing may
// come out after it; half of those are replayed at once.
//
// Edges are the edge after which a signal is seen (inputs set in the cycle after edge k are
// taken on edge k + 1). Per the header: θ0 kTh0 (kTh0u without a ratio) after the edge that
// takes start; a step starts T = max(its pilots' edge, the last step's end) + 1; its outer
// request is seen at T + 7; an angle is seen 18 edges after its take is seen; C = max(T + 15,
// the last quotient's edge + 1), G0x = max(C + 9, outer + 1), G1x = max(G0x + 1, inner + 1)
// (outer, inner: the edges the angles are seen, + 1); then SIGNAL's step ends at G1x + 3 (no
// measurement) or G1x + 13, a data symbol's θ is seen at G1x + 4 (no measurement) or its
// dv_req at G1x + 13, its quotient 27 edges after its grant is seen, θ 6 edges after that.
// (The multiplier pipelined, 2026-11-25: every rule a product feeds one edge later.)
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

#include "Vtrack_chain.h"
#include "verilated.h"

namespace {

constexpr long kTh0 = 7;
constexpr long kTh0u = 4;
constexpr long kReqO = 7;
constexpr long kAngle = 18;
constexpr long kDiv = 27;
constexpr long kThFit = 6;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (bits == 64) ? ~0ull : ((1ull << bits) - 1);
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}
uint64_t mask(long long v, int bits) {
    return static_cast<uint64_t>(v) & ((bits == 64) ? ~0ull : ((1ull << bits) - 1));
}

struct Symbol {
    long long pil[4][2];
    long long theta, delta, shift;
};

struct Packet {
    long long cfo, ratio, co, ci, var;
    int nsym;
    std::vector<Symbol> sym;
    bool usable() const { return var > 0 && var <= 3 * 1024; }
};

struct Cordic {
    long long re, im, angle;
};

struct Rng {
    uint64_t s = 0x2545f4914f6cdd1dull;
    uint32_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<uint32_t>(s >> 11);
    }
    int below(int n) { return static_cast<int>(next() % static_cast<uint32_t>(n)); }
    uint64_t u64() { return (static_cast<uint64_t>(next()) << 32) ^ next(); }
};

struct Gate {
    Vtrack_chain dut;
    Rng rng;
    std::vector<Cordic> cases;
    long edge = 0;
    long mismatches = 0;
    long packets = 0, symbols = 0, thetas = 0, early = 0, late = 0, aborts = 0, resets = 0,
         replays = 0, fits = 0, unmeasured = 0, sig_unusable = 0, held_two = 0, late_q = 0,
         tr_cordic_waits = 0, tr_div_waits = 0, a_angles = 0, a_quots = 0, a_div_waited = 0,
         aborts_waiting = 0, noise_q = 0;

    void error(const char* what, long a = 0, long b = 0) {
        if (mismatches < 20) std::fprintf(stderr, "edge %ld: %s (%ld, %ld)\n", edge, what, a, b);
        ++mismatches;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++edge;
    }

    void idle_inputs() {
        dut.start = 0;
        dut.q_valid = 0;
        dut.pil_valid = 0;
        dut.ca_req = 0;
        dut.da_start = 0;
        // garbage on the operands outside a request: a waiting request must use its register
        dut.da_num = rng.u64() & ((1ull << 48) - 1);
        dut.da_den = rng.next() & 0xffffff;
        dut.da_qbits = rng.below(32);
        dut.ca_re = rng.u64() & ((1ull << 34) - 1);
        dut.ca_im = rng.u64() & ((1ull << 34) - 1);
        dut.drop = 0;
        dut.rst_n = 1;
    }

    void quiet(int n, const char* why) {
        for (int k = 0; k < n; ++k) {
            dut.eval();
            if (dut.theta_valid || dut.tr_cr_valid || dut.tr_dv_req || dut.ca_out ||
                dut.da_done || dut.tr_ca_valid)
                error(why);
            tick();
        }
    }

    // One packet; abort_at >= 0: abort at that edge from the start (by reset if by_reset);
    // -2: on the third cycle a tracker request waits. True when the packet completed.
    bool run(const Packet& p, long abort_at, bool by_reset) {
        const int ns = p.nsym;
        const long t0 = edge;
        const bool known = p.ratio != 0;
        // the quotients: jobs 1..3 once each, at random times; noise jobs now and then
        long q_at[4] = {0, 0, 0, 0};
        bool q_sent[4] = {false, false, false, false};
        const int q_spread = rng.below(3) == 0 ? 400 : 60;
        for (int j = 1; j <= 3; ++j) q_at[j] = t0 + 1 + rng.below(q_spread);
        long q_last = -1;
        // the other requesters' modes: 0 none, 1 sparse, 2 dense
        const int cmode = rng.below(3), dmode = rng.below(3);
        bool a_cpend = false, a_cout = false;
        int a_case = 0;
        long a_cnext = t0 + rng.below(40);
        bool a_dout = false;
        long a_dnext = t0 + rng.below(40);
        uint64_t a_dnum = 0, a_dden = 0;
        int a_dq = 0;
        bool a_dstarted = false;        // a's start was driven in this cycle
        long a_dfrom = -1;              // the edge a's division starts from (-2: waiting)
        bool tr_div_busy = false;       // the tracker's division is on the divider
        long inner_seen = -1;           // the step's inner request seen (per step, below)
        // pilots
        int ps = 0;
        long pil_next = t0 + 1 + rng.below(120);
        std::vector<long> ep(ns, -1), T(ns, -1), C(ns, -1), ao(ns, -1), ai(ns, -1), g1x(ns, -1),
            end(ns, -1), theta_seen(ns, -1), req_seen(ns, -1);
        int takes = 0, answers = 0;     // the tracker's CORDIC takes and answers, packet-wide
        std::vector<long> take_seen(2 * ns + 2, -1);
        long th0_seen = -1;
        int th_count = 0;               // θs seen
        long long shift_before = 0;
        long dv_req_seen = -1, gnt_seen = -1, done_seen = -1;
        int div_sym = -1;
        int tr_wait = 0;                // cycles a tracker request waits
        long end_at = -1;               // the packet is over: the drop once the others are done

        for (long guard = 0; guard < 400000; ++guard) {
            const long now = edge;
            idle_inputs();
            if ((abort_at >= 0 && now - t0 == abort_at) || (abort_at == -2 && tr_wait == 3)) {
                if (abort_at == -2) ++aborts_waiting;
                if (by_reset) dut.rst_n = 0; else dut.drop = 1;
                tick();
                idle_inputs();
                ++aborts;
                if (by_reset) ++resets;
                quiet(40, "output after an abort");
                return false;
            }
            // ---- inputs for the edge now + 1
            if (now == t0) {
                dut.start = 1;
                dut.cfo_word = static_cast<uint32_t>(mask(p.cfo, 22));
                dut.ratio = static_cast<uint32_t>(p.ratio);
            } else {
                for (int j = 1; j <= 3 && !dut.q_valid; ++j) {
                    if (!q_sent[j] && now >= q_at[j]) {
                        dut.q_valid = 1;
                        dut.q_job = j;
                        dut.q_value = static_cast<uint32_t>(j == 1 ? p.co : j == 2 ? p.ci : p.var);
                        q_sent[j] = true;
                        q_last = now + 1;
                    }
                }
                if (!dut.q_valid && rng.below(40) == 0) {
                    static const int noise[5] = {0, 4, 5, 6, 7};
                    dut.q_valid = 1;
                    dut.q_job = noise[rng.below(5)];
                    dut.q_value = rng.next() & 0xfffffff;
                    ++noise_q;
                }
                // pilots
                bool may = ps < ns && now >= pil_next;
                if (may && ps == 1) may = now + 1 >= ep[0] + 48;
                if (may && ps >= 2) may = theta_seen[ps - 1] >= 0 && now + 1 >= ep[ps - 1] + 48;
                if (may) {
                    dut.pil_valid = 1;
                    for (int k = 0; k < 4; ++k) {
                        dut.pil_re[k] = static_cast<uint32_t>(mask(p.sym[ps].pil[k][0], 17));
                        dut.pil_im[k] = static_cast<uint32_t>(mask(p.sym[ps].pil[k][1], 17));
                    }
                    ep[ps] = now + 1;
                    // the step's start: after its pilots and the last step's end
                    if (ps == 1 && (end[0] < 0 || end[0] > now + 1)) ++held_two;
                    ++ps;
                    pil_next = now + 1 + (rng.below(3) == 0 ? 0 : rng.below(150));
                }
            }
            // the symbol block's CORDIC requests (priority)
            if (cmode != 0 && end_at < 0 && !a_cpend && !a_cout && now >= a_cnext) {
                a_cpend = true;
                a_case = rng.below(static_cast<int>(cases.size()));
            }
            if (a_cpend) {
                dut.ca_req = 1;
                dut.ca_re = mask(cases[a_case].re, 34);
                dut.ca_im = mask(cases[a_case].im, 34);
            }
            // the channel block's divisions
            if (dmode != 0 && end_at < 0 && !a_dout && now >= a_dnext && now > t0) {
                a_dq = 1 + rng.below(28);
                a_dden = 1 + (rng.u64() % ((1ull << 24) - 1));
                const unsigned __int128 lim = static_cast<unsigned __int128>(a_dden) << a_dq;
                const uint64_t cap = (1ull << 48);
                const uint64_t top = lim < cap ? static_cast<uint64_t>(lim) : cap;
                a_dnum = rng.u64() % top;
                dut.da_start = 1;
                dut.da_num = a_dnum;
                dut.da_den = static_cast<uint32_t>(a_dden);
                dut.da_qbits = a_dq;
                a_dout = true;
                a_dstarted = true;
            }
            dut.eval();
            // a's division: it goes at once unless the tracker's is on the divider (the divider
            // is free in its done cycle); then on the tracker's done.
            if (a_dstarted) {
                a_dstarted = false;
                if (tr_div_busy && !dut.tr_dv_done) {
                    a_dfrom = -2;
                    ++a_div_waited;
                } else {
                    a_dfrom = now;
                }
            }
            if (dut.tr_dv_done && a_dfrom == -2) a_dfrom = now;
            if (dut.tr_dv_gnt) tr_div_busy = true;
            if (dut.tr_dv_done) tr_div_busy = false;
            // ---- combinational outputs of this cycle
            const bool trv = dut.tr_cr_valid, trt = dut.tr_cr_take;
            if (trv && !trt) ++tr_cordic_waits;
            if (dut.tr_dv_req && !dut.tr_dv_gnt) ++tr_div_waits;
            if ((trv && !trt) || (dut.tr_dv_req && !dut.tr_dv_gnt)) ++tr_wait;
            else tr_wait = 0;
            if (trv && takes == 2 * (answers / 2) && answers % 2 == 0) {
                // a step's first request: its outer pair
                const int s = answers / 2;
                if (s < ns && req_seen[s] < 0) {
                    req_seen[s] = now;
                    if (T[s] < 0 || now != T[s] + kReqO) error("outer request's edge", now, T[s] + kReqO);
                }
            }
            if (trv && takes % 2 == 1 && inner_seen < takes) {
                // the step's inner request after its outer was taken: raised on T + 11, so seen
                // there or, if the outer was taken later, in the cycle after its take
                const int s = takes / 2;
                inner_seen = takes;
                const long want = std::max(T[s] + 11, take_seen[takes - 1] + 1);
                if (s < ns && (T[s] < 0 || now != want)) error("inner request's edge", now, want);
            }
            if (trt) {
                if (takes >= 2 * ns) error("a tracker CORDIC request too many");
                else take_seen[takes] = now;
                ++takes;
            }
            if (dut.tr_ca_valid) {
                const int s = answers / 2;
                if (s >= ns) {
                    error("a tracker angle too many");
                } else {
                    if (now != take_seen[answers] + kAngle) error("angle's edge", now, take_seen[answers] + kAngle);
                    (answers % 2 == 0 ? ao : ai)[s] = now + 1;
                }
                ++answers;
            }
            if (a_cpend && dut.ca_take) {
                a_cpend = false;
                a_cout = true;
            }
            if (dut.ca_out) {
                if (!a_cout) error("an answer to no request of a");
                else if (sext(dut.angle, 16) != cases[a_case].angle) error("a's angle", a_case);
                a_cout = false;
                ++a_angles;
                a_cnext = now + 1 + (cmode == 2 ? 0 : rng.below(60));
            }
            if (dut.da_done) {
                if (!a_dout) error("a quotient for no request of a");
                else {
                    const uint64_t want = a_dnum / a_dden;
                    if (dut.quot != (want & ((1ull << 28) - 1)) || want >= (1ull << a_dq))
                        error("a's quotient", static_cast<long>(dut.quot), static_cast<long>(want));
                    if (a_dfrom < 0 || now != a_dfrom + 1 + a_dq)
                        error("a's quotient's edge", now, a_dfrom + 1 + a_dq);
                }
                a_dout = false;
                ++a_quots;
                a_dnext = now + 1 + (dmode == 2 ? rng.below(3) : rng.below(200));
            }
            if (dut.tr_dv_req && dv_req_seen < 0) {
                dv_req_seen = now;
                div_sym = th_count;  // the data symbol whose θ comes next
                if (div_sym < ns && g1x[div_sym] >= 0 && now != g1x[div_sym] + 13)
                    error("dv_req's edge", now, g1x[div_sym] + 13);
            }
            if (dut.tr_dv_gnt) {
                if (dv_req_seen < 0) error("a grant with no request");
                gnt_seen = now;
            }
            if (dut.tr_dv_done) {
                if (gnt_seen < 0) error("a tracker quotient with no division");
                else if (now != gnt_seen + kDiv) error("quotient's edge", now, gnt_seen + kDiv);
                done_seen = now;
            }
            // ---- registered outputs
            if (dut.theta_valid) {
                const int s = th_count;
                if (s >= ns) {
                    error("theta too many");
                } else {
                    const Symbol& y = p.sym[s];
                    if (sext(dut.theta, 23) != y.theta) error("theta", s, static_cast<long>(sext(dut.theta, 23) - y.theta));
                    const long long mv = y.shift - shift_before;
                    if (dut.move_early != (mv == -1) || dut.move_late != (mv == 1)) error("move", s, static_cast<long>(mv));
                    if (mv == -1) ++early;
                    if (mv == 1) ++late;
                    shift_before = y.shift;
                    if (sext(dut.fit_delta, 27) != y.delta)
                        error("delta", s, static_cast<long>(sext(dut.fit_delta, 27) - y.delta));
                    long want = -1;
                    if (s == 0) {
                        want = t0 + 1 + (known ? kTh0 : kTh0u);
                        th0_seen = now;
                    } else if (p.usable()) {
                        want = done_seen + kThFit;
                        ++fits;
                    } else {
                        want = g1x[s] + 4;
                        ++unmeasured;
                    }
                    if (now != want) error("theta's edge", now, want);
                    theta_seen[s] = now;
                    if (s > 0) end[s] = now;
                    dv_req_seen = gnt_seen = done_seen = -1;
                    ++thetas;
                    ++th_count;
                }
            }
            // ---- the steps' timeline (from what has been seen)
            for (int s = 0; s < ns; ++s) {
                const long prev = s == 0 ? th0_seen : end[s - 1];
                if (T[s] < 0 && ep[s] >= 0 && prev >= 0) T[s] = std::max(ep[s], prev) + 1;
                if (T[s] >= 0 && C[s] < 0 && q_last >= 0 && q_sent[1] && q_sent[2] && q_sent[3]) {
                    C[s] = std::max(T[s] + 15, q_last + 1);
                    if (C[s] > T[s] + 15) ++late_q;
                }
                if (C[s] >= 0 && g1x[s] < 0 && ao[s] >= 0 && ai[s] >= 0) {
                    const long g0x = std::max(C[s] + 9, ao[s] + 1);
                    g1x[s] = std::max(g0x + 1, ai[s] + 1);
                    if (s == 0) {
                        end[0] = g1x[0] + (p.usable() ? 13 : 3);
                        if (!p.usable()) ++sig_unusable;
                    }
                }
            }
            if (end_at >= 0 && (dut.theta_valid || trv || dut.tr_dv_req))
                error("tracker output after its last");
            tick();
            if (end_at < 0 && th_count == ns && end[ns - 1] >= 0 && edge > end[ns - 1])
                end_at = edge + 1 + rng.below(5);
            if (end_at >= 0 && edge >= end_at && !a_cpend && !a_cout && !a_dout) {
                idle_inputs();
                dut.drop = 1;
                tick();
                idle_inputs();
                ++packets;
                symbols += ns;
                return true;
            }
        }
        error("the packet never ended", th_count, ns);
        std::fprintf(stderr, "rx_track G3: stopped, %ld mismatches\n", mismatches);
        std::exit(1);
    }
};

bool parse(const char* path, std::vector<Packet>& out, std::vector<Cordic>& cases) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string tag;
        f >> tag;
        if (tag == "P") {
            Packet p{};
            f >> p.cfo >> p.ratio >> p.co >> p.ci >> p.var >> p.nsym;
            if (!f) return false;
            out.push_back(p);
        } else if (tag == "S") {
            if (out.empty()) return false;
            Symbol s{};
            for (auto& v : s.pil) f >> v[0] >> v[1];
            f >> s.theta >> s.delta >> s.shift;
            if (!f) return false;
            out.back().sym.push_back(s);
        } else if (tag == "C") {
            Cordic c{};
            f >> c.re >> c.im >> c.angle;
            if (!f) return false;
            cases.push_back(c);
        } else {
            return false;
        }
    }
    for (const auto& p : out)
        if (static_cast<int>(p.sym.size()) != p.nsym) return false;
    return !out.empty() && !cases.empty();
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_rx_track <vectors.txt>\n";
        return 2;
    }
    std::vector<Packet> packets;
    Gate g;
    if (!parse(argv[1], packets, g.cases)) {
        std::cerr << "cannot read " << argv[1] << "\n";
        return 2;
    }
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
                const long at = g.rng.below(3) == 0 ? -2 : g.rng.below(600);
                const bool by_reset = g.rng.below(4) == 0;
                if (g.run(p, at, by_reset)) {
                    expected_symbols += p.nsym;
                    continue;
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
    std::printf("rx_track G3: %zu packets x 2 passes, %ld completed, %ld symbols, %ld thetas "
                "(%ld fitted, %ld unmeasured; %ld moves early, %ld late), %ld SIGNAL steps "
                "unmeasured, %ld first data pilots held behind SIGNAL, %ld steps waiting for the "
                "quotients, %ld noise quotients, %ld tracker CORDIC waits, %ld tracker divider "
                "waits, %ld other angles, %ld other quotients (%ld behind the tracker), %ld aborts "
                "(%ld resets, %ld waiting, %ld replayed), %ld edges, %ld mismatches\n",
                packets.size(), g.packets, g.symbols, g.thetas, g.fits, g.unmeasured, g.early,
                g.late, g.sig_unusable, g.held_two, g.late_q, g.noise_q, g.tr_cordic_waits,
                g.tr_div_waits, g.a_angles, g.a_quots, g.a_div_waited, g.aborts, g.resets,
                g.aborts_waiting, g.replays, g.edge, g.mismatches);
    if (g.symbols == 0 || g.fits == 0 || g.unmeasured == 0 || g.early == 0 || g.late == 0 ||
        g.sig_unusable == 0 || g.held_two == 0 || g.late_q == 0 || g.noise_q == 0 ||
        g.tr_cordic_waits == 0 || g.tr_div_waits == 0 || g.a_angles == 0 || g.a_quots == 0 ||
        g.a_div_waited == 0 || g.aborts == 0 || g.resets == 0 || g.aborts_waiting == 0 ||
        g.replays == 0) {
        std::cerr << "a path was never exercised — refusing to pass\n";
        return 1;
    }
    if (g.symbols != expected_symbols) {
        std::cerr << "symbols " << g.symbols << " != " << expected_symbols << "\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
