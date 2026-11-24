// G3 gate for the channel and the per-packet quotients (hw/rtl/rx_channel.sv with
// hw/rtl/div_serial.sv, harness hw/tb/channel_chain.sv): replays hw/vectors/rx_channel.txt and
// requires, for every packet, the model's (radio_phy::rx_ofdm_fixed::channel_from_bins,
// tracker_constants, soft_scale):
//   - shift, Σg and its bit length with chan_valid, exactly CHAN edges after the edge that
//     took T2's last used bin;
//   - BPSK's scale, the two coefficients and the variance (saturated to 12 bits), in that order,
//     each exactly at its edge after chan_valid (Q_AT);
//   - the scale of every modulation on request, REQ_EXTRA + its quotient bits edges after the
//     request when the divider is free; and one request made while the packet's own quotients
//     are still running, served after the variance;
//   - Hs and g of the 52 used bins read back through the stores' ports, after the requests and
//     (random words) while the quotients run; the queued request's edge after the variance.
// The packets are fed in three passes over the file: (1) the bins back to back in fft64's
// order (bit-reversed), T2 right behind T1; (2) a bin every 8 clocks; (3) random gaps, random
// orders, and aborted packets (this one or another, dropped during T1, T2, the second pass or
// the quotients, random inputs during the drop), after which the divider is idle, nothing of
// the aborted packet may come out, and the packet follows (one time in three at once); a reset
// in place of the drop one time in four; requests dropped while divided; and on every T1 bin
// an outside read of the address it writes (the store must not see it); every seventh packet
// after a packet at the rails aborted in T2's window and dropped for one clock (at once).
// The unused bins carry the vectors' values (the block must ignore them). Registers start
// from random values (+verilator+rand+reset+2). Every wait is bounded. Exit code 0 = passed.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Vchannel_chain.h"
#include "verilated.h"

namespace {

constexpr int N = 64;
constexpr long CHAN = 63;
// q_valid of BPSK's scale, outer, inner, variance after chan_valid (the divider at
// qbits + 2 a division: 18 + 2, 14 + 2, 14 + 2, 12 + 2, the first asked one edge after
// chan_valid and taken one edge later).
constexpr long Q_AT[4] = {21, 37, 53, 67};
constexpr int SCALE_Q[4] = {18, 18, 19, 20};
// A request seen on edge R while idle: asked on R + 1, taken on R + 2, done after
// R + 2 + qbits, q_valid after R + 3 + qbits.
constexpr long REQ_EXTRA = 3;
constexpr long WAIT = 400;
constexpr long WATCHDOG = 50'000'000;

struct Packet {
    int t1[N][2], t2[N][2];
    long shift, n, gsum, q[4], outer, inner, var;
    long hs[N][2], g[N];
};

bool used(int b) { return (b >= 1 && b <= 26) || (b >= 38 && b <= 63); }

int rev6(int x) {
    int r = 0;
    for (int b = 0; b < 6; ++b) r |= ((x >> b) & 1) << (5 - b);
    return r;
}

bool parse(const std::string& path, std::vector<Packet>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        for (char& c : line)
            if (c == ',') c = ' ';
        std::istringstream s(line);
        Packet p{};
        for (auto& v : p.t1) s >> v[0] >> v[1];
        for (auto& v : p.t2) s >> v[0] >> v[1];
        s >> p.shift >> p.n >> p.gsum >> p.q[0] >> p.q[1] >> p.q[2] >> p.q[3] >> p.outer >>
            p.inner >> p.var;
        for (auto& v : p.hs) s >> v[0] >> v[1];
        for (auto& v : p.g) s >> v;
        if (!s) return false;
        out.push_back(p);
    }
    return !out.empty();
}

long sext(long v, int bits) {
    const long m = 1L << (bits - 1);
    v &= (1L << bits) - 1;
    return (v ^ m) - m;
}

struct Gate {
    Vchannel_chain dut;
    std::vector<Packet> pk;
    long edge = 0, mismatches = 0, packets = 0, quotients = 0, requests = 0, queued = 0,
         drops = 0, words = 0, prompt = 0, resets = 0,
         early_reads = 0, busy_drops = 0;
    bool probe = false;
    long strong = 0;
    unsigned lcg = 0xC4A7;
    // Strobes seen on the last edge.
    bool chan = false, q = false;
    int qjob = 0;
    long qval = 0;

    unsigned rnd() {
        lcg = lcg * 1103515245u + 12345u;
        return (lcg >> 16) & 0x7fff;
    }

    void fail(const std::string& what) {
        if (mismatches < 20) std::fprintf(stderr, "edge %ld: %s\n", edge, what.c_str());
        ++mismatches;
    }

    void idle_inputs() {
        dut.bin_valid = 0;
        dut.scale_req = 0;
        dut.hs_re = 0;
        dut.g_re = 0;
        dut.drop = 0;
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
        chan = dut.chan_valid;
        q = dut.q_valid;
        qjob = dut.q_job;
        qval = static_cast<long>(dut.q_value);
    }

    // One idle clock; a strobe where none is expected is an error.
    void quiet(const char* where) {
        idle_inputs();
        tick();
        if (chan || q) fail(std::string("a strobe while ") + where);
    }

    void reset() {
        idle_inputs();
        dut.rst_n = 0;
        for (int i = 0; i < 4; ++i) tick();
        dut.rst_n = 1;
        tick();
    }

    void bin(const Packet& p, bool t2, int b) {
        idle_inputs();
        dut.bin_valid = 1;
        dut.bin_t2 = t2;
        dut.bin_idx = b;
        const int* v = t2 ? p.t2[b] : p.t1[b];
        // Pass 3: an outside read of the address a T1 bin writes, on the same edge (the block
        // must not pass it to the store: the wrapper forbids it and stops the simulation).
        if (!t2 && probe) {
            dut.hs_re = 1;
            dut.hs_raddr = b;
        }
        dut.bin_re = static_cast<uint16_t>(v[0]);
        dut.bin_im = static_cast<uint16_t>(v[1]);
        tick();
        if (chan || q) fail("a strobe while bins come");
    }

    std::vector<int> order(int pass) {
        std::vector<int> o(N);
        for (int i = 0; i < N; ++i) o[i] = rev6(i);
        if (pass == 3 && rnd() % 2) {
            for (int i = N - 1; i > 0; --i) std::swap(o[i], o[rnd() % (i + 1)]);
        }
        return o;
    }

    int gap(int pass) {
        if (pass == 1) return 0;
        if (pass == 2) return 7;
        const unsigned r = rnd() % 8;
        return r < 4 ? 0 : static_cast<int>(r - 3);
    }

    // Feeds the windows; returns the edge that took T2's last used bin. `stop` > 0: a drop
    // after that many bins (of 128), then false.
    bool feed(const Packet& p, int pass, long& last, int stop) {
        int count = 0;
        for (int w = 0; w < 2; ++w) {
            const auto o = order(pass);
            for (int b : o) {
                if (stop > 0 && count == stop) return false;
                bin(p, w == 1, b);
                ++count;
                if (w == 1 && used(b)) last = edge;
                if (count == 2 * N) break;
                for (int g = gap(pass); g > 0; --g) quiet("between bins");
            }
        }
        return true;
    }

    void drop_now(bool at_once = false) {
        // Random inputs during the drop (bins among them).
        const int len = at_once ? 1 : 1 + static_cast<int>(rnd() % 3);
        // One time in four a reset instead of the drop.
        const bool by_reset = !at_once && rnd() % 4 == 0;
        if (by_reset) ++resets;
        for (int i = 0; i < len; ++i) {
            dut.drop = !by_reset;
            dut.rst_n = !by_reset;
            dut.bin_valid = rnd() % 2;
            dut.bin_t2 = rnd() % 2;
            dut.bin_idx = rnd() % 64;
            dut.bin_re = rnd();
            dut.bin_im = rnd();
            dut.scale_req = rnd() % 2;
            dut.scale_mod = rnd() % 4;
            tick();
        }
        idle_inputs();
        dut.rst_n = 1;
        ++drops;
        // The divider abandons its division with the drop.
        if (dut.div_busy) fail("the divider busy after a drop");
        // Nothing of the dropped packet may come out; one drop in three is followed at once by
        // the packet again (nothing of the dropped one may linger in the pipeline).
        const int quiet_for = at_once || rnd() % 3 == 0 ? 0 : static_cast<int>(rnd() % 200);
        if (quiet_for == 0) ++prompt;
        for (int i = 0; i < quiet_for; ++i) quiet("after a drop");
    }

    void check_q(const Packet& p, int job, long got, const char* what) {
        long want;
        if (job == 0) want = p.q[0];
        else if (job == 1) want = p.outer;
        else if (job == 2) want = p.inner;
        else if (job == 3) want = p.var;
        else want = p.q[job - 4];
        if (got != want) {
            char m[200];
            std::snprintf(m, sizeof m, "packet %ld %s job %d: got %ld want %ld", packets, what,
                          job, got, want);
            fail(m);
        }
        ++quotients;
    }

    void run(const Packet& p, int pass) {
        long last = -1;
        probe = pass == 3;
        // Pass 3, every seventh packet: a packet at the rails (the vectors' second line) aborted
        // in T2's window with its bins back to back, one clock of drop, this packet at once:
        // nothing of the aborted one's pipeline (larger |Hs|² than any other) may reach it.
        if (pass == 3 && packets % 7 == 3 && &p != &pk[1]) {
            int stop = 64 + 20 + static_cast<int>(rnd() % 40);
            feed(pk[1], 1, last, stop);
            ++strong;
            drop_now(true);
            last = -1;
        }
        // Pass 3: one packet in three is preceded by an aborted one (dropped somewhere).
        if (pass == 3 && rnd() % 3 == 0) {
            // The aborted packet: this one or (half the time) another, so that what lingers
            // of it would differ from the packet that follows.
            const Packet& a = rnd() % 2 ? pk[rnd() % pk.size()] : p;
            const int where = rnd() % 4;
            if (where < 2) {
                const int stop = where == 0 ? 1 + rnd() % 63 : 64 + 1 + rnd() % 63;
                feed(a, pass, last, stop);
            } else {
                feed(a, pass, last, 0);
                // During the second pass (some 10 to 60 clocks on) or the quotients.
                const int wait = where == 2 ? 10 + rnd() % 50 : 64 + rnd() % 60;
                for (int i = 0; i < wait; ++i) {
                    idle_inputs();
                    tick();
                }
            }
            drop_now();
            last = -1;
        }
        feed(p, pass, last, 0);
        // chan_valid.
        long chan_edge = -1;
        for (long i = 0; i < WAIT; ++i) {
            idle_inputs();
            tick();
            if (q) fail("a quotient before the channel");
            if (chan) {
                chan_edge = edge;
                break;
            }
        }
        char m[240];
        if (chan_edge < 0) {
            fail("no chan_valid");
            return;
        }
        if (chan_edge - last != CHAN) {
            std::snprintf(m, sizeof m, "packet %ld: chan_valid %ld edges after the last bin, want %ld",
                          packets, chan_edge - last, CHAN);
            fail(m);
        }
        if (static_cast<long>(dut.shift) != p.shift || static_cast<long>(dut.gsum) != p.gsum ||
            static_cast<long>(dut.gsum_n) != p.n) {
            std::snprintf(m, sizeof m, "packet %ld: shift %ld gsum %ld n %ld, want %ld %ld %ld",
                          packets, static_cast<long>(dut.shift), static_cast<long>(dut.gsum),
                          static_cast<long>(dut.gsum_n), p.shift, p.gsum, p.n);
            fail(m);
        }
        // The packet's quotients; in pass 3 sometimes a request while they run.
        int early = -1;
        long early_at = -1;
        if (pass == 3 && rnd() % 2) early_at = chan_edge + 1 + rnd() % 60;
        int next = 0;
        long q3_edge = -1;
        for (long i = 0; i < WAIT && next < 4; ++i) {
            idle_inputs();
            if (edge + 1 == early_at) {
                early = rnd() % 4;
                dut.scale_req = 1;
                dut.scale_mod = early;
            }
            // The stores answer while the quotients run (SIGNAL is equalised then).
            int rd = -1;
            if (rnd() % 2) {
                do rd = rnd() % N; while (!used(rd));
                dut.hs_re = 1;
                dut.hs_raddr = rd;
                dut.g_re = 1;
                dut.g_raddr = rd;
            }
            tick();
            if (rd >= 0) {
                check_word(p, rd);
                ++early_reads;
            }
            if (chan) fail("a second chan_valid");
            if (!q) continue;
            if (qjob != next) {
                std::snprintf(m, sizeof m, "packet %ld: job %d out, want %d", packets, qjob, next);
                fail(m);
            }
            if (edge - chan_edge != Q_AT[next]) {
                std::snprintf(m, sizeof m, "packet %ld: job %d %ld edges after chan, want %ld",
                              packets, next, edge - chan_edge, Q_AT[next]);
                fail(m);
            }
            check_q(p, qjob, qval, "quotient");
            if (next == 3) q3_edge = edge;
            ++next;
        }
        if (next < 4) fail("the packet's quotients did not all come");
        if (early >= 0) {
            ++queued;
            bool got = false;
            for (long i = 0; i < WAIT && !got; ++i) {
                quiet_or_q();
                if (q) {
                    if (qjob != 4 + early) fail("the queued request's job");
                    // Asked on the edge after the variance's q_valid.
                    if (edge - q3_edge != REQ_EXTRA + SCALE_Q[early]) {
                        std::snprintf(m, sizeof m, "packet %ld: queued scale %ld edges after the variance, want %ld",
                                      packets, edge - q3_edge, REQ_EXTRA + SCALE_Q[early]);
                        fail(m);
                    }
                    check_q(p, qjob, qval, "queued scale");
                    got = true;
                }
            }
            if (!got) fail("the queued request was not served");
        }
        // Every modulation's scale on request, in a random order.
        int mods[4] = {0, 1, 2, 3};
        for (int i = 3; i > 0; --i) std::swap(mods[i], mods[rnd() % (i + 1)]);
        for (int mod : mods) {
            idle_inputs();
            dut.scale_req = 1;
            dut.scale_mod = mod;
            tick();
            const long r = edge;
            if (chan || q) fail("a strobe with the request");
            bool got = false;
            for (long i = 0; i < WAIT && !got; ++i) {
                quiet_or_q();
                if (q) {
                    got = true;
                    if (qjob != 4 + mod) fail("the request's job");
                    if (edge - r != REQ_EXTRA + SCALE_Q[mod]) {
                        std::snprintf(m, sizeof m, "packet %ld: scale %d after %ld edges, want %ld",
                                      packets, mod, edge - r, REQ_EXTRA + SCALE_Q[mod]);
                        fail(m);
                    }
                    check_q(p, qjob, qval, "scale");
                }
            }
            if (!got) fail("a request was not served");
            ++requests;
        }
        // The stores.
        for (int b = 0; b < N; ++b) {
            if (!used(b)) continue;
            idle_inputs();
            dut.hs_re = 1;
            dut.hs_raddr = b;
            dut.g_re = 1;
            dut.g_raddr = b;
            tick();
            check_word(p, b);
            ++words;
        }
        ++packets;
        // Pass 3: now and then a request dropped (or reset) while it is divided.
        if (pass == 3 && rnd() % 6 == 0) {
            idle_inputs();
            dut.scale_req = 1;
            dut.scale_mod = rnd() % 4;
            tick();
            for (int i = static_cast<int>(rnd() % 15); i > 0; --i) quiet("before the drop");
            ++busy_drops;
            drop_now();
        }
    }

    void check_word(const Packet& p, int b) {
        const long re = sext(dut.hs_rdata_re, 18), im = sext(dut.hs_rdata_im, 18);
        const long g = dut.g_rdata;
        if (re != p.hs[b][0] || im != p.hs[b][1] || g != p.g[b]) {
            char m[200];
            std::snprintf(m, sizeof m, "packet %ld bin %d: Hs %ld,%ld g %ld; want %ld,%ld %ld",
                          packets, b, re, im, g, p.hs[b][0], p.hs[b][1], p.g[b]);
            fail(m);
        }
    }

    void quiet_or_q() {
        idle_inputs();
        tick();
        if (chan) fail("chan_valid while waiting for a quotient");
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string path = argc > 1 ? argv[1] : "vectors/rx_channel.txt";
    auto gate = std::make_unique<Gate>();
    if (!parse(path, gate->pk)) {
        std::fprintf(stderr, "cannot read %s\n", path.c_str());
        return 1;
    }
    gate->reset();
    for (int pass = 1; pass <= 3; ++pass)
        for (const auto& p : gate->pk) gate->run(p, pass);
    std::printf("rx_channel: %zu packets x 3 passes, %ld quotients, %ld requests (%ld queued), "
                "%ld drops (%ld replayed at once, %ld resets, %ld during a division, %ld after a packet at the rails), %ld store words (+%ld during the quotients), %ld edges, %ld mismatches\n",
                gate->pk.size(), gate->quotients, gate->requests, gate->queued, gate->drops,
                gate->prompt, gate->resets, gate->busy_drops, gate->strong, gate->words, gate->early_reads, gate->edge, gate->mismatches);
    const bool ok = gate->mismatches == 0 && gate->packets == 3 * static_cast<long>(gate->pk.size()) &&
                    gate->drops > 20 && gate->prompt > 5 && gate->queued > 20 &&
                    gate->resets > 5 && gate->busy_drops > 10 && gate->strong > 40 && gate->early_reads > 1000;
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
