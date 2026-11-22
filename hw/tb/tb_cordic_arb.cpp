// G3 gate for the CORDIC's arbiter (hw/rtl/cordic_arb.sv): two requesters on one CORDIC, the
// provisional latch (a: a pulse, always taken) before the decision engine (b: held until
// granted). Replays hw/vectors/cordic.txt (radio_phy::rx_front_fixed::cordic_angle): every
// request, from either side and under contention, must get the model's angle of its own
// input exactly kLatency clock edges after the edge that takes the request, on its own valid.
//
// Rules required of the RTL (ADR 0006: the latch before the decision; rx_decision: "a latch
// asking in the same cycle goes first and the decision asks again in the next"):
//   b_grant = b_req and not a_req, in the same cycle; a is never refused.
// Phases, in turn over the file: a alone back to back; b alone back to back; both at random
// (a in about a third of the cycles); a in every cycle with b waiting (b must not be granted)
// and then released; a reset with requests in the pipeline and at the inputs (nothing may
// come out).
// While b waits its inputs are held; a's inputs are garbage when a_req = 0, b's when
// b_req = 0. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vcordic_arb.h"
#include "verilated.h"

namespace {

constexpr int kInBits = 34;
constexpr int kAngleBits = 16;
// The arbiter's register and the CORDIC's 17 edges (rx_decision: the angle in cycle g + 19).
constexpr long kLatency = 18;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Vec {
    long long re, im, angle;
};

struct Pending {
    long cycle;   // the edge that took the request
    bool b;
    long long angle;
};

struct Gate {
    Vcordic_arb dut;
    long cycle = 0, mismatches = 0;
    long a_done = 0, b_done = 0, b_waits = 0, both = 0;
    std::deque<Pending> q;
    uint32_t lcg = 0x1234567u;
    bool quiet = false;
    long after_reset = 0;

    uint32_t rnd() {
        lcg = lcg * 1664525u + 1013904223u;
        return lcg >> 8;
    }

    void fail(const char* what, long a, long b) {
        if (mismatches < 12) std::fprintf(stderr, "cycle %ld: %s: rtl=%ld expected=%ld\n", cycle, what, a, b);
        ++mismatches;
    }

    void set(bool a, const Vec* av, bool b, const Vec* bv) {
        const uint64_t mask = (1ull << kInBits) - 1;
        dut.a_req = a;
        dut.a_re = a ? static_cast<uint64_t>(av->re) & mask : 0x155555555ull;
        dut.a_im = a ? static_cast<uint64_t>(av->im) & mask : 0x2aaaaaaaaull;
        dut.b_req = b;
        dut.b_re = b ? static_cast<uint64_t>(bv->re) & mask : 0x0f0f0f0f0ull;
        dut.b_im = b ? static_cast<uint64_t>(bv->im) & mask : 0x30f0f0f0full;
    }

    // One clock with the given requests; returns whether b was granted.
    bool step(bool a, const Vec* av, bool b, const Vec* bv) {
        set(a, av, b, bv);
        dut.clk = 0;
        dut.eval();
        const bool grant = dut.b_grant;
        if (grant != (b && !a)) fail("b_grant", grant, b && !a);
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (a) q.push_back({cycle, false, av->angle});
        if (b && !a) q.push_back({cycle, true, bv->angle});
        both += a && b;
        b_waits += a && b;
        if (quiet) {
            after_reset += dut.a_valid + dut.b_valid;
            return grant;
        }
        if (dut.a_valid && dut.b_valid) fail("both valids", 1, 0);
        if (dut.a_valid || dut.b_valid) {
            if (q.empty()) {
                fail("output without a request", 1, 0);
            } else {
                const Pending p = q.front();
                q.pop_front();
                if (cycle - p.cycle != kLatency) fail("latency", cycle - p.cycle, kLatency);
                if (static_cast<bool>(dut.b_valid) != p.b) fail("owner (1 = b)", dut.b_valid, p.b);
                const long long got = sext(dut.angle, kAngleBits);
                if (got != p.angle) fail("angle", static_cast<long>(got), static_cast<long>(p.angle));
                (p.b ? b_done : a_done) += 1;
            }
        } else if (!q.empty() && cycle - q.front().cycle >= kLatency) {
            fail("missing output", 0, 1);
            q.pop_front();
        }
        return grant;
    }

    void idle(int n) {
        for (int i = 0; i < n; ++i) step(false, nullptr, false, nullptr);
    }

    // A one-clock reset; `asking` keeps a request of each side up during it (it must be
    // dropped, not taken).
    void reset(const Vec* asking = nullptr) {
        dut.rst_n = 0;
        set(asking != nullptr, asking, asking != nullptr, asking);
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        dut.rst_n = 1;
        q.clear();
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_cordic_arb <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Vec> v;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        Vec x{};
        if (!(f >> x.re >> x.im >> x.angle)) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        v.push_back(x);
    }
    if (v.size() < 2000) {
        std::cerr << "too few vectors\n";
        return 2;
    }
    Gate g;
    g.reset();
    size_t i = 0;
    const size_t n = v.size();
    auto next = [&]() -> const Vec* { return &v[i++ % n]; };

    // 1. a alone, back to back.
    for (int k = 0; k < 600; ++k) g.step(true, next(), false, nullptr);
    g.idle(30);
    // 2. b alone, back to back.
    for (int k = 0; k < 600; ++k) g.step(false, nullptr, true, next());
    g.idle(30);
    // 3. both at random: b holds its request (and its input) until granted.
    const Vec* bv = nullptr;
    for (int k = 0; k < 20000; ++k) {
        const bool a = g.rnd() % 3 == 0;
        if (!bv && g.rnd() % 2 == 0) bv = next();
        const bool granted = g.step(a, a ? next() : nullptr, bv != nullptr, bv);
        if (granted) bv = nullptr;
    }
    for (int k = 0; bv && k < 4; ++k) {
        if (g.step(false, nullptr, true, bv)) bv = nullptr;
    }
    if (bv) g.fail("b never granted", 0, 1);
    g.idle(30);
    // 4. a in every cycle: b waits, then is taken in the first cycle a is silent.
    bv = next();
    for (int k = 0; k < 200; ++k) {
        if (g.step(true, next(), true, bv)) g.fail("b granted under a", 1, 0);
    }
    if (!g.step(false, nullptr, true, bv)) g.fail("b not granted when a is silent", 0, 1);
    g.idle(30);
    // 5. a reset with requests in the pipeline: nothing may come out.
    for (int k = 0; k < 6; ++k) g.step(k % 2 == 0, next(), k % 2 == 1, next());
    g.reset(next());
    g.quiet = true;
    g.idle(40);
    g.quiet = false;
    if (g.after_reset != 0) g.fail("outputs after reset", g.after_reset, 0);
    // And it works after the reset.
    for (int k = 0; k < 200; ++k) g.step(k % 3 == 0, next(), k % 3 != 0, next());
    g.idle(30);
    if (!g.q.empty()) g.fail("requests without an answer", static_cast<long>(g.q.size()), 0);

    std::printf("cordic_arb G3: %ld angles for the latch, %ld for the decision, %ld cycles with both "
                "asking, %ld mismatches\n",
                g.a_done, g.b_done, g.both, g.mismatches);
    if (g.a_done == 0 || g.b_done == 0 || g.both == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
