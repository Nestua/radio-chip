// G3 gate for the receiver's FFT (hw/rtl/fft64.sv): replays hw/vectors/fft64.txt and requires
// every bin to equal radio_phy::rx_ofdm_fixed::fft64_window_fixed's, each window's 64 bins
// exactly once, in bit-reversed order, and nothing else out.
//
// The windows are fed in three passes, each over every window of the file:
//   1. no gaps, a reader that looks at in_ready in the same cycle, a flush after each window;
//   2. a sample every 8 clocks (the hand-off's pace once caught up), a reader whose valid is
//      registered (it sees in_ready a cycle late, so it hits the skid when a flush starts),
//      every fourth window chained to the next with in_next (T1 then T2): in_ready must stay
//      high from the chained window's first cycle on (nothing flushes between the two);
//   3. random gaps, a random reader, random chaining, and `drop` pulsed now and then: in a
//      window (the window is lost), during a flush (the bins not yet out are lost), or between
//      windows. Nothing of a dropped window may come out after the drop; the next windows must.
// When a flush follows a window at once, its last bin must leave exactly LAST edges after the
// edge that took its 64th sample. Registers start from random values
// (+verilator+rand+reset+2). Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vfft64.h"
#include "verilated.h"

namespace {

constexpr int N = 64;
constexpr long LAST = 75;
constexpr long WATCHDOG = 20'000'000;
constexpr int IN_BITS = 13;

struct Window {
    std::vector<int> re, im, bre, bim;
};

int rev6(int x) {
    int r = 0;
    for (int b = 0; b < 6; ++b) r |= ((x >> b) & 1) << (5 - b);
    return r;
}

struct Pending {
    size_t window;
    int got = 0;
    long edge64;       // the edge that took its 64th sample
    bool flush_next;   // a flush followed at once
};

struct Gate {
    Vfft64 dut;
    std::vector<Window> w;
    long edge = 0, bins = 0, mismatches = 0, latency_checked = 0, skids = 0, drops = 0, chains = 0;
    std::deque<Pending> pending;
    unsigned lcg = 4242;
    bool ready_before = false;  // in_ready in the cycle before the last edge

    unsigned rnd() {
        lcg = lcg * 1103515245u + 12345u;
        return (lcg >> 16) & 0x7fff;
    }

    void fail(const std::string& what) {
        if (mismatches < 16) std::fprintf(stderr, "edge %ld: %s\n", edge, what.c_str());
        ++mismatches;
    }

    // One clock edge; then the registered outputs are checked.
    void tick() {
        dut.eval();
        ready_before = dut.in_ready;
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++edge;
        if (edge > WATCHDOG) {
            std::fprintf(stderr, "watchdog\n");
            std::exit(1);
        }
        if (!dut.out_valid) return;
        if (pending.empty()) {
            fail("a bin where none is due");
            return;
        }
        Pending& p = pending.front();
        const Window& win = w[p.window];
        const int bin = dut.out_bin;
        const int re = static_cast<int16_t>(dut.out_re);
        const int im = static_cast<int16_t>(dut.out_im);
        char msg[200];
        if (bin != rev6(p.got)) {
            std::snprintf(msg, sizeof msg, "window %zu: bin %d out, want %d", p.window, bin,
                          rev6(p.got));
            fail(msg);
        } else if (re != win.bre[bin] || im != win.bim[bin]) {
            std::snprintf(msg, sizeof msg, "window %zu bin %d: got %d,%d want %d,%d", p.window,
                          bin, re, im, win.bre[bin], win.bim[bin]);
            fail(msg);
        }
        ++bins;
        if (++p.got == N) {
            if (p.flush_next) {
                ++latency_checked;
                if (edge - p.edge64 != LAST) {
                    std::snprintf(msg, sizeof msg, "window %zu: last bin %ld edges after its 64th sample, want %ld",
                                  p.window, edge - p.edge64, LAST);
                    fail(msg);
                }
            }
            pending.pop_front();
        }
    }

    void idle_inputs() {
        dut.in_valid = 0;
        dut.in_next = rnd() & 1;
        dut.in_re = rnd() & ((1 << IN_BITS) - 1);
        dut.in_im = rnd() & ((1 << IN_BITS) - 1);
    }

    void drop_now() {
        dut.drop = 1;
        idle_inputs();
        pending.clear();
        ++drops;
        tick();
        dut.drop = 0;
    }

    // Feeds window `k`; `gaps`: 0 none, 1 a sample every 8 clocks, 2 random. `registered`: the
    // reader presents a sample only if in_ready was high after the last edge (else: high now).
    // `next`: chained to the following window. `cut` ≥ 0: drop after that many samples.
    // Returns false if the window was dropped.
    // `chained`: the window before was fed with in_next, so nothing may flush between the two:
    // in_ready must stay high from its first cycle on.
    bool feed(size_t k, int gaps, bool registered, bool next, int cut, bool chained = false) {
        const Window& win = w[k];
        int n = 0;
        // A registered reader decides its valid at an edge from what in_ready was before it:
        // it presents in cycle c iff in_ready was high in cycle c − 1.
        bool ready_seen = ready_before;
        while (n < N) {
            if (n == cut) {
                drop_now();
                return false;
            }
            bool gap = false;
            if (gaps == 1) gap = (edge % 8) != 0;
            if (gaps == 2) gap = (rnd() % 4) == 0;
            dut.eval();
            if (chained && !dut.in_ready) {
                char msg[96];
                std::snprintf(msg, sizeof msg, "window %zu: in_ready low after in_next", k);
                fail(msg);
                chained = false;
            }
            const bool may = registered ? ready_seen : static_cast<bool>(dut.in_ready);
            if (gap || !may) {
                idle_inputs();
            } else {
                if (!dut.in_ready) ++skids;
                dut.in_valid = 1;
                dut.in_next = (n == N - 1) && next;
                dut.in_re = win.re[n] & ((1 << IN_BITS) - 1);
                dut.in_im = win.im[n] & ((1 << IN_BITS) - 1);
                ++n;
            }
            dut.eval();
            const bool last = dut.in_valid && n == N;
            const bool r = dut.in_ready;
            tick();
            ready_seen = r;
            ready_before = r;
            if (last) pending.push_back({k, 0, edge, !next});
        }
        idle_inputs();
        return true;
    }

    void idle(long c) {
        idle_inputs();
        for (long i = 0; i < c; ++i) tick();
    }

    void reset() {
        dut.rst_n = 0;
        dut.drop = 0;
        idle(3);
        dut.rst_n = 1;
        pending.clear();
    }

    void pass(int which) {
        size_t k = 0;
        while (k < w.size()) {
            int gaps = which == 1 ? 0 : which == 2 ? 1 : static_cast<int>(rnd() % 3);
            bool registered = which == 2 || (which == 3 && (rnd() & 1));
            bool next = k + 1 < w.size() && (which == 2 ? (k % 4 == 0) : which == 3 && rnd() % 3 == 0);
            int cut = -1;
            if (which == 3 && rnd() % 6 == 0) cut = static_cast<int>(rnd() % N);
            bool ok = feed(k, gaps, registered, next, cut);
            if (ok && next) {
                ++k;
                feed(k, gaps, registered, false, -1, true);
                ++chains;
            }
            ++k;
            if (which == 3) {
                int r = rnd() % 8;
                if (r == 0) {
                    idle(rnd() % 80);  // inside the flush or after the bins
                    drop_now();
                } else if (r > 2) {
                    idle(rnd() % 100);
                }  // else the next window at once (a registered reader then hits the skid)
            } else {
                idle(which == 1 ? 0 : 5);
            }
        }
        idle(200);
        if (!pending.empty()) fail("windows whose bins never came out");
        pending.clear();
    }
};

bool parse(const std::string& path, std::vector<Window>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream in(line);
        Window win;
        std::string pair;
        for (int i = 0; i < 2 * N; ++i) {
            if (!(in >> pair)) return false;
            const auto c = pair.find(',');
            const int a = std::stoi(pair.substr(0, c)), b = std::stoi(pair.substr(c + 1));
            if (i < N) {
                win.re.push_back(a);
                win.im.push_back(b);
            } else {
                win.bre.push_back(a);
                win.bim.push_back(b);
            }
        }
        out.push_back(win);
    }
    return !out.empty();
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string path = argc > 1 ? argv[1] : "vectors/fft64.txt";
    auto gate = std::make_unique<Gate>();
    if (!parse(path, gate->w)) {
        std::fprintf(stderr, "cannot read %s\n", path.c_str());
        return 1;
    }
    gate->reset();
    for (int p = 1; p <= 3; ++p) gate->pass(p);
    std::printf("fft64: %zu windows, 3 passes, %ld bins compared, %ld latencies checked, %ld skids, "
                "%ld drops, %ld chains, %ld edges, %ld mismatches\n",
                gate->w.size(), gate->bins, gate->latency_checked, gate->skids, gate->drops,
                gate->chains, gate->edge, gate->mismatches);
    const bool ok = gate->mismatches == 0 && gate->bins > 0 && gate->skids > 0 && gate->drops > 0 &&
                    gate->chains > 0;
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
