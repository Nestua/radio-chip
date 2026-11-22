// G3 gate for the decision's datapath (hw/rtl/rx_decide.sv, hw/rtl/ltf_corr1.sv) in the
// receiver front with its loop closed (hw/tb/decide_chain.sv): replays hw/vectors/decide.txt.
// Only the ADC samples go in; the decision's own `acquired` clears the detector's run counter
// and closes the search. Required, against radio_phy::rx_front_fixed::RxFrontFixed::push_traced
// bit for bit:
//   - an acceptance strobe at exactly the samples the model accepts a candidate at;
//   - at each result (res_valid): whether the decision got to its sums, and the sums — the
//     final coarse sum, the confirmation, the coherence, the energy, and the fine sum where
//     the model made it (it does not for a rejected candidate);
//   - at sample n + L of each acceptance at n (dec_valid, kDecLatency edges after that sample):
//     whether an acquisition is applied there, with T1 and the three phase words.
// And against radio_phy::rx_decision::run_schedule, in the run at R = 8 and D = 4 (whose clock
// pattern rvec gave the model): the cycle of each full decision's result after its strobe.
//
// usage: sim_decide <vectors.txt> <R> <D> <W>   (R at least 8: ltf_corr)
//
// Samples come every R clocks, with one more idle clock after every 5th and two after every
// 13th (idle inputs carry garbage); D flush samples follow a segment. Before each segment the
// chain is reset in the middle of a decision (the segment up to its first acceptance and 12 to
// 40 samples more, a few clocks, a one-clock reset): nothing of it may reach the run that
// follows. The RTL's own checks ($error) abort the run. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vdecide_chain.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr int kSumBits = 34;
constexpr int kPBits = 22;
constexpr long kAccLatency = 23;   // the acceptance strobe, edges after sample n + D
constexpr long kDecLatency = 3;    // dec_valid, edges after sample n + L
constexpr long kL = 64;            // the decision latency of the vectors
constexpr long kPosMod = 512;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Accept {
    bool has_sums = false, has_fine = false, has_done = false;
    long long t1 = 0, cre = 0, cim = 0, conf = 0, coh = 0, en = 0, fre = 0, fim = 0, done = 0;
};

struct Acq {
    long long t1 = 0, cfo = 0, coarse = 0, fine = 0;
};

struct Sample {
    long xr = 0, xi = 0;
    int acc = -1, acq = -1;   // indices into the segment's records
};

struct Segment {
    std::string label;
    int w = 0;
    std::vector<Sample> s;
    std::vector<Accept> acc;
    std::vector<Acq> acq;
};

struct Flight {
    size_t n;      // the acceptance's sample
    long strobe;   // its cycle
};

struct Gate {
    Vdecide_chain dut;
    int R = 8, D = 4, W = 12;
    long cycle = 0, mismatches = 0;
    long accs = 0, results = 0, summed = 0, fines = 0, timed = 0, decs = 0, acqs = 0;
    long in_flight_resets = 0;
    const Segment* seg = nullptr;
    std::vector<long> fed;
    std::deque<Flight> waiting;        // accepted, result not yet seen
    std::vector<char> decided;         // per sample: dec_valid seen for the acceptance there
    bool junk = false, quiet = false;
    long after_reset = 0;

    void fail(const char* what, size_t idx, long long r, long long g) {
        if (mismatches < 16)
            std::fprintf(stderr, "%s %s %zu: rtl=%lld golden=%lld\n", seg->label.c_str(), what, idx, r, g);
        ++mismatches;
    }

    // The sample fed at `edge`, or −1.
    long sample_at(long edge) const {
        for (size_t k = fed.size(); k-- > 0;) {
            if (fed[k] == edge) return static_cast<long>(k);
            if (fed[k] < edge) break;
        }
        return -1;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        if (quiet || junk) {
            if (quiet) after_reset += dut.acc_valid + dut.res_valid + dut.dec_valid + dut.acquired;
            return;
        }
        const size_t n = seg->s.size();
        if (dut.acc_valid) {
            const long k = sample_at(cycle - kAccLatency);
            const long a = k - D;
            if (k < 0 || a < 0 || static_cast<size_t>(a) >= n || seg->s[a].acc < 0) {
                fail("acceptance the model does not make, at sample", static_cast<size_t>(a < 0 ? 0 : a), 1, 0);
            } else {
                if (dut.acc_n != a % kPosMod) fail("acc_n", a, dut.acc_n, a % kPosMod);
                waiting.push_back({static_cast<size_t>(a), cycle});
                ++accs;
            }
        }
        if (dut.res_valid) {
            if (waiting.empty()) {
                fail("result without an acceptance", 0, 1, 0);
            } else {
                const Flight f = waiting.front();
                waiting.pop_front();
                const Accept& g = seg->acc[seg->s[f.n].acc];
                ++results;
                if (static_cast<bool>(dut.res_sums) != g.has_sums) fail("sums made", f.n, dut.res_sums, g.has_sums);
                if (dut.res_sums && g.has_sums) {
                    ++summed;
                    const long long cre = sext(dut.res_coarse_re, kSumBits), cim = sext(dut.res_coarse_im, kSumBits);
                    if (cre != g.cre) fail("coarse re", f.n, cre, g.cre);
                    if (cim != g.cim) fail("coarse im", f.n, cim, g.cim);
                    if (static_cast<long long>(dut.res_confirmation) != g.conf)
                        fail("confirmation", f.n, static_cast<long long>(dut.res_confirmation), g.conf);
                    if (static_cast<long long>(dut.res_coherence) != g.coh)
                        fail("coherence", f.n, static_cast<long long>(dut.res_coherence), g.coh);
                    if (static_cast<long long>(dut.res_energy) != g.en) fail("energy", f.n, dut.res_energy, g.en);
                    if (g.has_fine) {
                        ++fines;
                        const long long fre = sext(dut.res_fine_re, kSumBits), fim = sext(dut.res_fine_im, kSumBits);
                        if (fre != g.fre) fail("fine re", f.n, fre, g.fre);
                        if (fim != g.fim) fail("fine im", f.n, fim, g.fim);
                    }
                    if (g.has_done && R == 8 && D == 4) {
                        ++timed;
                        if (cycle - f.strobe != g.done) fail("result's cycle after the strobe", f.n, cycle - f.strobe, g.done);
                    }
                }
            }
        }
        if (dut.acquired && !dut.dec_valid) fail("acquired without dec_valid", 0, 1, 0);
        if (dut.dec_valid) {
            const long q = sample_at(cycle - kDecLatency);
            const long a = q - kL;
            if (q < 0 || a < 0 || static_cast<size_t>(a) >= n || seg->s[a].acc < 0) {
                fail("dec_valid at a sample that is no acceptance + L", static_cast<size_t>(q < 0 ? 0 : q), 1, 0);
            } else {
                ++decs;
                decided[a] = 1;
                const int qi = static_cast<size_t>(q) < n ? seg->s[q].acq : -1;
                if (static_cast<bool>(dut.acquired) != (qi >= 0)) fail("acquired", q, dut.acquired, qi >= 0);
                if (dut.acquired && qi >= 0) {
                    const Acq& g = seg->acq[qi];
                    ++acqs;
                    if (dut.acq_t1 != g.t1 % kPosMod) fail("T1", q, dut.acq_t1, g.t1 % kPosMod);
                    if (sext(dut.acq_cfo, kPBits) != g.cfo) fail("cfo", q, sext(dut.acq_cfo, kPBits), g.cfo);
                    if (sext(dut.acq_coarse, kPBits) != g.coarse) fail("coarse phase", q, sext(dut.acq_coarse, kPBits), g.coarse);
                    if (sext(dut.acq_fine, kPBits) != g.fine) fail("fine phase", q, sext(dut.acq_fine, kPBits), g.fine);
                }
            }
        }
    }

    void feed(long xr, long xi, size_t k) {
        fed.push_back(cycle + 1);
        dut.in_valid = 1;
        dut.in_re = static_cast<uint32_t>(xr) & ((1u << kXBits) - 1);
        dut.in_im = static_cast<uint32_t>(xi) & ((1u << kXBits) - 1);
        tick();
        dut.in_valid = 0;
        dut.in_re = 0x7ff;  // garbage while idle
        dut.in_im = 0x800;
        int idle = R - 1;
        if (k % 5 == 4) idle += 1;
        if (k % 13 == 12) idle += 2;
        for (int i = 0; i < idle; ++i) tick();
    }

    void reset(int clocks) {
        dut.rst_n = 0;
        dut.in_valid = 0;
        for (int i = 0; i < clocks; ++i) tick();
        dut.rst_n = 1;
    }

    // A reset in the middle of a decision: the grid, the sums, the rotation or the tail,
    // by how far past the acceptance the segment is fed.
    void reset_in_flight(const Segment& s) {
        size_t a = 0;
        while (a < s.s.size() && s.s[a].acc < 0) ++a;
        const size_t upto = a + D + 3 + 9 * (in_flight_resets % 5);
        if (upto >= s.s.size()) return;
        const int more = static_cast<int>(in_flight_resets % 7);
        ++in_flight_resets;
        junk = true;
        reset(1);
        fed.clear();
        for (size_t k = 0; k <= upto; ++k) feed(s.s[k].xr, s.s[k].xi, k);
        for (int i = 0; i < more; ++i) tick();
        junk = false;
    }

    void run(const Segment& s) {
        seg = &s;
        reset_in_flight(s);
        junk = true;
        reset(1);
        junk = false;
        fed.clear();
        waiting.clear();
        const size_t n = s.s.size();
        decided.assign(n, 0);
        for (size_t k = 0; k < n; ++k) feed(s.s[k].xr, s.s[k].xi, k);
        for (int k = 0; k < D; ++k) feed(0, 0, n + k);   // flush
        for (int i = 0; i < 40; ++i) tick();
        // Every acceptance whose result's sample is in the segment was decided there.
        for (size_t k = 0; k + kL < n; ++k) {
            if (s.s[k].acc >= 0 && !decided[k]) fail("no dec_valid for the acceptance at sample", k, 0, 1);
        }
        size_t expected = 0;
        for (size_t k = 0; k + D < n + D && k < n; ++k) expected += s.s[k].acc >= 0;
        if (static_cast<size_t>(accs_in_segment()) != expected)
            fail("acceptances in the segment", 0, accs_in_segment(), static_cast<long long>(expected));
        quiet = true;
        reset(2);
        for (int i = 0; i < 60; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0);
            after_reset = 0;
        }
    }

    long seg_accs_before = 0;
    long accs_in_segment() const { return accs - seg_accs_before; }
};

bool field(std::istringstream& f, long long& v, bool& present) {
    std::string t;
    if (!(f >> t)) return false;
    present = t != "-";
    v = present ? std::stoll(t) : 0;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 5) {
        std::cerr << "usage: sim_decide <vectors.txt> <R> <D> <W>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    Gate g;
    g.R = std::atoi(argv[2]);
    g.D = std::atoi(argv[3]);
    g.W = std::atoi(argv[4]);
    if (g.R < 8 || g.D < 1 || g.W < 1) {
        std::cerr << "bad R, D or W\n";
        return 2;
    }
    std::vector<Segment> segments;
    std::string line;
    auto bad = [&line]() {
        std::cerr << "malformed vector line: " << line << "\n";
        return 2;
    };
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string a;
        if (!(f >> a)) continue;
        if (a == "reset") {
            segments.push_back({});
            if (!(f >> segments.back().label >> segments.back().w)) return bad();
            continue;
        }
        if (segments.empty()) return bad();
        Segment& s = segments.back();
        if (a == "A") {
            Accept x;
            bool pt = false, p1 = false, p2 = false, p3 = false, p4 = false, p5 = false, f1 = false, f2 = false;
            if (s.s.empty() || !field(f, x.t1, pt) || !field(f, x.cre, p1) || !field(f, x.cim, p2) ||
                !field(f, x.conf, p3) || !field(f, x.coh, p4) || !field(f, x.en, p5) ||
                !field(f, x.fre, f1) || !field(f, x.fim, f2) || !field(f, x.done, x.has_done) || !pt ||
                p1 != p2 || p1 != p3 || p1 != p4 || p1 != p5 || f1 != f2)
                return bad();
            x.has_sums = p1;
            x.has_fine = f1;
            s.s.back().acc = static_cast<int>(s.acc.size());
            s.acc.push_back(x);
        } else if (a == "Q") {
            Acq x;
            if (s.s.empty() || !(f >> x.t1 >> x.cfo >> x.coarse >> x.fine)) return bad();
            s.s.back().acq = static_cast<int>(s.acq.size());
            s.acq.push_back(x);
        } else {
            Sample x;
            x.xr = std::stol(a);
            if (!(f >> x.xi)) return bad();
            s.s.push_back(x);
        }
    }
    long samples = 0, run_segments = 0, want_acc = 0, want_acq = 0;
    for (const auto& s : segments) {
        if (s.w != g.W) continue;
        ++run_segments;
        samples += static_cast<long>(s.s.size());
        want_acc += static_cast<long>(s.acc.size());
        want_acq += static_cast<long>(s.acq.size());
        g.seg_accs_before = g.accs;
        g.run(s);
    }
    std::printf("decide G3 (R = %d, D = %d, W = %d): %ld segments, %ld samples, %ld acceptances, %ld results "
                "(%ld with sums, %ld with the fine sum, %ld held to the schedule's cycle), %ld decisions "
                "delivered, %ld acquisitions, %ld resets in flight, %ld mismatches\n",
                g.R, g.D, g.W, run_segments, samples, g.accs, g.results, g.summed, g.fines, g.timed, g.decs,
                g.acqs, g.in_flight_resets, g.mismatches);
    if (run_segments == 0 || g.summed == 0 || g.fines == 0 || g.acqs == 0 || g.in_flight_resets == 0 ||
        (g.R == 8 && g.D == 4 && g.timed == 0)) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.accs == want_acc && g.acqs == want_acq ? 0 : 1;
}
