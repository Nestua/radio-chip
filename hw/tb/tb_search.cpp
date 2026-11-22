// G3 gate for the LTF search (hw/rtl/rx_search.sv) in the chain dc_block → det_a → rx_latch →
// nco → ltf_corr → rx_search (hw/tb/search_chain.sv): replays hw/vectors/search.txt and
// requires, for every sample, the energy completed at it, whether a search is open after it,
// and the candidate accepted at it (the sample, the maximum's position and energy, the
// opening event) to match radio_phy::rx_front_fixed::RxFrontFixed::push_traced bit for bit.
// After every acceptance it reads the energy ring as the decision will (rx_decision's grid:
// imax … imax + w, oldest first, one a cycle from the strobe's cycle on) and requires the
// model's energies.
//
// usage: sim_search <vectors.txt> <R> <D> <W>   R = clocks per sample (at least 8: ltf_corr),
//        D, W = the build's D_SAMPLES and W. Only the segments of that W are run.
//
// Samples come every R clocks, with one more idle clock after every 5th and two after every
// 13th (idle inputs carry garbage). The search runs D samples behind, so each segment is
// followed by D flush samples. Exact latencies, in clock edges after sample n + D: the energy
// kELatency, the sample's strobe (searching, acceptance) kSLatency — the schedule's
// Y(n + D) + 5 + 15 + 3 (rx_decision). Before each segment the chain is reset in the clocks
// in which the search takes an accepted sample (the segment up to its first acceptance, a
// one-clock reset 19 … 24 clocks after that sample, in turn), and again between an
// acquisition's strobe and the search's own sample of it; three junk samples precede each
// segment's end reset (the last one R − 1 clocks old, in nco or ltf_corr): nothing of any of
// these may come out or disturb the next run. The RTL's own checks ($error) abort the run.
// Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vsearch_chain.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr long kELatency = 22;
constexpr long kSLatency = 23;
constexpr long kPosMod = 512;   // sample numbers are handed over modulo the history's depth
constexpr long kRing = 81;      // words of the energy ring

struct Sample {
    long xr, xi, acq, event, searching;
    bool has_e, has_acc;
    long e, imax, emax, opened;
};

struct Segment {
    std::string label;
    int w = 0;
    std::vector<Sample> s;
    std::vector<size_t> e_at;   // the samples that complete an energy, in order
};

struct Gate {
    Vsearch_chain dut;
    int R = 8, D = 4, W = 12;
    long cycle = 0, mismatches = 0;
    long es = 0, ss = 0, accs = 0, ring_reads = 0;   // totals compared
    long in_flight_resets = 0, acquired_resets = 0;
    size_t ne = 0, ns = 0;                // per-segment output indices
    const Segment* seg = nullptr;
    std::vector<long> fed;                // cycle of each fed sample (segment + flush)
    bool junk = false;                    // not compared
    bool quiet = false;                   // between a reset and the next segment
    long after_reset = 0;                 // outputs seen after a reset (must stay 0)
    // The ring read in progress: position rd_pos, rd_left words to go; rd_due: a word was
    // asked for in the last cycle and is on ring_data now.
    long rd_pos = 0, rd_left = 0, rd_due_pos = -1;

    void fail(const char* what, size_t idx, long r, long g) {
        if (mismatches < 12)
            std::fprintf(stderr, "%s %s %zu: rtl=%ld golden=%ld\n", seg->label.c_str(), what, idx, r, g);
        ++mismatches;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        dut.ring_re = 0;
        if (quiet || junk) {
            if (quiet) after_reset += dut.e_valid + dut.s_valid + dut.acc_valid + dut.searching;
            rd_left = 0;
            rd_due_pos = -1;
            return;
        }
        const size_t n = seg->s.size();
        if (rd_due_pos >= 0) {
            const long g = seg->s[seg->e_at[static_cast<size_t>(rd_due_pos)]].e;
            if (static_cast<long>(dut.ring_data) != g)
                fail("ring word at position", static_cast<size_t>(rd_due_pos), static_cast<long>(dut.ring_data), g);
            ++ring_reads;
            rd_due_pos = -1;
        }
        if (dut.e_valid) {
            if (ne >= seg->e_at.size()) {
                fail("unexpected energy", ne, static_cast<long>(dut.e), 0);
            } else {
                const size_t k = seg->e_at[ne];
                const long lat = cycle - fed[k + D];
                if (lat != kELatency) fail("energy latency", k, lat, kELatency);
                if (static_cast<long>(dut.e) != seg->s[k].e) fail("energy", k, static_cast<long>(dut.e), seg->s[k].e);
                ++es;
            }
            ++ne;
        }
        if (dut.acc_valid && !dut.s_valid) fail("acceptance without a sample strobe", ns, 1, 0);
        if (dut.s_valid) {
            if (ns >= n || ns + D >= fed.size()) {
                fail("unexpected sample strobe", ns, 0, 0);
            } else {
                const Sample& g = seg->s[ns];
                const long lat = cycle - fed[ns + D];
                if (lat != kSLatency) fail("strobe latency", ns, lat, kSLatency);
                if (dut.searching != g.searching) fail("searching", ns, dut.searching, g.searching);
                if (dut.acc_valid != g.has_acc) fail("acceptance", ns, dut.acc_valid, g.has_acc);
                if (dut.acc_valid && g.has_acc) {
                    if (dut.acc_n != static_cast<long>(ns) % kPosMod) fail("acc_n", ns, dut.acc_n, static_cast<long>(ns) % kPosMod);
                    if (dut.acc_imax != g.imax % kPosMod) fail("acc_imax", ns, dut.acc_imax, g.imax % kPosMod);
                    if (dut.acc_lapped != (g.imax >= kPosMod)) fail("acc_lapped", ns, dut.acc_lapped, g.imax >= kPosMod);
                    if (dut.acc_ring != g.imax % kRing) fail("acc_ring", ns, dut.acc_ring, g.imax % kRing);
                    if (static_cast<long>(dut.acc_emax) != g.emax) fail("acc_emax", ns, static_cast<long>(dut.acc_emax), g.emax);
                    if (dut.acc_opened != g.opened % kPosMod) fail("acc_opened", ns, dut.acc_opened, g.opened % kPosMod);
                    ++accs;
                    // The decision's grid, from this cycle on.
                    rd_pos = g.imax;
                    rd_left = W + 1;
                }
                ++ss;
            }
            ++ns;
        }
        if (rd_left > 0) {
            dut.ring_re = 1;
            dut.ring_addr = static_cast<uint32_t>(rd_pos % kRing);
            rd_due_pos = rd_pos;
            ++rd_pos;
            --rd_left;
        }
    }

    void feed(long xr, long xi, long acq, size_t k) {
        fed.push_back(cycle + 1);
        dut.in_valid = 1;
        dut.in_re = static_cast<uint32_t>(xr) & ((1u << kXBits) - 1);
        dut.in_im = static_cast<uint32_t>(xi) & ((1u << kXBits) - 1);
        dut.in_clear = acq;
        tick();
        dut.in_valid = 0;
        dut.in_re = 0x7ff;  // garbage while idle
        dut.in_im = 0x800;
        dut.in_clear = 1;
        int idle = R - 1;
        if (k % 5 == 4) idle += 1;
        if (k % 13 == 12) idle += 2;
        for (int i = 0; i < idle; ++i) tick();
    }

    // A reset in the clocks in which the search takes an accepted sample: the segment up to
    // its first acceptance (the search is D samples behind), then a one-clock reset 19 … 24
    // clocks after that sample's edge, a different one each time — the correlator's output,
    // the square, the energy and the ring's write, the strobe, a decision in flight. Nothing
    // is compared here; what survives the reset lands in the run that follows and breaks it.
    void reset_in_flight(const Segment& s) {
        size_t a = 0;
        while (a < s.s.size() && !s.s[a].has_acc) ++a;
        if (a + D >= s.s.size()) return;
        const long phase = 19 + in_flight_resets % 6;
        ++in_flight_resets;
        junk = true;
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        dut.rst_n = 1;
        fed.clear();
        for (size_t k = 0; k <= a + D; ++k) feed(s.s[k].xr, s.s[k].xi, s.s[k].acq, k);
        while (cycle < fed.back() + phase) tick();
        junk = false;
    }

    // A reset after `acquired` and before the search's own sample of the result (D samples
    // later): the segment up to its first acquisition, 6, 13 or 20 clocks, a one-clock
    // reset. A strobe remembered across the reset closes a search of the run that follows
    // at its first failed candidate.
    void reset_after_acquired(const Segment& s) {
        size_t q = 0;
        while (q < s.s.size() && !s.s[q].acq) ++q;
        if (q >= s.s.size()) return;
        const int wait = 6 + 7 * static_cast<int>(acquired_resets % 3);
        ++acquired_resets;
        junk = true;
        dut.rst_n = 0;
        dut.in_valid = 0;
        tick();
        dut.rst_n = 1;
        fed.clear();
        for (size_t k = 0; k <= q; ++k) feed(s.s[k].xr, s.s[k].xi, s.s[k].acq, k);
        for (int i = 0; i < wait; ++i) tick();
        junk = false;
    }

    void run(const Segment& s) {
        seg = &s;
        reset_in_flight(s);
        reset_after_acquired(s);
        fed.clear();
        ne = ns = 0;
        rd_left = 0;
        rd_due_pos = -1;
        dut.rst_n = 0;
        dut.in_valid = 0;
        dut.ring_re = 0;
        tick();
        dut.rst_n = 1;
        const size_t n = s.s.size();
        for (size_t k = 0; k < n; ++k) feed(s.s[k].xr, s.s[k].xi, s.s[k].acq, k);
        for (int k = 0; k < D; ++k) feed(0, 0, 0, n + k);   // flush
        for (int i = 0; i < 40; ++i) tick();
        if (ne != s.e_at.size() || ns != n) {
            std::fprintf(stderr, "%s: outputs %zu energies, %zu sample strobes; expected %zu, %zu\n",
                         s.label.c_str(), ne, ns, s.e_at.size(), n);
            ++mismatches;
        }
        // Junk in flight when the next segment's reset arrives.
        junk = true;
        for (int k = 0; k < 3; ++k) {
            dut.in_valid = 1;
            dut.in_re = 0x7ff;
            dut.in_im = 0x800;
            dut.in_clear = 0;
            tick();
            dut.in_valid = 0;
            for (int i = 0; i < R - 1; ++i) tick();
        }
        junk = false;
        quiet = true;
        dut.in_valid = 0;
        dut.rst_n = 0;
        tick();
        tick();
        dut.rst_n = 1;
        for (int i = 0; i < 60; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0);
            after_reset = 0;
        }
    }
};

bool field(std::istringstream& f, long& v, bool& present) {
    std::string t;
    if (!(f >> t)) return false;
    present = t != "-";
    v = present ? std::stol(t) : 0;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 5) {
        std::cerr << "usage: sim_search <vectors.txt> <R> <D> <W>\n";
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
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string a;
        if (!(f >> a)) continue;
        if (a == "reset") {
            segments.push_back({});
            if (!(f >> segments.back().label >> segments.back().w)) {
                std::cerr << "malformed reset line: " << line << "\n";
                return 2;
            }
            continue;
        }
        Sample x{};
        x.xr = std::stol(a);
        bool p = false, pi = false, pe = false, po = false;
        if (segments.empty() || !(f >> x.xi >> x.acq >> x.event >> x.searching) ||
            !field(f, x.e, x.has_e) || !field(f, x.imax, pi) || !field(f, x.emax, pe) ||
            !field(f, x.opened, po) || pi != pe || pi != po) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        (void)p;
        x.has_acc = pi;
        if (x.has_e) segments.back().e_at.push_back(segments.back().s.size());
        segments.back().s.push_back(x);
    }
    long samples = 0, energies = 0, accepted = 0, acqs = 0, opens = 0, closes = 0, run_segments = 0;
    for (const auto& s : segments) {
        if (s.w != g.W) continue;
        ++run_segments;
        samples += static_cast<long>(s.s.size());
        energies += static_cast<long>(s.e_at.size());
        long prev = 0;
        for (const auto& x : s.s) {
            accepted += x.has_acc;
            acqs += x.acq;
            opens += x.searching && !prev;
            closes += !x.searching && prev;
            prev = x.searching;
        }
        g.run(s);
    }
    std::printf("search G3 (R = %d, D = %d, W = %d): %ld segments, %ld samples (%ld energies, %ld searches "
                "opened, %ld closed, %ld acceptances, %ld acquisitions, %ld ring words read, %ld + %ld resets "
                "in flight), %ld mismatches\n",
                g.R, g.D, g.W, run_segments, samples, energies, opens, closes, accepted, acqs,
                g.ring_reads, g.in_flight_resets, g.acquired_resets, g.mismatches);
    if (run_segments == 0 || accepted == 0 || acqs == 0 || closes == 0 || g.in_flight_resets == 0 ||
        g.acquired_resets == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.es == energies && g.ss == samples && g.accs == accepted &&
                   g.ring_reads == accepted * (g.W + 1)
               ? 0
               : 1;
}
