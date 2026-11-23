// G3 gate for the LTF search alone (hw/rtl/rx_search.sv as the top module): replays
// hw/vectors/search_alone.txt and requires, for every sample, the energy completed at it,
// whether a search is open after it, and the candidate accepted at it (the sample, the
// maximum's position, its ring address and energy, the opening event) to match
// radio_phy::rx_front_fixed::SearchAloneFixed — the search of RxFrontFixed at its correlator's
// output — bit for bit. After every acceptance it reads the energy ring as the decision will
// (imax … imax + W, oldest first, one a cycle from the strobe's cycle on).
//
// Why a bench of the block alone: the chain in front of it reaches 32 of the energy's 44 bits
// (gate `search`); here the correlator's port is driven with full-scale values the chain does
// not produce, so the upper 12 bits, the squares at full width and the comparison at full
// width are tested (the founder's decision of 2026-11-23, docs/gates/rx_front.md).
//
// usage: sim_search_alone <vectors.txt> <R> <W> <L> <PATH_DELAY>   R = clocks per sample (at
//        least 8: ltf_corr); W, L, PATH_DELAY = the build's parameters. Only the segments of
//        that (W, L) are run.
//
// Timeline (edges counted by the testbench): sample k sampled at edge F (s_valid, s_event);
// its correlator output (from k = 127 on) at edge F + PATH_DELAY; its energy out at
// F + PATH_DELAY + 1 (e_valid); its strobe at F + PATH_DELAY + 2 (out_valid, searching,
// acc_valid with the fields). Samples come every R clocks, one more idle clock after every
// 5th and two after every 13th; idle inputs carry garbage. `acquired` for a candidate whose
// decision acquires is driven, in turn, in the cycle of acc_valid (the earliest) and in the
// cycle before the search takes sample k + L (the latest legal one). Before each segment the
// block is reset in the clocks around its first acceptance (the run up to it, a one-clock
// reset at one of four phases: the square, the energy, the strobe, a decision in flight with
// `acquired` remembered); three junk samples precede each segment's end reset. Nothing of
// these may come out or disturb the next run. The RTL's own checks ($error) abort the run.
// Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vrx_search.h"
#include "verilated.h"

namespace {

constexpr int kCBits = 20;      // CORR_BITS
constexpr long kPosMod = 512;   // sample numbers are handed over modulo the history's depth
constexpr long kRing = 81;      // words of the energy ring
constexpr long kFirstCorr = 127;

struct Sample {
    long event = 0, cr = 0, ci = 0, acquire = 0, acquired = 0, searching = 0;
    bool has_c = false, has_e = false, has_acc = false;
    long long e = 0, emax = 0;
    long imax = 0, opened = 0;
};

struct Segment {
    std::string label;
    int w = 0, l = 0;
    std::vector<Sample> s;
    std::vector<size_t> e_at;   // the samples that complete an energy, in order of position
};

struct Pend {
    long edge;
    long cr, ci;
};

struct Gate {
    Vrx_search dut;
    int R = 8, W = 12, L = 64, PD = 19;
    long cycle = 0, mismatches = 0;
    long es = 0, ss = 0, accs = 0, ring_reads = 0, early = 0, late = 0, resets = 0;
    size_t ne = 0, ns = 0;
    const Segment* seg = nullptr;
    std::vector<long> fed;            // edge of each fed sample
    std::deque<Pend> corr;            // correlator outputs due, by edge
    std::vector<long> late_of;        // per sample: the sample whose late `acquired` it carries
    bool junk = false, quiet = false;
    long after_reset = 0;
    bool acq_next = false;            // drive `acquired` at the next edge
    long rd_pos = 0, rd_left = 0, rd_due_pos = -1;

    void fail(const char* what, size_t idx, long long r, long long g) {
        if (mismatches < 12)
            std::fprintf(stderr, "%s %s %zu: rtl=%lld golden=%lld\n", seg ? seg->label.c_str() : "-", what, idx, r, g);
        ++mismatches;
    }

    // Inputs for the coming edge (cycle + 1), then the edge, then the outputs after it.
    void tick() {
        const long edge = cycle + 1;
        if (!corr.empty() && corr.front().edge == edge) {
            dut.c_valid = 1;
            dut.c_re = static_cast<uint32_t>(corr.front().cr) & ((1u << kCBits) - 1);
            dut.c_im = static_cast<uint32_t>(corr.front().ci) & ((1u << kCBits) - 1);
            corr.pop_front();
        } else {
            dut.c_valid = 0;
            dut.c_re = 0x80000;   // garbage while idle
            dut.c_im = 0x7ffff;
        }
        // The latest legal `acquired`: in the cycle before the search takes sample q + L,
        // i.e. sampled at edge F(q + L) + PD + 1.
        bool late_now = false;
        for (size_t k = 0; k < late_of.size() && k < fed.size(); ++k)
            if (late_of[k] >= 0 && fed[k] + PD + 1 == edge) late_now = true;
        dut.acquired = acq_next || late_now;
        acq_next = false;

        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        dut.s_valid = 0;
        dut.s_event = 1;   // garbage while idle
        dut.ring_re = 0;
        if (quiet) {
            after_reset += dut.e_valid + dut.out_valid + dut.acc_valid + dut.searching;
            return;
        }
        if (junk) {
            // Not compared; a decision that acquires is told so at once (a remembered strobe
            // across the reset is one of the cases).
            if (dut.acc_valid && ns < seg->s.size() && seg->s[ns].has_acc && seg->s[ns].acquire) acq_next = true;
            ns += dut.out_valid;
            return;
        }
        const size_t n = seg->s.size();
        if (rd_due_pos >= 0) {
            const long long g = seg->s[seg->e_at[static_cast<size_t>(rd_due_pos)]].e;
            if (static_cast<long long>(dut.ring_data) != g)
                fail("ring word at position", static_cast<size_t>(rd_due_pos), static_cast<long long>(dut.ring_data), g);
            ++ring_reads;
            rd_due_pos = -1;
        }
        if (dut.e_valid) {
            if (ne >= seg->e_at.size()) {
                fail("unexpected energy", ne, static_cast<long long>(dut.e), 0);
            } else {
                const size_t k = seg->e_at[ne];
                const long lat = cycle - fed[k];
                if (lat != PD + 1) fail("energy latency", k, lat, PD + 1);
                if (static_cast<long long>(dut.e) != seg->s[k].e) fail("energy", k, static_cast<long long>(dut.e), seg->s[k].e);
                ++es;
            }
            ++ne;
        }
        if (dut.acc_valid && !dut.out_valid) fail("acceptance without a sample strobe", ns, 1, 0);
        if (dut.out_valid) {
            if (ns >= n) {
                fail("unexpected sample strobe", ns, 0, 0);
            } else {
                const Sample& g = seg->s[ns];
                const long lat = cycle - fed[ns];
                if (lat != PD + 2) fail("strobe latency", ns, lat, PD + 2);
                if (dut.searching != g.searching) fail("searching", ns, dut.searching, g.searching);
                if (dut.acc_valid != g.has_acc) fail("acceptance", ns, dut.acc_valid, g.has_acc);
                if (dut.acc_valid && g.has_acc) {
                    if (dut.acc_n != static_cast<long>(ns) % kPosMod) fail("acc_n", ns, dut.acc_n, static_cast<long>(ns) % kPosMod);
                    if (dut.acc_imax != g.imax % kPosMod) fail("acc_imax", ns, dut.acc_imax, g.imax % kPosMod);
                    if (dut.acc_lapped != (g.imax >= kPosMod)) fail("acc_lapped", ns, dut.acc_lapped, g.imax >= kPosMod);
                    if (dut.acc_ring != g.imax % kRing) fail("acc_ring", ns, dut.acc_ring, g.imax % kRing);
                    if (static_cast<long long>(dut.acc_emax) != g.emax) fail("acc_emax", ns, static_cast<long long>(dut.acc_emax), g.emax);
                    if (dut.acc_opened != g.opened % kPosMod) fail("acc_opened", ns, dut.acc_opened, g.opened % kPosMod);
                    ++accs;
                    rd_pos = g.imax;
                    rd_left = W + 1;
                    if (g.acquire) {
                        const size_t q = ns + static_cast<size_t>(L);
                        if ((accs % 2 == 0) && q < n) {
                            late_of[q] = static_cast<long>(ns);
                            ++late;
                        } else {
                            acq_next = true;
                            ++early;
                        }
                    }
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

    void feed(const Sample& x, size_t k) {
        const long edge = cycle + 1;
        fed.push_back(edge);
        if (static_cast<long>(k) >= kFirstCorr) corr.push_back({edge + PD, x.cr, x.ci});
        dut.s_valid = 1;
        dut.s_event = static_cast<uint8_t>(x.event);
        tick();
        int idle = R - 1;
        if (k % 5 == 4) idle += 1;
        if (k % 13 == 12) idle += 2;
        for (int i = 0; i < idle; ++i) tick();
    }

    void reset_pulse(int clocks) {
        dut.rst_n = 0;
        dut.s_valid = 0;
        corr.clear();
        acq_next = false;
        for (int i = 0; i < clocks; ++i) tick();
        dut.rst_n = 1;
    }

    void start(const Segment& s) {
        seg = &s;
        fed.clear();
        corr.clear();
        late_of.assign(s.s.size(), -1);
        ne = ns = 0;
        rd_left = 0;
        rd_due_pos = -1;
    }

    // The segment up to its first acceptance, then a one-clock reset at one of four phases
    // after that sample's edge F: PD + 1 (its energy in the register), PD + 2 (the strobe's
    // cycle), PD + 3 (the acceptance out, the ring being read), PD + 3 + 2R (a decision in
    // flight, `acquired` remembered if it acquires).
    void reset_in_flight(const Segment& s) {
        size_t a = 0;
        while (a < s.s.size() && !s.s[a].has_acc) ++a;
        if (a + 3 >= s.s.size()) return;
        const long phases[4] = {PD + 1, PD + 2, PD + 3, PD + 3 + 2L * R};
        const long phase = phases[resets % 4];
        ++resets;
        start(s);
        junk = true;
        reset_pulse(1);
        for (size_t k = 0; k <= a; ++k) feed(s.s[k], k);
        while (cycle < fed[a] + phase) tick();
        reset_pulse(1);
        junk = false;
    }

    void run(const Segment& s) {
        reset_in_flight(s);
        start(s);
        reset_pulse(1);
        const size_t n = s.s.size();
        for (size_t k = 0; k < n; ++k) feed(s.s[k], k);
        for (int i = 0; i < PD + 8; ++i) tick();
        if (ne != s.e_at.size() || ns != n) {
            std::fprintf(stderr, "%s: outputs %zu energies, %zu sample strobes; expected %zu, %zu\n",
                         s.label.c_str(), ne, ns, s.e_at.size(), n);
            ++mismatches;
        }
        // Junk in flight when the next segment's reset arrives.
        junk = true;
        Sample j{};
        j.event = 1;
        j.cr = -(1 << (kCBits - 1));
        j.ci = (1 << (kCBits - 1)) - 1;
        for (int k = 0; k < 3; ++k) feed(j, kFirstCorr + static_cast<size_t>(k));
        junk = false;
        quiet = true;
        reset_pulse(2);
        for (int i = 0; i < 60; ++i) tick();
        quiet = false;
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0);
            after_reset = 0;
        }
    }
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
    if (argc < 6) {
        std::cerr << "usage: sim_search_alone <vectors.txt> <R> <W> <L> <PATH_DELAY>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    Gate g;
    g.R = std::atoi(argv[2]);
    g.W = std::atoi(argv[3]);
    g.L = std::atoi(argv[4]);
    g.PD = std::atoi(argv[5]);
    if (g.R < 8 || g.W < 1 || g.L < 1 || g.PD < 1) {
        std::cerr << "bad R, W, L or PATH_DELAY\n";
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
            if (!(f >> segments.back().label >> segments.back().w >> segments.back().l)) {
                std::cerr << "malformed reset line: " << line << "\n";
                return 2;
            }
            continue;
        }
        Sample x{};
        x.event = std::stol(a);
        long long cr = 0, ci = 0, imax = 0, opened = 0;
        bool pc = false, pci = false, pi = false, pe = false, po = false;
        if (segments.empty() || !field(f, cr, pc) || !field(f, ci, pci) || pc != pci ||
            !(f >> x.acquire >> x.acquired >> x.searching) || !field(f, x.e, x.has_e) ||
            !field(f, imax, pi) || !field(f, x.emax, pe) || !field(f, opened, po) || pi != pe || pi != po) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        x.has_c = pc;
        x.cr = static_cast<long>(cr);
        x.ci = static_cast<long>(ci);
        x.imax = static_cast<long>(imax);
        x.opened = static_cast<long>(opened);
        x.has_acc = pi;
        Segment& s = segments.back();
        if (x.has_c != (static_cast<long>(s.s.size()) >= kFirstCorr)) {
            std::cerr << "correlator output on the wrong sample: " << line << "\n";
            return 2;
        }
        if (x.has_e) s.e_at.push_back(s.s.size());
        s.s.push_back(x);
    }
    long samples = 0, energies = 0, accepted = 0, acqs = 0, wide = 0, run_segments = 0;
    for (const auto& s : segments) {
        if (s.w != g.W || s.l != g.L) continue;
        ++run_segments;
        samples += static_cast<long>(s.s.size());
        energies += static_cast<long>(s.e_at.size());
        for (const auto& x : s.s) {
            accepted += x.has_acc;
            acqs += x.acquired;
            wide += x.has_e && x.e >= (1LL << 32);
        }
        g.run(s);
    }
    std::printf("search_alone G3 (R = %d, W = %d, L = %d, PATH_DELAY = %d): %ld segments, %ld samples "
                "(%ld energies, %ld above 32 bits; %ld acceptances, %ld acquisitions: %ld told early, "
                "%ld late; %ld ring words read; %ld resets in flight), %ld mismatches\n",
                g.R, g.W, g.L, g.PD, run_segments, samples, energies, wide, accepted, acqs, g.early, g.late,
                g.ring_reads, g.resets, g.mismatches);
    if (run_segments == 0 || accepted == 0 || acqs == 0 || wide == 0 || g.early == 0 || g.late == 0 ||
        g.resets == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 && g.es == energies && g.ss == samples && g.accs == accepted &&
                   g.ring_reads == accepted * (g.W + 1)
               ? 0
               : 1;
}
