// Gate G3 for the hand-off (hw/rtl/rx_handoff.sv) behind the front's top level
// (hw/rtl/rx_front.sv), through hw/tb/handoff_chain.sv: only the ADC's samples, the reader's
// ready and the decoder's verdict strobes go in. Required, against
// radio_phy::rx_handoff::run_handoff a cycle at a time (which rvec holds to
// RxHandoffFixed::push before it writes the vectors):
//   - a beat taken (out_valid and out_ready) in exactly the cycles of the vectors' beats and
//     in no other, with the beat's fields: a sample's I and Q; on a stream's first sample
//     also out_first, the CFO word and the tag; a mark's out_end and kind;
//   - hold in every cycle: the vectors' value after the tick of the latest sample (the tick
//     is kTick cycles after the sample, and hold is a register: it shows from the next);
//   - the offset (a probe the wrapper brings out of the block): registered in exactly the
//     cycle the schedule says, not the cycle before, with the model's value.
// A segment with a digest lists no beats: their count and a digest over cycle and fields.
//
// usage: sim_handoff <vectors.txt> <R> <D> <W>   (segments of other R or W are skipped; D
// is the build's D_SAMPLES, for the run line only)
//
// Cycles count from the release of reset. Sample n is at the pins in cycle
// X(n) = X(n − 1) + R + idle(n), X(−1) = 0, idle(n) one more clock after every 5th sample
// and two after every 13th (rvec's tb_idle); idle inputs carry garbage. The reader is ready
// by the segment's pattern of the cycle (rvec's handoff_ready) but in the intervals the
// vectors say it is away. The first segment of a run is played straight after the power-on
// reset (registers without a reset and the memories start from random values). Before each
// other one the same segment is played three times part of the way without comparing and
// reset in flight for one clock, and after each of those resets the whole segment is
// played and compared: five cycles after the sample of its first declaration
// (the acquisition waits for its tick); some way into its first stream; and further, with
// a reader that takes nothing (a mark waiting, the buffer full). After a segment, a
// reset: nothing may come out. The RTL's own checks ($error) abort the run. Exit code
// 0 = passed.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Vhandoff_chain.h"
#include "verilated.h"

namespace {

constexpr int kXBits = 12;
constexpr int kZBits = 13;
constexpr int kPBits = 22;
constexpr long kTick = 6;   // cycles from a sample to its tick

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

bool ready_at(int pattern, long cycle) {
    const uint32_t c = static_cast<uint32_t>(cycle);
    const uint32_t hash = ((c * 0x9e3779b9u + (c >> 3)) >> 7) & 0xff;
    switch (pattern) {
        case 0: return true;
        case 1: return c % 2 == 0;
        case 2: return hash < 128;
        case 3: return hash < 24;
        case 4: return (c / 700) % 3 != 1;
        default: return c % 8 == 7;
    }
}

uint64_t digest(uint64_t h, long cycle, const long long f[5]) {
    h = (h ^ static_cast<uint64_t>(cycle)) * 0x00000100000001b3ull;
    for (int i = 0; i < 5; ++i) h = (h ^ static_cast<uint64_t>(f[i])) * 0x00000100000001b3ull;
    return h;
}

struct Sample {
    long xr = 0, xi = 0;
    int hold = 0;
};

struct Strobe {
    int tag = 0, failed = 0, symbols = 0, patient = 0;
};

struct Beat {
    long cycle = 0;
    long long f[5] = {0, 0, 0, 0, 0};
};

struct Segment {
    std::string label;
    int w = 0, r = 0, pattern = 0, digest = 0;
    std::vector<Sample> s;
    std::map<long, Strobe> strobes;
    std::map<long, std::pair<long, long>> offsets;   // the cycle an offset is registered in
    std::vector<std::pair<long, long>> away;
    std::vector<Beat> beats;
    long end = 0, count = 0;
    uint64_t hash = 0;
};

struct Gate {
    Vhandoff_chain dut;
    long mismatches = 0, beats = 0, marks = 0, firsts = 0, strobes = 0, hold_cycles = 0, cycles = 0;
    long offsets = 0, away_cycles = 0;
    long resets_in_flight = 0, after_reset = 0;
    bool first_done = false;
    const Segment* seg = nullptr;

    void fail(const char* what, long cycle, long long r, long long g) {
        if (mismatches < 16)
            std::fprintf(stderr, "%s %s, cycle %ld: rtl=%lld golden=%lld\n", seg->label.c_str(), what, cycle, r, g);
        ++mismatches;
    }

    void edge() {
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
    }

    void reset(int clocks) {
        dut.rst_n = 0;
        dut.in_valid = 0;
        dut.sig_valid = 0;
        dut.out_ready = 0;
        for (int i = 0; i < clocks; ++i) {
            dut.eval();
            edge();
        }
        dut.rst_n = 1;
    }

    // Plays the segment's first `samples` samples from the release of reset and `more`
    // cycles after the last of them (the whole segment: to its end); compares when `check`;
    // a reader that takes nothing when `deaf`. Returns the last cycle played.
    long play(const Segment& s, size_t samples, bool check, long more = 3, bool deaf = false) {
        size_t next = 0, beat = 0;
        long x_next = s.r, last_x = -1;   // X(0) = R (no idle before the first sample)
        int hold = 0;
        long hold_from = -1;              // the cycle the tick of the latest sample is in
        int hold_next = 0;
        uint64_t h = 0xcbf29ce484222325ull;
        long taken = 0;
        const long stop = samples == s.s.size() ? s.end : -1;
        long c = 1;
        for (;; ++c) {
            if (stop >= 0 ? c > stop : (next >= samples && c > last_x + more)) break;
            // inputs of cycle c
            if (next < samples && c == x_next) {
                dut.in_valid = 1;
                dut.in_re = static_cast<uint32_t>(s.s[next].xr) & ((1u << kXBits) - 1);
                dut.in_im = static_cast<uint32_t>(s.s[next].xi) & ((1u << kXBits) - 1);
                hold_from = c + kTick;
                hold_next = s.s[next].hold;
                last_x = c;
                ++next;
                int idle = 0;
                if (next % 5 == 0) idle += 1;      // after every 5th sample
                if (next % 13 == 0) idle += 2;     // after every 13th
                x_next = c + s.r + idle;
            } else {
                dut.in_valid = 0;
                dut.in_re = 0x7ff;  // garbage while idle
                dut.in_im = 0x800;
            }
            if (c == hold_from + 1) hold = hold_next;
            bool ready = !deaf && ready_at(s.pattern, c);
            for (const auto& a : s.away)
                if (a.first <= c && c <= a.second) {
                    ready = false;
                    if (check) ++away_cycles;
                }
            dut.out_ready = ready;
            const auto v = s.strobes.find(c);
            if (v != s.strobes.end()) {
                dut.sig_valid = 1;
                dut.sig_tag = v->second.tag;
                dut.sig_fail = v->second.failed;
                dut.sig_symbols = v->second.symbols;
                dut.sig_patient = v->second.patient;
                if (check) ++strobes;
            } else {
                dut.sig_valid = 0;
                dut.sig_tag = 0xa5;  // garbage without a strobe
                dut.sig_fail = 1;
                dut.sig_symbols = 0x555;
                dut.sig_patient = 1;
            }
            dut.eval();
            if (check) {
                ++cycles;
                hold_cycles += hold;
                if (dut.hold != hold) fail("hold", c, dut.hold, hold);
                const auto o = s.offsets.find(c);
                if (o != s.offsets.end()) {
                    ++offsets;
                    if (!dut.p_dc_v) fail("offset not registered in its cycle", c, 0, 1);
                    if (sext(dut.p_dc_re, kXBits) != o->second.first)
                        fail("offset's I", c, sext(dut.p_dc_re, kXBits), o->second.first);
                    if (sext(dut.p_dc_im, kXBits) != o->second.second)
                        fail("offset's Q", c, sext(dut.p_dc_im, kXBits), o->second.second);
                }
                if (s.offsets.count(c + 1) && dut.p_dc_v) fail("offset registered a cycle early", c, 1, 0);
                if (dut.out_valid && dut.out_ready) {
                    long long f[5] = {0, 0, 0, 0, 0};
                    if (dut.out_end) {
                        f[0] = 2;
                        f[1] = dut.out_kind;
                    } else {
                        f[0] = dut.out_first ? 1 : 0;
                        f[1] = sext(dut.out_re, kZBits);
                        f[2] = sext(dut.out_im, kZBits);
                        if (dut.out_first) {
                            f[3] = sext(dut.out_cfo, kPBits);
                            f[4] = dut.out_tag;
                        }
                    }
                    h = digest(h, c, f);
                    ++taken;
                    ++beats;
                    marks += f[0] == 2;
                    firsts += f[0] == 1;
                    if (!s.digest) {
                        if (beat >= s.beats.size() || s.beats[beat].cycle != c) {
                            fail("a beat the model does not hand on", c, f[0], -1);
                        } else {
                            const Beat& g = s.beats[beat];
                            static const char* names[5] = {"beat's kind", "beat's I (or the mark's kind)", "beat's Q",
                                                           "first beat's CFO word", "first beat's tag"};
                            for (int i = 0; i < 5; ++i)
                                if (f[i] != g.f[i]) fail(names[i], c, f[i], g.f[i]);
                            ++beat;
                        }
                    }
                } else if (!s.digest && beat < s.beats.size() && s.beats[beat].cycle == c) {
                    fail("no beat where the model hands one on", c, dut.out_valid, 1);
                    ++beat;
                }
            }
            edge();
        }
        if (check) {
            if (taken != s.count) fail("beats in the segment", c, taken, s.count);
            if (h != s.hash) fail("digest of the beats", c, static_cast<long long>(h), static_cast<long long>(s.hash));
        }
        return c;
    }

    void run(const Segment& s, int index) {
        seg = &s;
        reset(2);
        if (!first_done) {
            first_done = true;
        } else {
            // The sample of the segment's first declaration: the first the hold is up after.
            size_t d = 0;
            while (d < s.s.size() && !s.s[d].hold) ++d;
            if (d < s.s.size()) {
                // acq_valid comes four cycles after the sample: in the fifth it waits.
                play(s, d + 1, false, 4);
                ++resets_in_flight;
                reset(1);
                play(s, s.s.size(), true);
                reset(1);
                // Into the stream: the estimate, the replay catching up, or at the antenna.
                play(s, std::min(s.s.size(), d + 3 + static_cast<size_t>(index * 37 % 400)), false, index % 8);
                ++resets_in_flight;
                reset(1);
                play(s, s.s.size(), true);
                reset(1);
                // A reader that takes nothing: a full buffer, and past 512 samples a mark.
                play(s, std::min(s.s.size(), d + 30 + static_cast<size_t>(index * 211 % 900)), false, 3, true);
                ++resets_in_flight;
                reset(1);
            }
        }
        play(s, s.s.size(), true);
        // After a reset nothing comes out.
        reset(2);
        dut.out_ready = 1;
        for (int i = 0; i < 60; ++i) {
            dut.eval();
            after_reset += dut.out_valid + dut.hold;
            edge();
        }
        if (after_reset != 0) {
            fail("outputs after reset", 0, after_reset, 0);
            after_reset = 0;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 5) {
        std::cerr << "usage: sim_handoff <vectors.txt> <R> <D> <W>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    const int r = std::atoi(argv[2]);
    const int d = std::atoi(argv[3]);
    const int w = std::atoi(argv[4]);
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
            Segment& s = segments.back();
            if (!(f >> s.label >> s.w >> s.r >> s.pattern >> s.digest)) return bad();
            continue;
        }
        if (segments.empty()) return bad();
        Segment& s = segments.back();
        if (a == "x") {
            Sample x;
            if (!(f >> x.xr >> x.xi >> x.hold)) return bad();
            s.s.push_back(x);
        } else if (a == "r") {
            long more = 0;
            if (s.s.empty() || !(f >> more)) return bad();
            s.s.insert(s.s.end(), static_cast<size_t>(more), s.s.back());
        } else if (a == "A") {
            long from = 0, to = 0;
            if (!(f >> from >> to)) return bad();
            s.away.emplace_back(from, to);
        } else if (a == "D") {
            long cycle = 0, re = 0, im = 0;
            if (!(f >> cycle >> re >> im)) return bad();
            s.offsets[cycle] = {re, im};
        } else if (a == "V") {
            long cycle = 0;
            Strobe v;
            if (!(f >> cycle >> v.tag >> v.failed >> v.symbols >> v.patient)) return bad();
            if (!s.strobes.emplace(cycle, v).second) return bad();
        } else if (a == "B") {
            Beat b;
            if (!(f >> b.cycle >> b.f[0] >> b.f[1])) return bad();
            if (b.f[0] != 2 && !(f >> b.f[2])) return bad();
            if (b.f[0] == 1 && !(f >> b.f[3] >> b.f[4])) return bad();
            s.beats.push_back(b);
        } else if (a == "end") {
            if (!(f >> s.end >> s.count >> s.hash)) return bad();
        } else {
            return bad();
        }
    }
    Gate g;
    long run_segments = 0, samples = 0;
    int index = 0;
    const Segment* only = nullptr;
    for (const auto& s : segments) {
        ++index;
        if (s.w != w || s.r != r) continue;
        ++run_segments;
        samples += static_cast<long>(s.s.size());
        g.run(s, index);
        only = &s;
    }
    // A run of one segment has had no reset in flight: the segment again, with them.
    if (run_segments == 1) g.run(*only, index + 3);
    std::printf("handoff G3 (R = %d, D = %d, W = %d): %ld segments, %ld samples, %ld cycles, %ld beats "
                "(%ld streams, %ld marks), %ld offsets, %ld verdict strobes, hold up in %ld cycles, the reader "
                "away in %ld, %ld resets in flight, %ld mismatches\n",
                r, d, w, run_segments, samples, g.cycles, g.beats, g.firsts, g.marks, g.offsets, g.strobes,
                g.hold_cycles, g.away_cycles, g.resets_in_flight, g.mismatches);
    if (run_segments == 0 || g.beats == 0 || g.firsts == 0 || g.marks == 0 || g.hold_cycles == 0 || g.offsets == 0) {
        std::cerr << "nothing compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
