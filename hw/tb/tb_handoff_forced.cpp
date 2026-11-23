// Gate for a declaration that follows another closely (hw/rtl/rx_handoff.sv alone, through
// hw/tb/handoff_alone.sv), which the real front does not give and no vector of the gate
// `handoff` has: a declaration k samples before another, k = 1 … 80 (the first one's
// estimate reading, its dividers running, its replay catching up, its buffer full).
//
// The reference is the RTL's own run with the second declaration alone, which the gate
// `handoff` holds to the model: by the schedule (radio_phy::rx_handoff) a declaration drops
// the open stream and starts afresh from the history, so the second stream must not depend
// on the first. Required for every k and three readers (always ready; taking nothing until
// the second declaration's tick; ready one cycle in three):
//   - the second stream's offset, the cycle it is registered in, every sample beat with its
//     cycle and fields (the tag one more than alone), and their number;
//   - exactly one mark, an acquisition's (the first stream's abort);
//   - the hold up from the first declaration's tick on.
// (The independent review's bench of 2026-11-23, which found the first version adding a
// word of the abandoned estimate to the new sums for k up to 16.)
//
// usage: sim_handoff_forced
//
// Samples every 8 cycles, random about an offset; acq_valid four cycles after its sample's
// cycle, as rx_front gives it. Registers without a reset start from random values. The
// RTL's own checks ($error) fail the case. Exit code 0 = passed.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vhandoff_alone.h"
#include "verilated.h"

namespace {

constexpr int kR = 8;
constexpr int kSecond = 700;   // the sample of the second declaration
constexpr int kTotal = 1400;
constexpr long kTick = 6;

long long sext(uint64_t v, int bits) {
    const uint64_t m = 1ull << (bits - 1);
    v &= (1ull << bits) - 1;
    return static_cast<long long>(v ^ m) - static_cast<long long>(m);
}

struct Beat {
    long cycle;
    int end, kind, first, tag;
    long long re, im, cfo;
};

struct Result {
    std::vector<Beat> beats;   // after the second declaration's tick
    long dc_cycle = -1, hold_low = 0;
    long long dc_re = 0, dc_im = 0;
    int max_fill = 0;
    bool stopped = false;
};

uint32_t rnd(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

// k: the first declaration at sample kSecond − k (0: none).
Result run(int k, int reader, uint32_t seed) {
    Vhandoff_alone d;
    Result res;
    std::vector<int> xr(kTotal), xi(kTotal);
    uint32_t s = seed;
    for (int n = 0; n < kTotal; ++n) {
        xr[n] = static_cast<int>(rnd(s) % 1201) - 600 + 100;
        xi[n] = static_cast<int>(rnd(s) % 1201) - 600 - 70;
    }
    d.rst_n = 0;
    d.in_valid = 0;
    d.acq_valid = 0;
    d.out_ready = 0;
    for (int i = 0; i < 2; ++i) {
        d.clk = 0;
        d.eval();
        d.clk = 1;
        d.eval();
    }
    d.rst_n = 1;
    const long tick2 = static_cast<long>(kSecond + 1) * kR + kTick;
    const long tick1 = tick2 - static_cast<long>(k) * kR;
    bool seen_dc = false;
    for (long c = 1; c <= static_cast<long>(kTotal) * kR + 20; ++c) {
        d.clk = 0;
        const long n = c / kR - 1;
        d.in_valid = (c % kR == 0 && n >= 0 && n < kTotal);
        d.in_re = d.in_valid ? (xr[n] & 0xfff) : 0x7ff;
        d.in_im = d.in_valid ? (xi[n] & 0xfff) : 0x800;
        d.acq_valid = 0;
        d.acq_fft = 0x155;   // garbage without the strobe
        d.acq_cfo = 0x2aaaaa;
        const long m = (c - 4) / kR - 1;
        if ((c - 4) % kR == 0) {
            if (m == kSecond) {
                d.acq_valid = 1;
                d.acq_fft = (kSecond - 200) & 511;
                d.acq_cfo = 12345;
            } else if (k > 0 && m == kSecond - k) {
                d.acq_valid = 1;
                d.acq_fft = (kSecond - k - 250) & 511;
                d.acq_cfo = static_cast<uint32_t>(-40000) & 0x3fffff;
            }
        }
        bool ready = true;
        if (reader == 1) ready = c > tick2;
        if (reader == 2) ready = c % 3 == 0;
        d.out_ready = ready;
        d.eval();
        if (d.p_fcnt > res.max_fill) res.max_fill = d.p_fcnt;
        if (c > (k > 0 ? tick1 : tick2) && !d.hold) ++res.hold_low;
        if (c > tick2) {
            if (!seen_dc && d.p_dc_v) {
                seen_dc = true;
                res.dc_cycle = c - tick2;
                res.dc_re = sext(d.p_dc_re, 12);
                res.dc_im = sext(d.p_dc_im, 12);
            }
            if (d.out_valid && d.out_ready) {
                Beat b;
                b.cycle = c - tick2;
                b.end = d.out_end;
                b.kind = d.out_kind;
                b.first = d.out_first;
                b.re = sext(d.out_re, 13);
                b.im = sext(d.out_im, 13);
                b.cfo = sext(d.out_cfo, 22);
                b.tag = d.out_tag;
                res.beats.push_back(b);
            }
        }
        d.clk = 1;
        d.eval();
        if (Verilated::gotError() || Verilated::gotFinish()) {
            res.stopped = true;
            break;
        }
    }
    return res;
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    int bad = 0, cases = 0;
    long beats = 0;
    for (int reader = 0; reader < 3; ++reader) {
        const Result a = run(0, reader, 0x1234567u);
        if (a.beats.size() < 600 || a.dc_cycle != 147 || a.stopped || (a.dc_re == 0 && a.dc_im == 0)) {
            std::fprintf(stderr, "reader %d: the single declaration is no reference (%zu beats, offset at +%ld)\n",
                         reader, a.beats.size(), a.dc_cycle);
            return 1;
        }
        for (int k = 1; k <= 80; ++k) {
            const Result b = run(k, reader, 0x1234567u);
            ++cases;
            std::vector<Beat> samples;
            int marks = 0, mark_kind = -1;
            for (const auto& x : b.beats) {
                if (x.end) {
                    ++marks;
                    mark_kind = x.kind;
                } else {
                    samples.push_back(x);
                }
            }
            long diff = 0, first_diff = -1;
            const size_t common = samples.size() < a.beats.size() ? samples.size() : a.beats.size();
            for (size_t i = 0; i < common; ++i) {
                const Beat& p = a.beats[i];
                const Beat& q = samples[i];
                const bool same = p.cycle == q.cycle && p.re == q.re && p.im == q.im && p.first == q.first &&
                                  (!p.first || (p.cfo == q.cfo && q.tag == p.tag + 1));
                if (!same) {
                    if (first_diff < 0) first_diff = static_cast<long>(i);
                    ++diff;
                }
            }
            beats += static_cast<long>(common);
            const bool dc_same = a.dc_re == b.dc_re && a.dc_im == b.dc_im && a.dc_cycle == b.dc_cycle;
            const bool ok = !b.stopped && dc_same && diff == 0 && samples.size() == a.beats.size() && marks == 1 &&
                            mark_kind == 2 && b.hold_low == 0 && b.max_fill <= 7;
            if (!ok) {
                ++bad;
                if (bad <= 16)
                    std::fprintf(stderr,
                                 "reader %d, k = %d: offset (%lld, %lld) at +%ld against (%lld, %lld) at +%ld; "
                                 "%zu sample beats against %zu, %ld differ (the first: beat %ld); %d marks (kind %d); "
                                 "hold low in %ld cycles; buffer's peak %d%s\n",
                                 reader, k, b.dc_re, b.dc_im, b.dc_cycle, a.dc_re, a.dc_im, a.dc_cycle, samples.size(),
                                 a.beats.size(), diff, first_diff, marks, mark_kind, b.hold_low, b.max_fill,
                                 b.stopped ? "; stopped by a simulation check" : "");
            }
        }
    }
    std::printf("handoff_forced: %d cases (a declaration 1 to 80 samples before another, three readers), "
                "%ld sample beats compared, %d cases differ from the declaration alone\n",
                cases, beats, bad);
    return cases == 240 && bad == 0 ? 0 : 1;
}
