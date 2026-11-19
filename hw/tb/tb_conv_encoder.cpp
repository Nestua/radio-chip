// G3 gate for the convolutional encoder (hw/rtl/conv_encoder.sv): replays
// hw/vectors/conv_encoder.txt and requires every A and B bit to match
// radio_phy::convolutional::ConvolutionalEncoder bit for bit.
//
// Each line is a frame from state 0: `clear` is pulsed before it. Bits are fed with an idle
// cycle after every 3rd and two after every 7th (idle inputs carry the inverted bit); each
// output pair must come exactly one clock edge after its input. Before every third frame the
// clear is replaced by a reset, with a bit in flight that must not come out. At the end a
// frame of ones is fed twice without a clear between: the second pass must differ from the
// run from state 0 (the state really carries over). Exit code 0 = gate passed.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vconv_encoder.h"
#include "verilated.h"

namespace {

struct Frame {
    std::string in, a, b;
};

struct Gate {
    Vconv_encoder dut;
    long cycle = 0, bits = 0, mismatches = 0, outputs = 0;

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        ++cycle;
        outputs += dut.out_valid;
    }

    void fail(const char* what, size_t frame, size_t i) {
        if (mismatches < 12) std::fprintf(stderr, "frame %zu bit %zu: %s\n", frame, i, what);
        ++mismatches;
    }

    // Feeds a frame; returns the number of bits whose outputs differ from the golden ones.
    long feed(const Frame& f, size_t index, bool check) {
        long diff = 0;
        for (size_t i = 0; i < f.in.size(); ++i) {
            dut.in_valid = 1;
            dut.in_bit = f.in[i] == '1';
            const long before = outputs;
            tick();
            const bool ok = dut.out_valid && outputs == before + 1 &&
                            dut.out_a == (f.a[i] == '1') && dut.out_b == (f.b[i] == '1');
            if (!ok) {
                ++diff;
                if (check) fail(dut.out_valid ? "wrong A/B" : "no output one edge after the input", index, i);
            }
            if (check) ++bits;
            dut.in_valid = 0;
            dut.in_bit = f.in[i] != '1';
            int idle = (i % 3 == 2) + 2 * (i % 7 == 6);
            for (int k = 0; k < idle; ++k) {
                const long b2 = outputs;
                tick();
                if (outputs != b2) fail("output on an idle cycle", index, i);
            }
        }
        return diff;
    }

    void clear() {
        dut.in_valid = 0;
        dut.clear = 1;
        tick();
        dut.clear = 0;
    }

    void reset_with_a_bit_in_flight() {
        dut.in_valid = 1;
        dut.in_bit = 1;
        dut.rst_n = 0;
        const long before = outputs;
        tick();
        dut.in_valid = 0;
        tick();
        dut.rst_n = 1;
        tick();
        if (outputs != before) {
            std::fprintf(stderr, "an output came out of a reset\n");
            ++mismatches;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_conv_encoder <vectors.txt>\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    std::vector<Frame> frames;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        Frame f;
        if (!(fields >> f.in >> f.a >> f.b) || f.in.size() != f.a.size() || f.in.size() != f.b.size()) {
            std::cerr << "malformed vector line: " << line << "\n";
            return 2;
        }
        frames.push_back(f);
    }
    Gate g;
    g.dut.rst_n = 0;
    g.dut.clear = 0;
    g.dut.in_valid = 0;
    g.tick();
    g.dut.rst_n = 1;
    for (size_t k = 0; k < frames.size(); ++k) {
        if (k % 3 == 2) g.reset_with_a_bit_in_flight();
        else g.clear();
        g.feed(frames[k], k, true);
    }
    // Without a clear the state carries over: a frame that ends in ones, fed again at once,
    // must not match its run from state 0.
    long carried = 0;
    for (size_t k = 0; k < frames.size(); ++k) {
        if (frames[k].in.size() >= 6 && frames[k].in.substr(frames[k].in.size() - 6) == "111111") {
            g.clear();
            g.feed(frames[k], k, true);
            carried = g.feed(frames[k], k, false);
            break;
        }
    }
    if (carried == 0) {
        std::fprintf(stderr, "a frame fed without clear matched the cleared run: state does not carry over\n");
        ++g.mismatches;
    }
    std::printf("conv_encoder G3: %zu frames, %ld bits, %ld mismatches\n", frames.size(), g.bits, g.mismatches);
    if (g.bits == 0) {
        std::cerr << "no bits compared — refusing to pass an empty gate\n";
        return 1;
    }
    return g.mismatches == 0 ? 0 : 1;
}
