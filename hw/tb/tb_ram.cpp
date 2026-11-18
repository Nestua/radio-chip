// Gate for the memory wrappers (hw/rtl/lib/ram_1w1r.sv, ram_1w2r.sv) through hw/tb/ram_bank.sv:
// 200 000 random cycles against a reference model of the documented semantics — synchronous
// read, read-first on a same-cycle write, rdata held without re — on both shapes. A read of a
// word never written is not compared (contents are undefined until written). Address
// collisions are forced often (one address in 4 is drawn from a small hot set). Exit code 0 =
// gate passed.

#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "Vram_bank.h"
#include "verilated.h"

namespace {

struct Ref {
    std::vector<uint64_t> mem;
    std::vector<bool> valid;
    uint64_t mask;
    explicit Ref(size_t depth, int w)
        : mem(depth, 0), valid(depth, false), mask(w == 64 ? ~0ull : (1ull << w) - 1) {}
};

struct Port {
    uint64_t want = 0;
    bool known = false;  // rdata defined (read of a written word, then held)
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vram_bank dut;
    std::mt19937_64 rng(0x5a3d);
    Ref h(512, 26), c(72, 64);
    Port pa, pb, pc;
    long checks = 0, mismatches = 0, collisions = 0;
    auto addr = [&](size_t depth) -> uint32_t {
        if (rng() % 4 == 0) return static_cast<uint32_t>(rng() % 3);  // hot set
        return static_cast<uint32_t>(rng() % depth);
    };
    auto check = [&](const char* what, long cyc, uint64_t got, const Port& p) {
        if (!p.known) return;
        ++checks;
        if (got != p.want) {
            if (mismatches < 10)
                std::fprintf(stderr, "cycle %ld %s: rtl=%llx ref=%llx\n", cyc, what,
                             static_cast<unsigned long long>(got),
                             static_cast<unsigned long long>(p.want));
            ++mismatches;
        }
    };
    dut.clk = 0;
    dut.eval();
    for (long cyc = 0; cyc < 200000; ++cyc) {
        // drive this cycle's inputs
        const bool hwe = rng() % 2, hra = rng() % 3 != 0, hrb = rng() % 3 != 0;
        const uint32_t hw = addr(512), ha = rng() % 2 ? hw : addr(512), hb = addr(512);
        const uint64_t hd = rng() & h.mask;
        const bool cwe = rng() % 2, cre = rng() % 3 != 0;
        const uint32_t cw = addr(72), ca = rng() % 2 ? cw : addr(72);
        const uint64_t cd = rng();
        dut.h_we = hwe;
        dut.h_waddr = hw;
        dut.h_wdata = static_cast<uint32_t>(hd);
        dut.h_re_a = hra;
        dut.h_raddr_a = ha;
        dut.h_re_b = hrb;
        dut.h_raddr_b = hb;
        dut.c_we = cwe;
        dut.c_waddr = cw;
        dut.c_wdata = cd;
        dut.c_re = cre;
        dut.c_raddr = ca;
        // reference: read-first, then write
        if (hra) pa = {h.mem[ha], static_cast<bool>(h.valid[ha])};
        if (hrb) pb = {h.mem[hb], static_cast<bool>(h.valid[hb])};
        if (cre) pc = {c.mem[ca], static_cast<bool>(c.valid[ca])};
        collisions += (hwe && hra && ha == hw) + (hwe && hrb && hb == hw) + (cwe && cre && ca == cw);
        if (hwe) {
            h.mem[hw] = hd;
            h.valid[hw] = true;
        }
        if (cwe) {
            c.mem[cw] = cd;
            c.valid[cw] = true;
        }
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
        check("h.a", cyc, dut.h_rdata_a, pa);
        check("h.b", cyc, dut.h_rdata_b, pb);
        check("c", cyc, dut.c_rdata, pc);
    }
    std::printf("ram gate: 200000 cycles, %ld checks, %ld same-cycle read/write collisions, %ld "
                "mismatches\n", checks, collisions, mismatches);
    if (checks < 100000) {
        std::fprintf(stderr, "too few checks\n");
        return 1;
    }
    return mismatches == 0 ? 0 : 1;
}
