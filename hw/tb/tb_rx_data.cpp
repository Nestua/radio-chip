// G3 gate for the data path's top level (hw/rtl/rx_data.sv, docs/gates/rx_data.md): replays
// hw/vectors/rx_data.txt (rvec rx_data, from radio_phy::rx_data::RxDataSchedule) at the block's
// pins a cycle at a time. Each run: reset, then beat i offered from max(its avail, the cycle
// after beat i − 1 was taken) until taken; it must be taken in the vectors' cycle. Every
// verdict strobe, rx_start, octet and rx_end must be out in its cycle with its fields, and no
// other cycle may show one. The age's inputs as the vectors give them (adc_count = count0 +
// cycle / 8 mod 2^16; end_stamp from each P line's cycle on): rx_age_valid exactly with the
// ends that carry an age, rx_age with its value. Exit code 0 = gate passed.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Vrx_data.h"
#include "verilated.h"

struct Beat {
    long avail, taken;
    char type;  // S F M
    long re = 0, im = 0, cfo = 0;
    unsigned tag = 0, kind = 0;
};
struct Verdict { unsigned tag, fail, symbols, patient; };
struct Start { unsigned rate, length; };
struct Octet { unsigned data, first, seed, service; };
struct End { unsigned status, info, age_valid, age; };
struct Run {
    std::string label;
    unsigned ratio = 0;
    long last = 0;
    unsigned count0 = 0;
    std::map<long, unsigned> stamps;
    std::vector<Beat> beats;
    std::map<long, Verdict> verdicts;
    std::map<long, Start> starts;
    std::map<long, Octet> octets;
    std::map<long, End> ends;
};

static long sext(unsigned long v, int bits) {
    const unsigned long m = 1UL << (bits - 1);
    v &= (1UL << bits) - 1;
    return (long)((v ^ m) - m);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (argc < 2) {
        std::cerr << "usage: sim_rx_data <vectors.txt> [run label]\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 2;
    }
    const std::string only = argc > 2 && argv[2][0] != '+' ? argv[2] : "";
    std::vector<Run> runs;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream f(line);
        std::string t;
        f >> t;
        if (t == "R") {
            Run r;
            f >> r.label >> r.ratio;
            runs.push_back(r);
            continue;
        }
        if (runs.empty()) {
            std::cerr << "a line before any run\n";
            return 2;
        }
        Run& r = runs.back();
        long c;
        if (t == "B") {
            Beat b;
            std::string ty;
            f >> b.avail >> b.taken >> ty;
            b.type = ty[0];
            unsigned long re, im, cfo;
            if (b.type == 'S') {
                f >> std::hex >> re >> im;
                b.re = sext(re, 13);
                b.im = sext(im, 13);
            } else if (b.type == 'F') {
                f >> std::hex >> re >> im >> cfo >> b.tag;
                b.re = sext(re, 13);
                b.im = sext(im, 13);
                b.cfo = sext(cfo, 22);
            } else {
                f >> b.kind;
            }
            r.beats.push_back(b);
        } else if (t == "V") {
            Verdict v;
            f >> c >> std::hex >> v.tag >> std::dec >> v.fail >> v.symbols >> v.patient;
            r.verdicts[c] = v;
        } else if (t == "T") {
            Start s;
            f >> c >> s.rate >> s.length;
            r.starts[c] = s;
        } else if (t == "O") {
            Octet o;
            f >> c >> std::hex >> o.data >> o.first >> o.seed >> o.service;
            r.octets[c] = o;
        } else if (t == "E") {
            End e;
            f >> c >> e.status >> e.info >> e.age_valid >> e.age;
            r.ends[c] = e;
        } else if (t == "C") {
            f >> r.count0;
        } else if (t == "P") {
            unsigned v;
            f >> c >> v;
            r.stamps[c] = v;
        } else if (t == "Z") {
            f >> r.last;
        } else {
            std::cerr << "malformed vector line: " << line.substr(0, 80) << "\n";
            return 2;
        }
        if (f.fail()) {
            std::cerr << "malformed vector line: " << line.substr(0, 80) << "\n";
            return 2;
        }
    }

    Vrx_data dut;
    long total_cycles = 0, total_beats = 0, events = 0, mismatches = 0, ran = 0, ages = 0;
    for (const Run& r : runs) {
        if (!only.empty() && r.label != only) continue;
        ++ran;
        dut.rst_n = 0;
        dut.in_valid = 0;
        dut.ratio = r.ratio;
        for (int i = 0; i < 2; ++i) {
            dut.clk = 0;
            dut.eval();
            dut.clk = 1;
            dut.eval();
        }
        dut.rst_n = 1;
        long run_bad = 0;
        auto bad = [&](long c, const char* what, long got, long want) {
            if (run_bad < 12)
                std::fprintf(stderr, "%s cycle %ld: %s rtl=%lx golden=%lx\n", r.label.c_str(), c,
                             what, got, want);
            ++run_bad;
        };
        size_t bi = 0;
        long prev_taken = -1;
        unsigned stamp = 0x5a5a;   // garbage before the first stamp
        for (long c = 0; c <= r.last; ++c) {
            const auto p = r.stamps.find(c);
            if (p != r.stamps.end()) stamp = p->second;
            dut.adc_count = (r.count0 + static_cast<unsigned>(c / 8)) & 0xffff;
            dut.end_stamp = stamp;
            // The beat offered in this cycle.
            bool offer = false;
            if (bi < r.beats.size()) {
                const Beat& b = r.beats[bi];
                offer = c >= b.avail && c > prev_taken;
                dut.in_valid = offer;
                dut.in_end = b.type == 'M';
                dut.in_kind = b.kind;
                dut.in_first = b.type == 'F';
                dut.in_re = (uint32_t)b.re & 0x1fff;
                dut.in_im = (uint32_t)b.im & 0x1fff;
                dut.in_cfo = (uint32_t)b.cfo & 0x3fffff;
                dut.in_tag = b.tag;
            } else {
                dut.in_valid = 0;
            }
            dut.clk = 0;
            dut.eval();

            if (offer) {
                const Beat& b = r.beats[bi];
                const bool took = dut.in_ready;
                if (took != (c == b.taken)) {
                    bad(c, took ? "beat taken early" : "beat not taken", (long)bi, b.taken);
                    if (run_bad > 12) break;
                }
                if (took) {
                    prev_taken = c;
                    ++bi;
                    ++total_beats;
                }
            }
            // The outputs of this cycle.
            auto v = r.verdicts.find(c);
            if (v == r.verdicts.end()) {
                if (dut.sig_valid) bad(c, "sig_valid", 1, 0);
            } else {
                ++events;
                if (!dut.sig_valid) bad(c, "sig_valid", 0, 1);
                if (dut.sig_tag != v->second.tag) bad(c, "sig_tag", dut.sig_tag, v->second.tag);
                if (dut.sig_fail != v->second.fail) bad(c, "sig_fail", dut.sig_fail, v->second.fail);
                if (!v->second.fail) {
                    if (dut.sig_symbols != v->second.symbols)
                        bad(c, "sig_symbols", dut.sig_symbols, v->second.symbols);
                    if (dut.sig_patient != v->second.patient)
                        bad(c, "sig_patient", dut.sig_patient, v->second.patient);
                }
            }
            auto s = r.starts.find(c);
            if (s == r.starts.end()) {
                if (dut.rx_start) bad(c, "rx_start", 1, 0);
            } else {
                ++events;
                if (!dut.rx_start) bad(c, "rx_start", 0, 1);
                if (dut.rx_rate != s->second.rate) bad(c, "rx_rate", dut.rx_rate, s->second.rate);
                if (dut.rx_length != s->second.length)
                    bad(c, "rx_length", dut.rx_length, s->second.length);
            }
            auto o = r.octets.find(c);
            auto e = r.ends.find(c);
            if (o == r.octets.end()) {
                if (dut.oct_valid) bad(c, "oct_valid", 1, 0);
            } else {
                ++events;
                if (!dut.oct_valid) bad(c, "oct_valid", 0, 1);
                if (dut.oct_data != o->second.data) bad(c, "oct_data", dut.oct_data, o->second.data);
                if (dut.oct_first != o->second.first)
                    bad(c, "oct_first", dut.oct_first, o->second.first);
                const unsigned last = e != r.ends.end() && e->second.status == 0;
                if (dut.oct_last != last) bad(c, "oct_last", dut.oct_last, last);
                if (o->second.first) {
                    if (dut.oct_seed != o->second.seed) bad(c, "oct_seed", dut.oct_seed, o->second.seed);
                    if (dut.oct_service != o->second.service)
                        bad(c, "oct_service", dut.oct_service, o->second.service);
                }
            }
            const unsigned age_valid = e != r.ends.end() && e->second.age_valid;
            if (dut.rx_age_valid != age_valid) bad(c, "rx_age_valid", dut.rx_age_valid, age_valid);
            if (age_valid) {
                ++ages;
                if (dut.rx_age != e->second.age) bad(c, "rx_age", dut.rx_age, e->second.age);
            }
            if (e == r.ends.end()) {
                if (dut.rx_end) bad(c, "rx_end", 1, 0);
            } else {
                ++events;
                if (!dut.rx_end) bad(c, "rx_end", 0, 1);
                if (dut.rx_end_status != e->second.status)
                    bad(c, "rx_end_status", dut.rx_end_status, e->second.status);
                if (dut.rx_end_info != e->second.info)
                    bad(c, "rx_end_info", dut.rx_end_info, e->second.info);
            }

            dut.clk = 1;
            dut.eval();
            ++total_cycles;
        }
        if (bi != r.beats.size()) bad(r.last, "beats left", (long)(r.beats.size() - bi), 0);
        mismatches += run_bad;
    }
    dut.final();

    std::printf("rx_data G3: %ld runs, %ld cycles, %ld beats, %ld events, %ld ages, %ld mismatches\n", ran,
                total_cycles, total_beats, events, ages, mismatches);
    if (ran == 0 || events == 0 || (only.empty() && ages == 0)) {
        std::cerr << "nothing checked — refusing to pass an empty gate\n";
        return 1;
    }
    return mismatches == 0 ? 0 : 1;
}
