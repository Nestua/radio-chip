# Testing: the gate ladder

A block is accepted when it has passed the gates its stage requires, and the evidence is in its
journal (`docs/gates/<block>.md`). Gates are designed so that they do not depend on whoever wrote
the code — including Claude. That is what makes a small team with an AI author safe.

## Principles

- **Test first.** The gate exists and fails before the implementation exists.
- **Independent references.** At least one reference per block comes from outside the project.
- **Test the test.** Every new gate is shown to fail on a deliberately broken implementation.
- **Generated, not typed.** Vectors and reference tables are produced by tools and regenerated in CI.
- **Evidence, not adjectives.** A result is a journal entry: date, commit, command, numbers.

## The gates

### G0 — Specified

The block has a `BACKLOG.md` entry with: the clause of the standard it implements, its interface
(ports or function signature), number formats, latency/throughput target, and which gates apply.

### G1 — Model matches an independent reference

`crates/<crate>/tests/g1_<block>.rs`. Acceptable references, strongest first:

1. Sequences and worked examples printed in the standard (e.g. the 127-bit scrambler sequence in
   IEEE 802.11-2020 §17.3.5.5; the standard's annex with a fully worked OFDM frame encoding).
2. Published conformance or test vectors.
3. Black-box output of an external implementation (gr-ieee802-11, MATLAB WLAN Toolbox, a
   commodity Wi-Fi card in monitor mode). Run it, save its output as a fixture, record the tool
   version. Never copy its code.
4. Mathematical properties that pin the behaviour down (period, inverse, linearity, energy).
   Properties support a reference; on their own they do not pass G1.

### G2 — Fixed-point budget

For DSP blocks only. A float model and the fixed-point model run the same channel; the loss in
BER, PER or EVM must stay within the block's budget (stated in G0, typically ≤ 0.1–0.2 dB).
The sweep that chose each word length is kept as a report and cited in the journal.

### G3 — RTL bit-exact with the model

`make -C hw <block>`: the Verilator testbench replays `hw/vectors/<block>.txt` and every output must
match. The testbench refuses to pass on zero vectors. New G3 gates are mutation-tested once.
Vectors cover: all modes/rates of the block, every seed or state where that is finite, random
payloads, boundary lengths, back-to-back frames.

### G4 — Beyond the vectors

Formal properties with SymbiYosys (no lost or duplicated samples, handshake rules, reset
behaviour, equivalence between a reference and an optimised version), plus long randomised
co-simulation against the model. Required before a block goes into an MPW tape-out.

### G5 — Real hardware, real chips

The block runs on the FPGA platform over the air. Interop: commodity Wi-Fi cards in monitor mode
receive our frames and we decode theirs. A commercial chip acting as judge is the strongest
independent check we have.

### G6 — Field

Recorded IQ captures, PER versus distance, video from the drone over the TDMA link. Captures live
outside git (`/captures/` is ignored); the journal records where they are and their checksums.

### ASIC probes (record, not pass/fail)

Area, timing and power of a block in OpenROAD (ASAP7 first, then the IHP open PDK). Kept as numbers
in the journal so that architecture choices are made with silicon in mind.

## Which gates each stage needs

| Stage | Gates |
|---|---|
| R0 Golden model | G0, G1, G2 |
| R1 Over the air on SDR | + G6 (model in real time on SDR) |
| R2 RTL blocks | + G3, G4 for blocks with state machines or handshakes |
| R3 FPGA link | + G5 |
| R4 Serial module | all, plus CE/RED pre-tests |
| R5 Test chips | all, G4 mandatory for every block on the die |

## Gate journal format

One file per block, `docs/gates/<block>.md`, newest entry first:

```
## 2026-11-16 · G3 · <commit>
Command: make -C hw scrambler
Result: 127 vectors, 32 512 bits, 0 mismatches
Mutation check: x7^x5 tap → 16 384 mismatches, gate failed as expected
Notes: —
```

## The pre-PR gate

`scripts/check.sh` runs what CI runs: format, clippy, Rust tests, vector regeneration with a diff
check, RTL lint and every G3 gate. It must be green before a PR is opened.
