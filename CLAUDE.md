# CLAUDE.md

Instructions for Claude Code (and, through the `AGENTS.md` symlink, every coding agent) working in
this repository. Read at the start of every session. Keep it terse and current.

## What this is

An open European 802.11-compatible radio chip, built FPGA-first and designed for ASIC from day one.
One OFDM/LDPC core serves three profiles: a long-range TDMA mode for drone and point-to-point links
(first user: the Nestua drone-in-a-box), Wi-Fi 4 → 6/6E → 7, and HaLow. The team is small on
purpose: the founder, Claude, and one or two hands-on engineers. Claude writes most of the digital
work; independent gates decide what is accepted.

Read before substantial work:

- `ROADMAP.md` — stages R0–R6, their exit gates, and where we are.
- `BACKLOG.md` — the issues of the current stage (source of truth for scope).
- `docs/testing.md` — the gate ladder G0–G6. **Nothing is "done" without its gates.**
- `docs/decisions/` — ADRs. A change that contradicts one needs a new ADR first.
- `docs/gates/<block>.md` — the evidence journal of the block you are touching.

## Non-negotiables

1. **The golden model is the spec.** `crates/radio-phy` (and later `radio-mac`) defines every bit
   the hardware produces, in the hardware's own fixed-point arithmetic. RTL is correct when it is
   bit-exact with the model, not when it "looks right" (ADR 0003).
2. **Test first.** Write the gate — reference vectors, the failing test — before the implementation,
   and run it to see it fail. A test that has never failed has not been shown to test anything.
3. **Every block needs a reference we did not write.** The standard's printed sequences and worked
   examples, published test vectors, a black-box run of an external tool, a capture from a
   commercial chip. Agreement between our own model and our own RTL is G3, never G1.
4. **Test the test.** When a gate is new, mutate the implementation (flip a tap, an index, a sign)
   and confirm the gate fails. Record that in the gate journal.
5. **Generated files are never hand-edited.** `hw/vectors/*` come from `rvec`; CI regenerates them
   and fails on any diff. Change the generator, then regenerate.
6. **No GPL/AGPL code enters the repo.** openwifi (AGPLv3), gr-ieee802-11 (GPLv3), Linux drivers
   and similar are read for understanding and run as black boxes to produce reference vectors.
   Their code is never copied or translated (ADR 0005). Cite what you used.
7. **RTL is ASIC-portable.** No vendor primitives or IP in `hw/rtl/`; see the RTL rules below.
8. **Claims need evidence.** "Passes", "bit-exact", "meets timing" are written only next to a
   gate-journal entry with the commit, the command and the result.
9. **Independent review at risk points only:** PHY architecture decisions, anything sent to a fab
   (MPW), anything touching patents or licences, CE/RED test plans, and the pre-PR review. Routine
   blocks pass on their gates.

## Gate ladder (summary — details in `docs/testing.md`)

| Gate | Question | Typical evidence |
|---|---|---|
| G0 | Is the block specified? | Clause of the standard, interface, number formats in the block doc |
| G1 | Does the model match an independent reference? | `crates/*/tests/g1_*.rs` |
| G2 | Does fixed point lose less than its budget vs float? | Sweep report, BER/EVM loss in dB |
| G3 | Is the RTL bit-exact with the model? | `make -C hw <block>` on committed vectors |
| G4 | Does it hold beyond the vectors? | Formal properties (SymbiYosys), long random runs |
| G5 | Does it interoperate on real hardware? | FPGA over the air with commodity Wi-Fi cards |
| G6 | Does it work in the field? | IQ captures, PER vs distance, drone video |

## Layout

```
crates/radio-phy/     golden PHY model (the spec); tests/g1_* are the G1 gates
crates/radio-tools/   rvec: golden model → hw/vectors/*.txt
hw/rtl/               synthesisable SystemVerilog, ASIC-portable
hw/tb/                Verilator testbenches (C++), one per block
hw/vectors/           generated, committed vectors (never edit by hand)
hw/Makefile           G3 gates: make -C hw [block|lint|vectors]
docs/decisions/       ADRs
docs/gates/           per-block evidence journals
scripts/check.sh      the pre-PR gate: everything CI runs, locally
```

FPGA board wrappers (`hw/fpga/`), formal properties (`hw/formal/`), the ASIC flow (`hw/asic/`), the
MAC model and the Linux driver arrive with the stages that need them (see `ROADMAP.md`).

## Workflow per block

1. Spec (G0): add the block to `BACKLOG.md` with its standard clause, interface and number formats.
2. Reference first (G1): transcribe or generate the independent reference; write `tests/g1_<block>.rs`; see it fail.
3. Model: implement in `radio-phy` until G1 passes. Fixed point where the hardware is fixed point.
4. Fixed-point budget (G2) for DSP blocks: sweep word lengths against a float model; record the loss.
5. Vectors: add the block to `rvec`, `make -C hw vectors`, commit the file.
6. RTL (G3): write `hw/rtl/<block>.sv` + `hw/tb/tb_<block>.cpp`, add the target to `hw/Makefile`; mutate once to prove the gate bites.
7. Journal: append the evidence to `docs/gates/<block>.md`.
8. `scripts/check.sh` green → PR.

## Rust conventions

- Edition 2021, MSRV 1.89, `cargo fmt`, `cargo clippy --workspace --all-targets -- -D warnings`.
- No `unwrap`/`expect`/`panic!` in library code (denied by workspace lints; allowed in tests).
- No `unsafe` (forbidden). No new dependency without a one-line justification in the PR.
- Bits are `u8` holding 0/1 in transmission order. Fixed-point types state their format (`Q1.15`) in the name or doc.
- Cite the standard next to the code it justifies: `// IEEE 802.11-2020 §17.3.5.5`.
- Comments say why, not what. Every public item has a doc comment.

## RTL conventions

- SystemVerilog, synthesisable subset; `` `default_nettype none `` at the top, restored at the bottom.
- One clock domain per block unless an ADR says otherwise. Synchronous, active-low `rst_n`.
- No `initial` blocks, no vendor primitives, no vendor IP (FFT, FIR, DDS are written here).
  FPGA-specific code lives only in `hw/fpga/` wrappers.
- Memories and clock gating only through wrappers in `hw/rtl/lib/` (added with the first block that needs them).
- Streaming interfaces: `valid`/`ready`, or document why a block never stalls.
- `make -C hw lint` is clean under `verilator -Wall`; warnings are errors.
- Each module's header names its golden-model counterpart and its gate target.

## Git

- Branch per task. Commit messages: `[R0-02] convolutional encoder: G1 against the standard's worked example`.
- One block per PR where possible. `scripts/check.sh` must be green before a PR.
- CLAUDE.md changes go in their own PR (`[meta] ...`), with an ADR when they change a rule.

## Session discipline

- Start: read `ROADMAP.md` (current stage), the `BACKLOG.md` item, the block's gate journal.
- End: update the gate journal and `BACKLOG.md` status; note open questions in the journal.
- Project memory lives in the repo, not in chat. If it matters next week, write it down here.

## Quick reference

| Need to… | Run |
|---|---|
| Everything CI runs | `scripts/check.sh` |
| Rust tests | `cargo test --workspace` |
| Lint / format | `cargo clippy --workspace --all-targets -- -D warnings` / `cargo fmt` |
| All RTL gates | `make -C hw` |
| One RTL gate | `make -C hw scrambler` |
| Regenerate vectors | `make -C hw vectors` |

Tools: Rust stable (via `rust-toolchain.toml`), Verilator ≥ 5.050 (`brew install verilator`).
Formal (SymbiYosys) and the ASIC flow (OpenROAD) are added when a stage needs them.

## Common mistakes

- Trusting a test that agrees with the code it was written alongside. Find an outside reference.
- Bit order. 802.11 transmits LSB first in most fields; check the clause, then test it.
- Floating point creeping into the model of a fixed-point block. The model must round like the RTL.
- Editing `hw/vectors/` by hand to make a gate pass.
- Copying code from an external project "just for one function". Read, close it, re-implement.
