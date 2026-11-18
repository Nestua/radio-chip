# Memory wrappers — gate journal

`hw/rtl/lib/ram_1w1r.sv` and `hw/rtl/lib/ram_1w2r.sv` are the only way RTL in `hw/rtl/` may hold
a memory (CLAUDE.md, RTL rules). An FPGA infers block RAM from the behavioural body; an ASIC flow
substitutes a macro with the same ports and timing. First used by R0-08b part d (ADR 0006): the
receiver's history (26 bits × 512, two read ports) and detector A's C delay line (64 bits × 72).

## 2026-11-18 · gate · R0-08b part d, step 1

**Semantics (G0)**
- One clock; synchronous read with a latency of 1.
- A read and a write of the same address in the same cycle return the old word (read-first).
- `rdata` holds while `re` is low.
- No reset; the contents are undefined until written.
- DEPTH need not be a power of two.
- `ram_1w2r` is two `ram_1w1r` copies written together.

**Gate** (`make -C hw ram`, `hw/tb/tb_ram.cpp` on `hw/tb/ram_bank.sv`).
- A reference model of exactly those semantics. A memory has no golden model in `radio-phy`; the
  documented contract is the spec, and the macro an ASIC flow substitutes must meet it.
- 200 000 random cycles on both shapes, one address in 4 drawn from a hot set of three. Reads of
  never-written words are not compared.
- Result: 597 858 checks, 69 656 same-cycle read/write collisions, 0 mismatches.

**Mutations** (built with lint warnings non-fatal, since some leave a signal unused)
| Mutation | Mismatches |
|---|---|
| write-first | 103 804 |
| `rdata` not held without `re` | 195 527 |
| write address bit 0 flipped | 597 858 |
| port b reads port a's address | 194 419 |

Without the lint relaxation, the first, second and fourth never build under `-Wall`.

**Open:**
- No FPGA inference check yet (Vivado should map both shapes to block RAM or LUT RAM). It will be
  run with the top level's OOC synthesis.
- The ASIC macro mapping comes with R5.
