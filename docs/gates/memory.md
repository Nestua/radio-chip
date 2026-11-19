# Memory wrappers — gate journal

`hw/rtl/lib/ram_1w1r.sv` and `hw/rtl/lib/ram_1w2r.sv` are the only way RTL in `hw/rtl/` may hold
a memory (CLAUDE.md, RTL rules). An FPGA infers block RAM from the behavioural body; an ASIC flow
substitutes a macro with the same ports and timing. First used by R0-08b part d (ADR 0006): the
receiver's history (26 bits × 512, two read ports) and detector A's C delay line (64 bits × 72).

## 2026-11-18 · gate · R0-08b part d, step 1

**Semantics (G0).** This is the subset every FPGA block RAM and ASIC SRAM macro provides; the
pre-PR review warned that read-first and hold-without-enable are not universal, so the contract
promises neither.
- One clock; synchronous read with a latency of 1.
- A read of the address written in the same cycle is **illegal**: `rdata` is undefined. With
  `CHECK_COLLISION` (default 1) simulation stops on one.
- `rdata` is undefined after a cycle without `re`. Callers register what they keep.
- No reset; the contents are undefined until written.
- DEPTH need not be a power of two; addresses ≥ DEPTH are illegal.
- `ram_1w2r` is two `ram_1w1r` copies written together.

**Why part d never collides:**
- The history writes the newest sample, and a decision reads samples at least 64 samples older.
- The C delay line reads C(n − 24) and C(n − 72) while it writes C(n).

**Gate** (`make -C hw ram`, `hw/tb/tb_ram.cpp` on `hw/tb/ram_bank.sv`, collision check off).
- A reference model of those semantics. A memory has no golden model in `radio-phy`; the
  documented contract is the spec, and the macro an ASIC flow substitutes must meet it.
- 200 000 random cycles on both shapes. Collisions are made on purpose, and their reads are not
  compared, so the gate checks that they disturb nothing else. Reads of unwritten words and
  `rdata` after a cycle without `re` are not compared either.
- Result: 328 712 checks, 69 656 collisions, 0 mismatches.

**Mutations** (built with lint warnings non-fatal, since some leave a signal unused)
| Mutation | Mismatches |
|---|---|
| `we` ignored (always write) | 164 765 |
| copy b never written | 131 091 |
| write address bit 0 flipped | 328 712 |
| port b reads port a's address | 128 804 |

- Write-first is now a legal implementation: 0 mismatches, as expected.
- With `CHECK_COLLISION` on, the random test stops at the first collision (`$error` from
  `g_check`), so the check itself works.

**Open:**
- No FPGA inference check yet (Vivado should map both shapes to block RAM or LUT RAM). It will be
  run with the top level's OOC synthesis.
- The ASIC macro mapping comes with R5.

## 2026-11-19 · note · the wrapper's contract against the IHP SG13G2 macro set

The phase review (`docs/reviews/2026-11-19-r0-phase-review.md`, finding 8) asked whether the
open PDK of R5 offers a memory with the wrapper's ports before more blocks depend on it.
Checked by reading the PDK (IHP-Open-PDK at e73adb4, `ihp-sg13g2/libs.ref/sg13g2_sram`); nothing
was synthesised.

- The library has two-port macros `RM_IHPSG13_2P_*`: ports A and B, each with its own clock,
  enable, write enable, read enable, address and data. Sizes: 64 × 22, 64 × 32, 256 × 8/16/32,
  512 × 8/16/32, 1024 × 16/32. (Single-port `1P` macros go to 8192 × 32.)
- `ram_1w1r` maps to one of them: write on A, read on B, the same clock on both. `ram_1w2r`
  is two, as the wrapper already builds it.
- The receiver front's memories fit: the 512 × 26-bit history in a 512 × 32 (two for the two
  read ports), the 73 × 64-bit delay line of `rx_latch` in two 256 × 32 (73 words exceed the
  64-word macros) or in flip-flops.
- A read and a write of one address in the same cycle is what the wrapper forbids
  (`CHECK_COLLISION`); the macro's behaviour in that case therefore does not matter.
- Not checked: the macros' timing at our clock, their bit-mask and BIST pins (tied off in a
  wrapper body), and `rdata` after a cycle without a read. These come with R0-14 and R5.
