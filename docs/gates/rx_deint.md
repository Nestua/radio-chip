# Gate journal: `rx_deint`

Block: the deinterleaver and depuncturer, R0-09's data path, ADR 0010 decisions 1 and 6.2 ("the
deinterleaver and depuncturer"), with the soft buffer of decision 3 (two symbols). Model: per
symbol `radio_phy::rx_ofdm_fixed::deinterleave_soft`, then `radio_phy::viterbi::depuncture`, the
block's first `steps` pairs (what `WindowedViterbi::push` takes up to the anchor).
`deinterleave_soft` is the map `symbol_with` made inline, split out without a change of
behaviour; `tests/g0_rx_deint.rs` holds it equal to the interleaver's G1-gated `deinterleave`
(`docs/gates/interleaver.md`). RTL: `hw/rtl/rx_deint.sv`. Gate: `make -C hw rx_deint`.

## 2026-11-24 · G0 · the specification and the architecture

**Two banks of one write and one read port: decided by the founder, 2026-11-24** (the
recommendation). The alternatives were one memory with two read ports (`ram_1w2r` is two
copies: 9216 bits for 4608) and flip-flops with the deinterleaver on the write side and free
reads through multiplexers (≈ 0.25 mm² or more, every bit toggling).

### What the block computes (fixed by the model)

- In: a symbol's soft values in on-air order, as the symbol block's soft pass gives them: a data
  subcarrier a beat (j = 0 … 47), its N_BPSC 8-bit values in lanes 0 … N_BPSC − 1.
- Coded value k of a symbol is on-air value `interleaved_index(k)` (IEEE 802.11-2020
  §17.3.5.7). The symbols' coded values in order are depunctured by the rate's pattern
  (§17.3.5.6, Figure 17-9: 1/2 `11`; 2/3 `1110`; 3/4 `111001`), a stolen value being an
  erasure (0), and handed on as (A, B) pairs, a trellis step each.
- A block is SIGNAL (one BPSK symbol at rate 1/2, 24 steps) or a frame's data (its symbols, up
  to the anchor 16 + 8·LENGTH + 6 steps; the pad after it is not fed, as `viterbi.sv` requires).
  The anchor is at most 16 + 8·4095 + 6 = 32 782: **16 bits** (below).

### The architecture

- **A subcarrier is one 48-bit word** (six lanes of 8 bits, lane i in bits 8i + 7 … 8i), written
  in the beat it comes.
- **Coded bit k lies on data subcarrier 3·(k mod 16) + ⌊⌊k/16⌋ / N_BPSC⌋** (the first
  permutation's column is k mod 16, its row ⌊k/16⌋; the second permutation moves a bit only
  within its subcarrier). So the subcarrier's group ⌊j/3⌋ is k mod 16, and its parity is k's.
  **Bank b holds the groups of parity b**: word slot·24 + 3·⌊g/2⌋ + j mod 3, 2 slots × 24 words
  of 48 bits a bank. **Two consecutive coded bits are always in different banks**, so one read
  of each bank a clock gives any two of them: a step a clock at every rate (a step keeps at
  most two values, and two kept values of a step are consecutive coded bits).
- **The lane** of bit k: with c = k mod 16, r = ⌊k/16⌋, r' = r mod N_BPSC and s = max(N_BPSC/2,
  1), s·⌊r'/s⌋ + (r' − c) mod s.
- **The reader** walks k with counters (c, r mod N_BPSC, ⌊r/N_BPSC⌋), increments only: a step
  takes the pattern's kept values, each the next k; a stolen one is 0. **A symbol is whole
  periods** (48·N_BPSC is a multiple of the kept count of every rate's period: 2, 3, 4), so the
  phase is 0 at every symbol's start: the reader does not force it (G0 asserts it). It is reset
  at a block's start, since a block can end mid-period.
- **Two slots**: a symbol's soft pass writes one while the decoder's steps read the other, as the
  schedule model's `soft_symbols = 2` assumes. The modulation of a slot is latched with its last
  subcarrier. At a block's last step the rest of its symbol is not read; the slot is freed.

### The gate

`crates/radio-phy/tests/g0_rx_deint.rs` (`cargo test -p radio-phy --test g0_rx_deint`):
- `walk_is_the_interleaver`: the counters' (bank, word, lane) of every coded bit of every
  modulation is where `interleaved_index` puts it, and its bank is k's parity.
- `signal_block`, `data_blocks`: the architecture (two banks, two slots, the counters, the
  pattern by phase) on random values in ±127 against the model, pair for pair — SIGNAL 200
  times; every rate with lengths 0 … 3, 23, 24, 100, 1500, 4095, 40 random ones, and the
  lengths whose anchor reads the fewest and the most steps of the last symbol (the anchor is 6
  modulo 8, so at no rate does it end exactly on a symbol's end). Asserted on the way: no step
  reads one bank twice, the phase is 0 at every symbol's start, a symbol's walk ends on its end.
- `deinterleave_soft_is_the_g1_deinterleaver`.

**Seen to fail** (mutants of the architecture in the test, 2026-11-24): bank by ⌊c/2⌋ mod 2;
the lane's rotation (r' + c) mod s; no rotation; the write word's j mod 3 off by one; the row
counter wrapping a row late; A and B's pattern bits swapped; a stolen B as 1 — 7 of 8 fail.
**Alive: both symbols in slot 0** — expected: in G0 each symbol is written and then read before
the next is written, so the slots matter only when the two overlap in time, which is the
RTL gate's (G3) to show (and does: the RTL mutant "rd never toggles" fails, below).

## 2026-11-24 · G3 · `hw/rtl/rx_deint.sv` bit-exact with the model, every step in its edge

Commit: `59e1e4a`. Command: `make -C hw rx_deint`. Vectors:
`hw/vectors/rx_deint.txt` (`rvec rx_deint`): 153 packets of two blocks, SIGNAL and the data,
at all eight rates; lengths 0 … 3, 23, 24, 100, the anchor's edge lengths and 8 random ones a
rate; at 54 Mb/s the longest PSDU (4095 octets, 152 symbols, 32 782 steps).

Result (after the review, below): **`rx_deint: 306 blocks, 126981 steps compared, 211669
slot_ok checks, 0 mismatches; 13 drops, 8 resets (24 with a beat or a block), 11 replays, 3552
write gaps, 1089 waits, 118 late blocks` — PASS**; `scripts/check.sh` green.

The bench (`hw/tb/tb_rx_deint.cpp`) checks:
- every step's (A, B), `v_first`, `v_last` against the vectors, and **its edge** against the
  header's rules: a symbol's first step 2 edges after the latest of its `soft_last`'s edge, its
  block's `blk_valid`'s edge (the block's first symbol) and the edge before the previous step's;
  then a step an edge. Nothing else may come out.
- `slot_ok` in every cycle against "fewer than two symbols written and not freed", a slot freed
  at the edge before its last step's `v_valid`.
- The driving: symbols written as soon as `slot_ok` allows or after random waits, with random
  idle beats, random values in the unused lanes, `soft_last` random while `soft_valid` is low
  (`rx_symbol` holds it after its 48th beat), `soft_mod` random but at `soft_last`; blocks announced before, during or after their
  first symbol's write, the data's at the earliest the contract allows (the edge of SIGNAL's
  last step) or later; packets back to back (the next SIGNAL written while the data are read);
  drops and resets in a sixth of the packets and always in the longest, at a random edge from
  the packet's last write on, some with a write beat or a block in their cycle (the drop wins);
  the packet then replayed (the longest always) or skipped, nothing due from the drop's edge on
  allowed out. Registers start random (`+verilator+rand+reset+2`).

**Found by the gate before it passed**: the step counter was 15 bits, as the header first said
"at most 32 782" without counting: 2^15 = 32 768. The longest PSDU, put in the vectors for this,
ended the block at step 13 of 32 782. Now 16 bits.

**Mutants** (2026-11-24, `make -C hw rx_deint` with `-Wno-fatal` on each, so that no mutant
dies of a lint warning; restored by `cp` and `touch`): 28 of 28 fail on the bench, rerun on the
bench as amended after the review. The bank by bit 1 of c; A's or
B's pattern bit at the wrong phase; B always the bit after A's; the walk always two ahead; c
wrapping at 14; the 16-QAM lane without its rotation; the 64-QAM rotation reversed, c mod 4 for
c mod 3, r' ≥ 3 as > 3, the upper half's offset dropped; the read word without ⌊r/N⌋; the write
word without j mod 3; ⌊j/3⌋ by 42/128; the symbol's end a row early; the last step one late;
`v_first` on step 1; 2/3's phase never wrapping; a block's end not freeing its slot; `slot_ok`
from the read slot; a stolen A not zeroed; the read slot's modulation from the write slot; the
read slot never toggling; QPSK's N_BPSC as 1; the issue without the block; the phase not reset
at a block's start; `v_last` at a symbol's end; every slot latched as 64-QAM.

### Independent review (fresh agent, 2026-11-24)

No defect found in the RTL (collisions, banks, the ⌊j/3⌋ product, lanes, patterns, ends
mid-symbol and mid-period, slot accounting, drop and reset, the counter's width traced by hand;
the bench passed with six other seeds). Found and fixed:
- **The bench never raised `soft_last` without `soft_valid`**, which `rx_symbol` does after its
  48th beat: the mutant "`full` set on `soft_last` alone" passed. `soft_last` is now random
  while idle, and the header says it counts only with `soft_valid`; the mutant fails.
- **`soft_mod` never changed inside a symbol**: the mutant "the modulation latched at beat 0"
  passed. Now random but at `soft_last`; the mutant fails.
- **Drops were less random than said** (fixed packets, never the longest, never with a beat or
  a block in their cycle): widened as above.
- The header's rule for the next block is now in edges (no earlier than the edge of the
  previous block's last `v_valid`).
- Noted, not changed: the counter reset at a block's end is redundant with the one at
  `blk_valid` (an equivalent mutant); the final count check (`compared` ≥ steps of packets not
  skipped) lets replays add to it, while every step's own check catches a lost or extra step.

### Latency, against the schedule model

The first step's `v_valid` is 2 edges after the edge that took the symbol's last soft value;
the schedule model (`rx_data`, `DataTiming`) starts the trellis steps the cycle after the values
are written (L + 1). The step comes one clock later than the allowance (the read is registered,
then the lane's multiplexer): to be updated with the top level, as `rx_track`'s 87 clocks. A slot
is free one clock before its symbol's last step is out, earlier than the model's "the cycle after
the decoder's last step", so the model's rule stays conservative.

### Open

- The memory: two `ram_1w1r` of 48 × 48 bits. The IHP SG13G2 two-port macros (`memory.md`) are
  8, 16 or 32 bits wide and 64 words or more: a bank is one 32-bit and one 16-bit macro, or
  flip-flops (2 × 2304). With R0-14/R5.
- The Viterbi decoder's contract (a block's first step n + 2 edges after the previous block's
  last) is the top level's: SIGNAL's verdict and the data's soft pass lie between the two.
