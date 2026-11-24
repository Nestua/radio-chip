# Gate journal: `rx_octets`

Block: the descrambler and the octets out, R0-09's data path, ADR 0010 decision 5 (octets and a
status to the MAC) and PR B of `rx_data.md`'s three: the data block's decoded bits, from the
first SERVICE bit, to the PSDU's octets with the seed and SERVICE on the first, and the end
statuses that are this block's (decoded, seed zero). Model: `radio_phy::rx_octets::RxOctets`,
the bits-to-octets part of `RxDataFixed` split out unchanged (the scrambler's G1 is
`docs/gates/scrambler.md`). RTL: `hw/rtl/rx_octets.sv`. Gate: `make -C hw rx_octets`.

## 2026-11-24 · G0, G1, G3 · the specification, the architecture, the RTL

No architecture question went to the founder: the block's behaviour is fixed by the model and
the founder's rules of 2026-11-24 (`rx_data.md`: a seed of zero reads the frame to its end and
ends where the last octet would have been; octets with valid, last and first, no ready), and
its one latency is the schedule's allowance.

### What the block computes (fixed by the model)

- IEEE 802.11-2020 §17.3.5.2: SERVICE's 16 bits come first, bit 0 first; bits 0 … 6 are zero
  before scrambling, 7 … 15 reserved (zero from a conforming sender; handed on as received). The
  PSDU follows, each octet LSB first; then the tail and pad, which the receiver does not read.
- §17.3.5.5: the scrambler x⁷ + x⁴ + 1; its first seven outputs are the first seven received
  bits (they scrambled zeros), so they give the transmitter's seed. All seven zero is no seed
  (no non-zero state gives seven zeros): no octet, the end `Seed` on the bit the last octet
  would have come on (`rx_data.md`, 2026-11-24).
- Out: each octet descrambled, the first received bit in bit 0; the first with the seed (bit i =
  x(i+1), as `scrambler.sv` and `Scrambler::new`) and SERVICE descrambled (bit 0 first; bits
  0 … 6 always zero); the last with the end `Decoded`. LENGTH is SIGNAL's, 1 … 4095.

### The architecture and its arithmetic (`tests/g0_rx_octets.rs`)

- **No seed search.** The model tries the 127 seeds; the RTL shifts the head into the scrambler's
  state as it is received: after seven bits that is the state the transmitter's scrambler had
  (x1 the newest), and each later bit is descrambled with x7 ⊕ x4 and that sequence bit shifted
  in. Written as one path: the descrambled bit c is forced to 0 in the head and the state takes
  in_bit ⊕ c (= the received bit in the head, the sequence bit after). The gate: from the head's
  state the next 127 outputs are the model scrambler's, on all 127 non-zero heads.
- **The seed** for the MAC, the state seven steps before the head, from y(n − 7) = y(n) ⊕
  y(n − 4) run backwards: seven XORs (`seed_of`), equal to the model's search on all 128 heads;
  the all-zero head gives 0, read as "no seed".
- **Counts**: 4 bits through SERVICE (wrapping to 0 into the PSDU, where its low 3 bits count the
  octet's bits), 12 bits of octets left (LENGTH ≤ 4095; G3 runs a 4095-octet frame, which is what shows the
  RTL's count wide enough; G0 only holds the model's end on it).
- **Registers**: the state (7), the octet so far (7), SERVICE bits 7 … 15 (9: a shift of every
  SERVICE bit, the last nine kept), the seed (7), the octets left (12), the count (4), four
  flags; outputs registered. About 60 flops, no memory, no arithmetic beyond a 12-bit decrement.

### The interface (`hw/rtl/rx_octets.sv`'s header)

- In: `start` with `length` (taken only with the start: the verdict when SIGNAL decoded),
  `drop` (the packet ended by a mark), the decoder's `in_valid`/`in_bit` of the data block (a
  bit a clock at most, gaps anywhere; bits with no frame armed are ignored: SIGNAL's, the
  tail's). Priority drop, start, then the cycle's bit (a bit with a start or a drop is not
  taken).
- Out: `oct_valid` one cycle with `oct_data`, `oct_first`, `oct_last`; `oct_seed`,
  `oct_service` steady from the first octet to the frame's end (they follow the next frame's
  bits, even of a frame dropped or without a seed: read them with `oct_first`); `end_valid` one
  cycle with `end_seed` (0 decoded, with the last octet; 1 seed zero). The data outputs are not
  reset and mean something only with their valid. The block never stalls.
- **Latency: the octet (and its end) registered on the edge that takes its last bit**, one edge
  after `viterbi.sv` registered that bit: the schedule's `DataTiming::octet` = 1, as allowed —
  no schedule value changes (`g2_rx_data_schedule` unchanged, octets 721 / 809 / 817 / 905 seen
  in the hand table).
- Left to the top level (PR C): the other end statuses (SIGNAL failed, truncated, aborted and
  the age), gating `in_valid` to the data block's bits if it prefers (the block ignores bits
  while not armed, so SIGNAL's are harmless). LENGTH 0 never reaches the block
  (`signal_checked` rejects it); were it given, the RTL would end after 4096 octets and the
  model never: PR C asserts the start's length is not 0.
- Review (pre-PR, independent, 2026-11-24): nothing serious; it regenerated the vectors
  (byte-identical), reran G3, and ran 3000 random segments of its own (random starts, drops and
  their combinations, a third with a zero head): 0 mismatches over 359 582 cycles. Its
  low-severity points fixed: the hold of seed and SERVICE stated as it is, LENGTH 0 noted, the
  G0 test of the octet count renamed (the RTL's 12-bit count is shown by G3's 4095-octet
  frame, not by G0).

### G1: the standard's worked example (`tests/g1_annex_i1_hashes.rs`, `descrambler_and_octets`)

Annex I.1 (802.11-2024), in CI by hashes (ADR 0005 addendum): the model's scrambled DATA bits,
equal to Table I-15 by its hash, through `RxOctets` give the seed of Table I-14 (0x5d, the one
seed whose 127-bit sequence is the table) and SERVICE plus the PSDU in clear equal to Table
I-13 by its hash (the tail and pad, not read, are I-13's zeros). The annex's SERVICE is all
zero, so it cannot see SERVICE's bit order: a unit test (`rx_octets::tests::
service_bit_k_is_the_kth_bit_sent`) holds §17.3.5.2's order bit by bit.

**Model mutants** (`rx_octets.rs`, against the G1 test, the unit tests and G0; restored by copy
and touch): octet MSB first, the seed ^ 1, SERVICE's order reversed, no descrambling, SERVICE
15 bits: **5 of 5 fail** (SERVICE's order only after the unit test was added; G1 alone passed
it, as above).

### G3

Vectors: `rvec rx_octets` (`crates/radio-tools/src/rx_octets_vec.rs`), cycles at the pins with
the model's events in the cycle of the edge that takes their bit: all 127 seeds (frames of 1 to
40 octets from `data_bits`), lengths at the counters' edges (1 … 17, 255 … 257, 1500) and 12
random ones on random bits (any SERVICE), the 4095-octet frame a bit a clock, seed zero at six
lengths up to 4095, a bit every cycle and gaps of up to 3, SIGNAL's 24 bits before each start
and the tail after the end (ignored), drops before the seed, inside SERVICE, inside the PSDU, on
an octet's last bit (drop wins) and on a zero-seed frame, a start with a bit, back-to-back
frames, a start during a frame, a start and a drop in one cycle, bits and a drop with nothing
armed. The bench drives a random `length` in every cycle without a start and a random `in_bit`
when not valid, and requires every octet and end in its cycle with its fields and nothing in
any other cycle.

```
make -C hw rx_octets
rx_octets G3: 167 segments, 216760 cycles, 13326 octets, 160 ends, 0 mismatches
```

The stub (ports only) failed first: 27 256 mismatches. `make -C hw lint` clean.

**Mutants** (35 on the first RTL, each in a private copy of `hw/`): 30 failed; the five that
passed were all equivalent and each pointed at logic that could not act — the `!start && !drop`
in `take` (shadowed by the if-chain), `zero` cleared at a start (always rewritten at the head's
seventh bit before it is read), the count's resets to 0 at SERVICE's end and at each octet (the
4-bit wrap does it), SERVICE's shift starting at bit 7 (the extra bit is shifted out): removed.
On the RTL as committed, 34 mutants (taps, head length, shift sources, octet and SERVICE order,
each seed XOR and their order, the zero test, the counts' compares, the length taken, each
start clear, first/last/end flags, disarm at the end, drop and start priorities, a bit with a
start, take while not armed): **34 of 34 fail** — one (start wins over drop) only after the
vectors gained the start-and-drop cycle. Script: the session's scratchpad (`mutate_octets.py`),
not in the repo.

Commit and command: this branch (`r0-09-octets`), `make -C hw rx_octets`, `cargo test --release
-p radio-phy --test g0_rx_octets --test g1_annex_i1_hashes`, `cargo test -p radio-phy --lib
rx_octets`.

### Open

- PR C: `rx_data.sv` wires this block behind `viterbi.sv` (the data block's bits), `start` on
  the verdict when decoded, `drop` with the packet's end by a mark; the MAC's `rx_end` merges
  this block's two ends with the top level's.
- Through the open ASIC flow with the data path's next run (small: about 60 flops).
