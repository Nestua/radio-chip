# Gate journal: `rx_signal`

Block: SIGNAL and the verdict, R0-09's data path, ADR 0010 decisions 1, 5 and 6.2 ("SIGNAL and
the verdict"): the field from the Viterbi decoder's bits, its checks, the check of its coded bits
against the field encoded again (`SignalCheck`), the verdict's strobe for the front
(`rx_handoff`'s `sig_*`), the start for the MAC, and the data block for `rx_deint`. Model:
`radio_phy::rx_ofdm::signal_checked`, `radio_phy::rx_front_fixed::SignalVerdict::of` with
`HandoffConfig::default()` (patient at 40 agreeing), `radio_phy::ppdu::data_symbols`; G1 of the
field's decode in `docs/gates/rx_ofdm.md` and `g1_signal_check.rs`. RTL: `hw/rtl/rx_signal.sv`,
with `hw/rtl/conv_encoder.sv` (R0-04's, gated on its own). Gate: `make -C hw rx_signal`, behind
`hw/rtl/viterbi.sv` (`hw/tb/signal_chain.sv`).

## 2026-11-24 · G0 · the specification and the architecture

**The stream from the decoder's flush: decided by the founder, 2026-11-24** (the
recommendation). The alternative was a parallel tap of state 0's survivor register in
`viterbi.sv` (all 24 bits a clock after the last step, the verdict some 24 clocks — 0.15 µs —
sooner), which would reopen a gated block for a margin the SIFS budget does not need (ADR 0010:
about 9 µs left).

### What the block computes (fixed by the model)

- SIGNAL is one BPSK symbol at rate 1/2: 24 trellis steps, nothing stolen. The decoder's window
  (96) is longer than the block, so its 24 bits are `decode`'s (`viterbi.md`, G3).
- The field (IEEE 802.11-2020 §17.3.4): RATE R1 … R4 in bits 0 … 3, the reserved bit 4 (not
  checked, §17.3.4.3), LENGTH in bits 5 … 16 LSB first, even parity over bits 0 … 17, the tail in
  18 … 23 (not checked). Checks in the model's order: parity, then the rate (Table 17-6's eight
  patterns), then the length (0 fails; 12 bits cannot exceed 4095).
- `SignalCheck`: the 24 bits encoded again; a coded value agrees when it is nonzero and its sign
  is the encoded bit's (negative for 1), disagrees when nonzero with the other sign; an erasure
  is in neither count.
- The verdict: data symbols ⌈(16 + 8·LENGTH + 6) / N_DBPS⌉ (never above 1366, so the model's
  clamps never act); patient at 6, 9 and 12 Mb/s with at least 40 agreeing. The data block:
  steps = the anchor 16 + 8·LENGTH + 6, the code rate and the modulation.

### The architecture

- SIGNAL's 24 pairs are kept as they pass from `rx_deint` to the decoder: per coded value its
  sign and whether it is 0 (a 24-deep shift register of 4 bits), the first 24 pairs after a drop.
- The decoder's bits are taken only once those 24 pairs are in, the first 24 of them: each is
  encoded again as it comes (`conv_encoder.sv`) and compared with its pair a clock later; bits
  0 … 17 go to the field and the parity.
- **The symbol count without a divider.** N_DBPS = 2^a·t with t ∈ {3, 9, 27}, so
  ⌈anchor / N_DBPS⌉ = ⌊z·M_t / 2^s_t⌋ with z = (anchor + N_DBPS − 1) >> a and
  (M, s) = (2731, 13), (3641, 15), (4855, 17) for t = 3, 9, 27. z is at most 8204 (9 Mb/s, 4095
  octets): **14 bits**; z·M below 2^26.

### The gate

`crates/radio-phy/tests/g0_rx_signal.rs` (`cargo test -p radio-phy --test g0_rx_signal`):
- `symbols_without_a_divider`: the rate table's (rate, a, t) against `Rate`'s own bits and
  N_DBPS; the count by shift and product against `data_symbols` at every rate and every length
  0 … 4095, never above `MAX_DATA_SYMBOLS`; z and z·M within the RTL's widths.
- `field_check_verdict`: 20 000 cases — valid fields at every rate with up to 11 sign flips and
  9 erasures, bad parity, random 4-bit rate patterns, lengths 0, 4095 and random ones, noise
  alone — the
  architecture (fed `decode`'s bits) against `signal_checked` and `SignalVerdict::of`: the
  field, the failure, the counts, the verdict, the steps. Each outcome (decoded, parity, rate,
  length) and both patience values more than 300 times.

**Seen to fail** (mutants of the architecture in the test, 2026-11-24): M 2731 → 2730; s 17 → 16;
R18's a; R9 read as R12; the agree test without the erasure guard; the field 17 bits; the length
from bits 4 … 15; length 4095 failing instead of 0; patience above 40 instead of at it; 18 Mb/s
patient — 10 of 12 fail. **Alive, both equivalent:** `v <= 0` for `v < 0` inside the guard
`v != 0`; and the ceiling with N_DBPS − 2 for N_DBPS − 1, because the anchor is never a multiple
of N_DBPS: it is 6 modulo 8, every N_DBPS but 36 is a multiple of 8, and 36 is a multiple of 4
while the anchor is 2 modulo 4. The test runs every length at every rate, so the mutant's output
is the same on the whole domain.

**Found by the RTL gate, then added to G0:** the RTL's first z was 13 bits; at 9 Mb/s and 4095
octets z = 8204 > 8191, and the gate showed 1 data symbol for 911. G0 computed in `usize` and
had no width to check; it now asserts z < 2^14 and z·M < 2^26, and with 13 bits it fails.

## 2026-11-24 · G3 · `hw/rtl/rx_signal.sv` behind `viterbi.sv`, every verdict in its edge

Commit: this branch (`r0-09-signal`). Command: `make -C hw rx_signal`. Vectors:
`hw/vectors/rx_signal.txt` (`rvec rx_signal`): 3000 cases (1596 decoded, 262 of them patient;
899 parity, 389 rate, 116 length failures), as in G0 plus random tails.

Result (after the review, below): **`rx_signal: 3000 cases (1596 decoded), 2284 verdicts, 65326
SIGNAL bits checked, 0 mismatches; cut among the pairs 119, in the flush 169, at the verdict 200,
resets 228 (64 among the pairs); 53 drops with a pair; 1733 data blocks, 898 drops inside their
flush, 4777 gaps` — PASS**; `scripts/check.sh` green.

The bench (`hw/tb/tb_rx_signal.cpp`, top `signal_chain`: the pairs into `viterbi.sv` and
`rx_signal` together, `rx_signal` on the decoder's own bits) checks:
- every verdict's fields against the vectors — the counts of agreeing and disagreeing coded
  bits also on a failure (`signal_field`, below), the frame's fields 0 then — and **its edge**: 29 edges after the edge that took SIGNAL's last pair (the
  flush's first bit registered 4 edges after, a bit an edge, the 24th taken an edge after it is
  registered, the verdict an edge later). No other `sig_valid`.
- SIGNAL's 24 decoded bits in their edges (the decoder's G3 again, on these blocks).
- The driving: SIGNAL's pairs back to back or with idle cycles; in three packets of four a data
  block of 1 … 300 random pairs from the decoder's earliest (24 + 2 edges after SIGNAL's last
  step) or later, which `rx_signal` must ignore; the drop that ends every packet, half the time
  inside the data block's flush (whose bits come while the next SIGNAL's pairs do); cut packets:
  a drop among SIGNAL's pairs (the decoder keeps a block that never ends; some with a pair in the
  drop's own cycle, which the decoder takes and the block must not), a reset there, in the flush (the next
  SIGNAL from the decoder's earliest, while the old flush's last bits still come), in the
  verdict's own edge, or a reset in the flush. Registers start random.

**Found by the gate before it passed:** z one bit short (above, G0); and my header's latency,
28 edges: a bit the decoder registers at edge E is taken at E + 1, which I had not counted. The
RTL was right; the header and the bench now say 29.

**Mutants** (2026-11-24, `make -C hw rx_signal` with `-Wno-fatal` on each; restored by copy and
`touch`): 27 of 28 fail on the bench, rerun on the bench as amended after the review (and
the review's own mutant, the counts 0 on a failure, fails too). The pair capture without "24 pairs in" (the old flush's bits
taken); the field 17 bits; no parity check; R18 at rate 2/3; R9's a; M 3641 → 3640; s 17 → 16;
N_DBPS 216 → 208; length 1 failing; agree with the sign test reversed or without the erasure
guard; patience above 40; 18 Mb/s patient; steps + 21; the verdict at the 23rd bit; the pair one
ahead; A and B swapped in the store; the field shifted the other way; a drop not clearing the
pair count, or the counts; rate checked before parity; a failed verdict with its steps; B
compared with A's bit; the encoder not cleared by a drop; 12 Mb/s as BPSK; the parity of the last
bit only; the verdict without the encoder's valid. **Alive:** the ceiling with N_DBPS − 2,
equivalent on the whole domain (G0, above).

### Independent review (fresh agent, 2026-11-24; mutating in a private copy only)

No defect found in the RTL (the rate table, the field, the patience, the counts, every width
and the reciprocals over the reachable z, drops and resets anywhere, stale flush bits — 22 edges
of margin before the next SIGNAL's 24th pair). Found and fixed:
- **The counts on a failed SIGNAL were promised and not checked**: the model formed them and
  dropped them with the error, and the mutant "the counts 0 on a failure" passed. The model now
  has `signal_field` (the field or its failed check, with the counts either way; `signal_checked`
  is built on it, its results unchanged — every `radio-phy` test passes), the vectors carry the
  counts for every case and the bench compares them; the mutant fails (1060 mismatches).
- **The schedule model contradicted the RTL** (`signal_check` 10 against 29): now 30 (cycle to
  cycle, one more than the RTL's edges after the taking edge), below.
- Bench paths added: a reset among the pairs, a drop with a pair in its cycle.
- The header states the drop at every packet's end including a failed SIGNAL and an abort, that
  a pair in the drop's cycle is not kept, and why a cut block's flush cannot be taken (it ends
  before the next SIGNAL's 24th pair, not before its first).
- Noted, not changed: one mutant (bits taken past the 24th) is killed by the block's own
  assertion rather than by the bench's comparison; the bench never puts gaps between the
  decoder's bits (they cannot have any behind `viterbi.sv`).

### Latency, against the schedule model

The verdict comes 29 edges after the edge that took SIGNAL's last trellis step: 4 + 23 for the
decoder's flush, 2 here. The schedule model allowed 10 cycles: it did not count the flush, a bit
a clock. **`DataTiming::signal_check` is now 30** (`rx_data`, cycle to cycle) and
`g2_rx_data_schedule` holds the hand-worked table with the verdict and the start at 509 after the
first beat (was 489; 670 after the declaration's tick); everything else in the table is
unchanged — the data's scale, asked at the verdict, still waits for the divider until 529.
`docs/gates/rx_ofdm.md` has the entry. Other paces than the table's were not examined.

### Open

- `DataTiming::viterbi_out` (3) against `viterbi.sv`'s LAT (4 edges, 5 cycle to cycle): the
  octets' cycles, with the top level.

- The tag of the verdict (`rx_handoff`'s `sig_tag`, the stream's first beat's) and the MAC's
  `rx_start` / `rx_end` are the top level's; this block gives the fields.
