# Gate journal: `rx_data` (the data path's top level)

Block: the top level of R0-09's data path behind `rx_handoff`, ADR 0010 decision 6.2 (last
item: "the top level behind `rx_handoff`, a cycle at a time against the schedule model") with
decision 5's interfaces. Models: `radio_phy::rx_data_fixed::RxDataFixed` for *what* comes out
(gated in `g2_rx_data_stream`), `radio_phy::rx_data::RxDataSchedule` for *when* (gate
`g2_rx_data_schedule`). RTL: `hw/rtl/rx_data.sv` (PR C, 2026-11-25). Gate: `make -C hw rx_data` (vectors
`rvec rx_data`, bench `hw/tb/tb_rx_data.cpp`).

## 2026-11-24 · G0 · the interfaces, the wiring, the reader, and the schedule restated from the RTL

### The founder's decisions (2026-11-24, each asked on its own; the recommendation taken in each)

1. **Three PRs, in order**: (A) this G0 and the schedule model restated from the RTL as
   built; (B) the descrambler and the octets as a block of their own with its G3; (C)
   `rx_data.sv` with its G3 at the pins against the restated schedule.
2. **The time mark of ADR 0010 decision 5 is an age**: `rx_end` carries the number of ADC
   samples since the frame's last sample was at the ADC's pins (never negative: the frame is
   decoded after its last sample). The MAC stamps `rx_end` with its own clock and subtracts;
   no free-running counter is exposed and no time base is shared. The data path does not know
   the antenna's time: `rx_handoff` gains a free-running sample count and the count of the
   stream's first sample with its first beat (a small change to it, with its gate, in PR C).
   The hold's fall is not a time mark: for a short frame the verdict comes after the frame's
   end at the antenna, and the hold falls then.
3. **T1 → T2 → SIGNAL chained** in `fft64` (`in_next` on T1's and T2's 64th sample): SIGNAL's
   window does not wait for T2's flush. While the reader is behind the antenna (the SIFS case)
   the verdict and the last octet come 48 clocks earlier; at the antenna's pace (8 clocks a
   sample) T2's last bin comes with SIGNAL's sample 62, and the channel is ready before
   SIGNAL's last bin, so SIGNAL does not wait for it either way. The data windows are flushed (each one's
   place waits for the last θ).

### What the top level holds besides the blocks

- **The reader**: `out_ready` to the hand-off is a register (ADR 0010 decision 2); a window's
  samples go to `fft64` through one register (its `in_ready` is for a registered valid; the
  skid of one is never used: the reader waits for the flush itself). Rules in
  `radio_phy::rx_data`'s text ("The reader"): samples before a window's old place less one are
  taken freely; from there on the reader waits for the last θ (+ 2: the ready is registered),
  for the verdict from data symbol 2 on (it learns the number of symbols there), and for the
  window's first sample also for the FFT's flush (65 cycles after the last flushed window's
  64th sample) and for `rx_symbol`'s slot (`bins_ok`, 43 after the first soft value of the
  symbol two before). After the frame's last window, or a failed SIGNAL, the tail is taken and
  dropped. After a mark nothing is taken until the packet's last event is out.
- **The routing of the bins**: T1's and T2's to `rx_channel` (`bin_t2`), the symbols' to
  `rx_symbol`, by a count of windows.
- **The packet's start**, registered from the first beat: `rx_track`'s `start` with the CFO
  word and the fs/fc register (an input written by the MAC when it tunes; 0: none), SIGNAL's
  block to `rx_deint` (rate 1/2, 24 steps), the tag kept for the verdict.
- **The shared units**: `cordic_share` (a = `rx_symbol`, b = `rx_track`) before `cordic_iter`;
  `div_share` (a = `rx_channel`, b = `rx_track`) before `div_serial`.
- **The quotients**: `rx_channel`'s `q_valid` to `rx_track` as they pass, and job 0 (BPSK) and
  4 + m (modulation m) to `rx_symbol`'s `scale_valid`.
- **At the verdict** (`rx_signal`'s `sig_valid`, wired directly): to `rx_handoff` with the
  tag; when it decoded, the data block to `rx_deint` (`sig_steps`, `sig_code`), the data's
  modulation to `rx_symbol` (`mod_valid`), and the data's scale request to `rx_channel` unless
  the data is BPSK (its scale is already in: `rx_symbol` keeps one per modulation); `rx_start`
  to the MAC.
- **The drop**, to every block with the packet's last event (its `rx_end`).
- **The MAC's side** (ADR 0010 decision 5): `rx_start` with rate and length; the octets with a
  valid, a last, and on the first the seed and SERVICE (no ready: a bit a clock at most);
  `rx_end` with a status (decoded; SIGNAL failed and which check; a seed of zero; the stream
  ended before the frame; aborted by a mark and its kind) and the age. The octets themselves
  are PR B's block.

### Which allowance became which latency

The schedule model's `DataTiming` held allowances for blocks not yet built. Restated from the
blocks' headers, each checked by the block's own bench (the model's text lists every rule;
counts here are cycle to cycle, as the old allowances were):

| Old allowance | Was | Now | From |
|---|---|---|---|
| `fft`, `fft_read`, `window_stores` | 96, 64, 2 stores | a streaming transform: bin j of a flushed window 14 + j after its 64th sample (77 for the last), of a chained one 14 after the next window's sample j − 1; the next window 65 after a flush; no store | `fft64.md` (75 edges) |
| `channel` | 110 | 63 + 1 after T2's last bin | `rx_channel.md` |
| divisions | the quotient bits + 2, from the channel | `rx_channel`'s own: 21, 37, 53, 67 after chan_valid; a request 3 + q; the tracker's 27 | `rx_channel.md`, `rx_track.md` |
| `product` (pilots, pair products) | 3, the pair products on the products unit | pil_valid 9 + 1 after the last of its triggers; the pair products are the tracker's | `rx_symbol.md`, `rx_track.md` |
| `cordic` | 18 | 18 a take, the angle 17 + 1 after | `cordic_iter` |
| `fit_in`, `after_fit` | 5, 2 | the tracker's timeline (T, λ, G₀, G₁; the division asked on G₁ + 10, θ 5 + 1 after its done): θ 87 after the pilots with every unit free | `rx_track.md` |
| `rotator` + 1 (φ asked) | 4 + 1 | 10 + 1 after the last of: the products written, θ, the last soft pass read | `rx_symbol.md` |
| `demap` | 9 | the first soft value 8 + 1 after the last of: φ, the scale, the data's modulation | `rx_symbol.md` |
| `soft_products`, `soft_symbols` | 6 a clock, 2 | a subcarrier a clock; `rx_deint`'s two banks | `rx_symbol.md`, `rx_deint.md` |
| (steps from L + 1) | 1 | 2 + 1 after the last soft value | `rx_deint.md` |
| `viterbi_out` | 3 | 4 + 1; the flush from the edge after the last regular bit | `viterbi.md` |
| `signal_check` | 30 | 29 + 1 | `rx_signal.md` |
| `octet` | 1 | 1 (PR B: `rx_octets.sv`, the octet on the edge that takes its last bit) | `rx_octets.md` |

### Changes to the models

- **A seed of zero** (ADR 0010 Context 3, rule 1, settled in `RxDataFixed`, 2026-11-23): still
  no octet and the end `Seed`, but the end now comes **where the last octet would have been**,
  the frame read to its last window. The RTL's reader learns of a zero seed only from the
  decoder's seventh bit, hundreds of clocks after the model's window that decided it, and
  would have to be modelled taking windows the model had already given up; reading the frame
  to its end makes both the same, and costs nothing that matters (the MAC drops such a frame).
  `g2_rx_data_stream` no longer pads the stream of such a frame.
- `RxDataFixed::measures`: whether the frame's tracker constants are usable (the tracker's
  timeline differs when it measures nothing).
- `RxDataSchedule::violations`: the blocks' contracts the schedule would break — the CORDIC's
  requests out of the order the schedule assumes, the data scale still waiting at a second fit,
  pilots before `rx_track`'s one register is free, a soft pass into `rx_deint`'s full slot
  (`rx_symbol` has no hold: the margin is a symbol's work), the Viterbi decoder's contract
  between SIGNAL and the data, a first beat with no mark before it. None on any run of the
  gate. Some are a block's own contract (the slot, the pilot register, the decoder's), the
  others the schedule's assumptions (the CORDIC's order, the data's scale after the
  quotients); PR C's RTL is to assert those it can see.

### Evidence

`cargo test --release -p radio-phy --test g2_rx_data_schedule` (6 tests, green):
- **The table of ADR 0010 worked by hand** (24 octets at 54 Mb/s, one data symbol, a beat a
  cycle; the module's text has it row by row): every window, division, symbol event and MAC
  event in the edge the rules give. The verdict is seen in cycle **482** after the first beat
  (was 509 on the allowances), the last octet and the end in **905** (was 922).
- **240 clocks a symbol** at every rate on frames of 1500 octets (was 245): from one θ to the
  next, the sample before the window (θ + 2) and the window (1 + 63), the transform's last bin
  (77), the pilots (9), the step's start (2), the step with its division (86). Each data
  window's first sample waits for the last θ and the flush; BPSK's data asks no second scale.
- **With the front** (`run_handoff` with this reader, every rate, 1, 24 and 300 octets, ±40
  ppm): every event the model's on the beats taken, every strobe the hand-off saw the verdict
  in its cycle, no overrun, no violation. The verdict at most **7.31 µs** after SIGNAL's end at
  the ADC's pins (was 7.2 to 7.35), the last octet **1.74 to 5.95 µs** after the frame's (the
  one-symbol frame at 54 Mb/s was 6.06): ADR 0010's 6.9 to 7.5 and 5.6 to 6.2 hold.
- A train of frames behind a long one, a stronger frame inside the data, a stream cut after a
  window: decoded, aborted in order, the end after the octets of the windows taken.

The hand table is the only check of the rules' *values* here; the rules' *source* is the
blocks' benches, and the cycle-for-cycle check is PR C's gate at the RTL's pins. A model gate:
no reference from outside the project.

**The gate's mutants** (`rx_data.rs` at b194186, each against `cargo test --release -p
radio-phy --test g2_rx_data_schedule`, a script that copies the file back and touches it):
34 mutants: every latency constant moved by one or more, T2 flushed instead of chained, the
divider's order forced each way, the reader's θ + 1, `free_below` at the window's start, the
verdict's wait for data symbol 2 removed, the end by a mark at the last octet, the tail one
cycle early, `rx_deint`'s "previous step" removed, BPSK's data asking a second scale. The first
run killed 27; two of the survivors showed the gate too loose and two assertions were added
(the tail's first waiting beat taken in cycle 483, the end by a mark exactly 7 after the last
step of the windows taken, after their last octet): **29 of 34 killed**. The five left are
rules that never act at these latencies, each by a margin the rules themselves give:
- `rx_symbol`'s slot (41 → 60): a window's first sample waits for θ of the symbol before,
  about 240 clocks after θ of the one before that, whose first soft value is about 37 after
  its θ: the slot is free some 160 clocks before it is asked for;
- λ (C = T + 14 → 30): G₀ = max(C + 7, the outer angle + 2) and the outer angle is taken no
  earlier than T + 7, so T + 14 wins only above T + 19; on SIGNAL the variance's quotient is
  what C waits for (the table);
- the divider's order between the data's scale and the first fit (fit first forced): either
  way the data's soft pass waits for φ, 29 clocks after θ, while the scale is taken at most 18
  after θ; `div_share`'s priority is gated by its own bench;
- the verdict before data symbol 2's window (removed): θ of data symbol 1 comes 28
  clocks after the verdict in the table's case (its window waits for SIGNAL's flush, 65 + 63 + 77 + 9 + 2 + 86 after SIGNAL's last sample against the verdict's 274).
  **Corrected 2026-11-25** (PR C's review): only for a frame that measures its timing; one
  that does not has θ on G₁ + 3, before the verdict, and data window 2 then waits for the
  verdict itself. The schedule's gate has no such frame; `make -C hw rx_data` does
  (`no-measure-*`), and there the rule binds;
- `rx_deint`'s "previous step" (removed): two symbols' first steps are at least 240 clocks
  apart, a symbol at most 216 steps.
PR C's RTL is to keep every one of these rules; the margins are restated if a block's
latency changes.

### Review (independent, 2026-11-24)

A fresh agent checked every rule against the blocks' RTL and benches (the conventions of each
bench converted), re-derived the hand table and the 240 clocks a symbol independently, and
found no wrong latency rule. Taken:
- **an abort mark after SIGNAL's window and before its verdict** made the end wait for the
  verdict (the model's events keep their order), which the text did not say: the rule is now
  written (`rx_data.rs`, "An end by a mark": the verdict and `rx_start` come as without the
  mark, the end in `rx_start`'s cycle at the earliest; events of one cycle in the model's
  order) and gated (`a_mark_after_signals_window_waits_for_the_verdict`);
- **the seed-zero end's position was not gated** (its mutant "one octet early" passed both
  gates; no frame of the schedule's gate has a zero seed, and `build_ppdu` cannot make one):
  a unit test feeds the bits (`rx_data_fixed::tests`), the Seed end on the bit Decoded would
  come on; the mutant fails it. Noted: a zero-seed frame cut short now ends Truncated (or
  Aborted), not Seed;
- the data's scale asked during the quotients would be served 2 edges later by
  `rx_channel` than the model says: unreachable (the verdict is ~130 clocks after the
  variance), now a violation;
- `cordic_share.sv`'s header said SIGNAL's phase comes before its pair angles: false in this
  schedule (taken 302, 320, then 352); comment corrected;
- the drop after a failed SIGNAL must also clear the top level's register in front of the
  FFT (a sample taken in V + 1 would otherwise enter after the drop): in the model's text;
- the journal's claims: the mutants' command and commit, "the RTL stops its simulation"
  (no RTL yet), the channel's time at the antenna's pace — corrected;
- the symbols' list filtered by tag (two packets may share one): by the packet's first entry.

### Open, for PR B and PR C

- ~~The octet's latency and the seed and SERVICE with the first octet: PR B.~~ Done
  2026-11-24 (`docs/gates/rx_octets.md`): 1 edge, as the allowance; no schedule value
  changed.
- The age on `rx_end`: `rx_handoff`'s free-running count and the stream's first sample's
  count, the model's side (the schedule knows only clocks: the count comes from
  `run_handoff`), PR C.
- The reader's registered ready into `rx_handoff`'s tick (ADR 0010 decision 2's untimed path)
  goes through timing with the front and the data path together.

## 2026-11-25 · G3 · `hw/rtl/rx_data.sv` at its pins, a cycle at a time against the schedule

PR C of the three (the founder's decision 1 above). Not merged: left for the founder with
the PRs stacked on it.

### What was built

`hw/rtl/rx_data.sv` wires every block of the data path as "What the top level holds" says:
`fft64` behind the reader's register (T1 and T2 with `in_next`), the bins routed by a count of
output windows (T1, T2 to `rx_channel`, the symbols to `rx_symbol`), `rx_track` started from the
first beat (CFO word, the MAC's `ratio`), `cordic_share` (a `rx_symbol`, b `rx_track`) before
`cordic_iter`, `div_share` (a `rx_channel`, b `rx_track`) before `div_serial`, the quotients to
`rx_track` and the scales (job 0, 4 + m) to `rx_symbol`, `rx_deint` with SIGNAL's block from the
first beat and the data's at the verdict, `viterbi` → `rx_signal` and `rx_octets` (PR B). The
verdict goes straight out to `rx_handoff` (`sig_*`, the stream's tag), `rx_start` is
`rx_signal`'s `sig_valid` when it decoded, the octets are `rx_octets`', and `rx_end` merges the
four ends (decoded or seed zero from `rx_octets`, SIGNAL failed from `rx_signal`, truncated or
aborted by a mark from this level). The drop to every block is `rx_end` itself; it also
clears the register in front of the FFT.

The reader's `in_ready` is a register computed from the state after each edge (the model's "a
beat that waits for an output registered on E is taken in E + 2"). Its rules are the model's,
each from a signal the RTL sees: θ by counting `theta_valid` (θ0 the first), σ by
`move_early`/`move_late`, the verdict by `sig_valid`, the FFT's flush by a counter of 64 from
a flushed window's 64th sample, the slot by `rx_symbol`'s `bins_ok`, the frame's last window by
a count of data windows against `sig_symbols` (0 when SIGNAL failed: the same rule drains the
tail). The end by a mark: at once before SIGNAL's window is complete; otherwise with the
verdict (in `rx_start`'s own cycle when no data window is complete, from `sig_valid` itself),
when the frame is not over (complete data windows < symbols), and once the trellis steps of
the complete windows (each a full N_DBPS) have been counted and 5 more edges have passed: the
model's last step + 7. After a mark `in_ready` stays low until the packet's end is out.

Not here: the age on `rx_end` (the founder's decision 2): it needs `rx_handoff`'s sample
count, next PR.

### The gate

`make -C hw rx_data`: vectors `rvec rx_data` (`crates/radio-tools/src/rx_data_vec.rs`), bench
`hw/tb/tb_rx_data.cpp`. The streams are the front's own (`RxHandoffFixed` on frames through
AWGN at 30 dB, some with a clock offset), the cycles `RxDataSchedule`'s with the beats offered a
beat a cycle or at a pace (8 clocks a sample: the antenna's; 5 for one case); the bench resets
the block per run, offers beat i from max(its pace, the cycle after beat i − 1), and requires
every beat taken in the model's cycle, every verdict strobe, start, octet (data, first, last,
seed, SERVICE) and end (status, info) in its cycle, and nothing in any other.

55 runs: ADR 0010's table; every rate at 1 and 100 octets a beat a cycle and at 24 octets at
the antenna's pace; 1500 octets at 6 Mb/s with ±40 ppm, at 54 Mb/s −40 and 24 Mb/s +40; no
ratio written; a CFO word forged under HaLow's fs/fc ratio (θ0 moves the windows: SIGNAL's
stays; at ±2^20 over 100 symbols the shift runs to tens of samples; windows moved early 450
times, late 450); frames that do not measure their timing (θ of data symbol 1 before the
verdict: a search over SNR near 1 dB, the generator requires it); marks cutting a frame in
T2, in SIGNAL's window, after data window 1 and before the verdict (a beat a cycle and every
2 clocks), after SIGNAL's window before the verdict, inside the data, and between the last
window and the frame's end (before and after the verdict), of each kind, each followed by a
frame that decodes; SIGNAL failed on noise (one run takes a beat in the drop's own cycle; the
generator requires it), and noise that passed SIGNAL (truncated by its mark); data window 1
complete in the verdict's own cycle and a mark after it (a search of two paces and a gap,
required); two frames back to back. Not at this level: the seed-zero end (status 1; no frame
`build_ppdu` makes has one: `rx_octets`' own gate has it).

```
make -C hw rx_data
rx_data G3: 55 runs, 565949 cycles, 185633 beats, 9900 events, 0 mismatches
```

First run: 414 mismatches (the reader not ready in its first cycle after reset). The HaLow
runs found a rule the RTL had wrong: θ0's move had been applied to SIGNAL's own window; the
model places SIGNAL at 144 and moves from data symbol 1 on (its text said so: σ₀ = 0). Fixed.
`make -C hw lint` clean. The RTL stops its simulation on a first beat with no mark before it
and on a sample into the FFT's skid (the schedule's violations it can see; the blocks' own
assertions stop it on the rest: `rx_deint`'s full slot, `rx_channel`'s, `rx_symbol`'s).

### Review (independent, 2026-11-25)

The reviewer reproduced the gate and the vectors (byte-identical), swept marks and paces with
its own generator (about 2000 runs) and found:
- **a deadlock** (fixed): a mark taken after data window 1 and before the verdict (one beat
  a cycle, beats 288 … 302 of a 6 Mb/s frame; paces of 1 to 3 clocks) left the verdict
  unrecorded (it was taken only while the packet was open), so the end by the mark never
  came and `in_ready` stayed low until a reset. The verdict is now taken while a mark waits
  too. The gate's cuts had been at 100, 180, 218, 414 and 688; two runs now cut at 295 and 290;
  both failed on the old RTL (215 mismatches each);
- **σ wrapped at ±8** (fixed): it was 4 bits, the model's shift has no bound but the frame's:
  θ0's move and one a data symbol at most, |σ| ≤ 1 + 1366, so σ is 12 bits (G0, below). A
  6 Mb/s frame of 4095 octets at +70 ppm reaches +8; HaLow's ratio with a forged word ±2^20
  reaches ±45. Both forged runs are now in the gate and failed on the old RTL;
- **a false argument** in this journal (G0, the schedule's survivors, and the first version
  of this entry): "θ of data symbol 1 comes after the verdict" holds only for a frame that
  measures its timing; one that does not has θ on G₁ + 3, before the verdict, and data
  window 2 then waits for the verdict itself. The RTL had it right; the gate did not reach
  it. The runs that do not measure now make the mutant fail;
- the `ifndef SYNTHESIS` comment named a check this level does not make: corrected. Everything
  else held: the wiring against the blocks' headers, the anchor argument below, θ never in a
  window's last cycle, no vacuous pass.

**G0, added**: σ's bound. A move is ±1 with each θ, θ0's and one a data symbol: |σ| ≤ 1 +
1366 < 2^11. 12 bits signed.

**Mutants** (each in a private copy of `hw/`; script in the session's scratchpad): 44 on the
RTL as committed — every reader rule moved by one (free beats, θ, the verdict, the flush 63 and
65, the slot, the frame's last window, the symbol's 80 and SIGNAL's 144 and σ on SIGNAL, σ's
sign, each move, σ at 4 bits), the routing (`bin_t2`, the bins' window count, `in_next` on T1
and on SIGNAL), each end rule (the 5 edges ±1, the end with `rx_start`, the verdict only while
open, the status of the kinds, the ready after a mark at once and a cycle late, the expected
steps at the verdict, per window and before the verdict's own update), the drop's reach (the
FFT's register, all ends), the first beat's tag and the start's edge: **38 of 44 fail**. Four
of them only after the vectors grew (the expected steps before the verdict's update, the
verdict wait, the verdict only while open, σ at 4 bits). The six that pass:
- two rules the schedule's own gate cannot bind either (G0, "The gate's mutants"): the slot
  (free ~160 clocks before it is asked for), BPSK's second scale (done before the fit needs
  the divider);
- four that cannot act: the free beats with θ's move of the same cycle (θ never comes in a
  window's last cycle) or with θ0's (θ0 is ~140 beats early); the end by a mark before
  SIGNAL's window waiting for the steps (none exist yet); the end by a mark after the frame's
  last window (the count of its steps reaches N_DBPS·symbols, never the anchor 16 + 8·L + 6,
  which is ≡ 6 mod 8 while every N_DBPS but 36 is a multiple of 8, and 36·k mod 8 is 0 or 4).
One more was a dead branch, removed (`start` of SIGNAL in the ready rule).

### Open

- The age on `rx_end`: decided 2026-11-25 (below); being built.
- `rx_data` through the open ASIC flow (area and timing of the whole data path against ADR
  0010's 1.5 to 2.0 mm²).

## 2026-11-25 · G0 (a proposal, waiting for the founder) · the age on `rx_end`

The founder's decision 2 of 2026-11-24 (above): `rx_end` carries an **age**, the ADC samples
since the frame's last sample was at the ADC's pins; the MAC stamps `rx_end` with its own clock
and subtracts. `rx_handoff` gains a free-running sample count and the count of the stream's
first sample with its first beat. PR C left it out; this entry is its G0. No RTL is written
until the questions below are answered, one at a time, each with a recommendation.

### What is fixed already

- **The frame's end**, in the stream's samples: `RxDataEvent::Start`'s `frame_end` =
  `symbol_start(0, symbols) + 64` = 144 + 80·symbols + 64, one past the last data symbol's
  FFT window, counted from T1's first window sample; known at the verdict. That stream starts
  BACKOFF samples inside T1's cyclic prefix, so this is BACKOFF samples (up to 16) earlier than
  the frame's true last sample at the pins: the age must add BACKOFF, as `rx_handoff`'s own
  frame end already does (`sig_hend` = `SIGNAL_END` + BACKOFF + 80·symbols,
  `rx_handoff.sv:237`). (The doc comment of `rx_data_fixed.rs:93`, "at the antenna", has the
  same slip and is corrected with the model's change.) (Whether "the frame's last
  sample" should be that, or the window's end moved by the tracker's σ, is part of
  question 3's precision: σ is a few samples at most.)
- **Where the count comes from**: `rx_handoff` writes ADC sample n at history address
  n mod 512 (`HISTORY`), and the front's `acq_fft` is the history address of T1's first
  sample. A free-running count c of ADC samples written gives that sample's full count as
  c − ((c − `acq_fft`) mod 512) at the declaration's tick, carried with the first beat
  (`out_first_n`).
- **The age at `rx_end`**: (the count as `rx_end` is seen) − (`out_first_n` + `frame_end`).
  In `rx_data.sv`: an adder at the verdict (the frame's end in samples), a subtractor at the
  end; `rx_handoff`'s count is an input.
- **Its range for a decoded frame**: the reader is at most a history behind the pins (512
  samples: further is an overrun, a mark), and `g2_rx_data_schedule` asserts the end within
  7 µs (140 samples) of the frame's end at the ADC's pins (it prints 1.74 to 5.95 µs, measured
  from the pins, not from the reader's read): the age < 512 + 140 < 2^10. Counted modulo 2^N,
  an unsigned age is exact for any N ≥ 10; the signed age of question 1(b) needs N ≥ 11.
- **What `rx_handoff` keeps already** (the review, 2026-11-25): `wr_cnt`, samples written
  modulo 512 (`rx_handoff.sv:103`); `wr_off`, a 17-bit count of samples written since T1's
  window (`:156`); `hend`, one past the frame's last sample (`:159`, `:237`). So an
  alternative to the plan below is the age as `wr_off − hend` inside `rx_handoff`, with no new
  `HandoffStart::at` and no `out_first_n`. The catch: `wr_off` belongs to the open packet and
  restarts when a new acquisition replaces it (an abort does), so it would have to be latched
  for `rx_end` or `rx_end` read before the restart. The free-running count of the plan below
  could simply be `wr_cnt` widened. Both bear on questions 2 and 4.

### The questions for the founder

1. **Which ends carry an age.** (a) *Recommended:* only the decoded end and the seed-zero end
   (the frame's end is known and is in the past; these are the ones the MAC times an ACK or
   a CCA busy period by); every other end gives `rx_age_valid` = 0. (b) Every end after
   `rx_start` (truncated and aborted too): the frame's end is known from SIGNAL but may lie
   in the future (an abort mid-frame), so the age would be signed, or clamped at 0.
   (c) Every end, the ends without SIGNAL timed from the stream's last sample taken.
2. **The width.** *Recommended:* a 16-bit count in `rx_handoff` and a 16-bit age (the bound
   above needs 10 unsigned, 11 signed; 16 leaves room for a larger history or a slower reader without a change of
   interface), wrapping, no saturation needed.
3. **"Now"**: *recommended:* the count of ADC samples written before `rx_end`'s cycle (a
   register in `rx_handoff`, read by `rx_data` combinationally in that cycle); the MAC's
   stamp is of the same cycle. The alternative, the count at the end's register edge, is a
   cycle's difference (1/8 of a sample) and needs a second register.
4. **The model's side.** `HandoffStart` gains the first sample's ADC index (`at`); the
   schedule (`RxDataSchedule`) gets the ADC count per cycle from `run_handoff` (it knows when
   each sample was written) and gives the age with each end; `g2_rx_data_schedule` checks it
   against the frame's end at the antenna directly (`train`'s starts); `rvec rx_data` carries
   the count and the first sample's count in its beats. `rx_handoff`'s gates (`handoff`,
   `handoff_forced`) gain the count and `out_first_n`.

### What changes where (once answered)

| Where | What |
|---|---|
| `radio_phy::rx_front_fixed` | `HandoffStart::at` (9 places build it) |
| `radio_phy::rx_handoff` | `run_handoff` gives the count per cycle |
| `radio_phy::rx_data` | the age with every end that carries one |
| `hw/rtl/rx_handoff.sv` | the count (N bits; `W` is the back-off parameter), `out_first_n` |
| `hw/rtl/rx_data.sv` | `in_first_n`, `adc_count` in; `rx_age`, `rx_age_valid` out |
| gates | `handoff`, `handoff_forced`, `rx_data`, `g2_rx_data_schedule`, `g2_rx_handoff`, `g2_rx_handoff_schedule`, `g2_rx_data_stream` rerun; `hw/vectors/handoff.txt` and `rx_data.txt` regenerated |

## 2026-11-25 · G0 (decided) · the age on `rx_end`: the founder's answers

The founder answered the four questions above on 2026-11-25, each with the recommendation, and
on question 2 chose the stamp in `rx_handoff` that the review had opened up over the plan's
`out_first_n`.

1. **Which ends.** Only the decoded end (status 0) and the seed-zero end (status 1) carry an
   age; every other end gives `rx_age_valid` = 0.
2. **Where, and how wide.** `rx_handoff` keeps a free-running 16-bit count of the ADC samples
   written (`adc_count`) and a 16-bit **stamp** of the frame's end in that count
   (`end_stamp`). `rx_data` gives `rx_age` = `adc_count` − `end_stamp`, modulo 2^16.
3. **"Now".** The count of samples written before `rx_end`'s cycle: `adc_count` as a
   register's output in that cycle, read combinationally. No second register.
4. **The model.** The frame's nominal end, from SIGNAL, without the tracker's σ.
   `run_handoff` gives the count and the stamp; the age is a function of the end, the count
   and the stamp; `g2_rx_data_schedule` holds the age of every decoded frame to the frame's
   true end at the antenna (`train`'s starts), within the timing estimate and the drift.

**The stamp, as built.** The frame's end is known from the verdict on, and at the verdict it
may already be in the past: a one-symbol frame ends 80 samples after SIGNAL, the verdict comes
about 7 µs (140 samples) after SIGNAL. So the stamp is not taken when the frame's last sample
is written. It is computed when the verdict is applied, at the tick where `hend` takes the
frame's end (`len`):

    end_stamp ← (adc_count − 1 − wr_off) + hend1        (mod 2^16)

`adc_count − 1 − wr_off` is the count of T1's first window sample: both registers step on
`in_valid` and `wr_off` counts the newest sample from that one, so their difference is fixed
for the packet. `hend1` is `SIGNAL_END` + BACKOFF + 80·symbols, the model's `packet_end`, so
BACKOFF is in (the slip the review found in the proposal). In the model the stamp is
`packet_end(fft_start, symbols, backoff)` in ADC samples, mod 2^16, a register's output from
the cycle after that tick; the count in cycle c is the number of samples n with X(n) < c. A
frame's last sample just written gives an age of 0.

**Why a later packet cannot move the stamp first.** The stamp changes only when a verdict is
applied, and verdicts come from `rx_data`, which decodes one packet after another: the next
packet's verdict leaves `rx_data` after the present packet's `rx_end` (the Viterbi decoder
is shared and in order), and is applied at a tick after that. The gate checks it: the age of
every decoded frame against its true end.

The plan's `HandoffStart::at`, `out_first_n` and the adder of `frame_end` in `rx_data` are
dropped.

| Where | What |
|---|---|
| `radio_phy::rx_handoff` | `HandoffRun`: the count per cycle, the stamps |
| `radio_phy::rx_data` | the age of an end from the count and the stamp; the ages of a closed-loop run |
| `hw/rtl/rx_handoff.sv` | `adc_count`, `end_stamp` out |
| `hw/rtl/rx_data.sv` | `adc_count`, `end_stamp` in; `rx_age`, `rx_age_valid` out |
| gates | `g2_rx_data_schedule` (the age against the truth), `handoff` and `handoff_forced` (the stamp and the count), `rx_data` (the age at the pins); vectors regenerated |

## 2026-11-25 · G3 · the age on `rx_end` (`rx_handoff`'s count and stamp, `rx_data`'s age)

**What was built** (per the decided G0 above):
- `hw/rtl/rx_handoff.sv`: `adc_count` (16 bits, a step per `in_valid`) and `end_stamp`,
  registered at the tick where `len` applies a verdict with a length:
  `adc_count − 1 − wr_off + slot_hend`, mod 2^16. Both reset to 0.
- `hw/rtl/rx_data.sv`: `adc_count` and `end_stamp` in; `rx_age = adc_count − end_stamp` and
  `rx_age_valid = rx_end && rx_end_status ≤ 1`, combinational in `rx_end`'s cycle.
- The model: `HandoffRun::{written, stamps, count_before, stamp_at}` (`rx_handoff.rs`),
  `rx_end_age` and `rx_end_ages` (`rx_data.rs`); `ppdu::build_ppdu_bits`, a PPDU from DATA
  bits as sent, for a frame with a seed of 0 (`build_ppdu` refuses one).

**Gates** (branch `r0-09-age-rtl`; the commit of this entry):

| Gate | Command | Result |
|---|---|---|
| The age against the frame's true end (model, closed loop) | `cargo test -p radio-phy --test g2_rx_data_schedule` | every rate × 1, 24, 300, 1500 octets × −40, 0, +40 ppm: error 0 samples on all 96; a train of 10 frames at the shortest gap: 0 on each; a seed of 0 at 6 and 54 Mb/s: the seed-zero end with its age, exact; every other end without one |
| Which ends, the wrap | `cargo test -p radio-phy --lib rx_data` | the decoded and seed-zero ends only; 5 − 65 530 = 11 |
| `rx_handoff`'s count and stamp | `make -C hw handoff` | 0 mismatches in every configuration (R 8, 11, 16; W 4, 8, 12, 16); at R = 8, W = 12: 180 stamps compared in 4 978 336 cycles, the count past 2^16 in 6 plays (segment `count_wraps`: a stamp of 4164 after the wrap) |
| The forced declarations | `make -C hw handoff_forced` | 240 cases, 0 differ |
| `rx_data`'s age at its pins | `make -C hw rx_data` | 57 runs, 570 085 cycles, 58 ages (2 seed-zero, 18 whose subtraction wraps), 0 mismatches |
| Lint | `make -C hw lint` | clean |

**The tolerance is 0.** At 30 dB the acquisition finds T1's window exactly on every run, so
the nominal end from SIGNAL is the transmitter's last sample and any difference is a defect.
The sample clock's drift (up to 1.6 samples over a 1500-octet frame at 6 Mb/s and 40 ppm) is
not in the age, by decision 4, and not in the gate's truth (the transmitter's samples).

**Mutants** (each in a private copy; the gate named must fail):

| Mutant | Gate | Result |
|---|---|---|
| model: stamp one sample early (`hold_end − 1`) | `g2_rx_data_schedule` | dies |
| model: the count includes the end's own cycle (`w <= clock`) | `g2_rx_data_schedule` | dies |
| model: stamp registered a cycle later | `g2_rx_data_schedule` | **survives**: rx_end is hundreds of cycles after the stamp; the cycle is bound by `make -C hw handoff` (S lines, every cycle) |
| model: a truncated end carries an age / the seed end none / saturating subtraction | `--lib` unit test, and the seed-zero closed loop | die |
| RTL `rx_handoff`: stamp without the − 1 | `handoff` | dies (4 978 336) |
| RTL: the stamp from the old `hend` instead of `slot_hend` | `handoff` | dies |
| RTL: the stamp taken at every tick | `handoff` | dies |
| RTL: the count steps every cycle | `handoff` | dies |
| RTL: the stamp not reset | `handoff` | dies (84) |
| RTL `rx_data`: valid on status 0 only / on ≤ 2 / without `rx_end` | `rx_data` | die (2 / 4 / 570 011) |
| RTL: `end_stamp − adc_count` / one less | `rx_data` | die (58 / 58) |
| RTL: the stamp ignored (`rx_age = adc_count`) | `rx_data` | dies at lint (`end_stamp` unused, `-Wall`) |

(One more mutant, `wr_off[C_W-2:0]` for `CNT_W'(wr_off)`, is the same 16 bits: equivalent,
not counted.)

### Open

- The age leaves out the tracker's σ and the sample clock's drift (decision 4). If the MAC's
  SIFS timing needs better than the ±18 samples of the standard's ±900 ns, σ can be added
  in `rx_data` later without changing the interface.
