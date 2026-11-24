# Gate journal: `rx_data` (the data path's top level)

Block: the top level of R0-09's data path behind `rx_handoff`, ADR 0010 decision 6.2 (last
item: "the top level behind `rx_handoff`, a cycle at a time against the schedule model") with
decision 5's interfaces. Models: `radio_phy::rx_data_fixed::RxDataFixed` for *what* comes out
(gated in `g2_rx_data_stream`), `radio_phy::rx_data::RxDataSchedule` for *when* (gate
`g2_rx_data_schedule`). RTL: `hw/rtl/rx_data.sv` (to come, PR C). Gate: `make -C hw rx_data`
(to come).

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
| `octet` | 1 | 1 (an allowance until PR B) | — |

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
  clocks after the verdict in the table's case (its window waits for SIGNAL's flush, 65 + 63 + 77 + 9 + 2 + 86 after SIGNAL's last sample against the verdict's 274);
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

- The octet's latency (`DataTiming::octet`, an allowance) and the seed and SERVICE with the
  first octet: PR B.
- The age on `rx_end`: `rx_handoff`'s free-running count and the stream's first sample's
  count, the model's side (the schedule knows only clocks: the count comes from
  `run_handoff`), PR C.
- The reader's registered ready into `rx_handoff`'s tick (ADR 0010 decision 2's untimed path)
  goes through timing with the front and the data path together.
