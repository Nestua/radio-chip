# ADR 0008 — Receiver front, amended: hand-off of samples, DC removal for the decoder, the gain's hold, latency, folding, area and power

Status: **proposed** · 2026-11-21 · PHY architecture (risk point). Amends ADR 0006; announced
by ADR 0007 (decision 2) and asked for by ADR 0009 (the count for one to eight chains, the
sample interface). The founder decides each item; the text is the proposal put to them.
An independent review of the first draft (fresh agent, 2026-11-21) changed decisions 2 to 6;
what it found is at the end.
Sources: `docs/gates/rx_front.md` (2026-11-21, G0 and results), `docs/gates/rx_ofdm.md`
(2026-11-20, stage B part 2), `docs/gates/silicon_probe.md`, the reports
`rx_front_dc_cfo.txt`, `rx_front_dc_cand.txt`, `rx_front_dc_hold.txt`, `rx_front_dc_edge.txt`,
`rx_ofdm_agc_*.txt`.

Notation as in ADR 0006: R clocks per sample (R = 8: 160 MHz at 20 MS/s), T1 the first sample
of the first long training symbol, L = 64 the decision latency. **Every cycle count, area
and power figure of something not yet built is an estimate**, made as ADR 0006's were, before
any RTL; the gates of the blocks measure the real ones.

Scope: the formats whose preamble is the 20 MHz legacy one (802.11a/g, and the legacy part
of later formats), one chain. Wider channels and formats without a legacy preamble are
named under "Open".

## What ADR 0006 left open

ADR 0006 ends at an acquisition: a timing, a CFO word, declared up to T1 + 223 + L = T1 + 287
samples, which is 3.9 µs after the SIGNAL symbol has ended. It says nothing about how the
samples from T1 on reach the rest of the receiver, what is removed from them on the way, who
keeps the gain still while the packet lasts, how long the answer takes, or what the front
costs. The probe on IHP 130 nm then put a number on the last: about 3 mm² and 1.1 W for the
blocks built so far, before the decision engine.

## Decision 1 — what the decoder is handed

**Proposed:** the decoder is handed, from T1 on, the ADC's own samples less one DC offset per
packet, turned by the acquisition's CFO word, 13 bits wide.

1. **A second history, of the ADC's samples** (12 bits, 512 deep), beside the history of the
   DC blocker's output that the decision reads. It has its own read port, so ADR 0006's port
   schedule is untouched.
2. **The DC offset is estimated once per packet from the two long training symbols**
   (`ltf_dc_offset_fixed`): behind a CFO the field repeats after 64 samples turned by the
   CFO's phase over 64 samples, the offset repeats unturned, so the 64 differences
   x[n + 64] − e^(jφ)·x[n] hold the offset times a known number and, at the true CFO,
   nothing of the signal, whatever the channel and whatever the transmitter leaks at its own
   carrier. (With the estimated CFO a residual of the signal remains; the packet error runs
   are made with the estimated one.) e^(jφ) is one number per packet, so the hardware needs
   two running sums over the 128 reads, one complex multiplication and one division.
   Where the CFO is below 3.1 kHz (0.54 ppm at 5.8 GHz) the division is skipped and the
   offset left in: there it lies within a hundredth of a subcarrier spacing of the unused
   subcarrier.
3. **Replay:** the samples from the FFT window of T1 on are read back from that history, the
   offset subtracted (13 bits), turned by the CFO word with a rotator of the front's kind
   (`correct_fixed`; an instance of its own, the decision's stays with the search) and
   handed on at up to one sample per clock, eight times real time at 20 MS/s, until the
   reader has caught up with the antenna; from then on at the antenna's pace. One stream
   with valid and ready; a marker on the first sample of T1's window with the CFO word; an
   end marker. A new declaration while a stream is open, or a reader so slow that the
   history is overrun, ends the stream with an abort mark. What the front searches for
   while a packet is being read is not decided here (part d step 4).
4. **Width: 13 bits, the FFT word as it is** (`in_bits` 13, `in_shift` 1): the rotator's
   product is rounded to the ADC's last bit instead of to twice it.
5. **The DC blocker stays where it is**, in front of the detector and the acquisition, at
   its pole of 2⁻⁷. Whether it needs a control at a gain change (`restart`, `rescale`, float
   model only) is a question about the acquisition alone now. Under the offsets at −20 dBFS
   the acquisition loses no frame of 2000 at 20, 25 and 40 dB (`rx_front_dc_edge.txt`,
   columns `acq`), so no control is proposed; it loses frames only where the gain control
   is blinded (508 of 2000, the offset before the gain at −10 dBFS, 25 dB).

**Why not the blocker's output** (which every measurement before 2026-11-20 assumed;
54 Mb/s, frames lost of 2000, `rx_front_dc_cand.txt` unless said):

| What the decoder is handed | no offset, AWGN 20 dB | no offset, TGn D 40 dB | offset −30 dBFS, AWGN 40 dB | offset −20 dBFS, AWGN 40 dB | offset −20 dBFS, AWGN 20 dB |
|---|---|---|---|---|---|
| the blocker's output, pole 2⁻⁷ (today), integers 13 / 2 | 412 | 251 | 19 | 98 | 808 |
| the same in float, pole 2⁻⁹ / 2⁻¹¹ (`rx_ofdm_agc_pole.txt`, other frames) | 200 / 172 | 55 / 14 | — | — | — |
| the samples before it, nothing removed (float) | 167 | 0 | 377 | 1636 | 1844 |
| the blocker's estimate frozen at the acquisition (`rxfront agcdc`, float, other frames) | worse than the blocker left running (35 of 2000 against 8 at 25 dB) | | | | |
| **the offset from the long training field, integers 13 / 2** | 170 | 0 | 0 | 0 | 458 |
| the same, integers 13 / 1, the proposed hand-off (`rx_front_dc_edge.txt`) | 172 | — | 0 | 0 | 460 |

(The offset is `GainDc::per_index`, at that level at every gain; the packet settles at
−10 dBFS. The reports have the offset before the gain, 6 Mb/s, TGn D and the 12-bit hand-off
as well.)

- The blocker's cost is the CFO's. With the clock offset below 10 ppm (58 kHz at 5.8 GHz)
  it loses nothing; from 10 to 20 ppm a little (208 against 189 at 20 dB); between 30 and
  40 ppm it loses 919 of 2000 frames at 54 Mb/s and 20 dB where the samples before it lose
  163, and 912 against none through TGn D at 40 dB (`rx_front_dc_cfo.txt`). The explanation
  that fits (reasoned, not measured apart from that dependence): the blocker is a high-pass
  with a corner at 25 kHz and a memory of 128 samples, placed before the CFO is taken out;
  at 40 ppm the subcarrier next to DC lies 80 kHz from it, on the filter's skirt, and the
  filter's estimate carries that subcarrier's earlier symbols into the present one. A
  slower pole narrows the skirt and does not remove it.
- Leaving the offset in is not an option either: 20 dB below the packet it loses a fifth of
  the 54 Mb/s frames.
- The estimate from the long training field removes both wherever the gain control itself
  is not disturbed. **It is not clean everywhere:** in the last column the gain control,
  seeing an offset 10 dB above the noise at the largest gain, changes the gain in 0.39 of
  the frames where it otherwise does not, and 458 frames are lost against 170. None of
  them is lost by the acquisition (0 of 2000 not acquired or mistimed): at 20 dB the
  control has no lock inside the packet for the hold to start from, and what the offset
  provokes is a gain change somewhere in the frame (not separated further).
  An offset 20 dB above that noise blinds the model's control altogether (842 of 2000 at
  25 dB in every column, 508 of them not acquired). Both are requirements on the radio's own offset correction in
  front of the gain control, which no path after the ADC can meet for it.
- Small clock offsets (the TDMA link's ±2 ppm an end; below 0.54 ppm the offset is left
  in): with the clock offset within 2 ppm the 13 / 1 chain with the estimate loses, with no
  offset / −30 / −20 dBFS, 0 / 0 / 0 frames of 2000 at 54 Mb/s and 40 dB, 179 / 176 / 471
  at 20 dB (the float chain 178 / 178 / 470; the last is the gain control again) and
  195 / 187 / 193 at 6 Mb/s and 3.5 dB. At these clock offsets nothing removed at all loses
  0 / 0 / 0, 174 / 181 / 499 and 187 / 191 / 225: the offset is then nearly on the unused
  subcarrier.

**Why 13 bits:** the lower edge of the input level is the hand-off's last bit
(`rx_ofdm_agc_level.txt`, with the blocker in the path). Rounded to twice the ADC's last bit
(12 bits) the receiver needs the radio's noise at the largest gain at or above −42 dBFS at
the ADC; with the last bit kept the edge is about 6 dB lower. With the estimate in place of
the blocker the 13 / 1 chain loses 196 and 227 frames of 2000 at 54 Mb/s with the noise at
−48 and −54 dBFS (the float chain 186 and 199) and 221 and 276 at 6 Mb/s (214 and 238):
the same edge. The 13th bit costs one bit of the stream and nothing in the FFT
word. `in_shift` 2 (a wider FFT word) buys nothing measurable more and costs a bit of
headroom.

**Not measured, not proposed:** a notch at the offset's place after the CFO is out, and a
pole switched to a slower one after the acquisition (`rx_ofdm.md` named both). The first is
the same long-memory filter at another frequency; the second is bounded by the row of
slower poles above. Neither was run.

**What this is not.** One offset per packet: it is right while the gain is still from T1 to
the packet's end (decision 2) and the offset does not drift inside a packet (no data on the
AD9361's residual offset exists here; R1's captures decide). A gain change inside the long
training field spoils it, as it spoils the timing (ADR 0007). A wrong timing gives a wrong
estimate for a packet that is lost anyway. Back-to-back packets each get their own. The
estimate is whole ADC LSB.

## Decision 2 — who holds the gain

**Proposed, in two steps:**
1. **The gain control does not leave a lock for 512 samples after taking it** (25.6 µs, to
   the latest declaration and past it), whatever its overload detectors say.
2. **The front raises a `hold` when it declares an acquisition** and drops it at the
   packet's end — the end computed from SIGNAL's length, at once if SIGNAL does not decode,
   and by a watchdog at the longest PPDU.

- Without a hold the model's control leaves its lock inside the packet on a peak of the
  data: in 2 to 4 % of the 54 Mb/s frames of 1000 octets and 37 to 60 % of the 6 Mb/s ones at
  30 and 60 dB, none at 10 dB (`rx_front.md`, 2026-11-19). Every run since held the gain
  from the control's first lock inside the packet; steps 1 and 2 together are that
  assumption made buildable, for a true packet.
- With step 2 alone, held from a fixed 479 samples after the packet's first (the latest
  declaration) with the control free until then, 0 to 5 more frames of 2000 are lost at
  54 Mb/s (AWGN 25 to 60 dB: 2, 1, 5, 0; TGn D 40 and 60 dB: 2, 4) and none at 6 Mb/s
  (`rx_front_dc_hold.txt`). In one of those rows the mean level is −4.7 dBFS against −10:
  a few frames were held at a gain caught in mid-change. The code reviewer instrumented the
  run: the rows differ only in the frames whose gain changed between the lock and the hold
  (2, 1, 5, 0 at 54 Mb/s in AWGN), the control was unlocked at the hold in all of them, and
  at 54 Mb/s every one was lost. So the small counts say how seldom this model's control
  leaves its lock in the first 479 samples, not that a change there is harmless: it costs
  the frame, which is what step 1 is for.
- **What the measurement is not:** the hold's end was the packet's true end, known to the
  tool. The declaration of the front itself, the release from SIGNAL, a SIGNAL that fails
  and the watchdog are not modelled. A false or foreign acquisition whose SIGNAL passes its
  one parity bit by chance holds the gain for the length it claims, up to 5.5 ms at
  6 Mb/s: a stronger packet arriving then is not re-gained. Step 1 costs the same for
  25.6 µs after every lock on anything.
- The requirement of ADR 0007 stands beside it and is not enforced by either step: the gain
  still from the end of the STF.
- `channel.md` records that the AD9361 has a pin that holds the gain; how fast it acts and
  whether its fast-attack machine has step 1's timer are to be read from its documentation
  and tried at R1. A hold by SPI is slower; how slow a hold may be was not measured (a
  delayed-hold row in `rxagc hold` would say).

## Decision 3 — the latency budget

SIFS is 16 µs at 20 MHz (32 µs at 10 MHz, 64 µs at 5 MHz; IEEE 802.11-2020 Table 17-21, from
memory of the table, to be checked against the text): from the end of the frame at the
antenna to the first sample of the response. The shortest frame that must be answered is one
data symbol long at 54 Mb/s (an RTS of 20 octets), so a frame can end at T1 + 288, the
sample at which its acquisition is declared. The whole frame is then in the history and
nothing has been decoded.

**Proposed budget at R = 8** (clocks of 6.25 ns after the frame's last sample at the ADC's
pins; every figure an estimate before RTL):

| Step | Clocks | Running total, µs |
|---|---|---|
| DC estimate: 128 reads on one port, one multiplication, one division | ≈ 160 | 1.0 |
| replay of T1, T2, SIGNAL and one data symbol at one sample per clock | 288 | 2.8 |
| FFT of the last symbol, pilots, the equaliser; the data symbol waits for SIGNAL's rate | ≈ 200 | 4.1 |
| de-interleaver and Viterbi decoder: the last symbol's 216 bits at one trellis step per clock, and the traceback | ≈ 320 | 6.1 |
| descrambler, FCS, the MAC's decision to answer | ≈ 100 | 6.7 |
| left for the radio's receive latency (antenna to pins), its turn-around and the transmit chain | | 9.3 of 16 |

- **Antenna to SIGNAL decoded:** the same path to the end of SIGNAL, about 600 clocks
  (3.7 µs) after the declaration, so at the latest 7.6 µs after SIGNAL has ended.
- **Requirement on everything behind the hand-off: one sample per clock**, eight times real
  time at 20 MS/s, and a Viterbi decoder at one trellis step per clock (160 Mb/s against the
  54 it must sustain; whether R0-10's decoder closes 160 MHz on 130 nm is not known: no
  RTL exists). A decoder path that only keeps pace with the antenna would stay 15 µs behind
  for the whole packet and miss the answer.
- A frame of ordinary length is caught up with long before it ends: the backlog of 300
  samples shrinks by seven samples in every eight clocks, so the reader is at the antenna
  43 samples (2.1 µs) after it starts.
- At the same clock the 10 and 5 MHz modes leave 25 and 57 µs.
- L = 64 stays if decision 4's schedule allows it; L = 71 costs 0.35 µs of this budget.
- Nothing is known yet of the radio's three items in the last row. R1 measures them; the
  9 µs are what is left for them.

## Decision 4 — folding the multipliers

The probe (`silicon_probe.md`, IHP 130 nm, 160 MHz): `ltf_corr` is 1.22 mm² routed. Its
synthesis report (on the build server, not in the journal) has 7932 flip-flops of 49 µm²,
0.39 mm²; the rest is the 128 constant multiplications and their adder tree. `det_a` is
0.81 mm², 0.11 mm² of it flip-flops, the rest two squares, a product and an 80-bit
comparison, and it misses the clock by 2.5 ns. Both do their arithmetic once per sample,
one clock in eight. ADR 0006 adds a second correlator for the decision.

| Option | Correlators | Detector | Cost |
|---|---|---|---|
| A. ADR 0006 as written | 2 × 1.22 mm² | 0.81 mm², fails timing | none |
| B. one unfolded array shared with the decision (ADR 0006's deferred idea) | ≈ 1.4 mm² (estimate) | as A | a multiplexer on the array's critical path; the main search must pause or the decision wait |
| **C. the search correlator folded over R, the decision's over 4, the detector's arithmetic serial over R** | ≈ 0.4 + 0.45 mm² (estimates) | ≈ 0.35 mm² (estimate) | `ltf_corr` and `det_a` rewritten in RTL; their results arrive later; ADR 0006's R = 6 fallback is given up |

**Proposed: C, and a probe of the folded search correlator through the flow before the
decision engine is written.**
- A folded search correlator has 128/R = 16 general multipliers that take a tap from a
  small table each clock, in place of 128 wired shift-and-add ones; the window of 128
  samples (3072 flip-flops, 0.15 mm²) stays. No folded block has been through the flow: the
  0.4 mm² is the window plus an eighth of today's logic plus an allowance, not a result.
- The decision's correlator is a second one, on a window of 64, not a share of the first:
  the main search runs while a decision is pending (PR #24), and at R = 8 an array folded
  over 8 is busy on every clock with the search alone.
- **The schedule.** ADR 0006's formula with each of the decision's 24 correlations taking F
  clocks instead of one:
  13 + 96·R/(R−1) + 18 + max(140·R/(R−1) + 10 + 24·(F−1), 176·R/(R−1)) + 18 + 5.
  At R = 8 and F = 8 that is about 500 clocks against a budget of (64 − 5 − 1)·8 = 464 (an
  event is known about R clocks later, D = 5, and the folded search reports a sample
  later): **it does not fit L = 64.** With L = 71 the budget is 520 (4 % of slack). With the
  decision's correlator folded over 4 (32 multipliers) the work is about 406 clocks: 13 % of
  slack at L = 64. Proposed: F = 4 and L = 64; the cycle-exact schedule, with the energy
  ring's margin and `det_a`'s latency in it, is the first deliverable of part d step 3,
  before any RTL of the decision engine.
- **R = 6 no longer fits** by the same formula (about 460 clocks against 390 at L = 71, the
  correlations being the long pole). ADR 0006's fallback clock is given up with option C;
  if 160 MHz does not close on a board, the fallback is the unfolded decision correlator
  on that board.
- **What stays bit-exact and what does not carry over.** `ltf_corr`'s sums are the same
  integers, so its committed vectors still hold the values. The interface does change: a
  folded block takes a sample every R clocks at most, where today's takes one per clock and
  its testbench feeds it so; the testbench's timing and the mutation records are redone for
  the new RTL. `det_a` is retuned first (R0-08c), in the model, which changes its vectors;
  it is folded once, after that. The latch's gate is rerun at the new D.
- On an FPGA the same change takes the correlators from about 250 DSP blocks (ADR 0006's
  count) to 48.
- The correlators' arithmetic is enabled only while a search is open. Listening, the front
  clocks the detector, the windows and the DC blocker.

Not proposed: narrower words. The G2 sweep left Z ≥ 10, T ≥ 6, K ≥ 15 passing one at a
time and the joint lower set failing (`rx_front.md`); what narrowing is safe is measured
again as packet error rate with R0-08c's retune, not here.

## Decision 5 — area and power of one chain's front

The founder's starting point (2026-11-19) was ≤ 1 mm² and ≤ 150 mW on 130 nm, to be
justified here. **The numbers do not support it. Proposed: ≤ 2.5 mm² at 130 nm as the
working target, to be tightened after the folded search correlator's probe; no power
target is signed until a folded block's switching has been simulated.**

| Part (130 nm) | Today (routed) | After decisions 1 and 4 (estimates) |
|---|---|---|
| search correlator | 1.22 mm² | ≈ 0.4 |
| decision correlator (ADR 0006) | + 1.2 (not built) | ≈ 0.45 |
| detector | 0.81 | ≈ 0.35 |
| latch with its CORDIC; the delay line 4672 flip-flops today | 0.71 | ≈ 0.5 with the line as a macro (0.23 mm² of flip-flops out, its read multiplexer not separated) |
| DC blocker, three rotators, MAC, squarer, the DC estimate | 0.1 + not built | ≈ 0.3 |
| memories: two histories and the delay line, 30 kbit of data, about 65 kbit of the PDK's two-port macros as `memory.md` maps them | not built | 0.15 to 0.3 (macro sizes not read from the PDK) |
| **sum** | ≈ 4.2 with ADR 0006 complete (estimate) | **≈ 2.2, ± 30 %** |

- 1 mm² is out of reach by a factor of two with this algorithm on this node, by estimates
  that have not been through the flow. What would move it further is narrower words
  (decision 4, not proposed) or a different search; neither is counted.
- **Power.** The probe's 1.1 W assumes the flow's default switching in every cell on every
  clock. An unfolded array switches when its window shifts, one clock in eight, so that
  figure overstates today's blocks; folded logic does switch every clock. Scaled from the
  probe's power per mm² (0.37 W/mm²; not a run of the flow) the folded front is about
  0.8 W during an acquisition and about 0.3 W listening at 130 nm and 160 MHz.
- **For ADR 0009's configuration** (eight chains): eight such fronts are 0.22 million gate
  equivalents each by the probe's density, 1.8 million together, against ADR 0009's first
  count of 2 to 4 million for the whole chip — **that count did not have eight fronts of
  this size in it and needs revisiting.** At 130 nm they would be 18 mm² and, by the
  scaling above, 2.4 W listening: 130 nm stays the test vehicle. By the density ADR 0009
  itself uses for 22 nm (about twelve times the probe's) they are 1.5 mm². The data path's
  per-chain part (FFT, channel estimate) is not counted here; it has no RTL.
- Whether every chain needs its own search, or a link searches on the chains that last
  heard best and replays the others from the same timing and CFO word, decides most of that
  figure. It is R0-17's question, and it puts one requirement on decision 1's stream now:
  a replay can be started from outside, with a timing and a CFO word, on a chain that did
  not acquire.

## Decision 6 — the sample interface and the pool of chains

**Proposed:**
- Inside the chip a chain is one stream: 12-bit I and Q with a valid, one sample in R
  clocks. Every block of this ADR is written against that stream.
- For the 20 MHz formats of this ADR's scope the front is one per chain and runs at
  20 MS/s (10 and 5 MS/s half- and quarter-clocked).
- **The pool** (ADR 0009: chains × bandwidth as the build's parameter) **is not settled
  here.** Decision 3 makes the path behind the hand-off take one sample per clock, 160 MS/s,
  which is the sample rate of eight chains of 20 MHz; but its budget uses that whole rate
  for one chain's catch-up. Eight chains of one link all hold the same backlog at the same
  moment: shared among them, one path replays 288 samples per chain in 2304 clocks
  (14 µs) and misses SIFS. How many paths a build needs is a count of chains, SIFS and the
  TDMA profile's own turn-round (which is not SIFS), for R0-17.
- **Pins:** not settled either. From memory of the prototype transceiver's documentation,
  to be checked: an LVDS mode with six data pairs, a frame and a clock in each direction
  for both channels (16 pairs a device) and a CMOS mode of about as many pins; the open
  130 nm I/O library may have no LVDS cells. Four devices are of the order of 120 pins
  either way. A multi-gigabit serial link (JESD204) needs a SerDes we do not have and is
  not proposed. The choice goes with the radio at R1.

## Decision 7 — the CFO word and the timing tracker

**Proposed:** the timing tracker's prior rate is the CFO word times one number per channel,
fs/fc, written by the MAC when it tunes (a 16-bit mantissa is ample: the CFO word itself is
good to a few hundred hertz of 232 000). The product is formed once per packet where the
hand-off's stream starts. Today that conversion is the one floating-point step between the
ADC and the PSDU (`FixedTracking::from_clock_offset`); its integer form is a model change
with its own G1 and G2, before the tracker's RTL.

Not decided here: narrowing the tracker (R 6, M 8, K 8, V 4 showed no loss on 1000-octet
frames, `rx_ofdm.md`). It is measured on the full grids and on the longest frame before its
RTL; the defaults stand until then.

## Consequences

- **Model:** `ltf_dc_offset`, `ltf_dc_offset_fixed`, `remove_dc_fixed` (this PR, with their
  gates). `FixedOfdmConfig`'s default hand-off becomes 13 / 1 and `rxper` hands the decoder
  this stream (a follow-up with its G2 rerun). The tracker's start from the CFO word in
  integers (decision 7). `RxFrontFixed` gains the second history and the replay as a
  streaming model before part d step 4. `FastAttackAgc` gains decision 2's timer and a hold
  input, and `rxagc hold` the front's own declaration and release.
- **RTL, in this order:** the folded search correlator and its probe through the flow; the
  cycle-exact schedule of the decision (decision 4); `det_a` after R0-08c's retune, serial;
  then part d steps 2 to 4 as planned, with the hand-off as a new last step.
- **G3 so far stays valid** for `dc_block`, `nco`, `cordic` and the memories. `ltf_corr`
  keeps its vectors' values and gets new RTL, a new testbench timing and new mutation
  records; `det_a` gets new vectors with its retune; the latch's gate is rerun at the new D.
- **Requirements on the radio, to check at R1:** the gain still from the end of the STF, a
  lock kept for 25.6 µs and a hold on a pin (decision 2); its noise at the largest gain at
  or above −48 dBFS at the ADC with the 13-bit hand-off; its own offset correction leaving
  less than the noise at the largest gain in front of the gain control's detectors
  (10 dB above it already costs frames); its receive latency and turn-around inside the
  9 µs of decision 3.
- **Backlog:** R0-08d (3) is rewritten to the order above; R0-17 takes the pool's sizing,
  the search per chain and the externally started replay; ADR 0009's gate count is
  revisited there.

## Open

- The AD9361's residual offset and its drift inside a packet (R1): the one-offset-per-packet
  assumption.
- Wider channels: a 40, 80 or 160 MHz chain needs a channeliser in front of this front, its
  receiver's offset lies at the wide channel's centre and not in the primary 20 MHz, and
  the replay must hand on samples at the full rate from a history that many times deeper.
  None of it is designed or counted here. Formats without a legacy preamble (HaLow, HT
  greenfield) need their own acquisition.
- The folded blocks' timing and area through the flow; the schedule's slack.
- Power from simulated switching.
- The macros' sizes and timing at 160 MHz (`memory.md` checked their ports only).
- The radio's pins, its hold, its latencies (decisions 2, 3, 6).
- TGn B and E, frames other than 1000 octets, the upper edge of the level: not measured.

## The review of the first draft (fresh agent, 2026-11-21)

It checked every number against its report, rebuilt the arithmetic and questioned each
decision. Corrected after it: the folded schedule (the draft's "about 430 clocks, 9 % of
slack at L = 64" could not be rebuilt from ADR 0006's formula, which gives about 500 and no
fit; hence the decision's correlator folded over 4, and the R = 6 fallback given up); the
pool (the draft equated one data path with eight chains and contradicted its own latency
budget); "always acquire at 20 MS/s on the primary channel" (restricted to the scope, the
wide channels named open); the area target (≤ 2 mm² was inside the error of its own
estimate; the latch and the memories were counted too small; the split into flip-flops and
logic was not in the journal); "below 20 ppm it loses nothing" (10 ppm); the table of
decision 1 showed only the columns where the estimate is clean; the proposed 13 / 1
hand-off had not been measured under an offset or at the lower edge, and clock offsets
below the guard hardly at all (`rxagc edge` added); the hold (a timer at the control added;
what the measurement leaves out stated; the pin recorded in `channel.md`); the latency
table (one read port, the radio's receive latency, SIGNAL before the data symbol, antenna
to SIGNAL); the stream's end, abort and overrun; "the vectors do not change" (true of
`ltf_corr`'s values only); ADR 0009's count; the pins (the draft's comparison was with a
parallel interface nobody proposed). Left as the reviewer could not verify either: the
SIFS table, the transceiver's pins and hold, the PDK's macro sizes.
