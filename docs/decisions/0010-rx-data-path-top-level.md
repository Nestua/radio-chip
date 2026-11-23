# ADR 0010 — Receiver data path, top level: from the hand-off's stream to the PSDU

Status: **accepted** · proposed and accepted by the founder 2026-11-23 · PHY architecture
(risk point). Follows ADR 0008 (the hand-off's stream, the latency budget of its decision
3, the verdict of its decision 2). An independent review of the first draft (fresh agent,
2026-11-23) changed decisions 2, 3 and 5, the latency table and the area table; what it
found is at the end.

## Decided (the founder, 2026-11-23, each item asked on its own; the recommendation taken in each)

- **Decisions 2 and 3:** one symbol after another (the next symbol's window only when
  this one's tracker step has ended), the model as it is; the pace that follows, 2.3 to 3
  times real time.
- **Decision 4:** register exchange, and the Viterbi decoder through the open flow before
  the blocks around it.
- **Decision 5:** octets and a status to the MAC; the frame check sequence is the MAC's.

Decisions 1 (the blocks) and 6 (the order of work) were not asked on their own: they are
the proposal's, and the first is partly left to the blocks' own G0 (the FFT's
architecture, the metric's width).

The text below is the proposal as it was put to the founder.

Scope: the 20 MHz legacy formats (802.11a/g: BCC, eight rates), one chain. LDPC, HT and
later formats, and more than one chain, are named under "Open".

Notation as in ADR 0006 and 0008: R clocks per sample (8: 160 MHz at 20 MS/s); a symbol is
80 samples, 640 clocks of air time. **Every cycle count and every area figure here is an
estimate made before any RTL or schedule model**, from the model's loop bounds and the
stage counts of blocks that exist. The schedule model of decision 6 replaces them; a
figure such as "9 µs left" is good to about a microsecond.

## Context

The front is complete up to its stream (`rx_front`, `rx_handoff`; `docs/gates/rx_front.md`):
beats of 13-bit samples from the first long symbol's FFT window on, a beat a clock while
the reader is behind the antenna, the CFO word and a tag on the first beat, marks at the
end. It expects back one strobe per packet, for one cycle: SIGNAL's verdict (failed, or
the number of data symbols and the patient bit).

What decodes that stream exists as a model only: `radio_phy::rx_ofdm_fixed`
(`decode_ppdu_fixed`: FFT, channel, tracker, rotation, demapper, soft scale, all in
integers, G2 done) and `radio_phy::viterbi` (8-bit soft values, clip 3, window 64; G2
done on those; the 13-bit metric register is a bound of the journal, left to G3). The
model takes a whole buffer of samples and returns a PSDU. It says nothing of when
anything happens. What it does that a streaming data path cannot do as written:

1. **It decodes the data with a traceback over the whole block** (`viterbi::decode`). The
   hardware's variant (`decode_windowed`, window 64, chosen by R0-10's G2 at a loss of
   0.024 dB on AWGN) is in the model but not in the receiver: no packet error figure of
   `rx_ofdm.md` was measured with it.
2. **The FFT window of symbol i + 1 is placed by symbol i's finished tracker step**
   (`FixedTracking::shift` moves by a sample when the slope left in symbol i passes half
   a sample, and that slope includes symbol i's own fit). The model indexes a buffer; a
   stream gives each sample once, and a pipeline that starts symbol i + 1's transform
   before symbol i's tracker has ended does not know where its window is. The reviewer
   measured the moves on the model: 24 to 28 in 8202 symbols at 6 Mb/s, ±40 ppm, 6 dB
   (0.3 %), a few of them back again; none at 54 Mb/s in a frame of 4095 octets.
3. Smaller things the streaming model (decision 6.1b) has to settle, each with a status
   or a rule: a scrambler seed of zero found after the MAC was told the frame began; a
   window moved later than the stream's tail (12 samples past the last symbol) holds;
   the number of data symbols for the verdict (a division by the rate's bits per symbol);
   octets already handed on when a mark aborts the packet; a new packet's first beat
   while the last one's tail is still being decoded.

## Decision 1 — the blocks

The data path of the model, block for block, each with a gate of its own before the top
level, as the front's were:

| Block | Model | Per |
|---|---|---|
| the stream's reader (decision 2) | `window_of` | symbol |
| `fft64` | `fft64_fixed` | symbol |
| channel: Hs, the gains, the packet's shift, Σg | `estimate_channel_fixed` | packet |
| per-packet quotients: the tracker's two coefficients and variance, the soft scale | `tracker_constants`, the scale of `symbol_soft_fixed` | packet (the scale twice: BPSK for SIGNAL, the data's modulation) |
| products u = Z·Hs*, held for the symbol (52 × 32 bits), pilots | `symbol_soft_fixed` | symbol |
| tracker: two pair angles, the fit, the drift | `FixedTracking` | symbol |
| common phase, rotation, demapper, soft scale | `symbol_soft_fixed` | symbol |
| deinterleaver, depuncturer | `interleaver`, `viterbi::depuncture` | symbol |
| Viterbi decoder | `viterbi::decode_windowed` | bit |
| SIGNAL: field, parity, rate, length, the check, the number of data symbols, the verdict | `signal_checked`, `SignalVerdict::of` | packet |
| descrambler, octets out | `rx_ofdm::descramble` | bit |

**Proposed:** one CORDIC in the data path, its own and **iterative** (three angles a
symbol, 18 clocks each, a fraction of the pipelined `cordic`'s 0.115 mm²; the same
arithmetic). Sharing the front's through `cordic_arb` would work most of the time (it is
nearly always idle) and would tie this path's schedule to the front's latches by a cycle
now and then; not proposed. One serial divider for the per-packet quotients and the
tracker's division per symbol. The rotator's and the CORDIC's tables are the front's ROMs.
The FFT's architecture (in place on a register array with two butterfly units, or a
pipelined single-path transform in the model's order fed in bit-reversed order from a
64-word store) is `fft64`'s own G0: the memory wrappers have one write port, so "in place
on a memory" as the first draft said cannot do two butterflies a clock.

## Decision 2 — the stream's reader and the window's moves: one symbol after another

**Proposed: the data path takes symbol i + 1's window only when symbol i's tracker step
has ended**, and places it by counting on the stream: between two windows it lets 15, 16
or 17 samples pass (the guard interval, less or more the tracker's move). No addressed
buffer of samples; `out_ready` to the hand-off is a register.

- This is the model as it is: no speculation, no redo, no change of what is decoded.
- The cost is the pace (decision 3): the transform of a symbol cannot overlap the tracker
  of the one before.
- The alternatives, not proposed: start the next transform early and do it again when the
  window moved (a buffer of two to three symbols and a flush path, for 0.3 % of the
  symbols); or change the model so that a move is applied a symbol late (a model change
  with its own G2).
- The registered `out_ready` removes the risk the hand-off's timing probe named (a reader
  whose ready is combinational). The path inside `rx_handoff` from `out_ready` into the
  tick's comparisons stays untimed until the two blocks go through timing together.
- While the data path is behind, the backlog is in the hand-off's history (512 samples;
  about 305 at the first beat), whose overrun rule says what happens to a reader too slow.
- Marks: a packet's end mark closes the packet; an abort (acquisition, overrun) or a stop
  drops what is in flight and is reported (decision 5); the next first beat starts afresh.

## Decision 3 — the pace: what "one sample per clock" has to mean behind the hand-off

ADR 0008 (decision 3) required everything behind the hand-off to take one sample per
clock, and counted the Viterbi decoder at one trellis step per clock. The two do not
meet: at 54 Mb/s a symbol is 216 decoded bits, 216 clocks at a step a clock, against the
80 clocks in which the hand-off can deliver its samples. With decision 2 the part in
front of the decoder is no faster either.

**Proposed:**
- the Viterbi decoder at one trellis step per clock (64 add-compare-select units), as
  ADR 0008 counted, behind a soft-value buffer of two symbols (2 × 288 values of 8 bits);
- in front of it one symbol after another: 64 samples in, the transform, the pilots'
  products, two angles, the fit with its division, then the next symbol's samples while
  the rest of this one (the phase, the rotation, the demapper, the scale) runs. About
  **215 to 280 clocks a symbol at every rate, 2.3 to 3 times real time.**

**The latency this gives** for a frame of one data symbol at 54 Mb/s that must be
answered after a SIFS. The longest such frame is 24 octets (a BlockAckReq: 214 bits of
the symbol's 216; ADR 0008 took an RTS of 20 octets). Its last sample is 288 samples
after T1; its acquisition is applied 265 to 277 samples after T1 (`rx_front.md`,
189 + w + L to 189 + 2w + L at w = 12), so the declaration's tick t is 82 to 178 clocks
*before* the first sample after the frame. (ADR 0008 put the declaration at the frame's
end, ADR 0006's latest case; the front as built declares earlier.) In clocks after t:

| Event | Clocks after t | How |
|---|---|---|
| the stream's first beat | 161 | `rx_handoff` as built |
| T2's, SIGNAL's, the data symbol's last sample | 289, 369, 449 | a beat a clock; these three windows are placed at the first beat (below) |
| transforms of T1, T2, SIGNAL, the data symbol done | 321, 417, 513, 609 | 96 clocks each after the later of its last sample and the transform before |
| channel: Hs, the gains, the shift, Σg | ≈ 530 | two passes over 52 bins after T2's transform |
| SIGNAL: BPSK's scale, the phase, the soft values | ≈ 552, 560, 616 | one division; the pilots' rotation and one angle; 48 subcarriers through the pipeline |
| SIGNAL decoded and checked: **the verdict's strobe** | **≈ 650 (4.1 µs)** | 24 trellis steps, the re-encoding, the number of symbols |
| the tracker's three quotients | ≈ 633 | on the one divider, after the scale's |
| the data symbol's tracker step, the data's scale | ≈ 708, 735 | the fit with its division; a division that waits for the rate and the divider |
| its soft values | ≈ 791 | six scale products a clock (two would add ≈ 96 clocks) |
| its 214 bits decoded | ≈ 1005 | a step a clock |
| the PSDU's last octet out | **≈ 1070** | the window's last 64 bits come out of state 0's register a bit a clock |

**SIGNAL's tracker step is not on its own critical path.** With one measurement the fit's
numerator n·Στz − Στ·Σz is 7z − 7z = 0: the model's learned rate after SIGNAL is zero
whatever the pilots say (the reviewer: 0 of 100 000 random cases and 0 of 54 frames differ).
So SIGNAL's slope is the prior's alone, known at the first beat; so are SIGNAL's window
and the first data symbol's. SIGNAL's two pair angles are needed only for the sums that
the first data symbol's fit uses. BPSK's demapper needs no gains either: SIGNAL waits for
the channel and one division, not for the tracker's quotients. This is the model as it
is, not a change of it.

That is 890 to 990 clocks after the frame's end at the ADC's pins: **5.6 to 6.2 µs**,
and with the MAC's decision (≈ 100 clocks, ADR 0008's figure) at most 6.8 µs. **About
9 µs of the SIFS's 16 are left** for the radio's receive delay, its turn-around and our
own transmit chain (ADR 0008 had 9.3 µs for the same). What ADR 0008 did not count (four
transforms one after another, the channel's passes, the per-packet divisions, the last
window's flush) is about what the earlier declaration gives back.

- The verdict comes about 650 clocks after t, which is 462 to 558 clocks after SIGNAL
  has ended at the antenna: **6.9 to 7.5 µs after SIGNAL's end** (ADR 0008 expected
  7.6 µs). The front's hold has been up since the declaration.
- A longer frame is caught up with at 2.3 to 3 times real time, within a few tens of
  symbols; from then on its last octet is out about one symbol's work after the frame's
  end (some 500 clocks at 54 Mb/s), less than the table's case.
- A reader this fast never comes near the hand-off's overrun.

## Decision 4 — the Viterbi decoder's survivors

The model's hardware variant decides bit i by tracing back 64 steps from the state with
the best metric at step i + 64 (ties: the lowest state). Two ways to build it:

| | Register exchange | Traceback from a memory |
|---|---|---|
| survivors | 64 registers of 58 bits (the newest six bits of a path are its state), every one rewritten every step | a memory of 64 bits a step, some 128 to 192 steps deep (the wrapper) |
| the model's rule (the best state at every step) | as it is | not as it is: a traceback is started once a block of steps, so the model's variant would change and R0-10's G2 be rerun |
| latency of a bit | 64 steps | 128 to 192 steps: up to 0.8 µs more before the last octet |
| cost (estimate, not checked by the reviewer for the memory) | 3712 flip-flops and as many multiplexers: 0.18 mm² of flip-flop cells by the probe's 49 µm² each; all of them toggle every step | perhaps a third of that in area; little toggling |

**Proposed: register exchange**, the model as it is, **and the decoder through the open
flow first** (as ADR 0008 did with the folded correlator): area, the timing of the
add-compare-select loop at 160 MHz, and a power figure from simulated switching. It
depends only on `decode_windowed`, which exists, so it can run beside the model work of
decision 6. If it does not fit, the fallback is the memory and a changed model, with its
G2.

- The best-state selection is a tree of 63 compare-selects on modulo metrics. It can be
  pipelined by delaying the column of oldest bits with it, without changing the model.
- **The metric's width is not settled by the model**: `viterbi.rs` keeps 64-bit metrics
  and marks the 63 states that cannot be reached at the start; 13 bits is the journal's
  bound for the steady state. The RTL needs valid flags for the start or a 14th bit:
  the decoder's own G0.
- R0-14 named "a serial-ACS Viterbi if it fits" for the silicon probe; that is a
  different block (one unit, 64 clocks a step) and would not meet this ADR's pace.

## Decision 5 — the data path's interfaces

**Proposed.** In, besides the stream: the channel's fs/fc register, written by the MAC
when it tunes (ADR 0008, decision 7); the tracker's prior is formed from it and the first
beat's CFO word, once per packet. Out:
- to the front: the verdict's strobe as `rx_handoff` takes it (tag, failed, data symbols,
  patient), **for one cycle**, once per packet, when SIGNAL has decoded or failed;
- to the MAC: `rx_start` with rate and length when SIGNAL has decoded; then the PSDU's
  octets, descrambled, with a valid and a last; `rx_end` with a status: decoded to the
  length; SIGNAL failed, with which check; aborted by a mark, with its kind; a scrambler
  seed of zero; the stream ended before the frame did. The scrambler's seed and the
  SERVICE field's bits with the first octet. **A time mark of the frame's end at the
  antenna** (the sample count the hold's end is computed from), because `rx_end` itself
  comes 0.6 µs earlier or later from frame to frame and the MAC times its answer from the
  frame's end;
- **the frame check sequence is the MAC's**: the PHY hands on every octet of a PSDU whose
  SIGNAL decoded, right or wrong. The local copy of the standard (2024, §17.3.12) agrees
  on no start indication after a failed parity or rate and on a format violation
  reported instead; its figure of the receive state machine has not been read against
  this yet.

The octets have no ready. Their pace is the decoder's: a bit a clock at most, so an octet
in eight clocks, also in the flush at a frame's end.

## Decision 6 — order of work

1. **Model** (each with its gate):
   a. `decode_windowed` in `decode_ppdu_fixed`, and the G2 points rerun with it (through
      the receiver and in fading the window's loss has not been measured);
   b. a streaming data path, `RxDataFixed`: beats in, the verdict and the octets out, one
      symbol after another as decision 2 says, the items of Context 3 settled, held to
      `decode_ppdu_fixed` frame for frame on the frames that complete;
   c. its schedule in clocks (as `rx_decision` and `rx_handoff`): decision 3's table made
      exact, the shared CORDIC, divider and multipliers.
2. **RTL**, in this order: the Viterbi decoder and its run through the flow (decision 4;
   it may start beside 1a); `fft64`; the channel and the per-packet quotients; the
   products, rotation, demapper and scale; the tracker; the deinterleaver and depuncturer;
   SIGNAL and the verdict; the top level behind `rx_handoff`, a cycle at a time against
   the schedule model.
3. The front and the data path together through timing, and the open flow on both.

## Area (rough, 130 nm, cell area; no block through the flow)

| Part | Estimate | From |
|---|---|---|
| Viterbi decoder, register exchange | 0.5 to 0.7 mm² | 3712 + ≈ 900 flip-flops at the probe's 49 µm², as many multiplexers, 64 ACS, the tree |
| `fft64`: 64 words of 32 bits, two to four complex multipliers 16 × 12 | ≈ 0.25 | `nco`'s 0.075 mm² for one 13 × 11 complex product as the yardstick |
| channel store (52 × 34 and 52 × 16 bits), the symbol's products (52 × 32), the soft buffer (4.6 kbit) | 0.2 to 0.4 | macros or flip-flops; the macros' sizes not read from the PDK |
| products and rotator; demapper | ≈ 0.2 | one 16 × 17 complex product, a rotator of `nco`'s kind |
| soft scale: six products of about 17 × 20 bits (or two, 96 clocks slower) | ≈ 0.2 (0.07) | 3.5 times `nco`'s four 13 × 11 |
| tracker: iterative CORDIC, the sums, shared products, the divider | ≈ 0.2 | |
| **sum** | **≈ 1.5 to 2.0 mm², ± 40 %** | |

Cell area: the probe placed at 40 % utilisation, so a floorplan is larger. Against ADR
0008's working target of 2.5 mm² for the front, the data path is of the same order. No
target is proposed for it until the Viterbi decoder has been through the flow.

## Consequences

- **Model:** the follow-ups of decision 6.1; `rx_ofdm.md`'s packet error figures hold for
  the whole-block traceback until 6.1a is measured.
- **ADR 0008, decision 3:** its requirement "one sample per clock behind the hand-off"
  and its "caught up 43 samples after it starts" are replaced by decision 3 here (2.3 to
  3 times real time); its SIFS budget by this one's table (about 9 µs left where it said
  9.3; 6.9 to 7.5 µs from SIGNAL's end to the verdict where it said 7.6). A SIGNAL that
  fails keeps the hold up as long as a verdict takes.
- **ADR 0008, decision 6:** its arithmetic for a pool of chains assumed eight times real
  time behind the hand-off. With this pace one data path serves one chain; R0-17 takes it.
- **`rx_handoff`:** no change.
- **Backlog:** R0-09 gains the streaming model and the schedule; the data path's RTL is
  new items in the order of decision 6.

## Open

- LDPC (R0-12's decoder, 20 iterations approved) beside the Viterbi decoder: its place in
  this path, its latency against SIFS, its area.
- HT and later formats: more symbols of training, other pilots, 40 MHz and wider.
- More than one chain (ADR 0009, R0-17): one data path per chain up to the equaliser, or
  shared; combining.
- The radio's receive delay and turn-around, unknown until R1, against the 9 µs.
- The standard's receive state machine read against decision 5.
- Transforming T1 + T2 once instead of each (a model change with its G2; one transform
  and some 30 clocks less); channel-estimate smoothing. Neither here.

## The review of the first draft (fresh agent, 2026-11-23)

It rebuilt the draft's arithmetic from its sources (the 82 to 178 clocks, the
microseconds, ADR 0008's own budget, the divider's widths, the area's inputs) and found
it consistent with them, and the table built on a wrong picture in four places:
- **the window's move** (Context 2): the draft let symbol i + 1's transform run before
  symbol i's tracker had ended, and named only the learned rate as what ties the two.
  Decision 2 is new: one symbol after another, and with it the addressed sample buffer
  of the draft is gone;
- **SIGNAL's fit is identically zero** (decision 3): the draft had SIGNAL wait for the
  tracker's quotients and its division, and listed "SIGNAL without its fit" as an option
  not taken, as if it changed the model. The verdict moved from ≈ 867 to ≈ 650 clocks;
- **the flush at the frame's end** was missing (58 PSDU bits still in the window when the
  tail is reached), and the draft's "an octet in eight clocks" contradicted its own last
  line; the longest one-symbol frame is 24 octets, not 20;
- **the FFT "in place on a 64-word memory with two butterfly units"** cannot be built on
  the memory wrappers; left to `fft64`'s G0 with the pipelined alternative it proposed.
Also corrected after it: the CORDIC counted iterative in time and pipelined in area; the
draft's third "cannot be built" item, which was a scheduling fact, replaced by the list
of Context 3; the metric's 13 bits; 58 bits a survivor, the tree's pipelining, the probe
beside the model work; the scale's multipliers and the products' store in the area; the
seed, SERVICE, the statuses, the time mark and the inputs in decision 5; ADR 0008's
decision 6 in the consequences; "counted twice" (ADR 0008 put the declaration at the
frame's end); what the registered ready does and does not settle. Its own times in the
new table are estimates of the same kind as the draft's. Not checked by it: any G2 or
packet error figure, any timing or synthesis, the traceback memory's area and latency,
fading or a dying signal for the moves' statistics, the standard's state machine figure.
