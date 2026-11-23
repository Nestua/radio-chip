# ADR 0010 — Receiver data path, top level: from the hand-off's stream to the PSDU

Status: **proposed** · 2026-11-23 · PHY architecture (risk point). Follows ADR 0008 (the
hand-off's stream, the latency budget of its decision 3, the verdict of its decision 2).
Not yet reviewed, not yet put to the founder.

Scope: the 20 MHz legacy formats (802.11a/g: BCC, eight rates), one chain. LDPC, HT and
later formats, and more than one chain, are named under "Open".

Notation as in ADR 0006 and 0008: R clocks per sample (8: 160 MHz at 20 MS/s); a symbol is
80 samples, 640 clocks of air time. **Every cycle count and every area figure here is an
estimate made before any RTL or schedule model**, from the model's loop bounds and the
stage counts of blocks that exist (`cordic` 18 cycles, a memory 1, `nco` 4). The schedule
model of decision 6 replaces them.

## Context

The front is complete up to its stream (`rx_front`, `rx_handoff`; `docs/gates/rx_front.md`):
beats of 13-bit samples from the first long symbol's FFT window on, a beat a clock while
the reader is behind the antenna, the CFO word and a tag on the first beat, marks at the
end. It expects back one strobe per packet: SIGNAL's verdict (failed, or the number of data
symbols and the patient bit).

What decodes that stream exists as a model only: `radio_phy::rx_ofdm_fixed`
(`decode_ppdu_fixed`: FFT, channel, tracker, rotation, demapper, soft scale, all in
integers, G2 done) and `radio_phy::viterbi` (8-bit soft values, clip 3, window 64, 13-bit
metrics, G2 done). The model takes a whole buffer of samples and returns a PSDU. It says
nothing of when anything happens. Three things it does cannot be built as they stand:

1. **It decodes the data with a traceback over the whole block** (`viterbi::decode`). The
   hardware's variant (`decode_windowed`, window 64, chosen by R0-10's G2 at a loss of
   0.024 dB) is in the model but not in the receiver: no packet error figure of
   `rx_ofdm.md` was measured with it.
2. **It moves the FFT window by a sample when the tracker says so** (`FixedTracking::shift`),
   by indexing the buffer. A stream gives each sample once.
3. **It knows the modulation of the data symbols before it reads them.** In hardware the
   first data symbol is in the buffer long before SIGNAL is decoded.

## Decision 1 — the blocks

The data path of the model, block for block, each with a gate of its own before the top
level, as the front's were:

| Block | Model | Per |
|---|---|---|
| sample buffer | — (decision 2) | sample |
| `fft64` | `fft64_fixed` | symbol |
| channel: Hs, the gains, the packet's shift, Σg | `estimate_channel_fixed` | packet |
| per-packet quotients: the tracker's two coefficients and variance, the soft scale | `tracker_constants`, the scale of `symbol_soft_fixed` | packet (the scale twice: BPSK for SIGNAL, the data's modulation) |
| products u = Z·Hs*, pilots | `symbol_soft_fixed` | symbol |
| tracker: two pair angles, the fit, the drift | `FixedTracking` | symbol |
| common phase, rotation, demapper, soft scale | `symbol_soft_fixed` | symbol |
| deinterleaver, depuncturer | `interleaver`, `viterbi::depuncture` | symbol |
| Viterbi decoder | `viterbi::decode_windowed` | bit |
| SIGNAL: field, parity, rate, length, the check, the verdict | `signal_checked`, `SignalVerdict::of` | packet |
| descrambler, octets out | `rx_ofdm::descramble` | bit |

**Proposed:** one CORDIC of the front's kind in the data path, its own (three angles a
symbol; the front's is busy with latches while a packet is read, because the front goes
on searching). One serial divider for the per-packet quotients and the tracker's
division per symbol. The rotator's table and the CORDIC's are the front's ROMs.

## Decision 2 — the stream's reader: a sample buffer, and how the window moves

**Proposed:** a buffer of 128 samples (26 bits a word, through the memory wrapper) written
by the stream's beats and addressed by the sample's number in the stream. `out_ready` to
the hand-off is a register: "not full". The FFT's 64 samples of a symbol are read from it
at the window's start, nominal plus the tracker's shift.

- A shift of one sample later or earlier is then an address, as in the model. The buffer
  keeps the 16 samples before a window (the guard interval, already received) for a move
  earlier.
- The registered `out_ready` closes the question the hand-off's timing probe left open
  (its path from `out_ready` into the tick's comparisons was not timed).
- While the decoder is behind, the backlog stays in the hand-off's history (512 samples),
  whose overrun rule already says what happens to a reader too slow.
- Marks: a packet's end mark closes the packet (decision 5); an abort (acquisition,
  overrun) or a stop drops what is in flight, and the next first beat starts afresh.

## Decision 3 — the pace: what "one sample per clock" has to mean behind the buffer

ADR 0008 (decision 3) required everything behind the hand-off to take one sample per
clock, and counted the Viterbi decoder at one trellis step per clock. The two do not
meet: at 54 Mb/s a symbol is 216 decoded bits, 216 clocks at a step a clock, against the
80 clocks in which the hand-off can deliver its samples. **The decoder cannot follow a
beat a clock at the high rates whatever the FFT does**; it does not have to, because the
buffer's `out_ready` stops the hand-off.

**Proposed:**
- the Viterbi decoder at one trellis step per clock (64 add-compare-select units), as
  ADR 0008 counted;
- the path in front of it (FFT to soft values) at **one symbol in at most 160 clocks**,
  four times real time, its stages working on consecutive symbols at once; a soft-value
  buffer of two symbols (2 × 288 values of 8 bits) between the two;
- an FFT in place on a 64-word memory with **two butterfly units** (192 butterflies in 96
  clocks).

The slowest stage then sets the pace: the Viterbi at 54 Mb/s (216 clocks a symbol, 3.0
times real time), the tracker's loop at the low rates (about 115 clocks: its division is
65 bits by 43 on a serial divider, and symbol i + 1's prediction needs symbol i's fit).

**The latency this gives** for the shortest frame that must be answered (ADR 0008: one
data symbol at 54 Mb/s, an RTS of 20 octets), in clocks after the declaration's tick t.
Such a frame's last sample is 288 samples after T1; its acquisition is applied 265 to 277
samples after T1 (`rx_front.md`: 189 + w + L to 189 + 2w + L at w = 12), so t is 82 to 178
clocks *before* the frame's last sample (ADR 0008 took the two as one moment):

| Event | Clocks after t | How |
|---|---|---|
| the stream's first beat | 161 | `rx_handoff` as built |
| T2's, SIGNAL's, the data symbol's last sample in the buffer | 289, 369, 449 | a beat a clock |
| FFT of T1, T2, SIGNAL, the data symbol done | 321, 417, 513, 609 | one after another, 96 each, the first when its 64 samples are in |
| channel: Hs, the gains, the shift, Σg | ≈ 530 | two passes over 52 bins after T2's FFT |
| per-packet quotients (three for the tracker, one scale) | ≈ 640 | four divisions on the serial divider |
| SIGNAL: tracker, phase, soft values | ≈ 755, 779, 835 | two angles 36, the fit with its division ≈ 75; rotation of the pilots and one angle 24; 48 subcarriers through a pipeline of ≈ 8 |
| SIGNAL decoded and checked: **the verdict's strobe** | ≈ 867 (5.4 µs) | 24 trellis steps and the re-encoding |
| the data symbol: tracker, the data's scale, soft values | ≈ 870, 897, 953 | the tracker waits for SIGNAL's fit; the scale's division waits for the rate |
| its 182 bits decoded (to the tail's end) | ≈ 1135 | a step a clock; the tail's known state ends the traceback |
| the PSDU's last octet out | **≈ 1140** | the descrambler in the stream |

That is 960 to 1060 clocks after the frame's last sample at the ADC's pins: **6.0 to
6.6 µs**, and with the MAC's decision (≈ 100 clocks, ADR 0008's figure) at most 7.2 µs.
ADR 0008's table had 6.1 and 6.7 µs and left 9.3 µs of SIFS (16 µs) for the radio.
**This table leaves about 8.8 µs.** What ADR 0008 did not count is four transforms one
after another, the channel's passes and the per-packet divisions (some 500 clocks); what
it counted twice is the time between the declaration and the frame's end. What would buy
more, none proposed now: four butterfly units, a second divider, SIGNAL's tracker step
without its fit.

- The verdict comes about 870 clocks after the declaration's tick, which is 462 to 558
  clocks after SIGNAL has ended at the antenna: **8.3 to 8.9 µs after SIGNAL's end**,
  where ADR 0008 expected 7.6 µs. The front's hold has been up since the declaration, so
  the verdict's lateness costs the gain control nothing; what it delays is the patient
  rule and the hold's end for a frame shorter than that (one data symbol or two).
- A frame of ordinary length is caught up with at three to four times real time: the
  backlog of some 300 samples and the latency above are gone within a few tens of
  symbols.
- A reader this fast never comes near the hand-off's overrun (512 samples behind).

## Decision 4 — the Viterbi decoder's survivors

The model's hardware variant decides bit i by tracing back 64 steps from the state with
the best metric at step i + 64 (ties: the lowest state). Two ways to build it:

| | Register exchange | Traceback from a memory |
|---|---|---|
| survivors | 64 registers of 64 bits, every one rewritten every step | a memory of 64 bits a step, some 128 to 192 steps deep (the wrapper) |
| the model's rule (the best state at every step) | as it is | not as it is: a traceback is started once a block of steps, so the model's variant would change and R0-10's G2 be rerun |
| latency of a bit | 64 steps | 128 to 192 steps: up to 0.8 µs more before the last octet, and before a frame's end is known |
| cost (estimate) | 4096 flip-flops and as many multiplexers, about 0.2 mm² of flip-flops alone by the probe's 49 µm² each; all of them toggle every step | about a third of that in area; little toggling |

**Proposed: register exchange**, the model as it is, **and the decoder through the open
flow before the blocks around it are written** (as ADR 0008 did with the folded
correlator): area, timing of the add-compare-select loop at 160 MHz, and a power figure
from simulated switching. If it does not fit, the fallback is the memory and a changed
model, with its G2.

- The best-state selection is a tree of 63 compare-selects on 13-bit modulo metrics every
  step, in both builds; its timing at 160 MHz is not known.
- Nothing here has been through the flow. R0-14 named "a serial-ACS Viterbi if it fits"
  for the silicon probe; that is a different block (one ACS, 64 clocks a step) and would
  not meet this ADR's pace.

## Decision 5 — the data path's outputs

**Proposed:**
- to the front: the verdict's strobe as `rx_handoff` takes it (tag, failed, data symbols,
  patient), once per packet, when SIGNAL has decoded or failed;
- to the MAC: `rx_start` with rate and length when SIGNAL has decoded; then the PSDU's
  octets, descrambled, with a valid and a last; `rx_end` with a status (decoded to the
  length; aborted by a mark, with its kind; SIGNAL failed, with which check). **The frame
  check sequence is the MAC's**: the PHY hands on every octet of a PSDU whose SIGNAL
  decoded, right or wrong, as the standard's PHY does (IEEE 802.11-2020 §17.3.12, to be
  read against the text before the RTL);
- the tracker's prior from the CFO word and the channel's fs/fc register (ADR 0008,
  decision 7), formed once per packet at the stream's first beat.

The octets have no ready: the MAC takes them as they come (at most one in eight clocks).

## Decision 6 — order of work

1. **Model first** (each with its gate; no RTL before them):
   a. `decode_windowed` in `decode_ppdu_fixed`, and the G2 points rerun with it (the loss
      measured in `viterbi.md` was on AWGN with BPSK; through the receiver and in fading
      it has not been measured);
   b. a streaming data path, `RxDataFixed`: beats in, the verdict and the octets out, the
      window's moves on the buffer, held to `decode_ppdu_fixed` frame for frame (as
      `RxHandoffFixed` was held to `handoff_fixed`);
   c. its schedule in clocks (as `rx_decision` and `rx_handoff`): the table of decision 3
      made exact, the memories read at their cycles, the shared CORDIC and divider.
2. **RTL**, in this order: the Viterbi decoder and its run through the flow (decision 4);
   `fft64`; the channel and the per-packet quotients; the products, rotation, demapper and
   scale; the tracker; the deinterleaver and depuncturer; SIGNAL and the verdict; the top
   level behind `rx_handoff`, a cycle at a time against the schedule model.
3. The front and the data path together through timing, and the open flow on both.

## Area (rough, 130 nm; no block through the flow)

| Part | Estimate | From |
|---|---|---|
| Viterbi decoder, register exchange | 0.5 to 0.7 mm² | 4096 + ≈ 900 flip-flops at the probe's 49 µm², as many multiplexers, 64 ACS |
| `fft64`: 64 × 32-bit memory, two butterflies (eight 16 × 12 multipliers) | ≈ 0.25 | `nco`'s 0.075 mm² for one 13 × 11 complex product as the yardstick |
| channel store (52 × 34 and 52 × 16 bits), sample and soft buffers (3.3 and 4.6 kbit) | 0.15 to 0.4 | macros or flip-flops; the macros' sizes not read from the PDK |
| products, rotator, demapper, scale | ≈ 0.2 | one 16 × 17 complex product, a rotator of `nco`'s kind |
| tracker: CORDIC, the sums, two wide products, the divider | ≈ 0.3 | `cordic` 0.115 mm² |
| **sum** | **≈ 1.4 to 1.9 mm², ± 40 %** | |

Against ADR 0008's working target of 2.5 mm² for the front: the data path is of the same
order. No target is proposed for it until the Viterbi decoder has been through the flow.

## Consequences

- **Model:** the three follow-ups of decision 6.1; `rx_ofdm.md`'s packet error figures
  hold for the whole-block traceback until 6.1a is measured.
- **ADR 0008, decision 3:** its requirement "one sample per clock behind the hand-off" is
  replaced by decision 3 here; its SIFS budget by this one's table (about 8.8 µs left for
  the radio where it said 9.3), and its 7.6 µs from SIGNAL's end to the verdict by 8.3
  to 8.9 µs.
- **`rx_handoff`:** no change. Its reader is the buffer of decision 2, with a registered
  ready.
- **Backlog:** R0-09 gains the streaming model and the schedule; the data path's RTL is
  new items in the order of decision 6.

## Open

- LDPC (R0-12's decoder, 20 iterations approved) beside the Viterbi decoder: its place in
  this path, its latency against SIFS, its area. Not in this ADR.
- HT and later formats: more symbols of training, other pilots, 40 MHz and wider.
- More than one chain (ADR 0009, R0-17): one data path per chain up to the equaliser, or
  shared; combining.
- What the radio and the MAC do with the 8.8 µs: the radio's latencies are unknown until
  R1.
- The standard's receive procedure (§17.3.12) read against decision 5.
- Channel-estimate smoothing (`rx_ofdm.md` left it out of stage A): not here either.
