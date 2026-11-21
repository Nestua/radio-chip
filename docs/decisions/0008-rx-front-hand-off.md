# ADR 0008 — Receiver front, amended: hand-off of samples, DC removal for the decoder, the gain's hold, latency, folding, area and power

Status: **proposed** · 2026-11-21 · PHY architecture (risk point). Amends ADR 0006; announced
by ADR 0007 (decision 2) and asked for by ADR 0009 (the count for one to eight chains, the
sample interface). The founder decides each item; the text is the proposal put to them.
Sources: `docs/gates/rx_front.md` (2026-11-21, G0 and results), `docs/gates/rx_ofdm.md`
(2026-11-20, stage B part 2), `docs/gates/silicon_probe.md`, the reports
`rx_front_dc_cfo.txt`, `rx_front_dc_cand.txt`, `rx_front_dc_hold.txt`, `rx_ofdm_agc_*.txt`.

Notation as in ADR 0006: R clocks per sample (R = 8: 160 MHz at 20 MS/s), T1 the first sample
of the first long training symbol, L = 64 the decision latency. Cycle counts and areas of
things not yet built are estimates and are marked so; the gates of the blocks measure the
real ones.

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

1. **A second history, of the ADC's samples** (12 bits, 512 deep: 12.3 kbit, a two-port
   macro), beside the history of the DC blocker's output that the decision reads. It has its
   own read port, so ADR 0006's port schedule is untouched.
2. **The DC offset is estimated once per packet from the two long training symbols**
   (`ltf_dc_offset_fixed`): behind a CFO the field repeats after 64 samples turned by the
   CFO's phase over 64 samples, the offset repeats unturned, so the 64 differences
   x[n + 64] − e^(jφ)·x[n] hold the offset times a known number and nothing of the signal,
   whatever the channel and whatever the transmitter leaks at its own carrier. 128 reads of
   the new history, 64 uses of the rotator the decision engine already has, one division.
   Where the CFO is below 3.1 kHz the division is skipped and the offset left in: there it
   lies within a hundredth of a subcarrier spacing of the unused subcarrier.
3. **Replay:** the samples from the FFT window of T1 on are read back from that history, the
   offset subtracted (13 bits), turned by the CFO word with the front's rotator
   (`correct_fixed`) and handed on at up to one sample per clock, eight times real time at
   20 MS/s, until the reader has caught up with the antenna; from then on at the antenna's
   pace. One stream with valid and ready, a marker on the first sample of T1's window, and
   the CFO word beside it.
4. **Width: 13 bits, the FFT word as it is** (`in_bits` 13, `in_shift` 1): the rotator's
   product is rounded to the ADC's last bit instead of to twice it.
5. **The DC blocker stays where it is**, in front of the detector and the acquisition, at
   its pole of 2⁻⁷. Its controls at a gain change (`hold`, `rescale`, `restart`, float model
   only) are not built: nothing after the acquisition reads its output any more.

**Why not the blocker's output** (which every measurement before 2026-11-20 assumed):

| What the decoder is handed (54 Mb/s, frames lost of 2000) | no offset, AWGN 20 dB | no offset, TGn D 40 dB | offset −30 dBFS, AWGN 40 dB | offset −20 dBFS, AWGN 40 dB |
|---|---|---|---|---|
| the blocker's output, pole 2⁻⁷ (today), integers | 412 | 251 | 19 | 98 |
| the same, pole 2⁻⁹ / 2⁻¹¹ (`rxagc pole`) | 200 / 172 | 55 / 14 | — | — |
| the samples before it, nothing removed (float) | 167 | 0 | 377 | 1636 |
| the blocker's estimate frozen at the acquisition (`rxfront agcdc`, float, other frames) | worse than the blocker left running (35 of 2000 against 8 at 25 dB) | | | |
| **the offset from the long training field, integers, 13 bits** (`in_shift` 2 in this run; 13 / 1 in `rx_front_dc_hold.txt`) | 170 | 0 | 0 | 0 |

(The offset is `GainDc::per_index`, at that level at every gain; the packet settles at
−10 dBFS. `rx_front_dc_cand.txt` has the offset before the gain, 6 Mb/s, TGn D and the 12-bit
hand-off as well.)

- The blocker's cost is the CFO's. With the clock offset below 20 ppm (116 kHz at 5.8 GHz)
  it loses nothing; between 30 and 40 ppm it loses 919 of 2000 frames at 54 Mb/s and 20 dB
  where the samples before it lose 163, and 912 against none through TGn D at 40 dB
  (`rx_front_dc_cfo.txt`). The explanation that fits (reasoned, not measured apart from
  that dependence): the blocker is a high-pass with a corner at 25 kHz and a memory of 128
  samples, placed before the CFO is taken out; at 40 ppm the subcarrier next to DC lies
  80 kHz from it, on the filter's skirt, and the filter's estimate carries that
  subcarrier's earlier symbols into the present one. A slower pole narrows the skirt and
  does not remove it.
- Leaving the offset in is not an option either: 20 dB below the packet it loses a fifth of
  the 54 Mb/s frames.
- The estimate from the long training field removes both: with no offset it loses what the
  samples before the blocker lose, and under every offset of the table it loses the same.

**Why 13 bits:** the lower edge of the input level is the hand-off's last bit
(`rx_ofdm_agc_level.txt`). Rounded to twice the ADC's last bit (12 bits) the receiver needs
the radio's noise at the largest gain at or above −42 dBFS at the ADC; with the last bit
kept the edge is about 6 dB lower. The 13th bit costs one bit of the stream and of the FFT's
input rounding and nothing in the FFT word, which is the same in both. `in_shift` 2 (a
wider FFT word) buys nothing measurable more and costs a bit of headroom.

**What this is not.** One offset per packet: it is right while the gain is still from T1 to
the packet's end (decision 2) and the offset does not drift inside a packet (no data on the
AD9361's residual offset exists here; R1's captures decide). A gain change inside the long
training field spoils it, as it spoils the timing (ADR 0007). The estimate is whole ADC LSB.
The offset the gain control itself sees is not removed by any of this: one 10 dB above the
packet's noise floor at the largest gain blinds the model's control (`rx_front.md`,
2026-11-19), which is a requirement on the radio's own offset correction, not on this path.

## Decision 2 — who holds the gain

**Proposed:** the front raises a `hold` to the gain control when it declares an acquisition
and drops it at the packet's end — the end computed from SIGNAL's length, at once if SIGNAL
does not decode, and by a watchdog at the longest PPDU. Between its own lock and the
declaration the control is on its own.

- Without a hold the model's control leaves its lock inside the packet on a peak of the
  data: in 2 to 4 % of the 54 Mb/s frames of 1000 octets and 37 to 60 % of the 6 Mb/s ones
  (`rx_front.md`, 2026-11-19). Every run since then held the gain from the control's first
  lock inside the packet, which no hardware can do: the control does not know a packet.
- Held from the latest declaration instead (T1 + 287), with the control free until then:
  0 to 5 more frames of 2000 are lost at 54 Mb/s (AWGN 25 to 60 dB: 2, 1, 5, 0; TGn D 40
  and 60 dB: 2, 4) and none at 6 Mb/s, where a quarter of the frames would leave the lock
  later without the hold (`rx_front_dc_hold.txt`).
- The requirement of ADR 0007 stands beside it: the gain still from the end of the STF.
- Whether the AD9361's fast-attack state machine takes such a hold from a pin, and how fast,
  is to be read from its documentation and tried at R1. If it does not, the hold is a
  register write over SPI and too slow; the fallback is a lock that is not left on an
  overload for the length of a PPDU, which costs the packets that arrive stronger during
  one.

## Decision 3 — the latency budget

SIFS is 16 µs at 20 MHz (32 µs at 10 MHz, 64 µs at 5 MHz; IEEE 802.11-2020 Table 17-21):
from the end of the frame at the antenna to the first sample of the response. The shortest
frame that must be answered is one data symbol long at 54 Mb/s (an RTS of 20 octets), so a
frame can end at T1 + 288, the sample at which its acquisition is declared. The whole frame
is then in the history and nothing has been decoded.

**Proposed budget at R = 8** (clocks of 6.25 ns after the frame's last sample; every figure
an estimate before RTL):

| Step | Clocks | Running total, µs |
|---|---|---|
| DC estimate: 128 reads at two per clock, the sums, one division | ≈ 100 | 0.6 |
| replay of T1, T2, SIGNAL and one data symbol at one sample per clock | 288 | 2.4 |
| FFT of the last symbol and the equaliser behind it | ≈ 150 | 3.4 |
| de-interleaver and Viterbi decoder: the last symbol's 216 bits at one trellis step per clock, and the traceback | ≈ 320 | 5.4 |
| descrambler, FCS, the MAC's decision to answer | ≈ 100 | 6.0 |
| left for the transmitter's own latency and the radio's turn-around | | 10.0 of 16 |

- **Requirement on everything behind the hand-off: one sample per clock**, eight times real
  time at 20 MS/s, and a Viterbi decoder at one trellis step per clock (160 Mb/s against the
  54 it must sustain). A decoder path that only keeps pace with the antenna would stay
  15 µs behind for the whole packet and miss the answer.
- A frame of ordinary length is caught up with long before it ends: the backlog of 300
  samples shrinks by seven samples in every eight clocks, so the reader is at the antenna
  43 samples (2.1 µs) after it starts.
- The 10 and 5 MHz modes have twice and four times these margins at the same clock.
- L = 64 stays. The fallbacks of ADR 0006 that lengthen it to 71 cost 0.35 µs of this budget.
- Not in the table because nothing is known yet: the radio's turn-around from receive to
  transmit, and the transmit chain's latency. Both are R1's to measure; the 10 µs are what
  is left for them.

## Decision 4 — folding the multipliers

The probe (`silicon_probe.md`, IHP 130 nm, 160 MHz): `ltf_corr` is 1.22 mm² routed, of
which 0.39 mm² are its 7932 flip-flops and the rest the 128 constant multiplications and
their adder tree; `det_a` is 0.81 mm², 0.11 mm² of it flip-flops, the rest two squares, a
product and an 80-bit comparison, and it misses the clock by 2.5 ns. Both do their
arithmetic once per sample, one clock in eight. ADR 0006 adds a second correlator for the
decision.

| Option | Correlators | Detector | Cost |
|---|---|---|---|
| A. ADR 0006 as written | 2 × 1.22 mm² | 0.81 mm², fails timing | none |
| B. one unfolded array shared with the decision (ADR 0006's deferred idea) | ≈ 1.4 mm² | as A | a multiplexer on the array's critical path; the main search must pause or the decision wait |
| **C. both correlators folded over R, the detector's arithmetic spread over R** | ≈ 2 × 0.4 mm² (estimate) | ≈ 0.35 mm² (estimate), its long path cut in R pieces | `ltf_corr` and `det_a` rewritten in RTL; their results arrive R clocks later |

**Proposed: C.**
- A folded correlator has 128/R = 16 multipliers that take a tap from a small table each
  clock, in place of 128 wired ones; the window of 128 samples (3072 flip-flops, 0.15 mm²)
  stays. The decision's correlator is a second folded one on a window of 64, not a share of
  the first: the main search runs while a decision is pending (PR #24) and at R = 8 a folded
  array has no idle clock to lend.
- **The model does not change, and neither do the vectors**: the sums are the same integers.
  The committed vectors of `ltf_corr` and `det_a` gate the rewritten blocks as they gate
  today's; only the latency in clocks moves, and with it the testbenches' timing.
- The schedule of ADR 0006 moves with it: an event is known about R clocks later (D = 5
  samples instead of 3 at R = 8), and the decision's 24 correlations take 8 clocks each
  instead of one. By the same arithmetic as ADR 0006's table the work comes to about 430
  clocks against (64 − 5)·8 = 472, 9 % of slack; with L = 71 (lossless, inside the bound
  already gated) 528 and 19 %. **The cycle-exact schedule is the first deliverable of
  part d step 3, before any RTL of the decision engine**, and decides between L = 64 and 71.
- R = 6 (ADR 0006's fallback clock) folds over 6: 22 multipliers. Folding ties the block to
  a known R; R stays a parameter of the RTL and the gate runs at two values, as ADR 0006
  already requires.
- On an FPGA the same change takes the correlators from about 250 DSP blocks to 32.
- The correlators' arithmetic is enabled only while a search is open. Listening, the front
  clocks the detector, the windows and the DC blocker.
- `det_a`'s retune (R0-08c) and this rewrite touch the same block: one piece of work, the
  retune's algorithm first in the model, then the folded RTL once.

Not proposed: narrower words. The G2 sweep left Z ≥ 10, T ≥ 6, K ≥ 15 passing one at a
time and the joint lower set failing (`rx_front.md`); what narrowing is safe is measured
again as packet error rate with R0-08c's retune, not here.

## Decision 5 — area and power of one chain's front

The founder's starting point (2026-11-19) was ≤ 1 mm² and ≤ 150 mW on 130 nm, to be
justified here. **The numbers do not support it; proposed instead: ≤ 2 mm², and ≤ 250 mW
listening and ≤ 600 mW during an acquisition by the flow's estimator, at 130 nm and
160 MHz.**

| Part (130 nm, routed or estimated) | Today | After decisions 1 and 4 |
|---|---|---|
| search correlator | 1.22 mm² | ≈ 0.4 (estimate) |
| decision correlator (ADR 0006) | + 1.2 (not built) | ≈ 0.35 (estimate) |
| detector | 0.81 | ≈ 0.35 (estimate) |
| latch with its CORDIC; the delay line 4672 flip-flops today | 0.71 | ≈ 0.3 with the line as a macro (estimate) |
| DC blocker, two rotators, MAC, squarer, the DC estimate's divider | 0.1 + not built | ≈ 0.25 (estimate) |
| memories: two histories and the delay line, 30 kbit | not built | ≈ 0.15 (not from the PDK's macro views) |
| **sum** | ≈ 4.2 with ADR 0006 complete | **≈ 1.8, ± 30 %** |

- A large part of what is left is flip-flops and memory: two windows, the histories, the
  pipeline. A front of 1 mm² at 130 nm would need a search without a 128-sample matched window, which
  is the algorithm, not its implementation.
- **Power.** The probe's 1.1 W assumes the flow's default switching in every cell on every
  clock. An unfolded array switches when its window shifts, one clock in eight, so that
  figure overstates today's blocks; folded logic does switch every clock. Scaled from the
  probe's power per mm² (not a run of the flow) the folded front is about 0.6 W during an
  acquisition and about 0.2 W listening. A figure to budget with needs the switching of a simulated packet (a gate of
  the folded blocks), not this estimator.
- 130 nm is the test vehicle for blocks (ROADMAP). At the node the eight-chain design is
  planned for, typical scaling (not from a PDK of ours) makes one front of the order of
  0.07 mm² and 20 mW listening; eight of them 0.6 mm² and 0.2 W. Against ADR 0009's first
  count (2 to 4 million gate equivalents) eight fronts of 0.18 million each are 1.4 million,
  its largest single item: whether every chain needs its own search, or a
  link searches on the chains that last heard best, is R0-17's question.

## Decision 6 — the sample interface and the pool of chains

**Proposed:**
- Inside the chip a chain is one stream: 12-bit I and Q with a valid, one sample in R
  clocks. Every block of this ADR is written against that stream.
- **The acquisition always runs at 20 MS/s on the primary 20 MHz channel**, whatever the
  channel's width: the legacy preamble is sent on every 20 MHz of a wider channel. The front
  of this ADR is therefore one per chain and does not grow with the bandwidth.
- **What is pooled is everything behind the hand-off.** Decision 3 makes it take one sample
  per clock: one such data path carries 160 MS/s, which is eight chains of 20 MHz, two of
  80 MHz or one of 160 MHz (ADR 0009's product). The number of data paths is the build's
  parameter; with one path and eight chains nothing is left to catch up with, so a build
  that must answer within SIFS on eight chains needs two. R0-17 sizes it.
- **Pins:** the prototype's transceiver has a source-synchronous LVDS mode that carries both
  of its channels on six data pairs, a frame and a clock in each direction (16 pairs a
  device; to be checked against its documentation). Four devices are 64 pairs (128 pins) for both
  directions, against 192 pins for the receive samples of eight chains in parallel. First silicon takes that format; a
  multi-gigabit serial link (JESD204) needs a SerDes we do not have and is not proposed.

## Decision 7 — the CFO word and the timing tracker

**Proposed:** the timing tracker's prior rate is the CFO word times one number per channel,
fs/fc, written by the MAC when it tunes (a 16-bit mantissa is ample: the CFO word itself is
good to a few hundred hertz of 232 000). The
product is formed once per packet where the hand-off's stream starts. Today that conversion
is the one floating-point step between the ADC and the PSDU (`FixedTracking::from_clock_offset`);
its integer form is a model change with its own G1 and G2, before the tracker's RTL.

Not decided here: narrowing the tracker (R 6, M 8, K 8, V 4 showed no loss on 1000-octet
frames, `rx_ofdm.md`). It is measured on the full grids and on the longest frame before its
RTL; the defaults stand until then.

## Consequences

- **Model:** `ltf_dc_offset`, `ltf_dc_offset_fixed`, `remove_dc_fixed` (this PR, with their
  gates). `FixedOfdmConfig`'s default hand-off becomes 13 / 1 and `rxper` hands the decoder
  this stream (a follow-up with its G2 rerun). The tracker's start from the CFO word in
  integers (decision 7). `RxFrontFixed` gains the second history and the replay as a
  streaming model before part d step 4.
- **RTL, in this order:** the cycle-exact schedule of the folded decision (decision 4); the
  folded `ltf_corr`; `det_a` after R0-08c's retune, folded; then part d steps 2 to 4 as
  planned, with the hand-off as a new last step.
- **G3 so far stays valid** for `dc_block`, `nco`, `cordic`, `rx_latch`, the memories.
  `ltf_corr` and `det_a` keep their vectors and get new RTL.
- **Requirements on the radio, to check at R1:** the gain still from the end of the STF and
  held on a pin from the declaration (decision 2); its noise at the largest gain at or above
  −48 dBFS at the ADC with the 13-bit hand-off; its own offset correction leaving less than
  the packet's noise floor at the largest gain in front of the gain control's detectors;
  its turn-around inside the 10 µs of decision 3.
- **Backlog:** R0-08d (3) is rewritten to the order above; R0-17 takes the pool's sizing and
  the question of a search per chain.

## Open

- The AD9361's residual offset and its drift inside a packet (R1): the one-offset-per-packet
  assumption.
- The schedule's slack at L = 64 with folded correlators (first deliverable of part d step 3).
- Power from simulated switching, not from the flow's default.
- The macros' sizes and timing at 160 MHz (`memory.md` checked their ports only).
- TGn B and E, frames other than 1000 octets, the upper edge of the level: not measured.
