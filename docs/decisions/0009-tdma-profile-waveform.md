# ADR 0009 — The long-range TDMA profile: the colleague's work is taken; the waveform that goes into silicon stays 802.11's

Status: **proposed** · 2026-11-20 · PHY architecture, licences and CE/RED (risk points): the
founder decides. Does not amend ADR 0001, 0003 or 0004.
Sources: `docs/reviews/2026-11-20-e021-waveform-comparison.md` (the comparison run); the
review of `waverage/phy-sim` of 2026-11-18 (reproduced on the build server; its record is
not yet in this repository — to be committed with this ADR's follow-up, until then its
figures are this ADR's word); an independent review of the first draft of this ADR
(2026-11-20), which changed its decision; two directions of the founder of 2026-11-20
(below). The number 0008 is kept for the receiver-front hand-off ADR that ADR 0007 announced.

## Context

ADR 0001: long range comes from a TDMA MAC profile on the standard OFDM PHY. A colleague
has since shared a complete custom OFDM video PHY (`waverage/phy-sim`, Apache-2.0 since
07b53c5): a 10 MHz, 256-point waveform ("n10") with its own preamble and pilots, the 5G NR
LDPC code, a TDD superframe of 22 downlink frames and 1 uplink frame, a fixed-point modem
with bit-exact twins, and the radio's digital part in RTL.

The founder asked for a decision: is his work good enough to take, or do we go the whole
way from nothing and do it better? And gave two directions on 2026-11-20: **no patent
exposure beyond the patents Wi-Fi needs anyway**, and **the regulatory question is to be
researched and decided here**.

## What we know

**The waveform (e021).** The agreed criterion — n10 within about 0.5 dB of the 802.11ax
numerology at equal energy per bit in fading — is met. But the run does not show n10
better anywhere a 10 km link operates: at QPSK and 16-QAM with a line of sight every 10 MHz
waveform, plain 802.11a at half clock included, needs the same energy per bit within
0.5 to 1 dB, and n10 with three pilot symbols and 802.11ax with a midamble are
indistinguishable. n10's clear wins are at 64-QAM at a Doppler of 2000 Hz (103 m/s) and
without line of sight, with a pilot-only receiver; it carries 14 to 21 % less than the
802.11ax numerology in the same channel. Synchronisation, gain control, the oscillator
and the amplifier were ideal in the run.

**The implementation (our review of 2026-11-18).** His modem, 232 harness tests, every RTL
block and his 1 % FER point reproduce. The state of the head start, exactly: the RTL is
bit-exact with his model in simulation and runs on an Arty Z7-20 through an on-chip channel
emulator — a digital loop, no radio; the board run used older RTL; there are no saved
timing or utilisation reports (block RAM 124 of 140); the gain control is an ideal one, so
none exists; his 6-bit LDPC decoder has an error floor near 1e-4; the RTL test fails at the
transmitter under one simulator (a missing reset); vectors are not deterministic and the
committed ones are stale. It is real, careful work that has not yet met an antenna.

**Regulation (researched 2026-11-20; sources at the end; to be confirmed by a CE test lab
before anything is built on it).**

- 5 GHz RLAN bands in the EU (Decision (EU) 2022/179, harmonised standard EN 301 893):
  unmanned aircraft may use **5170–5250 MHz only**, at **200 mW e.i.r.p. and 10 mW/MHz**.
  The bands above 5250 MHz are closed to them. A 10 MHz signal is therefore limited to
  100 mW; the full 200 mW needs 20 MHz.
- EN 301 893 requires listening before talking, in one of two forms. The one a TDMA link
  can meet is **frame-based equipment**: a fixed frame period of 1 to 10 ms; one channel
  check by the initiating device before each period; occupancy of at most 95 % of the
  period, then idle for at least 5 %; within the occupancy, transmissions with gaps of at
  most 16 µs need no new check. If the check finds the channel busy, the period is lost.
- 5725–5875 MHz: 25 mW e.i.r.p. as a short-range device. Not a 10 km band under CE.
- 2.4 GHz (EN 300 328): 100 mW e.i.r.p.; equipment that does not listen is held to a
  medium utilisation of 10 %, so listening is needed there too.
- An outside reference for what these limits allow: the market leader's current digital
  video link is specified at 10 to 15 km under FCC power and **6 to 8 km under CE**
  (5.1 GHz below 23 dBm, 5.8 GHz below 14 dBm), with small antennas at both ends.

## Proposed decision

1. **We take the colleague's work. We do not start again.** A custom waveform designed
   from nothing would not be better than his: nothing in e021 points to a grid that beats
   it, and a year would go on arriving where he is. What can be done better is the
   implementation, and that is done to his code, not instead of it (4).
2. **What goes into silicon as the TDMA profile's waveform stays 802.11's, as ADR 0001
   says:** the 802.11ax single-user format with its Doppler-mode midamble, at 20 MHz and
   clocked at 10 MHz, under our TDMA MAC. Reasons, none of them the waveform's quality,
   where the two are equal:
   - the founder's direction on patents: one set of essential patents, Wi-Fi's;
   - regulation asks for a 20 MHz mode, and n10 at twice its clock is the 802.11ax 20 MHz
     numerology anyway;
   - it is the Wi-Fi 6 core the roadmap builds regardless: one waveform in the chip, not
     two, and 14 to 21 % more rate in the same channel;
   - it keeps non-negotiable 3: the standard is the reference we did not write, and at
     20 MHz a commodity Wi-Fi 6 chip can be the other end of a test (G1 and G5). n10's
     preamble, frame and pilots have no reference but their author's model.
3. **n10 is the prototype and the path there.** The first flying link (R3) is attempted on
   his RTL with a radio in front of it, because it exists. From there n10 is moved toward
   the 802.11ax format a step at a time, each step measured: 802.11n's LDPC in place of the
   NR code; 242 tones; the HE long training field and midamble in place of the block
   pilots; the HE preamble. If a step costs range that captures from the air show we need
   — the short preamble, the third pilot symbol — that element is kept and a new ADR
   amends ADR 0001 for it, on evidence. This ADR does not presume it.
4. **His code comes in through our gates, and the defects are ours to fix:** a real gain
   control (R0-07e's); a decoder without the floor; deterministic vectors under a CI diff;
   saved synthesis and timing reports; the reset. Until a block has passed a gate here, a
   claim about it is his, cited as such. "Fast path to R3" is an expectation, not a
   result: the design has to meet an AD9361 first.
5. **The code is 802.11's LDPC** (R0-12, already modelled), in the prototype as soon as the
   comparison at n10's block lengths has been run, and in anything that leaves the lab
   from the start. The NR code does not enter the chip.
6. **ADR 0003 stands as written.** His fixed-point modem is the specification of the
   prototype only. Whatever goes toward silicon is specified in `crates/radio-phy` and
   reaches RTL through `rvec`, under ADR 0004's rules; his RTL is assessed against those
   rules block by block before any of it is reused there.
7. **The TDMA MAC is designed as frame-based equipment of EN 301 893 from the start:**
   a superframe that is the fixed frame period (at most 10 ms), opened by one channel
   check, at most 95 % occupied — his 22 + 1 frames of 432 µs are 9.94 ms and become 20 + 1
   or fewer — with turnarounds inside it of at most 16 µs or a check of their own. A busy
   channel costs a whole period: the video path must ride through gaps of 10 ms and more.
8. **Bands and bandwidth are parameters of the profile:** 5170–5250 MHz and 2.4 GHz for
   the CE variant, 10 and 20 MHz; the regulatory domain (limits, channel access) is
   configuration, so that a market with other limits is another table and not another
   design.
9. **Nothing changes for the 802.11 work in progress.** It was the Wi-Fi profile's first
   step; it is now also on the TDMA profile's own path. 802.11a at 10 MHz remains the
   fallback for a first long-range link.

## What it means at 10 km

A budget, not a measurement. Received C/N0 at 10 km, free space, receiver noise figure
6 dB (a system figure, front-end loss included): EIRP + G_rx + 41.2 dB-Hz at 5.2 GHz,
+ 47.8 at 2.44 GHz, + 40.3 at 5.8 GHz. Requirement from the run (n10-p3 on the
line-of-sight channel at 600 Hz; 802.11ax with a midamble is within a dB): 74.3 dB-Hz for
QPSK 1/2 at 5 Mbit/s in 10 MHz, 79.7 for 16-QAM 1/2 at 10 Mbit/s, 89.3 for 64-QAM 3/4 at
23 Mbit/s; plus 3 dB for what the run idealised and 6 dB of margin (assumptions of this
ADR until R3 measures them; the 3 dB is optimistic for 64-QAM, and the 6 dB does not cover
a ground-reflection null, which wants antenna diversity or height).

| Case (downlink, drone to ground) | EIRP + G_rx | C/N0 | What it carries |
|---|---|---|---|
| CE, 5.2 GHz, 10 MHz: 20 dBm, 17 dBi panel on the ground | 37 dBm | 78.2 | QPSK 1/2 with 3.9 dB of the 9 dB allowance: not a dependable link |
| CE, 5.2 GHz, 10 MHz: 20 dBm, 23 dBi tracking dish | 43 dBm | 84.2 | **5 Mbit/s** (QPSK 1/2), 0.9 dB beyond the allowance |
| CE, 5.2 GHz, 20 MHz: 23 dBm, 23 dBi tracking dish | 46 dBm | 87.2 | **10 Mbit/s** (QPSK 1/2 in 20 MHz) at the same margin, or 5 Mbit/s with 3 dB more |
| CE, 2.4 GHz, 10 MHz: 20 dBm, 17 dBi panel | 37 dBm | 84.8 | **5 Mbit/s**, 1.5 dB beyond the allowance; a busier band |
| Outside CE limits: 30 dBm EIRP, 23 dBi dish, 5.8 GHz | 53 dBm | 93.3 | 10 Mbit/s (16-QAM 1/2 in 10 MHz), 4.6 dB beyond |
| Outside CE limits: 36 dBm EIRP, 23 dBi dish, 5.8 GHz | 59 dBm | 99.3 | 23 Mbit/s (64-QAM 3/4), 1 dB beyond |

After the superframe's uplink share, its idle 5 % and 1 % frame loss, about 0.85 to 0.9 of
these rates is left before the MAC's headers; a video packet spans several frames, so
packet loss without retransmission is several times the frame loss.

**The uplink is the tighter direction.** The EIRP limit applies to the ground station too,
so its dish does not help it transmit, and the drone's antenna is small: 23 dBm EIRP into
2 dBi on the drone is 25 dBm, 66.2 dB-Hz at 5.2 GHz. With the same allowances that is
about **100 kbit/s** (50 kbit/s at 20 dBm in 10 MHz): enough for command and telemetry,
but it needs a mode some 17 dB below QPSK 1/2 that none of the waveforms compared has
(802.11ax's extended-range format gives a few dB). To be designed: repetition or spreading
for the uplink, or gain and diversity on the drone.

So under CE limits 10 km is at the edge — beyond what the market leader specifies — and is
reached only with a high-gain tracking antenna on the ground: about 5 Mbit/s of video in
10 MHz, about 10 Mbit/s in 20 MHz, and a narrow uplink.

## Why not the alternatives

- **n10 as the chip's TDMA waveform, as it is.** Equal quality, less rate, a second
  waveform and a second LDPC family in the chip, patents outside Wi-Fi's set, no outside
  reference, and ADR 0001, 0003 and 0004 to amend. It would be chosen for a head start
  that decision 3 uses anyway.
- **Our own waveform from nothing.** Decision 1.
- **802.11a at 10 MHz only.** Equal at QPSK and 16-QAM with a line of sight and already
  being built; fails beyond its 1.6 µs guard and has no midamble. The fallback, not the
  profile.
- **802.11bd** (10 MHz, LDPC, midambles every few symbols, made for vehicles): a standard
  answer to the Doppler problem that e021 did not include. To be added to the next run;
  if it beats the 802.11ax format at 10 MHz it is a candidate under the same reasoning.

## Open

- **Patents (risk point, for the lawyer).** Undertakings to license on FRAND terms cover
  implementations of a standard. A TDMA MAC under an 802.11 PHY, and still more an
  802.11 code inside a frame that is not 802.11's, may not be covered as a Wi-Fi product
  is. Decision 2 keeps the PHY the standard's for this reason too; the question goes into
  the freedom-to-operate analysis of ADR 0001.
- **The import (risk point, licences).** A snapshot at a cited commit, not the history:
  the history holds commits made before the licence and may hold the AD9361 datasheet,
  which is never copied. Before it: written confirmation of who holds the copyright (the
  author or an employer), a licence scan of the tree, and whether his RTL or build leans on
  vendor reference designs. The LDPC base-graph files are Sionna's and stay out with the NR
  code. One repository or two is a `[meta]` change of its own.
- **The CE test plan (risk point).** Everything under "Regulation" and decisions 7 and 8,
  with a test lab: frame-based access for an airborne link, which end initiates, the
  out-of-band limits at 10 and 20 MHz.
- **To measure next (e022):** the 802.11ax format at 20 MHz with 1.6 and 3.2 µs guard,
  802.11bd, a receiver with phase tracking for every waveform, what the simulator does
  with the line-of-sight Doppler shift, 802.11n's LDPC at these frame sizes, an uplink mode
  17 dB below QPSK 1/2, and the frequency acquisition range against ±2 ppm at each end
  (0.6 of a tone at 39 kHz spacing).
- **Dependence on one person** for the prototype's code, and his agreement to the gates and
  to where the code lives.
- FPGA headroom on his board (block RAM at 89 % before a gain control and a MAC).

## Consequences

- ADR 0001 is kept whole; the roadmap's order is unchanged; the Wi-Fi 6 core gains a
  second customer.
- Two toolchains live side by side while the prototype does (his: Python, Zig, Verilog-2005,
  cocotb); a standing cost on a team small on purpose, accepted for the head start and
  ended by decision 3's convergence.
- `ROADMAP.md` and `BACKLOG.md` get the prototype track, the convergence steps and e022
  when this ADR is accepted.

## Sources for the regulatory facts

Commission Implementing Decision (EU) 2022/179 as amended by (EU) 2022/2307 (EUR-Lex,
consolidated text of 2022-11-25): the 5150–5250 MHz row and its unmanned-aircraft note;
ETSI EN 301 893 (frame-based equipment: summaries by a chip vendor's application note and
by test houses, not yet read against the standard's own text); ERC Recommendation 70-03
annex and a national regulator's page for 25 mW in 5725–5875 MHz; ETSI EN 300 328
(summaries); the market leader's published specification pages for the range and power
figures. None of this replaces the test lab's reading.
