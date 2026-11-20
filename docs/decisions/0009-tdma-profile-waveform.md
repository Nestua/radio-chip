# ADR 0009 — The long-range TDMA profile: the colleague's work as the lab and field prototype; the silicon's waveform stays 802.11's; two bands

Status: **proposed** · 2026-11-20 · PHY architecture, licences and CE/RED (risk points): the
founder decides. Does not amend ADR 0001, 0003 or 0004.
Sources: `docs/reviews/2026-11-20-e021-waveform-comparison.md` (the comparison run); the
review of `waverage/phy-sim` of 2026-11-18 (reproduced on the build server; its record is
not yet in this repository, so its figures are this ADR's word until it is committed); two
independent reviews of drafts of this ADR (2026-11-20), the second of which read the
regulatory texts themselves; two directions of the founder of 2026-11-20 (below). The
number 0008 is kept for the receiver-front hand-off ADR that ADR 0007 announced.

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
without line of sight, with a pilot-only receiver. Synchronisation, gain control, the
oscillator and the amplifier were ideal in the run.

**The implementation (our review of 2026-11-18).** His modem, 232 harness tests, every RTL
block and his 1 % FER point reproduce. The state of the head start, exactly: the RTL is
bit-exact with his model in simulation and runs on an Arty Z7-20 through an on-chip channel
emulator — a digital loop, no radio; the board run used older RTL; there are no saved
timing or utilisation reports (block RAM 124 of 140); the gain control is an ideal one, so
none exists; his 6-bit LDPC decoder has an error floor near 1e-4; the RTL test fails at the
transmitter under one simulator (a missing reset); vectors are not deterministic and the
committed ones are stale. It is real, careful work that has not yet met an antenna.

**Regulation** (read in Decision (EU) 2022/179 as replaced by 2022/2307, ETSI EN 301 893
V2.2.1 and ETSI EN 300 328 V2.2.2 by the second reviewer; a test lab and the national
regulator confirm before anything is built on it).

- 5150–5250 MHz: unmanned aircraft may use 5170–5250 MHz, at 200 mW e.i.r.p. and
  10 mW/MHz, with or without power control, no radar detection required. The bands above
  5250 MHz are closed to them (5250–5350 indoor only; 5470–5725 "nor use for UAS").
- **The same row's note 1: "If used outdoors, equipment shall not be attached to a fixed
  outdoor antenna, fixed infrastructure or to the external body of road vehicles."** On a
  plain reading a dock with a mast or a dish may not transmit in this band at all. It may
  receive there.
- The 10 mW/MHz applies to the bandwidth occupied: 19.8 dBm for 242 tones in 10 MHz
  (18.9 dBm for n10's 199), 22.8 dBm for the 802.11ax 20 MHz format. Channels of 10 MHz are
  allowed, on the 20 MHz raster: four centre frequencies in 5170–5250.
- EN 301 893 requires listening before talking. The form a TDMA link can meet is
  **frame-based equipment**: a fixed frame period of 1 to 10 ms, changed at most once per
  200 ms; one channel check of at least 9 µs by the initiating device immediately before
  each period; if busy, nothing is sent in that period; occupancy at most 95 % of the
  period, then idle for at least 5 % of the occupancy (100 µs at least). A responding
  device sends on a grant, within 18 µs or after a channel check of its own. Short control
  signalling — up to 2.5 ms in 50 ms — needs no check.
- 5725–5875 MHz: 25 mW e.i.r.p. as a short-range device in the harmonised position (from
  CEPT material; some countries allow more under national conditions). Not a 10 km band.
- 2.4 GHz (EN 300 328): 100 mW e.i.r.p., 10 dBm/MHz, **fixed outdoor use allowed**;
  equipment that does not listen is held to a medium utilisation of 10 %; listening
  equipment has its own frame-based rules (a check of at least 18 µs) and short control
  signalling of 10 % in 50 ms.
- An outside reference: one maker's current FPV video units are specified at 10 and 15 km
  under FCC power and 6 and 8 km under CE (5.1 GHz below 23 dBm, 5.8 GHz below 14 dBm),
  with small antennas at both ends.

## Proposed decision

1. **We use the colleague's work, as it is, as the lab and field prototype; we do not
   design a waveform of our own from nothing.** e021 gives no reason to expect a better
   grid than his (we have not searched for one), and what the project needs first — a
   radio in front of a working modem, a real gain control, captures from the air — his
   RTL can give soonest. "Soonest" is an expectation: the design has not met an AD9361.
2. **His RTL is frozen for that purpose, NR code included, and is not rebuilt toward
   802.11.** Changing its code, tones and pilots would be the 802.11ax format built a
   second time outside our model and vectors. What it teaches goes into the golden-model
   path. A flight on n10 is a prototype result; **it does not close R3**, whose exit is
   the roadmap's. Whether an unsold lab prototype with the NR code is a patent question
   at all goes to the lawyer; nothing with that code leaves the lab as a product.
3. **What goes into silicon as the TDMA profile's waveform stays 802.11's, as ADR 0001
   says:** the 802.11ax single-user format with its Doppler-mode midamble, in two modes —
   the standard 20 MHz one, and the same format at half clock in 10 MHz (the format is the
   standard's; that mode is not one of its modes) — under our TDMA MAC. The two waveforms
   being equal in quality, the reasons are:
   - the founder's direction: no code from outside 802.11 (the NR code does not enter the
     chip). This does not make the patent position simple — see Open;
   - it is the Wi-Fi 6 core the roadmap builds regardless: one waveform and one LDPC
     family in the chip, not two;
   - the regulatory limit is per megahertz occupied, so 242 tones may radiate 0.85 dB more
     than n10's 199, and the 20 MHz mode 3 dB more again;
   - the standard is a reference we did not write (non-negotiable 3). Plain 802.11ax
     frames at 20 MHz can be exchanged with a commodity chip; frames with the midamble
     probably cannot (it is optional and we know of no commodity chip that sends it), so
     their reference is a black-box waveform generator. n10's preamble, frame and pilots
     have no reference but their author's model.
4. **If captures from the air show that the standard format loses range that an element of
   n10 would keep** — the short preamble, the third pilot symbol — that element is taken
   and a new ADR amends ADR 0001 for it, on that evidence. This ADR does not presume it.
5. **The code is 802.11's LDPC** (R0-12). Its cost at these frame sizes — three codewords
   of 1944 bits where the run used one of about 5600 — is not measured yet.
6. **ADR 0003 stands as written.** His fixed-point modem is the specification of the
   prototype only. Whatever goes toward silicon is specified in `crates/radio-phy` and
   reaches RTL through `rvec`, under ADR 0004's rules; any block of his that is to be
   reused there is assessed against those rules first, and passes our gates like any
   other. Until then a claim about his code is his, cited as such.
7. **Two bands, by regulation and by the founder's design point: video down in
   5170–5250 MHz and in 2.4 GHz together, command up in 2.4 GHz.** If note 1 means what it says, the dock cannot transmit at 5.2 GHz, and
   2.4 GHz is where a fixed outdoor station may. It is also the better direction for the
   weak link: 6.6 dB less path loss, and the uplink no longer shares the video's superframe.
   The chip's TDMA profile therefore runs a transmitter in one band and a receiver in the
   other at once. A single-band mode stays for a ground end that is portable.
8. **Channel access is designed in from the start, to the union of the two standards:**
   at 5.2 GHz the drone is the initiating device of frame-based equipment — a fixed period
   of at most 10 ms opened by its own channel check, at most 95 % occupied — so no grant
   has to arrive over the weak direction before video may be sent, and the check is made
   by the end without a high-gain antenna. A busy channel costs a whole period: the video
   path must ride through gaps of 10 ms and more. At 2.4 GHz the ground listens before it
   talks (a check of at least 18 µs) or keeps to short control signalling. Which of these
   a lab accepts for an airborne initiator is the first question for it.
9. **Bands, bandwidth and the regulatory domain are parameters**, so that a market with
   other limits is another table and not another design.
10. **Nothing changes for the 802.11 work in progress.** It was the Wi-Fi profile's first
    step; it is now also the TDMA profile's own path. 802.11a at 10 MHz remains the
    fallback for a first long-range link through our own model.

## What it means at 10 km

A budget, not a measurement. Received C/N0 at 10 km in free space with a system noise
figure of 6 dB: EIRP + G_rx + 41.2 dB-Hz at 5.2 GHz, + 47.8 at 2.44 GHz, + 40.3 at 5.8 GHz.
Requirement from the run for 802.11ax with a midamble on the line-of-sight channel at
600 Hz: 75.2 dB-Hz for QPSK 1/2 (5.9 Mbit/s in 10 MHz), 80.4 for 16-QAM 1/2 (11.8),
90.4 for 64-QAM 3/4 (26.6), with the run's NR code; plus an allowance of 9 dB — 3 dB for
what the run idealised and 6 dB of margin, both assumptions until R3 measures them. Not in
the allowance: the drone antenna's pattern and the airframe's shadow toward the dock, a
ground-reflection null, interference (the budget is thermal noise only, and 5150–5250 MHz
is the busiest indoor Wi-Fi band), the shorter 802.11 codewords. The 20 MHz row assumes
equal energy per bit at twice the rate; the 802.11ax format was not simulated at 20 MHz
(802.11a at 20 MHz in the run supports the assumption on this channel).

| Downlink, drone to ground | Power, antenna | C/N0 | Against 9 dB of allowance |
|---|---|---|---|
| CE, 5.2 GHz, 10 MHz, QPSK 1/2: 5.9 Mbit/s | 19.8 dBm, 17 dBi panel | 78.0 | 6.2 dB short: not a dependable link |
| CE, 5.2 GHz, 10 MHz, QPSK 1/2: 5.9 Mbit/s | 19.8 dBm, 23 dBi tracking dish | 84.0 | 0.2 dB short: at the edge |
| CE, 5.2 GHz, 20 MHz, QPSK 1/2: 11.8 Mbit/s | 22.8 dBm, 23 dBi tracking dish | 87.0 | 0.2 dB short: at the edge |
| CE, 2.4 GHz, 10 MHz, QPSK 1/2: 5.9 Mbit/s | 19.8 dBm, 17 dBi panel | 84.6 | 0.4 dB to spare; a busier band |
| Outside CE limits, 5.8 GHz, 16-QAM 1/2: 11.8 Mbit/s | 30 dBm, 23 dBi dish | 93.3 | 3.9 dB to spare |
| Outside CE limits, 5.8 GHz, 64-QAM 3/4: 26.6 Mbit/s | 36 dBm, 23 dBi dish | 99.3 | 0.1 dB short |

These are PHY rates. With the period 95 % occupied and 1 % of frames lost, 0.94 of them is
left when the uplink is in the other band (0.85 if it takes a slot of the same superframe),
before the MAC's headers and before periods lost to a busy channel: about 5.5 and
11 Mbit/s of video for the edge rows. A video packet spans several frames, so its loss
without retransmission is several times the frame loss.

**The uplink.** In 2.4 GHz from a fixed ground station (19.8 dBm e.i.r.p., 2 dBi on the
drone): 69.6 dB-Hz, with the same allowance about **200 kbit/s if it transmits
continuously** — and it may not, without listening: a tenth of that under the 10 %
utilisation rule, more with channel checks. As one slot in 21 of a 5.2 GHz superframe
(a portable ground end) it would be about **4 kbit/s**. Either way it needs a mode some
15 to 17 dB below QPSK 1/2 that none of the waveforms compared has (802.11ax's
extended-range format gives a few dB): repetition or spreading, to be designed.

What moves these numbers under the same limits, all on the receiving side, where the
e.i.r.p. limit does not reach: two receive antennas combined (about 3 dB and protection
from nulls), a low-noise amplifier at the antenna (2 to 3 dB), a larger dish (30 dBi: 7 dB).
With them 16-QAM in 20 MHz at 10 km comes into reach. At 1 km everything has 20 dB more.

### The design point (the founder, 2026-11-20): built-in antennas, four on the drone, four to six on the dock, both bands summed

No tracking dish. The link works with antennas that fit the drone and the dock, as the
market's products do; range comes from several receive antennas combined, a low-noise
front end, both bands carrying video at once, and the rate falling with distance.

An estimate under CE limits: 22.8 dBm at 5.2 GHz and 20 dBm at 2.4 GHz from 2 dBi antennas
on the drone, each in a 20 MHz channel; patches of 8 dBi on the dock; each doubling of
combined receive antennas counted as 3 dB; a system noise figure of 3 dB; 2 dB taken off
the run's fading requirement for the diversity; the same 3 dB and 6 dB of allowance.
**None of the multi-antenna or two-band figures is measured: e021 had one antenna at each
end and one band.** Rates are PHY rates on a coarse ladder (the run's three modes and
halvings below QPSK 1/2, which do not exist yet); thermal noise only.

| Distance | 4 patches: 5.2 GHz | 4 patches: 2.4 GHz | 4 patches: both | 6 patches: both |
|---|---|---|---|---|
| 1 km | 53 | 53 | 106 Mbit/s | 106 Mbit/s |
| 3 km | 24 | 24 | 47 | 47 |
| 5 km | 12 | 24 | 35 | 35 |
| 7 km | 6 | 12 | 18 | 24 |
| 10 km | 3 | 6 | **9** | **18** |
| 15 km | 1.5 | 3 | 4.4 | 9 |
| 20 km | 0.7 | 1.5 | 2.2 | 4.4 |

Distances carry the 6 dB margin. With one antenna pair of 3 dBi at a handheld ground end
the same estimate gives 1.5 to 3 Mbit/s at 5 to 7 km in one band, which is where the maker
quoted above specifies its CE range — a check from outside. At 2.4 GHz the path loses
6.6 dB less, which is why it carries the larger half at range; it is also the band with
the most interference, and an airborne receiver sees all of it, so the 2.4 GHz column is
the one most likely to fall short in the air.

**The uplink stops being the problem:** four receive antennas on the drone give the
2.4 GHz command link about 79 dB-Hz at 10 km, megabits per second if sent continuously and
hundreds of kilobits under the 10 % utilisation rule, without a mode below QPSK 1/2.

What four transmit antennas on the drone do not do: raise the power. The e.i.r.p. limit
counts the gain of antennas transmitting the same signal coherently. They give transmit
diversity, a pattern without the airframe's shadow (the best-placed antennas are used),
and two spatial streams where the signal is strong — the first kilometres, where that
would double the table's rates.

What this asks of the profile and the chip:

- a receiver for up to four antennas per band, combining per subcarrier. The integer data
  path of R0-09 already has the form: u = Σ Z_i·H_i*, g = Σ |H_i|² over the antennas, one
  demapper and one decoder after the sum. Six on the dock is two chips or a wider build:
  open;
- two PHY instances at once, one per band, with the MAC splitting or duplicating the video
  across them (what 802.11be calls multi-link operation, which the roadmap reaches
  anyway) and 2.4 GHz shared in time between video down and command up;
- rate adaptation over the whole ladder per band, with modes below QPSK 1/2;
- the area and power targets of ADR 0008 counted for this configuration, not for one
  antenna and one band;
- on the prototype: four receive chains in each of two bands is four two-channel
  transceivers at the dock (each has one receive synthesiser), not one.

## Why not the alternatives

- **n10 as the chip's TDMA waveform, as it is.** Equal quality in the operating region; a
  second waveform and a second LDPC family in the chip; a code from outside 802.11; no
  outside reference; ADR 0001, 0003 and 0004 to amend. Its 64-QAM advantage lies outside
  what the link budget allows at range.
- **Converging his RTL to 802.11 step by step** (the previous draft of this ADR): double
  work, and replacing his largest block first would spend the head start. Decision 2.
- **802.11a at 10 MHz only.** Equal at QPSK and 16-QAM with a line of sight, already being
  built, the oldest patents, and commodity chips with half-rate modes to test against;
  fails beyond its 1.6 µs guard and has no midamble. The fallback, and the comparison
  point for the lawyer.
- **802.11bd** (10 MHz, LDPC, midambles every few symbols, made for vehicles): a standard
  answer to the Doppler problem that e021 did not include. In the next run; a candidate
  under the same reasoning if it beats the 802.11ax format at 10 MHz.

## Open

- **Patents (risk point, for the lawyer).** The 802.11ax features chosen (its training
  field, the midamble) carry younger patents than an 802.11a/n format would, and the TDMA
  module would ship them before the Wi-Fi 6 profile does. Undertakings to license on FRAND
  terms cover implementations of a standard; a TDMA MAC under an 802.11 PHY, and a
  half-clocked mode, may not be covered as a Wi-Fi product is. Decision 3 keeps to the
  founder's direction (nothing from outside 802.11) but is not shown to minimise exposure;
  the comparison with an 802.11a/n-based format goes into the freedom-to-operate analysis.
- **The import (risk point, licences).** A snapshot at a cited commit, not the history:
  the history holds commits made before the licence and may hold the AD9361 datasheet,
  which is never copied. Before it: written confirmation of who holds the copyright (the
  author or an employer), a licence scan of the tree, whether his RTL or build leans on
  vendor reference designs. One repository or two is a `[meta]` change of its own.
- **The CE test plan (risk point).** Note 1 and the dock; an airborne initiating device;
  the access rules at 2.4 GHz for the uplink; out-of-band limits at 10 and 20 MHz; the
  receiver requirements of EN 301 893 V2.2.1 at 10 MHz; which version of each standard
  carries the presumption of conformity when we test; national differences.
- **To measure next (e022):** the 802.11ax format at 20 MHz with 1.6 and 3.2 µs guard;
  802.11bd; a receiver with phase tracking for every waveform; what the simulator does with
  the line-of-sight Doppler shift; 802.11's LDPC at these frame sizes; two receive
  antennas; an uplink mode 15 to 17 dB below QPSK 1/2; the frequency acquisition range
  against ±2 ppm at each end (0.6 of a tone at 39 kHz spacing).
- **The radio for two bands.** The prototype's transceiver has separate synthesisers for
  transmit and receive but one for both receive channels (to be checked against its
  documentation): transmit in one band and receive in the other fits one device; two
  receive antennas in one band fit one device; receiving in two bands at once does not.
- **Dependence on one person** for the prototype's code, and his agreement to how it is
  used and where it lives. FPGA headroom on his board (block RAM at 89 %).

## Consequences

- ADR 0001 is kept whole. The Wi-Fi 6 core gains a second customer and moves up in
  importance; the TDMA profile needs it, a second band, and an uplink mode nobody has yet.
- Two toolchains live side by side while the prototype does (his: Python, Zig,
  Verilog-2005, cocotb); a standing cost on a team small on purpose, bounded by decision 2
  (his RTL is not developed further here).
- `ROADMAP.md` and `BACKLOG.md` get the prototype track (radio, gain control, captures),
  e022 and the two-band items when this ADR is accepted.

## Sources for the regulatory facts

Commission Implementing Decision (EU) 2022/2307 replacing the Annex of 2022/179 (OJ L 305;
Table 1 and its note 1); ETSI EN 301 893 V2.2.1 (2024-11), clauses 4.2.2.2 and 4.2.7.3;
ETSI EN 300 328 V2.2.2 (2019-07); ERC Recommendation 70-03 material for 5725–5875 MHz
(secondary); the maker's published specification pages. Read by a reviewer, not by a test
lab; none of it replaces the lab's reading.
