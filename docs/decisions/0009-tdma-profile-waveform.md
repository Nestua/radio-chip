# ADR 0009 — The waveform of the long-range TDMA profile: the colleague's n10, adopted

Status: **proposed** · 2026-11-20 · PHY architecture and licences (risk point): waits for an
independent review and the founder's decision. Amends ADR 0001.
Source: `docs/reviews/2026-11-20-e021-waveform-comparison.md` (the comparison run) and the
review of `waverage/phy-sim` of 2026-11-18 (reproduced on the build server at ff1c483).
The number 0008 is kept for the receiver-front hand-off ADR that ADR 0007 announced.

## Context

ADR 0001 says long range comes from a TDMA MAC profile "on the standard OFDM PHY with
5/10/20 MHz channels". Since then a colleague has shared a complete custom OFDM video PHY
(`waverage/phy-sim`, Apache-2.0 since 07b53c5): a 10 MHz, 256-point waveform ("n10") with a
TDD superframe of 22 downlink frames and 1 uplink frame, a fixed-point modem with
bit-exact twins, and the whole radio in RTL, bit-exact in simulation and placed on a
Zynq-7020 board. Our review reproduced his modem, his 232 harness tests, every RTL block and
his 1 % FER point within 0.05 dB, and found defects of implementation, not of design: an
ideal gain control that is not disclosed, an error floor near 1e-4 in his 6-bit LDPC
decoder, vectors that are not deterministic, board timing claims without saved reports.

The founder's position (2026-11-18): adopt it if the comparison against 802.11 holds up;
where it falls short, take it and do it better. The criterion agreed before the results
(2026-11-19): n10 if it is within about 0.5 dB of the 802.11ax numerology at equal net
rate on the fading channels, or if it wins.

## What the comparison says

From the report, in the words that matter here:

- The criterion is met. n10 with three pilot symbols is not worse than the 802.11ax
  numerology in any fading cell and is the only waveform of the six with no error floor at
  64-QAM 3/4; n10 with two pilot symbols meets it up to 16-QAM.
- At QPSK and 16-QAM with a line of sight, where a 10 km link operates, every 10 MHz
  waveform including plain 802.11a at half clock needs the same Eb/N0 within about 1.4 dB.
  The waveform is not what makes 10 km possible; the link budget is.
- What separates the waveforms is the guard interval (802.11a's 1.6 µs at 10 MHz fails on a
  300 ns channel without line of sight) and how often the channel is measured within a
  frame (64-QAM in fading).
- n10-p3 carries 21 % less than the 802.11ax numerology at one modulation and code (199
  tones against 242, three pilot symbols of 14), 8 % less than n10-p2.
- Not shown by the run: synchronisation, gain control, phase noise and the power amplifier
  were ideal. A 256-point waveform at 10 MHz has a quarter of 802.11a's tone spacing.

## Proposed decision

1. **The long-range TDMA profile uses the n10 waveform**: 10 MHz, 256-point FFT, CP 32,
   the colleague's preamble and frame, his TDD superframe as the starting point of the MAC.
   ADR 0001's sentence becomes: long range comes from a TDMA profile with its own waveform
   on the shared OFDM/LDPC core; 802.11 compatibility is the property of the Wi-Fi
   profiles, not of the TDMA one. Everything else in ADR 0001 stands.
2. **The pilot pattern is a parameter of the link, two values: p2 and p3.** They need the
   same C/N0 on the line-of-sight channel at 600 Hz, p2 carries 9 % more, p3 is the one
   that holds 64-QAM at high Doppler and without line of sight. Which is the default is
   decided on captures from a flying link (R3), not here.
3. **We take the colleague's work; we do not start again.** His repository is brought
   into the project with its history and his copyright notices (Apache-2.0; the LDPC
   base-graph files are Sionna's, attributed separately; the AD9361 datasheet is never
   copied). It is the fast path to the roadmap's R3 exit — video over 10 km on Zynq and
   AD9361 — because his RTL already runs on that class of board.
4. **It is accepted block by block through our gates, and the listed defects are ours to
   fix**: a real gain control (ours, R0-07e) in place of the ideal one; an LDPC decoder
   without the floor (ours, R0-12, or his repaired — a measurement decides); deterministic
   vectors under a CI diff (his PR #1); saved synthesis and timing reports. Until a block
   has passed, a claim about it is his claim, cited as such.
5. **His fixed-point modem and its twins are the golden model of the n10 blocks for now.**
   ADR 0003 (the golden model is the spec) holds; what changes is that the spec of these
   blocks is written in his language. Blocks that both profiles share — FFT, LDPC decoder,
   CORDIC and NCO, gain control, DC removal — get one implementation, specified in
   `crates/radio-phy`. Whether the rest is ported is decided per block when it is touched.
6. **Nothing changes for the 802.11 work in progress** (R0: the 802.11a receiver). It is
   the Wi-Fi profile's first step and the source of the shared blocks.

## Why not the alternatives

- **Start again and design a better waveform.** The run shows what a better one would
  have: a guard of 3.2 µs or so, a channel measurement every five symbols or so, a short
  preamble. n10-p3 has these. A design from nothing would arrive at about the same grid a
  year later. What can be done better is the implementation, and decision 4 does that
  without discarding what works.
- **The 802.11ax numerology at half clock with a midamble (he10-ma).** Nearest to a
  standard and 17 % more rate than n10-p3, but it reaches 1 % at 64-QAM only just at
  2000 Hz and is 6 dB behind there; 802.11ax allows a midamble every 10 or 20 symbols only,
  so a denser one is no more standard than n10; nobody clocks 802.11ax at 10 MHz, so there
  is no commodity chip to test against either; and the 802.11ax core is stages away, while
  n10's RTL exists.
- **802.11a at 10 MHz, which is what we are building anyway.** Equal to n10 at QPSK and
  16-QAM with a line of sight. It stays the fallback: if n10 is delayed, a first long-range
  link can fly on it. It is not the profile because it fails without line of sight and
  at 64-QAM.

## Open, and not decided here

- **Patents (risk point).** n10 uses the 5G NR LDPC code, whose essential patents are
  3GPP's, not 802.11's. Options: keep it and add it to the freedom-to-operate analysis of
  ADR 0001; or carry n10's frames on 802.11n's LDPC (one code family in the chip, already
  modelled in R0-12). A code-only comparison at n10's block lengths decides whether the
  second costs anything. To be shown to the lawyer with the rest.
- **Regulation (risk point, CE/RED test plan).** The 5 GHz bands' harmonised standards ask
  for channel access behaviour (listen before talk, DFS, power control, EIRP limits per
  sub-band) that a TDMA link must be assessed against whatever its waveform. This ADR
  assumes nothing about it; the figures below are parametrised by EIRP for that reason.
- **Phase noise, frequency error and the power amplifier at 39 kHz tone spacing** on the
  AD9361: the colleague's impairment experiments rerun under our gates, then captures.
- **The 21 % of rate.** Whether n10 can use more of the 256 tones (242 as 802.11ax does)
  within the spectral mask, and whether p2 serves as the default.
- **One repository or two**, and where his code sits in the tree: a `[meta]` change of
  its own with the layout.

## What it means at 10 km

A budget, not a measurement. Free-space loss at 5.8 GHz over 10 km is 127.7 dB. With a
receiver noise figure of 6 dB the received C/N0 is EIRP + G_rx + 40.3 dB-Hz (EIRP in dBm,
the receive antenna's gain in dBi). The run's requirement for n10-p3 on the line-of-sight
channel at 600 Hz, plus 3 dB for what the run idealised (synchronisation, estimation,
fixed point, phase noise) and 6 dB of margin (pointing, polarisation, ground reflection):

| Modulation and code | PHY net rate | C/N0 needed (run) | with 3 + 6 dB | EIRP + G_rx needed | After the 22 : 1 superframe and 1 % loss |
|---|---|---|---|---|---|
| QPSK 1/2 | 5.07 Mbit/s | 74.3 dB-Hz | 83.3 | 43.0 dBm | about 4.8 Mbit/s |
| 16-QAM 1/2 | 10.13 Mbit/s | 79.7 dB-Hz | 88.7 | 48.4 dBm | about 9.6 Mbit/s |
| 64-QAM 3/4 | 22.80 Mbit/s | 89.3 dB-Hz | 98.3 | 58.1 dBm | about 21.6 Mbit/s |

Examples: 0.5 W into a 2 dBi antenna on the drone (EIRP 29 dBm) and a 17 dBi panel on the
ground give 46 dBm: QPSK 1/2 with 3 dB to spare. 1 W EIRP and a 23 dBi tracking dish give
53 dBm: 16-QAM 1/2 with 4.6 dB to spare. 64-QAM 3/4 needs 4 W EIRP and the dish, with
nothing to spare. 25 mW EIRP and a 14 dBi antenna give 28 dBm: no link at 10 km at any
rate (QPSK 1/2 to about 1.8 km). The 3 and 6 dB are assumptions of this ADR until R3
measures them; the MAC's headers and guard times, and packet loss of video packets that
span several frames, come off the last column.

## Consequences

- The chip carries two waveforms on one core, and the TDMA profile is no longer testable
  against commodity Wi-Fi cards (G5 for it is our two nodes and a signal analyser; the
  Wi-Fi profiles keep their G5).
- R3's exit can be attempted on the colleague's RTL before our 802.11 receiver has RTL.
- The colleague becomes a contributor to an Apache-2.0 project; his notices stay.
- `ROADMAP.md` and `BACKLOG.md` get the n10 items (gain control, decoder, vectors, reports,
  the code comparison) when this ADR is accepted.
