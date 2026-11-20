# e021 — the colleague's waveform against 802.11's, through the same channels — 2026-11-20

The run that feeds ADR 0009 (the waveform of the long-range TDMA profile). It was made in
the colleague's simulator, `waverage/phy-sim` (Apache-2.0 since 07b53c5; Sionna 2.2.0, torch
2.15.0, CPU), with one experiment script written here, `e021/e021_wifi_waveforms.py`,
which is not yet in his repository. The 24 result files are in `e021/` as the simulator
wrote them; every number below is computed from them (`--summary` of the script prints the
C/N0 table). Run on the build server, 2026-11-18 to 2026-11-20 09:32, profile `reference`.

Provenance, as far as it goes: the checkout on the server was at ff1c483, but the result
files record `"git": "none"` (the script was untracked), so the commit is this report's
word and not the files'. The simulator's harness is not in this repository; the run cannot
be repeated from here. The n10 curves in fading were produced by an earlier version of the
script that ran each curve on to a point with no errors; the others stop at the first point
under 0.3 % (the committed version). The 1 % crossings do not depend on which. An
independent reviewer recomputed every table cell from the files (2026-11-20).

## Question

At the same transmit power, noise density and channel, does the colleague's waveform
("n10": 10 MHz, 256-point FFT, CP 32 samples, 199 used tones, block pilot symbols, an own
28.8 µs preamble) reach more net rate per required C/N0 than 802.11's OFDM waveforms do?

Agreed with the founder on 2026-11-19, when the AWGN files and two of n10's fading files
existed and none of the 802.11ax numerology's in fading: n10 is taken if it is within about
0.5 dB of the 802.11ax numerology at equal energy per bit on the fading channels, or if it
wins.

## What was compared, and what was held equal

| Name | What it is |
|---|---|
| n10-p2 | the colleague's waveform as he has it: pilot symbols 2 and 11 of the frame's 14 |
| n10-p3 | the same with three pilot symbols: 1, 6, 11 |
| he10 | 802.11ax HE SU 20 MHz numerology clocked at 10 MHz: 242-tone RU, 8 continual pilots, one HE-LTF at the start, 72 µs preamble |
| he10-ma | the same with a midamble after 10 data symbols (802.11ax's Doppler mode) |
| a10 | 802.11a clocked at 10 MHz (802.11p's numerology): 52 tones, 4 continual pilots, two long training symbols at the start, 50 symbols |
| a20 | 802.11a on 20 MHz, 100 symbols |

Held equal, so that the difference is the waveform's (numerology, pilots, preamble
overhead): the same 5G NR LDPC code for all; the same LMMSE channel estimator with genie
statistics; **genie timing and frequency, no oscillator impairments, no gain control, no
power-amplifier model**; frames of about 400 µs. Channels: AWGN; 3GPP TDL-D (a line-of-sight
profile) with 100 ns delay spread at 600 Hz and at 2000 Hz of Doppler; TDL-C (no line of
sight) with 300 ns at 600 Hz; carrier 5.8 GHz, where 600 Hz is 31 m/s and 2000 Hz is
103 m/s.

Metric: the C/N0 needed for a frame error rate of 1 %, against the net rate (information
bits per frame over the frame's air time plus its preamble). Same power and noise density,
so a dB of C/N0 is a dB of link budget. A frame carries 2189 (n10-p3, QPSK 1/2) to 21 168
bits (a20, 64-QAM 3/4), not a whole video packet. The sweep covers 16 dB in steps of
0.5 dB from a start below each modulation's reference point.

## Results

### Net rate, Mbit/s

| Waveform | used tones / FFT | CP, µs | preamble charged, µs | data REs per frame | QPSK 1/2 | 16-QAM 1/2 | 64-QAM 3/4 |
|---|---|---|---|---|---|---|---|
| n10-p2 | 199 / 256 | 3.2 | 28.8 | 2388 | 5.53 | 11.06 | 24.88 |
| n10-p3 | 199 / 256 | 3.2 | 28.8 | 2189 | 5.07 | 10.13 | 22.80 |
| he10 | 242 / 256 | 3.2 | 72.0 | 3042 | 6.40 | 12.80 | 28.80 |
| he10-ma | 242 / 256 | 3.2 | 72.0 | 2808 | 5.91 | 11.82 | 26.59 |
| a10 | 52 / 64 | 1.6 | 24.0 | 2304 | 5.43 | 10.87 | 24.45 |
| a20 | 52 / 64 | 0.8 | 12.0 | 4704 | 11.42 | 22.83 | 51.38 |

### C/N0 for 1 % FER, dB-Hz

| Waveform | Channel | QPSK 1/2 | 16-QAM 1/2 | 64-QAM 3/4 |
|---|---|---|---|---|
| n10-p2 | awgn | 70.3 | 75.5 | 84.7 |
| n10-p2 | D100-600 | 74.4 | 79.6 | 89.3 |
| n10-p2 | D100-2000 | 73.8 | 80.2 | — (59.2 %, flat) |
| n10-p2 | C300-600 | 78.2 | 83.5 | 97.0 |
| n10-p3 | awgn | 70.3 | 75.5 | 84.7 |
| n10-p3 | D100-600 | 74.3 | 79.7 | 89.3 |
| n10-p3 | D100-2000 | 73.4 | 78.8 | 89.9 |
| n10-p3 | C300-600 | 78.1 | 83.4 | 94.9 |
| he10 | awgn | 71.2 | 76.2 | 85.4 |
| he10 | D100-600 | 75.4 | 81.1 | 93.1 |
| he10 | D100-2000 | 74.6 | 80.6 | — (6.8 %, flat) |
| he10 | C300-600 | 79.9 | 86.7 | — (62.0 %, flat) |
| he10-ma | awgn | 71.0 | 76.2 | 85.6 |
| he10-ma | D100-600 | 75.2 | 80.4 | 90.4 |
| he10-ma | D100-2000 | 74.5 | 80.0 | — (1.2 %, flat) |
| he10-ma | C300-600 | 78.8 | 84.1 | 96.3 |
| a10 | awgn | 70.5 | 75.7 | 85.1 |
| a10 | D100-600 | 74.8 | 80.5 | — (6.4 %, flat) |
| a10 | D100-2000 | 73.8 | 79.7 | — (27.6 %, flat) |
| a10 | C300-600 | 82.3 | — (3.4 %, still falling) | — (97.2 %, flat) |
| a20 | awgn | 73.3 | 78.8 | 88.2 |
| a20 | D100-600 | 77.7 | 83.7 | — (21.8 %, flat) |
| a20 | D100-2000 | 76.8 | 83.4 | — (73.6 %, flat) |
| a20 | C300-600 | — (1.3 %, still falling) | — (27.0 %, flat) | — (100.0 %, flat) |

### Eb/N0 for 1 % FER, dB (C/N0 less 10·log10 of the net rate)

| Waveform | Channel | QPSK 1/2 | 16-QAM 1/2 | 64-QAM 3/4 |
|---|---|---|---|---|
| n10-p2 | awgn | 2.9 | 5.1 | 10.8 |
| n10-p2 | D100-600 | 6.9 | 9.2 | 15.4 |
| n10-p2 | D100-2000 | 6.3 | 9.7 | — (59.2 %, flat) |
| n10-p2 | C300-600 | 10.8 | 13.0 | 23.0 |
| n10-p3 | awgn | 3.3 | 5.5 | 11.2 |
| n10-p3 | D100-600 | 7.3 | 9.6 | 15.8 |
| n10-p3 | D100-2000 | 6.3 | 8.7 | 16.3 |
| n10-p3 | C300-600 | 11.1 | 13.3 | 21.3 |
| he10 | awgn | 3.1 | 5.1 | 10.8 |
| he10 | D100-600 | 7.3 | 10.0 | 18.5 |
| he10 | D100-2000 | 6.5 | 9.5 | — (6.8 %, flat) |
| he10 | C300-600 | 11.8 | 15.6 | — (62.0 %, flat) |
| he10-ma | awgn | 3.3 | 5.4 | 11.3 |
| he10-ma | D100-600 | 7.5 | 9.7 | 16.2 |
| he10-ma | D100-2000 | 6.8 | 9.3 | — (1.2 %, flat) |
| he10-ma | C300-600 | 11.1 | 13.4 | 22.0 |
| a10 | awgn | 3.2 | 5.3 | 11.2 |
| a10 | D100-600 | 7.4 | 10.1 | — (6.4 %, flat) |
| a10 | D100-2000 | 6.4 | 9.3 | — (27.6 %, flat) |
| a10 | C300-600 | 14.9 | — (3.4 %, still falling) | — (97.2 %, flat) |
| a20 | awgn | 2.8 | 5.3 | 11.0 |
| a20 | D100-600 | 7.2 | 10.1 | — (21.8 %, flat) |
| a20 | D100-2000 | 6.2 | 9.8 | — (73.6 %, flat) |
| a20 | C300-600 | — (1.3 %, still falling) | — (27.0 %, flat) | — (100.0 %, flat) |

### The tails of the 64-QAM 3/4 curves in fading (frame errors / frames at the five highest C/N0 simulated)

| Waveform | Channel | C/N0 range, dB-Hz | errors / frames |
|---|---|---|---|
| n10-p2 | D100-600 | 90.5 … 92.5 | 13/5000, 9/5000, 3/5000, 4/5000, 0/5000 |
| n10-p2 | D100-2000 | 96.5 … 98.5 | 157/250, 158/250, 140/250, 147/250, 148/250 |
| n10-p2 | C300-600 | 96.5 … 98.5 | 77/5000, 49/5000, 41/5000, 36/5000, 35/5000 |
| n10-p3 | D100-600 | 90.0 … 92.0 | 21/5000, 14/5000, 4/5000, 3/5000, 0/5000 |
| n10-p3 | D100-2000 | 91.5 … 93.5 | 6/5000, 2/5000, 1/5000, 1/5000, 0/5000 |
| n10-p3 | C300-600 | 96.0 … 98.0 | 15/5000, 2/5000, 1/5000, 1/5000, 0/5000 |
| he10 | D100-600 | 93.8 … 95.8 | 45/5000, 33/5000, 25/5000, 26/5000, 12/5000 |
| he10 | D100-2000 | 97.3 … 99.3 | 100/1750, 106/1500, 107/1750, 113/1750, 119/1750 |
| he10 | C300-600 | 97.3 … 99.3 | 154/250, 149/250, 144/250, 151/250, 155/250 |
| he10-ma | D100-600 | 89.3 … 91.3 | 102/3750, 81/5000, 60/5000, 25/5000, 12/5000 |
| he10-ma | D100-2000 | 97.3 … 99.3 | 59/5000, 49/5000, 55/5000, 60/5000, 60/5000 |
| he10-ma | C300-600 | 96.3 … 98.3 | 45/5000, 34/5000, 24/5000, 22/5000, 14/5000 |
| a10 | D100-600 | 96.7 … 98.7 | 111/1750, 102/2000, 111/1750, 105/1750, 112/1750 |
| a10 | D100-2000 | 96.7 … 98.7 | 170/500, 169/500, 161/500, 160/500, 138/500 |
| a10 | C300-600 | 96.7 … 98.7 | 245/250, 241/250, 242/250, 242/250, 243/250 |
| a20 | D100-600 | 99.7 … 101.7 | 128/500, 119/500, 111/500, 124/500, 109/500 |
| a20 | D100-2000 | 99.7 … 101.7 | 183/250, 171/250, 195/250, 184/250, 184/250 |
| a20 | C300-600 | 99.7 … 101.7 | 250/250, 250/250, 250/250, 250/250, 250/250 |

An entry "— (x %, flat)" is a curve that ended above 1 % with the error rate no longer
falling (less than a fifth over the last 2 dB): an error floor of this receiver. "Still
falling" is a curve the 16 dB grid cut short; it might cross 1 % a dB or two further on.

## How good the numbers are

- Each 1 % crossing in fading is bracketed by two points of 4250 to 5000 frames, typically
  50 to 100 errors above and 20 to 50 below, half a dB apart: good to a few tenths of a dB.
  In AWGN the bracket is a point of 250 to 750 frames and one of 5000, two to three decades
  apart in error rate.
- Shallow crossings, to be read as "about": n10-p2 at 64-QAM on C300-600 (77, 49, 41, 36,
  35 errors of 5000 over the last 2 dB: flattening near 0.7 %) and a10 at QPSK on C300-600.
- he10-ma at 64-QAM on D100-2000 dips under 1 % once (48 of 5000) and ends at 1.2 %: a
  floor near 1.1 %, shown as such.
- The 802.11 waveforms were not observed below 0.3 % (the stop rule); n10 was.

## How fair the comparison is

- **The receiver is pilot-only for every waveform**: no decision-directed tracking, no
  phase or frequency loop between channel measurements. Receivers for single-preamble
  waveforms normally have these. This is what puts a10, a20 and he10 on floors at 64-QAM,
  and it is unfavourable to them.
- **What the simulator does with the line-of-sight path's Doppler shift is not known
  here.** In TDL-D that path carries a shift of its own (a common rotation any frequency
  tracker removes). The data hint it is left in: he10-ma, with 8 continual pilot tones
  between measurements 11 symbols apart, floors at 1.2 % at 2000 Hz, and n10-p2, with none
  between measurements 9 symbols apart, at 59 %. If so, a simple phase tracker might rescue
  n10-p2 and he10 there. To be asked of the author.
- **A Jakes spread of 600 or 2000 Hz is harsher than an air-to-ground link with a line of
  sight**, which has a large shift and a small spread. 2000 Hz is 103 m/s, not a
  multirotor.
- **The preamble is accounting, not a test.** With genie timing and frequency nothing shows
  that 28.8 µs is enough to synchronise on. The 802.11ax numerology is charged 72 µs, which
  includes 32 µs of rate and length signalling and the legacy fields; n10 is charged pure
  synchronisation with no header. Taking the signalling out of he10 is worth 0.3 dB to it.
- **The frame is n10's**: 14 symbols, about 400 µs, for all. The floors of the
  single-measurement waveforms grow with frame length; shorter frames would help them.
- Frame sizes differ, so a20 and he10 get a small coding-length gain; it does not favour
  n10.

## Findings

1. **Where a 10 km link will work — QPSK and 16-QAM with a line of sight (TDL-D) — the
   waveform hardly matters.** Among n10-p2, n10-p3, he10, he10-ma and a10 the Eb/N0 needed
   differs by 0.5 dB at QPSK and by 0.9 to 1.05 dB at 16-QAM within a channel, at 600 and at
   2000 Hz. In AWGN it is about 0.4 dB.
2. **With this pilot-only receiver and 400 µs frames, 64-QAM 3/4 in fading goes to the
   waveform that measures the channel most often.** n10-p3 (three measurements per frame)
   is the only one that reaches 1 % on all three fading channels: 89.3, 89.9 and 94.9 dB-Hz.
   n10-p2 and he10-ma (two) reach it at 600 Hz and not at 2000 Hz; he10, a10 and a20 (one)
   end on floors on some or all channels.
3. **A long delay spread is decided by the guard interval.** On TDL-C with 300 ns (maximum
   excess delay about 2.6 µs) the 802.11a numerologies, with 1.6 and 0.8 µs of guard against
   3.2 µs, lose: a10 needs 4 dB more at QPSK and does not reach 1 % at 16-QAM within the
   grid; a20 does not reach it at any rate (QPSK still falling at 1.3 %).
4. **n10 against the 802.11ax numerology, the agreed criterion.** At equal energy per bit
   (the Eb/N0 table) n10-p3 and he10-ma are indistinguishable up to 16-QAM: n10-p3 is ahead
   by 0.0 to 0.6 dB as the preambles are charged here and behind by up to 0.3 dB in four
   cells if he10's signalling is not charged, with crossings good to a few tenths. n10-p3
   is ahead of he10 without a midamble by 2.3 dB at 16-QAM on C300-600. n10-p2 is within
   0.5 dB of both up to 16-QAM (worst 0.46 dB). **The criterion — within about 0.5 dB — is
   met by n10-p3 under either accounting; the run does not show n10 better anywhere a
   10 km link operates.** Its clear wins are at 64-QAM.
5. **What n10-p3 pays is rate in the channel:** at one modulation and code it carries
   5.07 / 10.13 / 22.80 Mbit/s, against 5.53 / 11.06 / 24.88 for n10-p2, 5.91 / 11.82 /
   26.59 for he10-ma and 6.40 / 12.80 / 28.80 for he10 — 8 % less than n10-p2, 14 % less
   than he10-ma and 21 % less than he10 in the same 10 MHz. Two causes: 199 used tones
   against 242, and three pilot symbols of 14. Equal energy per bit is the fair measure
   for a power-limited link; in a fixed channel the others carry more. On D100-600 n10-p2
   and n10-p3 need the same C/N0 at every rate.
6. n10 is close to the 802.11ax numerology at half clock (256 points, CP 32); what differs
   is the tone count, the pilots (block symbols against 8 continual tones plus a midamble)
   and the preamble.

## What this run does not show

- **Synchronisation, gain control, the oscillator and the power amplifier are all ideal.**
  A 256-point waveform at 10 MHz has 39 kHz between tones against 156 kHz for a10. A
  reference of ±2 ppm at each end is up to about 23 kHz at 5.8 GHz: 0.6 of n10's tone
  spacing against 0.15 of a10's, so the acquisition range of the short preamble matters.
  (The colleague's own experiments e002–e017 look at impairments of his waveform; they
  were not rerun here.)
- The estimator has genie statistics: a ceiling for every waveform, which does not mean
  every waveform is equally far below it in a real receiver.
- One code for all. 802.11's own codes (the convolutional code of a10 and a20, 802.11n's
  LDPC) are a separate, code-only comparison that has not been made.
- Waveforms not in the run: 802.11bd (a 10 MHz waveform with LDPC and midambles, made for
  the Doppler problem), 802.11a at 5 MHz (its guard is 3.2 µs, n10's), a10 with shorter
  frames or a tracking receiver. Nothing inside n10 was varied (tone count, guard, pilot
  positions other than p2 and p3).
- Three modulation and code pairs; the steps between them and anything below QPSK 1/2
  (which the uplink of a 10 km link needs, see ADR 0009) were not simulated.
- Frame error rate of 400 µs frames, not packet loss of video packets; no retransmission.
- The two-ray ground reflection of a long, low path: a flat fade over the whole 10 MHz,
  which TDL-D does not contain.
