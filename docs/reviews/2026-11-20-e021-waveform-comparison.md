# e021 — the colleague's waveform against 802.11's, through the same channels — 2026-11-20

The run that feeds ADR 0009 (the waveform of the long-range TDMA profile). It was made in
the colleague's simulator, `waverage/phy-sim` at ff1c483 (Apache-2.0 since 07b53c5; Sionna
2.2.0, torch 2.15.0, CPU), with one experiment script written here,
`e021/e021_wifi_waveforms.py`, which is not yet in his repository. The 24 result files are
in `e021/` as the simulator wrote them; every number below is computed from them
(`--summary` of the script prints the C/N0 table). Run on the build server, 2026-11-18 to
2026-11-20 09:32, profile `reference`.

## Question

At the same transmit power, noise density and channel, does the colleague's waveform
("n10": 10 MHz, 256-point FFT, CP 32 samples, 199 used tones, block pilot symbols, an own
28.8 µs preamble) reach more net rate per required C/N0 than 802.11's OFDM waveforms do?

Agreed with the founder before the results (2026-11-19): n10 is taken if it is within about
0.5 dB of the 802.11ax numerology at equal net rate on the fading channels, or if it wins.

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
bits (a20, 64-QAM 3/4), not a whole video packet.

## Results

### Net rate, Mbit/s

| Waveform | used tones / FFT | CP, µs | preamble, µs | data REs per frame | QPSK 1/2 | 16-QAM 1/2 | 64-QAM 3/4 |
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
| n10-p2 | D100-2000 | 73.8 | 80.2 | — (floor 59.2 %) |
| n10-p2 | C300-600 | 78.2 | 83.5 | 97.0 |
| n10-p3 | awgn | 70.3 | 75.5 | 84.7 |
| n10-p3 | D100-600 | 74.3 | 79.7 | 89.3 |
| n10-p3 | D100-2000 | 73.4 | 78.8 | 89.9 |
| n10-p3 | C300-600 | 78.1 | 83.4 | 94.9 |
| he10 | awgn | 71.2 | 76.2 | 85.4 |
| he10 | D100-600 | 75.4 | 81.1 | 93.1 |
| he10 | D100-2000 | 74.6 | 80.6 | — (floor 6.8 %) |
| he10 | C300-600 | 79.9 | 86.7 | — (floor 62.0 %) |
| he10-ma | awgn | 71.0 | 76.2 | 85.6 |
| he10-ma | D100-600 | 75.2 | 80.4 | 90.4 |
| he10-ma | D100-2000 | 74.5 | 80.0 | 96.7 |
| he10-ma | C300-600 | 78.8 | 84.1 | 96.3 |
| a10 | awgn | 70.5 | 75.7 | 85.1 |
| a10 | D100-600 | 74.8 | 80.5 | — (floor 6.4 %) |
| a10 | D100-2000 | 73.8 | 79.7 | — (floor 27.6 %) |
| a10 | C300-600 | 82.3 | — (floor 3.4 %) | — (floor 97.2 %) |
| a20 | awgn | 73.3 | 78.8 | 88.2 |
| a20 | D100-600 | 77.7 | 83.7 | — (floor 21.8 %) |
| a20 | D100-2000 | 76.8 | 83.4 | — (floor 73.6 %) |
| a20 | C300-600 | — (floor 1.3 %) | — (floor 27.0 %) | — (floor 100.0 %) |

### Eb/N0 for 1 % FER, dB (C/N0 less 10·log10 of the net rate)

| Waveform | Channel | QPSK 1/2 | 16-QAM 1/2 | 64-QAM 3/4 |
|---|---|---|---|---|
| n10-p2 | awgn | 2.9 | 5.1 | 10.8 |
| n10-p2 | D100-600 | 6.9 | 9.2 | 15.4 |
| n10-p2 | D100-2000 | 6.3 | 9.7 | — (floor 59.2 %) |
| n10-p2 | C300-600 | 10.8 | 13.0 | 23.0 |
| n10-p3 | awgn | 3.3 | 5.5 | 11.2 |
| n10-p3 | D100-600 | 7.3 | 9.6 | 15.8 |
| n10-p3 | D100-2000 | 6.3 | 8.7 | 16.3 |
| n10-p3 | C300-600 | 11.1 | 13.3 | 21.3 |
| he10 | awgn | 3.1 | 5.1 | 10.8 |
| he10 | D100-600 | 7.3 | 10.0 | 18.5 |
| he10 | D100-2000 | 6.5 | 9.5 | — (floor 6.8 %) |
| he10 | C300-600 | 11.8 | 15.6 | — (floor 62.0 %) |
| he10-ma | awgn | 3.3 | 5.4 | 11.3 |
| he10-ma | D100-600 | 7.5 | 9.7 | 16.2 |
| he10-ma | D100-2000 | 6.8 | 9.3 | 22.5 |
| he10-ma | C300-600 | 11.1 | 13.4 | 22.0 |
| a10 | awgn | 3.2 | 5.3 | 11.2 |
| a10 | D100-600 | 7.4 | 10.1 | — (floor 6.4 %) |
| a10 | D100-2000 | 6.4 | 9.3 | — (floor 27.6 %) |
| a10 | C300-600 | 14.9 | — (floor 3.4 %) | — (floor 97.2 %) |
| a20 | awgn | 2.8 | 5.3 | 11.0 |
| a20 | D100-600 | 7.2 | 10.1 | — (floor 21.8 %) |
| a20 | D100-2000 | 6.2 | 9.8 | — (floor 73.6 %) |
| a20 | C300-600 | — (floor 1.3 %) | — (floor 27.0 %) | — (floor 100.0 %) |

### The tails of the 64-QAM 3/4 curves in fading (frame errors / frames at the five highest C/N0)

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

The remaining tails are in the result files; "floor" in the tables is the frame error rate
at the highest C/N0 simulated, where the curve had stopped falling.

## How good the numbers are

- Each 1 % crossing in fading is bracketed by two points of 5000 frames, typically 50 to
  100 errors above and 20 to 50 below, half a dB apart: the crossing is good to a few
  tenths of a dB. In AWGN the bracket is a point of 250 to 750 frames and one of 5000.
- One crossing is not a crossing: **he10-ma at 64-QAM 3/4 on D100-2000 sits on a floor of
  about 1.1 %** (59, 49, 55, 60, 60 errors of 5000 over the last 2 dB). Its "96.7 dB-Hz" is
  where a flat curve happens to pass under 1 %.
- The entries marked "floor" are error floors, not curves cut short: the error rate no
  longer falls with C/N0.

## Findings

1. **Where a 10 km link will work — QPSK and 16-QAM with a line of sight (TDL-D) — the
   waveform hardly matters.** All of n10-p2, n10-p3, he10, he10-ma and a10 need an Eb/N0
   within about 1.4 dB of each other (QPSK 6.3 to 7.5 dB, 16-QAM 8.7 to 10.1 dB), at 600 and
   at 2000 Hz. In AWGN all six are within 0.5 dB.
2. **64-QAM 3/4 in fading is decided by how often the channel is measured within a frame.**
   Waveforms that measure it once, at the start (a10, a20, he10), end on an error floor on
   some or all fading channels. he10-ma (two measurements) reaches 1 % everywhere but only
   just at 2000 Hz. n10-p2 (two) fails at 2000 Hz. n10-p3 (three) is the only waveform with
   no floor anywhere: 89.3, 89.9 and 94.9 dB-Hz.
3. **A long delay spread is decided by the guard interval.** On TDL-C with 300 ns the
   802.11a numerologies fall behind or fail (a10: 4 dB worse at QPSK, a floor at 16-QAM;
   a20: floors at every rate), with 1.6 and 0.8 µs of guard against 3.2 µs for the 256-point
   waveforms.
4. **n10 against the 802.11ax numerology, the agreed criterion:** at equal net rate (the
   Eb/N0 table) n10-p3 is not worse than he10 or he10-ma in any fading cell (equal within
   0.1 dB where it does not win), and is better by 2.3 dB at 16-QAM on C300-600 against he10
   and by 6 dB at 64-QAM on D100-2000 against he10-ma. n10-p2 is within 0.4 dB of both up to
   16-QAM, 1 dB behind he10-ma at 64-QAM on C300-600, and fails at 64-QAM on D100-2000.
   **The criterion is met by n10-p3, and by n10-p2 up to 16-QAM.**
5. **What n10-p3 pays is rate:** at one modulation and code it carries 5.07 / 10.13 /
   22.80 Mbit/s, against 5.53 / 11.06 / 24.88 for n10-p2, 5.91 / 11.82 / 26.59 for he10-ma
   and 6.40 / 12.80 / 28.80 for he10 — 8 % less than n10-p2 and 21 % less than he10 in the
   same 10 MHz. Two causes: 199 used tones against 242, and three pilot symbols of 14.
   On D100-600, the nearest of these channels to a drone with a line of sight, n10-p2 and
   n10-p3 need the same C/N0 at every rate.
6. n10 is close to the 802.11ax numerology at half clock (256 points, CP 32); what differs
   is the tone count, the pilots (block symbols against 8 continual tones plus a midamble)
   and the preamble (28.8 µs against 72 µs).

## What this run does not show

- **Synchronisation, gain control, the oscillator and the power amplifier are all ideal.**
  A 256-point waveform at 10 MHz has 39 kHz between tones against 156 kHz for a10: four
  times less room for phase noise and residual frequency error. The run cannot say what
  that costs n10 and he10 against a10. (The colleague's own experiments e002–e017 look at
  impairments of his waveform; they were not rerun here.)
- The estimator has genie statistics: a ceiling for every waveform, not a receiver.
- One code for all. 802.11's own codes (the convolutional code of a10 and a20, 802.11n's
  LDPC) are a separate, code-only comparison that has not been made.
- Three modulation and code pairs of the table; the steps between them (QPSK 3/4,
  16-QAM 3/4, 64-QAM 2/3) were not simulated.
- Frame error rate of 400 µs frames, not packet loss of video packets; no retransmission.
- The review of the colleague's repository (2026-11-18, reproduced on the build server)
  found defects that do not touch this comparison but touch the adoption: an ideal gain
  control not disclosed, an error floor near 1e-4 in his 6-bit LDPC decoder, vectors not
  deterministic, board timing claims without saved reports.
