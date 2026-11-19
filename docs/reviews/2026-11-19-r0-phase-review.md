# Independent review of the decisions through R0-08b — 2026-11-19

Asked for by the founder: a fresh reviewer (code and documents only, no access to the authoring
sessions) looked for wrong or risky decisions in everything built so far. This file keeps its
findings, what was checked afterwards in the repo, and what is proposed. The decisions
themselves are in ADR 0007 (proposed, in its own pull request).

Status words: **reproduced** = rerun here with a committed tool, numbers in
`docs/gates/reports/`; **confirmed by reading** = the code or document says so; **open** = not
checked yet.

## Findings

| # | Finding | Status | Where it goes |
|---|---|---|---|
| 1 | The final CFO is coarse + 0.771·(fine − coarse). The coarse window (STF samples 32…144) spans the AGC settling period and is not normalised, so clipping or a phase jump inside it moves the output by kHz. The only AGC test in the STF steps at sample 24, before the window. | reproduced (`rxfront agc`) | ADR 0007, decision 1 |
| 2 | RTL is ahead of the algorithm: five receiver-front blocks are bit-exact while there is no AGC model, no channel estimation, no decoded packet, no capture from air. R0-08c will retune `det_a` after its RTL. G2 budgets are "agrees with float", not dB of PER. | confirmed by reading | ADR 0007, decision 3 |
| 3 | The acquisition is applied up to T1 + 279 samples: 3.5 µs after SIGNAL ends. ADR 0006 gives no path for the samples of LTF, SIGNAL and DATA to the rest of the receiver, and no latency budget to SIGNAL-decoded or to the ACK (SIFS, 16 µs). | confirmed by reading | ADR 0007, decision 2 |
| 4 | Area and timing follow "exact", not "sufficient": 128 constant multipliers used one clock in R, a second array planned; `det_a` stage 3 chains 32 × 32 squares into a 15-bit constant multiply and an 80-bit compare. | reproduced in part (Vivado OOC at 160 MHz: `det_a` −0.64 ns in that stage, `ltf_corr` −1.89 ns; the other four blocks pass) | journal; ADR 0007, decision 2 |
| 5 | "0 false alarms in 10⁹" holds for white noise. Behind a channel filter the noise is correlated at lag 16 and the detector fires. | reproduced (`rxfront tailbw`) | journal; R0-08c |
| 6 | The fixed-point budget was measured at −15 dBFS only. | reproduced (`rxfront level`) | journal; AGC target window |
| 7 | The strongest G1 reference (Annex I.1) never runs in CI: its fixture is not committed (copyright), so the four tests are `#[ignore]`. | confirmed by reading | backlog proposal: hash-only fixture |
| 8 | Smaller: the DC blocker's 6.4 µs time constant leaves ≈ 45 % of a DC change at a gain step in the LTF; the memory wrapper's same-cycle read/write on two ports must exist in the IHP macro set; 2.4 GHz Wi-Fi 4/6 implies DSSS/CCK reception. | open | backlog proposals |

## What the reruns show

The reruns were themselves reviewed by a second fresh reviewer before this file was merged; its
corrections are in the text below (the first draft overstated findings 1 and 5).

**Finding 1** (`docs/gates/reports/rx_front_agc_combined.txt`, 500 frames per case, 20 dB SNR). One event
at STF sample 48:

| Case | Combined CFO RMS | Coarse | Fine alone |
|---|---|---|---|
| no gain change | 422 Hz | 866 Hz | 477 Hz |
| −20 dB step, no clipping, no phase jump | 923 Hz | 3.5 kHz | 520 Hz |
| −20 dB step, start clipped, no phase jump | 3.2 kHz | 13.4 kHz | 448 Hz |
| no gain change, phase jump ±30° | 2.3 kHz | 9.8 kHz | 454 Hz |
| −20 dB step, clipped, ±30° | 8.1 kHz | 34.6 kHz | 478 Hz |

- The finding is reproduced, with two qualifications the reviewer's own table did not show.
- **The cause is the clipped stretch and the phase jump, not the gain step.** The phase jumps
  (±10° to ±90° per gain change) are assumed. Nothing measured on a real AGC is behind them, so
  the size of the effect is conditional until R1's captures exist.
- **The fine estimate is not robust in general.** It is untouched by events before the long
  training field and damaged by events inside it: a −2 dB trim with ±30° at sample 176, 208 or
  272 gives 2.4, 8.3 and 7.0 kHz. Fine-only output moves the AGC's deadline from STF sample 32
  to the end of the STF.
- Combining buys 11 % in clean AWGN (422 against 477 Hz).
- A gain step in the last STF period or later breaks the timing by a whole training symbol
  (214 of 500 frames at sample 144, 419 at 176). This is a separate weakness.

**Finding 5** (`rx_front_tailbw.txt`, fixed receiver): see the journal for the counts. Behind a
filter that keeps the occupied band (±8.3 MHz and wider) the detector-event rate stays under
the budget, by a factor of a few rather than the fifty of white noise. Filters that exceed the
budget (±7.5 MHz and narrower) cut into the signal's own subcarriers and are not usable channel
filters; they show the trend. No false acquisition in any case. The claim "0 false alarms in
10⁹" stays a white-noise statement; adjacent-channel signals remain untested.

**Finding 4**: the timing table is in `docs/gates/rx_front.md` (2026-11-19, measurements). The
failure in `det_a` is one pipeline register away, not a redesign; the area argument (two
arrays of 128 multipliers, 124 DSPs each on the KV260) stands as the reviewer made it.

**Finding 6** (`rx_front_level.txt`): the fixed model's CFO RMS equals float within 0.5 % from
−3 to −15 dBFS, is +6 % at −30 dBFS, +62 % at −45 dBFS and 2.6× at −55 dBFS (25 dB SNR, the
worst column). Acquisition and timing are unaffected down to −60 dBFS.

## What the reviewer found sound

Detector A over B and its threshold arithmetic; the CFO ranges and the ambiguity resolution at
20/10/5 MHz; the streaming restatement and its acceptance rule; the Viterbi decoder's widths and
method; the TX chain's references (Annex I.1, OpenOFDM, CommPy are independent); the mutation
discipline; reset strategy and portability of the RTL; ADR 0005. Nothing in the front is a
dead end for wider bandwidths, MIMO or HaLow.

## Not covered

The reviewer did not read the CORDIC and NCO RTL, did not audit `tools/ref` for licence hygiene
line by line, and did not run the RTL gates.
