# Independent review of the decisions through R0-08b — 2026-11-19

Asked for by the founder: a fresh reviewer (code and documents only, no access to the authoring
sessions) looked for wrong or risky decisions in everything built so far. This file keeps its
findings, what was checked afterwards in the repo, and what is proposed. The decisions
themselves are in ADR 0007 (proposed).

Status words: **reproduced** = rerun here with a committed tool, numbers in
`docs/gates/reports/`; **confirmed by reading** = the code or document says so; **open** = not
checked yet.

## Findings

| # | Finding | Status | Where it goes |
|---|---|---|---|
| 1 | The final CFO is coarse + 0.771·(fine − coarse). The coarse window (STF samples 32…144) spans the AGC settling period and is not normalised, so a gain step with clipping or a phase jump inside it moves the output by kHz. The only AGC test in the STF steps at sample 24, before the window. | reproduced (`rxfront agc`) | ADR 0007, decision 1 |
| 2 | RTL is ahead of the algorithm: five receiver-front blocks are bit-exact while there is no AGC model, no channel estimation, no decoded packet, no capture from air. R0-08c will retune `det_a` after its RTL. G2 budgets are "agrees with float", not dB of PER. | confirmed by reading | ADR 0007, decision 3 |
| 3 | The acquisition is applied up to T1 + 279 samples: 3.5 µs after SIGNAL ends. ADR 0006 gives no path for the samples of LTF, SIGNAL and DATA to the rest of the receiver, and no latency budget to SIGNAL-decoded or to the ACK (SIFS, 16 µs). | confirmed by reading | ADR 0007, decision 2 |
| 4 | Area and timing follow "exact", not "sufficient": 128 constant multipliers used one clock in R, a second array planned; `det_a` stage 3 chains 32 × 32 squares into a 15-bit constant multiply and an 80-bit compare. | reproduced in part (Vivado OOC at 160 MHz: `det_a` −0.64 ns in that stage, `ltf_corr` −1.89 ns; the other four blocks pass) | journal; ADR 0007, decision 2 |
| 5 | "0 false alarms in 10⁹" holds for white noise. Behind a channel filter the noise is correlated at lag 16 and the detector fires. | reproduced (`rxfront tailbw`) | journal; R0-08c |
| 6 | The fixed-point budget was measured at −15 dBFS only. | reproduced (`rxfront level`) | journal; AGC target window |
| 7 | The strongest G1 reference (Annex I.1) never runs in CI: its fixture is not committed (copyright), so the four tests are `#[ignore]`. | confirmed by reading | backlog proposal: hash-only fixture |
| 8 | Smaller: the DC blocker's 6.4 µs time constant leaves ≈ 45 % of a DC change at a gain step in the LTF; the memory wrapper's same-cycle read/write on two ports must exist in the IHP macro set; 2.4 GHz Wi-Fi 4/6 implies DSSS/CCK reception. | open | backlog proposals |

## What the reruns show

**Finding 1** (`docs/gates/reports/rx_front_agc.txt`, 500 frames per case, 20 dB SNR):

| Case | Combined CFO RMS | Coarse | Fine alone |
|---|---|---|---|
| no gain change | 395 Hz | 818 Hz | 444 Hz |
| −20 dB at STF sample 48, clipping before | 3.2 kHz | 13.8 kHz | 476 Hz |
| same, phase jump within ±30° | 8.8 kHz | 37 kHz | 457 Hz |
| −2 dB trim at sample 100, ±30° | 2.1 kHz | 9.1 kHz | 481 Hz |
| −20 dB at 48, ±90° | 13 kHz, 324 of 500 acquired | 55 kHz | 770 Hz |

- Combining buys 11 % in clean AWGN and costs 5–20× whenever the AGC acts at STF sample 48 or
  later.
- A step at sample 32 or earlier is harmless (the window starts there), as the model intended.
- The fine estimate's lag-64 ambiguity was never resolved wrongly (0 slips in 15 000 frames).
- At ±90° the acquisition itself fails in a third of the frames: the confirmation rotates the
  LTF by the damaged coarse estimate.

**Finding 5** (`rx_front_tailbw.txt`, fixed receiver, 4·10⁸ samples per case):

| Noise passband | Detector events per sample | Acquisitions |
|---|---|---|
| white, ±9 MHz | 0 | 0 |
| ±8.3 MHz | 1.0·10⁻⁸ | 0 |
| ±7.5 MHz | 5.5·10⁻⁸ (the budget is 5·10⁻⁸) | 0 |
| ±6 MHz | 1.6·10⁻⁶ | 0 |

The LTF stage rejected every one. The false-acquisition claim stands; the detector-event
margin does not.

**Finding 4**: the timing table is in `docs/gates/rx_front.md` (2026-11-19, measurements). The
failure in `det_a` is one pipeline register away, not a redesign; the area argument (two
arrays of 128 multipliers, 124 DSPs each on the KV260) stands as the reviewer made it.

**Finding 6** (`rx_front_level.txt`): the fixed model's CFO RMS is within 6 % of float from
−3 to −30 dBFS at every SNR. Below that it grows: +62 % at −45 dBFS and 2.6× at −55 dBFS
(25 dB SNR). Acquisition and timing are unaffected down to −60 dBFS.

## What the reviewer found sound

Detector A over B and its threshold arithmetic; the CFO ranges and the ambiguity resolution at
20/10/5 MHz; the streaming restatement and its acceptance rule; the Viterbi decoder's widths and
method; the TX chain's references (Annex I.1, OpenOFDM, CommPy are independent); the mutation
discipline; reset strategy and portability of the RTL; ADR 0005. Nothing in the front is a
dead end for wider bandwidths, MIMO or HaLow.

## Not covered

The reviewer did not read the CORDIC and NCO RTL, did not audit `tools/ref` for licence hygiene
line by line, and did not run the RTL gates.
