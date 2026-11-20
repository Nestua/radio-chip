# Roadmap

Goal: an open European 802.11-compatible radio chip. Stages R0–R4 give Nestua its own serial radio
link in about two years; R5–R6 lead to the open chip. Each stage ends at a gate that can be shown to
an investor or a grant reviewer. Durations are estimates; stages overlap.

**Current stage: R0 — golden model.** The transmit chain is complete in the golden model (R0-01…R0-06, R0-11): an independent receiver (OpenOFDM) decodes our frames at all eight rates. The exit gate's transmit half is met: the model reproduces the standard's worked example (802.11-2024 Annex I.1): every bit table exactly, the 881 time samples within the annex's 3-decimal print rounding. The receive half is met on a clean channel in float: R0-09 with the R0-10 decoder decodes the annex's 881 printed samples back to its PSDU from the known frame start (`docs/gates/rx_ofdm.md`), and with the R0-08a front in the chain it decodes simulated frames through AWGN and TGn channels. Its fixed-point form (R0-09 G2) and the work ordered by ADR 0007 (the AGC model, the fine CFO estimate as the output, ADR 0008 before the rest of the receiver-front RTL) remain.

| Stage | What | Exit gate | Estimate |
|---|---|---|---|
| **R0** Golden model | Rust PHY/MAC model in fixed point, channel simulator, 802.11a-level transmit and receive chain | The model encodes the standard's worked example frame bit-exactly and decodes it back through a simulated channel | 0–3 months |
| **R1** Over the air on SDR | The same model running in real time on a PC with an SDR; first field measurements | A link over the air; range measured and logged | 3–6 months |
| **R2** RTL blocks | FFT, synchronisation, equaliser, convolutional/LDPC codecs, TDMA scheduler in RTL; ASIC probes in parallel | Every block passes G3 (and G4 where required) | 6–12 months |
| **R3** FPGA link | Zynq + AD9361, two nodes, Linux netdev; WebRTC and MAVLink unchanged on top; reference oscillator TCXO ≤ ±2 ppm (R0-08 G0) | Video from the drone over 10 km | 9–15 months |
| **R4** Serial module | Own board for the drone, CE/RED | CE/RED passed, module in series production | 12–24 months |
| **R5** Baseband ASIC | Digital chip: baseband + MAC + RISC-V, external RF transceiver; MPW on the IHP open PDK, then GF 22FDX | First silicon works with the external RF | 2–4 years |
| **R5b** One package | Our baseband die and a bought transceiver die in one package: one part for the customer, no analogue design of ours | The packaged part passes the R4 module's tests | after R5 |
| **R6** Full SoC | RF on the die through a licensed RF block or a partner, not designed from nothing: Wi-Fi 6/6E, HaLow, TDMA; Bluetooth LE | Serial open chip | 4+ years |

## Profiles of the one core

- **Long-range TDMA** — 802.11 OFDM PHY plus our TDMA MAC (the approach MikroTik NV2 and
  Ubiquiti airMAX proved at 10+ km). First user: Nestua. Since ADR 0009: the 802.11ax format
  with its midamble at 20 MHz and at half clock; video down in 5.2 and 2.4 GHz together,
  command up in 2.4 GHz; built-in antennas, four on the drone and four per band on the dock;
  the drone opens each frame period (frame-based access). The colleague's n10 radio is the
  lab and field prototype on the way; a flight on it does not close R3.
- **Wi-Fi 4 → 6/6E → 7** — Wi-Fi 4 is the stepping stone; the target is Wi-Fi 6/6E (OFDMA,
  1024-QAM, 6 GHz); Wi-Fi 7 after that.
- **HaLow (802.11ah)** — the same OFDM, roughly 10× slower clocking, 863–868 MHz in the EU.

## Parameters of the one design (the founder, 2026-11-20; ADR 0009)

So that the long-range configuration is one setting of a general Wi-Fi design and not a
chip of its own:

| Question | Now | Later |
|---|---|---|
| Receive chains and bandwidth | a pool sized by chains × bandwidth, 160 MHz of samples: eight chains of 20 MHz, two of 80 MHz or one of 160 MHz, assigned at run time; more antennas than chains, with selection; one to four transmit chains | the pool's size is a parameter: 320 MHz is one more doubling, when a radio and a market ask for it |
| Links | multi-link operation, built first for the TDMA profile's two bands | Wi-Fi 7's multi-link on the same machinery |
| Decoders | the number of decoder cores is its own parameter (tens of megabits for the long-range link, over a gigabit for a two-stream 80 MHz client) | — |
| Radio | a defined digital interface to the transceivers (serial: eight chains are too many pins), calibration from the digital side | R5b, then R6 |
| Bluetooth | an arbiter in the MAC that shares the 2.4 GHz chain in time | digital Bluetooth LE after the Wi-Fi core; qualification and its patent licence with the lawyer |
| Process | 130 nm (IHP) for test chips of blocks only | 40 nm or below for the eight-chain design; 22FDX, already the plan, fits it (first count in ADR 0009; ADR 0008 does the real one) |

## Budget envelope (lean path, rough orders)

R0–R1 €20–50k, R2 €10–30k, R3 €40–90k, R4 €100–250k, R5 test chips €20–200k. Serial masks and the
full RF SoC need external funding (€1M+); we go for it after R3 and a working R5 test chip.
Strategy, patents and funding are summarised in ADRs 0001, 0002 and 0005.
