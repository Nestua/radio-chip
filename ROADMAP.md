# Roadmap

Goal: an open European 802.11-compatible radio chip. Stages R0–R4 give Nestua its own serial radio
link in about two years; R5–R6 lead to the open chip. Each stage ends at a gate that can be shown to
an investor or a grant reviewer. Durations are estimates; stages overlap.

**Current stage: R0 — golden model.** The transmit chain is complete in the golden model (R0-01…R0-06, R0-11): an independent receiver (OpenOFDM) decodes our frames at all eight rates. The exit gate's transmit half is met: the model reproduces the standard's worked example (802.11-2024 Annex I.1): every bit table exactly, the 881 time samples within the annex's 3-decimal print rounding. The receive half (R0-07…R0-10) is next.

| Stage | What | Exit gate | Estimate |
|---|---|---|---|
| **R0** Golden model | Rust PHY/MAC model in fixed point, channel simulator, 802.11a-level transmit and receive chain | The model encodes the standard's worked example frame bit-exactly and decodes it back through a simulated channel | 0–3 months |
| **R1** Over the air on SDR | The same model running in real time on a PC with an SDR; first field measurements | A link over the air; range measured and logged | 3–6 months |
| **R2** RTL blocks | FFT, synchronisation, equaliser, convolutional/LDPC codecs, TDMA scheduler in RTL; ASIC probes in parallel | Every block passes G3 (and G4 where required) | 6–12 months |
| **R3** FPGA link | Zynq + AD9361, two nodes, Linux netdev; WebRTC and MAVLink unchanged on top | Video from the drone over 10 km | 9–15 months |
| **R4** Serial module | Own board for the drone, CE/RED | CE/RED passed, module in series production | 12–24 months |
| **R5** Baseband ASIC | Digital chip: baseband + MAC + RISC-V, external RF transceiver; MPW on the IHP open PDK, then GF 22FDX | First silicon works with the external RF | 2–4 years |
| **R6** Full SoC | Own RF on the die: Wi-Fi 6/6E, HaLow, TDMA | Serial open chip | 4+ years |

## Profiles of the one core

- **Long-range TDMA** — 802.11 OFDM PHY with 5/10/20 MHz channels plus our TDMA MAC with the dock
  as master (the approach MikroTik NV2 and Ubiquiti airMAX proved at 10+ km). First user: Nestua.
- **Wi-Fi 4 → 6/6E → 7** — Wi-Fi 4 is the stepping stone; the target is Wi-Fi 6/6E (OFDMA,
  1024-QAM, 6 GHz); Wi-Fi 7 after that.
- **HaLow (802.11ah)** — the same OFDM, roughly 10× slower clocking, 863–868 MHz in the EU.

## Budget envelope (lean path, rough orders)

R0–R1 €20–50k, R2 €10–30k, R3 €40–90k, R4 €100–250k, R5 test chips €20–200k. Serial masks and the
full RF SoC need external funding (€1M+); we go for it after R3 and a working R5 test chip.
Strategy, patents and funding are summarised in ADRs 0001, 0002 and 0005.
