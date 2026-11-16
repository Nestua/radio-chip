# Backlog — R0 (golden model)

Source of truth for scope. Each item lists its reference (for G1) and the gates it needs.
Status: `todo` · `doing` · `done` (with the journal that proves it).

The R0 transmit chain follows the 802.11a/g OFDM PHY (IEEE 802.11-2020 Clause 17) because the
standard prints a fully worked frame encoding we can check every stage against. The receive chain,
LDPC and the TDMA MAC follow. Get the standard through the IEEE GET Program (802 standards are free). Clause numbers below were
written from memory for 802.11-2020; confirm each one against the edition we use at G0.

| ID | Block | Reference for G1 | Gates | Status |
|---|---|---|---|---|
| R0-01 | Scrambler, S(x) = x^7+x^4+1 (§17.3.5.5) | 127-bit sequence printed in §17.3.5.5 | G1, G3 | done — `docs/gates/scrambler.md` |
| R0-02 | Convolutional encoder K=7, (133, 171)₈, puncturing for rates 2/3, 3/4 (§17.3.5.6) | Worked frame example in the standard's annex | G1 | todo |
| R0-03 | Interleaver, two permutations per symbol (§17.3.5.7) | Worked frame example | G1 | todo |
| R0-04 | Subcarrier mapping BPSK…64-QAM, Gray, normalisation (§17.3.5.8) | Worked frame example | G1 | todo |
| R0-05 | OFDM modulation: 64-point IFFT, pilots and polarity, cyclic prefix (§17.3.5.10) | Worked frame example (time-domain samples) | G1, G2 | todo |
| R0-06 | PPDU builder: preamble (STF/LTF), SIGNAL, DATA, tail and pad bits | The whole worked example end to end — the R0 exit gate, transmit half | G1 | todo |
| R0-07 | Channel simulator: AWGN, CFO, phase noise, multipath (TGn models), Doppler for drone speeds | Analytical BER curves for AWGN; published channel model definitions | G1 | todo |
| R0-08 | Receiver front: packet detect, coarse/fine CFO, symbol timing (STF/LTF) | Detection/false-alarm rates vs SNR in simulation; gr-ieee802-11 black-box decode of our frames | G1, G2 | todo |
| R0-09 | Channel estimation and equalisation | BER vs SNR against theory and against the float model | G1, G2 | todo |
| R0-10 | Viterbi decoder (soft decision) | BER curves vs published K=7 performance; decodes R0-06 frames | G1, G2 | todo |
| R0-11 | Fixed-point framework: Q-format types, rounding/saturation modes, word-length sweep tool | Unit properties; G2 reports of R0-05/08/09/10 use it | G1 | todo |
| R0-12 | LDPC encoder/decoder for 802.11n codes, normalised min-sum (reuse the aleph relay-BP experience) | Code definitions from the standard; BER vs published curves | G1, G2 | todo |
| R0-13 | IQ capture tooling for R1: record/replay SDR files, checksum, metadata | Round-trip property; replay of a known capture | G1 | todo |

Next stage items (R1) are written when R0-06 passes.
