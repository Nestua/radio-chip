# Backlog — R0 (golden model)

Source of truth for scope. Each item lists its reference (for G1) and the gates it needs.
Status: `todo` · `doing` · `done` (with the journal that proves it).

The R0 transmit chain follows the 802.11a/g OFDM PHY (IEEE 802.11-2020 Clause 17) because the
standard prints a fully worked frame encoding we can check every stage against. The receive chain,
LDPC and the TDMA MAC follow. Get the standard through the IEEE GET Program (802 standards are free). Clause numbers below were
written from memory for 802.11-2020; confirm each one against the edition we use at G0.

| ID | Block | Reference for G1 | Gates | Status |
|---|---|---|---|---|
| R0-01 | Scrambler, S(x) = x^7+x^4+1 (§17.3.5.5) | 127-bit sequence printed in §17.3.5.5 + Annex I.1 (Tables I-14, I-15) | G1, G3 | done — `docs/gates/scrambler.md` |
| R0-02 | Convolutional encoder K=7, (133, 171)₈, puncturing for rates 2/3, 3/4 (§17.3.5.6) | CommPy black box + UBC worked example on Fig. 17-8 + 802.11-2024 Annex I.1 (Tables I-8, I-16) | G1 | done — `docs/gates/convolutional_encoder.md` (patterns confirmed by OpenOFDM's receiver, R0-06) |
| R0-03 | Interleaver, two permutations per symbol (§17.3.5.7) | OpenOFDM black box (all 8 rates) + §17.3.5.7 design properties + Annex I.1 (Tables I-9, I-17…I-19) | G1 | done — `docs/gates/interleaver.md` |
| R0-04 | Subcarrier mapping BPSK…64-QAM, Gray, normalisation (§17.3.5.8) | OpenOFDM black box on every point + Gray/energy properties + Annex I.1 (Tables I-10, I-11, I-20) | G1 | done — `docs/gates/mapper.md` (unnormalised grid; K_MOD in fixed point with R0-05/R0-11) |
| R0-05 | OFDM modulation: 64-point IFFT, pilots and polarity, cyclic prefix (§17.3.5.10) | OpenOFDM tables + OpenOFDM receiver decoding our symbols + textbook DFT + Annex I.1 samples | G1, G2 | done — `docs/gates/ofdm.md` (W = 13, approved by the founder) |
| R0-06 | PPDU builder: preamble (STF/LTF), SIGNAL, DATA, tail and pad bits | OpenOFDM decodes our frames at all 8 rates + Annex I.1 worked example, all 881 samples (local fixture, `g1_annex_i1`) | G1 | done — `docs/gates/ppdu.md` |
| R0-07a | Channel simulator I: seeded RNG, AWGN (SNR over the full 20 MHz), CFO from a ppm offset (±20 ppm per side, §17.3.9.5), Wiener phase noise from dBc/Hz | Exact SER of BPSK…64-QAM in AWGN (textbook formulas, erfc vs CPython black box); DFT shift theorem for CFO; Lorentzian PSD for phase noise | G1 | done — `docs/gates/channel.md` |
| R0-07b | Channel simulator II: fractional delay, SFO, fading taps (Bell/Jakes Doppler, Rician K), TGn multipath models A–F (SISO) | TGn Appendix C tables (IEEE 802.11-03/940r4); Fourier shift theorem; Rayleigh/Rician moments; J0 autocorrelation; BPSK BER in Rayleigh fading (Proakis) | G1 | done — `docs/gates/channel.md` |
| R0-07c | Front-end impairments for R0-08: DC offset, CW tone, AGC gain step | Exact DFT of a tone, geometric series, Schmidl–Cox metric of a tone, dB definitions | G1 | done — `docs/gates/channel.md` |
| R0-07d | Drone air-to-ground channel (two-ray, elevation-dependent K, path loss) | Published air-to-ground measurement models (e.g. NASA/Matolak, 3GPP TR 36.777) | G1 | todo |
| R0-08a | Receiver front, float reference: DC blocker, two detectors compared (STF autocorrelation vs sign-quantised STF matched filter), coarse + fine CFO combined, LTF timing with energy window and back-off in ns | Ground truth through the channel simulator; CFO variance theory + CRB; outside receiver black box (OpenOFDM); end-to-end decode with R0-10 | G1 | done — `docs/gates/rx_front.md` (detector A chosen by measurement: Pd 0.999 at 4 dB, < 10⁻⁹ false alarms/sample) |
| R0-08b | Receiver front, fixed point: correlators, CORDIC, NCO, sign-quantised filters — first restated as a streaming receiver (causal timing, causal DC start-up) | Float reference of R0-08a | G1 (streaming), G2, G3 | done — `docs/gates/rx_front.md` (streaming G1; fixed point G2: CFO ≤ +0.5 %, FFT start identical in 99.96 %, 0 false alarms in 10⁹; G3: LTF correlator done, CORDIC/NCO, DC + detector, top level next) |
| R0-08c | Tune detector A to the maximum (founder, 2026-11-18): window L and run length vs Pd/latency, per-mode thresholds (802.11 vs slot-gated TDMA), plateau-shape and energy-rise checks, adaptive threshold from the noise floor, early-detection latency for a longer STF budget, spur resilience (notch / switch to exact-tap B while A saturates), hybrid A + B for TDMA at ≤ 2 dB; every change measured with `rxfront` at the 1 FA/s budget, AWGN + TGn B/D/E + impairments | The R0-08a gates and `rxfront` reports as baseline (Pd 0.825 @ 2 dB, 0.999 @ 4 dB AWGN; TGn D 0.988 @ 9 dB) | G1 | todo — `docs/gates/rx_front.md` open questions 4, 6 |
| R0-09 | Channel estimation and equalisation | BER vs SNR against theory and against the float model | G1, G2 | todo |
| R0-10 | Viterbi decoder (soft decision), depuncturing, termination from LENGTH | Brute-force ML on 12-bit blocks; published d_free 10/6/5; CommPy black-box BER; decodes all R0-06 frames incl. the one OpenOFDM missed | G1, G2 | done — `docs/gates/viterbi.md` (4-bit soft, window 64, 9-bit metrics: 0.11 dB) |
| R0-11 | Fixed-point framework: Q-format types, rounding/saturation modes, word-length sweep tool | fxpmath black box (2160 cases), 6.02·W+1.76 dB sine SQNR, exact-integer oracle; G2 reports of R0-05/08/09/10 use it | G1 | done — `docs/gates/fixed_point.md` |
| R0-12 | LDPC encoder/decoder for 802.11n codes, normalised min-sum (reuse the aleph relay-BP experience) | Code definitions from the standard; BER vs published curves | G1, G2 | todo |
| R0-13 | IQ capture tooling for R1: record/replay SDR files, checksum, metadata | Round-trip property; replay of a known capture | G1 | todo |

Next stage items (R1) are written when R0-06 passes.
