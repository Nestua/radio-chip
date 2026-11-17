# Gate journal: PPDU builder

Block: complete 802.11a/g PPDU — short and long training fields, SIGNAL, DATA (SERVICE, PSDU,
tail, pad; scrambling, coding, puncturing, interleaving, mapping, OFDM) (IEEE 802.11-2020
§17.3.2–17.3.5). Model: `crates/radio-phy/src/ppdu.rs`. Composes R0-01…R0-05 and R0-11.

## 2026-11-17 · G0 · first commit
- Frame: STF 160 + LTF 160 + SIGNAL 80 + N_SYM · 80 samples, no transition window.
- STF: S_k = √(13/6)·s·(1+j) on k = ±4…±24 (step 4), 64-point IFFT, repeated (period 16) over
  160 samples. LTF: L_(−26..26) ±1, GI2 = last 32 samples, then the symbol twice.
- SIGNAL: RATE R1…R4 (R1 first), reserved 0, LENGTH 12 bits LSB first, even parity over bits
  0…16, 6 tail zeros; BPSK rate 1/2, not scrambled, pilot polarity p_0.
- DATA: SERVICE 16 zeros + PSDU (octets LSB first) + 6 tail + pad zeros to N_SYM·N_DBPS,
  N_SYM = ⌈(16 + 8·LENGTH + 6)/N_DBPS⌉; scrambled with the given seed, then the tail reset to 0;
  coded and punctured continuously; interleaved and mapped per symbol; symbol k uses p_(k+1).
- RATE bits: 6 = 1101, 9 = 1111, 12 = 0101, 18 = 0111, 24 = 1001, 36 = 1011, 48 = 0001,
  54 = 0011 (transmission order). **Correction during this gate:** the first draft took a table
  from OpenOFDM's hardware-LUT generator (`gen_deinter_lut.py`), which lists the same bits
  mirrored; the full decode read our 6 Mb/s frames as 36 Mb/s and the reverse, and failed every
  rate whose bits are not palindromes. The receiver's own table (`decode.py`) and the decode now
  agree with the values above.

## 2026-11-17 · G1 · first commit
Commands: `cargo test -p radio-phy --test g1_ppdu --test g1_ppdu_openofdm_decode`
References:
1. OpenOFDM receiver tables (Apache-2.0, commit ea638eb), printed unmodified
   (`tools/ref/preamble_openofdm.py` → `tests/fixtures/preamble_openofdm.txt`): the long-
   training samples and one short-training period in the 3-decimal format of the standard's
   annex, and LTS_REF. Least-squares fit of our fixed-point fields to them: **scale 0.9998
   (STF) and 0.9997 (LTF), residual 0.59 % and 0.43 %** — same shape and same absolute level, no
   correction factor (sample 0 of the LTF is excluded: the annex shows it windowed). LTS_REF
   equals our L_k on all 52 subcarriers.
2. Structural rules: SIGNAL layout and parity for every rate and edge lengths; N_SYM; DATA
   stream = scrambled SERVICE+PSDU with the tail reset; frame length and field order.
3. **OpenOFDM decodes our whole frames**: `rvec ppdu-frames` (3 frames per rate: 1, tens and
   hundreds of octets, random PSDUs and seeds) → int16 IQ file → `Decoder.decode_next` run
   unmodified (`tools/ref/ppdu_decode_openofdm.py`, `python:2.7-slim`, numpy 1.16.6, its bundled
   CommPy for Viterbi; matplotlib and wltrace's dot11 parser stubbed) →
   `tests/fixtures/ppdu_decode_openofdm.txt`. Run natively on the build server (x86; 4.5 min
   for 24 frames — the pure-Python Viterbi is slow under emulation on the Mac). The test rebuilds
   every frame bit-exactly (FNV-1a hash) and checks the verdict: **SIGNAL rate, length and parity
   correct on 24/24; PSDU octet-exact on 23/24, at all eight rates** — including 2/3 and 3/4
   puncturing, which closes R0-02's open question about the patterns.
Result: 7/7 pass. Test-first: the six structural/preamble tests failed against the stub.

Known receiver limitation (the 24th frame): 9 Mb/s, 1 octet, only 6 pad bits after the tail.
OpenOFDM's Viterbi appends zeros and traces back from the end as if the encoder ended in the
zero state; our pad bits are scrambled, as §17.3.5.4 requires. Experiment on the build server
(scratch program, not committed): the same 1-octet PSDU at 9 Mb/s with eight seeds decoded on
4 and failed on 4, depending only on the pad values; 2–5 octet frames (10–34 pad bits) all
decoded. The gate therefore exempts at most one frame that both has < 12 pad bits at a
punctured rate and mismatches, and still requires its SIGNAL to decode.

Mutation check (each restored with `cp`):
| Mutation | g1_ppdu | decode |
|---|---|---|
| L_−26 sign flipped | 2 fail | fails |
| STF without √(13/6) | 1 fails | fails |
| STF bin +4 sign flipped | 1 fails | fails |
| SIGNAL parity odd | 1 fails | fails |
| Tail not reset after scrambling | 1 fails | fails |
| GI2 from the first half | 1 fails | fails |
| RATE bits mirrored (the real bug) | passes | fails |
| DATA polarity starts at p_0 | passes | fails |

## Open questions
1. The standard's annex worked example (SIGNAL and DATA bits and samples for its 100-octet
   message) once the text is at hand — the R0 exit gate names it explicitly. Source: IEEE GET,
   802.11-2024 (ieeexplore document 10979691; free, needs a personal IEEE account; the PDF is
   licensed for personal use, so it stays in gitignored `target/ext/` — whether its numbers may
   be committed as fixtures is a licence question for the founder). 2026-11-17: the free public
   draft of that example, IEEE 802.11-99/107 (April 1999, ieee802.org 1999 archive), was
   checked and is **not usable as a reference**: it predates the final 802.11a text — its short
   training sequence differs from the published one (ours matches the published one, see
   evidence 1) and its SIGNAL field puts LENGTH before RATE. Only the published annex counts.
2. A receiver that handles short pad correctly (our own Viterbi, R0-10) should decode the
   exempt frame; add it as a cross-check then.
