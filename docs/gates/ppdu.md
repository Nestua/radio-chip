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

## 2026-11-17 · G1 · the standard's worked example
Command: `cargo test -p radio-phy --test g1_annex_i1 -- --ignored`
Reference: IEEE Std 802.11-2024 Annex I.1 "Example 1—BCC encoding" (36 Mb/s, 100-octet PSDU,
"Ode to Joy"), from the user's personal copy via IEEE GET (ieeexplore 10979691). The PDF and the
extracted numbers stay in gitignored `target/ext/` (founder's decision 2026-11-17: commit the test
and the extractor, not the numbers). Extraction: `python3 -I tools/ref/annex_i1_extract.py
<pdf> target/ext/annex_i1.fixture` (`pdftotext -layout`, poppler). Table I-1 cross-checked
against the PSDU the test takes from I-13: 100/100 octets.
Result: 4/4 tests pass.
- SIGNAL: I-7 bits, I-8 coded, I-9 interleaved — bit-exact; I-10/I-11 within ±0.0005.
- DATA: I-13 (clear), I-14 (scrambling sequence = our seed 0x5D), I-15 (scrambled), I-16 (coded,
  rate 3/4), I-17/I-18 (permutations), I-19 (interleaved) — bit-exact; I-20 within ±0.0005.
- Whole PPDU, I-22…I-30, samples 0…880 (STF, LTF, SIGNAL, 6 DATA symbols): the annex applies the
  informative ½-weight window and overlaps one sample at each field boundary, so the test
  compares ½·(x[b] + x[b−64]) at b = 0, 160, 320, 400, …, 880 and plain samples elsewhere.
  Worst |ours − annex|: **0.00050 at W = 24** (the print rounding itself) and **0.00132 at the
  transmit W = 13**, over all 881 samples.
Mutation check (each restored with `cp`): 3/4 puncturing pattern changed → data and frame tests
fail; PILOT_BASE reordered → SIGNAL, DATA and frame fail; STF bin −24 sign flipped → frame fails
at sample 0; SIGNAL parity inverted → SIGNAL and frame fail.
This closes the transmit half of the R0 exit gate ("encodes the standard's worked example frame
bit-exactly"); the decode-back half waits for R0-07…R0-10.

## 2026-11-19 · G1 · R0-06b: Annex I.1 in CI, by hashes

Until today the strongest reference of the transmitter ran only where a licensed copy of the
standard was at hand (`g1_annex_i1`, `#[ignore]`d in CI). Decided by the founder on
2026-11-19 (ADR 0005, addendum): the repository holds the SHA-512 of each table, not the
tables; and, asked separately, the example's input (the 100-octet PSDU of Table I-1), without
which nothing downstream of the SIGNAL field can be computed.

- `tools/ref/annex_i1_hash.py` (CPython hashlib, run on the local fixture of
  `annex_i1_extract.py`) → `crates/radio-phy/tests/fixtures/annex_i1_hashes.txt`: the PSDU
  and 21 hashes, each over a canonical text of one table (bit tables by index, octet tables
  in transmission order, the composition of the two interleaver permutations I-17 and I-18,
  frequency-domain tables with X for an empty pilot, the nine tables of time samples; values
  with three decimals, −0.000 written 0.000).
- `tests/g1_annex_i1_hashes.rs` (runs in CI): the model computes each table from the PSDU,
  writes the same text and must reach the same hash. Hashing is `radio-capture`'s SHA-512
  (itself held to hashlib, `docs/gates/capture.md`), a dev-dependency inside the workspace.

| Test | Tables |
|---|---|
| `signal_field` | I-7, I-8, I-9 (bits), I-10, I-11 (subcarriers without and with pilots) |
| `data_bits_and_scrambler` | I-13, I-15, I-16 (864, 864, 1152 bits), I-14 (exactly one of the 127 seeds gives the sequence: 0x5D), I-17 ∘ I-18, I-19, I-20 |
| `whole_ppdu_is_the_printed_waveform` | I-22 … I-30: all 881 samples, 1762 numbers, equal to the print in every third decimal |
| `printed_waveform_decodes` | the receive half: that waveform, rounded to the printed digits, through `decode_ppdu` at five window positions gives the PSDU, 36 Mb/s, length 100, seed 0x5D |

**What the hash of the waveform rests on**
- It holds the 24-bit transmitter (`IfftConfig::with_word_bits(24)`): measured before
  writing the gate, the number of printed values the model does not round to is 303 of
  1762 at W = 13, 65 at W = 16, 3 at W = 20 and 0 at W = 24. The 13-bit transmitter of the
  design is still held to the print within its tolerance by the local gate only.
- The transmitter is integer arithmetic, so the text is the same on every machine. 12 of
  the 1762 values sit exactly on a rounding boundary of the print (dyadic values); the
  nearest other one is 2.4·10⁻⁷ from a boundary, about two LSB of the 24-bit word. A change
  of the IFFT's rounding can therefore break a hash while the print still holds: the hash
  says "different", the local gate (`cargo test -p radio-phy --test g1_annex_i1 --
  --ignored`) says where and by how much. A false alarm is possible; a missed difference in
  a printed digit is not.
- Not hashed (the local gate does not use them either): I-2 … I-6 and I-12 (the preamble and
  SIGNAL in pieces; their samples are inside I-22 and I-23) and I-21.
- A hash of a short table is not secrecy: I-7's 24 bits can be found by search. The tables
  are kept out because they are the document's text, and the model reproduces them anyway.

**Mutations** (each fails the tests named; `cp` + `touch` to restore)
| Mutation | Fails |
|---|---|
| encoder generator 171 → 175 (octal) | `signal_field`, `data_bits_and_scrambler`, waveform |
| scrambler x⁷ ⊕ x⁴ → x⁷ ⊕ x³ | `data_bits_and_scrambler`, waveform, decode |
| interleaver, second permutation off by one | `data_bits_and_scrambler`, waveform |
| sign of one pilot | `signal_field`, `data_bits_and_scrambler`, waveform |
| 16-QAM normalisation 1/√10 → 1/√9 | `data_bits_and_scrambler`, waveform |

**Evidence:** `cargo test -p radio-phy --test g1_annex_i1_hashes` — 4 pass; the local
`g1_annex_i1 -- --ignored` — 5 pass on the same tree; `scripts/check.sh` green; this entry's
pull request.

## Open questions
1. Annex worked example: done (entry above, commit the test + extractor, numbers stay local).
   Keep in mind: the free public draft IEEE 802.11-99/107 (April 1999) is **not usable as a
   reference** — it predates the final 802.11a text (different short training sequence, LENGTH
   before RATE in SIGNAL). Only the published annex counts.
2. ~~Decode the exempt frame with our own Viterbi~~ — done in R0-10 (`g1_viterbi`): it decodes
   with the termination after the tail, and forcing state 0 at the frame end reproduces
   OpenOFDM's 0x28 exactly.
