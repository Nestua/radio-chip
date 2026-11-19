# Gate journal: IQ capture files

Block: reading, writing, checking and replaying IQ captures for the SDR and field stages (R0-13;
gates G5 and G6 will rest on it). Code: `crates/radio-capture` (no dependencies),
`crates/radio-tools/src/bin/rcap.rs`.

## 2026-11-19 · G0 · format and interface

- **Format: SigMF v1.2.** A recording is `<name>.sigmf-meta` (JSON) and `<name>.sigmf-data`
  (raw interleaved I, Q). It is the open format for IQ recordings, read by GNU Radio, inspectrum
  and the SDR vendors' tools, and its metadata has a place for what a field capture must carry:
  sample rate, centre frequency, time, hardware, a checksum, annotations.
  - Implementing the format is not copying code (ADR 0005): the reader and writer here are
    written from the specification's field names; the specification's text and examples are
    not in the repo.
  - Rejected: a format of our own (nothing else would read it), and bare `.cs16` files (no
    metadata, so a capture cannot say what it is).
- **Sample formats:** `ci16_le` (the SDRs' native format and our 12-bit ADC in 16 bits),
  `cf32_le`, `ci8`, `cu8`. Others are refused by name.
- **Typed fields:** `core:datatype`, `core:sample_rate` (required here: a receiver cannot use
  a recording without it), `core:version`, `core:sha512`, `core:description`, `core:hw`,
  `core:author`; per capture segment `core:sample_start`, `core:frequency`, `core:datetime`;
  per annotation `core:sample_start`, `core:sample_count`, `core:label`, `core:comment`.
  Every other field, at every level, is kept as read and written back.
- **Checksum:** SHA-512 of the dataset (FIPS 180-4), lower-case hexadecimal in `core:sha512`.
  Written always; checked on reading whenever present. A dataset that is not whole samples is
  refused. The streaming reader checks both when it reaches the end.
- **JSON:** RFC 8259, strict (no trailing commas, no bare control characters, no lone
  surrogates, nothing after the value), nesting limited to 64. Objects keep their member
  order and numbers keep their text, so a 64-bit sample index is not pushed through a double.
- **`rcap`:** `info`, `verify`, `synth` (a simulated capture with an annotation per frame),
  `replay` (the fixed-point front over a `ci16_le` capture, optionally dropping low bits: 4
  for an SDR's 16-bit samples into the model's 12-bit ADC).
- No dependency was added: SHA-512 and the JSON reader are about 500 lines, each with an
  outside reference. The workspace still has none.

## 2026-11-19 · G1

**References we did not write** (`tools/ref/capture_python.py`: CPython's standard library as
a black box, Python 3.9.6; fixtures in `crates/radio-capture/tests/fixtures/`)
| # | Reference | Test (`radio-capture/tests/g1_capture.rs`) |
|---|---|---|
| 1 | FIPS 180-4's printed examples: "abc", the 896-bit message, the empty message | `sha512_fips_examples` |
| 2 | `hashlib.sha512` on 328 messages: every length to 299, around the block boundaries to 5 blocks, 1000, 4096, 65 536 and 1 000 003 bytes; each also fed in uneven pieces | `sha512_matches_hashlib` |
| 3 | `json.dumps`: 23 values with every escape, control characters, Cyrillic, an astral character, U+2028, integers beyond 2⁵³, exponents, nesting; each written with `\u` escapes and raw | `json_reads_what_python_writes` |
| 4 | A SigMF recording written by Python (two segments, an annotation, foreign fields, a 600-sample ramp, `hashlib`'s checksum) | `reads_the_recording_python_wrote` |
| 5 | Python's verdict on a recording we wrote: `json.load` reads it, the three top-level keys, the fields, `hashlib` agrees with our checksum, the first samples, the description's characters | `python_reads_the_recording_we_write` |

- Reference 5 holds for today's code because the test first checks that the committed
  `rs.sigmf-*` are byte for byte what the code writes, and that the verdict names that file's
  hash. Regenerate with `UPDATE_FIXTURES=1 cargo test -p radio-capture`, then
  `python3 -I tools/ref/capture_python.py check crates/radio-capture/tests/fixtures`.

**Properties**
- `round_trip_keeps_everything`: four formats, 0 to 4097 samples, awkward strings, foreign
  fields at three levels, a `sample_start` of 2⁶⁴ − 1: read back equal, and written again byte
  for byte the same. The sample values of each format at their extremes.
- `json_refuses_what_is_not_json`: 30 malformed texts; nesting 64 accepted, 66 and 100 000
  refused (no stack overflow).
- `damage_is_reported`: one flipped bit of the dataset (both readers), a dataset one byte
  short, no dataset, seven kinds of wrong metadata, capture segments out of order.

**Replay of a known capture** (`radio-tools/tests/g1_capture_replay.rs`): `rcap synth` writes
six frames at 12 dB SNR; the fixed-point front finds six acquisitions, the same through the
file in 1009-sample pieces as from the samples, each within one sample of where the
simulator put the frame; `rcap replay` lists them; `rcap verify` refuses the file after one
flipped bit.

**Mutations** (each fails the tests named)
| Mutation | Fails |
|---|---|
| one bit of K[79]; σ0's shift; length in bytes | FIPS examples, hashlib, both recordings |
| padding boundary 112 → 111 | hashlib (not the FIPS examples) |
| surrogate pair offset; `\b`; control characters unescaped on writing | Python's JSON |
| leading zeros; text after the value; no depth limit; raw control characters accepted | refusals |
| checksum not compared; length not checked | damage |
| `cu8` offset; unknown fields dropped | round trip |
| 16-bit samples big endian on writing | Python's verdict |
| capture order not checked | damage (after a test was added for it) |

"Maj as an OR of the three ANDs" is equivalent (the majority function either way).

**Open questions**
1. Validation by the SigMF project's own library (`sigmf` on PyPI, its JSON schema) is not
   done: installing a package needs the founder's permission, which was not available during
   the session. It would be a sixth reference; until then the field names rest on references
   4 and 5, both written from our reading of the specification.
2. Time: `core:datetime` is kept as text. A field capture needs the time base (GPS, NTP, the
   SDR's clock) defined before it means anything; that is R1's.
3. Recording from an SDR is not here: R1 chooses the SDR and its driver. This block is what
   every recorder will write and every analysis will read.
4. `rcap replay` runs the front only. The decode of what it finds comes with R0-09.
5. Captures live outside git (`/captures/`); a journal entry names a capture by its
   `core:sha512`.
