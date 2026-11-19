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
- **Refused by name**, because reading on would give wrong samples: `core:num_channels`
  other than 1, `core:dataset` (a dataset under another name), `core:trailing_bytes` and
  `core:header_bytes` other than 0, `core:metadata_only`. Also refused: a member name twice
  at the top level or in `global`, and a metadata file whose name does not end in
  `.sigmf-meta` (the dataset's name is derived from it and must never be the same file).
- **Writing** refuses what could not be read back: a sample rate or frequency that is not
  finite, a dataset that is not whole samples. The dataset is written before the metadata.
- **Checksum:** SHA-512 of the dataset (FIPS 180-4), lower-case hexadecimal in `core:sha512`.
  Written always; checked on reading whenever present. A dataset that is not whole samples is
  refused. The streaming reader checks both when it reaches the end.
- **JSON:** RFC 8259, strict (no trailing commas, no bare control characters, no lone
  surrogates, nothing after the value), at most 64 arrays and objects inside one another.
  Objects keep their member order and numbers keep their text; whether a number is an integer
  is decided on its digits (`9007199254740993.0` is one, `0.99999999999999999999` is not).
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
- `json_refuses_what_is_not_json`: 30 malformed texts; nesting 64 accepted, 65 and 100 000
  refused (no stack overflow), for arrays and for objects.
- `json_reads_what_python_writes` also holds what Python never writes: `\/`, hexadecimal
  escapes in capitals, and 21 numbers at the edges of "is an integer".
- `damage_is_reported`: one flipped bit of the dataset (both readers; the streaming reader
  repeats its error), a dataset one byte short, no dataset, seven kinds of wrong metadata,
  the five unsupported fields and two duplicates, five wrong file names, three recordings
  that must not be written, capture segments out of order.

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

**Evidence:** `cargo test -p radio-capture -p radio-tools` — 8 + 1 tests pass; this entry's pull
request is the commit. The code was written before its tests in this block (the references
came first, as fixtures); that the tests can fail is shown by the mutations, not by a failing
first run.

**Pre-PR review (fresh agent, code only), 2026-11-19:** no defect in SHA-512 or in the JSON
reader and writer.
- Its own runs: 4800 random and edge messages against hashlib (single, in pieces, cloned);
  450 000 mutated JSON texts against Python's `json`, the only disagreement being lone
  surrogates, which Python accepts and we refuse on purpose; every text we write read back
  by Python.
- Fixed after the review:
  - third-party recordings with several channels, header or trailing bytes, a named dataset
    or no dataset were read as if they had none of these: now refused by name;
  - `rcap replay` with a shift of 32 or more used shift mod 32, and arguments that were not
    numbers silently became defaults: now refused;
  - `Recording::write` could write files its own reader refuses (a NaN rate as `null`, half a
    sample) and, given a name ending in `.sigmf-data`, overwrite the dataset with the
    metadata: now refused before anything is written;
  - "is an integer" went through a double (`9007199254740993.0` read as 2⁵³);
  - the nesting limit was 65, and the test did not pin it;
  - `Reader::next_chunk` could overflow on an absurd request and forgot an error on the next
    call;
  - duplicate member names were handled three different ways;
  - the escapes Python never writes (`\/`, capital hexadecimal) had no test: two mutations
    survived there;
  - Python's verdict now lists the member names of `global`, the capture and the annotation,
    so a misspelt name we write is caught by Python and not only by the byte comparison.
- Noted, not changed: most of `json_reads_what_python_writes` compares our reading of the
  escaped form with our reading of the raw form of the same value; the content itself is
  pinned by the spot checks and by reference 4. Writing is not atomic (no temporary file);
  the checksum catches a torn pair.

**Open questions**
1. Closed 2026-11-19 (entry below): validation by the SigMF project's own library.
2. Time: `core:datetime` is kept as text. A field capture needs the time base (GPS, NTP, the
   SDR's clock) defined before it means anything; that is R1's.
3. Recording from an SDR is not here: R1 chooses the SDR and its driver. This block is what
   every recorder will write and every analysis will read.
4. `rcap replay` runs the front only. The decode of what it finds comes with R0-09.
5. Captures live outside git (`/captures/`); a journal entry names a capture by its
   `core:sha512`.

## 2026-11-19 · G1 · the SigMF project's own library as a reference

The founder allowed the install. `tools/ref/capture_sigmf.py` runs `sigmf` 1.11.1 from PyPI
(LGPL-3.0; installed in a throw-away virtual environment and run as a black box, nothing
read or copied, ADR 0005) with numpy 2.0.2 on Python 3.9.6. The fixtures are committed, so CI
needs neither.

| Reference | Test (`radio-capture/tests/g1_capture.rs`) |
|---|---|
| A recording written by the library (`sg.sigmf-*`: two segments, an annotation with frequency edges, a 400-sample ramp, its checksum) | `reads_the_recording_the_sigmf_library_wrote` |
| The library's verdict on the recording we write (`sg_verdict.txt` on the committed `rs.sigmf-*`): valid against the project's JSON schema, checksum verified on load, 500 samples, the first four read back, rate, frequency, annotation | `the_sigmf_library_accepts_the_recording_we_write` |

- Both tests passed on the first run: the writer and reader of #34 needed no change.
- "Valid" is shown to mean something inside the verdict: the same metadata is refused by the
  library with `core:datatype` missing or unknown (`ci17_le`), the sample rate or the
  frequency as text, `core:sample_start` missing or negative (six lines, all `True`).
- The library writes `core:num_channels: 1` and `core:offset: 0` on its own. Our reader keeps
  both among the other fields (one channel and no offset are what it assumes; other values
  are refused, #34).
- Regenerate: `UPDATE_FIXTURES=1 cargo test -p radio-capture`, then both `check` commands
  (`capture_python.py`, `capture_sigmf.py`).

**Mutations of the writer** (fixtures regenerated, both verdicts rerun, then everything restored)
| Mutation | CPython verdict (reference 5) | `sigmf` library |
|---|---|---|
| `core:sample_count` written as text `"320"` | passes: it prints the same | refuses (the library fails on it), the verdict is not produced and the test fails |
| datatype written as `ci16` (no byte order) | fails (`ci16` ≠ `ci16_le`) | **accepts** |
| `core:version` written as `1.2` | fails | **accepts** (the library overwrites the version on load) |

The two references cover different mistakes: the library knows the types of the fields, the
CPython verdict the exact texts. Neither replaces the other. The library is lenient where
the specification is not (a multi-byte datatype without `_le`/`_be`): its "valid" is weaker
than the specification's.

**Evidence:** `cargo test -p radio-capture` — 10 tests pass; this entry's pull request.
