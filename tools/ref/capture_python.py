"""References for the capture tooling (R0-13) from CPython's standard library, as black boxes.

hashlib.sha512 and the json module (PSF licence) are run, never read: we record what they
return. Our SHA-512 and our JSON reader and writer (crates/radio-capture) are written from
FIPS 180-4 and RFC 8259 and must agree with them.

    python3 -I tools/ref/capture_python.py make  crates/radio-capture/tests/fixtures
    python3 -I tools/ref/capture_python.py check crates/radio-capture/tests/fixtures

`make` writes:
  sha512_hashlib.txt     digests of deterministic messages of many lengths
  json_python.txt        JSON texts written by json.dumps, one per line, with awkward content
  py.sigmf-meta/-data    a SigMF recording written here (ci16_le, 600 samples of a known ramp)
`check` reads the recording our code wrote (rs.sigmf-meta/-data, committed) with json and
hashlib and writes its verdict to rs_verdict.txt.
"""

import hashlib
import json
import platform
import struct
import sys

TOOL = f"# Tool: Python {platform.python_version()}, hashlib and json from the standard library"


def message(n):
    """n bytes of a fixed pattern (the Rust test builds the same)."""
    return bytes((i * 131 + (i >> 8) * 17 + 7) & 0xFF for i in range(n))


def make(out):
    lengths = sorted(
        set(range(0, 300))
        | {i * 128 + d for i in range(1, 6) for d in (-18, -17, -16, -1, 0, 1, 111, 112, 113)}
        | {1000, 4096, 65536, 1_000_003}
    )
    with open(f"{out}/sha512_hashlib.txt", "w") as f:
        f.write("# SHA-512 reference: CPython hashlib, black box\n")
        f.write(TOOL + "\n")
        f.write("# Command: python3 -I tools/ref/capture_python.py make <dir>\n")
        f.write("# <length> <sha512 hex> of bytes ((i*131 + (i>>8)*17 + 7) & 255 for i < length)\n")
        for n in lengths:
            f.write(f"{n} {hashlib.sha512(message(n)).hexdigest()}\n")

    values = [
        {},
        [],
        "",
        0,
        -1,
        20000000,
        5.8e9,
        2400000000.5,
        1e-07,
        -3.25e17,
        0.1,
        12345678901234567890,
        True,
        False,
        None,
        "plain",
        "quote \" backslash \\ slash / tab \t newline \n return \r",
        "control \u0001 \u001f bell \b formfeed \f",
        "Київ, 5,8 ГГц — «тест»",
        "astral \U0001F4E1 and   separator",
        [1, [2, [3, [4, []]]], {"a": {"b": {"c": None}}}],
        {"core:datatype": "ci16_le", "core:sample_rate": 20000000, "list": [1.5, -2, "x"]},
        {"": 1, " spaced key ": [True, False, None], "k": "v"},
    ]
    with open(f"{out}/json_python.txt", "w") as f:
        f.write("# JSON written by CPython json.dumps, black box; one text per line.\n")
        f.write(TOOL + "\n")
        f.write("# Odd lines: ensure_ascii=True (\\u escapes); even lines: the same value, ensure_ascii=False.\n")
        for v in values:
            f.write(json.dumps(v, ensure_ascii=True) + "\n")
            f.write(json.dumps(v, ensure_ascii=False) + "\n")

    samples = [((7 * n) % 4096 - 2048, 2047 - (13 * n) % 4096) for n in range(600)]
    data = b"".join(struct.pack("<hh", i, q) for i, q in samples)
    with open(f"{out}/py.sigmf-data", "wb") as f:
        f.write(data)
    meta = {
        "global": {
            "core:datatype": "ci16_le",
            "core:sample_rate": 20000000,
            "core:version": "1.2.0",
            "core:sha512": hashlib.sha512(data).hexdigest(),
            "core:description": "ramp written by tools/ref/capture_python.py, \"quoted\", Київ",
            "core:hw": "none (synthetic)",
            "core:author": "capture_python.py",
            "nestua:adc_bits": 12,
        },
        "captures": [
            {
                "core:sample_start": 0,
                "core:frequency": 5805000000,
                "core:datetime": "2026-11-19T00:00:00Z",
            },
            {"core:sample_start": 300, "core:frequency": 5825000000.5},
        ],
        "annotations": [
            {
                "core:sample_start": 100,
                "core:sample_count": 50,
                "core:label": "burst",
                "core:comment": "first",
                "other:field": [1, 2, 3],
            }
        ],
    }
    with open(f"{out}/py.sigmf-meta", "w") as f:
        json.dump(meta, f, indent=4)
        f.write("\n")


def check(out):
    with open(f"{out}/rs.sigmf-meta", encoding="utf-8") as f:
        meta = json.load(f)
    with open(f"{out}/rs.sigmf-data", "rb") as f:
        data = f.read()
    g = meta["global"]
    lines = [
        "# Verdict of CPython json and hashlib on the recording our code wrote (rs.sigmf-*).",
        TOOL,
        "# Command: python3 -I tools/ref/capture_python.py check <dir>",
        f"meta_sha512 {hashlib.sha512(open(f'{out}/rs.sigmf-meta', 'rb').read()).hexdigest()}",
        f"top_level_keys {' '.join(sorted(meta))}",
        f"datatype {g['core:datatype']}",
        f"sample_rate {g['core:sample_rate']!r}",
        f"version {g['core:version']}",
        f"sha512_matches {g['core:sha512'] == hashlib.sha512(data).hexdigest()}",
        f"samples {len(data) // 4}",
        f"first_samples {' '.join(str(v) for v in struct.unpack('<8h', data[:16]))}",
        f"captures {len(meta['captures'])} frequency {meta['captures'][0]['core:frequency']!r}",
        f"annotations {len(meta['annotations'])}",
        f"global_keys {' '.join(sorted(g))}",
        f"capture_keys {' '.join(sorted(meta['captures'][0]))}",
        f"annotation_keys {' '.join(sorted(meta['annotations'][0]))}",
        f"datetime {meta['captures'][0]['core:datetime']}",
        f"annotation {meta['annotations'][0]['core:sample_start']} {meta['annotations'][0]['core:sample_count']} {meta['annotations'][0]['core:label']}",
        f"description {json.dumps(g['core:description'], ensure_ascii=True)}",
        f"canonical_sha512 {hashlib.sha512(json.dumps(meta, sort_keys=True, ensure_ascii=True, separators=(',', ':')).encode()).hexdigest()}",
    ]
    with open(f"{out}/rs_verdict.txt", "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    {"make": make, "check": check}[sys.argv[1]](sys.argv[2])
