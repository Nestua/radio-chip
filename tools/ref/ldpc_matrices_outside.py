"""An outside copy of the HT LDPC matrix prototypes, as a reference for our transcription
of Annex F (crates/radio-phy/src/ldpc_tables.rs).

Source: github.com/tavildar/LDPC (MIT licence), file LdpcC/WiFiLDPC.h at commit
05ee7f4af36ed5dacf52861315af68b8a17e71e0 (SHA-256 of the file
b2a6e22dce37ab57c883503f7078feb16fcc8e5dc029efc7bc596c10de196934): a C++ simulator's tables
of the twelve 802.11n matrices, written by someone else from the standard. Only its numbers
are read, as data; no code of it is run, copied or translated (ADR 0005).

    curl -sLO https://raw.githubusercontent.com/tavildar/LDPC/05ee7f4af36ed5dacf52861315af68b8a17e71e0/LdpcC/WiFiLDPC.h
    python3 -I tools/ref/ldpc_matrices_outside.py WiFiLDPC.h \
        crates/radio-phy/tests/fixtures/ldpc_matrices_outside.txt

Writes, per code, the SHA-512 (hashlib) of the canonical text of its prototype: one line per
row, the 24 entries separated by one space, -1 for the zero block. The model's test writes
the same text from its own table and compares.
"""

import hashlib
import re
import sys

SHA256 = "b2a6e22dce37ab57c883503f7078feb16fcc8e5dc029efc7bc596c10de196934"
MATRIX = re.compile(r"int\s+H_(\d+)_(\d)_(\d)\[(\d+)\]\[(\d+)\]\s*=\s*\{(.*?)\};", re.S)


def main(src, dst):
    raw = open(src, "rb").read()
    assert hashlib.sha256(raw).hexdigest() == SHA256, "not the file this script was written for"
    found = {}
    for n, num, den, rows, cols, body in MATRIX.findall(raw.decode()):
        matrix = [[int(v) for v in row.split(",")] for row in re.findall(r"\{([^{}]*)\}", body)]
        assert len(matrix) == int(rows) and all(len(r) == int(cols) == 24 for r in matrix), (n, num, den)
        found[(int(n), f"{num}/{den}")] = matrix
    want = [(n, r) for n in (648, 1296, 1944) for r in ("1/2", "2/3", "3/4", "5/6")]
    assert sorted(found) == sorted(want), sorted(found)
    with open(dst, "w", encoding="utf-8") as out:
        out.write("# HT LDPC matrix prototypes from an outside source: SHA-512 of each one's canonical text.\n")
        out.write("# Source: github.com/tavildar/LDPC (MIT), LdpcC/WiFiLDPC.h, commit 05ee7f4af36ed5dacf52861315af68b8a17e71e0.\n")
        out.write("# Tool: CPython hashlib. Command: python3 -I tools/ref/ldpc_matrices_outside.py WiFiLDPC.h <this file>\n")
        out.write("# <n> <rate> <rows> <sha512>\n")
        for n, r in want:
            text = "".join(" ".join(map(str, row)) + "\n" for row in found[(n, r)])
            out.write(f"{n} {r} {len(found[(n, r)])} {hashlib.sha512(text.encode()).hexdigest()}\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
