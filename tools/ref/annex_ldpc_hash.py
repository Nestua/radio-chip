"""The two LDPC worked examples of IEEE Std 802.11-2024 Annex I (I.2 and I.3) as references
for the LDPC encoder: hashes for CI, the bits themselves for a local gate.

Reads the text of your own copy of the standard and parses the bit tables I-32 … I-36
(example 1) and I-38 … I-42 (example 2). Writes
  <hashes>   committed: per table its length and the SHA-512 (hashlib) of its canonical text
             ("<n> <bit>\n" per bit, n counting in transmission order), and the two examples'
             inputs, the PSDUs of Tables I-31 and I-37, in hex (ADR 0005, addendum
             2026-11-19: tables by hash, a test's input in clear);
  <fixture>  not committed (target/ext/): "<table> <bits>" for the value-by-value gate.

    pdftotext -layout 80211-2024.pdf 80211-2024.txt
    python3 -I tools/ref/annex_ldpc_hash.py 80211-2024.txt \
        crates/radio-phy/tests/fixtures/annex_ldpc_hashes.txt target/ext/annex_ldpc.fixture
"""

import hashlib
import re
import sys

TABLE = re.compile(r"^\s*Table (I-\d+)—")
HEADER = re.compile(r"^\s*B(\d+)\s+B(\d+)\s+B\d+\s+B\d+")
ROW = re.compile(r"^\s*(\d+)-(\d+)\s+([01]{8})\s+(.*)$")
WANT = ["I-32", "I-33", "I-34", "I-35", "I-36", "I-38", "I-39", "I-40", "I-41", "I-42"]


def main(src, hashes, fixture):
    lines = open(src, encoding="utf-8").read().replace("–", "-").replace("−", "-").splitlines()
    start = next(i for i, l in enumerate(lines) if l.strip() == "I.2.1 General")
    bits, msb_first, table = {t: [] for t in WANT}, {}, None
    for line in lines[start:]:
        if m := TABLE.match(line):
            table = m.group(1) if m.group(1) in bits else None
            if m.group(1) == "I-43":
                break
        elif table and (m := HEADER.match(line)):
            first = int(m.group(1)) > int(m.group(2))
            assert msb_first.setdefault(table, first) == first, table
        elif table and (m := ROW.match(line)):
            # A row is up to three octets in binary, then the same in hexadecimal. The last
            # row of a table may be short: missing bits are printed as "-" (possibly spaced).
            lo, hi = int(m.group(1)), int(m.group(2))
            assert lo == len(bits[table]), (table, lo)
            binary = re.split(r"\s+(?=0x|----\s|----$)", m.group(3) + " " + m.group(4), maxsplit=1)
            printed = binary[0].replace(" ", "").replace("-", "")
            assert len(printed) == hi - lo + 1 and set(printed) <= {"0", "1"}, (table, lo)
            hexes = [h for h in binary[1].split() if re.fullmatch(r"0x[0-9A-F]{2}", h)]
            for k in range(0, len(printed), 8):
                o = printed[k : k + 8]
                if len(o) == 8:
                    # The hexadecimal column is the printed binary string read as a number.
                    assert int(o, 2) == int(hexes[k // 8], 16), (table, lo, o)
                bits[table].extend(int(c) for c in (o[::-1] if msb_first[table] else o))
    want_len = {"I-32": 816, "I-33": 816, "I-34": 1458, "I-35": 1944, "I-36": 1248,
                "I-38": 1136, "I-39": 1136, "I-40": 1296, "I-41": 2592, "I-42": 2592}
    assert {t: len(b) for t, b in bits.items()} == want_len, {t: len(b) for t, b in bits.items()}

    def psdu(t):
        b = bits[t][16:]
        return bytes(sum(v << i for i, v in enumerate(b[n : n + 8])) for n in range(0, len(b), 8))

    with open(hashes, "w", encoding="utf-8") as out:
        out.write("# IEEE Std 802.11-2024 Annex I.2 and I.3 (LDPC examples 1 and 2): SHA-512 of each bit\n")
        out.write("# table's canonical text (see tools/ref/annex_ldpc_hash.py), not the tables.\n")
        out.write("# Tool: CPython hashlib. Command: python3 -I tools/ref/annex_ldpc_hash.py <text> <this file> <fixture>\n")
        out.write("# <table> <bits> <sha512>; psdu1, psdu2 = the examples' inputs (Tables I-31, I-37) in hex.\n")
        out.write(f"psdu1 {psdu('I-32').hex()}\n")
        out.write(f"psdu2 {psdu('I-38').hex()}\n")
        for t in WANT:
            text = "".join(f"{n} {b}\n" for n, b in enumerate(bits[t]))
            out.write(f"{t} {len(bits[t])} {hashlib.sha512(text.encode()).hexdigest()}\n")
    with open(fixture, "w", encoding="utf-8") as out:
        for t in WANT:
            out.write(f"{t} {''.join(map(str, bits[t]))}\n")


if __name__ == "__main__":
    main(*sys.argv[1:4])
