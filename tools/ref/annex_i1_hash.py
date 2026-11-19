"""Hashes of the tables of IEEE Std 802.11-2024 Annex I.1, so that CI can check the model
against the standard's worked example without the example being in the repository.

Reads the local fixture written by annex_i1_extract.py (from your own copy of the standard)
and writes, per table, the SHA-512 (hashlib) of a canonical text of its content. The model's
test (crates/radio-phy/tests/g1_annex_i1_hashes.rs) builds the same text from its own output
and compares hashes. The tables themselves are not committed (ADR 0005, addendum
2026-11-19); the one exception is the example's input, the 100-octet PSDU of Table I-1,
without which no test can run (founder, 2026-11-19).

    python3 -I tools/ref/annex_i1_hash.py target/ext/annex_i1.fixture \
        crates/radio-phy/tests/fixtures/annex_i1_hashes.txt

Canonical text, one entry per line, lines ending in "\n":
    bits  I-7 I-8 I-9 I-14 I-19     "<index> <bit>", by index
    bits  I-13 I-15 I-16            "<n> <bit>", n counting bits in transmission order
    perm  I-17+I-18                 "<k> <j>", j = second[first[k]], k = 0…191
    iq    I-10 I-11 I-20            "<subcarrier> <re> <im>", −32…31, three decimals, X = empty
    iq    I-22 … I-30               "<sample> <re> <im>", three decimals
A value that prints as -0.000 is written 0.000.
"""

import hashlib
import sys
from collections import defaultdict


def num(s):
    if s == "X":
        return "X"
    t = f"{float(s):.3f}"
    return "0.000" if t == "-0.000" else t


def main(src, dst):
    pairs, octets, headers, iq = defaultdict(list), defaultdict(list), {}, defaultdict(dict)
    for line in open(src, encoding="utf-8"):
        f = line.split()
        if f[1] == "pair":
            pairs[f[0]].append((int(f[2]), int(f[3])))
        elif f[1] == "octet":
            octets[f[0]].append(f[2])
        elif f[1] == "header":
            headers[f[0]] = " ".join(f[2:])
        elif f[1] == "iq":
            iq[f[0]][int(f[2])] = (num(f[3]), num(f[4]))
        else:
            raise SystemExit(f"unknown kind {f[1]}")

    def octet_bits(t):
        msb_first = headers[t].startswith("7-0")
        return [int(c) for o in octets[t] for c in (o[::-1] if msb_first else o)]

    texts = {}
    for t in ("I-7", "I-8", "I-9", "I-14", "I-19"):
        entries = sorted(pairs[t])
        assert [i for i, _ in entries] == list(range(len(entries))), t
        texts[t] = entries
    for t in ("I-13", "I-15", "I-16"):
        texts[t] = list(enumerate(octet_bits(t)))
    first, second = dict(pairs["I-17"]), dict(pairs["I-18"])
    assert len(first) == len(second) == 192
    texts["I-17+I-18"] = [(k, second[first[k]]) for k in range(192)]
    for t in ("I-10", "I-11", "I-20"):
        assert sorted(iq[t]) == list(range(-32, 32)), t
    for t in ("I-10", "I-11", "I-20") + tuple(f"I-{n}" for n in range(22, 31)):
        texts[t] = [(i, *iq[t][i]) for i in sorted(iq[t])]
    assert sum(len(iq[f"I-{n}"]) for n in range(22, 31)) == 881

    bits = octet_bits("I-13")[16 : 16 + 800]
    psdu = bytes(sum(b << i for i, b in enumerate(bits[n : n + 8])) for n in range(0, 800, 8))

    with open(dst, "w", encoding="utf-8") as out:
        out.write("# IEEE Std 802.11-2024 Annex I.1 (Example 1 - BCC encoding): SHA-512 of each table's\n")
        out.write("# canonical text (see tools/ref/annex_i1_hash.py), not the tables. Tool: CPython hashlib.\n")
        out.write("# Command: python3 -I tools/ref/annex_i1_hash.py target/ext/annex_i1.fixture <this file>\n")
        out.write("# <table> <entries> <first index> <sha512>; psdu = the example's input, Table I-1, in hex.\n")
        out.write(f"psdu {psdu.hex()}\n")
        for t, entries in texts.items():
            text = "".join(" ".join(str(v) for v in e) + "\n" for e in entries)
            digest = hashlib.sha512(text.encode()).hexdigest()
            out.write(f"{t} {len(entries)} {entries[0][0]} {digest}\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
