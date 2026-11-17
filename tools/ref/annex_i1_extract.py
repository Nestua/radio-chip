"""Extract the tables of IEEE Std 802.11-2024 Annex I.1 (Example 1 — BCC encoding) from your own
copy of the standard into the fixture read by crates/radio-phy/tests/g1_annex_i1.rs.

The standard is under the IEEE personal-use licence: neither the PDF nor the extracted numbers
are committed. Everything lands in target/ext/ (gitignored).

    pdftotext is required (brew install poppler)
    python3 -I tools/ref/annex_i1_extract.py path/to/80211-2024.pdf target/ext/annex_i1.fixture
    cargo test -p radio-phy --test g1_annex_i1 -- --ignored

Output, one value per line, keyed by table number:
    I-n pair <a> <b>              index/bit tables (I-7…I-9, I-14, I-17…I-19, I-21)
    I-n header <Bx-By> ...        bit order of the binary columns of an octet table
    I-n octet <8 binary digits>   octet tables (I-13, I-15, I-16), in the printed order
    I-n iq <index> <re> <im>      complex tables (I-2…I-6, I-10…I-12, I-20, I-22…I-30); X = empty
Tables continued across pages are merged; page furniture (headers, footers, the licence stamp)
is dropped. Table I-1 (the PSDU) is not parsed: the test takes the PSDU from I-13.
"""

import re
import subprocess
import sys

START = re.compile(r"^\s*I\.1\.1\s+Introduction\s*$")
END = re.compile(r"^\s*I\.2\s+Generating encoded DATA bits")
TABLE = re.compile(r"\s*Table (I-\d+)—")
SUBCLAUSE = re.compile(r"\s*I\.1\.\d")
NUM = r"-?\d+\.\d+|X"


def annex_lines(pdf):
    text = subprocess.run(
        ["pdftotext", "-layout", pdf, "-"], check=True, capture_output=True, text=True
    ).stdout
    lines = text.replace("–", "-").replace("−", "-").splitlines()
    # The table of contents also lists I.1.1, with leader dots; the body heading has none.
    start = next(i for i, l in enumerate(lines) if START.match(l))
    end = next(i for i in range(start, len(lines)) if END.match(lines[i]))
    return lines[start:end]


def tables(lines):
    out, order, cur = {}, [], None
    for ln in lines:
        m = TABLE.match(ln)
        if m:
            cur = m.group(1)
            if cur not in out:
                out[cur] = []
                order.append(cur)
            continue
        if SUBCLAUSE.match(ln):
            cur = None
            continue
        if cur:
            out[cur].append(ln)
    return [(t, out[t]) for t in order]


def main(pdf, dest):
    with open(dest, "w", encoding="utf-8") as f:
        for t, body in tables(annex_lines(pdf)):
            joined = "\n".join(body)
            cplx = re.findall(rf"(?<!\S)(-?\d+)\s+({NUM})\s+({NUM})(?!\S)", joined)
            octs = re.findall(r"(?<![\d.])([01]{8})(?![\d.])", joined)
            if cplx:
                for i, re_, im in cplx:
                    f.write(f"{t} iq {i} {re_} {im}\n")
            elif octs:
                hdr = re.findall(r"B(\d+)\s+B(\d+)", joined)
                f.write(f"{t} header {' '.join(a + '-' + b for a, b in hdr[:4])}\n")
                for o in octs:
                    f.write(f"{t} octet {o}\n")
            else:
                # Page furniture has no "<int> <int>" pairs; the page number line is a single int.
                for ln in body:
                    for a, b in re.findall(r"(?<!\S)(\d+)\s+(-?\d+)(?!\S)", ln):
                        f.write(f"{t} pair {a} {b}\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
