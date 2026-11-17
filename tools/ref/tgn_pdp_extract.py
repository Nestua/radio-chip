"""Power-delay profiles of the TGn channel models B–F from the published document.

Source: V. Erceg et al., "TGn Channel Models", IEEE 802.11-03/940r4 (May 2004), Appendix C, a
public IEEE 802.11 contribution:
    https://mentor.ieee.org/802.11/dcn/03/11-03-0940-04-000n-tgn-channel-models.doc

macOS `textutil` converts the Word file to text keeping table cells as BEL (0x07) separators,
empty cells included, so each cluster's powers come out under the tap they belong to:

    textutil -convert txt -output tgn-r4.txt 11-03-0940-04-000n-tgn-channel-models.doc
    python3 -I tools/ref/tgn_pdp_extract.py tgn-r4.txt \
        > crates/radio-phy/tests/fixtures/tgn_pdp.txt

Only the SISO power-delay profile is extracted (tap delays and per-cluster powers); the MIMO
angle parameters are not used yet. Model A (one tap, flat fading) needs no table.
"""

import re
import sys

SECTION = re.compile(r"Appendix C\s*[–-]\s*Model ([A-F])(?: \((\d)/2\))?")


def cells_after(cells, i, n):
    return [c.strip() for c in cells[i + 1 : i + 1 + n]]


def main(path):
    text = open(path, encoding="utf-8", errors="replace").read()
    parts = SECTION.split(text)
    models = {}
    for k in range(1, len(parts), 3):
        name, body = parts[k], parts[k + 2]
        if name == "A":
            continue
        cells = body.split("\x07")
        m = models.setdefault(name, {"delays": None, "clusters": []})
        for i, c in enumerate(cells):
            label = " ".join(c.split())
            if label == "Excess delay [ns]":
                vals = []
                for v in cells[i + 1 :]:
                    if not v.strip().isdigit():
                        break
                    vals.append(int(v))
                m["delays"] = vals
            elif label == "Power [dB]":
                n = len(m["delays"])
                row = cells_after(cells, i, n)
                taps = [j for j, v in enumerate(row) if v]
                assert taps == list(range(taps[0], taps[-1] + 1)), (name, row)
                m["clusters"].append((taps[0], [float(row[j]) for j in taps]))
    print("# TGn channel models B-F: SISO power-delay profiles (NLOS), per cluster.")
    print("# Source: IEEE 802.11-03/940r4 (Erceg et al., May 2004), Appendix C.")
    print("# Command: python3 -I tools/ref/tgn_pdp_extract.py tgn-r4.txt (textutil -convert txt)")
    print("# delays <model> <tap delays, ns>")
    print("# cluster <model> <cluster number> <first tap, 0-based> <powers, dB>")
    for name in "BCDEF":
        m = models[name]
        n = len(m["delays"])
        covered = {s + j for s, p in m["clusters"] for j in range(len(p))}
        assert covered == set(range(n)), (name, "a tap without power")
        print(f"delays {name} " + " ".join(map(str, m["delays"])))
        for k, (s, p) in enumerate(m["clusters"], 1):
            print(f"cluster {name} {k} {s} " + " ".join(f"{v:.1f}" for v in p))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
