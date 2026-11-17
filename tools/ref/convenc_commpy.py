"""Reference vectors for the 802.11 convolutional encoder from an external tool (CommPy).

CommPy (scikit-commpy, BSD-3-Clause) is used strictly as a black box: we call its public API and
record what it outputs. Nothing from it is copied into the repository (ADR 0005).

Run in an isolated venv:
    python -m venv /tmp/commpy && /tmp/commpy/bin/pip install scikit-commpy==0.8.0
    /tmp/commpy/bin/python -I tools/ref/convenc_commpy.py \
        > crates/radio-phy/tests/fixtures/convolutional_commpy.txt

What CommPy independently checks: the K=7 trellis, the order of the serialised outputs
(A0 B0 A1 B1 ...) and the puncturing mechanics.

Bit order of the generators: 802.11 writes g0 = 133_8, g1 = 171_8 with the leftmost (MSB) bit on
the current input (Figure 17-8: output B taps the input and delays 1, 2, 3, 6). CommPy reads an
octal generator with the current input in the LSB: probing it with (133, 171) gives the impulse
responses A = 1101101, B = 1001111, the mirror code. So we pass the bit-reversed generators
(155, 117)_8, which is the same 802.11 code in CommPy's convention.

The puncturing patterns are inputs here (IEEE 802.11-2020 §17.3.5.6, Figure 17-9 as we recall
it), so for rates 2/3 and 3/4 CommPy checks only the stealing mechanics; the patterns themselves
still need confirming against the standard (see the gate journal).
"""

import importlib.metadata
import sys

import numpy as np
from commpy.channelcoding import Trellis, conv_encode

# 802.11 g0 = 133_8 and g1 = 171_8, bit-reversed into CommPy's LSB-is-input convention.
G0_COMMPY = 0o155
G1_COMMPY = 0o117

# Pattern over the serialised stream A0 B0 A1 B1 ...; 1 = transmitted, 0 = stolen.
PATTERNS = {
    "1/2": [1, 1],
    "2/3": [1, 1, 1, 0],          # A0 B0 A1 (B1 stolen)
    "3/4": [1, 1, 1, 0, 0, 1],    # A0 B0 A1 B2 (B1, A2 stolen)
}
# Data bits per OFDM symbol for the eight 802.11a/g rates; all are multiples of 6, so every
# vector ends on a whole puncturing period.
LENGTHS = [24, 36, 48, 72, 96, 144, 192, 216]
VECTORS_PER_LENGTH = 2


class XorShift32:
    """Same generator as rvec, so inputs are reproducible without numpy's RNG."""

    def __init__(self, seed):
        self.x = seed

    def bit(self):
        x = self.x
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.x = x
        return x & 1


def bits(a):
    return "".join("1" if b else "0" for b in a)


def main():
    trellis = Trellis(np.array([6]), np.array([[G0_COMMPY, G1_COMMPY]]))
    rng = XorShift32(0xC0DE_0002)
    out = sys.stdout
    out.write("# Convolutional encoder reference, 802.11 K=7 (133,171)_8, continuous (no tail added).\n")
    out.write("# CommPy generators (155,117)_8 = the same code in CommPy's LSB-is-input convention.\n")
    out.write(f"# Tool: scikit-commpy {importlib.metadata.version('scikit-commpy')}, "
              f"numpy {importlib.metadata.version('numpy')}, "
              f"python {sys.version.split()[0]}\n")
    out.write("# Command: python -I tools/ref/convenc_commpy.py\n")
    out.write("# <rate> <input bits> <encoded bits>\n")
    for rate, pattern in PATTERNS.items():
        punct = np.array([pattern])
        for n in LENGTHS:
            for _ in range(VECTORS_PER_LENGTH):
                msg = np.array([rng.bit() for _ in range(n)])
                coded = conv_encode(msg, trellis, termination="cont", puncture_matrix=punct)
                # CommPy returns a buffer sized for rate 1/2; keep only the transmitted bits.
                kept = 2 * n * sum(pattern) // len(pattern)
                out.write(f"{rate} {bits(msg)} {bits(coded[:kept])}\n")


if __name__ == "__main__":
    main()
