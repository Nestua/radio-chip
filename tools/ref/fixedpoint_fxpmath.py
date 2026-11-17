"""Reference quantisation table for the fixed-point framework from an external tool (fxpmath).

fxpmath (github.com/francof2a/fxpmath, MIT) is used strictly as a black box: we hand it real
values and Q formats and record the raw integers it produces. Nothing from it is copied into the
repository (ADR 0005).

    python -m venv /tmp/fxp && /tmp/fxp/bin/pip install fxpmath==0.4.10
    /tmp/fxp/bin/python -I tools/ref/fixedpoint_fxpmath.py \
        > crates/radio-phy/tests/fixtures/fixedpoint_fxpmath.txt

Mapping of our modes to fxpmath (probed, fxpmath 0.4.10):
- floor     -> rounding='floor'  (toward -inf; fxpmath's 'trunc' is toward zero, not used)
- half_even -> rounding='around' (numpy.around: ties to even)
- half_up   -> rounding='floor' applied to x + 2^-(F+1). The test values are dyadic
               (k / 2^(F+6)), so that addition is exact in binary floating point.
- saturate / wrap -> overflow='saturate' / 'wrap'
"""

import importlib.metadata
import sys

from fxpmath import Fxp

# (word bits W, fractional bits F), signed two's complement.
FORMATS = [(4, 3), (8, 7), (8, 4), (12, 0), (16, 15), (16, 10), (18, 17), (24, 20), (6, 8)]
VALUES_PER_FORMAT = 40


class XorShift32:
    """Same generator as rvec, so inputs are reproducible."""

    def __init__(self, seed):
        self.x = seed

    def next(self):
        x = self.x
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.x = x
        return x


def quantise(value, w, f, rounding, overflow):
    if rounding == "half_up":
        value = value + 2.0 ** -(f + 1)
        rounding = "floor"
    fx_round = {"floor": "floor", "half_even": "around"}[rounding]
    return int(Fxp(value, signed=True, n_word=w, n_frac=f, rounding=fx_round,
                   overflow=overflow).val)


def main():
    out = sys.stdout
    out.write("# Fixed-point quantisation reference: fxpmath, black box.\n")
    out.write(f"# Tool: fxpmath {importlib.metadata.version('fxpmath')}, "
              f"numpy {importlib.metadata.version('numpy')}, python {sys.version.split()[0]}\n")
    out.write("# Command: python -I tools/ref/fixedpoint_fxpmath.py\n")
    out.write("# <value> <W> <F> <rounding> <overflow> <raw>\n")
    rng = XorShift32(0xC0DE0011)
    for w, f in FORMATS:
        # Values span about 1.5x the representable range so both overflow modes are exercised,
        # with resolution 2^-(F+6) so ties (x.5 LSB) occur often.
        scale = 2 ** (f + 6)
        span = int(1.5 * 2 ** (w - 1 - f) * scale)
        values = [0.0, 2.0 ** -(f + 1), -(2.0 ** -(f + 1)), 1.5 * 2.0 ** -f, -1.5 * 2.0 ** -f]
        while len(values) < VALUES_PER_FORMAT:
            k = rng.next() % (2 * span + 1) - span
            values.append(k / scale)
        for v in values:
            for rounding in ["floor", "half_up", "half_even"]:
                for overflow in ["saturate", "wrap"]:
                    raw = quantise(v, w, f, rounding, overflow)
                    out.write(f"{v!r} {w} {f} {rounding} {overflow} {raw}\n")


if __name__ == "__main__":
    main()
