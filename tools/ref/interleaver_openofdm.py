"""Reference vectors for the 802.11a/g interleaver from an external tool (OpenOFDM).

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is run unmodified, as a black box: we call Decoder.deinterleave on random
interleaved bits and record what it returns. Nothing from it is copied into the repository
(ADR 0005). decode.py is Python 2, so this harness is too.

    git clone https://github.com/jhshi/openofdm target/ext/openofdm
    git -C target/ext/openofdm checkout ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859
    docker run --rm --network none -v "$PWD/target/ext/openofdm:/openofdm:ro" \
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim \
        python /ref/interleaver_openofdm.py /openofdm/scripts \
        > crates/radio-phy/tests/fixtures/interleaver_openofdm.txt

decode.py imports numpy, wltrace and CommPy for code we do not call (FFT, frame parsing, Viterbi);
Decoder.deinterleave is plain Python. The harness registers empty stand-ins for those modules so
the import succeeds without them (no pip, no network in the container).
"""

import platform
import sys
import types

# Empty stand-ins for imports the deinterleaver does not use.
for name in ["numpy", "wltrace", "commpy", "commpy.channelcoding", "commpy.channelcoding.convcode"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["wltrace"].dot11 = None
sys.modules["commpy"].channelcoding = sys.modules["commpy.channelcoding"]
sys.modules["commpy.channelcoding"].convcode = sys.modules["commpy.channelcoding.convcode"]

sys.path.insert(0, sys.argv[1])
import decode  # noqa: E402  (OpenOFDM scripts/decode.py)

SYMBOLS_PER_VECTOR = 3
VECTORS_PER_RATE = 2


class XorShift32(object):
    """Same generator as rvec, so inputs are reproducible."""

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
    out = sys.stdout
    out.write("# 802.11a/g interleaver reference: OpenOFDM Decoder.deinterleave, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s\n"
              % platform.python_version())
    out.write("# Command: python tools/ref/interleaver_openofdm.py <openofdm>/scripts\n")
    out.write("# <rate Mb/s> <n_bpsc> <n_cbps> <interleaved bits> <deinterleaved bits>\n")
    rng = XorShift32(0xC0DE0003)
    dec = decode.Decoder(None)
    for rate in sorted(decode.RATE_PARAMETERS):
        n_bpsc, n_cbps, _ = decode.RATE_PARAMETERS[rate]
        for _ in range(VECTORS_PER_RATE):
            rx = [rng.bit() for _ in range(SYMBOLS_PER_VECTOR * n_cbps)]
            tx = dec.deinterleave(rx, rate=rate)
            out.write("%d %d %d %s %s\n" % (rate, n_bpsc, n_cbps, bits(rx), bits(tx)))


if __name__ == "__main__":
    main()
