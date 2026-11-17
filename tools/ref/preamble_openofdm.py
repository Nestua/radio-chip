"""Preamble references for the 802.11a/g PPDU builder from an external tool (OpenOFDM).

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is imported unmodified and three module tables its receiver uses are printed:
- LONG_PREAMBLE: 160 time-domain samples of the long training field (the packet detector
  correlates against its last 64), in the 3-decimal format of the standard's annex;
- SHORT_PREAMBLE: one 16-sample period of the short training field, same format;
- LTS_REF: the long training sequence per subcarrier (used for channel estimation).
Nothing from it is copied into the repository (ADR 0005). Python 2.

    docker run --rm --network none -v "$PWD/target/ext/openofdm:/openofdm:ro" \\
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim \\
        python /ref/preamble_openofdm.py /openofdm/scripts \\
        > crates/radio-phy/tests/fixtures/preamble_openofdm.txt

numpy, wltrace and CommPy are imported by decode.py for code not used here; empty stand-ins.
"""

import platform
import sys
import types

for name in ["numpy", "wltrace", "commpy", "commpy.channelcoding", "commpy.channelcoding.convcode"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["wltrace"].dot11 = None
sys.modules["commpy"].channelcoding = sys.modules["commpy.channelcoding"]
sys.modules["commpy.channelcoding"].convcode = sys.modules["commpy.channelcoding.convcode"]

sys.path.insert(0, sys.argv[1])
import decode  # noqa: E402  (OpenOFDM scripts/decode.py)


def main():
    out = sys.stdout
    out.write("# 802.11a/g preamble references: OpenOFDM decode.py tables, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s\n"
              % platform.python_version())
    out.write("# Command: python tools/ref/preamble_openofdm.py <openofdm>/scripts\n")
    out.write("long %s\n" % " ".join("%r,%r" % (c.real, c.imag) for c in decode.LONG_PREAMBLE))
    out.write("short %s\n" % " ".join("%r,%r" % (c.real, c.imag) for c in decode.SHORT_PREAMBLE))
    out.write("lts_ref %s\n" % " ".join("%d:%d" % (c, decode.LTS_REF[c])
                                         for c in decode.SUBCARRIERS))


if __name__ == "__main__":
    main()
