"""OFDM constants for the 802.11a/g modulator from an external tool (OpenOFDM).

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is imported unmodified and the module-level tables its receiver uses are
printed: the 127-entry pilot polarity sequence, the pilot subcarriers and the occupied
subcarriers. Nothing from it is copied into the repository (ADR 0005). Python 2.

    git clone https://github.com/jhshi/openofdm target/ext/openofdm
    git -C target/ext/openofdm checkout ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859
    docker run --rm --network none -v "$PWD/target/ext/openofdm:/openofdm:ro" \\
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim \\
        python /ref/ofdm_constants_openofdm.py /openofdm/scripts \\
        > crates/radio-phy/tests/fixtures/ofdm_constants_openofdm.txt

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
    out.write("# 802.11a/g OFDM constants: OpenOFDM decode.py module tables, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s\n"
              % platform.python_version())
    out.write("# Command: python tools/ref/ofdm_constants_openofdm.py <openofdm>/scripts\n")
    out.write("polarity %s\n" % " ".join(str(p) for p in decode.polarity))
    out.write("pilots %s\n" % " ".join(str(c) for c in decode.PILOT_SUBCARRIES))
    out.write("subcarriers %s\n" % " ".join(str(c) for c in decode.SUBCARRIERS))
    out.write("fft_bins %s\n" % " ".join(str(decode.FFT_MAPPING[c]) for c in decode.SUBCARRIERS))


if __name__ == "__main__":
    main()
