"""Reference table for the 802.11a/g subcarrier mapper from an external tool (OpenOFDM).

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is run unmodified, as a black box: every point of the integer constellation
grid is handed to Demodulator.demodulate and the bits it returns are recorded. Nothing from it is
copied into the repository (ADR 0005). decode.py is Python 2, so this harness is too.

    git clone https://github.com/jhshi/openofdm target/ext/openofdm
    git -C target/ext/openofdm checkout ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859
    docker run --rm --platform linux/amd64 -v "$PWD/target/ext/openofdm:/openofdm:ro" \
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim sh -c \
        'pip install -q numpy==1.16.6 && python /ref/mapper_openofdm.py /openofdm/scripts' \
        > crates/radio-phy/tests/fixtures/mapper_openofdm.txt

The demodulator needs real numpy (argmin over the constellation); numpy 1.16.6 has an x86_64
wheel for Python 2.7, hence linux/amd64. wltrace and CommPy are imported by decode.py for code we
do not call; the harness registers empty stand-ins for them.

OpenOFDM works on the unnormalised grid (it multiplies the received value by 1, 1, 3 or 7 and
compares with odd integers), so this table checks bit labels and Gray order, not K_MOD.
"""

import platform
import sys
import types

for name in ["wltrace", "commpy", "commpy.channelcoding", "commpy.channelcoding.convcode"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["wltrace"].dot11 = None
sys.modules["commpy"].channelcoding = sys.modules["commpy.channelcoding"]
sys.modules["commpy.channelcoding"].convcode = sys.modules["commpy.channelcoding.convcode"]

sys.path.insert(0, sys.argv[1])
import decode  # noqa: E402  (OpenOFDM scripts/decode.py)
import numpy  # noqa: E402

# (rate in Mb/s selecting the modulation, N_BPSC, odd levels per axis, has Q axis)
MODULATIONS = [(6, 1, [-1, 1], False), (12, 2, [-1, 1], True),
               (24, 4, [-3, -1, 1, 3], True), (48, 6, [-7, -5, -3, -1, 1, 3, 5, 7], True)]


def main():
    out = sys.stdout
    out.write("# 802.11a/g mapper reference: OpenOFDM Demodulator.demodulate, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s, numpy %s\n"
              % (platform.python_version(), numpy.__version__))
    out.write("# Command: python tools/ref/mapper_openofdm.py <openofdm>/scripts\n")
    out.write("# <n_bpsc> <I> <Q> <bits b0..b(n-1)>  (unnormalised integer grid)\n")
    for rate, n_bpsc, levels, has_q in MODULATIONS:
        demod = decode.Demodulator(rate=rate)
        for i in levels:
            for q in (levels if has_q else [0]):
                # The demodulator scales its input by `scale`; hand it the grid point / scale.
                point = complex(float(i) / demod.scale, float(q) / demod.scale)
                bits = demod.demodulate([point])
                out.write("%d %d %d %s\n" % (n_bpsc, i, q, "".join(str(b) for b in bits)))


if __name__ == "__main__":
    main()
