"""OpenOFDM's receiver decodes our OFDM symbols (black box) for the R0-05 gate.

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is run unmodified: ChannelEstimator.next_symbol takes 80-sample symbols,
drops the cyclic prefix, runs numpy's FFT, removes the pilot polarity, estimates the residual
phase from the pilots (printed as "[PILOT OFFSET] beta") and returns the 48 data carriers.
We set the estimator's state directly (flat channel: gain 1 on every subcarrier, no frequency
offset) instead of running its preamble path, which needs a whole packet. Nothing from it is
copied into the repository (ADR 0005). Python 2.

    cargo run -q --release -p radio-tools --bin rvec -- ofdm-symbols > target/ext/ofdm_tx.txt
    docker run --rm --platform linux/amd64 -v "$PWD/target/ext:/ext:ro" \\
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim sh -c \\
        'pip install -q numpy==1.16.6 && python /ref/ofdm_rx_openofdm.py /ext/openofdm/scripts \\
         /ext/ofdm_tx.txt' > crates/radio-phy/tests/fixtures/ofdm_rx_openofdm.txt

Output, one line per symbol: the input line from rvec, then `beta <rad>` and the 48 decoded
carriers as `re,im` floats. numpy is real here (the FFT); wltrace and CommPy are stand-ins.
"""

import itertools
import platform
import sys
import types
from StringIO import StringIO

for name in ["wltrace", "commpy", "commpy.channelcoding", "commpy.channelcoding.convcode"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["wltrace"].dot11 = None
sys.modules["commpy"].channelcoding = sys.modules["commpy.channelcoding"]
sys.modules["commpy.channelcoding"].convcode = sys.modules["commpy.channelcoding.convcode"]

sys.path.insert(0, sys.argv[1])
import decode  # noqa: E402  (OpenOFDM scripts/decode.py)
import numpy  # noqa: E402


def receiver(samples):
    est = decode.ChannelEstimator.__new__(decode.ChannelEstimator)
    est.ht = False
    est.short_gi = False
    est.subcarriers = decode.SUBCARRIERS
    est.fft_mapping = decode.FFT_MAPPING
    est.gain = dict((c, complex(1, 0)) for c in decode.SUBCARRIERS)
    est.polarity = itertools.cycle(decode.polarity)
    est.idx = 0
    est.data_samples = samples
    return est


def main():
    lines = [l.split() for l in open(sys.argv[2]) if l.strip() and not l.startswith("#")]
    out = sys.stdout
    out.write("# OpenOFDM ChannelEstimator.next_symbol on rvec ofdm-symbols, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s, numpy %s\n"
              % (platform.python_version(), numpy.__version__))
    out.write("# <n_bpsc> <pol idx> <frac> <48 points> <80 samples> beta <rad> <48 carriers>\n")
    groups = itertools.groupby(lines, key=lambda f: f[0])
    for _, group in groups:
        group = list(group)
        samples = []
        for f in group:
            frac = int(f[2])
            for pair in f[3 + 48:3 + 48 + 80]:
                re, im = pair.split(",")
                samples.append(complex(int(re), int(im)) / float(2 ** frac))
        est = receiver(samples)
        for f in group:
            captured, sys.stdout = sys.stdout, StringIO()
            try:
                carriers = est.next_symbol()
                printed = sys.stdout.getvalue()
            finally:
                sys.stdout = captured
            beta = float(printed.split("[PILOT OFFSET]")[1].split()[0])
            out.write("%s beta %.9f %s\n" % (" ".join(f), beta,
                      " ".join("%.9f,%.9f" % (c.real, c.imag) for c in carriers)))


if __name__ == "__main__":
    main()
