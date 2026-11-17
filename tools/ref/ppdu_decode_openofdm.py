"""OpenOFDM decodes our whole PPDUs (black box) for the R0-06 gate.

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0) is an FPGA 802.11 receiver. Its Python decoder
(scripts/decode.py) is run unmodified on an int16 IQ file holding our frames: Decoder.decode_next
detects each packet (power trigger, then correlation with its long-preamble table), estimates
frequency offset and channel from our preamble, decodes SIGNAL, demodulates, deinterleaves,
depunctures and Viterbi-decodes DATA (with the CommPy copy bundled in its scripts/), descrambles
and returns the PSDU octets. Nothing from it is copied into the repository (ADR 0005). Python 2.

    cargo run -q --release -p radio-tools --bin rvec -- ppdu-frames > target/ext/ppdu_frames.txt
    docker run --rm --platform linux/amd64 -v "$PWD/target/ext:/ext:ro" \\
        -v "$PWD/tools/ref:/ref:ro" python:2.7-slim sh -c \\
        'pip install -q numpy==1.16.6 && python /ref/ppdu_decode_openofdm.py \\
         /ext/openofdm/scripts /ext/ppdu_frames.txt' \\
        > crates/radio-phy/tests/fixtures/ppdu_decode_openofdm.txt

Stand-ins: matplotlib (imported by the bundled CommPy for plotting only) and wltrace's dot11
parser (applied to the decoded octets after decoding; we compare the octets themselves).
Each frame is preceded by 437 zero samples (OpenOFDM's detector needs the packet not to start
on a read-window boundary) and followed by 400; samples are scaled to int16 by 20000.
"""

import array
import os
import platform
import sys
import types
from StringIO import StringIO

for name in ["matplotlib", "matplotlib.pyplot", "matplotlib.collections", "matplotlib.patches",
             "wltrace"]:
    sys.modules[name] = types.ModuleType(name)
for sub in ["pyplot", "collections", "patches"]:
    setattr(sys.modules["matplotlib"], sub, sys.modules["matplotlib." + sub])
sys.modules["matplotlib.collections"].PatchCollection = object
dot11 = types.ModuleType("wltrace.dot11")
dot11.Dot11Packet = lambda fh: None
sys.modules["wltrace"].dot11 = dot11
sys.modules["wltrace.dot11"] = dot11

sys.path.insert(0, sys.argv[1])
import decode  # noqa: E402  (OpenOFDM scripts/decode.py)
import numpy  # noqa: E402

SCALE = 20000.0
LEAD, TRAIL = 437, 400


def main():
    frames = [l.split() for l in open(sys.argv[2]) if l.strip() and not l.startswith("#")]
    out = sys.stdout
    out.write("# OpenOFDM Decoder.decode_next on rvec ppdu-frames, black box.\n")
    out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s, numpy %s\n"
              % (platform.python_version(), numpy.__version__))
    out.write("# <Mb/s> <seed> <psdu hex> <frame hash> | rate <r> length <n> parity <ok> "
              "decoded <hex>\n")
    path = "/tmp/frame.bin"
    for f in frames:
        mbps, seed, psdu_hex, frac, fhash = f[:5]
        iq = array.array("h", [0] * (2 * LEAD))
        for pair in f[5:]:
            re, im = pair.split(",")
            iq.append(int(round(int(re) * SCALE / 2 ** int(frac))))
            iq.append(int(round(int(im) * SCALE / 2 ** int(frac))))
        iq.extend([0] * (2 * TRAIL))
        with open(path, "wb") as fh:
            iq.tofile(fh)
        dec = decode.Decoder(path)
        captured, sys.stdout = sys.stdout, StringIO()
        try:
            result = dec.decode_next()
            verdict = None
            if result is not None and len(result) > 1:
                signal, data_bytes = result[1], result[7]
                verdict = "rate %d length %d parity %d decoded %s" % (
                    signal.rate, signal.length, int(signal.parity_ok),
                    "".join("%02x" % b for b in data_bytes))
        except Exception as e:  # report, do not hide: the Rust test fails on it
            verdict = "error %s" % type(e).__name__
        finally:
            sys.stdout = captured
        out.write("%s %s %s %s | %s\n" % (mbps, seed, psdu_hex, fhash, verdict or "error none"))
    os.remove(path)


if __name__ == "__main__":
    main()
