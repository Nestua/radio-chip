"""OpenOFDM's receiver front as a black box on impaired frames, for the R0-08a gate.

OpenOFDM (github.com/jhshi/openofdm, Apache-2.0, commit ea638eb) is an FPGA 802.11 receiver;
its Python decoder (scripts/decode.py) is run unmodified. Per frame we record what it printed
and returned: the packet start it found (power trigger, then the two strongest correlation
peaks with the long training symbol, 64 apart), its frequency offset (lag 16 over STF samples
80..160; its fine estimate is disabled in this commit), the SIGNAL field and the PSDU.
Nothing from it is copied into the repository (ADR 0005). Python 2.

    cargo run -q --release -p radio-tools --bin rxfront -- frames 200 > target/ext/rxbb/frames.txt
    # one process per chunk (pure-Python Viterbi is slow), on the build server:
    docker run --rm --platform linux/amd64 -v "$PWD/target/ext:/ext" -v "$PWD/tools/ref:/ref:ro" \\
        python:2.7-slim sh -c 'pip install -q numpy==1.16.6 && \\
        python /ref/rx_front_openofdm.py /ext/openofdm/scripts /ext/rxbb/frames.txt <chunk> <chunks>' \\
        > part_<chunk>
    sort -n part_* (after the header) > crates/radio-phy/tests/fixtures/rx_front_openofdm.txt

Its input file is wrapped to raise at the end (decode_next never checks for EOF and would spin
forever on a frame it could not find). Stand-ins as in ppdu_decode_openofdm.py (matplotlib for the bundled CommPy's plotting, wltrace's
dot11 parser applied after decoding). Frames are int16 at x20000 with 437 leading and 400
trailing zero samples; noise covers only the frame, so OpenOFDM's absolute power trigger (200)
fires at the frame: a genie gate in its favour.
"""

import array
import os
import platform
import re
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


class EofFile(object):
    """Its input file, raising at the end: decode_next loops forever on an exhausted file when
    no packet was found (no EOF check in this commit). The algorithm is untouched."""

    def __init__(self, fh):
        self.fh = fh

    def read(self, n):
        data = self.fh.read(n)
        if not data:
            raise EOFError("end of frame file")
        return data


def main():
    chunk, chunks = int(sys.argv[3]), int(sys.argv[4])
    out = sys.stdout
    if chunk == 0:
        out.write("# OpenOFDM Decoder.decode_next on rxfront frames, black box.\n")
        out.write("# Tool: openofdm ea638eb2d3abd6c6d9c0c87a430f393fc3ffc859, python %s, "
                  "numpy %s\n" % (platform.python_version(), numpy.__version__))
        out.write("# <k> | start <sample or none> phase <rad/sample or none> "
                  "signal <rate> <length> <parity> | psdu <hex or none>\n")
    path = "/tmp/rxbb_%d.bin" % chunk
    for line in open(sys.argv[2]):
        if not line.strip() or line.startswith("#"):
            continue
        f = line.split()
        k = int(f[0])
        if k % chunks != chunk:
            continue
        iq = array.array("h")
        for pair in f[3:]:
            re_, im = pair.split(",")
            iq.append(int(re_))
            iq.append(int(im))
        with open(path, "wb") as fh:
            iq.tofile(fh)
        dec = decode.Decoder(path)
        dec.fh = EofFile(dec.fh)
        captured, sys.stdout = sys.stdout, StringIO()
        signal_s, psdu_s = "none", "none"
        try:
            result = dec.decode_next()
            if result is not None and len(result) > 7:
                sig = result[1]
                signal_s = "%d %d %d" % (sig.rate, sig.length, int(sig.parity_ok))
                psdu_s = "".join("%02x" % b for b in result[7])
        except Exception:  # no packet (EOFError), or decode() returned None on a bad SIGNAL;
            pass            # what it printed is parsed below
        finally:
            log = sys.stdout.getvalue()
            sys.stdout = captured
        m = re.search(r"\[SIGNAL\] \{([^}]*)\}", log)
        if m and signal_s == "none":
            d = dict(re.findall(r"'(\w+)': ([^,]+)", m.group(1)))
            signal_s = "%s %s %d" % (d.get("rate", "0"), d.get("length", "0"),
                                     int(d.get("parity_ok", "False") == "True"))
        start = re.search(r"Decoding packet starting from sample (\d+)", log)
        phase = re.search(r"\[COARSE OFFSET\] (-?[0-9.e+-]+)", log)
        out.write("%d | start %s phase %s signal %s | psdu %s\n" % (
            k, start.group(1) if start else "none", phase.group(1) if phase else "none",
            signal_s, psdu_s))
        out.flush()
    os.remove(path)


if __name__ == "__main__":
    main()
