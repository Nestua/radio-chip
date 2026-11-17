"""BER of the 802.11 K=7 code under soft-decision Viterbi decoding, from an external decoder.

CommPy (scikit-commpy, BSD-3-Clause) is used strictly as a black box: its `conv_encode` and
`viterbi_decode(decoding_type='unquantized')` produce the decoded bits; we only count errors.
Nothing from it is copied into the repository (ADR 0005).

    python3 -m venv /tmp/commpy && /tmp/commpy/bin/pip install scikit-commpy==0.8.0
    # one process per (point, chunk) — the points are slow in pure Python:
    for p in 0..5, c in 0..9: /tmp/commpy/bin/python -I tools/ref/viterbi_commpy.py run p c > part_p_c
    /tmp/commpy/bin/python -I tools/ref/viterbi_commpy.py combine part_* \
        > crates/radio-phy/tests/fixtures/viterbi_ber_commpy.txt

Setup (mirrored by crates/radio-phy/tests/g1_viterbi.rs, with its own noise):
- blocks of 1002 random information bits + 6 tail zeros (CommPy termination='term'),
  punctured with the 802.11 patterns, BPSK over AWGN with σ² = 1/(2·R·Eb/N0), R the code
  rate, stolen positions re-inserted as 0 (erasures);
- CommPy's 'unquantized' metric is the Euclidean distance to 2·bit − 1, so bit 1 ↦ +1 here;
- traceback depth 120 (CommPy's default 30 is short for punctured codes; 120 is ~ML);
- per block: errors over the 1002 information bits. The fixture keeps the mean and the standard
  error of the per-block BER, so bursty Viterbi errors are judged honestly.
Generators (155, 117)_8 are 802.11's (133, 171)_8 in CommPy's LSB-is-input convention
(probed in tools/ref/convenc_commpy.py).
"""

import importlib.metadata
import sys

import numpy as np
from commpy.channelcoding import Trellis, conv_encode, viterbi_decode

PATTERNS = {"1/2": [1, 1], "2/3": [1, 1, 1, 0], "3/4": [1, 1, 1, 0, 0, 1]}
RATE = {"1/2": 0.5, "2/3": 2 / 3, "3/4": 0.75}
# Points where the BER is 1e-3…1e-2, so a thousand blocks hold hundreds of error events.
POINTS = [("1/2", 1.5), ("1/2", 2.5), ("2/3", 2.5), ("2/3", 3.5), ("3/4", 3.0), ("3/4", 4.0)]
BLOCK_BITS = 1002  # + 6 tail = 1008 bits: whole periods of both puncturing patterns
CHUNKS = 10
BLOCKS_PER_CHUNK = 100
TB_DEPTH = 120


def run(idx, chunk):
    rate, ebn0_db = POINTS[idx]
    trellis = Trellis(np.array([6]), np.array([[0o155, 0o117]]))
    # One generator per (point, chunk): chunks run in parallel with reproducible results.
    rng = np.random.default_rng([0xC0DE_0010, idx, chunk])
    pattern = np.array(PATTERNS[rate], dtype=bool)
    sigma = np.sqrt(1.0 / (2.0 * RATE[rate] * 10 ** (ebn0_db / 10)))
    errors = []
    for _ in range(BLOCKS_PER_CHUNK):
        msg = rng.integers(0, 2, BLOCK_BITS)
        coded = conv_encode(msg, trellis, termination="term")
        keep = np.resize(pattern, coded.size)
        assert coded.size % pattern.size == 0
        tx = 2.0 * coded[keep] - 1.0
        rx = tx + sigma * rng.standard_normal(tx.size)
        soft = np.zeros(coded.size)
        soft[keep] = rx
        dec = viterbi_decode(soft, trellis, tb_depth=TB_DEPTH, decoding_type="unquantized")
        errors.append(int(np.count_nonzero(dec[:BLOCK_BITS] != msg)))
    print(idx, chunk, " ".join(map(str, errors)))


def combine(paths):
    per_point = {}
    for path in paths:
        for line in open(path):
            f = line.split()
            per_point.setdefault(int(f[0]), {})[int(f[1])] = [int(x) for x in f[2:]]
    out = sys.stdout
    out.write("# Soft-decision Viterbi BER, 802.11 K=7 (133,171)_8, CommPy black box.\n")
    out.write(f"# Tool: scikit-commpy {importlib.metadata.version('scikit-commpy')}, "
              f"numpy {importlib.metadata.version('numpy')}, python {sys.version.split()[0]}\n")
    out.write("# Command: python -I tools/ref/viterbi_commpy.py run|combine (see the script)\n")
    out.write(f"# {CHUNKS * BLOCKS_PER_CHUNK} blocks x {BLOCK_BITS} bits + 6 tail, "
              f"unquantized metric, tb_depth {TB_DEPTH}\n")
    out.write("# <rate> <Eb/N0 dB> <blocks> <bits per block> <mean BER> <standard error> <bit errors>\n")
    for idx, (rate, ebn0_db) in enumerate(POINTS):
        chunks = per_point[idx]
        assert sorted(chunks) == list(range(CHUNKS)), (idx, sorted(chunks))
        errs = np.array([e for c in range(CHUNKS) for e in chunks[c]])
        bers = errs / BLOCK_BITS
        se = bers.std(ddof=1) / np.sqrt(bers.size)
        out.write(f"{rate} {ebn0_db} {bers.size} {BLOCK_BITS} {bers.mean():.6e} {se:.6e} {errs.sum()}\n")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "run" and len(sys.argv) == 4:
        run(int(sys.argv[2]), int(sys.argv[3]))
    elif len(sys.argv) >= 3 and sys.argv[1] == "combine":
        combine(sys.argv[2:])
    else:
        sys.exit(__doc__)
