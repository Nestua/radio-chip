"""Error rates of the HT LDPC codes under belief-propagation decoding, from an outside decoder.

CommPy (scikit-commpy 0.8.0, BSD-3-Clause) is used strictly as a black box: its
`ldpc_bp_decode` (sum-product 'SPA' and min-sum 'MSA', flooding schedule) decodes; we only
count errors. The code's parity-check matrix is expanded here from the outside copy of the
matrix prototypes (github.com/tavildar/LDPC, MIT, LdpcC/WiFiLDPC.h at commit 05ee7f4, read as
data; see ldpc_matrices_outside.py), so nothing of our model is on this side. Nothing of
either project is copied into the repository (ADR 0005).

    python3 -m venv <venv> && <venv>/bin/pip install scikit-commpy==0.8.0
    for p in 0..N-1, c in 0..CHUNKS-1:
        <venv>/bin/python -I tools/ref/ldpc_ber_commpy.py run WiFiLDPC.h p c > part_p_c
    <venv>/bin/python -I tools/ref/ldpc_ber_commpy.py combine part_* \
        > crates/radio-phy/tests/fixtures/ldpc_ber_commpy.txt

Setup (mirrored by crates/radio-phy/tests/g1_ldpc_decoder.rs, with its own noise):
- the all-zero codeword (the code is linear and the channel symmetric, so the error rate does
  not depend on the codeword; this side needs no encoder), BPSK over AWGN, bit 0 ↦ +1,
  σ² = 1/(2·R·Eb/N0), R the code rate; LLR = 2·y/σ² (positive = bit 0);
- at most ITERS iterations, flooding; CommPy stops a block early once its checks hold;
- per block: bit errors over all n bits and whether the block has any. The fixture keeps the
  mean bit error rate with the standard error of the per-block rate, and the block error
  rate.
A block of the prototype with shift s has its one of row r in column (r + s) mod Z
(IEEE 802.11-2020 §19.3.11.7.4).
"""

import hashlib
import importlib.metadata
import os
import re
import sys
import tempfile

import numpy as np
from commpy.channelcoding.ldpc import get_ldpc_code_params, ldpc_bp_decode

SHA256 = "b2a6e22dce37ab57c883503f7078feb16fcc8e5dc029efc7bc596c10de196934"
MATRIX = re.compile(r"int\s+H_(\d+)_(\d)_(\d)\[(\d+)\]\[(\d+)\]\s*=\s*\{(.*?)\};", re.S)
ITERS = 50
BLOCKS = 500  # per chunk
CHUNKS = 10
# (n, rate numerator, denominator, algorithm, Eb/N0 in dB)
POINTS = [
    (648, 1, 2, "SPA", 1.0),
    (648, 1, 2, "SPA", 1.5),
    (648, 1, 2, "SPA", 2.0),
    (648, 1, 2, "MSA", 1.5),
    (648, 1, 2, "MSA", 2.0),
    (648, 1, 2, "MSA", 2.5),
    (648, 5, 6, "SPA", 3.0),
    (648, 5, 6, "SPA", 3.5),
    (1296, 3, 4, "SPA", 2.0),
    (1296, 3, 4, "SPA", 2.5),
]


def design(header, n, num, den):
    """CommPy's design file for the code, from the outside prototypes; returns its path."""
    raw = open(header, "rb").read()
    assert hashlib.sha256(raw).hexdigest() == SHA256, "not the file this script was written for"
    proto = None
    for pn, pnum, pden, _, _, body in MATRIX.findall(raw.decode()):
        if (int(pn), int(pnum), int(pden)) == (n, num, den):
            proto = [[int(v) for v in row.split(",")] for row in re.findall(r"\{([^{}]*)\}", body)]
    z = n // 24
    checks = [
        sorted(col * z + (r + s) % z for col, s in enumerate(row) if s >= 0)
        for row in proto
        for r in range(z)
    ]
    bits = [[] for _ in range(n)]
    for c, check in enumerate(checks):
        for b in check:
            bits[b].append(c)
    fd, path = tempfile.mkstemp(suffix=".txt")
    with os.fdopen(fd, "w") as f:
        f.write(f"{n} {len(checks)}\n{max(map(len, bits))} {max(map(len, checks))}\n")
        # The degree lists end with a separator, as in CommPy's own design files (without it
        # the last node is not read).
        f.write("".join(f"{len(b)} " for b in bits) + "\n")
        f.write("".join(f"{len(c)} " for c in checks) + "\n")
        for b in bits:
            f.write("\t".join(str(c + 1) for c in b) + "\n")
        for c in checks:
            f.write("\t".join(str(b + 1) for b in c) + "\n")
    return path


def run(header, point, chunk):
    n, num, den, algorithm, ebn0_db = POINTS[point]
    path = design(header, n, num, den)
    params = get_ldpc_code_params(path)
    os.unlink(path)
    rng = np.random.default_rng(1_000_003 * point + chunk)
    sigma2 = 1.0 / (2.0 * num / den * 10 ** (ebn0_db / 10))
    y = 1.0 + np.sqrt(sigma2) * rng.standard_normal(n * BLOCKS)
    decoded, _ = ldpc_bp_decode(2.0 * y / sigma2, params, algorithm, ITERS)
    errors = np.asarray(decoded).reshape(n, BLOCKS).sum(axis=0)  # one column per block
    print(point, chunk, " ".join(str(int(e)) for e in errors))


def combine(parts):
    per_point = {}
    for part in parts:
        f = open(part).read().split()
        per_point.setdefault(int(f[0]), {})[int(f[1])] = [int(e) for e in f[2:]]
    print("# Error rates of HT LDPC codes under flooding belief propagation: CommPy's ldpc_bp_decode, black box.")
    print(f"# Tool: scikit-commpy {importlib.metadata.version('scikit-commpy')}, numpy {np.__version__};"
          " matrices from github.com/tavildar/LDPC at 05ee7f4.")
    print("# Command: tools/ref/ldpc_ber_commpy.py run … / combine (see the script).")
    print(f"# All-zero codeword, BPSK, AWGN, at most {ITERS} iterations; BER over all n bits.")
    print("# n rate algorithm Eb/N0[dB] blocks BER stderr(BER) block_errors")
    for point in sorted(per_point):
        n, num, den, algorithm, ebn0_db = POINTS[point]
        chunks = per_point[point]
        assert sorted(chunks) == list(range(CHUNKS)), (point, sorted(chunks))
        errors = np.array([e for c in sorted(chunks) for e in chunks[c]]) / n
        print(f"{n} {num}/{den} {algorithm} {ebn0_db} {len(errors)} {errors.mean():.6e} "
              f"{errors.std(ddof=1) / np.sqrt(len(errors)):.3e} {int((errors > 0).sum())}")


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    else:
        combine(sys.argv[2:])
