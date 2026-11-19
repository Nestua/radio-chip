"""Reference for the capture tooling (R0-13) from the SigMF project's own library, as a black box.

`sigmf` (PyPI, LGPL-3.0) is installed and run, never read or copied (ADR 0005): we record what
it returns. It carries the project's JSON schema, so it judges the field names and types our
writer produces, which the CPython references (capture_python.py) cannot.

    python3 -m venv <venv> && <venv>/bin/pip install sigmf
    <venv>/bin/python -I tools/ref/capture_sigmf.py make  crates/radio-capture/tests/fixtures
    <venv>/bin/python -I tools/ref/capture_sigmf.py check crates/radio-capture/tests/fixtures

`make` writes sg.sigmf-meta/-data: a recording written by the library (ci16_le, 400 samples).
`check` gives the library's verdict on the recording our code wrote (rs.sigmf-*, committed) in
sg_verdict.txt: schema validation, the checksum, what it reads back, and that it refuses the
same recording with a member damaged (so that "valid" is shown to mean something).
"""

import copy
import hashlib
import json
import platform
import sys
from importlib.metadata import version

import numpy as np
import sigmf
from sigmf import SigMFFile, sigmffile

TOOL = f"# Tool: sigmf {version('sigmf')} (PyPI), numpy {np.__version__}, Python {platform.python_version()}"


def make(out):
    n = np.arange(400)
    iq = np.empty(800, dtype="<i2")
    iq[0::2] = (11 * n) % 4096 - 2048
    iq[1::2] = 2047 - (5 * n) % 4096
    iq.tofile(f"{out}/sg.sigmf-data")
    meta = SigMFFile(
        data_file=f"{out}/sg.sigmf-data",
        global_info={
            sigmf.DATATYPE_KEY: "ci16_le",
            sigmf.SAMPLE_RATE_KEY: 20000000,
            sigmf.AUTHOR_KEY: "capture_sigmf.py",
            sigmf.DESCRIPTION_KEY: "ramp written by the sigmf library",
            sigmf.HW_KEY: "none (synthetic)",
        },
    )
    meta.add_capture(
        0,
        metadata={
            sigmf.FREQUENCY_KEY: 5805000000,
            sigmf.DATETIME_KEY: "2026-11-19T00:00:00Z",
        },
    )
    meta.add_capture(200, metadata={sigmf.FREQUENCY_KEY: 5825000000})
    meta.add_annotation(
        40,
        120,
        metadata={
            sigmf.LABEL_KEY: "burst",
            sigmf.COMMENT_KEY: "second reference",
            sigmf.FREQ_LOWER_EDGE_KEY: 5800000000.0,
            sigmf.FREQ_UPPER_EDGE_KEY: 5810000000.0,
        },
    )
    meta.validate()
    meta.tofile(f"{out}/sg.sigmf-meta")


def refused(meta, edit):
    """True when the library's validation refuses the metadata after `edit` damaged it."""
    damaged = copy.deepcopy(meta)
    edit(damaged)
    try:
        SigMFFile(metadata=damaged).validate()
    except Exception:  # the library raises its own and jsonschema's exception types
        return True
    return False


def check(out):
    raw = open(f"{out}/rs.sigmf-meta", "rb").read()
    meta = json.loads(raw)
    handle = sigmffile.fromfile(f"{out}/rs.sigmf-meta")  # verifies core:sha512 on load
    handle.validate()
    samples = handle.read_samples(0, 4)
    capture = handle.get_captures()[0]
    annotation = handle.get_annotations()[0]

    def drop(section, key):
        def edit(m):
            target = m[section] if section == "global" else m[section][0]
            del target[key]

        return edit

    def put(section, key, value):
        def edit(m):
            target = m[section] if section == "global" else m[section][0]
            target[key] = value

        return edit

    lines = [
        "# Verdict of the SigMF project's library on the recording our code wrote (rs.sigmf-*).",
        TOOL,
        "# Command: <venv>/bin/python -I tools/ref/capture_sigmf.py check <dir>",
        f"meta_sha512 {hashlib.sha512(raw).hexdigest()}",
        "schema_valid True",
        "checksum_verified True",
        f"samples {handle.sample_count}",
        f"first_samples {' '.join(f'{int(round(v.real * 32768))} {int(round(v.imag * 32768))}' for v in samples)}",
        f"sample_rate {handle.get_global_field(sigmf.SAMPLE_RATE_KEY)!r}",
        f"captures {len(handle.get_captures())} frequency {capture[sigmf.FREQUENCY_KEY]!r}",
        f"annotation {annotation[sigmf.SAMPLE_START_KEY]} {annotation[sigmf.SAMPLE_COUNT_KEY]} {annotation[sigmf.LABEL_KEY]}",
        f"refuses_missing_datatype {refused(meta, drop('global', 'core:datatype'))}",
        f"refuses_unknown_datatype {refused(meta, put('global', 'core:datatype', 'ci17_le'))}",
        f"refuses_text_sample_rate {refused(meta, put('global', 'core:sample_rate', '20e6'))}",
        f"refuses_missing_sample_start {refused(meta, drop('captures', 'core:sample_start'))}",
        f"refuses_negative_sample_start {refused(meta, put('annotations', 'core:sample_start', -1))}",
        f"refuses_text_frequency {refused(meta, put('captures', 'core:frequency', 'x'))}",
    ]
    with open(f"{out}/sg_verdict.txt", "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    {"make": make, "check": check}[sys.argv[1]](sys.argv[2])
