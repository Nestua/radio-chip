"""Reference values of the complementary error function from an external implementation.

CPython's math.erfc (PSF licence) is used strictly as a black
box: we pass x and record the double it returns. Our erfc (radio_phy::channel) is derived
independently (power series below 2, continued fraction above) and must match to 1e-13
relative. erfc feeds Q(x) = erfc(x/√2)/2 and every theoretical error-rate curve.

    python3 -I tools/ref/erfc_python.py > crates/radio-phy/tests/fixtures/erfc_python.txt
"""

import math
import platform
import sys

xs = sorted(
    {round(i * 0.01, 2) for i in range(-300, 301)}
    | {round(i * 0.25, 2) for i in range(0, 105)}
    | {1.9999, 2.0, 2.0001, 0.5 * math.sqrt(2), 26.5}
)
print("# erfc reference: CPython math.erfc, black box")
print(f"# Tool: Python {platform.python_version()} ({sys.platform}), math.erfc")
print("# Command: python3 -I tools/ref/erfc_python.py")
print("# <x> <erfc(x)>  (repr: shortest string that round-trips the double)")
for x in xs:
    print(f"{x!r} {math.erfc(x)!r}")
