"""e021: this waveform against 802.11's, through the same channels.

Question: at the same transmit power and the same channel, does the n10
waveform (10 MHz, 256-point FFT, CP 32, block pilots, short own preamble)
reach more throughput per required C/N0 than 802.11's OFDM waveforms do?

What is compared is the waveform: numerology, pilots, channel-estimation
opportunities and preamble overhead. Everything else is held equal so the
difference is the waveform's alone: the same 5G LDPC code (802.11n's LDPC
and BCC are a separate, code-only question), the same genie-statistics
LMMSE estimator (the practical ceiling for every waveform), genie timing and
frequency, no oscillator impairments, frames of about the same air time
(~400 us, a 1500-byte video packet at 16-QAM 1/2 on 20 MHz 802.11a).

Waveforms (preamble = what is sent before the grid; a grid's pilot symbols
stand for the preamble's long training symbols, so their cost is in the grid):
  n10-p2   this repository's baseline: pilot symbols 2 and 11, 28.8 us preamble
  n10-p3   the high-Doppler variant: pilot symbols 1, 6, 11
  a20      802.11a/g (and the legacy part of n/ac) on 20 MHz: 64-point FFT,
           CP 16, 52 tones of which 4 continual pilots (+-7, +-21), two long
           training symbols at the start and nothing after them (no
           midamble exists in a/g/n/ac), 100 symbols; preamble L-STF + L-SIG
  a10      the same clocked at 10 MHz (802.11p's numerology), 50 symbols
  he10     802.11ax HE SU 20 MHz numerology clocked at 10 MHz: 256-point FFT,
           CP 32 (the 1.6 us guard interval), the 242-tone RU with its 8
           continual pilots (+-22, +-48, +-90, +-116), one HE-LTF at the
           start; preamble L-STF..HE-STF (36 us at 20 MHz, 72 us here)
  he10-ma  the same with an HE midamble after 10 data symbols (802.11ax's
           Doppler mode, midamble periodicity 10)
The 242-tone RU has three DC nulls in the standard and one here; the
difference is two tones in 242.

Metric: the C/N0 (dB-Hz) a waveform needs for 1 % FER, against its net rate
(information bits per frame over the frame's air time plus its preamble).
Es/N0 is per resource element and the IFFT is unitary, so the signal power
per sample is Es * used/fft and C/N0 = Es/N0 + 10 log10(fs * used / fft).
Same power and noise density, so a dB of C/N0 is a dB of link budget.
Run:  .venv/bin/python -I experiments/e021_wifi_waveforms.py [--profile quick|reference] [--only a20,n10-p2]
"""
import argparse
import dataclasses
import json
import math
import pathlib
import sys
import time

import numpy as np
import sionna
import torch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from harness.chain import OfdmLink                          # noqa: E402
from harness.sweep import (RESULTS_DIR, default_device, reference_esno, run_point, snr_at_fer,  # noqa: E402
                           snr_grid, sweep_kwargs, write_result)
from harness.waveform import MCS_TABLE, NUM_10MHZ, Numerology, plan_codewords  # noqa: E402

NAME = "e021_wifi_waveforms"
MCS_NAMES = ("qpsk-1/2", "16qam-1/2", "64qam-3/4")
CHANNELS = {
    "awgn": dict(channel="awgn"),
    "D100-600": dict(channel="tdl", tdl_model="D", delay_spread_s=100e-9, doppler_hz=600.0),
    "C300-600": dict(channel="tdl", tdl_model="C", delay_spread_s=300e-9, doppler_hz=600.0),
    "D100-2000": dict(channel="tdl", tdl_model="D", delay_spread_s=100e-9, doppler_hz=2000.0),
}
SPAN_DB = 16.0


def eff_index(tones: tuple[int, ...], low_guard: int, fft: int) -> tuple[int, ...]:
    """Effective-subcarrier indices of signed tone numbers (DC nulled)."""
    first = -fft // 2 + low_guard
    return tuple(t - first - (1 if t > 0 else 0) for t in tones)


A20 = Numerology(name="a20", bandwidth_hz=20e6, fft_size=64, cp_len=16, num_guard=(6, 5), dc_null=True,
                 num_ofdm_symbols=100, pilot_ofdm_symbols=(0, 1),
                 continual_pilot_subcarriers=eff_index((-21, -7, 7, 21), 6, 64))
A10 = dataclasses.replace(A20, name="a10", bandwidth_hz=10e6, num_ofdm_symbols=50)
HE10 = Numerology(name="he10", bandwidth_hz=10e6, fft_size=256, cp_len=32, num_guard=(7, 6), dc_null=True,
                  num_ofdm_symbols=14, pilot_ofdm_symbols=(0,),
                  continual_pilot_subcarriers=eff_index((-116, -90, -48, -22, 22, 48, 90, 116), 7, 256))
WAVEFORMS = {
    # name: (numerology, preamble seconds)
    "n10-p2": (NUM_10MHZ, 288 / 10e6),
    "n10-p3": (dataclasses.replace(NUM_10MHZ, name="n10-p3", pilot_ofdm_symbols=(1, 6, 11)), 288 / 10e6),
    "a20": (A20, 12e-6),
    "a10": (A10, 24e-6),
    "he10": (HE10, 72e-6),
    "he10-ma": (dataclasses.replace(HE10, name="he10-ma", pilot_ofdm_symbols=(0, 11)), 72e-6),
}


def cn0_offset_db(num: Numerology) -> float:
    used = num.num_effective_subcarriers
    return 10 * math.log10(num.bandwidth_hz * used / num.fft_size)


def net_rate_bps(num: Numerology, mcs, preamble_s: float) -> float:
    return plan_codewords(num, mcs).info_bits_per_frame / (num.frame_duration_s + preamble_s)


def sweep_to(link, grid, kw: dict, stop_fer: float = 3e-3) -> list[dict]:
    """run_sweep, but stop at the first point below stop_fer: only the 1 % crossing is
    needed (run_sweep goes on until a point has no errors in max_frames)."""
    torch.manual_seed(kw["seed"])
    np.random.seed(kw["seed"])
    pts = []
    for s in grid:
        pt = run_point(link, float(s), batch=kw["batch"], min_frame_errors=kw["min_frame_errors"],
                       max_frames=kw["max_frames"])
        pts.append(pt)
        print(f"  Es/N0 {s:6.2f} dB  FER {pt['fer']:.4f}  frames {pt['frames']:6d}  {pt['seconds']:6.1f} s", flush=True)
        if pt["fer"] < stop_fer:
            break
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=("quick", "reference"), default="quick")
    ap.add_argument("--only", default=",".join(WAVEFORMS))
    ap.add_argument("--channels", default=",".join(CHANNELS))
    args = ap.parse_args()
    dev = default_device()
    mcs_list = [m for m in MCS_TABLE if m.name in MCS_NAMES]
    t0 = time.time()
    results = {}
    for wname in args.only.split(","):
        num, pre = WAVEFORMS[wname]
        assert num.num_effective_subcarriers == {"a20": 52, "a10": 52, "he10": 242, "he10-ma": 242}.get(
            wname, num.num_effective_subcarriers), wname
        for cname in args.channels.split(","):
            key = f"{wname}|{cname}"
            results[key] = {"waveform": wname, "channel": cname, "numerology": num.as_dict(), "preamble_s": pre,
                            "cn0_offset_db": cn0_offset_db(num), "curves": {}}
            for mcs in mcs_list:
                rate = net_rate_bps(num, mcs, pre)
                print(f"{key} {mcs.name}  net {rate / 1e6:.2f} Mbit/s", flush=True)
                kw = dict(CHANNELS[cname], carrier_hz=5.8e9) if cname != "awgn" else CHANNELS[cname]
                link = OfdmLink(num, mcs, device=dev, estimator="lmmse", compile=False, **kw)
                sionna.phy.config.seed = 1
                start = reference_esno(mcs.name) - 1.5
                # only the 1 % crossing is needed: stop once the FER is resolved below 0.3 %
                pts = sweep_to(link, snr_grid(start, SPAN_DB, args.profile), sweep_kwargs(args.profile))
                c = {"points": pts, "esno_db_at_fer10": snr_at_fer(pts, 0.10), "esno_db_at_fer1": snr_at_fer(pts, 0.01)}
                s1 = c["esno_db_at_fer1"]
                c.update(net_rate_bps=rate, cn0_dbhz_at_fer1=None if s1 is None else s1 + cn0_offset_db(num))
                results[key]["curves"][mcs.name] = c
    tag = "" if args.profile == "reference" else ".quick"
    part = "" if args.only == ",".join(WAVEFORMS) and args.channels == ",".join(CHANNELS) else \
        "." + args.only.replace(",", "+") + "." + args.channels.replace(",", "+")
    out = write_result(f"{NAME}{tag}{part}", {"profile": args.profile, "device": dev, "threads": torch.get_num_threads(),
                                              "seconds": round(time.time() - t0), "configs": results})
    print(f"\nwrote {out}")
    summary(results)


def summary(results: dict):
    print(f"\n{'waveform|channel':22s} " + " ".join(f"{m:>22s}" for m in MCS_NAMES))
    print(f"{'':22s} " + " ".join(f"{'Mbit/s  C/N0@1% dB-Hz':>22s}" for _ in MCS_NAMES))
    for key, r in results.items():
        cells = []
        for m in MCS_NAMES:
            c = r["curves"][m]
            v = c["cn0_dbhz_at_fer1"]
            cells.append(f"{c['net_rate_bps'] / 1e6:6.2f} {v:8.2f}" if v is not None else
                         f"{c['net_rate_bps'] / 1e6:6.2f} {'n/a':>8s}")
        print(f"{key:22s} " + " ".join(f"{c:>22s}" for c in cells))


if __name__ == "__main__":
    if "--summary" in sys.argv:
        merged = {}
        for f in sorted(RESULTS_DIR.glob(f"{NAME}*.json")):
            merged.update(json.loads(f.read_text())["configs"])
        summary(merged)
    else:
        main()
