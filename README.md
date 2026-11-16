# nestua-radio-chip

An open European 802.11-compatible radio chip, built FPGA-first and designed for ASIC from day one.

One OFDM/LDPC core, three profiles:

- **Long-range TDMA** for drone and point-to-point links at 10+ km (first user: the Nestua
  security drone-in-a-box);
- **Wi-Fi 4 → 6/6E → 7** for anyone who builds Wi-Fi into a product;
- **HaLow (802.11ah)** for industrial IoT.

Status: **R0 — golden model.** See [`ROADMAP.md`](ROADMAP.md) for stages and
[`BACKLOG.md`](BACKLOG.md) for current work.

## How it is built

A bit-exact Rust model is the specification. Every hardware block must match it bit for bit, and
the model itself must match references we did not write: the IEEE 802.11 standard's printed
examples, external tools run as black boxes, and commercial Wi-Fi chips over the air. The full
gate ladder is in [`docs/testing.md`](docs/testing.md); decisions are in
[`docs/decisions/`](docs/decisions/).

## Quick start

Requires Rust (stable, pinned by `rust-toolchain.toml`) and Verilator ≥ 5.050.

```bash
cargo test --workspace     # golden model + G1 gates
make -C hw                 # RTL lint + G3 bit-exact gates
scripts/check.sh           # everything CI runs
```

## Licence

Apache-2.0 (provisional, see [ADR 0005](docs/decisions/0005-licence-and-external-code.md)).
