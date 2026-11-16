# ADR 0004 — RTL is ASIC-portable from the first line

Status: accepted · 2026-11-16

## Decision

`hw/rtl/` contains only synthesisable SystemVerilog that a standard-cell flow can take unchanged:

- no vendor primitives, no vendor IP (FFT, FIR, NCO/DDS, FIFOs are written here);
- memories and clock gating only through wrappers in `hw/rtl/lib/`, with FPGA and ASIC
  implementations behind them;
- one clock domain per block unless an ADR says otherwise; synchronous active-low reset;
- AXI and other SoC buses only at the boundary of the core, never inside DSP blocks;
- FPGA-specific code (clocking, I/O, transceiver interfaces) only in `hw/fpga/`.

Blocks get ASIC probes in OpenROAD (ASAP7, then the IHP open PDK) during R2, recorded in their
gate journals, so that architecture decisions account for area, timing and power.

## Consequences

The FPGA build may be less efficient than vendor-IP designs. That is the price of reaching silicon
without a rewrite.
