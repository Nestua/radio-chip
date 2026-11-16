# ADR 0002 — Lean team, Claude as the main digital author, independent gates

Status: accepted · 2026-11-16

## Context

A classic team for this work costs €1–2M before the first serial module. Most of the digital work —
models, RTL, verification, drivers, the ASIC flow, documentation — can be written by Claude,
provided its output is judged by checks it cannot influence. The aleph project showed the method
works: a Rust golden model as spec, generated vectors, bit-exact RTL, FPGA, ASIC probes.

## Decision

- Team: the founder (architecture, decisions, gate acceptance), Claude (digital development), one or
  two hands-on engineers (boards, lab, field), an hourly DSP consultant for PHY review, a patent
  lawyer and a CE lab when needed.
- Money goes to the physical world: field, lab, silicon, certification, legal.
- Acceptance is by the gate ladder in `docs/testing.md`, built so that no gate depends on the
  author: outside references (G1), bit-exactness (G3), formal methods (G4), commercial chips as
  judges (G5), the field (G6).
- Independent human or agent review is reserved for risk points: PHY architecture, tape-outs,
  patents and licences, CE test plans, and the pre-PR review.

## Consequences

- Test-first is mandatory: without it, AI-written RTL is not trustworthy.
- Field testing starts early (R1) so physics does not surprise us at R3.
- Project memory must live in the repo (ROADMAP, BACKLOG, ADRs, gate journals, CLAUDE.md).
- RF/analog IC design and serial masks are out of reach of this setup; they need funding later.
