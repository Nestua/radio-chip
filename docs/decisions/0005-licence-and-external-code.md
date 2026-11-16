# ADR 0005 — Licence and use of external code

Status: accepted (licence choice provisional) · 2026-11-16

## Decision

- The repository is licensed under **Apache-2.0** for now: permissive, with an explicit patent grant
  from contributors (the same choice as aleph's hardware). CERN-OHL v2 (permissive or weakly
  reciprocal) is the alternative for the hardware parts; the final choice is made with a lawyer
  before the repository is made public.
- **No GPL/AGPL code enters the repository.** openwifi (AGPLv3), gr-ieee802-11 (GPLv3), Linux
  wireless drivers and similar projects may be read for understanding and run as black boxes to
  produce reference vectors. Their code is never copied, translated or paraphrased line by line.
- Fixtures produced by external tools record the tool, version and command that produced them.
- Standards text is cited by clause, not reproduced beyond what a test needs (e.g. a printed
  reference sequence).

## Consequences

The repository stays clean for any licence we pick, and for commercial users of the chip.
Patents are a separate matter from copyright; see the strategy doc and the FTO analysis.
