# ADR 0005 — Licence and use of external code

Status: accepted (licence choice provisional) · 2026-11-16

## Decision

- The repository is licensed under **Apache-2.0** for now: permissive, with an explicit patent grant
  from contributors (the same choice as aleph's hardware). CERN-OHL v2 (permissive or weakly
  reciprocal) is the alternative for the hardware parts; the final choice is made with a lawyer
  early in R0 (the repository is public from day one, so a change of licence must happen before
  outside contributions arrive).
- **No GPL/AGPL code enters the repository.** openwifi (AGPLv3), gr-ieee802-11 (GPLv3), Linux
  wireless drivers and similar projects may be read for understanding and run as black boxes to
  produce reference vectors. Their code is never copied, translated or paraphrased line by line.
- Fixtures produced by external tools record the tool, version and command that produced them.
- Standards text is cited by clause, not reproduced beyond what a test needs (e.g. a printed
  reference sequence).

## Addendum 2026-11-19 — what of the standard's text goes into the repository

Decided by the founder; Claude's assessment of the risk, not a lawyer's opinion. The whole
approach is shown to a lawyer once before a public release or an MPW.

- **Parameters of the protocol are transcribed with the clause cited.** The LDPC parity-check
  matrices of 802.11n (R0-12) go into the model like the generator polynomials, the pilot
  tables and the training sequences before them: numbers without which an implementation
  cannot interoperate, carried by every open implementation. They are checked against an
  independent black box, as any transcription is.
- **Worked examples are not reproduced; their hashes are.** Annex I.1 (tens of pages of
  tables) is a substantial part of the document, not a parameter. The repository holds the
  SHA-512 of each table (R0-06b) so that CI can compare the model's output; the tables
  themselves stay in a local fixture extracted from a licensed copy. One exception, decided
  separately the same day: the example's input, the 100-octet PSDU of Table I-1, is in the
  repository, because no test of the example can run without it (a printed reference
  sequence a test needs, as the decision above already allows).
- Patents on what the parameters describe are not touched by this addendum (see below).

## Consequences

The repository stays clean for any licence we pick, and for commercial users of the chip.
Patents are a separate matter from copyright; see the strategy doc and the FTO analysis.
