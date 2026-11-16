# ADR 0001 — Open, 802.11-compatible chip with a long-range TDMA profile

Status: accepted · 2026-11-16

## Context

Nestua's drone link depends on Realtek and MediaTek Wi-Fi chips that are hard to buy; NXP is
considered on the same terms as a stopgap. Europe has no open Wi-Fi silicon. The link must reach
10 km and pass CE/RED for a serial product.

Options weighed: closed or open, 802.11-compatible or an own protocol. A closed own protocol is the
least work but cannot attract funding, has no outside reference to test against, and its silicon
would never pay back on our volumes. An open own protocol is a niche. A closed 802.11 clone is all
of the cost with none of the upside.

## Decision

Build an **open, 802.11-compatible** baseband and, later, chip. Long range comes from a **TDMA MAC
profile** on the standard OFDM PHY with 5/10/20 MHz channels — the approach MikroTik NV2 and
Ubiquiti airMAX use in production at 10+ km. One parametrised OFDM/LDPC core also serves Wi-Fi 4
→ 6/6E → 7 and HaLow.

## Consequences

- Every block can be checked against the standard and against commodity chips (strong G1 and G5).
- The project is useful to all of Europe, which is what makes grants and partners possible.
- Licensed standard-essential patents apply to products built on newer generations; we plan for
  FRAND royalties and commission a freedom-to-operate analysis before the first grant application.
- The code can never be closed later.
