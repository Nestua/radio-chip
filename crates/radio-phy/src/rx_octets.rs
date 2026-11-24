//! The descrambler and the octets out: the decoded DATA bits of one frame, from the first
//! SERVICE bit, to the PSDU's octets for the MAC (IEEE 802.11-2020 §17.3.5.2, §17.3.5.5).
//!
//! The model of `hw/rtl/rx_octets.sv` (gate G3: `make -C hw rx_octets`; G0 and G1:
//! `docs/gates/rx_octets.md`), and the part of [`RxDataFixed`] that turns bits into octets.
//!
//! - The first seven SERVICE bits are zero before scrambling (§17.3.5.2), so the first seven
//!   decoded bits are the scrambler's first seven outputs: they give the transmitter's seed
//!   ([`scrambler_seed_of`]). All seven zero is no seed (no non-zero state gives seven zeros):
//!   no octet is handed on and the end, [`RxDataEnd::Seed`], comes on the bit the last octet
//!   would have come on.
//! - SERVICE's 16 bits, descrambled (bits 0 to 6 are then zero by construction), and the seed
//!   come with the first octet.
//! - The octets are the PSDU's, descrambled, the first received bit in bit 0 (§17.3.5.2: the
//!   PSDU is sent LSB first). The last one carries the end [`RxDataEnd::Decoded`]; bits after
//!   it (the tail) are not read.
//!
//! [`RxDataFixed`]: crate::rx_data_fixed::RxDataFixed

use crate::rx_data_fixed::{FirstOctet, RxDataEnd, RxDataEvent};
use crate::rx_ofdm::scrambler_seed_of;
use crate::scrambler::Scrambler;

/// The SERVICE field's length in bits (§17.3.5.2).
pub const SERVICE_BITS: usize = 16;
/// The SERVICE bits that carry the scrambler's state (zero before scrambling).
pub const SEED_BITS: usize = 7;

/// The octets of one frame, a decoded bit at a time.
#[derive(Debug, Clone)]
pub struct RxOctets {
    /// The PSDU's length in octets (SIGNAL's LENGTH, 1…4095).
    length: usize,
    /// Bits taken so far.
    bits: usize,
    head: Vec<u8>,
    scrambler: Option<(u8, Scrambler)>,
    service: u16,
    octet: u8,
    octets: usize,
    /// The seed was zero: the frame is read on to its end with no octet handed on.
    zero_seed: bool,
    done: bool,
}

impl RxOctets {
    /// A frame of `length` PSDU octets.
    pub fn new(length: usize) -> Self {
        Self {
            length,
            bits: 0,
            head: Vec::with_capacity(SEED_BITS),
            scrambler: None,
            service: 0,
            octet: 0,
            octets: 0,
            zero_seed: false,
            done: false,
        }
    }

    /// Whether the frame's end has been given (later bits are ignored).
    pub fn done(&self) -> bool {
        self.done
    }

    /// One decoded bit (0/1): its events in `out`; whether the frame goes on.
    pub fn bit(&mut self, b: u8, out: &mut Vec<RxDataEvent>) -> bool {
        if self.done {
            return false;
        }
        let more = self.step(b & 1, out);
        self.done = !more;
        more
    }

    fn step(&mut self, b: u8, out: &mut Vec<RxDataEvent>) -> bool {
        let k = self.bits;
        self.bits += 1;
        if self.zero_seed {
            // No octet; the end comes where the last octet would have.
            if k >= SERVICE_BITS && (k - SERVICE_BITS) % 8 == 7 {
                self.octets += 1;
                if self.octets == self.length {
                    out.push(RxDataEvent::End(RxDataEnd::Seed));
                    return false;
                }
            }
            return true;
        }
        if self.scrambler.is_none() {
            self.head.push(b);
            if self.head.len() < SEED_BITS {
                return true;
            }
            let Some((seed, sc)) = scrambler_seed_of(&self.head)
                .and_then(|s| Scrambler::new(s).ok().map(|sc| (s, sc)))
            else {
                // Read on to the frame's end (`docs/gates/rx_data.md`, G0): the reader of the
                // RTL learns no earlier that the frame is over.
                self.zero_seed = true;
                return true;
            };
            let mut sc = sc;
            for (j, &h) in self.head.iter().enumerate() {
                let c = sc.scramble(&[h]).first().copied().unwrap_or(0);
                self.service |= u16::from(c) << j;
            }
            self.scrambler = Some((seed, sc));
            return true;
        }
        let Some((seed, sc)) = self.scrambler.as_mut() else {
            return false;
        };
        let c = sc.scramble(&[b]).first().copied().unwrap_or(0);
        if k < SERVICE_BITS {
            self.service |= u16::from(c) << k;
            return true;
        }
        let at = (k - SERVICE_BITS) % 8;
        self.octet |= c << at;
        if at < 7 {
            return true;
        }
        out.push(RxDataEvent::Octet {
            octet: self.octet,
            first: (self.octets == 0).then_some(FirstOctet {
                seed: *seed,
                service: self.service,
            }),
        });
        self.octet = 0;
        self.octets += 1;
        if self.octets == self.length {
            out.push(RxDataEvent::End(RxDataEnd::Decoded));
            return false;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feeds `bits` until the frame ends; the bit index and the events of the end.
    fn run(length: usize, bits: &[u8]) -> (usize, Vec<RxDataEvent>) {
        let mut o = RxOctets::new(length);
        let mut all = Vec::new();
        for (k, &b) in bits.iter().enumerate() {
            let mut out = Vec::new();
            let more = o.bit(b, &mut out);
            all.extend(out);
            if !more {
                return (k, all);
            }
        }
        (usize::MAX, all)
    }

    #[test]
    fn a_zero_seed_ends_where_the_last_octet_would_have_been() {
        for length in [1usize, 2, 24, 100] {
            let n = 16 + 8 * length + 6;
            // A seed found (the first seven bits not all zero): Decoded with the last octet.
            let mut bits = vec![1u8; n];
            let (at, ev) = run(length, &bits);
            assert_eq!(at, 16 + 8 * length - 1, "{length}");
            assert_eq!(ev.last(), Some(&RxDataEvent::End(RxDataEnd::Decoded)));
            assert_eq!(ev.len(), length + 1);
            // The first seven all zero: no octet, Seed on the same bit.
            bits[..7].fill(0);
            let (at, ev) = run(length, &bits);
            assert_eq!(at, 16 + 8 * length - 1, "{length}");
            assert_eq!(ev, [RxDataEvent::End(RxDataEnd::Seed)], "{length}");
        }
    }

    /// §17.3.5.2 (Figure 17-6): SERVICE's bit 0 is transmitted first. Annex I.1's SERVICE is
    /// all zero, so its G1 cannot see the order: a clear SERVICE with one bit set at a time,
    /// scrambled, must come back in that bit.
    #[test]
    fn service_bit_k_is_the_kth_bit_sent() {
        for k in 7..16 {
            let mut clear = vec![0u8; 16 + 8];
            clear[k] = 1;
            let sent = Scrambler::new(0x5d).unwrap().scramble(&clear);
            let mut o = RxOctets::new(1);
            let mut out = Vec::new();
            for &b in &sent {
                o.bit(b, &mut out);
            }
            let Some(RxDataEvent::Octet {
                first: Some(f),
                octet: 0,
            }) = out.first().copied()
            else {
                panic!("{out:?}");
            };
            assert_eq!(f.service, 1 << k, "bit {k}");
            assert_eq!(f.seed, 0x5d);
        }
    }

    #[test]
    fn bits_after_the_end_are_ignored() {
        let mut o = RxOctets::new(1);
        let mut out = Vec::new();
        for _ in 0..24 {
            o.bit(1, &mut out);
        }
        assert!(o.done());
        let n = out.len();
        assert!(!o.bit(1, &mut out));
        assert_eq!(out.len(), n);
    }
}
