//! G0 of `hw/rtl/rx_octets.sv` (`docs/gates/rx_octets.md`): the arithmetic the RTL uses in place
//! of the model's, shown equal to the model's on every input it can see.
//!
//! - The seed: the model searches the 127 seeds for the one whose first seven outputs are the
//!   received head ([`scrambler_seed_of`]); the RTL computes it by running the LFSR backwards
//!   seven steps, a network of XORs, and loads the state after the head (the head itself,
//!   newest bit in `x1`). Both are checked here on all 128 heads.
//! - The longest PSDU (4095 octets, below 2^12) ends on its last bit in the model; the RTL's
//!   12-bit count is shown by G3's 4095-octet frame.

use radio_phy::rx_octets::{RxOctets, SEED_BITS, SERVICE_BITS};
use radio_phy::rx_ofdm::scrambler_seed_of;
use radio_phy::scrambler::Scrambler;

/// The RTL's seed: state bit i = x(i+1) = y(−(i+1)) of y(n) = y(n−7) ⊕ y(n−4), from y(0…6) = h.
fn rtl_seed(h: [u8; 7]) -> u8 {
    let m1 = h[6] ^ h[2];
    let m2 = h[5] ^ h[1];
    let m3 = h[4] ^ h[0];
    let m4 = h[3] ^ m1;
    let m5 = h[2] ^ m2;
    let m6 = h[1] ^ m3;
    let m7 = h[0] ^ m4;
    [m1, m2, m3, m4, m5, m6, m7]
        .iter()
        .enumerate()
        .fold(0, |a, (i, &b)| a | (b << i))
}

/// The RTL's state after the head: x1 the newest bit.
fn rtl_state(h: [u8; 7]) -> u8 {
    (0..7).fold(0, |a, i| a | (h[6 - i] << i))
}

fn head(v: u8) -> [u8; 7] {
    std::array::from_fn(|i| (v >> i) & 1)
}

#[test]
fn the_seed_by_xors_is_the_models_on_every_head() {
    for v in 0u8..128 {
        let h = head(v);
        match scrambler_seed_of(&h) {
            None => assert_eq!(v, 0, "no seed only for the all-zero head"),
            Some(s) => {
                assert_eq!(rtl_seed(h), s, "head {v:07b}");
                // The state after the head is the head, newest in x1: the next output matches.
                let mut sc = Scrambler::new(s).unwrap();
                sc.scramble(&h);
                let mut from_state = Scrambler::new(rtl_state(h)).unwrap();
                for k in 0..127 {
                    assert_eq!(
                        sc.next_bit(),
                        from_state.next_bit(),
                        "head {v:07b}, bit {k}"
                    );
                }
            }
        }
    }
    // The all-zero head: the XORs give 0, which the RTL reads as "no seed".
    assert_eq!(rtl_seed([0; 7]), 0);
}

#[test]
fn the_longest_psdu_fits_twelve_bits_and_ends_on_its_last_bit() {
    const MAX_LENGTH: usize = 4095;
    const { assert!(MAX_LENGTH < 1 << 12) };
    // The bit within the octet: 3 bits; SERVICE's bits: 4 bits.
    assert_eq!(SERVICE_BITS, 16);
    const { assert!(SEED_BITS < SERVICE_BITS) };
    // The longest frame ends on its last octet's last bit.
    let mut o = RxOctets::new(MAX_LENGTH);
    let mut out = Vec::new();
    let n = SERVICE_BITS + 8 * MAX_LENGTH;
    for k in 0..n {
        assert_eq!(o.bit(1, &mut out), k + 1 < n, "bit {k}");
    }
    assert_eq!(out.len(), MAX_LENGTH + 1);
}
