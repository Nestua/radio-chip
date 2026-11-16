//! G1 gate for the scrambler: the golden model against a reference we did not write.
//!
//! Reference: IEEE 802.11-2020 §17.3.5.5 (PLCP DATA scrambler). With the all-ones initial state the
//! standard prints the 127-bit repeating sequence reproduced below. It is the same in 802.11-2016.

use radio_phy::scrambler::Scrambler;

/// The 127-bit sequence from §17.3.5.5, all-ones initial state, transcribed in the standard's grouping.
const STD_SEQUENCE_ALL_ONES: &str =
    "00001110 11110010 11001001 00000010 00100110 00101110 10110110 \
     00001100 11010100 11100111 10110100 00101010 11111010 01010001 10111000 1111111";

fn std_bits() -> Vec<u8> {
    STD_SEQUENCE_ALL_ONES
        .chars()
        .filter(|c| !c.is_whitespace())
        .map(|c| if c == '1' { 1 } else { 0 })
        .collect()
}

#[test]
fn reference_sequence_has_127_bits() {
    assert_eq!(
        std_bits().len(),
        127,
        "transcription error in the reference itself"
    );
}

#[test]
fn all_ones_seed_reproduces_the_standard_sequence() {
    let mut s = Scrambler::new(0x7f).unwrap();
    let ours: Vec<u8> = (0..127).map(|_| s.next_bit()).collect();
    assert_eq!(ours, std_bits());
}

#[test]
fn sequence_period_is_127_for_every_seed() {
    for seed in 1u8..=0x7f {
        let mut s = Scrambler::new(seed).unwrap();
        let first: Vec<u8> = (0..127).map(|_| s.next_bit()).collect();
        let second: Vec<u8> = (0..127).map(|_| s.next_bit()).collect();
        assert_eq!(first, second, "seed {seed:#04x}: period is not 127");
    }
}

#[test]
fn scrambling_twice_with_the_same_seed_is_identity() {
    let data: Vec<u8> = (0..1000u32).map(|i| ((i * 7 + 3) % 5 % 2) as u8).collect();
    let once = Scrambler::new(0x5d).unwrap().scramble(&data);
    let twice = Scrambler::new(0x5d).unwrap().scramble(&once);
    assert_eq!(twice, data);
}

#[test]
fn zero_and_out_of_range_seeds_are_rejected() {
    assert!(Scrambler::new(0).is_err(), "all-zero state locks the LFSR");
    assert!(Scrambler::new(0x80).is_err(), "seed is 7 bits");
}
