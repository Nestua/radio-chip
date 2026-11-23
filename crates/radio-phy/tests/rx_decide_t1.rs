//! `latest_start`, the decision's choice of T1 (the last grid position keeping 7/8 of the
//! maximum), on energies worked by hand at the search's full width (44 bits; the chain reaches
//! 32): the reference of the bench `make -C hw decide_t1`. `RxFrontFixed::decide` calls it, and
//! every committed vector regenerated without a diff when it did.

use radio_phy::rx_front_fixed::latest_start;

const TOP: i128 = 1 << 43; // the bound at W = 16: 16 · 2 · (2^19)²

#[test]
fn the_threshold_is_seven_eighths_of_the_maximum_at_full_width() {
    // keep = 2^43 − 2^40 = 7·2^40 exactly.
    let keep = 7 * (1i128 << 40);
    assert_eq!(latest_start(TOP, &[TOP, keep, keep - 1]), Some(1));
    assert_eq!(latest_start(TOP, &[TOP, keep - 1, keep]), Some(2));
    assert_eq!(latest_start(TOP, &[TOP, keep - 1, keep - 1]), Some(0));
    // Ties with the maximum: the last one.
    assert_eq!(latest_start(TOP, &[TOP, TOP, TOP]), Some(2));
}

#[test]
fn the_low_32_bits_alone_would_choose_otherwise() {
    // 7·2^40 is a multiple of 2^32: keep − 1 has its low 32 bits all ones, so a comparison of
    // the low words takes it (0xffff_ffff ≥ 0) where the full one does not.
    let keep = 7 * (1i128 << 40);
    assert_eq!((keep - 1) & 0xffff_ffff, 0xffff_ffff);
    assert_eq!(keep & 0xffff_ffff, 0);
    assert_eq!(latest_start(TOP, &[TOP, keep - 1]), Some(0));
    // And the other way: a word above keep whose low 32 bits are below keep's. emax = 2^42 +
    // 2^34 gives keep = 7·2^39 + 7·2^31, whose low word is 2^31; keep + 2^31 has a low word 0.
    let emax = (1i128 << 42) + (1 << 34);
    let keep = emax - (emax >> 3);
    assert_eq!(keep, 7 * (1i128 << 39) + 7 * (1 << 31));
    assert_eq!(keep & 0xffff_ffff, 1 << 31);
    let e = keep + (1 << 31);
    assert_eq!(e & 0xffff_ffff, 0);
    assert_eq!(latest_start(emax, &[emax, e]), Some(1));
    assert_eq!(latest_start(emax, &[emax, keep - 1]), Some(0));
}

#[test]
fn the_shift_is_by_three_and_floors() {
    // emax = 15: keep = 15 − 1 = 14 (not 15 − 15/8 rounded = 13).
    assert_eq!(latest_start(15, &[15, 14]), Some(1));
    assert_eq!(latest_start(15, &[15, 13]), Some(0));
    // emax = 16: keep = 14.
    assert_eq!(latest_start(16, &[16, 14, 13]), Some(1));
}

#[test]
fn the_deadline_cuts_the_grid() {
    // The caller hands only the positions before the deadline.
    assert_eq!(latest_start(TOP, &[TOP, TOP][..1]), Some(0));
    assert_eq!(latest_start(TOP, &[]), None);
}
