//! The deinterleaver and depuncturer's G0 (`docs/gates/rx_deint.md`): the architecture chosen for
//! `hw/rtl/rx_deint.sv` hands the Viterbi decoder the model's depunctured pairs, step for step,
//! from a soft buffer written in on-air order.
//!
//! The architecture (the founder chose two banks of one write and one read port, 2026-11-24):
//! - The symbol block's soft pass gives a data subcarrier a clock, its N_BPSC values in on-air
//!   order (lanes). Each subcarrier is one 48-bit word (six lanes of 8 bits).
//! - Coded bit k of a symbol (IEEE 802.11-2020 §17.3.5.7) lies on data subcarrier
//!   sc(k) = 3·(k mod 16) + ⌊⌊k/16⌋ / N_BPSC⌋, so its group ⌊sc/3⌋ is k mod 16 and the group's
//!   parity is k's. Bank b holds the groups of parity b: word = slot·24 + 3·⌊g/2⌋ + sc mod 3.
//!   Two consecutive coded bits are always in different banks: one read of each a clock gives
//!   any two of them.
//! - The lane: with c = k mod 16, r = ⌊k/16⌋, r' = r mod N_BPSC and s = max(N_BPSC/2, 1), the
//!   second permutation keeps the bit on its subcarrier and puts it on lane
//!   s·⌊r'/s⌋ + (r' − c) mod s.
//! - The reader walks k with counters (c, r', ⌊r/N_BPSC⌋), a trellis step a clock: the rate's
//!   pattern says which of A and B are kept (1/2: both; 2/3: both, A; 3/4: both, A, B); a kept
//!   one takes the next k, a stolen one is an erasure (0). A symbol is whole periods, so the
//!   pattern's phase is 0 at every symbol's start. At the block's last step (24 for SIGNAL, the
//!   anchor 16 + 8·LENGTH + 6 for the data) it stops; the rest of the symbol is never read.
//!
//! Each case below runs the architecture and the model (`deinterleave_soft`, `depuncture`, the pairs
//! up to the block's steps) on the same random values and asserts the pairs are equal, that the
//! two reads of a step never hit one bank, and that the phase is 0 at every symbol's start.

#![allow(clippy::unwrap_used)]

use radio_phy::channel::Rng;
use radio_phy::convolutional::CodeRate;
use radio_phy::interleaver::interleaved_index;
use radio_phy::modulation::Modulation;
use radio_phy::ppdu::Rate;
use radio_phy::rx_ofdm_fixed::deinterleave_soft;
use radio_phy::viterbi::{depuncture, SOFT_MAX};

/// Data subcarriers, lanes, words of a slot in one bank.
const DATA: usize = 48;
const LANES: usize = 6;
const SLOT_WORDS: usize = 24;

/// The RTL's buffer: two banks of 2 slots × 24 words of six lanes.
struct Buffer {
    bank: [Vec<[i32; LANES]>; 2],
}

impl Buffer {
    fn new() -> Self {
        Self {
            bank: [
                vec![[0; LANES]; 2 * SLOT_WORDS],
                vec![[0; LANES]; 2 * SLOT_WORDS],
            ],
        }
    }

    /// Subcarrier j of a symbol, written as the soft pass gives it.
    fn write(&mut self, slot: usize, j: usize, lanes: [i32; LANES]) {
        let g = j / 3;
        self.bank[g % 2][slot * SLOT_WORDS + 3 * (g / 2) + j % 3] = lanes;
    }
}

/// The reader's address of coded bit k from its counters: (bank, word in the slot, lane).
fn address(c: usize, rq: usize, rr: usize, nb: usize) -> (usize, usize, usize) {
    let s = (nb / 2).max(1);
    let lane = s * (rr / s) + (rr + s * 16 - c) % s;
    (c % 2, 3 * (c / 2) + rq, lane)
}

/// The reader's counters for coded bit k: c = k mod 16, r = ⌊k/16⌋ split into ⌊r/N⌋ and r mod N.
#[derive(Clone, Copy, Default)]
struct Walk {
    c: usize,
    rq: usize,
    rr: usize,
}

impl Walk {
    /// The next coded bit, by increments only (the RTL's counters).
    fn step(self, nb: usize) -> Self {
        if self.c < 15 {
            return Self {
                c: self.c + 1,
                ..self
            };
        }
        if self.rr + 1 < nb {
            Self {
                c: 0,
                rq: self.rq,
                rr: self.rr + 1,
            }
        } else {
            Self {
                c: 0,
                rq: self.rq + 1,
                rr: 0,
            }
        }
    }
}

/// Which of (A, B) are kept at a pattern phase; the phase's period.
fn kept(rate: CodeRate, phase: usize) -> (bool, bool) {
    let p = rate.pattern();
    (p[2 * phase] == 1, p[2 * phase + 1] == 1)
}

fn period(rate: CodeRate) -> usize {
    rate.pattern().len() / 2
}

/// The architecture: symbols (on-air order) written a subcarrier a clock into alternating slots,
/// read back as `steps` depunctured pairs. Asserts the structural facts on the way.
fn architecture(symbols: &[Vec<i32>], nb: usize, rate: CodeRate, steps: usize) -> Vec<(i32, i32)> {
    let mut buf = Buffer::new();
    let mut out = Vec::with_capacity(steps);
    let mut phase = 0;
    for (n, sym) in symbols.iter().enumerate() {
        let slot = n % 2;
        for j in 0..DATA {
            let mut lanes = [0; LANES];
            lanes[..nb].copy_from_slice(&sym[j * nb..(j + 1) * nb]);
            buf.write(slot, j, lanes);
        }
        assert_eq!(phase, 0, "the pattern's phase at symbol {n}'s start");
        let mut w = Walk::default();
        let mut taken = 0;
        let read = |w: Walk, buf: &Buffer| {
            let (b, word, lane) = address(w.c, w.rq, w.rr, nb);
            assert!(word < SLOT_WORDS && lane < nb);
            (b, buf.bank[b][slot * SLOT_WORDS + word][lane])
        };
        while taken < DATA * nb && out.len() < steps {
            let (ka, kb) = kept(rate, phase);
            let mut banks = Vec::new();
            let mut take = |w: &mut Walk, taken: &mut usize| {
                let (b, v) = read(*w, &buf);
                banks.push(b);
                *w = w.step(nb);
                *taken += 1;
                v
            };
            let a = if ka { take(&mut w, &mut taken) } else { 0 };
            let b = if kb { take(&mut w, &mut taken) } else { 0 };
            if banks.len() == 2 {
                assert_ne!(banks[0], banks[1], "two reads of one bank in a step");
            }
            out.push((a, b));
            phase = (phase + 1) % period(rate);
        }
        if out.len() == steps {
            break;
        }
        assert_eq!(taken, DATA * nb, "a symbol is whole periods");
        assert_eq!(
            (w.c, w.rq, w.rr),
            (0, 3, 0),
            "the walk ends on the symbol's end"
        );
    }
    out
}

/// The model: each symbol deinterleaved, the stream depunctured, the first `steps` pairs.
fn model(
    symbols: &[Vec<i32>],
    modulation: Modulation,
    rate: CodeRate,
    steps: usize,
) -> Vec<(i32, i32)> {
    let coded: Vec<i32> = symbols
        .iter()
        .flat_map(|sym| deinterleave_soft(sym, modulation))
        .collect();
    let full = depuncture(&coded, rate).unwrap();
    full.chunks(2).take(steps).map(|p| (p[0], p[1])).collect()
}

fn random_symbols(rng: &mut Rng, count: usize, nb: usize) -> Vec<Vec<i32>> {
    (0..count)
        .map(|_| {
            (0..DATA * nb)
                .map(|_| (rng.next_u64() % (2 * SOFT_MAX as u64 + 1)) as i32 - SOFT_MAX)
                .collect()
        })
        .collect()
}

const RATES: [Rate; 8] = [
    Rate::R6,
    Rate::R9,
    Rate::R12,
    Rate::R18,
    Rate::R24,
    Rate::R36,
    Rate::R48,
    Rate::R54,
];

/// The structural fact the banks rest on: the counters' (subcarrier, lane) of every coded bit
/// is where the standard's interleaver puts it (`interleaved_index`), and its bank is k's
/// parity — every position of every modulation.
#[test]
fn walk_is_the_interleaver() {
    for m in [
        Modulation::Bpsk,
        Modulation::Qpsk,
        Modulation::Qam16,
        Modulation::Qam64,
    ] {
        let nb = m.bits_per_subcarrier();
        let mut w = Walk::default();
        for k in 0..m.bits_per_symbol() {
            let (bank, word, lane) = address(w.c, w.rq, w.rr, nb);
            let sc = 3 * (2 * (word / 3) + bank) + word % 3;
            assert_eq!(sc * nb + lane, interleaved_index(k, m), "{m:?} k {k}");
            assert_eq!(bank, k % 2);
            w = w.step(nb);
        }
    }
}

/// SIGNAL: one BPSK symbol at rate 1/2, 24 steps.
#[test]
fn signal_block() {
    let mut rng = Rng::new(1);
    for _ in 0..200 {
        let syms = random_symbols(&mut rng, 1, 1);
        assert_eq!(
            architecture(&syms, 1, CodeRate::R1_2, 24),
            model(&syms, Modulation::Bpsk, CodeRate::R1_2, 24)
        );
    }
}

/// Data blocks of every rate, the anchor anywhere in the last symbol, one to many symbols.
#[test]
fn data_blocks() {
    let mut rng = Rng::new(2);
    let mut cases = 0;
    for rate in RATES {
        let m = rate.modulation();
        let nb = m.bits_per_subcarrier();
        let n_dbps = rate.data_bits_per_symbol();
        let mut lengths: Vec<usize> = vec![0, 1, 2, 3, 23, 24, 100, 1500, 4095];
        // Lengths whose anchor reads the fewest and the most steps of the last symbol (it is
        // 6 modulo 8, so never a symbol's exact end at any rate).
        let used = |len: usize| (16 + 8 * len + 6 - 1) % n_dbps + 1;
        let lo = (0..2 * n_dbps).map(used).min().unwrap();
        let hi = (0..2 * n_dbps).map(used).max().unwrap();
        lengths.extend((0..2 * n_dbps).filter(|&l| used(l) == lo || used(l) == hi));
        for _ in 0..40 {
            lengths.push((rng.next_u64() % 400) as usize);
        }
        for len in lengths {
            let anchor = 16 + 8 * len + 6;
            let count = anchor.div_ceil(n_dbps);
            let syms = random_symbols(&mut rng, count, nb);
            let code = rate.code_rate();
            assert_eq!(
                architecture(&syms, nb, code, anchor),
                model(&syms, m, code, anchor),
                "{rate:?} length {len}"
            );
            cases += 1;
        }
    }
    assert!(cases >= 8 * 49, "{cases} cases");
}

/// `deinterleave_soft` (the receiver's) is the interleaver's G1-gated `deinterleave` on the same
/// values: every modulation, values offset into u8.
#[test]
fn deinterleave_soft_is_the_g1_deinterleaver() {
    let mut rng = Rng::new(3);
    for m in [
        Modulation::Bpsk,
        Modulation::Qpsk,
        Modulation::Qam16,
        Modulation::Qam64,
    ] {
        let soft = random_symbols(&mut rng, 1, m.bits_per_subcarrier()).remove(0);
        let bytes: Vec<u8> = soft.iter().map(|&v| (v + 128) as u8).collect();
        let theirs: Vec<i32> = radio_phy::interleaver::deinterleave(&bytes, m)
            .unwrap()
            .iter()
            .map(|&b| i32::from(b) - 128)
            .collect();
        assert_eq!(deinterleave_soft(&soft, m), theirs, "{m:?}");
    }
}
