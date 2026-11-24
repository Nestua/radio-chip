//! `rvec rx_octets`: the descrambler and octets' vectors (`hw/rtl/rx_octets.sv`,
//! `docs/gates/rx_octets.md`), from `radio_phy::rx_octets::RxOctets`.
//!
//! The vectors are cycles at the block's pins. In each cycle the bench drives the inputs a
//! character gives and, after the clock edge, reads the outputs. The model's events of a bit
//! are expected after the edge that takes the bit (G0: the octet registered on the edge that
//! takes its last bit). Every other cycle must show no octet and no end.
//!
//! Characters: `.` nothing; `0`/`1` a bit (in_valid); `S` start (with the `L` line's length);
//! `D` drop; `s`/`t` start with a bit 0/1 in the same cycle (start wins: the bit is not
//! taken); `a`/`b` drop with a bit 0/1 (drop wins); `X` start and drop (drop wins). Bits while no frame is armed are ignored
//! (SIGNAL's bits, a frame's tail).

use std::io::{self, Write};

use radio_phy::channel::Rng;
use radio_phy::ppdu::{data_bits, Rate};
use radio_phy::rx_data_fixed::{RxDataEnd, RxDataEvent};
use radio_phy::rx_octets::RxOctets;

/// One cycle's inputs.
#[derive(Clone, Copy)]
enum Cycle {
    Idle,
    Bit(u8),
    Start,
    Drop,
    StartBit(u8),
    DropBit(u8),
    /// Start and drop in one cycle (drop wins).
    StartDrop,
}

impl Cycle {
    fn char(self) -> char {
        match self {
            Cycle::Idle => '.',
            Cycle::Bit(b) => (b'0' + b) as char,
            Cycle::Start => 'S',
            Cycle::Drop => 'D',
            Cycle::StartBit(b) => (b's' + b) as char,
            Cycle::DropBit(b) => (b'a' + b) as char,
            Cycle::StartDrop => 'X',
        }
    }
}

/// A segment: the length for its starts, its cycles; runs the model over them.
struct Segment {
    length: usize,
    cycles: Vec<Cycle>,
}

impl Segment {
    fn new(length: usize) -> Self {
        Self {
            length,
            cycles: Vec::new(),
        }
    }

    fn push(&mut self, c: Cycle) {
        self.cycles.push(c);
    }

    /// Bits with `gap` idle cycles drawn before each (0 = a bit every cycle).
    fn bits(&mut self, rng: &mut Rng, bits: &[u8], max_gap: u64) {
        for &b in bits {
            if max_gap > 0 {
                for _ in 0..rng.next_u64() % (max_gap + 1) {
                    self.push(Cycle::Idle);
                }
            }
            self.push(Cycle::Bit(b));
        }
    }

    /// The model over the cycles, the frame armed or not at the start (`frame`), written out.
    fn write(&self, out: &mut impl Write, frame: &mut Option<RxOctets>) -> io::Result<()> {
        writeln!(out, "L {:03x}", self.length)?;
        let line: String = self.cycles.iter().map(|c| c.char()).collect();
        writeln!(out, "I {line}")?;
        for (n, &c) in self.cycles.iter().enumerate() {
            let bit = match c {
                Cycle::Start | Cycle::StartBit(_) => {
                    *frame = Some(RxOctets::new(self.length));
                    None
                }
                Cycle::Drop | Cycle::DropBit(_) | Cycle::StartDrop => {
                    *frame = None;
                    None
                }
                Cycle::Bit(b) => Some(b),
                Cycle::Idle => None,
            };
            let (Some(b), Some(f)) = (bit, frame.as_mut()) else {
                continue;
            };
            let mut ev = Vec::new();
            let more = f.bit(b, &mut ev);
            let last = !more && ev.contains(&RxDataEvent::End(RxDataEnd::Decoded));
            for e in ev {
                match e {
                    RxDataEvent::Octet { octet, first } => {
                        let (seed, service) = first.map_or((0, 0), |f| (f.seed, f.service));
                        writeln!(
                            out,
                            "O {n} {octet:02x} {} {} {seed:02x} {service:04x}",
                            u8::from(first.is_some()),
                            u8::from(last)
                        )?;
                    }
                    RxDataEvent::End(RxDataEnd::Decoded) => writeln!(out, "E {n} 0")?,
                    RxDataEvent::End(RxDataEnd::Seed) => writeln!(out, "E {n} 1")?,
                    other => return Err(io::Error::other(format!("unexpected {other:?}"))),
                }
            }
            if !more {
                *frame = None;
            }
        }
        Ok(())
    }
}

fn random_bits(rng: &mut Rng, n: usize) -> Vec<u8> {
    (0..n).map(|_| (rng.next_u64() & 1) as u8).collect()
}

fn psdu(rng: &mut Rng, length: usize) -> Vec<u8> {
    (0..length).map(|_| rng.next_u64() as u8).collect()
}

/// A frame's decoded bits: SERVICE, the PSDU, the tail and pad, scrambled with `seed` at 54 Mb/s.
fn frame_bits(rng: &mut Rng, length: usize, seed: u8) -> io::Result<Vec<u8>> {
    data_bits(&psdu(rng, length), Rate::R54, seed).map_err(|e| io::Error::other(format!("{e:?}")))
}

/// SIGNAL's 24 bits before the start (ignored), the start, the frame's bits with gaps, the
/// tail's bits after the end (ignored).
fn frame(rng: &mut Rng, length: usize, bits: &[u8], max_gap: u64) -> Segment {
    let mut s = Segment::new(length);
    let signal = random_bits(rng, 24);
    s.bits(rng, &signal, max_gap);
    for _ in 0..rng.next_u64() % 4 {
        s.push(Cycle::Idle);
    }
    s.push(Cycle::Start);
    s.bits(rng, bits, max_gap);
    s.push(Cycle::Idle);
    s
}

/// The vectors: every one of the 127 seeds, the all-zero head (seed zero), lengths 1…4095,
/// a bit every cycle and with gaps, drops before, inside and on an octet's last bit, starts
/// with a bit, back to back and during a frame, bits while nothing is armed.
pub fn vectors(out: &mut impl Write) -> io::Result<()> {
    writeln!(
        out,
        "# rvec rx_octets — generated, do not edit. Regenerate: make -C hw vectors"
    )?;
    writeln!(
        out,
        "# L <length, hex: the length input with every start until the next L>"
    )?;
    writeln!(
        out,
        "# I <a character a cycle: . idle, 0/1 bit, S start, D drop, s/t start+bit 0/1, a/b drop+bit 0/1, X start+drop>"
    )?;
    writeln!(
        out,
        "# O <cycle in the I line> <octet hex> <first> <last> <seed hex> <service hex>: after that cycle's edge"
    )?;
    writeln!(
        out,
        "# E <cycle> <0 decoded, 1 seed zero>: the end, after that cycle's edge"
    )?;
    writeln!(
        out,
        "# The segments run back to back without a reset; any other cycle shows no octet, no end."
    )?;
    let mut rng = Rng::new(0x0c7e_7500);
    let mut armed: Option<RxOctets> = None;

    // Every seed, short frames, a bit a cycle and with gaps.
    for seed in 1u8..=0x7f {
        let length = 1 + (rng.next_u64() % 40) as usize;
        let gap = [0, 0, 1, 3][(seed % 4) as usize];
        let bits = frame_bits(&mut rng, length, seed)?;
        frame(&mut rng, length, &bits, gap).write(out, &mut armed)?;
    }
    // Lengths at the counters' edges and random ones; random bits (any head, any SERVICE).
    let mut lengths = vec![1usize, 2, 3, 7, 8, 15, 16, 17, 255, 256, 257, 1500];
    for _ in 0..12 {
        lengths.push(1 + (rng.next_u64() % 600) as usize);
    }
    for &length in &lengths {
        let mut bits = random_bits(&mut rng, 16 + 8 * length + 6);
        if bits[..7].iter().all(|&b| b == 0) {
            bits[3] = 1;
        }
        let gap = rng.next_u64() % 3;
        frame(&mut rng, length, &bits, gap).write(out, &mut armed)?;
    }
    // The longest PSDU, a bit every cycle.
    let bits = frame_bits(&mut rng, 4095, 0x5d)?;
    frame(&mut rng, 4095, &bits, 0).write(out, &mut armed)?;
    // Seed zero: no octet, the end where the last octet would have been.
    for length in [1usize, 2, 9, 24, 100, 4095] {
        let mut bits = random_bits(&mut rng, 16 + 8 * length + 6);
        bits[..7].fill(0);
        frame(&mut rng, length, &bits, (length % 3) as u64).write(out, &mut armed)?;
    }
    // Drops: before the seed, inside SERVICE, inside the PSDU, on an octet's last bit (drop
    // wins), on a zero-seed frame; a drop with nothing armed.
    for (length, at, zero) in [
        (10usize, 3usize, false),
        (10, 12, false),
        (10, 40, false),
        (10, 16 + 8 * 4 + 7, false),
        (10, 16 + 8 * 9 + 7, false),
        (10, 30, true),
    ] {
        let mut bits = random_bits(&mut rng, 16 + 8 * length + 6);
        if zero {
            bits[..7].fill(0);
        } else {
            bits[0] = 1;
        }
        let mut s = Segment::new(length);
        s.push(Cycle::Start);
        s.bits(&mut rng, &bits[..at], 1);
        s.push(Cycle::DropBit(bits[at]));
        s.bits(&mut rng, &bits[at + 1..], 1);
        s.push(Cycle::Drop);
        s.push(Cycle::Idle);
        s.write(out, &mut armed)?;
    }
    // A start with a bit (not taken), back-to-back frames (the next start in the cycle after
    // the end), a start during a frame (it restarts).
    {
        let mut s = Segment::new(3);
        let a = frame_bits(&mut rng, 3, 0x11)?;
        let b = frame_bits(&mut rng, 3, 0x7f)?;
        let c = frame_bits(&mut rng, 3, 0x01)?;
        s.push(Cycle::StartBit(1));
        s.bits(&mut rng, &a[..16 + 24], 0);
        s.push(Cycle::StartBit(0));
        s.bits(&mut rng, &b[..20], 0);
        s.push(Cycle::Start);
        s.bits(&mut rng, &c, 0);
        s.push(Cycle::Idle);
        s.write(out, &mut armed)?;
    }
    // A start and a drop in one cycle, during a frame and with nothing armed: nothing armed
    // after it, the bits that follow ignored.
    {
        let mut s = Segment::new(4);
        let a = frame_bits(&mut rng, 4, 0x2a)?;
        s.push(Cycle::Start);
        s.bits(&mut rng, &a[..30], 0);
        s.push(Cycle::StartDrop);
        s.bits(&mut rng, &a, 0);
        s.push(Cycle::StartDrop);
        s.bits(&mut rng, &a, 1);
        s.push(Cycle::Idle);
        s.write(out, &mut armed)?;
    }
    // Bits with nothing armed, then a drop with nothing armed.
    {
        let mut s = Segment::new(5);
        let noise = random_bits(&mut rng, 64);
        s.bits(&mut rng, &noise, 0);
        s.push(Cycle::Drop);
        s.push(Cycle::Idle);
        s.write(out, &mut armed)?;
    }
    Ok(())
}
