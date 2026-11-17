//! G1 gate for the subcarrier mapper: the golden model against references we did not write.
//!
//! 1. A black-box run of OpenOFDM's receiver (`scripts/decode.py`, Demodulator.demodulate,
//!    Apache-2.0, commit ea638eb) on every point of every constellation:
//!    `tests/fixtures/mapper_openofdm.txt`, produced by `tools/ref/mapper_openofdm.py`.
//! 2. Analytic properties of IEEE 802.11-2020 §17.3.5.8: the constellations are Gray coded
//!    (nearest neighbours differ in one bit), and K_MOD normalises the mean energy to 1, i.e. the
//!    mean |point|² of the unnormalised grid equals 1/K_MOD² = 1, 2, 10, 42.

use radio_phy::mapper::{map_bits, map_group, mean_energy, PartialGroup, Point};
use radio_phy::modulation::Modulation;

const ALL: [Modulation; 4] = [
    Modulation::Bpsk,
    Modulation::Qpsk,
    Modulation::Qam16,
    Modulation::Qam64,
];

const OPENOFDM: &str = include_str!("fixtures/mapper_openofdm.txt");

fn group(value: usize, n: usize) -> Vec<u8> {
    // b0 is the most significant bit of `value`, matching the order bits enter the mapper.
    (0..n).map(|b| ((value >> (n - 1 - b)) & 1) as u8).collect()
}

fn all_points(m: Modulation) -> Result<Vec<(Vec<u8>, Point)>, PartialGroup> {
    let n = m.bits_per_subcarrier();
    (0..1usize << n)
        .map(|v| {
            let g = group(v, n);
            map_group(&g, m).map(|p| (g, p))
        })
        .collect()
}

#[test]
fn matches_openofdm_on_every_point() {
    let mut per_mod = [0usize; 7];
    for line in OPENOFDM
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(f.len(), 4, "malformed fixture line: {line}");
        let n_bpsc: usize = f[0].parse().unwrap();
        let m = Modulation::from_bits_per_subcarrier(n_bpsc).unwrap();
        let want = Point {
            i: f[1].parse().unwrap(),
            q: f[2].parse().unwrap(),
        };
        let bits: Vec<u8> = f[3].bytes().map(|c| u8::from(c == b'1')).collect();
        assert_eq!(map_group(&bits, m).unwrap(), want, "{m:?} bits {}", f[3]);
        per_mod[n_bpsc] += 1;
    }
    assert_eq!(
        [per_mod[1], per_mod[2], per_mod[4], per_mod[6]],
        [2, 4, 16, 64],
        "fixture must hold every point"
    );
}

#[test]
fn every_label_maps_to_a_distinct_grid_point() {
    for m in ALL {
        let points = all_points(m).unwrap();
        let max = match m {
            Modulation::Bpsk | Modulation::Qpsk => 1,
            Modulation::Qam16 => 3,
            Modulation::Qam64 => 7,
        };
        for (g, p) in &points {
            assert!(p.i % 2 != 0 && p.i.abs() <= max, "{m:?} {g:?} -> {p:?}");
            let q_ok = if m == Modulation::Bpsk {
                p.q == 0
            } else {
                p.q % 2 != 0 && p.q.abs() <= max
            };
            assert!(q_ok, "{m:?} {g:?} -> {p:?}");
        }
        for (a, (_, pa)) in points.iter().enumerate() {
            for (_, pb) in points.iter().skip(a + 1) {
                assert_ne!(pa, pb, "{m:?}: two labels on one point");
            }
        }
    }
}

#[test]
fn nearest_neighbours_differ_in_one_bit() {
    for m in ALL {
        let points = all_points(m).unwrap();
        for (ga, pa) in &points {
            for (gb, pb) in &points {
                let dist = (pa.i - pb.i).abs() + (pa.q - pb.q).abs();
                if dist == 2 {
                    let diff = ga.iter().zip(gb).filter(|(x, y)| x != y).count();
                    assert_eq!(diff, 1, "{m:?}: {ga:?}@{pa:?} vs {gb:?}@{pb:?}");
                }
            }
        }
    }
}

#[test]
fn mean_energy_is_one_over_k_mod_squared() {
    for m in ALL {
        let points = all_points(m).unwrap();
        let total: u32 = points
            .iter()
            .map(|(_, p)| (i32::from(p.i).pow(2) + i32::from(p.q).pow(2)) as u32)
            .sum();
        assert_eq!(total, mean_energy(m) * points.len() as u32, "{m:?}");
    }
    let expected: Vec<u32> = ALL.iter().map(|&m| mean_energy(m)).collect();
    assert_eq!(expected, vec![1, 2, 10, 42]);
}

#[test]
fn stream_mapping_groups_bits_in_order() {
    let bits = [1, 0, 1, 1, 0, 0, 0, 1];
    let points = map_bits(&bits, Modulation::Qpsk).unwrap();
    let by_group: Vec<Point> = bits
        .chunks(2)
        .map(|g| map_group(g, Modulation::Qpsk).unwrap())
        .collect();
    assert_eq!(points, by_group);
    assert_eq!(
        map_bits(&bits[..7], Modulation::Qam16),
        Err(PartialGroup { len: 7, n_bpsc: 4 })
    );
    assert!(map_group(&bits[..3], Modulation::Qpsk).is_err());
}
