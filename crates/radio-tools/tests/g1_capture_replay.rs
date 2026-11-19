//! Replay of a known capture (R0-13): what the receiver front finds in a capture file is what
//! it finds in the samples the file was written from, and that is where the frames are.
//!
//! The capture is `rcap synth`'s; its annotations are the ground truth (the simulator placed
//! the frames). The file formats themselves are gated in `radio-capture/tests/g1_capture.rs`.

#![allow(clippy::unwrap_used)]

use std::process::Command;

use radio_capture::sigmf::{Reader, Recording};
use radio_phy::rx_front::{backoff_samples, RxConfig};
use radio_phy::rx_front_fixed::{acquisitions_fixed, FixedRxConfig, RxFrontFixed};

#[test]
fn replay_finds_the_annotated_frames() {
    let dir = std::env::temp_dir().join(format!("rcap-replay-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("known.sigmf-meta");
    let rcap = env!("CARGO_BIN_EXE_rcap");
    let run = |args: &[&str]| {
        let out = Command::new(rcap).args(args).output().unwrap();
        assert!(
            out.status.success(),
            "rcap {args:?}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8(out.stdout).unwrap()
    };
    let p = path.to_str().unwrap();
    run(&["synth", p, "6", "12"]);
    assert!(run(&["verify", p]).contains("SHA-512 matches"));

    let rec = Recording::read(&path).unwrap();
    let samples = rec.to_i16().unwrap();
    let cfg = RxConfig::new(rec.meta.sample_rate_hz);
    let fx = FixedRxConfig::default();
    let direct = acquisitions_fixed(&samples, &cfg, &fx);

    // The same through the streaming reader, in awkward pieces.
    let mut reader = Reader::open(&path).unwrap();
    let mut rx = RxFrontFixed::new(cfg.clone(), fx);
    let mut streamed = Vec::new();
    loop {
        let chunk = reader.next_chunk(1009).unwrap();
        if chunk.is_empty() {
            break;
        }
        streamed.extend(
            chunk
                .iter()
                .filter_map(|s| rx.push((s.0 as i16, s.1 as i16))),
        );
    }
    assert_eq!(streamed, direct);

    // Every annotated frame is found where the simulator put it: T1 is 192 samples into the
    // frame, the FFT window a back-off earlier; the fractional delay moves it by at most one.
    assert_eq!(rec.meta.annotations.len(), 6);
    assert_eq!(direct.len(), 6);
    for (a, found) in rec.meta.annotations.iter().zip(&direct) {
        let want = a.sample_start as usize + 192 - backoff_samples(&cfg);
        assert!(
            found.fft_start.abs_diff(want) <= 1,
            "{} vs {want}",
            found.fft_start
        );
    }
    // And the tool prints the same acquisitions.
    let listed = run(&["replay", p]);
    for found in &direct {
        assert!(
            listed.contains(&format!(" {} ", found.fft_start)),
            "{listed}"
        );
    }
    assert!(listed.contains("6 acquisitions"));

    // Arguments that are not numbers in range are refused, not replaced by defaults.
    for args in [
        vec!["replay", p, "32"],
        vec!["replay", p, "-1"],
        vec!["replay", p, "abc"],
        vec!["synth", p, "abc"],
        vec!["synth", p, "3", "xyz"],
        vec!["synth", "plain-name", "3"],
    ] {
        let out = Command::new(rcap).args(&args).output().unwrap();
        assert!(!out.status.success(), "{args:?}");
    }
    assert!(
        Recording::read(&path).is_ok(),
        "a refused command must not touch the capture"
    );

    // A damaged capture is refused by the tool.
    let data = radio_capture::sigmf::data_path(&path).unwrap();
    let mut bytes = std::fs::read(&data).unwrap();
    bytes[100] ^= 1;
    std::fs::write(&data, bytes).unwrap();
    let out = Command::new(rcap).args(["verify", p]).output().unwrap();
    assert!(!out.status.success());
    assert!(String::from_utf8_lossy(&out.stderr).contains("SHA-512"));
    std::fs::remove_dir_all(dir).unwrap();
}
