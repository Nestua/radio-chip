//! G1 gate of the capture tooling (R0-13, `radio-capture`).
//!
//! References we did not write: CPython's standard library run as a black box
//! (`tools/ref/capture_python.py`, ADR 0005), FIPS 180-4's own examples, and the SigMF
//! project's own library (`tools/ref/capture_sigmf.py`, references 5 and 6):
//! 1. `hashlib.sha512` on 328 deterministic messages (every length to 299, the block
//!    boundaries, 1 MB), and the digests printed in FIPS 180-4 / NIST's example files.
//! 2. `json.dumps`: JSON texts with every escape, non-ASCII text, astral characters, numbers
//!    in every form: our reader must read them, and what we write back must read the same.
//! 3. A SigMF recording written by Python (`py.sigmf-*`): our reader gives its samples and
//!    fields.
//! 4. Python's verdict on a recording we wrote (`rs.sigmf-*`, committed, and checked here to
//!    be what today's code writes): it parses, the checksum holds, the fields are there.
//! 5. A SigMF recording written by the `sigmf` library (`sg.sigmf-*`): our reader gives its
//!    samples and fields, including the members the library adds on its own.
//! 6. The `sigmf` library's verdict on `rs.sigmf-*`: valid against the project's schema, the
//!    checksum verified, the samples read back; and the same metadata refused with a member
//!    damaged, so that "valid" means something.
//!
//! Round trips, damage and limits are properties (no outside reference needed).

#![allow(clippy::unwrap_used)]

use std::path::{Path, PathBuf};

use radio_capture::json::{parse, Value, MAX_DEPTH};
use radio_capture::sha512::{hex, sha512, Sha512};
use radio_capture::sigmf::{
    data_path, Annotation, Datatype, Meta, Reader, Recording, Segment, SigmfError,
};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("radio-capture-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The message of `capture_python.py`: n bytes of a fixed pattern.
fn message(n: usize) -> Vec<u8> {
    (0..n)
        .map(|i| ((i * 131 + (i >> 8) * 17 + 7) & 0xff) as u8)
        .collect()
}

#[test]
fn sha512_fips_examples() {
    // FIPS 180-4's examples (NIST, "SHA512.pdf"): "abc", the 896-bit message, and the
    // empty message.
    assert_eq!(
        hex(&sha512(b"abc")),
        "ddaf35a193617abacc417349ae20413112e6fa4e89a97ea20a9eeee64b55d39a\
         2192992a274fc1a836ba3c23a3feebbd454d4423643ce80e2a9ac94fa54ca49f"
    );
    assert_eq!(
        hex(&sha512(
            b"abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmno\
              ijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
        )),
        "8e959b75dae313da8cf4f72814fc143f8f7779c6eb9f7fa17299aeadb6889018\
         501d289e4900f7e4331b99dec4b5433ac7d329eeb6dd26545e96e55b874be909"
    );
    assert_eq!(
        hex(&sha512(b"")),
        "cf83e1357eefb8bdf1542850d66d8007d620e4050b5715dc83f4a921d36ce9ce\
         47d0d13c5d85f2b0ff8318d2877eec2f63b931bd47417a81a538327af927da3e"
    );
}

#[test]
fn sha512_matches_hashlib() {
    let text = std::fs::read_to_string(fixtures().join("sha512_hashlib.txt")).unwrap();
    let mut n = 0;
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let (len, want) = line.split_once(' ').unwrap();
        let m = message(len.parse().unwrap());
        assert_eq!(hex(&sha512(&m)), want, "length {len}");
        // Fed in uneven pieces, the same digest.
        let mut h = Sha512::new();
        let mut at = 0;
        let mut step = 1;
        while at < m.len() {
            let end = (at + step).min(m.len());
            h.update(&m[at..end]);
            at = end;
            step = step * 3 % 257 + 1;
        }
        assert_eq!(hex(&h.finalize()), want, "length {len} in pieces");
        n += 1;
    }
    println!("{n} digests agree with hashlib");
    assert_eq!(n, 328);
}

#[test]
fn json_reads_what_python_writes() {
    let text = std::fs::read_to_string(fixtures().join("json_python.txt")).unwrap();
    let lines: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
    assert!(lines.len() >= 46 && lines.len().is_multiple_of(2));
    for pair in lines.chunks(2) {
        // The same value written with \u escapes and with raw UTF-8 must read the same.
        let (a, b) = (parse(pair[0]).unwrap(), parse(pair[1]).unwrap());
        assert_eq!(a, b, "{} vs {}", pair[0], pair[1]);
        // What we write reads back as the same value, compact and indented.
        assert_eq!(parse(&a.to_compact()).unwrap(), a, "{}", pair[0]);
        assert_eq!(parse(&a.to_pretty()).unwrap(), a, "{}", pair[0]);
    }
    // Spot checks of the content itself, against what the Python source says.
    let find = |needle: &str| {
        lines
            .iter()
            .find(|l| l.contains(needle))
            .map(|l| parse(l).unwrap())
            .unwrap()
    };
    assert_eq!(
        find("backslash"),
        Value::String("quote \" backslash \\ slash / tab \t newline \n return \r".into())
    );
    assert_eq!(
        find("control"),
        Value::String("control \u{1} \u{1f} bell \u{8} formfeed \u{c}".into())
    );
    assert_eq!(
        find("astral"),
        Value::String("astral \u{1F4E1} and \u{2028} separator".into())
    );
    assert_eq!(parse(lines[10]).unwrap().as_u64(), Some(20_000_000));
    assert_eq!(parse("5800000000.0").unwrap().as_f64(), Some(5.8e9));
    assert_eq!(parse("2e7").unwrap().as_u64(), Some(20_000_000));
    assert_eq!(
        parse("12345678901234567890").unwrap().as_u64(),
        Some(12_345_678_901_234_567_890)
    );
    assert_eq!(parse("-1").unwrap().as_u64(), None);
    assert_eq!(parse("0.5").unwrap().as_u64(), None);
    // RFC 8259 §7, the escapes Python never writes: the solidus, and hexadecimal in capitals.
    assert_eq!(parse(r#""a\/b""#).unwrap(), Value::String("a/b".into()));
    assert_eq!(
        parse(r#""\u00AF\u00af\uABCD\uD83D\uDCE1\uFFFF""#).unwrap(),
        Value::String("\u{af}\u{af}\u{abcd}\u{1F4E1}\u{ffff}".into())
    );
    // Integers are decided on the digits, not through a double.
    for (text, want) in [
        ("9007199254740993", Some(9_007_199_254_740_993u64)),
        ("9007199254740993.0", Some(9_007_199_254_740_993)),
        ("9007199254740992.5", None),
        ("0.99999999999999999999", None),
        ("1e-400", None),
        ("0e-400", Some(0)),
        ("1.5e1", Some(15)),
        ("1.50e1", Some(15)),
        ("1.55e1", None),
        ("120e-1", Some(12)),
        ("125e-1", None),
        ("18446744073709551615", Some(u64::MAX)),
        ("18446744073709551616", None),
        ("1e19", Some(10_000_000_000_000_000_000)),
        ("1e20", None),
        ("1e400", None),
        ("-0", Some(0)),
        ("-0.0", Some(0)),
        ("-1e-400", None),
        ("0.0", Some(0)),
        ("00012e0", None),
    ] {
        match parse(text) {
            Ok(v) => assert_eq!(v.as_u64(), want, "{text}"),
            Err(_) => assert_eq!(want, None, "{text}"),
        }
    }
}

#[test]
fn json_refuses_what_is_not_json() {
    for bad in [
        "",
        " ",
        "nul",
        "tru",
        "[1,]",
        "[1 2]",
        "{\"a\":1,}",
        "{\"a\" 1}",
        "{a:1}",
        "01",
        "1.",
        ".5",
        "-",
        "1e",
        "+1",
        "\"abc",
        "\"\\x\"",
        "\"\\u12\"",
        "\"\\ud800\"",
        "\"\\udc00\"",
        "\"\\ud800\\u0041\"",
        "\"tab\there\"",
        "[1] 2",
        "{} {}",
        "NaN",
        "Infinity",
        "'a'",
        "[\"a\":1]",
        "{\"a\"}",
        "\u{feff}1",
    ] {
        assert!(parse(bad).is_err(), "accepted {bad:?}");
    }
    let deep = |n: usize| format!("{}{}", "[".repeat(n), "]".repeat(n));
    assert!(parse(&deep(MAX_DEPTH)).is_ok());
    assert!(parse(&deep(MAX_DEPTH + 1)).is_err());
    let objects = |n: usize| format!("{}1{}", "{\"a\":".repeat(n), "}".repeat(n));
    assert!(parse(&objects(MAX_DEPTH)).is_ok());
    assert!(parse(&objects(MAX_DEPTH + 1)).is_err());
    assert!(parse(&deep(100_000)).is_err());
    assert_eq!(Value::from_f64(f64::NAN), None);
    assert_eq!(Value::from_f64(f64::INFINITY), None);
}

#[test]
fn reads_the_recording_python_wrote() {
    let r = Recording::read(&fixtures().join("py.sigmf-meta")).unwrap();
    assert_eq!(r.meta.datatype, Datatype::Ci16Le);
    assert_eq!(r.meta.sample_rate_hz, 20e6);
    assert_eq!(r.meta.version, "1.2.0");
    assert_eq!(
        r.meta.description.as_deref(),
        Some("ramp written by tools/ref/capture_python.py, \"quoted\", Київ")
    );
    assert_eq!(r.meta.hw.as_deref(), Some("none (synthetic)"));
    assert_eq!(
        r.meta.other,
        vec![("nestua:adc_bits".to_string(), Value::Number("12".into()))]
    );
    assert_eq!(r.meta.captures.len(), 2);
    assert_eq!(r.meta.captures[0].frequency_hz, Some(5.805e9));
    assert_eq!(
        r.meta.captures[0].datetime.as_deref(),
        Some("2026-11-19T00:00:00Z")
    );
    assert_eq!(r.meta.captures[1].sample_start, 300);
    assert_eq!(r.meta.captures[1].frequency_hz, Some(5_825_000_000.5));
    let a = &r.meta.annotations[0];
    assert_eq!((a.sample_start, a.sample_count), (100, Some(50)));
    assert_eq!(
        (a.label.as_deref(), a.comment.as_deref()),
        (Some("burst"), Some("first"))
    );
    assert_eq!(a.other[0].0, "other:field");
    // The ramp of capture_python.py.
    let want: Vec<(i16, i16)> = (0..600i32)
        .map(|n| {
            (
                ((7 * n) % 4096 - 2048) as i16,
                (2047 - (13 * n) % 4096) as i16,
            )
        })
        .collect();
    assert_eq!(r.to_i16().unwrap(), want);
    assert_eq!(r.len(), 600);
    // In pieces, the same samples, and the checksum is verified at the end.
    let mut reader = Reader::open(&fixtures().join("py.sigmf-meta")).unwrap();
    let mut got = Vec::new();
    loop {
        let chunk = reader.next_chunk(77).unwrap();
        if chunk.is_empty() {
            break;
        }
        got.extend(chunk);
    }
    let want_f: Vec<(f64, f64)> = want
        .iter()
        .map(|s| (f64::from(s.0), f64::from(s.1)))
        .collect();
    assert_eq!(got, want_f);
}

#[test]
fn reads_the_recording_the_sigmf_library_wrote() {
    let r = Recording::read(&fixtures().join("sg.sigmf-meta")).unwrap();
    assert_eq!(r.meta.datatype, Datatype::Ci16Le);
    assert_eq!(r.meta.sample_rate_hz, 20e6);
    assert_eq!(
        r.meta.description.as_deref(),
        Some("ramp written by the sigmf library")
    );
    assert_eq!(r.meta.author.as_deref(), Some("capture_sigmf.py"));
    // The library writes these two on its own; they say what we assume, and are kept.
    assert_eq!(
        r.meta.other,
        vec![
            ("core:num_channels".to_string(), Value::Number("1".into())),
            ("core:offset".to_string(), Value::Number("0".into())),
        ]
    );
    assert_eq!(r.meta.captures.len(), 2);
    assert_eq!(r.meta.captures[0].frequency_hz, Some(5.805e9));
    assert_eq!(
        r.meta.captures[0].datetime.as_deref(),
        Some("2026-11-19T00:00:00Z")
    );
    assert_eq!(
        (
            r.meta.captures[1].sample_start,
            r.meta.captures[1].frequency_hz
        ),
        (200, Some(5.825e9))
    );
    let a = &r.meta.annotations[0];
    assert_eq!((a.sample_start, a.sample_count), (40, Some(120)));
    assert_eq!(
        (a.label.as_deref(), a.comment.as_deref()),
        (Some("burst"), Some("second reference"))
    );
    let names: Vec<&str> = a.other.iter().map(|(k, _)| k.as_str()).collect();
    assert_eq!(names, ["core:freq_lower_edge", "core:freq_upper_edge"]);
    // The ramp of capture_sigmf.py.
    let want: Vec<(i16, i16)> = (0..400i32)
        .map(|n| {
            (
                ((11 * n) % 4096 - 2048) as i16,
                (2047 - (5 * n) % 4096) as i16,
            )
        })
        .collect();
    assert_eq!(r.to_i16().unwrap(), want);
}

#[test]
fn the_sigmf_library_accepts_the_recording_we_write() {
    // `python_reads_the_recording_we_write` shows the committed rs.sigmf-* are what today's
    // code writes; here the library's verdict must be about those same bytes.
    let meta = std::fs::read(fixtures().join("rs.sigmf-meta")).unwrap();
    let verdict = std::fs::read_to_string(fixtures().join("sg_verdict.txt")).unwrap();
    let field = |name: &str| {
        verdict
            .lines()
            .find_map(|l| l.strip_prefix(name).and_then(|r| r.strip_prefix(' ')))
            .unwrap_or_else(|| panic!("no {name} in the verdict"))
            .to_string()
    };
    assert_eq!(
        field("meta_sha512"),
        hex(&sha512(&meta)),
        "the verdict is about another file"
    );
    assert_eq!(field("schema_valid"), "True");
    assert_eq!(field("checksum_verified"), "True");
    let ours = ours();
    assert_eq!(field("samples"), ours.len().to_string());
    let first: Vec<String> = ours.to_i16().unwrap()[..4]
        .iter()
        .flat_map(|s| [s.0.to_string(), s.1.to_string()])
        .collect();
    assert_eq!(field("first_samples"), first.join(" "));
    assert_eq!(field("sample_rate"), "20000000");
    assert_eq!(field("captures"), "1 frequency 5805000000");
    assert_eq!(field("annotation"), "16 320 preamble");
    // The validator bites: the same metadata with one member damaged is refused.
    let refusals: Vec<&str> = verdict
        .lines()
        .filter(|l| l.starts_with("refuses_"))
        .collect();
    assert_eq!(refusals.len(), 6);
    for line in refusals {
        assert!(line.ends_with(" True"), "{line}");
    }
}

/// The recording committed as `rs.sigmf-*`, the one Python gave its verdict on.
fn ours() -> Recording {
    let mut meta = Meta::new(Datatype::Ci16Le, 20e6, Some(5.805e9));
    meta.description =
        Some("written by radio-capture: \"quotes\", a tab\t, Київ, \u{1F4E1}".into());
    meta.hw = Some("none (synthetic)".into());
    meta.author = Some("g1_capture".into());
    meta.other
        .push(("nestua:adc_bits".into(), Value::from_u64(12)));
    meta.captures[0].datetime = Some("2026-11-19T00:00:00Z".into());
    meta.annotations.push(Annotation {
        sample_start: 16,
        sample_count: Some(320),
        label: Some("preamble".into()),
        comment: None,
        other: Vec::new(),
    });
    let samples: Vec<(i16, i16)> = (0..500i32)
        .map(|n| {
            (
                ((n * n) % 4001 - 2000) as i16,
                ((n * 37) % 3999 - 1999) as i16,
            )
        })
        .collect();
    Recording::from_i16(meta, &samples)
}

#[test]
fn python_reads_the_recording_we_write() {
    let dir = scratch("ours");
    let path = dir.join("rs.sigmf-meta");
    ours().write(&path).unwrap();
    let (meta, data) = (
        std::fs::read(&path).unwrap(),
        std::fs::read(data_path(&path).unwrap()).unwrap(),
    );
    if std::env::var_os("UPDATE_FIXTURES").is_some() {
        std::fs::write(fixtures().join("rs.sigmf-meta"), &meta).unwrap();
        std::fs::write(fixtures().join("rs.sigmf-data"), &data).unwrap();
    }
    // The committed files are what today's code writes, so the verdict is about this code.
    assert_eq!(
        meta,
        std::fs::read(fixtures().join("rs.sigmf-meta")).unwrap(),
        "rs.sigmf-meta is stale"
    );
    assert_eq!(
        data,
        std::fs::read(fixtures().join("rs.sigmf-data")).unwrap(),
        "rs.sigmf-data is stale"
    );
    let verdict = std::fs::read_to_string(fixtures().join("rs_verdict.txt")).unwrap();
    let field = |name: &str| {
        verdict
            .lines()
            .find_map(|l| l.strip_prefix(name).and_then(|r| r.strip_prefix(' ')))
            .unwrap_or_else(|| panic!("no {name} in the verdict"))
            .to_string()
    };
    assert_eq!(
        field("meta_sha512"),
        hex(&sha512(&meta)),
        "the verdict is about another file"
    );
    assert_eq!(field("top_level_keys"), "annotations captures global");
    assert_eq!(field("datatype"), "ci16_le");
    assert_eq!(field("sample_rate"), "20000000");
    assert_eq!(field("version"), "1.2.0");
    assert_eq!(field("sha512_matches"), "True");
    assert_eq!(field("samples"), "500");
    assert_eq!(
        field("first_samples"),
        "-2000 -1999 -1999 -1962 -1996 -1925 -1991 -1888"
    );
    assert_eq!(field("captures"), "1 frequency 5805000000");
    assert_eq!(field("annotations"), "1");
    assert_eq!(
        field("global_keys"),
        "core:author core:datatype core:description core:hw core:sample_rate core:sha512 core:version nestua:adc_bits"
    );
    assert_eq!(
        field("capture_keys"),
        "core:datetime core:frequency core:sample_start"
    );
    assert_eq!(
        field("annotation_keys"),
        "core:label core:sample_count core:sample_start"
    );
    assert_eq!(field("datetime"), "2026-11-19T00:00:00Z");
    assert_eq!(field("annotation"), "16 320 preamble");
    assert_eq!(
        field("description"),
        "\"written by radio-capture: \\\"quotes\\\", a tab\\t, \\u041a\\u0438\\u0457\\u0432, \\ud83d\\udce1\""
    );
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn round_trip_keeps_everything() {
    let dir = scratch("roundtrip");
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for (k, datatype) in [
        Datatype::Ci16Le,
        Datatype::Cf32Le,
        Datatype::Ci8,
        Datatype::Cu8,
    ]
    .into_iter()
    .enumerate()
    {
        let n = [0usize, 1, 1000, 4097][k];
        let mut meta = Meta::new(
            datatype,
            [20e6, 10e6, 5e6, 2.4e6][k],
            Some(5.8e9 + k as f64 * 0.5),
        );
        meta.description = Some(format!("case {k}: \u{0} \u{7f} \\ \" \n \u{10ffff}"));
        meta.other.push((
            "x:nested".into(),
            parse("{\"a\":[1,2.5e-3,null,true],\"b\":{}}").unwrap(),
        ));
        meta.other_top
            .push(("x:top".into(), Value::String("kept".into())));
        meta.captures.push(Segment {
            sample_start: u64::MAX,
            frequency_hz: None,
            datetime: None,
            other: vec![("x:gain".into(), Value::Number("-12.5".into()))],
        });
        let data: Vec<u8> = (0..n * datatype.sample_bytes())
            .map(|_| next() as u8)
            .collect();
        let rec = Recording { meta, data };
        let path = dir.join(format!("case{k}.sigmf-meta"));
        let written = rec.write(&path).unwrap();
        let back = Recording::read(&path).unwrap();
        assert_eq!(back.meta, written);
        assert_eq!(back.data, rec.data);
        assert_eq!(back.len(), n);
        assert_eq!(back.to_f64().len(), n);
        // Written again from what was read: the same bytes.
        let again = dir.join(format!("again{k}.sigmf-meta"));
        back.write(&again).unwrap();
        assert_eq!(
            std::fs::read(&path).unwrap(),
            std::fs::read(&again).unwrap()
        );
    }
    // The sample values of each format.
    let rec = |datatype, data: Vec<u8>| Recording {
        meta: Meta::new(datatype, 1.0, None),
        data,
    };
    assert_eq!(
        rec(Datatype::Ci16Le, vec![0x00, 0x80, 0xff, 0x7f]).to_f64(),
        vec![(-32768.0, 32767.0)]
    );
    assert_eq!(
        rec(Datatype::Ci8, vec![0x80, 0x7f]).to_f64(),
        vec![(-128.0, 127.0)]
    );
    assert_eq!(
        rec(Datatype::Cu8, vec![0, 255]).to_f64(),
        vec![(-127.5, 127.5)]
    );
    let mut f = Vec::new();
    f.extend_from_slice(&(-1.5f32).to_le_bytes());
    f.extend_from_slice(&0.25f32.to_le_bytes());
    assert_eq!(rec(Datatype::Cf32Le, f).to_f64(), vec![(-1.5, 0.25)]);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn damage_is_reported() {
    let dir = scratch("damage");
    let path = dir.join("d.sigmf-meta");
    let rec = ours();
    rec.write(&path).unwrap();
    let data = data_path(&path).unwrap();
    let good = std::fs::read(&data).unwrap();
    // One bit of the dataset.
    let mut bad = good.clone();
    bad[1234] ^= 0x10;
    std::fs::write(&data, &bad).unwrap();
    assert!(matches!(
        Recording::read(&path),
        Err(SigmfError::Checksum { .. })
    ));
    let mut reader = Reader::open(&path).unwrap();
    assert!(reader.next_chunk(100).is_ok());
    assert!(matches!(
        reader.next_chunk(1000),
        Err(SigmfError::Checksum { .. })
    ));
    // Not whole samples.
    std::fs::write(&data, &good[..good.len() - 1]).unwrap();
    assert!(matches!(
        Recording::read(&path),
        Err(SigmfError::Length(1999))
    ));
    // No dataset.
    std::fs::remove_file(&data).unwrap();
    assert!(matches!(Recording::read(&path), Err(SigmfError::Io(_))));
    // Metadata.
    let meta = std::fs::read_to_string(&path).unwrap();
    for (from, to, want) in [
        ("\"ci16_le\"", "\"cf64_be\"", "datatype"),
        ("\"core:datatype\"", "\"core:datatyp\"", "no core:datatype"),
        ("20000000", "\"fast\"", "core:sample_rate"),
        ("20000000", "0", "core:sample_rate"),
        ("\"captures\": [", "\"captures\": 3, \"x\": [", "captures"),
        (
            "\"core:sample_start\": 16",
            "\"core:sample_start\": -16",
            "core:sample_start",
        ),
        ("{", "", "JSON"),
    ] {
        assert!(meta.contains(from), "{from}");
        let e = Meta::from_json(&meta.replacen(from, to, 1)).unwrap_err();
        assert!(e.to_string().contains(want), "{from} -> {to}: {e}");
    }
    // Parts of SigMF that would move the samples are refused by name, and so are duplicates.
    for (from, to, want) in [
        (
            "\"core:version\"",
            "\"core:num_channels\": 2, \"core:version\"",
            "core:num_channels",
        ),
        (
            "\"core:version\"",
            "\"core:dataset\": \"other.bin\", \"core:version\"",
            "core:dataset",
        ),
        (
            "\"core:version\"",
            "\"core:trailing_bytes\": 4, \"core:version\"",
            "core:trailing_bytes",
        ),
        (
            "\"core:version\"",
            "\"core:metadata_only\": true, \"core:version\"",
            "core:metadata_only",
        ),
        (
            "\"core:frequency\"",
            "\"core:header_bytes\": 4, \"core:frequency\"",
            "core:header_bytes",
        ),
        (
            "\"core:version\"",
            "\"core:hw\": \"twice\", \"core:version\"",
            "duplicate",
        ),
        ("\"captures\"", "\"global\": {}, \"captures\"", "duplicate"),
    ] {
        let e = Meta::from_json(&meta.replacen(from, to, 1)).unwrap_err();
        assert!(e.to_string().contains(want), "{to}: {e}");
    }
    // The harmless spellings of the same fields are accepted and kept.
    let ok = meta.replacen(
        "\"core:version\"",
        "\"core:num_channels\": 1, \"core:trailing_bytes\": 0, \"core:metadata_only\": false, \"core:version\"",
        1,
    );
    assert_eq!(Meta::from_json(&ok).unwrap().other.len(), 4);
    // Writing: nothing that could not be read back, and never over the metadata itself.
    let rec = ours();
    for name in [
        "x.sigmf-data",
        "x",
        "rec.2026-11-19",
        ".sigmf-meta",
        "x.sigmf-meta.bak",
    ] {
        assert!(
            matches!(rec.write(&dir.join(name)), Err(SigmfError::Name(_))),
            "{name}"
        );
        assert!(!dir.join(name).exists(), "{name} was written");
    }
    assert_eq!(
        data_path(Path::new("a/rec.2026-11-19.sigmf-meta")).unwrap(),
        Path::new("a/rec.2026-11-19.sigmf-data")
    );
    let target = dir.join("bad.sigmf-meta");
    let mut bad = ours();
    bad.meta.sample_rate_hz = f64::NAN;
    assert!(matches!(bad.write(&target), Err(SigmfError::Field(_))));
    let mut bad = ours();
    bad.meta.captures[0].frequency_hz = Some(f64::INFINITY);
    assert!(matches!(bad.write(&target), Err(SigmfError::Field(_))));
    let mut bad = ours();
    bad.data.pop();
    assert!(matches!(bad.write(&target), Err(SigmfError::Length(1999))));
    assert!(!target.exists() && !data_path(&target).unwrap().exists());
    // The streaming reader: an absurd request is capped, and an error is not forgotten.
    rec.write(&path).unwrap();
    let mut reader = Reader::open(&path).unwrap();
    assert_eq!(reader.next_chunk(usize::MAX).unwrap().len(), 500);
    assert!(reader.next_chunk(usize::MAX).unwrap().is_empty());
    let mut bytes = std::fs::read(&data).unwrap();
    bytes[7] ^= 0x80;
    std::fs::write(&data, bytes).unwrap();
    let mut reader = Reader::open(&path).unwrap();
    assert!(matches!(
        reader.next_chunk(1000),
        Err(SigmfError::Checksum { .. })
    ));
    assert!(matches!(
        reader.next_chunk(1000),
        Err(SigmfError::Checksum { .. })
    ));
    // Capture segments out of order.
    let py = std::fs::read_to_string(fixtures().join("py.sigmf-meta")).unwrap();
    assert!(Meta::from_json(&py).is_ok());
    let swapped = py
        .replacen("\"core:sample_start\": 300", "\"core:sample_start\": 0", 1)
        .replacen("\"core:sample_start\": 0", "\"core:sample_start\": 300", 1);
    assert_ne!(swapped, py);
    let e = Meta::from_json(&swapped).unwrap_err();
    assert!(e.to_string().contains("not in order"), "{e}");
    std::fs::remove_dir_all(dir).unwrap();
}
