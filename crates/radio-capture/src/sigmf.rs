//! SigMF recordings (specification v1.2): `<name>.sigmf-meta` and `<name>.sigmf-data`.
//!
//! Only what the receiver work needs is typed: the sample format and rate, the dataset's
//! checksum, the capture segments (centre frequency, time) and the annotations. Every other
//! field is kept as read and written back unchanged.

use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use crate::json::{parse, JsonError, Value};
use crate::sha512::{hex, Sha512};

/// The SigMF version written into new metadata.
pub const SIGMF_VERSION: &str = "1.2.0";

/// Sample formats read and written (`core:datatype`): complex, interleaved I then Q.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Datatype {
    /// `ci16_le`: signed 16-bit, little endian (the SDRs' native format; our 12-bit ADC).
    Ci16Le,
    /// `cf32_le`: IEEE 754 single precision, little endian.
    Cf32Le,
    /// `ci8`: signed 8-bit.
    Ci8,
    /// `cu8`: unsigned 8-bit, zero at 127.5 (RTL-SDR).
    Cu8,
}

impl Datatype {
    /// The `core:datatype` string.
    pub fn name(self) -> &'static str {
        match self {
            Datatype::Ci16Le => "ci16_le",
            Datatype::Cf32Le => "cf32_le",
            Datatype::Ci8 => "ci8",
            Datatype::Cu8 => "cu8",
        }
    }

    /// The format named by a `core:datatype` string, if it is one of ours.
    pub fn from_name(name: &str) -> Option<Datatype> {
        [
            Datatype::Ci16Le,
            Datatype::Cf32Le,
            Datatype::Ci8,
            Datatype::Cu8,
        ]
        .into_iter()
        .find(|d| d.name() == name)
    }

    /// Bytes of one complex sample.
    pub fn sample_bytes(self) -> usize {
        match self {
            Datatype::Ci16Le => 4,
            Datatype::Cf32Le => 8,
            Datatype::Ci8 | Datatype::Cu8 => 2,
        }
    }
}

/// Why a recording could not be read or written.
#[derive(Debug, Clone, PartialEq)]
pub enum SigmfError {
    /// The file system refused: the operation and the system's message.
    Io(String),
    /// The metadata is not JSON.
    Json(JsonError),
    /// A required field is missing.
    Missing(&'static str),
    /// A field has the wrong type or an impossible value.
    Field(&'static str),
    /// `core:datatype` is not one this crate reads.
    Datatype(String),
    /// The dataset's SHA-512 is not the metadata's `core:sha512`.
    Checksum {
        /// The digest in the metadata.
        expected: String,
        /// The digest of the dataset.
        found: String,
    },
    /// The dataset's length is not a whole number of samples.
    Length(u64),
    /// The recording uses a part of SigMF this crate does not read (several channels, header
    /// or trailing bytes, a dataset under another name, metadata without a dataset). Reading
    /// on would give wrong samples.
    Unsupported(&'static str),
    /// The metadata file's name does not end in `.sigmf-meta`.
    Name(String),
}

impl std::fmt::Display for SigmfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SigmfError::Io(e) => write!(f, "{e}"),
            SigmfError::Json(e) => write!(f, "{e}"),
            SigmfError::Missing(k) => write!(f, "metadata has no {k}"),
            SigmfError::Field(k) => write!(f, "metadata field {k} has the wrong type or value"),
            SigmfError::Datatype(d) => write!(f, "datatype {d} is not supported"),
            SigmfError::Checksum { expected, found } => {
                write!(
                    f,
                    "dataset SHA-512 is {found}, the metadata says {expected}"
                )
            }
            SigmfError::Length(n) => write!(f, "dataset of {n} bytes is not whole samples"),
            SigmfError::Unsupported(k) => write!(f, "{k} is not supported"),
            SigmfError::Name(n) => write!(f, "{n} does not end in .sigmf-meta"),
        }
    }
}

impl std::error::Error for SigmfError {}

impl From<JsonError> for SigmfError {
    fn from(e: JsonError) -> Self {
        SigmfError::Json(e)
    }
}

fn io(what: &str, path: &Path, e: std::io::Error) -> SigmfError {
    SigmfError::Io(format!("{what} {}: {e}", path.display()))
}

/// A capture segment: the samples from `sample_start` on share these parameters.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Segment {
    /// First sample of the segment (`core:sample_start`).
    pub sample_start: u64,
    /// Centre frequency in Hz (`core:frequency`).
    pub frequency_hz: Option<f64>,
    /// Time of the first sample, ISO 8601 (`core:datetime`).
    pub datetime: Option<String>,
    /// Every other field, as read.
    pub other: Vec<(String, Value)>,
}

/// An annotation: a stretch of samples with a label.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Annotation {
    /// First sample (`core:sample_start`).
    pub sample_start: u64,
    /// Number of samples (`core:sample_count`).
    pub sample_count: Option<u64>,
    /// A short label (`core:label`).
    pub label: Option<String>,
    /// A comment (`core:comment`).
    pub comment: Option<String>,
    /// Every other field, as read.
    pub other: Vec<(String, Value)>,
}

/// The metadata of a recording.
#[derive(Debug, Clone, PartialEq)]
pub struct Meta {
    /// Sample format (`core:datatype`).
    pub datatype: Datatype,
    /// Sample rate in Hz (`core:sample_rate`).
    pub sample_rate_hz: f64,
    /// Specification version (`core:version`).
    pub version: String,
    /// SHA-512 of the dataset, lower-case hexadecimal (`core:sha512`).
    pub sha512: Option<String>,
    /// Free text (`core:description`).
    pub description: Option<String>,
    /// The hardware that recorded it (`core:hw`).
    pub hw: Option<String>,
    /// Who recorded it (`core:author`).
    pub author: Option<String>,
    /// Every other global field, as read.
    pub other: Vec<(String, Value)>,
    /// Capture segments, in order of `sample_start`.
    pub captures: Vec<Segment>,
    /// Annotations.
    pub annotations: Vec<Annotation>,
    /// Top-level members other than `global`, `captures` and `annotations`, as read.
    pub other_top: Vec<(String, Value)>,
}

fn members<'a>(v: &'a Value, what: &'static str) -> Result<&'a [(String, Value)], SigmfError> {
    match v {
        Value::Object(m) => Ok(m),
        _ => Err(SigmfError::Field(what)),
    }
}

fn text(v: &Value, what: &'static str) -> Result<String, SigmfError> {
    v.as_str()
        .map(str::to_string)
        .ok_or(SigmfError::Field(what))
}

fn count(v: &Value, what: &'static str) -> Result<u64, SigmfError> {
    v.as_u64().ok_or(SigmfError::Field(what))
}

fn real(v: &Value, what: &'static str) -> Result<f64, SigmfError> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or(SigmfError::Field(what))
}

fn number(v: f64) -> Value {
    // Integers print without a point, so 20 MHz stays "20000000". Not finite gives null, which
    // no reader accepts: `Meta::validate` refuses such metadata before it is written.
    Value::from_f64(v).unwrap_or(Value::Null)
}

/// Whether a member name occurs twice.
fn duplicate(m: &[(String, Value)]) -> bool {
    m.iter()
        .enumerate()
        .any(|(i, (k, _))| m[..i].iter().any(|(e, _)| e == k))
}

impl Meta {
    /// Metadata of a new recording with one capture segment starting at sample 0.
    pub fn new(datatype: Datatype, sample_rate_hz: f64, frequency_hz: Option<f64>) -> Meta {
        Meta {
            datatype,
            sample_rate_hz,
            version: SIGMF_VERSION.to_string(),
            sha512: None,
            description: None,
            hw: None,
            author: None,
            other: Vec::new(),
            captures: vec![Segment {
                frequency_hz,
                ..Segment::default()
            }],
            annotations: Vec::new(),
            other_top: Vec::new(),
        }
    }

    /// Reads metadata from the text of a `.sigmf-meta` file.
    pub fn from_json(json: &str) -> Result<Meta, SigmfError> {
        let top = parse(json)?;
        let global = top.get("global").ok_or(SigmfError::Missing("global"))?;
        if duplicate(members(&top, "top level")?) || duplicate(members(global, "global")?) {
            return Err(SigmfError::Field("duplicate member name"));
        }
        let mut meta = Meta::new(Datatype::Ci16Le, 0.0, None);
        meta.captures.clear();
        let (mut datatype, mut rate, mut version) = (None, None, None);
        for (name, v) in members(global, "global")? {
            match name.as_str() {
                "core:datatype" => {
                    let t = text(v, "core:datatype")?;
                    datatype = Some(Datatype::from_name(&t).ok_or(SigmfError::Datatype(t))?);
                }
                "core:sample_rate" => rate = Some(real(v, "core:sample_rate")?),
                "core:version" => version = Some(text(v, "core:version")?),
                "core:sha512" => meta.sha512 = Some(text(v, "core:sha512")?),
                "core:description" => meta.description = Some(text(v, "core:description")?),
                "core:hw" => meta.hw = Some(text(v, "core:hw")?),
                "core:author" => meta.author = Some(text(v, "core:author")?),
                // Parts of SigMF that change where the samples are: refused, not ignored.
                "core:num_channels" if v.as_u64() != Some(1) => {
                    return Err(SigmfError::Unsupported("core:num_channels other than 1"));
                }
                "core:dataset" => return Err(SigmfError::Unsupported("core:dataset")),
                "core:trailing_bytes" if v.as_u64() != Some(0) => {
                    return Err(SigmfError::Unsupported("core:trailing_bytes"));
                }
                "core:metadata_only" if *v != Value::Bool(false) => {
                    return Err(SigmfError::Unsupported("core:metadata_only"));
                }
                _ => meta.other.push((name.clone(), v.clone())),
            }
        }
        meta.datatype = datatype.ok_or(SigmfError::Missing("core:datatype"))?;
        // The specification makes the rate optional; a receiver cannot use a recording
        // without it.
        meta.sample_rate_hz = rate.ok_or(SigmfError::Missing("core:sample_rate"))?;

        meta.version = version.ok_or(SigmfError::Missing("core:version"))?;
        for (name, v) in members(&top, "top level")? {
            match (name.as_str(), v) {
                ("global", _) => {}
                ("captures", Value::Array(items)) => {
                    for item in items {
                        let mut s = Segment::default();
                        let mut start = None;
                        for (k, f) in members(item, "captures")? {
                            match k.as_str() {
                                "core:sample_start" => start = Some(count(f, "core:sample_start")?),
                                "core:frequency" => {
                                    s.frequency_hz = Some(real(f, "core:frequency")?)
                                }
                                "core:datetime" => s.datetime = Some(text(f, "core:datetime")?),
                                "core:header_bytes" if f.as_u64() != Some(0) => {
                                    return Err(SigmfError::Unsupported("core:header_bytes"));
                                }
                                _ => s.other.push((k.clone(), f.clone())),
                            }
                        }
                        s.sample_start = start.ok_or(SigmfError::Missing("core:sample_start"))?;
                        meta.captures.push(s);
                    }
                }
                ("annotations", Value::Array(items)) => {
                    for item in items {
                        let mut a = Annotation::default();
                        let mut start = None;
                        for (k, f) in members(item, "annotations")? {
                            match k.as_str() {
                                "core:sample_start" => start = Some(count(f, "core:sample_start")?),
                                "core:sample_count" => {
                                    a.sample_count = Some(count(f, "core:sample_count")?)
                                }
                                "core:label" => a.label = Some(text(f, "core:label")?),
                                "core:comment" => a.comment = Some(text(f, "core:comment")?),
                                _ => a.other.push((k.clone(), f.clone())),
                            }
                        }
                        a.sample_start = start.ok_or(SigmfError::Missing("core:sample_start"))?;
                        meta.annotations.push(a);
                    }
                }
                ("captures", _) => return Err(SigmfError::Field("captures")),
                ("annotations", _) => return Err(SigmfError::Field("annotations")),
                _ => meta.other_top.push((name.clone(), v.clone())),
            }
        }
        meta.validate()?;
        Ok(meta)
    }

    /// Whether the metadata can be written and read back: a finite, positive sample rate,
    /// finite frequencies, capture segments in order of `sample_start`.
    pub fn validate(&self) -> Result<(), SigmfError> {
        if !(self.sample_rate_hz.is_finite() && self.sample_rate_hz > 0.0) {
            return Err(SigmfError::Field("core:sample_rate"));
        }
        if self
            .captures
            .iter()
            .any(|s| s.frequency_hz.is_some_and(|f| !f.is_finite()))
        {
            return Err(SigmfError::Field("core:frequency"));
        }
        if self
            .captures
            .windows(2)
            .any(|w| w[0].sample_start > w[1].sample_start)
        {
            return Err(SigmfError::Field(
                "captures (not in order of core:sample_start)",
            ));
        }
        Ok(())
    }

    /// The text of the `.sigmf-meta` file: typed fields first, in a fixed order, then the
    /// fields kept as read.
    pub fn to_json(&self) -> String {
        let string = |s: &String| Value::String(s.clone());
        let mut global = vec![
            (
                "core:datatype".to_string(),
                Value::String(self.datatype.name().into()),
            ),
            ("core:sample_rate".to_string(), number(self.sample_rate_hz)),
            ("core:version".to_string(), string(&self.version)),
        ];
        for (name, v) in [
            ("core:sha512", &self.sha512),
            ("core:description", &self.description),
            ("core:hw", &self.hw),
            ("core:author", &self.author),
        ] {
            if let Some(v) = v {
                global.push((name.to_string(), string(v)));
            }
        }
        global.extend(self.other.iter().cloned());
        let captures = self
            .captures
            .iter()
            .map(|s| {
                let mut m = vec![(
                    "core:sample_start".to_string(),
                    Value::from_u64(s.sample_start),
                )];
                if let Some(f) = s.frequency_hz {
                    m.push(("core:frequency".to_string(), number(f)));
                }
                if let Some(t) = &s.datetime {
                    m.push(("core:datetime".to_string(), string(t)));
                }
                m.extend(s.other.iter().cloned());
                Value::Object(m)
            })
            .collect();
        let annotations = self
            .annotations
            .iter()
            .map(|a| {
                let mut m = vec![(
                    "core:sample_start".to_string(),
                    Value::from_u64(a.sample_start),
                )];
                if let Some(n) = a.sample_count {
                    m.push(("core:sample_count".to_string(), Value::from_u64(n)));
                }
                if let Some(t) = &a.label {
                    m.push(("core:label".to_string(), string(t)));
                }
                if let Some(t) = &a.comment {
                    m.push(("core:comment".to_string(), string(t)));
                }
                m.extend(a.other.iter().cloned());
                Value::Object(m)
            })
            .collect();
        let mut top = vec![
            ("global".to_string(), Value::Object(global)),
            ("captures".to_string(), Value::Array(captures)),
            ("annotations".to_string(), Value::Array(annotations)),
        ];
        top.extend(self.other_top.iter().cloned());
        Value::Object(top).to_pretty()
    }
}

/// The dataset file that belongs to a metadata file: `<name>.sigmf-data` beside
/// `<name>.sigmf-meta`. Refuses any other metadata name (so that the dataset can never be the
/// metadata file itself, and a dotted name is not cut short).
pub fn data_path(meta_path: &Path) -> Result<PathBuf, SigmfError> {
    let name = meta_path.to_string_lossy();
    match name.strip_suffix(".sigmf-meta") {
        Some(stem) if !stem.is_empty() && !stem.ends_with(['/', '\\']) => {
            Ok(PathBuf::from(format!("{stem}.sigmf-data")))
        }
        _ => Err(SigmfError::Name(name.into_owned())),
    }
}

/// A recording held in memory: metadata and the dataset's bytes.
#[derive(Debug, Clone, PartialEq)]
pub struct Recording {
    /// The metadata.
    pub meta: Meta,
    /// The dataset, as in the file.
    pub data: Vec<u8>,
}

impl Recording {
    /// A `ci16_le` recording of `samples` (I, Q).
    pub fn from_i16(mut meta: Meta, samples: &[(i16, i16)]) -> Recording {
        meta.datatype = Datatype::Ci16Le;
        let mut data = Vec::with_capacity(samples.len() * 4);
        for (i, q) in samples {
            data.extend_from_slice(&i.to_le_bytes());
            data.extend_from_slice(&q.to_le_bytes());
        }
        Recording { meta, data }
    }

    /// Number of complex samples.
    pub fn len(&self) -> usize {
        self.data.len() / self.meta.datatype.sample_bytes()
    }

    /// Whether the recording has no samples.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The samples of a `ci16_le` recording; `None` for the other formats.
    pub fn to_i16(&self) -> Option<Vec<(i16, i16)>> {
        (self.meta.datatype == Datatype::Ci16Le).then(|| {
            self.data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|c| {
                    (
                        i16::from_le_bytes([c[0], c[1]]),
                        i16::from_le_bytes([c[2], c[3]]),
                    )
                })
                .collect()
        })
    }

    /// The samples as doubles in the file's own units: the integer value for `ci16_le` and
    /// `ci8`, the value for `cf32_le`, the byte less 127.5 for `cu8`.
    pub fn to_f64(&self) -> Vec<(f64, f64)> {
        decode(self.meta.datatype, &self.data)
    }

    /// Reads a recording: `meta_path` and the dataset beside it. The dataset must be whole
    /// samples and, if the metadata has `core:sha512`, must have that digest.
    pub fn read(meta_path: &Path) -> Result<Recording, SigmfError> {
        let path = data_path(meta_path)?;
        let text = std::fs::read_to_string(meta_path).map_err(|e| io("read", meta_path, e))?;
        let meta = Meta::from_json(&text)?;
        let data = std::fs::read(&path).map_err(|e| io("read", &path, e))?;
        check(&meta, data.len() as u64, || {
            let mut h = Sha512::new();
            h.update(&data);
            hex(&h.finalize())
        })?;
        Ok(Recording { meta, data })
    }

    /// Writes the recording as `meta_path` and the dataset beside it, with the dataset's
    /// SHA-512 in the metadata. Returns the metadata as written. Nothing is written unless the
    /// recording could be read back ([`Meta::validate`], whole samples). The dataset is written
    /// first: after a crash in between, the old metadata's checksum no longer matches.
    pub fn write(&self, meta_path: &Path) -> Result<Meta, SigmfError> {
        let path = data_path(meta_path)?;
        self.meta.validate()?;
        if !self
            .data
            .len()
            .is_multiple_of(self.meta.datatype.sample_bytes())
        {
            return Err(SigmfError::Length(self.data.len() as u64));
        }
        let mut meta = self.meta.clone();
        let mut h = Sha512::new();
        h.update(&self.data);
        meta.sha512 = Some(hex(&h.finalize()));
        File::create(&path)
            .and_then(|mut f| f.write_all(&self.data))
            .map_err(|e| io("write", &path, e))?;
        File::create(meta_path)
            .and_then(|mut f| f.write_all(meta.to_json().as_bytes()))
            .map_err(|e| io("write", meta_path, e))?;
        Ok(meta)
    }
}

fn check(meta: &Meta, bytes: u64, digest: impl FnOnce() -> String) -> Result<(), SigmfError> {
    if !bytes.is_multiple_of(meta.datatype.sample_bytes() as u64) {
        return Err(SigmfError::Length(bytes));
    }
    if let Some(expected) = &meta.sha512 {
        let found = digest();
        if !found.eq_ignore_ascii_case(expected) {
            return Err(SigmfError::Checksum {
                expected: expected.clone(),
                found,
            });
        }
    }
    Ok(())
}

fn decode(datatype: Datatype, bytes: &[u8]) -> Vec<(f64, f64)> {
    match datatype {
        Datatype::Ci16Le => bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|c| {
                (
                    f64::from(i16::from_le_bytes([c[0], c[1]])),
                    f64::from(i16::from_le_bytes([c[2], c[3]])),
                )
            })
            .collect(),
        Datatype::Cf32Le => bytes
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| {
                (
                    f64::from(f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
                    f64::from(f32::from_le_bytes([c[4], c[5], c[6], c[7]])),
                )
            })
            .collect(),
        Datatype::Ci8 => bytes
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| (f64::from(c[0] as i8), f64::from(c[1] as i8)))
            .collect(),
        Datatype::Cu8 => bytes
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| (f64::from(c[0]) - 127.5, f64::from(c[1]) - 127.5))
            .collect(),
    }
}

/// A dataset read in pieces, for recordings too large to hold (20 MS/s is 80 MB per second).
#[derive(Debug)]
pub struct Reader {
    /// The recording's metadata.
    pub meta: Meta,
    file: File,
    hash: Sha512,
    bytes: u64,
    done: bool,
    failed: Option<SigmfError>,
}

/// Samples read from the file per call at most, whatever the caller asks for.
const MAX_CHUNK: usize = 1 << 20;

impl Reader {
    /// Opens the recording at `meta_path`.
    pub fn open(meta_path: &Path) -> Result<Reader, SigmfError> {
        let path = data_path(meta_path)?;
        let text = std::fs::read_to_string(meta_path).map_err(|e| io("read", meta_path, e))?;
        let meta = Meta::from_json(&text)?;
        let file = File::open(&path).map_err(|e| io("open", &path, e))?;
        Ok(Reader {
            meta,
            file,
            hash: Sha512::new(),
            bytes: 0,
            done: false,
            failed: None,
        })
    }

    /// The next samples, at most `max` and at most 2²⁰ (in the units of
    /// [`Recording::to_f64`]); an empty vector at the end. The checks of [`Recording::read`]
    /// are made when the end is reached: a recording is verified only once it has been read
    /// through. After an error every further call returns that error.
    pub fn next_chunk(&mut self, max: usize) -> Result<Vec<(f64, f64)>, SigmfError> {
        if let Some(e) = &self.failed {
            return Err(e.clone());
        }
        let result = self.read_chunk(max.min(MAX_CHUNK));
        if let Err(e) = &result {
            self.failed = Some(e.clone());
        }
        result
    }

    fn read_chunk(&mut self, max: usize) -> Result<Vec<(f64, f64)>, SigmfError> {
        if self.done || max == 0 {
            return Ok(Vec::new());
        }
        let size = self.meta.datatype.sample_bytes();
        let mut buf = vec![0u8; max * size];
        let mut got = 0;
        while got < buf.len() {
            match self.file.read(&mut buf[got..]) {
                Ok(0) => break,
                Ok(n) => got += n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) => return Err(SigmfError::Io(format!("read dataset: {e}"))),
            }
        }
        buf.truncate(got);
        self.hash.update(&buf);
        self.bytes += got as u64;
        if got < max * size {
            self.done = true;
            let hash = std::mem::take(&mut self.hash);
            check(&self.meta, self.bytes, || hex(&hash.finalize()))?;
        }
        Ok(decode(self.meta.datatype, &buf[..got - got % size]))
    }
}
