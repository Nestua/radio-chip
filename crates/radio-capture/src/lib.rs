//! IQ capture files for the SDR and field stages (R1, G6): SigMF recordings, read and written
//! without any dependency.
//!
//! A recording is two files, `<name>.sigmf-meta` (JSON) and `<name>.sigmf-data` (raw samples),
//! as the SigMF specification v1.2 defines them. [`sigmf`] holds the recording, [`json`] the
//! small JSON reader and writer it needs, [`sha512`] the checksum SigMF prescribes
//! (`core:sha512`). Definitions and evidence: `docs/gates/capture.md`.

pub mod json;
pub mod sha512;
pub mod sigmf;
