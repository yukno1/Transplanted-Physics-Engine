//! Tagfile container reader.
//!
//! Rust port of the container layout from
//! `original/Common/Source/Common/Serialize/Format/Tagfile/Detail/hkTagfileDetail.h`.
//!
//! A tagfile is an HFF tree whose root is `TAG0` (normal bundle) or `TCM0`
//! (type compendium). Children may include `SDKV` (SDK version string), `DATA`
//! (object data), `TYPE` (type declarations) and `INDX` (object/fixup index).
//! This module exposes that tree; reflection-driven object rebuilding, patch
//! fixups and bundle/compendium semantics are not ported.

use crate::hff::{self, HffError, HffNode, four_cc};

/// Normal tagfile root (`TAG0`).
pub const TAGFILE: u32 = four_cc(b"TAG0");
/// Type-compendium root (`TCM0`).
pub const COMPENDIUM: u32 = four_cc(b"TCM0");
/// SDK version leaf (`SDKV`).
pub const SDK_VERSION: u32 = four_cc(b"SDKV");
/// Main object data branch (`DATA`).
pub const DATA: u32 = four_cc(b"DATA");
/// Type declarations branch (`TYPE`).
pub const TYPES: u32 = four_cc(b"TYPE");
/// Object/fixup index branch (`INDX`).
pub const INDEX: u32 = four_cc(b"INDX");

/// Errors produced by [`Tagfile::parse`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TagfileError {
    /// Underlying HFF parse error.
    Hff(HffError),
    /// Root section is neither `TAG0` nor `TCM0`.
    NotATagfile {
        /// Root identifier.
        ident: u32,
    },
}

impl From<HffError> for TagfileError {
    fn from(value: HffError) -> Self {
        Self::Hff(value)
    }
}

impl core::fmt::Display for TagfileError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Hff(error) => write!(f, "HFF error: {error}"),
            Self::NotATagfile { ident } => {
                write!(f, "root section {:08x} is not a tagfile", ident)
            }
        }
    }
}

impl std::error::Error for TagfileError {}

/// Parsed tagfile container.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Tagfile {
    /// Full HFF section tree.
    pub root: HffNode,
    /// Decoded `SDKV` value (e.g. `20211202`), when present.
    pub version: Option<String>,
}

impl Tagfile {
    /// Parse a tagfile byte stream.
    pub fn parse(bytes: &[u8]) -> Result<Self, TagfileError> {
        let root = hff::parse(bytes)?;
        if root.ident != TAGFILE && root.ident != COMPENDIUM {
            return Err(TagfileError::NotATagfile { ident: root.ident });
        }
        let version = root.find(SDK_VERSION).map(|section| {
            String::from_utf8_lossy(&section.data)
                .trim_end_matches('\0')
                .to_string()
        });
        Ok(Self { root, version })
    }

    /// Find a direct section by identifier.
    pub fn section(&self, ident: u32) -> Option<&HffNode> {
        self.root.find(ident)
    }

    /// The `DATA` section, if present.
    pub fn data(&self) -> Option<&HffNode> {
        self.section(DATA)
    }

    /// The `TYPE` section, if present.
    pub fn types(&self) -> Option<&HffNode> {
        self.section(TYPES)
    }

    /// The `INDX` section, if present.
    pub fn index(&self) -> Option<&HffNode> {
        self.section(INDEX)
    }

    /// Whether this is a compendium (`TCM0`) rather than a normal bundle.
    pub fn is_compendium(&self) -> bool {
        self.root.ident == COMPENDIUM
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hff::pack_scope;

    fn section(scope: u32, ident: u32, parts: &[Vec<u8>]) -> Vec<u8> {
        let size = 8 + parts.iter().map(Vec::len).sum::<usize>();
        let mut bytes = Vec::with_capacity(size);
        bytes.extend_from_slice(&pack_scope(scope, size as u32).to_be_bytes());
        bytes.extend_from_slice(&ident.to_be_bytes());
        for part in parts {
            bytes.extend_from_slice(part);
        }
        bytes
    }

    fn leaf(ident: u32, payload: &[u8]) -> Vec<u8> {
        section(1, ident, &[payload.to_vec()])
    }

    fn branch(ident: u32, children: &[Vec<u8>]) -> Vec<u8> {
        section(0, ident, children)
    }

    fn sample_tagfile() -> Vec<u8> {
        branch(
            TAGFILE,
            &[
                leaf(SDK_VERSION, b"20211202"),
                branch(DATA, &[leaf(four_cc(b"TEST"), b"objects")]),
                branch(TYPES, &[leaf(four_cc(b"TNA1"), b"types")]),
                branch(INDEX, &[leaf(four_cc(b"ITEM"), b"index")]),
            ],
        )
    }

    #[test]
    fn parses_tagfile_sections_and_version() {
        let bytes = sample_tagfile();
        let tagfile = Tagfile::parse(&bytes).expect("tagfile");
        assert_eq!(tagfile.version.as_deref(), Some("20211202"));
        assert!(!tagfile.is_compendium());
        assert!(tagfile.data().is_some());
        assert!(tagfile.types().is_some());
        assert!(tagfile.index().is_some());
        assert_eq!(tagfile.section(DATA).unwrap().name(), "DATA");
        assert_eq!(tagfile.data().unwrap().children[0].data, b"objects");
    }

    #[test]
    fn compendium_root_is_accepted() {
        let bytes = branch(COMPENDIUM, &[leaf(SDK_VERSION, b"20211202")]);
        let tagfile = Tagfile::parse(&bytes).expect("compendium");
        assert!(tagfile.is_compendium());
        assert!(tagfile.data().is_none());
    }

    #[test]
    fn rejects_non_tagfile_root() {
        let bytes = branch(four_cc(b"XXXX"), &[leaf(SDK_VERSION, b"20211202")]);
        assert_eq!(
            Tagfile::parse(&bytes),
            Err(TagfileError::NotATagfile {
                ident: four_cc(b"XXXX")
            })
        );
    }

    #[test]
    fn propagates_hff_errors() {
        let mut bytes = sample_tagfile();
        bytes.pop();
        assert!(matches!(
            Tagfile::parse(&bytes),
            Err(TagfileError::Hff(HffError::SectionOutOfBounds { .. }))
        ));
    }

    #[test]
    fn version_is_optional_and_decoded() {
        let bytes = branch(TAGFILE, &[]);
        let tagfile = Tagfile::parse(&bytes).expect("empty tagfile");
        assert_eq!(tagfile.version, None);
        assert!(tagfile.data().is_none());
    }
}
