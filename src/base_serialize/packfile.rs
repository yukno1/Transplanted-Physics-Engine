//! 2014 binary packfile header parsing and conversion boundary.
//!
//! The SDK distribution only contains the 16-byte header detector and an
//! optional-component hook that binds `hkCompatFormats.dll`. The DLL converts a
//! packfile into a legacy `CAB00D1E` tagfile stream; the actual container
//! parser is not in the source tree. This module keeps that boundary explicit:
//! it validates the header and delegates conversion to a caller-supplied
//! [`PackfileConverter`], which must return bytes the existing readers accept.

use crate::base_serialize::version::{
    BINARY_PACKFILE_MAGIC_0, BINARY_PACKFILE_MAGIC_1, SerializeFormat, detect_format,
};

/// Size of the `BinaryPackfile2014` header in bytes.
pub const HEADER_LEN: usize = 16;
/// File version currently written by the SDK (`BinaryPackfile2014::m_fileVersion`).
pub const CURRENT_FILE_VERSION: i32 = 11;

/// Errors produced while parsing or converting a 2014 binary packfile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PackfileError {
    /// Input was shorter than the 16-byte header.
    ShortHeader {
        /// Available byte count.
        len: usize,
    },
    /// The two header magic words did not match.
    BadMagic,
    /// A caller-supplied policy rejected the stored user tag.
    UnexpectedUserTag {
        /// Stored value.
        actual: i32,
        /// Required value.
        expected: i32,
    },
    /// A caller-supplied policy rejected the stored file version.
    UnexpectedFileVersion {
        /// Stored value.
        actual: i32,
        /// Required value.
        expected: i32,
    },
    /// No converter was supplied for a recognised packfile.
    ConverterMissing,
    /// The external converter reported a failure.
    ConversionFailed(String),
    /// The converter output was not a tagfile container.
    ConvertedFormatNotTagfile(SerializeFormat),
}

impl core::fmt::Display for PackfileError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ShortHeader { len } => {
                write!(f, "packfile header needs {HEADER_LEN} bytes, got {len}")
            }
            Self::BadMagic => write!(f, "invalid binary packfile magic"),
            Self::UnexpectedUserTag { actual, expected } => {
                write!(f, "packfile user tag {actual}, expected {expected}")
            }
            Self::UnexpectedFileVersion { actual, expected } => {
                write!(f, "packfile version {actual}, expected {expected}")
            }
            Self::ConverterMissing => write!(
                f,
                "binary packfile conversion requires an external converter"
            ),
            Self::ConversionFailed(reason) => write!(f, "packfile conversion failed: {reason}"),
            Self::ConvertedFormatNotTagfile(format) => {
                write!(
                    f,
                    "packfile converter produced non-tagfile format {format:?}"
                )
            }
        }
    }
}

impl std::error::Error for PackfileError {}

/// Parsed `BinaryPackfile2014` header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PackfileHeader {
    /// Byte-symmetric magic words from the file.
    pub magic: [u32; 2],
    /// Application-defined tag; the SDK detector does not interpret it.
    pub user_tag: i32,
    /// Packfile format version; the SDK comments call out version 11.
    pub file_version: i32,
}

impl PackfileHeader {
    /// Parse a header without applying optional policy checks.
    pub fn parse(bytes: &[u8]) -> Result<Self, PackfileError> {
        if bytes.len() < HEADER_LEN {
            return Err(PackfileError::ShortHeader { len: bytes.len() });
        }
        let magic = [
            u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        ];
        if magic != [BINARY_PACKFILE_MAGIC_0, BINARY_PACKFILE_MAGIC_1] {
            return Err(PackfileError::BadMagic);
        }
        Ok(Self {
            magic,
            user_tag: i32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            file_version: i32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
        })
    }

    /// Whether this is the file version currently emitted by the SDK.
    pub fn is_current_version(&self) -> bool {
        self.file_version == CURRENT_FILE_VERSION
    }

    /// Apply caller-supplied header constraints.
    pub fn validate(&self, policy: &PackfilePolicy) -> Result<(), PackfileError> {
        if let Some(expected) = policy.expected_user_tag {
            if self.user_tag != expected {
                return Err(PackfileError::UnexpectedUserTag {
                    actual: self.user_tag,
                    expected,
                });
            }
        }
        if let Some(expected) = policy.expected_file_version {
            if self.file_version != expected {
                return Err(PackfileError::UnexpectedFileVersion {
                    actual: self.file_version,
                    expected,
                });
            }
        }
        Ok(())
    }
}

/// Optional header constraints for [`PackfileHeader::validate`].
///
/// The SDK detector only checks magic words, so all constraints are opt-in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PackfilePolicy {
    /// Required user tag, when the caller knows it.
    pub expected_user_tag: Option<i32>,
    /// Required file version, when the caller knows it.
    pub expected_file_version: Option<i32>,
}

impl PackfilePolicy {
    /// Policy accepting only the SDK's current file version.
    pub fn current_sdk() -> Self {
        Self {
            expected_user_tag: None,
            expected_file_version: Some(CURRENT_FILE_VERSION),
        }
    }
}

/// External conversion boundary mirroring `hkCompatFormats.dll`.
///
/// Implementations must return a `TAG0` or legacy `CAB00D1E` byte stream; the
/// loader then uses the existing tagfile readers. No converter is built in.
pub trait PackfileConverter {
    /// Convert `bytes` using the already parsed `header`.
    fn convert(&self, header: &PackfileHeader, bytes: &[u8]) -> Result<Vec<u8>, PackfileError>;
}

/// Parse, validate and convert a packfile into a tagfile byte stream.
pub fn convert_packfile(
    bytes: &[u8],
    policy: &PackfilePolicy,
    converter: Option<&dyn PackfileConverter>,
) -> Result<(PackfileHeader, Vec<u8>), PackfileError> {
    let header = PackfileHeader::parse(bytes)?;
    header.validate(policy)?;
    let converter = converter.ok_or(PackfileError::ConverterMissing)?;
    let converted = converter.convert(&header, bytes)?;
    match detect_format(&converted) {
        SerializeFormat::BinaryTagfile | SerializeFormat::LegacyBinaryTagfile => {}
        other => return Err(PackfileError::ConvertedFormatNotTagfile(other)),
    }
    Ok((header, converted))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header_bytes(version: i32) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(HEADER_LEN);
        bytes.extend_from_slice(&BINARY_PACKFILE_MAGIC_0.to_le_bytes());
        bytes.extend_from_slice(&BINARY_PACKFILE_MAGIC_1.to_le_bytes());
        bytes.extend_from_slice(&0x1234_i32.to_le_bytes());
        bytes.extend_from_slice(&version.to_le_bytes());
        bytes
    }

    struct IdentityConverter;

    impl PackfileConverter for IdentityConverter {
        fn convert(
            &self,
            _header: &PackfileHeader,
            _bytes: &[u8],
        ) -> Result<Vec<u8>, PackfileError> {
            Ok(b"\0\0\0\0TAG0".to_vec())
        }
    }

    #[test]
    fn parses_header_fields_and_checks_current_version() {
        let header = PackfileHeader::parse(&header_bytes(CURRENT_FILE_VERSION)).expect("header");
        assert_eq!(header.user_tag, 0x1234);
        assert_eq!(header.file_version, CURRENT_FILE_VERSION);
        assert!(header.is_current_version());
    }

    #[test]
    fn rejects_short_header_and_bad_magic() {
        assert_eq!(
            PackfileHeader::parse(&[0; 15]),
            Err(PackfileError::ShortHeader { len: 15 })
        );
        assert_eq!(
            PackfileHeader::parse(&[0; HEADER_LEN]),
            Err(PackfileError::BadMagic)
        );
    }

    #[test]
    fn policy_rejects_mismatched_tag_or_version() {
        let header = PackfileHeader::parse(&header_bytes(CURRENT_FILE_VERSION)).expect("header");
        assert_eq!(
            header.validate(&PackfilePolicy {
                expected_user_tag: Some(7),
                expected_file_version: None,
            }),
            Err(PackfileError::UnexpectedUserTag {
                actual: 0x1234,
                expected: 7,
            })
        );
        assert_eq!(header.validate(&PackfilePolicy::current_sdk()), Ok(()));
    }

    #[test]
    fn converter_is_required_and_output_must_be_tagfile() {
        let bytes = header_bytes(CURRENT_FILE_VERSION);
        assert_eq!(
            convert_packfile(&bytes, &PackfilePolicy::default(), None).map(|_| ()),
            Err(PackfileError::ConverterMissing)
        );
        let (header, _) =
            convert_packfile(&bytes, &PackfilePolicy::default(), Some(&IdentityConverter))
                .expect("converted");
        assert_eq!(header, PackfileHeader::parse(&bytes).expect("header"));
    }
}
