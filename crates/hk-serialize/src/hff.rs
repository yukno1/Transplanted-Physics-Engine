//! HFF structured stream parser.
//!
//! Rust port of `hkSerialize::HffReaderBase` from
//! `original/Common/Source/Common/Base/System/Io/Structured/hkStructuredStream.h`.
//!
//! Every HFF section starts with two big-endian 32-bit integers:
//!
//! * `packed`: top 2 bits = scope type (`0` branch, `1` leaf), lower 30 bits =
//!   total section size in bytes (including the 8-byte header);
//! * `ident`: a 4CC / tag identifying the section.
//!
//! Branch contents are nested sections; leaf contents are opaque bytes. The
//! parser builds a tree and copies leaf payloads. This is the container layer
//! only: reflection-driven object rebuilding is not ported.

/// Pack the scope type and size into the HFF header word.
#[inline]
pub const fn pack_scope(scope: u32, size: u32) -> u32 {
    (scope << 30) | (size & 0x3FFF_FFFF)
}

/// Scope type encoded in the top two bits of the HFF header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HffScope {
    /// Branch holding nested sections.
    Branch,
    /// Leaf holding opaque payload bytes.
    Leaf,
}

/// A parsed HFF section.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HffNode {
    /// FourCC identifier.
    pub ident: u32,
    /// Section kind.
    pub scope: HffScope,
    /// Leaf payload (empty for branches).
    pub data: Vec<u8>,
    /// Nested sections (empty for leaves).
    pub children: Vec<HffNode>,
}

impl HffNode {
    /// Whether the section is a leaf.
    pub fn is_leaf(&self) -> bool {
        self.scope == HffScope::Leaf
    }

    /// Printable FourCC name (`TAG0`, `DATA`, …); non-printable bytes become `.`.
    pub fn name(&self) -> String {
        ident_name(self.ident)
    }

    /// Find a direct child by identifier.
    pub fn find(&self, ident: u32) -> Option<&HffNode> {
        self.children.iter().find(|child| child.ident == ident)
    }

    /// Find a child by FourCC name.
    pub fn find_name(&self, name: &[u8; 4]) -> Option<&HffNode> {
        self.find(four_cc(name))
    }
}

/// Errors produced by the HFF parser.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HffError {
    /// Input ended before a complete section header.
    TruncatedHeader {
        /// Byte offset of the incomplete header.
        offset: usize,
    },
    /// Declared section size is smaller than the 8-byte header.
    SectionTooSmall {
        /// Byte offset of the section.
        offset: usize,
        /// Declared size.
        size: usize,
    },
    /// The top two bits are not `0` (branch) or `1` (leaf).
    InvalidScopeType {
        /// Byte offset of the section.
        offset: usize,
        /// Encoded scope type.
        scope: u32,
    },
    /// A section extends past its parent/file bounds.
    SectionOutOfBounds {
        /// Byte offset of the section.
        offset: usize,
        /// Declared size.
        size: usize,
        /// Available bytes.
        available: usize,
    },
    /// Trailing bytes remain after the root section.
    TrailingData {
        /// Number of bytes left after the root.
        remaining: usize,
    },
}

impl core::fmt::Display for HffError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TruncatedHeader { offset } => {
                write!(f, "truncated HFF header at byte {offset}")
            }
            Self::SectionTooSmall { offset, size } => {
                write!(f, "HFF section at {offset} is too small ({size} bytes)")
            }
            Self::InvalidScopeType { offset, scope } => {
                write!(f, "invalid HFF scope type {scope} at byte {offset}")
            }
            Self::SectionOutOfBounds {
                offset,
                size,
                available,
            } => write!(
                f,
                "HFF section at {offset} has size {size}, only {available} bytes available"
            ),
            Self::TrailingData { remaining } => {
                write!(f, "{remaining} trailing bytes after the HFF root")
            }
        }
    }
}

impl std::error::Error for HffError {}

/// Parse an HFF file into its section tree.
///
/// The root section must cover the whole input; trailing bytes are rejected.
pub fn parse(bytes: &[u8]) -> Result<HffNode, HffError> {
    let (root, consumed) = parse_section(bytes, 0)?;
    if consumed != bytes.len() {
        return Err(HffError::TrailingData {
            remaining: bytes.len() - consumed,
        });
    }
    Ok(root)
}

fn parse_section(bytes: &[u8], offset: usize) -> Result<(HffNode, usize), HffError> {
    let available = bytes.len().saturating_sub(offset);
    if available < 8 {
        return Err(HffError::TruncatedHeader { offset });
    }
    let packed = read_u32_be(bytes, offset);
    let ident = read_u32_be(bytes, offset + 4);
    let size = (packed & 0x3FFF_FFFF) as usize;
    let scope = packed >> 30;

    if size < 8 {
        return Err(HffError::SectionTooSmall { offset, size });
    }
    if size > available {
        return Err(HffError::SectionOutOfBounds {
            offset,
            size,
            available,
        });
    }
    let end = offset + size;

    match scope {
        0 => {
            let mut children = Vec::new();
            let mut cursor = offset + 8;
            while cursor < end {
                let (child, child_end) = parse_section(bytes, cursor)?;
                if child_end > end {
                    return Err(HffError::SectionOutOfBounds {
                        offset: cursor,
                        size: child_end - cursor,
                        available: end - cursor,
                    });
                }
                children.push(child);
                cursor = child_end;
            }
            if cursor != end {
                return Err(HffError::TruncatedHeader { offset: cursor });
            }
            Ok((
                HffNode {
                    ident,
                    scope: HffScope::Branch,
                    data: Vec::new(),
                    children,
                },
                end,
            ))
        }
        1 => Ok((
            HffNode {
                ident,
                scope: HffScope::Leaf,
                data: bytes[offset + 8..end].to_vec(),
                children: Vec::new(),
            },
            end,
        )),
        other => Err(HffError::InvalidScopeType {
            offset,
            scope: other,
        }),
    }
}

#[inline]
fn read_u32_be(bytes: &[u8], offset: usize) -> u32 {
    u32::from_be_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

/// Build a FourCC identifier from four bytes (`FOUR_CC`).
#[inline]
pub const fn four_cc(bytes: &[u8; 4]) -> u32 {
    (bytes[0] as u32) << 24 | (bytes[1] as u32) << 16 | (bytes[2] as u32) << 8 | bytes[3] as u32
}

/// Render a FourCC identifier; non-printable bytes become `.`.
pub fn ident_name(ident: u32) -> String {
    let mut output = String::with_capacity(4);
    for shift in [24, 16, 8, 0] {
        let byte = ((ident >> shift) & 0xFF) as u8;
        output.push(if byte.is_ascii_graphic() || byte == b' ' {
            char::from(byte)
        } else {
            '.'
        });
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section(scope: u32, ident: &[u8; 4], payload_or_children: &[Vec<u8>]) -> Vec<u8> {
        let size = 8 + payload_or_children.iter().map(Vec::len).sum::<usize>();
        let mut bytes = Vec::with_capacity(size);
        bytes.extend_from_slice(&pack_scope(scope, size as u32).to_be_bytes());
        bytes.extend_from_slice(&four_cc(ident).to_be_bytes());
        for part in payload_or_children {
            bytes.extend_from_slice(part);
        }
        bytes
    }

    fn leaf(ident: &[u8; 4], payload: &[u8]) -> Vec<u8> {
        section(1, ident, &[payload.to_vec()])
    }

    fn branch(ident: &[u8; 4], children: &[Vec<u8>]) -> Vec<u8> {
        section(0, ident, children)
    }

    #[test]
    fn four_cc_roundtrip() {
        assert_eq!(four_cc(b"TAG0"), 0x5441_4730);
        assert_eq!(four_cc(b"DATA"), 0x4441_5441);
        assert_eq!(ident_name(four_cc(b"TAG0")), "TAG0");
        assert_eq!(ident_name(0x0102_0304), "....");
    }

    #[test]
    fn parses_nested_branches_and_leaf_payloads() {
        let file = branch(
            b"TAG0",
            &[
                leaf(b"SDKV", b"20211202"),
                branch(b"DATA", &[leaf(b"TEST", b"hello")]),
            ],
        );
        let root = parse(&file).expect("parse");
        assert_eq!(root.ident, four_cc(b"TAG0"));
        assert!(!root.is_leaf());
        assert_eq!(root.name(), "TAG0");
        let version = root.find_name(b"SDKV").expect("version leaf");
        assert!(version.is_leaf());
        assert_eq!(version.data, b"20211202");
        let data = root.find_name(b"DATA").expect("data branch");
        assert_eq!(data.children.len(), 1);
        assert_eq!(data.children[0].data, b"hello");
    }

    #[test]
    fn rejects_truncated_file() {
        let mut file = branch(b"TAG0", &[leaf(b"SDKV", b"2021")]);
        file.pop();
        assert!(matches!(
            parse(&file),
            Err(HffError::SectionOutOfBounds { .. })
        ));

        assert_eq!(parse(&[]), Err(HffError::TruncatedHeader { offset: 0 }));
        assert_eq!(
            parse(&[0, 0, 0, 4, 0, 0, 0, 0]),
            Err(HffError::SectionTooSmall { offset: 0, size: 4 })
        );
    }

    #[test]
    fn rejects_invalid_scope_and_trailing_data() {
        // Header-only section with scope type 2.
        let mut bytes = pack_scope(2, 8).to_be_bytes().to_vec();
        bytes.extend_from_slice(&four_cc(b"TAG0").to_be_bytes());
        assert_eq!(
            parse(&bytes),
            Err(HffError::InvalidScopeType {
                offset: 0,
                scope: 2
            })
        );

        let mut trailing = leaf(b"SDKV", b"2021");
        trailing.push(0);
        assert_eq!(
            parse(&trailing),
            Err(HffError::TrailingData { remaining: 1 })
        );
    }

    #[test]
    fn empty_leaf_is_valid() {
        let file = leaf(b"EMPT", b"");
        let root = parse(&file).expect("parse");
        assert!(root.is_leaf());
        assert!(root.data.is_empty());
    }
}
