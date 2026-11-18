//! Reflection metadata stored in a tagfile `TYPE` section.
//!
//! Direct subset of `hkTagfileReadFormat::readType` /
//! `handleTypeIdentityImpl` from `hkTagfileReadFormat.cpp`.  Type names come
//! from `TST1`, field names from `FST1`, identities from `TNA1` and the encoded
//! type bodies from `TBDY`.  The V0 (`TNAM`/`TBOD`) variants are rejected.

use crate::vle::{self, VleError};

/// Identifier used by `TYPE`/`ITEM`/`PTCH` sections.  Index `0` is the null type.
pub type TypeId = u32;

/// `hkReflect::Kind` encoded in the low five bits of a format word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// `KIND_VOID`
    Void,
    /// `KIND_OPAQUE`
    Opaque,
    /// `KIND_BOOL`
    Bool,
    /// `KIND_STRING`
    String,
    /// `KIND_INT`
    Int,
    /// `KIND_FLOAT`
    Float,
    /// `KIND_POINTER`
    Pointer,
    /// `KIND_RECORD`
    Record,
    /// `KIND_ARRAY`
    Array,
}

/// Decoded `hkReflect::Format` value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Format {
    /// Raw encoded value, retained for diagnostics.
    pub raw: u32,
    /// Base kind.
    pub kind: Kind,
    /// Whether an integer is signed (`SIGNED_BIT`).
    pub signed: bool,
    /// Integer/bool width in bits (`INT_NUM_BITS_SHIFT`).
    pub bits: u32,
    /// Fixed element count for arrays (`format >> 8`), `0` for dynamic arrays.
    pub fixed_count: u32,
}

impl Format {
    /// Decode a raw format word, rejecting unknown kinds.
    pub fn from_raw(raw: u32) -> Result<Self, MetaError> {
        let kind = match raw & 0x1f {
            0 => Kind::Void,
            1 => Kind::Opaque,
            2 => Kind::Bool,
            3 => Kind::String,
            4 => Kind::Int,
            5 => Kind::Float,
            6 => Kind::Pointer,
            7 => Kind::Record,
            8 => Kind::Array,
            other => return Err(MetaError::UnsupportedKind(other)),
        };
        Ok(Self {
            raw,
            kind,
            signed: raw & (1 << 9) != 0,
            bits: raw >> 10,
            fixed_count: if kind == Kind::Array { raw >> 8 } else { 0 },
        })
    }

    /// Total float width implied by exponent/significand fields.
    pub fn float_bits(&self) -> u32 {
        1 + ((self.raw >> 11) & 0x1f) + ((self.raw >> 16) & 0xff)
    }
}

/// Errors produced while decoding reflection metadata.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MetaError {
    /// A VLE integer was malformed or truncated.
    Vle(VleError),
    /// A kind value is not part of `hkReflect::Kind`.
    UnsupportedKind(u32),
    /// An optional bit not listed in `s_optMapping` was present.
    InvalidOptBits(u32),
    /// An optional bit that this subset does not implement was present.
    UnsupportedOpt(u32),
    /// A string id fell outside the corresponding string table.
    StringIndexOutOfBounds {
        /// Table name for diagnostics.
        table: &'static str,
        /// Invalid index.
        index: u64,
        /// Table length.
        len: usize,
    },
    /// A type id fell outside the identity/body arrays.
    TypeIdOutOfBounds {
        /// Invalid id.
        id: u64,
        /// Number of known types.
        len: usize,
    },
    /// The same type body was encoded twice.
    DuplicateTypeBody(TypeId),
    /// A field did not have `DECL_DATA_FIELD` semantics.
    InvalidFieldFlags(u16),
    /// The `TBDY` padding was longer than three bytes or contained non-zero data.
    InvalidPadding,
    /// The section ended in the middle of a type body.
    TruncatedBody,
}

impl From<VleError> for MetaError {
    fn from(value: VleError) -> Self {
        Self::Vle(value)
    }
}

impl core::fmt::Display for MetaError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Vle(error) => write!(f, "VLE error: {error}"),
            Self::UnsupportedKind(kind) => write!(f, "unsupported reflect kind {kind}"),
            Self::InvalidOptBits(bits) => write!(f, "invalid type opt bits {bits:08x}"),
            Self::UnsupportedOpt(bits) => write!(f, "unsupported type opt bits {bits:08x}"),
            Self::StringIndexOutOfBounds { table, index, len } => {
                write!(
                    f,
                    "{table} string index {index} out of bounds ({len} strings)"
                )
            }
            Self::TypeIdOutOfBounds { id, len } => {
                write!(f, "type id {id} out of bounds ({len} types)")
            }
            Self::DuplicateTypeBody(id) => write!(f, "duplicate type body for id {id}"),
            Self::InvalidFieldFlags(flags) => write!(f, "invalid decl flags {flags:04x}"),
            Self::InvalidPadding => write!(f, "invalid type body padding"),
            Self::TruncatedBody => write!(f, "truncated type body"),
        }
    }
}

impl std::error::Error for MetaError {}

/// A template parameter attached to a type identity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TemplateParam {
    /// String id of the `t…`/`v…` kind-and-name descriptor.
    pub kind_name_id: u64,
    /// Type id for `t` params, numeric value for `v` params.
    pub value: u64,
}

/// Name and template parameters of a type, stored in `TNA1`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypeIdentity {
    /// Type name, or `None` for an unnamed type.
    pub name: Option<String>,
    /// Template parameters in file order.
    pub params: Vec<TemplateParam>,
}

/// A serialized data field (`DECL_DATA_FIELD`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Field {
    /// Field name from `FST1`.
    pub name: String,
    /// Byte offset inside the owning record.
    pub offset: u16,
    /// Type of the field value.
    pub type_id: TypeId,
    /// Raw `DeclFlags` value.
    pub flags: u16,
}

/// Decoded type body (`TBDY`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypeMeta {
    /// Self id.
    pub id: TypeId,
    /// Parent type id; `0` when there is none.
    pub parent: TypeId,
    /// Optional type name from the identity table.
    pub name: Option<String>,
    /// Optional format word.
    pub format: Option<Format>,
    /// Container element or pointer target type.
    pub subtype: Option<TypeId>,
    /// Declared version, when present.
    pub version: Option<u32>,
    /// Object size in bytes.
    pub size: u32,
    /// Object alignment in bytes.
    pub align: u32,
    /// Raw type flags.
    pub flags: u16,
    /// Data fields declared by this type (not inherited).
    pub fields: Vec<Field>,
    /// Interfaces declared by this type.
    pub interfaces: Vec<(TypeId, u32)>,
}

/// Complete metadata tables built from the `TYPE` section.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypeInfo {
    /// Type identities; index `0` is always `None` (null type).
    pub identities: Vec<Option<TypeIdentity>>,
    /// Type bodies; index `0` is always `None` (null type).
    pub types: Vec<Option<TypeMeta>>,
}

/// Split a string table into NUL-terminated strings.
///
/// Trailing bytes without a terminator are padding and are dropped, matching
/// `TagfileReadFormat::Impl::readStrings`.
pub fn parse_strings(data: &[u8]) -> Vec<String> {
    let mut strings = Vec::new();
    let mut start = 0;
    for (index, byte) in data.iter().enumerate() {
        if *byte == 0 {
            strings.push(String::from_utf8_lossy(&data[start..index]).into_owned());
            start = index + 1;
        }
    }
    strings
}

const OPT_MAPPING: [u32; 9] = [
    1 << 0,  // FORMAT
    1 << 1,  // SUBTYPE
    1 << 4,  // VERSION
    1 << 20, // SIZE_ALIGN
    1 << 21, // FLAGS
    1 << 23, // DECLS
    1 << 14, // INTERFACES
    1 << 25, // ATTRIBUTE_STRING
    1 << 28, // MUTABLE
];

const OPT_FORMAT: u32 = OPT_MAPPING[0];
const OPT_SUBTYPE: u32 = OPT_MAPPING[1];
const OPT_VERSION: u32 = OPT_MAPPING[2];
const OPT_SIZE_ALIGN: u32 = OPT_MAPPING[3];
const OPT_FLAGS: u32 = OPT_MAPPING[4];
const OPT_DECLS: u32 = OPT_MAPPING[5];
const OPT_INTERFACES: u32 = OPT_MAPPING[6];
const OPT_ATTRIBUTE_STRING: u32 = OPT_MAPPING[7];

const DECL_PROPERTY_FIELD: u16 = 0x10;
const DECL_DATA_FIELD: u16 = 0x20;
const DECL_ALIGN_FLAG: u16 = 0x80;

fn vle_u16(data: &[u8], cursor: &mut usize) -> Result<u16, MetaError> {
    let value = vle::read_at(data, cursor)?;
    u16::try_from(value).map_err(|_| MetaError::TruncatedBody)
}

fn vle_u32(data: &[u8], cursor: &mut usize) -> Result<u32, MetaError> {
    let value = vle::read_at(data, cursor)?;
    u32::try_from(value).map_err(|_| MetaError::TruncatedBody)
}

fn validate_type_id(id: u64, len: usize) -> Result<TypeId, MetaError> {
    if id == 0 || id >= len as u64 {
        return Err(MetaError::TypeIdOutOfBounds { id, len });
    }
    Ok(id as TypeId)
}

/// Parse `TNA1` type identities.
pub fn parse_identities(
    data: &[u8],
    strings: &[String],
) -> Result<Vec<Option<TypeIdentity>>, MetaError> {
    let mut cursor = 0;
    let next_num_types = vle_u32(data, &mut cursor)? as usize;
    if next_num_types == 0 {
        return Ok(vec![None]);
    }

    let mut identities: Vec<Option<TypeIdentity>> = (0..next_num_types).map(|_| None).collect();
    for slot in identities.iter_mut().skip(1) {
        let name_index = vle::read_at(data, &mut cursor)?;
        let name = if name_index == u64::MAX {
            None
        } else {
            let index =
                usize::try_from(name_index).map_err(|_| MetaError::StringIndexOutOfBounds {
                    table: "TST1",
                    index: name_index,
                    len: strings.len(),
                })?;
            if index >= strings.len() {
                return Err(MetaError::StringIndexOutOfBounds {
                    table: "TST1",
                    index: name_index,
                    len: strings.len(),
                });
            }
            let name = strings[index].clone();
            if name.is_empty() { None } else { Some(name) }
        };

        let num_params = vle::read_at(data, &mut cursor)?;
        let mut params = Vec::with_capacity(num_params.min(64) as usize);
        for _ in 0..num_params {
            let kind_name_id = vle_u32(data, &mut cursor)? as u64;
            let kind_name =
                strings
                    .get(kind_name_id as usize)
                    .ok_or(MetaError::StringIndexOutOfBounds {
                        table: "TST1",
                        index: kind_name_id,
                        len: strings.len(),
                    })?;
            if !matches!(kind_name.as_bytes().first(), Some(b't' | b'v')) {
                return Err(MetaError::StringIndexOutOfBounds {
                    table: "TST1",
                    index: kind_name_id,
                    len: strings.len(),
                });
            }
            let value = vle::read_at(data, &mut cursor)?;
            params.push(TemplateParam {
                kind_name_id,
                value,
            });
        }

        *slot = Some(TypeIdentity { name, params });
    }
    Ok(identities)
}

fn map_optbits(compressed: u32) -> Result<u32, MetaError> {
    let mut out = 0;
    for (index, mask) in OPT_MAPPING.iter().enumerate() {
        let bit = 1 << index;
        if compressed & bit != 0 {
            out |= *mask;
        }
    }
    let leftover = compressed & !((1u32 << OPT_MAPPING.len()) - 1);
    if leftover != 0 {
        return Err(MetaError::InvalidOptBits(leftover));
    }
    Ok(out)
}

/// Parse `TBDY` type bodies.
pub fn parse_bodies(
    data: &[u8],
    identities: &[Option<TypeIdentity>],
    field_strings: &[String],
) -> Result<Vec<Option<TypeMeta>>, MetaError> {
    let mut types: Vec<Option<TypeMeta>> = (0..identities.len()).map(|_| None).collect();
    let mut cursor = 0;

    while cursor < data.len() && data[cursor] != 0 {
        let type_id = validate_type_id(vle::read_at(data, &mut cursor)?, types.len())?;
        let parent_id = vle_u32(data, &mut cursor)?;
        if parent_id as u64 >= types.len() as u64 && parent_id != 0 {
            return Err(MetaError::TypeIdOutOfBounds {
                id: parent_id as u64,
                len: types.len(),
            });
        }
        if types[type_id as usize].is_some() {
            return Err(MetaError::DuplicateTypeBody(type_id));
        }

        let optbits = map_optbits(vle_u32(data, &mut cursor)?)?;

        let format = if optbits & OPT_FORMAT != 0 {
            Some(Format::from_raw(vle_u32(data, &mut cursor)?)?)
        } else {
            None
        };
        let subtype = if optbits & OPT_SUBTYPE != 0 {
            let id = vle_u32(data, &mut cursor)? as u64;
            if id < identities.len() as u64 {
                Some(id as TypeId)
            } else {
                Some(0)
            }
        } else {
            None
        };
        let version = if optbits & OPT_VERSION != 0 {
            let raw = vle::read_at(data, &mut cursor)?;
            let high = raw & 0xffff_ffff_0000_0000;
            if high != 0 && high != 0xffff_ffff_0000_0000 {
                return Err(MetaError::InvalidOptBits(high as u32));
            }
            let value = raw as u32 as i32;
            if value < 0 {
                return Err(MetaError::TruncatedBody);
            }
            Some(value as u32)
        } else {
            None
        };
        let (size, align) = if optbits & OPT_SIZE_ALIGN != 0 {
            (
                vle_u32(data, &mut cursor)?,
                vle_u32(data, &mut cursor)? & 0xfff,
            )
        } else {
            (0, 0)
        };
        let flags = if optbits & OPT_FLAGS != 0 {
            vle_u16(data, &mut cursor)?
        } else {
            0
        };

        let mut fields = Vec::new();
        if optbits & OPT_DECLS != 0 {
            let encoded = vle_u32(data, &mut cursor)?;
            let num_fields = (encoded & 0xffff) as usize;
            if num_fields > (data.len() - cursor) / 4 {
                return Err(MetaError::TruncatedBody);
            }
            for _ in 0..num_fields {
                let sid = vle_u16(data, &mut cursor)? as usize;
                let name = field_strings
                    .get(sid)
                    .ok_or(MetaError::StringIndexOutOfBounds {
                        table: "FST1",
                        index: sid as u64,
                        len: field_strings.len(),
                    })?
                    .clone();
                let mut field_flags = vle_u16(data, &mut cursor)?;
                if field_flags & DECL_ALIGN_FLAG != 0 {
                    field_flags &= !DECL_ALIGN_FLAG;
                    let _requested_align = vle::read_at(data, &mut cursor)?;
                }
                let offset = vle_u16(data, &mut cursor)?;
                let field_type_id =
                    validate_type_id(vle::read_at(data, &mut cursor)?, types.len())?;
                if field_flags & (DECL_PROPERTY_FIELD | DECL_DATA_FIELD) != DECL_DATA_FIELD {
                    return Err(MetaError::InvalidFieldFlags(field_flags));
                }
                fields.push(Field {
                    name,
                    offset,
                    type_id: field_type_id,
                    flags: field_flags,
                });
            }
        }

        let mut interfaces = Vec::new();
        if optbits & OPT_INTERFACES != 0 {
            let count = vle_u32(data, &mut cursor)? as usize;
            if count > (data.len() - cursor) / 2 {
                return Err(MetaError::TruncatedBody);
            }
            for _ in 0..count {
                let tid = vle_u32(data, &mut cursor)?;
                let offset = vle_u32(data, &mut cursor)?;
                interfaces.push((tid, offset));
            }
        }
        if optbits & OPT_ATTRIBUTE_STRING != 0 {
            let _attribute_index = vle_u32(data, &mut cursor)?;
        }

        types[type_id as usize] = Some(TypeMeta {
            id: type_id,
            parent: parent_id,
            name: identities[type_id as usize]
                .as_ref()
                .and_then(|identity| identity.name.clone()),
            format,
            subtype,
            version,
            size,
            align,
            flags,
            fields,
            interfaces,
        });
    }

    let padding = &data[cursor..];
    if padding.len() > 3 || padding.iter().any(|byte| *byte != 0) {
        return Err(MetaError::InvalidPadding);
    }
    Ok(types)
}

/// Convenience wrapper that parses all four `TYPE` leaves.
pub fn parse_type_info(
    type_strings: &[u8],
    field_strings: &[u8],
    identities: &[u8],
    bodies: &[u8],
) -> Result<TypeInfo, MetaError> {
    let type_names = parse_strings(type_strings);
    let field_names = parse_strings(field_strings);
    let parsed_identities = parse_identities(identities, &type_names)?;
    let types = parse_bodies(bodies, &parsed_identities, &field_names)?;
    Ok(TypeInfo {
        identities: parsed_identities,
        types,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vle;

    fn writer() -> Vec<u8> {
        Vec::new()
    }

    #[test]
    fn parses_string_tables() {
        assert_eq!(
            parse_strings(b"A\0BC\0\0padding"),
            vec!["A".to_string(), "BC".to_string(), String::new()]
        );
    }

    #[test]
    fn parses_identities_and_bodies() {
        let mut type_strings = writer();
        type_strings.extend_from_slice(b"Demo\0char\0");
        let mut field_strings = writer();
        field_strings.extend_from_slice(b"value\0next\0");

        let mut identities = writer();
        vle::write(3, &mut identities); // ids 1..2
        vle::write(0, &mut identities); // Demo -> "Demo"
        vle::write(0, &mut identities); // no template params
        vle::write(1, &mut identities); // char -> "char"
        vle::write(0, &mut identities);

        let mut bodies = writer();
        // char: int8 unsigned, size 1, align 1.
        vle::write(2, &mut bodies);
        vle::write(0, &mut bodies);
        vle::write((1 << 0) | (1 << 3), &mut bodies); // FORMAT | SIZE_ALIGN
        vle::write(4 | (8 << 10), &mut bodies);
        vle::write(1, &mut bodies);
        vle::write(1, &mut bodies);
        // Demo: record at id 1 with two fields.
        vle::write(1, &mut bodies);
        vle::write(0, &mut bodies);
        vle::write((1 << 0) | (1 << 3) | (1 << 5), &mut bodies); // FORMAT | SIZE_ALIGN | DECLS
        vle::write(7, &mut bodies);
        vle::write(16, &mut bodies);
        vle::write(8, &mut bodies);
        vle::write(2, &mut bodies); // two fields
        vle::write(0, &mut bodies); // field name "value"
        vle::write(0x20, &mut bodies); // DECL_DATA_FIELD
        vle::write(0, &mut bodies);
        vle::write(2, &mut bodies); // int8
        vle::write(1, &mut bodies); // field name "next"
        vle::write(0x20, &mut bodies);
        vle::write(8, &mut bodies);
        vle::write(2, &mut bodies);
        bodies.extend_from_slice(&[0, 0]); // padding

        let info = parse_type_info(&type_strings, &field_strings, &identities, &bodies)
            .expect("type info");
        assert_eq!(info.types.len(), 3);
        assert_eq!(
            info.types[1].as_ref().unwrap().name.as_deref(),
            Some("Demo")
        );
        assert_eq!(
            info.types[2].as_ref().unwrap().format.unwrap().kind,
            Kind::Int
        );
        let demo = info.types[1].as_ref().unwrap();
        assert_eq!(demo.size, 16);
        assert_eq!(demo.fields.len(), 2);
        assert_eq!(demo.fields[1].name, "next");
        assert_eq!(demo.fields[1].offset, 8);
    }
}
