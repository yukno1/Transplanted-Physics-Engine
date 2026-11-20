//! Generic object rebuild from a `TAG0` tagfile bundle.
//!
//! This is the first object-loading subset: it parses type identities/bodies,
//! the item index and in-place fixups, then materialises every object into a
//! generic [`Value`] tree.  Type compendiums can supply shared `TYPE` metadata
//! through [`Bundle::load_with_compendia`].  It does **not** construct Havok
//! physics objects, apply version patches or support binary packfiles.

use std::collections::{BTreeMap, HashSet};

use crate::compendium::{self, Compendium};
use crate::hff::{HffNode, four_cc};
use crate::tagfile::{Tagfile, TagfileError};
use crate::type_meta::{self, Format, Kind, TypeId, TypeInfo, TypeMeta};
use crate::vle::VleError;

const TST1: u32 = four_cc(b"TST1");
const FST1: u32 = four_cc(b"FST1");
const TNA1: u32 = four_cc(b"TNA1");
const TBDY: u32 = four_cc(b"TBDY");
const TNAM: u32 = four_cc(b"TNAM");
const TBOD: u32 = four_cc(b"TBOD");
const ITEM: u32 = four_cc(b"ITEM");
const PATCH: u32 = four_cc(b"PTCH");

/// Errors produced while loading a bundle.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LoadError {
    /// The container is not a tagfile.
    Tagfile(TagfileError),
    /// Reflection metadata was malformed.
    Meta(type_meta::MetaError),
    /// A VLE integer was malformed.
    Vle(VleError),
    /// A required section was absent.
    MissingSection(&'static str),
    /// A legacy or otherwise unsupported section was present.
    Unsupported(&'static str),
    /// The `ITEM` section length was not a multiple of 12 bytes.
    InvalidItemSection,
    /// The `PTCH` section length was not a multiple of 4 bytes.
    InvalidPatchSection,
    /// The `TCID` section length was not a multiple of 8 bytes.
    InvalidSignatureSection,
    /// No loaded compendium had the signature required by `TCRF`.
    CompendiumNotFound(u64),
    /// A fixup source offset had no room for its target index.
    PatchOutOfBounds {
        /// Source byte offset.
        offset: usize,
        /// Length of the `DATA` section.
        data_len: usize,
    },
    /// A fixup referenced a type or item id that does not exist.
    InvalidIndex {
        /// What kind of index was invalid.
        what: &'static str,
        /// Invalid index.
        index: u32,
        /// Number of valid entries.
        len: usize,
    },
    /// A value read ran past the end of `DATA`.
    DataOutOfBounds {
        /// Requested start.
        offset: usize,
        /// Requested length.
        length: usize,
        /// Length of the `DATA` section.
        data_len: usize,
    },
    /// A type id had no decoded type body.
    MissingType(TypeId),
    /// A parent chain contained a cycle.
    ParentCycle(TypeId),
    /// A type used a field encoding this subset cannot materialise.
    UnsupportedValue(&'static str),
}

impl From<TagfileError> for LoadError {
    fn from(value: TagfileError) -> Self {
        Self::Tagfile(value)
    }
}

impl From<type_meta::MetaError> for LoadError {
    fn from(value: type_meta::MetaError) -> Self {
        Self::Meta(value)
    }
}

impl From<VleError> for LoadError {
    fn from(value: VleError) -> Self {
        Self::Vle(value)
    }
}

impl core::fmt::Display for LoadError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Tagfile(error) => write!(f, "tagfile error: {error}"),
            Self::Meta(error) => write!(f, "type metadata error: {error}"),
            Self::Vle(error) => write!(f, "VLE error: {error}"),
            Self::MissingSection(name) => write!(f, "missing tagfile section {name}"),
            Self::Unsupported(name) => write!(f, "unsupported tagfile feature {name}"),
            Self::InvalidItemSection => write!(f, "ITEM section is not a multiple of 12 bytes"),
            Self::InvalidPatchSection => write!(f, "PTCH section is not a multiple of 4 bytes"),
            Self::InvalidSignatureSection => write!(f, "TCID section is not a multiple of 8 bytes"),
            Self::CompendiumNotFound(signature) => {
                write!(f, "no loaded compendium has signature {signature:016x}")
            }
            Self::PatchOutOfBounds { offset, data_len } => {
                write!(f, "fixup at {offset} outside DATA ({data_len} bytes)")
            }
            Self::InvalidIndex { what, index, len } => {
                write!(f, "invalid {what} index {index} ({} entries)", len)
            }
            Self::DataOutOfBounds {
                offset,
                length,
                data_len,
            } => write!(
                f,
                "read of {length} bytes at {offset} outside DATA ({data_len} bytes)"
            ),
            Self::MissingType(id) => write!(f, "no decoded type body for type id {id}"),
            Self::ParentCycle(id) => write!(f, "parent cycle at type id {id}"),
            Self::UnsupportedValue(name) => write!(f, "unsupported value encoding {name}"),
        }
    }
}

impl std::error::Error for LoadError {}

/// Kind of an entry in the `ITEM` section (`TagfileItem::KIND_*`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ItemKind {
    /// Null entry / note target sentinel.
    Empty,
    /// One inline object (`KIND_VAR0`).
    Var0,
    /// `count` inline objects (`KIND_VARN`).
    VarN,
    /// Alias of the item whose index is stored in `count` (`KIND_NOTE`).
    Note,
    /// Type descriptor (`KIND_TYPE`).
    Type,
    /// Decl descriptor (`KIND_DECL`).
    Decl,
}

impl ItemKind {
    fn from_packed(packed: u32) -> Self {
        match packed & 0xf000_0000 {
            0x1000_0000 => Self::Var0,
            0x2000_0000 => Self::VarN,
            0x3000_0000 => Self::Note,
            0x4000_0000 => Self::Type,
            0x5000_0000 => Self::Decl,
            _ => Self::Empty,
        }
    }
}

/// One `TagfileItem` triple (`packed`, `offset`, `count`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Item {
    /// Entry kind.
    pub kind: ItemKind,
    /// Type id stored in the low 24 bits of `packed`.
    pub type_id: TypeId,
    /// Byte offset into `DATA`.
    pub offset: u32,
    /// Element count (or note target index).
    pub count: u32,
}

/// A resolved in-place fixup.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fixup {
    /// Type of the field being fixed up.
    pub src_type_id: TypeId,
    /// Target item index, or `None` for a null pointer/string.
    pub target: Option<u32>,
}

/// Generic value produced by object rebuilding.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// No value (null type or non-object item).
    Void,
    /// Opaque bytes of the declared size.
    Opaque(Vec<u8>),
    /// Boolean field.
    Bool(bool),
    /// Signed or unsigned integer, widened to `i128`.
    Int(i128),
    /// Floating-point field, widened to `f64`.
    Float(f64),
    /// NUL-terminated string referenced by a string field.
    String(String),
    /// Explicit null pointer/string.
    NullPointer,
    /// Pointer to another item in the same bundle.
    Ref(u32),
    /// Fixed or dynamic array of values.
    Array(Vec<Value>),
    /// Record with fields in parent-to-child order.
    Record(Record),
}

/// A materialised record value.
#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    /// Type id of the record.
    pub type_id: TypeId,
    /// Field name/value pairs in parent-to-child declaration order.
    pub fields: Vec<(String, Value)>,
}

/// A fully loaded generic bundle.
#[derive(Clone, Debug, PartialEq)]
pub struct Bundle {
    /// SDK version string from the `SDKV` section, when present.
    pub sdk_version: Option<String>,
    /// Decoded reflection metadata.
    pub type_info: TypeInfo,
    /// Item index from `INDX/ITEM`.
    pub items: Vec<Item>,
    /// Raw object data from `DATA`.
    pub data: Vec<u8>,
    /// Fixups keyed by byte offset in `DATA`.
    pub fixups: BTreeMap<u32, Fixup>,
    /// Materialised values, indexed in parallel with [`Self::items`].
    pub objects: Vec<Value>,
}

impl Bundle {
    /// Load a `TAG0` tagfile into a generic value bundle.
    pub fn load(bytes: &[u8]) -> Result<Self, LoadError> {
        Self::load_inner(bytes, &[])
    }

    /// Load a bundle whose types may live in `TCM0` compendiums.
    ///
    /// A `TCRF` signature is checked against the supplied compendia; their
    /// types are merged in caller order before the bundle's own `TYPE`
    /// section, matching the SDK's shared-then-local registration.
    pub fn load_with_compendia(bytes: &[u8], compendia: &[Compendium]) -> Result<Self, LoadError> {
        Self::load_inner(bytes, compendia)
    }

    fn load_inner(bytes: &[u8], compendia: &[Compendium]) -> Result<Self, LoadError> {
        let tagfile = Tagfile::parse(bytes)?;
        if tagfile.is_compendium() {
            return Err(LoadError::Unsupported("TCM0 compendium"));
        }

        if let Some(signature) = tagfile.compendium_reference()? {
            if !compendia
                .iter()
                .any(|compendium| compendium.contains_signature(signature))
            {
                return Err(LoadError::CompendiumNotFound(signature));
            }
        }

        let data = leaf_bytes(tagfile.data(), "DATA")?;
        let mut type_info = TypeInfo {
            identities: vec![None],
            types: vec![None],
        };
        for compendium in compendia {
            compendium::merge_type_info(&mut type_info, &compendium.type_info);
        }
        if tagfile.types().is_some() {
            compendium::merge_type_info(&mut type_info, &type_info_from_tagfile(&tagfile)?);
        } else if compendia.is_empty() {
            return Err(LoadError::MissingSection("TYPE"));
        }

        let index = tagfile.index().ok_or(LoadError::MissingSection("INDX"))?;
        let item_bytes = leaf_bytes(index.find(ITEM), "ITEM")?;
        let items = parse_items(item_bytes)?;
        let patch_bytes = leaf_bytes_optional(index.find(PATCH));
        let fixups = match patch_bytes {
            Some(bytes) => parse_patch(bytes, data, items.len())?,
            None => BTreeMap::new(),
        };

        let context = ReadCtx {
            data,
            type_info: &type_info,
            items: &items,
            fixups: &fixups,
        };
        let mut objects = Vec::with_capacity(items.len());
        for item in &items {
            objects.push(build_object(&context, item)?);
        }

        Ok(Self {
            sdk_version: tagfile.version.clone(),
            type_info,
            items,
            data: data.to_vec(),
            fixups,
            objects,
        })
    }

    /// Materialised value for an item index.
    pub fn object(&self, item: u32) -> Option<&Value> {
        self.objects.get(item as usize)
    }
}

fn leaf_bytes<'a>(node: Option<&'a HffNode>, name: &'static str) -> Result<&'a [u8], LoadError> {
    let node = node.ok_or(LoadError::MissingSection(name))?;
    if !node.is_leaf() {
        return Err(LoadError::Unsupported(name));
    }
    Ok(&node.data)
}

fn leaf_bytes_optional(node: Option<&HffNode>) -> Option<&[u8]> {
    node.filter(|node| node.is_leaf())
        .map(|node| node.data.as_slice())
}

/// Decode the reflection metadata in a `TYPE` section.
pub(crate) fn type_info_from_tagfile(tagfile: &Tagfile) -> Result<TypeInfo, LoadError> {
    let types = tagfile.types().ok_or(LoadError::MissingSection("TYPE"))?;
    if types.find(TNAM).is_some() || types.find(TBOD).is_some() {
        return Err(LoadError::Unsupported("V0 TNAM/TBOD type sections"));
    }
    let type_strings = leaf_bytes(types.find(TST1), "TST1")?;
    let field_strings = leaf_bytes(types.find(FST1), "FST1")?;
    let identities = leaf_bytes(types.find(TNA1), "TNA1")?;
    let bodies = leaf_bytes(types.find(TBDY), "TBDY")?;
    Ok(type_meta::parse_type_info(
        type_strings,
        field_strings,
        identities,
        bodies,
    )?)
}

fn parse_items(bytes: &[u8]) -> Result<Vec<Item>, LoadError> {
    if bytes.len() % 12 != 0 {
        return Err(LoadError::InvalidItemSection);
    }
    let mut items = Vec::with_capacity(bytes.len() / 12);
    for chunk in bytes.chunks_exact(12) {
        let packed = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        let offset = u32::from_be_bytes([chunk[4], chunk[5], chunk[6], chunk[7]]);
        let count = u32::from_be_bytes([chunk[8], chunk[9], chunk[10], chunk[11]]);
        items.push(Item {
            kind: ItemKind::from_packed(packed),
            type_id: packed & 0x00ff_ffff,
            offset,
            count,
        });
    }
    Ok(items)
}

fn read_u32_le(data: &[u8], offset: u32) -> Result<u32, LoadError> {
    let offset = offset as usize;
    let slice = data
        .get(offset..offset + 4)
        .ok_or(LoadError::PatchOutOfBounds {
            offset,
            data_len: data.len(),
        })?;
    Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

fn insert_fixup(
    fixups: &mut BTreeMap<u32, Fixup>,
    offset: u32,
    src_type_id: u32,
    target: Option<u32>,
    num_items: usize,
) -> Result<(), LoadError> {
    if let Some(index) = target {
        if index as usize >= num_items {
            return Err(LoadError::InvalidIndex {
                what: "fixup target",
                index,
                len: num_items,
            });
        }
    }
    fixups.insert(
        offset,
        Fixup {
            src_type_id,
            target,
        },
    );
    Ok(())
}

fn parse_patch(
    bytes: &[u8],
    data: &[u8],
    num_items: usize,
) -> Result<BTreeMap<u32, Fixup>, LoadError> {
    if bytes.len() % 4 != 0 {
        return Err(LoadError::InvalidPatchSection);
    }
    let ints: Vec<u32> = bytes
        .chunks_exact(4)
        .map(|chunk| u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
        .collect();
    let mut cursor = 0;
    let mut fixups = BTreeMap::new();

    // Internal fixups: (srcTid, count, offsets…), terminated by a zero id or
    // by the end of the section when no external fixups follow.
    while cursor + 2 < ints.len() {
        let src_type_id = ints[cursor];
        cursor += 1;
        if src_type_id == 0 {
            break;
        }
        let count = ints[cursor] as usize;
        cursor += 1;
        if cursor + count > ints.len() {
            return Err(LoadError::InvalidPatchSection);
        }
        for _ in 0..count {
            let offset = ints[cursor];
            cursor += 1;
            let target = match read_u32_le(data, offset)? {
                0 => None,
                index => Some(index),
            };
            insert_fixup(&mut fixups, offset, src_type_id, target, num_items)?;
        }
    }

    // External fixups: (srcTid, numTargets, (tgtIdx, count, offsets…)…).
    while cursor + 2 < ints.len() {
        let src_type_id = ints[cursor];
        cursor += 1;
        if src_type_id == 0 {
            break;
        }
        let num_targets = ints[cursor] as usize;
        cursor += 1;
        if cursor + 2 * num_targets > ints.len() {
            return Err(LoadError::InvalidPatchSection);
        }
        for _ in 0..num_targets {
            let target = match ints[cursor] {
                0 => None,
                index => Some(index),
            };
            cursor += 1;
            let count = ints[cursor] as usize;
            cursor += 1;
            if cursor + count > ints.len() {
                return Err(LoadError::InvalidPatchSection);
            }
            for _ in 0..count {
                let offset = ints[cursor];
                cursor += 1;
                insert_fixup(&mut fixups, offset, src_type_id, target, num_items)?;
            }
        }
    }

    if cursor != ints.len() {
        return Err(LoadError::InvalidPatchSection);
    }
    Ok(fixups)
}

struct ReadCtx<'a> {
    data: &'a [u8],
    type_info: &'a TypeInfo,
    items: &'a [Item],
    fixups: &'a BTreeMap<u32, Fixup>,
}

fn meta<'a>(ctx: &'a ReadCtx<'a>, type_id: TypeId) -> Result<&'a TypeMeta, LoadError> {
    ctx.type_info
        .types
        .get(type_id as usize)
        .and_then(Option::as_ref)
        .ok_or(LoadError::MissingType(type_id))
}

fn slice_at<'a>(ctx: &'a ReadCtx<'a>, offset: usize, length: usize) -> Result<&'a [u8], LoadError> {
    ctx.data
        .get(offset..offset + length)
        .ok_or(LoadError::DataOutOfBounds {
            offset,
            length,
            data_len: ctx.data.len(),
        })
}

fn read_int(
    ctx: &ReadCtx<'_>,
    format: &Format,
    fallback_size: u32,
    offset: usize,
) -> Result<i128, LoadError> {
    let bits = if format.bits == 0 {
        fallback_size.saturating_mul(8)
    } else {
        format.bits
    };
    if bits == 0 || bits > 64 || bits % 8 != 0 {
        return Err(LoadError::UnsupportedValue("integer width"));
    }
    let bytes = slice_at(ctx, offset, (bits / 8) as usize)?;
    let mut value = 0u128;
    for (index, byte) in bytes.iter().enumerate() {
        value |= (*byte as u128) << (8 * index);
    }
    if format.signed && value & (1u128 << (bits - 1)) != 0 {
        value |= !0u128 << bits;
    }
    Ok(value as i128)
}

fn read_float(ctx: &ReadCtx<'_>, format: &Format, offset: usize) -> Result<f64, LoadError> {
    match format.float_bits() {
        32 => {
            let bytes = slice_at(ctx, offset, 4)?;
            Ok(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as f64)
        }
        64 => {
            let bytes = slice_at(ctx, offset, 8)?;
            Ok(f64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ]))
        }
        _ => Err(LoadError::UnsupportedValue("float width")),
    }
}

fn fixup_target(ctx: &ReadCtx<'_>, offset: usize) -> Option<u32> {
    ctx.fixups
        .get(&(offset as u32))
        .and_then(|fixup| fixup.target)
}

fn read_string(ctx: &ReadCtx<'_>, target: u32) -> Result<String, LoadError> {
    if target == 0 {
        return Ok(String::new());
    }
    let item = ctx
        .items
        .get(target as usize)
        .ok_or(LoadError::InvalidIndex {
            what: "string item",
            index: target,
            len: ctx.items.len(),
        })?;
    let start = item.offset as usize;
    let bytes = slice_at(ctx, start, item.count as usize)?;
    let end = bytes
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(bytes.len());
    Ok(String::from_utf8_lossy(&bytes[..end]).into_owned())
}

fn collect_fields<'a>(
    ctx: &'a ReadCtx<'a>,
    type_id: TypeId,
) -> Result<Vec<&'a crate::type_meta::Field>, LoadError> {
    let mut chain = Vec::new();
    let mut seen = HashSet::new();
    let mut current = type_id;
    loop {
        if !seen.insert(current) {
            return Err(LoadError::ParentCycle(current));
        }
        let info = meta(ctx, current)?;
        chain.push(info);
        if info.parent == 0 {
            break;
        }
        current = info.parent;
    }
    let mut fields = Vec::new();
    for info in chain.into_iter().rev() {
        fields.extend(info.fields.iter());
    }
    Ok(fields)
}

fn read_value(ctx: &ReadCtx<'_>, type_id: TypeId, offset: usize) -> Result<Value, LoadError> {
    let info = meta(ctx, type_id)?;
    let Some(format) = info.format else {
        return Ok(if info.size == 0 {
            Value::Void
        } else {
            Value::Opaque(slice_at(ctx, offset, info.size as usize)?.to_vec())
        });
    };

    match format.kind {
        Kind::Void => Ok(Value::Void),
        Kind::Opaque => Ok(Value::Opaque(
            slice_at(ctx, offset, info.size as usize)?.to_vec(),
        )),
        Kind::Bool => Ok(Value::Bool(read_int(ctx, &format, info.size, offset)? != 0)),
        Kind::Int => Ok(Value::Int(read_int(ctx, &format, info.size, offset)?)),
        Kind::Float => Ok(Value::Float(read_float(ctx, &format, offset)?)),
        Kind::String => match fixup_target(ctx, offset) {
            Some(target) => Ok(Value::String(read_string(ctx, target)?)),
            None => Ok(Value::String(String::new())),
        },
        Kind::Pointer => match fixup_target(ctx, offset) {
            Some(target) => Ok(Value::Ref(target)),
            None => Ok(Value::NullPointer),
        },
        Kind::Array => {
            let subtype = info.subtype.ok_or(LoadError::MissingType(type_id))?;
            if subtype == 0 {
                return Err(LoadError::UnsupportedValue("variant array"));
            }
            let element = meta(ctx, subtype)?;
            let element_size = element.size as usize;
            if element_size == 0 {
                return Err(LoadError::UnsupportedValue("array element size"));
            }

            if format.fixed_count != 0 {
                let mut values = Vec::with_capacity(format.fixed_count as usize);
                for index in 0..format.fixed_count as usize {
                    values.push(read_value(ctx, subtype, offset + index * element_size)?);
                }
                Ok(Value::Array(values))
            } else {
                let Some(target) = fixup_target(ctx, offset) else {
                    return Ok(Value::Array(Vec::new()));
                };
                let size_offset = offset
                    .checked_add(8)
                    .ok_or(LoadError::InvalidPatchSection)?;
                let bytes = slice_at(ctx, size_offset, 4)?;
                let count = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
                if count < 0 {
                    return Err(LoadError::UnsupportedValue("negative array size"));
                }
                let count = count as usize;
                let item = ctx
                    .items
                    .get(target as usize)
                    .ok_or(LoadError::InvalidIndex {
                        what: "array item",
                        index: target,
                        len: ctx.items.len(),
                    })?;
                let start = item.offset as usize;
                let mut values = Vec::with_capacity(count);
                for index in 0..count {
                    let element_offset = start
                        .checked_add(index * element_size)
                        .ok_or(LoadError::InvalidPatchSection)?;
                    values.push(read_value(ctx, subtype, element_offset)?);
                }
                Ok(Value::Array(values))
            }
        }
        Kind::Record => {
            let mut fields = Vec::new();
            for field in collect_fields(ctx, type_id)? {
                let value = read_value(ctx, field.type_id, offset + field.offset as usize)?;
                fields.push((field.name.clone(), value));
            }
            Ok(Value::Record(Record { type_id, fields }))
        }
    }
}

fn build_object(ctx: &ReadCtx<'_>, item: &Item) -> Result<Value, LoadError> {
    match item.kind {
        ItemKind::Empty | ItemKind::Type | ItemKind::Decl => Ok(Value::Void),
        ItemKind::Note => Ok(Value::Ref(item.count)),
        ItemKind::Var0 | ItemKind::VarN => {
            if item.type_id == 0 {
                return Ok(Value::Void);
            }
            let info = meta(ctx, item.type_id)?;
            let count = if item.kind == ItemKind::Var0 {
                1usize
            } else {
                item.count as usize
            };
            if count == 0 {
                return Ok(Value::Void);
            }
            if info.size == 0 {
                return Err(LoadError::UnsupportedValue("zero-sized object"));
            }
            let stride = info.size as usize;
            let mut values = Vec::with_capacity(count);
            for index in 0..count {
                let offset = (item.offset as usize)
                    .checked_add(index * stride)
                    .ok_or(LoadError::InvalidPatchSection)?;
                values.push(read_value(ctx, item.type_id, offset)?);
            }
            if item.kind == ItemKind::Var0 {
                Ok(values.remove(0))
            } else {
                Ok(Value::Array(values))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compendium::Compendium;
    use crate::hff::{four_cc, pack_scope};
    use crate::tagfile::{
        COMPENDIUM, COMPENDIUM_ID, COMPENDIUM_REFERENCE, DATA, INDEX, SDK_VERSION, TAGFILE, TYPES,
    };
    use crate::vle;

    const T_DEMO: u32 = 1;
    const T_CHILD: u32 = 2;
    const T_I8: u32 = 3;
    const T_I32: u32 = 4;
    const T_F32: u32 = 5;
    const T_STRING: u32 = 6;
    const T_PTR: u32 = 7;
    const T_ARRAY_FIXED: u32 = 8;
    const T_ARRAY_DYN: u32 = 9;
    const T_BOOL: u32 = 10;

    const FMT_I8: u32 = 4 | (8 << 10);
    const FMT_I32: u32 = 4 | (1 << 9) | (32 << 10);
    const FMT_F32: u32 = 5 | (1 << 9) | (1 << 10) | (8 << 11) | (23 << 16);
    const FMT_STRING: u32 = 3;
    const FMT_POINTER: u32 = 6;
    const FMT_ARRAY_FIXED: u32 = 8 | (1 << 5) | (2 << 8);
    const FMT_ARRAY_DYN: u32 = 8;
    const FMT_BOOL: u32 = 2 | (8 << 10);
    const FMT_RECORD: u32 = 7;

    const OPT_FORMAT: u32 = 1 << 0;
    const OPT_SUBTYPE: u32 = 1 << 1;
    const OPT_SIZE_ALIGN: u32 = 1 << 3;
    const OPT_DECLS: u32 = 1 << 5;

    fn leaf(ident: u32, payload: &[u8]) -> Vec<u8> {
        let size = 8 + payload.len();
        let mut bytes = pack_scope(1, size as u32).to_be_bytes().to_vec();
        bytes.extend_from_slice(&ident.to_be_bytes());
        bytes.extend_from_slice(payload);
        bytes
    }

    fn branch(ident: u32, children: &[Vec<u8>]) -> Vec<u8> {
        let size = 8 + children.iter().map(Vec::len).sum::<usize>();
        let mut bytes = pack_scope(0, size as u32).to_be_bytes().to_vec();
        bytes.extend_from_slice(&ident.to_be_bytes());
        for child in children {
            bytes.extend_from_slice(child);
        }
        bytes
    }

    fn put_u32_le(out: &mut [u8], value: u32) {
        out.copy_from_slice(&value.to_le_bytes());
    }

    fn put_i32_le(out: &mut [u8], value: i32) {
        out.copy_from_slice(&value.to_le_bytes());
    }

    fn type_section() -> Vec<u8> {
        let type_names =
            b"Demo\0Child\0i8\0i32\0f32\0string\0ptr_child\0arr_fixed\0arr_dyn\0bool\0";
        let field_names = b"count\0ratio\0label\0child\0values\0items\0flag\0value\0";

        let mut identities = Vec::new();
        vle::write(11, &mut identities); // high mark: null type + ids 1..10
        for tid in 1..=10 {
            vle::write(tid - 1, &mut identities);
            vle::write(0, &mut identities);
        }

        let mut bodies = Vec::new();
        // Demo record, 64 bytes, 7 fields.
        vle::write(T_DEMO as u64, &mut bodies);
        vle::write(0, &mut bodies);
        vle::write(
            (OPT_FORMAT | OPT_SIZE_ALIGN | OPT_DECLS) as u64,
            &mut bodies,
        );
        vle::write(FMT_RECORD as u64, &mut bodies);
        vle::write(64, &mut bodies);
        vle::write(8, &mut bodies);
        vle::write(7, &mut bodies);
        for (sid, offset, type_id) in [
            (0u64, 0u64, T_I32 as u64),
            (1, 4, T_F32 as u64),
            (2, 8, T_STRING as u64),
            (3, 16, T_PTR as u64),
            (4, 24, T_ARRAY_FIXED as u64),
            (5, 40, T_ARRAY_DYN as u64),
            (6, 56, T_BOOL as u64),
        ] {
            vle::write(sid, &mut bodies);
            vle::write(0x20, &mut bodies);
            vle::write(offset, &mut bodies);
            vle::write(type_id, &mut bodies);
        }

        // Child record, 8 bytes, one field.
        vle::write(T_CHILD as u64, &mut bodies);
        vle::write(0, &mut bodies);
        vle::write(
            (OPT_FORMAT | OPT_SIZE_ALIGN | OPT_DECLS) as u64,
            &mut bodies,
        );
        vle::write(FMT_RECORD as u64, &mut bodies);
        vle::write(8, &mut bodies);
        vle::write(4, &mut bodies);
        vle::write(1, &mut bodies);
        vle::write(7, &mut bodies);
        vle::write(0x20, &mut bodies);
        vle::write(0, &mut bodies);
        vle::write(T_I32 as u64, &mut bodies);

        for (id, format, subtype, size, align) in [
            (T_I8, FMT_I8, None, 1, 1),
            (T_I32, FMT_I32, None, 4, 4),
            (T_F32, FMT_F32, None, 4, 4),
            (T_STRING, FMT_STRING, None, 8, 8),
            (T_PTR, FMT_POINTER, Some(T_CHILD), 8, 8),
            (T_ARRAY_FIXED, FMT_ARRAY_FIXED, Some(T_I32), 8, 4),
            (T_ARRAY_DYN, FMT_ARRAY_DYN, Some(T_I32), 16, 8),
            (T_BOOL, FMT_BOOL, None, 1, 1),
        ] {
            vle::write(id as u64, &mut bodies);
            vle::write(0, &mut bodies);
            let mut optbits = OPT_FORMAT | OPT_SIZE_ALIGN;
            if subtype.is_some() {
                optbits |= OPT_SUBTYPE;
            }
            vle::write(optbits as u64, &mut bodies);
            vle::write(format as u64, &mut bodies);
            if let Some(subtype) = subtype {
                vle::write(subtype as u64, &mut bodies);
            }
            vle::write(size as u64, &mut bodies);
            vle::write(align as u64, &mut bodies);
        }

        branch(
            TYPES,
            &[
                leaf(TST1, type_names),
                leaf(FST1, field_names),
                leaf(TNA1, &identities),
                leaf(TBDY, &bodies),
            ],
        )
    }

    fn data_section() -> Vec<u8> {
        let mut data = vec![0u8; 64];
        put_u32_le(&mut data[0..4], 42);
        data[4..8].copy_from_slice(&1.5f32.to_le_bytes());
        put_u32_le(&mut data[8..12], 3); // string -> item 3
        put_u32_le(&mut data[16..20], 2); // child -> item 2
        put_u32_le(&mut data[24..28], 7);
        put_u32_le(&mut data[28..32], 9);
        put_u32_le(&mut data[40..44], 4); // dynamic array -> item 4
        put_i32_le(&mut data[48..52], 3);
        put_i32_le(&mut data[52..56], 3);
        data[56] = 1;

        data.extend_from_slice(&99i32.to_le_bytes()); // Child at 64
        data.extend_from_slice(&[0u8; 4]); // 8-byte Child stride
        data.extend_from_slice(b"hello\0"); // string at 72
        data.extend_from_slice(&[0u8; 2]); // align array to 4 bytes
        for value in [10i32, 20, 30] {
            data.extend_from_slice(&value.to_le_bytes());
        }
        data
    }

    fn index_section() -> Vec<u8> {
        let mut items = Vec::new();
        for (kind, tid, offset, count) in [
            (0u32, 0u32, 0u32, 0u32),
            (0x1000_0000, T_DEMO, 0, 1),
            (0x1000_0000, T_CHILD, 64, 1),
            (0x2000_0000, T_I8, 72, 6),
            (0x2000_0000, T_I32, 80, 3),
        ] {
            items.extend_from_slice(&(kind | tid).to_be_bytes());
            items.extend_from_slice(&offset.to_be_bytes());
            items.extend_from_slice(&count.to_be_bytes());
        }

        // Three internal fixup groups: string at 8, pointer at 16, array at 40.
        let words = [T_STRING, 1, 8, T_PTR, 1, 16, T_ARRAY_DYN, 1, 40];
        let mut patch = Vec::new();
        for word in words {
            patch.extend_from_slice(&word.to_be_bytes());
        }

        branch(INDEX, &[leaf(ITEM, &items), leaf(PATCH, &patch)])
    }

    fn sample_tagfile() -> Vec<u8> {
        branch(
            TAGFILE,
            &[
                leaf(SDK_VERSION, b"20211202"),
                leaf(DATA, &data_section()),
                type_section(),
                index_section(),
            ],
        )
    }

    #[test]
    fn loads_record_fields_and_fixups() {
        let bundle = Bundle::load(&sample_tagfile()).expect("bundle");
        assert_eq!(bundle.items.len(), 5);
        assert_eq!(bundle.fixups.len(), 3);

        let Value::Record(root) = bundle.object(1).expect("root object") else {
            panic!("root is not a record");
        };
        assert_eq!(root.type_id, T_DEMO);
        let field = |name: &str| {
            root.fields
                .iter()
                .find(|(field_name, _)| field_name == name)
                .map(|(_, value)| value)
                .expect("field")
        };
        assert_eq!(field("count"), &Value::Int(42));
        assert_eq!(field("ratio"), &Value::Float(1.5));
        assert_eq!(field("label"), &Value::String("hello".to_string()));
        assert_eq!(field("child"), &Value::Ref(2));
        assert_eq!(field("flag"), &Value::Bool(true));
        assert_eq!(
            field("values"),
            &Value::Array(vec![Value::Int(7), Value::Int(9)])
        );
        assert_eq!(
            field("items"),
            &Value::Array(vec![Value::Int(10), Value::Int(20), Value::Int(30)])
        );

        let Value::Record(child) = bundle.object(2).expect("child object") else {
            panic!("child is not a record");
        };
        assert_eq!(child.fields[0].1, Value::Int(99));
    }

    #[test]
    fn rejects_compendium_and_legacy_type_sections() {
        let bytes = branch(crate::tagfile::COMPENDIUM, &[]);
        assert_eq!(
            Bundle::load(&bytes),
            Err(LoadError::Unsupported("TCM0 compendium"))
        );

        let mut legacy = sample_tagfile();
        let legacy_name = four_cc(b"TAG0").to_be_bytes();
        // Rebuild the root with a V0 TNAM leaf instead of TST1.
        let mut data = leaf(DATA, &data_section());
        let mut types = branch(
            TYPES,
            &[
                leaf(TNAM, b"Demo\0"),
                leaf(FST1, b"count\0"),
                leaf(TNA1, &[1, 0, 0]),
                leaf(TBDY, &[0]),
            ],
        );
        legacy.clear();
        legacy
            .extend_from_slice(&pack_scope(0, (8 + data.len() + types.len()) as u32).to_be_bytes());
        legacy.extend_from_slice(&legacy_name);
        legacy.append(&mut data);
        legacy.append(&mut types);
        assert_eq!(
            Bundle::load(&legacy),
            Err(LoadError::Unsupported("V0 TNAM/TBOD type sections"))
        );
    }

    #[test]
    fn reports_bad_item_and_patch_sizes() {
        assert_eq!(parse_items(&[0; 11]), Err(LoadError::InvalidItemSection));
        assert_eq!(
            parse_patch(&[0; 3], &[], 1),
            Err(LoadError::InvalidPatchSection)
        );
    }

    const COMPENDIUM_SIG: u64 = 0x1122_3344_5566_7788;

    fn compendium_bytes(signature: u64) -> Vec<u8> {
        branch(
            COMPENDIUM,
            &[
                leaf(COMPENDIUM_ID, &signature.to_le_bytes()),
                type_section(),
            ],
        )
    }

    fn bundle_with_reference(with_types: bool) -> Vec<u8> {
        let mut sections = vec![leaf(SDK_VERSION, b"20211202"), leaf(DATA, &data_section())];
        if with_types {
            sections.push(type_section());
        }
        sections.push(leaf(COMPENDIUM_REFERENCE, &COMPENDIUM_SIG.to_le_bytes()));
        sections.push(index_section());
        branch(TAGFILE, &sections)
    }

    #[test]
    fn loads_bundle_using_compendium_types() {
        let compendium = Compendium::load(&compendium_bytes(COMPENDIUM_SIG)).expect("compendium");
        assert_eq!(compendium.signatures, vec![COMPENDIUM_SIG]);
        assert!(compendium.contains_signature(COMPENDIUM_SIG));

        let bundle = Bundle::load_with_compendia(&bundle_with_reference(false), &[compendium])
            .expect("bundle using shared types");
        let Value::Record(root) = bundle.object(1).expect("root object") else {
            panic!("root is not a record");
        };
        let field = |name: &str| {
            root.fields
                .iter()
                .find(|(field_name, _)| field_name == name)
                .map(|(_, value)| value)
                .expect("field")
        };
        assert_eq!(field("count"), &Value::Int(42));
        assert_eq!(field("label"), &Value::String("hello".to_string()));
        assert_eq!(field("child"), &Value::Ref(2));
    }

    #[test]
    fn requires_matching_compendium_signature() {
        let bytes = bundle_with_reference(false);
        assert_eq!(
            Bundle::load(&bytes),
            Err(LoadError::CompendiumNotFound(COMPENDIUM_SIG))
        );
        assert_eq!(
            Bundle::load_with_compendia(&bytes, &[]),
            Err(LoadError::CompendiumNotFound(COMPENDIUM_SIG))
        );

        let wrong = Compendium::load(&compendium_bytes(0xdead_beef)).expect("compendium");
        assert_eq!(
            Bundle::load_with_compendia(&bundle_with_reference(true), &[wrong]),
            Err(LoadError::CompendiumNotFound(COMPENDIUM_SIG))
        );
    }

    #[test]
    fn bundle_types_override_compendium_types() {
        let compendium = Compendium::load(&compendium_bytes(COMPENDIUM_SIG)).expect("compendium");
        // The bundle carries its own identical TYPE section; local definitions
        // are merged last and must produce the same materialised record.
        let from_compendium = Bundle::load_with_compendia(
            &bundle_with_reference(false),
            std::slice::from_ref(&compendium),
        )
        .expect("compendium bundle");
        let from_bundle = Bundle::load_with_compendia(&bundle_with_reference(true), &[compendium])
            .expect("local bundle");
        assert_eq!(from_compendium.objects, from_bundle.objects);
    }

    #[test]
    fn compendium_loader_rejects_data_bundles() {
        assert_eq!(
            Compendium::load(&sample_tagfile()),
            Err(LoadError::Unsupported("TCM0 compendium required"))
        );
    }
}
