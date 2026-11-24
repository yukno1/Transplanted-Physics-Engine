//! Legacy binary tagfile (`CAB00D1E`/`D011FACE`) reader subset.
//!
//! Rust port of `hkTagfileReadFormat2014::Reader` from
//! `original/Common/Source/Common/Serialize/Format/Tagfile2014/`.  Unlike the
//! HFF `TAG0` container in [`crate::base_serialize::tagfile`], this format is a stream of
//! variable-length tags: `TAG_FILE_INFO`, `TAG_METADATA` (class definitions),
//! then `TAG_OBJECT*` records whose members are selected by a presence
//! bitfield.
//!
//! Supported: versions 2–6 (including double-precision reals in v6),
//! little/big endian magic, string backreferences, class inheritance,
//! byte/int/real/vector/cstring fields, nested structs, remembered objects,
//! backward and forward references, arrays, tuples and homogeneous arrays.
//! Versions 0/1 (static tag string list), unknown legacy field types and
//! vtable/data-object features are rejected with explicit errors.

use std::collections::HashMap;

/// First magic word of a legacy binary tagfile.
pub const BINARY_MAGIC_0: u32 = 0xCAB0_0D1E;
/// Second magic word of a legacy binary tagfile.
pub const BINARY_MAGIC_1: u32 = 0xD011_FACE;

const TAG_EOF: i64 = -1;
const TAG_FILE_INFO: i64 = 1;
const TAG_METADATA: i64 = 2;
const TAG_OBJECT: i64 = 3;
const TAG_OBJECT_REMEMBER: i64 = 4;
const TAG_OBJECT_BACKREF: i64 = 5;
const TAG_OBJECT_NULL: i64 = 6;
const TAG_FILE_END: i64 = 7;

const TYPE_MASK: i64 = 0xf;
const TYPE_ARRAY: i64 = 0x10;
const TYPE_TUPLE: i64 = 0x20;
const TYPE_VOID: i64 = 0;
const TYPE_BYTE: i64 = 1;
const TYPE_INT: i64 = 2;
const TYPE_REAL: i64 = 3;
const TYPE_VEC4: i64 = 4;
const TYPE_VEC8: i64 = 5;
const TYPE_VEC12: i64 = 6;
const TYPE_VEC16: i64 = 7;
const TYPE_OBJECT: i64 = 8;
const TYPE_STRUCT: i64 = 9;
const TYPE_CSTRING: i64 = 10;

/// Errors produced by [`Bundle2014::load`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// Magic words did not match, in either byte order.
    BadMagic,
    /// The byte stream ended in the middle of a value.
    Truncated,
    /// Tagfile version is not in the supported 2–6 range.
    UnsupportedVersion(i64),
    /// The specified version uses the static tag string list (unsupported).
    LegacyVersion(i64),
    /// A string backreference was outside the string table.
    InvalidStringRef(i64),
    /// A class index was outside the class table.
    InvalidClass(i64),
    /// A class name was needed but the string was null.
    NullClassName,
    /// A field used a legacy type this subset cannot materialise.
    UnsupportedType(i64),
    /// A vector tuple had an unsupported element count.
    InvalidVectorCount(i64),
    /// A pointer/object id could not be resolved after the whole file was read.
    UnresolvedReference(i64),
    /// A string field or class/field name was malformed.
    InvalidString,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BadMagic => write!(f, "not a legacy binary tagfile"),
            Self::Truncated => write!(f, "truncated binary tagfile"),
            Self::UnsupportedVersion(version) => {
                write!(f, "unsupported tagfile version {version}")
            }
            Self::LegacyVersion(version) => {
                write!(
                    f,
                    "legacy tagfile version {version} needs static tag strings"
                )
            }
            Self::InvalidStringRef(index) => write!(f, "invalid string backreference {index}"),
            Self::InvalidClass(index) => write!(f, "invalid class index {index}"),
            Self::NullClassName => write!(f, "null class or field name"),
            Self::UnsupportedType(kind) => write!(f, "unsupported legacy field type {kind}"),
            Self::InvalidVectorCount(count) => write!(f, "invalid vector element count {count}"),
            Self::UnresolvedReference(id) => write!(f, "unresolved object reference {id}"),
            Self::InvalidString => write!(f, "invalid string encoding"),
        }
    }
}

impl std::error::Error for Error {}

/// One field declared by a `TAG_METADATA` class.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FieldDef {
    /// Field name from the string table.
    pub name: String,
    /// Raw `hkLegacyType::Type` value.
    pub legacy_type: i64,
    /// Tuple length when `TYPE_TUPLE` is set, otherwise zero.
    pub tuple_count: i64,
    /// Class name for `TYPE_STRUCT` / `TYPE_OBJECT` fields.
    pub type_name: Option<String>,
}

/// A class definition from `TAG_METADATA`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClassDef {
    /// Class name.
    pub name: String,
    /// Class version.
    pub version: i64,
    /// Parent class index (`None` for no parent).
    pub parent: Option<usize>,
    /// Fields declared by this class only.
    pub fields: Vec<FieldDef>,
}

/// A generic value from a legacy binary tagfile.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// Non-serialised / absent value.
    Void,
    /// Byte, integer or object id.
    Int(i64),
    /// 32-bit or 64-bit floating point value.
    Float(f64),
    /// C-style string.
    String(String),
    /// Null object or string pointer.
    Null,
    /// Fixed tuple or dynamic array.
    Array(Vec<Value>),
    /// Nested struct or top-level object.
    Record(Record),
    /// Reference to a remembered object id (resolved on load).
    Ref(usize),
    /// Forward reference resolved after the whole stream is read.
    ForwardRef(i64),
}

/// A materialised record value.
#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    /// Class table index.
    pub class: usize,
    /// Field name/value pairs in parent-to-child declaration order.
    pub fields: Vec<(String, Value)>,
}

/// Fully parsed legacy binary tagfile.
#[derive(Clone, Debug, PartialEq)]
pub struct Bundle2014 {
    /// Tagfile version from `TAG_FILE_INFO`.
    pub version: i64,
    /// SDK version string for versions 4 and later.
    pub sdk_version: Option<String>,
    /// Whether reals are double precision (version 6).
    pub double_precision: bool,
    /// Class table in `TAG_METADATA` order (index 0 is the null class).
    pub classes: Vec<Option<ClassDef>>,
    /// Top-level objects in stream order.
    pub objects: Vec<Value>,
}

impl Bundle2014 {
    /// Parse a legacy binary tagfile byte stream.
    pub fn load(bytes: &[u8]) -> Result<Self, Error> {
        Reader::new(bytes)?.read()
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
    swap: bool,
    version: i64,
    sdk_version: Option<String>,
    double_precision: bool,
    strings: Vec<String>,
    classes: Vec<Option<ClassDef>>,
    class_by_name: HashMap<String, usize>,
    remembered: Vec<usize>,
    objects: Vec<Value>,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Result<Self, Error> {
        let mut reader = Self {
            bytes,
            pos: 0,
            swap: false,
            version: -1,
            sdk_version: None,
            double_precision: false,
            strings: vec![String::new(), String::new()],
            classes: vec![None],
            class_by_name: HashMap::new(),
            remembered: vec![usize::MAX],
            objects: Vec::new(),
        };
        let magic0 = reader.raw_u32()?;
        if magic0 == BINARY_MAGIC_0 {
            reader.swap = false;
        } else if u32::from_be_bytes(magic0.to_le_bytes()) == BINARY_MAGIC_0 {
            reader.swap = true;
        } else {
            return Err(Error::BadMagic);
        }
        if reader.fixed_u32()? != BINARY_MAGIC_1 {
            return Err(Error::BadMagic);
        }
        Ok(reader)
    }

    fn take(&mut self, count: usize) -> Result<&'a [u8], Error> {
        let end = self.pos.checked_add(count).ok_or(Error::Truncated)?;
        let slice = self.bytes.get(self.pos..end).ok_or(Error::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    fn raw_u32(&mut self) -> Result<u32, Error> {
        let bytes = self.take(4)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn fixed_u32(&mut self) -> Result<u32, Error> {
        let bytes = self.take(4)?;
        Ok(if self.swap {
            u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
        } else {
            u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
        })
    }

    fn fixed_u16(&mut self) -> Result<u16, Error> {
        let bytes = self.take(2)?;
        Ok(if self.swap {
            u16::from_be_bytes([bytes[0], bytes[1]])
        } else {
            u16::from_le_bytes([bytes[0], bytes[1]])
        })
    }

    fn fixed_f32(&mut self) -> Result<f64, Error> {
        Ok(f32::from_bits(self.fixed_u32()?) as f64)
    }

    fn fixed_f64(&mut self) -> Result<f64, Error> {
        let bytes = self.take(8)?;
        let bits = if self.swap {
            u64::from_be_bytes(bytes.try_into().expect("8 bytes"))
        } else {
            u64::from_le_bytes(bytes.try_into().expect("8 bytes"))
        };
        Ok(f64::from_bits(bits))
    }

    fn read_real(&mut self) -> Result<f64, Error> {
        if self.double_precision {
            self.fixed_f64()
        } else {
            self.fixed_f32()
        }
    }

    /// `Reader::readInt`: sign-magnitude variable-length integer.
    fn read_int(&mut self) -> Result<i64, Error> {
        let mut byte = self.take(1)?[0];
        let mut value = u64::from(byte & 0x7f) >> 1;
        let negative = byte & 1 != 0;
        let mut shift = 6u32;
        while byte & 0x80 != 0 {
            byte = self.take(1)?[0];
            if shift >= 64 {
                return Err(Error::Truncated);
            }
            value |= u64::from(byte & 0x7f) << shift;
            shift += 7;
        }
        if value > i64::MAX as u64 {
            return Err(Error::Truncated);
        }
        Ok(if negative {
            -(value as i64)
        } else {
            value as i64
        })
    }

    /// `Reader::readString`: positive length is new, negative is a backreference.
    fn read_string(&mut self) -> Result<Option<String>, Error> {
        let len = self.read_int()?;
        if len > 0 {
            let bytes = self.take(len as usize)?;
            let text = String::from_utf8_lossy(bytes).into_owned();
            self.strings.push(text.clone());
            Ok(Some(text))
        } else if len == 0 {
            Ok(Some(String::new()))
        } else {
            let index = -len;
            if index == 1 {
                Ok(None)
            } else {
                self.strings
                    .get(index as usize)
                    .cloned()
                    .map(Some)
                    .ok_or(Error::InvalidStringRef(len))
            }
        }
    }

    fn read_name(&mut self) -> Result<String, Error> {
        self.read_string()?.ok_or(Error::NullClassName)
    }

    fn read_class(&mut self) -> Result<(), Error> {
        let name = self.read_name()?;
        let version = self.read_int()?;
        let parent_index = self.read_int()?;
        let parent = if parent_index <= 0 {
            None
        } else {
            Some(self.class_index(parent_index)?)
        };
        let num_fields = self.read_int()?;
        if !(0..=4096).contains(&num_fields) {
            return Err(Error::InvalidClass(num_fields));
        }

        let mut fields = Vec::with_capacity(num_fields as usize);
        for _ in 0..num_fields {
            let field_name = self.read_name()?;
            let legacy_type = self.read_int()?;
            let tuple_count = if legacy_type & TYPE_TUPLE != 0 {
                self.read_int()?
            } else {
                0
            };
            let basic = legacy_type & TYPE_MASK;
            let type_name = if basic == TYPE_STRUCT || basic == TYPE_OBJECT {
                self.read_string()?
            } else {
                None
            };
            fields.push(FieldDef {
                name: field_name,
                legacy_type,
                tuple_count,
                type_name,
            });
        }

        let index = self.classes.len();
        self.classes.push(Some(ClassDef {
            name: name.clone(),
            version,
            parent,
            fields,
        }));
        self.class_by_name.insert(name, index);
        Ok(())
    }

    fn class_index(&self, index: i64) -> Result<usize, Error> {
        if index <= 0 || index as usize >= self.classes.len() {
            return Err(Error::InvalidClass(index));
        }
        Ok(index as usize)
    }

    fn chain_fields(&self, class: usize) -> Result<Vec<FieldDef>, Error> {
        let mut chain = Vec::new();
        let mut seen = Vec::new();
        let mut current = Some(class);
        while let Some(index) = current {
            if seen.contains(&index) {
                return Err(Error::InvalidClass(index as i64));
            }
            seen.push(index);
            let info = self
                .classes
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(Error::InvalidClass(index as i64))?;
            chain.push(info);
            current = info.parent;
        }
        let mut fields = Vec::new();
        for info in chain.into_iter().rev() {
            fields.extend(info.fields.iter().cloned());
        }
        Ok(fields)
    }

    fn read_bitfield(&mut self, count: usize) -> Result<Vec<bool>, Error> {
        let byte_count = count.div_ceil(8);
        let mut bits = Vec::with_capacity(count);
        for _ in 0..byte_count {
            let byte = self.take(1)?[0];
            for bit in 0..8 {
                if bits.len() < count {
                    bits.push(byte & (1 << bit) != 0);
                }
            }
        }
        Ok(bits)
    }

    fn read_pointer(&mut self) -> Result<Value, Error> {
        let id = self.read_int()?;
        if id <= 0 {
            return Ok(Value::Null);
        }
        Ok(match self.remembered.get(id as usize) {
            Some(&object) if object != usize::MAX => Value::Ref(object),
            _ => Value::ForwardRef(id),
        })
    }

    fn read_record(&mut self, class: usize) -> Result<Value, Error> {
        let fields = self.chain_fields(class)?;
        let present = self.read_bitfield(fields.len())?;
        let mut values = Vec::new();
        for (index, field) in fields.iter().enumerate() {
            if present[index] {
                values.push((field.name.clone(), self.read_field(field)?));
            }
        }
        Ok(Value::Record(Record {
            class,
            fields: values,
        }))
    }

    fn read_field(&mut self, field: &FieldDef) -> Result<Value, Error> {
        let legacy = field.legacy_type;
        if legacy & TYPE_ARRAY != 0 {
            let count = self.read_int()?;
            if count < 0 {
                return Err(Error::UnsupportedType(legacy));
            }
            let mut class_hint = None;
            if legacy & TYPE_MASK == TYPE_STRUCT && field.type_name.is_none() {
                let raw = self.read_int()?;
                class_hint = Some(self.class_index(raw)?);
            }
            // Every dynamic array element carries its own child count before
            // the element items (`_readArrayItems`, hkTagfileReadFormat2014).
            let mut values = Vec::new();
            for _ in 0..count {
                let children = self.read_int()?;
                if children < 0 {
                    return Err(Error::UnsupportedType(legacy));
                }
                values.extend(self.read_items(
                    legacy & !TYPE_ARRAY,
                    field.type_name.as_deref(),
                    class_hint,
                    children as usize,
                )?);
            }
            return Ok(Value::Array(values));
        }
        if legacy & TYPE_TUPLE != 0 {
            let count = field.tuple_count;
            if count < 0 {
                return Err(Error::UnsupportedType(legacy));
            }
            return Ok(Value::Array(self.read_items(
                legacy & !TYPE_TUPLE,
                field.type_name.as_deref(),
                None,
                count as usize,
            )?));
        }
        self.read_single(legacy, field.type_name.as_deref(), None)
    }

    #[allow(clippy::too_many_arguments)]
    fn read_items(
        &mut self,
        legacy: i64,
        type_name: Option<&str>,
        class_hint: Option<usize>,
        count: usize,
    ) -> Result<Vec<Value>, Error> {
        if count > 1 << 20 {
            return Err(Error::Truncated);
        }
        let basic = legacy & TYPE_MASK;
        if basic == TYPE_VEC4 && count != 0 {
            // Arrays of vec4 store one component count (3 or 4) before the items.
            let components = self.read_int()?;
            if components != 3 && components != 4 {
                return Err(Error::InvalidVectorCount(components));
            }
            let mut values = Vec::with_capacity(count);
            for _ in 0..count {
                let mut components_value = Vec::with_capacity(4);
                for _ in 0..components {
                    components_value.push(Value::Float(self.read_real()?));
                }
                for _ in components..4 {
                    components_value.push(Value::Float(0.0));
                }
                values.push(Value::Array(components_value));
            }
            return Ok(values);
        }
        if basic == TYPE_STRUCT {
            if let Some(class) = class_hint {
                let mut values = Vec::with_capacity(count);
                for _ in 0..count {
                    values.push(self.read_record(class)?);
                }
                return Ok(values);
            }
        }
        let mut values = Vec::with_capacity(count);
        for _ in 0..count {
            values.push(self.read_single(legacy, type_name, class_hint)?);
        }
        Ok(values)
    }

    fn read_single(
        &mut self,
        legacy: i64,
        type_name: Option<&str>,
        class_hint: Option<usize>,
    ) -> Result<Value, Error> {
        if legacy & (TYPE_ARRAY | TYPE_TUPLE) != 0 {
            return Err(Error::UnsupportedType(legacy));
        }
        match legacy {
            TYPE_VOID => Ok(Value::Void),
            TYPE_BYTE => Ok(Value::Int(i64::from(self.take(1)?[0]))),
            TYPE_INT => self.read_int().map(Value::Int),
            TYPE_REAL => self.read_real().map(Value::Float),
            TYPE_VEC4 | TYPE_VEC8 | TYPE_VEC12 | TYPE_VEC16 => {
                let components = match legacy {
                    TYPE_VEC4 => 4,
                    TYPE_VEC8 => 8,
                    TYPE_VEC12 => 12,
                    _ => 16,
                };
                let mut values = Vec::with_capacity(components);
                for _ in 0..components {
                    values.push(Value::Float(self.read_real()?));
                }
                Ok(Value::Array(values))
            }
            TYPE_OBJECT => self.read_pointer(),
            TYPE_STRUCT => {
                let name = type_name.ok_or(Error::NullClassName)?;
                let class = class_hint
                    .or_else(|| self.class_by_name.get(name).copied())
                    .ok_or(Error::InvalidClass(0))?;
                self.read_record(class)
            }
            TYPE_CSTRING => Ok(self.read_string()?.map_or(Value::Null, Value::String)),
            _ => Err(Error::UnsupportedType(legacy)),
        }
    }

    fn read_object(&mut self, tag: i64) -> Result<(Value, bool), Error> {
        if tag == TAG_OBJECT_NULL {
            return Ok((Value::Null, false));
        }
        if tag == TAG_OBJECT_BACKREF {
            let id = self.read_int()?;
            let object = self
                .remembered
                .get(id as usize)
                .copied()
                .unwrap_or(usize::MAX);
            return Ok((
                if object == usize::MAX {
                    Value::ForwardRef(id)
                } else {
                    Value::Ref(object)
                },
                false,
            ));
        }
        let raw_class = self.read_int()?;
        let class = self.class_index(raw_class)?;
        let value = self.read_record(class)?;
        Ok((value, tag == TAG_OBJECT_REMEMBER))
    }

    fn read(mut self) -> Result<Bundle2014, Error> {
        loop {
            let tag = self.read_int()?;
            match tag {
                TAG_EOF | TAG_FILE_END => break,
                TAG_FILE_INFO => {
                    self.version = self.read_int()?;
                    match self.version {
                        0 | 1 => return Err(Error::LegacyVersion(self.version)),
                        2..=6 => {}
                        _ => return Err(Error::UnsupportedVersion(self.version)),
                    }
                    if self.version >= 4 {
                        self.sdk_version = self.read_string()?;
                    }
                    if self.version >= 5 {
                        let _max_predicate = self.fixed_u16()?;
                        let verified = self.fixed_u16()?;
                        for _ in 0..verified {
                            let _ = self.fixed_u16()?;
                        }
                    }
                    self.double_precision = self.version >= 6;
                }
                TAG_METADATA => self.read_class()?,
                TAG_OBJECT | TAG_OBJECT_REMEMBER | TAG_OBJECT_NULL | TAG_OBJECT_BACKREF => {
                    let (value, remembered) = self.read_object(tag)?;
                    if remembered {
                        self.remembered.push(self.objects.len());
                    }
                    self.objects.push(value);
                }
                other => return Err(Error::UnsupportedType(other)),
            }
        }
        if self.version < 0 {
            return Err(Error::UnsupportedVersion(self.version));
        }
        for object in &mut self.objects {
            resolve_forward_refs(object, &self.remembered)?;
        }
        Ok(Bundle2014 {
            version: self.version,
            sdk_version: self.sdk_version,
            double_precision: self.double_precision,
            classes: self.classes,
            objects: self.objects,
        })
    }
}

fn resolve_forward_refs(value: &mut Value, remembered: &[usize]) -> Result<(), Error> {
    match value {
        Value::ForwardRef(id) => {
            let object = remembered.get(*id as usize).copied().unwrap_or(usize::MAX);
            if object == usize::MAX {
                return Err(Error::UnresolvedReference(*id));
            }
            *value = Value::Ref(object);
        }
        Value::Array(values) => {
            for item in values {
                resolve_forward_refs(item, remembered)?;
            }
        }
        Value::Record(record) => {
            for (_, item) in &mut record.fields {
                resolve_forward_refs(item, remembered)?;
            }
        }
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn varint(mut value: u64, negative: bool, out: &mut Vec<u8>) {
        let first = ((value & 0x3f) << 1) | u64::from(negative);
        value >>= 6;
        if value == 0 {
            out.push(first as u8);
            return;
        }
        out.push((first | 0x80) as u8);
        while value != 0 {
            let byte = value & 0x7f;
            value >>= 7;
            out.push((byte | if value == 0 { 0 } else { 0x80 }) as u8);
        }
    }

    fn int(value: i64, out: &mut Vec<u8>) {
        varint(value.unsigned_abs(), value < 0, out);
    }

    fn string(text: &str, out: &mut Vec<u8>) {
        if text.is_empty() {
            int(0, out);
        } else {
            int(text.len() as i64, out);
            out.extend_from_slice(text.as_bytes());
        }
    }

    fn header(version: i64, out: &mut Vec<u8>) {
        out.extend_from_slice(&BINARY_MAGIC_0.to_le_bytes());
        out.extend_from_slice(&BINARY_MAGIC_1.to_le_bytes());
        int(TAG_FILE_INFO, out);
        int(version, out);
    }

    fn class(name: &str, fields: &[(&str, i64, Option<&str>)], out: &mut Vec<u8>) {
        int(TAG_METADATA, out);
        string(name, out);
        int(1, out); // class version
        int(0, out); // no parent
        int(fields.len() as i64, out);
        for (field_name, legacy, type_name) in fields {
            string(field_name, out);
            int(*legacy, out);
            if let Some(name) = type_name {
                string(name, out);
            }
        }
    }

    #[test]
    fn parses_object_fields_and_class_metadata() {
        let mut bytes = Vec::new();
        header(2, &mut bytes);
        class(
            "Demo",
            &[
                ("count", TYPE_INT, None),
                ("ratio", TYPE_REAL, None),
                ("label", TYPE_CSTRING, None),
            ],
            &mut bytes,
        );
        int(TAG_OBJECT, &mut bytes);
        int(1, &mut bytes); // class index
        bytes.push(0b111); // all three fields present
        int(42, &mut bytes);
        bytes.extend_from_slice(&1.5f32.to_le_bytes());
        string("hi", &mut bytes);
        int(TAG_FILE_END, &mut bytes);

        let bundle = Bundle2014::load(&bytes).expect("bundle");
        assert_eq!(bundle.version, 2);
        assert!(!bundle.double_precision);
        assert_eq!(bundle.classes.len(), 2);
        assert_eq!(bundle.classes[1].as_ref().unwrap().name, "Demo");
        let Value::Record(record) = &bundle.objects[0] else {
            panic!("not a record");
        };
        assert_eq!(record.class, 1);
        assert_eq!(
            record.fields,
            vec![
                ("count".to_string(), Value::Int(42)),
                ("ratio".to_string(), Value::Float(1.5)),
                ("label".to_string(), Value::String("hi".to_string())),
            ]
        );
    }

    #[test]
    fn resolves_backward_and_forward_object_references() {
        let mut bytes = Vec::new();
        header(2, &mut bytes);
        class("Node", &[("child", TYPE_OBJECT, Some("Node"))], &mut bytes);
        int(TAG_OBJECT_REMEMBER, &mut bytes);
        int(1, &mut bytes);
        bytes.push(0b1);
        int(2, &mut bytes); // forward reference to the next remembered object
        int(TAG_OBJECT_REMEMBER, &mut bytes);
        int(1, &mut bytes);
        bytes.push(0);
        int(TAG_OBJECT_BACKREF, &mut bytes);
        int(1, &mut bytes);
        int(TAG_FILE_END, &mut bytes);

        let bundle = Bundle2014::load(&bytes).expect("bundle");
        assert_eq!(bundle.objects.len(), 3);
        let Value::Record(first) = &bundle.objects[0] else {
            panic!("first object");
        };
        assert_eq!(first.fields[0].1, Value::Ref(1));
        assert_eq!(bundle.objects[2], Value::Ref(0));
    }

    #[test]
    fn version_six_uses_double_precision_and_sdk_string() {
        let mut bytes = Vec::new();
        header(6, &mut bytes);
        string("20211202", &mut bytes);
        bytes.extend_from_slice(&0u16.to_le_bytes());
        bytes.extend_from_slice(&0u16.to_le_bytes());
        class("D", &[("x", TYPE_REAL, None)], &mut bytes);
        int(TAG_OBJECT, &mut bytes);
        int(1, &mut bytes);
        bytes.push(0b1);
        bytes.extend_from_slice(&2.5f64.to_le_bytes());
        int(TAG_FILE_END, &mut bytes);

        let bundle = Bundle2014::load(&bytes).expect("bundle");
        assert!(bundle.double_precision);
        assert_eq!(bundle.sdk_version.as_deref(), Some("20211202"));
        let Value::Record(record) = &bundle.objects[0] else {
            panic!("not a record");
        };
        assert_eq!(record.fields[0].1, Value::Float(2.5));
    }

    #[test]
    fn rejects_bad_magic_and_legacy_versions() {
        assert_eq!(Bundle2014::load(&[0; 8]), Err(Error::BadMagic));

        let mut bytes = Vec::new();
        header(1, &mut bytes);
        int(TAG_FILE_END, &mut bytes);
        assert_eq!(Bundle2014::load(&bytes), Err(Error::LegacyVersion(1)));
    }
}
