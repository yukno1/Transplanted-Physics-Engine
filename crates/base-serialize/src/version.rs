//! Serialization format detection and a synthetic version-patching subset.
//!
//! The 2021.2 SDK has no in-tree `hkVersionRegistry`: patch records are
//! compiled C++ `hkReflect::Version::PatchInfo` constants registered with a
//! `PatchRegistry` and applied to a loaded `hkSerialize::Bundle`. This module
//! models the structural operations (member rename/remove/add/default-set)
//! as Rust values and applies them to the generic `Value` trees produced by
//! [`crate::object::Bundle`]. There is no patch encoding inside hkx files, so
//! callers supply the table, just as the SDK links in compiled patch lists.
//!
//! The 2014 binary packfile container parser itself lives in
//! `hkCompatFormats.dll` and is not available here; only its 16-byte header
//! sniffer is reproduced by [`detect_format`].

use crate::object::{Bundle, Record, Value};
use crate::type_meta::TypeInfo;

/// `BinaryPackfile2014::MAGIC_0`.
pub const BINARY_PACKFILE_MAGIC_0: u32 = 0x57e0_e057;
/// `BinaryPackfile2014::MAGIC_1`.
pub const BINARY_PACKFILE_MAGIC_1: u32 = 0x10c0_c010;
/// 2014 binary tagfile magic (either endianness).
pub const LEGACY_TAGFILE_MAGIC_0: u32 = 0xcab0_0d1e;
/// Companion magic for the 2014 binary tagfile header.
pub const LEGACY_TAGFILE_MAGIC_1: u32 = 0xd011_face;
/// NT6 binary tagfile companion magic.
pub const NT6_MAGIC_1: u32 = 0xdead_c0de;

/// Serialization formats recognised by `hkSerializeDetail::createReadFormat`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SerializeFormat {
    /// 2014 binary packfile (16-byte header; body converted by a DLL).
    BinaryPackfile2014,
    /// 2015+/2021 binary tagfile with a `TAG0` root.
    BinaryTagfile,
    /// 2015+/2021 type compendium with a `TCM0` root.
    BinaryTagfileCompendium,
    /// 2014 binary tagfile stream (`CAB00D1E` / `D011FACE`).
    LegacyBinaryTagfile,
    /// NT6 binary tagfile (`CAB00D1E` / `DEADC0DE`).
    Nt6BinaryTagfile,
    /// XML packfile.
    XmlPackfile,
    /// XML tagfile.
    XmlTagfile,
    /// Unrecognised bytes.
    Unknown,
}

fn u32_le(bytes: &[u8], offset: usize) -> Option<u32> {
    let slice = bytes.get(offset..offset + 4)?;
    Some(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

fn four_cc(bytes: &[u8], offset: usize, value: &[u8; 4]) -> bool {
    bytes.get(offset..offset + 4) == Some(value.as_slice())
}

/// Detect a serialization format from a file prefix.
pub fn detect_format(bytes: &[u8]) -> SerializeFormat {
    if bytes.len() >= 16
        && u32_le(bytes, 0) == Some(BINARY_PACKFILE_MAGIC_0)
        && u32_le(bytes, 4) == Some(BINARY_PACKFILE_MAGIC_1)
    {
        return SerializeFormat::BinaryPackfile2014;
    }
    if four_cc(bytes, 4, b"TAG0") || four_cc(bytes, 4, b"TAGF") {
        return SerializeFormat::BinaryTagfile;
    }
    if four_cc(bytes, 4, b"TCM0") || four_cc(bytes, 4, b"TCMP") {
        return SerializeFormat::BinaryTagfileCompendium;
    }
    if let Some(first) = u32_le(bytes, 0) {
        if first == LEGACY_TAGFILE_MAGIC_0 {
            match u32_le(bytes, 4) {
                Some(NT6_MAGIC_1) => return SerializeFormat::Nt6BinaryTagfile,
                Some(LEGACY_TAGFILE_MAGIC_1) => return SerializeFormat::LegacyBinaryTagfile,
                _ => {}
            }
        }
        if first.swap_bytes() == LEGACY_TAGFILE_MAGIC_0 {
            if u32_le(bytes, 4) == Some(LEGACY_TAGFILE_MAGIC_1.swap_bytes()) {
                return SerializeFormat::LegacyBinaryTagfile;
            }
            if u32_le(bytes, 4) == Some(NT6_MAGIC_1.swap_bytes()) {
                return SerializeFormat::Nt6BinaryTagfile;
            }
        }
    }
    if bytes.starts_with(b"<hkpackfile ") || bytes.starts_with(b"<hkpackfile>") {
        return SerializeFormat::XmlPackfile;
    }
    if bytes.starts_with(b"<hktagfile") {
        return SerializeFormat::XmlTagfile;
    }
    SerializeFormat::Unknown
}

/// `hkReflect::Version::LegacyType` values used by patch records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LegacyType {
    /// No value.
    Void = 0,
    /// Byte / bool.
    Byte = 1,
    /// Integer.
    Int = 2,
    /// Floating point.
    Real = 3,
    /// Four-component vector.
    Vec4 = 4,
    /// Eight-component vector.
    Vec8 = 5,
    /// Twelve-component vector.
    Vec12 = 6,
    /// Sixteen-component vector.
    Vec16 = 7,
    /// Object reference.
    Object = 8,
    /// Embedded struct.
    Struct = 9,
    /// C string.
    CString = 10,
    /// Dynamic array flag (`TYPE_ARRAY`).
    Array = 0x10,
}

/// Default value written by `MEMBER_ADDED` / `MEMBER_DEFAULT_SET`.
#[derive(Clone, Debug, PartialEq)]
pub enum PatchDefault {
    /// Integer default.
    Int(i128),
    /// Floating-point default.
    Float(f64),
    /// Boolean default.
    Bool(bool),
    /// String default.
    String(String),
    /// Four-component vector default.
    Vec4([f32; 4]),
    /// Null pointer default.
    NullPointer,
    /// Zero / empty default.
    Zero,
}

impl PatchDefault {
    /// Materialise the default as a generic value.
    pub fn to_value(&self) -> Value {
        match self {
            Self::Int(value) => Value::Int(*value),
            Self::Float(value) => Value::Float(*value),
            Self::Bool(value) => Value::Bool(*value),
            Self::String(value) => Value::String(value.clone()),
            Self::Vec4(value) => {
                Value::Array(value.iter().map(|v| Value::Float(*v as f64)).collect())
            }
            Self::NullPointer => Value::NullPointer,
            Self::Zero => Value::Int(0),
        }
    }
}

/// One structural patch component (`hkReflect::Version::Component` subset).
#[derive(Clone, Debug, PartialEq)]
pub enum PatchComponent {
    /// `PATCH_MEMBER_RENAMED`.
    MemberRenamed {
        /// Field name in the old type.
        old: String,
        /// Field name in the new type.
        new: String,
    },
    /// `PATCH_MEMBER_REMOVED`.
    MemberRemoved {
        /// Field to drop.
        name: String,
    },
    /// `PATCH_MEMBER_ADDED` with a typed default.
    MemberAdded {
        /// Field to append.
        name: String,
        /// Declared legacy type (kept for fidelity).
        ty: LegacyType,
        /// Value written into loaded records.
        default: PatchDefault,
    },
    /// `PATCH_MEMBER_DEFAULT_SET`.
    MemberDefaultSet {
        /// Field whose value is replaced.
        name: String,
        /// Replacement value.
        default: PatchDefault,
    },
}

/// Sentinel used by `HK_CLASS_ADDED` and `HK_CLASS_REMOVED`.
pub const CLASS_ADDED: i32 = -1;
/// Sentinel used by `HK_CLASS_REMOVED`.
pub const CLASS_REMOVED: i32 = -1;

/// `hkReflect::Version::PatchInfo` subset.
#[derive(Clone, Debug, PartialEq)]
pub struct PatchInfo {
    /// Source class name (empty for a newly added class).
    pub old_name: String,
    /// Target class name (empty for a removed class).
    pub new_name: String,
    /// Source declared version.
    pub old_version: i32,
    /// Target declared version.
    pub new_version: i32,
    /// Structural operations, applied in order.
    pub components: Vec<PatchComponent>,
}

impl PatchInfo {
    /// Construct a patch from version `old_version` to `new_version`.
    pub fn new(
        old_name: impl Into<String>,
        new_name: impl Into<String>,
        old_version: i32,
        new_version: i32,
    ) -> Self {
        Self {
            old_name: old_name.into(),
            new_name: new_name.into(),
            old_version,
            new_version,
            components: Vec::new(),
        }
    }

    /// Append a component.
    #[must_use]
    pub fn with_component(mut self, component: PatchComponent) -> Self {
        self.components.push(component);
        self
    }

    /// Whether the class was renamed.
    pub fn is_rename(&self) -> bool {
        self.old_name != self.new_name
    }

    /// Whether this patch introduces a class.
    pub fn is_class_added(&self) -> bool {
        self.old_name.is_empty()
    }

    /// Whether this patch removes a class.
    pub fn is_class_removed(&self) -> bool {
        self.new_name.is_empty()
    }
}

/// Operations applied during one patch run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PatchReport {
    /// Number of matched record types.
    pub records_patched: usize,
    /// Number of renamed fields.
    pub fields_renamed: usize,
    /// Number of removed fields.
    pub fields_removed: usize,
    /// Number of appended fields.
    pub fields_added: usize,
    /// Number of overwritten defaults.
    pub defaults_set: usize,
}

/// Registered patch table (`PatchRegistry` / `PatchSet` subset).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PatchSet {
    patches: Vec<PatchInfo>,
}

impl PatchSet {
    /// Empty patch table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a patch; lookup follows registration order.
    pub fn add(&mut self, patch: PatchInfo) {
        self.patches.push(patch);
    }

    /// All patches whose source class/version match the given type.
    pub fn patches_for(&self, type_name: &str, version: i32) -> Vec<&PatchInfo> {
        self.patches
            .iter()
            .filter(|patch| patch.old_name == type_name && patch.old_version == version)
            .collect()
    }

    /// Apply all matching patches recursively to a loaded bundle.
    pub fn apply(&self, bundle: &mut Bundle) -> PatchReport {
        let mut report = PatchReport::default();
        let Bundle {
            type_info, objects, ..
        } = bundle;
        for value in objects.iter_mut() {
            patch_value(value, type_info, self, &mut report);
        }
        report
    }
}

fn patch_value(
    value: &mut Value,
    type_info: &TypeInfo,
    patches: &PatchSet,
    report: &mut PatchReport,
) {
    match value {
        Value::Record(record) => {
            if let Some((name, version)) = type_key(type_info, record.type_id) {
                let matching = patches.patches_for(&name, version);
                if !matching.is_empty() {
                    report.records_patched += 1;
                    for patch in matching {
                        apply_components(record, patch, report);
                    }
                }
            }
            for (_, field) in &mut record.fields {
                patch_value(field, type_info, patches, report);
            }
        }
        Value::Array(values) => {
            for value in values {
                patch_value(value, type_info, patches, report);
            }
        }
        _ => {}
    }
}

fn type_key(type_info: &TypeInfo, type_id: u32) -> Option<(String, i32)> {
    let meta = type_info.types.get(type_id as usize)?.as_ref()?;
    let name = meta.name.as_ref().or_else(|| {
        type_info
            .identities
            .get(type_id as usize)
            .and_then(|identity| identity.as_ref())
            .and_then(|identity| identity.name.as_ref())
    })?;
    Some((name.clone(), meta.version.unwrap_or(0) as i32))
}

fn apply_components(record: &mut Record, patch: &PatchInfo, report: &mut PatchReport) {
    for component in &patch.components {
        match component {
            PatchComponent::MemberRenamed { old, new } => {
                if let Some((name, _)) = record.fields.iter_mut().find(|(name, _)| name == old) {
                    *name = new.clone();
                    report.fields_renamed += 1;
                }
            }
            PatchComponent::MemberRemoved { name } => {
                let before = record.fields.len();
                record.fields.retain(|(field, _)| field != name);
                report.fields_removed += before - record.fields.len();
            }
            PatchComponent::MemberAdded { name, default, .. } => {
                if !record.fields.iter().any(|(field, _)| field == name) {
                    record.fields.push((name.clone(), default.to_value()));
                    report.fields_added += 1;
                }
            }
            PatchComponent::MemberDefaultSet { name, default } => {
                if let Some((_, value)) = record.fields.iter_mut().find(|(field, _)| field == name)
                {
                    *value = default.to_value();
                } else {
                    record.fields.push((name.clone(), default.to_value()));
                }
                report.defaults_set += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::type_meta::{TypeInfo, TypeMeta};

    #[test]
    fn detects_all_binary_headers() {
        let mut packfile = vec![0u8; 16];
        packfile[0..4].copy_from_slice(&BINARY_PACKFILE_MAGIC_0.to_le_bytes());
        packfile[4..8].copy_from_slice(&BINARY_PACKFILE_MAGIC_1.to_le_bytes());
        assert_eq!(
            detect_format(&packfile),
            SerializeFormat::BinaryPackfile2014
        );

        let mut modern = vec![0u8; 8];
        modern[4..8].copy_from_slice(b"TAG0");
        assert_eq!(detect_format(&modern), SerializeFormat::BinaryTagfile);
        modern[4..8].copy_from_slice(b"TCM0");
        assert_eq!(
            detect_format(&modern),
            SerializeFormat::BinaryTagfileCompendium
        );

        let mut legacy = Vec::new();
        legacy.extend_from_slice(&LEGACY_TAGFILE_MAGIC_0.to_le_bytes());
        legacy.extend_from_slice(&LEGACY_TAGFILE_MAGIC_1.to_le_bytes());
        assert_eq!(detect_format(&legacy), SerializeFormat::LegacyBinaryTagfile);
        legacy[4..8].copy_from_slice(&NT6_MAGIC_1.to_le_bytes());
        assert_eq!(detect_format(&legacy), SerializeFormat::Nt6BinaryTagfile);

        assert_eq!(detect_format(b""), SerializeFormat::Unknown);
        assert_eq!(
            detect_format(b"<hkpackfile >"),
            SerializeFormat::XmlPackfile
        );
    }

    #[test]
    fn patch_selection_matches_class_and_version() {
        let mut patches = PatchSet::new();
        patches.add(PatchInfo::new("OldNode", "Node", 1, 2));
        patches.add(PatchInfo::new("OldNode", "Node", 2, 3));
        assert_eq!(patches.patches_for("OldNode", 1).len(), 1);
        assert_eq!(patches.patches_for("OldNode", 2).len(), 1);
        assert!(patches.patches_for("Node", 1).is_empty());
        let rename = PatchInfo::new("Old", "New", 0, 1);
        assert!(rename.is_rename());
        assert!(!rename.is_class_added());
        assert!(PatchInfo::new("", "New", CLASS_ADDED, 1).is_class_added());
    }

    #[test]
    fn components_transform_records() {
        let meta = TypeMeta {
            id: 1,
            parent: 0,
            name: Some("OldNode".into()),
            format: None,
            subtype: None,
            version: Some(1),
            size: 0,
            align: 1,
            flags: 0,
            fields: Vec::new(),
            interfaces: Vec::new(),
        };
        let type_info = TypeInfo {
            identities: vec![None, None],
            types: vec![None, Some(meta)],
        };
        let mut record = Record {
            type_id: 1,
            fields: vec![
                ("m_old".into(), Value::Int(7)),
                ("m_drop".into(), Value::Bool(true)),
            ],
        };
        let patch = PatchInfo::new("OldNode", "Node", 1, 2)
            .with_component(PatchComponent::MemberRenamed {
                old: "m_old".into(),
                new: "m_new".into(),
            })
            .with_component(PatchComponent::MemberRemoved {
                name: "m_drop".into(),
            })
            .with_component(PatchComponent::MemberAdded {
                name: "m_added".into(),
                ty: LegacyType::Real,
                default: PatchDefault::Float(1.5),
            })
            .with_component(PatchComponent::MemberDefaultSet {
                name: "m_new".into(),
                default: PatchDefault::Int(9),
            });
        let mut report = PatchReport::default();
        apply_components(&mut record, &patch, &mut report);
        assert_eq!(record.fields[0].0, "m_new");
        assert_eq!(record.fields[0].1, Value::Int(9));
        assert_eq!(record.fields[1], ("m_added".into(), Value::Float(1.5)));
        assert_eq!(report.fields_renamed, 1);
        assert_eq!(report.fields_removed, 1);
        assert_eq!(report.fields_added, 1);
        assert_eq!(report.defaults_set, 1);
        assert_eq!(type_key(&type_info, 1).unwrap(), ("OldNode".into(), 1));
    }
}
