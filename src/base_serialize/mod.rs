//! `base-serialize` — Havok serialization container layer.
//!
//! Rust port of `original/Common/Source/Common/Serialize/` (migration
//! Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`hff`] | `hkSerialize::HffReaderBase` / `HffWriter` | structured section tree parser (branch/leaf, sizes, FourCC) |
//! | [`tagfile`] | `hkTagfileReadFormat` container layout | `TAG0`/`TCM0` root, `SDKV`/`DATA`/`TYPE`/`INDX` section access |
//! | [`tagfile2014`] | `hkTagfileReadFormat2014` | legacy `CAB00D1E` binary tagfile: metadata, objects, references |
//! | [`vle`] | `hkUintVle` | variable-length integer codec used by `TYPE` |
//! | [`type_meta`] | `hkReflect::Type` / `TNA1`/`TBDY` | reflection metadata decoding |
//! | [`compendium`] | `TagfileReadFormat::Impl::Compendium` | `TCM0` signatures and shared `TYPE` tables |
//! | [`object`] | `ReadFormat::read` subset | generic object rebuild, `ITEM`/`PTCH` fixups |
//! | [`version`] | `hkSerializeDetail` sniffer + `hkReflect::Version::PatchInfo` subset | format detection, structural/function/type-change patches on `Value` trees |
//! | [`packfile`] | `BinaryPackfile2014` header + `hkCompatFormats` hook | header validation and external conversion boundary |
//! | [`loader`] | `hkSerializeUtil` format routing subset | unified hkx load entry with optional patching |
//!
//! # Scope
//!
//! [`object::Bundle::load`] decodes the current `TAG0` tagfile format into a
//! generic [`object::Value`] tree: type identities and bodies, item index,
//! internal/external fixups, scalars, strings, pointers, fixed/dynamic arrays
//! and records are supported.  Object data is interpreted as little-endian,
//! matching the SDK's x86 in-place layout; Havok physics objects are **not**
//! constructed here.  A `TCM0` type compendium can be loaded with
//! [`compendium::Compendium`] and passed to
//! [`object::Bundle::load_with_compendia`] when a bundle carries `TCRF`.
//!
//! # Not ported
//!
//! * Compiled Havok patch lists and native pointer sections; callers register
//!   patch tables explicitly, including deterministic function pointers.
//! * The 2014 binary packfile container parser (converted by an external SDK
//!   DLL). Headers are parsed and validated; a caller-supplied converter must
//!   produce bytes for an existing tagfile reader.
//! * Writers, resource handles, property decls and vtable initialisation.
//! * Conversion of a generic bundle into `phys-world`/`phys-extensions` objects.

pub mod compendium;
pub mod hff;
pub mod loader;
pub mod object;
pub mod packfile;
pub mod tagfile;
pub mod tagfile2014;
pub mod type_meta;
pub mod version;
pub mod vle;

pub use compendium::Compendium;
pub use hff::{HffError, HffNode, HffScope, four_cc, ident_name, pack_scope};
pub use loader::{LoadHkxError, LoadOptions, LoadedHkx, load_hkx};
pub use object::{Bundle, Fixup, Item, ItemKind, LoadError, Record, Value};
pub use packfile::{
    CURRENT_FILE_VERSION, HEADER_LEN as PACKFILE_HEADER_LEN, PackfileConverter, PackfileError,
    PackfileHeader, PackfilePolicy, convert_packfile,
};
pub use tagfile::{
    COMPENDIUM, COMPENDIUM_ID, COMPENDIUM_REFERENCE, DATA, INDEX, SDK_VERSION, TAGFILE, TYPES,
    Tagfile, TagfileError,
};
pub use tagfile2014::{
    BINARY_MAGIC_0, BINARY_MAGIC_1, Bundle2014, ClassDef, Error as Error2014, FieldDef,
    Record as Record2014, Value as Value2014,
};
pub use type_meta::{
    Field, Format, Kind, MetaError, TemplateParam, TypeId, TypeIdentity, TypeInfo, TypeMeta,
};
pub use version::{
    BINARY_PACKFILE_MAGIC_0, BINARY_PACKFILE_MAGIC_1, CLASS_ADDED, CLASS_REMOVED,
    DataObjectPatchFn, LEGACY_TAGFILE_MAGIC_0, LEGACY_TAGFILE_MAGIC_1, LegacyType,
    MAX_PATCH_CHAIN_STEPS, NT6_MAGIC_1, PatchComponent, PatchDefault, PatchError,
    PatchFunctionHelper, PatchInfo, PatchReport, PatchSet, SerializeFormat, TypeChangePatchFn,
    VarPatchFn, VarTypeChangePatchFn, detect_format,
};
