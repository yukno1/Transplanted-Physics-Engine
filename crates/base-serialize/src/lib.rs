//! `base-serialize` — Havok serialization container layer.
//!
//! Rust port of `original/Common/Source/Common/Serialize/` (migration
//! Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`hff`] | `hkSerialize::HffReaderBase` / `HffWriter` | structured section tree parser (branch/leaf, sizes, FourCC) |
//! | [`tagfile`] | `hkTagfileReadFormat` container layout | `TAG0`/`TCM0` root, `SDKV`/`DATA`/`TYPE`/`INDX` section access |
//! | [`vle`] | `hkUintVle` | variable-length integer codec used by `TYPE` |
//! | [`type_meta`] | `hkReflect::Type` / `TNA1`/`TBDY` | reflection metadata decoding |
//! | [`compendium`] | `TagfileReadFormat::Impl::Compendium` | `TCM0` signatures and shared `TYPE` tables |
//! | [`object`] | `ReadFormat::read` subset | generic object rebuild, `ITEM`/`PTCH` fixups |
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
//! * Version patching and native pointer sections.
//! * Binary packfile (`hkBinaryPackfile`) sections and XML/2014 variants.
//! * Writers, resource handles, property decls and vtable initialisation.
//! * Conversion of a generic bundle into `phys-world`/`phys-extensions` objects.

pub mod compendium;
pub mod hff;
pub mod object;
pub mod tagfile;
pub mod type_meta;
pub mod vle;

pub use compendium::Compendium;
pub use hff::{HffError, HffNode, HffScope, four_cc, ident_name, pack_scope};
pub use object::{Bundle, Fixup, Item, ItemKind, LoadError, Record, Value};
pub use tagfile::{
    COMPENDIUM, COMPENDIUM_ID, COMPENDIUM_REFERENCE, DATA, INDEX, SDK_VERSION, TAGFILE, TYPES,
    Tagfile, TagfileError,
};
pub use type_meta::{
    Field, Format, Kind, MetaError, TemplateParam, TypeId, TypeIdentity, TypeInfo, TypeMeta,
};
