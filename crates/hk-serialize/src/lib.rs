//! `hk-serialize` — Havok serialization container layer.
//!
//! Rust port of `original/Common/Source/Common/Serialize/` (migration
//! Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`hff`] | `hkSerialize::HffReaderBase` / `HffWriter` | structured section tree parser (branch/leaf, sizes, FourCC) |
//! | [`tagfile`] | `hkTagfileReadFormat` container layout | `TAG0`/`TCM0` root, `SDKV`/`DATA`/`TYPE`/`INDX` section access |
//!
//! # Scope
//!
//! This crate implements the **container layer**: HFF section parsing and
//! tagfile section access. It does **not** rebuild objects from the `TYPE`/`DATA`
//! sections; that requires the Havok reflection/type registry, in-place patch
//! fixups and version-specific object layouts, which are not ported.
//!
//! It is therefore not yet a drop-in replacement for `hkx` asset loading. The
//! parser is dependency-free and intended for diagnostics, container inspection
//! and future object-loading work.
//!
//! # Not ported
//!
//! * Reflection type registry / `hkClass`/`hkClassMember` metadata.
//! * Object data rebuild (`ReadFormat::read`), patch/fixup application, bundle
//!   and type-compendium semantics.
//! * Binary packfile (`hkBinaryPackfile`) sections and native pointer patching.
//! * Writers, XML formats, resource handles and 2014 compatibility variants.

pub mod hff;
pub mod tagfile;

pub use hff::{HffError, HffNode, HffScope, four_cc, ident_name, pack_scope};
pub use tagfile::{COMPENDIUM, DATA, INDEX, SDK_VERSION, TAGFILE, TYPES, Tagfile, TagfileError};
