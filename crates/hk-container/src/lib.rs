//! `hk-container` — Havok containers.
//!
//! Rust port of `original/Common/Source/Common/Base/Container/`.
//!
//! Planned contents (migration Phase 3):
//! - `Array` / `SmallArray` (ports of `hkArray`/`hkSmallArray`)
//! - `HashMap` / `HashSet` (port of `hkHashMap`, or `hashbrown` if layout
//!   compatibility with serialized data is not required)
//! - block streams (`hkBlockStream`, `hkTypedBlockStream`)
//! - string helpers (`hkStringPtr`, `hkStringBuf`)
//!
//! This crate is currently a scaffold.
