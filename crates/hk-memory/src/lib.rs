//! `hk-memory` — Havok memory system.
//!
//! Rust port of `original/Common/Source/Common/Base/Memory/`.
//!
//! Planned contents (migration Phase 3):
//! - block stream allocators (port of `hkBlockStream` allocators)
//! - heap allocators and the memory router/system
//! - arena helpers used by the physics caches
//!
//! The C++ design is allocator-heavy; the Rust port replaces raw pointers
//! with ID/index access wherever possible and only keeps custom allocators
//! where block-stream semantics are actually required.
//!
//! This crate is currently a scaffold.
