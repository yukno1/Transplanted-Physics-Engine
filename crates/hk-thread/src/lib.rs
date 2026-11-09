//! `hk-thread` — Havok threading primitives.
//!
//! Rust port of `original/Common/Source/Common/Base/Thread/`.
//!
//! Planned contents (migration Phase 4):
//! - atomics and memory-ordering helpers (port of `hkatomic.h`)
//! - spinlock / critical section wrappers
//! - thread pool and job queue / task graph
//!   (`rayon` is a candidate for the multithreaded simulation, Phase 10)
//!
//! The single-threaded simulation path is implemented first; this crate
//! only needs the primitives until Phase 10.
//!
//! This crate is currently a scaffold.
