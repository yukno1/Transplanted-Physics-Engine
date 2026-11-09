//! `hknp-collide` — hknp collision detection.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Collide/`.
//!
//! Planned contents (migration Phase 7):
//! - broad phase: grid broad phase (port of `hknpBroadPhase` / CsGrid)
//! - narrow phase: GJK simplex solver, manifold generation
//!   (`hknpManifold`), per-shape-pair detectors
//! - queries: ray cast, AABB query, closest points, shape cast
//! - collision filters (`hknpCollisionFilter` trait + default group filter)
//!
//! This crate is currently a scaffold.
