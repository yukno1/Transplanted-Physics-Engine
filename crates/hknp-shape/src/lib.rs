//! `hknp-shape` — hknp shape hierarchy.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Collide/Shape/`.
//!
//! Planned contents (migration Phase 5):
//! - `Shape` trait + enum dispatch (port of the `hknpShapeFunctions` vtable)
//! - convex shapes: Sphere, Box, Capsule, Cylinder, Triangle, ConvexHull
//! - composite shapes: Mesh (BVH), HeightField, Compound, BreakableCompound
//! - decorator shapes, dummy/null shape
//! - shape tag codecs, mass properties, shape key iteration
//!
//! Dispatch strategy: prefer a `Shape` enum with inline match over trait
//! objects where cache friendliness matters; keep trait objects only for
//! user-provided custom shapes.
//!
//! This crate is currently a scaffold.
