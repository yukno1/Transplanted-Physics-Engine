//! `base-geometry` — Havok geometry utilities.
//!
//! Rust subset of `original/Common/Source/Common/GeometryUtilities/` and the
//! deterministic hull algorithms used by the SDK:
//!
//! - [`convex_hull`]: point-cloud 3D hull construction with deterministic
//!   QuickHull-style tie-breaking and lower-dimensional fallbacks.
//! - [`weld`]: vertex welding with remaps, degenerate-triangle removal and
//!   optional duplicate-triangle removal.
//!
//! # Scope and determinism
//!
//! This crate implements only the geometry needed to construct shapes from
//! point clouds. It does not port `hkgpConvexHull` random simplification,
//! plane merging, Boolean operations or mesh decimation, and it does not
//! depend on any `phys-*` crate.
//!
//! All predicates use f32 with fixed evaluation order. Sorting uses explicit
//! total orders with original indices as the final tie-break; no `RandomState`,
//! parallel iteration or floating-point reassociation is used. Results are
//! therefore reproducible across runs. The subset is not claimed to be
//! bit-identical to C++ internal hull coordinates.

pub mod convex_hull;
pub mod weld;

pub use convex_hull::{
    ConvexHull3D, DEFAULT_HULL_TOLERANCE, HullDimension, HullError, build_convex_hull,
    build_convex_hull_with_tolerance,
};
pub use weld::{WeldError, WeldOptions, WeldResult, weld_vertices};
