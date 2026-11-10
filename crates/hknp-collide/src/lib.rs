//! `hknp-collide` — collision detection.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Collide/`
//! (migration Phase 7, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`BroadPhase`] | `hknpBroadPhase` (grid subset) | uniform grid over world AABBs, deterministic canonical [`BodyPair`]s |
//! | [`gjk`] | `hkcdGsk::getClosestPoint` (GJK) | world-space closest points/intersection for convex shapes |
//! | `epa` | `hkcdGsk` penetration branch | penetration depth/normal from the GJK tetrahedron |
//! | [`Manifold`] / [`ContactPoint`] | `hknpManifold` / `hkcdManifold4` | normal B → A, positions on B, negative distance = penetration |
//! | [`collide`] / [`collide_with_tolerance`] | `hknpConvexConvexCollisionDetector` | analytic sphere-sphere + GJK/EPA for general convex pairs |
//!
//! # Supported pairs
//!
//! Convex-convex only: sphere, box, capsule, cylinder, triangle and convex hull.
//! Sphere-sphere uses an analytic contact; every other convex pair goes through
//! GJK and, when intersecting, EPA. Composite shapes (mesh/height field/
//! compound) have no support function, so [`collide`] returns `None` for them;
//! their leaf traversal is Phase 8+ work together with the world collision
//! pipeline.
//!
//! # Conventions
//!
//! The manifold normal points from B to A, contact positions are stored on B,
//! and a negative distance means penetration. The surface point on A is
//! `position + normal * distance`. This mirrors `hkcdManifold4`.
//!
//! # Not ported in this phase
//!
//! SIMD-tree/CsGrid broad-phase layers and `hknpBroadPhaseConfig` layer
//! semantics, collision filters, collision caches/welding, contact manifold
//! clipping (up to 4 points), composite/height-field/mesh narrow phase, SDF and
//! particle collision paths, queries (ray cast/AABB cast/closest points) and
//! continuous collision detection.

pub mod broadphase;
pub mod gjk;
pub mod manifold;
pub mod narrowphase;

pub(crate) mod epa;
pub(crate) mod support;

pub use broadphase::{BodyAabb, BodyPair, BroadPhase, BroadPhaseConfig};
pub use gjk::{GjkResult, closest_points, intersects};
pub use manifold::{ContactPoint, Manifold};
pub use narrowphase::{collide, collide_with_tolerance};
