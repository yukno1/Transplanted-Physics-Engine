//! `hknp-world` — world assembly and simulation skeleton.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/World/`
//! (migration Phase 6, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`WorldCinfo`] | `hknpWorldCinfo` | builder with gravity/capacities/solver settings |
//! | [`World`] | `hknpWorld` | owns body/motion managers, environment and step skeleton |
//! | [`WorldStats`] | counters | snapshot of body/motion/step counts |
//!
//! # Phase 6 scope
//!
//! - world creation from [`WorldCinfo`], gravity accessors, body creation
//!   (`allocateBody` + `addBody` collapsed into [`World::create_body`]),
//!   destruction, transform/velocity accessors and cached AABB queries;
//! - [`World::step`] integrates dynamic motions (linear velocity + angular
//!   velocity) and refreshes body transforms/AABBs.
//!
//! Collision detection (`stepCollide`) arrives with `hknp-collide` (Phase 7);
//! contact solving and gravity integration (`stepSolve`) arrive with
//! `hknp-solver` (Phase 8). Until then `step` is intentionally a kinematic
//! skeleton and does **not** resolve contacts or apply gravity.
//!
//! # Not ported in this phase
//!
//! Broadphase/space splitter, body addition queues, command queue/determinism
//! checking, deactivation manager, collision filters, events/modifiers,
//! user body buffers, `hknpWorldReader`/`hknpWorldWriter` traits, body
//! attachment and motion sharing. Those follow with Phases 7–10.

pub mod cinfo;
pub mod world;

pub use cinfo::WorldCinfo;
pub use world::World;

/// Snapshot of world counters (`WorldStats`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldStats {
    /// Number of live bodies.
    pub body_count: usize,
    /// Number of live non-static motions.
    pub motion_count: usize,
    /// Number of completed steps.
    pub step_count: u64,
}
