//! `hknp-world` — world assembly and full single-threaded step pipeline.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/World/`
//! (migration Phase 6, extended in Phase 8, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`WorldCinfo`] | `hknpWorldCinfo` | builder with gravity/capacities/solver settings |
//! | [`World`] | `hknpWorld` | owns body/motion managers and runs collide → solve → integrate |
//! | [`ContactManifold`] | manifold stream item | body pair plus [`hknp_collide::Manifold`] |
//! | [`WorldStats`] | counters | body/motion/step/contact counters |
//!
//! # Step pipeline (Phase 8)
//!
//! [`World::step`] runs the single-threaded simulation pipeline:
//!
//! 1. **Collide**: build world AABBs, run the deterministic grid broad phase,
//!    then narrow-phase convex-convex contact generation (`hknp-collide`).
//! 2. **Solve**: apply gravity to dynamic motions, build solver bodies and
//!    single-point contact constraints, run the iterative sequential-impulse
//!    solver (`hknp-solver`), and write solved velocities back to motions.
//! 3. **Integrate**: semi-implicit Euler integration of dynamic motions and
//!    refresh of body transforms/AABBs for the next step.
//!
//! The pipeline is deliberately single-threaded and deterministic: bodies are
//! visited in slot order, contacts in canonical broad-phase pair order, and the
//! solver sweeps contacts in insertion order.
//!
//! # Scope limits
//!
//! Only convex shapes participate in narrow-phase contacts; mesh/height-field/
//! compound bodies generate no contacts yet. The solver uses one contact point
//! per pair (multi-point clipping is deferred), together with a velocity-level
//! Baumgarte recovery bias and Coulomb friction. Material libraries, restitution,
//! warm starting, islands, deactivation, broad-phase layers, command queues,
//! events/modifiers and thread-parallel solving remain future work.
//!
//! # Not ported in this phase
//!
//! Body addition queues, command queue/determinism checking, deactivation
//! manager, collision filters, events/modifiers, user body buffers,
//! `hknpWorldReader`/`hknpWorldWriter` traits, body attachment and motion
//! sharing.

use hknp_collide::Manifold;
use hknp_types::BodyId;

pub mod cinfo;
pub mod world;

pub use cinfo::WorldCinfo;
pub use world::World;

/// One body pair with a generated contact manifold.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// Body A (lower broad-phase pair id; manifold normal points from B to A).
    pub body_a: BodyId,
    /// Body B.
    pub body_b: BodyId,
    /// Narrow-phase manifold (normal B → A, positions on B).
    pub manifold: Manifold,
}

/// Snapshot of world counters (`WorldStats`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldStats {
    /// Number of live bodies.
    pub body_count: usize,
    /// Number of live non-static motions.
    pub motion_count: usize,
    /// Number of completed steps.
    pub step_count: u64,
    /// Manifolds generated during the most recent step.
    pub last_manifold_count: usize,
    /// Contact points generated during the most recent step.
    pub last_contact_count: usize,
}
