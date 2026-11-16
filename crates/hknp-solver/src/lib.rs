//! `hknp-solver` — iterative contact solver.
//!
//! Rust port of the contact branch of
//! `original/Physics/Source/Physics/Physics/Dynamics/Solver/` (migration
//! Phase 8, README §4).
//!
//! This phase ports the sequential-impulse contact solver and the integrator
//! wiring used by [`crate::Solver`]. The full constraint-atom system, friction
//! caching, warm starting, restitution modifiers and islands are not part of
//! this first iteration (see the crate-level table below and README §4).
//!
//! | Module | Contents |
//! |---|---|
//! | `solver` | `SolverBody`, `ContactConstraint`, `SolverSettings`, `Solver`, `SolverStats` |
//! | `constraint` | `JointConstraint` (`BallAndSocket`, `Hinge`, `Fixed`, `Ragdoll`, `Prismatic`, `Spring`, `PointToPlane`) and `Solver::solve_joints` |
//!
//! Joint pivots/axes are supplied in world space by the world each step; the
//! solver applies sequential impulses with a Baumgarte drift bias, unilateral
//! limits and clamped motors, in slice order.
//!
//! # Determinism
//!
//! Contacts are solved in insertion order and within each sweep in that same
//! order, with only scalar-then-vector arithmetic. The world builds bodies in
//! body-slot order and contacts in broad-phase pair order, so a given scene
//! produces the same impulse sequence every step.

pub mod constraint;
pub mod solver;

pub use constraint::{
    BallAndSocketJoint, FixedJoint, HingeJoint, HingeLimits, HingeMotor, JointConstraint,
    JointSolverStats, PointToPlaneJoint, PrismaticJoint, RagdollJoint, SpringJoint,
};
pub use solver::{ContactConstraint, Solver, SolverBody, SolverSettings, SolverStats};
