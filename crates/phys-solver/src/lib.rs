//! `phys-solver` — iterative contact solver.
//!
//! Rust port of the contact branch of
//! `original/Physics/Source/Physics/Physics/Dynamics/Solver/` (migration
//! Phase 8, README §4).
//!
//! This phase ports the sequential-impulse contact solver and the integrator
//! wiring used by [`crate::Solver`]. The full constraint-atom system, friction
//! caching, restitution modifiers and deactivation are not part of this
//! iteration (see the crate-level table below and README §4).
//!
//! | Module | Contents |
//! |---|---|
//! | `solver` | `SolverBody`, `ContactConstraint`, `ContactImpulse`, `contact_tangents`, `SolverSettings`, `Solver`, `SolverStats` |
//! | `constraint` | `JointConstraint` (`BallAndSocket`, `Hinge`, `Fixed`, `Ragdoll`, `SixDof`, `Prismatic`, `Spring`, `PointToPlane`, `Wheel`), hinge/prismatic motors (`HingeMotor`, `PrismaticMotor`), wheel suspension/drive (`WheelSuspension`, `WheelDrive`), `JointImpulse` and `Solver::solve_joints[_with_impulses]` |
//!
//! Joint pivots/axes are supplied in world space by the world each step; the
//! solver applies sequential impulses with a Baumgarte drift bias, unilateral
//! limits and clamped motors, in slice order.
//!
//! # Warm starting and islands
//!
//! [`ContactImpulse`] and [`JointImpulse`] carry per-row accumulated impulses
//! across steps, and `solve_contacts_subset` / `solve_joints_subset_with_impulses`
//! restrict a solve to one constraint island (the world's island partition).
//!
//! # Determinism
//!
//! Contacts are solved in insertion order and within each sweep in that same
//! order, with only scalar-then-vector arithmetic. Friction uses a tangent
//! basis derived from the contact normal only. The world builds bodies in
//! body-slot order and contacts in broad-phase pair order, so a given scene
//! produces the same impulse sequence every step.

pub mod constraint;
pub mod solver;

pub use constraint::{
    BallAndSocketJoint, FixedJoint, HingeJoint, HingeLimits, HingeMotor, JointConstraint,
    JointImpulse, JointSolverStats, MAX_JOINT_ROWS, PointToPlaneJoint, PrismaticJoint,
    PrismaticMotor, RagdollJoint, SixDofJoint, SpringJoint, WheelDrive, WheelJoint,
    WheelSuspension,
};
pub use solver::{
    ContactConstraint, ContactImpulse, Solver, SolverBody, SolverSettings, SolverStats,
    contact_tangents,
};
