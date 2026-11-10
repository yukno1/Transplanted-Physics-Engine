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
//!
//! # Determinism
//!
//! Contacts are solved in insertion order and within each sweep in that same
//! order, with only scalar-then-vector arithmetic. The world builds bodies in
//! body-slot order and contacts in broad-phase pair order, so a given scene
//! produces the same impulse sequence every step.

pub mod solver;

pub use solver::{ContactConstraint, Solver, SolverBody, SolverSettings, SolverStats};
