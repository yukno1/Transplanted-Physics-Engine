//! `hknp-solver` — hknp constraint solver.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/Solver/`.
//!
//! Planned contents (migration Phase 8):
//! - constraint atom system (`ConstraintAtom/*`)
//! - iterative contact constraint solver (`Contact/*`)
//! - integrators (semi-implicit Euler, `Integrator/*`)
//! - solver info/data, velocity buffers, solver results
//!
//! Determinism is critical here: keep the exact operation order of the C++
//! solver (accumulation order, iteration order) and validate against golden
//! traces in Phase 9.
//!
//! This crate is currently a scaffold.
