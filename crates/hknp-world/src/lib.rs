//! `hknp-world` — hknp world and simulation.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/{World,Simulation}/`.
//!
//! Planned contents (migration Phases 6–8):
//! - `World` (port of `hknpWorld`) and `WorldCinfo` builder
//! - `Simulation` trait with `collide()` / `solve()` phases
//!   (single-threaded implementation first, mirroring `Simulation/SingleThreaded/`)
//! - step input, `SimulationStage` state machine
//! - modifier manager, event dispatch, deactivation manager
//!
//! This crate is currently a scaffold.
