//! `phys-extensions` — hknp extensions.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Extensions/`
//! (migration Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`world_hash`] | `Extensions/WorldHash` | deterministic FNV-1a hash of `WorldSnapshot` |
//! | [`vehicle`] | `Extensions/Vehicle` | ray-cast suspension + default engine/transmission + simplified friction solver (Phase 13 subset) |
//!
//! # Blocked extension systems
//!
//! Ragdoll and CharacterControl are not ported. They depend on systems that
//! are still missing from the port:
//!
//! * **Ragdoll** — ragdoll-driven body binding/motion sharing and the
//!   character/ragdoll scheduling layer (the ragdoll constraint family itself
//!   is in `phys-solver`).
//! * **CharacterControl** — shape-cast/penetration queries, character proxy
//!   state, and collision filtering/contact listeners.
//!
//! They are recorded as blocked in `docs/phase9-test-matrix.md` (the original
//! UnitTest matrix) rather than approximated here.

pub mod vehicle;
pub mod world_hash;

pub use vehicle::{
    DriverInput, EngineOutput, EngineSetup, TransmissionSetup, VehicleInstance, VehicleSetup,
    WheelSetup, WheelState, calc_engine,
};
pub use world_hash::{FNV_OFFSET_BASIS, FNV_PRIME, WorldHash, hash_snapshot, hash_world};
