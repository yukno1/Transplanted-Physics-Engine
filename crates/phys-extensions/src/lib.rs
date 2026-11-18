//! `phys-extensions` — hknp extensions.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Extensions/`
//! (migration Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`world_hash`] | `Extensions/WorldHash` | deterministic FNV-1a hash of `WorldSnapshot` |
//! | [`vehicle`] | `Extensions/Vehicle` | ray-cast suspension + default engine/transmission + simplified friction solver (Phase 13 subset) |
//! | [`character`] | `Extensions/CharacterControl` | surface info + simplex solver + character proxy + state machine (OnGround/Jumping/InAir) (Phase 15 subset) |
//!
//! # Blocked extension systems
//!
//! Ragdoll (the scheduling layer) and the CharacterControl `RigidBody` /
//! `RayCasting` routes are not ported. They depend on systems that are still
//! missing from the port:
//!
//! * **Ragdoll** — ragdoll-driven body binding/motion sharing and the
//!   character/ragdoll scheduling layer (the ragdoll constraint family itself
//!   is in `phys-solver`).
//! * **CharacterControl** — the rigid-body route and the ray-casting route
//!   need the post-step deactivation/contact framework; the proxy + state
//!   machine subset lives in [`character`].

pub mod character;
pub mod vehicle;
pub mod world_hash;

pub use character::{
    CharacterContext, CharacterInput, CharacterOutput, CharacterProxy, CharacterProxyCinfo,
    CharacterState, CharacterStateManager, CharacterStateType, Contact as CharacterContact,
    SupportedState, SurfaceInfo,
};

pub use vehicle::{
    DriverInput, EngineOutput, EngineSetup, TransmissionSetup, VehicleInstance, VehicleSetup,
    WheelSetup, WheelState, calc_engine,
};
pub use world_hash::{FNV_OFFSET_BASIS, FNV_PRIME, WorldHash, hash_snapshot, hash_world};
