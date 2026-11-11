//! `hknp-extensions` — hknp extensions.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Extensions/`
//! (migration Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`world_hash`] | `Extensions/WorldHash` | deterministic FNV-1a hash of `WorldSnapshot` |
//!
//! # Blocked extension systems
//!
//! Ragdoll, Vehicle and CharacterControl are not ported. They depend on systems
//! that are still missing from the port:
//!
//! * **Ragdoll** — constraints/joints (`ConstraintAtom`/`hknpConstraint`), body
//!   attachment/motion sharing, and a constraint solver.
//! * **Vehicle** — wheel/ray-cast and shape-cast queries, friction/torque
//!   constraints, and a vehicle-specific action scheduler.
//! * **CharacterControl** — shape-cast/penetration queries, character proxy
//!   state, and collision filtering/contact listeners.
//!
//! They are recorded as blocked in `docs/phase9-test-matrix.md` (the original
//! UnitTest matrix) rather than approximated here.

pub mod world_hash;

pub use world_hash::{FNV_OFFSET_BASIS, FNV_PRIME, WorldHash, hash_snapshot, hash_world};
