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
//! | [`ragdoll`] | `Extensions/Ragdoll` + `KeyFrameUtil` | bone hierarchy binding, constraint creation, drive-to-pose and hard keyframes |
//! | [`ragdoll_controller`] | `Extensions/Ragdoll/StateMachine` | Keyframed/Dynamic/Powered states, timed transition and powered motor blender |
//! | [`ragdoll_penetration`] | `Extensions/Ragdoll/Utils/hknpRagdollPenetrationUtil` | root/bone penetration detection, status propagation and corrective impulses |
//! | [`ragdoll_ease`] | `Extensions/Actions/EaseConstraints` + state easing modes | save/loosen/restore ragdoll cone/twist limits, at-rest transition helper |
//!
//! # Blocked extension systems
//!
//! The CharacterControl `RigidBody` / `RayCasting` routes are not ported; they
//! need the post-step deactivation/contact framework.  Limited-hinge and 6DOF
//! easing, motion sharing and the `ActionManager` integration are out of scope;
//! the proxy/state subset lives in [`character`] and the explicit ragdoll
//! driving subset lives in [`ragdoll`].

pub mod character;
pub mod ragdoll;
pub mod ragdoll_controller;
pub mod ragdoll_ease;
pub mod ragdoll_penetration;
pub mod vehicle;
pub mod world_hash;

pub use character::{
    CharacterContext, CharacterInput, CharacterOutput, CharacterProxy, CharacterProxyCinfo,
    CharacterState, CharacterStateManager, CharacterStateType, Contact as CharacterContact,
    SupportedState, SurfaceInfo,
};

pub use ragdoll::{Ragdoll, RagdollBone, RagdollData, RagdollError, RagdollLimits};
pub use ragdoll_controller::{
    BlendedRagdollMotor, RagdollController, RagdollControllerCinfo, RagdollMotorParams,
    RagdollState,
};
pub use ragdoll_ease::{
    ConstraintEasingMode, DEFAULT_EASING_DURATION, EaseConstraintsAction, RagdollLimitSnapshot,
    SMALL_TOLERANCE, lerp,
};
pub use ragdoll_penetration::{
    BodyHierarchy, DetectionOutput, EXCLUDED, HierarchyNode, NO_PARENT, Penetration,
    PenetrationInput, PenetrationStatus, ROOT, ResolveMode, corrective_impulse,
    detect_and_resolve_penetration, detect_penetration,
};

pub use vehicle::{
    DriverInput, EngineOutput, EngineSetup, TransmissionSetup, VehicleInstance, VehicleSetup,
    WheelSetup, WheelState, calc_engine,
};
pub use world_hash::{FNV_OFFSET_BASIS, FNV_PRIME, WorldHash, hash_snapshot, hash_world};
