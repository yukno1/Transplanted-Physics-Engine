//! `phys-extensions` — hknp extensions.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Extensions/`
//! (migration Phase 10, README §4).
//!
//! | Module | Original C++ | Scope |
//! |---|---|---|
//! | [`world_hash`] | `Extensions/WorldHash` | deterministic FNV-1a hash of `WorldSnapshot` |
//! | [`vehicle`] | `Extensions/Vehicle` | ray-cast suspension + default engine/transmission + simplified friction solver (Phase 13 subset) |
//! | [`character`] | `Extensions/CharacterControl` | surface info + simplex solver + character proxy + state machine (OnGround/Jumping/InAir/Climbing) + trigger tracking + local listener/phantom presence + post-solve manager + interactions + rigid-body route (Phase 15 + follow-up subsets) |
//! | [`ragdoll`] | `Extensions/Ragdoll` + `KeyFrameUtil` | bone hierarchy binding, constraint creation, drive-to-pose and hard keyframes |
//! | [`ragdoll_controller`] | `Extensions/Ragdoll/StateMachine` | Keyframed/Dynamic/Powered states, timed transition and powered motor blender |
//! | [`ragdoll_penetration`] | `Extensions/Ragdoll/Utils/hknpRagdollPenetrationUtil` | root/bone penetration detection, status propagation and corrective impulses |
//! | [`ragdoll_ease`] | `Extensions/Actions/EaseConstraints` + state easing modes | save/loosen/restore ragdoll cone/twist limits, at-rest transition helper |
//!
//! # Blocked extension systems
//!
//! The SDK has no ray-casting rigid-body class; the `RayCasting` utility is
//! available as [`character::get_character_params`]. VDB streams and raw
//! block-stream Jacobian memory are not ported; local monitor snapshots and
//! pre-solve row views are available through
//! [`character::CharacterContactBridge`] / [`character::CharacterMonitorEvent`].
//! Ragdoll easing now covers
//! the limited-hinge and 6DOF limit sets and the state manager accepts ten
//! user-defined states, optional ragdoll plane-angle and 6DOF per-axis
//! translation limits, and shared-motion mass merges. The `ActionManager`
//! integration, full inertia-tensor mass properties and non-diagonal 6DOF
//! translations remain out of scope. The explicit ragdoll driving subset
//! lives in [`ragdoll`].

pub mod character;
pub mod ragdoll;
pub mod ragdoll_controller;
pub mod ragdoll_ease;
pub mod ragdoll_penetration;
pub mod vehicle;
pub mod world_hash;

/// World-level event names are prefixed to keep them distinct from the
/// query-based character [`TriggerEvent`] / [`TriggerStatus`] pair.
pub use crate::phys_world::{
    ContactEditAction, ContactModifier, ContactModifierId, ContactModifierRegistry,
    EditableContact, PostCollideEvent as WorldPostCollideEvent,
    SolverDataHook as WorldSolverDataHook, SolverDataHookId as WorldSolverDataHookId,
    SolverDataView as WorldSolverDataView, TriggerEvent as WorldTriggerEvent,
    TriggerEventMerger as WorldTriggerEventMerger, TriggerSignal as WorldTriggerSignal,
    TriggerStatus as WorldTriggerStatus,
};

pub use character::{
    CharacterContactBridge, CharacterContext, CharacterInput, CharacterInteractionInfo,
    CharacterMonitorEvent, CharacterMonitorPhase, CharacterMonitorSpan,
    CharacterObjectInteractionEvent, CharacterObjectInteractionResult, CharacterOutput,
    CharacterParam, CharacterProxy, CharacterProxyCinfo, CharacterProxyListener,
    CharacterProxyManager, CharacterProxyState, CharacterRigidBody, CharacterRigidBodyCinfo,
    CharacterRigidBodyListener, CharacterRigidBodyState, CharacterState, CharacterStateManager,
    CharacterStateType, Contact as CharacterContact, ContactPointInfo, DeferredInteraction,
    RayCastingCharacterCinfo, SupportedState, SurfaceInfo, TriggerEvent, TriggerObservation,
    TriggerStatus, TriggerTracker, get_character_params, shape_cast_from_body,
};

pub use ragdoll::{Ragdoll, RagdollBone, RagdollData, RagdollError, RagdollLimits};
pub use ragdoll_controller::{
    BlendedRagdollMotor, RagdollController, RagdollControllerCinfo, RagdollMotorParams,
    RagdollState, RagdollStateBehavior, RagdollStateContext, USER_STATE_COUNT,
};
pub use ragdoll_ease::{
    ConstraintEasingMode, DEFAULT_EASING_DURATION, EaseConstraintsAction, LimitSnapshot,
    LimitedHingeLimitSnapshot, RagdollLimitSnapshot, SMALL_TOLERANCE, SixDofLimitSnapshot, lerp,
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

#[cfg(test)]
#[path = "tests/character_climbing.rs"]
mod character_climbing;
#[cfg(test)]
#[path = "tests/character_contact_editing.rs"]
mod character_contact_editing;
#[cfg(test)]
#[path = "tests/character_control.rs"]
mod character_control;
#[cfg(test)]
#[path = "tests/character_interactions.rs"]
mod character_interactions;
#[cfg(test)]
#[path = "tests/character_listeners.rs"]
mod character_listeners;
#[cfg(test)]
#[path = "tests/character_monitor.rs"]
mod character_monitor;
#[cfg(test)]
#[path = "tests/character_multithreading.rs"]
mod character_multithreading;
#[cfg(test)]
#[path = "tests/character_post_solve.rs"]
mod character_post_solve;
#[cfg(test)]
#[path = "tests/character_ray_casting.rs"]
mod character_ray_casting;
#[cfg(test)]
#[path = "tests/character_rigid_body.rs"]
mod character_rigid_body;
#[cfg(test)]
#[path = "tests/character_shape_key_triggers.rs"]
mod character_shape_key_triggers;
#[cfg(test)]
#[path = "tests/character_solver_data.rs"]
mod character_solver_data;
#[cfg(test)]
#[path = "tests/character_trigger_events.rs"]
mod character_trigger_events;
