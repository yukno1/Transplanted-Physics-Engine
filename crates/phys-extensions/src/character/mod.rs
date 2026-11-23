//! CharacterControl extension (Phase 15): `hknpCharacterControl` subset.
//!
//! Port of `original/Physics/Source/Physics/Physics/Extensions/
//! CharacterControl/`:
//!
//! * [`surface`] — `hknpCharacterSurfaceInfo` (support state + surface
//!   velocity).
//! * [`simplex_solver`] — `hkSimplexSolver` (point motion under plane
//!   constraints with static/dynamic friction).
//! * [`proxy`] — `hknpCharacterProxy` (self-maintained contact manifold +
//!   shape-cast driven movement + listeners/phantom presence).
//! * [`ray_casting`] — `hknpRayCastingCharacterControllerUtil` (eight directional
//!   casts, step-height diffs and bump/easing parameters).
//! * [`listener`] — local subset of `hknpCharacterProxyListener`, including
//!   object/character interaction callbacks.
//! * [`manager`] — world post-solve bridge, batched stepping and serial
//!   deferred interaction application.
//! * [`monitor`] — local monitor-style diagnostic snapshots (no VDB stream).
//! * [`contact_editing`] — world contact-modifier / post-collide bridge for the
//!   rigid-body route (deterministic contact filtering without Jacobian access).
//! * [`rigid_body`] / [`rigid_body_listener`] — rigid-body character route and
//!   its three listener entry points.
//! * [`state_machine`] — `hknpCharacterStateManager` / `hknpCharacterContext`
//!   with the four wired default states (OnGround / Jumping / InAir / Climbing).
//! * [`triggers`] — per-frame trigger enter/exit events by AABB overlap diff.
//!
//! VDB streams and raw block-stream Jacobian editing remain unported. Local
//! monitor snapshots are available through [`monitor`], contact filtering and
//! pre-solve row views through [`contact_editing`], and
//! [`ray_casting`] supplies height/displacement parameters without requiring a
//! dedicated ray-cast rigid-body class (the SDK has only the utility).
//! A typical frame:
//!
//! ```ignore
//! let surface = proxy.check_support(&world, -up, dt);
//! let input = CharacterInput { surface_info: surface, velocity: proxy.linear_velocity(), .. };
//! let mut output = CharacterOutput::default();
//! context.update(&input, &mut output);
//! proxy.set_linear_velocity(output.velocity);
//! proxy.integrate(&mut world, dt, &surface);
//! ```

pub mod contact_editing;
pub mod listener;
pub mod manager;
pub mod monitor;
pub mod proxy;
pub mod ray_casting;
pub mod rigid_body;
pub mod rigid_body_listener;
pub mod simplex_solver;
pub mod state_machine;
pub mod surface;
pub mod triggers;

pub use contact_editing::CharacterContactBridge;
pub use listener::{
    CharacterInteractionInfo, CharacterObjectInteractionEvent, CharacterObjectInteractionResult,
    CharacterProxyListener, CharacterProxyState,
};
pub use manager::CharacterProxyManager;
pub use monitor::{CharacterMonitorEvent, CharacterMonitorPhase, CharacterMonitorSpan};
pub use proxy::{CharacterProxy, CharacterProxyCinfo, Contact, DeferredInteraction};
pub use ray_casting::{
    CharacterParam, RayCastingCharacterCinfo, get_character_params, shape_cast_from_body,
};
pub use rigid_body::{CharacterRigidBody, CharacterRigidBodyCinfo};
pub use rigid_body_listener::{
    CharacterRigidBodyListener, CharacterRigidBodyState, ContactPointInfo,
};
pub use simplex_solver::{
    InteractionStatus, SimplexSolverInput, SimplexSolverOutput, SurfaceConstraint,
    SurfaceInteraction, solve as simplex_solve,
};
pub use state_machine::{
    CharacterContext, CharacterInput, CharacterOutput, CharacterState, CharacterStateManager,
    CharacterStateType,
};
pub use state_machine::{movement_util, states};
pub use surface::{SupportedState, SurfaceInfo};
pub use triggers::{TriggerEvent, TriggerObservation, TriggerStatus, TriggerTracker};
