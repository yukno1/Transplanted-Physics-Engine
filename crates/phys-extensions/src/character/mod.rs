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
//! * [`listener`] — local subset of `hknpCharacterProxyListener`, including
//!   object/character interaction callbacks.
//! * [`manager`] — world post-solve bridge, batched stepping and serial
//!   deferred interaction application.
//! * [`rigid_body`] / [`rigid_body_listener`] — rigid-body character route and
//!   its three listener entry points.
//! * [`state_machine`] — `hknpCharacterStateManager` / `hknpCharacterContext`
//!   with the four wired default states (OnGround / Jumping / InAir / Climbing).
//! * [`triggers`] — per-frame trigger enter/exit events by AABB overlap diff.
//!
//! The `RayCasting` route and solver-level contact editing are not ported
//! (see README Phase 15 / current status).
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

pub mod listener;
pub mod manager;
pub mod proxy;
pub mod rigid_body;
pub mod rigid_body_listener;
pub mod simplex_solver;
pub mod state_machine;
pub mod surface;
pub mod triggers;

pub use listener::{
    CharacterInteractionInfo, CharacterObjectInteractionEvent, CharacterObjectInteractionResult,
    CharacterProxyListener, CharacterProxyState,
};
pub use manager::CharacterProxyManager;
pub use proxy::{CharacterProxy, CharacterProxyCinfo, Contact, DeferredInteraction};
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
