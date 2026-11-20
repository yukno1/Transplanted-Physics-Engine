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
//!   shape-cast driven movement).
//! * [`state_machine`] — `hknpCharacterStateManager` / `hknpCharacterContext`
//!   with the four wired default states (OnGround / Jumping / InAir / Climbing).
//! * [`triggers`] — per-frame trigger enter/exit events by AABB overlap diff.
//!
//! The `RigidBody` and `RayCasting` routes depend on the post-step
//! deactivation/contact framework and are not ported (see README Phase 15).
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

pub mod proxy;
pub mod simplex_solver;
pub mod state_machine;
pub mod surface;
pub mod triggers;

pub use proxy::{CharacterProxy, CharacterProxyCinfo, Contact};
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
pub use triggers::{TriggerEvent, TriggerStatus, TriggerTracker};
