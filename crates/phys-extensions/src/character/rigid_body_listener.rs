//! Listener for the rigid-body character route
//! (`hknpCharacterRigidBodyListener` subset).
//!
//! The SDK passes `hknpSolverData` to `onPostCollide`; this port exposes a
//! local, copyable contact summary instead. There is no Jacobian access and
//! no VDB monitor callback.

use base_math::Vector4;
use phys_types::BodyId;

/// One contact point visible to the rigid-body character listener.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPointInfo {
    /// Other body.
    pub body: BodyId,
    /// Contact position in world space.
    pub position: Vector4,
    /// Normal from the other body towards the character.
    pub normal: Vector4,
    /// Signed separation (negative when penetrating).
    pub distance: f32,
}

/// Copyable rigid-body character state passed to `onPostSolve`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterRigidBodyState {
    /// Character body id.
    pub body: BodyId,
    /// World position.
    pub position: Vector4,
    /// Linear velocity.
    pub velocity: Vector4,
    /// Whether the most recent support check found a walkable surface.
    pub supported: bool,
}

/// Three-entry subset of `hknpCharacterRigidBodyListener`.
pub trait CharacterRigidBodyListener: Send {
    /// Decide whether a contact point participates in the character step.
    /// Returning `false` disables it (SDK: `disableContacts`).
    fn filter_contact_point(&mut self, _point: &mut ContactPointInfo) -> bool {
        true
    }

    /// Contacts collected after collision; the SDK allows adding/removing
    /// contacts here, this subset only exposes filtered observations.
    fn on_post_collide(&mut self, _points: &[ContactPointInfo]) {}

    /// Once after the world step, intended for control logic.
    fn on_post_solve(&mut self, _state: &CharacterRigidBodyState) {}
}
