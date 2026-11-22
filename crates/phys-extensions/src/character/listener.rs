//! Character proxy listener subset.
//!
//! Rust port of `hknpCharacterProxyListener` callbacks that are local to the
//! proxy: contact added/removed, shape changed, the per-step character
//! callback, trigger enter/exit, plus object/character interaction callbacks.
//! Listeners dispatch in reverse registration order, like the SDK's `fire*`
//! helpers.
//!
//! Constraint filtering, VDB monitor callbacks and the rigid-body route's
//! listener are out of scope for this trait (see `rigid_body_listener`).

use base_math::Vector4;
use phys_types::BodyId;

use super::proxy::Contact;
use super::triggers::TriggerEvent;

/// Snapshot passed to [`CharacterProxyListener::character_callback`].
///
/// The callback receives a value rather than `&CharacterProxy` so listeners
/// can be stored inside the proxy without a borrow conflict.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterProxyState {
    /// Manager-assigned proxy id (zero for standalone proxies).
    pub id: u64,
    /// Final world position for the step.
    pub position: Vector4,
    /// Final linear velocity for the step.
    pub velocity: Vector4,
    /// Number of contacts in the manifold at callback time.
    pub contact_count: usize,
}

/// Read-only input of `objectInteractionCallback`
/// (`hknpCharacterObjectInteractionEvent` subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterObjectInteractionEvent {
    /// Body being pushed by the character.
    pub body: BodyId,
    /// Contact point on the body.
    pub position: Vector4,
    /// Contact normal from the body towards the character.
    pub normal: Vector4,
    /// Expected scalar impulse magnitude (`m_objectImpulse`).
    pub object_impulse: f32,
    /// Step duration.
    pub timestep: f32,
    /// Projected relative velocity along the normal.
    pub projected_velocity: f32,
    /// Reciprocal mass of the hit body.
    pub object_mass_inv: f32,
}

/// Mutable result of `objectInteractionCallback`
/// (`hknpCharacterObjectInteractionResult` subset).
///
/// Defaults to the computed scalar impulse along the contact normal; listeners
/// may change magnitude/direction or zero it to cancel the push.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterObjectInteractionResult {
    /// Vector impulse applied to the object.
    pub object_impulse: Vector4,
    /// Point at which the impulse is applied.
    pub impulse_position: Vector4,
}

/// Summary of a character↔character interaction
/// (`characterInteractionCallback` subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterInteractionInfo {
    /// Id of the other proxy.
    pub other_id: u64,
    /// Contact position between the two proxies.
    pub position: Vector4,
    /// Normal from the other proxy towards this one.
    pub normal: Vector4,
}

/// Local subset of `hknpCharacterProxyListener`.
pub trait CharacterProxyListener: Send {
    /// A contact was appended to the manifold.
    fn contact_added(&mut self, _contact: &Contact) {}

    /// A contact was removed from the manifold.
    fn contact_removed(&mut self, _contact: &Contact) {}

    /// The proxy's shape changed; fired immediately when a listener is added.
    fn shape_changed(&mut self) {}

    /// One callback after the proxy's movement step.
    ///
    /// Standalone proxies fire this at the end of `CharacterProxy::integrate`.
    /// Proxies owned by a `CharacterProxyManager` receive it from the world's
    /// post-solve signal instead.
    fn character_callback(&mut self, _state: &CharacterProxyState) {}

    /// A trigger volume was entered or exited (forwarded from `TriggerTracker`).
    fn trigger_volume_interaction(&mut self, _event: TriggerEvent) {}

    /// Constraints from the simplex solve were processed
    /// (`fireConstraintsProcessed`).
    fn process_constraints(&mut self, _contact_count: usize) {}

    /// This proxy interacted with another character proxy.
    fn character_interaction(&mut self, _info: &CharacterInteractionInfo) {}

    /// This proxy pushed a dynamic body. The result is passed through every
    /// listener in reverse registration order and may be modified.
    fn object_interaction(
        &mut self,
        _event: &CharacterObjectInteractionEvent,
        _result: &mut CharacterObjectInteractionResult,
    ) {
    }
}
