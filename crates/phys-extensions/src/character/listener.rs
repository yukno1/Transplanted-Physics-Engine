//! Character proxy listener subset.
//!
//! Rust port of `hknpCharacterProxyListener` callbacks that are local to the
//! proxy: contact added/removed, shape changed, the per-step character
//! callback, and trigger enter/exit. Listeners dispatch in reverse
//! registration order, like the SDK's `fire*` helpers.
//!
//! Constraint filtering, character↔character, object interaction and VDB
//! monitor callbacks are not ported; the rigid-body route's listener is also
//! out of scope.

use base_math::Vector4;

use super::proxy::Contact;
use super::triggers::TriggerEvent;

/// Snapshot passed to [`CharacterProxyListener::character_callback`].
///
/// The callback receives a value rather than `&CharacterProxy` so listeners
/// can be stored inside the proxy without a borrow conflict.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterProxyState {
    /// Final world position for the step.
    pub position: Vector4,
    /// Final linear velocity for the step.
    pub velocity: Vector4,
    /// Number of contacts in the manifold at callback time.
    pub contact_count: usize,
}

/// Local subset of `hknpCharacterProxyListener`.
pub trait CharacterProxyListener {
    /// A contact was appended to the manifold.
    fn contact_added(&mut self, _contact: &Contact) {}

    /// A contact was removed from the manifold.
    fn contact_removed(&mut self, _contact: &Contact) {}

    /// The proxy's shape changed; fired immediately when a listener is added.
    fn shape_changed(&mut self) {}

    /// One callback after the proxy's movement step.
    ///
    /// The SDK fires this from the world's post-solve signal; this port has no
    /// action/signal manager, so it fires at the end of `CharacterProxy::integrate`.
    fn character_callback(&mut self, _state: &CharacterProxyState) {}

    /// A trigger volume was entered or exited (forwarded from `TriggerTracker`).
    fn trigger_volume_interaction(&mut self, _event: TriggerEvent) {}
}
