//! Bridge between world contact hooks and `CharacterRigidBodyListener`.
//!
//! [`CharacterContactBridge`] is both a deterministic [`ContactModifier`] and
//! a post-collide dispatcher. It looks up the listener slot of a managed
//! character by body id, maps the world's contact rows into the listener's
//! [`ContactPointInfo`], and maps filter decisions/normal edits back to the
//! solver rows. Listener lookups never hold the registry lock while callbacks
//! run, so the bridge stays `Send + Sync` and safe for parallel step paths.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use crate::base_math::Vector4;
use crate::phys_types::BodyId;
use crate::phys_world::{
    ContactEditAction, ContactModifier, EditableContact, PostCollideEvent, SolverDataHook,
    SolverDataView,
};

use super::monitor::{CharacterMonitorEvent, CharacterMonitorPhase};
use super::rigid_body::RigidBodyListenerCell;
use super::rigid_body_listener::{CharacterRigidBodyListener, ContactPointInfo};

type ListenerHandle = Arc<Mutex<Box<dyn CharacterRigidBodyListener>>>;

/// Shared registry of character listener slots keyed by body id.
#[derive(Clone, Default)]
pub struct CharacterContactBridge {
    listeners: Arc<Mutex<BTreeMap<BodyId, RigidBodyListenerCell>>>,
}

impl CharacterContactBridge {
    /// Register a character's shared listener slot.
    pub fn register(&self, body: BodyId, slot: RigidBodyListenerCell) {
        self.listeners
            .lock()
            .expect("character contact bridge lock poisoned")
            .insert(body, slot);
    }

    /// Stop forwarding world contact hooks for a character.
    pub fn unregister(&self, body: BodyId) {
        self.listeners
            .lock()
            .expect("character contact bridge lock poisoned")
            .remove(&body);
    }

    /// Remove all registrations.
    pub fn clear(&self) {
        self.listeners
            .lock()
            .expect("character contact bridge lock poisoned")
            .clear();
    }

    /// Number of registered characters.
    pub fn len(&self) -> usize {
        self.listeners
            .lock()
            .expect("character contact bridge lock poisoned")
            .len()
    }

    /// Whether no character is registered.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn listener_for(&self, body: BodyId) -> Option<ListenerHandle> {
        let slot = {
            let listeners = self
                .listeners
                .lock()
                .expect("character contact bridge lock poisoned");
            Arc::clone(listeners.get(&body)?)
        };
        let guard = slot.lock().expect("character listener lock poisoned");
        guard.as_ref().cloned()
    }

    fn point_for(character: BodyId, contact: &EditableContact) -> ContactPointInfo {
        let character_is_a = character == contact.body_a;
        ContactPointInfo {
            body: if character_is_a {
                contact.body_b
            } else {
                contact.body_a
            },
            position: contact.position,
            normal: if character_is_a {
                contact.normal
            } else {
                -contact.normal
            },
            distance: -contact.penetration,
        }
    }

    /// Dispatch filtered world contact points to registered characters.
    ///
    /// Both bodies of a pair are visited in canonical id order; when both are
    /// managed characters each receives the same points with its own normal
    /// convention. Listeners are invoked without holding the registry lock.
    pub fn dispatch_post_collide(&self, event: &PostCollideEvent) {
        let mut bodies = [event.body_a, event.body_b];
        bodies.sort_unstable();
        for character in bodies {
            let Some(listener) = self.listener_for(character) else {
                continue;
            };
            let character_is_a = character == event.body_a;
            let points: Vec<ContactPointInfo> = event
                .points
                .iter()
                .map(|point| ContactPointInfo {
                    body: if character_is_a {
                        event.body_b
                    } else {
                        event.body_a
                    },
                    position: point.position,
                    normal: if character_is_a {
                        point.normal
                    } else {
                        -point.normal
                    },
                    distance: -point.penetration,
                })
                .collect();
            let mut listener = listener.lock().unwrap_or_else(|error| error.into_inner());
            listener.on_post_collide(&points);
            listener.monitor(&CharacterMonitorEvent {
                step_count: event.step_count,
                phase: CharacterMonitorPhase::PostCollide,
                body: character,
                position: points.first().map_or(Vector4::ZERO, |point| point.position),
                linear_velocity: Vector4::ZERO,
                contact_count: points.len(),
            });
        }
    }
}

impl ContactModifier for CharacterContactBridge {
    fn modify(&self, contact: &mut EditableContact) -> ContactEditAction {
        let mut bodies = [contact.body_a, contact.body_b];
        bodies.sort_unstable();
        for character in bodies {
            let Some(listener) = self.listener_for(character) else {
                continue;
            };
            let character_is_a = character == contact.body_a;
            let mut point = Self::point_for(character, contact);
            let keep = {
                let mut listener = listener.lock().unwrap_or_else(|error| error.into_inner());
                listener.filter_contact_point(&mut point)
            };
            if !keep {
                return ContactEditAction::Drop;
            }
            contact.penetration = (-point.distance).max(0.0);
            let mapped = if character_is_a {
                point.normal
            } else {
                -point.normal
            };
            if mapped.length3() > 1e-10 {
                contact.normal = mapped.normalized3();
            }
            // Only the first managed character edits the row, keeping the
            // result independent of registration order.
            return ContactEditAction::Keep;
        }
        ContactEditAction::Keep
    }
}

impl SolverDataHook for CharacterContactBridge {
    fn modify(&self, data: &mut SolverDataView<'_>) {
        let mut bodies = [data.body_a, data.body_b];
        bodies.sort_unstable();
        for character in bodies {
            let Some(listener) = self.listener_for(character) else {
                continue;
            };
            let mut listener = listener.lock().unwrap_or_else(|error| error.into_inner());
            listener.modify_solver_data(data);
        }
    }
}
