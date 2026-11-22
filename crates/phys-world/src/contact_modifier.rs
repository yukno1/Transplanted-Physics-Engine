//! Deterministic pre-solve contact editing (`hknpContactJacobian` modifier subset).
//!
//! [`ContactModifier`] callbacks run while the world flattens contact manifolds
//! into solver rows, before islands and the solver are built. They can change a
//! point's normal, position, penetration or friction, or drop the point. This
//! mirrors the part of `hknpCharacterRigidBody::filterContactPoint` that
//! disables contacts, without exposing raw Jacobian memory.
//!
//! Callbacks are invoked in registration order over a snapshot taken once per
//! step, so parallel narrow-phase and parallel solving see identical rows.
//! Dropped points are removed from the solver rows and warm-start arrays
//! together, keeping the three parallel arrays aligned.

use std::fmt;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use base_math::Vector4;
use phys_types::BodyId;

/// Listener handle returned by [`ContactModifierRegistry::register`].
pub type ContactModifierId = u64;

/// What to do with one contact point after modifiers ran.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactEditAction {
    /// Keep the (possibly modified) contact point.
    Keep,
    /// Remove the point from this step's solver and warm-start arrays.
    Drop,
}

/// A mutable contact row presented to [`ContactModifier`] callbacks.
#[derive(Clone, Debug, PartialEq)]
pub struct EditableContact {
    /// First body of the pair (low broad-phase id).
    pub body_a: BodyId,
    /// Second body of the pair.
    pub body_b: BodyId,
    /// World-space contact normal (B → A).
    pub normal: Vector4,
    /// World-space contact position (on B).
    pub position: Vector4,
    /// Non-negative penetration depth.
    pub penetration: f32,
    /// Coulomb friction coefficient (default 0.5).
    pub friction: f32,
}

/// Deterministic contact-row editor.
///
/// Implementations must be deterministic and must not mutate the world: they
/// are plain `Fn`-style callbacks shared across threads.
pub trait ContactModifier: Send + Sync {
    /// Inspect and optionally edit one contact row.
    fn modify(&self, contact: &mut EditableContact) -> ContactEditAction;
}

type StoredModifier = (ContactModifierId, Arc<dyn ContactModifier>);

/// Thread-safe ordered registry of contact modifiers.
#[derive(Clone, Default)]
pub struct ContactModifierRegistry {
    modifiers: Arc<Mutex<Vec<StoredModifier>>>,
    next_id: Arc<AtomicU64>,
}

impl fmt::Debug for ContactModifierRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ContactModifierRegistry")
            .field("modifiers", &self.len())
            .finish()
    }
}

impl ContactModifierRegistry {
    /// Register a modifier; callbacks run in ascending id (registration) order.
    pub fn register(&self, modifier: Arc<dyn ContactModifier>) -> ContactModifierId {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.modifiers
            .lock()
            .expect("contact modifier registry poisoned")
            .push((id, modifier));
        id
    }

    /// Remove a modifier by id.
    pub fn unregister(&self, id: ContactModifierId) -> bool {
        let mut modifiers = self
            .modifiers
            .lock()
            .expect("contact modifier registry poisoned");
        if let Some(index) = modifiers.iter().position(|entry| entry.0 == id) {
            modifiers.remove(index);
            true
        } else {
            false
        }
    }

    /// Remove every registered modifier.
    pub fn clear(&self) {
        self.modifiers
            .lock()
            .expect("contact modifier registry poisoned")
            .clear();
    }

    /// Number of registered modifiers.
    pub fn len(&self) -> usize {
        self.modifiers
            .lock()
            .expect("contact modifier registry poisoned")
            .len()
    }

    /// Whether no modifiers are registered.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Snapshot used by one step so registration changes mid-step cannot
    /// change contact rows for part of the solver.
    pub fn snapshot(&self) -> Vec<Arc<dyn ContactModifier>> {
        self.modifiers
            .lock()
            .expect("contact modifier registry poisoned")
            .iter()
            .map(|(_, modifier)| Arc::clone(modifier))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct SetFriction(f32);
    impl ContactModifier for SetFriction {
        fn modify(&self, contact: &mut EditableContact) -> ContactEditAction {
            contact.friction = self.0;
            ContactEditAction::Keep
        }
    }

    struct DropAll;
    impl ContactModifier for DropAll {
        fn modify(&self, _: &mut EditableContact) -> ContactEditAction {
            ContactEditAction::Drop
        }
    }

    fn contact() -> EditableContact {
        EditableContact {
            body_a: BodyId::new(1),
            body_b: BodyId::new(2),
            normal: Vector4::UNIT_Y,
            position: Vector4::ZERO,
            penetration: 0.1,
            friction: 0.5,
        }
    }

    #[test]
    fn registry_applies_in_order_and_can_drop() {
        let registry = ContactModifierRegistry::default();
        let first = registry.register(Arc::new(SetFriction(0.25)));
        let snapshot = registry.snapshot();
        let mut contact = contact();
        assert_eq!(snapshot[0].modify(&mut contact), ContactEditAction::Keep);
        assert_eq!(contact.friction, 0.25);

        let second = registry.register(Arc::new(DropAll));
        let snapshot = registry.snapshot();
        assert_eq!(snapshot.len(), 2);
        assert_eq!(snapshot[1].modify(&mut contact), ContactEditAction::Drop);
        assert!(registry.unregister(first));
        assert!(registry.unregister(second));
        assert!(!registry.unregister(second));
        assert!(registry.is_empty());
    }
}
