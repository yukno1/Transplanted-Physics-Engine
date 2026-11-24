//! Pre-solve Jacobian-like row view (`hknpSolverData` subset).
//!
//! The SDK lets `hknpCharacterRigidBody::onPostCollideSignal` read and edit
//! solver data before the solve. This module exposes the deterministic part:
//! a contact row's normal, tangent basis, penetration, friction and warm-start
//! impulses. It intentionally does **not** reproduce `hknpSolverData`'s
//! block-stream memory layout or raw `hknpContactJacobian` pointers.
//!
//! Hooks run after [`crate::phys_world::ContactModifier`] callbacks and before islands and
//! the solver are built, in registration order over a per-step snapshot.

use std::fmt;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::base_math::Vector4;
use crate::phys_types::BodyId;

/// Listener handle returned by [`SolverDataHookRegistry::register`].
pub type SolverDataHookId = u64;

/// Mutable view of one Jacobian-like contact row.
pub struct SolverDataView<'a> {
    /// First body of the pair.
    pub body_a: BodyId,
    /// Second body of the pair.
    pub body_b: BodyId,
    /// Unit contact normal (B → A).
    pub normal: &'a mut Vector4,
    /// Deterministic first tangent from [`proton::solver::contact_tangents`].
    pub tangent_1: &'a mut Vector4,
    /// Deterministic second tangent from [`proton::solver::contact_tangents`].
    pub tangent_2: &'a mut Vector4,
    /// Non-negative penetration depth.
    pub penetration: &'a mut f32,
    /// Coulomb friction coefficient.
    pub friction: &'a mut f32,
    /// Cached normal impulse used for warm starting.
    pub normal_impulse: &'a mut f32,
    /// Cached first tangent impulse.
    pub tangent_impulse_1: &'a mut f32,
    /// Cached second tangent impulse.
    pub tangent_impulse_2: &'a mut f32,
}

impl SolverDataView<'_> {
    /// Clamp cached impulses to the box approximation of the friction cone.
    ///
    /// The normal impulse is non-negative and each tangential component is
    /// bounded by `friction * normal_impulse`, matching the solver's box clamp.
    pub fn clamp_impulses(&mut self) {
        *self.normal_impulse = self.normal_impulse.max(0.0);
        let limit = self.friction.max(0.0) * *self.normal_impulse;
        *self.tangent_impulse_1 = self.tangent_impulse_1.clamp(-limit, limit);
        *self.tangent_impulse_2 = self.tangent_impulse_2.clamp(-limit, limit);
    }
}

/// Deterministic pre-solve row editor.
pub trait SolverDataHook: Send + Sync {
    /// Inspect and optionally edit one row. Must be side-effect free.
    fn modify(&self, data: &mut SolverDataView<'_>);
}

type StoredHook = (SolverDataHookId, Arc<dyn SolverDataHook>);

/// Thread-safe ordered registry of solver-data hooks.
#[derive(Clone, Default)]
pub struct SolverDataHookRegistry {
    hooks: Arc<Mutex<Vec<StoredHook>>>,
    next_id: Arc<AtomicU64>,
}

impl fmt::Debug for SolverDataHookRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SolverDataHookRegistry")
            .field("hooks", &self.len())
            .finish()
    }
}

impl SolverDataHookRegistry {
    /// Register a hook; callbacks run in ascending id (registration) order.
    pub fn register(&self, hook: Arc<dyn SolverDataHook>) -> SolverDataHookId {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.hooks
            .lock()
            .expect("solver data hook registry poisoned")
            .push((id, hook));
        id
    }

    /// Remove a hook by id.
    pub fn unregister(&self, id: SolverDataHookId) -> bool {
        let mut hooks = self
            .hooks
            .lock()
            .expect("solver data hook registry poisoned");
        if let Some(index) = hooks.iter().position(|entry| entry.0 == id) {
            hooks.remove(index);
            true
        } else {
            false
        }
    }

    /// Remove every hook.
    pub fn clear(&self) {
        self.hooks
            .lock()
            .expect("solver data hook registry poisoned")
            .clear();
    }

    /// Number of registered hooks.
    pub fn len(&self) -> usize {
        self.hooks
            .lock()
            .expect("solver data hook registry poisoned")
            .len()
    }

    /// Whether no hooks are registered.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Snapshot used by one step so registration changes cannot split a solve.
    pub fn snapshot(&self) -> Vec<Arc<dyn SolverDataHook>> {
        self.hooks
            .lock()
            .expect("solver data hook registry poisoned")
            .iter()
            .map(|(_, hook)| Arc::clone(hook))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct ScaleImpulse(f32);
    impl SolverDataHook for ScaleImpulse {
        fn modify(&self, view: &mut SolverDataView<'_>) {
            *view.normal_impulse *= self.0;
            view.clamp_impulses();
        }
    }

    #[test]
    fn registry_applies_in_order_and_clamps() {
        let registry = SolverDataHookRegistry::default();
        let id = registry.register(Arc::new(ScaleImpulse(2.0)));
        let hooks = registry.snapshot();
        let mut normal = Vector4::UNIT_Y;
        let mut tangent_1 = Vector4::UNIT_X;
        let mut tangent_2 = Vector4::UNIT_Z;
        let mut penetration = 0.1;
        let mut friction = 0.5;
        let mut normal_impulse = 2.0f32;
        let mut tangent_impulse_1 = 4.0f32;
        let mut tangent_impulse_2 = -4.0f32;
        let mut view = SolverDataView {
            body_a: BodyId::new(1),
            body_b: BodyId::new(2),
            normal: &mut normal,
            tangent_1: &mut tangent_1,
            tangent_2: &mut tangent_2,
            penetration: &mut penetration,
            friction: &mut friction,
            normal_impulse: &mut normal_impulse,
            tangent_impulse_1: &mut tangent_impulse_1,
            tangent_impulse_2: &mut tangent_impulse_2,
        };
        hooks[0].modify(&mut view);
        assert_eq!(normal_impulse, 4.0);
        assert_eq!(tangent_impulse_1, 2.0);
        assert_eq!(tangent_impulse_2, -2.0);
        assert!(registry.unregister(id));
        assert!(registry.is_empty());
    }
}
