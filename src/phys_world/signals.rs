//! Thread-safe mass-property change signals for [`crate::phys_world::World`].

use std::fmt;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use crate::base_math::{Matrix3, Quaternion, Vector4};
use crate::phys_shape::MassProperties;
use crate::phys_types::BodyId;

/// Listener handle returned by [`MassSignal::subscribe`].
pub type MassConnectionId = u64;

/// Why a body's mass properties changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MassChangeReason {
    /// The body's shape was replaced.
    ShapeReplaced,
    /// The body's `MassConfig` was replaced.
    MassConfigChanged,
    /// Shared-motion mass properties were rebuilt.
    MotionRebuilt,
    /// Inverse inertia was assigned explicitly through a world API.
    InverseInertiaSet,
}

/// Small copyable summary carried by a mass-change event.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassSummary {
    /// Resolved mass.
    pub mass: f32,
    /// Unit-density volume.
    pub volume: f32,
    /// Centre of mass.
    pub center_of_mass: Vector4,
    /// World-space inverse inertia matrix.
    pub inverse_inertia_world: Matrix3,
    /// Principal-axis frame.
    pub major_axis_space: Quaternion,
}

impl MassSummary {
    /// Summarize optional full mass properties.
    pub fn from_properties(properties: Option<&MassProperties>) -> Option<Self> {
        properties.map(|properties| Self {
            mass: properties.mass,
            volume: properties.volume,
            center_of_mass: properties.center_of_mass,
            inverse_inertia_world: Matrix3::ZERO,
            major_axis_space: properties.major_axis_space,
        })
    }
}

/// A mass-property change notification.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassChangeEvent {
    /// Body whose mass properties changed.
    pub body: BodyId,
    /// Cause of the change.
    pub reason: MassChangeReason,
    /// Summary before the change, when available.
    pub old: Option<MassSummary>,
    /// Summary after the change, when available.
    pub new: Option<MassSummary>,
}

type MassListener = Arc<dyn Fn(&MassChangeEvent) + Send + Sync>;

/// Thread-safe synchronous signal for mass changes.
///
/// Subscription order, deferred removal during firing and nested emissions
/// mirror `proton::types::Signal`, but the callback container uses `Arc<Mutex<_>>`
/// so [`crate::phys_world::World`] remains `Send + Sync`.
#[derive(Clone)]
pub struct MassSignal {
    listeners: Arc<Mutex<Vec<(MassConnectionId, MassListener)>>>,
    next_id: Arc<AtomicU64>,
    firing: Arc<AtomicUsize>,
}

impl Default for MassSignal {
    fn default() -> Self {
        Self {
            listeners: Arc::new(Mutex::new(Vec::new())),
            next_id: Arc::new(AtomicU64::new(1)),
            firing: Arc::new(AtomicUsize::new(0)),
        }
    }
}

impl fmt::Debug for MassSignal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MassSignal")
            .field("subscriptions", &self.num_subscriptions())
            .field("firing", &self.is_firing())
            .finish()
    }
}

impl MassSignal {
    /// Subscribe a callback; returned handle can be passed to [`Self::unsubscribe`].
    pub fn subscribe<F>(&self, listener: F) -> MassConnectionId
    where
        F: Fn(&MassChangeEvent) + Send + Sync + 'static,
    {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.listeners
            .lock()
            .expect("mass signal lock poisoned")
            .push((id, Arc::new(listener)));
        id
    }

    /// Remove a previously registered callback.
    pub fn unsubscribe(&self, id: MassConnectionId) -> bool {
        let mut listeners = self.listeners.lock().expect("mass signal lock poisoned");
        if let Some(index) = listeners.iter().position(|entry| entry.0 == id) {
            listeners.remove(index);
            true
        } else {
            false
        }
    }

    /// Remove all callbacks.
    pub fn unsubscribe_all(&self) {
        self.listeners
            .lock()
            .expect("mass signal lock poisoned")
            .clear();
    }

    /// Number of registered callbacks.
    pub fn num_subscriptions(&self) -> usize {
        self.listeners
            .lock()
            .expect("mass signal lock poisoned")
            .len()
    }

    /// Whether this signal is currently being emitted.
    pub fn is_firing(&self) -> bool {
        self.firing.load(Ordering::Relaxed) > 0
    }

    /// Emit an event to the callbacks registered when emission starts.
    ///
    /// Callbacks that unsubscribe during firing remain in the current pass
    /// (matching deferred removal), while callbacks added during firing are
    /// only seen by later emissions. Nested emissions are allowed.
    pub fn emit(&self, event: &MassChangeEvent) {
        let listeners = {
            let guard = self.listeners.lock().expect("mass signal lock poisoned");
            guard.clone()
        };
        self.firing.fetch_add(1, Ordering::Relaxed);
        for (_, listener) in listeners {
            listener(event);
        }
        self.firing.fetch_sub(1, Ordering::Relaxed);
    }
}

use crate::phys_world::events::{PostCollideEvent, TriggerEvent};

/// Listener handle returned by [`PostSolveSignal::subscribe`].
pub type PostSolveConnectionId = u64;

/// Listener handle returned by [`TriggerSignal::subscribe`].
pub type TriggerConnectionId = u64;

/// Listener handle returned by [`PostCollideSignal::subscribe`].
pub type PostCollideConnectionId = u64;

/// Copyable summary of a completed world step.
///
/// The event deliberately carries no `World` or proxy reference: subscribers
/// can be shared across threads and cannot borrow world state mutably during
/// a signal emission.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostSolveEvent {
    /// Number of completed steps, including the one that emitted this event.
    pub step_count: u64,
    /// Delta time used by the completed step.
    pub delta_time: f32,
    /// Contact manifolds generated during the completed step.
    pub manifold_count: usize,
    /// Contact points generated during the completed step.
    pub contact_count: usize,
    /// Constraint islands solved during the completed step.
    pub island_count: usize,
}

type PostSolveListener = Arc<dyn Fn(&PostSolveEvent) + Send + Sync>;

/// Thread-safe synchronous signal emitted after a world step is integrated.
///
/// Callbacks are invoked in subscription order over a snapshot taken when the
/// emission starts. Unsubscribing during an emission therefore only affects
/// later emissions, and nested emissions are allowed. `Arc<Mutex<_>>` keeps
/// [`crate::phys_world::World`] `Send + Sync`.
#[derive(Clone)]
pub struct PostSolveSignal {
    listeners: Arc<Mutex<Vec<(PostSolveConnectionId, PostSolveListener)>>>,
    next_id: Arc<AtomicU64>,
    firing: Arc<AtomicUsize>,
}

impl Default for PostSolveSignal {
    fn default() -> Self {
        Self {
            listeners: Arc::new(Mutex::new(Vec::new())),
            next_id: Arc::new(AtomicU64::new(1)),
            firing: Arc::new(AtomicUsize::new(0)),
        }
    }
}

impl fmt::Debug for PostSolveSignal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PostSolveSignal")
            .field("subscriptions", &self.num_subscriptions())
            .field("firing", &self.is_firing())
            .finish()
    }
}

impl PostSolveSignal {
    /// Subscribe a callback; returned handle can be passed to [`Self::unsubscribe`].
    pub fn subscribe<F>(&self, listener: F) -> PostSolveConnectionId
    where
        F: Fn(&PostSolveEvent) + Send + Sync + 'static,
    {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.listeners
            .lock()
            .expect("post-solve signal lock poisoned")
            .push((id, Arc::new(listener)));
        id
    }

    /// Remove a previously registered callback. Removals during emission are
    /// applied to the next emission.
    pub fn unsubscribe(&self, id: PostSolveConnectionId) -> bool {
        let mut listeners = self
            .listeners
            .lock()
            .expect("post-solve signal lock poisoned");
        if let Some(index) = listeners.iter().position(|entry| entry.0 == id) {
            if self.is_firing() {
                listeners[index].1 = Arc::new(|_| {});
            } else {
                listeners.remove(index);
            }
            true
        } else {
            false
        }
    }

    /// Remove all callbacks.
    pub fn unsubscribe_all(&self) {
        let mut listeners = self
            .listeners
            .lock()
            .expect("post-solve signal lock poisoned");
        if self.is_firing() {
            for (_, listener) in listeners.iter_mut() {
                *listener = Arc::new(|_| {});
            }
        } else {
            listeners.clear();
        }
    }

    /// Number of registered callbacks.
    pub fn num_subscriptions(&self) -> usize {
        self.listeners
            .lock()
            .expect("post-solve signal lock poisoned")
            .len()
    }

    /// Whether this signal is currently being emitted.
    pub fn is_firing(&self) -> bool {
        self.firing.load(Ordering::Relaxed) > 0
    }

    /// Emit an event to the callbacks registered when emission starts.
    pub fn emit(&self, event: &PostSolveEvent) {
        let listeners = {
            let guard = self
                .listeners
                .lock()
                .expect("post-solve signal lock poisoned");
            guard.clone()
        };
        self.firing.fetch_add(1, Ordering::Relaxed);
        for (_, listener) in listeners {
            listener(event);
        }
        self.firing.fetch_sub(1, Ordering::Relaxed);
    }
}

type EventListener<E> = Arc<dyn Fn(&E) + Send + Sync>;
type EventListenerEntry<E> = (u64, EventListener<E>);

/// Thread-safe synchronous signal shared by world trigger and post-collide
/// notifications.
///
/// Listener snapshots are taken under the lock and invoked in subscription
/// order after the lock is released. Unsubscribing while an emission is in
/// progress replaces the listener with a no-op for the current pass, matching
/// [`PostSolveSignal`] semantics; nested emissions are allowed.
#[derive(Clone)]
pub struct EventSignal<E> {
    listeners: Arc<Mutex<Vec<EventListenerEntry<E>>>>,
    next_id: Arc<AtomicU64>,
    firing: Arc<AtomicUsize>,
}

impl<E> Default for EventSignal<E> {
    fn default() -> Self {
        Self {
            listeners: Arc::new(Mutex::new(Vec::new())),
            next_id: Arc::new(AtomicU64::new(1)),
            firing: Arc::new(AtomicUsize::new(0)),
        }
    }
}

impl<E> fmt::Debug for EventSignal<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EventSignal")
            .field("subscriptions", &self.num_subscriptions())
            .field("firing", &self.is_firing())
            .finish()
    }
}

impl<E> EventSignal<E> {
    /// Subscribe a callback; the returned handle can be passed to [`Self::unsubscribe`].
    pub fn subscribe<F>(&self, listener: F) -> u64
    where
        F: Fn(&E) + Send + Sync + 'static,
    {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.listeners
            .lock()
            .expect("event signal lock poisoned")
            .push((id, Arc::new(listener)));
        id
    }

    /// Remove a previously registered callback.
    pub fn unsubscribe(&self, id: u64) -> bool {
        let mut listeners = self.listeners.lock().expect("event signal lock poisoned");
        if let Some(index) = listeners.iter().position(|entry| entry.0 == id) {
            if self.is_firing() {
                listeners[index].1 = Arc::new(|_| {});
            } else {
                listeners.remove(index);
            }
            true
        } else {
            false
        }
    }

    /// Remove all callbacks.
    pub fn unsubscribe_all(&self) {
        let mut listeners = self.listeners.lock().expect("event signal lock poisoned");
        if self.is_firing() {
            for (_, listener) in listeners.iter_mut() {
                *listener = Arc::new(|_| {});
            }
        } else {
            listeners.clear();
        }
    }

    /// Number of registered callbacks.
    pub fn num_subscriptions(&self) -> usize {
        self.listeners
            .lock()
            .expect("event signal lock poisoned")
            .len()
    }

    /// Whether this signal is currently being emitted.
    pub fn is_firing(&self) -> bool {
        self.firing.load(Ordering::Relaxed) > 0
    }

    /// Emit an event to the callbacks registered when emission starts.
    pub fn emit(&self, event: &E) {
        let listeners = {
            let guard = self.listeners.lock().expect("event signal lock poisoned");
            guard.clone()
        };
        self.firing.fetch_add(1, Ordering::Relaxed);
        for (_, listener) in listeners {
            listener(event);
        }
        self.firing.fetch_sub(1, Ordering::Relaxed);
    }
}

/// World trigger event signal (`hknpWorld::m_signals` trigger dispatch subset).
pub type TriggerSignal = EventSignal<TriggerEvent>;

/// World post-collide event signal (`hknpWorld::m_postCollide` subset).
pub type PostCollideSignal = EventSignal<PostCollideEvent>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_preserves_order_and_allows_unsubscribe() {
        let signal = MassSignal::default();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let first = {
            let seen = Arc::clone(&seen);
            signal.subscribe(move |_| seen.lock().unwrap().push(1))
        };
        {
            let seen = Arc::clone(&seen);
            signal.subscribe(move |_| seen.lock().unwrap().push(2));
        }
        let event = MassChangeEvent {
            body: BodyId::INVALID,
            reason: MassChangeReason::MotionRebuilt,
            old: None,
            new: None,
        };
        signal.emit(&event);
        assert_eq!(*seen.lock().unwrap(), vec![1, 2]);
        assert!(signal.unsubscribe(first));
        signal.emit(&event);
        assert_eq!(*seen.lock().unwrap(), vec![1, 2, 2]);
    }

    #[test]
    fn post_solve_signal_preserves_order_and_defers_unsubscribe() {
        let signal = PostSolveSignal::default();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let first = {
            let seen = Arc::clone(&seen);
            signal.subscribe(move |event| seen.lock().unwrap().push(event.step_count as i32))
        };
        signal.subscribe({
            let seen = Arc::clone(&seen);
            move |_| {
                seen.lock().unwrap().push(-1);
            }
        });
        let event = PostSolveEvent {
            step_count: 3,
            delta_time: 1.0 / 60.0,
            manifold_count: 1,
            contact_count: 1,
            island_count: 1,
        };
        signal.emit(&event);
        assert_eq!(*seen.lock().unwrap(), vec![3, -1]);
        assert!(signal.unsubscribe(first));
        signal.emit(&event);
        assert_eq!(*seen.lock().unwrap(), vec![3, -1, -1]);
        assert_eq!(signal.num_subscriptions(), 1);
        assert!(!signal.is_firing());
    }
}
