//! World-level trigger and post-collide event data (`hknpCollideEvents` subset).
//!
//! The trigger merger follows `hknpEventMergeAndDispatcher`: a shape-pair key
//! accumulates `+1` for an enter and `-1` for an exit during one step; at flush
//! time a positive count becomes [`TriggerStatus::Entered`], a negative count
//! becomes [`TriggerStatus::Exited`], and a zero count becomes
//! [`TriggerStatus::Updated`] only when the caller enables it. A zero count
//! without that flag is dropped.
//!
//! Shape keys are resolved through `phys-collide`'s shared leaf-key helper:
//! convex hull faces, mesh triangles, height-field cells and compound children
//! can produce leaf keys. Shapes without a ported subdivision still emit
//! [`ShapeKey::INVALID`], and query-based character triggers remain a separate
//! event stream.

use std::collections::BTreeMap;

use base_math::Vector4;
use phys_types::{BodyId, ShapeKey};

/// Status carried by a world-level trigger event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerStatus {
    /// Overlap started during this step (`STATUS_ENTERED`).
    Entered,
    /// Overlap ended and immediately restarted on the same shape pair
    /// (`STATUS_UPDATED`). Only emitted when explicitly enabled.
    Updated,
    /// Overlap ended during this step (`STATUS_EXITED`).
    Exited,
}

/// A world-level trigger event (`hknpTriggerEvent` subset).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerEvent {
    /// First body of the event pair (canonical low id).
    pub body_a: BodyId,
    /// Second body of the event pair.
    pub body_b: BodyId,
    /// Leaf shape key on body A (`INVALID` for non-hull pairs).
    pub shape_key_a: ShapeKey,
    /// Leaf shape key on body B (`INVALID` for non-hull pairs).
    pub shape_key_b: ShapeKey,
    /// Merged status for this step.
    pub status: TriggerStatus,
}

type TriggerPairKey = (BodyId, BodyId, ShapeKey, ShapeKey);

/// Deterministic per-step merger for trigger enter/exit events.
///
/// Entries are kept in a `BTreeMap` so that emitted events follow canonical
/// `(body_a, body_b, shape_key_a, shape_key_b)` order regardless of the order
/// in which narrow-phase candidates were visited (including parallel
/// narrow-phase).
#[derive(Clone, Debug, Default)]
pub struct TriggerEventMerger {
    counts: BTreeMap<TriggerPairKey, i32>,
}

impl TriggerEventMerger {
    /// Record an entering overlap for one shape pair.
    pub fn enter(&mut self, body_a: BodyId, body_b: BodyId, key_a: ShapeKey, key_b: ShapeKey) {
        *self
            .counts
            .entry((body_a, body_b, key_a, key_b))
            .or_insert(0) += 1;
    }

    /// Record an ending overlap for one shape pair.
    pub fn exit(&mut self, body_a: BodyId, body_b: BodyId, key_a: ShapeKey, key_b: ShapeKey) {
        *self
            .counts
            .entry((body_a, body_b, key_a, key_b))
            .or_insert(0) -= 1;
    }

    /// Whether no events are pending.
    pub fn is_empty(&self) -> bool {
        self.counts.is_empty()
    }

    /// Number of distinct shape-pair keys pending in the merger.
    pub fn pending_count(&self) -> usize {
        self.counts.len()
    }

    /// Emit merged events and clear the per-step buffer.
    pub fn flush(&mut self, raise_updated_events: bool) -> Vec<TriggerEvent> {
        let mut events = Vec::with_capacity(self.counts.len());
        for (&(body_a, body_b, key_a, key_b), &count) in &self.counts {
            let status = if count > 0 {
                TriggerStatus::Entered
            } else if count < 0 {
                TriggerStatus::Exited
            } else if raise_updated_events {
                TriggerStatus::Updated
            } else {
                continue;
            };
            events.push(TriggerEvent {
                body_a,
                body_b,
                shape_key_a: key_a,
                shape_key_b: key_b,
                status,
            });
        }
        self.counts.clear();
        events
    }
}

/// One contact point visible to post-collide listeners.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostCollideContactPoint {
    /// World-space contact position (on B, like a manifold point).
    pub position: Vector4,
    /// World-space contact normal (B → A).
    pub normal: Vector4,
    /// Non-negative penetration depth.
    pub penetration: f32,
    /// Effective friction after contact modifiers ran.
    pub friction: f32,
}

/// Snapshot of one body pair emitted before the solver builds its rows.
///
/// Listeners receive a copy; contact editing is done through
/// [`crate::ContactModifier`], not by mutating this event.
#[derive(Clone, Debug, PartialEq)]
pub struct PostCollideEvent {
    /// Completed step count of the world at emission time.
    pub step_count: u64,
    /// Delta time of the current step.
    pub delta_time: f32,
    /// First solver body.
    pub body_a: BodyId,
    /// Second solver body.
    pub body_b: BodyId,
    /// Contact points retained after contact modifiers ran.
    pub points: Vec<PostCollideContactPoint>,
    /// Number of contact points dropped by contact modifiers.
    pub dropped_point_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ids(a: u64, b: u64) -> (BodyId, BodyId) {
        (BodyId::new(a), BodyId::new(b))
    }

    #[test]
    fn merger_emits_entered_and_exited_in_key_order() {
        let (a, b) = ids(2, 1);
        let mut merger = TriggerEventMerger::default();
        merger.enter(b, a, ShapeKey::INVALID, ShapeKey::INVALID);
        merger.exit(a, ids(3, 0).0, ShapeKey::INVALID, ShapeKey::INVALID);
        let events = merger.flush(false);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].body_a, b);
        assert_eq!(events[0].body_b, a);
        assert_eq!(events[0].status, TriggerStatus::Entered);
        assert_eq!(events[1].status, TriggerStatus::Exited);
        assert!(merger.is_empty());
    }

    #[test]
    fn merger_cancels_same_step_enter_exit_and_optionally_emits_updated() {
        let (a, b) = ids(1, 2);
        let mut merger = TriggerEventMerger::default();
        merger.enter(a, b, ShapeKey::INVALID, ShapeKey::INVALID);
        merger.exit(a, b, ShapeKey::INVALID, ShapeKey::INVALID);
        assert!(merger.flush(false).is_empty());
        merger.enter(a, b, ShapeKey::INVALID, ShapeKey::INVALID);
        merger.exit(a, b, ShapeKey::INVALID, ShapeKey::INVALID);
        let events = merger.flush(true);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].status, TriggerStatus::Updated);
    }
}
