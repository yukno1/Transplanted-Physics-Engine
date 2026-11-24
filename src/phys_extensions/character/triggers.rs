//! Trigger-volume enter/exit/update tracking subset.
//!
//! Rust port of `hknpCharacterProxy` trigger bookkeeping
//! (`updateTriggersSeen` / `updateOverlappingTriggers`): the SDK's query
//! collector separates trigger hits from collision contacts and identifies
//! each overlap by `(bodyId, shapeKey)`. This port exposes that granularity
//! through [`TriggerTracker::update_observations`] and keeps the older
//! body-level AABB approximation in [`TriggerTracker::update`].
//!
//! Material `TRIGGER_TYPE` and broad/contact-solver modifiers remain outside
//! this subset. `Updated` here is an opt-in **observation-level** status: it is
//! emitted only when a caller supplies observations with a changed
//! position/normal for an already overlapping key. It is intentionally
//! distinct from the world-level
//! [`proton::world::TriggerStatus::Updated`],
//! which means "enter and exit merged for the same pair within one step"; the
//! two paths are independent and are never dispatched twice for one event.
//! World-level events are re-exported as `WorldTrigger*` from this crate.

use std::collections::{BTreeMap, HashSet};

use crate::base_math::{Aabb, Vector4};
use crate::phys_types::{BodyId, ShapeKey};
use crate::phys_world::{QueryFilter, World};

/// `hknpTriggerEvent::Status` subset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum TriggerStatus {
    /// Trigger became overlapping this frame (`STATUS_ENTERED`).
    Entered,
    /// Trigger overlap changed while already active (`STATUS_UPDATED` subset).
    Updated,
    /// Trigger stopped overlapping this frame (`STATUS_EXITED`).
    Exited,
}

/// One trigger enter/exit/update event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerEvent {
    /// Trigger body.
    pub body: BodyId,
    /// Leaf shape key (invalid for the legacy body-level tracker).
    pub shape_key: ShapeKey,
    /// Enter or exit status.
    pub status: TriggerStatus,
}

/// A trigger overlap observation at leaf granularity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriggerObservation {
    /// Trigger body.
    pub body: BodyId,
    /// Leaf shape key.
    pub shape_key: ShapeKey,
    /// Observed contact position (used to detect `Updated`).
    pub position: Vector4,
    /// Observed contact normal (used to detect `Updated`).
    pub normal: Vector4,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct OverlapState {
    position: Vector4,
    normal: Vector4,
}

/// Per-frame trigger overlap tracker (`hknpCharacterProxy` subset).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriggerTracker {
    overlapping: BTreeMap<(BodyId, ShapeKey), OverlapState>,
}

impl TriggerTracker {
    /// Empty tracker: no triggers are currently overlapping.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bodies overlapping in the most recent [`Self::update`].
    pub fn overlapping(&self) -> impl Iterator<Item = BodyId> + '_ {
        let mut last = None;
        self.overlapping.keys().filter_map(move |(body, _)| {
            if last == Some(*body) {
                None
            } else {
                last = Some(*body);
                Some(*body)
            }
        })
    }

    /// Advance tracking with leaf-level observations.
    ///
    /// Events are ordered by `(body, shape_key, status)`. An observation for a
    /// previously unseen key emits `Entered`; a missing key emits `Exited`.
    /// A persistent key whose position/normal changed emits `Updated`.
    pub fn update_observations(
        &mut self,
        observations: &[TriggerObservation],
    ) -> Vec<TriggerEvent> {
        let mut seen = BTreeMap::new();
        for observation in observations {
            if !observation.body.is_valid() {
                continue;
            }
            seen.insert(
                (observation.body, observation.shape_key),
                OverlapState {
                    position: observation.position,
                    normal: observation.normal,
                },
            );
        }

        let mut events = Vec::new();
        for (key, state) in &seen {
            match self.overlapping.get(key) {
                None => events.push(TriggerEvent {
                    body: key.0,
                    shape_key: key.1,
                    status: TriggerStatus::Entered,
                }),
                Some(previous) if previous != state => events.push(TriggerEvent {
                    body: key.0,
                    shape_key: key.1,
                    status: TriggerStatus::Updated,
                }),
                Some(_) => {}
            }
        }
        for key in self.overlapping.keys() {
            if !seen.contains_key(key) {
                events.push(TriggerEvent {
                    body: key.0,
                    shape_key: key.1,
                    status: TriggerStatus::Exited,
                });
            }
        }
        events.sort_by_key(|event| (event.body, event.shape_key, event.status));
        self.overlapping = seen;
        events
    }

    /// Advance tracking by one frame using the legacy body-level AABB query.
    ///
    /// Candidate bodies come from the broad phase in slot order and are
    /// accepted when their cached AABB overlaps the query AABB. Events are
    /// sorted by body id so results are deterministic.
    pub fn update(
        &mut self,
        world: &World,
        position: Vector4,
        radius: f32,
        filter: &QueryFilter,
    ) -> Vec<TriggerEvent> {
        let query_aabb = Aabb::from_center_radius(position, radius.max(0.0));
        let mut seen = HashSet::new();
        for body in world.query_aabb(&query_aabb, filter) {
            if world
                .body_aabb(body)
                .map(|aabb| aabb.overlaps(&query_aabb))
                .unwrap_or(false)
            {
                seen.insert(body);
            }
        }

        let mut observations = Vec::with_capacity(seen.len());
        for body in seen {
            observations.push(TriggerObservation {
                body,
                shape_key: ShapeKey::INVALID,
                position,
                normal: Vector4::ZERO,
            });
        }
        self.update_observations(&observations)
            .into_iter()
            .filter(|event| event.status != TriggerStatus::Updated)
            .collect()
    }

    /// Drop all tracked overlaps, emitting an exit event for each.
    pub fn clear(&mut self) -> Vec<TriggerEvent> {
        let mut events: Vec<_> = self
            .overlapping
            .keys()
            .map(|(body, shape_key)| TriggerEvent {
                body: *body,
                shape_key: *shape_key,
                status: TriggerStatus::Exited,
            })
            .collect();
        events.sort_by_key(|event| (event.body, event.shape_key));
        self.overlapping.clear();
        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_tracker_has_no_overlaps() {
        let tracker = TriggerTracker::new();
        assert_eq!(tracker.overlapping().count(), 0);
    }

    #[test]
    fn shape_keys_are_tracked_independently() {
        let mut tracker = TriggerTracker::new();
        let body = BodyId::new(7);
        let a = ShapeKey::new(1);
        let b = ShapeKey::new(2);
        let first = tracker.update_observations(&[
            TriggerObservation {
                body,
                shape_key: a,
                position: Vector4::ZERO,
                normal: Vector4::UNIT_Y,
            },
            TriggerObservation {
                body,
                shape_key: b,
                position: Vector4::ZERO,
                normal: Vector4::UNIT_Y,
            },
        ]);
        assert_eq!(first.len(), 2);
        assert!(
            first
                .iter()
                .all(|event| event.status == TriggerStatus::Entered)
        );
        let changed = tracker.update_observations(&[
            TriggerObservation {
                body,
                shape_key: a,
                position: Vector4::UNIT_X,
                normal: Vector4::UNIT_Y,
            },
            TriggerObservation {
                body,
                shape_key: b,
                position: Vector4::ZERO,
                normal: Vector4::UNIT_Y,
            },
        ]);
        assert_eq!(changed.len(), 1);
        assert_eq!(changed[0].shape_key, a);
        assert_eq!(changed[0].status, TriggerStatus::Updated);
        let exited = tracker.update_observations(&[]);
        assert_eq!(exited.len(), 2);
    }
}
