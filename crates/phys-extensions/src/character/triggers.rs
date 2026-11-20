//! Trigger-volume enter/exit tracking subset.
//!
//! Rust port of `hknpCharacterProxy` trigger bookkeeping
//! (`updateTriggersSeen` / `updateOverlappingTriggers`): the proxy's query
//! collector separates trigger hits from collision contacts; this subset
//! approximates them with AABB overlap against the character position and
//! reports entered/exited events by diffing consecutive frames.
//!
//! SDK material `TRIGGER_TYPE`, shape keys, `STATUS_UPDATED`, and the internal
//! keyframed phantom body are not reproduced; triggers are identified by
//! [`BodyId`] and filtered with the world's [`QueryFilter`].

use std::collections::HashSet;

use base_math::{Aabb, Vector4};
use phys_types::BodyId;
use phys_world::{QueryFilter, World};

/// `hknpTriggerEvent::Status` subset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum TriggerStatus {
    /// Trigger became overlapping this frame (`STATUS_ENTERED`).
    Entered,
    /// Trigger stopped overlapping this frame (`STATUS_EXITED`).
    Exited,
}

/// One trigger enter/exit event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerEvent {
    /// Trigger body.
    pub body: BodyId,
    /// Enter or exit status.
    pub status: TriggerStatus,
}

/// Per-frame trigger overlap tracker (`hknpCharacterProxy` subset).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriggerTracker {
    overlapping: HashSet<BodyId>,
}

impl TriggerTracker {
    /// Empty tracker: no triggers are currently overlapping.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bodies overlapping in the most recent [`Self::update`].
    pub fn overlapping(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.overlapping.iter().copied()
    }

    /// Advance tracking by one frame.
    ///
    /// `position`/`radius` define a sphere around the character; candidate
    /// bodies come from the broad phase in slot order and are accepted when
    /// their cached AABB overlaps the query AABB. Events are sorted by body id
    /// so results are deterministic.
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

        let mut events = Vec::new();
        for &body in &seen {
            if !self.overlapping.contains(&body) {
                events.push(TriggerEvent {
                    body,
                    status: TriggerStatus::Entered,
                });
            }
        }
        for &body in &self.overlapping {
            if !seen.contains(&body) {
                events.push(TriggerEvent {
                    body,
                    status: TriggerStatus::Exited,
                });
            }
        }
        events.sort_by_key(|event| (event.body, event.status));
        self.overlapping = seen;
        events
    }

    /// Drop all tracked overlaps, emitting an exit event for each.
    pub fn clear(&mut self) -> Vec<TriggerEvent> {
        let mut events: Vec<_> = self
            .overlapping
            .iter()
            .map(|&body| TriggerEvent {
                body,
                status: TriggerStatus::Exited,
            })
            .collect();
        events.sort_by_key(|event| event.body);
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
}
