//! Motion slot pool (`hknpMotionManager`).

use hknp_types::MotionId;

use crate::Motion;

/// Motion slot pool (`hknpMotionManager`).
///
/// Slot 0 is always the shared static motion (`hknpMotionId::PRESET_STATIC`);
/// dynamic motions occupy slots 1.. and are recycled through a free list.
///
/// **Staleness note**: `hknpMotionId` is a plain u32 slot index in the SDK (there
/// is no serial number), so a stale id whose slot has been reused cannot be
/// distinguished from the new occupant. This matches the C++ behaviour; callers
/// that keep motion ids across destruction must use body ids (which do carry a
/// serial).
#[derive(Debug)]
pub struct MotionManager {
    slots: Vec<Option<Motion>>,
    free: Vec<u32>,
    len: usize,
}

impl Default for MotionManager {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionManager {
    /// Create a manager containing only the shared static motion.
    pub fn new() -> Self {
        Self {
            slots: vec![Some(Motion::static_motion())],
            free: Vec::new(),
            len: 0,
        }
    }

    /// Create a manager with reserved capacity for non-static motions.
    pub fn with_capacity(capacity: usize) -> Self {
        let mut manager = Self::new();
        manager.slots.reserve(capacity);
        manager
    }

    /// Number of live non-static motions.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether there are no non-static motions.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Total slot count (including the static slot).
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    /// Allocate a slot for `motion` and return its id.
    ///
    /// # Panics
    ///
    /// Panics if the motion index space would exceed `MotionId::INVALID`.
    pub fn allocate(&mut self, motion: Motion) -> MotionId {
        let slot = if let Some(slot) = self.free.pop() {
            slot as usize
        } else {
            let slot = self.slots.len();
            assert!(
                slot < MotionId::INVALID.raw() as usize,
                "motion index space exhausted"
            );
            self.slots.push(None);
            slot
        };
        debug_assert!(slot > 0, "static slot is never allocatable");
        self.slots[slot] = Some(motion);
        self.len += 1;
        MotionId::new(slot as u32)
    }

    /// Resolve a motion id (including `PRESET_STATIC`).
    pub fn get(&self, motion_id: MotionId) -> Option<&Motion> {
        let slot = motion_id.raw() as usize;
        self.slots.get(slot)?.as_ref()
    }

    /// Resolve a motion id mutably.
    pub fn get_mut(&mut self, motion_id: MotionId) -> Option<&mut Motion> {
        let slot = motion_id.raw() as usize;
        self.slots.get_mut(slot)?.as_mut()
    }

    /// The shared static motion.
    pub fn static_motion(&self) -> &Motion {
        self.slots[0]
            .as_ref()
            .expect("static motion slot is permanent")
    }

    /// Mutable access to the shared static motion.
    ///
    /// The SDK treats the static motion as read-only during simulation; this
    /// accessor exists mainly for tests/tools.
    pub fn static_motion_mut(&mut self) -> &mut Motion {
        self.slots[0]
            .as_mut()
            .expect("static motion slot is permanent")
    }

    /// Destroy a non-static motion, returning whether it was alive.
    ///
    /// `PRESET_STATIC` and the invalid id are rejected.
    pub fn destroy(&mut self, motion_id: MotionId) -> bool {
        if motion_id == MotionId::PRESET_STATIC || !motion_id.is_valid() {
            return false;
        }
        let slot = motion_id.raw() as usize;
        if slot >= self.slots.len() || self.slots[slot].take().is_none() {
            return false;
        }
        self.free.push(slot as u32);
        self.len -= 1;
        true
    }

    /// Whether `motion_id` resolves to a live motion.
    pub fn is_valid(&self, motion_id: MotionId) -> bool {
        self.get(motion_id).is_some()
    }

    /// Iterate over live non-static motions with their ids, in slot order.
    pub fn iter(&self) -> impl Iterator<Item = (MotionId, &Motion)> + '_ {
        self.slots
            .iter()
            .enumerate()
            .skip(1)
            .filter_map(|(slot, motion)| {
                motion
                    .as_ref()
                    .map(|motion| (MotionId::new(slot as u32), motion))
            })
    }

    /// Remove all non-static motions, keeping the static motion.
    pub fn clear(&mut self) {
        for slot in 1..self.slots.len() {
            if self.slots[slot].take().is_some() {
                self.free.push(slot as u32);
            }
        }
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::{Quaternion, Vector4};
    use hknp_types::MotionType;

    fn dynamic_motion() -> Motion {
        Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::ZERO,
            Vector4::ZERO,
            1.0,
        )
    }

    #[test]
    fn static_motion_lives_at_id_zero() {
        let manager = MotionManager::new();
        assert!(manager.is_empty());
        assert!(manager.get(MotionId::PRESET_STATIC).is_some());
        assert_eq!(manager.static_motion().motion_type, MotionType::Static);
        assert!(manager.get(MotionId::new(1)).is_none());
    }

    #[test]
    fn allocate_and_destroy_motions() {
        let mut manager = MotionManager::with_capacity(8);
        let first = manager.allocate(dynamic_motion());
        let second = manager.allocate(dynamic_motion());
        assert_eq!(first.raw(), 1);
        assert_eq!(second.raw(), 2);
        assert_eq!(manager.len(), 2);
        assert!(manager.get(first).unwrap().is_dynamic());

        assert!(manager.destroy(first));
        assert!(manager.get(first).is_none());
        assert_eq!(manager.len(), 1);
        assert!(!manager.destroy(first), "double destroy fails");
        assert!(!manager.destroy(MotionId::PRESET_STATIC));
        assert!(!manager.destroy(MotionId::invalid()));

        // The freed slot is reused by the next allocation.
        let third = manager.allocate(dynamic_motion());
        assert_eq!(third.raw(), 1);
        assert_eq!(manager.len(), 2);
    }

    #[test]
    fn iteration_skips_static_and_free_slots() {
        let mut manager = MotionManager::new();
        let first = manager.allocate(dynamic_motion());
        manager.allocate(dynamic_motion());
        manager.destroy(first);
        let motions: Vec<_> = manager.iter().map(|(id, _)| id).collect();
        assert_eq!(motions.len(), 1);
        assert_eq!(motions[0].raw(), 2);
    }

    #[test]
    fn clear_keeps_static_motion() {
        let mut manager = MotionManager::new();
        manager.allocate(dynamic_motion());
        manager.allocate(dynamic_motion());
        manager.clear();
        assert!(manager.is_empty());
        assert!(manager.get(MotionId::PRESET_STATIC).is_some());
        assert_eq!(manager.iter().count(), 0);
    }
}
