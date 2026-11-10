//! Body index pool (`hknpBodyManager`).

use hknp_types::{BodyId, BodyIndex};

use crate::Body;

/// Index-pool based body storage (`hknpBodyManager`).
///
/// Slots are reused through a free list; every reuse bumps an 8-bit serial
/// stored in the [`BodyIndex`] embedded in the [`BodyId`]. This reproduces the
/// SDK validity rule: a stale `hknpBodyId` from a destroyed body can never
/// accidentally resolve to the new occupant of the same slot.
#[derive(Default)]
pub struct BodyManager {
    slots: Vec<Option<Body>>,
    serials: Vec<u8>,
    free: Vec<u32>,
    len: usize,
}

impl BodyManager {
    /// Create an empty manager.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create an empty manager with pre-reserved slot capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        assert!(
            capacity < BodyIndex::INDEX_MASK as usize,
            "body capacity exceeds the 24-bit index range"
        );
        Self {
            slots: Vec::with_capacity(capacity),
            serials: Vec::with_capacity(capacity),
            free: Vec::new(),
            len: 0,
        }
    }

    /// Number of live bodies.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether there are no live bodies.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Total number of allocated slots (live + free).
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    /// Allocate a slot for `body` and assign its ids.
    ///
    /// # Panics
    ///
    /// Panics if the manager exhausted the 24-bit body index space.
    pub fn allocate(&mut self, mut body: Body) -> BodyId {
        let index = if let Some(index) = self.free.pop() {
            // Reuse bumps the serial so stale ids are rejected.
            self.serials[index as usize] = self.serials[index as usize].wrapping_add(1);
            index as usize
        } else {
            assert!(
                self.slots.len() < BodyIndex::INDEX_MASK as usize,
                "body index space exhausted"
            );
            self.slots.push(None);
            self.serials.push(0);
            self.slots.len() - 1
        };

        let body_index = BodyIndex::new(index as u32, self.serials[index]);
        let body_id = BodyId::from_body_index(body_index);
        body.id = body_id;
        body.index = body_index;
        self.slots[index] = Some(body);
        self.len += 1;
        body_id
    }

    /// Destroy the body with `body_id`, returning whether it existed.
    ///
    /// This is the C++ `freeBody` slot operation; removing the body from the
    /// world/broadphase is the world's responsibility.
    pub fn destroy(&mut self, body_id: BodyId) -> bool {
        let Some(body_index) = body_id.as_body_index() else {
            return false;
        };
        let slot = body_index.index() as usize;
        if slot >= self.slots.len() || self.serials[slot] != body_index.serial() {
            return false;
        }
        if self.slots[slot].take().is_none() {
            return false;
        }
        self.free.push(slot as u32);
        self.len -= 1;
        true
    }

    /// Resolve a live body id.
    pub fn get(&self, body_id: BodyId) -> Option<&Body> {
        let body_index = body_id.as_body_index()?;
        let slot = body_index.index() as usize;
        if slot >= self.slots.len() || self.serials[slot] != body_index.serial() {
            return None;
        }
        self.slots[slot].as_ref()
    }

    /// Resolve a live body id mutably.
    pub fn get_mut(&mut self, body_id: BodyId) -> Option<&mut Body> {
        let body_index = body_id.as_body_index()?;
        let slot = body_index.index() as usize;
        if slot >= self.slots.len() || self.serials[slot] != body_index.serial() {
            return None;
        }
        self.slots[slot].as_mut()
    }

    /// Resolve a body index (live slot only).
    pub fn get_by_index(&self, body_index: BodyIndex) -> Option<&Body> {
        let slot = body_index.index() as usize;
        if slot >= self.slots.len() || self.serials[slot] != body_index.serial() {
            return None;
        }
        self.slots[slot].as_ref()
    }

    /// Iterate over live bodies in slot order.
    pub fn iter(&self) -> impl Iterator<Item = &Body> + '_ {
        self.slots.iter().filter_map(Option::as_ref)
    }

    /// Iterate mutably over live bodies in slot order.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Body> + '_ {
        self.slots.iter_mut().filter_map(Option::as_mut)
    }

    /// Whether `body_id` resolves to a live body.
    pub fn is_valid(&self, body_id: BodyId) -> bool {
        self.get(body_id).is_some()
    }

    /// Remove every body without resetting slot serials.
    pub fn clear(&mut self) {
        for (slot, body) in self.slots.iter_mut().enumerate() {
            if body.take().is_some() {
                self.free.push(slot as u32);
            }
        }
        self.len = 0;
    }
}

impl core::fmt::Debug for BodyManager {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("BodyManager")
            .field("len", &self.len)
            .field("capacity", &self.slots.len())
            .field("free", &self.free.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Body;
    use hk_math::{Transform, Vector4};
    use hknp_shape::{Shape, SphereShape};
    use hknp_types::MotionId;
    use std::sync::Arc;

    fn make_body() -> Body {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        Body::new(shape, Transform::IDENTITY, MotionId::PRESET_STATIC)
    }

    #[test]
    fn allocate_assigns_ids_and_resolves() {
        let mut manager = BodyManager::new();
        let first = manager.allocate(make_body());
        let second = manager.allocate(make_body());
        assert_eq!(manager.len(), 2);
        assert!(first.is_body_index());
        assert!(second.is_body_index());

        let body = manager.get(first).unwrap();
        assert_eq!(body.id, first);
        assert_eq!(manager.get(second).unwrap().id, second);
        assert_eq!(manager.get_by_index(body.index).unwrap().id, first);
    }

    #[test]
    fn destroyed_ids_are_not_resolved_after_reuse() {
        let mut manager = BodyManager::new();
        let first = manager.allocate(make_body());
        assert!(manager.destroy(first));
        assert!(manager.get(first).is_none());
        assert!(!manager.is_valid(first));

        let second = manager.allocate(make_body());
        assert_eq!(manager.len(), 1);
        assert_ne!(first, second);
        assert!(manager.get(second).is_some());
        assert!(
            manager.get(first).is_none(),
            "stale id must not alias reused slot"
        );
    }

    #[test]
    fn destroy_rejects_invalid_ids() {
        let mut manager = BodyManager::new();
        assert!(!manager.destroy(BodyId::new(123)));
        assert!(!manager.destroy(BodyId::invalid()));
        let body = manager.allocate(make_body());
        assert!(manager.destroy(body));
        assert!(!manager.destroy(body), "double destroy fails");
        assert!(manager.is_empty());
    }

    #[test]
    fn iteration_and_clear() {
        let mut manager = BodyManager::new();
        let ids: Vec<_> = (0..3).map(|_| manager.allocate(make_body())).collect();
        assert_eq!(manager.iter().count(), 3);
        for body in manager.iter_mut() {
            body.user_data += 1;
        }
        assert_eq!(manager.iter().map(|body| body.user_data).sum::<u64>(), 3);

        manager.clear();
        assert!(manager.is_empty());
        assert!(manager.iter().next().is_none());
        for id in ids {
            assert!(manager.get(id).is_none());
        }
    }

    #[test]
    fn capacity_is_slot_count() {
        let mut manager = BodyManager::with_capacity(16);
        assert_eq!(
            manager.capacity(),
            0,
            "capacity is the number of allocated slots"
        );
        manager.allocate(make_body());
        assert_eq!(manager.capacity(), 1);
    }
}
