//! Contact impulse cache used for warm starting.
//!
//! The solver's [`phys_solver::ContactImpulse`] values are only useful if the
//! same contact row is found again next step. Rows are matched by body pair and
//! by the contact point expressed in body B's local space, so a moving body's
//! local feature survives the step. Entries are rebuilt every step from the
//! contacts that were actually solved, which prunes contacts that disappeared.
//!
//! Only convex single-point contacts exist today, but the cache stores a list
//! per pair so multi-point manifolds can reuse the same matching.

use base_math::Vector4;
use phys_solver::ContactImpulse;
use phys_types::BodyId;

/// Maximum distance (in metres) between the current contact point and a cached
/// point for the two to be treated as the same feature.
pub(crate) const MATCH_DISTANCE: f32 = 0.05;

/// Minimum dot product between the current and cached normal for a match.
pub(crate) const MATCH_NORMAL_DOT: f32 = 0.9;

/// Identity of a contact row across steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ContactKey {
    /// Body A (lower broad-phase pair id).
    pub body_a: BodyId,
    /// Body B (manifold normal points from B to A).
    pub body_b: BodyId,
    /// Contact point on B in B's local space.
    pub local_point_b: Vector4,
    /// Contact normal (B → A) in world space.
    pub normal: Vector4,
}

#[derive(Clone, Copy, Debug)]
struct CachedContact {
    key: ContactKey,
    impulse: ContactImpulse,
}

/// Per-world contact impulse cache (`hknpContactCache` subset).
#[derive(Clone, Debug, Default)]
pub(crate) struct ContactImpulseCache {
    entries: Vec<CachedContact>,
}

impl ContactImpulseCache {
    /// Find the cached impulse for `key`, or a zero impulse.
    ///
    /// Among all candidates with the same body pair, a similar normal and a
    /// point within [`MATCH_DISTANCE`], the nearest point wins; ties keep the
    /// entry inserted first, which makes the result deterministic.
    pub(crate) fn match_impulse(&self, key: ContactKey) -> ContactImpulse {
        let mut best: Option<(f32, ContactImpulse)> = None;
        for entry in &self.entries {
            if entry.key.body_a != key.body_a || entry.key.body_b != key.body_b {
                continue;
            }
            if entry.key.normal.dot3(key.normal) < MATCH_NORMAL_DOT {
                continue;
            }
            let distance = (entry.key.local_point_b - key.local_point_b).length3();
            if distance > MATCH_DISTANCE {
                continue;
            }
            match best {
                Some((best_distance, _)) if best_distance <= distance => {}
                _ => best = Some((distance, entry.impulse)),
            }
        }
        best.map(|(_, impulse)| impulse).unwrap_or_default()
    }

    /// Replace the cache with this step's solved rows.
    pub(crate) fn replace(&mut self, entries: impl Iterator<Item = (ContactKey, ContactImpulse)>) {
        self.entries.clear();
        self.entries
            .extend(entries.map(|(key, impulse)| CachedContact { key, impulse }));
    }

    /// Drop all cached impulses.
    pub(crate) fn clear(&mut self) {
        self.entries.clear();
    }

    /// Number of cached contact rows.
    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(body_a: u64, local_x: f32, normal: Vector4) -> ContactKey {
        ContactKey {
            body_a: BodyId::new(body_a),
            body_b: BodyId::new(body_a + 1),
            local_point_b: Vector4::from_xyz(local_x, 0.0, 0.0),
            normal,
        }
    }

    fn impulse(normal: f32) -> ContactImpulse {
        ContactImpulse {
            normal,
            tangent_1: 0.0,
            tangent_2: 0.0,
        }
    }

    #[test]
    fn matching_row_returns_cached_impulse() {
        let mut cache = ContactImpulseCache::default();
        cache.replace([(key(1, 0.0, Vector4::UNIT_Y), impulse(2.0))].into_iter());
        assert_eq!(cache.len(), 1);
        assert_eq!(
            cache.match_impulse(key(1, 0.0, Vector4::UNIT_Y)).normal,
            2.0
        );
        // Slightly displaced point still matches (within MATCH_DISTANCE).
        assert_eq!(
            cache.match_impulse(key(1, 0.02, Vector4::UNIT_Y)).normal,
            2.0
        );
    }

    #[test]
    fn distant_point_or_flipped_normal_does_not_match() {
        let mut cache = ContactImpulseCache::default();
        cache.replace([(key(1, 0.0, Vector4::UNIT_Y), impulse(2.0))].into_iter());
        assert_eq!(
            cache.match_impulse(key(1, MATCH_DISTANCE * 2.0, Vector4::UNIT_Y)),
            ContactImpulse::default()
        );
        assert_eq!(
            cache.match_impulse(key(1, 0.0, Vector4::UNIT_X)),
            ContactImpulse::default()
        );
    }

    #[test]
    fn different_body_pair_does_not_match() {
        let mut cache = ContactImpulseCache::default();
        cache.replace([(key(1, 0.0, Vector4::UNIT_Y), impulse(2.0))].into_iter());
        assert_eq!(
            cache.match_impulse(key(3, 0.0, Vector4::UNIT_Y)),
            ContactImpulse::default()
        );
    }

    #[test]
    fn nearest_of_two_rows_wins() {
        let mut cache = ContactImpulseCache::default();
        cache.replace(
            [
                (key(1, 0.0, Vector4::UNIT_Y), impulse(1.0)),
                (key(1, 0.03, Vector4::UNIT_Y), impulse(3.0)),
            ]
            .into_iter(),
        );
        assert_eq!(
            cache.match_impulse(key(1, 0.025, Vector4::UNIT_Y)).normal,
            3.0
        );
    }

    #[test]
    fn replace_prunes_stale_rows() {
        let mut cache = ContactImpulseCache::default();
        cache.replace([(key(1, 0.0, Vector4::UNIT_Y), impulse(2.0))].into_iter());
        cache.replace([(key(7, 0.0, Vector4::UNIT_Y), impulse(1.0))].into_iter());
        assert_eq!(cache.len(), 1);
        assert_eq!(
            cache.match_impulse(key(1, 0.0, Vector4::UNIT_Y)),
            ContactImpulse::default()
        );
        cache.clear();
        assert_eq!(cache.len(), 0);
    }
}
