//! Deterministic grid broad phase.
//!
//! Port of the grid layer of `hknpBroadPhase` (CsGrid / SIMD tree in the SDK).
//! Bodies are inserted into uniform grid cells derived from their world AABBs;
//! candidate pairs are produced per cell and deduplicated, so the result only
//! depends on the input order and the cell size.

use std::collections::BTreeMap;

use hk_math::Aabb;
use hknp_types::BodyId;

/// One body and its current world AABB (the broad phase input).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyAabb {
    /// Body identifier.
    pub body: BodyId,
    /// World-space AABB (typically `hknpBody::m_aabb`).
    pub aabb: Aabb,
}

impl BodyAabb {
    /// Construct a broad-phase entry.
    pub fn new(body: BodyId, aabb: Aabb) -> Self {
        Self { body, aabb }
    }
}

/// A candidate overlapping pair, ordered by id so `(a, b)` is canonical.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyPair {
    /// Lower id.
    pub a: BodyId,
    /// Higher id.
    pub b: BodyId,
}

impl BodyPair {
    /// Construct a canonical pair (`a <= b`).
    pub fn new(a: BodyId, b: BodyId) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
    }
}

/// Grid broad-phase configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadPhaseConfig {
    /// Uniform cell edge length in world units.
    pub cell_size: f32,
}

impl Default for BroadPhaseConfig {
    fn default() -> Self {
        Self { cell_size: 2.0 }
    }
}

impl BroadPhaseConfig {
    /// Construct a config with the given cell size.
    ///
    /// # Panics
    ///
    /// Panics in debug builds for a non-positive/non-finite cell size.
    pub fn with_cell_size(cell_size: f32) -> Self {
        debug_assert!(
            cell_size > 0.0 && cell_size.is_finite(),
            "invalid broad-phase cell size"
        );
        Self { cell_size }
    }
}

/// Uniform-grid broad phase (`hknpBroadPhase` grid subset).
#[derive(Clone, Debug, Default)]
pub struct BroadPhase {
    config: BroadPhaseConfig,
}

/// A body spanning more cells than this is handled by an O(n) fallback instead
/// of exploding the grid (matches the SDK's "large object" concern).
const MAX_CELLS_PER_BODY: usize = 4096;

impl BroadPhase {
    /// Construct a broad phase with the given config.
    pub fn new(config: BroadPhaseConfig) -> Self {
        Self { config }
    }

    /// Current config.
    pub fn config(&self) -> BroadPhaseConfig {
        self.config
    }

    /// Find all AABB-overlapping candidate pairs.
    ///
    /// The output is sorted by `(a, b)` and deduplicated, making the result
    /// deterministic for a given input ordering. Bodies with empty AABBs are
    /// ignored. `DONT_COLLIDE` filtering belongs to the collision filter layer
    /// (not yet ported) and is intentionally left to the caller.
    pub fn find_pairs(&self, bodies: &[BodyAabb]) -> Vec<BodyPair> {
        let cell_size = self.config.cell_size;
        let mut cells: BTreeMap<(i32, i32, i32), Vec<u32>> = BTreeMap::new();
        let mut large_bodies: Vec<u32> = Vec::new();

        for (index, entry) in bodies.iter().enumerate() {
            if entry.aabb.is_empty() || !entry.aabb.is_valid() {
                continue;
            }
            let min = cell_coordinate(entry.aabb.min, cell_size);
            let max = cell_coordinate(entry.aabb.max, cell_size);
            let span_x = (max.0 - min.0 + 1) as usize;
            let span_y = (max.1 - min.1 + 1) as usize;
            let span_z = (max.2 - min.2 + 1) as usize;
            let cell_count = span_x.saturating_mul(span_y).saturating_mul(span_z);
            if cell_count > MAX_CELLS_PER_BODY {
                large_bodies.push(index as u32);
                continue;
            }
            for x in min.0..=max.0 {
                for y in min.1..=max.1 {
                    for z in min.2..=max.2 {
                        cells.entry((x, y, z)).or_default().push(index as u32);
                    }
                }
            }
        }

        let mut pairs: Vec<BodyPair> = Vec::new();
        for indices in cells.values() {
            for (offset, &first) in indices.iter().enumerate() {
                for &second in &indices[offset + 1..] {
                    add_pair(&mut pairs, bodies, first, second);
                }
            }
        }

        // Bodies too large for the grid are paired against every other body.
        for &large in &large_bodies {
            for (index, _) in bodies.iter().enumerate() {
                if index as u32 != large {
                    add_pair(&mut pairs, bodies, large, index as u32);
                }
            }
        }

        pairs.sort_unstable_by_key(|pair| (pair.a.value(), pair.b.value()));
        pairs.dedup();
        pairs
    }
}

/// Grid cell of a world coordinate.
fn cell_coordinate(point: hk_math::Vector4, cell_size: f32) -> (i32, i32, i32) {
    let coordinate = |value: f32| {
        let scaled = (value / cell_size).floor();
        scaled.clamp(i32::MIN as f32, i32::MAX as f32) as i32
    };
    (
        coordinate(point.x),
        coordinate(point.y),
        coordinate(point.z),
    )
}

fn add_pair(pairs: &mut Vec<BodyPair>, bodies: &[BodyAabb], first: u32, second: u32) {
    if first == second {
        return;
    }
    let first = &bodies[first as usize];
    let second = &bodies[second as usize];
    if first.body == second.body {
        return;
    }
    if first.aabb.overlaps(&second.aabb) {
        pairs.push(BodyPair::new(first.body, second.body));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Vector4;

    fn aabb(min: f32, max: f32) -> Aabb {
        Aabb::new(
            Vector4::from_xyz(min, min, min),
            Vector4::from_xyz(max, max, max),
        )
    }

    #[test]
    fn finds_overlapping_pair() {
        let broadphase = BroadPhase::new(BroadPhaseConfig::default());
        let bodies = vec![
            BodyAabb::new(BodyId::new(1), aabb(0.0, 1.0)),
            BodyAabb::new(BodyId::new(2), aabb(0.5, 1.5)),
            BodyAabb::new(BodyId::new(3), aabb(10.0, 11.0)),
        ];
        let pairs = broadphase.find_pairs(&bodies);
        assert_eq!(pairs, vec![BodyPair::new(BodyId::new(1), BodyId::new(2))]);
    }

    #[test]
    fn pair_order_is_canonical_regardless_of_input_order() {
        let broadphase = BroadPhase::new(BroadPhaseConfig::default());
        let bodies = vec![
            BodyAabb::new(BodyId::new(9), aabb(0.0, 1.0)),
            BodyAabb::new(BodyId::new(4), aabb(0.2, 1.2)),
            BodyAabb::new(BodyId::new(7), aabb(0.4, 1.4)),
        ];
        let pairs = broadphase.find_pairs(&bodies);
        assert_eq!(
            pairs,
            vec![
                BodyPair::new(BodyId::new(4), BodyId::new(7)),
                BodyPair::new(BodyId::new(4), BodyId::new(9)),
                BodyPair::new(BodyId::new(7), BodyId::new(9)),
            ]
        );
        for pair in &pairs {
            assert!(pair.a < pair.b);
        }
    }

    #[test]
    fn empty_and_separated_aabbs_are_ignored() {
        let broadphase = BroadPhase::new(BroadPhaseConfig::default());
        let bodies = vec![
            BodyAabb::new(BodyId::new(1), Aabb::EMPTY),
            BodyAabb::new(BodyId::new(2), aabb(0.0, 1.0)),
            BodyAabb::new(BodyId::new(3), aabb(5.0, 6.0)),
        ];
        assert!(broadphase.find_pairs(&bodies).is_empty());
    }

    #[test]
    fn bodies_spanning_multiple_cells_are_still_pair_deduped() {
        let broadphase = BroadPhase::new(BroadPhaseConfig::with_cell_size(1.0));
        let bodies = vec![
            BodyAabb::new(BodyId::new(1), aabb(-2.0, 2.0)),
            BodyAabb::new(BodyId::new(2), aabb(-1.0, 1.0)),
        ];
        let pairs = broadphase.find_pairs(&bodies);
        assert_eq!(pairs.len(), 1, "pair must appear once despite shared cells");
        assert_eq!(pairs[0], BodyPair::new(BodyId::new(1), BodyId::new(2)));
    }

    #[test]
    fn large_body_falls_back_to_linear_scan() {
        let broadphase = BroadPhase::new(BroadPhaseConfig::with_cell_size(0.01));
        let bodies = vec![
            BodyAabb::new(BodyId::new(1), aabb(-100.0, 100.0)),
            BodyAabb::new(BodyId::new(2), aabb(-1.0, 1.0)),
            BodyAabb::new(BodyId::new(3), aabb(10.0, 12.0)),
        ];
        let pairs = broadphase.find_pairs(&bodies);
        assert_eq!(pairs.len(), 2);
        assert_eq!(pairs[0], BodyPair::new(BodyId::new(1), BodyId::new(2)));
        assert_eq!(pairs[1], BodyPair::new(BodyId::new(1), BodyId::new(3)));
    }
}
