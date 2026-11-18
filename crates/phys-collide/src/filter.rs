//! Collision filters (`hknpCollisionFilter` subset).
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Collide/Filter/`
//! (migration Phase 14, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`CollisionFilter`] | `hknpCollisionFilter` | trait with the info/body-pair/query decision entry points |
//! | [`GroupCollisionFilter`] | `hknpGroupCollisionFilter` (`Config<5,5,5,16>`) | 5-bit layer + 5-bit subsystem + 5-bit don't-collide + 16-bit system group, 32-entry layer lookup table |
//! | [`PairCollisionFilter`] | `hknpPairCollisionFilter` | reference-counted body-pair disable list with optional child filter |
//!
//! # Filter info
//!
//! The per-body payload is the 32-bit `collision_filter_info` already carried
//! by [`phys_types`] bodies (`hknpBody::m_collisionFilterInfo`). Its bit
//! layout is defined by [`GroupCollisionFilter::calc_filter_info`], matching
//! the SDK default configuration: layer in bits 0–4, subsystem id in bits
//! 5–9, subsystem don't-collide-with in bits 10–14, system group in bits
//! 15–30.
//!
//! Bodies with filter info `0` (the default) live in layer 0, no subsystem
//! and no system group, and therefore collide with everything unless the
//! layer table is explicitly edited.
//!
//! # Query type
//!
//! The SDK passes an `hknpCollisionQueryType::Enum` (simple ray / shape cast
//! / …) to every filter overload. The subset here does not branch on the
//! query type — built-in filters never did either.
//!
//! # Not ported
//!
//! `hknpDisableCollisionFilter` (subsumed by [`PairCollisionFilter`]),
//! `hknpConstraintCollisionFilter` (the world's `DISABLE_COLLISIONS` joint
//! handling already covers it), broad-phase `filterBodyPairs` stream
//! compaction (pairs are filtered after the grid broad phase instead),
//! `getLodInfo`, filter serialization and `hknpFilterTranslator`.

use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;

use phys_types::BodyId;

/// Collision decision interface (`hknpCollisionFilter` subset).
///
/// Implementations must be deterministic and thread-safe (`Send + Sync`) so
/// the world can consult them from the step pipeline and queries.
pub trait CollisionFilter: Send + Sync {
    /// Info-level decision (`isCollisionEnabled(infoA, infoB)` overload).
    ///
    /// The default allows every pair.
    fn is_collision_enabled(&self, info_a: u32, info_b: u32) -> bool {
        let _ = (info_a, info_b);
        true
    }

    /// Body-pair decision (`isCollisionEnabled(bodyIdA, bodyIdB)` overload),
    /// consulted for every broad-phase candidate pair during contact
    /// generation. Receives the body ids in addition to their filter infos.
    ///
    /// The default delegates to [`CollisionFilter::is_collision_enabled`].
    fn is_body_pair_enabled(
        &self,
        body_a: BodyId,
        info_a: u32,
        body_b: BodyId,
        info_b: u32,
    ) -> bool {
        let _ = (body_a, body_b);
        self.is_collision_enabled(info_a, info_b)
    }

    /// Query-vs-body decision (`isCollisionEnabled(queryFilterData, body)`
    /// overload), consulted by world queries.
    ///
    /// The default delegates to [`CollisionFilter::is_collision_enabled`].
    fn is_query_enabled(&self, query_info: u32, body_info: u32) -> bool {
        self.is_collision_enabled(query_info, body_info)
    }
}

/// Layer/subsystem/system-group filter (`hknpGroupCollisionFilter`, default
/// `Config<5,5,5,16>` specialization).
///
/// The layer lookup table starts fully enabled (every layer collides with
/// every layer, matching the SDK constructor). `disable_collisions_between`
/// clears the symmetric pair of table bits; layer 0 can never be disabled
/// from colliding (SDK assertion).
#[derive(Debug)]
pub struct GroupCollisionFilter {
    /// `m_collisionLookupTable`: per-layer bit mask of enabled target layers.
    collision_lookup_table: [u32; GROUP_LAYER_COUNT],
}

/// Number of collision layers in the default group-filter configuration
/// (`NUM_LAYER_BITS = 5`).
pub const GROUP_LAYER_COUNT: usize = 32;

impl GroupCollisionFilter {
    /// Create the filter with every layer pair enabled (SDK constructor).
    #[must_use]
    pub fn new() -> Self {
        Self {
            collision_lookup_table: [u32::MAX; GROUP_LAYER_COUNT],
        }
    }

    /// Pack filter info bits (`hknpGroupCollisionFilter::calcFilterInfo`).
    ///
    /// `layer | subsystem_id << 5 | subsystem_dont_collide_with << 10 |
    /// system_group << 15`.
    #[must_use]
    pub fn calc_filter_info_full(
        layer: u32,
        system_group: u32,
        subsystem_id: u32,
        subsystem_dont_collide_with: u32,
    ) -> u32 {
        debug_assert!(layer < GROUP_LAYER_COUNT as u32, "layer out of range");
        debug_assert!(
            subsystem_id < (1 << 5) && subsystem_dont_collide_with < (1 << 5),
            "subsystem ids out of range"
        );
        debug_assert!(system_group <= 0xFFFF, "system group out of range");
        layer | (subsystem_id << 5) | (subsystem_dont_collide_with << 10) | (system_group << 15)
    }

    /// Pack filter info for a plain layer (all other fields zero).
    #[must_use]
    pub fn calc_filter_info(layer: u32) -> u32 {
        Self::calc_filter_info_full(layer, 0, 0, 0)
    }

    /// Extract the collision layer (bits 0–4).
    #[must_use]
    pub const fn layer(info: u32) -> u32 {
        info & 0x1F
    }

    /// Extract the subsystem id (bits 5–9).
    #[must_use]
    pub const fn subsystem_id(info: u32) -> u32 {
        (info >> 5) & 0x1F
    }

    /// Extract the subsystem don't-collide-with id (bits 10–14).
    #[must_use]
    pub const fn subsystem_dont_collide_with(info: u32) -> u32 {
        (info >> 10) & 0x1F
    }

    /// Extract the system group (bits 15–30).
    #[must_use]
    pub const fn system_group(info: u32) -> u32 {
        (info >> 15) & 0xFFFF
    }

    /// Allow collisions between two layers (`enableCollisionsBetween`).
    pub fn enable_collisions_between(&mut self, layer_a: u32, layer_b: u32) {
        debug_assert!(
            layer_a < GROUP_LAYER_COUNT as u32 && layer_b < GROUP_LAYER_COUNT as u32,
            "layer out of range"
        );
        let (a, b) = (layer_a as usize, layer_b as usize);
        self.collision_lookup_table[a] |= 1 << b;
        self.collision_lookup_table[b] |= 1 << a;
    }

    /// Forbid collisions between two layers (`disableCollisionsBetween`).
    ///
    /// Layer 0 always collides; disabling it is an SDK assertion violation.
    pub fn disable_collisions_between(&mut self, layer_a: u32, layer_b: u32) {
        debug_assert!(
            layer_a < GROUP_LAYER_COUNT as u32 && layer_b < GROUP_LAYER_COUNT as u32,
            "layer out of range"
        );
        debug_assert!(
            layer_a != 0 && layer_b != 0,
            "layer 0 can never be disabled (SDK assertion)"
        );
        let (a, b) = (layer_a as usize, layer_b as usize);
        self.collision_lookup_table[a] &= !(1 << b);
        self.collision_lookup_table[b] &= !(1 << a);
    }

    /// The raw layer lookup table (`getCollisionLookupTable`).
    #[must_use]
    pub fn lookup_table(&self) -> &[u32; GROUP_LAYER_COUNT] {
        &self.collision_lookup_table
    }

    /// Core decision (`hknpGroupCollisionFilter::_isCollisionEnabled`).
    ///
    /// Two bodies in the same *non-zero* system group bypass the layer table:
    /// they collide unless either body's subsystem id equals the other body's
    /// subsystem don't-collide-with id. Otherwise the decision is the layer
    /// table bit `LUT[layer_a] & (1 << layer_b)` (the table is kept
    /// symmetric, so one lookup suffices).
    #[must_use]
    pub fn is_enabled(&self, info_a: u32, info_b: u32) -> bool {
        if Self::system_group(info_a ^ info_b) == 0 && Self::system_group(info_a) != 0 {
            return Self::subsystem_dont_collide_with(info_a) != Self::subsystem_id(info_b)
                && Self::subsystem_dont_collide_with(info_b) != Self::subsystem_id(info_a);
        }
        let table = self.collision_lookup_table[Self::layer(info_a) as usize];
        (table & (1 << Self::layer(info_b))) != 0
    }
}

impl Default for GroupCollisionFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl CollisionFilter for GroupCollisionFilter {
    fn is_collision_enabled(&self, info_a: u32, info_b: u32) -> bool {
        self.is_enabled(info_a, info_b)
    }
}

/// Body-pair disable list (`hknpPairCollisionFilter`).
///
/// Keeps a reference-counted set of disabled body pairs in front of an
/// optional child filter: a pair is enabled only when the child filter
/// allows it and the pair is not disabled. `disable_collisions_between` may
/// be called multiple times for the same pair; each call must be matched by
/// an [`PairCollisionFilter::enable_collisions_between`] before the pair
/// re-collides (the SDK's disable counter).
#[derive(Default)]
pub struct PairCollisionFilter {
    /// `m_disabledPairs`: normalized `(min, max)` body id pair → disable count.
    disabled_pairs: HashMap<(BodyId, BodyId), u32>,
    /// `m_childFilter`: consulted first for every decision.
    child: Option<Arc<dyn CollisionFilter>>,
}

impl fmt::Debug for PairCollisionFilter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PairCollisionFilter")
            .field("disabled_pairs", &self.disabled_pairs)
            .field("child", &self.child.is_some())
            .finish()
    }
}

impl PairCollisionFilter {
    /// Create an empty disable list without a child filter.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create an empty disable list in front of `child`
    /// (`hknpPairCollisionFilter::setChildFilter`).
    #[must_use]
    pub fn with_child(child: Arc<dyn CollisionFilter>) -> Self {
        Self {
            disabled_pairs: HashMap::new(),
            child: Some(child),
        }
    }

    /// Disable collisions between two bodies, bumping the disable count
    /// (`disableCollisionsBetween`).
    pub fn disable_collisions_between(&mut self, body_a: BodyId, body_b: BodyId) {
        let key = self.normalize(body_a, body_b);
        *self
            .disabled_pairs
            .entry(key)
            .and_modify(|count| debug_assert!(*count > 0))
            .or_insert(0) += 1;
    }

    /// Re-enable collisions between two bodies by decrementing the disable
    /// count (`enableCollisionsBetween`).
    pub fn enable_collisions_between(&mut self, body_a: BodyId, body_b: BodyId) {
        let key = self.normalize(body_a, body_b);
        let Some(count) = self.disabled_pairs.get_mut(&key) else {
            debug_assert!(false, "enable_collisions_between on unknown pair");
            return;
        };
        debug_assert!(*count > 0, "unbalanced enable_collisions_between");
        *count -= 1;
        if *count == 0 {
            self.disabled_pairs.remove(&key);
        }
    }

    /// Drop every disabled pair (`clearAll`).
    pub fn clear_all(&mut self) {
        self.disabled_pairs.clear();
    }

    /// Whether a specific body pair is currently disabled.
    #[must_use]
    pub fn is_disabled(&self, body_a: BodyId, body_b: BodyId) -> bool {
        self.disabled_pairs
            .contains_key(&self.normalize(body_a, body_b))
    }

    /// Number of disabled body pairs.
    #[must_use]
    pub fn disabled_pair_count(&self) -> usize {
        self.disabled_pairs.len()
    }

    /// The child filter, if any (`getChildFilter`).
    #[must_use]
    pub fn child(&self) -> Option<&Arc<dyn CollisionFilter>> {
        self.child.as_ref()
    }

    fn normalize(&self, body_a: BodyId, body_b: BodyId) -> (BodyId, BodyId) {
        if body_a < body_b {
            (body_a, body_b)
        } else {
            (body_b, body_a)
        }
    }
}

impl CollisionFilter for PairCollisionFilter {
    fn is_collision_enabled(&self, info_a: u32, info_b: u32) -> bool {
        self.child
            .as_ref()
            .is_none_or(|child| child.is_collision_enabled(info_a, info_b))
    }

    fn is_body_pair_enabled(
        &self,
        body_a: BodyId,
        info_a: u32,
        body_b: BodyId,
        info_b: u32,
    ) -> bool {
        let child_allows = self
            .child
            .as_ref()
            .is_none_or(|child| child.is_body_pair_enabled(body_a, info_a, body_b, info_b));
        child_allows && !self.is_disabled(body_a, body_b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: u64) -> BodyId {
        BodyId::new(value)
    }

    #[test]
    fn calc_filter_info_bit_layout_matches_sdk() {
        let info = GroupCollisionFilter::calc_filter_info_full(3, 0x1234, 7, 19);
        assert_eq!(GroupCollisionFilter::layer(info), 3);
        assert_eq!(GroupCollisionFilter::subsystem_id(info), 7);
        assert_eq!(GroupCollisionFilter::subsystem_dont_collide_with(info), 19);
        assert_eq!(GroupCollisionFilter::system_group(info), 0x1234);
        assert_eq!(GroupCollisionFilter::calc_filter_info(9), 9);
    }

    #[test]
    fn default_table_allows_every_layer_pair() {
        let filter = GroupCollisionFilter::new();
        for layer_a in 0..GROUP_LAYER_COUNT as u32 {
            for layer_b in 0..GROUP_LAYER_COUNT as u32 {
                let info_a = GroupCollisionFilter::calc_filter_info(layer_a);
                let info_b = GroupCollisionFilter::calc_filter_info(layer_b);
                assert!(filter.is_enabled(info_a, info_b));
            }
        }
        for mask in filter.lookup_table() {
            assert_eq!(*mask, u32::MAX);
        }
    }

    #[test]
    fn layer_disable_is_symmetric() {
        let mut filter = GroupCollisionFilter::new();
        let layer_a = GroupCollisionFilter::calc_filter_info(1);
        let layer_b = GroupCollisionFilter::calc_filter_info(2);
        filter.disable_collisions_between(1, 2);
        assert!(!filter.is_enabled(layer_a, layer_b));
        assert!(!filter.is_enabled(layer_b, layer_a));
        // Same layer keeps colliding with itself.
        assert!(filter.is_enabled(layer_a, layer_a));
        assert!(filter.is_enabled(layer_b, layer_b));
        // Other pairs stay untouched.
        let layer_c = GroupCollisionFilter::calc_filter_info(4);
        assert!(filter.is_enabled(layer_a, layer_c));
        assert!(filter.is_enabled(layer_b, layer_c));
        filter.enable_collisions_between(1, 2);
        assert!(filter.is_enabled(layer_a, layer_b));
        assert!(filter.is_enabled(layer_b, layer_a));
    }

    #[test]
    fn same_system_group_bypasses_layer_table() {
        let mut filter = GroupCollisionFilter::new();
        filter.disable_collisions_between(1, 2);
        let info_a = GroupCollisionFilter::calc_filter_info_full(1, 42, 1, 0);
        let info_b = GroupCollisionFilter::calc_filter_info_full(2, 42, 2, 0);
        // Same non-zero group with disjoint subsystem ids: the layer-1/2
        // disable is ignored.
        assert!(filter.is_enabled(info_a, info_b));
        assert!(filter.is_enabled(info_b, info_a));
        // A's don't-collide id matching B's subsystem id disables the pair.
        let info_c = GroupCollisionFilter::calc_filter_info_full(1, 42, 1, 2);
        assert!(!filter.is_enabled(info_c, info_b));
        // Zero subsystem ids inside a non-zero group never collide (SDK:
        // `idA == dontCollideB` short-circuits to disabled).
        let info_zero_a = GroupCollisionFilter::calc_filter_info_full(1, 42, 0, 0);
        let info_zero_b = GroupCollisionFilter::calc_filter_info_full(2, 42, 0, 0);
        assert!(!filter.is_enabled(info_zero_a, info_zero_b));
        // Different groups: the layer table decides.
        let info_d = GroupCollisionFilter::calc_filter_info_full(2, 43, 1, 0);
        assert!(!filter.is_enabled(info_a, info_d));
        // Same group zero falls back to the layer table.
        let info_e = GroupCollisionFilter::calc_filter_info_full(1, 0, 0, 0);
        let info_f = GroupCollisionFilter::calc_filter_info_full(2, 0, 0, 0);
        assert!(!filter.is_enabled(info_e, info_f));
    }

    #[test]
    fn system_group_subsystem_exclusion() {
        let filter = GroupCollisionFilter::new();
        let info_a = GroupCollisionFilter::calc_filter_info_full(3, 7, 2, 4);
        let info_b = GroupCollisionFilter::calc_filter_info_full(3, 7, 4, 0);
        let info_c = GroupCollisionFilter::calc_filter_info_full(3, 7, 5, 0);
        // A don't-collides-with B's subsystem id (and vice versa).
        assert!(!filter.is_enabled(info_a, info_b));
        assert!(!filter.is_enabled(info_b, info_a));
        assert!(filter.is_enabled(info_a, info_c));
        assert!(filter.is_enabled(info_c, info_b));
    }

    #[test]
    fn pair_filter_counts_disable_enable() {
        let mut filter = PairCollisionFilter::new();
        let (a, b) = (id(1), id(2));
        assert!(filter.is_body_pair_enabled(a, 0, b, 0));
        filter.disable_collisions_between(a, b);
        assert!(filter.is_disabled(a, b));
        assert!(!filter.is_body_pair_enabled(a, 0, b, 0));
        // Second disable requires two enables.
        filter.disable_collisions_between(b, a);
        assert_eq!(filter.disabled_pair_count(), 1);
        filter.enable_collisions_between(a, b);
        assert!(!filter.is_body_pair_enabled(a, 0, b, 0));
        filter.enable_collisions_between(b, a);
        assert!(filter.is_body_pair_enabled(a, 0, b, 0));
        assert_eq!(filter.disabled_pair_count(), 0);
    }

    #[test]
    fn pair_filter_delegates_to_child() {
        let mut group = GroupCollisionFilter::new();
        group.disable_collisions_between(1, 2);
        let mut filter = PairCollisionFilter::with_child(Arc::new(group));
        let info_a = GroupCollisionFilter::calc_filter_info(1);
        let info_b = GroupCollisionFilter::calc_filter_info(2);
        assert!(!filter.is_collision_enabled(info_a, info_b));
        let (a, b) = (id(10), id(20));
        filter.disable_collisions_between(a, b);
        assert!(!filter.is_body_pair_enabled(a, 0, b, 0));
        filter.enable_collisions_between(a, b);
        // Info-level still blocked by the child filter.
        assert!(!filter.is_collision_enabled(info_a, info_b));
        // Default infos (layer 0) pass the child.
        assert!(filter.is_body_pair_enabled(a, 0, b, 0));
    }

    #[test]
    fn default_filter_allows_everything() {
        let filter = PairCollisionFilter::new();
        assert!(filter.is_collision_enabled(u32::MAX, u32::MAX));
        assert!(filter.is_body_pair_enabled(id(1), 0xFFFF, id(2), 0xFFFF));
        assert!(filter.is_query_enabled(0, 0xFFFF));
    }
}
