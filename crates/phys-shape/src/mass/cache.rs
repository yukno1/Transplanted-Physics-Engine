//! Deterministic cache for shape mass properties.

use std::sync::Arc;

use crate::{CompressedMassProperties, MassConfig, MassError, MassProperties, Shape};

const GROUP_SIZE: usize = 4;

/// Shape-identity key used by [`ShapeMassCache`].
///
/// Pointer identity is stable while an `Arc<dyn Shape>` is alive. Callers must
/// invalidate entries when a shape is replaced so recycled addresses cannot
/// alias a different shape.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShapeCacheKey(usize);

impl ShapeCacheKey {
    /// Derive a key from a shared shape handle.
    pub fn new(shape: &Arc<dyn Shape>) -> Self {
        Self(Arc::as_ptr(shape) as *const () as usize)
    }
}

/// Insertion-ordered mass-property cache (`hknpShapeMassPropertiesCache` subset).
#[derive(Clone, Debug, Default)]
pub struct ShapeMassCache {
    entries: Vec<(ShapeCacheKey, MassConfig, MassProperties)>,
}

impl ShapeMassCache {
    /// Create an empty cache.
    pub fn new() -> Self {
        Self::default()
    }

    /// Cached properties for this shape/config pair, if present.
    pub fn get(&self, shape: &Arc<dyn Shape>, config: MassConfig) -> Option<MassProperties> {
        let key = ShapeCacheKey::new(shape);
        self.entries
            .iter()
            .find(|entry| entry.0 == key && entry.1 == config)
            .map(|entry| entry.2)
    }

    /// Return cached properties or compute and cache them once.
    pub fn compute(
        &mut self,
        shape: &Arc<dyn Shape>,
        config: MassConfig,
    ) -> Result<MassProperties, MassError> {
        if let Some(properties) = self.get(shape, config) {
            return Ok(properties);
        }
        let properties = shape.mass_properties(config)?;
        let key = ShapeCacheKey::new(shape);
        self.entries.push((key, config, properties));
        Ok(properties)
    }

    /// Drop all entries for one shape identity.
    pub fn invalidate(&mut self, shape: &Arc<dyn Shape>) {
        let key = ShapeCacheKey::new(shape);
        self.entries.retain(|entry| entry.0 != key);
    }

    /// Drop all cached entries.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Number of cached shape/config pairs.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Deterministic 4-child aggregate cache (`hknpMassPropertiesTree` subset).
///
/// Full groups of four are compressed immediately; partial groups are kept at
/// full precision until [`Self::finish`]. Insertion order is preserved and no
/// hash-container iteration participates in the result.
#[derive(Clone, Debug, Default)]
pub struct MassPropertiesTree {
    groups: Vec<CompressedMassProperties>,
    pending: Vec<MassProperties>,
}

impl MassPropertiesTree {
    /// Add one child's world-space mass properties.
    pub fn add(&mut self, properties: MassProperties) -> Result<(), MassError> {
        self.pending.push(properties);
        if self.pending.len() == GROUP_SIZE {
            let group = MassProperties::combine_all(std::mem::take(&mut self.pending));
            self.groups.push(CompressedMassProperties::pack(&group)?);
        }
        Ok(())
    }

    /// Combine cached groups and the final partial group.
    pub fn finish(self) -> Result<MassProperties, MassError> {
        let mut groups: Vec<MassProperties> = self
            .groups
            .into_iter()
            .map(|group| group.unpack())
            .collect();
        groups.extend(self.pending);
        Ok(MassProperties::combine_all(groups))
    }

    /// Number of compressed full groups.
    pub fn group_count(&self) -> usize {
        self.groups.len()
    }

    /// Number of full-precision children not yet grouped.
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SphereShape;

    #[test]
    fn cache_computes_once_and_invalidates_explicitly() {
        let shape = Arc::new(SphereShape::new(base_math::Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let mut cache = ShapeMassCache::new();
        assert!(cache.is_empty());
        let first = cache
            .compute(&shape, MassConfig::from_density(1.0))
            .unwrap();
        assert_eq!(
            cache.get(&shape, MassConfig::from_density(1.0)),
            Some(first)
        );
        assert_eq!(cache.len(), 1);
        cache.invalidate(&shape);
        assert!(cache.is_empty());
    }

    #[test]
    fn mass_tree_compresses_full_groups_deterministically() {
        let sphere = Arc::new(SphereShape::new(base_math::Vector4::ZERO, 0.5)) as Arc<dyn Shape>;
        let properties = sphere
            .mass_properties(MassConfig::from_density(1.0))
            .unwrap();
        let mut first = MassPropertiesTree::default();
        let mut second = MassPropertiesTree::default();
        for _ in 0..6 {
            first.add(properties).unwrap();
            second.add(properties).unwrap();
        }
        assert_eq!(first.group_count(), 1);
        assert_eq!(first.pending_count(), 2);
        assert_eq!(first.finish().unwrap(), second.finish().unwrap());
    }
}
