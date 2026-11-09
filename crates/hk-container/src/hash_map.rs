//! `hkHashMap` — hash map keyed by `Eq + Hash` values.
//!
//! Port of `hkHashMap` (`Common/Base/Container/Hash/`) onto
//! `std::collections::HashMap` with the deterministic [`HkHasher`](crate::HkHasher).
//!
//! The C++ map is a `hkHashBase<hkPair<KEY, VALUE>>`; keys are unique and
//! `insert` overwrites an existing key while reporting whether the key was new.
//! Iterator/reference invalidation rules are the same as a Rust `HashMap`
//! ("invalidated by mutation"), documented per method.
//!
//! ```
//! use hk_container::HashMap;
//! let mut map = HashMap::new();
//! assert!(map.insert("a", 1));       // new key -> true
//! assert!(!map.insert("a", 2));      // existing key overwritten -> false
//! assert_eq!(map.try_get(&"a"), Some(&2));
//! assert_eq!(map.get_or_insert_default("b"), &mut 0);
//! ```

use core::hash::Hash;

use crate::hash::{BuildHkHasher, RawHashMap};

/// Result of [`HashMap::try_insert`] (`hkHashMap::TryInsertResult`).
///
/// The C++ version exposes a writable reference to the whole `hkPair`; this port
/// returns the value reference because the key is already owned by the map (Rust
/// cannot hand out both halves without a self-referential type).
pub struct TryInsertResult<'a, V> {
    /// Value for the key, inserted or pre-existing.
    pub value: &'a mut V,
    /// `true` if a new key was inserted, `false` if an existing value was kept.
    pub was_inserted: bool,
}

/// Hash map with a deterministic hasher (`hkHashMap<KEY, VALUE>`).
#[derive(Clone, Debug)]
pub struct HashMap<K, V> {
    inner: RawHashMap<K, V>,
}

impl<K, V> Default for HashMap<K, V>
where
    K: Eq + Hash,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<K, V> HashMap<K, V>
where
    K: Eq + Hash,
{
    /// Empty map (`hkHashMap()`).
    #[inline]
    pub fn new() -> Self {
        Self {
            inner: RawHashMap::with_hasher(BuildHkHasher::default()),
        }
    }

    /// Empty map with a preallocated bucket count.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: RawHashMap::with_capacity_and_hasher(capacity, BuildHkHasher::default()),
        }
    }

    /// `hkHashMap::insert` — overwrite and return `true` if the key was new.
    #[inline]
    pub fn insert(&mut self, key: K, value: V) -> bool {
        self.inner.insert(key, value).is_none()
    }

    /// `hkHashMap::tryInsert` — insert only if the key is absent, returning the
    /// value slot either way. Does not modify an existing entry.
    pub fn try_insert(&mut self, key: K, value: V) -> TryInsertResult<'_, V> {
        match self.inner.entry(key) {
            std::collections::hash_map::Entry::Occupied(entry) => TryInsertResult {
                value: entry.into_mut(),
                was_inserted: false,
            },
            std::collections::hash_map::Entry::Vacant(entry) => TryInsertResult {
                value: entry.insert(value),
                was_inserted: true,
            },
        }
    }

    /// `hkHashMap::tryGet` — value for `key`, or `None`.
    #[inline]
    pub fn try_get(&self, key: &K) -> Option<&V> {
        self.inner.get(key)
    }

    /// Mutable variant of [`HashMap::try_get`].
    #[inline]
    pub fn try_get_mut(&mut self, key: &K) -> Option<&mut V> {
        self.inner.get_mut(key)
    }

    /// `hkHashMap::at` — value for `key`, panicking when absent.
    #[inline]
    pub fn at(&self, key: &K) -> &V {
        self.inner.get(key).expect("hkHashMap::at: key not found")
    }

    /// Mutable variant of [`HashMap::at`].
    #[inline]
    pub fn at_mut(&mut self, key: &K) -> &mut V {
        self.inner
            .get_mut(key)
            .expect("hkHashMap::at_mut: key not found")
    }

    /// `hkHashMap::hasKey` / `contains`.
    #[inline]
    pub fn contains_key(&self, key: &K) -> bool {
        self.inner.contains_key(key)
    }

    /// `hkHashBase::remove` — remove the key, returning its value.
    #[inline]
    pub fn remove(&mut self, key: &K) -> Option<V> {
        self.inner.remove(key)
    }

    /// `hkHashMap::getOrInsertDefault` — value for `key`, inserting
    /// `V::default()` when absent.
    #[inline]
    pub fn get_or_insert_default(&mut self, key: K) -> &mut V
    where
        V: Default,
    {
        self.inner.entry(key).or_default()
    }

    /// Insert the provided value when `key` is absent and return the value slot.
    #[inline]
    pub fn get_or_insert(&mut self, key: K, value: V) -> &mut V {
        self.inner.entry(key).or_insert(value)
    }

    /// Insert the value produced by `create` when `key` is absent
    /// (`findOrInsertKeyWithFunctor` / `getOrInsertKeyWithFunctor`).
    #[inline]
    pub fn get_or_insert_with<F>(&mut self, key: K, create: F) -> &mut V
    where
        F: FnOnce() -> V,
    {
        self.inner.entry(key).or_insert_with(create)
    }

    /// `hkHashMap::getWithDefault` — cloned value for `key`, or `default`.
    ///
    /// The C++ version returns a reference to `default`; returning a value avoids
    /// a dangling-reference-shaped API.
    #[inline]
    pub fn get_with_default(&self, key: &K, default: &V) -> V
    where
        V: Clone,
    {
        match self.inner.get(key) {
            Some(value) => value.clone(),
            None => default.clone(),
        }
    }

    /// Underlying `entry` API for in-place mutation.
    #[inline]
    pub fn entry(&mut self, key: K) -> std::collections::hash_map::Entry<'_, K, V> {
        self.inner.entry(key)
    }

    /// Number of entries.
    #[inline]
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Whether the map is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Bucket capacity.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    /// Reserve capacity for additional entries.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional);
    }

    /// Remove every entry, keeping the allocation.
    #[inline]
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Iterate key/value pairs.
    #[inline]
    pub fn iter(&self) -> std::collections::hash_map::Iter<'_, K, V> {
        self.inner.iter()
    }

    /// Iterate key/value pairs mutably.
    #[inline]
    pub fn iter_mut(&mut self) -> std::collections::hash_map::IterMut<'_, K, V> {
        self.inner.iter_mut()
    }

    /// Iterate keys.
    #[inline]
    pub fn keys(&self) -> std::collections::hash_map::Keys<'_, K, V> {
        self.inner.keys()
    }

    /// Iterate values.
    #[inline]
    pub fn values(&self) -> std::collections::hash_map::Values<'_, K, V> {
        self.inner.values()
    }

    /// Iterate values mutably.
    #[inline]
    pub fn values_mut(&mut self) -> std::collections::hash_map::ValuesMut<'_, K, V> {
        self.inner.values_mut()
    }
}

impl<K, V> FromIterator<(K, V)> for HashMap<K, V>
where
    K: Eq + Hash,
{
    fn from_iter<I: IntoIterator<Item = (K, V)>>(iter: I) -> Self {
        let mut map = Self::new();
        map.inner.extend(iter);
        map
    }
}

impl<K, V> Extend<(K, V)> for HashMap<K, V>
where
    K: Eq + Hash,
{
    fn extend<I: IntoIterator<Item = (K, V)>>(&mut self, iter: I) {
        self.inner.extend(iter);
    }
}

impl<K, V> IntoIterator for HashMap<K, V>
where
    K: Eq + Hash,
{
    type Item = (K, V);
    type IntoIter = std::collections::hash_map::IntoIter<K, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.into_iter()
    }
}

impl<'a, K, V> IntoIterator for &'a HashMap<K, V>
where
    K: Eq + Hash,
{
    type Item = (&'a K, &'a V);
    type IntoIter = std::collections::hash_map::Iter<'a, K, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_reports_new_key_and_overwrites() {
        let mut map = HashMap::new();
        assert!(map.insert(1, "one"));
        assert_eq!(map.try_get(&1), Some(&"one"));
        assert!(!map.insert(1, "uno"));
        assert_eq!(map.try_get(&1), Some(&"uno"));
        assert_eq!(map.len(), 1);

        map.insert(2, "two");
        assert_eq!(map.len(), 2);
        assert!(map.contains_key(&2));
    }

    #[test]
    fn try_insert_does_not_overwrite() {
        let mut map = HashMap::new();
        map.insert(1, 10);

        let result = map.try_insert(1, 20);
        assert!(!result.was_inserted);
        assert_eq!(*result.value, 10);

        let result = map.try_insert(2, 30);
        assert!(result.was_inserted);
        assert_eq!(*result.value, 30);
        assert_eq!(map.try_get(&2), Some(&30));
    }

    #[test]
    fn insert_or_default_helpers() {
        let mut map = HashMap::new();
        *map.get_or_insert_default(1) += 1;
        assert_eq!(map.at(&1), &1);
        *map.get_or_insert_default(1) += 1;
        assert_eq!(map.at(&1), &2);

        let value = map.get_or_insert(2, 5);
        assert_eq!(*value, 5);
        let value = map.get_or_insert(2, 9);
        assert_eq!(*value, 5, "existing value is kept");

        let value = map.get_or_insert_with(3, || 7);
        assert_eq!(*value, 7);
        let value = map.get_or_insert_with(3, || 8);
        assert_eq!(*value, 7, "functor must only run when absent");
    }

    #[test]
    fn lookups_remove_and_defaults() {
        let mut map = HashMap::new();
        map.insert(1, 10);

        assert_eq!(map.at(&1), &10);
        *map.at_mut(&1) = 11;
        assert_eq!(map.try_get(&1), Some(&11));
        assert_eq!(map.try_get(&9), None);

        assert_eq!(map.get_with_default(&1, &0), 11);
        assert_eq!(map.get_with_default(&9, &0), 0);

        assert_eq!(map.remove(&1), Some(11));
        assert_eq!(map.remove(&1), None);
        assert_eq!(map.len(), 0);
    }

    #[test]
    fn iteration_and_collection_round_trips() {
        let map: HashMap<i32, i32> = (0..5).map(|value| (value, value * 2)).collect();
        assert_eq!(map.len(), 5);

        let total: i32 = map.values().sum();
        assert_eq!(total, 20);

        let sum_keys: i32 = map.keys().sum();
        assert_eq!(sum_keys, 10);

        let mut map2 = HashMap::new();
        map2.extend([(1, 1), (2, 2)]);
        assert_eq!(map2.len(), 2);
    }

    #[test]
    fn iteration_order_is_deterministic() {
        let build = || -> Vec<(i32, i32)> {
            let map: HashMap<i32, i32> = (0..32).map(|value| (value, value)).collect();
            map.iter().map(|(key, value)| (*key, *value)).collect()
        };
        assert_eq!(build(), build(), "same hasher seed => same iteration order");
    }
}
