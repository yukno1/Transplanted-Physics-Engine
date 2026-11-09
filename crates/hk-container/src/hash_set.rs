//! `hkHashSet` — collection of unique hashable elements.
//!
//! Port of `hkHashSet` (`Common/Base/Container/Hash/hkHashSet.h`) onto
//! `std::collections::HashSet` with the deterministic [`HkHasher`](crate::HkHasher).
//!
//! ```
//! use hk_container::HashSet;
//! let mut set = HashSet::new();
//! assert!(set.insert(1));       // new element -> true
//! assert!(!set.insert(1));      // already present -> false
//! assert!(set.contains(&1));
//! ```

use core::hash::Hash;

use crate::hash::{BuildHkHasher, RawHashSet};

/// Hash set with a deterministic hasher (`hkHashSet<KEY>`).
#[derive(Clone, Debug)]
pub struct HashSet<T> {
    inner: RawHashSet<T>,
}

impl<T> Default for HashSet<T>
where
    T: Eq + Hash,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> HashSet<T>
where
    T: Eq + Hash,
{
    /// Empty set (`hkHashSet()`).
    #[inline]
    pub fn new() -> Self {
        Self {
            inner: RawHashSet::with_hasher(BuildHkHasher::default()),
        }
    }

    /// Empty set with a preallocated bucket count.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: RawHashSet::with_capacity_and_hasher(capacity, BuildHkHasher::default()),
        }
    }

    /// `hkHashSet::insert` — `true` if the element was new.
    #[inline]
    pub fn insert(&mut self, value: T) -> bool {
        self.inner.insert(value)
    }

    /// `hkHashSet::tryInsert` — insert and return `(stored element, was_inserted)`.
    ///
    /// Requires `T: Clone` because the C++ API returns a reference to the stored
    /// key while taking ownership of the lookup key; Rust needs a second lookup
    /// after the insert to return the map-owned element.
    pub fn try_insert(&mut self, value: T) -> (&T, bool)
    where
        T: Clone,
    {
        let existed = self.inner.contains(&value);
        if !existed {
            self.inner.insert(value.clone());
        }
        let stored = self.inner.get(&value).expect("element just inserted");
        (stored, !existed)
    }

    /// `hkHashSet::tryGet` — stored element equal to `value`.
    #[inline]
    pub fn try_get(&self, value: &T) -> Option<&T> {
        self.inner.get(value)
    }

    /// `hkHashSet::getOrInsertKey` — stored element or a newly inserted clone.
    pub fn get_or_insert(&mut self, value: T) -> &T
    where
        T: Clone,
    {
        if !self.inner.contains(&value) {
            self.inner.insert(value.clone());
        }
        self.inner
            .get(&value)
            .expect("element present after insert")
    }

    /// `hkHashSet::remove` — `true` if the element existed.
    #[inline]
    pub fn remove(&mut self, value: &T) -> bool {
        self.inner.remove(value)
    }

    /// `hkHashSet::contains` / `hasKey`.
    #[inline]
    pub fn contains(&self, value: &T) -> bool {
        self.inner.contains(value)
    }

    /// Number of elements.
    #[inline]
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Whether the set is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Bucket capacity.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    /// Reserve capacity for additional elements.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional);
    }

    /// Remove every element, keeping the allocation.
    #[inline]
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Iterate the elements.
    #[inline]
    pub fn iter(&self) -> std::collections::hash_set::Iter<'_, T> {
        self.inner.iter()
    }
}

impl<T> FromIterator<T> for HashSet<T>
where
    T: Eq + Hash,
{
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let mut set = Self::new();
        set.inner.extend(iter);
        set
    }
}

impl<T> Extend<T> for HashSet<T>
where
    T: Eq + Hash,
{
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.inner.extend(iter);
    }
}

impl<T> IntoIterator for HashSet<T>
where
    T: Eq + Hash,
{
    type Item = T;
    type IntoIter = std::collections::hash_set::IntoIter<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.into_iter()
    }
}

impl<'a, T> IntoIterator for &'a HashSet<T>
where
    T: Eq + Hash,
{
    type Item = &'a T;
    type IntoIter = std::collections::hash_set::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_reports_uniqueness() {
        let mut set = HashSet::new();
        assert!(set.insert(1));
        assert!(!set.insert(1));
        assert!(set.insert(2));
        assert_eq!(set.len(), 2);
        assert!(set.contains(&2));
        assert!(!set.contains(&3));
    }

    #[test]
    fn try_insert_and_get_or_insert() {
        let mut set = HashSet::new();
        let (stored, was_inserted) = set.try_insert("alpha");
        assert!(was_inserted);
        assert_eq!(*stored, "alpha");

        let (stored, was_inserted) = set.try_insert("alpha");
        assert!(!was_inserted);
        assert_eq!(*stored, "alpha");

        set.get_or_insert("beta");
        assert_eq!(set.len(), 2);
        assert_eq!(set.try_get(&"beta"), Some(&"beta"));
        assert_eq!(set.try_get(&"gamma"), None);
    }

    #[test]
    fn remove_and_clear() {
        let mut set: HashSet<i32> = (0..5).collect();
        assert!(set.remove(&3));
        assert!(!set.remove(&3));
        assert_eq!(set.len(), 4);
        set.clear();
        assert!(set.is_empty());
    }

    #[test]
    fn iteration_and_collection() {
        let set: HashSet<i32> = [5, 1, 3, 1].into_iter().collect();
        assert_eq!(set.len(), 3);
        let values: Vec<i32> = set.iter().copied().collect();
        assert_eq!(values.len(), 3);
        assert!(values.contains(&1) && values.contains(&3) && values.contains(&5));
    }

    #[test]
    fn iteration_order_is_deterministic() {
        let build = || -> Vec<i32> {
            let set: HashSet<i32> = (0..32).collect();
            set.iter().copied().collect()
        };
        assert_eq!(build(), build());
    }
}
