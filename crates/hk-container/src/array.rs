//! `hkArray` — growable array.
//!
//! Port of `hkArray` from
//! `original/Common/Source/Common/Base/Container/Array/` onto `Vec<T>`.
//!
//! The C++ array stores `m_data` / `m_size` / `m_capacityAndFlags` and manually
//! constructs/destructs elements. `Vec<T>` already provides that storage model,
//! so the port keeps the *API semantics* that matter:
//!
//! * `pushBack` / `popBack(num)` / `clear` / `clearAndDeallocate`
//! * `removeAt` — **unordered** removal: the last element is moved into the hole
//!   (`hkArrayBase::removeAt`, `hkArray.inl` line 360)
//! * `removeAtAndCopy` — ordered removal, elements after the hole shift down
//! * `insertAt`, `append`, `expandBy`, `setSize`, `reserve`, `optimizeCapacity`
//! * `indexOf` / `lastIndexOf`
//! * capacity growth doubles at least the requested size (the `Vec` growth policy
//!   already doubles, matching the SDK's `_growCapacity`)
//!
//! ```
//! use hk_container::Array;
//! let mut a = Array::new();
//! a.push_back(1);
//! a.push_back(2);
//! a.remove_at(0); // last element moves into index 0
//! assert_eq!(a.as_slice(), &[2]);
//! ```

use core::ops::{Index, IndexMut};

/// Growable array (`hkArray<T>`).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Array<T> {
    data: Vec<T>,
}

impl<T> Default for Array<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Array<T> {
    /// Empty array (`hkArray()`).
    #[inline]
    pub const fn new() -> Self {
        Self { data: Vec::new() }
    }

    /// Empty array with a reserved capacity (`hkArray::reserve` at construction).
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
        }
    }

    /// Array with `size` clones of `fill` (`hkArray(size, fill)`).
    #[inline]
    pub fn with_size(size: usize, fill: T) -> Self
    where
        T: Clone,
    {
        Self {
            data: vec![fill; size],
        }
    }

    /// Wrap an existing `Vec` (the port's equivalent of `hkArray::setData*`).
    #[inline]
    pub fn from_vec(data: Vec<T>) -> Self {
        Self { data }
    }

    /// Unwrap into the backing `Vec`.
    #[inline]
    pub fn into_vec(self) -> Vec<T> {
        self.data
    }

    /// `hkArray::getSize` / `getSize()` in element counts.
    #[inline]
    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// Number of elements (`getSize`).
    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// `hkArray::getCapacity`.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.data.capacity()
    }

    /// Whether the array is empty (`getSize() == 0`).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// `hkArray::isValidIndex`.
    #[inline]
    pub fn is_valid_index(&self, index: usize) -> bool {
        index < self.data.len()
    }

    /// `hkArray::reserve` — ensure capacity for at least `additional` more elements.
    ///
    /// Like the C++ version this may over-allocate (growth is amortized), and it
    /// fails fast on OOM.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.data.reserve(additional);
    }

    /// `hkArray::reserveExactly` — ensure the exact capacity.
    #[inline]
    pub fn reserve_exactly(&mut self, capacity: usize) {
        if self.data.capacity() < capacity {
            self.data.reserve_exact(capacity - self.data.len());
        }
    }

    /// Fallible variant of [`Array::reserve`].
    #[inline]
    pub fn try_reserve(
        &mut self,
        additional: usize,
    ) -> Result<(), std::collections::TryReserveError> {
        self.data.try_reserve(additional)
    }

    /// `hkArray::pushBack` (copy) — port uses the value directly.
    #[inline]
    pub fn push_back(&mut self, value: T) {
        self.data.push(value);
    }

    /// `hkArray::pushBackUnchecked` — the capacity is assumed to be sufficient.
    #[inline]
    pub fn push_back_unchecked(&mut self, value: T) {
        debug_assert!(
            self.data.len() < self.data.capacity(),
            "push_back_unchecked called without spare capacity"
        );
        self.data.push(value);
    }

    /// `hkArray::popBack()` — remove and return the last element.
    #[inline]
    pub fn pop_back(&mut self) -> Option<T> {
        self.data.pop()
    }

    /// `hkArray::popBack(numElemsToRemove)` — truncate by `num` elements.
    ///
    /// # Panics
    ///
    /// Panics if `num > len`.
    #[inline]
    pub fn pop_back_n(&mut self, num: usize) {
        assert!(num <= self.data.len(), "pop_back_n beyond array size");
        self.data.truncate(self.data.len() - num);
    }

    /// `hkArray::removeAt` — unordered removal: last element moves into `index`.
    ///
    /// Returns the removed element. Use [`Array::remove_at_and_copy`] when order
    /// must be preserved.
    #[inline]
    pub fn remove_at(&mut self, index: usize) -> T {
        self.data.swap_remove(index)
    }

    /// `hkArray::removeAtAndCopy` — ordered removal, shifting later elements down.
    #[inline]
    pub fn remove_at_and_copy(&mut self, index: usize) -> T {
        self.data.remove(index)
    }

    /// `hkArray::removeAtAndCopy(index, numToRemove)` — remove a contiguous run.
    #[inline]
    pub fn remove_at_and_copy_n(&mut self, index: usize, count: usize) {
        let end = index + count;
        assert!(end <= self.data.len(), "remove_at_and_copy_n out of range");
        self.data.drain(index..end);
    }

    /// `hkArray::orderedRemoveIf` — remove matching elements, preserving order.
    ///
    /// Returns the number of removed elements.
    #[inline]
    pub fn ordered_remove_if<F>(&mut self, mut predicate: F) -> usize
    where
        F: FnMut(&T) -> bool,
    {
        let before = self.data.len();
        self.data.retain_mut(|value| !predicate(value));
        before - self.data.len()
    }

    /// `hkArray::unorderedRemoveIf` — remove matching elements by swap-removal.
    ///
    /// Returns the number of removed elements.
    #[inline]
    pub fn unordered_remove_if<F>(&mut self, mut predicate: F) -> usize
    where
        F: FnMut(&T) -> bool,
    {
        let mut removed = 0;
        let mut index = 0;
        while index < self.data.len() {
            if predicate(&self.data[index]) {
                self.data.swap_remove(index);
                removed += 1;
            } else {
                index += 1;
            }
        }
        removed
    }

    /// `hkArray::insertAt(index, value)`.
    #[inline]
    pub fn insert_at(&mut self, index: usize, value: T) {
        self.data.insert(index, value);
    }

    /// `hkArray::append(a, numElems)` — append a slice (elements are cloned).
    #[inline]
    pub fn append(&mut self, values: &[T])
    where
        T: Clone,
    {
        self.data.extend_from_slice(values);
    }

    /// Append owned values.
    #[inline]
    pub fn append_owned<I>(&mut self, values: I)
    where
        I: IntoIterator<Item = T>,
    {
        self.data.extend(values);
    }

    /// `hkArray::expandBy(n)` — grow by `n` elements, returning the new tail slice.
    #[inline]
    pub fn expand_by(&mut self, count: usize, fill: T) -> &mut [T]
    where
        T: Clone,
    {
        let start = self.data.len();
        self.data.resize(start + count, fill);
        &mut self.data[start..]
    }

    /// `hkArray::expandOne()` — append one element and return it mutably.
    #[inline]
    pub fn expand_one(&mut self, value: T) -> &mut T {
        self.data.push(value);
        self.data.last_mut().expect("just pushed")
    }

    /// `hkArray::setSize(size, fill)`.
    #[inline]
    pub fn set_size(&mut self, size: usize, fill: T)
    where
        T: Clone,
    {
        self.data.resize(size, fill);
    }

    /// `hkArray::clear` — drop all elements, keep capacity.
    #[inline]
    pub fn clear(&mut self) {
        self.data.clear();
    }

    /// `hkArray::clearAndDeallocate` — drop elements and free storage.
    #[inline]
    pub fn clear_and_deallocate(&mut self) {
        self.data = Vec::new();
    }

    /// `hkArray::optimizeCapacity` — shrink when the free tail is too large.
    ///
    /// `num_free_elems_left` is the number of unused slots to keep;
    /// `shrink_exact` forces an exact shrink instead of a heuristic one.
    pub fn optimize_capacity(&mut self, num_free_elems_left: usize, shrink_exact: bool) {
        let capacity = self.data.capacity();
        let wanted = self.data.len() + num_free_elems_left;
        if shrink_exact {
            self.data.shrink_to_fit();
        } else if capacity > wanted.max(self.data.len()) {
            self.data.shrink_to(wanted.max(self.data.len()));
        }
    }

    /// `hkArray::indexOf` — first index where `value == needle`.
    #[inline]
    pub fn index_of(&self, needle: &T) -> Option<usize>
    where
        T: PartialEq,
    {
        self.data.iter().position(|value| value == needle)
    }

    /// `hkArray::lastIndexOf`.
    #[inline]
    pub fn last_index_of(&self, needle: &T) -> Option<usize>
    where
        T: PartialEq,
    {
        self.data.iter().rposition(|value| value == needle)
    }

    /// `hkArray::front` — first element, if any.
    #[inline]
    pub fn front(&self) -> Option<&T> {
        self.data.first()
    }

    /// `hkArray::back` — last element, if any.
    #[inline]
    pub fn back(&self) -> Option<&T> {
        self.data.last()
    }

    /// `hkArray::swap`.
    #[inline]
    pub fn swap(&mut self, other: &mut Self) {
        core::mem::swap(&mut self.data, &mut other.data);
    }

    /// Borrow the contents as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// Borrow the contents as a mutable slice.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// `hkArray::get(index)` — bounds-checked element access.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&T> {
        self.data.get(index)
    }

    /// Mutable bounds-checked element access.
    #[inline]
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.data.get_mut(index)
    }

    /// Iterator over the elements.
    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, T> {
        self.data.iter()
    }

    /// Mutable iterator over the elements.
    #[inline]
    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, T> {
        self.data.iter_mut()
    }

    /// `hkArray::begin()` as a raw pointer to the first element (for FFI-style code).
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
    }

    /// `hkArray::begin()` as a mutable raw pointer.
    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.data.as_mut_ptr()
    }
}

impl<T> Index<usize> for Array<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: usize) -> &T {
        &self.data[index]
    }
}

impl<T> IndexMut<usize> for Array<T> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut T {
        &mut self.data[index]
    }
}

impl<T> FromIterator<T> for Array<T> {
    #[inline]
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        Self {
            data: Vec::from_iter(iter),
        }
    }
}

impl<T> Extend<T> for Array<T> {
    #[inline]
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.data.extend(iter);
    }
}

impl<'a, T: Clone> Extend<&'a T> for Array<T> {
    #[inline]
    fn extend<I: IntoIterator<Item = &'a T>>(&mut self, iter: I) {
        self.data.extend(iter.into_iter().cloned());
    }
}

impl<T> IntoIterator for Array<T> {
    type Item = T;
    type IntoIter = std::vec::IntoIter<T>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.data.into_iter()
    }
}

impl<'a, T> IntoIterator for &'a Array<T> {
    type Item = &'a T;
    type IntoIter = core::slice::Iter<'a, T>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.data.iter()
    }
}

impl<'a, T> IntoIterator for &'a mut Array<T> {
    type Item = &'a mut T;
    type IntoIter = core::slice::IterMut<'a, T>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.data.iter_mut()
    }
}

impl<T> From<Vec<T>> for Array<T> {
    #[inline]
    fn from(data: Vec<T>) -> Self {
        Self::from_vec(data)
    }
}

impl<T> From<Array<T>> for Vec<T> {
    #[inline]
    fn from(array: Array<T>) -> Self {
        array.into_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_pop_and_clear() {
        let mut a = Array::new();
        assert!(a.is_empty());
        a.push_back(10);
        a.push_back(20);
        a.push_back(30);
        assert_eq!(a.len(), 3);
        assert_eq!(a.as_slice(), &[10, 20, 30]);
        assert_eq!(a.pop_back(), Some(30));
        assert_eq!(a.len(), 2);
        a.pop_back_n(2);
        assert!(a.is_empty());
        assert!(a.pop_back().is_none());

        a.push_back(1);
        a.clear();
        assert_eq!(a.len(), 0);
        assert!(a.capacity() >= 1, "clear keeps the allocation");
        a.clear_and_deallocate();
        assert_eq!(a.capacity(), 0);
    }

    #[test]
    fn remove_at_is_unordered_and_remove_at_and_copy_is_ordered() {
        let mut unordered = Array::from_vec(vec![0, 1, 2, 3]);
        let removed = unordered.remove_at(1);
        assert_eq!(removed, 1);
        assert_eq!(
            unordered.as_slice(),
            &[0, 3, 2],
            "last element moves into the hole"
        );

        let mut ordered = Array::from_vec(vec![0, 1, 2, 3]);
        let removed = ordered.remove_at_and_copy(1);
        assert_eq!(removed, 1);
        assert_eq!(ordered.as_slice(), &[0, 2, 3]);

        let mut run = Array::from_vec(vec![0, 1, 2, 3, 4]);
        run.remove_at_and_copy_n(1, 2);
        assert_eq!(run.as_slice(), &[0, 3, 4]);
    }

    #[test]
    fn insert_append_and_expand() {
        let mut a = Array::from_vec(vec![1, 3]);
        a.insert_at(1, 2);
        assert_eq!(a.as_slice(), &[1, 2, 3]);

        a.append(&[4, 5]);
        assert_eq!(a.as_slice(), &[1, 2, 3, 4, 5]);

        let tail = a.expand_by(2, 9);
        assert_eq!(tail, &mut [9, 9]);
        assert_eq!(a.len(), 7);

        let one = a.expand_one(7);
        *one = 8;
        assert_eq!(a.as_slice(), &[1, 2, 3, 4, 5, 9, 9, 8]);

        a.set_size(3, 0);
        assert_eq!(a.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn predicates_and_index_search() {
        let mut a = Array::from_vec(vec![5, -2, 7, -4, 9]);
        let removed = a.ordered_remove_if(|value| *value < 0);
        assert_eq!(removed, 2);
        assert_eq!(a.as_slice(), &[5, 7, 9]);

        let mut b = Array::from_vec(vec![5, -2, 7, -4, 9]);
        let removed = b.unordered_remove_if(|value| *value < 0);
        assert_eq!(removed, 2);
        assert_eq!(b.as_slice(), &[5, 9, 7]);

        let c = Array::from_vec(vec![1, 2, 3, 2]);
        assert_eq!(c.index_of(&2), Some(1));
        assert_eq!(c.last_index_of(&2), Some(3));
        assert_eq!(c.index_of(&9), None);
    }

    #[test]
    fn reserve_and_optimize_capacity() {
        let mut a: Array<u32> = Array::new();
        a.reserve(100);
        assert!(a.capacity() >= 100);
        a.push_back(1);
        a.push_back(2);

        a.optimize_capacity(0, true);
        assert_eq!(a.capacity(), 2, "exact shrink keeps the live elements");

        a.optimize_capacity(16, false);
        assert!(a.capacity() >= 2);
    }

    #[test]
    fn indexing_iteration_and_conversions() {
        let mut a = Array::from_vec(vec![10, 20, 30]);
        assert_eq!(a[1], 20);
        a[1] = 21;
        assert_eq!(a[1], 21);
        assert_eq!(a.get(9), None);
        assert_eq!(a.iter().copied().sum::<i32>(), 61);

        let collected: Array<i32> = (1..=3).collect();
        assert_eq!(collected.as_slice(), &[1, 2, 3]);

        let vec = collected.into_vec();
        assert_eq!(vec, vec![1, 2, 3]);
    }

    #[test]
    fn try_reserve_reports_failure_without_panicking() {
        let mut a: Array<u8> = Array::new();
        // A request of usize::MAX elements cannot be satisfied.
        assert!(a.try_reserve(usize::MAX).is_err());
    }

    #[test]
    fn push_back_unchecked_requires_capacity() {
        let mut a = Array::with_capacity(2);
        a.push_back_unchecked(1);
        a.push_back_unchecked(2);
        assert_eq!(a.len(), 2);
    }
}
