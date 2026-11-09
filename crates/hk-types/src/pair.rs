//! `hkPair` — a two-field key/value struct.
//!
//! Port of `hkPair` from
//! `original/Common/Source/Common/Base/Types/hkBaseTypes.h` (line 18).
//!
//! The original has reflection metadata and piecewise construction for
//! `hkHashMap`; neither is needed in this phase. The field order (`m_0`, `m_1`)
//! is preserved so that any future serialized layout stays compatible.

/// Key/value pair.
///
/// In C++ this is the element type of `hkHashMap` / `hkHashSet`; in the Rust port
/// `hashbrown`/`std` map entry pairs (`(K, V)`) are often a better fit. `Pair`
/// remains available where the exact C++ layout or API matters (serialization,
/// reflection-shaped call sites).
///
/// ```
/// use hk_types::Pair;
/// let p = Pair::new("a", 1u32);
/// assert_eq!(p.get_key(), &"a");
/// assert_eq!(p.get_value(), &1);
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Pair<K, V> {
    /// `hkPair::m_0`.
    pub key: K,
    /// `hkPair::m_1`.
    pub value: V,
}

impl<K, V> Pair<K, V> {
    /// Create a pair from its two fields.
    #[inline]
    pub const fn new(key: K, value: V) -> Self {
        Self { key, value }
    }

    /// `hkPair::getKey`.
    #[inline]
    pub const fn get_key(&self) -> &K {
        &self.key
    }

    /// `hkPair::getValue` (const form).
    #[inline]
    pub const fn get_value(&self) -> &V {
        &self.value
    }

    /// `hkPair::getValue` (mutable form).
    #[inline]
    pub fn get_value_mut(&mut self) -> &mut V {
        &mut self.value
    }

    /// Borrow both halves as a Rust tuple.
    #[inline]
    pub fn as_tuple(&self) -> (&K, &V) {
        (&self.key, &self.value)
    }

    /// Consume the pair and return a Rust tuple.
    #[inline]
    pub fn into_tuple(self) -> (K, V) {
        (self.key, self.value)
    }
}

impl<K, V> From<(K, V)> for Pair<K, V> {
    #[inline]
    fn from((key, value): (K, V)) -> Self {
        Self { key, value }
    }
}

impl<K, V> From<Pair<K, V>> for (K, V) {
    #[inline]
    fn from(pair: Pair<K, V>) -> Self {
        (pair.key, pair.value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_order_and_accessors() {
        let mut p = Pair::new(1u32, 2u64);
        assert_eq!(*p.get_key(), 1);
        assert_eq!(*p.get_value(), 2);
        *p.get_value_mut() = 3;
        assert_eq!(p.get_value(), &3);
        assert_eq!(p.as_tuple(), (&1, &3));
        assert_eq!(Pair::from((1u32, 3u64)), p);
        assert_eq!(<(u32, u64)>::from(p), (1, 3));
    }

    #[test]
    fn layout_is_key_then_value() {
        // `#[repr(C)]` guarantees field order, matching `hkPair::m_0, m_1`.
        // (`offset_of!` is the sound way to inspect layout: padding bytes are not
        // readable, unlike C++ where reading them is merely questionable.)
        assert_eq!(core::mem::offset_of!(Pair<u32, u64>, key), 0);
        assert_eq!(core::mem::offset_of!(Pair<u32, u64>, value), 8);
        assert_eq!(core::mem::size_of::<Pair<u32, u64>>(), 16);
    }
}
