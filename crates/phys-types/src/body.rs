//! `hknpBodyId` / `hknpBodyIndex`.
//!
//! Ports of
//! `original/Physics/Source/Physics/Physics/Dynamics/Body/hknpBodyId.h` and
//! `hknpBodyIndex.h`.
//!
//! `hknpBodyIndex` packs a 24-bit buffer index and an 8-bit serial number; an
//! all-ones index marks the invalid value. `hknpBodyId` is a 64-bit value whose
//! most significant bit says whether the rest encodes an internal
//! [`BodyIndex`] (set) or an arbitrary user ID (clear).

/// Internal body index (24-bit index + 8-bit serial).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct BodyIndex(u32);

impl BodyIndex {
    /// `NUM_INDEX_BITS`.
    pub const NUM_INDEX_BITS: u32 = 24;
    /// `INDEX_MASK`.
    pub const INDEX_MASK: u32 = 0x00FF_FFFF;
    /// The invalid index (`0xffffffff`: serial 0xff, index 0xffffff).
    pub const INVALID: Self = Self(u32::MAX);

    /// Set both the 24-bit index and the 8-bit serial number.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if the index does not fit into 24 bits.
    #[inline]
    pub const fn new(index: u32, serial: u8) -> Self {
        debug_assert!(index & Self::INDEX_MASK == index, "body index out of range");
        Self((serial as u32) << Self::NUM_INDEX_BITS | (index & Self::INDEX_MASK))
    }

    /// `hknpBodyIndex::invalid`.
    #[inline]
    pub const fn invalid() -> Self {
        Self::INVALID
    }

    /// Whether the 24-bit index is not the invalid sentinel.
    #[inline]
    pub const fn is_valid(self) -> bool {
        (self.0 & Self::INDEX_MASK) != Self::INDEX_MASK
    }

    /// The 24-bit index, asserting validity in debug builds.
    #[inline]
    pub const fn index(self) -> u32 {
        debug_assert!(self.is_valid(), "invalid body index");
        self.0 & Self::INDEX_MASK
    }

    /// The 24-bit index without a validity check (`indexUnchecked`).
    #[inline]
    pub const fn index_unchecked(self) -> u32 {
        self.0 & Self::INDEX_MASK
    }

    /// The 8-bit serial number.
    #[inline]
    pub const fn serial(self) -> u8 {
        (self.0 >> Self::NUM_INDEX_BITS) as u8
    }

    /// `hknpBodyIndex::setIndex`.
    #[inline]
    pub fn set_index(&mut self, index: u32) {
        debug_assert!(index & Self::INDEX_MASK == index, "body index out of range");
        self.0 = (self.0 & !Self::INDEX_MASK) | (index & Self::INDEX_MASK);
    }

    /// `hknpBodyIndex::setSerial`.
    #[inline]
    pub fn set_serial(&mut self, serial: u8) {
        self.0 = (serial as u32) << Self::NUM_INDEX_BITS | (self.0 & Self::INDEX_MASK);
    }

    /// Raw packed value (`toRawInt`).
    #[inline]
    pub const fn to_raw_int(self) -> u32 {
        self.0
    }

    /// Wrap a raw packed value (`fromRawInt`).
    #[inline]
    pub const fn from_raw_int(raw: u32) -> Self {
        Self(raw)
    }
}

impl Default for BodyIndex {
    #[inline]
    fn default() -> Self {
        Self::INVALID
    }
}

impl core::fmt::Debug for BodyIndex {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.is_valid() {
            write!(
                f,
                "BodyIndex(index={}, serial={})",
                self.index_unchecked(),
                self.serial()
            )
        } else {
            f.write_str("BodyIndex(invalid)")
        }
    }
}

/// Unique body identifier (`hknpBodyId`).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct BodyId(u64);

impl BodyId {
    /// Most significant bit: set when the value stores an internal [`BodyIndex`].
    pub const INDEX_FLAG: u64 = 1u64 << 63;

    /// The invalid ID (`!0`).
    pub const INVALID: Self = Self(u64::MAX);

    /// Construct an arbitrary (user) ID. If the top bit is set it will be
    /// interpreted as a body index, exactly like the C++ constructor warning.
    #[inline]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// `hknpBodyId::invalid`.
    #[inline]
    pub const fn invalid() -> Self {
        Self::INVALID
    }

    /// Whether this is not the invalid ID.
    #[inline]
    pub const fn is_valid(self) -> bool {
        self.0 != u64::MAX
    }

    /// Raw stored value.
    #[inline]
    pub const fn value(self) -> u64 {
        self.0
    }

    /// Build an ID that stores an internal body index (sets the MSB).
    #[inline]
    pub const fn from_body_index(index: BodyIndex) -> Self {
        Self(Self::INDEX_FLAG | index.to_raw_int() as u64)
    }

    /// `hknpBodyId::isBodyIndex`.
    #[inline]
    pub const fn is_body_index(self) -> bool {
        self.0 & Self::INDEX_FLAG != 0
    }

    /// The stored body index, if this ID encodes one and is valid.
    #[inline]
    pub const fn as_body_index(self) -> Option<BodyIndex> {
        if self.is_body_index() && self.is_valid() {
            Some(BodyIndex::from_raw_int(self.0 as u32))
        } else {
            None
        }
    }
}

impl Default for BodyId {
    #[inline]
    fn default() -> Self {
        Self::INVALID
    }
}

impl core::fmt::Debug for BodyId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if !self.is_valid() {
            f.write_str("BodyId(invalid)")
        } else if let Some(index) = self.as_body_index() {
            write!(f, "BodyId(index={index:?})")
        } else {
            write!(f, "BodyId(user={:#x})", self.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_index_packing() {
        let index = BodyIndex::new(0x1234, 0xAB);
        assert_eq!(index.index(), 0x1234);
        assert_eq!(index.serial(), 0xAB);
        assert_eq!(index.to_raw_int(), (0xAB << 24) | 0x1234);
        assert!(index.is_valid());

        let invalid = BodyIndex::invalid();
        assert_eq!(invalid.to_raw_int(), u32::MAX);
        assert!(!invalid.is_valid());
        assert_eq!(BodyIndex::default(), invalid);
    }

    #[test]
    fn body_index_setters() {
        let mut index = BodyIndex::new(1, 2);
        index.set_index(0xFF_0000);
        index.set_serial(0x10);
        assert_eq!(index.index_unchecked(), 0xFF_0000);
        assert_eq!(index.serial(), 0x10);
        assert!(index.is_valid());
    }

    #[test]
    fn body_id_user_vs_index() {
        let user = BodyId::new(42);
        assert!(user.is_valid());
        assert!(!user.is_body_index());
        assert_eq!(user.value(), 42);
        assert_eq!(user.as_body_index(), None);

        let from_index = BodyId::from_body_index(BodyIndex::new(5, 1));
        assert!(from_index.is_valid());
        assert!(from_index.is_body_index());
        assert_eq!(from_index.as_body_index().map(BodyIndex::index), Some(5));
        assert_eq!(from_index.value(), BodyId::INDEX_FLAG | ((1u64 << 24) | 5));

        let invalid = BodyId::invalid();
        assert!(!invalid.is_valid());
        assert!(invalid.is_body_index(), "invalid is all ones");
        assert_eq!(invalid.as_body_index(), None, "invalid id yields no index");
        assert_eq!(BodyId::default(), invalid);
    }

    #[test]
    fn body_id_is_hashable() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        assert!(set.insert(BodyId::new(7)));
        assert!(!set.insert(BodyId::new(7)));
    }
}
