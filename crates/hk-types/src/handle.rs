//! Typed opaque handles.
//!
//! Port of `hkHandle` / `HK_DECLARE_HANDLE` from
//! `original/Common/Source/Common/Base/Types/hkHandle.h`.
//!
//! The C++ template is `hkHandle<TYPE, INVALID_VALUE, DISCRIMINANT>`; the
//! discriminant is a dummy type that makes handles with different names
//! incompatible even when their storage and invalid value are identical. In Rust
//! the same trick is used: the type parameter `T` is never instantiated, it only
//! distinguishes handle families at compile time. The storage is `u32`, as stated
//! by the migration plan (README §4 Phase 1).
//!
//! ```
//! use hk_types::Handle;
//! struct TagA;
//! struct TagB;
//!
//! type A = Handle<TagA, 0xFFFF_FFFF>;
//! type B = Handle<TagB, 0xFFFF_FFFF>;
//!
//! let a = A::new(7);
//! assert!(a.is_valid());
//! assert_eq!(a.value(), 7);
//! let _b = B::new(7); // same value, different type; `a == _b` does not compile.
//! ```
//!
//! Handles are declared in bulk with [`crate::declare_handle!`].

use core::any::type_name;
use core::fmt;
use core::hash::{Hash, Hasher};
use core::marker::PhantomData;

/// An opaque typed index.
///
/// * `T` is a *tag* (discriminant) type and carries no data.
/// * `INVALID` is the sentinel value marking "no handle"; it defaults to
///   `u32::MAX` (`0xFFFF_FFFF`), which is the invalid value used by most Havok
///   handle families.
///
/// Construction from a raw integer is explicit (the C++ constructor is
/// `explicit` too), which prevents mixing integer indices and handles by
/// accident. Comparing handles of different tag types is a compile error.
#[repr(transparent)]
pub struct Handle<T, const INVALID: u32 = { u32::MAX }> {
    value: u32,
    // Covariant in `T`, no drop glue, and no auto-trait impact: it only tags.
    _tag: PhantomData<fn() -> T>,
}

impl<T, const INVALID: u32> Handle<T, INVALID> {
    /// Wrap a raw index value (explicit, like the C++ constructor).
    ///
    /// This does **not** validate the value; use [`Handle::is_valid`] /
    /// [`Handle::value`] to check.
    #[inline]
    pub const fn new(raw: u32) -> Self {
        Self {
            value: raw,
            _tag: PhantomData,
        }
    }

    /// The handle with the invalid sentinel value.
    #[inline]
    pub const fn invalid() -> Self {
        Self::new(INVALID)
    }

    /// The `INVALID_VALUE` template argument.
    #[inline]
    pub const fn invalid_value() -> u32 {
        INVALID
    }

    /// `hkHandle::isValid`: `value != INVALID_VALUE`.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.value != INVALID
    }

    /// Read the raw value, checking for the invalid sentinel in debug builds.
    ///
    /// This is the safe counterpart of `hkHandle::value()`; the C++ version
    /// asserts on the invalid value, which is mapped to `debug_assert!` here.
    #[inline]
    pub const fn value(&self) -> u32 {
        debug_assert!(
            self.is_valid(),
            "Handle::value() called on an invalid handle"
        );
        self.value
    }

    /// `hkHandle::valueUnchecked`: read the raw value without any check.
    #[inline]
    pub const fn value_unchecked(&self) -> u32 {
        self.value
    }

    /// `hkHandle::setInvalid`: overwrite with the sentinel value.
    #[inline]
    pub const fn set_invalid(&mut self) {
        self.value = INVALID;
    }

    /// Construct from a `u32` without any checks; alias for [`Handle::new`].
    #[inline]
    pub const fn from_raw_unchecked(raw: u32) -> Self {
        Self::new(raw)
    }
}

impl<T, const INVALID: u32> Copy for Handle<T, INVALID> {}

impl<T, const INVALID: u32> Clone for Handle<T, INVALID> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}

impl<T, const INVALID: u32> Default for Handle<T, INVALID> {
    /// Rust default is the *invalid* handle, unlike the C++ default constructor
    /// which leaves the storage uninitialized. Choosing the sentinel is strictly
    /// safer and matches the `hk::Default(INVALID_VALUE)` reflection attribute.
    #[inline]
    fn default() -> Self {
        Self::invalid()
    }
}

impl<T, const INVALID: u32> PartialEq for Handle<T, INVALID> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<T, const INVALID: u32> Eq for Handle<T, INVALID> {}

impl<T, const INVALID: u32> PartialOrd for Handle<T, INVALID> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T, const INVALID: u32> Ord for Handle<T, INVALID> {
    #[inline]
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.value.cmp(&other.value)
    }
}

impl<T, const INVALID: u32> Hash for Handle<T, INVALID> {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.value.hash(state);
    }
}

impl<T, const INVALID: u32> fmt::Debug for Handle<T, INVALID> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Handle")
            .field("tag", &type_name::<T>())
            .field("value", &self.value)
            .field("valid", &self.is_valid())
            .finish()
    }
}

/// Declare a typed handle family, mirroring `HK_DECLARE_HANDLE`.
///
/// The C++ macro is
/// `HK_DECLARE_HANDLE(name, type, invalid)`, which declares a discriminant
/// struct and a typedef. Rust needs the tag struct to be named explicitly because
/// identifiers cannot be concatenated without an extra crate.
///
/// # Examples
///
/// ```
/// use hk_types::declare_handle;
///
/// declare_handle! {
///     /// A body index.
///     pub struct BodyId, tag: BodyIdTag, invalid: 0xFFFF_FFFF,
/// }
///
/// let id = BodyId::new(3);
/// assert_eq!(id.value(), 3);
/// assert!(!BodyId::invalid().is_valid());
/// ```
///
/// The doc comment is applied to both the tag type and the alias.
#[macro_export]
macro_rules! declare_handle {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident, tag: $tag:ident, invalid: $invalid:expr $(,)?
    ) => {
        $(#[$meta])*
        /// Discriminant type; never instantiated.
        $vis struct $tag;

        $(#[$meta])*
        $vis type $name = $crate::Handle<$tag, { $invalid }>;
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::any::TypeId;

    crate::declare_handle! {
        /// Test handle family A.
        pub struct TestHandleA, tag: TestHandleATag, invalid: 0xFFFF_FFFF,
    }

    crate::declare_handle! {
        /// Test handle family B, different invalid value.
        pub struct TestHandleB, tag: TestHandleBTag, invalid: 0,
    }

    #[test]
    fn new_and_validity() {
        let h = TestHandleA::new(42);
        assert!(h.is_valid());
        assert_eq!(h.value(), 42);
        assert_eq!(h.value_unchecked(), 42);

        assert!(!TestHandleA::invalid().is_valid());
        assert_eq!(TestHandleA::invalid_value(), 0xFFFF_FFFF);
        assert_eq!(TestHandleA::default(), TestHandleA::invalid());
    }

    #[test]
    fn default_is_invalid_for_every_family() {
        assert!(!TestHandleB::default().is_valid());
        assert_eq!(TestHandleB::invalid_value(), 0);
        assert!(!TestHandleB::new(0).is_valid());
        assert!(TestHandleB::new(1).is_valid());
    }

    #[test]
    fn set_invalid() {
        let mut h = TestHandleA::new(7);
        h.set_invalid();
        assert!(!h.is_valid());
        assert_eq!(h.value_unchecked(), 0xFFFF_FFFF);
    }

    #[test]
    fn ordering_and_hash_use_the_raw_value() {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let a = TestHandleA::new(1);
        let b = TestHandleA::new(2);
        assert!(a < b);
        assert_eq!(a.cmp(&b), core::cmp::Ordering::Less);

        let mut ha = DefaultHasher::new();
        a.hash(&mut ha);
        let mut hb = DefaultHasher::new();
        TestHandleA::new(1).hash(&mut hb);
        assert_eq!(ha.finish(), hb.finish());
    }

    #[test]
    fn handle_families_are_distinct_types() {
        assert_ne!(
            TypeId::of::<TestHandleA>(),
            TypeId::of::<TestHandleB>(),
            "different tags must produce different types"
        );
        // Same storage and invalid value, still distinct because of the tag.
        struct SameTagA;
        struct SameTagB;
        type A = Handle<SameTagA, 0xFFFF_FFFF>;
        type B = Handle<SameTagB, 0xFFFF_FFFF>;
        assert_ne!(TypeId::of::<A>(), TypeId::of::<B>());
    }

    #[test]
    fn size_of_handle_is_one_u32() {
        assert_eq!(core::mem::size_of::<TestHandleA>(), 4);
        // `u32` has no niche, so `Option<Handle>` grows by a discriminant byte.
        // The C++ API uses the INVALID_VALUE sentinel instead of an optional.
        assert_eq!(core::mem::size_of::<Option<TestHandleA>>(), 8);
    }
}
