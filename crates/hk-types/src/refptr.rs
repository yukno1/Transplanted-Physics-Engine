//! Intrusive reference counting.
//!
//! Port of `hkReferencedObject`
//! (`original/Common/Source/Common/Base/Object/hkReferencedObject.*`) and
//! `hkRefPtr` (`original/Common/Source/Common/Base/Types/hkRefPtr.*`).
//!
//! # Design
//!
//! The original stores an atomic 64-bit reference count *inside* every
//! `hkReferencedObject`, adds a reference when an object is handed out, and calls
//! `delete this` when the last reference is removed. This port keeps that
//! intrusive shape rather than wrapping objects in [`std::sync::Arc`]:
//!
//! * [`RefCount`] is the atomic counter embedded in the object (default value `1`,
//!   matching `hkReferencedObject`'s constructor).
//! * [`RefCounted`] is an `unsafe trait` implemented by types that embed a
//!   `RefCount` field. Implementing it promises that `refcount()` returns that
//!   field and that the object was allocated in the way [`RefCounted::destroy`]
//!   expects.
//! * [`RefPtr`] owns one reference; cloning adds one, dropping removes one and
//!   destroys the object at zero.
//!
//! ```
//! use hk_types::{RefCount, RefCounted, RefPtr};
//!
//! struct MyObject {
//!     ref_count: RefCount,
//!     value: i32,
//! }
//!
//! // SAFETY: `refcount` returns the embedded counter, the object is heap
//! // allocated by `RefPtr::new`, and there is no custom destroy hook.
//! unsafe impl RefCounted for MyObject {
//!     fn refcount(&self) -> &RefCount {
//!         &self.ref_count
//!     }
//! }
//!
//! let a = RefPtr::new(MyObject { ref_count: RefCount::new(), value: 7 });
//! let b = a.clone();
//! assert_eq!(a.ref_count(), 2);
//! assert_eq!(a.value, 7);
//! drop(b);
//! assert_eq!(a.ref_count(), 1);
//! ```
//!
//! # Differences from `hkRefPtr`
//!
//! `hkViewPtr` (the non-owning "weak" wrapper) has no counterpart here yet:
//! non-owning access is expressed with plain references or raw pointers in Rust.
//! `hkDynCast`-style casts depend on the reflection system and are deferred.
//! `RefPtr` is single-threaded in the sense that it does not require `T: Sync` to
//! exist, but the counter itself is atomic like the original.

use core::fmt;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU32, Ordering};

/// The atomic reference count embedded in a [`RefCounted`] object.
///
/// A freshly constructed count is `1`, exactly like `hkReferencedObject`'s
/// constructor. Counts are `u32` in this port; the original uses `hkUlong`
/// (64-bit), which does not affect reachable reference counts in practice but is
/// noted here for people diffing layouts.
#[repr(transparent)]
pub struct RefCount {
    count: AtomicU32,
}

impl RefCount {
    /// Creates a count with the initial value `1` (the object owns one reference).
    #[inline]
    pub const fn new() -> Self {
        Self {
            count: AtomicU32::new(1),
        }
    }

    /// Current strong reference count.
    #[inline]
    pub fn get(&self) -> u32 {
        self.count.load(Ordering::Acquire)
    }

    /// `hkReferencedObject::addReference`: relaxed increment.
    ///
    /// The caller already holds a reference, so no ordering is required for the
    /// increment; the release side uses `AcqRel` (see [`RefCount::release`]).
    #[inline]
    fn add(&self) {
        let old = self.count.fetch_add(1, Ordering::Relaxed);
        debug_assert!(
            old != u32::MAX,
            "reference count overflow; are you calling addReference() manually?"
        );
    }

    /// `hkReferencedObject::removeReference`: `AcqRel` decrement.
    ///
    /// Returns the count *before* the decrement, so `1` means this was the last
    /// reference.
    #[inline]
    fn release(&self) -> u32 {
        let old = self.count.fetch_sub(1, Ordering::AcqRel);
        debug_assert!(
            old != 0,
            "removing a reference from an object with zero references"
        );
        old
    }
}

impl Default for RefCount {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Debug for RefCount {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("RefCount").field(&self.get()).finish()
    }
}

/// Objects that a [`RefPtr`] can own.
///
/// # Safety
///
/// Implementors must:
///
/// * embed a [`RefCount`] and return it from [`RefCounted::refcount`];
/// * be allocated in the way their [`RefCounted::destroy`] implementation expects
///   (the default implementation calls `Box::from_raw`, so objects created with
///   [`RefPtr::new`] are fine);
/// * start with a reference count of `1` (`RefCount::new()` does this).
pub unsafe trait RefCounted: Sized + 'static {
    /// Borrow the embedded reference count.
    fn refcount(&self) -> &RefCount;

    /// Destroy `this` when the last reference goes away.
    ///
    /// The default frees a `Box`-allocated object. Override this if the original
    /// class had a custom `deleteThisReferencedObject` (for example pooling or
    /// deferred deletion).
    ///
    /// # Safety
    ///
    /// Called exactly once, with the last reference already removed, and `this` is
    /// the only remaining pointer to the object.
    unsafe fn destroy(this: NonNull<Self>) {
        // SAFETY: the trait contract guarantees `this` came from `Box::new`, and
        // the reference count reaching zero means no other owner remains.
        unsafe { drop(Box::from_raw(this.as_ptr())) };
    }
}

/// Owning smart pointer to a [`RefCounted`] object.
///
/// `RefPtr<T>` is `None` (a null pointer) when empty, so it can be default
/// constructed. `Deref`/`DerefMut` give access to the object, and comparisons
/// compare addresses (aliasing pointers to the same object are equal).
pub struct RefPtr<T: RefCounted> {
    ptr: Option<NonNull<T>>,
    _marker: PhantomData<T>,
}

impl<T: RefCounted> RefPtr<T> {
    /// Allocate `value` on the heap and return an owning `RefPtr` to it.
    ///
    /// Equivalent to `hk::makeRef<T>(...)`. The embedded count must be `1`
    /// (`RefCount::new()` / `RefCount::default()`).
    #[inline]
    pub fn new(value: T) -> Self {
        debug_assert!(
            value.refcount().get() == 1,
            "RefCounted values must start with a reference count of 1"
        );
        let raw = Box::into_raw(Box::new(value));
        // SAFETY: `Box::into_raw` never returns null.
        unsafe { Self::from_raw_owned(raw) }
    }

    /// Adopt an existing reference encoded in a raw pointer.
    ///
    /// This does **not** increment the count: the caller transfers the reference
    /// it already owns to the `RefPtr`. Port of the `DontIncrementRefcount`
    /// constructor (`hkRefPtr(RTYPE* ptr, DontIncrementRefcount)`).
    ///
    /// # Safety
    ///
    /// `ptr` must be null, or point to a live `T` with at least one reference that
    /// the caller hands over. The pointer must not be used again as a borrowed
    /// reference after this call.
    pub unsafe fn from_raw_owned(ptr: *mut T) -> Self {
        match NonNull::new(ptr) {
            Some(ptr) => {
                // SAFETY: the caller guarantees `ptr` is a live object.
                debug_assert!(unsafe { ptr.as_ref() }.refcount().get() >= 1);
                Self {
                    ptr: Some(ptr),
                    _marker: PhantomData,
                }
            }
            None => Self::default(),
        }
    }

    /// Create an owning pointer from a raw pointer, adding one reference.
    ///
    /// Port of `hkRefPtr(TYPE* e)` (the pointer constructor that increments).
    ///
    /// # Safety
    ///
    /// `ptr` must be null, or point to a live `T` that stays alive while this
    /// `RefPtr` exists.
    pub unsafe fn from_raw_shared(ptr: *mut T) -> Self {
        match NonNull::new(ptr) {
            Some(ptr) => {
                // SAFETY: caller guarantees a live object.
                unsafe { ptr.as_ref() }.refcount().add();
                Self {
                    ptr: Some(ptr),
                    _marker: PhantomData,
                }
            }
            None => Self::default(),
        }
    }

    /// Whether this pointer is null.
    #[inline]
    pub fn is_null(&self) -> bool {
        self.ptr.is_none()
    }

    /// Raw pointer, null when empty.
    #[inline]
    pub fn as_ptr(&self) -> *mut T {
        match self.ptr {
            Some(ptr) => ptr.as_ptr(),
            None => core::ptr::null_mut(),
        }
    }

    /// Borrow the object, if any.
    #[inline]
    pub fn as_ref(&self) -> Option<&T> {
        match self.ptr {
            Some(ptr) => {
                // SAFETY: `RefPtr` owns a live reference for as long as `self` lives.
                Some(unsafe { ptr.as_ref() })
            }
            None => None,
        }
    }

    /// Mutably borrow the object, if any.
    #[inline]
    pub fn as_mut(&mut self) -> Option<&mut T> {
        match self.ptr {
            Some(mut ptr) => {
                // SAFETY: `&mut self` gives unique access through this owner.
                Some(unsafe { ptr.as_mut() })
            }
            None => None,
        }
    }

    /// Current reference count of the pointed object (`0` when empty).
    #[inline]
    pub fn ref_count(&self) -> u32 {
        match self.as_ref() {
            Some(object) => object.refcount().get(),
            None => 0,
        }
    }

    /// `hkRefPtr::release`: give up ownership of the reference and return the raw
    /// pointer without decrementing the count.
    ///
    /// The caller becomes responsible for the reference. Returns null when empty.
    #[inline]
    pub fn release(&mut self) -> *mut T {
        match self.ptr.take() {
            Some(ptr) => ptr.as_ptr(),
            None => core::ptr::null_mut(),
        }
    }

    /// `hkRefPtr::unsetAndDontDecrementRefCount`: forget the object without
    /// removing the reference.
    ///
    /// This leaks the reference unless the caller obtained it separately (usually
    /// via [`RefPtr::as_ptr`] plus [`RefPtr::from_raw_owned`] elsewhere).
    #[inline]
    pub fn unset_and_dont_decrement(&mut self) {
        self.ptr = None;
    }

    /// `hkRefPtr::setAndDontIncrementRefCount`: replace the stored pointer without
    /// incrementing the reference count of `ptr`, removing the old reference.
    ///
    /// # Safety
    ///
    /// `ptr` must be null, or point to a live object whose existing reference the
    /// caller transfers to this `RefPtr`.
    pub unsafe fn set_and_dont_increment_ref_count(&mut self, ptr: *mut T) {
        let new_ptr = NonNull::new(ptr);
        if self.ptr == new_ptr {
            return;
        }
        if let Some(old) = self.ptr.take() {
            Self::release_ptr(old);
        }
        self.ptr = new_ptr;
    }

    /// Swap the stored pointers without touching the reference counts.
    ///
    /// Port of `hkRefPtr::swap`.
    #[inline]
    pub fn swap(&mut self, other: &mut Self) {
        core::mem::swap(&mut self.ptr, &mut other.ptr);
    }

    /// Remove one reference and destroy the object when it reaches zero.
    #[inline]
    fn release_ptr(ptr: NonNull<T>) {
        // SAFETY: this `RefPtr` owns a live reference to `ptr`.
        let old_count = unsafe { ptr.as_ref() }.refcount().release();
        if old_count == 1 {
            // SAFETY: last reference removed; contract of `RefCounted::destroy`.
            unsafe { T::destroy(ptr) };
        }
    }
}

impl<T: RefCounted> Default for RefPtr<T> {
    #[inline]
    fn default() -> Self {
        Self {
            ptr: None,
            _marker: PhantomData,
        }
    }
}

impl<T: RefCounted> Clone for RefPtr<T> {
    #[inline]
    fn clone(&self) -> Self {
        if let Some(ptr) = self.ptr {
            // SAFETY: `self` owns a live reference.
            unsafe { ptr.as_ref() }.refcount().add();
            Self {
                ptr: Some(ptr),
                _marker: PhantomData,
            }
        } else {
            Self::default()
        }
    }
}

impl<T: RefCounted> Drop for RefPtr<T> {
    fn drop(&mut self) {
        if let Some(ptr) = self.ptr.take() {
            Self::release_ptr(ptr);
        }
    }
}

impl<T: RefCounted> Deref for RefPtr<T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        self.as_ref().expect("dereferencing a null RefPtr")
    }
}

impl<T: RefCounted> DerefMut for RefPtr<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        self.as_mut().expect("dereferencing a null RefPtr")
    }
}

impl<T: RefCounted> PartialEq for RefPtr<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.as_ptr() == other.as_ptr()
    }
}

impl<T: RefCounted> Eq for RefPtr<T> {}

impl<T: RefCounted> fmt::Debug for RefPtr<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RefPtr")
            .field("ptr", &self.as_ptr())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

    /// Test object that records how many times it was destroyed.
    struct Tracked {
        rc: RefCount,
        drops: Arc<AtomicUsize>,
        value: i32,
    }

    impl Tracked {
        fn new(drops: &Arc<AtomicUsize>, value: i32) -> Self {
            Self {
                rc: RefCount::new(),
                drops: Arc::clone(drops),
                value,
            }
        }
    }

    impl Drop for Tracked {
        fn drop(&mut self) {
            self.drops.fetch_add(1, AtomicOrdering::SeqCst);
        }
    }

    // SAFETY: `refcount` returns the embedded counter and the default `destroy`
    // (free a `Box`) matches `RefPtr::new`.
    unsafe impl RefCounted for Tracked {
        fn refcount(&self) -> &RefCount {
            &self.rc
        }
    }

    #[test]
    fn new_starts_with_one_reference() {
        let drops = Arc::new(AtomicUsize::new(0));
        let p = RefPtr::new(Tracked::new(&drops, 5));
        assert_eq!(p.ref_count(), 1);
        assert_eq!(p.value, 5);
        drop(p);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn clone_and_drop_track_the_count() {
        let drops = Arc::new(AtomicUsize::new(0));
        let a = RefPtr::new(Tracked::new(&drops, 1));
        assert_eq!(a.ref_count(), 1);

        let b = a.clone();
        assert_eq!(a.ref_count(), 2);
        assert_eq!(b.ref_count(), 2);

        drop(b);
        assert_eq!(a.ref_count(), 1);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 0);

        drop(a);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn default_is_null() {
        let p: RefPtr<Tracked> = RefPtr::default();
        assert!(p.is_null());
        assert_eq!(p.as_ptr(), core::ptr::null_mut());
        assert_eq!(p.ref_count(), 0);
        assert!(p.as_ref().is_none());
    }

    #[test]
    fn release_transfers_the_reference() {
        let drops = Arc::new(AtomicUsize::new(0));
        let mut p = RefPtr::new(Tracked::new(&drops, 9));
        let raw = p.release();
        assert!(p.is_null());
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 0);
        assert_eq!(unsafe { (*raw).refcount().get() }, 1);

        let adopted = unsafe { RefPtr::from_raw_owned(raw) };
        assert_eq!(adopted.value, 9);
        drop(adopted);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn from_raw_shared_adds_a_reference() {
        let drops = Arc::new(AtomicUsize::new(0));
        let a = RefPtr::new(Tracked::new(&drops, 3));
        let raw = a.as_ptr();

        let b = unsafe { RefPtr::from_raw_shared(raw) };
        assert_eq!(a.ref_count(), 2);
        drop(b);
        assert_eq!(a.ref_count(), 1);
        drop(a);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn unset_and_dont_decrement_keeps_the_object_alive() {
        let drops = Arc::new(AtomicUsize::new(0));
        let mut a = RefPtr::new(Tracked::new(&drops, 4));
        let raw = a.as_ptr();
        a.unset_and_dont_decrement();
        assert!(a.is_null());
        assert_eq!(unsafe { (*raw).refcount().get() }, 1);

        // Clean up the reference we kept alive by hand.
        let owner = unsafe { RefPtr::from_raw_owned(raw) };
        drop(owner);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn set_and_dont_increment_moves_ownership() {
        let drops = Arc::new(AtomicUsize::new(0));
        let mut a = RefPtr::new(Tracked::new(&drops, 10));
        let mut b: RefPtr<Tracked> = RefPtr::default();

        let raw = a.release();
        unsafe { b.set_and_dont_increment_ref_count(raw) };
        assert_eq!(b.value, 10);
        assert_eq!(b.ref_count(), 1);
        drop(b);
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 1);
    }

    #[test]
    fn swap_does_not_touch_counts() {
        let drops = Arc::new(AtomicUsize::new(0));
        let mut a = RefPtr::new(Tracked::new(&drops, 1));
        let mut b = RefPtr::new(Tracked::new(&drops, 2));
        a.swap(&mut b);
        assert_eq!(a.value, 2);
        assert_eq!(b.value, 1);
        assert_eq!(a.ref_count(), 1);
        assert_eq!(b.ref_count(), 1);
    }

    #[test]
    fn clone_of_empty_pointer_is_empty() {
        let empty: RefPtr<Tracked> = RefPtr::default();
        let cloned = empty.clone();
        assert!(cloned.is_null());
    }
}
