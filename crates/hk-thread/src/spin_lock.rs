//! Spin lock.
//!
//! Port of the spin-lock primitives under
//! `original/Common/Source/Common/Base/Thread/Atomic/` (`hkSpinLockSimple`) and
//! the critical-section contract used by the SDK's job system.
//!
//! Unlike [`std::sync::Mutex`], a spin lock busy-waits instead of parking the
//! thread; it is meant for very short critical sections inside the job system.
//! There is no fairness guarantee (a spinning thread can starve a waiter), which
//! matches the C++ implementation.
//!
//! ```
//! use hk_thread::SpinLock;
//! let lock = SpinLock::new(0u32);
//! *lock.lock() += 1;
//! assert_eq!(*lock.lock(), 1);
//! ```

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

/// Spin lock guarding a value.
#[repr(C)]
pub struct SpinLock<T> {
    locked: AtomicBool,
    value: UnsafeCell<T>,
}

// SAFETY: access to `value` is only handed out through a guard obtained by
// winning the atomic lock; `T: Send` means moving it between threads is sound.
unsafe impl<T: Send> Sync for SpinLock<T> {}

impl<T> SpinLock<T> {
    /// Create an unlocked spin lock.
    #[inline]
    pub const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }

    /// Acquire the lock, spinning until it becomes available.
    #[inline]
    pub fn lock(&self) -> SpinGuard<'_, T> {
        loop {
            if self
                .locked
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
            {
                return SpinGuard { lock: self };
            }
            while self.locked.load(Ordering::Relaxed) {
                spin_loop();
            }
        }
    }

    /// Try to acquire the lock without spinning.
    #[inline]
    pub fn try_lock(&self) -> Option<SpinGuard<'_, T>> {
        if self
            .locked
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
        {
            Some(SpinGuard { lock: self })
        } else {
            None
        }
    }

    /// Direct mutable access through `&mut self` (no locking required).
    #[inline]
    pub fn get_mut(&mut self) -> &mut T {
        // SAFETY: `&mut self` guarantees exclusive access.
        unsafe { &mut *self.value.get() }
    }

    /// Consume the lock and return the value.
    #[inline]
    pub fn into_inner(self) -> T {
        self.value.into_inner()
    }
}

impl<T: Default> Default for SpinLock<T> {
    #[inline]
    fn default() -> Self {
        Self::new(T::default())
    }
}

impl<T: core::fmt::Debug> core::fmt::Debug for SpinLock<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SpinLock")
            .field("value", &*self.lock())
            .finish()
    }
}

/// RAII guard for a locked [`SpinLock`].
pub struct SpinGuard<'a, T> {
    lock: &'a SpinLock<T>,
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        // SAFETY: the guard holds the lock exclusively.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: the guard holds the lock exclusively (and `&mut self`).
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T: core::fmt::Debug> core::fmt::Debug for SpinGuard<'_, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(&**self, f)
    }
}

impl<T> Drop for SpinGuard<'_, T> {
    #[inline]
    fn drop(&mut self) {
        self.lock.locked.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn lock_and_unlock() {
        let lock = SpinLock::new(0u32);
        {
            let mut guard = lock.lock();
            *guard += 1;
        }
        assert_eq!(*lock.lock(), 1);
    }

    #[test]
    fn try_lock_fails_while_held() {
        let lock = SpinLock::new(1u32);
        let guard = lock.lock();
        assert!(lock.try_lock().is_none());
        drop(guard);
        assert!(lock.try_lock().is_some());
    }

    #[test]
    fn get_mut_and_into_inner() {
        let mut lock = SpinLock::new(vec![1, 2, 3]);
        lock.get_mut().push(4);
        assert_eq!(lock.into_inner(), vec![1, 2, 3, 4]);
    }

    #[test]
    fn concurrent_increments_without_loss() {
        const THREADS: usize = 4;
        const PER_THREAD: usize = 25_000;

        let lock = Arc::new(SpinLock::new(0usize));
        let mut handles = Vec::new();
        for _ in 0..THREADS {
            let lock = Arc::clone(&lock);
            handles.push(std::thread::spawn(move || {
                for _ in 0..PER_THREAD {
                    *lock.lock() += 1;
                }
            }));
        }
        for handle in handles {
            handle.join().expect("worker thread panicked");
        }
        assert_eq!(*lock.lock(), THREADS * PER_THREAD);
    }
}
