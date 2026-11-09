//! Allocator interface and statistics.
//!
//! Port of the core of `hkMemory::Allocator` from
//! `original/Common/Source/Common/Base/Memory/hkMemoryAllocator.h`.
//!
//! The C++ allocator API is `allocate(hkUlong& reqInOut)` / `deallocate(void*, hkUlong)`
//! — a size in, size out, and no alignment parameter (the SDK allocators are
//! 16-byte aligned by construction). Rust deallocation needs the exact [`Layout`],
//! so this port takes a `Layout` on the way in and out. Everything else (fail-fast
//! on OOM, statistics counters, peak tracking) follows the original.

use core::alloc::Layout;
use core::ptr::NonNull;

/// Allocation statistics, mirroring `hkMemory::Allocator::SystemStats`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AllocStats {
    /// Currently live allocations (one per successful `alloc`).
    pub num_allocations: usize,
    /// Total number of allocations since the allocator was created.
    pub num_total_allocations: u64,
    /// Bytes currently in use.
    pub bytes_in_use: usize,
    /// High-water mark of `bytes_in_use`.
    pub peak_bytes_in_use: usize,
}

/// Heap allocator abstraction (`hkMemory::Allocator`).
///
/// Implementations must fail fast (abort or panic) when the underlying allocator
/// returns null; the SDK does the same and containers are written assuming
/// allocation never returns "null".
pub trait Allocator {
    /// Allocate `layout.size()` bytes with `layout.align()` alignment.
    ///
    /// The result is non-null even for zero-sized layouts (a dangling, correctly
    /// aligned pointer is returned and must be passed back to [`Allocator::dealloc`]).
    fn alloc(&self, layout: Layout) -> NonNull<u8>;

    /// Allocate zero-initialized memory.
    fn alloc_zeroed(&self, layout: Layout) -> NonNull<u8> {
        let pointer = self.alloc(layout);
        if layout.size() != 0 {
            // SAFETY: `alloc` returned at least `layout.size()` writable bytes.
            unsafe {
                core::ptr::write_bytes(pointer.as_ptr(), 0, layout.size());
            }
        }
        pointer
    }

    /// Free memory previously returned by [`Allocator::alloc`] with the same layout.
    fn dealloc(&self, pointer: NonNull<u8>, layout: Layout);

    /// Reallocate to `new_layout`, preserving the first `min(old, new)` bytes.
    ///
    /// The default implementation allocates a new block, copies the old contents
    /// and frees the old block. Allocators that can grow in place should override
    /// it (the system allocator does).
    ///
    /// # Panics
    ///
    /// Panics if `new_layout.align()` differs from `old_layout.align()`; use
    /// `alloc`/`dealloc` for changing alignment.
    fn realloc(&self, pointer: NonNull<u8>, old_layout: Layout, new_layout: Layout) -> NonNull<u8> {
        assert_eq!(
            old_layout.align(),
            new_layout.align(),
            "realloc cannot change alignment"
        );
        let new_pointer = self.alloc(new_layout);
        let copy_size = old_layout.size().min(new_layout.size());
        if copy_size != 0 {
            // SAFETY: both buffers are live and copy_size <= both sizes.
            unsafe {
                core::ptr::copy_nonoverlapping(pointer.as_ptr(), new_pointer.as_ptr(), copy_size);
            }
        }
        self.dealloc(pointer, old_layout);
        new_pointer
    }
}

/// Byte size of an allocation rounded up to a multiple of `alignment`.
///
/// The C++ API's `hkUlong& reqInOut` lets the allocator report the rounded size;
/// helpers use this to compute the final layout themselves.
#[inline]
pub const fn align_up(size: usize, alignment: usize) -> usize {
    debug_assert!(alignment.is_power_of_two());
    (size + alignment - 1) & !(alignment - 1)
}

/// Layout for an array of `count` values of type `T`.
#[inline]
pub fn layout_for<T>(count: usize) -> Layout {
    // SAFETY: Layout::from_size_align only fails for overflowing sizes; this
    // reports the error through the standard `expect` path.
    Layout::from_size_align(
        core::mem::size_of::<T>()
            .checked_mul(count)
            .expect("allocation size overflow"),
        core::mem::align_of::<T>(),
    )
    .expect("allocation layout overflow")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn align_up_rounds_to_power_of_two() {
        assert_eq!(align_up(0, 8), 0);
        assert_eq!(align_up(1, 8), 8);
        assert_eq!(align_up(8, 8), 8);
        assert_eq!(align_up(9, 8), 16);
        assert_eq!(align_up(17, 16), 32);
    }

    #[test]
    fn layout_for_u32() {
        let layout = layout_for::<u32>(3);
        assert_eq!(layout.size(), 12);
        assert_eq!(layout.align(), 4);
        assert_eq!(layout.pad_to_align().size(), 12);
    }
}
