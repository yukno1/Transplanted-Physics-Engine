//! System heap allocator with statistics.
//!
//! Port of `hkMemory::MallocAllocator`
//! (`Common/Base/Memory/Allocators/hkMallocAllocator.{h,cpp}`), including its
//! atomic counters (`m_numAllocations`, `m_numTotalAllocations`,
//! `m_numAllocatedBytes`, `m_numPeakInUseBytes`).
//!
//! The C++ allocator forwards to `malloc`/`free`; the Rust equivalent forwards to
//! `std::alloc::{alloc, alloc_zeroed, dealloc, realloc}` so layouts (alignment)
//! are preserved.

use core::alloc::Layout;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use crate::allocator::{AllocStats, Allocator};

/// Heap allocator backed by the process global allocator.
///
/// Counters are per-instance, so tests can create private instances
/// (`SystemAllocator::new()`) and observe stable statistics even when tests run
/// in parallel. [`global`] exposes the shared process-wide instance.
#[derive(Debug, Default)]
pub struct SystemAllocator {
    live_allocations: AtomicUsize,
    total_allocations: AtomicU64,
    bytes_in_use: AtomicUsize,
    peak_bytes_in_use: AtomicUsize,
}

impl SystemAllocator {
    /// Create a fresh, zero-initialized allocator (statistics start at zero).
    #[inline]
    pub const fn new() -> Self {
        Self {
            live_allocations: AtomicUsize::new(0),
            total_allocations: AtomicU64::new(0),
            bytes_in_use: AtomicUsize::new(0),
            peak_bytes_in_use: AtomicUsize::new(0),
        }
    }

    /// Snapshot of the allocation counters (`getSystemStats`).
    #[inline]
    pub fn stats(&self) -> AllocStats {
        let in_use = self.bytes_in_use.load(Ordering::Acquire);
        let mut peak = self.peak_bytes_in_use.load(Ordering::Relaxed);
        if in_use > peak {
            // Another thread may be updating the peak; keep the snapshot coherent.
            peak = in_use;
        }
        AllocStats {
            num_allocations: self.live_allocations.load(Ordering::Acquire),
            num_total_allocations: self.total_allocations.load(Ordering::Relaxed),
            bytes_in_use: in_use,
            peak_bytes_in_use: peak,
        }
    }

    /// Reset the peak watermark to the current in-use value (`resetPeakInUse`).
    #[inline]
    pub fn reset_peak(&self) {
        self.peak_bytes_in_use
            .store(self.bytes_in_use.load(Ordering::Relaxed), Ordering::Relaxed);
    }

    #[inline]
    fn register_allocation(&self, size: usize) {
        if size == 0 {
            return;
        }
        self.live_allocations.fetch_add(1, Ordering::Relaxed);
        self.total_allocations.fetch_add(1, Ordering::Relaxed);
        let in_use = self.bytes_in_use.fetch_add(size, Ordering::AcqRel) + size;
        self.peak_bytes_in_use.fetch_max(in_use, Ordering::Relaxed);
    }

    #[inline]
    fn register_deallocation(&self, size: usize) {
        if size == 0 {
            return;
        }
        self.live_allocations.fetch_sub(1, Ordering::Relaxed);
        self.bytes_in_use.fetch_sub(size, Ordering::AcqRel);
    }
}

/// The process-wide system allocator (`MallocAllocator::s_instance`).
static GLOBAL: SystemAllocator = SystemAllocator::new();

/// Shared system allocator instance.
#[inline]
pub fn global() -> &'static SystemAllocator {
    &GLOBAL
}

impl Allocator for SystemAllocator {
    fn alloc(&self, layout: Layout) -> NonNull<u8> {
        if layout.size() == 0 {
            // Rust allocations must be non-null; zero-sized requests get an
            // aligned dangling pointer (dealloc is a no-op for them).
            // SAFETY: the alignment value is non-zero and correctly aligned.
            return unsafe { NonNull::new_unchecked(layout.align() as *mut u8) };
        }

        // SAFETY: `Layout` guarantees a non-zero size and valid alignment.
        let pointer = unsafe { std::alloc::alloc(layout) };
        let pointer = match NonNull::new(pointer) {
            Some(pointer) => pointer,
            None => std::alloc::handle_alloc_error(layout),
        };
        self.register_allocation(layout.size());
        pointer
    }

    fn alloc_zeroed(&self, layout: Layout) -> NonNull<u8> {
        if layout.size() == 0 {
            // SAFETY: see `alloc`.
            return unsafe { NonNull::new_unchecked(layout.align() as *mut u8) };
        }

        // SAFETY: `Layout` guarantees a non-zero size and valid alignment.
        let pointer = unsafe { std::alloc::alloc_zeroed(layout) };
        let pointer = match NonNull::new(pointer) {
            Some(pointer) => pointer,
            None => std::alloc::handle_alloc_error(layout),
        };
        self.register_allocation(layout.size());
        pointer
    }

    fn dealloc(&self, pointer: NonNull<u8>, layout: Layout) {
        if layout.size() == 0 {
            return;
        }
        // SAFETY: per the trait contract, `pointer` came from this allocator with
        // the same layout and has not been freed.
        unsafe { std::alloc::dealloc(pointer.as_ptr(), layout) };
        self.register_deallocation(layout.size());
    }

    fn realloc(&self, pointer: NonNull<u8>, old_layout: Layout, new_layout: Layout) -> NonNull<u8> {
        assert_eq!(
            old_layout.align(),
            new_layout.align(),
            "realloc cannot change alignment"
        );

        if old_layout.size() == 0 {
            return self.alloc(new_layout);
        }
        if new_layout.size() == 0 {
            self.dealloc(pointer, old_layout);
            // SAFETY: zero-sized layouts return an aligned dangling pointer.
            return unsafe { NonNull::new_unchecked(new_layout.align() as *mut u8) };
        }

        // SAFETY: `pointer` is live with `old_layout`; `new_size` is derived from
        // `new_layout` and the alignment is unchanged.
        let new_pointer =
            unsafe { std::alloc::realloc(pointer.as_ptr(), old_layout, new_layout.size()) };
        let new_pointer = match NonNull::new(new_pointer) {
            Some(pointer) => pointer,
            None => std::alloc::handle_alloc_error(new_layout),
        };

        let old_size = old_layout.size();
        let new_size = new_layout.size();
        if new_size >= old_size {
            let growth = new_size - old_size;
            self.bytes_in_use.fetch_add(growth, Ordering::AcqRel);
            let in_use = self.bytes_in_use.load(Ordering::Relaxed);
            self.peak_bytes_in_use.fetch_max(in_use, Ordering::Relaxed);
        } else {
            self.bytes_in_use
                .fetch_sub(old_size - new_size, Ordering::AcqRel);
        }
        new_pointer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::allocator::layout_for;

    #[test]
    fn allocate_write_read_free() {
        let allocator = SystemAllocator::new();
        let layout = layout_for::<u64>(4);
        let block = allocator.alloc(layout);
        assert_eq!(block.as_ptr() as usize % layout.align(), 0, "alignment");

        // SAFETY: the allocation is live and covers 4 u64 values.
        unsafe {
            for i in 0..4u64 {
                core::ptr::write(block.as_ptr().cast::<u64>().add(i as usize), i + 10);
            }
            for i in 0..4u64 {
                assert_eq!(
                    core::ptr::read(block.as_ptr().cast::<u64>().add(i as usize)),
                    i + 10
                );
            }
        }

        assert_eq!(allocator.stats().num_allocations, 1);
        assert_eq!(allocator.stats().bytes_in_use, layout.size());

        allocator.dealloc(block, layout);
        let stats = allocator.stats();
        assert_eq!(stats.num_allocations, 0);
        assert_eq!(stats.bytes_in_use, 0);
        assert_eq!(stats.num_total_allocations, 1);
    }

    #[test]
    fn zeroed_allocation_is_zero() {
        let allocator = SystemAllocator::new();
        let layout = layout_for::<u32>(8);
        let block = allocator.alloc_zeroed(layout);
        // SAFETY: live allocation of 8 u32 values.
        unsafe {
            let values = core::slice::from_raw_parts(block.as_ptr().cast::<u32>(), 8);
            assert!(values.iter().all(|value| *value == 0));
        }
        allocator.dealloc(block, layout);
    }

    #[test]
    fn realloc_preserves_contents_and_tracks_peak() {
        let allocator = SystemAllocator::new();
        let old_layout = layout_for::<u32>(2);
        let new_layout = layout_for::<u32>(8);

        let block = allocator.alloc(old_layout);
        // SAFETY: live allocation of 2 u32 values.
        unsafe {
            core::ptr::write(block.as_ptr().cast::<u32>(), 7);
            core::ptr::write(block.as_ptr().cast::<u32>().add(1), 9);
        }

        let grown = allocator.realloc(block, old_layout, new_layout);
        // SAFETY: grown allocation of 8 u32 values, first two preserved.
        unsafe {
            assert_eq!(core::ptr::read(grown.as_ptr().cast::<u32>()), 7);
            assert_eq!(core::ptr::read(grown.as_ptr().cast::<u32>().add(1)), 9);
        }

        let stats = allocator.stats();
        assert_eq!(stats.bytes_in_use, new_layout.size());
        assert!(stats.peak_bytes_in_use >= new_layout.size());
        assert_eq!(
            stats.num_total_allocations, 1,
            "realloc counted as the same allocation"
        );

        allocator.reset_peak();
        assert_eq!(allocator.stats().peak_bytes_in_use, new_layout.size());
        allocator.dealloc(grown, new_layout);
    }

    #[test]
    fn zero_sized_layouts_are_dangling() {
        let allocator = SystemAllocator::new();
        let layout = Layout::from_size_align(0, 8).unwrap();
        let block = allocator.alloc(layout);
        assert_eq!(block.as_ptr() as usize % 8, 0);
        assert_eq!(allocator.stats().num_allocations, 0);
        allocator.dealloc(block, layout);
    }

    #[test]
    fn global_allocator_is_usable() {
        let block = crate::global().alloc(layout_for::<u8>(16));
        crate::global().dealloc(block, layout_for::<u8>(16));
    }
}
