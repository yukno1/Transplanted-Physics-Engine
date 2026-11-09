//! `hk-memory` — Havok memory system.
//!
//! Rust port of the allocator layer under
//! `original/Common/Source/Common/Base/Memory/` (migration Phase 3, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Allocator`] | `hkMemory::Allocator` | `Layout`-based, fail-fast allocation, statistics |
//! | [`AllocStats`] | `hkMemory::Allocator::SystemStats` | live/total allocations, in-use/peak bytes |
//! | [`SystemAllocator`] | `hkMemory::MallocAllocator` | forwards to the Rust global allocator, atomic counters |
//! | [`global`] | `MallocAllocator::s_instance` | process-wide instance |
//!
//! # Deliberately not ported in this phase
//!
//! The C++ memory system is much larger than the allocator interface: it contains
//! the memory router (`hkMemoryRouter`), debug tracking/snapshots
//! (`hkMemoryDebug`, `hkMemorySnapshot`, `hkMemoryTracker`), the free-list, LIFO
//! and thread-local allocators (`hkFreeListAllocator`, `hkLifoAllocator`,
//! `hkThreadMemoryAllocator`), and the deprecated compatibility layer.
//!
//! Per the migration strategy (README §4 Phase 3) the Rust port starts with
//! ordinary `Vec`-backed containers plus one heap allocator; specialised allocators
//! are introduced only when profiling shows they matter. The block-stream
//! *container* lives in `hk-container` (`BlockStream`).
//!
//! # Stats and determinism
//!
//! Counters are relaxed/acquire-release atomics like the SDK; a stats snapshot
//! may be slightly stale under concurrency (the C++ code has the same caveat).
//! Allocation order and addresses are obviously not deterministic, but the
//! *numerical* results of the simulation must not depend on them — that is the
//! ID/index design principle from the README.

pub mod allocator;
pub mod system;

pub use allocator::{AllocStats, Allocator, align_up, layout_for};
pub use system::{SystemAllocator, global};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allocating_through_trait_object() {
        let allocator = SystemAllocator::new();
        let object: &dyn Allocator = &allocator;
        let layout = layout_for::<f32>(4);
        let block = object.alloc(layout);
        object.dealloc(block, layout);
        assert_eq!(allocator.stats().num_allocations, 0);
        assert_eq!(allocator.stats().num_total_allocations, 1);
    }
}
