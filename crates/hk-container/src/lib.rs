//! `hk-container` — Havok containers.
//!
//! Rust port of the container layer under
//! `original/Common/Source/Common/Base/Container/` (migration Phase 3, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Array`] | `hkArray` | `Vec`-backed; `removeAt` is unordered, `removeAtAndCopy` is ordered |
//! | [`BlockStream`] | `hkBlockStream` / `hkTypedBlockStream` | append-only fixed-size blocks, deterministic iteration; single-threaded subset |
//! | [`HashMap`] | `hkHashMap` | `std::collections::HashMap` with a deterministic FNV-1a hasher |
//! | [`HashSet`] | `hkHashSet` | `std::collections::HashSet` with the same hasher |
//! | [`HkHasher`] | `hkHash` (`hkHashBase`) | fixed-seed deterministic hashing |
//!
//! # Zero external dependencies
//!
//! The original Phase-3 plan mentioned `hashbrown`; the port stays on
//! `std::collections` for now to keep the workspace dependency-free. The hasher
//! is deterministic (not `RandomState`), which matters for reproducible runs; see
//! [`hash`] for the caveat that iteration order is deterministic but not identical
//! to the C++ table order.
//!
//! # Not ported in this phase
//!
//! `hkLocalArray` / `hkInplaceArray` (stack-backed arrays), `hkArrayView`,
//! `hkString*`, `hkPointerMap`, `hkRelArray`, `hkTree`, `hkQueue`/`hkDeque`,
//! bitfields and the concurrent containers. Block-stream readers/consumers/ranges
//! (`hkBlockStreamIterators`, `hkBlockStreamRange`) are also deferred to the
//! multithreaded work.
//!
//! ```
//! use hk_container::{Array, HashMap, BlockStream};
//!
//! let mut array = Array::new();
//! array.push_back(1);
//! array.push_back(2);
//! assert_eq!(array.len(), 2);
//!
//! let mut map = HashMap::new();
//! map.insert("a", 1);
//! assert_eq!(map.at(&"a"), &1);
//!
//! let mut stream = BlockStream::with_block_capacity(4);
//! stream.push(7);
//! assert_eq!(stream.get(0), Some(&7));
//! ```

pub mod array;
pub mod block_stream;
pub mod hash;
pub mod hash_map;
pub mod hash_set;

pub use array::Array;
pub use block_stream::{
    BLOCK_DATA_SIZE, BLOCK_HEADER_SIZE, BLOCK_TOTAL_SIZE, BlockStream, default_block_capacity,
};
pub use hash::{BuildHkHasher, HkHasher};
pub use hash_map::{HashMap, TryInsertResult};
pub use hash_set::HashSet;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn containers_compose() {
        let mut array: Array<u32> = Array::new();
        for value in 0..10 {
            array.push_back(value);
        }

        let mut map = HashMap::new();
        for value in array.iter() {
            map.insert(*value, value * value);
        }

        let mut stream = BlockStream::with_block_capacity(3);
        for value in array.iter() {
            stream.push(*value);
        }

        assert_eq!(array.len(), 10);
        assert_eq!(map.len(), 10);
        assert_eq!(stream.len(), 10);
    }
}
