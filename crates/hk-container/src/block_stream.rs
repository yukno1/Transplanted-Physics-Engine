//! Block stream — append-only chunked storage.
//!
//! Port of the *storage* part of `hkBlockStream` / `hkTypedBlockStream`
//! (`Common/Base/Container/BlockStream/`). The C++ class is a linked list of
//! fixed-size blocks (`BLOCK_DATA_SIZE = 4000` bytes of payload, 64-byte aligned)
//! shared between a writer, readers and consumers; ranges and iterators are
//! separate types.
//!
//! This phase provides the single-threaded, typed subset needed by the narrow
//! phase and the solver:
//!
//! * append elements into blocks of a fixed element capacity,
//! * iterate the stream element-wise or block-wise (deterministic order),
//! * random access by element index,
//! * clear/reuse blocks (the C++ `clear` + `fixupConsumedBlocks` idea),
//! * byte accounting matching the C++ constants.
//!
//! Not ported yet: concurrent reader/consumer hand-off (the C++ block atomic
//! `numElementsAndBytesUsed`), `Range`, `LinkedRange`, the raw byte-level
//! `allocate`/`write` APIs, and the thread-local block allocator. Those belong
//! with the multithreaded simulation work (Phase 10).
//!
//! ```
//! use hk_container::BlockStream;
//! let mut stream = BlockStream::with_block_capacity(4);
//! let index = stream.push(7);
//! assert_eq!(index, 0);
//! assert_eq!(stream.get(0), Some(&7));
//! assert_eq!(stream.block_count(), 1);
//! ```

/// Payload bytes per block (`hkBlockStream::Block::BLOCK_DATA_SIZE`).
pub const BLOCK_DATA_SIZE: usize = 4000;
/// Header bytes per block (`hkBlockStream::Block::BLOCK_HEADER_SIZE`).
pub const BLOCK_HEADER_SIZE: usize = 32;
/// Total block size (`hkBlockStream::Block::BLOCK_TOTAL_SIZE` = 4096 - 64).
pub const BLOCK_TOTAL_SIZE: usize = BLOCK_DATA_SIZE + BLOCK_HEADER_SIZE;

/// Element capacity of a default block for type `T`.
#[inline]
pub fn default_block_capacity<T>() -> usize {
    match BLOCK_DATA_SIZE.checked_div(core::mem::size_of::<T>()) {
        Some(0) | None => 1,
        Some(count) => count,
    }
}

/// Append-only stream of `T` stored in fixed-capacity blocks.
///
/// The stream is a sequence of blocks; every block except the last is full. This
/// keeps the element-to-block mapping O(1) and makes block-wise iteration cheap.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockStream<T> {
    blocks: Vec<Vec<T>>,
    block_capacity: usize,
    len: usize,
}

impl<T> Default for BlockStream<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> BlockStream<T> {
    /// Stream with the SDK's default block size for `T`.
    #[inline]
    pub fn new() -> Self {
        Self::with_block_capacity(default_block_capacity::<T>())
    }

    /// Stream with a custom number of elements per block.
    ///
    /// # Panics
    ///
    /// Panics if `elements_per_block` is zero.
    #[inline]
    pub fn with_block_capacity(elements_per_block: usize) -> Self {
        assert!(elements_per_block > 0, "block capacity must be non-zero");
        Self {
            blocks: Vec::new(),
            block_capacity: elements_per_block,
            len: 0,
        }
    }

    /// Append an element, returning its stream index.
    ///
    /// The index is the element's position in stream order (equal to the previous
    /// length); [`BlockStream::block_and_offset`] maps it to the block coordinates.
    pub fn push(&mut self, value: T) -> usize {
        let index = self.len;
        let block_index = index / self.block_capacity;
        if block_index == self.blocks.len() {
            self.blocks.push(Vec::with_capacity(self.block_capacity));
        }
        self.blocks[block_index].push(value);
        self.len += 1;
        index
    }

    /// Append every element of an iterator.
    pub fn extend<I: IntoIterator<Item = T>>(&mut self, values: I) {
        for value in values {
            self.push(value);
        }
    }

    /// Append clones of a slice.
    pub fn extend_from_slice(&mut self, values: &[T])
    where
        T: Clone,
    {
        for value in values {
            self.push(value.clone());
        }
    }

    /// Number of elements (`hkBlockStream::getTotalNumElems`).
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the stream holds no elements (`hkBlockStream::isEmpty`).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Elements per block.
    #[inline]
    pub fn block_capacity(&self) -> usize {
        self.block_capacity
    }

    /// Number of allocated blocks.
    #[inline]
    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }

    /// Number of elements in block `block_index` (the last block may be partial).
    #[inline]
    pub fn block_len(&self, block_index: usize) -> Option<usize> {
        self.blocks.get(block_index).map(Vec::len)
    }

    /// Map a stream index to `(block, offset)`.
    #[inline]
    pub fn block_and_offset(&self, index: usize) -> Option<(usize, usize)> {
        if index >= self.len {
            return None;
        }
        Some((index / self.block_capacity, index % self.block_capacity))
    }

    /// Random access by stream index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&T> {
        let (block, offset) = self.block_and_offset(index)?;
        self.blocks[block].get(offset)
    }

    /// Mutable random access by stream index.
    #[inline]
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        let (block, offset) = self.block_and_offset(index)?;
        self.blocks[block].get_mut(offset)
    }

    /// Element-wise iterator in stream order.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.blocks.iter().flat_map(|block| block.iter())
    }

    /// Mutable element-wise iterator in stream order.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.blocks.iter_mut().flat_map(|block| block.iter_mut())
    }

    /// Block-wise iterator (each item is one block's slice).
    #[inline]
    pub fn blocks(&self) -> impl Iterator<Item = &[T]> {
        self.blocks.iter().map(Vec::as_slice)
    }

    /// Mutable block-wise iterator.
    #[inline]
    pub fn blocks_mut(&mut self) -> impl Iterator<Item = &mut [T]> {
        self.blocks.iter_mut().map(Vec::as_mut_slice)
    }

    /// Clear the elements but keep the allocated blocks for reuse
    /// (`hkBlockStream::clear` + block reuse).
    pub fn clear(&mut self) {
        for block in &mut self.blocks {
            block.clear();
        }
        self.len = 0;
    }

    /// Drop all blocks (`hkBlockStream::reset` without reallocating one block).
    pub fn reset(&mut self) {
        self.blocks.clear();
        self.len = 0;
    }

    /// Total bytes of the allocated blocks including headers
    /// (`hkBlockStream::getTotalBytesAllocated`).
    #[inline]
    pub fn total_bytes_allocated(&self) -> usize {
        self.blocks.len() * BLOCK_TOTAL_SIZE
    }

    /// Bytes actually occupied by elements (`calcTotalBytesUsed`).
    #[inline]
    pub fn total_bytes_used(&self) -> usize {
        self.len * core::mem::size_of::<T>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pushes_and_indexes_across_blocks() {
        let mut stream = BlockStream::with_block_capacity(4);
        for value in 0..9 {
            assert_eq!(stream.push(value), value as usize);
        }
        assert_eq!(stream.len(), 9);
        assert_eq!(stream.block_count(), 3);
        assert_eq!(stream.block_len(0), Some(4));
        assert_eq!(stream.block_len(1), Some(4));
        assert_eq!(stream.block_len(2), Some(1));
        assert_eq!(stream.block_len(3), None);

        assert_eq!(
            stream.iter().copied().collect::<Vec<_>>(),
            (0..9).collect::<Vec<_>>()
        );
        assert_eq!(stream.get(0), Some(&0));
        assert_eq!(stream.get(8), Some(&8));
        assert_eq!(stream.get(9), None);
        assert_eq!(stream.block_and_offset(5), Some((1, 1)));
        assert_eq!(stream.block_and_offset(9), None);
    }

    #[test]
    fn default_block_capacity_matches_sdk_data_size() {
        // u32: 4000/4 = 1000 elements per block.
        assert_eq!(default_block_capacity::<u32>(), 1000);
        // Zero-sized types still get a usable capacity.
        assert_eq!(default_block_capacity::<()>(), 1);
        assert_eq!(BLOCK_TOTAL_SIZE, 4032);
    }

    #[test]
    fn clear_keeps_blocks_and_reset_drops_them() {
        let mut stream = BlockStream::with_block_capacity(4);
        stream.extend(0..9);
        assert_eq!(stream.block_count(), 3);

        stream.clear();
        assert_eq!(stream.len(), 0);
        assert_eq!(stream.block_count(), 3, "clear keeps the block allocations");
        assert_eq!(stream.total_bytes_allocated(), 3 * BLOCK_TOTAL_SIZE);
        assert_eq!(stream.total_bytes_used(), 0);

        stream.push(42);
        assert_eq!(stream.get(0), Some(&42));
        assert_eq!(stream.block_count(), 3);

        stream.reset();
        assert_eq!(stream.block_count(), 0);
        assert_eq!(stream.len(), 0);
    }

    #[test]
    fn block_wise_iteration() {
        let mut stream = BlockStream::with_block_capacity(3);
        stream.extend(0..8);
        let blocks: Vec<Vec<i32>> = stream.blocks().map(|block| block.to_vec()).collect();
        assert_eq!(blocks, vec![vec![0, 1, 2], vec![3, 4, 5], vec![6, 7]]);
    }

    #[test]
    fn mutable_access() {
        let mut stream = BlockStream::with_block_capacity(2);
        stream.extend(0..4);
        *stream.get_mut(3).unwrap() = 40;
        for value in stream.iter_mut() {
            *value += 1;
        }
        assert_eq!(
            stream.iter().copied().collect::<Vec<_>>(),
            vec![1, 2, 3, 41]
        );
    }

    #[test]
    fn byte_accounting() {
        let mut stream = BlockStream::<u32>::new();
        stream.extend(0..3);
        assert_eq!(stream.block_count(), 1);
        assert_eq!(stream.total_bytes_allocated(), BLOCK_TOTAL_SIZE);
        assert_eq!(stream.total_bytes_used(), 3 * core::mem::size_of::<u32>());
    }
}
