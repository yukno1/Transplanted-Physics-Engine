//! Deterministic hashing for the port's hash containers.
//!
//! The SDK's `hkHashBase` uses its own hash function (`hkHashValue`, mixing a
//! 32-bit hash) and stores items in an array, so iteration order is roughly
//! insertion order with swap-removal. Rust's `std::collections::HashMap` uses
//! SipHash with a per-process random seed by default — unacceptable here, because
//! the simulation must be reproducible across runs.
//!
//! This module therefore provides a fixed-seed FNV-1a hasher used by
//! [`crate::HashMap`] and [`crate::HashSet`]. Iteration order is deterministic for
//! a given set of keys and insertion/removal sequence, but it is **not** the same
//! order the C++ hash table would produce. Code that depends on a particular
//! iteration order should use [`crate::Array`] instead.

use core::hash::{BuildHasherDefault, Hasher};

const FNV_OFFSET_BASIS: u64 = 0xCBF2_9CE4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01B3;

/// Fixed-seed FNV-1a 64-bit hasher.
#[derive(Clone, Copy, Debug)]
pub struct HkHasher(u64);

impl Default for HkHasher {
    #[inline]
    fn default() -> Self {
        Self(FNV_OFFSET_BASIS)
    }
}

impl Hasher for HkHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(FNV_PRIME);
        }
    }
}

/// `BuildHasher` for [`HkHasher`] (no random seed).
pub type BuildHkHasher = BuildHasherDefault<HkHasher>;

/// Underlying standard hash map with the deterministic hasher.
pub(crate) type RawHashMap<K, V> = std::collections::HashMap<K, V, BuildHkHasher>;

/// Underlying standard hash set with the deterministic hasher.
pub(crate) type RawHashSet<T> = std::collections::HashSet<T, BuildHkHasher>;

#[cfg(test)]
mod tests {
    use super::*;
    use core::hash::BuildHasher;

    #[test]
    fn hashing_is_deterministic_and_seed_independent() {
        let build = BuildHkHasher::default();
        let a = build.hash_one(42u32);
        let b = BuildHkHasher::default().hash_one(42u32);
        assert_eq!(a, b, "two builders must produce the same hash");

        // Different values should (very likely) hash differently.
        assert_ne!(build.hash_one(1u32), build.hash_one(2u32));
    }
}
