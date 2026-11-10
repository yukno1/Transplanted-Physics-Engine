//! Scoped parallel loops.
//!
//! Rust counterpart of the chunked `hkParallelFor`-style helpers used around
//! `hkJobQueue` (the SDK has no single canonical function; the job system is fed
//! with per-range jobs). These helpers run on `std::thread::scope`, so closures
//! may borrow local data without `'static` bounds — unlike a thread pool.
//!
//! Work is handed out dynamically with an atomic cursor; per-index results are
//! deterministic, but the *order* in which indices are processed is not (the same
//! caveat as the SDK's multithreaded scheduling). Use [`parallel_for_chunks`] when
//! you want deterministic contiguous partitions instead.
//!
//! ```
//! use hk_thread::parallel::parallel_for;
//! use std::sync::atomic::{AtomicUsize, Ordering};
//!
//! let sum = AtomicUsize::new(0);
//! parallel_for(100, |index| {
//!     sum.fetch_add(index, Ordering::Relaxed);
//! });
//! assert_eq!(sum.load(Ordering::Relaxed), 100 * 99 / 2);
//! ```

use core::sync::atomic::{AtomicUsize, Ordering};

/// Number of hardware threads reported by the OS (at least 1).
#[inline]
pub fn num_hardware_threads() -> usize {
    std::thread::available_parallelism()
        .map(std::num::NonZeroUsize::get)
        .unwrap_or(1)
}

/// Run `function(index)` for every `index` in `0..len` using the hardware thread
/// count.
#[inline]
pub fn parallel_for<F>(len: usize, function: F)
where
    F: Fn(usize) + Sync,
{
    parallel_for_threads(len, num_hardware_threads(), function);
}

/// Run `function(index)` for every `index` in `0..len` with at most `threads`
/// worker threads.
///
/// `threads` is clamped to `[1, len]`; an empty range does nothing.
pub fn parallel_for_threads<F>(len: usize, threads: usize, function: F)
where
    F: Fn(usize) + Sync,
{
    if len == 0 {
        return;
    }
    let threads = threads.max(1).min(len);
    let next = AtomicUsize::new(0);

    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                loop {
                    let index = next.fetch_add(1, Ordering::Relaxed);
                    if index >= len {
                        break;
                    }
                    function(index);
                }
            });
        }
    });
}

/// Run `function(start, end)` over deterministic contiguous chunks of `0..len`.
///
/// Each chunk is visited by exactly one worker; the partition does not depend on
/// thread timing, which makes it suitable for reductions whose order matters.
pub fn parallel_for_chunks<F>(len: usize, threads: usize, function: F)
where
    F: Fn(usize, usize) + Sync,
{
    if len == 0 {
        return;
    }
    let threads = threads.max(1).min(len);
    let chunk_size = len.div_ceil(threads);
    let function = &function;

    std::thread::scope(|scope| {
        for thread in 0..threads {
            let start = thread * chunk_size;
            let end = (start + chunk_size).min(len);
            if start < end {
                scope.spawn(move || function(start, end));
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn parallel_for_visits_every_index_once() {
        let len = 1000;
        let visits: Mutex<Vec<u32>> = Mutex::new(vec![0; len]);
        parallel_for_threads(len, 4, |index| {
            visits.lock().expect("visit counter poisoned")[index] += 1;
        });
        let visits = visits.into_inner().expect("visit counter poisoned");
        assert!(visits.iter().all(|count| *count == 1));
    }

    #[test]
    fn parallel_for_sums_without_loss() {
        let sum = AtomicUsize::new(0);
        parallel_for_threads(10_000, 4, |index| {
            sum.fetch_add(index, Ordering::Relaxed);
        });
        assert_eq!(sum.load(Ordering::Relaxed), 10_000 * 9_999 / 2);
    }

    #[test]
    fn empty_ranges_do_nothing() {
        parallel_for_threads(0, 8, |_| panic!("must not be called"));
        parallel_for_chunks(0, 8, |_, _| panic!("must not be called"));
    }

    #[test]
    fn chunks_cover_the_range_exactly_once() {
        let covered = AtomicUsize::new(0);
        parallel_for_chunks(103, 4, |start, end| {
            covered.fetch_add(end - start, Ordering::Relaxed);
        });
        assert_eq!(covered.load(Ordering::Relaxed), 103);
    }

    #[test]
    fn thread_clamping() {
        // More threads than work must still visit every index exactly once.
        let count = AtomicUsize::new(0);
        parallel_for_threads(3, 16, |index| {
            assert!(index < 3);
            count.fetch_add(1, Ordering::Relaxed);
        });
        assert_eq!(count.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn hardware_thread_count_is_sane() {
        assert!(num_hardware_threads() >= 1);
    }
}
