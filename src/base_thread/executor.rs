//! Object-safe executor abstraction for task submission.
//!
//! Rust counterpart of the `hkTaskQueue` / `hkThreadPool` execution boundary
//! used by `hknpMultithreadedSimulation`. The physics task graph uses this
//! trait so FIFO and work-stealing backends can be selected without changing
//! task bodies or result merging.

use crate::base_thread::thread_pool::{JobHandle, PoolJob};

/// A backend that can execute boxed jobs (`hkTaskQueue` submission subset).
///
/// The trait is deliberately object-safe: [`crate::base_thread::TaskGraph`]
/// can run against `&dyn TaskExecutor`, while generic callers can use
/// `&impl TaskExecutor` for static dispatch.
pub trait TaskExecutor: Send + Sync {
    /// Submit one job and return its completion handle.
    fn submit(&self, job: PoolJob) -> JobHandle;

    /// Number of worker threads available to this executor.
    fn num_threads(&self) -> usize;

    /// Stable backend name used in debug output and panic context.
    fn name(&self) -> &'static str;
}
