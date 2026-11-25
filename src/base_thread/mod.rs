//! `base-thread` — threading primitives, thread pool, job queue and task graph.
//!
//! Rust port of the threading layer under
//! `original/Common/Source/Common/Base/Thread/` (migration Phase 4, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`SpinLock`] | `hkSpinLockSimple` (`Thread/Atomic/`) | busy-wait lock, no fairness (same as C++) |
//! | [`JobQueue`] | `hkJobQueue` / `hkTaskQueue` | single-threaded FIFO subset |
//! | [`TaskExecutor`] | `hkTaskQueue` submission boundary | object-safe job submission backend |
//! | [`ThreadPool`] | `hkThreadPool` | fixed FIFO workers, awaitable [`JobHandle`] |
//! | [`TaskGraph`] | `hkTaskGraph` / `hkTask` | one-shot closure tasks, dependency DAG, optional priority submission |
//! | [`parallel`] | chunked job submission around `hkJobQueue` | scoped `parallel_for`, deterministic chunk variant |
//!
//! `TaskGraph` offers two strategies:
//!
//! * [`TaskGraph::execute_sequential`] — deterministic single-threaded execution,
//!   ready tasks ordered by priority then task id. This is the path the
//!   single-threaded simulation uses (Phase 4 priority).
//! * [`TaskGraph::execute_layered_with_priority`] — runs one topological level
//!   at a time on any [`TaskExecutor`] with a barrier between levels. Optional
//!   priority ordering affects submission only; dependencies are preserved.
//!
//! # Zero external dependencies
//!
//! The Phase 4 plan mentioned `rayon`. crates.io was unreachable while this phase
//! was implemented (`SSL connect error`), and the workspace has stayed
//! dependency-free so far, so the port uses `std::thread` + `std::sync` directly.
//! A rayon-based backend can be added later behind the same `ThreadPool` /
//! `parallel_for` API if dependency-free operation is no longer required.
//!
//! # Not ported in this phase
//!
//! * Thread priority/affinity and OS handles (`hkThread`).
//! * `hkWorkerThreadContext`, per-type job queues, `hkSemaphore*`,
//!   readers/writer locks and `hkCriticalSection` (std `Mutex` covers the
//!   port's needs so far).
//! * Complete `hknpMultithreadedSimulation` behaviour: solver/transform task
//!   graphs and `hknpTaskGraph` dependency bookkeeping. The physics world uses
//!   a deterministic subset here: [`ThreadPool`] or [`WorkStealingPool`] plus
//!   a per-step [`TaskGraph`] over narrow-phase and island chunks.
//! * Task multiplicity (`addTaskWithMultiplicity`) and reference-counted
//!   `hkReferencedTask` nodes; graph merging (`hkTaskGraph::append`).
//!
//! # Determinism
//!
//! `execute_sequential` is reproducible: same graph, same order. Layered
//! execution preserves dependencies but not task completion order; priority
//! only orders submissions inside one level. Independent tasks must not write
//! to shared state (same rule as the SDK's multithreaded task queue).

pub mod executor;
pub mod job_queue;
pub mod parallel;
pub mod spin_lock;
pub mod task_graph;
pub mod thread_pool;
pub mod work_stealing;

pub use executor::TaskExecutor;
pub use job_queue::{Job, JobQueue};
pub use parallel::{num_hardware_threads, parallel_for, parallel_for_chunks, parallel_for_threads};
pub use spin_lock::{SpinGuard, SpinLock};
pub use task_graph::{Priority, TaskGraph, TaskGraphError, TaskId};
pub use thread_pool::{JobHandle, PoolJob, ThreadPool};
pub use work_stealing::WorkStealingPool;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn pooled_task_graph_composes_with_parallel_loops() {
        let pool = ThreadPool::new(2);
        let mut graph = TaskGraph::new();

        let counter = std::sync::Arc::new(AtomicUsize::new(0));
        let counter_for_loop = std::sync::Arc::clone(&counter);
        let first = graph.add_task(move || {
            counter_for_loop.fetch_add(1, Ordering::SeqCst);
        });

        let counter_for_graph = std::sync::Arc::clone(&counter);
        let second = graph.add_task(move || {
            counter_for_graph.fetch_add(1, Ordering::SeqCst);
        });
        graph.add_dependency(first, second);

        graph.execute_layered(&pool).expect("acyclic graph");
        assert_eq!(counter.load(Ordering::SeqCst), 2);

        let counter_for_loop = std::sync::Arc::clone(&counter);
        pool.parallel_for(16, move |_| {
            counter_for_loop.fetch_add(1, Ordering::SeqCst);
        });
        assert_eq!(counter.load(Ordering::SeqCst), 18);
    }
}
