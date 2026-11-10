//! `hk-thread` — threading primitives, thread pool, job queue and task graph.
//!
//! Rust port of the threading layer under
//! `original/Common/Source/Common/Base/Thread/` (migration Phase 4, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`SpinLock`] | `hkSpinLockSimple` (`Thread/Atomic/`) | busy-wait lock, no fairness (same as C++) |
//! | [`JobQueue`] | `hkJobQueue` / `hkTaskQueue` | single-threaded FIFO subset; multithreaded work stealing is Phase 10 |
//! | [`ThreadPool`] | `hkThreadPool` | fixed workers, fire-and-forget jobs with awaitable [`JobHandle`] |
//! | [`TaskGraph`] | `hkTaskGraph` / `hkTask` | one-shot closure tasks, dependency DAG |
//! | [`parallel`] | chunked job submission around `hkJobQueue` | scoped `parallel_for`, deterministic chunk variant |
//!
//! `TaskGraph` offers two strategies:
//!
//! * [`TaskGraph::execute_sequential`] — deterministic single-threaded execution,
//!   ready tasks ordered by priority then task id. This is the path the
//!   single-threaded simulation uses (Phase 4 priority).
//! * [`TaskGraph::execute_layered`] — runs one topological level at a time on a
//!   [`ThreadPool`] with a barrier between levels. Dependencies are preserved,
//!   but execution order within a level is nondeterministic.
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
//! * `hkWorkerThreadContext`, the job-type/priority queues and work stealing of
//!   `hkJobQueue`, `hkSemaphore*`, readers/writer locks, `hkCriticalSection`
//!   (std `Mutex` covers the port's needs so far).
//! * `hknpMultithreadedSimulation` / `hknpTaskGraph` (physics scheduling) — the
//!   single-threaded simulation is assembled in Phases 6/8, parallel scheduling
//!   is Phase 10.
//! * Task multiplicity (`addTaskWithMultiplicity`) and reference-counted
//!   `hkReferencedTask` nodes; graph merging (`hkTaskGraph::append`).
//!
//! # Determinism
//!
//! `execute_sequential` is reproducible: same graph, same order. `execute_layered`
//! preserves dependencies but not the relative order of independent tasks, so
//! independent tasks must not write to shared state (same rule as the SDK's
//! multithreaded task queue).

pub mod job_queue;
pub mod parallel;
pub mod spin_lock;
pub mod task_graph;
pub mod thread_pool;

pub use job_queue::{Job, JobQueue};
pub use parallel::{num_hardware_threads, parallel_for, parallel_for_chunks, parallel_for_threads};
pub use spin_lock::{SpinGuard, SpinLock};
pub use task_graph::{Priority, TaskGraph, TaskGraphError, TaskId};
pub use thread_pool::{JobHandle, PoolJob, ThreadPool};

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
