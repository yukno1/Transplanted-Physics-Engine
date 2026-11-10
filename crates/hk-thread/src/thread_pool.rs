//! Fixed-size worker thread pool.
//!
//! Rust counterpart of `hkThreadPool` / the worker half of `hkJobQueue`
//! (`original/Common/Source/Common/Base/Thread/Pool/hkThreadPool.h`,
//! `Thread/JobQueue/hkJobQueue.h`).
//!
//! The C++ pool processes a shared job/task queue from N worker threads and
//! exposes `waitForCompletion` / `getNumThreads`. This port implements the same
//! shape with `std::thread` + `mpsc`: workers block on one shared receiver, jobs
//! are boxed `FnOnce` closures, and every job gets a [`JobHandle`] that can be
//! awaited.
//!
//! ```
//! use hk_thread::ThreadPool;
//! let pool = ThreadPool::new(2);
//! let handle = pool.spawn(|| println!("run on a worker"));
//! handle.wait();
//! ```
//!
//! # Differences from the C++ pool
//!
//! * Thread priority/affinity (`hkThread::setPriority`, `setThreadAffinityMask`)
//!   is not exposed; the Rust standard library has no portable API for it.
//! * There is no timer/monitor stream (`appendTimerData`).
//! * Jobs are FIFO; `hkJobQueue`'s per-type queues and work stealing are Phase 10
//!   work.
//! * A job that panics unwinds its worker thread (standard Rust behaviour); the
//!   pool then has one fewer worker. Jobs in this port must not panic.

use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};

/// Boxed work item (`hkJob` as a closure).
pub type PoolJob = Box<dyn FnOnce() + Send + 'static>;

enum Message {
    Run(PoolJob),
    Shutdown,
}

struct CompletionState {
    done: bool,
    success: bool,
}

struct Completion {
    state: Mutex<CompletionState>,
    condvar: Condvar,
}

impl Completion {
    fn new() -> Self {
        Self {
            state: Mutex::new(CompletionState {
                done: false,
                success: false,
            }),
            condvar: Condvar::new(),
        }
    }
}

/// Marks the completion when the job finishes, including on unwind.
struct CompletionGuard {
    completion: Arc<Completion>,
}

impl Drop for CompletionGuard {
    fn drop(&mut self) {
        let mut state = self
            .completion
            .state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        state.done = true;
        state.success = !std::thread::panicking();
        drop(state);
        self.completion.condvar.notify_all();
    }
}

/// Handle that completes when the spawned job has finished.
pub struct JobHandle {
    completion: Arc<Completion>,
}

impl JobHandle {
    /// Block until the job has run.
    ///
    /// # Panics
    ///
    /// Panics if the worker panicked while running the job.
    pub fn wait(self) {
        let mut state = self
            .completion
            .state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        while !state.done {
            state = self
                .completion
                .condvar
                .wait(state)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
        let success = state.success;
        drop(state);
        assert!(success, "job panicked on a worker thread");
    }

    /// Non-blocking completion check.
    #[inline]
    pub fn is_done(&self) -> bool {
        self.completion
            .state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .done
    }
}

/// Fixed-size thread pool (`hkThreadPool`).
pub struct ThreadPool {
    sender: Mutex<Sender<Message>>,
    workers: Vec<JoinHandle<()>>,
    num_threads: usize,
}

impl ThreadPool {
    /// Create a pool with `num_threads` workers (at least one).
    #[inline]
    pub fn new(num_threads: usize) -> Self {
        Self::with_name("hk-worker", num_threads)
    }

    /// Create a pool sized to the hardware thread count.
    #[inline]
    pub fn with_hardware_threads() -> Self {
        Self::new(crate::parallel::num_hardware_threads())
    }

    /// Create a pool whose workers are named `"{name}-{index}"`.
    pub fn with_name(name: &str, num_threads: usize) -> Self {
        let num_threads = num_threads.max(1);
        let (sender, receiver) = mpsc::channel();
        let receiver = Arc::new(Mutex::new(receiver));

        let mut workers = Vec::with_capacity(num_threads);
        for index in 0..num_threads {
            let receiver = Arc::clone(&receiver);
            let handle = thread::Builder::new()
                .name(format!("{name}-{index}"))
                .spawn(move || worker_loop(&receiver))
                .expect("failed to spawn hk thread pool worker");
            workers.push(handle);
        }

        Self {
            sender: Mutex::new(sender),
            workers,
            num_threads,
        }
    }

    /// Number of worker threads.
    #[inline]
    pub fn num_threads(&self) -> usize {
        self.num_threads
    }

    /// Submit a job and return a handle for its completion (`processJobQueue`
    /// style fire-and-forget submission).
    pub fn spawn<F>(&self, job: F) -> JobHandle
    where
        F: FnOnce() + Send + 'static,
    {
        let completion = Arc::new(Completion::new());
        let guard = CompletionGuard {
            completion: Arc::clone(&completion),
        };
        self.send(Message::Run(Box::new(move || {
            let _guard = guard;
            job();
        })));
        JobHandle { completion }
    }

    /// Run `function(index)` for all indices in `0..len` on the pool and block
    /// until every index has been processed.
    ///
    /// The closure must be `'static` because it runs on detached workers; use
    /// [`crate::parallel::parallel_for`] for scoped borrowing.
    pub fn parallel_for<F>(&self, len: usize, function: F)
    where
        F: Fn(usize) + Send + Sync + 'static,
    {
        if len == 0 {
            return;
        }
        let workers = self.num_threads.min(len);
        let next = Arc::new(AtomicUsize::new(0));
        let function = Arc::new(function);

        let mut handles = Vec::with_capacity(workers);
        for _ in 0..workers {
            let next = Arc::clone(&next);
            let function = Arc::clone(&function);
            handles.push(self.spawn(move || {
                loop {
                    let index = next.fetch_add(1, Ordering::Relaxed);
                    if index >= len {
                        break;
                    }
                    function(index);
                }
            }));
        }
        for handle in handles {
            handle.wait();
        }
    }

    fn send(&self, message: Message) {
        let sender = self
            .sender
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        assert!(sender.send(message).is_ok(), "hk thread pool is shut down");
    }
}

impl Drop for ThreadPool {
    fn drop(&mut self) {
        // Ask every worker to exit. Queued jobs are FIFO ahead of the shutdown
        // message and therefore still run before the workers terminate.
        let sender = self
            .sender
            .get_mut()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        for _ in 0..self.num_threads {
            // The channel is closed only if all workers already exited (e.g. due
            // to a panicking job); nothing to do in that case.
            if sender.send(Message::Shutdown).is_err() {
                break;
            }
        }
        for worker in self.workers.drain(..) {
            if worker.join().is_err() {
                // A job panicked earlier; the pool is still safe to drop.
            }
        }
    }
}

fn worker_loop(receiver: &Mutex<Receiver<Message>>) {
    loop {
        let message = {
            let receiver = receiver
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            receiver.recv()
        };
        match message {
            Ok(Message::Run(job)) => job(),
            Ok(Message::Shutdown) | Err(_) => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn runs_jobs_and_waits() {
        let pool = ThreadPool::new(2);
        let counter = Arc::new(AtomicUsize::new(0));
        let mut handles = Vec::new();
        for _ in 0..8 {
            let counter = Arc::clone(&counter);
            handles.push(pool.spawn(move || {
                counter.fetch_add(1, Ordering::SeqCst);
            }));
        }
        for handle in handles {
            handle.wait();
        }
        assert_eq!(counter.load(Ordering::SeqCst), 8);
        assert_eq!(pool.num_threads(), 2);
    }

    #[test]
    fn zero_threads_is_clamped_to_one() {
        let pool = ThreadPool::new(0);
        assert_eq!(pool.num_threads(), 1);
        pool.spawn(|| {}).wait();
    }

    #[test]
    fn parallel_for_visits_every_index() {
        let pool = ThreadPool::new(4);
        let counter = Arc::new(AtomicUsize::new(0));
        let loop_counter = Arc::clone(&counter);
        pool.parallel_for(100, move |index| {
            assert!(index < 100);
            loop_counter.fetch_add(1, Ordering::Relaxed);
        });
        assert_eq!(counter.load(Ordering::Relaxed), 100);
    }

    #[test]
    fn empty_parallel_for_does_nothing() {
        let pool = ThreadPool::new(2);
        pool.parallel_for(0, |_| panic!("must not run"));
    }

    #[test]
    fn is_done_reports_completion() {
        let pool = ThreadPool::new(1);
        let handle = pool.spawn(|| {});
        while !handle.is_done() {
            std::hint::spin_loop();
        }
        handle.wait();
    }

    #[test]
    fn nested_pool_usage_is_safe() {
        let pool = ThreadPool::new(2);
        let counter = Arc::new(AtomicUsize::new(1));
        let handle = {
            let counter = Arc::clone(&counter);
            pool.spawn(move || {
                counter.fetch_add(1, Ordering::SeqCst);
            })
        };
        handle.wait();
        assert_eq!(counter.load(Ordering::SeqCst), 2);
    }
}
