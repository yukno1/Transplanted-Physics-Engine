//! Optional work-stealing executor.
//!
//! Rust counterpart of the work-stealing half of `hkJobQueue`
//! (`original/Common/Source/Common/Base/Thread/JobQueue/`). Each worker owns a
//! FIFO deque: it pops from the front of its own queue first and, when empty,
//! steals from the back of other workers in a fixed rotating order. Submission
//! uses round-robin so a batch of independent jobs is spread across workers.
//!
//! This backend is opt-in. The [`crate::base_thread::ThreadPool`] FIFO backend
//! remains the default because priority and submission order are easier to
//! reason about for small scenes; both implement [`TaskExecutor`].

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::collections::VecDeque;
use std::panic::{self, AssertUnwindSafe};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};

use crate::base_thread::executor::TaskExecutor;
use crate::base_thread::thread_pool::{Completion, CompletionGuard, JobHandle, PoolJob};

struct Worker {
    queue: Mutex<VecDeque<PoolJob>>,
    condvar: Condvar,
}

/// Fixed-size work-stealing pool (`hkJobQueue` steal subset).
pub struct WorkStealingPool {
    workers: Arc<Vec<Worker>>,
    worker_handles: Vec<JoinHandle<()>>,
    next: AtomicUsize,
    stopped: Arc<AtomicBool>,
    num_threads: usize,
}

impl WorkStealingPool {
    /// Create a pool with `num_threads` workers (at least one).
    pub fn new(num_threads: usize) -> Self {
        let num_threads = num_threads.max(1);
        let workers: Arc<Vec<Worker>> = Arc::new(
            (0..num_threads)
                .map(|_| Worker {
                    queue: Mutex::new(VecDeque::new()),
                    condvar: Condvar::new(),
                })
                .collect(),
        );
        let stopped = Arc::new(AtomicBool::new(false));
        let mut worker_handles = Vec::with_capacity(num_threads);

        for index in 0..num_threads {
            let workers = Arc::clone(&workers);
            let stopped = Arc::clone(&stopped);
            let handle = thread::Builder::new()
                .name(format!("hk-ws-{index}"))
                .spawn(move || worker_loop(index, workers, stopped))
                .expect("failed to spawn hk work-stealing worker");
            worker_handles.push(handle);
        }

        Self {
            workers,
            worker_handles,
            next: AtomicUsize::new(0),
            stopped,
            num_threads,
        }
    }

    /// Create a pool sized to the hardware thread count.
    pub fn with_hardware_threads() -> Self {
        Self::new(crate::base_thread::parallel::num_hardware_threads())
    }
}

impl TaskExecutor for WorkStealingPool {
    fn submit(&self, job: PoolJob) -> JobHandle {
        assert!(
            !self.stopped.load(Ordering::Acquire),
            "hk work-stealing pool is shut down"
        );
        let completion = Arc::new(Completion::new());
        let guard = CompletionGuard {
            completion: Arc::clone(&completion),
        };
        let job: PoolJob = Box::new(move || {
            let _guard = guard;
            job();
        });

        let index = self.next.fetch_add(1, Ordering::Relaxed) % self.num_threads;
        self.workers[index]
            .queue
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push_back(job);
        self.workers[index].condvar.notify_one();
        JobHandle::from_completion(completion)
    }

    fn num_threads(&self) -> usize {
        self.num_threads
    }

    fn name(&self) -> &'static str {
        "work-stealing"
    }
}

impl Drop for WorkStealingPool {
    fn drop(&mut self) {
        self.stopped.store(true, Ordering::Release);
        for worker in self.workers.iter() {
            worker.condvar.notify_all();
        }
        for handle in self.worker_handles.drain(..) {
            let _ = handle.join();
        }
    }
}

fn take_job(index: usize, workers: &[Worker]) -> Option<PoolJob> {
    if let Some(job) = workers[index]
        .queue
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .pop_front()
    {
        return Some(job);
    }
    for offset in 1..workers.len() {
        let target = (index + offset) % workers.len();
        if let Some(job) = workers[target]
            .queue
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .pop_back()
        {
            return Some(job);
        }
    }
    None
}

fn worker_loop(index: usize, workers: Arc<Vec<Worker>>, stopped: Arc<AtomicBool>) {
    loop {
        if let Some(job) = take_job(index, &workers) {
            // Catch the unwind so a panicking job still marks its handle failed
            // via CompletionGuard, while the worker remains usable.
            let _ = panic::catch_unwind(AssertUnwindSafe(job));
            continue;
        }
        if stopped.load(Ordering::Acquire) {
            break;
        }

        let local = &workers[index];
        let queue = local
            .queue
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if !queue.is_empty() || stopped.load(Ordering::Acquire) {
            continue;
        }
        drop(local.condvar.wait(queue));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn runs_jobs_and_waits() {
        assert_send_sync::<WorkStealingPool>();
        let pool = WorkStealingPool::new(3);
        let counter = Arc::new(AtomicUsize::new(0));
        let mut handles = Vec::new();
        for _ in 0..24 {
            let counter = Arc::clone(&counter);
            handles.push(pool.submit(Box::new(move || {
                counter.fetch_add(1, Ordering::SeqCst);
            })));
        }
        for handle in handles {
            handle.wait();
        }
        assert_eq!(counter.load(Ordering::SeqCst), 24);
        assert_eq!(pool.num_threads(), 3);
    }

    #[test]
    fn zero_threads_is_clamped_to_one() {
        let pool = WorkStealingPool::new(0);
        assert_eq!(pool.num_threads(), 1);
        pool.submit(Box::new(|| {})).wait();
    }

    #[test]
    fn panic_is_reported_and_worker_survives() {
        let pool = WorkStealingPool::new(1);
        let panicked = panic::catch_unwind(AssertUnwindSafe(|| {
            pool.submit(Box::new(|| panic!("boom"))).wait();
        }));
        assert!(panicked.is_err(), "job panic must reach JobHandle::wait");
        pool.submit(Box::new(|| {})).wait();
    }

    #[test]
    fn take_job_prefers_local_then_steals() {
        let workers: Vec<Worker> = (0..2)
            .map(|_| Worker {
                queue: Mutex::new(VecDeque::new()),
                condvar: Condvar::new(),
            })
            .collect();
        let local = Arc::new(AtomicBool::new(false));
        let stolen = Arc::new(AtomicBool::new(false));

        let local_flag = Arc::clone(&local);
        workers[0]
            .queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(Box::new(move || local_flag.store(true, Ordering::SeqCst)));
        let stolen_flag = Arc::clone(&stolen);
        workers[1]
            .queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(Box::new(move || stolen_flag.store(true, Ordering::SeqCst)));

        take_job(0, &workers).expect("local job")();
        assert!(local.load(Ordering::SeqCst));
        assert!(!stolen.load(Ordering::SeqCst));
        take_job(0, &workers).expect("stolen job")();
        assert!(stolen.load(Ordering::SeqCst));
    }
}
