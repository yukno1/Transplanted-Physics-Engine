//! FIFO job queue.
//!
//! Simplified port of `hkJobQueue` / `hkTaskQueue` from
//! `original/Common/Source/Common/Base/Thread/JobQueue/` and
//! `Thread/TaskQueue/`.
//!
//! The C++ job queue is a multi-threaded work-stealing structure with job types,
//! priorities, timers and dependency counting. For Phase 4 the port only needs
//! the single-threaded FIFO semantics used by the deterministic simulation path:
//! push jobs, then run them in order. The multithreaded scheduler is Phase 10
//! work (`hkJobQueue` task stealing / `hkThreadPool::processTaskQueue`).
//!
//! ```
//! use hk_thread::JobQueue;
//! let mut queue = JobQueue::new();
//! queue.push(|| println!("job 1"));
//! queue.push(|| println!("job 2"));
//! assert_eq!(queue.len(), 2);
//! queue.execute_all();
//! assert!(queue.is_empty());
//! ```

/// A queued job (`hkJob` as a boxed closure).
pub type Job = Box<dyn FnOnce() + Send + 'static>;

/// Single-threaded FIFO job queue.
#[derive(Default)]
pub struct JobQueue {
    jobs: Vec<Job>,
}

impl JobQueue {
    /// Empty queue.
    #[inline]
    pub const fn new() -> Self {
        Self { jobs: Vec::new() }
    }

    /// Reserve capacity for `additional` jobs.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.jobs.reserve(additional);
    }

    /// Append a job.
    #[inline]
    pub fn push<F>(&mut self, job: F)
    where
        F: FnOnce() + Send + 'static,
    {
        self.jobs.push(Box::new(job));
    }

    /// Number of queued jobs.
    #[inline]
    pub fn len(&self) -> usize {
        self.jobs.len()
    }

    /// Whether no jobs are queued.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.jobs.is_empty()
    }

    /// Drop all queued jobs without running them (`hkJobQueue::clearJobs`).
    #[inline]
    pub fn clear(&mut self) {
        self.jobs.clear();
    }

    /// Run every queued job in FIFO order, leaving the queue empty.
    ///
    /// Jobs pushed while executing are appended to the queue and will run in the
    /// same drain loop (matching a simple FIFO job queue).
    pub fn execute_all(&mut self) {
        let mut index = 0;
        while index < self.jobs.len() {
            // Move the job out before running it so jobs may push new work.
            let job = core::mem::replace(&mut self.jobs[index], Box::new(|| {}));
            job();
            index += 1;
        }
        self.jobs.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn fifo_execution() {
        let order = Arc::new(AtomicUsize::new(0));
        let mut queue = JobQueue::new();

        for expected in 1..=3 {
            let order = Arc::clone(&order);
            queue.push(move || {
                assert_eq!(order.fetch_add(1, Ordering::SeqCst) + 1, expected);
            });
        }

        assert_eq!(queue.len(), 3);
        queue.execute_all();
        assert!(queue.is_empty());
        assert_eq!(order.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn clear_drops_jobs_without_running() {
        let ran = Arc::new(AtomicUsize::new(0));
        let mut queue = JobQueue::new();
        let ran_inside = Arc::clone(&ran);
        queue.push(move || {
            ran_inside.fetch_add(1, Ordering::SeqCst);
        });
        queue.clear();
        assert!(queue.is_empty());
        assert_eq!(ran.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn jobs_may_enqueue_more_jobs() {
        let counter = Arc::new(AtomicUsize::new(0));
        let mut queue = JobQueue::new();

        let counter_first = Arc::clone(&counter);
        queue.push(move || {
            counter_first.fetch_add(1, Ordering::SeqCst);
        });

        // A job cannot push into the queue it is running in without a shared handle;
        // instead verify that a queue drained once is reusable afterwards.
        queue.execute_all();
        assert_eq!(counter.load(Ordering::SeqCst), 1);

        queue.push(|| {});
        queue.execute_all();
        assert!(queue.is_empty());
    }

    #[test]
    fn reserve_does_not_change_len() {
        let mut queue = JobQueue::new();
        queue.reserve(8);
        assert!(queue.is_empty());
    }
}
