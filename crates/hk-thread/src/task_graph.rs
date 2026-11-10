//! Task graph with dependencies.
//!
//! Rust counterpart of `hkTaskGraph` / `hkTask`
//! (`original/Common/Source/Common/Base/Thread/TaskQueue/hkTaskGraph.h`).
//!
//! The C++ graph stores `hkTask*` nodes plus predecessor/successor edges and is
//! submitted to a `hkTaskQueue`. Rust closures replace the `hkTask` virtual
//! interface, and the port offers two execution strategies:
//!
//! * [`TaskGraph::execute_sequential`] — deterministic, single-threaded. Ready
//!   tasks are chosen by **priority first, then task id**, so the order is fully
//!   reproducible. This is the strategy used by the single-threaded simulation
//!   path (README §4 Phase 4).
//! * [`TaskGraph::execute_layered`] — computes topological levels and runs tasks
//!   of the same level concurrently on a [`ThreadPool`], with a barrier between
//!   levels. Dependencies are respected; the order *inside* a level is
//!   nondeterministic, so tasks in a level must not conflict.
//!
//! Tasks are one-shot closures (`FnOnce`); executing a graph consumes them.
//! Structure (nodes and edges) stays queryable after execution.
//!
//! ```
//! use hk_thread::TaskGraph;
//!
//! let mut graph = TaskGraph::new();
//! let first = graph.add_task(|| println!("first"));
//! let second = graph.add_task(|| println!("second"));
//! graph.add_dependency(first, second);
//! graph.execute_sequential().unwrap();
//! ```

use core::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::thread_pool::ThreadPool;

/// Identifier of a task inside a graph (`hkTaskGraph::TaskId`).
pub type TaskId = usize;

/// Task priority (`hkTask::Priority`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// `hkTask::Priority::LOW`.
    Low,
    /// `hkTask::Priority::MEDIUM` (default).
    #[default]
    Medium,
    /// `hkTask::Priority::HIGH`.
    High,
}

impl Priority {
    #[inline]
    fn rank(self) -> u8 {
        match self {
            Self::Low => 0,
            Self::Medium => 1,
            Self::High => 2,
        }
    }
}

/// Why a graph could not be executed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskGraphError {
    /// The dependency edges contain a cycle (no topological order exists).
    Cycle,
}

impl core::fmt::Display for TaskGraphError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Cycle => f.write_str("task graph contains a dependency cycle"),
        }
    }
}

impl std::error::Error for TaskGraphError {}

struct Node {
    job: Option<Box<dyn FnOnce() + Send + 'static>>,
    priority: Priority,
}

/// Container of one-shot tasks and dependency edges (`hkTaskGraph`).
pub struct TaskGraph {
    nodes: Vec<Node>,
    dependencies: Vec<(TaskId, TaskId)>,
}

impl Default for TaskGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl TaskGraph {
    /// Empty graph.
    #[inline]
    pub const fn new() -> Self {
        Self {
            nodes: Vec::new(),
            dependencies: Vec::new(),
        }
    }

    /// Empty graph with reserved capacity (`hkTaskGraph::reserve`).
    pub fn with_capacity(num_tasks: usize, num_dependencies: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(num_tasks),
            dependencies: Vec::with_capacity(num_dependencies),
        }
    }

    /// Reserve space for tasks and dependencies.
    pub fn reserve(&mut self, num_tasks: usize, num_dependencies: usize) {
        self.nodes.reserve(num_tasks);
        self.dependencies.reserve(num_dependencies);
    }

    /// Add a task with default (`Medium`) priority (`hkTaskGraph::addTask`).
    #[inline]
    pub fn add_task<F>(&mut self, job: F) -> TaskId
    where
        F: FnOnce() + Send + 'static,
    {
        self.add_task_with_priority(job, Priority::Medium)
    }

    /// Add a task with an explicit priority.
    pub fn add_task_with_priority<F>(&mut self, job: F, priority: Priority) -> TaskId
    where
        F: FnOnce() + Send + 'static,
    {
        let id = self.nodes.len();
        self.nodes.push(Node {
            job: Some(Box::new(job)),
            priority,
        });
        id
    }

    /// Add a dependency edge `predecessor -> successor` (`addDependency`).
    ///
    /// Duplicate edges are ignored.
    ///
    /// # Panics
    ///
    /// Panics if either id is out of range.
    pub fn add_dependency(&mut self, predecessor: TaskId, successor: TaskId) {
        assert!(
            predecessor < self.nodes.len() && successor < self.nodes.len(),
            "dependency references an unknown task id"
        );
        if !self.dependencies.contains(&(predecessor, successor)) {
            self.dependencies.push((predecessor, successor));
        }
    }

    /// Add a dependency if both ids are valid and the edge is new
    /// (`tryAddDependency`).
    pub fn try_add_dependency(&mut self, predecessor: TaskId, successor: TaskId) -> bool {
        if predecessor >= self.nodes.len() || successor >= self.nodes.len() {
            return false;
        }
        if self.dependencies.contains(&(predecessor, successor)) {
            return false;
        }
        self.dependencies.push((predecessor, successor));
        true
    }

    /// Number of tasks (`getNodes().getSize()`).
    #[inline]
    pub fn task_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of dependency edges (`getDependencies().getSize()`).
    #[inline]
    pub fn dependency_count(&self) -> usize {
        self.dependencies.len()
    }

    /// Whether the graph has no tasks (`hkTaskGraph::isEmpty`).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Dependency edges as `(predecessor, successor)` pairs.
    #[inline]
    pub fn dependencies(&self) -> &[(TaskId, TaskId)] {
        &self.dependencies
    }

    /// Priority of a task.
    #[inline]
    pub fn task_priority(&self, id: TaskId) -> Option<Priority> {
        self.nodes.get(id).map(|node| node.priority)
    }

    /// Clear tasks and edges (`hkTaskGraph::clear`).
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.dependencies.clear();
    }

    /// Compute a deterministic topological order.
    ///
    /// Ready tasks are popped in priority order (High first), and by ascending
    /// task id within the same priority. Returns [`TaskGraphError::Cycle`] if no
    /// order exists.
    pub fn compute_order(&self) -> Result<Vec<TaskId>, TaskGraphError> {
        let count = self.nodes.len();
        let mut indegree = vec![0u32; count];
        let mut successors: Vec<Vec<TaskId>> = vec![Vec::new(); count];
        for &(predecessor, successor) in &self.dependencies {
            indegree[successor] += 1;
            successors[predecessor].push(successor);
        }

        let mut ready = BinaryHeap::new();
        for (id, &degree) in indegree.iter().enumerate() {
            if degree == 0 {
                ready.push(Ready {
                    priority: self.nodes[id].priority,
                    id,
                });
            }
        }

        let mut order = Vec::with_capacity(count);
        while let Some(entry) = ready.pop() {
            order.push(entry.id);
            for &successor in &successors[entry.id] {
                indegree[successor] -= 1;
                if indegree[successor] == 0 {
                    ready.push(Ready {
                        priority: self.nodes[successor].priority,
                        id: successor,
                    });
                }
            }
        }

        if order.len() == count {
            Ok(order)
        } else {
            Err(TaskGraphError::Cycle)
        }
    }

    /// Execute every task once, deterministically, on the current thread.
    ///
    /// Tasks are consumed; a job that panics propagates the panic to the caller.
    pub fn execute_sequential(&mut self) -> Result<(), TaskGraphError> {
        let order = self.compute_order()?;
        for id in order {
            if let Some(job) = self.nodes[id].job.take() {
                job();
            }
        }
        Ok(())
    }

    /// Execute the graph on a thread pool, one topological level at a time.
    ///
    /// Each level is a barrier: all jobs of level `n` must finish before level
    /// `n+1` starts. Priority hints are ignored here (the pool is FIFO).
    pub fn execute_layered(&mut self, pool: &ThreadPool) -> Result<(), TaskGraphError> {
        let order = self.compute_order()?;
        let count = self.nodes.len();

        let mut predecessors: Vec<Vec<TaskId>> = vec![Vec::new(); count];
        for &(predecessor, successor) in &self.dependencies {
            predecessors[successor].push(predecessor);
        }

        let mut levels = vec![0usize; count];
        for &id in &order {
            levels[id] = predecessors[id]
                .iter()
                .map(|&predecessor| levels[predecessor] + 1)
                .max()
                .unwrap_or(0);
        }
        let max_level = levels.iter().copied().max().unwrap_or(0);

        for level in 0..=max_level {
            let jobs: Vec<_> = (0..count)
                .filter(|&id| levels[id] == level)
                .filter_map(|id| self.nodes[id].job.take())
                .collect();
            if jobs.is_empty() {
                continue;
            }

            let mut handles = Vec::with_capacity(jobs.len());
            for job in jobs {
                handles.push(pool.spawn(job));
            }
            for handle in handles {
                handle.wait();
            }
        }

        Ok(())
    }
}

/// Priority-ordered ready list entry (max-heap: High first, lowest id first).
struct Ready {
    priority: Priority,
    id: TaskId,
}

impl PartialEq for Ready {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.id == other.id
    }
}

impl Eq for Ready {}

impl Ord for Ready {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority
            .rank()
            .cmp(&other.priority.rank())
            .then_with(|| other.id.cmp(&self.id))
    }
}

impl PartialOrd for Ready {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::Mutex;

    type Recorder = Arc<Mutex<Vec<TaskId>>>;

    fn recording_task(recorder: &Recorder, id: TaskId) -> impl FnOnce() + Send + 'static {
        let recorder = Arc::clone(recorder);
        move || {
            recorder.lock().expect("recorder poisoned").push(id);
        }
    }

    #[test]
    fn sequential_respects_dependencies() {
        // 0 -> 1, 0 -> 2, 1 -> 3, 2 -> 3
        let recorder: Recorder = Arc::new(Mutex::new(Vec::new()));
        let mut graph = TaskGraph::new();
        for id in 0..4 {
            graph.add_task(recording_task(&recorder, id));
        }
        graph.add_dependency(0, 1);
        graph.add_dependency(0, 2);
        graph.add_dependency(1, 3);
        graph.add_dependency(2, 3);

        graph.execute_sequential().expect("acyclic graph");
        let order = recorder.lock().expect("recorder poisoned");
        assert_eq!(order.len(), 4);

        let position = |id: TaskId| order.iter().position(|entry| *entry == id).unwrap();
        assert!(position(0) < position(1));
        assert!(position(0) < position(2));
        assert!(position(1) < position(3));
        assert!(position(2) < position(3));
    }

    #[test]
    fn priority_breaks_ties_deterministically() {
        let recorder: Recorder = Arc::new(Mutex::new(Vec::new()));
        let mut graph = TaskGraph::new();
        graph.add_task_with_priority(recording_task(&recorder, 0), Priority::Low);
        graph.add_task_with_priority(recording_task(&recorder, 1), Priority::High);
        graph.add_task_with_priority(recording_task(&recorder, 2), Priority::Medium);

        graph.execute_sequential().expect("acyclic graph");
        assert_eq!(*recorder.lock().expect("recorder poisoned"), vec![1, 2, 0]);
        assert_eq!(graph.task_priority(1), Some(Priority::High));
    }

    #[test]
    fn cycle_is_detected_before_running_anything() {
        let runs: Arc<Mutex<usize>> = Arc::new(Mutex::new(0));
        let mut graph = TaskGraph::new();
        for _ in 0..3 {
            let runs = Arc::clone(&runs);
            graph.add_task(move || {
                *runs.lock().expect("counter poisoned") += 1;
            });
        }
        graph.add_dependency(0, 1);
        graph.add_dependency(1, 2);
        graph.add_dependency(2, 0);

        assert_eq!(graph.execute_sequential(), Err(TaskGraphError::Cycle));
        assert_eq!(*runs.lock().expect("counter poisoned"), 0);
    }

    #[test]
    fn dependency_validation() {
        let mut graph = TaskGraph::new();
        graph.add_task(|| {});
        graph.add_task(|| {});
        assert!(graph.try_add_dependency(0, 1));
        assert!(!graph.try_add_dependency(0, 1), "duplicate edge");
        assert!(!graph.try_add_dependency(0, 9), "invalid successor");
        assert!(!graph.try_add_dependency(9, 0), "invalid predecessor");
        assert_eq!(graph.dependency_count(), 1);
    }

    #[test]
    #[should_panic(expected = "unknown task id")]
    fn add_dependency_panics_on_invalid_id() {
        let mut graph = TaskGraph::new();
        graph.add_task(|| {});
        graph.add_dependency(0, 1);
    }

    #[test]
    fn layered_execution_respects_dependencies() {
        let recorder: Recorder = Arc::new(Mutex::new(Vec::new()));
        let mut graph = TaskGraph::new();
        for id in 0..5 {
            graph.add_task(recording_task(&recorder, id));
        }
        graph.add_dependency(0, 1);
        graph.add_dependency(0, 2);
        graph.add_dependency(1, 3);
        graph.add_dependency(2, 3);
        graph.add_dependency(3, 4);
        graph.add_dependency(0, 4);

        let pool = ThreadPool::new(4);
        graph.execute_layered(&pool).expect("acyclic graph");

        let order = recorder.lock().expect("recorder poisoned");
        assert_eq!(order.len(), 5);
        let position = |id: TaskId| order.iter().position(|entry| *entry == id).unwrap();
        for &(predecessor, successor) in graph.dependencies() {
            assert!(
                position(predecessor) < position(successor),
                "{predecessor} must precede {successor}"
            );
        }
    }

    #[test]
    fn empty_and_cleared_graphs() {
        let mut graph = TaskGraph::new();
        assert!(graph.is_empty());
        assert!(graph.execute_sequential().is_ok());
        assert!(graph.compute_order().unwrap().is_empty());

        graph.add_task(|| {});
        assert_eq!(graph.task_count(), 1);
        graph.clear();
        assert!(graph.is_empty());
        assert_eq!(graph.dependency_count(), 0);
    }

    #[test]
    fn tasks_are_one_shot() {
        let runs: Arc<Mutex<usize>> = Arc::new(Mutex::new(0));
        let mut graph = TaskGraph::new();
        let runs_clone = Arc::clone(&runs);
        graph.add_task(move || {
            *runs_clone.lock().expect("counter poisoned") += 1;
        });
        graph.execute_sequential().unwrap();
        graph.execute_sequential().unwrap();
        assert_eq!(*runs.lock().expect("counter poisoned"), 1);
    }
}
