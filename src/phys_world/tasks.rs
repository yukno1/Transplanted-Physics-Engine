//! Deterministic per-step scheduling for the physics world.
//!
//! Rust counterpart of the `hknpTaskGraph` scheduling layer used by
//! `hknpMultithreadedSimulation`. Only independent compute stages are
//! dispatched here: broad-phase candidate narrow phase and per-island solver
//! sweeps. Stages that mutate the world (gravity, event dispatch, warm-start
//! write-back, sleeping and integration) stay on the calling thread, in the
//! original order, so `hvk-trace v1` remains bit-identical.
//!
//! A pooled step uses a fresh [`TaskGraph`] per stage: chunk tasks write to
//! indexed result slots and a dependent merge task restores stable candidate /
//! island order. The legacy scoped dispatch keeps the previous API behaviour
//! and shares the same chunk and merge helpers.

use std::sync::{Arc, Mutex};

use crate::base_thread::{Priority, TaskExecutor, TaskGraph};
use crate::phys_solver::Solver;
use crate::phys_world::ContactManifold;
use crate::phys_world::island::Island;
use crate::phys_world::world::{
    ContactCandidate, IslandSolveBuffers, IslandSolveResult, collide_candidate,
    contacts_from_candidates,
};

/// Candidate count below which a scoped narrow phase costs more than it saves.
pub(crate) const MIN_PARALLEL_CANDIDATES: usize = 32;
/// Island count below which the sequential solver sweep is used.
pub(crate) const MIN_PARALLEL_ISLANDS: usize = 2;

/// Stage name carried into panic messages so a worker failure is locatable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StepStage {
    NarrowPhase,
    IslandSolve,
}

impl StepStage {
    fn panic_message(self) -> &'static str {
        match self {
            Self::NarrowPhase => "narrow-phase task graph contains a cycle",
            Self::IslandSolve => "island-solve task graph contains a cycle",
        }
    }
}

/// How a compute stage is dispatched for one step.
#[derive(Clone, Copy)]
pub(crate) enum ParallelDispatch<'a> {
    /// Scoped workers created for this stage (the legacy `step_with_threads`
    /// behaviour). Useful for comparing scoped and pooled paths.
    Scoped(usize),
    /// Deterministic task graph submitted to a shared [`TaskExecutor`].
    Pooled(&'a dyn TaskExecutor, usize, bool),
}

impl ParallelDispatch<'_> {
    fn threads(self) -> usize {
        match self {
            Self::Scoped(threads) | Self::Pooled(_, threads, _) => threads,
        }
    }
}

/// Deterministic contiguous ranges matching `parallel_for_chunks`.
fn chunk_ranges(len: usize, threads: usize) -> Vec<(usize, usize)> {
    if len == 0 {
        return Vec::new();
    }
    let threads = threads.max(1).min(len);
    let chunk_size = len.div_ceil(threads);
    (0..threads)
        .filter_map(|chunk| {
            let start = chunk * chunk_size;
            let end = (start + chunk_size).min(len);
            (start < end).then_some((start, end))
        })
        .collect()
}

fn collect_contacts_chunk(
    candidates: &[ContactCandidate],
    tolerance: f32,
    start: usize,
    end: usize,
) -> Vec<(usize, ContactManifold)> {
    let mut local = Vec::new();
    for (index, candidate) in candidates.iter().enumerate().take(end).skip(start) {
        if let Some(manifold) = collide_candidate(candidate, tolerance) {
            local.push((index, manifold));
        }
    }
    local
}

fn solve_island_chunk(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[Island],
    start: usize,
    end: usize,
    delta_time: f32,
) -> Vec<IslandSolveResult> {
    let mut bodies = buffers.solver_bodies.to_vec();
    let mut contacts = buffers.contact_impulses.to_vec();
    let mut joints = buffers.joint_impulses.to_vec();
    let mut local = Vec::with_capacity(end - start);

    for (island_index, island) in islands.iter().enumerate().take(end).skip(start) {
        if !island.contacts.is_empty() {
            solver.solve_contacts_subset(
                &mut bodies,
                buffers.constraints,
                &island.contacts,
                &mut contacts,
                delta_time,
            );
        }
        if !island.joints.is_empty() {
            solver.solve_joints_subset_with_impulses(
                &mut bodies,
                buffers.solver_joints,
                &island.joints,
                &mut joints,
                delta_time,
            );
        }
        local.push(IslandSolveResult {
            index: island_index,
            bodies: island.bodies.iter().map(|&i| (i, bodies[i])).collect(),
            contacts: island.contacts.iter().map(|&i| (i, contacts[i])).collect(),
            joints: island.joints.iter().map(|&i| (i, joints[i])).collect(),
        });
    }
    local
}

fn sort_contacts(results: &mut [(usize, ContactManifold)]) {
    results.sort_unstable_by_key(|(index, _)| *index);
}

fn sort_islands(results: &mut [IslandSolveResult]) {
    results.sort_unstable_by_key(|result| result.index);
}

fn execute_graph(
    mut graph: TaskGraph,
    executor: &dyn TaskExecutor,
    respect_priority: bool,
    stage: StepStage,
) {
    graph
        .execute_layered_with_priority(executor, respect_priority)
        .unwrap_or_else(|_| panic!("{}", stage.panic_message()));
}

fn add_stage_task<F>(
    graph: &mut TaskGraph,
    respect_priority: bool,
    priority: Priority,
    job: F,
) -> usize
where
    F: FnOnce() + Send + 'static,
{
    if respect_priority {
        graph.add_task_with_priority(job, priority)
    } else {
        graph.add_task(job)
    }
}

/// Generate contact manifolds with the requested dispatcher.
pub(crate) fn collect_contacts(
    candidates: &[ContactCandidate],
    tolerance: f32,
    dispatch: ParallelDispatch<'_>,
) -> Vec<ContactManifold> {
    if candidates.len() < MIN_PARALLEL_CANDIDATES || dispatch.threads() <= 1 {
        return contacts_from_candidates(candidates, tolerance);
    }
    match dispatch {
        ParallelDispatch::Scoped(threads) => {
            collect_contacts_scoped(candidates, tolerance, threads)
        }
        ParallelDispatch::Pooled(executor, threads, respect_priority) => {
            collect_contacts_pooled(candidates, tolerance, executor, threads, respect_priority)
        }
    }
}

fn collect_contacts_scoped(
    candidates: &[ContactCandidate],
    tolerance: f32,
    threads: usize,
) -> Vec<ContactManifold> {
    let results: Mutex<Vec<(usize, ContactManifold)>> = Mutex::new(Vec::new());
    crate::base_thread::parallel::parallel_for_chunks(candidates.len(), threads, |start, end| {
        let local = collect_contacts_chunk(candidates, tolerance, start, end);
        if !local.is_empty() {
            results
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .extend(local);
        }
    });
    let mut results = results
        .into_inner()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    sort_contacts(&mut results);
    results.into_iter().map(|(_, manifold)| manifold).collect()
}

fn collect_contacts_pooled(
    candidates: &[ContactCandidate],
    tolerance: f32,
    executor: &dyn TaskExecutor,
    threads: usize,
    respect_priority: bool,
) -> Vec<ContactManifold> {
    let candidates = Arc::new(candidates.to_vec());
    let results: Arc<Mutex<Vec<(usize, ContactManifold)>>> = Arc::new(Mutex::new(Vec::new()));
    let merged: Arc<Mutex<Option<Vec<ContactManifold>>>> = Arc::new(Mutex::new(None));
    let ranges = chunk_ranges(candidates.len(), threads);

    let mut graph = TaskGraph::with_capacity(ranges.len() + 1, ranges.len());
    let mut chunk_tasks = Vec::with_capacity(ranges.len());
    for (start, end) in ranges {
        let candidates = Arc::clone(&candidates);
        let results = Arc::clone(&results);
        chunk_tasks.push(add_stage_task(
            &mut graph,
            respect_priority,
            Priority::High,
            move || {
                let local = collect_contacts_chunk(&candidates, tolerance, start, end);
                if !local.is_empty() {
                    results
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .extend(local);
                }
            },
        ));
    }

    let merge_results = Arc::clone(&results);
    let merge_output = Arc::clone(&merged);
    let merge_task = add_stage_task(&mut graph, respect_priority, Priority::Medium, move || {
        let mut rows = merge_results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        sort_contacts(&mut rows);
        let contacts = rows.drain(..).map(|(_, manifold)| manifold).collect();
        *merge_output
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(contacts);
    });
    for chunk in chunk_tasks {
        graph.add_dependency(chunk, merge_task);
    }

    execute_graph(graph, executor, respect_priority, StepStage::NarrowPhase);
    merged
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .take()
        .unwrap_or_default()
}

/// Sequential island sweep, returning per-island state in island order.
pub(crate) fn solve_islands_serial(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[Island],
    delta_time: f32,
) -> Vec<IslandSolveResult> {
    solve_island_chunk(solver, buffers, islands, 0, islands.len(), delta_time)
}

/// Solve constraint islands with the requested dispatcher.
pub(crate) fn solve_islands(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[Island],
    delta_time: f32,
    dispatch: ParallelDispatch<'_>,
) -> Vec<IslandSolveResult> {
    if islands.len() < MIN_PARALLEL_ISLANDS || dispatch.threads() <= 1 {
        return solve_islands_serial(solver, buffers, islands, delta_time);
    }
    match dispatch {
        ParallelDispatch::Scoped(threads) => {
            solve_islands_scoped(solver, buffers, islands, delta_time, threads)
        }
        ParallelDispatch::Pooled(executor, threads, respect_priority) => solve_islands_pooled(
            solver,
            buffers,
            islands,
            delta_time,
            executor,
            threads,
            respect_priority,
        ),
    }
}

fn solve_islands_scoped(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[Island],
    delta_time: f32,
    threads: usize,
) -> Vec<IslandSolveResult> {
    let results: Mutex<Vec<IslandSolveResult>> = Mutex::new(Vec::new());
    crate::base_thread::parallel::parallel_for_chunks(islands.len(), threads, |start, end| {
        let local = solve_island_chunk(solver, buffers, islands, start, end, delta_time);
        if !local.is_empty() {
            results
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .extend(local);
        }
    });
    let mut results = results
        .into_inner()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    sort_islands(&mut results);
    results
}

fn solve_islands_pooled(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[Island],
    delta_time: f32,
    executor: &dyn TaskExecutor,
    threads: usize,
    respect_priority: bool,
) -> Vec<IslandSolveResult> {
    let solver = *solver;
    let solver_bodies = Arc::new(buffers.solver_bodies.to_vec());
    let constraints = Arc::new(buffers.constraints.to_vec());
    let contact_impulses = Arc::new(buffers.contact_impulses.to_vec());
    let solver_joints = Arc::new(buffers.solver_joints.to_vec());
    let joint_impulses = Arc::new(buffers.joint_impulses.to_vec());
    let islands = Arc::new(islands.to_vec());
    let results: Arc<Mutex<Vec<IslandSolveResult>>> = Arc::new(Mutex::new(Vec::new()));
    let merged: Arc<Mutex<Option<Vec<IslandSolveResult>>>> = Arc::new(Mutex::new(None));
    let ranges = chunk_ranges(islands.len(), threads);

    let mut graph = TaskGraph::with_capacity(ranges.len() + 1, ranges.len());
    let mut chunk_tasks = Vec::with_capacity(ranges.len());
    for (start, end) in ranges {
        let solver_bodies = Arc::clone(&solver_bodies);
        let constraints = Arc::clone(&constraints);
        let contact_impulses = Arc::clone(&contact_impulses);
        let solver_joints = Arc::clone(&solver_joints);
        let joint_impulses = Arc::clone(&joint_impulses);
        let islands = Arc::clone(&islands);
        let results = Arc::clone(&results);
        chunk_tasks.push(add_stage_task(
            &mut graph,
            respect_priority,
            Priority::High,
            move || {
                let snapshot = IslandSolveBuffers {
                    solver_bodies: solver_bodies.as_slice(),
                    constraints: constraints.as_slice(),
                    contact_impulses: contact_impulses.as_slice(),
                    solver_joints: solver_joints.as_slice(),
                    joint_impulses: joint_impulses.as_slice(),
                };
                let local =
                    solve_island_chunk(&solver, &snapshot, &islands, start, end, delta_time);
                if !local.is_empty() {
                    results
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .extend(local);
                }
            },
        ));
    }

    let merge_results = Arc::clone(&results);
    let merge_output = Arc::clone(&merged);
    let merge_task = add_stage_task(&mut graph, respect_priority, Priority::Medium, move || {
        let mut rows = merge_results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        sort_islands(&mut rows);
        let solved = rows.drain(..).collect();
        *merge_output
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(solved);
    });
    for chunk in chunk_tasks {
        graph.add_dependency(chunk, merge_task);
    }

    execute_graph(graph, executor, respect_priority, StepStage::IslandSolve);
    merged
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .take()
        .unwrap_or_default()
}
