//! Deterministic per-step scheduling for the physics world.
//!
//! Rust counterpart of the `hknpTaskGraph` scheduling layer used by
//! `hknpMultithreadedSimulation`. Independent compute stages are dispatched
//! here: broad-phase candidate narrow phase, per-island solver sweeps,
//! contact-row finalization after serial modifier/hook callbacks, and
//! write-back/sleep-timer candidate computation. Stages that mutate the world
//! (gravity, event dispatch, warm-start application, sleep decisions and
//! integration) stay on the calling thread in the original order, so
//! `hvk-trace v1` remains bit-identical.
//!
//! A pooled step uses a fresh [`TaskGraph`] per stage: chunk tasks write to
//! indexed result slots and a dependent merge task restores stable candidate /
//! island order. The legacy scoped dispatch keeps the previous API behaviour
//! and shares the same chunk and merge helpers.

use std::sync::{Arc, Mutex};

use crate::base_math::{Transform, Vec4, Vec4Ext};
use crate::phys_shape::{to_vec4, to_vector4};
use crate::base_thread::{Priority, TaskExecutor, TaskGraph};
use crate::phys_solver::{ContactConstraint, ContactImpulse, Solver, SolverBody, contact_tangents};
use crate::phys_types::{BodyId, MotionId};
use crate::phys_world::ContactManifold;
use crate::phys_world::cache::ContactKey;
use crate::phys_world::events::{PostCollideContactPoint, PostCollideEvent};
use crate::phys_world::island::Island;
use crate::phys_world::world::{
    ContactCandidate, IslandSolveBuffers, IslandSolveResult, collide_candidate,
    contacts_from_candidates,
};

/// Candidate count below which a scoped narrow phase costs more than it saves.
pub(crate) const MIN_PARALLEL_CANDIDATES: usize = 32;
/// Island count below which the sequential solver sweep is used.
pub(crate) const MIN_PARALLEL_ISLANDS: usize = 2;
/// Contact manifold count below which row construction stays serial.
pub(crate) const MIN_PARALLEL_CONTACT_ROWS: usize = 16;
/// Body count below which write-back candidate computation stays serial.
pub(crate) const MIN_PARALLEL_WRITEBACK: usize = 32;

/// Stage name carried into panic messages so a worker failure is locatable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StepStage {
    NarrowPhase,
    ContactRows,
    IslandSolve,
    Writeback,
}

impl StepStage {
    fn panic_message(self) -> &'static str {
        match self {
            Self::NarrowPhase => "narrow-phase task graph contains a cycle",
            Self::ContactRows => "contact-row task graph contains a cycle",
            Self::IslandSolve => "island-solve task graph contains a cycle",
            Self::Writeback => "writeback task graph contains a cycle",
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

/// One contact point after serial modifier/hook callbacks but before final
/// normalization and solver-row construction.
#[derive(Clone, Copy)]
pub(crate) struct PreparedContactPoint {
    pub(crate) position: Vec4,
    pub(crate) normal: Vec4,
    pub(crate) penetration: f32,
    pub(crate) friction: f32,
    pub(crate) normal_impulse: f32,
    pub(crate) tangent_impulse_1: f32,
    pub(crate) tangent_impulse_2: f32,
}

/// One manifold worth of prepared contact points.
#[derive(Clone)]
pub(crate) struct PreparedContactRowGroup {
    pub(crate) body_a: BodyId,
    pub(crate) body_b: BodyId,
    pub(crate) solver_a: usize,
    pub(crate) solver_b: usize,
    pub(crate) transform_b: Transform,
    pub(crate) step_number: u64,
    pub(crate) delta_time: f32,
    pub(crate) dropped: usize,
    pub(crate) points: Vec<PreparedContactPoint>,
}

struct ContactRowChunk {
    index: usize,
    constraints: Vec<ContactConstraint>,
    contact_keys: Vec<ContactKey>,
    contact_impulses: Vec<ContactImpulse>,
    events: Vec<PostCollideEvent>,
}

fn finalize_contact_rows_chunk(
    groups: &[PreparedContactRowGroup],
    start: usize,
    end: usize,
) -> ContactRowChunk {
    let point_count: usize = groups[start..end]
        .iter()
        .map(|group| group.points.len())
        .sum();
    let mut chunk = ContactRowChunk {
        index: start,
        constraints: Vec::with_capacity(point_count),
        contact_keys: Vec::with_capacity(point_count),
        contact_impulses: Vec::with_capacity(point_count),
        events: Vec::with_capacity(end - start),
    };

    for group in &groups[start..end] {
        let mut points = Vec::with_capacity(group.points.len());
        for point in &group.points {
            let mut normal = point.normal;
            if normal.length3() > 1e-10 {
                normal = normal.normalize_or_zero3();
            }
            let (tangent_1, tangent_2) = contact_tangents(normal);
            let normal_impulse = point.normal_impulse.max(0.0);
            let limit = point.friction.max(0.0) * normal_impulse;
            let tangent_impulse_1 = point.tangent_impulse_1.clamp(-limit, limit);
            let tangent_impulse_2 = point.tangent_impulse_2.clamp(-limit, limit);
            let penetration = point.penetration.max(0.0);
            let constraint_point = point.position - normal * (penetration * 0.5);

            chunk.constraints.push(ContactConstraint::new(
                group.solver_a,
                group.solver_b,
                normal,
                constraint_point,
                penetration,
                point.friction,
            ));
            chunk.contact_keys.push(ContactKey {
                body_a: group.body_a,
                body_b: group.body_b,
                local_point_b: to_vec4(
                    group.transform_b.inverse_transform_point(to_vector4(point.position)),
                ),
                normal,
            });
            chunk.contact_impulses.push(ContactImpulse {
                normal: normal_impulse,
                tangent_1: tangent_impulse_1,
                tangent_2: tangent_impulse_2,
            });
            points.push(PostCollideContactPoint {
                position: point.position,
                normal,
                penetration,
                friction: point.friction,
            });
            let _ = (tangent_1, tangent_2);
        }
        chunk.events.push(PostCollideEvent {
            step_count: group.step_number,
            delta_time: group.delta_time,
            body_a: group.body_a,
            body_b: group.body_b,
            points,
            dropped_point_count: group.dropped,
        });
    }
    chunk
}

fn merge_contact_chunks(
    mut chunks: Vec<ContactRowChunk>,
) -> (
    Vec<ContactConstraint>,
    Vec<ContactKey>,
    Vec<ContactImpulse>,
    Vec<PostCollideEvent>,
) {
    chunks.sort_unstable_by_key(|chunk| chunk.index);
    let mut constraints = Vec::new();
    let mut contact_keys = Vec::new();
    let mut contact_impulses = Vec::new();
    let mut events = Vec::new();
    for chunk in chunks {
        constraints.extend(chunk.constraints);
        contact_keys.extend(chunk.contact_keys);
        contact_impulses.extend(chunk.contact_impulses);
        events.extend(chunk.events);
    }
    (constraints, contact_keys, contact_impulses, events)
}

/// Build solver contact rows from already-prepared manifold snapshots.
///
/// Modifier and hook callbacks must have run on the caller thread before this
/// stage so their global registration-order side effects stay unchanged.
pub(crate) fn build_contact_rows(
    groups: &[PreparedContactRowGroup],
    dispatch: ParallelDispatch<'_>,
) -> (
    Vec<ContactConstraint>,
    Vec<ContactKey>,
    Vec<ContactImpulse>,
    Vec<PostCollideEvent>,
) {
    if groups.len() < MIN_PARALLEL_CONTACT_ROWS || dispatch.threads() <= 1 {
        return merge_contact_chunks(vec![finalize_contact_rows_chunk(groups, 0, groups.len())]);
    }
    match dispatch {
        ParallelDispatch::Scoped(threads) => build_contact_rows_scoped(groups, threads),
        ParallelDispatch::Pooled(executor, threads, respect_priority) => {
            build_contact_rows_pooled(groups, executor, threads, respect_priority)
        }
    }
}

fn build_contact_rows_scoped(
    groups: &[PreparedContactRowGroup],
    threads: usize,
) -> (
    Vec<ContactConstraint>,
    Vec<ContactKey>,
    Vec<ContactImpulse>,
    Vec<PostCollideEvent>,
) {
    let results: Mutex<Vec<ContactRowChunk>> = Mutex::new(Vec::new());
    crate::base_thread::parallel::parallel_for_chunks(groups.len(), threads, |start, end| {
        let chunk = finalize_contact_rows_chunk(groups, start, end);
        results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(chunk);
    });
    merge_contact_chunks(
        results
            .into_inner()
            .unwrap_or_else(|poisoned| poisoned.into_inner()),
    )
}

fn build_contact_rows_pooled(
    groups: &[PreparedContactRowGroup],
    executor: &dyn TaskExecutor,
    threads: usize,
    respect_priority: bool,
) -> (
    Vec<ContactConstraint>,
    Vec<ContactKey>,
    Vec<ContactImpulse>,
    Vec<PostCollideEvent>,
) {
    let groups = Arc::new(groups.to_vec());
    let results: Arc<Mutex<Vec<ContactRowChunk>>> = Arc::new(Mutex::new(Vec::new()));
    let merged: Arc<Mutex<Option<Vec<ContactRowChunk>>>> = Arc::new(Mutex::new(None));
    let ranges = chunk_ranges(groups.len(), threads);

    let mut graph = TaskGraph::with_capacity(ranges.len() + 1, ranges.len());
    let mut chunk_tasks = Vec::with_capacity(ranges.len());
    for (start, end) in ranges {
        let groups = Arc::clone(&groups);
        let results = Arc::clone(&results);
        chunk_tasks.push(add_stage_task(
            &mut graph,
            respect_priority,
            Priority::High,
            move || {
                let chunk = finalize_contact_rows_chunk(&groups, start, end);
                results
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .push(chunk);
            },
        ));
    }
    let merge_results = Arc::clone(&results);
    let merge_output = Arc::clone(&merged);
    let merge_task = add_stage_task(&mut graph, respect_priority, Priority::Medium, move || {
        let mut chunks = merge_results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        chunks.sort_unstable_by_key(|chunk| chunk.index);
        *merge_output
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(std::mem::take(&mut *chunks));
    });
    for chunk in chunk_tasks {
        graph.add_dependency(chunk, merge_task);
    }
    execute_graph(graph, executor, respect_priority, StepStage::ContactRows);
    merge_contact_chunks(
        merged
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take()
            .unwrap_or_default(),
    )
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

#[derive(Clone, Copy)]
pub(crate) struct VelocityWritebackCandidate {
    pub(crate) motion_id: MotionId,
    pub(crate) linear_velocity: Vec4,
    pub(crate) angular_velocity: Vec4,
}

#[derive(Clone, Copy)]
pub(crate) struct SleepTimerCandidate {
    pub(crate) body_id: BodyId,
    pub(crate) low_energy: bool,
}

struct WritebackChunk {
    index: usize,
    velocities: Vec<VelocityWritebackCandidate>,
    timers: Vec<SleepTimerCandidate>,
}

#[allow(clippy::too_many_arguments)]
fn compute_writeback_chunk(
    solver_bodies: &[SolverBody],
    body_ids: &[BodyId],
    motion_ids: &[Option<MotionId>],
    is_dynamic: &[bool],
    sleeping: &[bool],
    linear_threshold_sq: f32,
    angular_threshold_sq: f32,
    collect_timers: bool,
    start: usize,
    end: usize,
) -> WritebackChunk {
    let mut chunk = WritebackChunk {
        index: start,
        velocities: Vec::new(),
        timers: Vec::new(),
    };
    for index in start..end {
        let solved = solver_bodies[index];
        if solved.inverse_mass > 0.0 {
            if let Some(motion_id) = motion_ids[index] {
                chunk.velocities.push(VelocityWritebackCandidate {
                    motion_id,
                    linear_velocity: solved.linear_velocity,
                    angular_velocity: solved.angular_velocity,
                });
            }
        }
        if collect_timers && is_dynamic[index] && !sleeping[index] {
            let low_energy = solved.linear_velocity.dot3(solved.linear_velocity)
                < linear_threshold_sq
                && solved.angular_velocity.dot3(solved.angular_velocity) < angular_threshold_sq;
            chunk.timers.push(SleepTimerCandidate {
                body_id: body_ids[index],
                low_energy,
            });
        }
    }
    chunk
}

fn merge_writeback_chunks(
    mut chunks: Vec<WritebackChunk>,
) -> (Vec<VelocityWritebackCandidate>, Vec<SleepTimerCandidate>) {
    chunks.sort_unstable_by_key(|chunk| chunk.index);
    let mut velocities = Vec::new();
    let mut timers = Vec::new();
    for chunk in chunks {
        velocities.extend(chunk.velocities);
        timers.extend(chunk.timers);
    }
    (velocities, timers)
}

/// Compute solver write-back and sleep-timer candidates without mutating world
/// state. Candidates are returned in ascending body index order for serial
/// application by the caller.
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_writeback(
    solver_bodies: &[SolverBody],
    body_ids: &[BodyId],
    motion_ids: &[Option<MotionId>],
    is_dynamic: &[bool],
    sleeping: &[bool],
    linear_threshold: f32,
    angular_threshold: f32,
    collect_timers: bool,
    dispatch: ParallelDispatch<'_>,
) -> (Vec<VelocityWritebackCandidate>, Vec<SleepTimerCandidate>) {
    let linear_threshold_sq = linear_threshold * linear_threshold;
    let angular_threshold_sq = angular_threshold * angular_threshold;
    if body_ids.len() < MIN_PARALLEL_WRITEBACK || dispatch.threads() <= 1 {
        return merge_writeback_chunks(vec![compute_writeback_chunk(
            solver_bodies,
            body_ids,
            motion_ids,
            is_dynamic,
            sleeping,
            linear_threshold_sq,
            angular_threshold_sq,
            collect_timers,
            0,
            body_ids.len(),
        )]);
    }
    match dispatch {
        ParallelDispatch::Scoped(threads) => compute_writeback_scoped(
            solver_bodies,
            body_ids,
            motion_ids,
            is_dynamic,
            sleeping,
            linear_threshold_sq,
            angular_threshold_sq,
            collect_timers,
            threads,
        ),
        ParallelDispatch::Pooled(executor, threads, respect_priority) => compute_writeback_pooled(
            solver_bodies,
            body_ids,
            motion_ids,
            is_dynamic,
            sleeping,
            linear_threshold_sq,
            angular_threshold_sq,
            collect_timers,
            executor,
            threads,
            respect_priority,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn compute_writeback_scoped(
    solver_bodies: &[SolverBody],
    body_ids: &[BodyId],
    motion_ids: &[Option<MotionId>],
    is_dynamic: &[bool],
    sleeping: &[bool],
    linear_threshold_sq: f32,
    angular_threshold_sq: f32,
    collect_timers: bool,
    threads: usize,
) -> (Vec<VelocityWritebackCandidate>, Vec<SleepTimerCandidate>) {
    let results: Mutex<Vec<WritebackChunk>> = Mutex::new(Vec::new());
    crate::base_thread::parallel::parallel_for_chunks(body_ids.len(), threads, |start, end| {
        let chunk = compute_writeback_chunk(
            solver_bodies,
            body_ids,
            motion_ids,
            is_dynamic,
            sleeping,
            linear_threshold_sq,
            angular_threshold_sq,
            collect_timers,
            start,
            end,
        );
        results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(chunk);
    });
    merge_writeback_chunks(
        results
            .into_inner()
            .unwrap_or_else(|poisoned| poisoned.into_inner()),
    )
}

#[allow(clippy::too_many_arguments)]
fn compute_writeback_pooled(
    solver_bodies: &[SolverBody],
    body_ids: &[BodyId],
    motion_ids: &[Option<MotionId>],
    is_dynamic: &[bool],
    sleeping: &[bool],
    linear_threshold_sq: f32,
    angular_threshold_sq: f32,
    collect_timers: bool,
    executor: &dyn TaskExecutor,
    threads: usize,
    respect_priority: bool,
) -> (Vec<VelocityWritebackCandidate>, Vec<SleepTimerCandidate>) {
    let solver_bodies = Arc::new(solver_bodies.to_vec());
    let body_ids = Arc::new(body_ids.to_vec());
    let motion_ids = Arc::new(motion_ids.to_vec());
    let is_dynamic = Arc::new(is_dynamic.to_vec());
    let sleeping = Arc::new(sleeping.to_vec());
    let results: Arc<Mutex<Vec<WritebackChunk>>> = Arc::new(Mutex::new(Vec::new()));
    let merged: Arc<Mutex<Option<Vec<WritebackChunk>>>> = Arc::new(Mutex::new(None));
    let ranges = chunk_ranges(body_ids.len(), threads);

    let mut graph = TaskGraph::with_capacity(ranges.len() + 1, ranges.len());
    let mut chunk_tasks = Vec::with_capacity(ranges.len());
    for (start, end) in ranges {
        let solver_bodies = Arc::clone(&solver_bodies);
        let body_ids = Arc::clone(&body_ids);
        let motion_ids = Arc::clone(&motion_ids);
        let is_dynamic = Arc::clone(&is_dynamic);
        let sleeping = Arc::clone(&sleeping);
        let results = Arc::clone(&results);
        chunk_tasks.push(add_stage_task(
            &mut graph,
            respect_priority,
            Priority::High,
            move || {
                let chunk = compute_writeback_chunk(
                    &solver_bodies,
                    &body_ids,
                    &motion_ids,
                    &is_dynamic,
                    &sleeping,
                    linear_threshold_sq,
                    angular_threshold_sq,
                    collect_timers,
                    start,
                    end,
                );
                results
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .push(chunk);
            },
        ));
    }
    let merge_results = Arc::clone(&results);
    let merge_output = Arc::clone(&merged);
    let merge_task = add_stage_task(&mut graph, respect_priority, Priority::Medium, move || {
        let mut chunks = merge_results
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        chunks.sort_unstable_by_key(|chunk| chunk.index);
        *merge_output
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(std::mem::take(&mut *chunks));
    });
    for chunk in chunk_tasks {
        graph.add_dependency(chunk, merge_task);
    }
    execute_graph(graph, executor, respect_priority, StepStage::Writeback);
    merge_writeback_chunks(
        merged
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take()
            .unwrap_or_default(),
    )
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
