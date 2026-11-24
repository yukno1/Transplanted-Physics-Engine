//! Simplex solver: motion of a point under a set of moving plane constraints.
//!
//! Direct port of `original/Common/Source/Geometry/Collide/SimplexSolver/
//! hkSimplexSolver.{h,cpp}` (`hkSimplexSolverSolve`). The character controller
//! treats the character as a single point at the origin and each surface
//! contact as a half-space plane; the solver integrates the point along its
//! input velocity, adding planes to an active set as they are hit and
//! re-solving the allowed velocity against the active set with static and
//! dynamic friction until the whole time slice is consumed.
//!
//! Solver conventions (kept identical to the original):
//! * A constraint plane is `xyz = unit normal`, `w = signed distance along the
//!   normal`; a point violates the plane when `pos · normal + w < 0`. The
//!   solver reads the plane distance as `plane.xyz · pos.xyz + pos.w` (the
//!   point's `w` is the plane-constant slot, matching the original
//!   `plane.dot4xyz1(relativePos)` call).
//! * Planes are solved 1d (one active plane), 2d (two planes around their
//!   shared free axis) and 3d (three planes around their triple-product axis).
//!   Nearly parallel planes fall back to sequential 1d solves, and the
//!   `max_surface_velocity` clamp decides when to prefer penetrating a plane
//!   over an extreme simultaneous solve.
//! * Static friction snaps the velocity to the plane velocity when
//!   `|relative|` is small relative to the normal component; dynamic friction
//!   scales the tangential velocity (1.0 = project onto the plane, 0.0 =
//!   conserve input velocity).
//! * `EPS` below matches `hkSimdReal_Eps` (Havok's f32 vector constant table
//!   uses `1e-5`); it only steers degeneracy handling, not the numerics of the
//!   solves.
//!
//! Not ported: the serialization macros (`HK_DECLARE_CLASS`). The
//! `hkSurfaceConstraintInteraction::m_penaltyDistance` field is kept with its
//! original internal semantics.

use crate::base_math::{Matrix3, Vector4};

/// `hkSimdReal_Eps` (f32 build).
const EPS: f32 = 1e-5;
/// Penetration velocity threshold of the 1d `solveTest` probe
/// (`hkSimplexSolverSolveTest1d`, "replace by maxPenetrationVelocity").
const TEST_EPSILON: f32 = 0.001;
/// Smallest walk step considered valid ("must walk more than 10 µs").
const MIN_WALK_TIME: f32 = 1e-4;
/// `HK_REAL_EPSILON` used for the penalty-distance growth.
const REAL_EPSILON: f32 = 1e-5;
/// Safety cap on solver loop iterations. The original relies on the penalty
/// distance growth to terminate; the cap guards against pathological inputs.
const MAX_LOOP_ITERATIONS: usize = 256;

/// Status of one constraint in the solve (`hkSurfaceConstraintInteraction::
/// hkStatus`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InteractionStatus {
    /// The plane was handled without degeneracy.
    Ok,
    /// The 3d solve fell back to 2d sequential solves.
    Failure3d,
    /// The 2d solve fell back to 1d sequential solves.
    Failure2d,
}

impl InteractionStatus {
    /// Ordering used to pick the plane to drop (higher loses).
    fn rank(self) -> i32 {
        match self {
            Self::Ok => 0,
            Self::Failure3d => 1,
            Self::Failure2d => 2,
        }
    }
}

/// Per-constraint solve result (`hkSurfaceConstraintInteraction`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceInteraction {
    /// The solver walked against this plane during this step.
    pub touched: bool,
    /// The solver had to kill this plane's velocity to find a solution.
    pub stopped: bool,
    /// Time spent in contact with the surface.
    pub surface_time: f32,
    /// Internal penalty accumulator for repeatedly penetrating planes.
    pub penalty_distance: f32,
    /// Degeneracy status of the internal solving.
    pub status: InteractionStatus,
}

impl SurfaceInteraction {
    fn reset() -> Self {
        Self {
            touched: false,
            stopped: false,
            surface_time: 0.0,
            penalty_distance: 0.0,
            status: InteractionStatus::Ok,
        }
    }
}

/// A single surface constraint fed to the solver (`hkSurfaceConstraintInfo`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceConstraint {
    /// Plane direction (normally the contact normal); `w` is the signed
    /// distance along the normal.
    pub plane: Vector4,
    /// Linear velocity of the plane.
    pub velocity: Vector4,
    /// Angular velocity of the body represented by the plane (surfaced through
    /// the proxy; not used by the solver itself).
    pub angular_velocity: Vector4,
    /// Static friction (snaps the relative velocity to the plane when small).
    pub static_friction: f32,
    /// Extra static friction in the up direction (box friction split).
    pub extra_up_static_friction: f32,
    /// Extra static friction in the down direction (box friction split).
    pub extra_down_static_friction: f32,
    /// Dynamic friction: 1.0 projects the velocity onto the plane, 0.0 keeps
    /// the input velocity.
    pub dynamic_friction: f32,
    /// Relative priority of the surface (higher wins degenerate cases).
    pub priority: i32,
}

/// Solver input (`hkSimplexSolverInput`).
#[derive(Clone, Debug)]
pub struct SimplexSolverInput {
    /// Position of the particle (the proxy passes the origin).
    pub position: Vector4,
    /// Velocity of the particle.
    pub velocity: Vector4,
    /// Per-component clamp for simultaneous plane solves; a solve exceeding it
    /// falls back to sequential solves (penetrating the lower priority plane).
    pub max_surface_velocity: Vector4,
    /// Up vector used by the box friction model (must be normalized).
    pub up: Vector4,
    /// Time slice to integrate.
    pub delta_time: f32,
    /// Minimum time to walk before exiting early.
    pub min_delta_time: f32,
    /// Surface constraints (at most four become active simultaneously).
    pub constraints: Vec<SurfaceConstraint>,
}

/// Solver output (`hkSimplexSolverOutput`).
#[derive(Clone, Debug)]
pub struct SimplexSolverOutput {
    /// Integrated position (displacement when the input position is zero).
    pub position: Vector4,
    /// Velocity after the last solve.
    pub velocity: Vector4,
    /// Time solved to; smaller than `input.delta_time` when a surface was hit.
    pub delta_time: f32,
    /// One interaction per input constraint, in input order.
    pub interactions: Vec<SurfaceInteraction>,
}

/// Solver scratch state (`hkSimplexSolverInfo`).
struct SolveState {
    /// Indices into `input.constraints` of the active planes.
    planes: [usize; 4],
    num_planes: usize,
    current_time: f32,
}

impl SolveState {
    fn plane_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.planes[..self.num_planes].iter().copied()
    }

    fn contains(&self, index: usize) -> bool {
        self.plane_indices().any(|i| i == index)
    }
}

/// Bubble sort of the active planes by descending (priority, velocity length²)
/// (`hkSimplexSolverSortInfo`).
fn sort_info(input: &SimplexSolverInput, state: &mut SolveState) {
    for i in 0..state.num_planes.saturating_sub(1) {
        for j in (i + 1)..state.num_planes {
            let p0 = input.constraints[state.planes[i]];
            let p1 = input.constraints[state.planes[j]];
            let keep = if p0.priority < p1.priority {
                true
            } else if p0.priority == p1.priority {
                p0.velocity.length_squared3() < p1.velocity.length_squared3()
            } else {
                false
            };
            if !keep {
                state.planes.swap(i, j);
            }
        }
    }
}

/// Whether the plane still blocks `velocity_in` (`hkSimplexSolverSolveTest1d`).
fn solve_test_1d(constraint: &SurfaceConstraint, velocity_in: Vector4) -> bool {
    let relative_velocity = velocity_in - constraint.velocity;
    let plane_vel = relative_velocity.dot3(constraint.plane);
    plane_vel < -TEST_EPSILON
}

/// Solve the velocity against one plane (`hkSimplexSolverSolve1d`).
fn solve_1d(
    input: &SimplexSolverInput,
    constraint: &SurfaceConstraint,
    velocity_in: Vector4,
    velocity_out: &mut Vector4,
) {
    let ground_velocity = constraint.velocity;
    let mut relative_velocity = velocity_in - ground_velocity;
    let plane_vel = relative_velocity.dot3(constraint.plane);
    let orig_velocity2 = relative_velocity.length_squared3();
    relative_velocity -= constraint.plane * plane_vel;
    let vp2 = plane_vel * plane_vel;

    // Static friction is active when the normal component alone explains the
    // relative velocity.
    let extra_static_friction = if relative_velocity.dot3(input.up) > 0.0 {
        constraint.extra_up_static_friction
    } else {
        constraint.extra_down_static_friction
    };
    if extra_static_friction > 0.0 {
        // Box friction: split into a horizontal and a vertical component.
        let horizontal_unnormalized = input.up.cross(constraint.plane);
        let hor2 = horizontal_unnormalized.length_squared3();
        let mut hor_vel = 0.0;
        let mut horizontal = Vector4::ZERO;
        if hor2 > EPS {
            horizontal = horizontal_unnormalized * (1.0 / hor2.sqrt());
            hor_vel = relative_velocity.dot3(horizontal);

            let hor_vel2 = hor_vel * hor_vel;
            let f2 = constraint.static_friction * constraint.static_friction;
            if vp2 * f2 >= hor_vel2 {
                relative_velocity -= horizontal * hor_vel;
                hor_vel = 0.0;
            }
        }

        // Vertical component.
        let vert_vel2 = orig_velocity2 - hor_vel * hor_vel - vp2;
        let f2 = (constraint.static_friction + extra_static_friction)
            * (constraint.static_friction + extra_static_friction);
        if vp2 * f2 >= vert_vel2 {
            if hor_vel == 0.0 {
                *velocity_out = ground_velocity;
                return;
            }
            relative_velocity = horizontal * hor_vel;
        }
    } else {
        // Circle friction: static friction is active when
        // vp² · (1 + f²) ≥ |relative|².
        let f2 = constraint.static_friction * constraint.static_friction;
        if vp2 * (1.0 + f2) >= orig_velocity2 {
            *velocity_out = ground_velocity;
            return;
        }
    }

    // Dynamic friction scales the remaining tangential velocity
    // (0 = conserve input velocity, 1 = clip against the normal).
    if constraint.dynamic_friction < 1.0 {
        let vel_out2 = relative_velocity.length_squared3();
        if vel_out2 >= EPS && vel_out2 > TEST_EPSILON * orig_velocity2 {
            let f = (orig_velocity2 / vel_out2).sqrt();
            let f = constraint.dynamic_friction + (1.0 - constraint.dynamic_friction) * f;
            relative_velocity *= f;
            let p = constraint.plane.dot3(relative_velocity);
            relative_velocity -= constraint.plane * p;
        }
    }
    *velocity_out = relative_velocity + ground_velocity;
}

/// Sequential 1d fallback for two planes (`hkSimplexSolverSolve2dSolve
/// Sequentually`), lower priority plane first.
fn solve_2d_sequential(
    input: &SimplexSolverInput,
    interactions: &mut [SurfaceInteraction],
    index0: usize,
    index1: usize,
    velocity_in: Vector4,
    velocity_out: &mut Vector4,
) {
    interactions[index0].status = InteractionStatus::Failure2d;
    interactions[index1].status = InteractionStatus::Failure2d;
    let sci0 = &input.constraints[index0];
    let sci1 = &input.constraints[index1];
    if sci0.priority > sci1.priority {
        solve_1d(input, sci1, velocity_in, velocity_out);
        solve_1d(input, sci0, *velocity_out, velocity_out);
    } else {
        solve_1d(input, sci0, velocity_in, velocity_out);
        solve_1d(input, sci1, *velocity_out, velocity_out);
    }
}

/// Solve the velocity against two planes (`hkSimplexSolverSolve2d`).
fn solve_2d(
    input: &SimplexSolverInput,
    interactions: &mut [SurfaceInteraction],
    index0: usize,
    index1: usize,
    velocity_in: Vector4,
    velocity_out: &mut Vector4,
) {
    let sci0 = &input.constraints[index0];
    let sci1 = &input.constraints[index1];

    // Free axis shared by both planes.
    let axis_unnormalized = sci0.plane.cross(sci1.plane);
    let axis_len2 = axis_unnormalized.length_squared3();
    if axis_len2 <= EPS {
        // Parallel planes: solve sequentially.
        solve_2d_sequential(
            input,
            interactions,
            index0,
            index1,
            velocity_in,
            velocity_out,
        );
        return;
    }

    let inv_axis_len = 1.0 / axis_len2.sqrt();
    let axis = axis_unnormalized * inv_axis_len;

    // Velocity of the free axis (the velocity both planes agree on).
    let m = Matrix3::from_cols(
        sci0.plane.cross(sci1.plane),
        sci1.plane.cross(axis),
        axis.cross(sci0.plane),
    );
    let s_vel = sci0.velocity + sci1.velocity;
    let t = Vector4::new(
        0.5 * axis.dot3(s_vel),
        sci0.plane.dot3(sci0.velocity),
        sci1.plane.dot3(sci1.velocity),
        0.0,
    );
    let axis_vel = m.mul_vec(t) * inv_axis_len;
    let abs_vel = axis_vel.abs();
    let max_vel = input.max_surface_velocity;
    if abs_vel.x > max_vel.x || abs_vel.y > max_vel.y || abs_vel.z > max_vel.z {
        // Extreme simultaneous solve: fall back to sequential 1d solves.
        solve_2d_sequential(
            input,
            interactions,
            index0,
            index1,
            velocity_in,
            velocity_out,
        );
        return;
    }

    let ground_velocity = axis_vel;
    let relative_velocity = velocity_in - ground_velocity;
    let vel2 = relative_velocity.length_squared3();
    let axis_vert = input.up.dot3(axis);
    let mut axis_proj_velocity = relative_velocity.dot3(axis);

    // Combined friction parameters of both planes.
    let mut static_friction = sci0.static_friction + sci1.static_friction;
    if axis_vert * axis_proj_velocity > 0.0 {
        static_friction +=
            (sci0.extra_up_static_friction + sci1.extra_up_static_friction) * axis_vert;
    } else {
        static_friction +=
            (sci0.extra_down_static_friction + sci1.extra_down_static_friction) * axis_vert;
    }
    static_friction *= 0.5;
    let dynamic_friction = (sci0.dynamic_friction + sci1.dynamic_friction) * 0.5;

    // Static friction along the free axis:
    // (vel2 − av2) · f² ≥ av2.
    let f2 = static_friction * static_friction;
    let av2 = axis_proj_velocity * axis_proj_velocity;
    if (vel2 - av2) * f2 >= av2 {
        *velocity_out = ground_velocity;
        return;
    }

    // Dynamic friction along the free axis.
    if dynamic_friction < 1.0 && av2 > TEST_EPSILON * vel2 {
        let f = (1.0 / axis_proj_velocity).abs() * vel2.sqrt();
        let f = f * (1.0 - dynamic_friction) + dynamic_friction;
        axis_proj_velocity *= f;
    }
    *velocity_out = ground_velocity + axis * axis_proj_velocity;
}

/// Sequential 2d fallback shared by the degenerate 3d branches
/// (`hkSimplexSolverSolve3dSolveSequentually`).
fn solve_3d_sequential(
    input: &SimplexSolverInput,
    interactions: &mut [SurfaceInteraction],
    indices: [usize; 3],
    velocity_in: Vector4,
    velocity_out: &mut Vector4,
) {
    for index in indices {
        interactions[index].status = InteractionStatus::Failure3d;
    }
    solve_2d(
        input,
        interactions,
        indices[0],
        indices[1],
        velocity_in,
        velocity_out,
    );
    solve_2d(
        input,
        interactions,
        indices[0],
        indices[2],
        *velocity_out,
        velocity_out,
    );
    solve_2d(
        input,
        interactions,
        indices[1],
        indices[2],
        *velocity_out,
        velocity_out,
    );
}

/// Solve the velocity against three planes (`hkSimplexSolverSolve3d`).
fn solve_3d(
    input: &SimplexSolverInput,
    interactions: &mut [SurfaceInteraction],
    indices: [usize; 3],
    allow_resort: bool,
    state: &mut SolveState,
    velocity_in: Vector4,
    velocity_out: &mut Vector4,
) {
    let sci0 = &input.constraints[indices[0]];
    let sci1 = &input.constraints[indices[1]];
    let sci2 = &input.constraints[indices[2]];

    let m = Matrix3::from_cols(
        sci1.plane.cross(sci2.plane),
        sci2.plane.cross(sci0.plane),
        sci0.plane.cross(sci1.plane),
    );
    let det = m.col0.dot3(sci0.plane);
    let degenerate = det.abs() < EPS;
    if !degenerate {
        let t = Vector4::new(
            sci0.plane.dot3(sci0.velocity),
            sci1.plane.dot3(sci1.velocity),
            sci2.plane.dot3(sci2.velocity),
            0.0,
        );
        let point_vel = m.mul_vec(t) * (1.0 / det);
        let abs_vel = point_vel.abs();
        let max_vel = input.max_surface_velocity;
        if !(abs_vel.x > max_vel.x || abs_vel.y > max_vel.y || abs_vel.z > max_vel.z) {
            *velocity_out = point_vel;
            return;
        }
    }

    // Degenerate determinant or extreme solve: sort (if allowed) and solve
    // the plane pairs sequentially.
    if allow_resort {
        sort_info(input, state);
        let sorted = [state.planes[0], state.planes[1], state.planes[2]];
        solve_3d_sequential(input, interactions, sorted, velocity_in, velocity_out);
    } else {
        solve_3d_sequential(input, interactions, indices, velocity_in, velocity_out);
    }
}

/// Examine the active plane set and solve the velocity
/// (`hkSimplexSolverExamineActivePlanes`).
fn examine_active_planes(
    input: &SimplexSolverInput,
    state: &mut SolveState,
    interactions: &mut [SurfaceInteraction],
    velocity_out: &mut Vector4,
) {
    loop {
        let indices: [usize; 4] = {
            let mut buffer = [usize::MAX; 4];
            buffer[..state.num_planes].copy_from_slice(&state.planes[..state.num_planes]);
            buffer
        };
        match state.num_planes {
            1 => {
                solve_1d(
                    input,
                    &input.constraints[indices[0]],
                    input.velocity,
                    velocity_out,
                );
                return;
            }
            2 => {
                // Test whether plane 0 is still needed.
                let mut velocity = Vector4::ZERO;
                solve_1d(
                    input,
                    &input.constraints[indices[1]],
                    input.velocity,
                    &mut velocity,
                );
                if !solve_test_1d(&input.constraints[indices[0]], velocity) {
                    // Only plane 1 is needed: drop plane 0.
                    *velocity_out = velocity;
                    state.planes[0] = indices[1];
                    state.num_planes = 1;
                } else {
                    solve_2d(
                        input,
                        interactions,
                        indices[0],
                        indices[1],
                        input.velocity,
                        velocity_out,
                    );
                }
                return;
            }
            3 => {
                // Try to drop planes 0 and 1 at once.
                {
                    let mut velocity = Vector4::ZERO;
                    solve_1d(
                        input,
                        &input.constraints[indices[2]],
                        input.velocity,
                        &mut velocity,
                    );
                    if !solve_test_1d(&input.constraints[indices[0]], velocity)
                        && !solve_test_1d(&input.constraints[indices[1]], velocity)
                    {
                        *velocity_out = velocity;
                        state.planes[0] = indices[2];
                        state.num_planes = 1;
                        continue;
                    }
                }

                // Try to drop plane 0 or 1.
                let mut dropped = false;
                for test_plane in 0..2usize {
                    let mut velocity = Vector4::ZERO;
                    solve_2d(
                        input,
                        interactions,
                        indices[test_plane],
                        indices[2],
                        input.velocity,
                        &mut velocity,
                    );
                    let other = indices[1 - test_plane];
                    if !solve_test_1d(&input.constraints[other], velocity) {
                        state.planes[0] = indices[test_plane];
                        state.planes[1] = indices[2];
                        state.num_planes = 2;
                        dropped = true;
                        break;
                    }
                }
                if dropped {
                    continue;
                }
                // Solve all three planes.
                solve_3d(
                    input,
                    interactions,
                    [indices[0], indices[1], indices[2]],
                    true,
                    state,
                    input.velocity,
                    velocity_out,
                );
                return;
            }
            4 => {
                sort_info(input, state);
                let mut dropped = false;
                for i in 0..3usize {
                    let mut velocity = Vector4::ZERO;
                    solve_3d(
                        input,
                        interactions,
                        [indices[(i + 1) % 3], indices[(i + 2) % 3], indices[3]],
                        false,
                        state,
                        input.velocity,
                        &mut velocity,
                    );
                    if !solve_test_1d(&input.constraints[indices[i]], velocity) {
                        state.planes[i] = indices[2];
                        state.planes[2] = indices[3];
                        state.num_planes = 3;
                        dropped = true;
                        break;
                    }
                }
                if dropped {
                    continue;
                }
                // Nothing can be dropped: try all 3d combinations.
                let mut velocity = input.velocity;
                for combo in [
                    [indices[0], indices[1], indices[2]],
                    [indices[0], indices[1], indices[3]],
                    [indices[0], indices[2], indices[3]],
                    [indices[1], indices[2], indices[3]],
                ] {
                    solve_3d(
                        input,
                        interactions,
                        combo,
                        false,
                        state,
                        velocity,
                        &mut velocity,
                    );
                }
                *velocity_out = velocity;

                // Drop the plane with the highest penalty status.
                let max_status = (0..4)
                    .map(|j| interactions[indices[j]].status.rank())
                    .max()
                    .unwrap_or(0);
                for k in 0..4usize {
                    if interactions[indices[k]].status.rank() == max_status {
                        state.planes[k] = indices[3];
                        break;
                    }
                }
                state.num_planes = 3;
                for l in 0..3usize {
                    interactions[state.planes[l]].status = InteractionStatus::Ok;
                }
                continue;
            }
            _ => return,
        }
    }
}

/// Solve the motion of a particle under the given plane constraints
/// (`hkSimplexSolverSolve`).
pub fn solve(input: &SimplexSolverInput) -> SimplexSolverOutput {
    let state = SolveState {
        planes: [usize::MAX; 4],
        num_planes: 0,
        current_time: 0.0,
    };
    solve_inner(input, state)
}

fn solve_inner(input: &SimplexSolverInput, mut state: SolveState) -> SimplexSolverOutput {
    let mut interactions: Vec<SurfaceInteraction> = input
        .constraints
        .iter()
        .map(|_| SurfaceInteraction::reset())
        .collect();

    let mut remaining_time = input.delta_time;
    let mut velocity = input.velocity;
    let mut position = input.position;
    // `output.m_deltaTime` in the original is only assigned inside the walk
    // step or on completion; start from zero to mirror that.
    let mut delta_time = 0.0;

    let mut iterations = 0usize;
    while remaining_time >= 0.0 {
        iterations += 1;
        if iterations > MAX_LOOP_ITERATIONS {
            break;
        }

        // Search for the plane that the current velocity hits first.
        let mut hit_index: Option<usize> = None;
        let mut min_collision_time = remaining_time;

        for i in 0..input.constraints.len() {
            // Skip already active planes and planes with a failed status.
            if state.contains(i) || interactions[i].status != InteractionStatus::Ok {
                continue;
            }

            let constraint = &input.constraints[i];
            let relative_vel = velocity - constraint.velocity;
            let relative_projected_vel = -relative_vel.dot3(constraint.plane);

            // Skip planes the velocity moves away from.
            if relative_projected_vel <= 0.0 {
                continue;
            }

            // Distance to the plane along the normal (`plane.dot4xyz1(pos)`):
            // the point's `w` provides the plane-constant slot.
            let relative_pos = position - constraint.velocity * state.current_time;
            let mut projected_pos = constraint.plane.dot3(relative_pos) + relative_pos.w;

            // Treat penetrations: an (almost) touching plane is hit at once.
            if projected_pos <= EPS {
                projected_pos = 0.0;
            }
            projected_pos += interactions[i].penalty_distance;

            if projected_pos < min_collision_time * relative_projected_vel {
                min_collision_time = projected_pos / relative_projected_vel;
                hit_index = Some(i);
            }
        }

        // Walk to the hit position; skip very short steps.
        if min_collision_time > MIN_WALK_TIME {
            state.current_time += min_collision_time;
            remaining_time -= min_collision_time;
            position += velocity * min_collision_time;
            for plane_index in state.plane_indices() {
                interactions[plane_index].surface_time += min_collision_time;
                interactions[plane_index].touched = true;
            }
            delta_time = state.current_time;

            // Early exit once the minimum walk time is reached.
            if state.current_time > input.min_delta_time {
                return SimplexSolverOutput {
                    position,
                    velocity,
                    delta_time,
                    interactions,
                };
            }
        }

        let Some(hit_index) = hit_index else {
            // No new plane hit: the full time slice can be walked.
            return SimplexSolverOutput {
                position,
                velocity,
                delta_time: input.delta_time,
                interactions,
            };
        };

        // Activate the hit plane and grow its penetration penalty.
        state.planes[state.num_planes] = hit_index;
        state.num_planes += 1;
        let interaction = &mut interactions[hit_index];
        interaction.penalty_distance = (interaction.penalty_distance + REAL_EPSILON) * 2.0;

        // Move along the current active plane set (updates the velocity).
        examine_active_planes(input, &mut state, &mut interactions, &mut velocity);

        if remaining_time <= 0.0 {
            break;
        }
    }

    SimplexSolverOutput {
        position,
        velocity,
        delta_time,
        interactions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(normal: Vector4, distance: f32) -> SurfaceConstraint {
        SurfaceConstraint {
            plane: Vector4::new(normal.x, normal.y, normal.z, distance),
            velocity: Vector4::ZERO,
            angular_velocity: Vector4::ZERO,
            static_friction: 0.0,
            extra_up_static_friction: 0.0,
            extra_down_static_friction: 0.0,
            dynamic_friction: 1.0,
            priority: 0,
        }
    }

    fn input(velocity: Vector4, constraints: Vec<SurfaceConstraint>) -> SimplexSolverInput {
        SimplexSolverInput {
            position: Vector4::ZERO,
            velocity,
            max_surface_velocity: Vector4::new(10.0, 10.0, 10.0, 0.0),
            up: Vector4::UNIT_Y,
            delta_time: 1.0 / 60.0,
            min_delta_time: 1.0 / 60.0,
            constraints,
        }
    }

    #[test]
    fn free_motion_is_unchanged() {
        let velocity = Vector4::from_xyz(1.0, 2.0, 3.0);
        let output = solve(&input(velocity, Vec::new()));
        assert!(output.velocity.all_equal3(velocity, 1e-6));
        assert_eq!(output.delta_time, 1.0 / 60.0);
        let expected = velocity * (1.0 / 60.0);
        assert!(output.position.all_equal3(expected, 1e-6));
    }

    #[test]
    fn floor_plane_stops_downward_motion() {
        let floor = plane(Vector4::UNIT_Y, 0.0);
        let velocity = Vector4::from_xyz(2.0, -4.0, 0.0);
        let output = solve(&input(velocity, vec![floor]));
        assert!(
            output.velocity.y > -1e-4,
            "downward velocity must be killed: {:?}",
            output.velocity
        );
        assert!(
            (output.velocity.x - 2.0).abs() < 1e-3,
            "tangential velocity is preserved: {:?}",
            output.velocity
        );
        assert!(output.interactions[0].touched);
        // The plane is hit at time 0 (the point starts on it), so the full
        // time slice is walked tangentially from the plane.
        assert_eq!(output.delta_time, 1.0 / 60.0);
        let expected = output.velocity * (1.0 / 60.0);
        assert!(output.position.all_equal3(expected, 1e-6));
    }

    #[test]
    fn two_walls_form_a_corridor() {
        let left = plane(Vector4::from_xyz(1.0, 0.0, 0.0), 0.0);
        let right = plane(Vector4::from_xyz(-1.0, 0.0, 0.0), 0.0);
        let velocity = Vector4::from_xyz(5.0, 0.0, 1.0);
        let output = solve(&input(velocity, vec![left, right]));
        assert!(output.velocity.x.abs() < 1e-4, "{:?}", output.velocity);
        assert!(
            (output.velocity.z - 1.0).abs() < 1e-3,
            "{:?}",
            output.velocity
        );
    }

    #[test]
    fn corner_blocks_all_motion() {
        let floor = plane(Vector4::UNIT_Y, 0.0);
        let left = plane(Vector4::from_xyz(1.0, 0.0, 0.0), 0.0);
        let back = plane(Vector4::from_xyz(0.0, 0.0, 1.0), 0.0);
        let velocity = Vector4::from_xyz(-5.0, -1.0, -5.0);
        let output = solve(&input(velocity, vec![floor, left, back]));
        assert!(output.velocity.x > -1e-4, "{:?}", output.velocity);
        assert!(output.velocity.z > -1e-4, "{:?}", output.velocity);
        assert!(output.velocity.y > -1e-4, "{:?}", output.velocity);
    }

    #[test]
    fn static_friction_snaps_to_rest() {
        let mut floor = plane(Vector4::UNIT_Y, 0.0);
        floor.static_friction = 1.0;
        let velocity = Vector4::from_xyz(0.05, -0.05, 0.0);
        let output = solve(&input(velocity, vec![floor]));
        assert!(
            output.velocity.length_squared3() < 1e-6,
            "velocity should be zeroed by static friction: {:?}",
            output.velocity
        );
    }

    #[test]
    fn moving_plane_carries_the_character() {
        let mut belt = plane(Vector4::UNIT_Y, 0.0);
        // Static friction is required for the plane velocity to carry the
        // character (frictionless surfaces only clip the normal component).
        belt.static_friction = 1.0;
        belt.velocity = Vector4::from_xyz(3.0, 0.0, 0.0);
        let velocity = Vector4::from_xyz(0.0, -4.0, 0.0);
        let output = solve(&input(velocity, vec![belt]));
        assert!(
            (output.velocity.x - 3.0).abs() < 1e-3,
            "character should ride the moving plane: {:?}",
            output.velocity
        );
    }

    #[test]
    fn duplicate_planes_behave_like_single_plane() {
        // Two identical planes: the first hit projects the velocity, after
        // which the duplicate is never approached again (the parallel-plane
        // sequential fallback only triggers for simultaneously hit planes).
        let a = plane(Vector4::UNIT_Y, 0.0);
        let b = plane(Vector4::UNIT_Y, 0.0);
        let velocity = Vector4::from_xyz(1.0, -2.0, 0.0);
        let output = solve(&input(velocity, vec![a, b]));
        assert!(output.velocity.y > -1e-4);
        assert!((output.velocity.x - 1.0).abs() < 1e-3);
        assert!(output.interactions[0].touched);
        assert!(!output.interactions[1].touched);
    }

    #[test]
    fn solve_is_deterministic() {
        let constraints = vec![
            plane(Vector4::UNIT_Y, 0.0),
            plane(Vector4::from_xyz(0.70710677, 0.0, 0.70710677), 0.0),
            plane(Vector4::from_xyz(-0.70710677, 0.0, 0.70710677), 0.0),
        ];
        let velocity = Vector4::from_xyz(2.0, -3.0, -1.0);
        let a = solve(&input(velocity, constraints.clone()));
        let b = solve(&input(velocity, constraints));
        assert_eq!(a.velocity.to_array(), b.velocity.to_array());
        assert_eq!(a.position.to_array(), b.position.to_array());
    }
}
