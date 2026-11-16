//! Joint constraints solved with sequential impulses.
//!
//! Subset of `original/Physics/Source/Physics/Physics/Dynamics/Constraint/Data/`
//! (`BallAndSocket`, `Hinge`, `Fixed`) as solver-level constraints
//! (migration follow-up to Phase 8, README §4).
//!
//! Unlike the SDK's constraint-atom system, these joints are solved directly
//! from world-space data computed by the world each step: pivots and axes are
//! already transformed, and orientation errors are supplied as world-space
//! axis-angle vectors. The solver applies plain sequential impulses:
//!
//! * **Linear** (all joints): the two pivots are pulled together along the
//!   world `X`/`Y`/`Z` axes with a Baumgarte position bias derived from the
//!   current pivot separation.
//! * **Angular** ([`HingeJoint`]): the two hinge axes are kept aligned by
//!   constraining relative angular velocity about two axes perpendicular to
//!   the hinge, biased by the current axis misalignment.
//! * **Angular** ([`FixedJoint`]): relative angular velocity is zeroed about
//!   the world axes, biased by the supplied orientation-error vector.
//! * **Hinge limits** ([`HingeJoint::limits`]): a unilateral constraint that
//!   only pushes the relative angular velocity back when the hinge angle is
//!   outside `[min_angle, max_angle]` (Baumgarte bias on the violated bound).
//! * **Hinge motors** ([`HingeJoint::motor`]): velocity, position and spring
//!   motors about the hinge axis applied once per solve as a clamped impulse.
//! * **Prismatic motors** ([`PrismaticJoint::motor`]): position and spring
//!   motors along the slider axis, also applied once per solve.
//! * **Ragdoll** ([`RagdollJoint`]): point-to-point plus a cone limit on the
//!   swing angle between the twist axes and a twist-angle limit about the
//!   twist axis (the SDK's separate plane limits are not ported).
//! * **Prismatic** ([`PrismaticJoint`]): locked rotation and two locked
//!   translation directions plus a limited slider travel.
//! * **Stiff spring** ([`SpringJoint`]): a single distance row driving the
//!   pivot distance toward the rest length.
//! * **Point-to-plane** ([`PointToPlaneJoint`]): one signed-distance row along
//!   the plane normal, bilateral when the range collapses to a value.
//!
//! Rotation about a hinge axis and free spinning of a ball-and-socket remain
//! unconstrained; constraint islands and the SDK's constraint-atom system are
//! not ported.
//!
//! # Joint warm starting
//!
//! Every joint owns a fixed set of solver rows documented per variant (the
//! maximum is [`MAX_JOINT_ROWS`]). [`JointImpulse`] stores the accumulated
//! impulse of each row; [`Solver::solve_joints_with_impulses`] replays the
//! cached impulses before the first sweep — bilateral rows unconditionally,
//! unilateral rows only while they are violated — and writes the new
//! accumulation back, mirroring the contact warm starting. Unilateral rows
//! clamp their accumulated impulse to the restoring half-space during the
//! sweeps, so a replayed overshoot is pulled back instead of injecting energy.
//! Because the rows in this subset solve exactly, the replay is absorbed and
//! the final velocities match a cold start; the cache keeps the warm-start
//! structure in place for future clamped/relaxed rows. Motors are applied
//! once per solve and are never cached.
//!
//! # Determinism
//!
//! Joints are processed in slice order and constraint rows in the fixed order
//! documented above, matching the contact solver's insertion-order sweep.

use base_math::Vector4;

use crate::solver::{SolverBody, angular_mass};

/// Solver-level ball-and-socket joint (`hknpBallAndSocketConstraintData`).
///
/// The two world-space pivots must coincide. Linear DOF are removed; relative
/// rotation about the pivot is free.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallAndSocketJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space (center-of-mass relative).
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
}

/// Solver-level hinge joint (`hknpHingeConstraintData` linear + axis alignment).
///
/// The pivots must coincide and the two unit axes stay parallel; rotation
/// about the hinge axis is free apart from [`HingeJoint::limits`] and driven
/// by [`HingeJoint::motor`].
///
/// Row order: `[X, Y, Z]` linear pull, `[u, v]` axis alignment (`u`/`v` are
/// the deterministic perpendicular basis of `axis_a`), row 5 the angle limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// Hinge axis on body A in world space (unit).
    pub axis_a: Vector4,
    /// Hinge axis on body B in world space (unit).
    pub axis_b: Vector4,
    /// Current signed hinge angle in radians (B relative to A about the hinge
    /// axis, `(-pi, pi]`); consumed by [`HingeJoint::limits`] and
    /// [`HingeJoint::motor`].
    pub angle: f32,
    /// Optional angle limits (`None` keeps rotation about the hinge
    /// completely free).
    pub limits: Option<HingeLimits>,
    /// Optional motor about the hinge axis (`None` is frictionless).
    pub motor: Option<HingeMotor>,
}

/// Hinge angle limits (`hknpHingeConstraintData` limits subset, solver level).
///
/// `angle` lives on [`HingeJoint`]; the solver applies a restoring unilateral
/// impulse only while it is outside `[min_angle, max_angle]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeLimits {
    /// Lower bound in radians (`min_angle <= max_angle`).
    pub min_angle: f32,
    /// Upper bound in radians.
    pub max_angle: f32,
}

/// Motor about the hinge axis (`hknpAngularConstraintMotor` subset).
///
/// Every motor is applied once per solve as a single impulse clamped to
/// `±max_impulse` (before the constraint sweeps), which keeps the result
/// deterministic and iteration-count independent. The SDK's motors are
/// force-based and integrated over the step; the spring variant multiplies
/// its force by `delta_time` here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HingeMotor {
    /// Drive the relative angular velocity of B relative to A about the hinge
    /// axis toward a target (positive spins B in the sense of the hinge axes).
    Velocity {
        /// Desired relative angular velocity (rad/s).
        target_velocity: f32,
        /// Maximum impulse magnitude applied per solve.
        max_impulse: f32,
    },
    /// Drive the hinge angle toward a target
    /// (`hknpPositionConstraintMotor` subset): the motor drives the relative
    /// angular velocity toward `angle_error / tau`.
    Position {
        /// Desired hinge angle in radians (signed like [`HingeJoint::angle`]).
        target_angle: f32,
        /// Time constant of the position error in seconds (values `<= 1e-4`
        /// disable the motor).
        tau: f32,
        /// Maximum impulse magnitude applied per solve.
        max_impulse: f32,
    },
    /// Spring-damper pulling the hinge angle toward a rest angle
    /// (`hknpSpringConstraintMotor` subset): impulse
    /// `(stiffness * angle_error - damping * relative_velocity) * delta_time`,
    /// clamped to `±max_impulse`.
    Spring {
        /// Rest angle in radians.
        target_angle: f32,
        /// Spring stiffness (torque per radian).
        stiffness: f32,
        /// Damping coefficient (torque per rad/s).
        damping: f32,
        /// Maximum impulse magnitude applied per solve.
        max_impulse: f32,
    },
}

impl HingeMotor {
    /// The impulse clamp of this motor.
    pub fn max_impulse(&self) -> f32 {
        match self {
            Self::Velocity { max_impulse, .. }
            | Self::Position { max_impulse, .. }
            | Self::Spring { max_impulse, .. } => *max_impulse,
        }
    }
}

/// Solver-level fixed joint (`hknpFixedConstraintData`, velocity level).
///
/// The pivots must coincide and the relative angular velocity is zeroed about
/// the world axes. `orientation_error` is the world-space rotation error
/// (axis * angle) that rotating body A by would restore the bound relative
/// orientation; it feeds the Baumgarte drift correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// World-space orientation error (axis * angle), zero when aligned.
    pub orientation_error: Vector4,
}

/// Solver-level ragdoll joint (`hknpRagdollConstraintData` cone/twist subset).
///
/// The pivots must coincide; the swing angle between the two twist axes is
/// capped at `cone_angle` and the twist angle about the twist axis is limited
/// to `[twist_min, twist_max]`. The SDK's separate plane-angle limits are not
/// ported (the cone is the dominant ragdoll limit).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// Twist axis of body A in world space (unit).
    pub twist_axis_a: Vector4,
    /// Twist axis of body B in world space (unit).
    pub twist_axis_b: Vector4,
    /// Current swing angle between the twist axes, in radians.
    pub swing_angle: f32,
    /// Maximum swing angle (cone half-angle), in radians.
    pub cone_angle: f32,
    /// Current signed twist angle about the twist axis, in radians.
    pub twist_angle: f32,
    /// Lower twist bound in radians.
    pub twist_min: f32,
    /// Upper twist bound in radians.
    pub twist_max: f32,
}

/// Solver-level prismatic joint (`hknpPrismaticConstraintData` subset).
///
/// Translation along the slider axis is free between `min_translation` and
/// `max_translation`; the other two translation directions and all three
/// rotation directions are locked.
///
/// Row order: `[u, v]` linear lock (`u`/`v` are the deterministic
/// perpendicular basis of `axis_a`), `[X, Y, Z]` angular lock, row 5 the
/// travel limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// Slider axis of body A in world space (unit).
    pub axis_a: Vector4,
    /// Slider axis of body B in world space (unit).
    pub axis_b: Vector4,
    /// World-space orientation error (axis * angle), zero when aligned.
    pub orientation_error: Vector4,
    /// Current translation of A's pivot along the slider axis relative to B.
    pub translation: f32,
    /// Lower translation bound.
    pub min_translation: f32,
    /// Upper translation bound.
    pub max_translation: f32,
    /// Optional position/spring motor along the slider axis.
    pub motor: Option<PrismaticMotor>,
}

/// Motor along the slider axis (`hknpLinearConstraintMotor` subset).
///
/// Applied once per solve as a clamped impulse; the translation grows with
/// the relative velocity of A's pivot with respect to B's pivot along the
/// slider axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PrismaticMotor {
    /// Drive the slider translation toward a target: the motor drives the
    /// relative translation velocity toward `translation_error / tau`.
    Position {
        /// Desired slider translation.
        target_translation: f32,
        /// Time constant of the position error in seconds (values `<= 1e-4`
        /// disable the motor).
        tau: f32,
        /// Maximum impulse magnitude applied per solve.
        max_impulse: f32,
    },
    /// Spring-damper pulling the slider translation toward a rest value:
    /// impulse `-(stiffness * error + damping * relative_velocity) *
    /// delta_time`, clamped to `±max_impulse`.
    Spring {
        /// Rest translation.
        target_translation: f32,
        /// Spring stiffness (force per meter).
        stiffness: f32,
        /// Damping coefficient (force per m/s).
        damping: f32,
        /// Maximum impulse magnitude applied per solve.
        max_impulse: f32,
    },
}

impl PrismaticMotor {
    /// The impulse clamp of this motor.
    pub fn max_impulse(&self) -> f32 {
        match self {
            Self::Position { max_impulse, .. } | Self::Spring { max_impulse, .. } => *max_impulse,
        }
    }
}

/// Solver-level stiff spring (`hknpStiffSpringConstraintData`).
///
/// The pivot distance is driven toward `rest_length`; the pivots may still
/// rotate/slide freely about each other (a single distance row).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpringJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// Target distance between the pivots.
    pub rest_length: f32,
}

/// Solver-level point-to-plane joint (`hknpPointToPlaneConstraintData`).
///
/// A's pivot is kept at the signed distance range `[min_distance,
/// max_distance]` along the plane normal from B's pivot. A single linear row.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointToPlaneJoint {
    /// Solver index of body A.
    pub body_a: usize,
    /// Solver index of body B.
    pub body_b: usize,
    /// Pivot on body A in world space.
    pub pivot_a: Vector4,
    /// Pivot on body B in world space.
    pub pivot_b: Vector4,
    /// Plane normal in world space (unit).
    pub plane_normal: Vector4,
    /// Minimum signed distance along the plane normal.
    pub min_distance: f32,
    /// Maximum signed distance along the plane normal.
    pub max_distance: f32,
}

/// Any solver-level joint constraint, dispatched to a specialized solve path.
///
/// Each variant owns a fixed set of solver rows (maximum
/// [`MAX_JOINT_ROWS`]) whose accumulated impulses are stored in
/// [`JointImpulse`] in the row order documented on the variant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointConstraint {
    /// Point-to-point. Rows: `[X, Y, Z]` linear pull.
    BallAndSocket(BallAndSocketJoint),
    /// Point-to-point plus hinge-axis alignment. Rows: `[X, Y, Z]` linear,
    /// `[u, v]` axis alignment, row 5 the angle limit.
    Hinge(HingeJoint),
    /// Point-to-point plus locked relative angular velocity. Rows:
    /// `[X, Y, Z]` linear, `[X, Y, Z]` angular lock.
    Fixed(FixedJoint),
    /// Cone and twist limited ragdoll joint. Rows: `[X, Y, Z]` linear,
    /// row 3 the cone limit, row 4 the twist limit.
    Ragdoll(RagdollJoint),
    /// Slider with limited travel. Rows: `[u, v]` linear lock, `[X, Y, Z]`
    /// angular lock, row 5 the travel limit.
    Prismatic(PrismaticJoint),
    /// Distance-maintaining spring. Rows: row 0 the distance row.
    Spring(SpringJoint),
    /// Point constrained to a plane. Rows: row 0 the plane-normal row.
    PointToPlane(PointToPlaneJoint),
}

/// Maximum number of accumulated solver rows of one joint.
pub const MAX_JOINT_ROWS: usize = 8;

/// Accumulated impulses of one joint's rows (joint warm starting).
///
/// Row order is documented per [`JointConstraint`] variant; unused trailing
/// entries stay zero. [`Solver::solve_joints_with_impulses`] replays these
/// impulses before the first sweep (bilateral rows unconditionally, unilateral
/// rows only while violated) and stores the new accumulation back. Motors are
/// applied once per solve and never cached.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointImpulse {
    /// Accumulated row impulses in the joint's fixed row order.
    pub rows: [f32; MAX_JOINT_ROWS],
}

impl Default for JointImpulse {
    fn default() -> Self {
        Self {
            rows: [0.0; MAX_JOINT_ROWS],
        }
    }
}

impl JointImpulse {
    /// Whether no impulse has been accumulated yet.
    pub fn is_zero(&self) -> bool {
        self.rows == [0.0; MAX_JOINT_ROWS]
    }
}

impl JointConstraint {
    /// The two body indices.
    pub fn bodies(&self) -> (usize, usize) {
        match self {
            Self::BallAndSocket(joint) => (joint.body_a, joint.body_b),
            Self::Hinge(joint) => (joint.body_a, joint.body_b),
            Self::Fixed(joint) => (joint.body_a, joint.body_b),
            Self::Ragdoll(joint) => (joint.body_a, joint.body_b),
            Self::Prismatic(joint) => (joint.body_a, joint.body_b),
            Self::Spring(joint) => (joint.body_a, joint.body_b),
            Self::PointToPlane(joint) => (joint.body_a, joint.body_b),
        }
    }
}

/// Statistics returned by a joint solve.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JointSolverStats {
    /// Number of joint constraints processed.
    pub joints: usize,
    /// Number of sweeps executed.
    pub iterations: u32,
}

impl crate::solver::Solver {
    /// Solve `joints` in place on `bodies`, in slice order for
    /// `settings().iterations` sweeps, without joint warm starting.
    ///
    /// Joints referencing invalid body indices or two identical bodies are
    /// skipped. `delta_time` scales the drift-correction bias (0 disables it)
    /// and the spring-motor integration.
    pub fn solve_joints(
        &self,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        delta_time: f32,
    ) -> JointSolverStats {
        let mut impulses = vec![JointImpulse::default(); joints.len()];
        let indices: Vec<usize> = (0..joints.len()).collect();
        self.solve_joints_subset_with_impulses(bodies, joints, &indices, &mut impulses, delta_time)
    }

    /// Solve all `joints`, warm starting from (and updating) `impulses`.
    ///
    /// `impulses` is indexed like `joints`; entries carried over from a
    /// previous step are applied before the first sweep (bilateral rows
    /// unconditionally, unilateral rows only while violated). When
    /// [`SolverSettings::warm_starting`] is `false` the incoming impulses are
    /// discarded and the call behaves like [`Solver::solve_joints`].
    pub fn solve_joints_with_impulses(
        &self,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        impulses: &mut [JointImpulse],
        delta_time: f32,
    ) -> JointSolverStats {
        let indices: Vec<usize> = (0..joints.len()).collect();
        self.solve_joints_subset_with_impulses(bodies, joints, &indices, impulses, delta_time)
    }

    /// Solve only the joints listed in `subset`, in ascending index order,
    /// without joint warm starting.
    ///
    /// The world calls this with one constraint island's joints; islands share
    /// no dynamic body, so the impulse sequence is identical to a global sweep.
    pub fn solve_joints_subset(
        &self,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        subset: &[usize],
        delta_time: f32,
    ) -> JointSolverStats {
        let mut impulses = vec![JointImpulse::default(); joints.len()];
        self.solve_joints_subset_with_impulses(bodies, joints, subset, &mut impulses, delta_time)
    }

    /// Solve only the joints listed in `subset`, warm starting from (and
    /// updating) `impulses` (indexed like `joints`).
    pub fn solve_joints_subset_with_impulses(
        &self,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        subset: &[usize],
        impulses: &mut [JointImpulse],
        delta_time: f32,
    ) -> JointSolverStats {
        debug_assert!(delta_time >= 0.0 && delta_time.is_finite());
        debug_assert!(impulses.len() >= joints.len());
        let settings = self.settings();
        let bias_scale = if delta_time > 0.0 {
            settings.baumgarte / delta_time
        } else {
            0.0
        };

        if !settings.warm_starting {
            for &index in subset {
                impulses[index] = JointImpulse::default();
            }
        }

        // Motors are applied once per solve, before the sweeps (see
        // `solve_hinge_motor` / `solve_prismatic_motor`); limits run inside
        // the sweeps like any other constraint row.
        for &index in subset {
            match &joints[index] {
                JointConstraint::Hinge(joint) => {
                    if let Some(motor) = joint.motor {
                        solve_hinge_motor(bodies, joint, motor, delta_time);
                    }
                }
                JointConstraint::Prismatic(joint) => {
                    if let Some(motor) = joint.motor {
                        solve_prismatic_motor(bodies, joint, motor, delta_time);
                    }
                }
                _ => {}
            }
        }

        // Warm start: replay last step's row impulses along the current row
        // axes (unilateral rows only while they are violated).
        if settings.warm_starting {
            for &index in subset {
                apply_joint_warm_start(bodies, &joints[index], &impulses[index].rows);
            }
        }

        for _ in 0..settings.iterations {
            for &index in subset {
                let constraint = joints[index];
                let rows = &mut impulses[index].rows;
                match constraint {
                    JointConstraint::BallAndSocket(joint) => {
                        solve_ball_socket_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Hinge(joint) => {
                        solve_hinge_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Fixed(joint) => {
                        solve_fixed_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Ragdoll(joint) => {
                        solve_ragdoll_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Prismatic(joint) => {
                        solve_prismatic_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Spring(joint) => {
                        solve_spring_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::PointToPlane(joint) => {
                        solve_point_to_plane_rows(
                            bodies,
                            &joint,
                            rows,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                }
            }
        }

        JointSolverStats {
            joints: joints.len(),
            iterations: settings.iterations,
        }
    }
}

/// The world-axis basis used by bilateral linear rows (stable across steps,
/// which is what makes their cached impulses reusable).
const LINEAR_AXES: [Vector4; 3] = [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z];

/// Pull the two world pivots together along each of `axes`, accumulating the
/// impulses into `rows` (one row per axis, applied as `+impulse` on A).
#[allow(clippy::too_many_arguments)]
fn solve_linear_rows(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axes: &[Vector4],
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let separation = pivot_a - pivot_b;
    for (axis, row) in axes.iter().zip(rows.iter_mut()) {
        let relative = bodies[a].velocity_at(pivot_a) - bodies[b].velocity_at(pivot_b);
        let mass = linear_effective_mass(bodies, a, b, *axis, pivot_a, pivot_b);
        if mass <= 1e-12 {
            continue;
        }
        // Constraint: relative velocity along the axis must equal
        // -(separation * bias_scale) so the pivots converge.
        let bias = (separation.dot3(*axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (-bias - relative.dot3(*axis)) / mass;
        let next = *row + lambda;
        let delta = next - *row;
        *row = next;
        if delta != 0.0 {
            let impulse = *axis * delta;
            bodies[a].apply_impulse_at(pivot_a, impulse);
            bodies[b].apply_impulse_at(pivot_b, -impulse);
        }
    }
}

/// Replay cached linear-row impulses (`+impulse` on A per row).
fn warm_start_linear_rows(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axes: &[Vector4],
    rows: &[f32],
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    for (axis, &impulse) in axes.iter().zip(rows.iter()) {
        if impulse != 0.0 {
            let vector = *axis * impulse;
            bodies[a].apply_impulse_at(pivot_a, vector);
            bodies[b].apply_impulse_at(pivot_b, -vector);
        }
    }
}

/// Unilateral linear limit along `axis`, accumulating into `row`.
///
/// `translation` grows with the relative velocity of A's pivot with respect to
/// B's pivot along the axis; a restoring impulse is applied only when the
/// translation is outside `[min_translation, max_translation]`, and never in
/// the direction that would increase the violation. Like a contact normal row
/// the accumulated impulse is clamped to the restoring half-space (`<= 0` past
/// the upper bound, `>= 0` past the lower bound), so a warm-started
/// overshooting impulse is pulled back instead of injecting energy. When the
/// range is respected the row is zeroed.
#[allow(clippy::too_many_arguments)]
fn solve_unilateral_linear_row(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axis: Vector4,
    translation: f32,
    min_translation: f32,
    max_translation: f32,
    row: &mut f32,
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let mass = linear_effective_mass(bodies, a, b, axis, pivot_a, pivot_b);
    if mass <= 1e-12 {
        *row = 0.0;
        return;
    }
    let relative = bodies[a].velocity_at(pivot_a) - bodies[b].velocity_at(pivot_b);
    let upper_violation = translation - max_translation;
    let lower_violation = min_translation - translation;
    let (target, clamp): (f32, fn(f32, f32) -> f32) = if upper_violation > 0.0 {
        // Restoring impulses are negative; the accumulated impulse stays <= 0.
        (
            -(upper_violation * bias_scale).clamp(0.0, max_bias),
            f32::min,
        )
    } else if lower_violation > 0.0 {
        (
            (lower_violation * bias_scale).clamp(0.0, max_bias),
            f32::max,
        )
    } else {
        *row = 0.0;
        return;
    };
    let lambda = (target - relative.dot3(axis)) / mass;
    let next = clamp(*row + lambda, 0.0);
    let delta = next - *row;
    *row = next;
    if delta != 0.0 {
        let impulse = axis * delta;
        bodies[a].apply_impulse_at(pivot_a, impulse);
        bodies[b].apply_impulse_at(pivot_b, -impulse);
    }
}

/// Replay a cached unilateral linear-row impulse (`+impulse` on A).
fn warm_start_unilateral_linear_row(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axis: Vector4,
    impulse: f32,
) {
    if impulse == 0.0 {
        return;
    }
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let vector = axis * impulse;
    bodies[a].apply_impulse_at(pivot_a, vector);
    bodies[b].apply_impulse_at(pivot_b, -vector);
}

/// Ball-and-socket: three bilateral linear rows along the world axes.
fn solve_ball_socket_rows(
    bodies: &mut [SolverBody],
    joint: &BallAndSocketJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    solve_linear_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &LINEAR_AXES,
        &mut rows[0..3],
        bias_scale,
        max_bias,
    );
}

/// Prismatic joint: lock rotation and two translation directions, and limit
/// travel along the slider axis.
fn solve_prismatic_rows(
    bodies: &mut [SolverBody],
    joint: &PrismaticJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    let axis = normalized(joint.axis_a);
    let [u, v] = perpendicular_axes(axis);
    solve_linear_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &[u, v],
        &mut rows[0..2],
        bias_scale,
        max_bias,
    );
    solve_angular_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.orientation_error,
        &LINEAR_AXES,
        &mut rows[2..5],
        bias_scale,
        max_bias,
    );
    solve_unilateral_linear_row(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        axis,
        joint.translation,
        joint.min_translation,
        joint.max_translation,
        &mut rows[5],
        bias_scale,
        max_bias,
    );
}

/// Drive the slider toward the motor target with one clamped impulse.
fn solve_prismatic_motor(
    bodies: &mut [SolverBody],
    joint: &PrismaticJoint,
    motor: PrismaticMotor,
    delta_time: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b || motor.max_impulse() <= 0.0 {
        return;
    }
    let axis = normalized(joint.axis_a);
    let mass = linear_effective_mass(bodies, a, b, axis, joint.pivot_a, joint.pivot_b);
    if mass <= 1e-12 {
        return;
    }
    // Translation grows with the relative velocity of A w.r.t. B along the axis.
    let relative =
        (bodies[a].velocity_at(joint.pivot_a) - bodies[b].velocity_at(joint.pivot_b)).dot3(axis);
    let lambda = match motor {
        PrismaticMotor::Position {
            target_translation,
            tau,
            ..
        } => {
            if tau <= 1e-4 {
                return;
            }
            let desired = (target_translation - joint.translation) / tau;
            (desired - relative) / mass
        }
        PrismaticMotor::Spring {
            target_translation,
            stiffness,
            damping,
            ..
        } => {
            let error = joint.translation - target_translation;
            -(stiffness.max(0.0) * error + damping.max(0.0) * relative) * delta_time
        }
    };
    let lambda = lambda.clamp(-motor.max_impulse(), motor.max_impulse());
    let impulse = axis * lambda;
    bodies[a].apply_impulse_at(joint.pivot_a, impulse);
    bodies[b].apply_impulse_at(joint.pivot_b, -impulse);
}

/// Stiff spring: drive the pivot distance toward `rest_length` (row 0).
fn solve_spring_rows(
    bodies: &mut [SolverBody],
    joint: &SpringJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let delta = joint.pivot_b - joint.pivot_a;
    let distance = delta.length3();
    let direction = if distance > 1e-8 {
        delta / distance
    } else {
        // Degenerate distance: use the body centers to pick a stable axis.
        let centers = bodies[b].position - bodies[a].position;
        let length = centers.length3();
        if length > 1e-8 {
            centers / length
        } else {
            Vector4::UNIT_X
        }
    };
    let mass = linear_effective_mass(bodies, a, b, direction, joint.pivot_a, joint.pivot_b);
    if mass <= 1e-12 {
        return;
    }
    // Rate of distance change: (vB(pB) - vA(pA)) * direction.
    let relative = (bodies[b].velocity_at(joint.pivot_b) - bodies[a].velocity_at(joint.pivot_a))
        .dot3(direction);
    let error = distance - joint.rest_length;
    let bias = (error * bias_scale).clamp(-max_bias, max_bias);
    let lambda = -(bias + relative) / mass;
    let next = rows[0] + lambda;
    let impulse_delta = next - rows[0];
    rows[0] = next;
    if impulse_delta != 0.0 {
        let impulse = direction * impulse_delta;
        // +delta on B pulls it toward A when the spring is stretched.
        bodies[b].apply_impulse_at(joint.pivot_b, impulse);
        bodies[a].apply_impulse_at(joint.pivot_a, -impulse);
    }
}

/// Point-to-plane: keep A's pivot within the signed distance range along the
/// plane normal. An equal min/max is solved as a bilateral row (row 0).
fn solve_point_to_plane_rows(
    bodies: &mut [SolverBody],
    joint: &PointToPlaneJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let normal = normalized(joint.plane_normal);
    let distance = (joint.pivot_a - joint.pivot_b).dot3(normal);
    if (joint.min_distance - joint.max_distance).abs() <= 1e-8 {
        // Bilateral: drive the distance to the single target value.
        let relative = bodies[a].velocity_at(joint.pivot_a) - bodies[b].velocity_at(joint.pivot_b);
        let mass = linear_effective_mass(bodies, a, b, normal, joint.pivot_a, joint.pivot_b);
        if mass <= 1e-12 {
            return;
        }
        let bias = ((distance - joint.min_distance) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (-bias - relative.dot3(normal)) / mass;
        let next = rows[0] + lambda;
        let delta = next - rows[0];
        rows[0] = next;
        if delta != 0.0 {
            let impulse = normal * delta;
            bodies[a].apply_impulse_at(joint.pivot_a, impulse);
            bodies[b].apply_impulse_at(joint.pivot_b, -impulse);
        }
        return;
    }
    solve_unilateral_linear_row(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        normal,
        distance,
        joint.min_distance,
        joint.max_distance,
        &mut rows[0],
        bias_scale,
        max_bias,
    );
}

/// Hinge joint: three linear rows, two axis-alignment rows and the optional
/// angle-limit row.
fn solve_hinge_rows(
    bodies: &mut [SolverBody],
    joint: &HingeJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    solve_linear_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &LINEAR_AXES,
        &mut rows[0..3],
        bias_scale,
        max_bias,
    );
    let axis_a = normalized(joint.axis_a);
    let axis_b = normalized(joint.axis_b);
    // Misalignment error: rotating A about this vector (axis * sin(angle))
    // brings axis_a onto axis_b.
    let error = axis_a.cross(axis_b);
    let [u, v] = perpendicular_axes(axis_a);
    solve_angular_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        error,
        &[u, v],
        &mut rows[3..5],
        bias_scale,
        max_bias,
    );
    match joint.limits {
        Some(limits) => solve_unilateral_angular_row(
            bodies,
            joint.body_a,
            joint.body_b,
            axis_a,
            joint.angle,
            limits.min_angle,
            limits.max_angle,
            &mut rows[5],
            bias_scale,
            max_bias,
        ),
        None => rows[5] = 0.0,
    }
}

/// Drive the hinge toward the motor target with one clamped impulse.
///
/// The SDK's motors are force-based and integrated over the step; this subset
/// applies a single impulse per solve (before the constraint sweeps) clamped
/// to `±max_impulse`, which keeps the result deterministic and iteration
/// count independent. The spring variant is force-based and multiplies its
/// force by `delta_time`.
fn solve_hinge_motor(
    bodies: &mut [SolverBody],
    joint: &HingeJoint,
    motor: HingeMotor,
    delta_time: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b || motor.max_impulse() <= 0.0 {
        return;
    }
    let axis = normalized(joint.axis_a);
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        return;
    }
    // Relative velocity of B with respect to A along the hinge axis.
    let relative = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);
    let lambda = match motor {
        HingeMotor::Velocity {
            target_velocity, ..
        } => (target_velocity - relative) / mass,
        HingeMotor::Position {
            target_angle, tau, ..
        } => {
            if tau <= 1e-4 {
                return;
            }
            let desired = wrap_angle(target_angle - joint.angle) / tau;
            (desired - relative) / mass
        }
        HingeMotor::Spring {
            target_angle,
            stiffness,
            damping,
            ..
        } => {
            let error = wrap_angle(target_angle - joint.angle);
            (stiffness.max(0.0) * error - damping.max(0.0) * relative) * delta_time
        }
    };
    let lambda = lambda.clamp(-motor.max_impulse(), motor.max_impulse());
    apply_angular_impulse(bodies, b, a, axis, lambda);
}

/// Wrap an angle into `(-pi, pi]`.
fn wrap_angle(angle: f32) -> f32 {
    const TAU: f32 = core::f32::consts::TAU;
    let wrapped = angle.rem_euclid(TAU);
    if wrapped > core::f32::consts::PI {
        wrapped - TAU
    } else {
        wrapped
    }
}

/// Bilateral angular rows along `axes`: drive the relative angular velocity
/// of A w.r.t. B toward the axis-wise bias, accumulating into `rows`
/// (`+impulse` on A per row).
#[allow(clippy::too_many_arguments)]
fn solve_angular_rows(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    error: Vector4,
    axes: &[Vector4],
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    for (axis, row) in axes.iter().zip(rows.iter_mut()) {
        let relative = bodies[a].angular_velocity - bodies[b].angular_velocity;
        let mass = angular_effective_mass(bodies, a, b, *axis);
        if mass <= 1e-12 {
            continue;
        }
        let bias = (error.dot3(*axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (bias - relative.dot3(*axis)) / mass;
        let next = *row + lambda;
        let delta = next - *row;
        *row = next;
        if delta != 0.0 {
            apply_angular_impulse(bodies, a, b, *axis, delta);
        }
    }
}

/// Replay cached bilateral angular-row impulses (`+impulse` on A per row).
fn warm_start_angular_rows(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    axes: &[Vector4],
    rows: &[f32],
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    for (axis, &impulse) in axes.iter().zip(rows.iter()) {
        if impulse != 0.0 {
            apply_angular_impulse(bodies, a, b, *axis, impulse);
        }
    }
}

/// Unilateral angular limit about `axis`, accumulating into `row` (applied as
/// `+impulse` on B): only push back when `angle` is outside
/// `[min_angle, max_angle]`.
///
/// The angle grows with the relative angular velocity of B about A along the
/// axis, so the upper bound drives that velocity negative and the lower bound
/// positive. Like a contact normal row the accumulated impulse is clamped to
/// the restoring half-space (`<= 0` past the upper bound, `>= 0` past the
/// lower bound), so a warm-started overshooting impulse is pulled back instead
/// of injecting energy. Inside the dead zone the row is zeroed.
#[allow(clippy::too_many_arguments)]
fn solve_unilateral_angular_row(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
    angle: f32,
    min_angle: f32,
    max_angle: f32,
    row: &mut f32,
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        *row = 0.0;
        return;
    }
    // Relative velocity of B with respect to A along the axis: the angle grows
    // at this rate.
    let relative = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);

    let upper_violation = angle - max_angle;
    let lower_violation = min_angle - angle;
    let (target, clamp): (f32, fn(f32, f32) -> f32) = if upper_violation > 0.0 {
        // Past the upper bound: the restoring relative velocity is negative and
        // the accumulated impulse stays <= 0.
        (
            -(upper_violation * bias_scale).clamp(0.0, max_bias),
            f32::min,
        )
    } else if lower_violation > 0.0 {
        (
            (lower_violation * bias_scale).clamp(0.0, max_bias),
            f32::max,
        )
    } else {
        *row = 0.0;
        return;
    };

    let lambda = (target - relative) / mass;
    let next = clamp(*row + lambda, 0.0);
    let delta = next - *row;
    *row = next;
    if delta != 0.0 {
        apply_angular_impulse(bodies, b, a, axis, delta);
    }
}

/// Replay a cached unilateral angular-row impulse (`+impulse` on B).
fn warm_start_unilateral_angular_row(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
    impulse: f32,
) {
    if impulse == 0.0 {
        return;
    }
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), body_a),
        body_index(bodies.len(), body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    apply_angular_impulse(bodies, b, a, axis, impulse);
}

/// Ragdoll joint: three linear rows, the cone row and the twist row.
fn solve_ragdoll_rows(
    bodies: &mut [SolverBody],
    joint: &RagdollJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    solve_linear_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &LINEAR_AXES,
        &mut rows[0..3],
        bias_scale,
        max_bias,
    );
    solve_cone_row(bodies, joint, &mut rows[3], bias_scale, max_bias);
    solve_unilateral_angular_row(
        bodies,
        joint.body_a,
        joint.body_b,
        normalized(joint.twist_axis_a),
        joint.twist_angle,
        joint.twist_min,
        joint.twist_max,
        &mut rows[4],
        bias_scale,
        max_bias,
    );
}

/// Ragdoll cone limit: keep the swing angle between the twist axes at or below
/// `cone_angle` (row 3).
///
/// The correction axis is `twist_a × twist_b`; a positive rotation of A about
/// it swings A's twist axis toward B's, so the row has the same unilateral
/// shape as an upper angular limit.
fn solve_cone_row(
    bodies: &mut [SolverBody],
    joint: &RagdollJoint,
    row: &mut f32,
    bias_scale: f32,
    max_bias: f32,
) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let violation = joint.swing_angle - joint.cone_angle;
    if violation <= 0.0 {
        *row = 0.0;
        return;
    }
    let twist_a = normalized(joint.twist_axis_a);
    let twist_b = normalized(joint.twist_axis_b);
    let axis = normalized(twist_a.cross(twist_b));
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        *row = 0.0;
        return;
    }
    let rel = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);
    let target = -(violation * bias_scale).clamp(0.0, max_bias);
    let lambda = (target - rel) / mass;
    // The accumulated impulse stays <= 0 (restoring half-space), so a
    // warm-started overshooting impulse is pulled back.
    let next = (*row + lambda).min(0.0);
    let delta = next - *row;
    *row = next;
    if delta != 0.0 {
        apply_angular_impulse(bodies, b, a, axis, delta);
    }
}

/// Fixed joint: three linear rows plus the angular lock rows.
fn solve_fixed_rows(
    bodies: &mut [SolverBody],
    joint: &FixedJoint,
    rows: &mut [f32],
    bias_scale: f32,
    max_bias: f32,
) {
    solve_linear_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &LINEAR_AXES,
        &mut rows[0..3],
        bias_scale,
        max_bias,
    );
    solve_angular_rows(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.orientation_error,
        &LINEAR_AXES,
        &mut rows[3..6],
        bias_scale,
        max_bias,
    );
}

/// Replay the cached impulses of one joint before the sweeps.
///
/// Bilateral rows are replayed unconditionally along their (recomputed) row
/// axes. Unilateral rows (limits, cones, twists) are replayed only while they
/// are violated — their accumulated impulse is clamped to the restoring
/// half-space during the sweeps, so a replayed overshoot is pulled back
/// instead of injecting energy — and are otherwise zeroed.
fn apply_joint_warm_start(bodies: &mut [SolverBody], joint: &JointConstraint, rows: &[f32]) {
    match joint {
        JointConstraint::BallAndSocket(joint) => {
            warm_start_linear_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                joint.pivot_a,
                joint.pivot_b,
                &LINEAR_AXES,
                &rows[0..3],
            );
        }
        JointConstraint::Hinge(joint) => {
            warm_start_linear_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                joint.pivot_a,
                joint.pivot_b,
                &LINEAR_AXES,
                &rows[0..3],
            );
            let axis_a = normalized(joint.axis_a);
            let [u, v] = perpendicular_axes(axis_a);
            warm_start_angular_rows(bodies, joint.body_a, joint.body_b, &[u, v], &rows[3..5]);
            if let Some(limits) = joint.limits {
                if joint.angle > limits.max_angle || joint.angle < limits.min_angle {
                    warm_start_unilateral_angular_row(
                        bodies,
                        joint.body_a,
                        joint.body_b,
                        axis_a,
                        rows[5],
                    );
                }
            }
        }
        JointConstraint::Fixed(joint) => {
            warm_start_linear_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                joint.pivot_a,
                joint.pivot_b,
                &LINEAR_AXES,
                &rows[0..3],
            );
            warm_start_angular_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                &LINEAR_AXES,
                &rows[3..6],
            );
        }
        JointConstraint::Ragdoll(joint) => {
            warm_start_linear_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                joint.pivot_a,
                joint.pivot_b,
                &LINEAR_AXES,
                &rows[0..3],
            );
            if joint.swing_angle > joint.cone_angle {
                let axis = normalized(
                    normalized(joint.twist_axis_a).cross(normalized(joint.twist_axis_b)),
                );
                warm_start_unilateral_angular_row(
                    bodies,
                    joint.body_a,
                    joint.body_b,
                    axis,
                    rows[3],
                );
            }
            if joint.twist_angle < joint.twist_min || joint.twist_angle > joint.twist_max {
                warm_start_unilateral_angular_row(
                    bodies,
                    joint.body_a,
                    joint.body_b,
                    normalized(joint.twist_axis_a),
                    rows[4],
                );
            }
        }
        JointConstraint::Prismatic(joint) => {
            let axis = normalized(joint.axis_a);
            let [u, v] = perpendicular_axes(axis);
            warm_start_linear_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                joint.pivot_a,
                joint.pivot_b,
                &[u, v],
                &rows[0..2],
            );
            warm_start_angular_rows(
                bodies,
                joint.body_a,
                joint.body_b,
                &LINEAR_AXES,
                &rows[2..5],
            );
            if joint.translation > joint.max_translation
                || joint.translation < joint.min_translation
            {
                warm_start_unilateral_linear_row(
                    bodies,
                    joint.body_a,
                    joint.body_b,
                    joint.pivot_a,
                    joint.pivot_b,
                    axis,
                    rows[5],
                );
            }
        }
        JointConstraint::Spring(joint) => {
            let delta = joint.pivot_b - joint.pivot_a;
            let distance = delta.length3();
            if distance <= 1e-8 || rows[0] == 0.0 {
                return;
            }
            let direction = delta / distance;
            let vector = direction * rows[0];
            let (Ok(a), Ok(b)) = (
                body_index(bodies.len(), joint.body_a),
                body_index(bodies.len(), joint.body_b),
            ) else {
                return;
            };
            if a == b {
                return;
            }
            bodies[b].apply_impulse_at(joint.pivot_b, vector);
            bodies[a].apply_impulse_at(joint.pivot_a, -vector);
        }
        JointConstraint::PointToPlane(joint) => {
            let normal = normalized(joint.plane_normal);
            let distance = (joint.pivot_a - joint.pivot_b).dot3(normal);
            if (joint.min_distance - joint.max_distance).abs() <= 1e-8 {
                warm_start_linear_rows(
                    bodies,
                    joint.body_a,
                    joint.body_b,
                    joint.pivot_a,
                    joint.pivot_b,
                    &[normal],
                    &rows[0..1],
                );
            } else if distance < joint.min_distance || distance > joint.max_distance {
                warm_start_unilateral_linear_row(
                    bodies,
                    joint.body_a,
                    joint.body_b,
                    joint.pivot_a,
                    joint.pivot_b,
                    normal,
                    rows[0],
                );
            }
        }
    }
}

fn body_index(length: usize, index: usize) -> Result<usize, ()> {
    if index < length { Ok(index) } else { Err(()) }
}

fn normalized(direction: Vector4) -> Vector4 {
    let length = direction.length3();
    if length > 1e-10 {
        Vector4::from_xyz(
            direction.x / length,
            direction.y / length,
            direction.z / length,
        )
    } else {
        Vector4::UNIT_X
    }
}

/// Two deterministic unit axes perpendicular to `axis`.
fn perpendicular_axes(axis: Vector4) -> [Vector4; 2] {
    let helper = if axis.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    };
    let u = normalized(axis.cross(helper));
    let v = axis.cross(u);
    [u, v]
}

/// Inverse effective mass for a linear impulse pair at two pivot points:
/// each body contributes its inverse mass plus the angular term at its own
/// pivot arm.
fn linear_effective_mass(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
    pivot_a: Vector4,
    pivot_b: Vector4,
) -> f32 {
    bodies[body_a].inverse_mass
        + bodies[body_b].inverse_mass
        + angular_mass(&bodies[body_a], axis, pivot_a)
        + angular_mass(&bodies[body_b], axis, pivot_b)
}

/// Inverse effective mass for a pure angular impulse pair along `axis`.
fn angular_effective_mass(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
) -> f32 {
    inertia_term(&bodies[body_a], axis) + inertia_term(&bodies[body_b], axis)
}

fn inertia_term(body: &SolverBody, axis: Vector4) -> f32 {
    if body.inverse_inertia == Vector4::ZERO {
        return 0.0;
    }
    axis.x * axis.x * body.inverse_inertia.x
        + axis.y * axis.y * body.inverse_inertia.y
        + axis.z * axis.z * body.inverse_inertia.z
}

/// Apply opposing angular impulses `axis * lambda` to the two bodies.
fn apply_angular_impulse(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
    lambda: f32,
) {
    bodies[body_a].apply_angular_impulse_scaled(axis, lambda);
    bodies[body_b].apply_angular_impulse_scaled(axis, -lambda);
}

impl SolverBody {
    /// Apply a pure angular impulse `direction * scale` (diagonal inertia).
    pub(crate) fn apply_angular_impulse_scaled(&mut self, direction: Vector4, scale: f32) {
        if self.inverse_inertia == Vector4::ZERO {
            return;
        }
        self.angular_velocity += Vector4::from_xyz(
            direction.x * self.inverse_inertia.x * scale,
            direction.y * self.inverse_inertia.y * scale,
            direction.z * self.inverse_inertia.z * scale,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{Solver, SolverSettings};
    use base_math::Quaternion;

    fn dynamic_body(position: Vector4, velocity: Vector4) -> SolverBody {
        SolverBody::new(1.0, Vector4::splat(0.5), position, velocity, Vector4::ZERO)
    }

    fn static_body(position: Vector4) -> SolverBody {
        SolverBody::static_body(position)
    }

    #[test]
    fn ball_socket_pulls_pivots_together() {
        let mut bodies = vec![
            dynamic_body(Vector4::ZERO, Vector4::ZERO),
            static_body(Vector4::from_xyz(0.0, 2.0, 0.0)),
        ];
        let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::from_xyz(0.0, 1.0, 0.0),
            pivot_b: Vector4::from_xyz(0.0, 3.0, 0.0),
        });
        let stats = Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        assert_eq!(stats.joints, 1);
        assert_eq!(stats.iterations, 4);
        // A's pivot (0,1,0) lies below B's pivot (0,3,0), and B is static, so
        // A must gain upward velocity to close the gap.
        assert!(
            bodies[0].linear_velocity.y > 0.9,
            "vy {}",
            bodies[0].linear_velocity.y
        );
        assert_eq!(bodies[1].linear_velocity, Vector4::ZERO);
    }

    #[test]
    fn ball_socket_equalizes_pivot_velocities() {
        let mut bodies = vec![
            dynamic_body(Vector4::ZERO, Vector4::from_xyz(0.0, 0.0, 0.0)),
            dynamic_body(
                Vector4::from_xyz(4.0, 0.0, 0.0),
                Vector4::from_xyz(2.0, 0.0, 0.0),
            ),
        ];
        let pivot = Vector4::from_xyz(2.0, 0.0, 0.0);
        let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: pivot,
            pivot_b: pivot,
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        let relative = bodies[0].velocity_at(pivot) - bodies[1].velocity_at(pivot);
        assert!(
            relative.length3() < 1e-2,
            "relative velocity {}",
            relative.length3()
        );
    }

    #[test]
    fn hinge_zeroes_perpendicular_spin_when_aligned() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 0.0, 1.0),
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = JointConstraint::Hinge(HingeJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            angle: 0.0,
            limits: None,
            motor: None,
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        // With aligned axes there is no drift, so the out-of-plane spin about
        // Z must be driven to zero.
        assert!(
            bodies[0].angular_velocity.z.abs() < 1e-2,
            "wz {}",
            bodies[0].angular_velocity.z
        );
    }

    #[test]
    fn hinge_bias_rotates_axis_toward_the_target() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        // axis_a = +X, axis_b points towards -Y: rotating A about -Z aligns
        // the axes, so the drift bias must produce negative wz.
        let joint = JointConstraint::Hinge(HingeJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::from_xyz(
                core::f32::consts::FRAC_1_SQRT_2,
                -core::f32::consts::FRAC_1_SQRT_2,
                0.0,
            ),
            angle: 0.0,
            limits: None,
            motor: None,
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.z < 0.0,
            "wz {}",
            bodies[0].angular_velocity.z
        );
    }

    #[test]
    fn fixed_joint_zeroes_relative_angular_velocity() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::from_xyz(1.0, 2.0, 3.0),
            ),
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::from_xyz(-1.0, 1.0, 0.0),
            ),
        ];
        let joint = JointConstraint::Fixed(FixedJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            orientation_error: Vector4::ZERO,
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        let relative = bodies[0].angular_velocity - bodies[1].angular_velocity;
        assert!(
            relative.length3() < 1e-2,
            "relative angular velocity {}",
            relative.length3()
        );
    }

    #[test]
    fn fixed_joint_bias_drives_toward_bound_orientation() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = JointConstraint::Fixed(FixedJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            orientation_error: Vector4::from_xyz(0.0, 0.1, 0.0),
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.y > 0.0,
            "wy {}",
            bodies[0].angular_velocity.y
        );
    }

    #[test]
    fn invalid_body_indices_are_skipped() {
        let mut bodies = vec![dynamic_body(Vector4::ZERO, Vector4::UNIT_X)];
        let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
            body_a: 0,
            body_b: 99,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
        });
        Solver::default().solve_joints(&mut bodies, &[joint], 1.0 / 60.0);
        assert_eq!(bodies[0].linear_velocity, Vector4::UNIT_X);
    }

    #[test]
    fn joints_are_processed_in_slice_order() {
        // Two identical joints on the same bodies must give the same result
        // regardless of how many times they repeat (idempotent converged state).
        let mut bodies = vec![
            dynamic_body(Vector4::ZERO, Vector4::from_xyz(0.0, 0.0, 0.0)),
            dynamic_body(Vector4::from_xyz(0.0, 1.0, 0.0), Vector4::ZERO),
        ];
        let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
        });
        Solver::default().solve_joints(&mut bodies, &[joint, joint], 1.0 / 60.0);
        let relative = bodies[0].velocity_at(Vector4::ZERO) - bodies[1].velocity_at(Vector4::ZERO);
        assert!(relative.length3() < 1e-2);
    }

    #[test]
    fn joint_subset_only_solves_listed_joints() {
        let mut bodies = vec![
            dynamic_body(Vector4::ZERO, Vector4::UNIT_X),
            dynamic_body(Vector4::ZERO, Vector4::ZERO),
            dynamic_body(Vector4::from_xyz(5.0, 0.0, 0.0), Vector4::UNIT_X),
            dynamic_body(Vector4::from_xyz(5.0, 0.0, 0.0), Vector4::ZERO),
        ];
        let joint = |body_a, body_b| {
            JointConstraint::BallAndSocket(BallAndSocketJoint {
                body_a,
                body_b,
                pivot_a: Vector4::ZERO,
                pivot_b: Vector4::ZERO,
            })
        };
        let joints = [joint(0, 1), joint(2, 3)];
        Solver::default().solve_joints_subset(&mut bodies, &joints, &[1], 1.0 / 60.0);

        // Island 0 is not in the subset and stays untouched.
        assert_eq!(bodies[0].linear_velocity, Vector4::UNIT_X);
        assert_eq!(bodies[1].linear_velocity, Vector4::ZERO);
        // Island 1 had its relative velocity removed.
        let relative = (bodies[2].linear_velocity - bodies[3].linear_velocity).length3();
        assert!(relative < 1e-2, "relative velocity {relative}");
    }

    #[test]
    fn quaternion_composes_for_error_mapping() {
        // Sanity check used by the world: rotating A by q_e restores R0.
        let q_a = Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.2);
        let r0 = Quaternion::from_axis_angle(Vector4::UNIT_X, 0.1);
        let q_b = q_a.mul(r0);
        let r_rel = q_a.inverse_mul(q_b);
        let q_e = r_rel.mul_inverse(r0);
        assert!(q_e.all_equal(Quaternion::IDENTITY, 1e-5));
    }

    fn hinge_joint() -> HingeJoint {
        HingeJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            angle: 0.0,
            limits: None,
            motor: None,
        }
    }

    fn hinge_pair(spun: Vector4) -> Vec<SolverBody> {
        vec![
            SolverBody::new(1.0, Vector4::splat(0.5), Vector4::ZERO, Vector4::ZERO, spun),
            SolverBody::static_body(Vector4::ZERO),
        ]
    }

    #[test]
    fn hinge_limit_upper_bound_pushes_velocity_back() {
        // Angle 0.5 rad is past max 0.2. B is static, so the restoring
        // relative velocity of B w.r.t. A (negative) requires body A to spin
        // positively about the axis.
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.limits = Some(HingeLimits {
            min_angle: -0.2,
            max_angle: 0.2,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x > 0.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn hinge_limit_lower_bound_pushes_velocity_forward() {
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = -0.5;
        joint.limits = Some(HingeLimits {
            min_angle: -0.2,
            max_angle: 0.2,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x < 0.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn hinge_limit_inside_range_applies_nothing() {
        let mut bodies = hinge_pair(Vector4::UNIT_X);
        let mut joint = hinge_joint();
        joint.angle = 0.1;
        joint.limits = Some(HingeLimits {
            min_angle: -0.2,
            max_angle: 0.2,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert_eq!(bodies[0].angular_velocity, Vector4::UNIT_X);
    }

    #[test]
    fn hinge_limit_does_not_push_past_the_bound() {
        // Already returning fast (relative velocity of B w.r.t. A well below
        // the bias target): the unilateral limit must not slow it down or
        // accelerate it outward.
        let mut bodies = hinge_pair(Vector4::from_xyz(6.0, 0.0, 0.0));
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.limits = Some(HingeLimits {
            min_angle: -0.2,
            max_angle: 0.2,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x >= 6.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn hinge_motor_drives_toward_target_velocity() {
        // B is static, so driving B relative to A toward +2 spins A at -2.
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.motor = Some(HingeMotor::Velocity {
            target_velocity: 2.0,
            max_impulse: 10.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x < -1.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn hinge_motor_impulse_is_clamped() {
        // Huge target but a tiny max impulse: the applied velocity change is
        // bounded by max_impulse * inverse_inertia (x component: 0.5).
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.motor = Some(HingeMotor::Velocity {
            target_velocity: 1000.0,
            max_impulse: 0.1,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        let wx = bodies[0].angular_velocity.x;
        assert!(
            (wx + 0.05).abs() < 1e-4,
            "wx {} must equal impulse*inverse_inertia",
            wx
        );
    }

    #[test]
    fn hinge_motor_zero_when_at_target() {
        // A spinning at -2 makes the relative velocity of B w.r.t. A equal
        // the target, so the motor must not apply anything.
        let mut bodies = hinge_pair(Vector4::from_xyz(-2.0, 0.0, 0.0));
        let mut joint = hinge_joint();
        joint.motor = Some(HingeMotor::Velocity {
            target_velocity: 2.0,
            max_impulse: 10.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert_eq!(
            bodies[0].angular_velocity,
            Vector4::from_xyz(-2.0, 0.0, 0.0)
        );
    }

    #[test]
    fn hinge_limit_and_motor_cooperate_deterministically() {
        let build = || {
            let bodies = hinge_pair(Vector4::ZERO);
            let mut joint = hinge_joint();
            joint.angle = 0.5;
            joint.limits = Some(HingeLimits {
                min_angle: -0.2,
                max_angle: 0.2,
            });
            joint.motor = Some(HingeMotor::Velocity {
                target_velocity: 10.0,
                max_impulse: 1.0,
            });
            (bodies, joint)
        };
        let (mut a, joint) = build();
        let (mut b, joint2) = build();
        Solver::default().solve_joints(&mut a, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        Solver::default().solve_joints(&mut b, &[JointConstraint::Hinge(joint2)], 1.0 / 60.0);
        assert_eq!(a, b);
    }

    fn ragdoll_joint() -> RagdollJoint {
        RagdollJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            twist_axis_a: Vector4::UNIT_X,
            twist_axis_b: Vector4::UNIT_X,
            swing_angle: 0.0,
            cone_angle: core::f32::consts::FRAC_PI_4,
            twist_angle: 0.0,
            twist_min: -0.2,
            twist_max: 0.2,
        }
    }

    #[test]
    fn ragdoll_cone_limit_pushes_the_swing_back() {
        // twist_b is rotated +0.6 rad about Z from twist_a (+X), past the
        // 0.3 rad cone; rotating A about +Z reduces the swing angle.
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let mut joint = ragdoll_joint();
        joint.swing_angle = 0.6;
        joint.cone_angle = 0.3;
        joint.twist_axis_b = Vector4::from_xyz(0.6f32.cos(), 0.6f32.sin(), 0.0);
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Ragdoll(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.z > 0.0,
            "wz {}",
            bodies[0].angular_velocity.z
        );
    }

    #[test]
    fn ragdoll_cone_limit_does_nothing_inside_the_cone() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::UNIT_Z,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let mut joint = ragdoll_joint();
        joint.swing_angle = 0.1;
        joint.cone_angle = 0.3;
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Ragdoll(joint)], 1.0 / 60.0);
        assert!(
            (bodies[0].angular_velocity.z - 1.0).abs() < 1e-6,
            "wz {}",
            bodies[0].angular_velocity.z
        );
    }

    #[test]
    fn ragdoll_twist_limit_pushes_the_twist_back() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let mut joint = ragdoll_joint();
        joint.twist_angle = 0.5;
        joint.twist_max = 0.2;
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Ragdoll(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x > 0.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn prismatic_blocks_perpendicular_translation_only() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::from_xyz(2.0, 2.0, 3.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PrismaticJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            orientation_error: Vector4::ZERO,
            translation: 0.0,
            min_translation: -1.0,
            max_translation: 1.0,
            motor: None,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::Prismatic(joint)],
            1.0 / 60.0,
        );
        assert!(
            (bodies[0].linear_velocity.x - 2.0).abs() < 1e-3,
            "vx {}",
            bodies[0].linear_velocity.x
        );
        assert!(bodies[0].linear_velocity.y.abs() < 1e-3);
        assert!(bodies[0].linear_velocity.z.abs() < 1e-3);
    }

    #[test]
    fn prismatic_limit_stops_outward_translation() {
        // Past the upper bound while still moving outward: the limit row must
        // reverse the slider velocity.
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::UNIT_X * 3.0,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PrismaticJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            orientation_error: Vector4::ZERO,
            translation: 0.5,
            min_translation: -0.2,
            max_translation: 0.2,
            motor: None,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::Prismatic(joint)],
            1.0 / 60.0,
        );
        assert!(
            bodies[0].linear_velocity.x < 0.0,
            "vx {}",
            bodies[0].linear_velocity.x
        );
    }

    #[test]
    fn prismatic_locks_relative_rotation() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::from_xyz(1.0, 2.0, 3.0),
            ),
            SolverBody::new(
                1.0,
                Vector4::splat(0.5),
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::from_xyz(-1.0, 1.0, 0.0),
            ),
        ];
        let joint = PrismaticJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            orientation_error: Vector4::ZERO,
            translation: 0.0,
            min_translation: -1.0,
            max_translation: 1.0,
            motor: None,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::Prismatic(joint)],
            1.0 / 60.0,
        );
        let relative = bodies[0].angular_velocity - bodies[1].angular_velocity;
        assert!(
            relative.length3() < 1e-2,
            "relative angular velocity {}",
            relative.length3()
        );
    }

    #[test]
    fn spring_pulls_stretched_pivots_together() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::from_xyz(0.0, 2.0, 0.0)),
        ];
        let joint = SpringJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::from_xyz(0.0, 2.0, 0.0),
            rest_length: 1.0,
        };
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Spring(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].linear_velocity.y > 0.0,
            "vy {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn spring_pushes_compressed_pivots_apart() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::from_xyz(0.0, 0.5, 0.0)),
        ];
        let joint = SpringJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::from_xyz(0.0, 0.5, 0.0),
            rest_length: 1.0,
        };
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Spring(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].linear_velocity.y < 0.0,
            "vy {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn point_to_plane_equality_drives_the_pivot_onto_the_plane() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PointToPlaneJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::from_xyz(0.0, 1.0, 0.0),
            pivot_b: Vector4::ZERO,
            plane_normal: Vector4::UNIT_Y,
            min_distance: 0.0,
            max_distance: 0.0,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::PointToPlane(joint)],
            1.0 / 60.0,
        );
        assert!(
            bodies[0].linear_velocity.y < 0.0,
            "vy {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn point_to_plane_limits_leave_the_range_alone() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::UNIT_X,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PointToPlaneJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::from_xyz(0.0, 0.1, 0.0),
            pivot_b: Vector4::ZERO,
            plane_normal: Vector4::UNIT_Y,
            min_distance: 0.0,
            max_distance: 0.3,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::PointToPlane(joint)],
            1.0 / 60.0,
        );
        assert_eq!(bodies[0].linear_velocity, Vector4::UNIT_X);
    }

    #[test]
    fn point_to_plane_unilateral_pushes_back_outside_the_range() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PointToPlaneJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::from_xyz(0.0, -0.5, 0.0),
            pivot_b: Vector4::ZERO,
            plane_normal: Vector4::UNIT_Y,
            min_distance: 0.0,
            max_distance: 0.3,
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::PointToPlane(joint)],
            1.0 / 60.0,
        );
        assert!(
            bodies[0].linear_velocity.y > 0.0,
            "vy {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn new_joint_variants_are_deterministic_and_skip_bad_indices() {
        fn body_pair() -> Vec<SolverBody> {
            vec![
                SolverBody::new(
                    1.0,
                    Vector4::splat(0.5),
                    Vector4::ZERO,
                    Vector4::UNIT_X,
                    Vector4::UNIT_Z,
                ),
                SolverBody::new(
                    1.0,
                    Vector4::splat(0.5),
                    Vector4::from_xyz(1.0, 0.0, 0.0),
                    -Vector4::UNIT_X,
                    Vector4::ZERO,
                ),
            ]
        }
        fn joint_set() -> Vec<JointConstraint> {
            vec![
                JointConstraint::Ragdoll(ragdoll_joint()),
                JointConstraint::Prismatic(PrismaticJoint {
                    body_a: 0,
                    body_b: 1,
                    pivot_a: Vector4::ZERO,
                    pivot_b: Vector4::ZERO,
                    axis_a: Vector4::UNIT_X,
                    axis_b: Vector4::UNIT_X,
                    orientation_error: Vector4::ZERO,
                    translation: 0.1,
                    min_translation: -0.2,
                    max_translation: 0.2,
                    motor: None,
                }),
                JointConstraint::Spring(SpringJoint {
                    body_a: 0,
                    body_b: 1,
                    pivot_a: Vector4::ZERO,
                    pivot_b: Vector4::from_xyz(1.0, 0.0, 0.0),
                    rest_length: 0.5,
                }),
                JointConstraint::PointToPlane(PointToPlaneJoint {
                    body_a: 0,
                    body_b: 1,
                    pivot_a: Vector4::ZERO,
                    pivot_b: Vector4::ZERO,
                    plane_normal: Vector4::UNIT_Y,
                    min_distance: 0.0,
                    max_distance: 0.0,
                }),
            ]
        }
        let mut first = body_pair();
        let mut second = body_pair();
        Solver::default().solve_joints(&mut first, &joint_set(), 1.0 / 60.0);
        Solver::default().solve_joints(&mut second, &joint_set(), 1.0 / 60.0);
        assert_eq!(first, second);

        // Out-of-range body indices are skipped, leaving the input untouched.
        let mut single = vec![SolverBody::new(
            1.0,
            Vector4::ZERO,
            Vector4::ZERO,
            Vector4::UNIT_X,
            Vector4::ZERO,
        )];
        let joint = JointConstraint::Spring(SpringJoint {
            body_a: 0,
            body_b: 99,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            rest_length: 1.0,
        });
        Solver::default().solve_joints(&mut single, &[joint], 1.0 / 60.0);
        assert_eq!(single[0].linear_velocity, Vector4::UNIT_X);
    }

    // Position / spring motors and joint warm starting.

    #[test]
    fn hinge_position_motor_drives_the_angle_toward_the_target() {
        // Angle 0.5 with target 0 and tau 0.1: the desired relative velocity is
        // -5 rad/s; B is static, so A must spin positively about the axis.
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.motor = Some(HingeMotor::Position {
            target_angle: 0.0,
            tau: 0.1,
            max_impulse: 10.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert!(
            bodies[0].angular_velocity.x > 1.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }

    #[test]
    fn hinge_position_motor_with_zero_tau_is_inert() {
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.motor = Some(HingeMotor::Position {
            target_angle: 0.0,
            tau: 0.0,
            max_impulse: 10.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        assert_eq!(bodies[0].angular_velocity, Vector4::ZERO);
    }

    #[test]
    fn hinge_spring_motor_pulls_the_angle_back() {
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.motor = Some(HingeMotor::Spring {
            target_angle: 0.0,
            stiffness: 10.0,
            damping: 2.0,
            max_impulse: 100.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        let wx = bodies[0].angular_velocity.x;
        assert!(
            wx > 0.01 && wx < 0.1,
            "wx {wx} must be a small restoring kick"
        );
    }

    #[test]
    fn hinge_spring_motor_impulse_is_clamped() {
        // Huge stiffness but a tiny impulse clamp: the velocity change is
        // bounded by max_impulse * inverse_inertia (0.01 * 0.5).
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = 0.5;
        joint.motor = Some(HingeMotor::Spring {
            target_angle: 0.0,
            stiffness: 1.0e6,
            damping: 0.0,
            max_impulse: 0.01,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        let wx = bodies[0].angular_velocity.x;
        assert!((wx - 0.005).abs() < 1e-5, "wx {wx} must equal the clamp");
    }

    #[test]
    fn prismatic_position_motor_drives_the_slider() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PrismaticJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            orientation_error: Vector4::ZERO,
            translation: 0.0,
            min_translation: f32::NEG_INFINITY,
            max_translation: f32::INFINITY,
            motor: Some(PrismaticMotor::Position {
                target_translation: 1.0,
                tau: 0.1,
                max_impulse: 100.0,
            }),
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::Prismatic(joint)],
            1.0 / 60.0,
        );
        assert!(
            bodies[0].linear_velocity.x > 5.0,
            "vx {}",
            bodies[0].linear_velocity.x
        );
        assert!(bodies[0].linear_velocity.y.abs() < 1e-9);
        assert!(bodies[0].linear_velocity.z.abs() < 1e-9);
    }

    #[test]
    fn prismatic_spring_motor_pushes_a_compressed_slider_back() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let joint = PrismaticJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::ZERO,
            pivot_b: Vector4::ZERO,
            axis_a: Vector4::UNIT_X,
            axis_b: Vector4::UNIT_X,
            orientation_error: Vector4::ZERO,
            translation: -0.5,
            min_translation: f32::NEG_INFINITY,
            max_translation: f32::INFINITY,
            motor: Some(PrismaticMotor::Spring {
                target_translation: 0.0,
                stiffness: 50.0,
                damping: 0.0,
                max_impulse: 100.0,
            }),
        };
        Solver::default().solve_joints(
            &mut bodies,
            &[JointConstraint::Prismatic(joint)],
            1.0 / 60.0,
        );
        assert!(
            bodies[0].linear_velocity.x > 0.2,
            "vx {}",
            bodies[0].linear_velocity.x
        );
    }

    fn separated_ball_socket() -> (Vec<SolverBody>, JointConstraint) {
        let bodies = vec![
            dynamic_body(Vector4::ZERO, Vector4::ZERO),
            static_body(Vector4::from_xyz(0.0, 2.0, 0.0)),
        ];
        let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
            body_a: 0,
            body_b: 1,
            pivot_a: Vector4::from_xyz(0.0, 1.0, 0.0),
            pivot_b: Vector4::from_xyz(0.0, 3.0, 0.0),
        });
        (bodies, joint)
    }

    #[test]
    fn joint_impulses_accumulate_for_warm_starting() {
        let (mut bodies, joint) = separated_ball_socket();
        let mut impulses = vec![JointImpulse::default(); 1];
        Solver::default().solve_joints_with_impulses(
            &mut bodies,
            &[joint],
            &mut impulses,
            1.0 / 60.0,
        );
        assert!(
            impulses[0].rows[0..3].iter().any(|&row| row != 0.0),
            "the linear rows must accumulate impulses"
        );
    }

    #[test]
    fn joint_warm_starting_disabled_discards_impulses() {
        let (mut preloaded, joint) = separated_ball_socket();
        let (mut fresh, _joint2) = separated_ball_socket();
        let solver = Solver::new(SolverSettings {
            warm_starting: false,
            ..SolverSettings::default()
        });
        // Bogus incoming impulses must be discarded, so both runs produce the
        // same bodies and the same freshly accumulated impulses.
        let mut impulses = vec![JointImpulse {
            rows: [5.0; MAX_JOINT_ROWS],
        }];
        solver.solve_joints_with_impulses(&mut preloaded, &[joint], &mut impulses, 1.0 / 60.0);
        solver.solve_joints(&mut fresh, &[joint], 1.0 / 60.0);
        assert_eq!(preloaded, fresh);
        assert!(
            impulses[0].rows[0..3].iter().any(|&row| row != 0.0),
            "the cold solve still accumulates fresh impulses"
        );
    }

    #[test]
    fn joint_warm_start_is_absorbed_by_exact_rows_and_stays_deterministic() {
        // The rows in this subset solve each row exactly, so replaying cached
        // impulses must be fully absorbed: identical final velocities and the
        // same converged row totals as a cold start, bit-exact across runs.
        let build = || {
            let bodies = vec![
                SolverBody::new(
                    1.0,
                    Vector4::splat(0.5),
                    Vector4::ZERO,
                    Vector4::from_xyz(0.0, 1.0, 0.0),
                    Vector4::ZERO,
                ),
                SolverBody::new(
                    1.0,
                    Vector4::splat(0.5),
                    Vector4::from_xyz(1.0, 2.0, 0.0),
                    Vector4::from_xyz(1.0, 3.0, 0.0),
                    Vector4::UNIT_X,
                ),
            ];
            let joint = JointConstraint::BallAndSocket(BallAndSocketJoint {
                body_a: 0,
                body_b: 1,
                pivot_a: Vector4::from_xyz(0.0, 2.0, 0.0),
                pivot_b: Vector4::from_xyz(1.0, 3.0, 0.0),
            });
            (bodies, joint)
        };
        let solver = Solver::new(SolverSettings {
            iterations: 1,
            ..SolverSettings::default()
        });
        let (mut warm_a, joint) = build();
        let (mut warm_b, joint2) = build();
        let (mut cold, joint3) = build();

        // First solve accumulates rows into `impulses`.
        let mut impulses = vec![JointImpulse::default(); 1];
        solver.solve_joints_with_impulses(&mut warm_a, &[joint], &mut impulses, 1.0 / 60.0);
        let warm_rows = impulses[0];
        assert!(!warm_rows.is_zero());

        // Second run replays them; a cold run starts from zero rows.
        let mut impulses_b = vec![warm_rows; 1];
        solver.solve_joints_with_impulses(&mut warm_b, &[joint2], &mut impulses_b, 1.0 / 60.0);
        let mut cold_impulses = vec![JointImpulse::default(); 1];
        solver.solve_joints_with_impulses(&mut cold, &[joint3], &mut cold_impulses, 1.0 / 60.0);

        assert_eq!(warm_a, warm_b, "identical warm runs must be bit-exact");
        assert_eq!(warm_a, cold, "exact rows absorb the replayed impulses");
        assert_eq!(impulses_b[0], cold_impulses[0]);
    }

    #[test]
    fn inactive_unilateral_rows_are_zeroed_and_not_warm_started() {
        // Hinge inside its limits with a bogus cached limit impulse: the row
        // must neither be replayed nor keep its stale value.
        let (mut bodies, joint) = {
            let bodies = hinge_pair(Vector4::UNIT_X);
            let mut joint = hinge_joint();
            joint.angle = 0.1;
            joint.limits = Some(HingeLimits {
                min_angle: -0.2,
                max_angle: 0.2,
            });
            (bodies, JointConstraint::Hinge(joint))
        };
        let mut impulses = vec![JointImpulse::default(); 1];
        impulses[0].rows[5] = 3.0;
        Solver::default().solve_joints_with_impulses(
            &mut bodies,
            &[joint],
            &mut impulses,
            1.0 / 60.0,
        );
        assert_eq!(impulses[0].rows[5], 0.0);
        assert_eq!(bodies[0].angular_velocity, Vector4::UNIT_X);
    }

    #[test]
    fn wrapped_angle_targets_are_reached_by_the_position_motor() {
        // angle near +pi, target just past -pi: the wrapped error is small and
        // positive, so the motor drives B relative to A positively.
        let mut bodies = hinge_pair(Vector4::ZERO);
        let mut joint = hinge_joint();
        joint.angle = core::f32::consts::PI - 0.1;
        joint.motor = Some(HingeMotor::Position {
            target_angle: -core::f32::consts::PI + 0.1,
            tau: 0.1,
            max_impulse: 10.0,
        });
        Solver::default().solve_joints(&mut bodies, &[JointConstraint::Hinge(joint)], 1.0 / 60.0);
        // desired = wrap(target - angle)/tau = (+0.2)/0.1 = +2 → A spins -2.
        assert!(
            bodies[0].angular_velocity.x < -1.0,
            "wx {}",
            bodies[0].angular_velocity.x
        );
    }
}
