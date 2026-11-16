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
//! * **Hinge motor** ([`HingeJoint::motor`]): a velocity motor about the
//!   hinge axis applied once per solve as a clamped impulse (the SDK's
//!   force-based motors are not ported).
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
    /// Optional angle limits, with the current signed hinge angle supplied by
    /// the caller (`None` keeps rotation about the hinge completely free).
    pub limits: Option<HingeLimits>,
    /// Optional velocity motor about the hinge axis (`None` is frictionless).
    pub motor: Option<HingeMotor>,
}

/// Hinge angle limits (`hknpHingeConstraintData` limits subset, solver level).
///
/// `angle` is the signed angle between the world-space reference axes about
/// the hinge axis in radians; the solver applies a restoring unilateral
/// impulse only when it is outside `[min_angle, max_angle]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeLimits {
    /// Current signed hinge angle in radians.
    pub angle: f32,
    /// Lower bound in radians (`min_angle <= max_angle`).
    pub min_angle: f32,
    /// Upper bound in radians.
    pub max_angle: f32,
}

/// Velocity motor about the hinge axis (`hknpVelocityConstraintMotor` subset).
///
/// The motor drives the relative angular velocity of B relative to A about
/// the hinge axis toward `target_velocity` (positive spins B in the sense of
/// the hinge axes); the applied impulse per solve is clamped to
/// `±max_impulse`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeMotor {
    /// Desired relative angular velocity about the hinge axis (rad/s).
    pub target_velocity: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointConstraint {
    /// Point-to-point.
    BallAndSocket(BallAndSocketJoint),
    /// Point-to-point plus hinge-axis alignment.
    Hinge(HingeJoint),
    /// Point-to-point plus locked relative angular velocity.
    Fixed(FixedJoint),
    /// Cone and twist limited ragdoll joint.
    Ragdoll(RagdollJoint),
    /// Slider with limited travel.
    Prismatic(PrismaticJoint),
    /// Distance-maintaining spring.
    Spring(SpringJoint),
    /// Point constrained to a plane.
    PointToPlane(PointToPlaneJoint),
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
    /// `settings().iterations` sweeps.
    ///
    /// Joints referencing invalid body indices or two identical bodies are
    /// skipped. `delta_time` scales the drift-correction bias (0 disables it).
    pub fn solve_joints(
        &self,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        delta_time: f32,
    ) -> JointSolverStats {
        let indices: Vec<usize> = (0..joints.len()).collect();
        self.solve_joints_subset(bodies, joints, &indices, delta_time)
    }

    /// Solve only the joints listed in `subset`, in ascending index order.
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
        debug_assert!(delta_time >= 0.0 && delta_time.is_finite());
        let settings = self.settings();
        let bias_scale = if delta_time > 0.0 {
            settings.baumgarte / delta_time
        } else {
            0.0
        };

        // Motors are applied once per solve, before the sweeps (see
        // `solve_hinge_motor`); limits run inside the sweeps like any other
        // constraint row.
        for &index in subset {
            if let JointConstraint::Hinge(joint) = &joints[index] {
                if let Some(motor) = joint.motor {
                    solve_hinge_motor(bodies, joint, motor);
                }
            }
        }

        for _ in 0..settings.iterations {
            for &index in subset {
                match joints[index] {
                    JointConstraint::BallAndSocket(joint) => {
                        solve_point_constraint(
                            bodies,
                            joint.body_a,
                            joint.body_b,
                            joint.pivot_a,
                            joint.pivot_b,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Hinge(joint) => {
                        solve_point_constraint(
                            bodies,
                            joint.body_a,
                            joint.body_b,
                            joint.pivot_a,
                            joint.pivot_b,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                        solve_hinge_axes(bodies, &joint, bias_scale, settings.max_recovery_speed);
                        if let Some(limits) = joint.limits {
                            solve_hinge_limits(
                                bodies,
                                &joint,
                                limits,
                                bias_scale,
                                settings.max_recovery_speed,
                            );
                        }
                    }
                    JointConstraint::Fixed(joint) => {
                        solve_point_constraint(
                            bodies,
                            joint.body_a,
                            joint.body_b,
                            joint.pivot_a,
                            joint.pivot_b,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                        solve_fixed_axes(bodies, &joint, bias_scale, settings.max_recovery_speed);
                    }
                    JointConstraint::Ragdoll(joint) => {
                        solve_point_constraint(
                            bodies,
                            joint.body_a,
                            joint.body_b,
                            joint.pivot_a,
                            joint.pivot_b,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                        solve_ragdoll_cone(bodies, &joint, bias_scale, settings.max_recovery_speed);
                        solve_angular_limit(
                            bodies,
                            joint.body_a,
                            joint.body_b,
                            normalized(joint.twist_axis_a),
                            joint.twist_angle,
                            joint.twist_min,
                            joint.twist_max,
                            bias_scale,
                            settings.max_recovery_speed,
                        );
                    }
                    JointConstraint::Prismatic(joint) => {
                        solve_prismatic(bodies, &joint, bias_scale, settings.max_recovery_speed);
                    }
                    JointConstraint::Spring(joint) => {
                        solve_spring(bodies, &joint, bias_scale, settings.max_recovery_speed);
                    }
                    JointConstraint::PointToPlane(joint) => {
                        solve_point_to_plane(
                            bodies,
                            &joint,
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

/// Pull the two world pivots together along the world axes.
fn solve_point_constraint(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    bias_scale: f32,
    max_bias: f32,
) {
    solve_linear_axes(
        bodies,
        body_a,
        body_b,
        pivot_a,
        pivot_b,
        &[Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z],
        bias_scale,
        max_bias,
    );
}

/// Pull the two pivots together along each of `axes`.
#[allow(clippy::too_many_arguments)]
fn solve_linear_axes(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axes: &[Vector4],
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
    for axis in axes {
        let relative = bodies[a].velocity_at(pivot_a) - bodies[b].velocity_at(pivot_b);
        let mass = linear_effective_mass(bodies, a, b, *axis, pivot_a, pivot_b);
        if mass <= 1e-12 {
            continue;
        }
        // Constraint: relative velocity along the axis must equal
        // -(separation * bias_scale) so the pivots converge.
        let bias = (separation.dot3(*axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (-bias - relative.dot3(*axis)) / mass;
        let impulse = *axis * lambda;
        bodies[a].apply_impulse_at(pivot_a, impulse);
        bodies[b].apply_impulse_at(pivot_b, -impulse);
    }
}

/// Unilateral linear limit along `axis`.
///
/// `translation` grows with the relative velocity of A's pivot with respect to
/// B's pivot along the axis; a restoring impulse is applied only when the
/// translation is outside `[min_translation, max_translation]`, and never in
/// the direction that would increase the violation.
#[allow(clippy::too_many_arguments)]
fn solve_linear_limit(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    pivot_a: Vector4,
    pivot_b: Vector4,
    axis: Vector4,
    translation: f32,
    min_translation: f32,
    max_translation: f32,
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
    let relative = bodies[a].velocity_at(pivot_a) - bodies[b].velocity_at(pivot_b);
    let mass = linear_effective_mass(bodies, a, b, axis, pivot_a, pivot_b);
    if mass <= 1e-12 {
        return;
    }
    let upper_violation = translation - max_translation;
    let lower_violation = min_translation - translation;
    let (target, sign) = if upper_violation > 0.0 {
        (-(upper_violation * bias_scale).clamp(0.0, max_bias), -1.0)
    } else if lower_violation > 0.0 {
        ((lower_violation * bias_scale).clamp(0.0, max_bias), 1.0)
    } else {
        return;
    };
    let lambda = (target - relative.dot3(axis)) / mass;
    if lambda * sign < 0.0 {
        return;
    }
    let impulse = axis * lambda;
    bodies[a].apply_impulse_at(pivot_a, impulse);
    bodies[b].apply_impulse_at(pivot_b, -impulse);
}

/// Prismatic joint: lock rotation and two translation directions, and limit
/// travel along the slider axis.
fn solve_prismatic(
    bodies: &mut [SolverBody],
    joint: &PrismaticJoint,
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
    let axis = normalized(joint.axis_a);
    let [u, v] = perpendicular_axes(axis);
    solve_linear_axes(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        &[u, v],
        bias_scale,
        max_bias,
    );
    solve_relative_angular_lock(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.orientation_error,
        bias_scale,
        max_bias,
    );
    solve_linear_limit(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        axis,
        joint.translation,
        joint.min_translation,
        joint.max_translation,
        bias_scale,
        max_bias,
    );
}

/// Stiff spring: drive the pivot distance toward `rest_length`.
fn solve_spring(bodies: &mut [SolverBody], joint: &SpringJoint, bias_scale: f32, max_bias: f32) {
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
    let impulse = direction * lambda;
    // +lambda on B pulls it toward A when the spring is stretched.
    bodies[b].apply_impulse_at(joint.pivot_b, impulse);
    bodies[a].apply_impulse_at(joint.pivot_a, -impulse);
}

/// Point-to-plane: keep A's pivot within the signed distance range along the
/// plane normal. An equal min/max is solved as a bilateral row.
fn solve_point_to_plane(
    bodies: &mut [SolverBody],
    joint: &PointToPlaneJoint,
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
        let impulse = normal * lambda;
        bodies[a].apply_impulse_at(joint.pivot_a, impulse);
        bodies[b].apply_impulse_at(joint.pivot_b, -impulse);
        return;
    }
    solve_linear_limit(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.pivot_a,
        joint.pivot_b,
        normal,
        distance,
        joint.min_distance,
        joint.max_distance,
        bias_scale,
        max_bias,
    );
}

/// Keep the two hinge axes aligned (rotation about the hinge stays free).
fn solve_hinge_axes(bodies: &mut [SolverBody], joint: &HingeJoint, bias_scale: f32, max_bias: f32) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b {
        return;
    }
    let axis_a = normalized(joint.axis_a);
    let axis_b = normalized(joint.axis_b);
    // Misalignment error: rotating A about this vector (axis * sin(angle))
    // brings axis_a onto axis_b.
    let error = axis_a.cross(axis_b);
    let [u, v] = perpendicular_axes(axis_a);
    for axis in [u, v] {
        let relative = bodies[a].angular_velocity - bodies[b].angular_velocity;
        let mass = angular_effective_mass(bodies, a, b, axis);
        if mass <= 1e-12 {
            continue;
        }
        let bias = (error.dot3(axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (bias - relative.dot3(axis)) / mass;
        apply_angular_impulse(bodies, a, b, axis, lambda);
    }
}

/// Drive the relative angular velocity of B about A along the hinge axis
/// toward the motor target with one clamped impulse.
///
/// The SDK's motors are force-based and integrated over the step; this subset
/// applies a single impulse per solve (before the constraint sweeps) clamped
/// to `±max_impulse`, which keeps the result deterministic and iteration
/// count independent.
fn solve_hinge_motor(bodies: &mut [SolverBody], joint: &HingeJoint, motor: HingeMotor) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
    ) else {
        return;
    };
    if a == b || motor.max_impulse <= 0.0 {
        return;
    }
    let axis = normalized(joint.axis_a);
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        return;
    }
    // Relative velocity of B with respect to A along the hinge axis.
    let rel = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);
    let lambda =
        ((motor.target_velocity - rel) / mass).clamp(-motor.max_impulse, motor.max_impulse);
    apply_angular_impulse(bodies, b, a, axis, lambda);
}

/// Unilateral hinge-angle limits: only push back when the angle is outside
/// `[min_angle, max_angle]`.
///
/// The angle grows with the relative angular velocity of B about A along the
/// hinge axis, so the upper bound drives that velocity negative and the
/// lower bound positive. Inside the dead zone the limit row is skipped
/// entirely, and an impulse that would push the angle further out is never
/// applied (unilateral sign check).
fn solve_hinge_limits(
    bodies: &mut [SolverBody],
    joint: &HingeJoint,
    limits: HingeLimits,
    bias_scale: f32,
    max_bias: f32,
) {
    solve_angular_limit(
        bodies,
        joint.body_a,
        joint.body_b,
        normalized(joint.axis_a),
        limits.angle,
        limits.min_angle,
        limits.max_angle,
        bias_scale,
        max_bias,
    );
}

/// Unilateral angular limit about `axis`: only push back when `angle` is
/// outside `[min_angle, max_angle]`.
///
/// The angle grows with the relative angular velocity of B about A along the
/// axis, so the upper bound drives that velocity negative and the lower bound
/// positive. Inside the dead zone the row is skipped entirely, and an impulse
/// that would push the angle further out is never applied.
#[allow(clippy::too_many_arguments)]
fn solve_angular_limit(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    axis: Vector4,
    angle: f32,
    min_angle: f32,
    max_angle: f32,
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
        return;
    }
    // Relative velocity of B with respect to A along the axis: the angle grows
    // at this rate.
    let rel = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);

    let upper_violation = angle - max_angle;
    let lower_violation = min_angle - angle;
    let (target, sign) = if upper_violation > 0.0 {
        // Past the upper bound: the restoring relative velocity is negative.
        (-(upper_violation * bias_scale).clamp(0.0, max_bias), -1.0)
    } else if lower_violation > 0.0 {
        ((lower_violation * bias_scale).clamp(0.0, max_bias), 1.0)
    } else {
        return;
    };

    let lambda = (target - rel) / mass;
    if lambda * sign < 0.0 {
        return;
    }
    apply_angular_impulse(bodies, b, a, axis, lambda);
}

/// Ragdoll cone limit: keep the swing angle between the twist axes at or below
/// `cone_angle`.
///
/// The correction axis is `twist_a × twist_b`; a positive rotation of A about
/// it swings A's twist axis toward B's, so the row has the same unilateral
/// shape as an upper angular limit.
fn solve_ragdoll_cone(
    bodies: &mut [SolverBody],
    joint: &RagdollJoint,
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
        return;
    }
    let twist_a = normalized(joint.twist_axis_a);
    let twist_b = normalized(joint.twist_axis_b);
    let axis = normalized(twist_a.cross(twist_b));
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        return;
    }
    let rel = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);
    let target = -(violation * bias_scale).clamp(0.0, max_bias);
    let lambda = (target - rel) / mass;
    if lambda >= 0.0 {
        return;
    }
    apply_angular_impulse(bodies, b, a, axis, lambda);
}

/// Zero the relative angular velocity about the world axes, biased by the
/// supplied orientation error.
fn solve_fixed_axes(bodies: &mut [SolverBody], joint: &FixedJoint, bias_scale: f32, max_bias: f32) {
    solve_relative_angular_lock(
        bodies,
        joint.body_a,
        joint.body_b,
        joint.orientation_error,
        bias_scale,
        max_bias,
    );
}

/// Lock relative rotation about all world axes, biased by `orientation_error`.
fn solve_relative_angular_lock(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    orientation_error: Vector4,
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
    for axis in [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z] {
        let relative = bodies[a].angular_velocity - bodies[b].angular_velocity;
        let mass = angular_effective_mass(bodies, a, b, axis);
        if mass <= 1e-12 {
            continue;
        }
        let bias = (orientation_error.dot3(axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (bias - relative.dot3(axis)) / mass;
        apply_angular_impulse(bodies, a, b, axis, lambda);
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
    use crate::solver::Solver;
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
        joint.limits = Some(HingeLimits {
            angle: 0.5,
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
        joint.limits = Some(HingeLimits {
            angle: -0.5,
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
        joint.limits = Some(HingeLimits {
            angle: 0.1,
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
        joint.limits = Some(HingeLimits {
            angle: 0.5,
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
        joint.motor = Some(HingeMotor {
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
        joint.motor = Some(HingeMotor {
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
        joint.motor = Some(HingeMotor {
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
            joint.limits = Some(HingeLimits {
                angle: 0.5,
                min_angle: -0.2,
                max_angle: 0.2,
            });
            joint.motor = Some(HingeMotor {
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
}
