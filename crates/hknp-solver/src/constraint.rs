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
//!
//! Rotation about a hinge axis and free spinning of a ball-and-socket remain
//! unconstrained; springs, ragdoll-style multi-axis limits and constraint
//! islands are not ported.
//!
//! # Determinism
//!
//! Joints are processed in slice order and constraint rows in the fixed order
//! documented above, matching the contact solver's insertion-order sweep.

use hk_math::Vector4;

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

/// Any solver-level joint constraint, dispatched to a specialized solve path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointConstraint {
    /// Point-to-point.
    BallAndSocket(BallAndSocketJoint),
    /// Point-to-point plus hinge-axis alignment.
    Hinge(HingeJoint),
    /// Point-to-point plus locked relative angular velocity.
    Fixed(FixedJoint),
}

impl JointConstraint {
    /// The two body indices.
    pub fn bodies(&self) -> (usize, usize) {
        match self {
            Self::BallAndSocket(joint) => (joint.body_a, joint.body_b),
            Self::Hinge(joint) => (joint.body_a, joint.body_b),
            Self::Fixed(joint) => (joint.body_a, joint.body_b),
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
        for joint in joints {
            if let JointConstraint::Hinge(joint) = joint {
                if let Some(motor) = joint.motor {
                    solve_hinge_motor(bodies, joint, motor);
                }
            }
        }

        for _ in 0..settings.iterations {
            for joint in joints {
                match *joint {
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
    for axis in [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z] {
        let relative = bodies[a].velocity_at(pivot_a) - bodies[b].velocity_at(pivot_b);
        let mass = linear_effective_mass(bodies, a, b, axis, pivot_a, pivot_b);
        if mass <= 1e-12 {
            continue;
        }
        // Constraint: relative velocity along the axis must equal
        // -(separation * bias_scale) so the pivots converge.
        let bias = (separation.dot3(axis) * bias_scale).clamp(-max_bias, max_bias);
        let lambda = (-bias - relative.dot3(axis)) / mass;
        let impulse = axis * lambda;
        bodies[a].apply_impulse_at(pivot_a, impulse);
        bodies[b].apply_impulse_at(pivot_b, -impulse);
    }
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
    let mass = angular_effective_mass(bodies, a, b, axis);
    if mass <= 1e-12 {
        return;
    }
    // Relative velocity of B with respect to A along the hinge axis: the
    // hinge angle grows at this rate.
    let rel = (bodies[b].angular_velocity - bodies[a].angular_velocity).dot3(axis);

    let upper_violation = limits.angle - limits.max_angle;
    let lower_violation = limits.min_angle - limits.angle;
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

/// Zero the relative angular velocity about the world axes, biased by the
/// supplied orientation error.
fn solve_fixed_axes(bodies: &mut [SolverBody], joint: &FixedJoint, bias_scale: f32, max_bias: f32) {
    let (Ok(a), Ok(b)) = (
        body_index(bodies.len(), joint.body_a),
        body_index(bodies.len(), joint.body_b),
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
        let bias = (joint.orientation_error.dot3(axis) * bias_scale).clamp(-max_bias, max_bias);
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
    use hk_math::Quaternion;

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
}
