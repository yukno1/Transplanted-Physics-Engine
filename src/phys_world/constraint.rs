//! World-level joint constraints (`hknpConstraint` / `hknpConstraintManager`
//! subset).
//!
//! Ports the construction-info layer of
//! `original/Physics/Source/Physics/Physics/Dynamics/Constraint/Data/` for the
//! three joint families the solver supports: ball-and-socket, hinge and fixed.
//!
//! Pivots and axes are stored in the **body-local (shape) space** of each
//! side; every step the world transforms them to world space and builds
//! [`proton::solver::JointConstraint`] entries for the solver. The fixed joint
//! additionally stores the bound relative orientation (rotation of B relative
//! to A's frame) and converts the current orientation error into a world-space
//! axis-angle vector.
//!
//! Constraints live in a slot table indexed by [`proton::np_types::ConstraintId`].
//! Slots are never reused (`destroy_constraint` leaves a hole), so an ID always
//! refers to the same constraint and stale IDs cannot alias — the SDK's
//! constraint groups and connectivity graph are not ported.

use crate::base_math::{Quaternion, Transform, Vector4};
use crate::phys_solver::JointImpulse;
use crate::phys_types::{BodyId, ConstraintId, ConstraintIndex};

/// Ball-and-socket joint construction info (`hknpBallAndSocketConstraintData`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallAndSocketCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
}

impl BallAndSocketCinfo {
    /// Construct with explicit local pivots.
    pub fn new(body_a: BodyId, body_b: BodyId, pivot_a: Vector4, pivot_b: Vector4) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
        }
    }
}

/// Hinge joint construction info (`hknpHingeConstraintData`).
///
/// The two axes must be unit length in their bodies' local spaces; rotation
/// about the hinge is free apart from the optional angle limits
/// ([`HingeCinfo::limits`]) and motor ([`HingeCinfo::motor`]).
///
/// The signed hinge angle is measured about the hinge axis between the
/// reference axes ([`HingeCinfo::reference_axis_a`]/[`HingeCinfo::reference_axis_b`]);
/// the defaults are a deterministic perpendicular of each hinge axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Hinge axis in body A's local space (unit).
    pub axis_a: Vector4,
    /// Hinge axis in body B's local space (unit).
    pub axis_b: Vector4,
    /// Reference (twist) axis in body A's local space used to measure the
    /// signed hinge angle (`Vector4::ZERO` uses a deterministic perpendicular
    /// of the hinge axis).
    pub reference_axis_a: Vector4,
    /// Reference axis in body B's local space.
    pub reference_axis_b: Vector4,
    /// Optional angle limits (measured about the hinge axis; `None` is
    /// unrestricted).
    pub limits: Option<HingeLimitsCinfo>,
    /// Optional motor about the hinge axis (`None` is frictionless).
    pub motor: Option<HingeMotorCinfo>,
}

impl HingeCinfo {
    /// Construct with explicit local pivots and axes.
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        axis_a: Vector4,
        axis_b: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a,
            axis_b,
            reference_axis_a: Vector4::ZERO,
            reference_axis_b: Vector4::ZERO,
            limits: None,
            motor: None,
        }
    }

    /// Set the reference (twist) axes used to measure the hinge angle
    /// (`setAngularLimits` reference frame). Both axes must be unit length in
    /// their bodies' local spaces and not parallel to the hinge axes.
    #[must_use]
    pub fn with_reference_axes(mut self, axis_a: Vector4, axis_b: Vector4) -> Self {
        self.reference_axis_a = axis_a;
        self.reference_axis_b = axis_b;
        self
    }

    /// Attach angle limits (`setAngularLimits`).
    pub fn with_limits(mut self, limits: HingeLimitsCinfo) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Attach a motor about the hinge axis.
    pub fn with_motor(mut self, motor: HingeMotorCinfo) -> Self {
        self.motor = Some(motor);
        self
    }
}

/// Hinge angle limits (`hknpHingeConstraintData` limits subset).
///
/// The signed angle (measured about the hinge axis between the reference
/// axes, positive in the sense of the hinge axes) lives on the world step;
/// the solver applies a restoring unilateral impulse only while it is outside
/// `[min_angle, max_angle]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeLimitsCinfo {
    /// Lower angle bound in radians.
    pub min_angle: f32,
    /// Upper angle bound in radians (>= `min_angle`).
    pub max_angle: f32,
}

impl HingeLimitsCinfo {
    /// Construct limits from angle bounds.
    pub fn new(min_angle: f32, max_angle: f32) -> Self {
        debug_assert!(
            min_angle <= max_angle,
            "hinge limit min_angle must not exceed max_angle"
        );
        Self {
            min_angle,
            max_angle,
        }
    }
}

/// Limited-hinge construction info (`hknpLimitedHingeConstraintData`).
///
/// The limited-hinge family shares the hinge construction data (pivots, axes,
/// reference axes and angular limits); unlike a plain hinge it has no motor.
pub type LimitedHingeCinfo = HingeCinfo;

/// Motor about the hinge axis (`hknpAngularConstraintMotor` subset).
///
/// Every variant is applied once per solve as a single impulse clamped to
/// `±max_impulse`; the spring variant integrates its force over the step
/// (`impulse = force * delta_time`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HingeMotorCinfo {
    /// Drive the relative angular velocity of B relative to A about the hinge
    /// axis toward a target (`hknpVelocityConstraintMotor` subset).
    Velocity(HingeVelocityMotorCinfo),
    /// Drive the hinge angle toward a target (`hknpPositionConstraintMotor`
    /// subset): the motor drives the relative angular velocity toward
    /// `angle_error / tau`.
    Position(HingePositionMotorCinfo),
    /// Spring-damper pulling the hinge angle toward a rest angle
    /// (`hknpSpringConstraintMotor` subset).
    Spring(HingeSpringMotorCinfo),
}

/// Velocity motor about the hinge axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeVelocityMotorCinfo {
    /// Desired relative angular velocity about the hinge axis (rad/s).
    pub target_velocity: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl HingeVelocityMotorCinfo {
    /// Construct a motor with the given target and impulse clamp.
    pub fn new(target_velocity: f32, max_impulse: f32) -> Self {
        Self {
            target_velocity,
            max_impulse,
        }
    }
}

/// Position motor about the hinge axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingePositionMotorCinfo {
    /// Desired hinge angle in radians (signed about the hinge axis).
    pub target_angle: f32,
    /// Time constant of the position error in seconds (values `<= 1e-4`
    /// disable the motor).
    pub tau: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl HingePositionMotorCinfo {
    /// Construct a position motor.
    pub fn new(target_angle: f32, tau: f32, max_impulse: f32) -> Self {
        Self {
            target_angle,
            tau,
            max_impulse,
        }
    }
}

/// Spring motor about the hinge axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeSpringMotorCinfo {
    /// Rest angle in radians.
    pub target_angle: f32,
    /// Spring stiffness (torque per radian).
    pub stiffness: f32,
    /// Damping coefficient (torque per rad/s).
    pub damping: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl HingeSpringMotorCinfo {
    /// Construct a spring motor.
    pub fn new(target_angle: f32, stiffness: f32, damping: f32, max_impulse: f32) -> Self {
        Self {
            target_angle,
            stiffness,
            damping,
            max_impulse,
        }
    }
}

/// Fixed joint construction info (`hknpFixedConstraintData`, velocity level).
///
/// `relative_orientation` is the bound rotation of body B in body A's local
/// frame; orientation drift is corrected by the solver's Baumgarte bias.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Bound rotation of B relative to A's frame.
    pub relative_orientation: Quaternion,
}

impl FixedCinfo {
    /// Construct with explicit pivots and bound relative orientation.
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        relative_orientation: Quaternion,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            relative_orientation,
        }
    }

    /// Construct bound to the given current transforms (the SDK's
    /// "setInBodySpace" convenience): pivots are the world point `pivot`
    /// mapped into each body and the bound orientation is the current
    /// relative rotation.
    pub fn between_bodies(
        body_a: BodyId,
        body_b: BodyId,
        pivot: Vector4,
        transform_a: &Transform,
        transform_b: &Transform,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a: transform_a.inverse_transform_point(pivot),
            pivot_b: transform_b.inverse_transform_point(pivot),
            relative_orientation: transform_a
                .rotation_quaternion()
                .inverse_mul(transform_b.rotation_quaternion()),
        }
    }
}

/// Ragdoll joint construction info (`hknpRagdollConstraintData` cone/twist
/// subset).
///
/// The twist axis defines the primary axis of the joint; `plane_axis_a`/
/// `plane_axis_b` are perpendicular reference axes used to measure the twist
/// angle. `cone_angle` caps the swing angle between the twist axes and
/// `twist_min`/`twist_max` bound the twist angle. `plane_limits`, when set,
/// bounds the signed deviation of the angle between A's twist axis and B's
/// plane axis from 90° (the SDK's `setPlaneMin/MaxAngularLimit` convention).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollCinfo {
    /// First body (the "child" body).
    pub body_a: BodyId,
    /// Second body (the "parent" body).
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Twist axis in body A's local space (unit).
    pub twist_axis_a: Vector4,
    /// Twist axis in body B's local space (unit).
    pub twist_axis_b: Vector4,
    /// Reference (plane) axis in body A's local space (unit, not parallel to
    /// the twist axis).
    pub plane_axis_a: Vector4,
    /// Reference (plane) axis in body B's local space.
    pub plane_axis_b: Vector4,
    /// Maximum swing angle between the twist axes, in radians.
    pub cone_angle: f32,
    /// Lower twist bound in radians.
    pub twist_min: f32,
    /// Upper twist bound in radians.
    pub twist_max: f32,
    /// Signed plane-angle bounds relative to 90°; `None` disables the two
    /// plane rows and keeps the historical behavior.
    pub plane_limits: Option<(f32, f32)>,
}

impl RagdollCinfo {
    /// Construct with explicit local pivots, twist axes and reference axes;
    /// the cone defaults to 45° and the twist range to ±45°.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        twist_axis_a: Vector4,
        twist_axis_b: Vector4,
        plane_axis_a: Vector4,
        plane_axis_b: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            twist_axis_a,
            twist_axis_b,
            plane_axis_a,
            plane_axis_b,
            cone_angle: core::f32::consts::FRAC_PI_4,
            twist_min: -core::f32::consts::FRAC_PI_4,
            twist_max: core::f32::consts::FRAC_PI_4,
            plane_limits: None,
        }
    }

    /// Enable the SDK-style plane-angle limits (deviation from 90° in
    /// radians). The SDK defaults to `-0.025..=0.025`.
    #[must_use]
    pub fn with_plane_limits(mut self, min: f32, max: f32) -> Self {
        debug_assert!(
            min.is_finite() && max.is_finite() && min <= max,
            "invalid ragdoll plane limits"
        );
        self.plane_limits = Some((min, max));
        self
    }

    /// Set the cone and twist limits (`setConeAngle` + `setTwistLimits`).
    #[must_use]
    pub fn with_limits(mut self, cone_angle: f32, twist_min: f32, twist_max: f32) -> Self {
        debug_assert!(
            cone_angle >= 0.0 && twist_min <= twist_max,
            "invalid ragdoll limits"
        );
        self.cone_angle = cone_angle;
        self.twist_min = twist_min;
        self.twist_max = twist_max;
        self
    }
}

#[inline]
fn unit_axis(vector: Vector4) -> Vector4 {
    let length = vector.length3();
    if length > 1e-10 {
        vector * (1.0 / length)
    } else {
        Vector4::UNIT_X
    }
}

/// Build the world-space 6DOF constraint frame from twist and plane axes
/// (`hknp6DofConstraintData::getConstraintFrameA/B`).
///
/// Columns are `[twist, plane, twist × plane]`; the plane axis is projected onto
/// the twist plane and falls back to a deterministic perpendicular when
/// degenerate. This is the single source of truth for world construction,
/// solver rows, warm-start replay and ragdoll easing.
pub fn six_dof_constraint_frame_axes(twist: Vector4, plane: Vector4) -> [Vector4; 3] {
    let twist = unit_axis(twist);
    let projected = plane - twist * plane.dot3(twist);
    let plane = if projected.length3() > 1e-10 {
        unit_axis(projected)
    } else {
        let helper = if twist.x.abs() < 0.9 {
            Vector4::UNIT_X
        } else {
            Vector4::UNIT_Y
        };
        unit_axis(twist.cross(helper))
    };
    let binormal = unit_axis(twist.cross(plane));
    [twist, plane, binormal]
}

/// Per-axis translation mode (`hknp6DofConstraintData::AxisMode` subset).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SixDofAxisMode {
    /// No row for this axis.
    Unlimited,
    /// Clamp the axis translation to `[min, max]`.
    Limited,
    /// Lock the axis by using a zero-width `[0, 0]` interval.
    Fixed,
}

/// Limits of a 6DOF joint (`hknp6DofConstraintData` limits subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SixDofLimitsCinfo {
    /// Lower twist bound in radians.
    pub twist_min: f32,
    /// Upper twist bound in radians.
    pub twist_max: f32,
    /// Maximum absolute swing about the first perpendicular axis.
    pub swing0_max: f32,
    /// Maximum absolute swing about the second perpendicular axis.
    pub swing1_max: f32,
    /// Maximum distance between the two pivots (stiff-spring row).
    pub linear_limit: f32,
    /// Per-axis modes in constraint-frame order `[twist, plane, twist × plane]`.
    pub linear_axes: [SixDofAxisMode; 3],
    /// Per-axis lower translations, used when the mode is `Limited`.
    pub linear_min: [f32; 3],
    /// Per-axis upper translations, used when the mode is `Limited`.
    pub linear_max: [f32; 3],
}

impl SixDofLimitsCinfo {
    /// Construct limits from twist, swing and linear bounds. Per-axis
    /// translations default to `Unlimited` so existing scenes are unchanged.
    pub fn new(
        twist_min: f32,
        twist_max: f32,
        swing0_max: f32,
        swing1_max: f32,
        linear_limit: f32,
    ) -> Self {
        debug_assert!(
            twist_min <= twist_max && swing0_max >= 0.0 && swing1_max >= 0.0 && linear_limit >= 0.0,
            "invalid 6DOF limits"
        );
        Self {
            twist_min,
            twist_max,
            swing0_max,
            swing1_max,
            linear_limit,
            linear_axes: [SixDofAxisMode::Unlimited; 3],
            linear_min: [f32::NEG_INFINITY; 3],
            linear_max: [f32::INFINITY; 3],
        }
    }

    /// Set one axis mode and its interval (`setAxisMode` + `setAxisLimits`).
    #[must_use]
    pub fn with_linear_axis(
        mut self,
        axis: usize,
        mode: SixDofAxisMode,
        min: f32,
        max: f32,
    ) -> Self {
        debug_assert!(axis < 3, "6DOF axis must be 0, 1 or 2");
        if axis >= 3 {
            return self;
        }
        self.linear_axes[axis] = mode;
        self.linear_min[axis] = min;
        self.linear_max[axis] = max.max(min);
        self
    }
}

impl Default for SixDofLimitsCinfo {
    fn default() -> Self {
        Self {
            twist_min: -core::f32::consts::FRAC_PI_4,
            twist_max: core::f32::consts::FRAC_PI_4,
            swing0_max: core::f32::consts::FRAC_PI_4,
            swing1_max: core::f32::consts::FRAC_PI_4,
            linear_limit: 0.25,
            linear_axes: [SixDofAxisMode::Unlimited; 3],
            linear_min: [f32::NEG_INFINITY; 3],
            linear_max: [f32::INFINITY; 3],
        }
    }
}

/// 6DOF joint construction info (`hknp6DofConstraintData` subset).
///
/// The two pivots are kept within [`SixDofLimitsCinfo::linear_limit`] and the
/// relative orientation is bounded by twist and two swing limits. Optional
/// per-axis translation limits are configured through
/// [`SixDofLimitsCinfo::with_linear_axis`]. Angular plane limits are not
/// part of this joint family.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SixDofCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Twist axis in body A's local space (unit).
    pub twist_axis_a: Vector4,
    /// Twist axis in body B's local space (unit).
    pub twist_axis_b: Vector4,
    /// Reference (plane) axis in body A's local space.
    pub plane_axis_a: Vector4,
    /// Reference (plane) axis in body B's local space.
    pub plane_axis_b: Vector4,
    /// Twist, swing and linear limits.
    pub limits: SixDofLimitsCinfo,
}

impl SixDofCinfo {
    /// Construct with explicit local pivots, axes and limits.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        twist_axis_a: Vector4,
        twist_axis_b: Vector4,
        plane_axis_a: Vector4,
        plane_axis_b: Vector4,
        limits: SixDofLimitsCinfo,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            twist_axis_a,
            twist_axis_b,
            plane_axis_a,
            plane_axis_b,
            limits,
        }
    }

    /// Replace the joint limits (`setLimits`).
    #[must_use]
    pub fn with_limits(mut self, limits: SixDofLimitsCinfo) -> Self {
        self.limits = limits;
        self
    }
}

/// Prismatic joint construction info (`hknpPrismaticConstraintData` subset).
///
/// Translation along the slider axis is limited to
/// `[min_translation, max_translation]`; rotation and the other two
/// translation directions are locked to the bound relative pose. An optional
/// motor ([`PrismaticCinfo::motor`]) drives the translation toward a target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrismaticCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Slider axis in body A's local space (unit).
    pub axis_a: Vector4,
    /// Slider axis in body B's local space (unit).
    pub axis_b: Vector4,
    /// Bound rotation of B relative to A's frame.
    pub relative_orientation: Quaternion,
    /// Lower translation bound (may be `-inf`).
    pub min_translation: f32,
    /// Upper translation bound (may be `+inf`).
    pub max_translation: f32,
    /// Optional position/spring motor along the slider axis.
    pub motor: Option<PrismaticMotorCinfo>,
}

impl PrismaticCinfo {
    /// Construct with explicit local pivots/axes and unbounded travel.
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        axis_a: Vector4,
        axis_b: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a,
            axis_b,
            relative_orientation: Quaternion::IDENTITY,
            min_translation: f32::NEG_INFINITY,
            max_translation: f32::INFINITY,
            motor: None,
        }
    }

    /// Construct bound to the current relative pose of the two bodies (the
    /// SDK's `setInBodySpace`/`setInWorldSpace` convenience).
    pub fn between_bodies(
        body_a: BodyId,
        body_b: BodyId,
        pivot: Vector4,
        transform_a: &Transform,
        transform_b: &Transform,
        axis: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a: transform_a.inverse_transform_point(pivot),
            pivot_b: transform_b.inverse_transform_point(pivot),
            axis_a: transform_a.rotation.rotate_inverse(axis),
            axis_b: transform_b.rotation.rotate_inverse(axis),
            relative_orientation: transform_a
                .rotation_quaternion()
                .inverse_mul(transform_b.rotation_quaternion()),
            min_translation: f32::NEG_INFINITY,
            max_translation: f32::INFINITY,
            motor: None,
        }
    }

    /// Set the travel limits (`setLimits`).
    #[must_use]
    pub fn with_limits(mut self, min_translation: f32, max_translation: f32) -> Self {
        debug_assert!(
            min_translation <= max_translation,
            "invalid prismatic limits"
        );
        self.min_translation = min_translation;
        self.max_translation = max_translation;
        self
    }

    /// Attach a motor along the slider axis.
    #[must_use]
    pub fn with_motor(mut self, motor: PrismaticMotorCinfo) -> Self {
        self.motor = Some(motor);
        self
    }
}

/// Motor along the slider axis (`hknpLinearConstraintMotor` subset).
///
/// Applied once per solve as a clamped impulse; the translation grows with
/// the relative velocity of A's pivot with respect to B's pivot along the
/// slider axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PrismaticMotorCinfo {
    /// Drive the slider translation toward a target
    /// (`hknpPositionConstraintMotor` subset).
    Position(PrismaticPositionMotorCinfo),
    /// Spring-damper pulling the slider translation toward a rest value
    /// (`hknpSpringConstraintMotor` subset).
    Spring(PrismaticSpringMotorCinfo),
}

/// Position motor along the slider axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrismaticPositionMotorCinfo {
    /// Desired slider translation.
    pub target_translation: f32,
    /// Time constant of the position error in seconds (values `<= 1e-4`
    /// disable the motor).
    pub tau: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl PrismaticPositionMotorCinfo {
    /// Construct a position motor.
    pub fn new(target_translation: f32, tau: f32, max_impulse: f32) -> Self {
        Self {
            target_translation,
            tau,
            max_impulse,
        }
    }
}

/// Spring motor along the slider axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrismaticSpringMotorCinfo {
    /// Rest translation.
    pub target_translation: f32,
    /// Spring stiffness (force per meter).
    pub stiffness: f32,
    /// Damping coefficient (force per m/s).
    pub damping: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl PrismaticSpringMotorCinfo {
    /// Construct a spring motor.
    pub fn new(target_translation: f32, stiffness: f32, damping: f32, max_impulse: f32) -> Self {
        Self {
            target_translation,
            stiffness,
            damping,
            max_impulse,
        }
    }
}

/// Stiff spring construction info (`hknpStiffSpringConstraintData`).
///
/// The distance between the two pivots is driven toward `rest_length`; the
/// bodies may rotate freely about each other.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpringCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Target pivot distance.
    pub rest_length: f32,
}

impl SpringCinfo {
    /// Construct with explicit local pivots and rest length.
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        rest_length: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            rest_length,
        }
    }

    /// Construct from a shared world pivot pair at the current pose: the rest
    /// length is the current pivot distance.
    pub fn between_bodies(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        transform_a: &Transform,
        transform_b: &Transform,
    ) -> Self {
        let world_a = transform_a.transform_point(pivot_a);
        let world_b = transform_b.transform_point(pivot_b);
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            rest_length: (world_b - world_a).length3(),
        }
    }
}

/// Point-to-plane construction info (`hknpPointToPlaneConstraintData`).
///
/// A's pivot must stay within the signed-distance range along B's plane
/// normal; a zero-width range pins it to the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointToPlaneCinfo {
    /// First body.
    pub body_a: BodyId,
    /// Second body.
    pub body_b: BodyId,
    /// Pivot in body A's local space.
    pub pivot_a: Vector4,
    /// Pivot in body B's local space.
    pub pivot_b: Vector4,
    /// Plane normal in body B's local space (unit).
    pub plane_normal_b: Vector4,
    /// Minimum signed distance along the plane normal.
    pub min_distance: f32,
    /// Maximum signed distance along the plane normal.
    pub max_distance: f32,
}

impl PointToPlaneCinfo {
    /// Construct a plane-pinned joint (zero-width distance range).
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        plane_normal_b: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            plane_normal_b,
            min_distance: 0.0,
            max_distance: 0.0,
        }
    }

    /// Set the signed-distance range (`setLimits`).
    #[must_use]
    pub fn with_limits(mut self, min_distance: f32, max_distance: f32) -> Self {
        debug_assert!(
            min_distance <= max_distance,
            "invalid point-to-plane limits"
        );
        self.min_distance = min_distance;
        self.max_distance = max_distance;
        self
    }
}

/// Wheel joint construction info (`hknpWheelConstraintData` subset).
///
/// Body A is the wheel, body B the chassis. The suspension axis and steering
/// axis live in the chassis's local space; the axle lives in the wheel's local
/// space. Steering ([`WheelCinfo::steering_angle`]) rotates the chassis-side
/// axle about the steering axis; the solver's axle-alignment rows then pull the
/// wheel onto the commanded axle. Drive is a velocity motor about the axle
/// (the SDK resolves drive through the vehicle friction solver, which is not
/// ported).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelCinfo {
    /// Wheel body.
    pub body_a: BodyId,
    /// Chassis body.
    pub body_b: BodyId,
    /// Wheel center in body A's local space.
    pub pivot_a: Vector4,
    /// Suspension hardpoint in body B's local space.
    pub pivot_b: Vector4,
    /// Suspension direction in body B's local space (unit; typically down).
    pub suspension_axis_b: Vector4,
    /// Spin axle in body A's local space (unit).
    pub axle_a: Vector4,
    /// Initial axle in body B's local space (unit). Defaults to
    /// [`WheelCinfo::axle_a`]; use [`WheelCinfo::with_axle_b`] when the two
    /// local frames disagree on the axle direction. Steering rotates this
    /// axle about [`WheelCinfo::steering_axis_b`].
    pub axle_b: Vector4,
    /// Steering rotation axis in body B's local space (unit).
    pub steering_axis_b: Vector4,
    /// Current steering angle in radians (B's axle rotates about
    /// [`WheelCinfo::steering_axis_b`] by this amount).
    pub steering_angle: f32,
    /// Optional suspension spring-damper.
    pub suspension: Option<WheelSuspensionCinfo>,
    /// Lower suspension travel bound (fully compressed; signed distance of
    /// the wheel center relative to the hardpoint along the suspension axis).
    pub min_length: f32,
    /// Upper suspension travel bound (fully extended).
    pub max_length: f32,
    /// Optional drive motor about the axle.
    pub drive: Option<WheelDriveCinfo>,
}

impl WheelCinfo {
    /// Construct with explicit local pivots, suspension/steering axes and
    /// axle; travel is unlimited (`±inf`) and no suspension/drive is attached.
    pub fn new(
        body_a: BodyId,
        body_b: BodyId,
        pivot_a: Vector4,
        pivot_b: Vector4,
        suspension_axis_b: Vector4,
        axle_a: Vector4,
        steering_axis_b: Vector4,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            suspension_axis_b,
            axle_a,
            axle_b: axle_a,
            steering_axis_b,
            steering_angle: 0.0,
            suspension: None,
            min_length: f32::NEG_INFINITY,
            max_length: f32::INFINITY,
            drive: None,
        }
    }

    /// Attach a suspension spring-damper with rest length.
    pub fn with_suspension(mut self, suspension: WheelSuspensionCinfo) -> Self {
        self.suspension = Some(suspension);
        self
    }

    /// Override the chassis-side initial axle (body B's local space).
    pub fn with_axle_b(mut self, axle_b: Vector4) -> Self {
        self.axle_b = axle_b;
        self
    }

    /// Limit the suspension travel (signed lengths along the axis).
    pub fn with_travel(mut self, min_length: f32, max_length: f32) -> Self {
        self.min_length = min_length.min(max_length);
        self.max_length = max_length;
        self
    }

    /// Attach a drive motor about the axle.
    pub fn with_drive(mut self, drive: WheelDriveCinfo) -> Self {
        self.drive = Some(drive);
        self
    }
}

/// Suspension spring-damper construction info
/// (`hknpVehicleDefaultSuspension::WheelSpringSuspensionParameters` force
/// subset applied to the wheel constraint).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelSuspensionCinfo {
    /// Suspension length the spring pulls toward (signed along the axis).
    pub rest_length: f32,
    /// Spring stiffness (force per meter).
    pub stiffness: f32,
    /// Damping coefficient (force per m/s).
    pub damping: f32,
    /// Maximum impulse magnitude the spring may apply per solve.
    pub max_impulse: f32,
}

impl WheelSuspensionCinfo {
    /// Construct with explicit parameters.
    pub fn new(rest_length: f32, stiffness: f32, damping: f32, max_impulse: f32) -> Self {
        Self {
            rest_length,
            stiffness,
            damping,
            max_impulse,
        }
    }
}

/// Wheel drive motor construction info (velocity motor about the axle).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelDriveCinfo {
    /// Target spin velocity of the wheel relative to the chassis, in rad/s
    /// about the axle (signed by the axle direction).
    pub target_spin_velocity: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl WheelDriveCinfo {
    /// Construct with explicit parameters.
    pub fn new(target_spin_velocity: f32, max_impulse: f32) -> Self {
        Self {
            target_spin_velocity,
            max_impulse,
        }
    }
}

/// Any world-level joint constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WorldJoint {
    /// Point-to-point.
    BallAndSocket(BallAndSocketCinfo),
    /// Point-to-point plus hinge-axis alignment.
    Hinge(HingeCinfo),
    /// Limited hinge: hinge geometry with angular limits and no motor.
    LimitedHinge(LimitedHingeCinfo),
    /// Point-to-point plus locked relative orientation (velocity level).
    Fixed(FixedCinfo),
    /// Cone/twist limited ragdoll joint.
    Ragdoll(RagdollCinfo),
    /// 6DOF limits subset (linear distance, twist and two swings).
    SixDof(SixDofCinfo),
    /// Limited slider.
    Prismatic(PrismaticCinfo),
    /// Distance-maintaining spring.
    Spring(SpringCinfo),
    /// Point pinned to a plane (optionally a distance range).
    PointToPlane(PointToPlaneCinfo),
    /// Wheel: suspension slider (spring-damper + travel limits) and a driven
    /// spin axle.
    Wheel(WheelCinfo),
}

impl WorldJoint {
    /// The two connected bodies.
    pub fn bodies(&self) -> (BodyId, BodyId) {
        match self {
            Self::BallAndSocket(info) => (info.body_a, info.body_b),
            Self::Hinge(info) => (info.body_a, info.body_b),
            Self::LimitedHinge(info) => (info.body_a, info.body_b),
            Self::Fixed(info) => (info.body_a, info.body_b),
            Self::Ragdoll(info) => (info.body_a, info.body_b),
            Self::SixDof(info) => (info.body_a, info.body_b),
            Self::Prismatic(info) => (info.body_a, info.body_b),
            Self::Spring(info) => (info.body_a, info.body_b),
            Self::PointToPlane(info) => (info.body_a, info.body_b),
            Self::Wheel(info) => (info.body_a, info.body_b),
        }
    }

    /// The joint family (`hknpConstraintData::ConstraintType` subset).
    pub fn joint_type(&self) -> JointType {
        match self {
            Self::BallAndSocket(_) => JointType::BallAndSocket,
            Self::Hinge(_) => JointType::Hinge,
            Self::LimitedHinge(_) => JointType::LimitedHinge,
            Self::Fixed(_) => JointType::Fixed,
            Self::Ragdoll(_) => JointType::Ragdoll,
            Self::SixDof(_) => JointType::SixDof,
            Self::Prismatic(_) => JointType::Prismatic,
            Self::Spring(_) => JointType::Spring,
            Self::PointToPlane(_) => JointType::PointToPlane,
            Self::Wheel(_) => JointType::Wheel,
        }
    }
}

/// The joint family of a [`WorldJoint`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum JointType {
    /// Point-to-point.
    BallAndSocket,
    /// Hinge.
    Hinge,
    /// Limited hinge (angular limits, no motor).
    LimitedHinge,
    /// Fixed.
    Fixed,
    /// Ragdoll cone/twist.
    Ragdoll,
    /// 6DOF limits subset.
    SixDof,
    /// Prismatic slider.
    Prismatic,
    /// Stiff spring.
    Spring,
    /// Point-to-plane.
    PointToPlane,
    /// Wheel (suspension + driven axle).
    Wheel,
}

/// A stored joint plus its flags and warm-start state.
///
/// `impulses` holds the joint's accumulated solver-row impulses from the most
/// recent step (`JointImpulse` keyed by the constraint's fixed row order);
/// they are replayed before the next step's sweeps and cleared when the
/// constraint is disabled or destroyed (its slot is dropped).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointSlot {
    pub joint: WorldJoint,
    pub enabled: bool,
    pub collisions_enabled: bool,
    pub impulses: JointImpulse,
}

/// Build the solver-visible [`ConstraintId`] of a slot index.
pub(crate) fn constraint_id(slot: usize) -> ConstraintId {
    ConstraintId::from_index(ConstraintIndex::new(slot as u32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::Quaternion;

    #[test]
    fn constraint_ids_are_index_backed() {
        let id = constraint_id(7);
        assert!(id.is_valid());
        assert!(id.is_constraint_index());
        assert_eq!(id.as_constraint_index().value(), 7);
    }

    #[test]
    fn fixed_cinfo_between_bodies_binds_current_pose() {
        let transform_a = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let transform_b = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
            Vector4::from_xyz(2.0, 0.0, 0.0),
        );
        let cinfo = FixedCinfo::between_bodies(
            BodyId::new(1),
            BodyId::new(2),
            Vector4::from_xyz(1.0, 0.0, 0.0),
            &transform_a,
            &transform_b,
        );
        // The shared world pivot maps into both local frames.
        assert!(
            transform_a
                .transform_point(cinfo.pivot_a)
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-5)
        );
        assert!(
            transform_b
                .transform_point(cinfo.pivot_b)
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-5)
        );
        assert!(cinfo.relative_orientation.all_equal(
            Quaternion::IDENTITY.inverse_mul(Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3)),
            1e-5
        ));
    }

    #[test]
    fn world_joint_exposes_bodies() {
        let joint = WorldJoint::BallAndSocket(BallAndSocketCinfo::new(
            BodyId::new(3),
            BodyId::new(4),
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        assert_eq!(joint.bodies(), (BodyId::new(3), BodyId::new(4)));
    }
}
