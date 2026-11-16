//! World-level joint constraints (`hknpConstraint` / `hknpConstraintManager`
//! subset).
//!
//! Ports the construction-info layer of
//! `original/Physics/Source/Physics/Physics/Dynamics/Constraint/Data/` for the
//! three joint families the solver supports: ball-and-socket, hinge and fixed.
//!
//! Pivots and axes are stored in the **body-local (shape) space** of each
//! side; every step the world transforms them to world space and builds
//! [`phys_solver::JointConstraint`] entries for the solver. The fixed joint
//! additionally stores the bound relative orientation (rotation of B relative
//! to A's frame) and converts the current orientation error into a world-space
//! axis-angle vector.
//!
//! Constraints live in a slot table indexed by [`phys_types::ConstraintId`].
//! Slots are never reused (`destroy_constraint` leaves a hole), so an ID always
//! refers to the same constraint and stale IDs cannot alias — the SDK's
//! constraint groups and connectivity graph are not ported.

use base_math::{Quaternion, Transform, Vector4};
use phys_types::{BodyId, ConstraintId, ConstraintIndex};

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
/// ([`HingeCinfo::limits`]) and velocity motor ([`HingeCinfo::motor`]).
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
    /// Optional angle limits (measured about the hinge axis between the
    /// reference/twist axes; `None` is unrestricted).
    pub limits: Option<HingeLimitsCinfo>,
    /// Optional velocity motor about the hinge axis (`None` is frictionless).
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
            limits: None,
            motor: None,
        }
    }

    /// Attach angle limits (`setAngularLimits`).
    pub fn with_limits(mut self, limits: HingeLimitsCinfo) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Attach a velocity motor about the hinge axis.
    pub fn with_motor(mut self, motor: HingeMotorCinfo) -> Self {
        self.motor = Some(motor);
        self
    }
}

/// Hinge angle limits (`hknpHingeConstraintData` limits subset).
///
/// The signed angle is measured between `twist_axis_a` and `twist_axis_b`
/// about the hinge axis, in radians, positive in the sense of the hinge axes.
/// Both twist axes must be unit length in their bodies' local spaces and not
/// parallel to the hinge axes (they are projected onto the plane
/// perpendicular to the hinge).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeLimitsCinfo {
    /// Reference axis in body A's local space (unit, not parallel to `axis_a`).
    pub twist_axis_a: Vector4,
    /// Reference axis in body B's local space (unit, not parallel to `axis_b`).
    pub twist_axis_b: Vector4,
    /// Lower angle bound in radians.
    pub min_angle: f32,
    /// Upper angle bound in radians (>= `min_angle`).
    pub max_angle: f32,
}

impl HingeLimitsCinfo {
    /// Construct limits from local reference axes and bounds.
    pub fn new(
        twist_axis_a: Vector4,
        twist_axis_b: Vector4,
        min_angle: f32,
        max_angle: f32,
    ) -> Self {
        debug_assert!(
            min_angle <= max_angle,
            "hinge limit min_angle must not exceed max_angle"
        );
        Self {
            twist_axis_a,
            twist_axis_b,
            min_angle,
            max_angle,
        }
    }
}

/// Velocity motor about the hinge axis (`hknpVelocityConstraintMotor` subset).
///
/// Drives the relative angular velocity of B relative to A about the hinge
/// axis toward `target_velocity`; the impulse applied per solve is clamped
/// to `±max_impulse`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeMotorCinfo {
    /// Desired relative angular velocity about the hinge axis (rad/s).
    pub target_velocity: f32,
    /// Maximum impulse magnitude applied per solve.
    pub max_impulse: f32,
}

impl HingeMotorCinfo {
    /// Construct a motor with the given target and impulse clamp.
    pub fn new(target_velocity: f32, max_impulse: f32) -> Self {
        Self {
            target_velocity,
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
/// `twist_min`/`twist_max` bound the twist angle. The SDK's plane-angle limits
/// are not ported.
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
        }
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

/// Prismatic joint construction info (`hknpPrismaticConstraintData` subset).
///
/// Translation along the slider axis is limited to
/// `[min_translation, max_translation]`; rotation and the other two
/// translation directions are locked to the bound relative pose.
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

/// Any world-level joint constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WorldJoint {
    /// Point-to-point.
    BallAndSocket(BallAndSocketCinfo),
    /// Point-to-point plus hinge-axis alignment.
    Hinge(HingeCinfo),
    /// Point-to-point plus locked relative orientation (velocity level).
    Fixed(FixedCinfo),
    /// Cone/twist limited ragdoll joint.
    Ragdoll(RagdollCinfo),
    /// Limited slider.
    Prismatic(PrismaticCinfo),
    /// Distance-maintaining spring.
    Spring(SpringCinfo),
    /// Point pinned to a plane (optionally a distance range).
    PointToPlane(PointToPlaneCinfo),
}

impl WorldJoint {
    /// The two connected bodies.
    pub fn bodies(&self) -> (BodyId, BodyId) {
        match self {
            Self::BallAndSocket(info) => (info.body_a, info.body_b),
            Self::Hinge(info) => (info.body_a, info.body_b),
            Self::Fixed(info) => (info.body_a, info.body_b),
            Self::Ragdoll(info) => (info.body_a, info.body_b),
            Self::Prismatic(info) => (info.body_a, info.body_b),
            Self::Spring(info) => (info.body_a, info.body_b),
            Self::PointToPlane(info) => (info.body_a, info.body_b),
        }
    }

    /// The joint family (`hknpConstraintData::ConstraintType` subset).
    pub fn joint_type(&self) -> JointType {
        match self {
            Self::BallAndSocket(_) => JointType::BallAndSocket,
            Self::Hinge(_) => JointType::Hinge,
            Self::Fixed(_) => JointType::Fixed,
            Self::Ragdoll(_) => JointType::Ragdoll,
            Self::Prismatic(_) => JointType::Prismatic,
            Self::Spring(_) => JointType::Spring,
            Self::PointToPlane(_) => JointType::PointToPlane,
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
    /// Fixed.
    Fixed,
    /// Ragdoll cone/twist.
    Ragdoll,
    /// Prismatic slider.
    Prismatic,
    /// Stiff spring.
    Spring,
    /// Point-to-plane.
    PointToPlane,
}

/// A stored joint plus its flags.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointSlot {
    pub joint: WorldJoint,
    pub enabled: bool,
    pub collisions_enabled: bool,
}

/// Build the solver-visible [`ConstraintId`] of a slot index.
pub(crate) fn constraint_id(slot: usize) -> ConstraintId {
    ConstraintId::from_index(ConstraintIndex::new(slot as u32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use base_math::Quaternion;

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
