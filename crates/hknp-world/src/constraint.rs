//! World-level joint constraints (`hknpConstraint` / `hknpConstraintManager`
//! subset).
//!
//! Ports the construction-info layer of
//! `original/Physics/Source/Physics/Physics/Dynamics/Constraint/Data/` for the
//! three joint families the solver supports: ball-and-socket, hinge and fixed.
//!
//! Pivots and axes are stored in the **body-local (shape) space** of each
//! side; every step the world transforms them to world space and builds
//! [`hknp_solver::JointConstraint`] entries for the solver. The fixed joint
//! additionally stores the bound relative orientation (rotation of B relative
//! to A's frame) and converts the current orientation error into a world-space
//! axis-angle vector.
//!
//! Constraints live in a slot table indexed by [`hknp_types::ConstraintId`].
//! Slots are never reused (`destroy_constraint` leaves a hole), so an ID always
//! refers to the same constraint and stale IDs cannot alias — the SDK's
//! constraint groups and connectivity graph are not ported.

use hk_math::{Quaternion, Transform, Vector4};
use hknp_types::{BodyId, ConstraintId, ConstraintIndex};

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
/// about the hinge is free (limits are not ported).
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

/// Any world-level joint constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WorldJoint {
    /// Point-to-point.
    BallAndSocket(BallAndSocketCinfo),
    /// Point-to-point plus hinge-axis alignment.
    Hinge(HingeCinfo),
    /// Point-to-point plus locked relative orientation (velocity level).
    Fixed(FixedCinfo),
}

impl WorldJoint {
    /// The two connected bodies.
    pub fn bodies(&self) -> (BodyId, BodyId) {
        match self {
            Self::BallAndSocket(info) => (info.body_a, info.body_b),
            Self::Hinge(info) => (info.body_a, info.body_b),
            Self::Fixed(info) => (info.body_a, info.body_b),
        }
    }
}

/// A stored joint plus its enabled flag.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointSlot {
    pub joint: WorldJoint,
    pub enabled: bool,
}

/// Build the solver-visible [`ConstraintId`] of a slot index.
pub(crate) fn constraint_id(slot: usize) -> ConstraintId {
    ConstraintId::from_index(ConstraintIndex::new(slot as u32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;

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
