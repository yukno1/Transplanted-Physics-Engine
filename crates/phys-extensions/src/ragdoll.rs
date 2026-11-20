//! Ragdoll scheduling layer subset.
//!
//! Rust port of `Extensions/Ragdoll/hknpRagdoll`, `hknpRagdollController`
//! (`BodyController` / `calcDesiredPoseWorldSpace`) and
//! `Extensions/KeyFrameUtil/hknpKeyFrameUtil::calcRequiredVelocity`.
//!
//! The SDK derives joint cinfos from an `hknpPhysicsSystemData`; this subset
//! takes already-created bodies plus a bone hierarchy with reference poses and
//! builds the `RagdollCinfo` chain on [`phys_world::World`].  The animation
//! skeleton, action manager, ease-constraints action and penetration utility
//! are not ported.  The motor blender and ragdoll state machine live in
//! [`crate::ragdoll_controller`]; callers may also drive
//! [`Ragdoll::drive_to_pose`] or [`Ragdoll::apply_hard_keyframe`] explicitly,
//! before/after `World::step`, as the vehicle extension does.

use base_math::{Quaternion, Transform, Vector4};
use phys_types::{BodyId, ConstraintId};
use phys_world::{RagdollCinfo, World};

/// Cone/twist limits applied to a bone's joint to its parent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollLimits {
    /// Maximum swing angle between the two twist axes, in radians.
    pub cone_angle: f32,
    /// Lower twist bound in radians.
    pub twist_min: f32,
    /// Upper twist bound in radians.
    pub twist_max: f32,
}

impl Default for RagdollLimits {
    fn default() -> Self {
        let quarter = core::f32::consts::FRAC_PI_4;
        Self {
            cone_angle: quarter,
            twist_min: -quarter,
            twist_max: quarter,
        }
    }
}

/// One bone of a ragdoll and its associated body.
#[derive(Clone, Debug, PartialEq)]
pub struct RagdollBone {
    /// Bone name (used for lookup, mirroring `matchBoneToBody`).
    pub name: String,
    /// Parent bone index, or `None` for the root.
    pub parent: Option<usize>,
    /// Associated body, or `None` for a terminal (body-less) bone.
    pub body: Option<BodyId>,
    /// Reference transform from the parent (or model) space to this bone.
    pub reference: Transform,
    /// Twist axis in bone-local space (default `+Y`, the Havok ragdoll axis).
    pub twist_axis: Vector4,
    /// Perpendicular reference axis in bone-local space (default `+X`).
    pub plane_axis: Vector4,
    /// Joint limits used when connecting this bone to its parent.
    pub limits: RagdollLimits,
}

/// Bone hierarchy with body bindings, before world-side constraints exist.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RagdollData {
    /// Bones in parent-before-child order.
    pub bones: Vec<RagdollBone>,
}

impl RagdollData {
    /// Empty ragdoll description.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a bone with default axes/limits and return its index.
    pub fn add_bone(
        &mut self,
        name: impl Into<String>,
        parent: Option<usize>,
        body: Option<BodyId>,
        reference: Transform,
    ) -> usize {
        let index = self.bones.len();
        self.bones.push(RagdollBone {
            name: name.into(),
            parent,
            body,
            reference,
            twist_axis: Vector4::UNIT_Y,
            plane_axis: Vector4::UNIT_X,
            limits: RagdollLimits::default(),
        });
        index
    }

    /// Find a bone index by name.
    pub fn bone_index(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|bone| bone.name == name)
    }
}

/// Errors produced by ragdoll construction and driving.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RagdollError {
    /// A bone index did not exist or was not earlier than its child.
    InvalidBone(usize),
    /// A bone that must be driven has no body.
    MissingBody(usize),
    /// A pose slice had the wrong number of entries.
    PoseLength {
        /// Expected number of entries.
        expected: usize,
        /// Actual number of entries.
        actual: usize,
    },
    /// `delta_time` was zero, negative or non-finite.
    InvalidDeltaTime,
    /// Drive strength was outside `[0, 1]` or non-finite.
    InvalidStrength,
    /// A timed state transition had a non-positive or non-finite duration.
    InvalidDuration,
}

impl core::fmt::Display for RagdollError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidBone(index) => write!(f, "invalid ragdoll bone index {index}"),
            Self::MissingBody(index) => write!(f, "ragdoll bone {index} has no body"),
            Self::PoseLength { expected, actual } => {
                write!(f, "expected {expected} poses, got {actual}")
            }
            Self::InvalidDeltaTime => write!(f, "ragdoll delta time must be positive and finite"),
            Self::InvalidStrength => write!(f, "ragdoll strength must be within [0, 1]"),
            Self::InvalidDuration => {
                write!(f, "ragdoll transition duration must be positive and finite")
            }
        }
    }
}

impl std::error::Error for RagdollError {}

/// A world-side ragdoll: bone bindings plus the constraints joining them.
#[derive(Clone, Debug, PartialEq)]
pub struct Ragdoll {
    bones: Vec<RagdollBone>,
    constraints: Vec<Option<ConstraintId>>,
}

impl Ragdoll {
    /// Create ragdoll constraints from the description.
    ///
    /// Each bone with a parent and both bodies gets one `RagdollCinfo` whose
    /// pivot is the bone's reference origin transformed into the parent body's
    /// frame at creation time.  Adjacent body collisions are disabled, matching
    /// `hknpRagdollUtils` / `disableCollisionsBetweenConstraintBodies`.
    pub fn create(world: &mut World, data: &RagdollData) -> Result<Self, RagdollError> {
        for (index, bone) in data.bones.iter().enumerate() {
            if let Some(parent) = bone.parent {
                if parent >= index {
                    return Err(RagdollError::InvalidBone(parent));
                }
            }
        }

        let mut constraints = vec![None; data.bones.len()];
        for (index, bone) in data.bones.iter().enumerate() {
            let Some(parent_index) = bone.parent else {
                continue;
            };
            let parent = &data.bones[parent_index];
            let (Some(child_body), Some(parent_body)) = (bone.body, parent.body) else {
                continue;
            };
            let child_world = world
                .body_transform(child_body)
                .ok_or(RagdollError::MissingBody(index))?;
            let parent_world = world
                .body_transform(parent_body)
                .ok_or(RagdollError::MissingBody(parent_index))?;

            let pivot_world = parent_world.transform_point(bone.reference.translation);
            let pivot_a = child_world.inverse_transform_point(pivot_world);
            let pivot_b = parent_world.inverse_transform_point(pivot_world);

            let child_rotation = child_world.rotation_quaternion();
            let parent_rotation = parent_world.rotation_quaternion();
            let relative = parent_rotation.inverse_mul(child_rotation);
            let twist_a = bone.twist_axis.normalized3();
            let plane_a = bone.plane_axis.normalized3();
            let twist_b = relative.rotate(twist_a).normalized3();
            let plane_b = relative.rotate(plane_a).normalized3();

            let info = RagdollCinfo::new(
                child_body,
                parent_body,
                pivot_a,
                pivot_b,
                twist_a,
                twist_b,
                plane_a,
                plane_b,
            )
            .with_limits(
                bone.limits.cone_angle,
                bone.limits.twist_min,
                bone.limits.twist_max,
            );
            let id = world.create_ragdoll_constraint(&info);
            world.set_constraint_collisions_enabled(id, false);
            constraints[index] = Some(id);
        }

        Ok(Self {
            bones: data.bones.clone(),
            constraints,
        })
    }

    /// Bone descriptions, in the same order as the source [`RagdollData`].
    pub fn bones(&self) -> &[RagdollBone] {
        &self.bones
    }

    /// Constraint joining each bone to its parent (`None` for roots/terminal bones).
    pub fn constraints(&self) -> &[Option<ConstraintId>] {
        &self.constraints
    }

    /// Enable or disable every constraint in the ragdoll.
    pub fn set_enabled(&mut self, world: &mut World, enabled: bool) {
        for id in self.constraints.iter().flatten() {
            world.set_constraint_enabled(*id, enabled);
        }
    }

    /// Current world pose of every bone.
    ///
    /// Bones without a body are composed from their parent's world pose and
    /// the reference transform; `model_to_world` is used for the root.
    pub fn pose_world_space(&self, world: &World, model_to_world: Transform) -> Vec<Transform> {
        let mut poses = Vec::with_capacity(self.bones.len());
        for (index, bone) in self.bones.iter().enumerate() {
            let pose = if let Some(body) = bone.body {
                world.body_transform(body).unwrap_or(bone.reference)
            } else {
                let parent = bone
                    .parent
                    .and_then(|index| poses.get(index).copied())
                    .unwrap_or(model_to_world);
                parent * bone.reference
            };
            debug_assert!(index == poses.len());
            poses.push(pose);
        }
        poses
    }

    /// `hknpRagdollController::calcDesiredPoseWorldSpace`.
    ///
    /// Composes each desired parent-relative pose with the already-composed
    /// parent pose, starting from `model_to_world` at the root.
    pub fn calc_desired_pose_world_space(
        &self,
        desired_local: &[Transform],
        model_to_world: Transform,
    ) -> Result<Vec<Transform>, RagdollError> {
        if desired_local.len() != self.bones.len() {
            return Err(RagdollError::PoseLength {
                expected: self.bones.len(),
                actual: desired_local.len(),
            });
        }
        let mut poses = Vec::with_capacity(desired_local.len());
        for (index, bone) in self.bones.iter().enumerate() {
            let parent = bone
                .parent
                .and_then(|index| poses.get(index).copied())
                .unwrap_or(model_to_world);
            poses.push(parent * desired_local[index]);
        }
        Ok(poses)
    }

    /// `hknpRagdollBodyController::driveToPose`.
    ///
    /// Computes the linear/angular velocity needed to reach the desired pose in
    /// one step, then interpolates from each body's current velocity by
    /// `strength * bone_strengths[i]` (missing entries default to 1).
    pub fn drive_to_pose(
        &self,
        world: &mut World,
        desired_local: &[Transform],
        model_to_world: Transform,
        delta_time: f32,
        strength: f32,
        bone_strengths: &[f32],
    ) -> Result<(), RagdollError> {
        let inv_dt = inverse_delta_time(delta_time)?;
        if !strength.is_finite() || !(0.0..=1.0).contains(&strength) {
            return Err(RagdollError::InvalidStrength);
        }
        let desired = self.calc_desired_pose_world_space(desired_local, model_to_world)?;

        for (index, bone) in self.bones.iter().enumerate() {
            let Some(body) = bone.body else {
                continue;
            };
            let current = world
                .body_transform(body)
                .ok_or(RagdollError::MissingBody(index))?;
            let bone_strength = bone_strengths.get(index).copied().unwrap_or(1.0);
            let weight = strength * bone_strength;

            let mut linear = (desired[index].translation - current.translation) * inv_dt;
            linear.w = 0.0;
            if let Some(current_velocity) = world.body_linear_velocity(body) {
                linear.set_interpolate(current_velocity, linear, weight);
            }
            let mut angular = required_angular_velocity(
                current.rotation_quaternion(),
                desired[index].rotation_quaternion(),
                inv_dt,
            );
            angular.w = 0.0;
            if let Some(current_velocity) = world.body_angular_velocity(body) {
                angular.set_interpolate(current_velocity, angular, weight);
            }

            world.set_body_linear_velocity(body, linear);
            world.set_body_angular_velocity(body, angular);
        }
        Ok(())
    }

    /// `hknpRagdoll::applyHardKeyframe` with `hknpKeyFrameUtil` velocities.
    ///
    /// Sets each body's transform to the desired pose exactly and writes the
    /// velocity required to arrive there from its current pose in `delta_time`.
    pub fn apply_hard_keyframe(
        &self,
        world: &mut World,
        pose_model_space: &[Transform],
        world_from_model: Transform,
        delta_time: f32,
    ) -> Result<(), RagdollError> {
        let inv_dt = inverse_delta_time(delta_time)?;
        let desired = self.calc_desired_pose_world_space(pose_model_space, world_from_model)?;
        for (index, bone) in self.bones.iter().enumerate() {
            if bone.body.is_some() {
                self.hard_keyframe_bone(world, index, desired[index], inv_dt)?;
            }
        }
        Ok(())
    }

    /// Hard-keyframe a single bone to an already world-space transform.
    ///
    /// Used by the powered state's `keyframeRoot` path, where only the root
    /// body is locked while the remaining bones are driven by motors.
    pub fn apply_hard_keyframe_bone(
        &self,
        world: &mut World,
        bone_index: usize,
        pose_world: Transform,
        delta_time: f32,
    ) -> Result<(), RagdollError> {
        let inv_dt = inverse_delta_time(delta_time)?;
        self.hard_keyframe_bone(world, bone_index, pose_world, inv_dt)
    }

    fn hard_keyframe_bone(
        &self,
        world: &mut World,
        bone_index: usize,
        pose_world: Transform,
        inv_dt: f32,
    ) -> Result<(), RagdollError> {
        let bone = self
            .bones
            .get(bone_index)
            .ok_or(RagdollError::InvalidBone(bone_index))?;
        let body = bone.body.ok_or(RagdollError::MissingBody(bone_index))?;
        let current = world
            .body_transform(body)
            .ok_or(RagdollError::MissingBody(bone_index))?;
        let mut linear = (pose_world.translation - current.translation) * inv_dt;
        linear.w = 0.0;
        let mut angular = required_angular_velocity(
            current.rotation_quaternion(),
            pose_world.rotation_quaternion(),
            inv_dt,
        );
        angular.w = 0.0;
        world.set_body_transform(body, pose_world);
        world.set_body_linear_velocity(body, linear);
        world.set_body_angular_velocity(body, angular);
        Ok(())
    }
}

fn inverse_delta_time(delta_time: f32) -> Result<f32, RagdollError> {
    if delta_time <= 0.0 || !delta_time.is_finite() {
        return Err(RagdollError::InvalidDeltaTime);
    }
    Ok(1.0 / delta_time)
}

/// `hknpKeyFrameUtil::calcRequiredVelocity` angular branch.
fn required_angular_velocity(current: Quaternion, target: Quaternion, inv_dt: f32) -> Vector4 {
    let difference = target.mul_inverse(current);
    if difference.w.abs() < 0.939_692_6 {
        let normalized = difference.normalized();
        let imaginary = normalized.imag();
        if imaginary.length3() > 1e-8 {
            return imaginary.normalized3() * (normalized.angle() * inv_dt);
        }
    }
    let sign = if difference.w < 0.0 { -1.0 } else { 1.0 };
    difference.imag() * (2.0 * inv_dt * sign)
}

#[cfg(test)]
mod tests {
    use super::*;
    use base_math::Quaternion;
    use core::f32::consts::FRAC_PI_2;

    fn translate(x: f32, y: f32) -> Transform {
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(x, y, 0.0))
    }

    #[test]
    fn composes_desired_pose_hierarchy() {
        let mut data = RagdollData::new();
        data.add_bone("root", None, None, Transform::IDENTITY);
        data.add_bone("mid", Some(0), None, translate(1.0, 0.0));
        data.add_bone("tip", Some(1), None, translate(0.0, 1.0));
        let ragdoll = Ragdoll::create(&mut World::new(phys_world::WorldCinfo::new()), &data)
            .expect("ragdoll");

        let local = [
            translate(2.0, 3.0),
            translate(1.0, 0.0),
            translate(0.0, 2.0),
        ];
        let world = ragdoll
            .calc_desired_pose_world_space(&local, Transform::IDENTITY)
            .expect("poses");
        assert_eq!(world[0].translation, Vector4::from_xyz(2.0, 3.0, 0.0));
        assert_eq!(world[1].translation, Vector4::from_xyz(3.0, 3.0, 0.0));
        assert_eq!(world[2].translation, Vector4::from_xyz(3.0, 5.0, 0.0));

        let moved = ragdoll
            .calc_desired_pose_world_space(&local, translate(10.0, 0.0))
            .expect("poses");
        assert_eq!(moved[2].translation, Vector4::from_xyz(13.0, 5.0, 0.0));
    }

    #[test]
    fn rejects_invalid_poses_and_times() {
        let data = RagdollData::new();
        let ragdoll = Ragdoll::create(&mut World::new(phys_world::WorldCinfo::new()), &data)
            .expect("empty ragdoll");
        assert_eq!(
            ragdoll.calc_desired_pose_world_space(&[], Transform::IDENTITY),
            Ok(Vec::new())
        );
        assert_eq!(
            ragdoll.calc_desired_pose_world_space(&[Transform::IDENTITY], Transform::IDENTITY),
            Err(RagdollError::PoseLength {
                expected: 0,
                actual: 1
            })
        );
        assert_eq!(
            ragdoll.drive_to_pose(
                &mut World::new(phys_world::WorldCinfo::new()),
                &[],
                Transform::IDENTITY,
                0.0,
                1.0,
                &[],
            ),
            Err(RagdollError::InvalidDeltaTime)
        );
    }

    #[test]
    fn hard_keyframe_uses_required_velocity() {
        let rotation = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2),
            Vector4::from_xyz(1.0, 0.0, 0.0),
        );
        let velocity =
            required_angular_velocity(Quaternion::IDENTITY, rotation.rotation_quaternion(), 60.0);
        assert!(velocity.x.abs() < 1e-5 && velocity.y.abs() < 1e-5);
        assert!((velocity.z - FRAC_PI_2 * 60.0).abs() < 0.5, "{velocity:?}");
    }
}
