//! Ragdoll constraint-easing subset.
//!
//! Rust port of `Extensions/Actions/EaseConstraints/hknpEaseConstraintsAction`
//! and the `hknpRagdollStateManager` easing-mode rules. Only ragdoll cone/twist
//! limits are supported: the port has no limited-hinge or 6DOF constraint
//! atoms, and the SDK's plane-angle limits are not ported either.
//!
//! Loosening widens each joint's limits just enough to contain the current
//! relative pose plus `SMALL_TOLERANCE` (0.01 rad). Restoration then eases the
//! live limits back toward the saved originals with the SDK's telescoping
//! weight `dt / (duration - elapsed)`.

use base_math::Quaternion;
use phys_types::ConstraintId;
use phys_world::{RagdollCinfo, World, WorldJoint};

/// SDK `smallTolerance` used when widening limits.
pub const SMALL_TOLERANCE: f32 = 0.01;
/// Default `hknpRagdollState::Cinfo::m_constraintEasingDurationSeconds`.
pub const DEFAULT_EASING_DURATION: f32 = 4.0;

/// `hknpRagdollState::ConstraintEasingMode` subset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintEasingMode {
    /// Keep the configured limits; no action is created.
    Enforce,
    /// Loosen once and leave the limits loose until the next state change.
    LoosenNoRestore,
    /// Loosen and linearly restore over the state's easing duration.
    LoosenThenRestore,
}

/// Saved cone/twist limits of one ragdoll constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollLimitSnapshot {
    /// Constraint whose limits were saved.
    pub id: ConstraintId,
    /// Original maximum cone angle.
    pub cone_angle: f32,
    /// Original lower twist bound.
    pub twist_min: f32,
    /// Original upper twist bound.
    pub twist_max: f32,
}

/// `hknpEaseConstraintsAction` subset for ragdoll joints.
#[derive(Clone, Debug, PartialEq)]
pub struct EaseConstraintsAction {
    saved: Vec<RagdollLimitSnapshot>,
    duration: f32,
    elapsed: f32,
}

impl EaseConstraintsAction {
    /// Save the current limits of every ragdoll constraint in `ids`.
    pub fn new(world: &World, ids: &[ConstraintId]) -> Self {
        let mut saved = Vec::with_capacity(ids.len());
        for &id in ids {
            if let Some(limits) = current_limits(world, id) {
                saved.push(RagdollLimitSnapshot {
                    id,
                    cone_angle: limits.0,
                    twist_min: limits.1,
                    twist_max: limits.2,
                });
            }
        }
        Self {
            saved,
            duration: -1.0,
            elapsed: 0.0,
        }
    }

    /// Saved limits, in the order of the ids passed to [`Self::new`].
    pub fn saved(&self) -> &[RagdollLimitSnapshot] {
        &self.saved
    }

    /// Restoration duration; negative means loosened with no restore.
    pub fn duration(&self) -> f32 {
        self.duration
    }

    /// Elapsed restoration time.
    pub fn elapsed(&self) -> f32 {
        self.elapsed
    }

    /// Widen each live limit to contain the current relative pose.
    pub fn loosen(&mut self, world: &mut World) {
        self.duration = -1.0;
        self.elapsed = 0.0;
        for snapshot in &self.saved {
            let Some(info) = ragdoll_info(world, snapshot.id) else {
                continue;
            };
            let (cone, twist) = current_angles(world, &info);
            let loose_cone = info.cone_angle.max(cone + SMALL_TOLERANCE);
            let loose_min = info.twist_min.min(twist - SMALL_TOLERANCE);
            let loose_max = info.twist_max.max(twist + SMALL_TOLERANCE);
            world.set_ragdoll_limits(snapshot.id, loose_cone, loose_min, loose_max);
        }
    }

    /// Restore all saved limits immediately (`restoreConstraints(world, 0)`).
    pub fn restore_now(&mut self, world: &mut World) {
        for snapshot in &self.saved {
            world.set_ragdoll_limits(
                snapshot.id,
                snapshot.cone_angle,
                snapshot.twist_min,
                snapshot.twist_max,
            );
        }
        self.duration = 0.0;
        self.elapsed = 0.0;
    }

    /// Start an eased restoration over `duration` seconds.
    pub fn restore(&mut self, world: &mut World, duration: f32) {
        if duration <= 0.0 {
            self.restore_now(world);
            return;
        }
        self.duration = duration;
        self.elapsed = 0.0;
    }

    /// Advance the restoration by `delta_time`; returns `true` when complete.
    pub fn step(&mut self, world: &mut World, delta_time: f32) -> bool {
        if self.duration <= 0.0 || delta_time <= 0.0 {
            return self.duration <= 0.0;
        }
        if self.elapsed + delta_time >= self.duration {
            self.restore_now(world);
            return true;
        }
        let weight = delta_time / (self.duration - self.elapsed);
        self.elapsed += delta_time;
        for snapshot in &self.saved {
            let Some((cone, twist_min, twist_max)) = current_limits(world, snapshot.id) else {
                continue;
            };
            let lerp = |current: f32, original: f32| current + weight * (original - current);
            world.set_ragdoll_limits(
                snapshot.id,
                lerp(cone, snapshot.cone_angle),
                lerp(twist_min, snapshot.twist_min),
                lerp(twist_max, snapshot.twist_max),
            );
        }
        false
    }
}

fn ragdoll_info(world: &World, id: ConstraintId) -> Option<RagdollCinfo> {
    match world.constraint(id)? {
        WorldJoint::Ragdoll(info) => Some(*info),
        _ => None,
    }
}

fn current_limits(world: &World, id: ConstraintId) -> Option<(f32, f32, f32)> {
    ragdoll_info(world, id).map(|info| (info.cone_angle, info.twist_min, info.twist_max))
}

fn rotation(world: &World, body: phys_types::BodyId) -> Quaternion {
    world
        .body(body)
        .map(|body| body.transform.rotation_quaternion())
        .unwrap_or(Quaternion::IDENTITY)
}

/// Current cone angle and twist angle of a ragdoll joint, as seen from its
/// two bodies (`hknpConstraintDataUtils::loosenConstraintLimits` subset).
fn current_angles(world: &World, info: &RagdollCinfo) -> (f32, f32) {
    let rotation_a = rotation(world, info.body_a);
    let rotation_b = rotation(world, info.body_b);
    let twist_a = rotation_a.rotate(info.twist_axis_a).normalized3();
    let twist_b = rotation_b.rotate(info.twist_axis_b).normalized3();
    let cone = twist_a.dot3(twist_b).clamp(-1.0, 1.0).acos();

    let plane_a = rotation_a.rotate(info.plane_axis_a).normalized3();
    let plane_b = rotation_b.rotate(info.plane_axis_b).normalized3();
    let axis_sum = twist_a + twist_b;
    let axis = if axis_sum.length3() > 1e-16 {
        axis_sum.normalized3()
    } else {
        twist_a
    };
    let c1 = axis.cross(plane_b);
    let c2 = c1.cross(axis);
    let twist = c2.dot3(plane_a).atan2(c1.dot3(plane_a));
    let center = (info.twist_min + info.twist_max) * 0.5;
    let mut twist = twist;
    if twist > center + core::f32::consts::PI {
        twist -= 2.0 * core::f32::consts::PI;
    } else if twist < center - core::f32::consts::PI {
        twist += 2.0 * core::f32::consts::PI;
    }
    (cone, twist)
}

/// Lerp helper matching the SDK's `hknpEaseConstraintsAction_LERP`.
pub fn lerp(current: f32, original: f32, weight: f32) -> f32 {
    current + weight * (original - current)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ragdoll::{Ragdoll, RagdollData};
    use base_math::{Quaternion, Transform, Vector4};
    use phys_body::BodyCinfo;
    use phys_shape::{Shape, SphereShape};
    use phys_types::{ConstraintId, MotionType};
    use std::sync::Arc;

    fn limits(world: &World, id: ConstraintId) -> (f32, f32, f32) {
        match world.constraint(id).unwrap() {
            WorldJoint::Ragdoll(info) => (info.cone_angle, info.twist_min, info.twist_max),
            _ => panic!("expected ragdoll joint"),
        }
    }

    fn ragdoll_world() -> (World, phys_types::BodyId, Vec<ConstraintId>) {
        let mut world = World::new(phys_world::WorldCinfo::new());
        let shape = || Arc::new(SphereShape::new(Vector4::ZERO, 0.25)) as Arc<dyn Shape>;
        let a = world.create_body(
            &BodyCinfo::new(shape())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let b = world.create_body(
            &BodyCinfo::new(shape())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(1.0, 0.0, 0.0)),
        );
        let mut data = RagdollData::new();
        data.add_bone("a", None, Some(a), Transform::IDENTITY);
        data.add_bone(
            "b",
            Some(0),
            Some(b),
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
        );
        let ragdoll = Ragdoll::create(&mut world, &data).unwrap();
        let ids: Vec<_> = ragdoll.constraints().iter().flatten().copied().collect();
        (world, b, ids)
    }

    #[test]
    fn lerp_matches_sdk() {
        assert_eq!(lerp(2.0, 4.0, 0.25), 2.5);
        assert_eq!(lerp(2.0, 4.0, 1.0), 4.0);
    }

    #[test]
    fn action_defaults_are_inert() {
        let mut world = World::new(phys_world::WorldCinfo::new());
        let mut action = EaseConstraintsAction::new(&world, &[]);
        assert!(action.saved().is_empty());
        assert!(action.duration() < 0.0);
        assert!(action.step(&mut world, 1.0 / 60.0));
    }

    #[test]
    fn loosen_widens_to_current_pose_and_restore_is_exact() {
        let (mut world, child, ids) = ragdoll_world();
        let original = limits(&world, ids[0]);
        let mut action = EaseConstraintsAction::new(&world, &ids);
        assert_eq!(action.saved().len(), 1);

        let rotated = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_3),
            Vector4::from_xyz(1.0, 0.0, 0.0),
        );
        world.set_body_transform(child, rotated);
        action.loosen(&mut world);
        let loose = limits(&world, ids[0]);
        assert!(loose.0 > original.0);
        assert!(
            loose.1 < original.1 || loose.2 > original.2,
            "at least one twist bound must widen"
        );

        action.restore_now(&mut world);
        assert_eq!(limits(&world, ids[0]), original);
    }

    #[test]
    fn eased_restore_converges_and_completes() {
        let (mut world, child, ids) = ragdoll_world();
        let original = limits(&world, ids[0]);
        let mut action = EaseConstraintsAction::new(&world, &ids);
        world.set_body_transform(
            child,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_3),
                Vector4::from_xyz(1.0, 0.0, 0.0),
            ),
        );
        action.loosen(&mut world);
        let loose = limits(&world, ids[0]);
        action.restore(&mut world, 1.0);
        assert!(!action.step(&mut world, 0.25));
        let halfway = limits(&world, ids[0]);
        assert!(halfway.0 < loose.0 && halfway.0 > original.0);
        assert!(!action.step(&mut world, 0.25));
        assert!(!action.step(&mut world, 0.25));
        assert!(action.step(&mut world, 0.25));
        assert_eq!(limits(&world, ids[0]), original);
    }
}
