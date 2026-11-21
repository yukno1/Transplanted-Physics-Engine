//! Constraint-easing subset (`hknpEaseConstraintsAction`).
//!
//! Supports the three families the SDK action handles: ragdoll cone/twist
//! (plus optional plane limits), limited-hinge angular bounds and 6DOF
//! twist/swing/linear bounds (plus optional per-axis translation limits).
//!
//! Loosening widens each joint's limits just enough to contain the current
//! relative pose plus `SMALL_TOLERANCE` (0.01 rad). Restoration then eases the
//! live limits back toward the saved originals with the SDK's telescoping
//! weight `dt / (duration - elapsed)`.
//!
//! SDK note: `hknpConstraintDataUtils::loosenConstraintLimits` only widens
//! ragdoll and limited-hinge limits in this source version (6DOF has
//! save/restore plumbing but no loosen helper). The 6DOF branch here therefore
//! uses an explicit conservative widening of the saved scalar limits, which
//! is documented as a Rust-side subset rather than a direct translation.

use base_math::{Quaternion, Vector4};
use phys_types::{BodyId, ConstraintId};
use phys_world::{
    HingeCinfo, HingeLimitsCinfo, RagdollCinfo, SixDofAxisMode, SixDofCinfo, SixDofLimitsCinfo,
    World, WorldJoint,
};

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
    /// Optional plane-angle bounds (SDK deviation from 90°).
    pub plane_limits: Option<(f32, f32)>,
}

/// Saved angular bounds of one limited-hinge constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LimitedHingeLimitSnapshot {
    /// Constraint whose limits were saved.
    pub id: ConstraintId,
    /// Original lower angle bound.
    pub min_angle: f32,
    /// Original upper angle bound.
    pub max_angle: f32,
}

/// Saved twist/swing/linear limits of one 6DOF constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SixDofLimitSnapshot {
    /// Constraint whose limits were saved.
    pub id: ConstraintId,
    /// Original lower twist bound.
    pub twist_min: f32,
    /// Original upper twist bound.
    pub twist_max: f32,
    /// Original first swing bound.
    pub swing0_max: f32,
    /// Original second swing bound.
    pub swing1_max: f32,
    /// Original maximum pivot distance.
    pub linear_limit: f32,
    /// Original per-axis translation modes.
    pub linear_axes: [SixDofAxisMode; 3],
    /// Original per-axis lower bounds.
    pub linear_min: [f32; 3],
    /// Original per-axis upper bounds.
    pub linear_max: [f32; 3],
}

/// One saved limit set, dispatched by joint family.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LimitSnapshot {
    /// Ragdoll cone/twist limits.
    Ragdoll(RagdollLimitSnapshot),
    /// Limited-hinge angular bounds.
    LimitedHinge(LimitedHingeLimitSnapshot),
    /// 6DOF twist/swing/linear limits.
    SixDof(SixDofLimitSnapshot),
}

impl LimitSnapshot {
    /// Constraint this snapshot belongs to.
    pub fn id(&self) -> ConstraintId {
        match self {
            Self::Ragdoll(snapshot) => snapshot.id,
            Self::LimitedHinge(snapshot) => snapshot.id,
            Self::SixDof(snapshot) => snapshot.id,
        }
    }
}

/// `hknpEaseConstraintsAction` subset.
#[derive(Clone, Debug, PartialEq)]
pub struct EaseConstraintsAction {
    saved: Vec<LimitSnapshot>,
    duration: f32,
    elapsed: f32,
}

impl EaseConstraintsAction {
    /// Save the current limits of every supported constraint in `ids`, in
    /// input order. Unsupported families are skipped.
    pub fn new(world: &World, ids: &[ConstraintId]) -> Self {
        let mut saved = Vec::with_capacity(ids.len());
        for &id in ids {
            let snapshot = match world.constraint(id) {
                Some(WorldJoint::Ragdoll(info)) => {
                    Some(LimitSnapshot::Ragdoll(RagdollLimitSnapshot {
                        id,
                        cone_angle: info.cone_angle,
                        twist_min: info.twist_min,
                        twist_max: info.twist_max,
                        plane_limits: info.plane_limits,
                    }))
                }
                Some(WorldJoint::LimitedHinge(info)) => info.limits.map(|limits| {
                    LimitSnapshot::LimitedHinge(LimitedHingeLimitSnapshot {
                        id,
                        min_angle: limits.min_angle,
                        max_angle: limits.max_angle,
                    })
                }),
                Some(WorldJoint::SixDof(info)) => {
                    Some(LimitSnapshot::SixDof(SixDofLimitSnapshot {
                        id,
                        twist_min: info.limits.twist_min,
                        twist_max: info.limits.twist_max,
                        swing0_max: info.limits.swing0_max,
                        swing1_max: info.limits.swing1_max,
                        linear_limit: info.limits.linear_limit,
                        linear_axes: info.limits.linear_axes,
                        linear_min: info.limits.linear_min,
                        linear_max: info.limits.linear_max,
                    }))
                }
                _ => None,
            };
            if let Some(snapshot) = snapshot {
                saved.push(snapshot);
            }
        }
        Self {
            saved,
            duration: -1.0,
            elapsed: 0.0,
        }
    }

    /// Saved limits, in the order of the ids passed to [`Self::new`].
    pub fn saved(&self) -> &[LimitSnapshot] {
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
            match *snapshot {
                LimitSnapshot::Ragdoll(saved) => {
                    let Some(info) = ragdoll_info(world, saved.id) else {
                        continue;
                    };
                    let (cone, twist) = current_angles(world, &info);
                    let loose_cone = info.cone_angle.max(cone + SMALL_TOLERANCE);
                    let loose_min = info.twist_min.min(twist - SMALL_TOLERANCE);
                    let loose_max = info.twist_max.max(twist + SMALL_TOLERANCE);
                    world.set_ragdoll_limits(saved.id, loose_cone, loose_min, loose_max);
                    if let Some((plane_min, plane_max)) = saved.plane_limits {
                        let angle = ragdoll_plane_angle(world, &info);
                        world.set_ragdoll_plane_limits(
                            saved.id,
                            Some((
                                plane_min.min(angle - SMALL_TOLERANCE),
                                plane_max.max(angle + SMALL_TOLERANCE),
                            )),
                        );
                    }
                }
                LimitSnapshot::LimitedHinge(saved) => {
                    let Some(info) = hinge_info(world, saved.id) else {
                        continue;
                    };
                    let angle = hinge_angle(world, &info);
                    let loose_min = saved.min_angle.min(angle - SMALL_TOLERANCE);
                    let loose_max = saved.max_angle.max(angle + SMALL_TOLERANCE);
                    world.set_limited_hinge_limits(
                        saved.id,
                        Some(HingeLimitsCinfo::new(loose_min, loose_max)),
                    );
                }
                LimitSnapshot::SixDof(saved) => {
                    let Some(info) = six_dof_info(world, saved.id) else {
                        continue;
                    };
                    let (twist, swing0, swing1, distance) = six_dof_state(world, &info);
                    let translations = six_dof_axis_state(world, &info);
                    let mut limits = SixDofLimitsCinfo::new(
                        saved.twist_min.min(twist - SMALL_TOLERANCE),
                        saved.twist_max.max(twist + SMALL_TOLERANCE),
                        saved.swing0_max.max(swing0.abs() + SMALL_TOLERANCE),
                        saved.swing1_max.max(swing1.abs() + SMALL_TOLERANCE),
                        saved.linear_limit.max(distance + SMALL_TOLERANCE),
                    );
                    limits.linear_axes = saved.linear_axes;
                    limits.linear_min = saved.linear_min;
                    limits.linear_max = saved.linear_max;
                    for axis in 0..3 {
                        match saved.linear_axes[axis] {
                            SixDofAxisMode::Unlimited => {}
                            SixDofAxisMode::Limited => {
                                limits.linear_min[axis] = saved.linear_min[axis]
                                    .min(translations[axis] - SMALL_TOLERANCE);
                                limits.linear_max[axis] = saved.linear_max[axis]
                                    .max(translations[axis] + SMALL_TOLERANCE);
                            }
                            SixDofAxisMode::Fixed => {
                                limits.linear_axes[axis] = SixDofAxisMode::Limited;
                                limits.linear_min[axis] = translations[axis] - SMALL_TOLERANCE;
                                limits.linear_max[axis] = translations[axis] + SMALL_TOLERANCE;
                            }
                        }
                    }
                    world.set_6dof_limits(saved.id, limits);
                }
            }
        }
    }

    /// Restore all saved limits immediately (`restoreConstraints(world, 0)`).
    pub fn restore_now(&mut self, world: &mut World) {
        for snapshot in &self.saved {
            match *snapshot {
                LimitSnapshot::Ragdoll(saved) => {
                    world.set_ragdoll_limits(
                        saved.id,
                        saved.cone_angle,
                        saved.twist_min,
                        saved.twist_max,
                    );
                    if saved.plane_limits.is_some() {
                        world.set_ragdoll_plane_limits(saved.id, saved.plane_limits);
                    }
                }
                LimitSnapshot::LimitedHinge(saved) => {
                    world.set_limited_hinge_limits(
                        saved.id,
                        Some(HingeLimitsCinfo::new(saved.min_angle, saved.max_angle)),
                    );
                }
                LimitSnapshot::SixDof(saved) => {
                    let mut limits = SixDofLimitsCinfo::new(
                        saved.twist_min,
                        saved.twist_max,
                        saved.swing0_max,
                        saved.swing1_max,
                        saved.linear_limit,
                    );
                    limits.linear_axes = saved.linear_axes;
                    limits.linear_min = saved.linear_min;
                    limits.linear_max = saved.linear_max;
                    world.set_6dof_limits(saved.id, limits);
                }
            }
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
        let lerp = |current: f32, original: f32| current + weight * (original - current);
        for snapshot in &self.saved {
            match (snapshot, world.constraint(snapshot.id()).copied()) {
                (LimitSnapshot::Ragdoll(saved), Some(WorldJoint::Ragdoll(info))) => {
                    world.set_ragdoll_limits(
                        saved.id,
                        lerp(info.cone_angle, saved.cone_angle),
                        lerp(info.twist_min, saved.twist_min),
                        lerp(info.twist_max, saved.twist_max),
                    );
                    if let (Some((current_min, current_max)), Some((saved_min, saved_max))) =
                        (info.plane_limits, saved.plane_limits)
                    {
                        world.set_ragdoll_plane_limits(
                            saved.id,
                            Some((lerp(current_min, saved_min), lerp(current_max, saved_max))),
                        );
                    }
                }
                (LimitSnapshot::LimitedHinge(saved), Some(WorldJoint::LimitedHinge(info))) => {
                    let Some(limits) = info.limits else {
                        continue;
                    };
                    world.set_limited_hinge_limits(
                        saved.id,
                        Some(HingeLimitsCinfo::new(
                            lerp(limits.min_angle, saved.min_angle),
                            lerp(limits.max_angle, saved.max_angle),
                        )),
                    );
                }
                (LimitSnapshot::SixDof(saved), Some(WorldJoint::SixDof(info))) => {
                    let limits = info.limits;
                    let mut restored = SixDofLimitsCinfo::new(
                        lerp(limits.twist_min, saved.twist_min),
                        lerp(limits.twist_max, saved.twist_max),
                        lerp(limits.swing0_max, saved.swing0_max),
                        lerp(limits.swing1_max, saved.swing1_max),
                        lerp(limits.linear_limit, saved.linear_limit),
                    );
                    restored.linear_axes = saved.linear_axes;
                    for axis in 0..3 {
                        restored.linear_min[axis] =
                            lerp(limits.linear_min[axis], saved.linear_min[axis]);
                        restored.linear_max[axis] =
                            lerp(limits.linear_max[axis], saved.linear_max[axis]);
                    }
                    world.set_6dof_limits(saved.id, restored);
                }
                _ => {}
            }
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

fn hinge_info(world: &World, id: ConstraintId) -> Option<HingeCinfo> {
    match world.constraint(id)? {
        WorldJoint::LimitedHinge(info) => Some(*info),
        _ => None,
    }
}

fn six_dof_info(world: &World, id: ConstraintId) -> Option<SixDofCinfo> {
    match world.constraint(id)? {
        WorldJoint::SixDof(info) => Some(*info),
        _ => None,
    }
}

fn unit(direction: Vector4) -> Vector4 {
    let length = direction.length3();
    if length > 1e-10 {
        direction * (1.0 / length)
    } else {
        Vector4::UNIT_X
    }
}

/// Deterministic perpendicular of `axis` used by the 6DOF swing rows.
fn deterministic_perpendicular(axis: Vector4) -> Vector4 {
    let helper = if axis.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    };
    unit(axis.cross(helper))
}

/// Project `reference` onto the plane perpendicular to `axis`, with the
/// same deterministic fallback as the world's hinge construction.
fn perpendicular_reference(axis: Vector4, reference: Vector4) -> Vector4 {
    let component = reference - axis * reference.dot3(axis);
    if component.length3() > 1e-10 {
        return unit(component);
    }
    unit(axis.cross(if axis.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    }))
}

/// Signed hinge angle of B relative to A (`hknpHinge` reference-axis
/// convention), used to decide which angular bound to widen.
fn hinge_angle(world: &World, info: &HingeCinfo) -> f32 {
    let (Some(body_a), Some(body_b)) = (world.body(info.body_a), world.body(info.body_b)) else {
        return 0.0;
    };
    let transform_a = body_a.transform;
    let transform_b = body_b.transform;
    let axis_a = unit(transform_a.rotation.rotate(info.axis_a));
    let axis_b = unit(transform_b.rotation.rotate(info.axis_b));
    let reference_a =
        perpendicular_reference(axis_a, transform_a.rotation.rotate(info.reference_axis_a));
    let reference_b =
        perpendicular_reference(axis_b, transform_b.rotation.rotate(info.reference_axis_b));
    let tangential = |reference: Vector4, axis: Vector4| {
        let component = reference - axis * reference.dot3(axis);
        if component.length3() > 1e-10 {
            unit(component)
        } else {
            Vector4::ZERO
        }
    };
    let a = tangential(reference_a, axis_a);
    let b = tangential(reference_b, axis_b);
    a.cross(b).dot3(axis_a).atan2(a.dot3(b))
}

/// Current twist, swing0, swing1 and pivot distance of a 6DOF joint. The
/// row axes match `phys_world::build_solver_joints` and the solver's swing
/// rows, so loosening observes the same geometry that is being constrained.
fn six_dof_state(world: &World, info: &SixDofCinfo) -> (f32, f32, f32, f32) {
    let (Some(body_a), Some(body_b)) = (world.body(info.body_a), world.body(info.body_b)) else {
        return (0.0, 0.0, 0.0, 0.0);
    };
    let transform_a = body_a.transform;
    let transform_b = body_b.transform;
    let twist_a = unit(transform_a.rotation.rotate(info.twist_axis_a));
    let twist_b = unit(transform_b.rotation.rotate(info.twist_axis_b));
    let u = deterministic_perpendicular(twist_a);
    let v = twist_a.cross(u);
    let error = twist_a.cross(twist_b);
    let plane_a = perpendicular_reference(twist_a, transform_a.rotation.rotate(info.plane_axis_a));
    let plane_b = perpendicular_reference(twist_a, transform_b.rotation.rotate(info.plane_axis_b));
    let twist = twist_a
        .dot3(plane_a.cross(plane_b))
        .atan2(plane_a.dot3(plane_b));
    let pivot_a = transform_a.transform_point(info.pivot_a);
    let pivot_b = transform_b.transform_point(info.pivot_b);
    (
        twist,
        error.dot3(u),
        error.dot3(v),
        (pivot_a - pivot_b).length3(),
    )
}

/// Current per-axis translations of a 6DOF joint in the solver's
/// `[twist, u, v]` basis.
fn six_dof_axis_state(world: &World, info: &SixDofCinfo) -> [f32; 3] {
    let (Some(body_a), Some(body_b)) = (world.body(info.body_a), world.body(info.body_b)) else {
        return [0.0; 3];
    };
    let transform_a = body_a.transform;
    let transform_b = body_b.transform;
    let twist = unit(transform_a.rotation.rotate(info.twist_axis_a));
    let u = deterministic_perpendicular(twist);
    let v = twist.cross(u);
    let delta =
        transform_a.transform_point(info.pivot_a) - transform_b.transform_point(info.pivot_b);
    [delta.dot3(twist), delta.dot3(u), delta.dot3(v)]
}

/// Current SDK-style plane angle (deviation of the A-twist/B-plane angle
/// from 90°), matching `RagdollCinfo::plane_limits`.
fn ragdoll_plane_angle(world: &World, info: &RagdollCinfo) -> f32 {
    let (Some(body_a), Some(body_b)) = (world.body(info.body_a), world.body(info.body_b)) else {
        return 0.0;
    };
    let twist_a = unit(body_a.transform.rotation.rotate(info.twist_axis_a));
    let plane_b = unit(body_b.transform.rotation.rotate(info.plane_axis_b));
    core::f32::consts::FRAC_PI_2 - twist_a.dot3(plane_b).clamp(-1.0, 1.0).acos()
}

fn rotation(world: &World, body: BodyId) -> Quaternion {
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
