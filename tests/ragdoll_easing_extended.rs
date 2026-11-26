//! Easing of the newly supported limited-hinge and 6DOF limit sets.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{EaseConstraintsAction, LimitSnapshot, Ragdoll, RagdollData};
use proton::math::{Quaternion, Transform, Vector4, Vec4};
use proton::np_types::{BodyId, ConstraintId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{
    HingeCinfo, HingeLimitsCinfo, SixDofAxisMode, SixDofCinfo, SixDofLimitsCinfo, World, WorldCinfo,
};

fn body(world: &mut World, position: Vector4) -> BodyId {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.2)) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(position),
    )
}

fn hinge(world: &mut World) -> (BodyId, BodyId, ConstraintId) {
    let a = body(world, Vector4::ZERO);
    let b = body(world, Vector4::from_xyz(1.0, 0.0, 0.0));
    let info = HingeCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::from_xyz(-1.0, 0.0, 0.0),
        Vector4::UNIT_Y,
        Vector4::UNIT_Y,
    )
    .with_reference_axes(Vector4::UNIT_X, Vector4::UNIT_X)
    .with_limits(HingeLimitsCinfo::new(-0.1, 0.1));
    (a, b, world.create_limited_hinge_constraint(&info))
}

fn six_dof(world: &mut World) -> (BodyId, ConstraintId) {
    let a = body(world, Vector4::ZERO);
    let b = body(world, Vector4::from_xyz(0.0, 1.0, 0.0));
    let info = SixDofCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::from_xyz(0.0, -1.0, 0.0),
        Vector4::UNIT_Y,
        Vector4::UNIT_Y,
        Vector4::UNIT_X,
        Vector4::UNIT_X,
        SixDofLimitsCinfo::new(-0.2, 0.2, 0.2, 0.2, 0.1),
    );
    (b, world.create_6dof_constraint(&info))
}

fn hinge_limits(world: &World, id: ConstraintId) -> (f32, f32) {
    let proton::world::WorldJoint::LimitedHinge(info) = world.constraint(id).unwrap() else {
        panic!("expected limited hinge");
    };
    let limits = info.limits.expect("hinge limits");
    (limits.min_angle, limits.max_angle)
}

fn six_dof_limits(world: &World, id: ConstraintId) -> SixDofLimitsCinfo {
    let proton::world::WorldJoint::SixDof(info) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    info.limits
}

#[test]
fn e4_limited_hinge_loosens_violated_bound_and_restores_exactly() {
    let mut world = World::new(WorldCinfo::new());
    let (_, child, id) = hinge(&mut world);
    let original = hinge_limits(&world, id);
    world.set_body_transform(
        child,
        Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Y, 0.6),
            Vector4::from_xyz(1.0, 0.0, 0.0),
        ),
    );
    let mut action = EaseConstraintsAction::new(&world, &[id]);
    assert!(matches!(action.saved()[0], LimitSnapshot::LimitedHinge(_)));
    action.loosen(&mut world);
    let loose = hinge_limits(&world, id);
    assert!(loose.0 < original.0 || loose.1 > original.1);
    action.restore_now(&mut world);
    assert_eq!(hinge_limits(&world, id), original);
}

#[test]
fn e5_six_dof_saves_loosens_and_eases_back() {
    let mut world = World::new(WorldCinfo::new());
    let (child, id) = six_dof(&mut world);
    let original = six_dof_limits(&world, id);
    world.set_body_transform(
        child,
        Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.6),
            Vector4::from_xyz(0.0, 1.0, 0.0),
        ),
    );
    let mut action = EaseConstraintsAction::new(&world, &[id]);
    let LimitSnapshot::SixDof(saved) = action.saved()[0] else {
        panic!("expected 6DOF snapshot");
    };
    assert_eq!(saved.id, id);
    assert_eq!(saved.twist_min, original.twist_min);
    assert_eq!(saved.twist_max, original.twist_max);
    assert_eq!(saved.swing0_max, original.swing0_max);
    assert_eq!(saved.swing1_max, original.swing1_max);
    assert_eq!(saved.linear_limit, original.linear_limit);
    assert!(
        saved
            .linear_axes
            .iter()
            .all(|mode| *mode == SixDofAxisMode::Unlimited)
    );
    action.loosen(&mut world);
    let loose = six_dof_limits(&world, id);
    assert_eq!(loose.swing0_max, original.swing0_max);
    assert!(loose.swing1_max > original.swing1_max);
    action.restore(&mut world, 1.0);
    assert!(!action.step(&mut world, 0.25));
    let halfway = six_dof_limits(&world, id).swing1_max;
    assert!(halfway < loose.swing1_max && halfway > original.swing1_max);
    assert!(!action.step(&mut world, 0.25));
    assert!(!action.step(&mut world, 0.25));
    assert!(action.step(&mut world, 0.25));
    assert_eq!(six_dof_limits(&world, id), original);
}

#[test]
fn e6_mixed_families_keep_snapshot_order() {
    let mut world = World::new(WorldCinfo::new());
    let (_, _, hinge_id) = hinge(&mut world);
    let (_, dof_id) = six_dof(&mut world);
    let a = body(&mut world, Vector4::from_xyz(4.0, 0.0, 0.0));
    let b = body(&mut world, Vector4::from_xyz(5.0, 0.0, 0.0));
    let mut data = RagdollData::new();
    data.add_bone("a", None, Some(a), Transform::IDENTITY);
    data.add_bone(
        "b",
        Some(0),
        Some(b),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
    );
    let ragdoll = Ragdoll::create(&mut world, &data).expect("ragdoll");
    let ragdoll_id = ragdoll.constraints()[1].expect("ragdoll joint");

    let action = EaseConstraintsAction::new(&world, &[hinge_id, dof_id, ragdoll_id]);
    assert!(matches!(action.saved()[0], LimitSnapshot::LimitedHinge(_)));
    assert!(matches!(action.saved()[1], LimitSnapshot::SixDof(_)));
    assert!(matches!(action.saved()[2], LimitSnapshot::Ragdoll(_)));
}
