//! Ragdoll detail subsets: plane-angle limits, 6DOF per-axis translation
//! limits and shared-motion mass merging.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{
    EaseConstraintsAction, LimitSnapshot, Ragdoll, RagdollData, RagdollLimits,
};
use proton::math::{Quaternion, Transform, Vector4, Vec4};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{
    MotionMassUpdateMode, SixDofAxisMode, SixDofCinfo, SixDofLimitsCinfo, World, WorldCinfo,
    WorldJoint,
};

fn body(world: &mut World, position: Vector4, mass: f32) -> BodyId {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.2)) as Arc<dyn Shape>;
    let id = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(mass)
            .with_position(position),
    );
    world.set_body_inverse_inertia(id, Vector4::splat(0.5));
    id
}

#[test]
fn d1_plane_angle_limits_are_configurable_and_default_off() {
    fn run() -> (String, bool) {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vector4::ZERO);
        let a = body(&mut world, Vector4::ZERO, 1.0);
        let b = body(&mut world, Vector4::from_xyz(1.0, 0.0, 0.0), 1.0);
        let mut data = RagdollData::new();
        data.add_bone("a", None, Some(a), Transform::IDENTITY);
        data.add_bone(
            "b",
            Some(0),
            Some(b),
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
        );
        data.bones[1].limits = RagdollLimits::default().with_plane_limits(-0.025, 0.025);
        let ragdoll = Ragdoll::create(&mut world, &data).expect("ragdoll");
        let id = ragdoll.constraints()[1].expect("child joint");
        let WorldJoint::Ragdoll(info) = world.constraint(id).unwrap() else {
            panic!("expected ragdoll joint");
        };
        assert_eq!(info.plane_limits, Some((-0.025, 0.025)));

        assert!(world.set_ragdoll_plane_limits(id, Some((-0.2, 0.2))));
        let WorldJoint::Ragdoll(info) = world.constraint(id).unwrap() else {
            panic!("expected ragdoll joint");
        };
        assert_eq!(info.plane_limits, Some((-0.2, 0.2)));
        assert!(!world.set_ragdoll_plane_limits(id, Some((0.2, -0.2))));
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        (
            world.snapshot().to_text(),
            world.set_ragdoll_plane_limits(id, None),
        )
    }
    let (trace, disabled) = run();
    assert!(disabled);
    assert_eq!(run().0, trace);
}

#[test]
fn d2_six_dof_per_axis_modes_push_back_and_are_settable() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let a = body(&mut world, Vector4::ZERO, 1.0);
    let b = body(&mut world, Vector4::ZERO, 1.0);
    let limits = SixDofLimitsCinfo::new(-1.0, 1.0, 1.0, 1.0, 1.0)
        .with_linear_axis(0, SixDofAxisMode::Limited, -0.1, 0.1)
        .with_linear_axis(1, SixDofAxisMode::Fixed, 0.0, 0.0);
    let info = SixDofCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
        Vector4::UNIT_Y,
        Vector4::UNIT_Y,
        Vector4::UNIT_Z,
        Vector4::UNIT_Z,
        limits,
    );
    let id = world.create_6dof_constraint(&info);
    world.set_body_transform(
        b,
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 0.4, 0.3)),
    );
    world.step(1.0 / 60.0);
    let velocity = world.body_linear_velocity(b).unwrap();
    assert!(
        velocity.y < 0.0,
        "limited twist axis must push back: {velocity:?}"
    );
    assert!(
        velocity.z < 0.0,
        "fixed perpendicular axis must push back: {velocity:?}"
    );

    let WorldJoint::SixDof(stored) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    assert_eq!(stored.limits.linear_axes[0], SixDofAxisMode::Limited);
    assert_eq!(stored.limits.linear_axes[1], SixDofAxisMode::Fixed);
    assert_eq!(stored.limits.linear_axes[2], SixDofAxisMode::Unlimited);

    let mut updated = stored.limits;
    updated = updated.with_linear_axis(0, SixDofAxisMode::Unlimited, 0.0, 0.0);
    assert!(world.set_6dof_limits(id, updated));
    let WorldJoint::SixDof(stored) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    assert_eq!(stored.limits.linear_axes[0], SixDofAxisMode::Unlimited);
    assert_eq!(stored.limits.linear_axes[1], SixDofAxisMode::Fixed);
}

#[test]
fn d3_shared_motion_rebuilds_mass_com_and_inertia() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let a = body(&mut world, Vector4::ZERO, 1.0);
    let b = body(&mut world, Vector4::from_xyz(2.0, 0.0, 0.0), 3.0);
    let motion = world.body_motion_id(a).unwrap();
    assert!(world.attach_bodies_with_mode(a, &[b], MotionMassUpdateMode::Rebuild));
    let combined = world.motion(motion).unwrap();
    assert_eq!(combined.mass, 4.0);
    assert!(
        combined
            .position
            .all_equal3(Vector4::from_xyz(1.5, 0.0, 0.0), 1e-6)
    );
    assert!(combined.inverse_inertia.x > 0.0);
    assert!(combined.inverse_inertia_world.element(0, 0) > 0.0);
    assert_eq!(
        combined.inverse_inertia,
        Vector4::from_xyz(
            combined.inverse_inertia_world.element(0, 0),
            combined.inverse_inertia_world.element(1, 1),
            combined.inverse_inertia_world.element(2, 2),
        )
    );

    assert!(world.detach_bodies_with_mode(&[b], MotionMassUpdateMode::Rebuild));
    let remaining = world.motion(motion).unwrap();
    assert_eq!(remaining.mass, 1.0);
    assert!(remaining.position.all_equal3(Vector4::ZERO, 1e-6));
    assert_eq!(
        world.motion(world.body_motion_id(b).unwrap()).unwrap().mass,
        3.0
    );
}

#[test]
fn d4_detail_scenario_is_bitwise_deterministic() {
    fn trace() -> String {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vector4::ZERO);
        let a = body(&mut world, Vector4::ZERO, 2.0);
        let b = body(&mut world, Vector4::from_xyz(1.0, 0.5, 0.0), 1.0);
        let cinfo = SixDofCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::from_xyz(-1.0, 0.0, 0.0),
            Vector4::UNIT_Y,
            Vector4::UNIT_Y,
            Vector4::UNIT_X,
            Vector4::UNIT_X,
            SixDofLimitsCinfo::new(-0.3, 0.3, 0.3, 0.3, 0.4).with_linear_axis(
                0,
                SixDofAxisMode::Limited,
                -0.1,
                0.1,
            ),
        );
        world.create_6dof_constraint(&cinfo);
        world.attach_bodies_with_mode(a, &[b], MotionMassUpdateMode::Rebuild);
        world.set_body_linear_velocity(a, Vector4::from_xyz(0.5, 0.0, 0.0));
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    }
    assert_eq!(trace(), trace());
}

fn plane_ragdoll(world: &mut World) -> (Vec<proton::np_types::ConstraintId>, BodyId) {
    let a = body(world, Vector4::ZERO, 1.0);
    let b = body(world, Vector4::from_xyz(1.0, 0.0, 0.0), 1.0);
    let mut data = RagdollData::new();
    data.add_bone("a", None, Some(a), Transform::IDENTITY);
    data.add_bone(
        "b",
        Some(0),
        Some(b),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
    );
    data.bones[1].limits = RagdollLimits::default().with_plane_limits(-0.025, 0.025);
    let ragdoll = Ragdoll::create(world, &data).expect("ragdoll");
    let ids: Vec<_> = ragdoll.constraints().iter().flatten().copied().collect();
    (ids, b)
}

#[test]
fn d5_plane_angle_limits_are_saved_loosened_and_restored() {
    let mut world = World::new(WorldCinfo::new());
    let (ids, child) = plane_ragdoll(&mut world);
    let id = ids[0];
    world.set_body_transform(
        child,
        Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Y, 0.6),
            Vector4::from_xyz(1.0, 0.0, 0.0),
        ),
    );
    let mut action = EaseConstraintsAction::new(&world, &[id]);
    let LimitSnapshot::Ragdoll(saved) = action.saved()[0] else {
        panic!("expected ragdoll snapshot");
    };
    assert_eq!(saved.plane_limits, Some((-0.025, 0.025)));
    action.loosen(&mut world);
    let WorldJoint::Ragdoll(info) = world.constraint(id).unwrap() else {
        panic!("expected ragdoll joint");
    };
    let loose = info.plane_limits.unwrap();
    assert!(loose.0 <= -0.025 || loose.1 >= 0.025);
    action.restore_now(&mut world);
    let WorldJoint::Ragdoll(info) = world.constraint(id).unwrap() else {
        panic!("expected ragdoll joint");
    };
    assert_eq!(info.plane_limits, Some((-0.025, 0.025)));
}

#[test]
fn d6_six_dof_axis_limits_are_saved_loosened_and_restored() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let a = body(&mut world, Vector4::ZERO, 1.0);
    let b = body(&mut world, Vector4::ZERO, 1.0);
    let limit = SixDofLimitsCinfo::new(-1.0, 1.0, 1.0, 1.0, 1.0)
        .with_linear_axis(0, SixDofAxisMode::Limited, -0.1, 0.1)
        .with_linear_axis(1, SixDofAxisMode::Fixed, 0.0, 0.0);
    let id = world.create_6dof_constraint(&SixDofCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
        Vector4::UNIT_Y,
        Vector4::UNIT_Y,
        Vector4::UNIT_X,
        Vector4::UNIT_X,
        limit,
    ));
    world.set_body_transform(
        b,
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 0.4, 0.3)),
    );
    let mut action = EaseConstraintsAction::new(&world, &[id]);
    let LimitSnapshot::SixDof(saved) = action.saved()[0] else {
        panic!("expected 6DOF snapshot");
    };
    assert_eq!(saved.linear_axes[0], SixDofAxisMode::Limited);
    assert_eq!(saved.linear_axes[1], SixDofAxisMode::Fixed);
    assert_eq!(saved.linear_min[0], -0.1);
    assert_eq!(saved.linear_max[0], 0.1);
    action.loosen(&mut world);
    let WorldJoint::SixDof(info) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    assert!(info.limits.linear_min[0] < -0.1);
    assert_eq!(info.limits.linear_axes[1], SixDofAxisMode::Limited);
    action.restore_now(&mut world);
    let WorldJoint::SixDof(info) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    assert_eq!(info.limits, limit);
}
