//! Limited-hinge and 6DOF joint-family integration scenarios.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::{Quaternion, Transform, Vector4};
use proton::np_types::{BodyId, ConstraintId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{
    HingeCinfo, HingeLimitsCinfo, JointType, SixDofCinfo, SixDofLimitsCinfo, World, WorldCinfo,
};

fn body(world: &mut World, position: Vector4) -> BodyId {
    let shape = Arc::new(SphereShape::new(Vector4::ZERO, 0.2)) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(position),
    )
}

fn limited_hinge(world: &mut World) -> (BodyId, BodyId, ConstraintId) {
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
    let id = world.create_limited_hinge_constraint(&info);
    (a, b, id)
}

fn six_dof(world: &mut World) -> (BodyId, BodyId, ConstraintId) {
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
    let id = world.create_6dof_constraint(&info);
    (a, b, id)
}

#[test]
fn j1_limited_hinge_stores_and_updates_angular_limits() {
    let mut world = World::new(WorldCinfo::new());
    let (_, _, id) = limited_hinge(&mut world);
    assert!(id.is_valid());
    assert_eq!(
        world.constraint(id).unwrap().joint_type(),
        JointType::LimitedHinge
    );

    assert!(world.set_limited_hinge_limits(id, None));
    let proton::world::WorldJoint::LimitedHinge(info) = world.constraint(id).unwrap() else {
        panic!("expected limited hinge");
    };
    assert!(info.limits.is_none());

    let limits = HingeLimitsCinfo::new(-0.4, 0.6);
    assert!(world.set_limited_hinge_limits(id, Some(limits)));
    let proton::world::WorldJoint::LimitedHinge(info) = world.constraint(id).unwrap() else {
        panic!("expected limited hinge");
    };
    assert_eq!(info.limits, Some(limits));
    assert!(!world.set_limited_hinge_limits(ConstraintId::INVALID, Some(limits)));
}

#[test]
fn j2_six_dof_limits_distance_and_angles() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let (_, child, id) = six_dof(&mut world);
    assert_eq!(
        world.constraint(id).unwrap().joint_type(),
        JointType::SixDof
    );

    world.set_body_transform(
        child,
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.4, 0.0)),
    );
    world.step(1.0 / 60.0);
    let velocity = world.body_linear_velocity(child).unwrap();
    assert!(
        velocity.y < 0.0,
        "6DOF linear limit should push the child back toward the anchor: {velocity:?}"
    );

    let limits = SixDofLimitsCinfo::new(-0.5, 0.5, 0.4, 0.3, 0.2);
    assert!(world.set_6dof_limits(id, limits));
    let proton::world::WorldJoint::SixDof(info) = world.constraint(id).unwrap() else {
        panic!("expected 6DOF joint");
    };
    assert_eq!(info.limits, limits);
}

#[test]
fn j3_new_joint_families_are_bit_deterministic() {
    fn run() -> String {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vector4::ZERO);
        let (_, hinge_child, hinge_id) = limited_hinge(&mut world);
        let (_, dof_child, dof_id) = six_dof(&mut world);
        world.set_body_transform(
            hinge_child,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Y, 0.5),
                Vector4::from_xyz(1.0, 0.0, 0.0),
            ),
        );
        world.set_body_transform(
            dof_child,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
                Vector4::from_xyz(0.0, 1.0, 0.0),
            ),
        );
        world.set_limited_hinge_limits(hinge_id, Some(HingeLimitsCinfo::new(-0.2, 0.2)));
        world.set_6dof_limits(dof_id, SixDofLimitsCinfo::new(-0.3, 0.3, 0.3, 0.3, 0.15));
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    }
    assert_eq!(run(), run());
}
