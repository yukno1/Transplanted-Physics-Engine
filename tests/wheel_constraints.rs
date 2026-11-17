//! Wheel joint integration scenarios (wheel/vehicle constraint subset,
//! `hknpWheelConstraintData` port).
//!
//! These tests exercise the public facade (`proton`): a chassis box resting
//! on sphere wheels through suspension spring-dampers, drive motors and
//! steering, plus the usual determinism guarantees.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::Vector4;
use proton::np_types::{BodyId, ConstraintId, MotionType};
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::world::{
    JointType, WheelCinfo, WheelDriveCinfo, WheelSuspensionCinfo, World, WorldCinfo,
};

fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
}

/// A chassis box resting on two sphere wheels via wheel joints.
fn wheeled_cart(world: &mut World, drive: Option<(f32, f32)>) -> (BodyId, [ConstraintId; 2]) {
    world.create_body(
        &BodyCinfo::new(box_shape(Vector4::from_xyz(10.0, 0.5, 10.0)))
            .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
    let chassis = world.create_body(
        &BodyCinfo::new(box_shape(Vector4::from_xyz(1.0, 0.25, 0.5)))
            .with_motion_type(MotionType::Dynamic)
            .with_mass(2.0)
            .with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
    );
    let mut ids = [ConstraintId::INVALID; 2];
    for (index, x) in [0.7, -0.7].into_iter().enumerate() {
        let wheel = world.create_body(
            &BodyCinfo::new(sphere_shape(0.3))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(0.2)
                .with_position(Vector4::from_xyz(x, 0.3, 0.0)),
        );
        let motion_id = world.body(wheel).unwrap().motion_id;
        world.motion_mut(motion_id).unwrap().inverse_inertia = Vector4::splat(0.05);
        let mut info = WheelCinfo::new(
            wheel,
            chassis,
            Vector4::ZERO,
            Vector4::from_xyz(x, -0.25, 0.0),
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::UNIT_X,
            Vector4::UNIT_Z,
        )
        .with_suspension(WheelSuspensionCinfo::new(0.45, 120.0, 8.0, 50.0))
        .with_travel(0.25, 0.65);
        info.axle_b = Vector4::UNIT_X;
        if let Some((velocity, max_impulse)) = drive {
            info.drive = Some(WheelDriveCinfo::new(velocity, max_impulse));
        }
        ids[index] = world.create_wheel_constraint(&info);
    }
    (chassis, ids)
}

#[test]
fn w1_suspension_supports_the_chassis_and_settles() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, ids) = wheeled_cart(&mut world, None);
    assert!(ids[0].is_valid() && ids[1].is_valid());
    assert_eq!(
        world.constraint(ids[0]).unwrap().joint_type(),
        JointType::Wheel
    );

    for _ in 0..180 {
        world.step(1.0 / 60.0);
    }
    let position = world.body_position(chassis).unwrap();
    assert!(
        position.y > 0.5 && position.y < 1.5,
        "chassis must settle on its suspension: {position:?}"
    );
    // Settled: the chassis velocity has damped out.
    let velocity = world.body_linear_velocity(chassis).unwrap();
    assert!(
        velocity.length3() < 0.5,
        "suspension damping must settle the chassis: {velocity:?}"
    );
}

#[test]
fn w2_drive_motor_propels_the_cart() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, _) = wheeled_cart(&mut world, Some((20.0, 2.0)));

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    // The axles point along X, so the cart drives along +Z.
    let position = world.body_position(chassis).unwrap();
    assert!(
        position.z > 0.5,
        "driven wheels must move the cart forward: {position:?}"
    );
    assert!(
        position.y > 0.0 && position.y < 1.6,
        "the cart must stay above the ground: {position:?}"
    );
}

#[test]
fn w3_steering_turns_the_cart() {
    // Same scene as W2 but with the front wheel steered: the heading must
    // diverge from the straight run.
    let run = |steer: f32| {
        let mut world = World::new(WorldCinfo::new());
        let (chassis, ids) = wheeled_cart(&mut world, Some((20.0, 2.0)));
        if steer != 0.0 {
            assert!(world.set_steering_angle(ids[0], steer));
        }
        for _ in 0..90 {
            world.step(1.0 / 60.0);
        }
        world.body_position(chassis).unwrap()
    };
    let straight = run(0.0);
    let steered = run(0.5);
    assert!(
        (steered.x - straight.x).abs() > 0.05 || (steered.y - straight.y).abs() > 0.05,
        "steering must change the heading: straight {straight:?} steered {steered:?}"
    );
    // Both runs still drive forward.
    assert!(straight.z > 0.5 && steered.z > 0.2);
}

#[test]
fn w4_steering_angle_setter_validates_ids() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let (_, ids) = wheeled_cart(&mut world, None);
    let id = ids[0];
    assert!(world.set_steering_angle(id, 0.3));
    assert!(world.set_steering_angle(id, -0.3));
    assert!(!world.set_steering_angle(ConstraintId::INVALID, 0.3));
    assert!(!world.set_wheel_drive(ConstraintId::INVALID, None));
}

#[test]
fn w5_wheel_scene_is_bitwise_deterministic() {
    let run = || {
        let mut world = World::new(WorldCinfo::new());
        wheeled_cart(&mut world, Some((20.0, 2.0)));
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    };
    assert_eq!(run(), run());
}

#[test]
fn w6_destroying_the_chassis_drops_the_wheel_joints() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let (chassis, ids) = wheeled_cart(&mut world, None);
    assert!(world.destroy_body(chassis));
    for id in ids {
        assert!(world.constraint(id).is_none());
    }
}
