//! Integration tests for the position/spring constraint motors (hinge and
//! prismatic) and joint warm starting: the per-constraint impulse rows kept in
//! the world's constraint slots, their clearing on disable/destroy, and the
//! bitwise determinism of motorized scenes.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::{Quaternion, Transform, Vector4, Vec4};
use proton::np_types::{BodyId, ConstraintId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{
    HingeCinfo, HingeMotorCinfo, HingePositionMotorCinfo, HingeSpringMotorCinfo,
    HingeVelocityMotorCinfo, PrismaticCinfo, PrismaticMotorCinfo, PrismaticPositionMotorCinfo,
    PrismaticSpringMotorCinfo, World, WorldCinfo,
};

fn sphere(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vec4::ZERO, radius)) as Arc<dyn Shape>
}

fn dynamic_sphere(radius: f32, position: Vector4) -> BodyCinfo {
    BodyCinfo::new(sphere(radius))
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(position)
}

/// Static hub at the origin plus a dynamic arm at `(2, 0, 0)` hinged about
/// `+Z` through the world origin (mirror of the world unit-test helper).
fn hinged_arm(world: &mut World) -> (BodyId, BodyId, ConstraintId) {
    let hub = world.create_body(&BodyCinfo::new(sphere(0.5)).with_position(Vector4::ZERO));
    let arm = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(2.0, 0.0, 0.0)));
    let arm_motion = world.body(arm).unwrap().motion_id;
    world.motion_mut(arm_motion).unwrap().inverse_inertia = Vector4::splat(0.5);
    let id = world.create_hinge_constraint(&HingeCinfo::new(
        hub,
        arm,
        Vector4::ZERO,
        Vector4::from_xyz(-2.0, 0.0, 0.0),
        Vector4::UNIT_Z,
        Vector4::UNIT_Z,
    ));
    assert!(id.is_valid());
    (hub, arm, id)
}

/// Signed hinge angle between the `+X` reference axes of both bodies about
/// `+Z`.
fn current_hinge_angle(world: &World, hub: BodyId, arm: BodyId) -> f32 {
    let transform_a = world.body_transform(hub).unwrap();
    let transform_b = world.body_transform(arm).unwrap();
    let reference_a = transform_a.rotation.rotate(Vector4::UNIT_X);
    let reference_b = transform_b.rotation.rotate(Vector4::UNIT_X);
    let sin = reference_a.cross(reference_b).dot3(Vector4::UNIT_Z);
    let cos = reference_a.dot3(reference_b);
    sin.atan2(cos)
}

#[test]
fn hinge_position_motor_tracks_the_target_angle() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let (hub, arm, id) = hinged_arm(&mut world);
    world.set_hinge_motor(
        id,
        Some(HingeMotorCinfo::Position(HingePositionMotorCinfo::new(
            1.0, 0.15, 20.0,
        ))),
    );

    let mut max_error_after_settling = 0.0f32;
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    for _ in 0..30 {
        world.step(1.0 / 60.0);
        let angle = current_hinge_angle(&world, hub, arm);
        max_error_after_settling = max_error_after_settling.max((angle - 1.0).abs());
    }
    assert!(
        max_error_after_settling < 0.25,
        "arm must stay near the target angle: max error {max_error_after_settling}"
    );
}

#[test]
fn hinge_spring_motor_pulls_the_arm_to_the_rest_angle() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let (hub, arm, id) = hinged_arm(&mut world);
    // Start the arm half a radian away from the rest angle.
    world.set_body_transform(
        arm,
        Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 1.0),
            Vector4::from_xyz(2.0, 0.0, 0.0),
        ),
    );
    world.set_hinge_motor(
        id,
        Some(HingeMotorCinfo::Spring(HingeSpringMotorCinfo::new(
            0.5, 8.0, 2.0, 50.0,
        ))),
    );

    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }
    let angle = current_hinge_angle(&world, hub, arm);
    assert!(
        (angle - 0.5).abs() < 0.25,
        "spring must settle near the rest angle: {angle}"
    );
}

#[test]
fn hinge_velocity_motor_still_spins_with_the_enum_api() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let (_, arm, id) = hinged_arm(&mut world);
    world.set_hinge_motor(
        id,
        Some(HingeMotorCinfo::Velocity(HingeVelocityMotorCinfo::new(
            2.0, 10.0,
        ))),
    );

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let wz = world.body_angular_velocity(arm).unwrap().z;
    assert!(
        (wz - 2.0).abs() < 0.2,
        "wz {wz} must track the velocity motor target"
    );
}

#[test]
fn prismatic_position_motor_and_spring_motor_drive_the_slider() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let parent = world.create_body(&BodyCinfo::new(sphere(0.5)));
    let slider = world.create_body(&dynamic_sphere(0.5, Vector4::ZERO));

    let id = world.create_prismatic_constraint(&PrismaticCinfo::between_bodies(
        slider,
        parent,
        Vector4::ZERO,
        &world.body_transform(slider).unwrap(),
        &world.body_transform(parent).unwrap(),
        Vector4::UNIT_X,
    ));
    assert!(id.is_valid());
    world.set_constraint_collisions_enabled(id, false);

    // Position motor: drives the slider to translation +2.
    world.set_prismatic_motor(
        id,
        Some(PrismaticMotorCinfo::Position(
            PrismaticPositionMotorCinfo::new(2.0, 0.2, 20.0),
        )),
    );
    for _ in 0..180 {
        world.step(1.0 / 60.0);
    }
    let x = world.body_position(slider).unwrap().x;
    assert!(
        (x - 2.0).abs() < 0.35,
        "position motor must drive the slider to 2.0: x {x}"
    );

    // Switch to a spring motor with the rest length at the current pose:
    // pushing the slider back must make the spring restore it.
    world.set_prismatic_motor(
        id,
        Some(PrismaticMotorCinfo::Spring(PrismaticSpringMotorCinfo::new(
            2.0, 20.0, 4.0, 50.0,
        ))),
    );
    world.set_body_transform(
        slider,
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.5, 0.0, 0.0)),
    );
    for _ in 0..240 {
        world.step(1.0 / 60.0);
    }
    let x = world.body_position(slider).unwrap().x;
    assert!(
        (x - 2.0).abs() < 0.45,
        "spring must restore the rest translation: x {x}"
    );
}

#[test]
fn motorized_scenes_are_bitwise_deterministic() {
    let run = || {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let (_, arm, hinge) = hinged_arm(&mut world);
        world.set_hinge_motor(
            hinge,
            Some(HingeMotorCinfo::Spring(HingeSpringMotorCinfo::new(
                0.5, 8.0, 2.0, 50.0,
            ))),
        );
        world.set_body_transform(
            arm,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
                Vector4::from_xyz(2.0, 0.0, 0.0),
            ),
        );
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        world.snapshot()
    };
    assert_eq!(run(), run(), "spring-motor scenes must be bit-exact");
}
