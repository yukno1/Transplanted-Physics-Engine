//! Integration tests for the Phase 10 follow-up deliverables: joint
//! constraints (ball-and-socket / hinge / fixed) and world queries
//! (raycast / closest point), mirroring the scenario style of
//! `simulation_scenarios.rs`.

use std::sync::Arc;

use havok::body::BodyCinfo;
use havok::math::{Quaternion, Vector4};
use havok::np_types::{BodyId, ConstraintId, MotionType};
use havok::shape::{BoxShape, SphereShape};
use havok::world::{BallAndSocketCinfo, FixedCinfo, HingeCinfo, World, WorldCinfo};

fn sphere(radius: f32) -> Arc<dyn havok::shape::Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn havok::shape::Shape>
}

fn box_shape(half_extents: Vector4) -> Arc<dyn havok::shape::Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn havok::shape::Shape>
}

fn dynamic_sphere(radius: f32, position: Vector4) -> BodyCinfo {
    BodyCinfo::new(sphere(radius))
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(position)
}

#[test]
fn ball_socket_pendulum_keeps_pivots_coincident() {
    let mut world = World::new(WorldCinfo::new().with_solver_settings(0.2, 0.0, 16));
    let anchor = Vector4::from_xyz(0.0, 10.0, 0.0);
    let anchor_body = world.create_body(&BodyCinfo::new(sphere(0.1)).with_position(anchor));
    let bob = world.create_body(&dynamic_sphere(
        0.5,
        anchor + Vector4::from_xyz(2.0, 0.0, 0.0),
    ));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        anchor_body,
        bob,
        Vector4::UNIT_X,
        -Vector4::UNIT_X,
    ));
    assert!(id.is_valid());

    let pivot = anchor + Vector4::UNIT_X;
    for _ in 0..240 {
        world.step(1.0 / 60.0);
        let pivot_b = world
            .body_transform(bob)
            .unwrap()
            .transform_point(-Vector4::UNIT_X);
        assert!((pivot_b - pivot).length3() < 0.4, "pivots drifted apart");
    }
}

#[test]
fn fixed_pair_falls_rigidly() {
    let mut world = World::new(WorldCinfo::new());
    let a = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(0.0, 5.0, 0.0)));
    let b = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(2.0, 5.0, 0.0)));
    let id = world.create_fixed_constraint(&FixedCinfo::between_bodies(
        a,
        b,
        Vector4::from_xyz(1.0, 5.0, 0.0),
        &world.body_transform(a).unwrap(),
        &world.body_transform(b).unwrap(),
    ));
    assert!(id.is_valid());
    assert_eq!(world.constraint_count(), 1);

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let separation = (world.body_position(b).unwrap() - world.body_position(a).unwrap()).length3();
    assert!(
        (separation - 2.0).abs() < 0.3,
        "fixed pair separation {}",
        separation
    );
    assert!(world.body_position(a).unwrap().y < 5.0 - 1.0);
}

#[test]
fn hinge_constraint_is_deterministic_across_runs() {
    let build = || {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let hub = world.create_body(&BodyCinfo::new(sphere(0.5)));
        let arm = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(2.0, 0.0, 0.0)));
        world.create_hinge_constraint(&HingeCinfo::new(
            hub,
            arm,
            Vector4::UNIT_X,
            -Vector4::UNIT_X,
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
        ));
        world.set_body_angular_velocity(arm, Vector4::from_xyz(0.0, 3.0, 1.0));
        (world, arm)
    };

    let (mut a, arm_a) = build();
    let (mut b, arm_b) = build();
    for _ in 0..120 {
        a.step(1.0 / 60.0);
        b.step(1.0 / 60.0);
    }
    assert_eq!(a.snapshot(), b.snapshot());
    // The hinge removes rotation about Y, so the arm stays close to the hub.
    let distance = (a.body_position(arm_a).unwrap() - Vector4::ZERO).length3();
    assert!(distance < 4.0, "arm distance {}", distance);
    let _ = arm_b;
}

#[test]
fn constraint_ids_survive_disable_and_destroy() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(&dynamic_sphere(0.5, Vector4::ZERO));
    let b = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(1.0, 0.0, 0.0)));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
    ));
    assert_ne!(id, ConstraintId::INVALID);
    assert!(id.is_constraint_index());

    assert!(world.set_constraint_enabled(id, false));
    assert!(world.set_constraint_enabled(id, true));
    assert!(world.constraint(id).is_some());

    assert!(world.destroy_constraint(id));
    assert!(!world.destroy_constraint(id));
    assert_eq!(world.constraint_count(), 0);

    // Destroying a body removes its constraints.
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
    ));
    assert!(world.destroy_body(b));
    assert!(world.constraint(id).is_none());
    assert_eq!(world.constraint_count(), 0);
}

#[test]
fn raycast_selects_closest_body_and_respects_range() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let near = world.create_body(&dynamic_sphere(1.0, Vector4::from_xyz(0.0, 4.0, 0.0)));
    world.create_body(&dynamic_sphere(1.0, Vector4::from_xyz(0.0, 10.0, 0.0)));

    let hit = world.raycast(Vector4::ZERO, Vector4::UNIT_Y, 50.0).unwrap();
    assert_eq!(hit.body, near);
    assert!((hit.hit_fraction - 3.0).abs() < 1e-2);
    assert!(world.raycast(Vector4::ZERO, Vector4::UNIT_Y, 2.0).is_none());
}

#[test]
fn raycast_hits_rotated_boxes_deterministically() {
    let build = || {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(1.0)))
                .with_orientation(Quaternion::from_axis_angle(
                    Vector4::UNIT_Z,
                    core::f32::consts::FRAC_PI_4,
                ))
                .with_position(Vector4::from_xyz(0.0, 5.0, 0.0)),
        );
        world
    };
    let mut first = build();
    let mut second = build();
    for _ in 0..5 {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
    }
    let hit_a = first.raycast(Vector4::from_xyz(0.0, 20.0, 0.0), -Vector4::UNIT_Y, 30.0);
    let hit_b = second.raycast(Vector4::from_xyz(0.0, 20.0, 0.0), -Vector4::UNIT_Y, 30.0);
    assert_eq!(hit_a, hit_b);
    let hit = hit_a.unwrap();
    assert!(hit.hit_fraction < 15.0);
    assert!(
        hit.normal.dot3(-Vector4::UNIT_Y) < -0.7,
        "normal {:?}",
        hit.normal
    );
}

#[test]
fn closest_point_finds_nearest_surface() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let near = world.create_body(&dynamic_sphere(1.0, Vector4::from_xyz(0.0, 3.0, 0.0)));
    world.create_body(&dynamic_sphere(1.0, Vector4::from_xyz(0.0, 20.0, 0.0)));

    let hit = world.closest_point(Vector4::ZERO, 50.0).unwrap();
    assert_eq!(hit.body, near);
    assert!((hit.distance - 2.0).abs() < 1e-2);
    assert!(hit.point.all_equal3(Vector4::from_xyz(0.0, 2.0, 0.0), 1e-2));
    assert!(world.closest_point(Vector4::ZERO, 1.0).is_none());

    // Inside a shape: distance 0.
    let inside = world
        .closest_point(Vector4::from_xyz(0.0, 3.0, 0.0), 10.0)
        .unwrap();
    assert_eq!(inside.distance, 0.0);
}

#[test]
fn invalid_constraint_bodies_are_rejected() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let body = world.create_body(&dynamic_sphere(0.5, Vector4::ZERO));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        BodyId::invalid(),
        body,
        Vector4::ZERO,
        Vector4::ZERO,
    ));
    assert_eq!(id, ConstraintId::INVALID);
    assert_eq!(world.constraint_count(), 0);
}

#[test]
fn raycast_and_contacts_coexist_in_one_scene() {
    let mut world = World::new(WorldCinfo::new());
    let ground = world.create_body(
        &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
            .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
    let ball = world.create_body(&dynamic_sphere(0.5, Vector4::from_xyz(0.0, 2.0, 0.0)));

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    // The settled ball rests on the ground and the ray hits its top surface.
    assert!(world.last_contact_count() > 0);
    let hit = world.raycast(Vector4::from_xyz(0.0, 5.0, 0.0), -Vector4::UNIT_Y, 10.0);
    let hit = hit.expect("ray must reach the settled ball");
    assert_eq!(hit.body, ball.min(ground).max(ball));
    assert!(hit.hit_fraction < 5.0);
}

/// Hub at the origin plus a hinged arm with rotational response, gravity off.
fn hinged_arm_world() -> (World, BodyId, BodyId, ConstraintId) {
    let mut world = World::new(
        WorldCinfo::new()
            .with_gravity(Vector4::ZERO)
            .with_solver_settings(0.2, 0.0, 16),
    );
    let hub = world.create_body(&BodyCinfo::new(sphere(0.5)));
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
    (world, hub, arm, id)
}

/// Signed angle of the arm about the hinge (world `+Z`), measured between
/// the identity twist axis `+X` mapped through both body rotations.
fn hinge_angle_of(world: &World, hub: BodyId, arm: BodyId) -> f32 {
    let twist_a = world
        .body_transform(hub)
        .unwrap()
        .rotation
        .rotate(Vector4::UNIT_X);
    let twist_b = world
        .body_transform(arm)
        .unwrap()
        .rotation
        .rotate(Vector4::UNIT_X);
    let sin = twist_a.cross(twist_b).dot3(Vector4::UNIT_Z);
    sin.atan2(twist_a.dot3(twist_b))
}

#[test]
fn hinge_limits_stop_the_swing_at_the_bound() {
    let (mut world, hub, arm, id) = hinged_arm_world();
    // Start the arm rotated past the ±0.2 rad window.
    world.set_body_transform(
        arm,
        havok::math::Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.6),
            Vector4::from_xyz(2.0, 0.0, 0.0),
        ),
    );
    world.set_hinge_limits(
        id,
        Some(havok::world::HingeLimitsCinfo::new(
            Vector4::UNIT_X,
            Vector4::UNIT_X,
            -0.2,
            0.2,
        )),
    );

    let initial = hinge_angle_of(&world, hub, arm);
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let final_angle = hinge_angle_of(&world, hub, arm);
    assert!(
        final_angle < initial - 0.1,
        "limits must drive the angle back: {} -> {}",
        initial,
        final_angle
    );
    assert!(
        final_angle < 0.45 && final_angle > -0.45,
        "angle must stay near the bound: {}",
        final_angle
    );
}

#[test]
fn hinge_motor_spins_the_arm_at_target_speed() {
    let (mut world, _, arm, id) = hinged_arm_world();
    world.set_hinge_motor(id, Some(havok::world::HingeMotorCinfo::new(1.5, 10.0)));

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let wz = world.body_angular_velocity(arm).unwrap().z;
    assert!((wz - 1.5).abs() < 0.2, "wz {} must track the target", wz);

    // Disabling the motor lets the arm keep its current speed (frictionless).
    assert!(world.set_hinge_motor(id, None));
    world.step(1.0 / 60.0);
    let wz_after = world.body_angular_velocity(arm).unwrap().z;
    assert!(
        (wz_after - wz).abs() < 0.3,
        "frictionless hinge must not brake: {} -> {}",
        wz,
        wz_after
    );
}

#[test]
fn hinge_limits_and_motor_are_bitwise_deterministic() {
    let run = || {
        let (mut world, _, arm, id) = hinged_arm_world();
        world.set_hinge_limits(
            id,
            Some(havok::world::HingeLimitsCinfo::new(
                Vector4::UNIT_X,
                Vector4::UNIT_X,
                -0.2,
                0.2,
            )),
        );
        world.set_hinge_motor(id, Some(havok::world::HingeMotorCinfo::new(3.0, 2.0)));
        world.set_body_transform(
            arm,
            havok::math::Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.5),
                Vector4::from_xyz(2.0, 0.0, 0.0),
            ),
        );
        for _ in 0..90 {
            world.step(1.0 / 60.0);
        }
        world.snapshot()
    };
    assert_eq!(run(), run());
}
