//! Integration tests for the constraint/joint system and the query system:
//! ball-and-socket / hinge (limits + motor) / fixed / ragdoll / prismatic /
//! spring / point-to-plane joints, world ray casts, shape casts, closest
//! points and AABB queries. Mirrors the scenario style of
//! `simulation_scenarios.rs`.

use std::sync::Arc;

use proton::body::BodyCinfo;
use glam::Vec3;
use proton::math::{Quaternion, Quat, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{BodyId, ConstraintId, MotionType};
use proton::shape::{BoxShape, MeshShape, Shape, SphereShape};
use proton::world::{
    BallAndSocketCinfo, FixedCinfo, HingeCinfo, PointToPlaneCinfo, PrismaticCinfo, QueryFilter,
    RagdollCinfo, SpringCinfo, World, WorldCinfo,
};

fn sphere(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vec4::ZERO, radius)) as Arc<dyn Shape>
}

fn box_shape(half_extents: Vec4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn mesh_ground(size: f32) -> Arc<dyn Shape> {
    Arc::new(MeshShape::new(
        vec![
            Vec4::from_xyz(-size, 0.0, -size),
            Vec4::from_xyz(size, 0.0, -size),
            Vec4::from_xyz(size, 0.0, size),
            Vec4::from_xyz(-size, 0.0, size),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )) as Arc<dyn Shape>
}

fn dynamic_sphere(radius: f32, position: Vec4) -> BodyCinfo {
    BodyCinfo::new(sphere(radius))
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(position)
}

#[test]
fn ball_socket_pendulum_keeps_pivots_coincident() {
    let mut world = World::new(WorldCinfo::new().with_solver_settings(0.2, 0.0, 16));
    let anchor = Vec4::from_xyz(0.0, 10.0, 0.0);
    let anchor_body = world.create_body(&BodyCinfo::new(sphere(0.1)).with_position(anchor));
    let bob = world.create_body(&dynamic_sphere(
        0.5,
        anchor + Vec4::from_xyz(2.0, 0.0, 0.0),
    ));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        anchor_body,
        bob,
        Vector4::UNIT_X,
        -Vector4::UNIT_X,
    ));
    assert!(id.is_valid());

    let pivot = Vector4::new(anchor.x, anchor.y, anchor.z, 0.0) + Vector4::UNIT_X;
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
    let a = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(0.0, 5.0, 0.0)));
    let b = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(2.0, 5.0, 0.0)));
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
        let arm = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(2.0, 0.0, 0.0)));
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
    let a = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
    let b = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(1.0, 0.0, 0.0)));
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
    let near = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 4.0, 0.0)));
    world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 10.0, 0.0)));

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
            &BodyCinfo::new(box_shape(Vec4::splat(1.0)))
                .with_orientation(Quat::from_axis_angle(
                    Vec3::Z,
                    core::f32::consts::FRAC_PI_4,
                ))
                .with_position(Vec4::from_xyz(0.0, 5.0, 0.0)),
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
    let near = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 3.0, 0.0)));
    world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 20.0, 0.0)));

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
    let body = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
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
        &BodyCinfo::new(box_shape(Vec4::from_xyz(3.0, 0.5, 3.0)))
            .with_position(Vec4::from_xyz(0.0, -0.5, 0.0)),
    );
    let ball = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(0.0, 2.0, 0.0)));

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
    let arm = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(2.0, 0.0, 0.0)));
    let arm_motion = world.body(arm).unwrap().motion_id;
    world.motion_mut(arm_motion).unwrap().inverse_inertia = Vec4::splat(0.5);
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
        proton::math::Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.6),
            Vector4::from_xyz(2.0, 0.0, 0.0),
        ),
    );
    world.set_hinge_limits(id, Some(proton::world::HingeLimitsCinfo::new(-0.2, 0.2)));

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
    world.set_hinge_motor(
        id,
        Some(proton::world::HingeMotorCinfo::Velocity(
            proton::world::HingeVelocityMotorCinfo::new(1.5, 10.0),
        )),
    );

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
        world.set_hinge_limits(id, Some(proton::world::HingeLimitsCinfo::new(-0.2, 0.2)));
        world.set_hinge_motor(
            id,
            Some(proton::world::HingeMotorCinfo::Velocity(
                proton::world::HingeVelocityMotorCinfo::new(3.0, 2.0),
            )),
        );
        world.set_body_transform(
            arm,
            proton::math::Transform::from_quaternion(
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

#[test]
fn shape_cast_reaches_mesh_ground_and_ignores_the_caster() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let ground = world.create_body(&BodyCinfo::new(mesh_ground(10.0)));
    let caster = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(0.0, 5.0, 0.0)));
    let query = sphere(0.5);
    let start = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));

    let hit = world
        .cast_shape(
            query.as_ref(),
            &start,
            -Vector4::UNIT_Y,
            10.0,
            &QueryFilter::default(),
        )
        .expect("cast must hit something");
    assert_eq!(hit.body, caster, "the casting body overlaps the start pose");
    assert!(hit.start_penetrating);

    let filter = QueryFilter::default().with_ignore_body(caster);
    let hit = world
        .cast_shape(query.as_ref(), &start, -Vector4::UNIT_Y, 10.0, &filter)
        .expect("cast must hit the mesh ground");
    assert_eq!(hit.body, ground);
    assert!(
        (hit.distance - 4.5).abs() < 5e-3,
        "distance {}",
        hit.distance
    );
    assert!(hit.normal.dot3(Vector4::UNIT_Y) > 0.9);
    assert!(!hit.start_penetrating);

    // The ray query reaches the same mesh ground when the caster is filtered.
    let ray = world
        .raycast_filtered(
            Vector4::from_xyz(0.0, 5.0, 0.0),
            -Vector4::UNIT_Y,
            10.0,
            &QueryFilter::default().with_ignore_body(caster),
        )
        .expect("ray must hit the mesh ground");
    assert_eq!(ray.body, ground);
    assert!((ray.hit_fraction - 5.0).abs() < 1e-3);
}

#[test]
fn closest_points_query_reports_the_nearest_body() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let body = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 3.0, 0.0)));
    let query = sphere(0.5);
    let hit = world
        .closest_points_to_shape(
            query.as_ref(),
            &Transform::IDENTITY,
            10.0,
            &QueryFilter::default(),
        )
        .expect("the query sphere is within range");
    assert_eq!(hit.body, body);
    assert!(
        (hit.distance - 1.5).abs() < 1e-2,
        "distance {}",
        hit.distance
    );
    assert!(hit.normal.dot3(Vector4::UNIT_Y) < -0.9);
}

#[test]
fn aabb_query_respects_the_filter() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let first = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(0.0, 0.0, 0.0)));
    let second = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(1.5, 0.0, 0.0)));
    world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(20.0, 0.0, 0.0)));
    let aabb = proton::math::Aabb::from_center_half_extents(Vector4::ZERO, Vector4::splat(1.0));
    assert_eq!(
        world.query_aabb(&aabb, &QueryFilter::default()),
        vec![first, second]
    );
    assert_eq!(
        world.query_aabb(&aabb, &QueryFilter::default().with_ignore_body(first)),
        vec![second]
    );
}

#[test]
fn prismatic_slider_moves_along_its_axis() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let parent = world.create_body(&BodyCinfo::new(sphere(0.5)));
    let slider = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
    let transform_a = world.body_transform(slider).unwrap();
    let transform_b = world.body_transform(parent).unwrap();
    let id = world.create_prismatic_constraint(
        &PrismaticCinfo::between_bodies(
            slider,
            parent,
            Vector4::ZERO,
            &transform_a,
            &transform_b,
            Vector4::UNIT_X,
        )
        .with_limits(-5.0, 5.0),
    );
    assert!(id.is_valid());
    assert!(world.set_constraint_collisions_enabled(id, false));
    world.set_body_linear_velocity(slider, Vector4::from_xyz(2.0, 1.0, 0.0));
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let position = world.body_position(slider).unwrap();
    assert!(position.x > 1.0, "slider x {}", position.x);
    assert!(
        position.y.abs() < 0.15 && position.z.abs() < 0.15,
        "{position:?}"
    );
}

#[test]
fn spring_constraint_holds_the_distance() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let anchor = world
        .create_body(&BodyCinfo::new(sphere(0.5)).with_position(Vec4::from_xyz(0.0, 2.0, 0.0)));
    let bob = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
    world.create_spring_constraint(&SpringCinfo::new(
        bob,
        anchor,
        Vector4::ZERO,
        Vector4::ZERO,
        1.0,
    ));
    for _ in 0..240 {
        world.step(1.0 / 60.0);
    }
    let distance = (world.body_position(bob).unwrap() - Vector4::from_xyz(0.0, 2.0, 0.0)).length3();
    assert!((distance - 1.0).abs() < 0.35, "spring distance {distance}");
}

#[test]
fn point_to_plane_keeps_the_pivot_on_the_plane() {
    let mut world = World::new(WorldCinfo::new());
    let plane_body = world.create_body(&BodyCinfo::new(sphere(0.5)));
    let body = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(0.0, 1.0, 0.0)));
    let id = world.create_point_to_plane_constraint(&PointToPlaneCinfo::new(
        body,
        plane_body,
        Vector4::from_xyz(0.0, -1.0, 0.0),
        Vector4::ZERO,
        Vector4::UNIT_Y,
    ));
    assert!(id.is_valid());
    assert!(world.set_constraint_collisions_enabled(id, false));
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let y = world.body_position(body).unwrap().y;
    assert!((y - 1.0).abs() < 0.25, "pivoted body y {y}");
}

#[test]
fn ragdoll_cone_limit_holds_the_swing() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let parent = world.create_body(&BodyCinfo::new(sphere(0.5)));
    let child = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
    let motion = world.body(child).unwrap().motion_id;
    world.motion_mut(motion).unwrap().inverse_inertia = Vec4::splat(0.5);
    let id = world.create_ragdoll_constraint(
        &RagdollCinfo::new(
            child,
            parent,
            Vector4::ZERO,
            Vector4::ZERO,
            Vector4::UNIT_X,
            Vector4::UNIT_X,
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
        )
        .with_limits(0.2, -0.1, 0.1),
    );
    assert!(id.is_valid());
    assert!(world.set_constraint_collisions_enabled(id, false));
    world.set_body_transform(
        child,
        Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.5),
            Vector4::ZERO,
        ),
    );
    for _ in 0..180 {
        world.step(1.0 / 60.0);
    }
    let twist = world
        .body_transform(child)
        .unwrap()
        .rotation
        .rotate(Vector4::UNIT_X);
    let angle = twist.dot3(Vector4::UNIT_X).clamp(-1.0, 1.0).acos();
    assert!(angle < 0.35, "ragdoll swing {angle}");
}

#[test]
fn joint_scene_is_bitwise_deterministic() {
    let build = || {
        let mut world = World::new(WorldCinfo::new().with_solver_settings(0.2, 0.0, 16));
        let anchor = world.create_body(
            &BodyCinfo::new(sphere(0.5)).with_position(Vec4::from_xyz(0.0, 6.0, 0.0)),
        );
        let link = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(0.0, 4.0, 0.0)));
        let motion = world.body(link).unwrap().motion_id;
        world.motion_mut(motion).unwrap().inverse_inertia = Vec4::splat(0.5);
        world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            link,
            anchor,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        let arm = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(2.0, 4.0, 0.0)));
        world.create_hinge_constraint(&HingeCinfo::new(
            arm,
            link,
            Vector4::from_xyz(-2.0, 0.0, 0.0),
            Vector4::ZERO,
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
        ));
        let slider = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(-3.0, 0.0, 0.0)));
        let anchor2 = world.create_body(
            &BodyCinfo::new(sphere(0.5)).with_position(Vec4::from_xyz(-3.0, 2.0, 0.0)),
        );
        world.create_prismatic_constraint(&PrismaticCinfo::new(
            slider,
            anchor2,
            Vector4::ZERO,
            Vector4::ZERO,
            Vector4::UNIT_X,
            Vector4::UNIT_X,
        ));
        world
    };
    let mut first = build();
    let mut second = build();
    for _ in 0..180 {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
    }
    assert_eq!(first.snapshot(), second.snapshot());
}

#[test]
fn joint_collision_suppression_removes_contacts() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(&dynamic_sphere(1.0, Vec4::ZERO));
    let b = world.create_body(&dynamic_sphere(1.0, Vec4::from_xyz(1.5, 0.0, 0.0)));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
    ));
    assert!(!world.collect_contacts().is_empty());
    assert!(world.set_constraint_collisions_enabled(id, false));
    assert!(world.collect_contacts().is_empty());
    let attached: Vec<_> = world.constraints_of_body(a).map(|(id, _)| id).collect();
    assert_eq!(attached, vec![id]);
}

#[test]
fn invalid_joint_limits_are_rejected_for_wrong_types() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(&dynamic_sphere(0.5, Vec4::ZERO));
    let b = world.create_body(&dynamic_sphere(0.5, Vec4::from_xyz(1.0, 0.0, 0.0)));
    let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
    ));
    assert!(!world.set_ragdoll_limits(id, 0.3, -0.1, 0.1));
    assert!(!world.set_prismatic_limits(id, -1.0, 1.0));
    assert!(!world.set_hinge_limits(id, None));
}
