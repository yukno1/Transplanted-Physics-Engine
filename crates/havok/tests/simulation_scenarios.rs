//! Phase 9 integration scenarios (README §4, matrix S1–S8).
//!
//! These tests exercise the full public facade (`havok`) rather than crate
//! internals: gravity, broad/narrow phase, contact solving and integration.

use std::sync::Arc;

use havok::body::BodyCinfo;
use havok::collide::{BodyAabb, BodyPair, BroadPhase, BroadPhaseConfig};
use havok::math::{Quaternion, Vector4};
use havok::np_types::{BodyId, MotionType};
use havok::shape::{BoxShape, CapsuleShape, CylinderShape, Shape, SphereShape};
use havok::world::{World, WorldCinfo};

fn sphere(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
}

fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn dynamic(cinfo: BodyCinfo) -> BodyCinfo {
    cinfo.with_motion_type(MotionType::Dynamic).with_mass(1.0)
}

fn ground(world: &mut World, half_extents: Vector4) -> BodyId {
    world.create_body(
        &BodyCinfo::new(box_shape(half_extents)).with_position(Vector4::from_xyz(
            0.0,
            -half_extents.y,
            0.0,
        )),
    )
}

fn run_steps(world: &mut World, steps: usize, delta_time: f32) {
    for _ in 0..steps {
        world.step(delta_time);
    }
}

#[test]
fn s1_free_fall_matches_analytic_integration() {
    let mut world = World::new(WorldCinfo::new());
    let body = world.create_body(
        &dynamic(BodyCinfo::new(sphere(0.5))).with_position(Vector4::from_xyz(0.0, 100.0, 0.0)),
    );

    let steps = 60u32;
    let dt = 1.0 / 60.0;
    run_steps(&mut world, steps as usize, dt);

    // Semi-implicit Euler: v_n = -g*dt*n, y_n = y0 - g*dt^2 * n(n+1)/2.
    let g = 9.81f32;
    let expected_velocity = -g * dt * steps as f32;
    let expected_y = 100.0 - g * dt * dt * (steps as f32 * (steps as f32 + 1.0) / 2.0);
    let velocity = world.body_linear_velocity(body).unwrap();
    let position = world.body_position(body).unwrap();
    assert!(
        (velocity.y - expected_velocity).abs() < 1e-4,
        "vy {} expected {}",
        velocity.y,
        expected_velocity
    );
    assert!(
        (position.y - expected_y).abs() < 1e-3,
        "y {} expected {}",
        position.y,
        expected_y
    );
    assert_eq!(world.last_contact_count(), 0);
}

#[test]
fn s2_sphere_rests_on_static_ground() {
    let mut world = World::new(WorldCinfo::new());
    let ground_id = ground(&mut world, Vector4::from_xyz(3.0, 0.5, 3.0));
    let sphere_id = world.create_body(
        &dynamic(BodyCinfo::new(sphere(0.5))).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)),
    );

    run_steps(&mut world, 240, 1.0 / 60.0);

    let position = world.body_position(sphere_id).unwrap();
    assert!(
        position.y > 0.35 && position.y < 0.75,
        "sphere y {}",
        position.y
    );
    assert!(
        world.body_linear_velocity(sphere_id).unwrap().y.abs() < 0.2,
        "sphere still moving"
    );
    assert!(world.last_contact_count() > 0);
    assert!(world.last_manifold_count() > 0);
    assert!(
        world
            .body_position(ground_id)
            .unwrap()
            .all_equal3(Vector4::from_xyz(0.0, -0.5, 0.0), 1e-6)
    );
}

#[test]
fn s3_three_boxes_stack_without_tunneling() {
    let mut world = World::new(WorldCinfo::new());
    let _ground_id = ground(&mut world, Vector4::from_xyz(4.0, 0.5, 4.0));
    let box_a = world.create_body(
        &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
            .with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
    );
    let box_b = world.create_body(
        &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
            .with_position(Vector4::from_xyz(0.0, 2.2, 0.0)),
    );
    let box_c = world.create_body(
        &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
            .with_position(Vector4::from_xyz(0.0, 3.4, 0.0)),
    );

    run_steps(&mut world, 360, 1.0 / 60.0);

    let a = world.body_position(box_a).unwrap().y;
    let b = world.body_position(box_b).unwrap().y;
    let c = world.body_position(box_c).unwrap().y;
    assert!(a > 0.35 && a < 0.9, "bottom box y {a}");
    assert!(b > a + 0.8, "middle box y {b}, bottom {a}");
    assert!(c > b + 0.8, "top box y {c}, middle {b}");
    assert!(c < 5.0, "stack drifted upward: top {c}");
    assert!(world.last_contact_count() > 0);
}

#[test]
fn s4_friction_slows_sliding_sphere() {
    let mut world = World::new(WorldCinfo::new());
    let _ground_id = ground(&mut world, Vector4::from_xyz(5.0, 0.5, 5.0));
    let sphere_id = world.create_body(
        &dynamic(BodyCinfo::new(sphere(0.5)))
            .with_position(Vector4::from_xyz(0.0, 0.5, 0.0))
            .with_linear_velocity(Vector4::from_xyz(2.0, 0.0, 0.0)),
    );

    run_steps(&mut world, 120, 1.0 / 60.0);

    let velocity = world.body_linear_velocity(sphere_id).unwrap();
    let position = world.body_position(sphere_id).unwrap();
    assert!(
        velocity.x < 1.9 && velocity.x >= 0.0,
        "vx after friction {}",
        velocity.x
    );
    assert!(
        position.x < 3.9,
        "frictionless distance would be ~4, got {}",
        position.x
    );
}

#[test]
fn s5_capsule_rests_on_ground() {
    let mut world = World::new(WorldCinfo::new());
    let _ground_id = ground(&mut world, Vector4::from_xyz(3.0, 0.5, 3.0));
    let capsule = Arc::new(CapsuleShape::new(
        Vector4::from_xyz(0.0, -0.5, 0.0),
        Vector4::from_xyz(0.0, 0.5, 0.0),
        0.5,
    )) as Arc<dyn Shape>;
    let capsule_id = world.create_body(
        &dynamic(BodyCinfo::new(capsule)).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)),
    );

    run_steps(&mut world, 240, 1.0 / 60.0);

    let y = world.body_position(capsule_id).unwrap().y;
    assert!(y > 0.6 && y < 1.5, "capsule y {y}");
    assert!(world.body_linear_velocity(capsule_id).unwrap().y.abs() < 0.3);
}

#[test]
fn s6_cylinder_rests_on_ground() {
    let mut world = World::new(WorldCinfo::new());
    let _ground_id = ground(&mut world, Vector4::from_xyz(3.0, 0.5, 3.0));
    let cylinder = Arc::new(CylinderShape::with_params(
        Vector4::from_xyz(0.0, -0.5, 0.0),
        Vector4::from_xyz(0.0, 0.5, 0.0),
        0.5,
        16,
        0.0,
    )) as Arc<dyn Shape>;
    let cylinder_id = world.create_body(
        &dynamic(BodyCinfo::new(cylinder)).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)),
    );

    run_steps(&mut world, 240, 1.0 / 60.0);

    let y = world.body_position(cylinder_id).unwrap().y;
    assert!(y > 0.4 && y < 1.0, "cylinder y {y}");
}

#[test]
fn s7_manifold_convention_is_consistent() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(&dynamic(BodyCinfo::new(sphere(1.0))).with_position(Vector4::ZERO));
    let b = world.create_body(
        &dynamic(BodyCinfo::new(sphere(1.0))).with_position(Vector4::from_xyz(1.5, 0.0, 0.0)),
    );
    let contacts = world.collect_contacts();
    assert_eq!(contacts.len(), 1);
    let contact = &contacts[0];
    assert_eq!(contact.body_a, a.min(b));
    assert_eq!(contact.body_b, a.max(b));
    let manifold = &contact.manifold;
    assert!((manifold.normal.length3() - 1.0).abs() < 1e-5);
    assert_eq!(manifold.num_points(), 1);
    let point = manifold.points[0];
    assert!(point.distance < 0.0, "overlap must be negative distance");
    let point_on_a = manifold.point_on_a(0).unwrap();
    assert!(point_on_a.all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-4));
}

#[test]
fn s8_broadphase_is_deterministic_and_canonical() {
    let bodies = vec![
        BodyAabb::new(
            BodyId::new(5),
            havok::math::Aabb::new(Vector4::ZERO, Vector4::splat(1.0)),
        ),
        BodyAabb::new(
            BodyId::new(3),
            havok::math::Aabb::new(Vector4::from_xyz(0.2, 0.2, 0.2), Vector4::splat(1.2)),
        ),
        BodyAabb::new(
            BodyId::new(9),
            havok::math::Aabb::new(Vector4::from_xyz(10.0, 0.0, 0.0), Vector4::splat(11.0)),
        ),
    ];
    let broadphase = BroadPhase::new(BroadPhaseConfig::default());
    let first = broadphase.find_pairs(&bodies);
    let second = broadphase.find_pairs(&bodies);
    assert_eq!(first, second);
    assert_eq!(first, vec![BodyPair::new(BodyId::new(3), BodyId::new(5))]);
}

#[test]
fn s9_transform_and_orientation_survive_step() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let body = world.create_body(
        &dynamic(BodyCinfo::new(sphere(1.0)))
            .with_position(Vector4::from_xyz(1.0, 2.0, 3.0))
            .with_orientation(Quaternion::from_axis_angle(
                Vector4::UNIT_Z,
                core::f32::consts::FRAC_PI_2,
            )),
    );
    world.step(1.0 / 60.0);
    let position = world.body_position(body).unwrap();
    assert!(position.all_equal3(Vector4::from_xyz(1.0, 2.0, 3.0), 1e-6));
    let body_ref = world.body(body).unwrap();
    let mapped = body_ref.transform.transform_point(Vector4::UNIT_X);
    // 90° about Z maps the local +X axis to +Y, then the body translation
    // (1, 2, 3) is applied: (1, 3, 3).
    assert!(
        mapped.all_equal3(Vector4::from_xyz(1.0, 3.0, 3.0), 1e-4),
        "mapped {mapped:?}"
    );
}

#[test]
fn s10_dont_collide_is_filtered() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(
        &dynamic(BodyCinfo::new(sphere(1.0)))
            .with_flags(havok::np_types::CollisionFlags::DONT_COLLIDE),
    );
    let b = world.create_body(
        &dynamic(BodyCinfo::new(sphere(1.0))).with_position(Vector4::from_xyz(1.5, 0.0, 0.0)),
    );
    assert!(world.collect_contacts().is_empty());
    world.step(1.0 / 60.0);
    assert_eq!(world.last_manifold_count(), 0);
    assert_ne!(a, b);
}
