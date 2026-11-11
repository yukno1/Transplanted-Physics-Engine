//! Phase 9 determinism and trace tests (matrix S9, S10, plus the partial
//! `NpWorldSnapshotTest` mapping).

use std::sync::Arc;

use havok::body::BodyCinfo;
use havok::math::{Quaternion, Vector4};
use havok::np_types::MotionType;
use havok::shape::{BoxShape, Shape, SphereShape};
use havok::world::{World, WorldCinfo, WorldSnapshot};

fn sphere(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
}

fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn build_scene() -> World {
    let mut world = World::new(WorldCinfo::new());
    world.create_body(
        &BodyCinfo::new(box_shape(Vector4::from_xyz(4.0, 0.5, 4.0)))
            .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
    world.create_body(
        &BodyCinfo::new(sphere(0.5))
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vector4::from_xyz(0.0, 2.0, 0.0)),
    );
    world.create_body(
        &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vector4::from_xyz(1.5, 3.0, 0.0))
            .with_orientation(Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3)),
    );
    world
}

fn assert_bitwise_equal(a: &WorldSnapshot, b: &WorldSnapshot) {
    assert_eq!(a.step_count, b.step_count);
    assert_eq!(a.bodies.len(), b.bodies.len());
    for (left, right) in a.bodies.iter().zip(&b.bodies) {
        assert_eq!(left.body, right.body);
        for (l, r) in [
            (left.position.x, right.position.x),
            (left.position.y, right.position.y),
            (left.position.z, right.position.z),
            (left.orientation.x, right.orientation.x),
            (left.orientation.y, right.orientation.y),
            (left.orientation.z, right.orientation.z),
            (left.orientation.w, right.orientation.w),
            (left.linear_velocity.x, right.linear_velocity.x),
            (left.linear_velocity.y, right.linear_velocity.y),
            (left.linear_velocity.z, right.linear_velocity.z),
            (left.angular_velocity.x, right.angular_velocity.x),
            (left.angular_velocity.y, right.angular_velocity.y),
            (left.angular_velocity.z, right.angular_velocity.z),
        ] {
            assert_eq!(
                l.to_bits(),
                r.to_bits(),
                "float mismatch at step {} body {:?}: {l} vs {r}",
                a.step_count,
                left.body
            );
        }
    }
}

#[test]
fn s9_identical_scenes_are_bitwise_identical() {
    let mut first = build_scene();
    let mut second = build_scene();
    for _ in 0..180 {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
        assert_eq!(
            first.last_manifold_count(),
            second.last_manifold_count(),
            "manifold count differs at step {}",
            first.step_count()
        );
        assert_eq!(
            first.last_contact_count(),
            second.last_contact_count(),
            "contact count differs at step {}",
            first.step_count()
        );
    }
    assert_bitwise_equal(&first.snapshot(), &second.snapshot());
}

#[test]
fn s10_snapshot_trace_text_is_stable() {
    let mut world = build_scene();
    for _ in 0..30 {
        world.step(1.0 / 60.0);
    }
    let snapshot = world.snapshot();
    let first = snapshot.to_text();
    let second = snapshot.to_text();
    assert_eq!(first, second, "trace rendering must be deterministic");
    assert!(first.starts_with("# hvk-trace v1\n"));
    assert!(first.contains(&format!("step {}", snapshot.step_count)));
    assert_eq!(first.lines().count(), snapshot.bodies.len() + 2);
    assert!(first.contains("body "));
}

#[test]
fn np_world_snapshot_test_partial() {
    // Partial mapping of NpWorldSnapshotTest: the port exposes deterministic
    // snapshots and text traces, but not full world copy/serialization.
    let world = build_scene();
    let snapshot = world.snapshot();
    assert_eq!(snapshot.bodies.len(), world.body_count());
    assert_eq!(snapshot.step_count, world.step_count());
    for (index, body_snapshot) in snapshot.bodies.iter().enumerate() {
        let body = world.body(body_snapshot.body).expect("body must resolve");
        assert_eq!(body_snapshot.position, body.transform.translation);
        if index == 0 {
            assert_eq!(body_snapshot.linear_velocity, Vector4::ZERO);
        }
    }
}
