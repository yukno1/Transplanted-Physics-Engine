//! Shared-motion attachment integration scenarios.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::{Vector4, Vec4};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{World, WorldCinfo};

fn body(world: &mut World, position: Vector4, motion_type: MotionType) -> BodyId {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.25)) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(motion_type)
            .with_mass(1.0)
            .with_position(position),
    )
}

fn attached_cinfo() -> BodyCinfo {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.25)) as Arc<dyn Shape>;
    BodyCinfo::new(shape)
        .with_motion_type(MotionType::Keyframed)
        .with_mass(1.0)
}

#[test]
fn m1_attached_body_reuses_target_motion() {
    let mut world = World::new(WorldCinfo::new());
    let target = body(&mut world, Vector4::ZERO, MotionType::Keyframed);
    let target_motion = world.body_motion_id(target).unwrap();
    let attached = world
        .create_attached_body(target, &attached_cinfo())
        .expect("attached body");
    assert_eq!(world.body_motion_id(attached), Some(target_motion));
    assert_eq!(
        world.bodies_sharing_motion(target_motion),
        &[target, attached]
    );
    assert!(world.body(attached).unwrap().is_keyframed());
}

#[test]
fn m2_last_member_keeps_motion_slot() {
    let mut world = World::new(WorldCinfo::new());
    let target = body(&mut world, Vector4::ZERO, MotionType::Keyframed);
    let motion = world.body_motion_id(target).unwrap();
    let first = world
        .create_attached_body(target, &attached_cinfo())
        .unwrap();
    let second = world
        .create_attached_body(target, &attached_cinfo())
        .unwrap();
    assert_eq!(world.bodies_sharing_motion(motion).len(), 3);

    assert!(world.destroy_body(first));
    assert_eq!(world.bodies_sharing_motion(motion).len(), 2);
    assert!(world.destroy_body(target));
    assert!(world.bodies_sharing_motion(motion).is_empty());
    assert_eq!(world.body_motion_id(second), Some(motion));
    assert!(world.motion(motion).is_some());
}

#[test]
fn m3_detach_allocates_fresh_motion_and_keeps_velocity() {
    let mut world = World::new(WorldCinfo::new());
    let target = body(&mut world, Vector4::ZERO, MotionType::Keyframed);
    let motion = world.body_motion_id(target).unwrap();
    let child = body(
        &mut world,
        Vector4::from_xyz(0.0, 1.0, 0.0),
        MotionType::Keyframed,
    );
    assert!(world.attach_bodies(target, &[child]));
    assert_eq!(world.body_motion_id(child), Some(motion));

    let velocity = Vector4::from_xyz(2.0, 0.0, 0.0);
    assert!(world.set_body_linear_velocity(target, velocity));
    assert!(world.detach_bodies(&[child]));
    let detached = world.body_motion_id(child).unwrap();
    assert_ne!(detached, motion);
    assert_eq!(world.body_linear_velocity(child), Some(velocity));
    assert!(world.bodies_sharing_motion(motion).is_empty());
}

#[test]
fn m4_shared_transform_updates_every_member() {
    let mut world = World::new(WorldCinfo::new());
    let target = body(&mut world, Vector4::ZERO, MotionType::Keyframed);
    let child = world
        .create_attached_body(target, &attached_cinfo())
        .unwrap();
    let transform = proton::math::Transform::from_quaternion(
        proton::math::Quaternion::IDENTITY,
        Vector4::from_xyz(3.0, 0.0, 0.0),
    );
    assert!(world.set_body_transform(child, transform));
    assert_eq!(
        world.body_position(target),
        Some(Vector4::from_xyz(3.0, 0.0, 0.0))
    );
    assert_eq!(world.body_position(child), world.body_position(target));
}

#[test]
fn m5_shared_motion_scenario_is_bit_deterministic() {
    fn run() -> String {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vector4::ZERO);
        let target = body(&mut world, Vector4::ZERO, MotionType::Dynamic);
        let first = world
            .create_attached_body(target, &attached_cinfo())
            .unwrap();
        let second = body(
            &mut world,
            Vector4::from_xyz(0.0, 2.0, 0.0),
            MotionType::Dynamic,
        );
        assert!(world.attach_bodies(target, &[second]));
        world.set_body_linear_velocity(target, Vector4::from_xyz(1.0, 0.0, 0.0));
        world.set_body_transform(
            first,
            proton::math::Transform::from_quaternion(
                proton::math::Quaternion::IDENTITY,
                Vector4::from_xyz(0.0, 1.0, 0.0),
            ),
        );
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    }
    assert_eq!(run(), run());
}
