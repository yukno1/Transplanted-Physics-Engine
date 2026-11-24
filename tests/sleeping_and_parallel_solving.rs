//! Phase 12 (后续子集) integration tests: island sleeping and per-island
//! parallel solving.
//!
//! These exercise the public facade (`proton`) end to end: quiet islands fall
//! asleep, wake on contact with active bodies, and parallel island solving
//! produces bitwise-identical results to the sequential path.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::Vector4;
use proton::np_types::{BodyId, MotionType};
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::thread::ThreadPool;
use proton::world::{World, WorldCinfo};

fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
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

fn resting_world(sleeping: bool) -> (World, BodyId) {
    let mut world = World::new(
        WorldCinfo::new()
            .with_sleeping(sleeping)
            .with_sleeping_settings(0.2, 0.1, 0.1),
    );
    ground(&mut world, Vector4::from_xyz(4.0, 0.5, 4.0));
    let ball = world.create_body(
        &dynamic(BodyCinfo::new(sphere_shape(0.5))).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)),
    );
    (world, ball)
}

#[test]
fn sleeping_world_matches_awake_world_until_sleep() {
    // Sleeping must not change results while everything is awake.
    let (mut sleeping, _) = resting_world(true);
    let (mut awake, _) = resting_world(false);
    for _ in 0..6 {
        sleeping.step(1.0 / 60.0);
        awake.step(1.0 / 60.0);
    }
    assert_eq!(sleeping.snapshot().to_text(), awake.snapshot().to_text());
}

#[test]
fn island_sleeps_and_stays_frozen_until_contact() {
    let (mut world, ball) = resting_world(true);
    for _ in 0..30 {
        world.step(1.0 / 60.0);
    }
    assert!(world.body_is_sleeping(ball).unwrap());
    assert_eq!(world.stats().sleeping_body_count, 1);
    let frozen = world.snapshot().bodies;
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }
    assert_eq!(world.snapshot().bodies, frozen);
    assert_eq!(world.collect_contacts().len(), 0);

    // An active body touching the sleeping island wakes it again.
    world.create_body(
        &dynamic(BodyCinfo::new(sphere_shape(0.5)))
            .with_position(Vector4::from_xyz(0.0, 2.5, 0.0))
            .with_linear_velocity(Vector4::from_xyz(0.0, -2.0, 0.0)),
    );
    let mut woke = false;
    for _ in 0..60 {
        world.step(1.0 / 60.0);
        if !world.body_is_sleeping(ball).unwrap() {
            woke = true;
            break;
        }
    }
    assert!(woke, "the sleeping island must wake on contact");
    assert!(world.last_contact_count() > 0);
}

#[test]
fn parallel_island_solving_matches_single_threaded() {
    let scene = || {
        let mut world = World::new(WorldCinfo::new());
        ground(&mut world, Vector4::from_xyz(6.0, 0.5, 6.0));
        // Two separate stacks (two islands) plus a freely flying body.
        for index in 0..3 {
            world.create_body(
                &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
                    .with_position(Vector4::from_xyz(-2.0, 1.0 + index as f32 * 1.1, 0.0)),
            );
            world.create_body(
                &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
                    .with_position(Vector4::from_xyz(2.0, 1.0 + index as f32 * 1.1, 0.0)),
            );
        }
        world.create_body(
            &dynamic(BodyCinfo::new(sphere_shape(0.5)))
                .with_position(Vector4::from_xyz(0.0, 5.0, 0.0))
                .with_linear_velocity(Vector4::from_xyz(1.5, 0.0, -1.0)),
        );
        world
    };

    let mut sequential = scene();
    let mut parallel = scene();
    // After ~0.5 s the two stacks rest on the ground: two islands, which puts
    // the parallel island solver to work.
    for _ in 0..30 {
        sequential.step(1.0 / 60.0);
        parallel.step_with_threads(1.0 / 60.0, 4);
    }
    assert!(sequential.last_island_count() >= 2);
    for _ in 0..150 {
        sequential.step(1.0 / 60.0);
        parallel.step_with_threads(1.0 / 60.0, 4);
    }
    assert_eq!(
        sequential.snapshot().to_text(),
        parallel.snapshot().to_text(),
        "per-island parallel solving must be bitwise identical"
    );
}

#[test]
fn parallel_solving_with_warm_starting_and_joints_is_bitwise_identical() {
    let scene = || {
        let mut world = World::new(WorldCinfo::new());
        ground(&mut world, Vector4::from_xyz(8.0, 0.5, 8.0));
        // Left stack with a ball-socket jointed pair; right stack plain.
        let a = world.create_body(
            &dynamic(BodyCinfo::new(sphere_shape(0.5)))
                .with_position(Vector4::from_xyz(-2.0, 0.5, 0.0)),
        );
        let b = world.create_body(
            &dynamic(BodyCinfo::new(sphere_shape(0.5)))
                .with_position(Vector4::from_xyz(-2.0, 1.6, 0.0)),
        );
        world.create_ball_socket_constraint(&proton::world::BallAndSocketCinfo::new(
            b,
            a,
            Vector4::ZERO,
            Vector4::from_xyz(0.0, 1.1, 0.0),
        ));
        for index in 0..3 {
            world.create_body(
                &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
                    .with_position(Vector4::from_xyz(2.0, 1.0 + index as f32 * 1.1, 0.0)),
            );
        }
        world
    };

    let mut sequential = scene();
    let mut parallel = scene();
    for _ in 0..180 {
        sequential.step(1.0 / 60.0);
        parallel.step_with_threads(1.0 / 60.0, 3);
    }
    assert_eq!(
        sequential.snapshot().to_text(),
        parallel.snapshot().to_text()
    );
}

fn many_bodies_scene() -> World {
    let mut world = World::new(WorldCinfo::new());
    ground(&mut world, Vector4::from_xyz(6.0, 0.5, 6.0));
    let sphere = Arc::new(SphereShape::new(Vector4::ZERO, 0.5)) as Arc<dyn Shape>;
    for index in 0..36 {
        let x = (index % 6) as f32 * 1.2 - 3.0;
        let z = (index / 6) as f32 * 1.2 - 3.0;
        world.create_body(
            &dynamic(BodyCinfo::new(Arc::clone(&sphere)))
                .with_position(Vector4::from_xyz(x, 4.0, z)),
        );
    }
    world
}

#[test]
fn explicit_persistent_pool_matches_single_threaded() {
    let mut sequential = many_bodies_scene();
    let mut pooled = many_bodies_scene();
    pooled.set_thread_pool(Arc::new(ThreadPool::with_name("hk-test-pool", 4)));
    assert_eq!(pooled.thread_pool().map(ThreadPool::num_threads), Some(4));

    for _ in 0..120 {
        sequential.step(1.0 / 60.0);
        pooled.step_with_threads(1.0 / 60.0, 4);
    }
    assert!(
        sequential.last_manifold_count() >= 32,
        "scene must exercise the parallel narrow-phase path"
    );
    assert_eq!(sequential.snapshot().to_text(), pooled.snapshot().to_text());
}

#[test]
fn thread_pool_is_lazy_and_grows_with_requested_threads() {
    let mut world = World::new(WorldCinfo::new());
    assert!(world.thread_pool().is_none());

    world.step(1.0 / 60.0);
    assert!(
        world.thread_pool().is_none(),
        "serial step must stay pool-free"
    );

    world.step_with_threads(1.0 / 60.0, 2);
    assert_eq!(world.thread_pool().map(ThreadPool::num_threads), Some(2));

    world.step_with_threads(1.0 / 60.0, 4);
    assert_eq!(world.thread_pool().map(ThreadPool::num_threads), Some(4));

    world.set_thread_pool(Arc::new(ThreadPool::new(8)));
    world.step_with_threads(1.0 / 60.0, 2);
    assert_eq!(world.thread_pool().map(ThreadPool::num_threads), Some(8));

    world.clear_thread_pool();
    assert!(world.thread_pool().is_none());
}
