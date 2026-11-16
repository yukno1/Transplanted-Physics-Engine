//! Phase 12 integration tests: constraint islands and contact warm starting.
//!
//! These exercise the public facade (`proton`) end to end: island partitioning
//! from real contacts/joints and the effect of reusing contact impulses across
//! steps.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::Vector4;
use proton::np_types::MotionType;
use proton::shape::{BoxShape, Shape, SphereShape};
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

fn ground(world: &mut World, half_extents: Vector4) -> proton::np_types::BodyId {
    world.create_body(
        &BodyCinfo::new(box_shape(half_extents)).with_position(Vector4::from_xyz(
            0.0,
            -half_extents.y,
            0.0,
        )),
    )
}

fn stack_world(warm_starting: bool) -> World {
    let mut world = World::new(WorldCinfo::new().with_warm_starting(warm_starting));
    ground(&mut world, Vector4::from_xyz(4.0, 0.5, 4.0));
    for index in 0..5 {
        world.create_body(
            &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
                .with_position(Vector4::from_xyz(0.0, 1.0 + index as f32 * 1.1, 0.0)),
        );
    }
    world
}

/// Total penetration depth over the current contact set (positive metres).
fn total_penetration(world: &World) -> f32 {
    world
        .collect_contacts()
        .iter()
        .flat_map(|contact| contact.manifold.points.iter())
        .map(|point| (-point.distance).max(0.0))
        .sum()
}

#[test]
fn two_stacks_on_one_ground_report_two_islands() {
    let mut world = World::new(WorldCinfo::new());
    ground(&mut world, Vector4::from_xyz(8.0, 0.5, 8.0));
    for x in [-3.0, 3.0] {
        for index in 0..2 {
            world.create_body(
                &dynamic(BodyCinfo::new(box_shape(Vector4::splat(0.5))))
                    .with_position(Vector4::from_xyz(x, 1.0 + index as f32 * 1.1, 0.0)),
            );
        }
    }

    for _ in 0..240 {
        world.step(1.0 / 60.0);
    }

    assert_eq!(world.last_island_count(), 2);
    assert_eq!(world.stats().last_island_count, 2);
    for island in world.islands() {
        assert_eq!(island.dynamic_bodies, 2);
        assert!(island.contacts >= 2, "contacts {}", island.contacts);
    }
}

#[test]
fn jointed_bodies_share_an_island_even_without_contacts() {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
    let a = world.create_body(&dynamic(BodyCinfo::new(sphere_shape(0.5))));
    let b = world.create_body(
        &dynamic(BodyCinfo::new(sphere_shape(0.5))).with_position(Vector4::from_xyz(2.0, 0.0, 0.0)),
    );
    let c = world.create_body(
        &dynamic(BodyCinfo::new(sphere_shape(0.5))).with_position(Vector4::from_xyz(8.0, 0.0, 0.0)),
    );

    let id = world.create_spring_constraint(&proton::world::SpringCinfo::new(
        a,
        b,
        Vector4::ZERO,
        Vector4::ZERO,
        2.0,
    ));
    assert!(id.is_valid());
    world.step(1.0 / 60.0);

    assert_eq!(world.last_island_count(), 1);
    let island = &world.islands()[0];
    assert_eq!(island.dynamic_bodies, 2);
    assert_eq!(island.joints, 1);
    assert_eq!(island.contacts, 0);
    // The unconstrained body is not part of any island.
    assert!(!island.bodies.contains(&c));
}

#[test]
fn warm_starting_reduces_stack_penetration() {
    let mut warm = stack_world(true);
    let mut cold = stack_world(false);
    for _ in 0..480 {
        warm.step(1.0 / 60.0);
        cold.step(1.0 / 60.0);
    }

    let warm_penetration = total_penetration(&warm);
    let cold_penetration = total_penetration(&cold);
    assert!(
        warm_penetration < cold_penetration,
        "warm starting must reduce residual penetration (warm {warm_penetration}, cold {cold_penetration})"
    );

    // Both stacks must stay upright and in order.
    let warm_y: Vec<f32> = warm
        .snapshot()
        .bodies
        .iter()
        .skip(1)
        .map(|body| body.position.y)
        .collect();
    assert!(
        warm_y.windows(2).all(|pair| pair[1] > pair[0] + 0.8),
        "warm stack order {warm_y:?}"
    );
}

#[test]
fn warm_starting_is_deterministic() {
    let mut first = stack_world(true);
    let mut second = stack_world(true);
    for _ in 0..240 {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
    }
    assert_eq!(first.snapshot().to_text(), second.snapshot().to_text());
    assert_eq!(first.last_island_count(), second.last_island_count());
}

#[test]
fn clearing_the_contact_cache_keeps_the_simulation_stable() {
    let mut world = stack_world(true);
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    world.clear_contact_cache();
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let top = world.snapshot().bodies.last().unwrap().position.y;
    assert!(top > 3.5, "top box fell after clearing the cache: {top}");
}
