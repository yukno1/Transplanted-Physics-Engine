//! Integration tests for the collision filter system: layer-based group
//! filtering, system-group/subsystem exclusion, body-pair disable lists and
//! query-side filter info. Mirrors the scenario style of
//! `simulation_scenarios.rs`.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::collide::{GroupCollisionFilter, PairCollisionFilter};
use proton::math::{Matrix3, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::MotionType;
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::world::{QueryFilter, World, WorldCinfo};

fn box_shape(half_extents: Vec4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
}

fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vec4::ZERO, radius)) as Arc<dyn Shape>
}

fn ground(layer: u32) -> BodyCinfo {
    BodyCinfo::new(box_shape(Vec4::from_xyz(10.0, 0.5, 10.0)))
        .with_position(Vec4::from_xyz(0.0, -0.5, 0.0))
        .with_collision_filter_info(GroupCollisionFilter::calc_filter_info(layer))
}

fn falling_ball(layer: u32, height: f32) -> BodyCinfo {
    BodyCinfo::new(sphere_shape(0.5))
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(Vec4::from_xyz(0.0, height, 0.0))
        .with_collision_filter_info(GroupCollisionFilter::calc_filter_info(layer))
}

fn dynamic_sphere(position: Vec4, info: u32) -> BodyCinfo {
    BodyCinfo::new(sphere_shape(0.5))
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(position)
        .with_collision_filter_info(info)
}

#[test]
fn group_filter_layer_disable_drops_contacts() {
    let mut filter = GroupCollisionFilter::new();
    filter.disable_collisions_between(1, 2);

    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(filter)));
    world.create_body(&ground(1));
    let ball = world.create_body(&falling_ball(2, 5.0));

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    // Layers 1/2 are disabled, so the ball falls straight through the ground.
    let position = world.body_position(ball).unwrap();
    assert!(
        position.y < -2.0,
        "ball should fall through, got y={}",
        position.y
    );
}

#[test]
fn group_filter_layer_enable_restores_collision() {
    let mut filter = GroupCollisionFilter::new();
    filter.disable_collisions_between(1, 2);
    filter.enable_collisions_between(1, 2);

    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(filter)));
    world.create_body(&ground(1));
    let ball = world.create_body(&falling_ball(2, 5.0));

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let position = world.body_position(ball).unwrap();
    // Ground top surface is y = 0, ball radius 0.5.
    assert!(
        position.y > 0.3,
        "ball should rest on the ground, got y={}",
        position.y
    );
}

#[test]
fn same_system_group_bypasses_layer_table() {
    // Layer 1/2 are disabled, but both bodies share system group 42 so the
    // layer table is bypassed (SDK `_isCollisionEnabled` semantics).
    let mut filter = GroupCollisionFilter::new();
    filter.disable_collisions_between(1, 2);

    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(filter)));
    world.create_body(&dynamic_sphere(
        Vec4::from_xyz(0.0, 5.0, 0.0),
        GroupCollisionFilter::calc_filter_info_full(1, 42, 1, 0),
    ));
    world.create_body(&dynamic_sphere(
        Vec4::from_xyz(0.6, 5.0, 0.0),
        GroupCollisionFilter::calc_filter_info_full(2, 42, 2, 0),
    ));

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let ids: Vec<_> = world.bodies().map(|body| body.id).collect();
    let pos_a = world.body_position(ids[0]).unwrap();
    let pos_b = world.body_position(ids[1]).unwrap();
    let distance = (pos_a - pos_b).length3();
    // Overlapping spheres in the same non-zero system group still generate
    // contacts and separate to (about) touching distance.
    assert!(
        distance > 0.8,
        "spheres should separate to touching distance, got {distance}"
    );
}

#[test]
fn system_group_subsystem_exclusion_blocks_pair() {
    // Same system group, but A's don't-collide subsystem id matches B's id.
    let filter = GroupCollisionFilter::new();
    let info_a = GroupCollisionFilter::calc_filter_info_full(0, 7, 2, 4);
    let info_b = GroupCollisionFilter::calc_filter_info_full(0, 7, 4, 0);

    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(filter)));
    world.create_body(&dynamic_sphere(Vec4::from_xyz(0.0, 5.0, 0.0), info_a));
    world.create_body(&dynamic_sphere(Vec4::from_xyz(0.6, 5.0, 0.0), info_b));

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let ids: Vec<_> = world.bodies().map(|body| body.id).collect();
    let pos_a = world.body_position(ids[0]).unwrap();
    let pos_b = world.body_position(ids[1]).unwrap();
    let distance = (pos_a - pos_b).length3();
    // The pair is filtered out, so the overlapping spheres are not pushed
    // apart while falling together.
    assert!(
        distance < 1.0,
        "spheres should stay interpenetrated, got {distance}"
    );
}

#[test]
fn pair_filter_disable_lets_ball_fall_through() {
    let mut world = World::new(WorldCinfo::new());
    let ground_body = world.create_body(&ground(0));
    let ball = world.create_body(&falling_ball(0, 5.0));

    let mut filter = PairCollisionFilter::new();
    filter.disable_collisions_between(ball, ground_body);
    world.set_collision_filter(Some(Arc::new(filter)));
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let position = world.body_position(ball).unwrap();
    assert!(
        position.y < -2.0,
        "disabled pair should fall through, got y={}",
        position.y
    );
}

#[test]
fn pair_filter_without_disable_collides() {
    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(PairCollisionFilter::new())));
    world.create_body(&ground(0));
    let ball = world.create_body(&falling_ball(0, 5.0));

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let position = world.body_position(ball).unwrap();
    assert!(
        position.y > 0.3,
        "empty disable list should not block contacts, got y={}",
        position.y
    );
}

#[test]
fn query_respects_world_filter_info() {
    let mut filter = GroupCollisionFilter::new();
    filter.disable_collisions_between(1, 2);

    let mut world = World::new(WorldCinfo::new());
    world.set_collision_filter(Some(Arc::new(filter)));
    world.create_body(&ground(1));

    // Default query filter info (layer 0) still sees the ground.
    let hit = world.raycast(
        Vec4::from_xyz(0.0, 2.0, 0.0),
        Vec4::from_xyz(0.0, -1.0, 0.0),
        10.0,
    );
    assert!(hit.is_some(), "layer-0 query should hit the ground");

    // A layer-2 query is disabled against layer 1.
    let filtered =
        QueryFilter::default().with_filter_info(GroupCollisionFilter::calc_filter_info(2));
    let hit = world.raycast_filtered(
        Vec4::from_xyz(0.0, 2.0, 0.0),
        Vec4::from_xyz(0.0, -1.0, 0.0),
        10.0,
        &filtered,
    );
    assert!(
        hit.is_none(),
        "layer-2 query must not hit the layer-1 ground"
    );

    // The shape cast follows the same decision path.
    let sphere = sphere_shape(0.2);
    let transform = Transform::new(Matrix3::IDENTITY, Vector4::from_xyz(0.0, 2.0, 0.0));
    let hit = world.cast_shape(
        sphere.as_ref(),
        &transform,
        Vec4::from_xyz(0.0, -1.0, 0.0),
        10.0,
        &filtered,
    );
    assert!(hit.is_none(), "layer-2 shape cast must not hit the ground");
}

#[test]
fn filtered_simulation_is_deterministic() {
    let build = || {
        let mut filter = GroupCollisionFilter::new();
        filter.disable_collisions_between(1, 2);
        filter.disable_collisions_between(2, 3);

        let mut world = World::new(WorldCinfo::new());
        world.set_collision_filter(Some(Arc::new(filter)));
        world.create_body(&ground(1));
        world.create_body(&falling_ball(2, 5.0));
        world.create_body(&falling_ball(3, 9.0));
        world
    };

    let mut world_a = build();
    let mut world_b = build();
    for _ in 0..180 {
        world_a.step(1.0 / 60.0);
        world_b.step(1.0 / 60.0);
    }
    assert_eq!(
        world_a.snapshot().to_text(),
        world_b.snapshot().to_text(),
        "two filtered runs must be bit-identical"
    );
}
