//! Ray-casting character utility integration coverage.

use std::sync::Arc;

use base_math::Vector4 as V4;
use phys_body::BodyCinfo;
use phys_extensions::{RayCastingCharacterCinfo, get_character_params};
use phys_shape::{BoxShape, MeshShape, Shape, SphereShape};
use phys_types::MotionType;
use phys_world::{World, WorldCinfo};

const DT: f32 = 1.0 / 60.0;

fn box_shape(half: V4) -> Arc<dyn Shape> {
    Arc::new(BoxShape::with_convex_radius(half, 0.0)) as Arc<dyn Shape>
}

fn scene() -> (World, phys_types::BodyId) {
    let mut world = World::new(WorldCinfo::new());
    world.create_body(
        &BodyCinfo::new(box_shape(V4::from_xyz(10.0, 0.5, 10.0)))
            .with_position(V4::from_xyz(0.0, -0.5, 0.0)),
    );
    let body = world.create_body(
        &BodyCinfo::new(Arc::new(SphereShape::new(V4::ZERO, 0.5)) as Arc<dyn Shape>)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(75.0)
            .with_position(V4::from_xyz(0.0, 1.0, 0.0)),
    );
    (world, body)
}

fn info(bump: f32) -> RayCastingCharacterCinfo {
    RayCastingCharacterCinfo::new(V4::UNIT_X, 1.0, 1.0, 0.5, 1.0, bump)
}

#[test]
fn flat_ground_has_support_and_stable_parameters() {
    let (world, body) = scene();
    let first = get_character_params(&world, body, &info(0.4), V4::ZERO);
    let second = get_character_params(&world, body, &info(0.4), V4::ZERO);
    assert!(first.has_support);
    assert!(first.can_climb);
    assert_eq!(first, second);
    assert_eq!(first.bump_factor, 1.0);
    assert_eq!(first.easing_factor, first.velocity_easing_factor_down);
}

#[test]
fn step_ahead_increases_displacement_towards_the_step() {
    let (mut world, body) = scene();
    world.create_body(
        &BodyCinfo::new(box_shape(V4::from_xyz(0.5, 0.2, 0.5)))
            .with_position(V4::from_xyz(0.8, 0.1, 0.0)),
    );
    let towards = get_character_params(&world, body, &info(0.4), V4::UNIT_X);
    let away = get_character_params(&world, body, &info(0.4), -V4::UNIT_X);
    assert!(towards.has_support && away.has_support);
    assert!(
        towards.desired_displacement > away.desired_displacement,
        "casts facing the step should report a higher target: {towards:?} vs {away:?}"
    );
}

#[test]
fn horizontal_obstacle_applies_bump_factor() {
    let (mut world, body) = scene();
    assert_eq!(
        get_character_params(&world, body, &info(0.4), V4::UNIT_X).bump_factor,
        1.0
    );
    world.create_body(
        &BodyCinfo::new(box_shape(V4::from_xyz(0.1, 0.5, 0.5)))
            .with_position(V4::from_xyz(0.9, 0.5, 0.0)),
    );
    let blocked = get_character_params(&world, body, &info(0.4), V4::UNIT_X);
    assert_eq!(blocked.bump_factor, 0.4);
}

#[test]
fn steep_slope_is_reported_as_unclimbable() {
    let (mut world, body) = scene();
    // A mesh triangle keeps an exact face normal (convex support normals are
    // approximate), so it reliably exercises the max-slope counter.
    let slope = Arc::new(MeshShape::new(
        vec![
            V4::from_xyz(-2.0, 0.0, -2.0),
            V4::from_xyz(2.0, 0.0, -2.0),
            V4::from_xyz(0.0, 1.0, 2.0),
        ],
        vec![[0, 1, 2]],
    )) as Arc<dyn Shape>;
    world.create_body(&BodyCinfo::new(slope));
    let info = RayCastingCharacterCinfo::new(V4::UNIT_X, 1.0, 1.0, 1.0, 0.1, 0.4);
    let param = get_character_params(&world, body, &info, V4::UNIT_X);
    assert!(!param.can_climb, "steep cast result: {param:?}");
    let _ = DT;
}

#[test]
fn ray_casting_is_deterministic_across_identical_stepped_worlds() {
    let (mut first, first_body) = scene();
    let (mut second, second_body) = scene();
    for _ in 0..30 {
        first.step(DT);
        second.step(DT);
    }
    let first_param = get_character_params(&first, first_body, &info(0.4), V4::UNIT_X);
    let second_param = get_character_params(&second, second_body, &info(0.4), V4::UNIT_X);
    assert_eq!(first_param, second_param);
}
