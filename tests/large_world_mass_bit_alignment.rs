use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::hash_world;
use proton::math::{DVec4, Mat3Ext, Vec4, Vec4Ext, Vector4};
use proton::np_types::MotionType;
use proton::shape::{BoxShape, CompressedMassProperties, MassConfig, MassPropertiesTree, Shape};
use proton::world::{World, WorldCinfo};

fn simple_world() -> (World, proton::np_types::BodyId) {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::from_xyz(0.0, -9.81, 0.0));
    let shape = BoxShape::with_convex_radius(Vec4::from_xyz(0.5, 0.5, 0.5), 0.0);
    let body = world.create_body(
        &BodyCinfo::new(Arc::new(shape) as Arc<dyn Shape>)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vec4::from_xyz(0.0, 4.0, 0.0)),
    );
    (world, body)
}

#[test]
fn precise_position_and_shift_origin_keep_large_world_coordinates() {
    let (mut world, body) = simple_world();
    let target = DVec4::new(1_000_000.0, 2.0, -3.0, 0.0);
    assert!(world.set_body_precise_position(body, target));
    let precise = world.body_precise_transform(body).unwrap();
    assert!((precise.translation_d().x - target.x).abs() < 1e-3);

    assert!(world.shift_origin(DVec4::new(1_000_000.0, 0.0, 0.0, 0.0)));
    assert!((world.world_origin().x - 1_000_000.0).abs() < 1e-9);
    let local = world.body_position(body).unwrap();
    assert!(local.x.abs() < 1e-3, "local x should stay small: {local:?}");
    let after = world.body_precise_transform(body).unwrap();
    assert!((after.translation_d().x - target.x).abs() < 1e-3);
}

#[test]
fn zero_origin_shift_does_not_change_simulation_hash() {
    let (mut first, _) = simple_world();
    let (mut second, _) = simple_world();
    for _ in 0..30 {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
    }
    assert!(second.shift_origin(DVec4::ZERO));
    assert_eq!(hash_world(&first), hash_world(&second));
}

#[test]
fn sdk_mass_bytes_and_32_child_word_cache_are_deterministic() {
    let shape = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 1.0, 1.0), 0.0);
    let properties = shape
        .mass_properties(MassConfig::from_density(1.0))
        .unwrap();
    let packed = CompressedMassProperties::pack_sdk(&properties).unwrap();
    assert_eq!(
        CompressedMassProperties::from_bytes(packed.to_bytes()),
        packed
    );

    let mut first = MassPropertiesTree::default();
    let mut second = MassPropertiesTree::default();
    for _ in 0..40 {
        first.add(properties).unwrap();
        second.add(properties).unwrap();
    }
    assert_eq!(first.word_count(), 1);
    assert_eq!(first.word_bytes(), second.word_bytes());
    assert_eq!(first.finish().unwrap(), second.finish().unwrap());
}

#[test]
fn world_applies_mass_distribution_with_explicit_mass() {
    let (mut world, body) = simple_world();
    let distribution = world.body_mass_distribution(body).unwrap();
    assert!(world.set_body_mass_distribution(body, distribution, 4.0));
    let properties = world.body_mass_properties(body).unwrap();
    assert!((properties.mass - 4.0).abs() < 1e-6);
    assert!(properties.inertia_tensor.element_sdk(0, 0) > 0.0);
}
