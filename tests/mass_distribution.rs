//! Unit-mass distribution integration scenarios.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::{Quaternion, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::MotionType;
use proton::shape::{BoxShape, MassConfig, MassDistribution, Shape};
use proton::world::{World, WorldCinfo};

#[test]
fn d1_world_exposes_body_mass_distribution() {
    let mut world = World::new(WorldCinfo::new());
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(1.0, 1.0, 1.0),
        0.0,
    )) as Arc<dyn Shape>;
    let body = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass_config(MassConfig::from_density(2.0)),
    );
    let distribution = world.body_mass_distribution(body).expect("distribution");
    assert!(distribution.is_valid());
    assert!((distribution.center_of_mass_and_volume.w - 8.0).abs() < 1e-5);

    let properties = distribution.to_mass_properties(2.0).unwrap();
    assert_eq!(properties.mass, 2.0);
}

#[test]
fn d2_distribution_from_shapes_matches_world_helper() {
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(0.5, 0.5, 0.5),
        0.0,
    )) as Arc<dyn Shape>;
    let shapes = [
        (
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
            Arc::clone(&shape),
        ),
        (
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(-1.0, 0.0, 0.0)),
            Arc::clone(&shape),
        ),
    ];
    let config = MassConfig::from_density(1.0);
    let direct = MassDistribution::from_shapes(&shapes, config).unwrap();
    let world = World::new(WorldCinfo::new());
    assert_eq!(
        world
            .mass_distribution_from_shapes(&shapes, config)
            .unwrap(),
        direct
    );
}
