//! Shape mass properties and full inertia tensor integration scenarios.

use std::f32::consts::FRAC_PI_4;
use std::sync::Arc;

use proton::body::BodyCinfo;
use glam::Vec3;
use proton::math::{Mat3, Mat3Ext, Matrix3, Quaternion, Transform, Vec4, Vec4Ext, Vector4};
use proton::np_types::MotionType;
use proton::shape::mass::{MassConfig, MassError};
use proton::shape::{BoxShape, CompoundShape, Shape, ShapeInstance, ShapeMassCache, SphereShape};
use proton::world::{World, WorldCinfo};

#[test]
fn analytic_primitives_resolve_density_and_explicit_mass() {
    let sphere = SphereShape::new(Vec4::ZERO, 1.0);
    let properties = sphere
        .mass_properties(MassConfig::from_density(2.0))
        .unwrap();
    assert!((properties.mass - 8.0 / 3.0 * std::f32::consts::PI).abs() < 1e-5);
    assert_eq!(properties.center_of_mass, Vec4::ZERO);
    assert!(
        (properties.inertia_tensor.element_sdk(0, 0) - 0.8 * properties.volume).abs() < 1e-5
    );

    let box_shape = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 2.0, 3.0), 0.0);
    let mut explicit = box_shape
        .mass_properties(MassConfig::from_density(1.0))
        .unwrap();
    assert_eq!(explicit.mass, 48.0);
    assert!((explicit.inertia_tensor.element_sdk(0, 0) - 208.0).abs() < 1e-4);
    explicit = explicit.with_mass(96.0).unwrap();
    assert_eq!(explicit.mass, 96.0);
    assert!((explicit.inertia_tensor.element_sdk(0, 0) - 416.0).abs() < 1e-4);
}

#[test]
fn compound_aggregation_preserves_off_diagonal_terms() {
    let box_shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(1.0, 2.0, 3.0),
        0.0,
    )) as Arc<dyn Shape>;
    let rotation = Transform::from_quaternion(
        Quaternion::from_axis_angle(Vector4::UNIT_Y, FRAC_PI_4),
        Vector4::ZERO,
    );
    let compound = CompoundShape::new(vec![ShapeInstance::new(rotation, box_shape)]);
    let properties = compound
        .mass_properties(MassConfig::from_mass(48.0))
        .unwrap();
    assert_eq!(properties.mass, 48.0);
    assert!(
        properties.inertia_tensor.element_sdk(0, 2).abs() > 1.0,
        "rotated box should produce a non-diagonal inertia tensor: {:?}",
        properties.inertia_tensor
    );
}

#[test]
fn invalid_mass_config_is_rejected() {
    let sphere = SphereShape::new(Vec4::ZERO, 1.0);
    assert_eq!(
        sphere.mass_properties(MassConfig::from_density(f32::NAN)),
        Err(MassError::InvalidConfig)
    );
}

#[test]
fn negative_mass_bodies_derive_shape_inertia() {
    let mut world = World::new(WorldCinfo::new());
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(1.0, 2.0, 3.0),
        0.0,
    )) as Arc<dyn Shape>;
    let body_id = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_position(Vec4::from_xyz(0.0, 5.0, 0.0)),
    );
    let body = world.body(body_id).unwrap();
    assert!((body.mass - 48.0).abs() < 1e-4);
    assert!(body.mass_properties.is_some());

    let motion = world
        .motion(world.body_motion_id(body_id).unwrap())
        .unwrap();
    assert!(motion.inverse_inertia_world.element_sdk(0, 0) > 0.0);
    assert!(motion.inverse_inertia.x > 0.0);

    let matrix = Matrix3::from_cols(
        Vector4::from_xyz(0.3, 0.0, 0.1),
        Vector4::from_xyz(0.0, 0.2, 0.0),
        Vector4::from_xyz(0.1, 0.0, 0.4),
    );
    assert!(world.set_body_inverse_inertia_world(body_id, matrix));
    let motion = world
        .motion(world.body_motion_id(body_id).unwrap())
        .unwrap();
    assert_eq!(
        motion.inverse_inertia_world,
        Mat3::from_cols(
            Vec3::new(0.3, 0.0, 0.1),
            Vec3::new(0.0, 0.2, 0.0),
            Vec3::new(0.1, 0.0, 0.4),
        )
    );
    assert_eq!(motion.inverse_inertia, Vec4::from_xyz(0.3, 0.2, 0.4));
}

#[test]
fn shape_derived_mass_trace_is_deterministic() {
    fn trace() -> String {
        let mut world = World::new(WorldCinfo::new());
        let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
        for index in 0..3 {
            world.create_body(
                &BodyCinfo::new(Arc::clone(&shape))
                    .with_motion_type(MotionType::Dynamic)
                    .with_position(Vec4::from_xyz(index as f32, 2.0 + index as f32, 0.0)),
            );
        }
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    }
    assert_eq!(trace(), trace());
}

#[test]
fn shape_mass_cache_reuses_and_invalidates_by_identity() {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
    let config = MassConfig::from_density(1.0);
    let mut cache = ShapeMassCache::new();
    let first = cache.compute(&shape, config).unwrap();
    assert_eq!(cache.get(&shape, config), Some(first));
    assert_eq!(cache.compute(&shape, config).unwrap(), first);
    cache.invalidate(&shape);
    assert!(cache.is_empty());
}
