//! Compressed mass properties and deterministic high-quality aggregation.

use std::sync::Arc;

use proton::math::{Matrix3, Quaternion, Vector4};
use proton::shape::{
    CompoundShape, CompressedMassProperties, MassConfig, MassError, MassProperties, MassQuality,
    Shape, ShapeInstance, SphereShape,
};

fn sample_properties() -> MassProperties {
    MassProperties::from_geometry(
        8.0,
        Vector4::from_xyz(0.5, -0.25, 0.125),
        Matrix3::from_cols(
            Vector4::from_xyz(8.0, 0.5, 0.0),
            Vector4::from_xyz(0.5, 4.0, 0.0),
            Vector4::from_xyz(0.0, 0.0, 2.0),
        ),
        Quaternion::IDENTITY,
        MassConfig::from_density(1.5),
    )
    .unwrap()
}

#[test]
fn c1_compressed_round_trip_is_bounded_and_deterministic() {
    let original = sample_properties();
    let packed = CompressedMassProperties::pack(&original).unwrap();
    assert_eq!(packed, CompressedMassProperties::pack(&original).unwrap());
    assert!(packed.max_error(&original) < 0.01);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.mass, original.mass);
    assert_eq!(unpacked.volume, original.volume);
}

#[test]
fn c2_invalid_geometry_is_rejected() {
    let mut invalid = sample_properties();
    invalid.center_of_mass = Vector4::from_xyz(f32::NAN, 0.0, 0.0);
    assert_eq!(
        CompressedMassProperties::pack(&invalid),
        Err(MassError::InvalidGeometry)
    );
}

#[test]
fn c3_high_quality_compound_is_deterministic() {
    let sphere = Arc::new(SphereShape::new(Vector4::ZERO, 0.25)) as Arc<dyn Shape>;
    let children = (0..6)
        .map(|index| {
            ShapeInstance::new(
                proton::math::Transform::from_quaternion(
                    Quaternion::IDENTITY,
                    Vector4::from_xyz(index as f32, 0.0, 0.0),
                ),
                Arc::clone(&sphere),
            )
        })
        .collect();
    let compound = CompoundShape::new(children);
    let config = MassConfig::from_density(1.0).with_quality(MassQuality::High);
    let first = compound.mass_properties(config).unwrap();
    let second = compound.mass_properties(config).unwrap();
    assert_eq!(first, second);
    assert!(first.mass > 0.0);
}
