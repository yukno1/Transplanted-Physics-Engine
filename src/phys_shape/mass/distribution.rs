//! Unit-mass distribution (`hknpMassDistribution`).

use std::sync::Arc;

use glam::{Mat3, Quat, Vec4};

use crate::base_math::{Mat3Ext, QuatExt, Transform, Vec4Ext};

use super::eigen::diagonalize_symmetric;
use super::{MassConfig, MassError, MassProperties, vector3_is_finite};
use crate::phys_shape::Shape;

/// Unit-mass shape distribution (`hknpMassDistribution` subset).
///
/// Inertia components are diagonalized and stored for a unit mass. A zero
/// component means infinite inertia about that principal axis, matching the
/// SDK convention. `center_of_mass_and_volume.w` stores volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassDistribution {
    /// Centre of mass in body-local space with volume in `w`.
    pub center_of_mass_and_volume: Vec4,
    /// Rotation from principal-axis space to body-local space.
    pub major_axis_space: Quat,
    /// Diagonal inertia for a unit mass. Zero means infinite inertia.
    pub unit_inertia: Vec4,
}

impl MassDistribution {
    /// A distribution with zero mass/volume and infinite inertia.
    pub const fn zero() -> Self {
        Self {
            center_of_mass_and_volume: Vec4::ZERO,
            major_axis_space: Quat::IDENTITY,
            unit_inertia: Vec4::ZERO,
        }
    }

    /// Build from full mass properties with unit-mass normalization.
    pub fn from_mass_properties(properties: &MassProperties) -> Result<Self, MassError> {
        if !properties.mass.is_finite()
            || properties.mass < 0.0
            || !properties.volume.is_finite()
            || properties.volume < 0.0
            || !vector3_is_finite(properties.center_of_mass)
        {
            return Err(MassError::InvalidGeometry);
        }
        let diagonalized = diagonalize_symmetric(properties.inertia_tensor);
        let unit_inertia = if properties.mass > 0.0 {
            diagonalized.diagonal / properties.mass
        } else {
            diagonalized.diagonal
        };
        Ok(Self {
            center_of_mass_and_volume: Vec4::new(
                properties.center_of_mass.x,
                properties.center_of_mass.y,
                properties.center_of_mass.z,
                properties.volume,
            ),
            major_axis_space: diagonalized.orientation,
            unit_inertia,
        })
    }

    /// Build from one shape and a mass configuration (`setFromShape`).
    pub fn from_shape(shape: &Arc<dyn Shape>, config: MassConfig) -> Result<Self, MassError> {
        let properties = shape.mass_properties(config)?;
        Self::from_mass_properties(&properties)
    }

    /// Build from transformed shapes (`setFromShapes`).
    pub fn from_shapes(
        shapes: &[(Transform, Arc<dyn Shape>)],
        config: MassConfig,
    ) -> Result<Self, MassError> {
        let properties: Vec<MassProperties> = shapes
            .iter()
            .map(|(transform, shape)| {
                shape
                    .mass_properties(config)
                    .map(|p| p.transformed(transform))
            })
            .collect::<Result<_, _>>()?;
        let combined = MassProperties::combine_all(properties);
        if combined.mass <= 0.0 {
            return Err(MassError::InvalidGeometry);
        }
        Self::from_mass_properties(&combined)
    }

    /// Materialize mass properties for a requested mass (`getMassProperties`).
    pub fn to_mass_properties(self, mass: f32) -> Result<MassProperties, MassError> {
        if !mass.is_finite() || mass < 0.0 {
            return Err(MassError::InvalidConfig);
        }
        let rotation = self.major_axis_space.to_matrix_sdk();
        let diagonal_value = self.unit_inertia * mass;
        let diagonal = Mat3::from_diagonal(glam::Vec3::new(
            diagonal_value.x,
            diagonal_value.y,
            diagonal_value.z,
        ));
        let inertia_tensor = rotation
            .mul_mat_sdk(diagonal)
            .mul_mat_sdk(rotation.transpose());
        Ok(MassProperties {
            volume: self.center_of_mass_and_volume.w,
            mass,
            center_of_mass: Vec4::from_xyz(
                self.center_of_mass_and_volume.x,
                self.center_of_mass_and_volume.y,
                self.center_of_mass_and_volume.z,
            ),
            inertia_tensor,
            major_axis_space: self.major_axis_space,
        })
    }

    /// Apply the distribution with a configured mass (`hknpBodyManager` path).
    ///
    /// `configured_mass >= 0` overrides the unit-mass distribution directly.
    /// A negative value derives the mass from `shape_fallback`, matching the SDK
    /// rule where `m_mass < 0` means "use shape mass". The resulting inertia is
    /// scaled by `mass / 1.0` through [`MassProperties::with_mass`].
    pub fn apply_with_mass(
        self,
        configured_mass: f32,
        shape_fallback: Option<MassProperties>,
    ) -> Result<MassProperties, MassError> {
        let unit = self.to_mass_properties(1.0)?;
        let mass = if configured_mass >= 0.0 {
            configured_mass
        } else {
            let fallback = shape_fallback.ok_or(MassError::InvalidConfig)?;
            if fallback.mass < 0.0 {
                return Err(MassError::InvalidGeometry);
            }
            fallback.mass
        };
        unit.with_mass(mass)
    }

    /// Whether all stored components are finite.
    pub fn is_valid(self) -> bool {
        vector3_is_finite(self.center_of_mass_and_volume)
            && self.center_of_mass_and_volume.w.is_finite()
            && vector3_is_finite(self.unit_inertia)
            && self.major_axis_space.length_squared().is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::{Mat3Ext, Quaternion, Vector4};
    use crate::phys_shape::BoxShape;

    #[test]
    fn unit_distribution_scales_linearly_with_mass() {
        let box_shape = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 2.0, 3.0), 0.0);
        let properties = box_shape
            .mass_properties(MassConfig::from_density(1.0))
            .unwrap();
        let distribution = MassDistribution::from_mass_properties(&properties).unwrap();
        assert!(distribution.is_valid());
        for mass in [1.0, 2.0, 5.0] {
            let rebuilt = distribution.to_mass_properties(mass).unwrap();
            assert!((rebuilt.mass - mass).abs() < 1e-6);
            for row in 0..3 {
                for column in 0..3 {
                    let expected = properties.inertia_tensor.element_sdk(row, column)
                        / properties.mass
                        * mass;
                    assert!(
                        (rebuilt.inertia_tensor.element_sdk(row, column) - expected).abs()
                            < 1e-3 * expected.abs().max(1.0),
                        "mass {mass}, row {row}, column {column}"
                    );
                }
            }
        }
    }

    #[test]
    fn from_shapes_uses_transforms_and_insertion_order() {
        let shape = Arc::new(BoxShape::with_convex_radius(
            Vec4::from_xyz(0.5, 0.5, 0.5),
            0.0,
        )) as Arc<dyn Shape>;
        let transforms = [
            (
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
                Arc::clone(&shape),
            ),
            (
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(-1.0, 0.0, 0.0)),
                Arc::clone(&shape),
            ),
        ];
        let distribution =
            MassDistribution::from_shapes(&transforms, MassConfig::from_density(1.0)).unwrap();
        let center = distribution.center_of_mass_and_volume;
        assert!(center.x.abs() < 1e-6 && center.y.abs() < 1e-6 && center.z.abs() < 1e-6);
        assert!((distribution.center_of_mass_and_volume.w - 2.0).abs() < 1e-5);
        assert_eq!(
            distribution,
            MassDistribution::from_shapes(&transforms, MassConfig::from_density(1.0)).unwrap()
        );
    }

    #[test]
    fn zero_inertia_means_infinite_axis() {
        let distribution = MassDistribution {
            center_of_mass_and_volume: Vec4::new(0.0, 0.0, 0.0, 1.0),
            major_axis_space: Quat::IDENTITY,
            unit_inertia: Vec4::from_xyz(0.0, 1.0, 2.0),
        };
        let properties = distribution.to_mass_properties(3.0).unwrap();
        assert_eq!(properties.inertia_tensor.element_sdk(0, 0), 0.0);
        assert!((properties.inertia_tensor.element_sdk(1, 1) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn apply_chain_handles_explicit_and_shape_derived_mass() {
        let shape = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 1.0, 1.0), 0.0);
        let shape_properties = shape
            .mass_properties(MassConfig::from_density(2.0))
            .unwrap();
        let distribution = MassDistribution::from_mass_properties(&shape_properties).unwrap();

        let explicit = distribution.apply_with_mass(4.0, None).unwrap();
        assert!((explicit.mass - 4.0).abs() < 1e-6);
        let scale = explicit.mass / shape_properties.mass;
        assert!(
            (explicit.inertia_tensor.element_sdk(0, 0)
                - shape_properties.inertia_tensor.element_sdk(0, 0) * scale)
                .abs()
                < 1e-3
        );

        let derived = distribution
            .apply_with_mass(-1.0, Some(shape_properties))
            .unwrap();
        assert!((derived.mass - shape_properties.mass).abs() < 1e-6);
        assert_eq!(
            derived,
            distribution
                .apply_with_mass(shape_properties.mass, None)
                .unwrap()
        );
    }
}
