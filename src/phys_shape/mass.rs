//! Shape mass properties (`hknpShape::MassConfig` / `hkMassProperties`).
//!
//! The SDK separates geometry (volume, centre of mass, inertia tensor) from the
//! mass configuration used to turn unit-density geometry into actual mass data.
//! This module mirrors that split without introducing a linear-algebra crate:
//! [`proton::math::Matrix3`] carries the full inertia tensor and
//! [`MassConfig`] carries either an explicit mass or a negative density.
//!
//! All tensors are expressed about the shape's centre of mass in shape-local
//! space. A deterministic portable 15-bit compression scheme is available via
//! [`CompressedMassProperties`] and a fixed-schedule principal-axis solver via
//! [`diagonalize_symmetric`]; neither is claimed to be bit-identical to the
//! SDK's float-bit packing without a real `.hkx` golden comparison.

use crate::base_math::{Aabb, Matrix3, Quaternion, Transform, Vector4};

mod cache;
mod compressed;
mod distribution;
mod eigen;

pub use cache::{MassPropertiesTree, ShapeCacheKey, ShapeMassCache};
pub use compressed::{CompressedMassProperties, PackedUnitVector4, PackedVector3};
pub use distribution::MassDistribution;
pub use eigen::{DiagonalizedSymmetric, diagonalize_symmetric};

/// Quality hint used when a composite shape has no closed-form inertia
/// (`hknpShape::MassConfig::MassPropertiesComputationQuality`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MassQuality {
    /// Use AABB approximation for composite children.
    Low,
    /// Use per-child analytic properties when available, AABB otherwise.
    #[default]
    Medium,
    /// Use per-child analytic properties, including triangle/cell aggregation.
    High,
}

/// Configuration used to turn unit-density geometry into mass properties
/// (`hknpShape::MassConfig`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassConfig {
    /// Non-negative means explicit mass; negative means `-density` (SDK
    /// `m_massOrNegativeDensity` convention).
    pub mass_or_negative_density: f32,
    /// Composite mass computation quality.
    pub quality: MassQuality,
    /// Extra multiplier applied to the inertia tensor.
    pub inertia_scale: f32,
}

impl MassConfig {
    /// SDK default: unit density (`m_massOrNegativeDensity = -1`).
    pub const DEFAULT: Self = Self {
        mass_or_negative_density: -1.0,
        quality: MassQuality::Medium,
        inertia_scale: 1.0,
    };

    /// Build from unit density.
    #[inline]
    pub const fn from_density(density: f32) -> Self {
        Self {
            mass_or_negative_density: -density,
            quality: MassQuality::Medium,
            inertia_scale: 1.0,
        }
    }

    /// Build from an explicit (non-negative) mass.
    #[inline]
    pub const fn from_mass(mass: f32) -> Self {
        Self {
            mass_or_negative_density: mass,
            quality: MassQuality::Medium,
            inertia_scale: 1.0,
        }
    }

    /// Set the composite mass quality.
    #[inline]
    pub const fn with_quality(mut self, quality: MassQuality) -> Self {
        self.quality = quality;
        self
    }

    /// Set the inertia multiplier.
    #[inline]
    pub const fn with_inertia_scale(mut self, scale: f32) -> Self {
        self.inertia_scale = scale;
        self
    }

    /// `true` when this config contains an explicit mass.
    #[inline]
    pub const fn uses_explicit_mass(self) -> bool {
        self.mass_or_negative_density >= 0.0
    }

    /// Density implied by this config; explicit mass configs have no density.
    #[inline]
    pub const fn density(self) -> Option<f32> {
        if self.uses_explicit_mass() {
            None
        } else {
            Some(-self.mass_or_negative_density)
        }
    }

    /// Resolve mass from a unit-density volume (`calcMassFromVolume`).
    #[inline]
    pub fn calc_mass_from_volume(self, volume: f32) -> Result<f32, MassError> {
        if !self.mass_or_negative_density.is_finite() || !self.inertia_scale.is_finite() {
            return Err(MassError::InvalidConfig);
        }
        if self.uses_explicit_mass() {
            Ok(self.mass_or_negative_density)
        } else {
            Ok(-self.mass_or_negative_density * volume)
        }
    }

    fn validate(self) -> Result<(), MassError> {
        if !self.mass_or_negative_density.is_finite()
            || (!self.uses_explicit_mass() && self.mass_or_negative_density == 0.0)
            || !self.inertia_scale.is_finite()
            || self.inertia_scale < 0.0
        {
            Err(MassError::InvalidConfig)
        } else {
            Ok(())
        }
    }
}

impl Default for MassConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Errors produced while building or transforming mass properties.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MassError {
    /// `MassConfig` contains a NaN, infinity, negative density or scale.
    InvalidConfig,
    /// Geometry produced non-finite volume/COM/inertia.
    InvalidGeometry,
    /// A shape has no mass-property implementation at the requested quality.
    UnsupportedShape,
}

impl core::fmt::Display for MassError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidConfig => f.write_str("invalid mass config"),
            Self::InvalidGeometry => f.write_str("mass properties contain non-finite geometry"),
            Self::UnsupportedShape => f.write_str("shape has no mass-property implementation"),
        }
    }
}

impl std::error::Error for MassError {}

/// Full mass properties of a shape (`hkMassProperties` plus major-axis frame).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    /// Unit-density volume of the solid surface.
    pub volume: f32,
    /// Resolved mass.
    pub mass: f32,
    /// Centre of mass in shape-local space.
    pub center_of_mass: Vector4,
    /// Full inertia tensor about [`MassProperties::center_of_mass`].
    pub inertia_tensor: Matrix3,
    /// Principal-axis frame of the diagonalized SDK representation.
    pub major_axis_space: Quaternion,
}

impl MassProperties {
    /// Build properties for a point of zero mass.
    #[inline]
    pub const fn zero() -> Self {
        Self {
            volume: 0.0,
            mass: 0.0,
            center_of_mass: Vector4::ZERO,
            inertia_tensor: Matrix3::ZERO,
            major_axis_space: Quaternion::IDENTITY,
        }
    }

    /// Build from unit-density geometry and a mass config.
    pub fn from_geometry(
        volume: f32,
        center_of_mass: Vector4,
        unit_inertia: Matrix3,
        major_axis_space: Quaternion,
        config: MassConfig,
    ) -> Result<Self, MassError> {
        config.validate()?;
        if !volume.is_finite()
            || !vector3_is_finite(center_of_mass)
            || !matrix_is_finite(unit_inertia)
        {
            return Err(MassError::InvalidGeometry);
        }

        let mass = config.calc_mass_from_volume(volume)?;
        let density_factor = if config.uses_explicit_mass() {
            if volume > 0.0 { mass / volume } else { 0.0 }
        } else {
            config.density().unwrap_or(0.0)
        };
        let inertia_scale = density_factor * config.inertia_scale;

        Ok(Self {
            volume,
            mass,
            center_of_mass,
            inertia_tensor: matrix_scale(unit_inertia, inertia_scale),
            major_axis_space,
        })
    }

    /// Return the same geometry with a different explicit mass, scaling the
    /// inertia tensor by `mass / old_mass` like `hknpBodyManager` does.
    pub fn with_mass(self, mass: f32) -> Result<Self, MassError> {
        if !mass.is_finite() || mass < 0.0 {
            return Err(MassError::InvalidConfig);
        }
        let inertia = if self.mass > 0.0 {
            matrix_scale(self.inertia_tensor, mass / self.mass)
        } else {
            Matrix3::ZERO
        };
        Ok(Self {
            mass,
            inertia_tensor: inertia,
            ..self
        })
    }

    /// Transform local properties by a rigid transform (`R I Rᵀ`).
    pub fn transformed(&self, transform: &Transform) -> Self {
        let rotation = transform.rotation;
        let inertia = rotation
            .mul_mat(&self.inertia_tensor)
            .mul_mat(&rotation.transpose());
        Self {
            volume: self.volume,
            mass: self.mass,
            center_of_mass: transform.transform_point(self.center_of_mass),
            inertia_tensor: inertia,
            major_axis_space: rotation.to_quaternion() * self.major_axis_space,
        }
    }

    /// Aggregate two mass properties with the parallel-axis theorem.
    pub fn combined(&self, other: &Self) -> Self {
        let total_mass = self.mass + other.mass;
        let center_of_mass = if total_mass != 0.0 {
            (self.center_of_mass * self.mass + other.center_of_mass * other.mass) / total_mass
        } else {
            self.center_of_mass
        };

        let left = self.inertia_about(center_of_mass);
        let right = other.inertia_about(center_of_mass);
        Self {
            volume: self.volume + other.volume,
            mass: total_mass,
            center_of_mass,
            inertia_tensor: matrix_add(&left, &right),
            major_axis_space: self.major_axis_space,
        }
    }

    /// Aggregate an ordered sequence of mass properties.
    pub fn combine_all<I>(properties: I) -> Self
    where
        I: IntoIterator<Item = Self>,
    {
        let mut result = Self::zero();
        for property in properties {
            result = result.combined(&property);
        }
        result
    }

    /// Inertia of this body about `point` in the same frame.
    fn inertia_about(&self, point: Vector4) -> Matrix3 {
        let offset = self.center_of_mass - point;
        let distance_squared = offset.dot3(offset);
        let shift = matrix_sub(
            &matrix_diagonal(Vector4::splat(distance_squared)),
            &matrix_outer(offset, offset),
        );
        matrix_add(&self.inertia_tensor, &matrix_scale(shift, self.mass))
    }
}

/// Build mass properties from a local AABB (SDK fallback implementation).
pub fn mass_from_aabb(aabb: &Aabb, config: MassConfig) -> Result<MassProperties, MassError> {
    let half = aabb.half_extents();
    let volume = 8.0 * half.x * half.y * half.z;
    let unit_inertia = matrix_diagonal(Vector4::from_xyz(
        volume / 3.0 * (half.y * half.y + half.z * half.z),
        volume / 3.0 * (half.x * half.x + half.z * half.z),
        volume / 3.0 * (half.x * half.x + half.y * half.y),
    ));
    MassProperties::from_geometry(
        volume,
        Vector4::from_xyz(aabb.center().x, aabb.center().y, aabb.center().z),
        unit_inertia,
        Quaternion::IDENTITY,
        config,
    )
}

/// Build a diagonal matrix from a vector.
#[inline]
pub fn matrix_diagonal(value: Vector4) -> Matrix3 {
    Matrix3::from_cols(
        Vector4::from_xyz(value.x, 0.0, 0.0),
        Vector4::from_xyz(0.0, value.y, 0.0),
        Vector4::from_xyz(0.0, 0.0, value.z),
    )
}

/// Outer product `a ⊗ b` as a column-major matrix.
#[inline]
pub fn matrix_outer(a: Vector4, b: Vector4) -> Matrix3 {
    Matrix3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// Component-wise matrix addition.
#[inline]
pub fn matrix_add(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    Matrix3::from_cols(a.col0 + b.col0, a.col1 + b.col1, a.col2 + b.col2)
}

/// Component-wise matrix subtraction.
#[inline]
pub fn matrix_sub(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    Matrix3::from_cols(a.col0 - b.col0, a.col1 - b.col1, a.col2 - b.col2)
}

/// Scale every component of a matrix.
#[inline]
pub fn matrix_scale(a: Matrix3, scale: f32) -> Matrix3 {
    Matrix3::from_cols(a.col0 * scale, a.col1 * scale, a.col2 * scale)
}

/// Trace of a matrix.
#[inline]
pub fn matrix_trace(a: Matrix3) -> f32 {
    a.element(0, 0) + a.element(1, 1) + a.element(2, 2)
}

/// Whether the first three components of a vector are finite.
#[inline]
pub fn vector3_is_finite(value: Vector4) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

/// Whether every matrix component is finite.
#[inline]
pub fn matrix_is_finite(a: Matrix3) -> bool {
    vector3_is_finite(a.col0) && vector3_is_finite(a.col1) && vector3_is_finite(a.col2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn explicit_mass_scales_inertia() {
        let unit = MassProperties::from_geometry(
            8.0,
            Vector4::ZERO,
            matrix_diagonal(Vector4::from_xyz(8.0 / 3.0 * 4.0, 0.0, 0.0)),
            Quaternion::IDENTITY,
            MassConfig::from_mass(2.0),
        )
        .unwrap();
        let scaled = unit.with_mass(6.0).unwrap();
        assert_eq!(scaled.mass, 6.0);
        assert!(
            (scaled.inertia_tensor.element(0, 0) / unit.inertia_tensor.element(0, 0) - 3.0).abs()
                < 1e-6
        );
    }

    #[test]
    fn density_uses_volume() {
        let volume = 4.0 / 3.0 * PI;
        let properties = MassProperties::from_geometry(
            volume,
            Vector4::ZERO,
            matrix_diagonal(Vector4::splat(0.4 * volume)),
            Quaternion::IDENTITY,
            MassConfig::from_density(2.0),
        )
        .unwrap();
        assert!((properties.mass - 2.0 * volume).abs() < 1e-5);
        assert!((properties.inertia_tensor.element(0, 0) - 0.8 * volume).abs() < 1e-5);
    }

    #[test]
    fn combine_uses_parallel_axis_theorem() {
        let left = MassProperties::from_geometry(
            1.0,
            Vector4::from_xyz(-1.0, 0.0, 0.0),
            Matrix3::ZERO,
            Quaternion::IDENTITY,
            MassConfig::from_mass(1.0),
        )
        .unwrap();
        let right = MassProperties::from_geometry(
            1.0,
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Matrix3::ZERO,
            Quaternion::IDENTITY,
            MassConfig::from_mass(1.0),
        )
        .unwrap();
        let combined = left.combined(&right);
        assert_eq!(combined.mass, 2.0);
        assert_eq!(combined.center_of_mass, Vector4::ZERO);
        assert_eq!(combined.inertia_tensor.element(1, 1), 2.0);
        assert_eq!(combined.inertia_tensor.element(0, 0), 0.0);
    }
}
