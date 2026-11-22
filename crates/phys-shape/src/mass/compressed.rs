//! Deterministic compressed mass properties (`hkCompressedMassProperties`).
//!
//! This is a portable approximation of the SDK's 15-bit packed representation:
//! a shared-exponent 15-bit vector for centre of mass and diagonal inertia,
//! and a 15-bit packed unit quaternion for the principal-axis frame. It is
//! deterministic and compact, but it is explicitly **not** claimed to be
//! bit-identical to Havok's float-bit packing without a real `.hkx` golden
//! comparison.

use base_math::{Matrix3, Quaternion, Vector4};

use super::eigen::diagonalize_symmetric;
use super::{MassError, MassProperties, matrix_diagonal, matrix_is_finite, vector3_is_finite};

const MANTISSA_SCALE: f32 = 32767.0;
const UNIT_SCALE: f32 = 30000.0 * 65536.0;

/// Portable `hkPackedVector3`: three 15-bit mantissas and a shared exponent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PackedVector3 {
    /// Mantissas for x/y/z followed by the exponent in `values[3]`.
    pub values: [i16; 4],
}

impl PackedVector3 {
    /// A packed zero vector.
    pub const ZERO: Self = Self {
        values: [0, 0, 0, 0],
    };

    /// Pack a finite vector into 15-bit mantissas with a shared base-2 exponent.
    pub fn pack(value: Vector4) -> Self {
        if !vector3_is_finite(value) {
            return Self::ZERO;
        }
        let max_abs = value.x.abs().max(value.y.abs()).max(value.z.abs());
        if max_abs <= 0.0 || !max_abs.is_finite() {
            return Self::ZERO;
        }
        let exponent = exponent_for(max_abs);
        let power = power_of_two(exponent);
        let scale = MANTISSA_SCALE / power;
        let mantissa = |component: f32| {
            (component * scale)
                .round()
                .clamp(-MANTISSA_SCALE, MANTISSA_SCALE) as i16
        };
        Self {
            values: [
                mantissa(value.x),
                mantissa(value.y),
                mantissa(value.z),
                exponent,
            ],
        }
    }

    /// Unpack to a full-precision vector.
    pub fn unpack(self) -> Vector4 {
        let power = power_of_two(self.values[3]);
        let scale = power / MANTISSA_SCALE;
        Vector4::from_xyz(
            self.values[0] as f32 * scale,
            self.values[1] as f32 * scale,
            self.values[2] as f32 * scale,
        )
    }
}

fn exponent_for(value: f32) -> i16 {
    let bits = value.to_bits();
    let biased = ((bits >> 23) & 0xff) as i32;
    let exponent = if biased == 0 { -126 } else { biased - 127 };
    let power = power_of_two(exponent as i16);
    let adjusted = if value > power {
        exponent + 1
    } else {
        exponent
    };
    adjusted.clamp(-126, 127) as i16
}

fn power_of_two(exponent: i16) -> f32 {
    let biased = (exponent as i32 + 127).clamp(1, 254) as u32;
    f32::from_bits(biased << 23)
}

/// Portable `hkPackedUnitVector<4>` using four signed 15-bit components.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PackedUnitVector4 {
    /// Packed quaternion components in `(x, y, z, w)` order.
    pub values: [u16; 4],
}

impl PackedUnitVector4 {
    /// Identity quaternion packing.
    pub const IDENTITY: Self = Self {
        values: [0x8000, 0x8000, 0x8000, 0x0000],
    };

    /// Pack a unit quaternion, canonicalized to the `w >= 0` hemisphere.
    pub fn pack(quaternion: Quaternion) -> Self {
        let mut quaternion = quaternion;
        if !quaternion.w.is_finite()
            || !quaternion.x.is_finite()
            || !quaternion.y.is_finite()
            || !quaternion.z.is_finite()
        {
            return Self::IDENTITY;
        }
        if quaternion.length_squared() <= 0.0 {
            return Self::IDENTITY;
        }
        quaternion.normalize();
        if quaternion.w < 0.0 {
            quaternion =
                Quaternion::new(-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w);
        }
        let components = [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
        let mut values = [0u16; 4];
        for (index, component) in components.iter().enumerate() {
            let fixed = (*component * UNIT_SCALE).round() as i64 + (1i64 << 31);
            values[index] = (fixed >> 16) as u16;
        }
        Self { values }
    }

    /// Unpack and normalize the quaternion.
    pub fn unpack(self) -> Quaternion {
        let mut components = [0.0f32; 4];
        for (index, value) in self.values.iter().enumerate() {
            let signed = (*value as i16) as i64 * 65536;
            let offset = if value & 0x8000 != 0 {
                1i64 << 31
            } else {
                -(1i64 << 31)
            };
            let fixed = signed + offset;
            components[index] = (fixed as f32 / UNIT_SCALE).clamp(-1.0, 1.0);
        }
        let mut quaternion =
            Quaternion::new(components[0], components[1], components[2], components[3]);
        if quaternion.length_squared() > 0.0 {
            quaternion.normalize();
        } else {
            quaternion = Quaternion::IDENTITY;
        }
        quaternion
    }
}

/// Compact mass-property representation used by caches and storage boundaries.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompressedMassProperties {
    /// Packed centre of mass in shape-local space.
    pub center_of_mass: PackedVector3,
    /// Packed diagonal inertia in principal-axis space.
    pub inertia: PackedVector3,
    /// Packed principal-axis frame.
    pub major_axis_space: PackedUnitVector4,
    /// Exact mass (not quantized in the SDK either).
    pub mass: f32,
    /// Exact volume (not quantized in the SDK either).
    pub volume: f32,
}

impl CompressedMassProperties {
    /// Pack full mass properties, diagonalizing the inertia tensor first.
    pub fn pack(properties: &MassProperties) -> Result<Self, MassError> {
        if !properties.mass.is_finite()
            || properties.mass < 0.0
            || !properties.volume.is_finite()
            || properties.volume < 0.0
            || !vector3_is_finite(properties.center_of_mass)
            || !matrix_is_finite(properties.inertia_tensor)
        {
            return Err(MassError::InvalidGeometry);
        }
        let diagonalized = diagonalize_symmetric(properties.inertia_tensor);
        Ok(Self {
            center_of_mass: PackedVector3::pack(properties.center_of_mass),
            inertia: PackedVector3::pack(diagonalized.diagonal),
            major_axis_space: PackedUnitVector4::pack(diagonalized.orientation),
            mass: properties.mass,
            volume: properties.volume,
        })
    }

    /// Unpack to full mass properties for diagnostics or cache reconstruction.
    pub fn unpack(self) -> MassProperties {
        let orientation = self.major_axis_space.unpack();
        let rotation = Matrix3::from_quaternion(orientation);
        let diagonal = matrix_diagonal(self.inertia.unpack());
        let inertia_tensor = rotation.mul_mat(&diagonal).mul_mat(&rotation.transpose());
        MassProperties {
            volume: self.volume,
            mass: self.mass,
            center_of_mass: self.center_of_mass.unpack(),
            inertia_tensor,
            major_axis_space: orientation,
        }
    }

    /// Maximum absolute difference between the original and reconstructed
    /// mass properties, useful for compression tolerance tests.
    pub fn max_error(self, original: &MassProperties) -> f32 {
        let unpacked = self.unpack();
        let mut error = (unpacked.mass - original.mass).abs();
        error = error.max((unpacked.volume - original.volume).abs());
        for axis in 0..3 {
            error = error.max(
                (unpacked.center_of_mass.component(axis) - original.center_of_mass.component(axis))
                    .abs(),
            );
            for row in 0..3 {
                error = error.max(
                    (unpacked.inertia_tensor.element(row, axis)
                        - original.inertia_tensor.element(row, axis))
                    .abs(),
                );
            }
        }
        error
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_vector_round_trip_stays_within_relative_bound() {
        let value = Vector4::from_xyz(12.5, -3.25, 0.75);
        let unpacked = PackedVector3::pack(value).unpack();
        for axis in 0..3 {
            let expected = value.component(axis);
            assert!(
                (unpacked.component(axis) - expected).abs() <= expected.abs() * 1.0e-4 + 1.0e-4,
                "axis {axis}: {unpacked:?} vs {value:?}"
            );
        }
    }

    #[test]
    fn packed_unit_vector_round_trip_is_normalized() {
        let quaternion = Quaternion::from_axis_angle(Vector4::from_xyz(1.0, 2.0, 3.0), 0.8);
        let packed = PackedUnitVector4::pack(quaternion);
        let unpacked = packed.unpack();
        assert!((unpacked.length() - 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn compressed_mass_properties_round_trip_is_deterministic() {
        let properties = MassProperties::from_geometry(
            8.0,
            Vector4::from_xyz(0.1, -0.2, 0.3),
            Matrix3::from_cols(
                Vector4::from_xyz(8.0, 0.4, 0.0),
                Vector4::from_xyz(0.4, 4.0, 0.0),
                Vector4::from_xyz(0.0, 0.0, 2.0),
            ),
            Quaternion::IDENTITY,
            super::super::MassConfig::from_density(2.0),
        )
        .unwrap();
        let packed = CompressedMassProperties::pack(&properties).unwrap();
        assert_eq!(packed, CompressedMassProperties::pack(&properties).unwrap());
        assert!(packed.max_error(&properties) < 0.01);
        assert_eq!(packed.unpack(), packed.unpack());
    }
}
