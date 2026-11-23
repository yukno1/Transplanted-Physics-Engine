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

const SDK_EXPONENT_MASK: u32 = 0x7f80_0000;
const SDK_OFFSET: u32 = 0x4e80_0000;
const SDK_ROUNDING: f32 = f32::from_bits(0x3f80_0089);
const SDK_MANTISSA_CORRECTION: u32 = 0x0000_8000;
const SDK_EXPONENT_CORRECTION: u32 = 0x3f80_0000;

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

    /// Pack using the SDK's float-bit algorithm (`hkPackedVector3::_pack`).
    ///
    /// The result uses the SDK mantissa/exponent words; byte order is the
    /// explicit little-endian layout used by [`Self::to_bytes`].
    pub fn pack_sdk(value: Vector4) -> Self {
        if !vector3_is_finite(value) {
            return Self::ZERO;
        }
        let epsilon_squared = f32::EPSILON * f32::EPSILON;
        let components = [value.x, value.y, value.z, epsilon_squared];
        let mut max_exponent = 0u32;
        for &component in &components {
            let rounded = component * SDK_ROUNDING;
            max_exponent = max_exponent.max(rounded.to_bits() & SDK_EXPONENT_MASK);
        }
        let mut words = [0i16; 4];
        for (index, &component) in components.iter().enumerate() {
            let corrected_bits = component
                .to_bits()
                .wrapping_add(SDK_OFFSET)
                .wrapping_sub(max_exponent);
            let corrected = f32::from_bits(corrected_bits);
            let integer = (corrected as i32) as u32;
            let word = if index == 3 {
                max_exponent
                    .wrapping_sub(SDK_OFFSET)
                    .wrapping_add(SDK_EXPONENT_CORRECTION)
            } else {
                integer.wrapping_add(SDK_MANTISSA_CORRECTION)
            };
            words[index] = (word >> 16) as u16 as i16;
        }
        Self { values: words }
    }

    /// Unpack SDK float-bit words (`hkPackedVector3::unpack`).
    pub fn unpack_sdk(self) -> Vector4 {
        let exponent = f32::from_bits((self.values[3] as u32) << 16);
        let component = |index: usize| {
            let mantissa = (self.values[index] as i64) << 16;
            (mantissa as f32) * exponent
        };
        Vector4::from_xyz(component(0), component(1), component(2))
    }

    /// Explicit little-endian byte storage (8 bytes).
    pub fn to_bytes(self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        for (index, value) in self.values.iter().enumerate() {
            bytes[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }

    /// Read the explicit little-endian byte storage.
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let mut values = [0i16; 4];
        for (index, value) in values.iter_mut().enumerate() {
            *value = i16::from_le_bytes([bytes[index * 2], bytes[index * 2 + 1]]);
        }
        Self { values }
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

    /// Pack with the SDK convention (no hemisphere canonicalization).
    pub fn pack_sdk(quaternion: Quaternion) -> Self {
        if !quaternion.x.is_finite()
            || !quaternion.y.is_finite()
            || !quaternion.z.is_finite()
            || !quaternion.w.is_finite()
            || quaternion.length_squared() <= 0.0
        {
            return Self::IDENTITY;
        }
        let components = [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
        let mut values = [0u16; 4];
        for (index, component) in components.iter().enumerate() {
            let fixed = (*component * UNIT_SCALE).round() as i64 + (1i64 << 31);
            values[index] = (fixed >> 16) as u16;
        }
        Self { values }
    }

    /// Unpack SDK words without normalizing the quaternion.
    pub fn unpack_sdk(self) -> Quaternion {
        let mut components = [0.0f32; 4];
        for (index, value) in self.values.iter().enumerate() {
            let signed = (*value as i16) as i64 * 65536;
            let offset = if value & 0x8000 != 0 {
                1i64 << 31
            } else {
                -(1i64 << 31)
            };
            components[index] = ((signed + offset) as f32 / UNIT_SCALE).clamp(-1.0, 1.0);
        }
        Quaternion::new(components[0], components[1], components[2], components[3])
    }

    /// Explicit little-endian byte storage (8 bytes).
    pub fn to_bytes(self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        for (index, value) in self.values.iter().enumerate() {
            bytes[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }

    /// Read the explicit little-endian byte storage.
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let mut values = [0u16; 4];
        for (index, value) in values.iter_mut().enumerate() {
            *value = u16::from_le_bytes([bytes[index * 2], bytes[index * 2 + 1]]);
        }
        Self { values }
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

    /// Pack with the SDK-compatible word layout (32 bytes).
    ///
    /// Mass and volume keep their exact f32 bit patterns; the vector and
    /// quaternion words use the SDK packing rules. This is still a portable
    /// little-endian byte layout, not a claim of real `.hkx` asset equivalence.
    pub fn pack_sdk(properties: &MassProperties) -> Result<Self, MassError> {
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
            center_of_mass: PackedVector3::pack_sdk(properties.center_of_mass),
            inertia: PackedVector3::pack_sdk(diagonalized.diagonal),
            major_axis_space: PackedUnitVector4::pack_sdk(diagonalized.orientation),
            mass: properties.mass,
            volume: properties.volume,
        })
    }

    /// Unpack SDK words, normalizing the recovered quaternion.
    pub fn unpack_sdk(self) -> MassProperties {
        let mut orientation = self.major_axis_space.unpack_sdk();
        if orientation.length_squared() > 0.0 {
            orientation.normalize();
        } else {
            orientation = Quaternion::IDENTITY;
        }
        let rotation = Matrix3::from_quaternion(orientation);
        let diagonal = matrix_diagonal(self.inertia.unpack_sdk());
        let inertia_tensor = rotation.mul_mat(&diagonal).mul_mat(&rotation.transpose());
        MassProperties {
            volume: self.volume,
            mass: self.mass,
            center_of_mass: self.center_of_mass.unpack_sdk(),
            inertia_tensor,
            major_axis_space: orientation,
        }
    }

    /// Explicit 32-byte little-endian storage.
    pub fn to_bytes(self) -> [u8; 32] {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&self.center_of_mass.to_bytes());
        bytes[8..16].copy_from_slice(&self.inertia.to_bytes());
        bytes[16..24].copy_from_slice(&self.major_axis_space.to_bytes());
        bytes[24..28].copy_from_slice(&self.mass.to_le_bytes());
        bytes[28..32].copy_from_slice(&self.volume.to_le_bytes());
        bytes
    }

    /// Read explicit 32-byte little-endian storage.
    pub fn from_bytes(bytes: [u8; 32]) -> Self {
        let mut center = [0u8; 8];
        center.copy_from_slice(&bytes[0..8]);
        let mut inertia = [0u8; 8];
        inertia.copy_from_slice(&bytes[8..16]);
        let mut orientation = [0u8; 8];
        orientation.copy_from_slice(&bytes[16..24]);
        let mut mass = [0u8; 4];
        mass.copy_from_slice(&bytes[24..28]);
        let mut volume = [0u8; 4];
        volume.copy_from_slice(&bytes[28..32]);
        Self {
            center_of_mass: PackedVector3::from_bytes(center),
            inertia: PackedVector3::from_bytes(inertia),
            major_axis_space: PackedUnitVector4::from_bytes(orientation),
            mass: f32::from_le_bytes(mass),
            volume: f32::from_le_bytes(volume),
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

    #[test]
    fn packed_words_have_explicit_little_endian_bytes() {
        assert_eq!(PackedVector3::ZERO.to_bytes(), [0u8; 8]);
        assert_eq!(
            PackedUnitVector4::IDENTITY.to_bytes(),
            [0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00]
        );
        assert_eq!(PackedVector3::from_bytes([0u8; 8]), PackedVector3::ZERO);
        assert_eq!(
            PackedUnitVector4::from_bytes(PackedUnitVector4::IDENTITY.to_bytes()),
            PackedUnitVector4::IDENTITY
        );
    }

    #[test]
    fn sdk_packed_vector_round_trip_is_stable() {
        let value = Vector4::from_xyz(12.5, -3.25, 0.75);
        let packed = PackedVector3::pack_sdk(value);
        assert_eq!(packed, PackedVector3::pack_sdk(value));
        let unpacked = packed.unpack_sdk();
        assert!((unpacked.x - value.x).abs() < 1e-3);
        assert!((unpacked.y - value.y).abs() < 1e-3);
        assert!((unpacked.z - value.z).abs() < 1e-3);
    }

    #[test]
    fn compressed_mass_sdk_bytes_round_trip() {
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
        let packed = CompressedMassProperties::pack_sdk(&properties).unwrap();
        let bytes = packed.to_bytes();
        assert_eq!(CompressedMassProperties::from_bytes(bytes), packed);
        assert_eq!(packed.unpack_sdk(), packed.unpack_sdk());
    }
}
