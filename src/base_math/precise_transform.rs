//! Precise rigid transform (`hkPreciseTransform` subset).
//!
//! The SDK stores an `hkTransformf` prefix (f32 rotation + f32 translation)
//! followed by a second f32 vector holding the low part of the translation.
//! The effective translation is `high + low`, giving roughly 48 mantissa bits
//! without putting f64 into the solver hot path. This module mirrors that
//! double-float layout and exposes explicit f64 conversion helpers.
//!
//! Storage is glam-backed (`Mat3` + `Vec4`) since the glam migration
//! (`docs/refactor-glam-plan.md`); every double-float arithmetic step below is
//! ported verbatim from the pre-migration implementation, so results stay
//! bit-identical. Legacy↔glam boundary conversions copy elements without
//! arithmetic and therefore preserve bits.

use glam::{DVec4, Mat3, Vec3, Vec4};

use crate::base_math::{Mat3Ext, Matrix3, Quaternion, Transform, Vector4};

/// Rigid transform with double-float translation (`hkPreciseTransform`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PreciseTransform {
    /// Rotation (`hkRotationf`).
    pub rotation: Mat3,
    /// High part of the translation; W is user data (SDK default 1).
    pub translation: Vec4,
    /// Low part of the translation; W is user data (SDK default 0).
    pub translation_adjustment: Vec4,
}

/// Copy a legacy `Matrix3`'s columns into a glam `Mat3` (bit-preserving).
fn mat3_from_legacy(rotation: Matrix3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(rotation.col0.x, rotation.col0.y, rotation.col0.z),
        Vec3::new(rotation.col1.x, rotation.col1.y, rotation.col1.z),
        Vec3::new(rotation.col2.x, rotation.col2.y, rotation.col2.z),
    )
}

/// Copy a glam `Mat3`'s columns into a legacy `Matrix3` (bit-preserving,
/// column `w` zeroed like the legacy rotation-matrix invariant).
fn legacy_from_mat3(rotation: Mat3) -> Matrix3 {
    Matrix3::from_cols(
        Vector4::from_xyz(rotation.x_axis.x, rotation.x_axis.y, rotation.x_axis.z),
        Vector4::from_xyz(rotation.y_axis.x, rotation.y_axis.y, rotation.y_axis.z),
        Vector4::from_xyz(rotation.z_axis.x, rotation.z_axis.y, rotation.z_axis.z),
    )
}

impl PreciseTransform {
    /// Identity precise transform.
    pub const IDENTITY: Self = Self {
        rotation: Mat3::IDENTITY,
        translation: Vec4::new(0.0, 0.0, 0.0, 1.0),
        translation_adjustment: Vec4::ZERO,
    };

    /// Build from an f32 transform (`hkPreciseTransform::set(const hkTransformf&)`).
    #[inline]
    pub fn from_transform(transform: Transform) -> Self {
        Self {
            rotation: mat3_from_legacy(transform.rotation),
            translation: Vec4::new(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.translation.w,
            ),
            translation_adjustment: Vec4::ZERO,
        }
    }

    /// Build from a rotation matrix and an f64 translation.
    pub fn from_translation_d(rotation: Matrix3, translation: DVec4) -> Self {
        let high = Vec4::new(
            translation.x as f32,
            translation.y as f32,
            translation.z as f32,
            1.0,
        );
        let low = Vec4::new(
            (translation.x - high.x as f64) as f32,
            (translation.y - high.y as f64) as f32,
            (translation.z - high.z as f64) as f32,
            0.0,
        );
        Self {
            rotation: mat3_from_legacy(rotation),
            translation: high,
            translation_adjustment: low,
        }
    }

    /// Build from a quaternion and an f64 translation.
    #[inline]
    pub fn from_quaternion_translation_d(rotation: Quaternion, translation: DVec4) -> Self {
        Self::from_translation_d(Matrix3::from_quaternion(rotation), translation)
    }

    /// Effective translation as f64 (`getTranslation`).
    #[inline]
    pub fn translation_d(&self) -> DVec4 {
        DVec4::new(
            self.translation.x as f64 + self.translation_adjustment.x as f64,
            self.translation.y as f64 + self.translation_adjustment.y as f64,
            self.translation.z as f64 + self.translation_adjustment.z as f64,
            1.0,
        )
    }

    /// f32 prefix view (`asTransformf`), keeping only the high translation.
    #[inline]
    pub fn to_transform(&self) -> Transform {
        Transform::new(
            legacy_from_mat3(self.rotation),
            Vector4::new(
                self.translation.x,
                self.translation.y,
                self.translation.z,
                self.translation.w,
            ),
        )
    }

    /// Apply the transform to an f32 point, dropping low precision at the end.
    #[inline]
    pub fn transform_point(&self, point: Vector4) -> Vector4 {
        let rotated = self
            .rotation
            .mul_vec_sdk(Vec3::new(point.x, point.y, point.z));
        let rotated = Vec4::new(rotated.x, rotated.y, rotated.z, 0.0);
        let low = self.translation_adjustment + rotated;
        Vector4::new(
            self.translation.x + low.x,
            self.translation.y + low.y,
            self.translation.z + low.z,
            self.translation.w + low.w,
        )
    }

    /// Apply the transform to a point with f64 translation precision.
    pub fn transform_point_d(&self, point: DVec4) -> DVec4 {
        let rotated = self.rotate_d(DVec4::new(point.x, point.y, point.z, 0.0));
        let translation = self.translation_d();
        DVec4::new(
            rotated.x + translation.x,
            rotated.y + translation.y,
            rotated.z + translation.z,
            point.w,
        )
    }

    /// Inverse-transform a point with f64 translation precision.
    pub fn inverse_transform_point_d(&self, point: DVec4) -> DVec4 {
        let translation = self.translation_d();
        let relative = DVec4::new(
            point.x - translation.x,
            point.y - translation.y,
            point.z - translation.z,
            point.w,
        );
        let rotation = self.rotation;
        DVec4::new(
            rotation.x_axis.x as f64 * relative.x
                + rotation.y_axis.x as f64 * relative.y
                + rotation.z_axis.x as f64 * relative.z,
            rotation.x_axis.y as f64 * relative.x
                + rotation.y_axis.y as f64 * relative.y
                + rotation.z_axis.y as f64 * relative.z,
            rotation.x_axis.z as f64 * relative.x
                + rotation.y_axis.z as f64 * relative.y
                + rotation.z_axis.z as f64 * relative.z,
            relative.w,
        )
    }

    /// Compensated translation add (`_addTranslation`).
    pub fn add_translation(&mut self, delta: Vec4) {
        let mut high = self.translation.to_array();
        let mut adjustment = self.translation_adjustment.to_array();
        let delta = delta.to_array();
        for axis in 0..3 {
            let high_axis = high[axis];
            let mut adjustment_axis = adjustment[axis] + delta[axis];
            let next_high = high_axis + adjustment_axis;
            adjustment_axis -= next_high - high_axis;
            high[axis] = next_high;
            adjustment[axis] = adjustment_axis;
        }
        self.translation = Vec4::from_array(high);
        self.translation_adjustment = Vec4::from_array(adjustment);
    }

    /// Compose `self = parent * local` with compensated translation.
    pub fn set_mul(&mut self, parent: PreciseTransform, local: Transform) {
        let local_rotation = mat3_from_legacy(local.rotation);
        let rotation = Mat3::from_cols(
            parent.rotation.mul_vec_sdk(local_rotation.x_axis),
            parent.rotation.mul_vec_sdk(local_rotation.y_axis),
            parent.rotation.mul_vec_sdk(local_rotation.z_axis),
        );
        let rotated_local = parent.rotation.mul_vec_sdk(Vec3::new(
            local.translation.x,
            local.translation.y,
            local.translation.z,
        ));
        let rotated_local = Vec4::new(rotated_local.x, rotated_local.y, rotated_local.z, 0.0);
        let mut adjustment = parent.translation_adjustment + rotated_local;
        let high = parent.translation + adjustment;
        adjustment -= high - parent.translation;
        *self = Self {
            rotation,
            translation: high,
            translation_adjustment: adjustment,
        };
    }

    /// f32 transform of `other` relative to `self` (`_setMulInverseMul`).
    pub fn relative_to(&self, other: PreciseTransform) -> Transform {
        let delta = (other.translation - self.translation)
            + (other.translation_adjustment - self.translation_adjustment);
        let other_rotation = other.rotation;
        let rotation = Mat3::from_cols(
            self.rotation.transpose().mul_vec_sdk(other_rotation.x_axis),
            self.rotation.transpose().mul_vec_sdk(other_rotation.y_axis),
            self.rotation.transpose().mul_vec_sdk(other_rotation.z_axis),
        );
        // Legacy `Matrix3::rotate_inverse`: `from_xyz(col_i.dot3(v), ..)` with
        // the fixed scalar addition order per row.
        let delta3 = Vec3::new(delta.x, delta.y, delta.z);
        let rotation_inv = self.rotation;
        let translation = Vector4::from_xyz(
            rotation_inv.x_axis.x * delta3.x
                + rotation_inv.x_axis.y * delta3.y
                + rotation_inv.x_axis.z * delta3.z,
            rotation_inv.y_axis.x * delta3.x
                + rotation_inv.y_axis.y * delta3.y
                + rotation_inv.y_axis.z * delta3.z,
            rotation_inv.z_axis.x * delta3.x
                + rotation_inv.z_axis.y * delta3.y
                + rotation_inv.z_axis.z * delta3.z,
        );
        Transform::new(legacy_from_mat3(rotation), translation)
    }

    fn rotate_d(&self, vector: DVec4) -> DVec4 {
        let rotation = self.rotation;
        DVec4::new(
            rotation.x_axis.x as f64 * vector.x
                + rotation.y_axis.x as f64 * vector.y
                + rotation.z_axis.x as f64 * vector.z,
            rotation.x_axis.y as f64 * vector.x
                + rotation.y_axis.y as f64 * vector.y
                + rotation.z_axis.y as f64 * vector.z,
            rotation.x_axis.z as f64 * vector.x
                + rotation.y_axis.z as f64 * vector.y
                + rotation.z_axis.z as f64 * vector.z,
            vector.w,
        )
    }
}

impl Default for PreciseTransform {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn f32_prefix_round_trip_is_exact() {
        let transform = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
            Vector4::from_xyz(4.0, -2.0, 1.0),
        );
        let precise = PreciseTransform::from_transform(transform);
        assert_eq!(precise.to_transform(), transform);
        assert_eq!(precise.translation_adjustment, Vec4::ZERO);
    }

    #[test]
    fn double_translation_round_trip_keeps_low_bits() {
        let expected = DVec4::new(1_234_567.25, -2_000_003.5, 12.000_000_1, 1.0);
        let precise = PreciseTransform::from_translation_d(Matrix3::IDENTITY, expected);
        let actual = precise.translation_d();
        assert!((actual.x - expected.x).abs() < 1e-6);
        assert!((actual.y - expected.y).abs() < 1e-6);
        assert!((actual.z - expected.z).abs() < 1e-6);
    }

    #[test]
    fn glam_storage_matches_legacy_layout() {
        let transform = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
            Vector4::from_xyz(4.0, -2.0, 1.0),
        );
        let precise = PreciseTransform::from_transform(transform);
        assert_eq!(
            precise.rotation.x_axis,
            Vec3::new(
                transform.rotation.col0.x,
                transform.rotation.col0.y,
                transform.rotation.col0.z
            )
        );
        assert_eq!(precise.translation.w, transform.translation.w);
    }

    #[test]
    fn compensated_add_preserves_small_delta() {
        let start = DVec4::new(10_000_000.0, 0.0, 0.0, 1.0);
        let mut precise = PreciseTransform::from_translation_d(Matrix3::IDENTITY, start);
        precise.add_translation(Vec4::new(0.25, 0.0, 0.0, 0.0));
        assert!((precise.translation_d().x - 10_000_000.25).abs() < 1e-4);
    }

    #[test]
    fn relative_transform_maps_large_world_to_small_f32() {
        let a = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            DVec4::new(1_000_000.0, 0.0, 0.0, 1.0),
        );
        let c = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            DVec4::new(1_000_002.5, 0.0, 0.0, 1.0),
        );
        let relative = a.relative_to(c);
        assert!((relative.translation.x - 2.5).abs() < 1e-3);
    }

    #[test]
    fn double_point_application_uses_precise_translation() {
        let precise = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            DVec4::new(1_000_000.0, 0.0, 0.0, 1.0),
        );
        let point = DVec4::new(1_000_003.5, 0.0, 0.0, 0.0);
        let local = precise.inverse_transform_point_d(point);
        assert!((local.x - 3.5).abs() < 1e-3);
        let back = precise.transform_point_d(local);
        assert!((back.x - point.x).abs() < 1e-3);
    }
}
