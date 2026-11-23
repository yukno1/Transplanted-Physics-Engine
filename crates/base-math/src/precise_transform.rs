//! Precise rigid transform (`hkPreciseTransform` subset).
//!
//! The SDK stores an `hkTransformf` prefix (f32 rotation + f32 translation)
//! followed by a second f32 vector holding the low part of the translation.
//! The effective translation is `high + low`, giving roughly 48 mantissa bits
//! without putting f64 into the solver hot path. This module mirrors that
//! double-float layout and exposes explicit f64 conversion helpers.

use crate::{Matrix3, Transform, Vector4, Vector4d};

/// Rigid transform with double-float translation (`hkPreciseTransform`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PreciseTransform {
    /// Rotation (`hkRotationf`).
    pub rotation: Matrix3,
    /// High part of the translation; W is user data (SDK default 1).
    pub translation: Vector4,
    /// Low part of the translation; W is user data (SDK default 0).
    pub translation_adjustment: Vector4,
}

impl PreciseTransform {
    /// Identity precise transform.
    pub const IDENTITY: Self = Self {
        rotation: Matrix3::IDENTITY,
        translation: Vector4::new(0.0, 0.0, 0.0, 1.0),
        translation_adjustment: Vector4::ZERO,
    };

    /// Build from an f32 transform (`hkPreciseTransform::set(const hkTransformf&)`).
    #[inline]
    pub fn from_transform(transform: Transform) -> Self {
        Self {
            rotation: transform.rotation,
            translation: transform.translation,
            translation_adjustment: Vector4::ZERO,
        }
    }

    /// Build from a rotation matrix and an f64 translation.
    pub fn from_translation_d(rotation: Matrix3, translation: Vector4d) -> Self {
        let high = Vector4::new(
            translation.x as f32,
            translation.y as f32,
            translation.z as f32,
            1.0,
        );
        let low = Vector4::from_xyz(
            (translation.x - high.x as f64) as f32,
            (translation.y - high.y as f64) as f32,
            (translation.z - high.z as f64) as f32,
        );
        Self {
            rotation,
            translation: high,
            translation_adjustment: low,
        }
    }

    /// Build from a quaternion and an f64 translation.
    #[inline]
    pub fn from_quaternion_translation_d(
        rotation: crate::Quaternion,
        translation: Vector4d,
    ) -> Self {
        Self::from_translation_d(Matrix3::from_quaternion(rotation), translation)
    }

    /// Effective translation as f64 (`getTranslation`).
    #[inline]
    pub fn translation_d(&self) -> Vector4d {
        Vector4d::new(
            self.translation.x as f64 + self.translation_adjustment.x as f64,
            self.translation.y as f64 + self.translation_adjustment.y as f64,
            self.translation.z as f64 + self.translation_adjustment.z as f64,
            1.0,
        )
    }

    /// f32 prefix view (`asTransformf`), keeping only the high translation.
    #[inline]
    pub fn to_transform(&self) -> Transform {
        Transform::new(self.rotation, self.translation)
    }

    /// Apply the transform to an f32 point, dropping low precision at the end.
    #[inline]
    pub fn transform_point(&self, point: Vector4) -> Vector4 {
        let low = self.translation_adjustment + self.rotation.mul_vec(point);
        self.translation + low
    }

    /// Apply the transform to a point with f64 translation precision.
    pub fn transform_point_d(&self, point: Vector4d) -> Vector4d {
        let rotated = self.rotate_d(Vector4d::new(point.x, point.y, point.z, 0.0));
        let translation = self.translation_d();
        Vector4d::new(
            rotated.x + translation.x,
            rotated.y + translation.y,
            rotated.z + translation.z,
            point.w,
        )
    }

    /// Inverse-transform a point with f64 translation precision.
    pub fn inverse_transform_point_d(&self, point: Vector4d) -> Vector4d {
        let translation = self.translation_d();
        let relative = Vector4d::new(
            point.x - translation.x,
            point.y - translation.y,
            point.z - translation.z,
            point.w,
        );
        let rotation = self.rotation;
        Vector4d::new(
            rotation.element(0, 0) as f64 * relative.x
                + rotation.element(0, 1) as f64 * relative.y
                + rotation.element(0, 2) as f64 * relative.z,
            rotation.element(1, 0) as f64 * relative.x
                + rotation.element(1, 1) as f64 * relative.y
                + rotation.element(1, 2) as f64 * relative.z,
            rotation.element(2, 0) as f64 * relative.x
                + rotation.element(2, 1) as f64 * relative.y
                + rotation.element(2, 2) as f64 * relative.z,
            relative.w,
        )
    }

    /// Compensated translation add (`_addTranslation`).
    pub fn add_translation(&mut self, delta: Vector4) {
        for axis in 0..3 {
            let high = self.translation.component(axis);
            let mut adjustment =
                self.translation_adjustment.component(axis) + delta.component(axis);
            let next_high = high + adjustment;
            adjustment -= next_high - high;
            self.translation.set_component(axis, next_high);
            self.translation_adjustment.set_component(axis, adjustment);
        }
    }

    /// Compose `self = parent * local` with compensated translation.
    pub fn set_mul(&mut self, parent: PreciseTransform, local: Transform) {
        let rotation = parent.rotation.mul_mat(&local.rotation);
        let rotated_local = parent.rotation.mul_vec(local.translation);
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
        Transform::new(
            self.rotation.transpose().mul_mat(&other.rotation),
            self.rotation.rotate_inverse(delta),
        )
    }

    fn rotate_d(&self, vector: Vector4d) -> Vector4d {
        let rotation = self.rotation;
        Vector4d::new(
            rotation.element(0, 0) as f64 * vector.x
                + rotation.element(0, 1) as f64 * vector.y
                + rotation.element(0, 2) as f64 * vector.z,
            rotation.element(1, 0) as f64 * vector.x
                + rotation.element(1, 1) as f64 * vector.y
                + rotation.element(1, 2) as f64 * vector.z,
            rotation.element(2, 0) as f64 * vector.x
                + rotation.element(2, 1) as f64 * vector.y
                + rotation.element(2, 2) as f64 * vector.z,
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
    use crate::Quaternion;

    #[test]
    fn f32_prefix_round_trip_is_exact() {
        let transform = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
            Vector4::from_xyz(4.0, -2.0, 1.0),
        );
        let precise = PreciseTransform::from_transform(transform);
        assert_eq!(precise.to_transform(), transform);
        assert_eq!(precise.translation_adjustment, Vector4::ZERO);
    }

    #[test]
    fn double_translation_round_trip_keeps_low_bits() {
        let expected = Vector4d::new(1_234_567.25, -2_000_003.5, 12.000_000_1, 1.0);
        let precise = PreciseTransform::from_translation_d(Matrix3::IDENTITY, expected);
        let actual = precise.translation_d();
        assert!((actual.x - expected.x).abs() < 1e-6);
        assert!((actual.y - expected.y).abs() < 1e-6);
        assert!((actual.z - expected.z).abs() < 1e-6);
    }

    #[test]
    fn compensated_add_preserves_small_delta() {
        let start = Vector4d::new(10_000_000.0, 0.0, 0.0, 1.0);
        let mut precise = PreciseTransform::from_translation_d(Matrix3::IDENTITY, start);
        precise.add_translation(Vector4::from_xyz(0.25, 0.0, 0.0));
        assert!((precise.translation_d().x - 10_000_000.25).abs() < 1e-4);
    }

    #[test]
    fn relative_transform_maps_large_world_to_small_f32() {
        let a = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            Vector4d::new(1_000_000.0, 0.0, 0.0, 1.0),
        );
        let c = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            Vector4d::new(1_000_002.5, 0.0, 0.0, 1.0),
        );
        let relative = a.relative_to(c);
        assert!((relative.translation.x - 2.5).abs() < 1e-3);
    }

    #[test]
    fn double_point_application_uses_precise_translation() {
        let precise = PreciseTransform::from_translation_d(
            Matrix3::IDENTITY,
            Vector4d::new(1_000_000.0, 0.0, 0.0, 1.0),
        );
        let point = Vector4d::new(1_000_003.5, 0.0, 0.0, 0.0);
        let local = precise.inverse_transform_point_d(point);
        assert!((local.x - 3.5).abs() < 1e-3);
        let back = precise.transform_point_d(local);
        assert!((back.x - point.x).abs() < 1e-3);
    }
}
