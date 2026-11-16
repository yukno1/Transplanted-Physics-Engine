//! Quaternion + translation transforms.
//!
//! Port of `hkQTransformf` from
//! `original/Common/Source/Common/Base/Math/QTransform/`.
//!
//! Composition is `(q1, t1) * (q2, t2) = (q1*q2, rotate(q1, t2) + t1)`, and the
//! inverse is `(q^-1, -rotate(q^-1, t))` — exactly the formulas in
//! `hkQTransformf.inl::_setMul` / `_setInverse`.

use core::ops::Mul;

use crate::quaternion::Quaternion;
use crate::transform::Transform;
use crate::vector::Vector4;

/// Quaternion-based transform (`hkQTransformf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QTransform {
    /// Rotation quaternion.
    pub rotation: Quaternion,
    /// Translation.
    pub translation: Vector4,
}

impl QTransform {
    /// Identity transform (`hkQTransformf::setIdentity`).
    pub const IDENTITY: Self = Self {
        rotation: Quaternion::IDENTITY,
        translation: Vector4::ZERO,
    };

    /// `hkQTransformf(q, t)`.
    #[inline]
    pub const fn new(rotation: Quaternion, translation: Vector4) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// `hkQTransformf::set`.
    #[inline]
    pub fn set(&mut self, rotation: Quaternion, translation: Vector4) {
        self.rotation = rotation;
        self.translation = translation;
    }

    /// `hkQTransformf::set(hkTransformf)`.
    #[inline]
    pub fn from_transform(transform: &Transform) -> Self {
        Self::new(transform.rotation_quaternion(), transform.translation)
    }

    /// Convert back to a matrix transform.
    #[inline]
    pub fn to_transform(self) -> Transform {
        Transform::from_quaternion(self.rotation, self.translation)
    }

    /// Rotate a point and translate it (`setTransformedPos(transform, p)`).
    #[inline]
    pub fn transform_point(&self, point: Vector4) -> Vector4 {
        self.rotation.rotate(point) + self.translation
    }

    /// Rotate a direction (no translation).
    #[inline]
    pub fn transform_vector(&self, vector: Vector4) -> Vector4 {
        self.rotation.rotate(vector)
    }

    /// Inverse point transform: `rotate_inverse(q, p - t)`.
    #[inline]
    pub fn inverse_transform_point(&self, point: Vector4) -> Vector4 {
        self.rotation.rotate_inverse(point - self.translation)
    }

    /// `hkQTransformf::setInverse`.
    pub fn inverse(&self) -> Self {
        let inverse_rotation = self.rotation.inverse();
        Self::new(inverse_rotation, -inverse_rotation.rotate(self.translation))
    }
}

impl Mul for QTransform {
    type Output = Self;

    /// `a * b` = apply `b` first, then `a` (`hkQTransformf::_setMul`).
    #[allow(clippy::suspicious_arithmetic_impl)] // affine composition is not just multiplication
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.rotation.mul(other.rotation),
            self.rotation.rotate(other.translation) + self.translation,
        )
    }
}

impl Default for QTransform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quaternion;
    use core::f32::consts::FRAC_PI_2;

    const TOL: f32 = 1e-5;

    fn make(angle_z: f32, t: Vector4) -> QTransform {
        QTransform::new(Quaternion::from_axis_angle(Vector4::UNIT_Z, angle_z), t)
    }

    #[test]
    fn point_transform() {
        let qt = make(FRAC_PI_2, Vector4::from_xyz(0.0, 1.0, 0.0));
        let p = qt.transform_point(Vector4::UNIT_X);
        assert!(p.all_equal3(Vector4::new(0.0, 2.0, 0.0, 0.0), TOL));

        let v = qt.transform_vector(Vector4::UNIT_X);
        assert!(v.all_equal3(Vector4::UNIT_Y, TOL));
    }

    #[test]
    fn multiplication_composes() {
        let a = make(FRAC_PI_2, Vector4::from_xyz(0.0, 2.0, 0.0));
        let b = make(0.0, Vector4::from_xyz(1.0, 0.0, 0.0));
        let composed = a * b;
        let p = Vector4::ZERO;
        assert!(
            composed
                .transform_point(p)
                .all_equal3(a.transform_point(b.transform_point(p)), TOL)
        );

        // Same result through the matrix representation.
        let as_matrix = composed.to_transform();
        assert!(
            as_matrix
                .transform_point(p)
                .all_equal3(composed.transform_point(p), 1e-4)
        );
    }

    #[test]
    fn inverse_round_trip() {
        let qt = make(0.6, Vector4::from_xyz(2.0, -3.0, 1.0));
        let inv = qt.inverse();
        let p = Vector4::from_xyz(1.0, 2.0, 0.0);
        assert!(
            inv.transform_point(qt.transform_point(p))
                .all_equal3(p, 1e-4)
        );
        assert!(
            qt.inverse_transform_point(qt.transform_point(p))
                .all_equal3(p, 1e-4)
        );
        assert!((qt * inv).translation.all_equal(Vector4::ZERO, 1e-4));
    }

    #[test]
    fn transform_conversion_round_trip() {
        let qt = make(0.3, Vector4::from_xyz(1.0, 2.0, 3.0));
        let t = qt.to_transform();
        let back = QTransform::from_transform(&t);
        assert!(back.translation.all_equal(qt.translation, 0.0));
        let q = back.rotation.closest_to(qt.rotation);
        assert!(q.all_equal(qt.rotation, 1e-4));
    }
}
