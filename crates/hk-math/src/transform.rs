//! Rigid transforms: rotation matrix plus translation.
//!
//! Port of `hkTransformf` from
//! `original/Common/Source/Common/Base/Math/Matrix/`.
//!
//! The C++ class stores a `hkRotationf` (a 3×3 rotation matrix, `m_rotation`)
//! followed by a translation vector (`m_translation`). Composition is
//! `(R1, t1) * (R2, t2) = (R1*R2, R1*t2 + t1)`, matching
//! `hkTransformUtil::_mulTransformTransform`.
//!
//! ```
//! use hk_math::{Transform, Quaternion, Vector4};
//! use core::f32::consts::FRAC_PI_2;
//!
//! let rotate_x_to_y = Transform::from_quaternion(
//!     Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2),
//!     Vector4::from_xyz(1.0, 0.0, 0.0),
//! );
//! let p = rotate_x_to_y.transform_point(Vector4::UNIT_X);
//! assert!(p.all_equal3(Vector4::new(1.0, 1.0, 0.0, 0.0), 1e-5));
//! ```

use core::ops::Mul;

use crate::matrix3::Matrix3;
use crate::quaternion::Quaternion;
use crate::vector::Vector4;

/// Rigid transform (`hkTransformf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// Rotation (`hkRotationf`).
    pub rotation: Matrix3,
    /// Translation (`m_translation`).
    pub translation: Vector4,
}

impl Transform {
    /// Identity transform (`hkTransformf::setIdentity`).
    pub const IDENTITY: Self = Self {
        rotation: Matrix3::IDENTITY,
        translation: Vector4::ZERO,
    };

    /// Build from a rotation matrix and a translation.
    #[inline]
    pub const fn new(rotation: Matrix3, translation: Vector4) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Build from a quaternion and a translation (`hkTransformf(q, t)`).
    #[inline]
    pub fn from_quaternion(rotation: Quaternion, translation: Vector4) -> Self {
        Self::new(Matrix3::from_quaternion(rotation), translation)
    }

    /// `hkTransformf::set(q, t)`.
    #[inline]
    pub fn set_quaternion(&mut self, rotation: Quaternion, translation: Vector4) {
        self.rotation = Matrix3::from_quaternion(rotation);
        self.translation = translation;
    }

    /// `hkTransformf::setRotation(quaternion)`.
    #[inline]
    pub fn set_rotation_quaternion(&mut self, rotation: Quaternion) {
        self.rotation = Matrix3::from_quaternion(rotation);
    }

    /// `hkTransformf::getRotation`.
    #[inline]
    pub fn rotation(&self) -> &Matrix3 {
        &self.rotation
    }

    /// `hkTransformf::getTranslation`.
    #[inline]
    pub const fn translation(&self) -> Vector4 {
        self.translation
    }

    /// `hkVector4f::setTransformedPos(transform, p)` — `R * p + t`.
    #[inline]
    pub fn transform_point(&self, point: Vector4) -> Vector4 {
        self.rotation.mul_vec(point) + self.translation
    }

    /// Rotate a direction (`R * v`, no translation).
    #[inline]
    pub fn transform_vector(&self, vector: Vector4) -> Vector4 {
        self.rotation.mul_vec(vector)
    }

    /// `hkVector4f::setTransformedInversePos` — `R^T * (p - t)`.
    #[inline]
    pub fn inverse_transform_point(&self, point: Vector4) -> Vector4 {
        self.rotation.rotate_inverse(point - self.translation)
    }

    /// Inverse transform (`hkTransformf::setInverse`):
    /// rotation becomes the transpose, translation becomes `R^T * (-t)`.
    pub fn inverse(&self) -> Self {
        let inverse_rotation = self.rotation.transpose();
        Self::new(
            inverse_rotation,
            inverse_rotation.mul_vec(-self.translation),
        )
    }

    /// Rotation part converted back to a quaternion
    /// (`hkQuaternionf::set(hkRotationf)`).
    #[inline]
    pub fn rotation_quaternion(&self) -> Quaternion {
        Quaternion::from_matrix(&self.rotation)
    }
}

impl Mul for Transform {
    type Output = Self;

    /// `a * b` = apply `b` first, then `a` (`hkTransformf::setMul`).
    #[allow(clippy::suspicious_arithmetic_impl)] // affine composition is not just multiplication
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.rotation.mul_mat(&other.rotation),
            self.rotation.mul_vec(other.translation) + self.translation,
        )
    }
}

impl Default for Transform {
    /// Identity transform (the C++ default constructor leaves storage uninitialized).
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    const TOL: f32 = 1e-5;

    fn translation_rotation(angle_z: f32, t: Vector4) -> Transform {
        Transform::from_quaternion(Quaternion::from_axis_angle(Vector4::UNIT_Z, angle_z), t)
    }

    #[test]
    fn identity() {
        let t = Transform::IDENTITY;
        assert_eq!(t, Transform::default());
        assert!(
            t.transform_point(Vector4::UNIT_X)
                .all_equal3(Vector4::UNIT_X, 0.0)
        );
        assert!(
            t.transform_vector(Vector4::UNIT_Y)
                .all_equal3(Vector4::UNIT_Y, 0.0)
        );
    }

    #[test]
    fn point_and_vector_transforms() {
        let t = translation_rotation(FRAC_PI_2, Vector4::from_xyz(1.0, 0.0, 0.0));
        let p = t.transform_point(Vector4::UNIT_X);
        assert!(p.all_equal3(Vector4::new(1.0, 1.0, 0.0, 0.0), TOL));
        assert_eq!(p.w, 0.0, "w component follows the C++ rotate+add");

        let v = t.transform_vector(Vector4::UNIT_X);
        assert!(v.all_equal3(Vector4::UNIT_Y, TOL));
    }

    #[test]
    fn composition_order() {
        let a = translation_rotation(FRAC_PI_2, Vector4::from_xyz(0.0, 2.0, 0.0));
        let b = translation_rotation(0.0, Vector4::from_xyz(1.0, 0.0, 0.0));

        let composed = a * b;
        let p = Vector4::from_xyz(0.0, 0.0, 0.0);
        let direct = a.transform_point(b.transform_point(p));
        assert!(composed.transform_point(p).all_equal3(direct, TOL));

        // Rotation parts compose as matrices.
        let expected_rotation = a.rotation.mul_mat(&b.rotation);
        assert!(
            composed
                .rotation
                .col0
                .all_equal(expected_rotation.col0, TOL)
        );
    }

    #[test]
    fn inverse_round_trip() {
        let t = translation_rotation(0.7, Vector4::from_xyz(3.0, -2.0, 1.0));
        let inv = t.inverse();
        let p = Vector4::from_xyz(1.0, 2.0, 3.0);
        assert!(
            inv.transform_point(t.transform_point(p))
                .all_equal3(p, 1e-4)
        );
        assert!(
            t.transform_point(inv.transform_point(p))
                .all_equal3(p, 1e-4)
        );
        assert!(
            t.inverse_transform_point(t.transform_point(p))
                .all_equal3(p, 1e-4)
        );

        // Composition with the inverse is the identity transform.
        let identity = t * inv;
        assert!(identity.rotation.col0.all_equal3(Vector4::UNIT_X, 1e-4));
        assert!(identity.rotation.col1.all_equal3(Vector4::UNIT_Y, 1e-4));
        assert!(identity.rotation.col2.all_equal3(Vector4::UNIT_Z, 1e-4));
        assert!(identity.translation.all_equal(Vector4::ZERO, 1e-4));
    }

    #[test]
    fn quaternion_conversion() {
        let q = Quaternion::from_axis_angle(Vector4::UNIT_X, 0.3);
        let t = Transform::from_quaternion(q, Vector4::ZERO);
        let back = t.rotation_quaternion().closest_to(q);
        assert!(back.all_equal(q, 1e-4));

        let mask = t.transform_vector(Vector4::UNIT_Y).equal(Vector4::UNIT_Y);
        // Rotation about X leaves x and w untouched (bits 0 and 3), and moves y/z.
        assert_eq!(mask.bits(), 0b1001);
    }
}
