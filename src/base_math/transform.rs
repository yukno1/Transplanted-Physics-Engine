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
//! use proton::math::{Quat, Transform, Vec4};
//! use core::f32::consts::FRAC_PI_2;
//!
//! let rotate_x_to_y = Transform::from_quat(
//!     Quat::from_axis_angle(glam::Vec3::Z, FRAC_PI_2),
//!     Vec4::new(1.0, 0.0, 0.0, 0.0),
//! );
//! let p = rotate_x_to_y.transform_point(Vec4::X);
//! assert!(p.abs_diff_eq(Vec4::new(1.0, 1.0, 0.0, 0.0), 1e-5));
//! ```

use core::ops::Mul;

use glam::{Mat3, Quat, Vec4};

/// Rigid transform (`hkTransformf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// Rotation (`hkRotationf`).
    pub rotation: Mat3,
    /// Translation (`m_translation`).
    pub translation: Vec4,
}

impl Transform {
    /// Identity transform (`hkTransformf::setIdentity`).
    pub const IDENTITY: Self = Self {
        rotation: Mat3::IDENTITY,
        translation: Vec4::ZERO,
    };

    /// Build from a rotation matrix and a translation.
    #[inline]
    pub const fn new(rotation: Mat3, translation: Vec4) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Build from a quaternion and a translation (`hkTransformf(q, t)`).
    #[inline]
    pub fn from_quat(rotation: Quat, translation: Vec4) -> Self {
        Self::new(Mat3::from_quat(rotation), translation)
    }

    /// `hkTransformf::set(q, t)`.
    #[inline]
    pub fn set_quaternion(&mut self, rotation: Quat, translation: Vec4) {
        self.rotation = Mat3::from_quat(rotation);
        self.translation = translation;
    }

    /// `hkTransformf::setRotation(quaternion)`.
    #[inline]
    pub fn set_rotation_quaternion(&mut self, rotation: Quat) {
        self.rotation = Mat3::from_quat(rotation);
    }

    /// `hkTransformf::getRotation`.
    #[inline]
    pub fn rotation(&self) -> &Mat3 {
        &self.rotation
    }

    /// `hkTransformf::getTranslation`.
    #[inline]
    pub const fn translation(&self) -> Vec4 {
        self.translation
    }

    /// `hkVec4f::setTransformedPos(transform, p)` — `R * p + t`.
    #[inline]
    pub fn transform_point(&self, point: Vec4) -> Vec4 {
        (self.rotation.mul_vec3(point.truncate()).extend(0.0)) + self.translation
    }

    /// Rotate a direction (`R * v`, no translation).
    #[inline]
    pub fn transform_vector(&self, vector: Vec4) -> Vec4 {
        self.rotation.mul_vec3(vector.truncate()).extend(0.0)
    }

    /// `hkVec4f::setTransformedInversePos` — `R^T * (p - t)`.
    #[inline]
    pub fn inverse_transform_point(&self, point: Vec4) -> Vec4 {
        self.rotation
            .transpose()
            .mul_vec3((point - self.translation).truncate())
            .extend(0.0)
    }

    /// Inverse transform (`hkTransformf::setInverse`):
    /// rotation becomes the transpose, translation becomes `R^T * (-t)`.
    pub fn inverse(&self) -> Self {
        let inverse_rotation = self.rotation.transpose();
        Self::new(
            inverse_rotation,
            inverse_rotation
                .mul_vec3((-self.translation).truncate())
                .extend(0.0),
        )
    }

    /// Rotation part converted back to a quaternion
    /// (`hkQuaternionf::set(hkRotationf)`).
    #[inline]
    pub fn rotation_quaternion(&self) -> Quat {
        Quat::from_mat3(&self.rotation)
    }
}

impl Mul for Transform {
    type Output = Self;

    /// `a * b` = apply `b` first, then `a` (`hkTransformf::setMul`).
    #[allow(clippy::suspicious_arithmetic_impl)] // affine composition is not just multiplication
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.rotation * other.rotation,
            self.rotation
                .mul_vec3(other.translation.truncate())
                .extend(0.0)
                + self.translation,
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
    use glam::Vec3;
    use core::f32::consts::FRAC_PI_2;

    const TOL: f32 = 1e-5;

    fn translation_rotation(angle_z: f32, t: Vec4) -> Transform {
        Transform::from_quat(Quat::from_axis_angle(Vec3::Z, angle_z), t)
    }

    #[test]
    fn identity() {
        let t = Transform::IDENTITY;
        assert_eq!(t, Transform::default());
        assert!(t.transform_point(Vec4::X).abs_diff_eq(Vec4::X, 0.0));
        assert!(t.transform_vector(Vec4::Y).abs_diff_eq(Vec4::Y, 0.0));
    }

    #[test]
    fn point_and_vector_transforms() {
        let t = translation_rotation(FRAC_PI_2, Vec4::new(1.0, 0.0, 0.0, 0.0));
        let p = t.transform_point(Vec4::X);
        assert!(p.abs_diff_eq(Vec4::new(1.0, 1.0, 0.0, 0.0), TOL));
        assert_eq!(p.w, 0.0, "w component follows the C++ rotate+add");

        let v = t.transform_vector(Vec4::X);
        assert!(v.abs_diff_eq(Vec4::Y, TOL));
    }

    #[test]
    fn composition_order() {
        let a = translation_rotation(FRAC_PI_2, Vec4::new(0.0, 2.0, 0.0, 0.0));
        let b = translation_rotation(0.0, Vec4::new(1.0, 0.0, 0.0, 0.0));

        let composed = a * b;
        let p = Vec4::from((0.0, 0.0, 0.0, 0.0));
        let direct = a.transform_point(b.transform_point(p));
        assert!(composed.transform_point(p).abs_diff_eq(direct, TOL));

        // Rotation parts compose as matrices.
        let expected_rotation = a.rotation * b.rotation;
        assert!(
            composed
                .rotation
                .x_axis
                .abs_diff_eq(expected_rotation.x_axis, TOL)
        );
    }

    #[test]
    fn inverse_round_trip() {
        let t = translation_rotation(0.7, Vec4::new(3.0, -2.0, 1.0, 0.0));
        let inv = t.inverse();
        let p = Vec4::new(1.0, 2.0, 3.0, 0.0);
        assert!(
            inv.transform_point(t.transform_point(p))
                .abs_diff_eq(p, 1e-4)
        );
        assert!(
            t.transform_point(inv.transform_point(p))
                .abs_diff_eq(p, 1e-4)
        );
        assert!(
            t.inverse_transform_point(t.transform_point(p))
                .abs_diff_eq(p, 1e-4)
        );

        // Composition with the inverse is the identity transform.
        let identity = t * inv;
        assert!(identity.rotation.x_axis.abs_diff_eq(Vec3::X, 1e-4));
        assert!(identity.rotation.y_axis.abs_diff_eq(Vec3::Y, 1e-4));
        assert!(identity.rotation.z_axis.abs_diff_eq(Vec3::Z, 1e-4));
        assert!(identity.translation.abs_diff_eq(Vec4::ZERO, 1e-4));
    }

    #[test]
    fn quaternion_conversion() {
        let q = Quat::from_axis_angle(Vec3::X, 0.3);
        let t = Transform::from_quat(q, Vec4::ZERO);
        let back = t.rotation_quaternion();
        // `q` and `-q` represent the same rotation; compare their action.
        assert!(
            back.mul_vec3(Vec3::Y)
                .abs_diff_eq(q.mul_vec3(Vec3::Y), 1e-4)
        );

        let mask = t.transform_vector(Vec4::Y).cmpeq(Vec4::Y);
        // Rotation about X leaves x and w untouched (bits 0 and 3), and moves y/z.
        assert_eq!(mask.bitmask(), 0b1001);
    }
}
