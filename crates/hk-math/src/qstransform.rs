//! Scale / rotation / translation transforms.
//!
//! Port of `hkQsTransformf` from
//! `original/Common/Source/Common/Base/Math/QsTransform/`.
//!
//! A point is transformed as `T(x) = R * (S * x) + t`: scale first, then rotate,
//! then translate (see the class comment in `hkQsTransformf.h`).
//!
//! Composition comes in two flavours mirroring the SDK:
//!
//! * [`QsTransform::mul`] (`setMul`) keeps scale "apart": `R1*R2`, `S1*S2`, and
//!   `t = t1 + R1*t2` (the intermediate translation is *not* scaled).
//! * [`QsTransform::mul_scaled`] (`setMulScaled`) performs the full affine
//!   composite `t = t1 + R1*(S1*t2)`; the SDK assumes uniform scale to avoid
//!   introducing skew.

use core::ops::Mul;

use crate::quaternion::Quaternion;
use crate::vector::Vector4;

/// Scale/rotation/translation transform (`hkQsTransformf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QsTransform {
    /// Translation (`m_translation`).
    pub translation: Vector4,
    /// Rotation (`m_rotation`).
    pub rotation: Quaternion,
    /// Scale (`m_scale`), component-wise.
    pub scale: Vector4,
}

impl QsTransform {
    /// Identity transform (`hkQsTransformf::setIdentity`).
    pub const IDENTITY: Self = Self {
        translation: Vector4::ZERO,
        rotation: Quaternion::IDENTITY,
        scale: Vector4::ONE,
    };

    /// Full constructor (`hkQsTransformf(t, r, s)`).
    #[inline]
    pub const fn new(translation: Vector4, rotation: Quaternion, scale: Vector4) -> Self {
        Self {
            translation,
            rotation,
            scale,
        }
    }

    /// Constructor without an explicit scale (`hkQsTransformf(t, r)`): scale is 1.
    #[inline]
    pub const fn from_translation_rotation(translation: Vector4, rotation: Quaternion) -> Self {
        Self::new(translation, rotation, Vector4::ONE)
    }

    /// `hkQsTransformf::set`.
    #[inline]
    pub fn set(&mut self, translation: Vector4, rotation: Quaternion, scale: Vector4) {
        self.translation = translation;
        self.rotation = rotation;
        self.scale = scale;
    }

    /// `hkVector4f::_setTransformedPos(qsTransform, p)`:
    /// scale, then rotate, then translate.
    #[inline]
    pub fn transform_point(&self, point: Vector4) -> Vector4 {
        let scaled = point * self.scale;
        self.rotation.rotate(scaled) + self.translation
    }

    /// Transform a direction: scale, then rotate (no translation).
    #[inline]
    pub fn transform_vector(&self, vector: Vector4) -> Vector4 {
        self.rotation.rotate(vector * self.scale)
    }

    /// `hkVector4f::_setTransformedInversePos`:
    /// subtract translation, inverse-rotate, then divide by scale.
    #[inline]
    pub fn inverse_transform_point(&self, point: Vector4) -> Vector4 {
        let unrotated = self.rotation.rotate_inverse(point - self.translation);
        unrotated / self.scale
    }

    /// `hkQsTransformf::setInverse`:
    /// `t = -(R^-1 * t)`, `q = q^-1`, `s = 1/s` (zero scale maps to zero).
    pub fn inverse(&self) -> Self {
        let inverse_rotation = self.rotation.inverse();
        let inverse_translation = -inverse_rotation.rotate(self.translation);
        let inverse_scale = Vector4::new(
            reciprocal(self.scale.x),
            reciprocal(self.scale.y),
            reciprocal(self.scale.z),
            reciprocal(self.scale.w),
        );
        Self::new(inverse_translation, inverse_rotation, inverse_scale)
    }

    /// `hkQsTransformf::setMulScaled`: full affine composite
    /// `t = t1 + R1*(S1*t2)`, `q = q1*q2`, `s = s1*s2`.
    ///
    /// The SDK documents that this assumes `S1` is uniform scaling.
    #[inline]
    pub fn mul_scaled(&self, other: &Self) -> Self {
        let translation = self.rotation.rotate(self.scale * other.translation) + self.translation;
        Self::new(
            translation,
            self.rotation.mul(other.rotation),
            self.scale * other.scale,
        )
    }

    /// `hkQsTransformf::isScaleUniform`.
    #[inline]
    pub fn is_scale_uniform(&self) -> bool {
        self.scale.y == self.scale.x && self.scale.z == self.scale.x
    }
}

/// `hkSimdFloat32::setReciprocal<HK_ACC_FULL, HK_DIV_SET_ZERO>`: `1/x` with
/// zero mapping to zero.
#[inline]
fn reciprocal(value: f32) -> f32 {
    if value == 0.0 { 0.0 } else { 1.0 / value }
}

impl Mul for QsTransform {
    type Output = Self;

    /// `a * b` = apply `b` first, then `a` (`hkQsTransformf::setMul`):
    /// `t = t1 + R1*t2`, `q = q1*q2`, `s = s1*s2`.
    #[allow(clippy::suspicious_arithmetic_impl)] // affine composition is not just multiplication
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.translation + self.rotation.rotate(other.translation),
            self.rotation.mul(other.rotation),
            self.scale * other.scale,
        )
    }
}

impl Default for QsTransform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector4;
    use core::f32::consts::FRAC_PI_2;

    const TOL: f32 = 1e-5;

    #[test]
    fn scale_then_rotate_then_translate() {
        let qs = QsTransform::new(
            Vector4::from_xyz(1.0, 2.0, 0.0),
            Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2),
            Vector4::new(2.0, 3.0, 1.0, 1.0),
        );

        // (1,0,0) scaled by (2,3,1) -> (2,0,0), rotated 90° -> (0,2,0), +t -> (1,4,0).
        let p = qs.transform_point(Vector4::UNIT_X);
        assert!(p.all_equal3(Vector4::new(1.0, 4.0, 0.0, 0.0), TOL));

        let v = qs.transform_vector(Vector4::UNIT_X);
        assert!(v.all_equal3(Vector4::new(0.0, 2.0, 0.0, 0.0), TOL));
    }

    #[test]
    fn inverse_round_trip() {
        // Pure rotation + translation (scale = 1) round-trips through `setInverse`.
        let rigid = QsTransform::new(
            Vector4::from_xyz(1.0, -2.0, 3.0),
            Quaternion::from_axis_angle(Vector4::UNIT_X, 0.5),
            Vector4::ONE,
        );
        let inverse = rigid.inverse();
        let p = Vector4::from_xyz(1.0, 2.0, 3.0);
        assert!(
            inverse
                .transform_point(rigid.transform_point(p))
                .all_equal3(p, 1e-4)
        );

        // `setInverse` stores `s^-1` and `t^-1 = -(R^-1 * t)`; the translation is
        // not rescaled (the C++ formula does not divide by the scale).
        let scaled = QsTransform::new(
            Vector4::from_xyz(1.0, -2.0, 3.0),
            Quaternion::from_axis_angle(Vector4::UNIT_X, 0.5),
            Vector4::splat(2.0),
        );
        let scaled_inverse = scaled.inverse();
        assert_eq!(scaled_inverse.scale, Vector4::splat(0.5));
        assert!(
            scaled_inverse
                .translation
                .all_equal(-scaled.rotation.inverse().rotate(scaled.translation), 1e-5)
        );

        // The exact inverse of the non-uniform transform is
        // `inverse_transform_point` (subtract t, rotate^-1, divide by scale).
        let non_uniform = QsTransform::new(
            Vector4::from_xyz(1.0, -2.0, 3.0),
            Quaternion::from_axis_angle(Vector4::UNIT_X, 0.5),
            Vector4::new(2.0, 3.0, 4.0, 1.0),
        );
        assert!(
            non_uniform
                .inverse_transform_point(non_uniform.transform_point(p))
                .all_equal3(p, 1e-4)
        );
    }

    #[test]
    fn compose_with_uniform_scale() {
        // `setMul` ignores the intermediate translation when the left-hand side
        // has a scale: `t = t1 + R1*t2` (not `t1 + R1*S1*t2`).
        let a = QsTransform::new(
            Vector4::from_xyz(0.0, 2.0, 0.0),
            Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2),
            Vector4::splat(2.0),
        );
        let b = QsTransform::from_translation_rotation(
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Quaternion::IDENTITY,
        );
        let p = Vector4::ZERO;

        let manual = a.transform_point(b.transform_point(p));
        let set_mul = a * b;
        assert!(
            !set_mul.transform_point(p).all_equal3(manual, TOL),
            "setMul does not scale the intermediate translation"
        );

        let set_mul_scaled = a.mul_scaled(&b);
        assert!(set_mul_scaled.transform_point(p).all_equal3(manual, TOL));

        // With unit scale both write the same transform.
        let a_unit_scale = QsTransform::new(a.translation, a.rotation, Vector4::ONE);
        let composed = a_unit_scale * b;
        assert!(
            composed
                .transform_point(p)
                .all_equal3(a_unit_scale.transform_point(b.transform_point(p)), TOL)
        );
    }

    #[test]
    fn mul_does_not_scale_intermediate_translation() {
        let a = QsTransform::new(Vector4::ZERO, Quaternion::IDENTITY, Vector4::splat(2.0));
        let b = QsTransform::from_translation_rotation(
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Quaternion::IDENTITY,
        );

        // setMul: t = t1 + R1*t2 = (1,0,0) (scale NOT applied to t2).
        let mul = a * b;
        assert_eq!(mul.translation, Vector4::from_xyz(1.0, 0.0, 0.0));

        // setMulScaled: t = t1 + R1*(S1*t2) = (2,0,0).
        let mul_scaled = a.mul_scaled(&b);
        assert_eq!(mul_scaled.translation, Vector4::from_xyz(2.0, 0.0, 0.0));

        assert_eq!(mul.scale, Vector4::splat(2.0));
        assert_eq!(mul_scaled.scale, Vector4::splat(2.0));
    }

    #[test]
    fn uniform_scale_detection() {
        assert!(
            QsTransform::new(Vector4::ZERO, Quaternion::IDENTITY, Vector4::splat(3.0))
                .is_scale_uniform()
        );
        assert!(
            !QsTransform::new(
                Vector4::ZERO,
                Quaternion::IDENTITY,
                Vector4::new(1.0, 2.0, 1.0, 1.0)
            )
            .is_scale_uniform()
        );
    }
}
