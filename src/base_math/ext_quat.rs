//! SDK-parity scalar-order helpers for `glam::Quat`.
//!
//! Formulas ported verbatim from the legacy `Quaternion` implementation
//! (`src/base_math/quaternion.rs`) so glam-backed callers keep bit-parity with
//! the C++ `Fpu` formulations (`docs/refactor-glam-plan.md`, 方案 B). glam's
//! `Quat::mul_quat` / `mul_vec3` use different (fused/SIMD) operation orders,
//! so rotation-hot callers must use these methods. Source line numbers in the
//! docs refer to the legacy file before the glam migration.

use glam::{Mat3, Quat, Vec4};

use crate::base_math::Vec4Ext;

/// `hkQuaternionf` scalar-order helpers for [`Quat`].
pub trait QuatExt: Sized {
    /// `hkVector4f::_setRotatedDir(quat, direction)` — rotate a direction by
    /// this quaternion; assumes the quaternion is normalized
    /// (legacy `Quaternion::rotate`, `quaternion.rs:195-205`):
    ///
    /// ```text
    /// r = 2 * ( v*(w*w - 0.5) + imag*dot(imag,v) + cross(imag,v)*w )
    /// ```
    fn rotate_sdk(self, direction: Vec4) -> Vec4;

    /// `hkVector4f::_setRotatedInverseDir(quat, direction)` — rotate by the
    /// inverse rotation (legacy `Quaternion::rotate_inverse`,
    /// `quaternion.rs:207-217`).
    fn rotate_inverse_sdk(self, direction: Vec4) -> Vec4;

    /// `hkQuaternionf::set(hkRotationf)` via
    /// `hkQuaternionUtil::convertRotationToUnnormalizedQuaternion`: 4-branch
    /// trace algorithm + post normalization
    /// (legacy `Quaternion::from_matrix`, `quaternion.rs:219-260`).
    fn from_matrix_sdk(rotation: Mat3) -> Self;
}

/// Hamilton product with the SDK's expansion order (`setMul`).
pub fn quat_mul(q0: Quat, q1: Quat) -> Quat {
    Quat::from_xyzw(
        q0.x * q1.w + q0.w * q1.x + q0.y * q1.z - q0.z * q1.y,
        q0.y * q1.w + q0.w * q1.y + q0.z * q1.x - q0.x * q1.z,
        q0.z * q1.w + q0.w * q1.z + q0.x * q1.y - q0.y * q1.x,
        q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z,
    )
}

/// `hkQuaternionf::normalize<HK_SQRT_SET_ZERO>` — zero maps to zero.
pub fn quat_normalize(mut q: Quat) -> Quat {
    let length = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    if length == 0.0 {
        return Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
    }
    q.x /= length;
    q.y /= length;
    q.z /= length;
    q.w /= length;
    q
}

/// `hkVector4f::setCross` — two commutators, zero `w`
/// (legacy `Vector4::set_cross`, `vector.rs:305-315`).
fn cross3_w0(a: Vec4, b: Vec4) -> Vec4 {
    Vec4::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
        0.0,
    )
}

impl QuatExt for Quat {
    #[inline]
    fn rotate_sdk(self, direction: Vec4) -> Vec4 {
        let q2_minus_1 = self.w * self.w - 0.5;
        let mut result = direction * q2_minus_1;
        let imag = Vec4::new(self.x, self.y, self.z, 0.0);
        result += imag * imag.dot3(direction);
        result += cross3_w0(imag, direction) * self.w;
        result + result
    }

    #[inline]
    fn rotate_inverse_sdk(self, direction: Vec4) -> Vec4 {
        let q2_minus_1 = self.w * self.w - 0.5;
        let mut result = direction * q2_minus_1;
        let imag = Vec4::new(self.x, self.y, self.z, 0.0);
        result += imag * imag.dot3(direction);
        result += cross3_w0(direction, imag) * self.w;
        result + result
    }

    fn from_matrix_sdk(rotation: Mat3) -> Self {
        let c0 = rotation.x_axis;
        let c1 = rotation.y_axis;
        let c2 = rotation.z_axis;

        let m00 = c0.x;
        let m11 = c1.y;
        let m22 = c2.z;
        let m01 = c0.y;
        let m10 = c1.x;
        let m02 = c0.z;
        let m20 = c2.x;
        let m12 = c1.z;
        let m21 = c2.y;

        let q = if m22 < 0.0 {
            let m01_p_10 = m01 + m10;
            let m00_m_11 = m00 - m11;
            let t = 1.0 - m22;
            if m00 > m11 {
                Quat::from_xyzw(t + m00_m_11, m01_p_10, m20 + m02, m12 - m21)
            } else {
                Quat::from_xyzw(m01_p_10, t - m00_m_11, m12 + m21, m20 - m02)
            }
        } else {
            let t = 1.0 + m22;
            let m00_p_11 = m00 + m11;
            let m01_m_10 = m01 - m10;
            if m00 < -m11 {
                Quat::from_xyzw(m20 + m02, m12 + m21, t - m00_p_11, m01_m_10)
            } else {
                Quat::from_xyzw(m12 - m21, m20 - m02, m01_m_10, t + m00_p_11)
            }
        };

        // The C++ helper deliberately skips the 0.5/sqrt(t) scaling here and
        // normalizes afterwards.
        quat_normalize(q)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::{Quaternion, Vector4};
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_3, PI};

    fn to_legacy(q: Quat) -> Quaternion {
        Quaternion::new(q.x, q.y, q.z, q.w)
    }

    #[test]
    fn rotate_sdk_matches_legacy_quaternion() {
        let axis = Vec4::new(1.0, 2.0, 2.0, 0.0).normalize_or_zero3();
        let angle = FRAC_PI_3;
        let legacy = Quaternion::from_axis_angle(Vector4::new(axis.x, axis.y, axis.z, 0.0), angle);
        let glam_q = Quat::from_axis_angle(glam::Vec3::new(axis.x, axis.y, axis.z), angle);
        let direction = Vec4::new(1.0, -2.0, 0.5, 0.0);

        let legacy_rotated = legacy.rotate(Vector4::new(
            direction.x,
            direction.y,
            direction.z,
            direction.w,
        ));
        let glam_rotated = glam_q.rotate_sdk(direction);
        for i in 0..4 {
            assert_eq!(
                glam_rotated.to_array()[i].to_bits(),
                legacy_rotated.to_array()[i].to_bits(),
                "component {i} must match the legacy rotation bit-for-bit"
            );
        }
    }

    #[test]
    fn rotate_inverse_undoes_rotate() {
        let axis = glam::Vec3::new(1.0, 2.0, 2.0).normalize();
        let q = Quat::from_axis_angle(axis, FRAC_PI_3);
        let v = Vec4::new(1.0, -2.0, 0.5, 0.0);
        let back = q.rotate_inverse_sdk(q.rotate_sdk(v));
        assert!((back.x - v.x).abs() < 1e-5);
        assert!((back.y - v.y).abs() < 1e-5);
        assert!((back.z - v.z).abs() < 1e-5);
    }

    #[test]
    fn from_matrix_sdk_matches_legacy() {
        for axis in [
            glam::Vec3::X,
            glam::Vec3::Y,
            glam::Vec3::Z,
            glam::Vec3::new(1.0, 2.0, 2.0).normalize(),
        ] {
            for &angle in &[0.4, FRAC_PI_2, PI] {
                let legacy =
                    Quaternion::from_axis_angle(Vector4::new(axis.x, axis.y, axis.z, 0.0), angle);
                let m = legacy.to_matrix();
                let glam_m = Mat3::from_cols(
                    glam::Vec3::new(m.col0.x, m.col0.y, m.col0.z),
                    glam::Vec3::new(m.col1.x, m.col1.y, m.col1.z),
                    glam::Vec3::new(m.col2.x, m.col2.y, m.col2.z),
                );
                let converted = Quat::from_matrix_sdk(glam_m);
                let legacy_converted = Quaternion::from_matrix(&m);
                assert!(
                    to_legacy(converted).all_equal(legacy_converted, 1e-6),
                    "axis {axis:?} angle {angle}: {converted:?} vs {legacy_converted:?}"
                );
            }
        }
    }

    #[test]
    fn quat_mul_matches_legacy_hamilton_product() {
        let a = Quat::from_axis_angle(glam::Vec3::X, 0.7);
        let b = Quat::from_axis_angle(glam::Vec3::Z, 0.3);
        let legacy = Quaternion::new(a.x, a.y, a.z, a.w).mul(Quaternion::new(b.x, b.y, b.z, b.w));
        let product = quat_mul(a, b);
        assert!(
            to_legacy(product).all_equal(legacy, 0.0),
            "Hamilton product must match bit-for-bit"
        );
    }
}
