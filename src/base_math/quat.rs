//! glam-backed quaternion helpers for [`Quat`].
//!
//! Previously this module ported the C++ `Fpu` scalar formulas verbatim
//! (`docs/refactor-glam-plan.md`, 方案 B). As part of the glam migration the
//! implementations now delegate to glam's native quaternion operations
//! (`mul_vec3`, `inverse`, `from_mat3`, `from_quat`, `normalize_or_zero`,
//! `Mul`); only the thin `Vec4`-lane boundary (`w` passthrough) is kept so
//! existing `Vec4`-based callers keep compiling.

use glam::{Mat3, Quat, Vec4};

/// glam quaternion helpers for [`Quat`].
pub trait QuatExt: Sized {
    /// Rotate a direction by this quaternion (`Quat::mul_vec3`); assumes the
    /// quaternion is normalized. The `w` lane passes through unchanged.
    fn rotate_sdk(self, direction: Vec4) -> Vec4;

    /// Rotate a direction by the inverse rotation (`inverse` + `mul_vec3`).
    fn rotate_inverse_sdk(self, direction: Vec4) -> Vec4;

    /// Build a quaternion from a rotation matrix (`Quat::from_mat3`).
    fn from_matrix_sdk(rotation: Mat3) -> Self;

    /// Build the rotation matrix of this quaternion (`Mat3::from_quat`).
    fn to_matrix_sdk(self) -> Mat3;
}

/// Hamilton product (`Quat * Quat`).
#[inline]
pub fn quat_mul(q0: Quat, q1: Quat) -> Quat {
    q0 * q1
}

/// `Quat::normalize` with a zero guard — zero maps to zero, matching the
/// legacy `hkQuaternionf::normalize<HK_SQRT_SET_ZERO>` behavior.
#[inline]
pub fn quat_normalize(q: Quat) -> Quat {
    if q.length_squared() == 0.0 {
        Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)
    } else {
        q.normalize()
    }
}

impl QuatExt for Quat {
    #[inline]
    fn rotate_sdk(self, direction: Vec4) -> Vec4 {
        self.mul_vec3(direction.truncate()).extend(direction.w)
    }

    #[inline]
    fn rotate_inverse_sdk(self, direction: Vec4) -> Vec4 {
        self.inverse()
            .mul_vec3(direction.truncate())
            .extend(direction.w)
    }

    #[inline]
    fn from_matrix_sdk(rotation: Mat3) -> Self {
        Quat::from_mat3(&rotation)
    }

    #[inline]
    fn to_matrix_sdk(self) -> Mat3 {
        Mat3::from_quat(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    #[test]
    fn rotate_inverse_undoes_rotate() {
        let q = Quat::from_axis_angle(
            glam::Vec3::new(1.0, 2.0, 2.0).normalize(),
            core::f32::consts::FRAC_PI_3,
        );
        let v = Vec4::new(1.0, -2.0, 0.5, 0.0);
        let back = q.rotate_inverse_sdk(q.rotate_sdk(v));
        assert!(back.abs_diff_eq(v, 1e-5));
    }

    #[test]
    fn rotate_matches_glam_vec3_rotation() {
        let q = Quat::from_axis_angle(glam::Vec3::Z, 0.7);
        let v = Vec4::new(1.0, -2.0, 0.5, 3.0);
        let rotated = q.rotate_sdk(v);
        let expected = q.mul_vec3(v.truncate());
        assert!(rotated.truncate().abs_diff_eq(expected, 0.0));
        assert_eq!(rotated.w, v.w, "w lane passes through");
    }

    #[test]
    fn identity_rotation_is_noop() {
        let q = Quat::IDENTITY;
        let v = Vec4::new(1.0, -2.0, 0.5, 3.0);
        assert!(q.rotate_sdk(v).abs_diff_eq(v, 0.0));
        assert!(q.rotate_inverse_sdk(v).abs_diff_eq(v, 0.0));
    }

    #[test]
    fn quarter_turn_about_z_maps_x_to_y() {
        let q = Quat::from_axis_angle(glam::Vec3::Z, FRAC_PI_2);
        let rotated = q.rotate_sdk(Vec4::new(1.0, 0.0, 0.0, 0.0));
        assert!(rotated.abs_diff_eq(Vec4::new(0.0, 1.0, 0.0, 0.0), 1e-6));
    }

    #[test]
    fn matrix_round_trip() {
        let q = Quat::from_axis_angle(
            glam::Vec3::new(1.0, 2.0, 2.0).normalize(),
            core::f32::consts::FRAC_PI_3,
        );
        let back = Quat::from_matrix_sdk(q.to_matrix_sdk());
        // `q` and `-q` represent the same rotation; compare their action.
        assert!(back.mul_vec3(glam::Vec3::Y).abs_diff_eq(q.mul_vec3(glam::Vec3::Y), 1e-5));
    }

    #[test]
    fn quat_mul_matches_glam_product() {
        let a = Quat::from_axis_angle(glam::Vec3::X, 0.7);
        let b = Quat::from_axis_angle(glam::Vec3::Z, 0.3);
        assert_eq!(quat_mul(a, b), a * b);
    }

    #[test]
    fn quat_normalize_zero_maps_to_zero() {
        let zero = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
        assert_eq!(quat_normalize(zero), zero);
        let q = Quat::from_xyzw(1.0, 2.0, 3.0, 4.0);
        let n = quat_normalize(q);
        let length = (n.x * n.x + n.y * n.y + n.z * n.z + n.w * n.w).sqrt();
        assert!((length - 1.0).abs() < 1e-6);
    }
}
