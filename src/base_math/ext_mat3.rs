//! SDK-parity scalar-order helpers for `glam::Mat3`.
//!
//! Formulas ported verbatim from the legacy `Matrix3` implementation
//! (`src/base_math/matrix3.rs`) so glam-backed callers keep bit-parity with
//! the C++ `Fpu` formulations (`docs/refactor-glam-plan.md`, 方案 B). glam's
//! `Mat3::mul_vec3` may reorder the column reduction on SIMD targets, so
//! hot callers use [`Mat3Ext::mul_vec_sdk`]. Source line numbers in the docs
//! refer to the legacy file before the glam migration.

use glam::{Mat3, Vec3};

/// `hkMatrix3f` scalar-order helpers for [`Mat3`].
pub trait Mat3Ext: Sized {
    /// `hkMatrix3f::multiplyVector` — `self * v` with the C++ operation order
    /// `v.x * col0 + v.y * col1 + v.z * col2`
    /// (legacy `Matrix3::mul_vec`, `matrix3.rs:112-123`).
    fn mul_vec_sdk(self, v: Vec3) -> Vec3;

    /// `hkMatrix3f::setInverse` with the default determinant check and
    /// `HK_INVERSION_FAILURE_SET_ZERO` behavior — returns `(inverse, determinant)`;
    /// a singular matrix yields a zero matrix and its determinant
    /// (legacy `Matrix3::inverse`, `matrix3.rs:178-216`).
    fn inverse_sdk(self) -> (Self, f32);
}

impl Mat3Ext for Mat3 {
    #[inline]
    fn mul_vec_sdk(self, v: Vec3) -> Vec3 {
        let mut result = self.x_axis * v.x;
        result += self.y_axis * v.y;
        result += self.z_axis * v.z;
        result
    }

    fn inverse_sdk(self) -> (Self, f32) {
        let c0 = self.x_axis;
        let c1 = self.y_axis;
        let c2 = self.z_axis;

        let r0 = c1.cross(c2);
        let r1 = c2.cross(c0);
        let r2 = c0.cross(c1);

        let determinant = c0.dot(r0);

        // Scaled epsilon check from hkMatrix3::_setInverse:
        //   eps = feps * L1(c0) * ( L1(c1) * L1(c2) )
        let abs_det = determinant.abs();
        let feps = f32::EPSILON;
        let eps = (feps * (c0.x.abs() + c0.y.abs() + c0.z.abs()))
            * ((c1.x.abs() + c1.y.abs() + c1.z.abs()) * (c2.x.abs() + c2.y.abs() + c2.z.abs()));
        let is_not_singular = abs_det > eps;

        let determinant_inverse = if is_not_singular {
            1.0 / determinant
        } else {
            0.0
        };

        // Columns of the inverse are the transposed cofactor vectors.
        let inv0 = Vec3::new(r0.x, r1.x, r2.x);
        let inv1 = Vec3::new(r0.y, r1.y, r2.y);
        let inv2 = Vec3::new(r0.z, r1.z, r2.z);

        (
            Mat3::from_cols(
                inv0 * determinant_inverse,
                inv1 * determinant_inverse,
                inv2 * determinant_inverse,
            ),
            determinant,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: Mat3, b: Mat3) -> bool {
        a.x_axis.abs_diff_eq(b.x_axis, 1e-5)
            && a.y_axis.abs_diff_eq(b.y_axis, 1e-5)
            && a.z_axis.abs_diff_eq(b.z_axis, 1e-5)
    }

    #[test]
    fn inverse_sdk_round_trips() {
        let a = Mat3::from_cols(
            Vec3::new(2.0, 1.0, 0.0),
            Vec3::new(1.0, 3.0, 1.0),
            Vec3::new(0.0, 1.0, 2.0),
        );
        let (inv, det) = a.inverse_sdk();
        assert!((det - 8.0).abs() < 1e-6, "det = {det}");
        assert!(a.mul_vec_sdk(inv.x_axis).abs_diff_eq(Vec3::X, 1e-5));
        assert!(a.mul_vec_sdk(inv.y_axis).abs_diff_eq(Vec3::Y, 1e-5));
        assert!(a.mul_vec_sdk(inv.z_axis).abs_diff_eq(Vec3::Z, 1e-5));
        assert!(inv.mul_vec_sdk(a.x_axis).abs_diff_eq(Vec3::X, 1e-5));
        assert!(inv.mul_vec_sdk(a.y_axis).abs_diff_eq(Vec3::Y, 1e-5));
        assert!(inv.mul_vec_sdk(a.z_axis).abs_diff_eq(Vec3::Z, 1e-5));
    }

    #[test]
    fn singular_matrix_inverts_to_zero() {
        let singular = Mat3::from_cols(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(2.0, 4.0, 6.0),
            Vec3::new(1.0, 1.0, 1.0),
        );
        let (inv, det) = singular.inverse_sdk();
        assert!(det.abs() < 1e-3);
        assert_eq!(inv, Mat3::ZERO);
    }

    #[test]
    fn rotation_inverse_is_transpose() {
        let r = Mat3::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
        let (r_inv, det) = r.inverse_sdk();
        assert!((det - 1.0).abs() < 1e-5);
        assert!(approx(r_inv, r.transpose()));
    }

    #[test]
    fn mul_vec_sdk_matches_column_expansion() {
        let a = Mat3::from_cols(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
            Vec3::new(7.0, 8.0, 9.0),
        );
        let v = Vec3::new(0.5, -1.5, 2.0);
        let expected = a.x_axis * v.x + a.y_axis * v.y + a.z_axis * v.z;
        assert_eq!(a.mul_vec_sdk(v), expected);
    }
}
