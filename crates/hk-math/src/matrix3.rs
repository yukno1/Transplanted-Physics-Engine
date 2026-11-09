//! 3×3 rotation matrices.
//!
//! Port of `hkMatrix3` / `hkRotationf` from
//! `original/Common/Source/Common/Base/Math/Matrix/`.
//!
//! The SDK stores a matrix as three columns of `hkVector4f` (`m_col0` … `m_col2`);
//! the fourth component of each column is unused (and stored as 0 for rotation
//! matrices). `setMul(a, b)` computes `a * b`: each result column is `a` applied to
//! the corresponding column of `b` (`hkTransformUtil::rotatePoints`).
//!
//! ```
//! use hk_math::{Matrix3, Vector4};
//! let m = Matrix3::IDENTITY;
//! assert!(m.mul_vec(Vector4::UNIT_X).all_equal3(Vector4::UNIT_X, 0.0));
//! ```
//!
//! Inversion follows `hkMatrix3::_setInverse<HK_MATRIX_ANY, HK_ACC_FULL,
//! HK_DETERMINANT_CHECK_EPS, HK_INVERSION_FAILURE_SET_ZERO>`: cofactor cross
//! products, the SDK's scaled epsilon determinant check, and a zero matrix when
//! the input is singular.

use crate::quaternion::Quaternion;
use crate::vector::Vector4;

/// Column-major 3×3 matrix (`hkMatrix3f`, and the storage of `hkRotationf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix3 {
    /// Column 0.
    pub col0: Vector4,
    /// Column 1.
    pub col1: Vector4,
    /// Column 2.
    pub col2: Vector4,
}

impl Matrix3 {
    /// Identity matrix (`hkMatrix3f::setIdentity`).
    pub const IDENTITY: Self = Self::from_cols(
        Vector4::new(1.0, 0.0, 0.0, 0.0),
        Vector4::new(0.0, 1.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 1.0, 0.0),
    );

    /// Zero matrix (`hkMatrix3f::setZero`).
    pub const ZERO: Self = Self::from_cols(Vector4::ZERO, Vector4::ZERO, Vector4::ZERO);

    /// Build from three columns (`hkMatrix3f::setCols`).
    #[inline]
    pub const fn from_cols(c0: Vector4, c1: Vector4, c2: Vector4) -> Self {
        Self {
            col0: c0,
            col1: c1,
            col2: c2,
        }
    }

    /// Build from three rows (`hkMatrix3f::setRows`).
    #[inline]
    pub fn from_rows(r0: Vector4, r1: Vector4, r2: Vector4) -> Self {
        Self::from_cols(
            Vector4::from_xyz(r0.x, r1.x, r2.x),
            Vector4::from_xyz(r0.y, r1.y, r2.y),
            Vector4::from_xyz(r0.z, r1.z, r2.z),
        )
    }

    /// `hkMatrix3f::setIdentity`.
    #[inline]
    pub fn set_identity(&mut self) {
        *self = Self::IDENTITY;
    }

    /// `hkMatrix3f::setZero`.
    #[inline]
    pub fn set_zero(&mut self) {
        *self = Self::ZERO;
    }

    /// Column by index (`hkMatrix3f::getColumn`).
    #[inline]
    pub fn column(&self, index: usize) -> Vector4 {
        match index {
            0 => self.col0,
            1 => self.col1,
            2 => self.col2,
            _ => panic!("Matrix3 column index out of range: {index}"),
        }
    }

    /// Element `m[row][col]` (same indexing as `hkMatrix3f::getElement<ROW, COL>`).
    #[inline]
    pub fn element(&self, row: usize, column: usize) -> f32 {
        self.column(column).component(row)
    }

    /// Set element `m[row][col]` (`hkMatrix3f::setElement`).
    #[inline]
    pub fn set_element(&mut self, row: usize, column: usize, value: f32) {
        self.column_mut(column).set_component(row, value);
    }

    fn column_mut(&mut self, index: usize) -> &mut Vector4 {
        match index {
            0 => &mut self.col0,
            1 => &mut self.col1,
            2 => &mut self.col2,
            _ => panic!("Matrix3 column index out of range: {index}"),
        }
    }

    /// `hkMatrix3f::multiplyVector` — `self * v`, using the C++ operation order
    /// (`v.x * col0 + v.y * col1 + v.z * col2`).
    #[inline]
    pub fn mul_vec(&self, v: Vector4) -> Vector4 {
        let mut result = self.col0 * v.x;
        result += self.col1 * v.y;
        result += self.col2 * v.z;
        result
    }

    /// Rotate a direction by this matrix (`hkVector4f::setRotatedDir(matrix, v)`).
    #[inline]
    pub fn rotate(&self, v: Vector4) -> Vector4 {
        self.mul_vec(v)
    }

    /// Rotate by the inverse rotation — equivalent to `self^T * v`.
    #[inline]
    pub fn rotate_inverse(&self, v: Vector4) -> Vector4 {
        Vector4::from_xyz(self.col0.dot3(v), self.col1.dot3(v), self.col2.dot3(v))
    }

    /// `hkMatrix3f::setMul` — `self = a * b`.
    #[inline]
    pub fn set_mul(&mut self, a: &Self, b: &Self) {
        *self = Self::from_cols(a.mul_vec(b.col0), a.mul_vec(b.col1), a.mul_vec(b.col2));
    }

    /// Matrix product `a * b`.
    #[inline]
    pub fn mul_mat(&self, other: &Self) -> Self {
        Self::from_cols(
            self.mul_vec(other.col0),
            self.mul_vec(other.col1),
            self.mul_vec(other.col2),
        )
    }

    /// Product `q * m`: rotate every column of `m` (`hkMatrix3f::setMul(quat, m)`).
    #[inline]
    pub fn mul_quaternion(&self, q: Quaternion) -> Self {
        Self::from_cols(
            q.rotate(self.col0),
            q.rotate(self.col1),
            q.rotate(self.col2),
        )
    }

    /// `hkMatrix3f::setTranspose`.
    #[inline]
    pub fn transpose(&self) -> Self {
        Self::from_cols(
            Vector4::from_xyz(self.col0.x, self.col1.x, self.col2.x),
            Vector4::from_xyz(self.col0.y, self.col1.y, self.col2.y),
            Vector4::from_xyz(self.col0.z, self.col1.z, self.col2.z),
        )
    }

    /// `hkMatrix3f::setInverse` with the default determinant check and
    /// `HK_INVERSION_FAILURE_SET_ZERO` behavior.
    ///
    /// Returns `(inverse, determinant)`. A singular matrix yields a zero matrix
    /// (the SDK's default failure mode) and its determinant.
    pub fn inverse(&self) -> (Self, f32) {
        let c0 = self.col0;
        let c1 = self.col1;
        let c2 = self.col2;

        let r0 = c1.cross(c2);
        let r1 = c2.cross(c0);
        let r2 = c0.cross(c1);

        let determinant = c0.dot3(r0);

        // Scaled epsilon check from hkMatrix3::_setInverse:
        //   eps = feps * L1(c0) * ( L1(c1) * L1(c2) )
        let abs_det = determinant.abs();
        let feps = f32::EPSILON;
        let eps = (feps * c0.abs().horizontal_add3())
            * (c1.abs().horizontal_add3() * c2.abs().horizontal_add3());
        let is_not_singular = abs_det > eps;

        let determinant_inverse = if is_not_singular {
            1.0 / determinant
        } else {
            0.0
        };

        // Columns of the inverse are the transposed cofactor vectors.
        let inv0 = Vector4::from_xyz(r0.x, r1.x, r2.x);
        let inv1 = Vector4::from_xyz(r0.y, r1.y, r2.y);
        let inv2 = Vector4::from_xyz(r0.z, r1.z, r2.z);

        (
            Self::from_cols(
                inv0 * determinant_inverse,
                inv1 * determinant_inverse,
                inv2 * determinant_inverse,
            ),
            determinant,
        )
    }

    /// Invert in place (`hkMatrix3f::setInverse(this, this)`).
    pub fn invert(&mut self) {
        *self = self.inverse().0;
    }

    /// Whether the matrix is orthonormal within `epsilon`
    /// (`hkRotationf::isOrthonormal`).
    pub fn is_orthonormal(&self, epsilon: f32) -> bool {
        let product = self.transpose().mul_mat(self);
        product.col0.all_equal3(Vector4::UNIT_X, epsilon)
            && product.col1.all_equal3(Vector4::UNIT_Y, epsilon)
            && product.col2.all_equal3(Vector4::UNIT_Z, epsilon)
    }

    /// Rotation matrix from a quaternion (`hkRotationf::set(quaternion)`).
    #[inline]
    pub fn from_quaternion(q: Quaternion) -> Self {
        q.to_matrix()
    }

    /// Quaternion from a rotation matrix (`hkQuaternionf::set(hkRotationf)`).
    #[inline]
    pub fn to_quaternion(&self) -> Quaternion {
        Quaternion::from_matrix(self)
    }

    /// Frobenius norm squared.
    #[inline]
    pub fn frobenius_norm_squared(&self) -> f32 {
        self.col0.dot3(self.col0) + self.col1.dot3(self.col1) + self.col2.dot3(self.col2)
    }
}

impl Default for Matrix3 {
    /// Identity matrix (the C++ default constructor leaves storage uninitialized).
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

    fn approx_matrix(a: &Matrix3, b: &Matrix3) -> bool {
        a.col0.all_equal(b.col0, TOL)
            && a.col1.all_equal(b.col1, TOL)
            && a.col2.all_equal(b.col2, TOL)
    }

    #[test]
    fn identity_and_transpose() {
        assert_eq!(Matrix3::default(), Matrix3::IDENTITY);
        assert!(
            Matrix3::IDENTITY
                .mul_vec(Vector4::UNIT_X)
                .all_equal3(Vector4::UNIT_X, 0.0)
        );
        let m = Matrix3::from_rows(
            Vector4::from_xyz(1.0, 2.0, 3.0),
            Vector4::from_xyz(4.0, 5.0, 6.0),
            Vector4::from_xyz(7.0, 8.0, 9.0),
        );
        let t = m.transpose();
        assert_eq!(t.element(0, 1), m.element(1, 0));
        assert_eq!(t.element(2, 0), m.element(0, 2));
        assert_eq!(m.element(1, 2), 6.0, "row 1, column 2 of [1..9] is 6");
    }

    #[test]
    fn matrix_product_matches_vector_products() {
        let a = Matrix3::from_rows(
            Vector4::from_xyz(1.0, 2.0, 3.0),
            Vector4::from_xyz(0.0, 1.0, 4.0),
            Vector4::from_xyz(5.0, 6.0, 0.0),
        );
        let b = Matrix3::from_rows(
            Vector4::from_xyz(-2.0, 1.0, 0.0),
            Vector4::from_xyz(3.0, 0.0, 0.0),
            Vector4::from_xyz(4.0, 5.0, 6.0),
        );
        let c = a.mul_mat(&b);
        let mut in_place = Matrix3::ZERO;
        in_place.set_mul(&a, &b);
        assert_eq!(c, in_place);

        for v in [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z] {
            let via_matrix = c.mul_vec(v);
            let via_two_steps = a.mul_vec(b.mul_vec(v));
            assert!(via_matrix.all_equal3(via_two_steps, TOL));
        }
    }

    #[test]
    fn inverse_round_trips() {
        let a = Matrix3::from_rows(
            Vector4::from_xyz(2.0, 1.0, 0.0),
            Vector4::from_xyz(1.0, 3.0, 1.0),
            Vector4::from_xyz(0.0, 1.0, 2.0),
        );
        let (inv, det) = a.inverse();
        assert!((det - 8.0).abs() < 1e-6, "det = {det}");
        assert!(approx_matrix(&a.mul_mat(&inv), &Matrix3::IDENTITY));
        assert!(approx_matrix(&inv.mul_mat(&a), &Matrix3::IDENTITY));

        // Inverse of a rotation is its transpose.
        let q = Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2);
        let r = q.to_matrix();
        let (r_inv, det) = r.inverse();
        assert!((det - 1.0).abs() < 1e-5);
        assert!(
            approx_matrix(&r_inv, &r.transpose()),
            "rotation inverse must be transpose"
        );
    }

    #[test]
    fn singular_matrix_inverts_to_zero() {
        let singular = Matrix3::from_cols(
            Vector4::from_xyz(1.0, 2.0, 3.0),
            Vector4::from_xyz(2.0, 4.0, 6.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        );
        let (inv, det) = singular.inverse();
        assert!(det.abs() < 1e-3);
        assert_eq!(inv, Matrix3::ZERO);
    }

    #[test]
    fn rotation_matrix_consistency() {
        let q = Quaternion::from_axis_angle(Vector4::UNIT_X, FRAC_PI_2);
        let m = Matrix3::from_quaternion(q);
        assert!(approx_matrix(&m, &q.to_matrix()));
        assert!(m.col0.all_equal3(q.rotate(Vector4::UNIT_X), TOL));
        assert!(m.col1.all_equal3(q.rotate(Vector4::UNIT_Y), TOL));
        assert!(m.col2.all_equal3(q.rotate(Vector4::UNIT_Z), TOL));
        assert!(m.is_orthonormal(1e-5));

        let quat = m.to_quaternion().closest_to(q);
        assert!(quat.all_equal(q, 1e-4));
    }

    #[test]
    fn rotate_inverse_is_transpose_multiplication() {
        let m = Matrix3::from_quaternion(Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.6));
        let v = Vector4::new(1.0, 2.0, -1.0, 0.0);
        assert!(
            m.rotate_inverse(v)
                .all_equal3(m.transpose().mul_vec(v), TOL)
        );
    }
}
