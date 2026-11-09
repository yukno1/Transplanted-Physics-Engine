//! Unit quaternions.
//!
//! Port of `hkQuaternionf` from
//! `original/Common/Source/Common/Base/Math/Quaternion/`.
//!
//! Storage is `(x, y, z, w)` matching `hkQuaternionf::m_vec` (`w` is the real
//! scalar part). Multiplication is the Hamilton product:
//!
//! ```text
//! (x,y,z,w) * (X,Y,Z,W) =
//!   ( x*W + w*X + y*Z - z*Y,
//!     y*W + w*Y + z*X - x*Z,
//!     z*W + w*Z + x*Y - y*X,
//!     w*W - x*X - y*Y - z*Z )
//! ```
//!
//! which is the same expansion documented in `hkQuaternionf.inl::setMul`.
//! Vector rotation uses the SDK's `_setRotatedDir` formulation
//! (`hkVector4f.inl` line 95), which assumes a normalized quaternion:
//!
//! ```text
//! r = 2 * ( v*(w*w - 0.5) + imag*dot(imag,v) + cross(imag,v)*w )
//! ```
//!
//! ```
//! use hk_math::{Quaternion, Vector4};
//! // 90 degrees about Z rotates X to Y.
//! let q = Quaternion::from_axis_angle(Vector4::UNIT_Z, std::f32::consts::FRAC_PI_2);
//! let rotated = q.rotate(Vector4::UNIT_X);
//! assert!(rotated.all_equal3(Vector4::UNIT_Y, 1e-6));
//! ```

use core::ops::Mul;

use crate::matrix3::Matrix3;
use crate::vector::Vector4;

/// Rotation quaternion (`hkQuaternionf`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quaternion {
    /// Imaginary part, component 0.
    pub x: f32,
    /// Imaginary part, component 1.
    pub y: f32,
    /// Imaginary part, component 2.
    pub z: f32,
    /// Real (scalar) part.
    pub w: f32,
}

impl Quaternion {
    /// Identity rotation `(0, 0, 0, 1)` (`hkQuaternionf::setIdentity`).
    pub const IDENTITY: Self = Self::new(0.0, 0.0, 0.0, 1.0);

    /// `hkQuaternionf(ix, iy, iz, r)`.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// `hkQuaternionf::setIdentity`.
    #[inline]
    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    /// Imaginary part as a 3-component vector.
    #[inline]
    pub const fn imag(self) -> Vector4 {
        Vector4::from_xyz(self.x, self.y, self.z)
    }

    /// Real (scalar) part.
    #[inline]
    pub const fn real(self) -> f32 {
        self.w
    }

    /// `hkQuaternionf::setReal`.
    #[inline]
    pub fn set_real(&mut self, real: f32) {
        self.w = real;
    }

    /// `hkQuaternionf::setImag`.
    #[inline]
    pub fn set_imag(&mut self, imag: Vector4) {
        self.x = imag.x;
        self.y = imag.y;
        self.z = imag.z;
    }

    /// Dot product of two quaternions as 4D vectors (`setClosest` uses it).
    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }

    /// Squared length.
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    /// Length.
    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// `hkQuaternionf::normalize<4, HK_ACC_FULL, HK_SQRT_SET_ZERO>`.
    ///
    /// A zero quaternion normalizes to zero (the SDK's `HK_SQRT_SET_ZERO`).
    #[inline]
    pub fn normalize(&mut self) {
        let length = self.length();
        if length == 0.0 {
            *self = Self::new(0.0, 0.0, 0.0, 0.0);
        } else {
            self.x /= length;
            self.y /= length;
            self.z /= length;
            self.w /= length;
        }
    }

    /// Normalized copy.
    #[inline]
    pub fn normalized(self) -> Self {
        let mut result = self;
        result.normalize();
        result
    }

    /// `hkQuaternionf::setInverse` — negates the imaginary part.
    ///
    /// For unit quaternions this is the inverse rotation; the C++ SDK does not
    /// divide by the squared norm here either.
    #[inline]
    pub fn inverse(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    /// `hkQuaternionf::setConjugate` (identical to [`Quaternion::inverse`]).
    #[inline]
    pub fn conjugate(self) -> Self {
        self.inverse()
    }

    /// `hkQuaternionf::setAxisAngle` — axis must be normalized.
    ///
    /// Uses Rust's `sin`/`cos`, so this is only bit-identical to a C++ build when
    /// the platform's libm produces the same results (see crate docs).
    #[inline]
    pub fn from_axis_angle(axis: Vector4, angle: f32) -> Self {
        let half_angle = 0.5 * angle;
        let (sin, cos) = half_angle.sin_cos();
        let imag = axis * sin;
        Self::new(imag.x, imag.y, imag.z, cos)
    }

    /// `hkQuaternionf::setMul` — Hamilton product `q0 * q1`.
    ///
    /// Operation order follows the expansion documented in the C++ inline
    /// implementation.
    #[allow(clippy::should_implement_trait)] // also exposed through `impl Mul`
    #[inline]
    pub fn mul(self, other: Self) -> Self {
        Self::new(
            self.x * other.w + self.w * other.x + self.y * other.z - self.z * other.y,
            self.y * other.w + self.w * other.y + self.z * other.x - self.x * other.z,
            self.z * other.w + self.w * other.z + self.x * other.y - self.y * other.x,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )
    }

    /// `hkQuaternionf::setMulInverse` — `q0 * inverse(q1)`.
    #[inline]
    pub fn mul_inverse(self, other: Self) -> Self {
        self.mul(other.inverse())
    }

    /// `hkQuaternionf::setInverseMul` — `inverse(q0) * q1`.
    #[inline]
    pub fn inverse_mul(self, other: Self) -> Self {
        self.inverse().mul(other)
    }

    /// `hkVector4f::_setRotatedDir(quat, direction)` — rotate a direction by this
    /// quaternion. Assumes the quaternion is normalized.
    #[inline]
    pub fn rotate(self, direction: Vector4) -> Vector4 {
        let q2_minus_1 = self.w * self.w - 0.5;
        let mut result = direction * q2_minus_1;
        let imag = self.imag();
        result += imag * imag.dot3(direction);
        result += imag.cross(direction) * self.w;
        result + result
    }

    /// `hkVector4f::_setRotatedInverseDir(quat, direction)` — rotate by the
    /// inverse rotation.
    #[inline]
    pub fn rotate_inverse(self, direction: Vector4) -> Vector4 {
        let q2_minus_1 = self.w * self.w - 0.5;
        let mut result = direction * q2_minus_1;
        let imag = self.imag();
        result += imag * imag.dot3(direction);
        result += direction.cross(imag) * self.w;
        result + result
    }

    /// `hkQuaternionf::set(hkRotationf)` — build a quaternion from a rotation
    /// matrix, using `hkQuaternionUtil::convertRotationToUnnormalizedQuaternion`.
    pub fn from_matrix(rotation: &Matrix3) -> Self {
        let c0 = rotation.col0;
        let c1 = rotation.col1;
        let c2 = rotation.col2;

        let m00 = c0.x;
        let m11 = c1.y;
        let m22 = c2.z;
        let m01 = c0.y;
        let m10 = c1.x;
        let m02 = c0.z;
        let m20 = c2.x;
        let m12 = c1.z;
        let m21 = c2.y;

        let mut q = if m22 < 0.0 {
            let m01_p_10 = m01 + m10;
            let m00_m_11 = m00 - m11;
            let t = 1.0 - m22;
            if m00 > m11 {
                Self::new(t + m00_m_11, m01_p_10, m20 + m02, m12 - m21)
            } else {
                Self::new(m01_p_10, t - m00_m_11, m12 + m21, m20 - m02)
            }
        } else {
            let t = 1.0 + m22;
            let m00_p_11 = m00 + m11;
            let m01_m_10 = m01 - m10;
            if m00 < -m11 {
                Self::new(m20 + m02, m12 + m21, t - m00_p_11, m01_m_10)
            } else {
                Self::new(m12 - m21, m20 - m02, m01_m_10, t + m00_p_11)
            }
        };

        // The C++ helper deliberately skips the 0.5/sqrt(t) scaling here and
        // normalizes afterwards.
        q.normalize();
        q
    }

    /// Build the rotation matrix (`hkRotationf::set(quaternion)` equivalent).
    ///
    /// Implemented by rotating the basis vectors with the same formulation as
    /// [`Quaternion::rotate`], so `q.to_matrix() * v == q.rotate(v)`.
    pub fn to_matrix(self) -> Matrix3 {
        let mut x = self.rotate(Vector4::UNIT_X);
        let mut y = self.rotate(Vector4::UNIT_Y);
        let mut z = self.rotate(Vector4::UNIT_Z);
        x.w = 0.0;
        y.w = 0.0;
        z.w = 0.0;
        Matrix3::from_cols(x, y, z)
    }

    /// Rotation angle in `[0, π]` (`hkQuaternionf::getAngle`).
    #[inline]
    pub fn angle(self) -> f32 {
        // |w| in [0,1] for a normalized quaternion; acos is the half angle doubled.
        2.0 * self.w.abs().min(1.0).acos()
    }

    /// Component-wise equality within `epsilon`.
    #[inline]
    pub fn all_equal(self, other: Self, epsilon: f32) -> bool {
        (self.x - other.x).abs() <= epsilon
            && (self.y - other.y).abs() <= epsilon
            && (self.z - other.z).abs() <= epsilon
            && (self.w - other.w).abs() <= epsilon
    }

    /// `hkQuaternionf::setClosest` — flip into the same hemisphere as `reference`.
    #[inline]
    pub fn closest_to(self, reference: Self) -> Self {
        if reference.dot(self) < 0.0 {
            Self::new(-self.x, -self.y, -self.z, -self.w)
        } else {
            self
        }
    }
}

impl Mul for Quaternion {
    type Output = Self;

    #[inline]
    fn mul(self, other: Self) -> Self {
        Quaternion::mul(self, other)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Matrix3, Vector4};
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_3, PI};

    const TOL: f32 = 1e-5;

    #[test]
    fn identity_rotates_nothing() {
        let q = Quaternion::IDENTITY;
        for v in [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z] {
            assert!(q.rotate(v).all_equal3(v, TOL));
        }
        assert_eq!(q.inverse(), Quaternion::new(-0.0, -0.0, -0.0, 1.0));
    }

    #[test]
    fn axis_angle_rotations() {
        let qz = Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2);
        assert!(qz.rotate(Vector4::UNIT_X).all_equal3(Vector4::UNIT_Y, TOL));
        assert!(qz.rotate(Vector4::UNIT_Y).all_equal3(-Vector4::UNIT_X, TOL));

        let qx = Quaternion::from_axis_angle(Vector4::UNIT_X, FRAC_PI_2);
        assert!(qx.rotate(Vector4::UNIT_Y).all_equal3(Vector4::UNIT_Z, TOL));

        // 180° about X: Y -> -Y, Z -> -Z.
        let q180 = Quaternion::from_axis_angle(Vector4::UNIT_X, PI);
        assert!(
            q180.rotate(Vector4::UNIT_Y)
                .all_equal3(-Vector4::UNIT_Y, TOL)
        );
        assert!(
            q180.rotate(Vector4::UNIT_Z)
                .all_equal3(-Vector4::UNIT_Z, TOL)
        );
    }

    #[test]
    fn rotate_inverse_undoes_rotate() {
        let q =
            Quaternion::from_axis_angle(Vector4::new(1.0, 2.0, 2.0, 0.0).normalized3(), FRAC_PI_3);
        let v = Vector4::new(1.0, -2.0, 0.5, 0.0);
        let rotated = q.rotate(v);
        let back = q.rotate_inverse(rotated);
        assert!(back.all_equal3(v, 1e-5), "got {back:?}");
    }

    #[test]
    fn multiplication_composes_rotations() {
        let qx = Quaternion::from_axis_angle(Vector4::UNIT_X, FRAC_PI_2);
        let qz = Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2);
        let composed = qx * qz;

        // `composed.rotate(v)` applies qz first, then qx (right-to-left).
        let v = Vector4::UNIT_X;
        let via_mul = composed.rotate(v);
        let via_two_steps = qx.rotate(qz.rotate(v));
        assert!(via_mul.all_equal3(via_two_steps, TOL));

        // Applying in the other order does not commute.
        let other = qz * qx;
        assert!(!other.rotate(v).all_equal3(via_mul, TOL));
    }

    #[test]
    fn inverse_is_the_rotation_inverse() {
        let q = Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.7);
        let v = Vector4::new(1.0, 0.0, 0.0, 0.0);
        let back = q.inverse().rotate(q.rotate(v));
        assert!(back.all_equal3(v, TOL));

        assert!(q.mul(q.inverse()).all_equal(Quaternion::IDENTITY, TOL));
        assert!(q.mul_inverse(q).all_equal(Quaternion::IDENTITY, TOL));
        assert!(q.inverse_mul(q).all_equal(Quaternion::IDENTITY, TOL));
    }

    #[test]
    fn normalization_preserves_rotation() {
        let raw = Quaternion::new(0.0, 0.0, 2.0, 2.0);
        let normalized = raw.normalized();
        assert!((normalized.length() - 1.0).abs() < 1e-6);
        let q = Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2);
        assert!(
            normalized
                .rotate(Vector4::UNIT_X)
                .all_equal3(q.rotate(Vector4::UNIT_X), 1e-5)
        );
    }

    #[test]
    fn matrix_conversions_round_trip() {
        let q = Quaternion::from_axis_angle(Vector4::UNIT_X, FRAC_PI_3);
        let m = q.to_matrix();
        let from_m = Quaternion::from_matrix(&m);
        // q and -q represent the same rotation.
        let reference = from_m.closest_to(q);
        assert!(
            reference.all_equal(q, 1e-4),
            "got {reference:?}, expected {q:?}"
        );
        assert!(
            m.mul_vec(Vector4::UNIT_Y)
                .all_equal3(q.rotate(Vector4::UNIT_Y), 1e-5)
        );
        assert_eq!(m.col0.w, 0.0);
        assert_eq!(m.col1.w, 0.0);
        assert_eq!(m.col2.w, 0.0);
    }

    #[test]
    fn angle_and_closest() {
        let q = Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.4);
        assert!((q.angle() - 0.4).abs() < 1e-5);

        let neg = Quaternion::new(-q.x, -q.y, -q.z, -q.w);
        assert_eq!(neg.closest_to(q), q);
        assert_eq!(q.closest_to(Quaternion::IDENTITY), q);
    }

    #[test]
    fn matrix_to_quaternion_covers_all_trace_branches() {
        // Identity (trace > 0, m00 >= -m11).
        let identity = Matrix3::IDENTITY;
        let q = Quaternion::from_matrix(&identity);
        assert!(q.all_equal(Quaternion::IDENTITY, 1e-6));

        // 180° rotations hit the `m22 < 0` branches.
        for axis in [Vector4::UNIT_X, Vector4::UNIT_Y, Vector4::UNIT_Z] {
            let q = Quaternion::from_axis_angle(axis, PI);
            let m = q.to_matrix();
            let back = Quaternion::from_matrix(&m).closest_to(q);
            assert!(back.all_equal(q, 1e-4), "axis {axis:?}: {back:?} vs {q:?}");
        }
    }
}
