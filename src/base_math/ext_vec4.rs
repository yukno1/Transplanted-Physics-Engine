//! SDK-parity scalar-order helpers for `glam::Vec4`.
//!
//! Every formula here is ported verbatim from the legacy scalar `Vector4`
//! implementation (`src/base_math/vector.rs`, macro `define_vector!`), keeping
//! the C++ `Fpu` operation order so glam-backed callers stay bit-parity with
//! the legacy layer (see `docs/refactor-glam-plan.md`, 方案 B). Source line
//! numbers in the docs refer to the legacy file before the glam migration.
//!
//! Note: per-component selection (`hkVector4f::setSelect`) is covered natively
//! by `Vec4::select(mask: BVec4A, if_true, if_false)`; no SDK-parity override
//! is needed.

use glam::Vec4;

/// `hkVector4f` scalar-order helpers for [`Vec4`].
pub trait Vec4Ext: Sized {
    /// `hkVector4f(x, y, z)` — `w` is zero
    /// (legacy `Vector4::from_xyz`, `vector.rs:191-195`).
    fn from_xyz(x: f32, y: f32, z: f32) -> Self;

    /// `hkVector4f::dot<3>` — `x*ox + y*oy + z*oz`, fixed addition order
    /// (legacy `Vector4::dot3`, `vector.rs:325-329`).
    fn dot3(self, other: Self) -> f32;

    /// `hkVector4f::length<3>` squared — `x*x + y*y + z*z`, fixed order
    /// (legacy `Vector4::length_squared3`, `vector.rs:319-321`).
    fn length_squared3(self) -> f32;

    /// `hkVector4f::length<3>` — `length_squared3().sqrt()`
    /// (legacy `Vector4::length3`, `vector.rs:325-327`).
    fn length3(self) -> f32;

    /// `hkVector4f::cross` — legacy `set_cross` scalar formula with `w` zeroed
    /// (legacy `Vector4::set_cross`, `vector.rs:294-301`). glam exposes `cross`
    /// only on `Vec3`/`Vec3A`, not `Vec4`.
    fn cross3(self, other: Self) -> Self;

    /// `hkVector4f::horizontalAdd<3>` — `x + y + z`
    /// (legacy `Vector4::horizontal_add3`, `vector.rs:453-457`).
    fn horizontal_add3(self) -> f32;

    /// `hkVector4f::allEqual<3>` — per-component `|a - b| <= eps` on `xyz` only
    /// (legacy `Vector4::all_equal3`, `vector.rs:487-491`).
    fn all_equal3(self, other: Self, epsilon: f32) -> bool;

    /// `hkVector4f::normalize<3, HK_SQRT_SET_ZERO>` — zero vector maps to zero;
    /// only `xyz` are divided, `w` is carried through unchanged
    /// (legacy `Vector4::normalize3`, `vector.rs:361-372`).
    fn normalize_or_zero3(self) -> Self;
}

impl Vec4Ext for Vec4 {
    #[inline]
    fn from_xyz(x: f32, y: f32, z: f32) -> Self {
        Self::new(x, y, z, 0.0)
    }

    #[inline]
    fn dot3(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    fn length_squared3(self) -> f32 {
        self.dot3(self)
    }

    #[inline]
    fn length3(self) -> f32 {
        self.length_squared3().sqrt()
    }

    #[inline]
    fn cross3(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
            0.0,
        )
    }

    #[inline]
    fn horizontal_add3(self) -> f32 {
        self.x + self.y + self.z
    }

    #[inline]
    fn all_equal3(self, other: Self, epsilon: f32) -> bool {
        (self.x - other.x).abs() <= epsilon
            && (self.y - other.y).abs() <= epsilon
            && (self.z - other.z).abs() <= epsilon
    }

    #[inline]
    fn normalize_or_zero3(self) -> Self {
        let length = self.dot3(self).sqrt();
        if length == 0.0 {
            Self::ZERO
        } else {
            Self::new(self.x / length, self.y / length, self.z / length, self.w)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sdk_helpers_match_legacy_scalar_semantics() {
        // dot3 / horizontal_add3: fixed scalar order.
        let a = Vec4::new(1.25, -2.5, 0.75, -0.125);
        let b = Vec4::new(3.5, 0.0625, -4.0, 2.0);
        assert_eq!(
            a.dot3(b).to_bits(),
            (a.x * b.x + a.y * b.y + a.z * b.z).to_bits()
        );
        assert_eq!(a.horizontal_add3().to_bits(), (a.x + a.y + a.z).to_bits());

        // length3: sqrt of the fixed-order xyz dot with self.
        assert_eq!(a.length3().to_bits(), a.length_squared3().sqrt().to_bits());

        // all_equal3: xyz-only tolerance comparison.
        assert!(a.all_equal3(Vec4::new(a.x, a.y, a.z, 0.0), 0.0));
        assert!(!a.all_equal3(Vec4::new(a.x + 1e-3, a.y, a.z, 0.0), 0.0));
        assert!(a.all_equal3(Vec4::new(a.x + 1e-3, a.y, a.z, 9.0), 1e-2));

        // from_xyz zeroes w.
        assert_eq!(Vec4::from_xyz(1.0, 2.0, 3.0), Vec4::new(1.0, 2.0, 3.0, 0.0));

        // normalize: zero maps to zero, w preserved.
        let zero = Vec4::ZERO;
        assert_eq!(zero.normalize_or_zero3(), Vec4::ZERO);
        let v = Vec4::new(3.0, 4.0, 0.0, 7.0);
        let n = v.normalize_or_zero3();
        assert_eq!(n.w, 7.0);
        assert!((n.x - 0.6).abs() < 1e-6 && (n.y - 0.8).abs() < 1e-6);

        // Selection is native: `Vec4::select(BVec4A, ..)`.
        let mask = glam::BVec4A::new(true, false, true, false);
        let picked = Vec4::select(mask, Vec4::splat(9.0), Vec4::splat(3.0));
        assert_eq!(picked, Vec4::new(9.0, 3.0, 9.0, 3.0));
    }
}
