//! Four-component floating point vectors.
//!
//! Port of `hkVector4f` / `hkVector4d` from
//! `original/Common/Source/Common/Base/Math/Vector/`.
//!
//! The C++ classes are large (hundreds of methods, most of them SIMD wrappers). This
//! port keeps the data layout and the arithmetic semantics of the scalar path
//! (`Fpu`) and exposes the operations the rest of the engine actually relies on:
//! component access, `+ - * /`, dot/cross products, lengths, normalization,
//! component-wise `min`/`max`/`abs`, interpolate, and comparison masks.
//!
//! Both the `f32` ([`Vector4`]) and `f64` ([`Vector4d`]) variants are generated
//! from the same macro so they cannot drift apart.

use core::ops::{
    Add, AddAssign, BitAnd, BitOr, BitXor, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg,
    Not, Sub, SubAssign,
};

macro_rules! define_mask {
    ($mask:ident) => {
        /// Per-component comparison mask (`hkVector4fComparison` / `hkVector4dComparison`).
        ///
        /// Bit `i` is set when component `i` satisfied the comparison. `MASK_XYZ` is
        /// bits 0..2, `MASK_XYZW` is bits 0..3 (matching `hkVector4ComparisonMask`).
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
        pub struct $mask(u8);

        impl $mask {
            /// No bits set.
            pub const NONE: Self = Self(0);
            /// `hkVector4ComparisonMask::MASK_X`.
            pub const X: Self = Self(0b0001);
            /// `hkVector4ComparisonMask::MASK_Y`.
            pub const Y: Self = Self(0b0010);
            /// `hkVector4ComparisonMask::MASK_Z`.
            pub const Z: Self = Self(0b0100);
            /// `hkVector4ComparisonMask::MASK_W`.
            pub const W: Self = Self(0b1000);
            /// `MASK_XYZ` — components x, y, z.
            pub const XYZ: Self = Self(0b0111);
            /// `MASK_XYZW` — all four components.
            pub const XYZW: Self = Self(0b1111);

            /// Build from the low four bits of `bits`.
            #[inline]
            pub const fn from_bits(bits: u8) -> Self {
                Self(bits & 0xF)
            }

            /// Raw mask bits.
            #[inline]
            pub const fn bits(self) -> u8 {
                self.0
            }

            /// `hkVector4fComparison::allAreSet` over all four components.
            #[inline]
            pub const fn all(self) -> bool {
                self.0 & 0xF == 0xF
            }

            /// `hkVector4fComparison::anyIsSet` over all four components.
            #[inline]
            pub const fn any(self) -> bool {
                self.0 & 0xF != 0
            }

            /// `allAreSet<MASK_XYZ>()`.
            #[inline]
            pub const fn all_xyz(self) -> bool {
                self.0 & 0b0111 == 0b0111
            }

            /// `anyIsSet<MASK_XYZ>()`.
            #[inline]
            pub const fn any_xyz(self) -> bool {
                self.0 & 0b0111 != 0
            }

            /// Component `i` (0..=3).
            #[inline]
            pub const fn get(self, index: usize) -> bool {
                (self.0 & (1 << index)) != 0
            }

            /// `hkVector4fComparison::setAnd`.
            #[inline]
            pub const fn and(self, other: Self) -> Self {
                Self::from_bits(self.0 & other.0)
            }

            /// `hkVector4fComparison::setOr`.
            #[inline]
            pub const fn or(self, other: Self) -> Self {
                Self::from_bits(self.0 | other.0)
            }

            /// `a AND NOT b` (`hkVector4fComparison::setAndNot`).
            #[inline]
            pub const fn and_not(self, other: Self) -> Self {
                Self::from_bits(self.0 & !other.0)
            }
        }

        impl BitAnd for $mask {
            type Output = Self;

            #[inline]
            fn bitand(self, rhs: Self) -> Self {
                self.and(rhs)
            }
        }

        impl BitOr for $mask {
            type Output = Self;

            #[inline]
            fn bitor(self, rhs: Self) -> Self {
                self.or(rhs)
            }
        }

        impl BitXor for $mask {
            type Output = Self;

            #[inline]
            fn bitxor(self, rhs: Self) -> Self {
                Self::from_bits(self.0 ^ rhs.0)
            }
        }

        impl Not for $mask {
            type Output = Self;

            #[inline]
            fn not(self) -> Self {
                Self::from_bits(!self.0)
            }
        }
    };
}

define_mask!(Vector4Mask);
define_mask!(Vector4dMask);

macro_rules! define_vector {
    ($vec:ident, $mask:ident, $ty:ty) => {
        /// Four-component floating point vector (`hkVector4f` / `hkVector4d`).
        ///
        /// 16-byte aligned like the SDK's quad types. Fields are public for
        /// serialization-friendly access; the arithmetic helpers below keep the
        /// C++ scalar operation order.
        #[repr(C, align(16))]
        #[derive(Clone, Copy, Debug, Default, PartialEq)]
        pub struct $vec {
            /// Component 0 (`hkVector4f::getComponent<0>`).
            pub x: $ty,
            /// Component 1.
            pub y: $ty,
            /// Component 2.
            pub z: $ty,
            /// Component 3 (the homogeneous `w`, usually 0 for points/directions).
            pub w: $ty,
        }

        impl $vec {
            /// `(0, 0, 0, 0)`.
            pub const ZERO: Self = Self::splat(0.0);
            /// `(1, 1, 1, 1)`.
            pub const ONE: Self = Self::splat(1.0);
            /// `(-1, -1, -1, -1)`.
            pub const NEG_ONE: Self = Self::splat(-1.0);
            /// `(0.5, 0.5, 0.5, 0.5)`.
            pub const HALF: Self = Self::splat(0.5);
            /// `(1, 0, 0, 0)` — `HK_QUADREAL_1000`.
            pub const UNIT_X: Self = Self::new(1.0, 0.0, 0.0, 0.0);
            /// `(0, 1, 0, 0)` — `HK_QUADREAL_0100`.
            pub const UNIT_Y: Self = Self::new(0.0, 1.0, 0.0, 0.0);
            /// `(0, 0, 1, 0)` — `HK_QUADREAL_0010`.
            pub const UNIT_Z: Self = Self::new(0.0, 0.0, 1.0, 0.0);
            /// `(0, 0, 0, 1)` — `HK_QUADREAL_0001`.
            pub const UNIT_W: Self = Self::new(0.0, 0.0, 0.0, 1.0);

            /// `hkVector4f(x, y, z, w)`.
            #[inline]
            pub const fn new(x: $ty, y: $ty, z: $ty, w: $ty) -> Self {
                Self { x, y, z, w }
            }

            /// `hkVector4f(x, y, z)` — `w` is zero.
            #[inline]
            pub const fn from_xyz(x: $ty, y: $ty, z: $ty) -> Self {
                Self::new(x, y, z, 0.0)
            }

            /// `hkVector4f::setAll` — all four components equal `value`.
            #[inline]
            pub const fn splat(value: $ty) -> Self {
                Self::new(value, value, value, value)
            }

            /// Component by index (0..=3).
            #[inline]
            pub fn component(&self, index: usize) -> $ty {
                match index {
                    0 => self.x,
                    1 => self.y,
                    2 => self.z,
                    3 => self.w,
                    _ => panic!("Vector component index out of range: {index}"),
                }
            }

            /// Set component by index (0..=3).
            #[inline]
            pub fn set_component(&mut self, index: usize, value: $ty) {
                match index {
                    0 => self.x = value,
                    1 => self.y = value,
                    2 => self.z = value,
                    3 => self.w = value,
                    _ => panic!("Vector component index out of range: {index}"),
                }
            }

            /// `HK_QUADREAL_1000`-style basis vector.
            #[inline]
            pub fn basis(index: usize) -> Self {
                let mut v = Self::ZERO;
                v.set_component(index, 1.0);
                v
            }

            /// `hkVector4f::set` — assign all four components.
            #[inline]
            pub fn set(&mut self, x: $ty, y: $ty, z: $ty, w: $ty) {
                *self = Self::new(x, y, z, w);
            }

            /// `hkVector4f::set` — assign `xyz` and keep `w`.
            #[inline]
            pub fn set_xyz(&mut self, x: $ty, y: $ty, z: $ty) {
                self.x = x;
                self.y = y;
                self.z = z;
            }

            /// `hkVector4f::setAll`.
            #[inline]
            pub fn set_all(&mut self, value: $ty) {
                *self = Self::splat(value);
            }

            /// `hkVector4f::setZero`.
            #[inline]
            pub fn set_zero(&mut self) {
                *self = Self::ZERO;
            }

            /// `hkVector4f::setAdd` — `self = a + b`.
            #[inline]
            pub fn set_add(&mut self, a: Self, b: Self) {
                *self = Self::new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
            }

            /// `hkVector4f::setSub` — `self = a - b`.
            #[inline]
            pub fn set_sub(&mut self, a: Self, b: Self) {
                *self = Self::new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
            }

            /// `hkVector4f::setMul` — `self = a * b` (component-wise).
            #[inline]
            pub fn set_mul(&mut self, a: Self, b: Self) {
                *self = Self::new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
            }

            /// `hkVector4f::setDiv` — `self = a / b` (component-wise, no checks).
            #[inline]
            pub fn set_div(&mut self, a: Self, b: Self) {
                *self = Self::new(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
            }

            /// `hkVector4f::setAddMul` — `self = a + b * c`.
            #[inline]
            pub fn set_add_mul(&mut self, a: Self, b: Self, c: Self) {
                *self = Self::new(
                    a.x + b.x * c.x,
                    a.y + b.y * c.y,
                    a.z + b.z * c.z,
                    a.w + b.w * c.w,
                );
            }

            /// `hkVector4f::setSubMul` — `self = a - b * c`.
            #[inline]
            pub fn set_sub_mul(&mut self, a: Self, b: Self, c: Self) {
                *self = Self::new(
                    a.x - b.x * c.x,
                    a.y - b.y * c.y,
                    a.z - b.z * c.z,
                    a.w - b.w * c.w,
                );
            }

            /// `hkVector4f::setCross` — 3-component cross product; `w` follows the
            /// C++ implementation (two commutators, zero `w`).
            #[inline]
            pub fn set_cross(&mut self, a: Self, b: Self) {
                *self = Self::new(
                    a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x,
                    0.0,
                );
            }

            /// `hkVector4f::cross`.
            #[inline]
            pub fn cross(self, other: Self) -> Self {
                let mut result = Self::ZERO;
                result.set_cross(self, other);
                result
            }

            /// `hkVector4f::dot<3>`.
            #[inline]
            pub fn dot3(self, other: Self) -> $ty {
                self.x * other.x + self.y * other.y + self.z * other.z
            }

            /// `hkVector4f::dot<4>`.
            #[inline]
            pub fn dot4(self, other: Self) -> $ty {
                self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
            }

            /// `hkVector4f::lengthSquared<3>`.
            #[inline]
            pub fn length_squared3(self) -> $ty {
                self.dot3(self)
            }

            /// `hkVector4f::lengthSquared<4>`.
            #[inline]
            pub fn length_squared4(self) -> $ty {
                self.dot4(self)
            }

            /// `hkVector4f::length<3>`.
            #[inline]
            pub fn length3(self) -> $ty {
                self.length_squared3().sqrt()
            }

            /// `hkVector4f::length<4>`.
            #[inline]
            pub fn length4(self) -> $ty {
                self.length_squared4().sqrt()
            }

            /// `hkVector4f::normalize<3>` — zero vector maps to zero.
            #[inline]
            pub fn normalize3(&mut self) {
                let length = self.length3();
                if length == 0.0 {
                    *self = Self::ZERO;
                } else {
                    self.x /= length;
                    self.y /= length;
                    self.z /= length;
                }
            }

            /// `hkVector4f::normalize<4>` — zero vector maps to zero.
            #[inline]
            pub fn normalize4(&mut self) {
                let length = self.length4();
                if length == 0.0 {
                    *self = Self::ZERO;
                } else {
                    self.x /= length;
                    self.y /= length;
                    self.z /= length;
                    self.w /= length;
                }
            }

            /// `hkVector4f::normalized<3>`.
            #[inline]
            pub fn normalized3(self) -> Self {
                let mut result = self;
                result.normalize3();
                result
            }

            /// `hkVector4f::normalized<4>`.
            #[inline]
            pub fn normalized4(self) -> Self {
                let mut result = self;
                result.normalize4();
                result
            }

            /// `hkVector4f::setInterpolate` — `v0 + t * (v1 - v0)`.
            #[inline]
            pub fn set_interpolate(&mut self, v0: Self, v1: Self, t: $ty) {
                let d = v1 - v0;
                *self = v0 + d * t;
            }

            /// `hkVector4f::absolute` / `setAbs`.
            #[inline]
            pub fn abs(self) -> Self {
                Self::new(self.x.abs(), self.y.abs(), self.z.abs(), self.w.abs())
            }

            /// `hkVector4f::setMin` — component-wise minimum.
            #[inline]
            pub fn min(self, other: Self) -> Self {
                Self::new(
                    self.x.min(other.x),
                    self.y.min(other.y),
                    self.z.min(other.z),
                    self.w.min(other.w),
                )
            }

            /// `hkVector4f::setMax` — component-wise maximum.
            #[inline]
            pub fn max(self, other: Self) -> Self {
                Self::new(
                    self.x.max(other.x),
                    self.y.max(other.y),
                    self.z.max(other.z),
                    self.w.max(other.w),
                )
            }

            /// Component-wise flip of the sign where `sign` is negative
            /// (`hkVector4f::setFlipSign(v, vSign)`).
            #[inline]
            pub fn flip_sign(self, sign: Self) -> Self {
                Self::new(
                    if sign.x < 0.0 { -self.x } else { self.x },
                    if sign.y < 0.0 { -self.y } else { self.y },
                    if sign.z < 0.0 { -self.z } else { self.z },
                    if sign.w < 0.0 { -self.w } else { self.w },
                )
            }

            /// `hkVector4f::horizontalAdd<3>` — `x + y + z`.
            #[inline]
            pub fn horizontal_add3(self) -> $ty {
                self.x + self.y + self.z
            }

            /// `hkVector4f::horizontalAdd<4>` — sum of all components.
            #[inline]
            pub fn horizontal_add4(self) -> $ty {
                self.x + self.y + self.z + self.w
            }

            /// `hkVector4f::less`.
            #[inline]
            pub fn less(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x < other.x) as u8
                        | ((self.y < other.y) as u8) << 1
                        | ((self.z < other.z) as u8) << 2
                        | ((self.w < other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::lessEqual`.
            #[inline]
            pub fn less_equal(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x <= other.x) as u8
                        | ((self.y <= other.y) as u8) << 1
                        | ((self.z <= other.z) as u8) << 2
                        | ((self.w <= other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::greater`.
            #[inline]
            pub fn greater(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x > other.x) as u8
                        | ((self.y > other.y) as u8) << 1
                        | ((self.z > other.z) as u8) << 2
                        | ((self.w > other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::greaterEqual`.
            #[inline]
            pub fn greater_equal(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x >= other.x) as u8
                        | ((self.y >= other.y) as u8) << 1
                        | ((self.z >= other.z) as u8) << 2
                        | ((self.w >= other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::equal`.
            #[inline]
            pub fn equal(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x == other.x) as u8
                        | ((self.y == other.y) as u8) << 1
                        | ((self.z == other.z) as u8) << 2
                        | ((self.w == other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::notEqual`.
            #[inline]
            pub fn not_equal(self, other: Self) -> $mask {
                $mask::from_bits(
                    (self.x != other.x) as u8
                        | ((self.y != other.y) as u8) << 1
                        | ((self.z != other.z) as u8) << 2
                        | ((self.w != other.w) as u8) << 3,
                )
            }

            /// `hkVector4f::lessZero`.
            #[inline]
            pub fn less_zero(self) -> $mask {
                self.less(Self::ZERO)
            }

            /// `hkVector4f::greaterZero`.
            #[inline]
            pub fn greater_zero(self) -> $mask {
                self.greater(Self::ZERO)
            }

            /// `hkVector4f::setSelect` — per-component `mask ? true_value : false_value`.
            #[inline]
            pub fn select(mask: $mask, true_value: Self, false_value: Self) -> Self {
                let pick = |bit: bool, t: $ty, f: $ty| if bit { t } else { f };
                Self::new(
                    pick(mask.get(0), true_value.x, false_value.x),
                    pick(mask.get(1), true_value.y, false_value.y),
                    pick(mask.get(2), true_value.z, false_value.z),
                    pick(mask.get(3), true_value.w, false_value.w),
                )
            }

            /// `hkVector4f::allEqual` with an epsilon per component.
            #[inline]
            pub fn all_equal(self, other: Self, epsilon: $ty) -> bool {
                (self.x - other.x).abs() <= epsilon
                    && (self.y - other.y).abs() <= epsilon
                    && (self.z - other.z).abs() <= epsilon
                    && (self.w - other.w).abs() <= epsilon
            }

            /// `hkVector4f::allEqual<3>` (x, y, z only).
            #[inline]
            pub fn all_equal3(self, other: Self, epsilon: $ty) -> bool {
                (self.x - other.x).abs() <= epsilon
                    && (self.y - other.y).abs() <= epsilon
                    && (self.z - other.z).abs() <= epsilon
            }

            /// Raw storage as `[f32; 4]` / `[f64; 4]`.
            #[inline]
            pub fn to_array(self) -> [$ty; 4] {
                [self.x, self.y, self.z, self.w]
            }

            /// Construct from raw storage.
            #[inline]
            pub const fn from_array(array: [$ty; 4]) -> Self {
                Self::new(array[0], array[1], array[2], array[3])
            }
        }

        impl Add for $vec {
            type Output = Self;

            #[inline]
            fn add(self, other: Self) -> Self {
                Self::new(
                    self.x + other.x,
                    self.y + other.y,
                    self.z + other.z,
                    self.w + other.w,
                )
            }
        }

        impl Sub for $vec {
            type Output = Self;

            #[inline]
            fn sub(self, other: Self) -> Self {
                Self::new(
                    self.x - other.x,
                    self.y - other.y,
                    self.z - other.z,
                    self.w - other.w,
                )
            }
        }

        impl Mul for $vec {
            type Output = Self;

            #[inline]
            fn mul(self, other: Self) -> Self {
                Self::new(
                    self.x * other.x,
                    self.y * other.y,
                    self.z * other.z,
                    self.w * other.w,
                )
            }
        }

        impl Div for $vec {
            type Output = Self;

            #[inline]
            fn div(self, other: Self) -> Self {
                Self::new(
                    self.x / other.x,
                    self.y / other.y,
                    self.z / other.z,
                    self.w / other.w,
                )
            }
        }

        impl Mul<$ty> for $vec {
            type Output = Self;

            #[inline]
            fn mul(self, scalar: $ty) -> Self {
                Self::new(
                    self.x * scalar,
                    self.y * scalar,
                    self.z * scalar,
                    self.w * scalar,
                )
            }
        }

        impl Mul<$vec> for $ty {
            type Output = $vec;

            #[inline]
            fn mul(self, vector: $vec) -> $vec {
                vector * self
            }
        }

        impl Div<$ty> for $vec {
            type Output = Self;

            #[inline]
            fn div(self, scalar: $ty) -> Self {
                Self::new(
                    self.x / scalar,
                    self.y / scalar,
                    self.z / scalar,
                    self.w / scalar,
                )
            }
        }

        impl Neg for $vec {
            type Output = Self;

            #[inline]
            fn neg(self) -> Self {
                Self::new(-self.x, -self.y, -self.z, -self.w)
            }
        }

        impl AddAssign for $vec {
            #[inline]
            fn add_assign(&mut self, other: Self) {
                *self = *self + other;
            }
        }

        impl SubAssign for $vec {
            #[inline]
            fn sub_assign(&mut self, other: Self) {
                *self = *self - other;
            }
        }

        impl MulAssign for $vec {
            #[inline]
            fn mul_assign(&mut self, other: Self) {
                *self = *self * other;
            }
        }

        impl MulAssign<$ty> for $vec {
            #[inline]
            fn mul_assign(&mut self, scalar: $ty) {
                *self = *self * scalar;
            }
        }

        impl DivAssign for $vec {
            #[inline]
            fn div_assign(&mut self, other: Self) {
                *self = *self / other;
            }
        }

        impl DivAssign<$ty> for $vec {
            #[inline]
            fn div_assign(&mut self, scalar: $ty) {
                *self = *self / scalar;
            }
        }

        impl Index<usize> for $vec {
            type Output = $ty;

            #[inline]
            fn index(&self, index: usize) -> &$ty {
                match index {
                    0 => &self.x,
                    1 => &self.y,
                    2 => &self.z,
                    3 => &self.w,
                    _ => panic!("Vector index out of range: {index}"),
                }
            }
        }

        impl IndexMut<usize> for $vec {
            #[inline]
            fn index_mut(&mut self, index: usize) -> &mut $ty {
                match index {
                    0 => &mut self.x,
                    1 => &mut self.y,
                    2 => &mut self.z,
                    3 => &mut self.w,
                    _ => panic!("Vector index out of range: {index}"),
                }
            }
        }
    };
}

define_vector!(Vector4, Vector4Mask, f32);
define_vector!(Vector4d, Vector4dMask, f64);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic_is_component_wise() {
        let a = Vector4::new(1.0, 2.0, 3.0, 4.0);
        let b = Vector4::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(a + b, Vector4::new(6.0, 8.0, 10.0, 12.0));
        assert_eq!(b - a, Vector4::new(4.0, 4.0, 4.0, 4.0));
        assert_eq!(a * b, Vector4::new(5.0, 12.0, 21.0, 32.0));
        assert_eq!(b / a, Vector4::new(5.0, 3.0, 7.0 / 3.0, 2.0));
        assert_eq!(a * 2.0, Vector4::new(2.0, 4.0, 6.0, 8.0));
        assert_eq!(-a, Vector4::new(-1.0, -2.0, -3.0, -4.0));
    }

    #[test]
    fn dot_and_cross_products() {
        let x = Vector4::UNIT_X;
        let y = Vector4::UNIT_Y;
        assert_eq!(x.dot3(y), 0.0);
        assert_eq!(x.dot3(x), 1.0);
        assert_eq!(x.dot4(Vector4::UNIT_W), 0.0);

        assert_eq!(x.cross(y), Vector4::UNIT_Z);
        assert_eq!(Vector4::UNIT_Y.cross(Vector4::UNIT_Z), Vector4::UNIT_X);
        assert_eq!(Vector4::UNIT_Z.cross(Vector4::UNIT_X), Vector4::UNIT_Y);

        // Cross product ignores `w` and writes 0 there.
        let a = Vector4::new(1.0, 0.0, 0.0, 7.0);
        let b = Vector4::new(0.0, 1.0, 0.0, 9.0);
        assert_eq!(a.cross(b), Vector4::new(0.0, 0.0, 1.0, 0.0));
    }

    #[test]
    fn lengths_and_normalization() {
        let v = Vector4::new(3.0, 4.0, 0.0, 0.0);
        assert_eq!(v.length_squared3(), 25.0);
        assert_eq!(v.length3(), 5.0);
        let n = v.normalized3();
        assert!(n.all_equal3(Vector4::new(0.6, 0.8, 0.0, 0.0), 1e-6));

        let zero = Vector4::ZERO;
        assert_eq!(zero.normalized3(), Vector4::ZERO, "zero maps to zero");
        assert_eq!(zero.normalized4(), Vector4::ZERO);

        let v4 = Vector4::new(1.0, 1.0, 1.0, 1.0);
        assert_eq!(v4.length4(), 2.0);
        let n4 = v4.normalized4();
        assert!(n4.all_equal(Vector4::splat(0.5), 1e-6));
    }

    #[test]
    fn comparison_masks_and_select() {
        let a = Vector4::new(1.0, 5.0, 2.0, -1.0);
        let b = Vector4::new(2.0, 4.0, 2.0, 0.0);

        let lt = a.less(b);
        assert_eq!(lt.bits(), 0b1001, "x and w compare less");
        assert!(!lt.all());
        assert!(lt.any());
        assert!(!lt.all_xyz());
        assert!(lt.any_xyz());

        let ge = a.greater_equal(b);
        assert_eq!(ge.bits(), 0b0110, "y and z compare greater-or-equal");

        let eq = a.equal(b);
        assert_eq!(eq.bits(), 0b0100);

        let selected = Vector4::select(lt, Vector4::splat(9.0), Vector4::splat(3.0));
        assert_eq!(selected, Vector4::new(9.0, 3.0, 3.0, 9.0));

        let and = lt.and(Vector4Mask::XYZ);
        assert_eq!(and.bits(), 0b0001);
        let not = !lt;
        assert_eq!(not.bits(), 0b0110);
    }

    #[test]
    fn min_max_abs_and_interpolate() {
        let a = Vector4::new(-1.0, 2.0, -3.0, 4.0);
        let b = Vector4::new(5.0, -6.0, 7.0, -8.0);
        assert_eq!(a.min(b), Vector4::new(-1.0, -6.0, -3.0, -8.0));
        assert_eq!(a.max(b), Vector4::new(5.0, 2.0, 7.0, 4.0));
        assert_eq!(a.abs(), Vector4::new(1.0, 2.0, 3.0, 4.0));

        let start = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let end = Vector4::new(10.0, 20.0, 30.0, 40.0);
        let mut result = Vector4::ZERO;
        result.set_interpolate(start, end, 0.5);
        assert_eq!(result, Vector4::new(5.0, 10.0, 15.0, 20.0));
    }

    #[test]
    fn index_and_components() {
        let mut v = Vector4::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(v[0], 1.0);
        assert_eq!(v.component(3), 4.0);
        v[1] = 9.0;
        v.set_component(2, 8.0);
        assert_eq!(v, Vector4::new(1.0, 9.0, 8.0, 4.0));
        assert_eq!(Vector4::basis(2), Vector4::UNIT_Z);
        assert_eq!(Vector4::from_array(v.to_array()), v);
    }

    #[test]
    fn f64_vector_matches_f32_semantics() {
        let a = Vector4d::new(1.0, 2.0, 3.0, 4.0);
        let b = Vector4d::new(2.0, 2.0, 2.0, 2.0);
        assert_eq!(a.dot4(b), 20.0);
        assert_eq!(
            a.cross(Vector4d::UNIT_Y),
            Vector4d::new(-3.0, 0.0, 1.0, 0.0)
        );
        assert_eq!(a.length3(), (14.0f64).sqrt());
    }

    #[test]
    fn alignment_is_16_bytes() {
        assert_eq!(core::mem::align_of::<Vector4>(), 16);
        assert_eq!(core::mem::size_of::<Vector4>(), 16);
        assert_eq!(core::mem::align_of::<Vector4d>(), 16);
        assert_eq!(core::mem::size_of::<Vector4d>(), 32);
    }
}
