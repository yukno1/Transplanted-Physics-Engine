//! `hk-math` — vectors, quaternions, matrices, transforms and AABBs.
//!
//! Rust port of the core math layer in
//! `original/Common/Source/Common/Base/Math/` (migration Phase 2, README §4).
//!
//! # Scope of this phase
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Vector4`], [`Vector4d`] | `hkVector4f`, `hkVector4d` | four-component float/double vectors, 16-byte aligned storage |
//! | [`Vector4Mask`], [`Vector4dMask`] | `hkVector4fComparison`, `hkVector4dComparison` | per-component comparison masks |
//! | [`Quaternion`] | `hkQuaternionf` | unit quaternion `(x, y, z, w)`, Hamilton product |
//! | [`Matrix3`] | `hkMatrix3`, `hkRotationf` | 3×3 column-major rotation matrix (columns are `Vector4`) |
//! | [`Transform`] | `hkTransformf` | rotation matrix + translation |
//! | [`QTransform`] | `hkQTransformf` | quaternion + translation |
//! | [`QsTransform`] | `hkQsTransformf` | translation + quaternion + scale |
//! | [`Aabb`] | `hkAabb` | axis-aligned bounding box, `min`/`max` as `Vector4` |
//! | [`Half`] | `hkFloat16` / `hkHalf16` | 16-bit float with the SDK's round-to-nearest conversion |
//!
//! Not ported in this phase: `hkVector2f/d`, `hkIntVector`, `hkPackedVector3`,
//! `hkMatrix4f`, `hkMatrix6*`, `hkSimdFloat32/d` as standalone wrappers, `hkMathFuncs`
//! and the per-platform SIMD backends (`Sse`, `Avx`, `Mx`, `Fpu` directories).
//!
//! # Determinism
//!
//! Every struct is plain IEEE-754 data plus arithmetic written in the same order as
//! the C++ scalar (`Fpu`) formulations: no FMA contraction, no reassociation. `sqrt`
//! and the trigonometric functions come from Rust's `std` (the C++ SDK uses the
//! platform libm/sincos as well), so bit-exactness across compilers is only
//! guaranteed for `+ - * /` and comparisons, not for `sin`/`cos`/`sqrt`-derived
//! values (tests use tolerances there, and Phase 9 golden traces are the final
//! arbiter for simulation data).
//!
//! # Module layout
//!
//! * [`vector`] — `Vector4`, `Vector4d` and comparison masks.
//! * [`quaternion`] — `Quaternion`.
//! * [`matrix3`] — `Matrix3` (rotation matrices).
//! * [`transform`] — `Transform` (matrix + translation).
//! * [`qtransform`] — `QTransform` (quaternion + translation).
//! * [`qstransform`] — `QsTransform` (quaternion + translation + scale).
//! * [`aabb`] — `Aabb`.
//! * [`half`] — `Half` 16-bit float.

pub mod aabb;
pub mod half;
pub mod matrix3;
pub mod qstransform;
pub mod qtransform;
pub mod quaternion;
pub mod transform;
pub mod vector;

pub use aabb::Aabb;
pub use half::Half;
pub use matrix3::Matrix3;
pub use qstransform::QsTransform;
pub use qtransform::QTransform;
pub use quaternion::Quaternion;
pub use transform::Transform;
pub use vector::{Vector4, Vector4Mask, Vector4d, Vector4dMask};

/// `HK_REAL_PI` — π as `f32`.
pub const PI: f32 = core::f32::consts::PI;
/// `2π`.
pub const TWO_PI: f32 = 2.0 * PI;
/// `π / 2`.
pub const HALF_PI: f32 = PI * 0.5;
/// `HK_REAL_EPSILON` — smallest step at `1.0` (`f32::EPSILON`).
pub const EPSILON: f32 = f32::EPSILON;
/// `HK_REAL_MAX` — largest finite `f32`.
pub const MAX_REAL: f32 = f32::MAX;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_are_reasonable() {
        assert_eq!(PI, core::f32::consts::PI);
        assert_eq!(TWO_PI, 2.0 * PI);
        assert_eq!(HALF_PI * 2.0, PI);
        assert_eq!(MAX_REAL, f32::MAX);
    }
}
