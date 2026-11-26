//! `base-math` — vectors, quaternions, matrices, transforms and AABBs.
//!
//! Rust port of the core math layer in
//! `original/Common/Source/Common/Base/Math/` (migration Phase 2, README §4).
//!
//! # glam transition layer
//!
//! The engine is migrating its math layer onto [`glam`] (see
//! `docs/refactor-glam-plan.md`). This module currently provides:
//!
//! * **glam re-exports** — the canonical types the codebase migrates to
//!   ([`Vec4`], [`Vec3A`], [`Quat`], [`Mat3`], [`Affine3A`], [`BVec4`],
//!   [`DVec4`]). New code should prefer these directly.
//! * **legacy SDK-parity types** — [`Vector4`], [`Vector4d`], [`Quaternion`],
//!   [`Matrix3`], [`Transform`], kept scalar and bit-faithful to the C++
//!   `Fpu` formulations until callers are migrated (Phase 3).
//! * **retained custom types with no glam equivalent** — [`Aabb`] and
//!   [`PreciseTransform`].
//!
//! # Scope of the legacy layer
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Vector4`], [`Vector4d`] | `hkVector4f`, `hkVector4d` | four-component float/double vectors, 16-byte aligned storage |
//! | [`Vector4Mask`], [`Vector4dMask`] | `hkVector4fComparison`, `hkVector4dComparison` | per-component comparison masks |
//! | [`Quaternion`] | `hkQuaternionf` | unit quaternion `(x, y, z, w)`, Hamilton product |
//! | [`Matrix3`] | `hkMatrix3`, `hkRotationf` | 3×3 column-major rotation matrix (columns are `Vector4`) |
//! | [`Transform`] | `hkTransformf` | rotation matrix + translation |
//! | [`PreciseTransform`] | `hkPreciseTransform` | rotation matrix + double-float translation |
//! | [`Aabb`] | `hkAabb` | axis-aligned bounding box |
//!
//! Not ported (glam covers them or they are unused): `hkVector2f/d`,
//! `hkIntVector`, `hkPackedVector3`, `hkMatrix4f`, `hkMatrix6*`,
//! `hkSimdFloat32/d` as standalone wrappers, `hkMathFuncs`, the per-platform
//! SIMD backends, `hkQTransformf`/`hkQsTransformf` (unused in this port) and
//! `hkFloat16` (unused in this port).
//!
//! # Determinism
//!
//! The legacy structs are plain IEEE-754 data plus arithmetic written in the
//! same order as the C++ scalar (`Fpu`) formulations: no FMA contraction, no
//! reassociation. `sqrt` and the trigonometric functions come from Rust's
//! `std` (the C++ SDK uses the platform libm/sincos as well), so bit-exactness
//! across compilers is only guaranteed for `+ - * /` and comparisons, not for
//! `sin`/`cos`/`sqrt`-derived values (tests use tolerances there, and Phase 9
//! golden traces are the final arbiter for simulation data).
//!
//! # Module layout
//!
//! * [`vector`] — legacy `Vector4`, `Vector4d` and comparison masks (scalar
//!   path; the former SSE2 helper was retired in favor of glam).
//! * [`quaternion`] — legacy `Quaternion`.
//! * [`matrix3`] — legacy `Matrix3` (rotation matrices).
//! * [`transform`] — legacy `Transform` (matrix + translation).
//! * [`precise_transform`] — [`PreciseTransform`] (matrix + double-float
//!   translation, glam-backed storage).
//! * [`aabb`] — [`Aabb`] (glam-backed storage).

pub mod aabb;
pub mod mat3;
// pub mod matrix3;
pub mod precise_transform;
pub mod quat;
// pub mod quaternion;
pub mod transform;
pub mod vec4;
// pub mod vector;

// glam re-exports: the canonical math types going forward
// (`docs/refactor-glam-plan.md`, Phase 2 transition layer).
pub use glam::{Affine3A, BVec4, DVec4, Mat3, Quat, Vec3A, Vec4};

pub use aabb::Aabb;
pub use mat3::Mat3Ext;
// pub use matrix3::Matrix3;
pub use precise_transform::PreciseTransform;
pub use quat::{QuatExt, quat_mul, quat_normalize};
// pub use quaternion::Quaternion;
pub use transform::Transform;
pub use vec4::Vec4Ext;
// pub use vector::{Vector4, Vector4Mask, Vector4d, Vector4dMask};

#[cfg(test)]
#[path = "tests/simd_parity.rs"]
mod simd_parity;
