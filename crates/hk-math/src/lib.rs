//! `hk-math` — Havok math library.
//!
//! Rust port of `original/Common/Source/Common/Base/Math/`.
//!
//! Planned contents (migration Phase 2):
//! - `Vec2/Vec3/Vec4` (f32/f64), hand-ported from `hkVector4f`/`hkVector4d`
//! - `SimdFloat32`/`SimdDouble64` backends (port of the SSE/FPU/AVX switch):
//!   `std::arch` intrinsics on x86_64 with a scalar fallback
//! - `Matrix3`/`Matrix4`, `Quaternion`, `QsTransform`/`QTransform`/`Transform`
//! - `Aabb`, comparison/mask types, `Half16`
//!
//! No external math crate (e.g. glam) is used: bit-exact float32
//! reproduction is a hard requirement of the port.
//!
//! This crate is currently a scaffold.
