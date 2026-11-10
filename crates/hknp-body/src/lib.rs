//! `hknp-body` — hknp bodies and motion.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/{Body,Motion}/`
//! (migration Phase 6, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Body`] | `hknpBody` | cached transform/AABB, shape, motion id, flags, ids |
//! | [`BodyCinfo`] | `hknpBodyCinfo` | builder for body construction |
//! | [`BodyManager`] | `hknpBodyManager` | index pool with 8-bit serial, reuses slots |
//! | [`Motion`] | `hknpMotion` | orientation, center of mass, linear/angular velocity, mass/inertia |
//! | [`MotionManager`] | `hknpMotionManager` | motion slot pool, static motion lives at id 0 |
//!
//! The C++ runtime stores raw pointers (`m_shape`, body/motion buffers) and
//! relies on `hknpThreadSafeObjectPool` free lists. Here bodies and motions live
//! in typed managers keyed by [`hknp_types::BodyId`] / [`hknp_types::MotionId`];
//! shapes are shared through `Arc<dyn Shape>`.
//!
//! # Simplifications in this phase
//!
//! - `hknpBody::m_transform` is a `hkPreciseTransform` (double translation). The
//!   port uses the f32 [`hk_math::Transform`] for now; precision migration is
//!   scheduled
//!   with the large-world/shift work (Phase 10).
//! - Motion angular velocity is stored in **world space** (the SDK stores it in
//!   motion/inertia space and converts on access). The integration math
//!   (`q' = Δq * q`, normalized) is equivalent for a world-space input.
//! - `mass`/`inverse_inertia` are stored as plain `f32`/`Vector4`; the SDK's
//!   packed `hknpHalf` representation and `hknpMotionPropertiesLibrary` are
//!   deferred to the mass-properties/solver work.
//! - Attached bodies (`allocateAttachedBody`), drag, deactivation, solver ids,
//!   space-splitter cells and previous-step velocity bookkeeping are not ported.
//! - `hknpBodyManager` body slots carry an 8-bit serial (`hknpBodyIndex`); a stale
//!   `BodyId` after slot reuse is rejected, matching the C++ validity rule.
//!   `MotionId` is a plain u32 slot index in the SDK, so stale motion ids are
//!   **not** detected by a serial; the manager documents this explicitly.

pub mod body;
pub mod body_manager;
pub mod motion;
pub mod motion_manager;

pub use body::{Body, BodyCinfo};
pub use body_manager::BodyManager;
pub use motion::Motion;
pub use motion_manager::MotionManager;
