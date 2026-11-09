//! `hknp-types` — hknp core types.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/hknpTypes.h` and the
//! ID headers under `Dynamics/{Body,Motion,Material,Constraint}/`.
//!
//! Planned contents (migration Phase 5):
//! - handles: `BodyId`, `BodyIndex`, `MotionId`, `MaterialId`,
//!   `BodyQualityId`, `MotionPropertiesId`, `ConstraintId`,
//!   `ImmediateConstraintId`
//! - `MotionType` (STATIC / KEYFRAMED / DYNAMIC)
//! - collision flags, `CommandDispatchType`
//! - `hknpHalf` helper (float16/float32 switch)
//!
//! This crate is currently a scaffold.
