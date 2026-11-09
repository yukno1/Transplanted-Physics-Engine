//! `hknp-body` — hknp bodies and motion.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/{Body,Motion,Material}/`.
//!
//! Planned contents (migration Phase 6):
//! - `Body` (port of `hknpBody`) and `BodyCinfo` builder
//! - `BodyManager` (index-pool based lifetime management)
//! - `Motion` / `MotionProperties` / `MotionPropertiesLibrary`
//! - `BodyQuality` / `BodyQualityLibrary`
//! - `Material` / `MaterialLibrary`
//!
//! C++ raw pointers between bodies/motions/shapes become ID handles stored
//! in `Vec`-backed managers, which maps well onto Rust's borrow checker.
//!
//! This crate is currently a scaffold.
