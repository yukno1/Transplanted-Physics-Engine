//! `havok` — facade crate re-exporting the public API of the Rust port.
//!
//! Main entry point for users of the port, mirroring how the original SDK
//! exposes the umbrella headers `Common/Base/hkBase.h` and
//! `Physics/Physics/hknpPhysics.h`.
//!
//! This crate is currently a scaffold; re-exports grow as phases land.

pub use hk_container as container;
pub use hk_geometry as geometry;
pub use hk_math as math;
pub use hk_memory as memory;
pub use hk_thread as thread;
pub use hk_types as types;
pub use hknp_body as body;
pub use hknp_collide as collide;
pub use hknp_shape as shape;
pub use hknp_solver as solver;
pub use hknp_types as np_types;
pub use hknp_world as world;
