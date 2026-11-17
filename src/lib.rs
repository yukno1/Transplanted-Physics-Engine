//! `havok` — facade crate re-exporting the public API of the Rust port.
//!
//! Main entry point for users of the port, mirroring how the original SDK
//! exposes the umbrella headers `Common/Base/hkBase.h` and
//! `Physics/Physics/hknpPhysics.h`.
//!
//! This crate is currently a scaffold; re-exports grow as phases land.

pub use base_container as container;
pub use base_geometry as geometry;
pub use base_math as math;
pub use base_memory as memory;
pub use base_serialize as serialize;
pub use base_thread as thread;
pub use base_types as types;
pub use phys_body as body;
pub use phys_collide as collide;
pub use phys_extensions as extensions;
pub use phys_shape as shape;
pub use phys_solver as solver;
pub use phys_types as np_types;
pub use phys_world as world;
