//! `proton` — Havok Physics 2021.2 的 Rust 单 crate 实现。
//!
//! 公共 API 以 `proton::*` 别名暴露，
//! 用户代码与既有集成测试无需感知内部结构变化。
//!
//! 设计准则：不追求与 C++ 1:1 结构复刻，优先使用 Rust 语言特性
//! （`enum` / `Option` / `Result` / trait / 迭代器 / RAII）；仅在数值热点与
//! `hvk-trace v1` 基线处保留确定性约束。详见 `README.md` §2、§3。

extern crate self as proton;

pub mod convert;

pub mod base_container;
pub mod base_geometry;
pub mod base_math;
pub mod base_memory;
pub mod base_serialize;
pub mod base_thread;
pub mod base_types;
pub mod phys_body;
pub mod phys_collide;
pub mod phys_extensions;
pub mod phys_shape;
pub mod phys_solver;
pub mod phys_types;
pub mod phys_world;

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
