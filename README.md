# Havok Physics 2021.2 → Rust 迁移项目

本项目旨在将 **Havok Physics SDK 2021.2**（BUILD #20211202，C++）迁移为 Rust 实现。

**目录边界**

- `havok/`（本仓库）：Rust 目标工程（workspace）。
- `../original/`（同级目录，**不纳入本仓库 / git**）：原始 C++ SDK 源码（约 5861 个文件）。

> **快速开始**：需要 Rust（edition 2024 / MSRV 1.85），在仓库根目录执行
> `cargo test --workspace`。完整命令与仓库约定见第 7 节。

## 0. 快速上手

用户侧通过门面 crate `proton` 使用（依赖 `proton-engine` 即可）：

```rust,ignore
use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::math::Vector4;
use proton::np_types::MotionType;
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::world::{World, WorldCinfo};

let mut world = World::new(WorldCinfo::new());
world.set_gravity(Vector4::from_xyz(0.0, -9.81, 0.0));

// 静态地面（半长 10 x 0.5 x 10 的盒子，顶面位于 y = 0）
let ground = BoxShape::with_convex_radius(Vector4::from_xyz(10.0, 0.5, 10.0), 0.0);
world.create_body(
    &BodyCinfo::new(Arc::new(ground) as Arc<dyn Shape>)
        .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
);

// 动态球：从 y = 10 自由下落
let ball = SphereShape::new(Vector4::ZERO, 0.5);
let body = world.create_body(
    &BodyCinfo::new(Arc::new(ball) as Arc<dyn Shape>)
        .with_motion_type(MotionType::Dynamic)
        .with_mass(1.0)
        .with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
);

for _ in 0..60 {
    world.step(1.0 / 60.0);
}

let position = world.body_position(body).unwrap();
let velocity = world.body_linear_velocity(body).unwrap();
```

更多完整可运行的场景见 `tests/`（自由落体解析解、堆叠、摩擦滑动、约束/查询、休眠与并行求解等）与基准 `benches/simulation.rs`。

---

## 1. 原始项目结构（`../original/`，不纳入本仓库）

```
original/                        # 统计日期：2026-11-09
├── Common/                      # 基础库 + 内容工具链（4439 文件）
│   ├── Source/
│   │   ├── Common/              # SDK 核心基础库
│   │   │   ├── Base/            # 类型 / 数学 / 内存 / 线程 / 反射 / 序列化
│   │   │   │   ├── Algorithm/   # 哈希、排序、查找
│   │   │   │   ├── Container/   # 数组、块流（hkBlockStream）
│   │   │   │   ├── Math/        # 向量 / 矩阵 / 四元数 / 变换（267 文件）
│   │   │   │   ├── Memory/      # 自定义内存系统与分配器
│   │   │   │   ├── Monitor/     # 性能监视流
│   │   │   │   ├── Reflect/     # 运行时反射（序列化基础设施）
│   │   │   │   ├── System/      # 错误、断言、入口
│   │   │   │   ├── Thread/      # 原子、自旋锁、线程池、任务队列
│   │   │   │   └── Types/       # hkBaseTypes、hkRefPtr、hkHandle、hkUFloat8
│   │   │   ├── Compat/          # 已废弃 API 兼容层（498 文件，迁移整体跳过）
│   │   │   ├── GeometryUtilities/  # 网格处理（凸包生成、焊接等）
│   │   │   ├── SceneData/       # 场景图 / 蒙皮网格数据
│   │   │   ├── Serialize/       # hkx / tagfile / packfile 序列化
│   │   │   ├── Utilities/
│   │   │   └── Visualize/       # VDB 远程调试可视化（146 文件）
│   │   ├── ContentTools/        # Maya / 3ds Max 导出插件管线（1037 文件）
│   │   └── Platform/
│   ├── Test/                    # Common 单元测试（200 文件）
│   ├── Tools/                   # C# 工具链（Visual Debugger 等）
│   └── ThirdParty/
└── Physics/                     # 物理引擎 hknp（1422 文件）
    ├── Source/Physics/Physics/
    │   ├── Base/                # 线程安全检查
    │   ├── Collide/             # 碰撞检测
    │   │   ├── BroadPhase/      # 网格宽相 + SimdTree
    │   │   ├── NarrowPhase/     # Manifold、GJK 单纯形、检测器
    │   │   ├── Query/           # RayCast / Aabb / ClosestPoints 查询
    │   │   ├── Filter/          # 碰撞过滤
    │   │   └── Shape/           # 形状体系（Convex / Composite / Decorator，173 文件）
    │   ├── Dynamics/
    │   │   ├── Body/            # hknpBody / BodyManager / BodyQuality
    │   │   ├── Motion/          # 运动属性
    │   │   ├── Constraint/      # 约束系统（136 文件）
    │   │   ├── Simulation/      # hknpSimulation（单线程 / 多线程 + 任务图）
    │   │   ├── Solver/          # 约束求解器、接触求解、积分器
    │   │   └── World/           # hknpWorld（世界入口）
    │   ├── Extensions/          # Ragdoll / Vehicle / CharacterControl 等（303 文件）
    │   └── Particles/           # 粒子碰撞器
    └── Test/UnitTest/           # 物理单元测试（41 个 cpp）
```

### 核心 API 入口

| C++ 组件 | 位置 | 职责 |
|---|---|---|
| `hknpWorld` | `Physics/Source/Physics/Physics/Dynamics/World/hknpWorld.h` | 世界对象：重力、碰撞过滤器、SimulationStage 生命周期 |
| `hknpBody` | `.../Dynamics/Body/hknpBody.h` | 刚体结构（IS_STATIC / IS_DYNAMIC / IS_KEYFRAMED、WorldAddedState） |
| `hknpSimulation` | `.../Dynamics/Simulation/hknpSimulation.h` | 抽象步进基类：`collide()` / `solve()` 纯虚函数 |
| `hknpTypes` | `Physics/Source/Physics/Physics/hknpTypes.h` | 全局 typedef 与 `HK_DECLARE_HANDLE`（ShapeKey / SolverId / ImmediateConstraintId）；Body/Motion/Material 等 Id 句柄定义在各自模块头文件 |
| `hkBaseTypes` | `Common/Source/Common/Base/Types/hkBaseTypes.h` | hkPair、位操作、hkBool、hkHalf16；同目录另有 `hkHandle.h` / `hkRefPtr.h` / `hkUFloat8.h` / `hkSignalSlots.*` |
| Math | `Common/Source/Common/Base/Math/` | hkVector4f/d（Sse / Fpu / Mx / Avx 后端）、Quaternion、QsTransform、Aabb |

---

## 2. 迁移策略

**不做 1:1 全量直译。** 原 SDK 规模百万行级，含工具链、序列化、可视化、内容导出。按优先级分三层：

| 优先级 | 范围 | 说明 |
|---|---|---|
| P0 必迁 | `Common/Base` + Physics 运行时（hknp 核心） | 物理仿真主链路 |
| P1 后迁 | 序列化（hkx/packfile）、Extensions（Ragdoll/Vehicle/Character） | 功能扩展 |
| P2 暂不迁 | `Compat`（废弃层）、ContentTools、Tools（C#）、Visualize（VDB） | 非运行时必需 |

---

## 3. 目标 Rust Workspace 结构

```
havok/                             # 本仓库根目录（workspace 根 + 门面 crate）
├── Cargo.toml                     # [workspace] members = ["crates/*"] + 门面 crate proton-engine
├── src/lib.rs                     # proton 门面：重导出公共 API
├── benches/                       # 基准 harness（随门面 crate）
├── tests/                         # 集成测试（随门面 crate）
├── docs/                          # 阶段文档（测试矩阵等）
├── LICENSE
├── README.md
└── crates/
```

> **命名约定（2026-11-16 已完成）**：门面 crate 改名 `proton-engine`（lib 名 `proton`，取代原 `havok`）；crate 前缀 `hk-* → base-*`、`hknp-* → phys-*`。目录、`Cargo.toml`、代码内引用（`hk_types → base_types` 等）已全部重命名，`cargo test --workspace` 全绿。

| crate | 职责 | 状态（2026-11-16） |
|---|---|---|
| `base-types` | 基础类型：hkReal、句柄、RefPtr、位运算、错误/断言 | ✅ 已实现（Phase 1） |
| `base-math` | 向量/矩阵/四元数/变换/Aabb（**自研直译 + SIMD**，不用 glam） | ✅ 已实现（标量后端，Phase 2；SIMD 待优化） |
| `base-memory` | 内存系统：分配器接口、系统堆分配器、分配统计 | ✅ 已实现（Phase 3） |
| `base-container` | hkArray / 块流 / 确定性 HashMap、HashSet | ✅ 已实现（Phase 3） |
| `base-thread` | 线程池、任务图、自旋锁（**std::thread，零依赖**） | ✅ 已实现（Phase 4） |
| `base-geometry` | GeometryUtilities：凸包、网格焊接 | 🚧 骨架 |
| `base-serialize` | HFF 结构化流 + Tagfile 容器解析 + 类型元数据/对象重建子集 | ✅ 已实现（Phase 10/16 子集） |
| `phys-types` | hknpTypes：BodyId/BodyIndex、句柄、MotionType/ShapeType、标志位 | ✅ 已实现（Phase 5） |
| `phys-shape` | 形状体系：统一 Shape trait + 凸体/复合/网格/高度场 | ✅ 已实现（Phase 5） |
| `phys-collide` | 宽相/窄相/Manifold + 查询（raycast/shape cast/closest points/AABB，含 mesh/height field/compound 遍历）+ 碰撞过滤（`CollisionFilter` trait、group layer/subsystem/system-group filter、pair 禁用表） | ✅ 已实现（Phase 7/11/14） |
| `phys-body` | hknpBody/BodyManager/Motion/MotionManager | ✅ 已实现（Phase 6） |
| `phys-solver` | 迭代式接触求解器 + warm starting（`ContactImpulse`/`JointImpulse` 累积冲量）+ 关节约束（ball-socket/hinge/fixed/ragdoll/prismatic/spring/point-to-plane/wheel，含 limits 与 velocity/position/spring 电机）+ 岛内子集求解 API | ✅ 已实现（Phase 8/11/12） |
| `phys-world` | hknpWorld 组装 + collide→islands→solve(contacts+joints)→integrate 管线 + 接触冲量缓存 + 关节冲量槽缓存 + 约束管理器（含 wheel：悬挂弹簧/行程限位/转向/驱动）+ 查询 API + 岛级休眠/唤醒 + 按岛并行求解 | ✅ 已实现（Phase 8/11/12） |
| `phys-extensions` | WorldHash + Vehicle 扩展层子集（raycast 悬挂 / 默认 engine / transmission / 简化 friction solver） | 🚧 部分实现（Phase 10/13；Ragdoll/CharacterControl 仍阻塞） |
| `proton` | 门面 crate（workspace 根，`proton-engine`），重导出公共 API + `convert` 模块（hkx Value 树 → shape/body/world，见 Phase 17） | ✅ 已迁至根目录（重导出已接好） |

依赖方向严格单向：`types → math → memory → container → ... → world → extensions`。

---

## 4. 分阶段迁移计划

### Phase 0：工程脚手架（✅ 已完成）
- 将 `havok/` 改造为虚拟 workspace（`[workspace]` + `crates/*`，共 15 个 crate）
- `.gitignore` 忽略 `/target`
- 检查命令：`cargo fmt`、`clippy -D warnings`、`test`、`doc -D warnings`、release build（CI 未配置，本地执行，见第 7 节）
- 基础依赖决策：数学库定为**从 C++ 直译**（不用 glam，保证 float32 确定性）；`rayon`/`thiserror`/`bytemuck` 按阶段引入

### Phase 1：`base-types`（✅ 已完成，2026-11-09）
交付位于 `crates/base-types/src/`（49 个单元测试 + 13 个文档测试）：

| 模块 | 内容 |
|---|---|
| `scalar.rs` | `Real = f32`（HK_REAL_IS_FLOAT）、`Time`、定宽整数、`Ulong = u64`、`Bool32/64` |
| `bitops.rs` | `set_bits/clear_bits`（无符号截断语义）、`bitmask/bitmask64`、`next/previous_power_of_two`、`bit_cast`、`align` |
| `pair.rs` | `Pair`（`repr(C)`，字段序对齐 `m_0/m_1`） |
| `handle.rs` | `Handle<Tag, INVALID>` + `declare_handle!` 宏（编译期隔离句柄族） |
| `refptr.rs` | **自定义侵入式引用计数**（`RefCount` + `RefCounted` + `RefPtr`），非 `Arc`/`Rc` |
| `ufloat8.rs` | 256 项表逐字移植、7 步二分编码、`decode_float` 位技巧 |
| `signal.rs` | 观察者模式；支持触发中退订、嵌套 emit、触发中新增订阅延迟到下一轮 |
| `result.rs` | `HkResult` 与 `HK_E_*` 原始错误码 |

偏差与决策：
- `Handle` 存储 `u32`；`RefCount` 用 `u32`（原 `hkUlong` 为 64 位，布局差异已在模块文档标注）
- `HK_ASSERT` → `hk_assert!`（`debug_assert!` 语义）、`hkOptional` → `Option`、`hkBool` → `bool`
- `PowerOfTwo` trait 方法命名为 `power_of_two_ceil/floor`，避免被标准库固有方法遮蔽（`0` 的语义与 C++ 一致）
- 跳过反射宏、`hkVarArgs`、`hkPtrAndInt`、`hkHandleMap`
- 验证：`cargo fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 2：`base-math`（✅ 核心已完成，2026-11-09；标量后端）
交付位于 `crates/base-math/src/`（52 个单元测试 + 5 个文档测试）：

| 模块 | 内容 |
|---|---|
| `vector.rs` | `Vector4`（`hkVector4f`）/ `Vector4d`：16 字节对齐存储、`+ - * /`、dot/cross、length/normalize、min/max/abs、interpolate、比较掩码与 select |
| `quaternion.rs` | `Quaternion`：Hamilton 乘法、`_setRotatedDir` 旋转公式、逆、归一化、轴角构造、与旋转矩阵互转 |
| `matrix3.rs` | `Matrix3`（列主序，`hkRotationf` 存储）：矩阵/向量乘法、转置、逆（SDK 的缩放 epsilon 判据 + 奇异置零）、四元数互转 |
| `transform.rs` | `Transform`（`hkTransformf`）：旋转+平移组装、`setInverse` 语义、点/方向变换 |
| `qtransform.rs` | `QTransform`（`hkQTransformf`）：四元数 + 平移 |
| `qstransform.rs` | `QsTransform`（`hkQsTransformf`）：缩放/旋转/平移，`setMul` 与 `setMulScaled` 两种复合语义 |
| `aabb.rs` | `Aabb`（`hkAabb`）：overlaps/contains/containsPoint、expand/include、中心/半长/体积、顶点/支撑点 |
| `half.rs` | `Half`（`hkFloat16`）：C++ 位运算逐字移植，含 round-to-nearest 与去规范化 |

决策与偏差：
- **先标量后端**：直接按 C++ 标量（Fpu）公式书写运算顺序，暂不引入 SSE/AVX；公开 API 不暴露平台类型，后续替换后端无需改签名
- SIMD 优化推迟到 Phase 9 基准之后：SSE2 为基线，AVX/NEON 可选；每个后端必须跑同一组 bit-exact 用例（`-0.0`、denormal、round-to-nearest），并与标量后端对齐
- `sqrt`/`sin`/`cos` 来自 Rust 标准库：`+ - * /` 与比较逐位等价，超越函数仅保证容差（C++ 端同样依赖平台 libm；最终以 Phase 9 黄金轨迹为准）
- 未纳入本阶段：`hkVector2f/d`、`hkIntVector`、`hkPackedVector3`、`hkMatrix4f`、`hkMatrix6f/d`、`hkMathFuncs`、独立 `hkSimdFloat32/d` 包装

### Phase 3：`base-memory` + `base-container`（✅ 已完成，2026-11-09）
交付（`base-memory` 8 个单元测试；`base-container` 27 个单元测试 + 5 个文档测试）：

| crate | 内容 |
|---|---|
| `base-memory` | `Allocator` trait（`Layout` 风格 `alloc`/`alloc_zeroed`/`dealloc`/`realloc`）、`AllocStats`、`SystemAllocator`（转发全局分配器 + 原子统计/峰值）、`global()` 进程实例；失败即 panic，与 SDK 的 fail-fast 语义一致 |
| `base-container` | `Array`（`hkArray`：`removeAt` 无序换尾、`removeAtAndCopy` 有序前移、`reserve`/`expandBy`/`insertAt`/`append`/`optimizeCapacity`/`indexOf`）、`BlockStream`（`hkBlockStream` 单线程子集：4000 字节定长块、确定性迭代、随机访问、`clear` 复用块、字节统计）、`HashMap`/`HashSet`（`std::collections` + 固定种子 FNV-1a；`insert`/`tryInsert`/`tryGet`/`getOrInsertDefault`/`getWithDefault`/`at`） |

决策与偏差：
- **保持零外部依赖**：`hashbrown` 暂缓，使用 `std::collections`；禁用 `RandomState`，改固定种子 FNV-1a，保证跨运行可复现。迭代顺序确定，但**不等同于** C++ `hkHashBase` 的数组序；依赖顺序的代码请用 `Array`
- `Array` 底层直接用 `Vec<T>`（容量倍增与 SDK 一致）；保留 `as_ptr`/`as_mut_ptr` 供 FFI 风格代码
- `BlockStream` 仅移植单线程 append/iterate/random-access 子集；reader/consumer/range、块头原子计数与线程局部块分配器留给 Phase 10
- 未纳入：`hkLocalArray`/`hkInplaceArray`、`hkArrayView`、`hkString*`、`hkPointerMap`/`hkRelArray`/`hkTree`、并发容器；内存路由、`hkMemoryDebug`、FreeList/Lifo/Thread allocator 按 README 策略待性能分析后再引入
- **核心原则**：指针访问全面改为 ID/索引访问，天然契合 Rust 借用检查

### Phase 4：`base-thread` + 任务系统（✅ 已完成，2026-11-10）
交付位于 `crates/base-thread/src/`（29 个单元测试 + 5 个文档测试）：

| 模块 | 内容 |
|---|---|
| `spin_lock.rs` | `SpinLock<T>`/`SpinGuard`：`compare_exchange` + `spin_loop()` 自旋，无公平性（与 C++ 一致） |
| `thread_pool.rs` | `ThreadPool`：固定 worker + 共享 `mpsc` 队列、`JobHandle`（完成等待/panic 检测）、池内 `parallel_for`；Drop 时排空队列后 join |
| `job_queue.rs` | `JobQueue`：`hkJobQueue`/`hkTaskQueue` 的单线程 FIFO 子集 |
| `task_graph.rs` | `TaskGraph`/`TaskId`/`Priority`：一次性闭包任务 + 依赖边；`execute_sequential` 按 **priority → id** 确定性调度；`execute_layered` 拓扑分层 + 线程池层内并行/层间屏障；环检测 |
| `parallel.rs` | 作用域 `parallel_for`（原子游标动态分发）与 `parallel_for_chunks`（确定性连续分块）；`num_hardware_threads` |

决策与偏差：
- **未引入 `rayon`**：实施时 crates.io 不可达（SSL 错误），且工作区保持零依赖；改用 `std::thread` 自建池。后续可通过同一 `ThreadPool`/`parallel_for` API 替换后端
- **确定性优先**：顺序执行固定按 priority→id；并行执行只在拓扑层内并发、层间 barrier，层内任务不得有共享写（与 SDK 多线程任务队列的约束一致）
- 未移植：`hkThread` 优先级/亲和性、`hkJobQueue` 工作窃取与任务类型队列、`hkSemaphore`/`ReadersWriterLock`/`hkCriticalSection`、`hkReferencedTask`/多样性/图合并
- `hknpMultithreadedSimulation` 与物理 `hknpTaskGraph`：Phase 10 已交付并行窄相子集（`step_with_threads`），完整任务图/求解器调度仍待
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 5：`phys-types` + `phys-shape`（✅ 已完成，2026-11-10）
交付（`phys-types` 16 个单元测试；`phys-shape` 41 个单元测试）：

| crate | 内容 |
|---|---|
| `phys-types` | `BodyId`（u64，MSB 区分内部索引/用户 ID）与 `BodyIndex`（24 位索引 + 8 位 serial）；`MotionId`/`MaterialId`/`ShapeKey`/`SolverId`/`ImmediateConstraintId`/`BodyQualityId`/`MotionPropertiesId`/`ConstraintSolverId` 等句柄（invalid 哨兵与 SDK 完全一致）；`MotionType`、完整 `ShapeType`、`CollisionDispatchType`；`CollisionFlags`/`ShapeFlags`/`WorldAddedState` |
| `phys-shape` | 统一 `Shape` trait（`shape_type`/`dispatch_type`/`flags`/`aabb`/`bounding_radius`/`calc_aabb`/`volume`/`support`/`is_convex`/`as_any`）；`SphereShape`、`BoxShape`、`CapsuleShape`、`CylinderShape`、`TriangleShape`、`ConvexHullShape`；`CompoundShape`（`ShapeInstance` + `Arc<dyn Shape>`）；`MeshShape`（顶点+三角形+中值分割 BVH，叶子 4 个三角形）；`HeightFieldShape`（XZ 网格 + 高度采样） |

决策与偏差：
- **Box**：本阶段为“局部轴对齐半长 + convex radius”，OBB 变换由 body/实例携带（Phase 6）；**Cylinder** 的 AABB 使用有限圆柱精确支撑公式（`h*|axis_i| + r*sqrt(1-axis_i²)`），而非保守球扫掠
- **凸体 support 语义**：盒取方向符号角点 + `radius*dir`、胶囊取两端点中的极值点 + `radius*dir`（共线 tie 取 A/正向轴）、圆柱为轴向分量 + 径向投影；**体积按名义实体计算**（不含 convex radius），与 SDK 的“几何 + 圆角分离”一致
- `ConvexHullShape` 未实现点云凸包构建（`hknpConvexHullUtil`），仅支持外部给定顶点 + 可选三角面；Mesh 暂不含 section/material/compressed-mesh；Compound 的 BVH 留 Phase 7
- `HeightFieldShape` 简化为 XZ 网格 + `(x,y,z)` scale，未含每 cell BVH/SDF；`hknpShapeFunctions` 的查询/GSK/face 接口留 Phase 7
- 未纳入：`hknpCompressedMeshShape`、`hknpExternMeshShape`、`hknpBreakableCompoundShape`、LOD、decorator/masked 形状、shape tag codec、shape key mask/iterator、`hknpShapeMassProperties`
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 6：`phys-body` + `phys-world`（✅ 已完成，2026-11-10）
交付（`phys-body` 18 个单元测试 + 1 个文档测试；`phys-world` 10 个单元测试 + 1 个文档测试）：

| crate | 内容 |
|---|---|
| `phys-body` | `Body`/`BodyCinfo`（shape、flags、material/quality、filter info、user data、位置/朝向/速度/质量）；`BodyManager` 索引池（24 位索引 + 8 位 serial，free-list 复用槽，复用后旧 `BodyId` 失效）；`Motion`（orientation、质心、线/角速度、mass/inverse inertia、积分与冲量）；`MotionManager`（slot 0 固定静态 motion `PRESET_STATIC`，动态槽复用） |
| `phys-world` | `WorldCinfo` builder（gravity、body/motion 容量、collisionTolerance、solverTau/Damp/Iterations、deactivation、expectedDeltaTime）；`World` 组装 body/motion manager，`create_body`/`destroy_body`、transform/position/velocity/AABB 访问器、`set_gravity`、`stats`；`step()` 单线程运动学骨架：动态 motion 做平移 + 旋转积分并同步 body transform/AABB，静态/Keyframed 保持不动（Phase 8 已扩展为 gravity→collide→solve→integrate 完整管线） |

决策与偏差：
- `Body::transform` 暂用 f32 `hkTransform`；SDK 的 `hkPreciseTransform`（double 平移）留到 Phase 10 大世界/shift 工作
- Motion 角速度按**世界空间**暴露（SDK 内部存局部/惯量空间并在访问时转换）；积分采用 `q' = normalize(Δq * q)`，对世界空间输入等价
- `create_body` 把 SDK 的 `allocateBody` + `addBody` 折叠为一步；质量 `<0`（使用 shape 质量）暂按无限质量处理，形状质量属性随 Phase 8 求解器一起接入
- `destroy_body` 会释放非静态 body 的 motion；暂不支持 `allocateAttachedBody` 的 motion 共享；`MotionId` 本身无 serial（与 SDK 一致），跨销毁持有需先用 `BodyId` 校验
- `step()` 在 Phase 6 时仅做运动学积分；Phase 8 已接入重力、宽窄相碰撞与接触求解（见 Phase 8 段）
- 未纳入：broadphase/space splitter、body 添加/延迟队列、command queue 与确定性检查、deactivation manager、collision filter、事件/modifier、用户 body buffer、`hknpWorldReader`/`hknpWorldWriter` traits
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 7：`phys-collide`（✅ 已完成，2026-11-10）
交付（22 个单元测试）：

| 模块 | 内容 |
|---|---|
| `broadphase.rs` | 均匀网格宽相（`cell_size` 默认 2.0，`BTreeMap` 保证确定性）；AABB 插入多 cell、单元格内候选对、按 `(a,b)` 排序去重；`BodyPair` 规范化；超大 AABB（>4096 cell）退化为线性扫描 |
| `gjk.rs` | 世界空间 support 映射（`R^T * dir` → 局部 support → 世界点）；GJK 最近点/相交，1–4 点 simplex 归约（点/线段/三角形/四面体重心 + 区域测试），输出 A/B witness 点与 B→A 法线 |
| `epa.rs` | 从 GJK simplex 与轴向/中心方向 support 采样构造**严格包含原点**的四面体，再做 EPA 面扩展求穿透深度、法线、接触点；退化或迭代失败时按 support 方向采样最小深度回退 |
| `manifold.rs` | `Manifold`/`ContactPoint`：法线 B→A、位置在 B、负距离=穿透，`point_a = position + normal*distance`，与 `hkcdManifold4` 约定一致 |
| `narrowphase.rs` | sphere-sphere 解析特化；一般凸体走 GJK/EPA 生成单点接触；`collide_with_tolerance` 支持推测接触（正距离容差）；非凸（Mesh/HeightField/Compound）返回 `None` |

决策与偏差：
- **单点流形**：SDK 的 4 点 face clipping、contact cache、welding 留到求解器/manifold 阶段；当前每对凸体最多 1 个接触点
- GJK 对光滑形状常在原点落在线段时提前判定相交，simplex 维度小于 4；EPA 因此增加“外采样构造四面体”步骤，并要求重心权重 > 1e-4，避免原点落在四面体边界导致深度 0
- 宽相为均匀网格简化实现，不是 SDK 的 SIMD tree/CsGrid 分层宽相；未实现 broadphase layer、`hknpCollisionFilter`、`DONT_COLLIDE` 过滤（由调用方/后续阶段处理）
- 未纳入：射线/AABB/形状投射查询、连续碰撞、SDF/粒子碰撞、复合形状窄相、接触缓存与 welding；这些留到 Phase 8+ 与查询阶段
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 8：`phys-solver` + 完整步进（✅ 已完成，2026-11-10）
交付（`phys-solver` 5 个单元测试；`phys-world` 13 个单元测试 + 1 个文档测试）：

| crate | 内容 |
|---|---|
| `phys-solver` | `SolverBody`（inverse mass/inertia、质心、线/角速度、点冲量）；`ContactConstraint`（body 索引、B→A 法线、世界接触点、穿透、摩擦）；`SolverSettings`（iterations/baumgarte/max_recovery_speed/friction）；`Solver` 顺序脉冲 Gauss-Seidel：法向累积冲量非负 clamp、Baumgarte 穿透恢复速度上限、单切向摩擦按 `μ * 累积法向冲量` clamp；`SolverStats` |
| `phys-world` | `World::step` 完整管线：重力 → `collect_contacts`（网格宽相 + GJK/EPA 窄相）→ 构建 SolverBody/接触约束 → 求解 → 写回速度 → 半隐式欧拉积分并刷新 transform/AABB；`collect_contacts` 公开 API；`WorldStats` 增加 `last_manifold_count`/`last_contact_count`；静态-静态与 `DONT_COLLIDE` 过滤 |

决策与偏差：
- 接触仍是**单点**（Phase 7 限制）；材质库未接入，摩擦系数固定 0.5、restitution 0，`WorldCinfo.solverTau/solverDamp` 暂时只保留字段，稳定化使用固定 Baumgarte 0.2
- 未移植 `Solver/ConstraintAtom` 的约束原子/Jacobian 系统、warm starting、island 分区、deactivation；当前是“每对单点约束 + 4 次顺序脉冲”的简化求解器
- 质量 `< 0` 仍按无限质量处理（Phase 6 决策），动态 body 需要显式 `with_mass`；shape 质量属性/惯量张量尚未接入，因此球的 inverse inertia 默认为 0（只做平动响应）
- `step` 的宽相使用上一步缓存的 AABB，积分后刷新；确定性依赖 body slot 顺序、宽相 `(a,b)` 顺序与求解器插入顺序
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 9：测试与验证（✅ Rust 侧完成；C++ 黄金基准外部阻塞，2026-11-11）
交付：

- **测试矩阵**：[docs/phase9-test-matrix.md](docs/phase9-test-matrix.md)——41 个原版 UnitTest 的逐项映射（6 个 Ported、3 个 Partial、32 个 Blocked 并注明前置系统），以及 S1–S12 场景矩阵、基准矩阵与验收标准
- **新增集成测试 19 个**（`tests/`）：
  - `simulation_scenarios.rs`（10）：自由落体解析解、球/盒堆叠/胶囊/圆柱落地、摩擦滑动、流形约定、宽相确定性、DONT_COLLIDE、transform/orientation
  - `original_unit_tests.rs`（6）：`NpBodyAllocationTest`、`NpBoxShapeUnitTest`、`NpBoundingSphereTest`、`NpMeshShapeUnitTest`（结构）、`NpHeightFieldShapeUnitTest`（结构）、`NpManyBodiesTest`
  - `determinism.rs`（3）：同场景两次运行逐位一致（`f32::to_bits`）、`hvk-trace v1` 文本稳定、`NpWorldSnapshotTest` 部分映射
- **确定性快照**：`phys-world` 新增 `World::snapshot()` / `WorldSnapshot` / `BodySnapshot` 与 `to_text()`，浮点用 `to_bits()` 输出，格式 `hvk-trace v1`
- **基准 harness**：`benches/simulation.rs`（`harness = false`，仅 `std::time`，无 criterion）；覆盖宽相、GJK 球/盒、窄相盒对、求解器、`world_fall_n`、`world_stack_10`
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；`cargo bench --quick` 输出全部指标

黄金基准状态：`../original` 是源码分发，没有生成好的 `.sln`/Physics 测试可执行文件，完整构建依赖原版 MSBuild 工具链，当前环境无法运行 C++ 场景。因此**不宣称 C++ 逐位对齐**；已实现 Rust 侧逐位确定性与 trace 格式，待原版 SDK 可构建后按矩阵 §5 执行对比。criterion 因 crates.io 不可达改用自研 harness。

### Phase 10：多线程 + 序列化 + 扩展（🚧 部分完成，2026-11-11）
已交付子集：

- **多线程窄相**（`phys-world`）：新增 `World::collect_contacts_parallel(threads)` 与 `World::step_with_threads(dt, threads)`；使用 `base_thread::parallel_for_chunks` 做确定性连续分块，chunk 结果按宽相 pair 索引回排，因此接触顺序与单线程完全一致。求解器与积分仍为串行；候选数 <32 时回退单线程（避免每步创建线程的开销）。新增测试：并行接触结果 == 单线程、双 world 120 步逐位一致（`phys-world` 15 个单元测试）。
- **`base-serialize` 容器层**：HFF 结构化流解析（branch/leaf、30 位 size、FourCC、错误）与 Tagfile 容器（`TAG0`/`TCM0` 根、`SDKV` 版本、`DATA`/`TYPE`/`INDX` 访问），10 个单元测试。仅容器解析，不做反射对象重建。
- **`phys-extensions` WorldHash**：对 `WorldSnapshot` 做 FNV-1a 64 位确定性哈希（浮点按 `to_bits`），5 个单元测试。
- 基准新增 `world_fall_n_parallel` 场景（同一 step 管线，4 线程窄相）。
- **约束/joint 系统**（2026-11-12，Phase 10 后续子集）：
  - `phys-types`：新增 `ConstraintIndex`（u32，invalid `HK_INT32_MAX`）与 `ConstraintId`（u64，MSB 区分内部索引/用户 ID，语义照抄 `hknpConstraintId.h`）。
  - `phys-solver`：新增 `constraint.rs`——世界空间 solver 级 joint（`BallAndSocketJoint` / `HingeJoint` / `FixedJoint`），顺序脉冲求解：线性 3 轴拉拢 pivot（分离量 Baumgarte bias）、铰链轴对齐（垂直轴角速度 + 轴误差 bias）、固定关节锁相对角速度（世界 X/Y/Z 轴 + 朝向误差 bias）。joint 在 slice 顺序、固定轴序下求解，确定性。新增 14 个单元测试。
  - `phys-world`：新增 `constraint.rs`——`BallAndSocketCinfo` / `HingeCinfo` / `FixedCinfo`（pivot/axis 存 body 局部空间；`FixedCinfo::between_bodies` 按当前位姿绑定）+ `WorldJoint` 槽表（**槽不复用**，ID 永不重指）与 `create_*_constraint` / `destroy_constraint` / `set_constraint_enabled` API；`step` 在接触求解后按槽序构建 solver joints 并求解；`destroy_body` 级联删除关联约束。新增 6 个约束单元测试。
  - 局限：速度级求解（无位置修正投影）、无 limits/motors/springs、无约束岛与 warm starting；球的 inverse inertia 默认 0，摆动类场景需手动设置 `motion.inverse_inertia`。
- **查询系统**（2026-11-12，Phase 10 后续子集）：
  - `phys-collide`：新增 `query.rs`——凸体 raycast（**conservative advancement**：GJK 点-形状距离除以射线与法线夹角推进，sphere/box/capsule/cylinder/hull 单一实现路径；射线起点在内部按 fraction 0 命中）、`point_distance`（点-形状 GJK 距离，供最近点查询）、`ray_aabb` slab 粗测（固定 x/y/z 轴序）。新增 10 个单元测试。
  - `phys-world`：新增 `query.rs`——`World::raycast`（最近命中，槽序遍历 + AABB 粗测 + 同 fraction 低 `BodyId` 优先，确定性）与 `World::closest_point`（凸体最近表面点/距离，内部点返回 distance 0）、`World::body_transform`。查询不受碰撞过滤影响（与 SDK 一致）；非凸形状（Mesh/Compound/HeightField）暂不参与。
  - 新增集成测试 `tests/constraints_and_queries.rs`（9 个）。
- **铰链 limits/motors**（2026-11-12，Phase 10 后续子集）：
  - `phys-solver`：`HingeJoint` 新增 `limits: Option<HingeLimits>` 与 `motor: Option<HingeMotor>`——limits 为单边约束（仅当角度越界时施加回复脉冲，方向由 lambda 符号约束，Baumgarte bias 上限同接触求解），motor 为绕铰链轴的速度电机（**每次 solve 施加一次限幅冲量**，与迭代次数无关，确定性）。角度/速度约定为 **B 相对 A 绕铰链轴**（与角度测量 `sin = t_a × t_b · axis` 一致）。新增 8 个单元测试。
  - `phys-world`：`HingeCinfo` 新增 `limits`/`motor` 字段与 `HingeLimitsCinfo`（twist 参考轴对 + min/max 角）/`HingeMotorCinfo`（目标速度 + 冲量上限）；每步由世界空间 twist 轴计算签名角 `hinge_angle`（投影到垂直于铰链轴的平面，atan2 取值 `(-pi, pi]`）传入 solver；新增 `set_hinge_limits`/`set_hinge_motor` 运行时 API（非铰链/无效 ID 返回 false）。新增 3 个单元测试；`constraints_and_queries.rs` 集成测试增至 12 个。
  - 局限：速度级 limit（越界小幅穿越后由 bias 拉回，无硬位置钳制）、仅速度电机（无位置/弹性电机、无 springs）、motors 不参与迭代内与 limits 联合收敛（预施加一次）。

未完成/阻塞项：

- **完整 `hknpMultithreadedSimulation`**：任务图/求解器/积分并行、work stealing、`hknpTaskGraph` 物理调度未移植；当前只并行窄相，且每步重建 scoped worker（小场景并行反而更慢，见 bench）。
- **`hkx` 对象加载**：TYPE/DATA 反射类型注册、对象 rebuild、patch/fixup、binary packfile、writer/XML/2014 兼容均未移植；`base-serialize` 不能替代资产加载。
- **Ragdoll / Vehicle / CharacterControl**：前置的 `约束/joint` 与 `查询` 系统已在 Phase 11 完成（见下）；扩展层本身（ragdoll 驱动/身体绑定、wheel 悬挂约束与 vehicle 调度、character action 调度）仍未移植。
- 大世界 `hkPreciseTransform`（double 平移）仍待（当前 f32 Transform）。
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；`cargo bench --quick` 输出全部指标。

### Phase 11：约束/joint 系统 + 查询系统（✅ 已完成，2026-11-16）
交付（`phys-solver` 34 个单元测试；`phys-collide` 47 个单元测试；`phys-world` 48 个单元测试 + 1 个文档测试；集成测试 `constraints_and_queries.rs` 22 个）：

| crate | 内容 |
|---|---|
| `phys-solver` | `Solver::solve_joints`：世界空间关节行按 slice 顺序做顺序脉冲求解。`BallAndSocketJoint`（3 线性行）、`HingeJoint`（3 线性 + 2 轴对齐 + 可选角度 limits + 速度 motor）、`FixedJoint`（3 线性 + 3 角度锁定 + 四元数误差偏置）、`RagdollJoint`（3 线性 + cone 摆动限制 + twist 角度限制）、`PrismaticJoint`（2 垂直线性行 + 3 角度锁定 + 沿轴行程限制）、`SpringJoint`（距离行）、`PointToPlaneJoint`（沿平面法线的距离行，零宽区间为双边）。limits 单边符号检查，motor 单次限幅冲量 |
| `phys-world` | 约束管理器：`ConstraintId` 槽表（不复用）、`WorldJoint`/`JointType`、`create_*_constraint`、`destroy_constraint`、`set_constraint_enabled`、hinge/ragdoll/prismatic 参数 setter、`set_constraint_collisions_enabled`（`DISABLE_COLLISIONS`：成对 body 不生成接触）、`constraints_of_body`（body↔constraint 映射）。每步从 body pose 重建世界空间行（pivot/axis、hinge/ragdoll 角度、prismatic 行程、fixed 相对朝向误差） |
| `phys-collide` | 查询：`raycast`（凸体 conservative advancement；mesh BVH、height field 单元、compound 递归；三角形 Möller–Trumbore）、`point_distance`、`ray_aabb`、`shape_cast`（凸-凸 conservative advancement + 复合目标叶子遍历；`ShapeCastHit` 带 fraction/position/normal/witness/start_penetrating） |
| `phys-world` 查询 | `QueryFilter`（ignore body + static/dynamic/keyframed 开关）、`raycast`/`raycast_filtered`、`cast_shape`、`closest_point`/`closest_point_filtered`、`closest_points_to_shape`（重叠时用 EPA 深度返回负距离）、`query_aabb`；结果按 `(distance, body id)` 决定，遍历顺序不影响确定性 |

决策与偏差：

- **不做 1:1 constraint-atom 直译**：SDK 的 `hknpConstraintAtom`（49 文件）、constraint group、connectivity graph 未移植；本阶段以“世界空间关节行 + 顺序脉冲”覆盖 Ragdoll/Vehicle/CharacterControl 所需的主要约束族，pivot/axis、limits、motor、碰撞屏蔽语义与 SDK 对齐
- **Ragdoll**：实现 cone（swing）与 twist 限制，SDK 的 plane-angle limits 未移植（cone 是 ragdoll 主限制）；twist 参考轴由 `plane_axis_a/b` 投影得到
- **Prismatic**：锁住两个平移方向与全部转动（相对朝向误差用四元数误差向量），沿轴行程用单边限制；`±inf` 表示无行程限制
- **Wheel/Vehicle**：wheel 约束（悬挂 + 转向/驱动）未移植，依赖后续 vehicle 阶段；查询系统已提供 vehicle 悬挂 raycast 所需 API
- **Shape cast**：仅凸体可作为 query shape（与 `hknpShapeCastQuery` 常见用法一致）；旋转 cast、`hknpDeflectedLinearCast` 与专用连续算法未移植；容差默认 `0.001`（`HKNP_DEFAULT_SHAPE_CAST_ACCURACY`）
- **查询**：`closest_points_to_shape` 仅凸体；mesh/height field/compound 的最近点查询未移植；shape key/material/filter codec 的 per-leaf 解码未移植
- **接触/约束**：约束求解在接触求解之后执行（同一速度缓冲），未做 island/constraint-batch 分区与 warm starting（Phase 12 已补上）
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；bench 新增 `raycast_world` / `shape_cast_world` / `joint_chain_ragdoll`

### Phase 12：约束岛 + warm starting（✅ 已完成，2026-11-16）
交付（`phys-solver` 40 个单元测试；`phys-world` 62 个单元测试 + 1 个文档测试；集成测试 `islands_and_warm_starting.rs` 5 个）：

| 模块 | 内容 |
|---|---|
| `phys-solver` | `ContactImpulse`（normal + 双切向累积冲量）、`contact_tangents`（仅由法线决定的确定性正交切向基）、`Solver::solve_with_impulses`（先施加上一步冲量再迭代，回写累积值）、`Solver::solve_contacts_subset` / `solve_joints_subset`（按索引子集求解）、`SolverSettings::warm_starting` 开关；摩擦改为双切向盒式 clamp |
| `phys-world` | `cache.rs`：接触冲量缓存——按 `(BodyId, BodyId)` + B 局部空间接触点（阈值 0.05 m、法线点积 ≥ 0.9）跨步匹配最近行，每步用实际求解的接触重建以自动剪枝；`island.rs`：union-find 约束岛（仅动态体连通，静态/关键帧体不合并岛，共享地面会列在多个岛中）；`World::step` 改为 collide → gravity → 建约束/匹配冲量 → 划岛 → 逐岛 solve(contacts→joints) → 回写速度 → 积分；`World::islands`/`last_island_count`/`clear_contact_cache`、`WorldStats::last_island_count`、`WorldCinfo::enable_warm_starting` / `with_warm_starting` |

决策与偏差：

- **岛不改变数值结果**：岛之间不共享动态体，逐岛求解与全局顺序脉冲在数学上等价且逐位一致（仅用于局部化求解状态，为后续休眠/并行铺路）；关节仍按 `constraints` 槽序 + 固定行序求解
- **warm starting 匹配靠局部特征**：单点流形下用 B 局部接触点 + 法线匹配，传送/移除 body 时距离或法线检查失败即自动失配；`clear_contact_cache` 供大幅改质量/摩擦后显式清理
- **摩擦基固定**：切向基由法线生成（`+X`，法线接近 `X` 时用 `+Y`），保证跨步可复用；盒式 clamp 近似锥形摩擦（`|t1|,|t2| ≤ μ·λn`）
- **验证**：5 层堆叠 480 步后 warm starting 残余穿透约为冷启动的一半（0.0147 m vs 0.0292 m），两次同场景运行 `hvk-trace v1` 逐位一致
- **未纳入**：岛级休眠/唤醒（deactivation manager）、按岛并行求解、接触多点流形的逐点匹配、warm starting 的冲量缩放系数、关节 warm starting
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；bench 指标与 Phase 11 同量级

### Phase 12（后续子集）：位置/弹性电机 + 关节 warm starting（✅ 已完成，2026-11-16）
交付（`phys-solver` 51 个单元测试；`phys-world` 70 个单元测试；新增集成测试 `motors_and_joint_warm_starting.rs` 5 个）：

| 模块 | 内容 |
|---|---|
| `phys-solver` | `HingeMotor` 扩展为三模式枚举：`Velocity` / `Position`（`hknpPositionConstraintMotor` 子集，期望角速度 = 角度误差/τ，τ ≤ 1e-4 视为禁用）/ `Spring`（`hknpSpringConstraintMotor` 子集，冲量 = (k·角误差 − c·相对角速度)·Δt，力积分到步长）；`PrismaticJoint` 新增 `motor: Option<PrismaticMotor>`（`Position` / `Spring` 沿滑轴，SDK 线性电机子集）；新增 `JointImpulse`（`MAX_JOINT_ROWS = 8` 固定行序累积冲量，行序逐关节文档化）与 `Solver::solve_joints_with_impulses` / `solve_joints_subset_with_impulses`——扫掠前回放缓存冲量（双边行无条件、单边行仅越界时），并回写新累积值；单边行（limits/cone/twist）改为接触式**回复半空间 clamp**（上限侧 `accum ≤ 0`、下限侧 `accum ≥ 0`），回放过冲会被拉回而不注入能量；`HingeJoint.angle` 提升为顶层字段（无 limits 也可供电机使用），`HingeLimits` 仅保留角度界；`wrap_angle` 处理跨 ±π 目标 |
| `phys-world` | `HingeMotorCinfo` 扩展为 `Velocity`/`Position`/`Spring` 枚举（新增 `HingeVelocityMotorCinfo`/`HingePositionMotorCinfo`/`HingeSpringMotorCinfo`）；`HingeCinfo` 提升参考（twist）轴 `reference_axis_a/b` + `with_reference_axes`（默认退化回退在**局部空间**计算铰链轴的确定性垂直轴，随刚体旋转），`HingeLimitsCinfo::new(min, max)`；`PrismaticCinfo` 新增 `motor` + `PrismaticMotorCinfo`（Position/Spring cinfo）与 `set_prismatic_motor` 运行时 API；`JointSlot` 新增 `impulses: JointImpulse`（跨步持久化，禁用约束即清零，销毁随槽释放）；`step` 用 `solve_joints_subset_with_impulses` 按岛求解并回写槽内冲量 |

决策与偏差：

- **铰链参考轴语义变更**：twist 轴从 `HingeLimitsCinfo` 移至 `HingeCinfo`（角度测量不再依赖 limits 是否配置），旧 `HingeLimitsCinfo::new(twist_a, twist_b, min, max)` 调用点已更新
- **单边行 warm starting 采用接触式 clamp**：朴素回放（先回放再按 `λ=(target−rel)/m` 求解、符号拒绝）会在速度已修正后再次全额施加上步冲量，造成能量注入（prismatic limit 实测穿透发散）；改为 `next = clamp(accum + λ, 回复半空间边界)` 后过冲被自动拉回
- **精确行的回放被完全吸收**：双边行 `λ` 与累积值无关（线性精确解），因此 warm start 在当前行实现下对速度结果中性（与接触行 `max(0,·)` 截断的路径依赖不同）；缓存结构为后续 clamped/松弛行铺路，并有测试固化该不变量
- **验证**：铰链位置电机 120 步收敛到目标角（±0.25）、弹簧电机阻尼振荡收敛、prismatic 位置/弹簧电机驱动滑块到位、电机与关节 warm starting 场景 `hvk-trace v1` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc` / release build 全绿

### Phase 12（后续子集）：岛级休眠/唤醒 + 按岛并行求解（✅ 已完成，2026-11-16）
交付（`phys-world` 75 个单元测试；新增集成测试 `sleeping_and_parallel_solving.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-world` | **岛级休眠**（deactivation 子集，`WorldCinfo::enable_sleeping` 默认关闭）：每个动态体维护低能量计时（线/角速度低于 `sleeping_linear_threshold`/`sleeping_angular_threshold` 累积，超阈值清零），整岛全部动态体计时达到 `time_to_sleep` 且无 keyframed 成员时入睡——速度清零、跳过窄相（休眠-休眠与休眠-静态对不再生成候选）、不施加重力、不积分（位姿冻结），岛内关节 warm-start 冲量清零；**唤醒传播**：岛内含醒着的动态体或 keyframed 成员即唤醒全部休眠成员（keyframed 接触/关节防止入睡），`wake_body` 沿关节图 BFS 传播，`set_body_transform/position/linear_velocity/angular_velocity` 自动唤醒；`World::body_is_sleeping`、`WorldStats::sleeping_body_count`；**按岛并行求解**：`step_with_threads(threads)` 在 `threads > 1` 且岛数 ≥ 2 时按确定性连续分块并行——每个 worker 克隆求解器缓冲、顺序求解自己的岛、按岛回并（岛在动态 solver 体上不相交，静态成员不变），结果与单线程**逐位一致** |

决策与偏差：

- **不移植 `hknpDeactivationManager`**：SDK 用位置/旋转量化与簇（cluster）比较判休眠，本阶段用 Box2D 风格的速度阈值 + 计时（简化实现，语义子集）；`enable_deactivation` 字段保留不动，实际开关为新增的 `enable_sleeping`（默认关闭，保证既有确定性基线与测试不变）
- **休眠体跳过窄相的代价**：休眠岛与外界的接触丢失，唤醒依赖"有活动伙伴的岛"重建接触；堆叠岛逐层唤醒（每步一层），不重建休眠期间的接触缓存
- **自由体（无接触/关节）不参与岛**，因此不会休眠（重力场景下也无法静止，语义一致）
- **并行求解不改变数值**：岛在动态 solver 体上不相交、静态体不被修改，逐岛求解与全局顺序在数学上等价；合并顺序无关；已有测试与新增集成测试固化 `threads=1` 与 `threads>1` 的 `hvk-trace v1` 逐位一致
- 顺手修复 `phys-solver/constraint.rs` 中 4 处失效的 intra-doc 链接（`doc -D warnings` 由红转绿）
- **验证**：静置球 30 步内入睡（位姿冻结、接触为空）、落球接触唤醒、`wake_body` 关节链传播、速度/transform 设置唤醒、keyframed 关节岛不入睡；双堆叠 + 飞行球场景 180 步 `threads=4` 与串行逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build / `cargo bench --quick` 全绿

### Phase 12（后续子集）：wheel 约束（✅ 已完成，2026-11-17）
交付（`phys-solver` 56 个单元测试；`phys-world` 78 个单元测试；新增集成测试 `wheel_constraints.rs` 6 个）：

| 模块 | 内容 |
|---|---|
| `phys-solver` | `WheelJoint`（`hknpWheelConstraintData` 子集）：悬挂轴垂直平面双线性锁定行、沿悬挂轴行程单边限位行、绕 axle 两轴对齐行（共 5 行）；`WheelSuspension` 弹簧-阻尼（`-(k·(length−rest) + c·rel_v)·Δt`，限幅冲量、不缓存）与 `WheelDrive` 速度电机（绕 axle 单次限幅冲量）均在扫掠前施加一次 |
| `phys-world` | `WheelCinfo`（body A = 车轮、body B = 底盘；pivot/悬挂轴/转向轴存底盘局部空间、axle 存车轮局部空间，`axle_b` 默认取 `axle_a` 可用 `with_axle_b` 覆盖）+ `WheelSuspensionCinfo`/`WheelDriveCinfo`；`create_wheel_constraint`、`set_steering_angle`（底盘侧 axle 绕转向轴旋转，对齐行据此拉正车轮）、`set_wheel_suspension`/`set_wheel_drive` 运行时 API；行程限位 `with_travel`，默认 `±inf` |

决策与偏差：

- **移植约束路线而非 vehicle 扩展层**：SDK 有两条独立路线——`hknpWheelConstraintData`（约束 atom 组，无 motor）与 `Extensions/Vehicle`（raycast 悬挂 + 手工冲量 + 摩擦求解器）。本阶段移植前者为世界空间关节行子集；vehicle 扩展层（engine/transmission/friction solver/raycast wheel collide）未移植
- **无 motor atom → 电机子集**：原约束的驱动自由度只是 2dAng 的自由旋转；本阶段按既有 hinge/prismatic 电机模式补了 `WheelDrive` 速度电机（SDK 靠 vehicle friction solver 施加驱动，不走约束）
- **转向通过对齐行实现**：`steering_angle` 只旋转底盘侧命令 axle（`setSteeringAngle` 语义），车轮 axle 由两行角对齐拉正（Baumgarte bias），不做独立 steering 约束
- 每步由 body pose 重建世界空间行；`suspension_length` 为 `(wheel_center − hardpoint)·axis` 有符号距离；悬挂弹簧与驱动电机与电机语义一致（每 solve 一次、不参与 warm starting 缓存）
- 未纳入：wheel 的软约束 tau/damping 因子（`m_lin0Soft` 的 solver 因子形式）、`WheelComponentParams` 的轮胎摩擦参数、`hknpVehicleInstance` 整车调度（引擎/传动/空气动力学/相机等）
- **验证**：悬挂支撑底盘落地收敛（180 步后静止于行程内）、驱动推动小车沿 axle 垂直方向前进、转向改变行进方向、destroy chassis 级联删约束、120 步场景 `hvk-trace v1` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build / `cargo bench --quick` 全绿

### Phase 13：vehicle 扩展层（✅ 已完成，2026-11-17）
交付（`phys-extensions` 17 个单元测试 + 1 个文档测试；新增集成测试 `vehicle_extension.rs` 8 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/vehicle` | `VehicleInstance::step`（`hknpVehicleInstance::applyAction` 子集，调用方显式驱动）：`DriverInput`（throttle/brake/steering/handbrake/reverse）→ raycast 轮碰撞（`World::raycast_filtered` 忽略底盘；`suspension_length = hit_distance − wheel_radius`，closing speed 按 SDK 约定压缩为负）→ 默认 transmission（`m_noSlipIdealSpinVelocity` 加权轮速 → RPM、upshift/downshift 阈值即时换挡、倒挡负齿比）→ 默认 engine（`calcEngine` 抛物线扭矩曲线 + clutch slip + resistance 逐式直译）→ 悬挂弹簧-阻尼力（`calcSuspension` 逐式直译，参数按单位底盘质量、力非负）→ 摩擦求解器（bullet 风格：驱动/制动力作为接触点前向冲量、刹车不过冲越滚动条件、侧向冲量消除侧滑、摩擦圆钳制 `μ·susp_force·dt`）→ 冲量写回底盘与地面速度（含运动空间对角惯量的角响应，经 `set_body_*` 自动唤醒） |

决策与偏差：

- **显式 step 代替 Action**：SDK 用 `hknpUnaryAction` 挂进 world 的 action manager；本阶段暴露 `vehicle.step(&mut world, dt, &input)` 由调用方在 `world.step` 之间调用（世界仍负责重力与接触）
- **raycast 悬挂子集**：仅 `hknpVehicleRayCastWheelCollide`；地面摩擦取轮胎设置（`WheelSetup::tyre_friction`）而非地面材质（材质库未接入）；法线裁剪角/`suspensionScalingFactor` 未移植
- **简化 friction solver**：不做 per-axle 2×2 jacobian 系统、无轮自旋状态与摩擦表曲线；采用 bullet 风格 raycast vehicle 模型（驱动力直推前向冲量 + 侧向锁滑移 + 摩擦圆钳制）
- **换挡无离合延迟**：阈值越线即时换挡（SDK 的 `m_delayed` 计时器未移植）；engine RPM 用 `|avg_spin·ratio|` 保证倒挡非负
- **冲量含角响应**：悬挂/摩擦冲量经运动空间对角惯量产生角速度变化；`inverse_inertia == 0` 的底盘不会俯仰/偏航（与关节一致的限制），集成测试显式设置底盘惯量
- **几何注意事项**：接触式悬挂下，若 COM 高度/轴距使摩擦受限的驱动或制动力（≈1g）载荷转移超过静轴载，空载轴会离地弹跳——场景设计需保证 `μ·m·g·h/L < 静轴载`（集成测试用低 COM + 长轴距固定该前提）
- 未纳入：`LinearCastWheelCollide`、Aerodynamics/VelocityDamper/Tyremarks/Camera、DriverInput 过滤管线、`m_lin0Soft` 软约束因子、per-leaf shape key/material codec
- **验证**：悬挂支撑沉降到位（y≈0.82，4 轮接触 + 非零弹簧力、怠速 RPM 来自 clutch slip）、油门沿 `suspension×axle` 方向加速、倒挡反向、转向改变航向、全刹/手刹（后轴载荷限制减速度）停车、转向角按 fraction 分配到轮、同场景两次运行 `WorldHash` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build / `cargo bench --quick` 全绿；顺手补了门面 `base-serialize` 重导出消除 unused dependency 警告

### Phase 14：碰撞过滤系统（✅ 已完成，2026-11-17）
交付（`phys-collide` 新增 8 个单元测试，共 55 个；`phys-world` 78 个单元测试；新增集成测试 `collision_filters.rs` 8 个）：

| 模块 | 内容 |
|---|---|
| `phys-collide/filter.rs` | `CollisionFilter` trait（`hknpCollisionFilter` 子集：info 对 / body 对 / query 三个决策入口，默认全放行，要求 `Send + Sync`）；`GroupCollisionFilter`（`hknpGroupCollisionFilter` 默认 `Config<5,5,5,16>` 特化：`calc_filter_info` 打包 5 位 layer + 5 位 subsystem id + 5 位 subsystem don't-collide-with + 16 位 system group，32 项 layer LUT 初始全开，对称 `enable/disable_collisions_between`，layer 0 永不禁用）；`PairCollisionFilter`（`hknpPairCollisionFilter` 子集：引用计数 body 对禁用表 + 可选子过滤器链，`clear_all`/`is_disabled`） |
| `phys-world` | `World::set_collision_filter` / `collision_filter`（`hknpWorld::setCollisionFilter`）；`contact_candidates` 在 `DONT_COLLIDE` 之后调用 `is_body_pair_enabled`（body 对重载）；`QueryFilter` 新增 `filter_info` 字段与 `with_filter_info`，全部查询（raycast/cast_shape/closest_point/closest_points_to_shape/query_aabb）经 `is_query_enabled` 过滤（对应 `isCollisionEnabled(queryFilterData, body)` 重载）；`World` 改手写 `Debug`（trait object 字段） |

决策与偏差：

- **`_isCollisionEnabled` 按原版 `.inl` 逐字直译**：同一非零 system group 的 body 对绕过 layer 表，由 subsystem 决定（`idA == dontCollideB` 或 `idB == dontCollideA` 即禁用）——注意 subsystem 全 0 的同组 body 对因此**不碰撞**（SDK 原版行为）；其余情况走 `LUT[layerA] & (1<<layerB)`（表保持对称，单次查询即可）
- 未纳入：`hknpDisableCollisionFilter`（被 `PairCollisionFilter` 覆盖）、`hknpConstraintCollisionFilter`（世界侧 `DISABLE_COLLISIONS` 关节屏蔽已覆盖同一语义）、宽相 `filterBodyPairs` 流压缩（候选对在网格宽相之后统一过滤）、`getLodInfo`、过滤器序列化、`hknpFilterTranslator`
- `CollisionQueryType` 参数未移植（内建过滤器从不分支查询类型）；`hknpQueryFilterData` 子集为 `QueryFilter.filter_info`（material id/user data 未进入过滤决策）
- **验证**：layer 禁用/恢复、同组绕过 LUT、subsystem 排斥、pair 禁用/启用、查询 filter info 过滤、过滤场景两次运行 `hvk-trace v1` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 15：CharacterControl（✅ 已完成，2026-11-18）
交付（`phys-extensions` 新增 `character` 模块，新增 29 个单元测试（surface/simplex/proxy/state machine，模块总计 17 → 46 个），新增集成测试 `character_control.rs` 5 个；`phys-extensions` 新增 `phys-collide` 依赖）：

| 模块 | 内容 |
|---|---|
| `character/surface.rs` | `hknpCharacterSurfaceInfo` 子集：`SupportedState`（Unsupported/Sliding/Supported）+ 支撑面平均 normal/速度/角速度（`m_surfaceDistanceExcess` 未移植，proxy 恒为 0） |
| `character/simplex_solver.rs` | `hkSimplexSolver` 逐式直译：活动平面集 1d/2d/3d 求解（退化/近平行回退顺序求解）、priority+速度长度冒泡排序、静态摩擦（圆/盒两模型 + extra up/down）、动态摩擦、`max_surface_velocity` 钳制回退、罚距增长防重入、逐约束 `SurfaceInteraction` |
| `character/proxy.rs` | `hknpCharacterProxy` 子集：Cinfo 全默认值（keepDistance 0.05、maxSlope π/2、maxCastIterations 10 等）+ builder；自维护接触流形（validate/findContact(1.1)/最近新接触合并(0.1)/相似剔除、compareContacts 几何项）；`check_support` 静态几何查询（约束速度清零 + 状态分类）；`integrate` 线性 cast → updateManifold → 约束（keepDistance 平面 + maxSlope 阻挡平面 + 贯穿恢复 + HVK-1477 速度回移）→ simplex 求解 → re-cast 验证 + keepDistance 停距 → resolveContacts 线性冲量推挤（含 character_mass/character_strength 钳制） |
| `character/state_machine.rs` | `hknpCharacterMovementUtil`（表面坐标系 + 增益限幅 + 最大速度增量钳制）逐式直译；`CharacterState` trait（update/get_next_state/enter/leave）；默认状态 `OnGround`（增益 0.95/走速 10/加速度 200/ground hugging/水平投影）、`Jumping`（v²=2as 单帧冲量，num==1 触发）、`InAir`（增益 0.05/空速 10/加速度 50 + 重力）逐式直译；`CharacterStateManager` 注册表；`CharacterContext`（update/predict、强制下一状态、`newState == previousState` 跳一帧语义、输出速度滤波 active `#else` 分支、默认 gain 1.0/maxSpeed 20/maxAccel 625） |

决策与偏差：

- **查询子集**：原版 `hknpAllHitsCollector`（起点接触 + cast）改为——起点接触由逐 body 最近点查询重建（`keepDistance + keepContactTolerance` 内全部凸体 body），cast 用 `World::cast_shape` 单最近命中；网格/HeightField/复合体不进入流形（GJK 仅凸体）
- **resolveContacts 子集**：仅线性冲量（跳过角惯性项），经 `set_body_linear_velocity` + `wake_body` 写回；phantom presence/trigger/listener/LOD/multi-thread task graph 未移植，proxy 为独立对象按调用传 `&World`
- StateMachine：`Climbing`/`Flying` 类型保留但默认管理器不注册（原版 Flying 也从不进入；Climbing 依赖梯子查询）；user states/克隆/`CharacterType`/弃用滤波分支（`#if 0`）未移植；`m_atLadder` 随 Climbing 一并裁剪
- **关键行为考证**：simplex 平面距离采用原版 `plane.dot4xyz1(pos)` 约定（点 w 分量为平面常数槽）；SLIDING 分类按原版 `cosSqr < maxSlopeCosine²`（大多数垂直输入方向场景被 max-slope 阻挡平面置零速度抢先判为 SUPPORTED，属原版行为）；`addMaxSlopePlane` 用原始 `maxSlopeCosine`（0.1 下限属未移植的旋转表面接触点投影修正）
- **验证**：simplex（自由运动/地面截停/双墙走廊/三面夹角/静态摩擦吸附/传送带携带/重复平面/确定性）、proxy（落地静置 y≈1.0/撞墙停距/陡坡阻挡/推挤 dynamic box/Cinfo 默认值/确定性）、状态机（走速收敛/跳跃状态链 OnGround→Jumping→InAir→OnGround/单帧冲量/空中重力/离地过渡/predict 不改状态/滤波限加速度/未注册状态 panic）、集成（走墙停距/跳跃落回/推箱/逐位确定性/陡坡流形+分类）；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 16：hkx 对象加载子集（✅ 已完成，2026-11-18）
交付（`base-serialize` 单元测试 10 → 18 个；新增门面集成测试 `hkx_object_loading.rs` 2 个）：

| 模块 | 内容 |
|---|---|
| `base-serialize/vle.rs` | `hkUintVle` 逐式直译：1/2/3/4/5/6/8/9 字节编码、符号值 `encodeSigned`/`decodeSigned`、越界/非法前缀校验；`write`/`write_signed` 供合成测试与未来 writer 使用 |
| `base-serialize/type_meta.rs` | `TST1`/`FST1` NUL 字符串表、`TNA1` 类型身份（name + template params，null type 位 0）、`TBDY` 类型体：`s_optMapping[9]` 压缩 optbits → `Opt::{FORMAT,SUBTYPE,VERSION,SIZE_ALIGN,FLAGS,DECLS,INTERFACES,ATTRIBUTE_STRING,MUTABLE}`，逐式读取 format/subtype/version/size/align/flags/data fields/interface；`Format` 解码 `Kind`/signed/bit 宽/固定数组长度；最多 3 字节全零 padding 校验 |
| `base-serialize/object.rs` | `Bundle::load`：`TAG0` + `DATA`/`TYPE`/`INDX` 容器装配；`ITEM` 12 字节三元组（kind/flags/tid24、offset、count）→ `Item`；`PTCH` 内部/外部 fixup（按 type 分组、DATA 内 u32 LE 目标索引、null 目标）；generic `Value`/`Record`（`Void`/`Opaque`/`Bool`/`Int(i128)`/`Float(f64)`/`String`/`NullPointer`/`Ref(item)`/`Array`/`Record`）；按 parent 链收集字段，固定/动态数组（动态数组读 8 字节指针槽 + +8 处 size）、字符串 fixup 指向 char item；拒绝 `TCM0`、`TNAM`/`TBOD` V0 类型段 |

决策与偏差：

- **generic bundle 而非物理对象**：`base-serialize` 不反向依赖 `phys-*`，输出 `Value` 树供上层转换；`proton::serialize::{Bundle, Value, ...}` 已通过门面导出。
- **fixup 不改原始 DATA**：外部 fixup 目标来自 `PTCH`，内部目标从 DATA 中读取 u32 LE；解析阶段只建立 `BTreeMap<offset, Fixup>`，对象重建时按偏移解析为 `Ref`/`String`/数组，天然避免 Rust 中的裸指针别名与环。
- **数据布局假设**：对象字节按小端解释（Havok SDK x86 in-place 布局），动态数组指针槽固定 8 字节；大端/native-pointer 场景未移植。
- **不支持**：版本 patching（SDKV 仅记录字符串）、compendium 引用、native pointer `TPTR`、属性 decl、vtable 初始化、binary packfile、XML/2014、writer；网格/HeightField 等物理形状到 `phys-shape` 的转换也未纳入。
- **无真实资产**：`original/` 与仓库内均无 `.hkx` 样例，测试使用与 `hff.rs` 一致的合成二进制 writer；reader/writer 同时固化误解的风险由逐式直译 C++ 常量 optbits/VLE/字段布局降低，但仍不宣称与真实 Havok 资产逐位兼容。
- **验证**：VLE 全宽度往返、类型身份/类型体解析、记录字段/字符串/指针/固定数组/动态数组/布尔、非法 item/patch 长度、legacy 段拒绝、门面 smoke test；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### Phase 17：hkx → phys-world 对象转换（✅ 已完成，2026-11-18）
交付（门面 `src/convert.rs` 新模块；新增集成测试 `hkx_scene_loading.rs` 5 个）：

| 模块 | 内容 |
|---|---|
| `proton::convert` | `shape_from_item` / `body_cinfo_from_item` / `scene_from_bundle` / `LoadedScene::populate` / `load_world`：按 `TypeMeta.name` 子串（`hknpSphereShape` / `hknpBoxShape` / `hknpCapsuleShape` / `hknpCylinderShape` / `hknpConvexPolytopeShape` / `hknpMeshShape` / `hknpHeightFieldShape` / `hknp*CompoundShape`）分发到 9 种 `phys-shape` 构造器，字段读取兼容 `m_` 前缀与裸名；vector/quaternion 同时接受 float 数组与 record；body 解析 `m_shape`（Ref 或内联 record）、`m_motionType`、`m_position`/`m_orientation`、`m_mass`、`m_collisionFilterInfo`、`m_userData`、`m_name`；`ConvertError` 覆盖越界/类型未知/缺字段/非法值 |

决策与偏差：

- **转换层放门面 crate**：`base-serialize` 不反向依赖 `phys-*` 的约束不变（Phase 16 决策）；依赖方向 `serialize → convert → shape/body/world`，`convert` 通过门面别名引用各 crate
- **type name 由 `bundle.type_info.types[type_id].name` 取**（源自 TNA1/TST1），`Record` 本身只含 `type_id` + `(字段名, Value)` 列表；形状族判断用 name 子串匹配，不做真实 Havok 类型继承图解析
- `BodyId` 仍由 `World::create_body` 分配（`create_body_with_id` 未移植），`LoadedScene::populate` 按 item 顺序返回 ID；复合体子形状支持内联 record 与 `Ref`；高度场 scale 用 `m_scale`，Mesh/凸体顶点已是局部坐标
- **非资产级兼容**：不处理版本 patching / compendium / vtable / binary packfile；测试用合成 `TAG0`（`Bundle::load` → `scene_from_bundle` → `World::step` 240 步落地收敛，两次加载 `position.to_bits` 逐位一致），仍不宣称与真实 Havok 资产逐位兼容

- **验证**：5 个集成测试覆盖形状转换（Box/Sphere 具体类型与半径/半长）、body 字段、错误输入、端到端仿真落球、跨加载确定性；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

---

## 5. 关键技术难点与对策

| 难点 | 对策 |
|---|---|
| 浮点确定性 | 全程 f32；禁用 fast-math；运算顺序严格照抄 C++；Phase 9 基准把关 |
| 指针 → 借用检查 | 全面采用 ID 索引模式，body/motion/material 存 `Vec`，互相持有 ID |
| C 函数表分发（hknpShapeFunctions） | Rust trait object 或 enum 分发（enum 更快且缓存友好） |
| 反射系统（HK_DECLARE_CLASS） | P0 阶段不需要；序列化阶段用 `serde` + 自定义 hkx 格式层 |
| 内存布局（AABB 16 字节对齐等） | 关键结构用 `#[repr(C, align(16))]` + `bytemuck` |
| 多线程任务图 | 先单线程跑通，再并行化，避免过早优化 |
| SIMD 与确定性 | 标量直译先行；SSE2/AVX 后端必须与标量后端逐位一致，禁止 FMA 收缩与重排；用 Phase 9 基准验收 |
| 哈希容器确定性 | 禁用 `RandomState`，固定种子 FNV-1a；迭代顺序跨运行可复现但不等于 C++ 表序，依赖顺序的逻辑用 `Array` |
| 任务图确定性 | 单线程按 priority→id 调度；并行按拓扑层 barrier，层内任务必须彼此独立；环在执行前检出 |
| 形状 support/AABB | 凸体 support 与 AABB 按 SDK 语义包含 convex radius；体积按名义实体（不含圆角）；Cylinder 用精确有限圆柱范围公式 |
| Body/Motion 生命周期 | `BodyId` 携带 8 位 serial，槽复用后旧 ID 失效；`MotionId` 无 serial（与 SDK 一致），需先用 body 校验；静态 motion 固定 slot 0 |
| 碰撞流形 | 法线 B→A、接触位置在 B、负距离=穿透；凸-凸单点；宽相网格对 `(a,b)` 规范化并排序去重 |
| 求解器/步进 | 顺序脉冲：法向累积 clamp≥0、Baumgarte 穿透恢复、双切向盒式摩擦 clamp；step 单线程按 body 槽位/宽相顺序/约束插入顺序确定性执行；材质未接入时摩擦 0.5、restitution 0 |
| 约束岛 + warm starting | union-find 按接触/关节连通动态体（静态体不合并岛）；逐岛 solve(contacts→joints)，因岛间无共享动态体而与全局顺序等价；接触冲量按 body 对 + B 局部点匹配后跨步复用，切向基仅由法线决定以保证可复用 |
| 岛级休眠 + 并行求解 | 休眠判定用速度阈值 + 低能量计时（`enable_sleeping` 默认关，不破坏既有确定性基线）；整岛同时入睡/唤醒（keyframed 成员或醒着的动态体阻止入睡），`wake_body` 沿关节图 BFS；休眠体跳过窄相/重力/积分，位姿冻结；并行求解按岛分块（worker 克隆缓冲后按岛回并），岛在动态体上不相交故与串行逐位一致 |
| 约束/joint | 关节行在世界空间由 body pose 重建，按 constraint 槽序 + 固定行序顺序脉冲；limits 为单边行（越界才施加回复冲量，lambda 符号检查），motor 每次 solve 一次限幅冲量；pivot 用 3 线性行，hinge 用 2 轴对齐，fixed/prismatic 用四元数误差向量，ragdoll 用 `twist_a × twist_b` cone 轴 + twist 单边行；wheel 用悬挂平面锁定 + 行程单边限位 + axle 对齐（转向 = 底盘侧 axle 旋转由对齐行拉正），悬挂弹簧/驱动电机每 solve 一次限幅冲量；`DISABLE_COLLISIONS` 通过 body 对集合过滤接触 |
| 碰撞过滤 | 32 位 `collision_filter_info` 按 group filter `Config<5,5,5,16>` 打包（layer/subsystem/dont-collide/system group）；同非零 system group 绕过 layer LUT 且 subsystem 全 0 的对不碰撞（原版 `.inl` 语义）；body 对决策在候选对收集末尾（`DONT_COLLIDE` 之后），查询决策经 `QueryFilter.filter_info`；pair 过滤器用引用计数禁用表 + 子过滤器链；所有决策确定性、无随机迭代序 |
| 查询 | raycast 用 GJK 点-形状距离做 conservative advancement（复合体按 BVH/单元/子实例遍历，三角形解析求交）；shape cast 用凸-凸 GJK 距离按闭合速率推进到容差；世界查询按 `(distance, body id)` 决胜负，`QueryFilter` 只决定参与集合；重叠最近点用 EPA 深度返回负距离 |
| vehicle 扩展层 | 显式 `VehicleInstance::step` 代替 Action 管线：raycast 轮碰撞（`hit_fraction` 为世界单位距离，悬挂长度 = 距离 − 半径；closing speed 压缩为负）→ engine/transmission 扭矩曲线直译 → 悬挂弹簧-阻尼（单位质量参数、力非负）→ bullet 风格摩擦冲量（前向驱动/刹车不过冲、侧向锁滑移、摩擦圆钳制）→ 经 `set_body_*` 写回（自动唤醒）；接触式悬挂要求场景满足 `μ·g·h/L < 静轴载`，否则空载轴离地弹跳 |
| 测试/黄金基准 | 41 个原版 UnitTest：6 Ported + 3 Partial + 32 Blocked（依赖未迁移系统）；新增 19 个集成测试；`World::snapshot` 输出 `hvk-trace v1` 位级文本；C++ 黄金基准因 SDK 无生成构建链暂阻塞，不宣称逐位对齐 |
| 多线程/序列化 | 窄相按确定性 chunk 并行，结果按 pair 序回排，接触顺序与单线程一致；HFF/Tagfile 解析容器，Phase 16 重建 generic `Value` 树、Phase 17 由门面 `convert` 按 type name 分发为 shape/body（不做反向依赖）；WorldHash 按 snapshot 的 f32 位值做 FNV-1a |
| Signal 重入语义 | 触发中退订 / 嵌套 emit / 触发中新增订阅（延迟到下一轮）已由 `base-types` 测试锁定，后续改动不得破坏 |

---

## 6. 已确认决策（2026-10-24，2026-11-09 更新）

| 决策点 | 结论 |
|---|---|
| Workspace 布局 | `havok/` 改为虚拟 workspace（`crates/*`，15 个 crate），删除默认模板代码 |
| Git | **已初始化**：首个提交 `791887c` 仅含 LICENSE，其余源码尚未提交（2026-11-09） |
| CI | **未配置**：不纳入 `.github/`，fmt / clippy / test / doc / release 在本地执行（见第 7 节） |
| 数学库 | **从 C++ 直译**，不用 glam，保证 bit-exact float32 |
| 迁移范围 | 按三层划分，暂不含 hkx 序列化 |

## 7. 构建与仓库约定

前置条件：Rust（workspace 声明 `edition = "2024"`、`rust-version = "1.85"`；本机验证工具链为 `rustc 1.101.0-nightly`）。所有命令都在**仓库根目录（`havok/`）**执行：

```powershell
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build --workspace --release

# Phase 9 benchmark（无外部依赖；--quick 为小规模冒烟）
cargo bench -p proton-engine --bench simulation -- --quick

# 文档链接检查（可选）
$env:RUSTDOCFLAGS='-D warnings'; cargo doc --workspace --no-deps
```

CI：当前**未配置**（仓库内无 `.github/`），上述检查需在本地或外部流水线执行。

脚手架约定：
- 所有 crate 设为 `publish = false`（源码衍生自专有 Havok 代码，防止误发布）
- crate 间依赖严格单向，方向见第 3 节
- 当前零外部依赖，`rayon`/`thiserror`/`bytemuck` 按阶段引入
- `Cargo.lock` 已提交入库（应用型 workspace，锁定依赖）
- `../original/` 与 `target/` 不纳入仓库（后者已由 `.gitignore` 忽略）

## 8. 当前状态

- [x] 原项目结构分析
- [x] 迁移计划制定
- [x] Phase 0：工程脚手架 —— workspace 根 + 15 个 crate 骨架 + `.gitignore`（检查命令见第 7 节；CI 未配置）
- [x] Phase 1：`base-types` —— scalar / bitops / Pair / Handle / RefPtr / UFloat8 / Signal / HkResult 全部实现，49 个单元测试 + 13 个文档测试
- [x] Phase 2：`base-math` —— Vector4/D、Quaternion、Matrix3、Transform、QTransform、QsTransform、Aabb、Half 完成（标量后端），52 个单元测试 + 5 个文档测试
- [x] Phase 3：`base-memory` + `base-container` —— 分配器/统计、Array、BlockStream、确定性 HashMap/HashSet 完成，8 + 27 个单元测试 + 5 个文档测试
- [x] Phase 4：`base-thread` —— SpinLock、ThreadPool、JobQueue、TaskGraph、parallel_for 完成（std::thread，零依赖），29 个单元测试 + 5 个文档测试
- [x] Phase 5：`phys-types` + `phys-shape` —— 句柄/枚举/标志位、统一 Shape trait、6 种凸体、Compound、Mesh+BVH、HeightField 完成，16 + 41 个单元测试
- [x] Phase 6：`phys-body` + `phys-world` —— Body/BodyCinfo/BodyManager、Motion/MotionManager、WorldCinfo/World 与单线程运动学 step 骨架完成，18 + 10 个单元测试
- [x] Phase 7：`phys-collide` —— 网格宽相、GJK 最近点/相交、EPA 穿透、单点凸-凸流形与 sphere-sphere 解析特化完成，22 个单元测试
- [x] Phase 8：`phys-solver` + 完整步进 —— 顺序脉冲接触求解器与 gravity→collide→solve→integrate 管线完成，5 + 13 个单元测试
- [x] Phase 9：测试与验证（Rust 侧）—— 41 个原版 UnitTest 矩阵（6 Ported / 3 Partial / 32 Blocked）、19 个集成测试、确定性 trace、无依赖基准 harness 完成；C++ 黄金基准待原版 SDK 可构建
- [x] Phase 10（可交付子集）：并行窄相 + `step_with_threads`、HFF + Tagfile 容器解析、WorldHash 完成
- [x] Phase 10（后续子集，2026-11-12）：约束/joint 系统（BallSocket/Hinge/Fixed + 世界接入）、查询系统（world raycast / closest point）与铰链 limits/motors 完成
- [x] Phase 11（约束/joint + 查询系统，2026-11-16）：Ragdoll cone/twist、Prismatic、StiffSpring、PointToPlane 关节与约束碰撞屏蔽、body↔constraint 映射；shape cast（凸-凸 conservative advancement + mesh/height field/compound 目标）、mesh/height field/compound raycast、`QueryFilter`、`closest_points_to_shape`、AABB 查询完成
- [x] Phase 12（约束岛 + warm starting，2026-11-16）：接触冲量累积与跨步缓存（B 局部点 + 法线匹配）、union-find 约束岛（静态体不合并）、逐岛 solve(contacts→joints)、`World::islands`/`WorldStats::last_island_count`/`WorldCinfo::with_warm_starting` 完成；`phys-solver` 40、`phys-world` 62 个单元测试，集成测试 5 个
- [x] Phase 12 后续子集（位置/弹性电机 + 关节 warm starting，2026-11-16）：铰链 `HingeMotor::{Velocity,Position,Spring}` 与滑轴 `PrismaticMotor::{Position,Spring}` 电机、`JointImpulse` 关节冲量缓存跨步复用（单边行接触式 clamp + 越界回放）、`HingeCinfo` 参考轴提升、`set_prismatic_motor` API 完成；`phys-solver` 51、`phys-world` 70 个单元测试，新增集成测试 `motors_and_joint_warm_starting.rs` 5 个
- [x] Phase 12 后续子集（岛级休眠/唤醒 + 按岛并行求解，2026-11-16）：`enable_sleeping` 速度阈值休眠（整岛入睡/唤醒传播、`wake_body` 关节图 BFS、休眠跳过窄相/重力/积分）与 `step_with_threads` 按岛并行求解（worker 分块克隆缓冲 + 按岛回并，与串行逐位一致）完成；`phys-world` 75 个单元测试，新增集成测试 `sleeping_and_parallel_solving.rs` 4 个
- [x] Phase 12 后续子集（wheel 约束，2026-11-17）：`WheelJoint`（悬挂平面锁定 + 行程限位 + 弹簧-阻尼 + axle 对齐/转向 + 速度驱动电机）与 `WheelCinfo`/`set_steering_angle`/`set_wheel_suspension`/`set_wheel_drive` 完成；`phys-solver` 56、`phys-world` 78 个单元测试，新增集成测试 `wheel_constraints.rs` 6 个
- [x] Phase 13（vehicle 扩展层，2026-11-17）：`phys-extensions/vehicle`——raycast 悬挂（ray 距离 → 悬挂长度、SDK closing speed 约定）、默认 transmission/engine（扭矩曲线逐式直译 + 阈值换挡 + 倒挡）、悬挂弹簧-阻尼力（单位质量参数）、bullet 风格摩擦求解器（前向驱动/刹车冲量 + 侧向锁滑移 + 摩擦圆钳制）、冲量写回底盘/地面（含角响应）；`phys-extensions` 17 个单元测试，新增集成测试 `vehicle_extension.rs` 8 个（行驶/倒挡/转向/刹车/手刹/确定性）
- [x] Phase 14（碰撞过滤系统，2026-11-17）：`phys-collide/filter.rs`——`CollisionFilter` trait（info/body-pair/query 三入口）、`GroupCollisionFilter`（5/5/5/16 位 filter info + 32 项 layer LUT，按原版 `.inl` 直译）、`PairCollisionFilter`（引用计数禁用表 + 子过滤器链）；`World::set_collision_filter` 接入候选对收集与全部查询（`QueryFilter::filter_info`）；新增集成测试 `collision_filters.rs` 8 个
- [x] Phase 15（CharacterControl，2026-11-18）：`phys-extensions/character`——`hkSimplexSolver` 逐式直译（1d/2d/3d 平面约束 + 摩擦 + 罚距）、`hknpCharacterProxy` 子集（自维护流形 + shape cast 移动 + maxSlope 阻挡平面 + 贯穿恢复 + resolveContacts 线性冲量）、`hknpCharacterSurfaceInfo`、状态机（MovementUtil + OnGround/Jumping/InAir + Manager/Context 速度滤波）；`phys-extensions` 单元测试 17 → 46 个，新增集成测试 `character_control.rs` 5 个
- [x] Phase 16（hkx 对象加载子集，2026-11-18）：`base-serialize` 新增 `hkUintVle`、`TNA1`/`TBDY` 反射元数据、`ITEM`/`PTCH` fixup 与 generic `Bundle`/`Value` 重建；`base-serialize` 单元测试 10 → 18 个，新增门面集成测试 `hkx_object_loading.rs` 2 个
- [x] Phase 17（hkx → phys-world 对象转换，2026-11-18）：门面 `src/convert.rs` 把 generic `Value` 树转成 shape/body（支持 9 种形状族 + body 字段），放门面避免 `base-serialize` 反向依赖；新增集成测试 `hkx_scene_loading.rs` 5 个（类型名分发/字段转换/错误输入/端到端落球仿真/跨加载逐位一致）
- [ ] 后续：`hkx` 版本 patching / compendium / binary packfile（Phase 16 仅覆盖当前 `TAG0` 子集）；Ragdoll 调度层（约束族已在 Phase 11 交付）；CharacterControl 后续（Climbing/ladder、触发器与 contact listener）

仓库现状（2026-11-18）：

- 工具链：`rustc 1.101.0-nightly`；workspace `rust-version = "1.85"`、`edition = "2024"`
- 命名迁移（2026-11-16 完成）：门面 crate 迁至 workspace 根并改名 `proton-engine`（lib 名 `proton`）；14 个子 crate 目录与 `Cargo.toml`、代码引用由 `hk-*` / `hknp-*` 改为 `base-*` / `phys-*`（`hk_types → base_types` 等），C++ 符号名（`hknpWorld`、`hkVector4f`、`HK_ASSERT` 等）保持不变；clippy 与全量测试通过
- git：Phase 12/13/14/15 均已提交（最新 `faf2ce5 CharacterControl`）；Phase 16（hkx 对象加载子集）与 Phase 17（hkx → phys-world 转换）已实现并全绿
- 外部依赖：0

## 9. 文档

| 文档 | 内容 |
|---|---|
| [docs/phase9-test-matrix.md](docs/phase9-test-matrix.md) | 41 个原版 UnitTest 的逐项映射（Ported / Partial / Blocked）、S1–S12 场景矩阵、基准矩阵与验收标准 |
| 各 crate 模块级 doc comment | 决策与偏差随 Phase 交付记录在对应 `crates/*/src/` 模块文档中 |

生成 API 文档：

```powershell
$env:RUSTDOCFLAGS='-D warnings'; cargo doc --workspace --no-deps --open
```

## 10. 许可证

[MIT License](LICENSE)。

注意：本仓库源码衍生自专有的 Havok SDK 代码，仅用于学习与研究；所有 crate 均设为 `publish = false`，不发布到 crates.io。原始 C++ SDK（`../original/`）受其自身许可约束，不随本仓库分发。

