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
| `base-serialize` | HFF 结构化流 + `TAG0` 容器 + `TCM0` compendium + 2014 legacy binary tagfile + 类型元数据/对象重建 + 格式嗅探/结构型·函数型·type-change 版本 patching + 2014 packfile 头校验/外部转换适配 + 统一加载入口 | ✅ 已实现（Phase 10/16 + 后续子集） |
| `phys-types` | hknpTypes：BodyId/BodyIndex、句柄、MotionType/ShapeType、标志位 | ✅ 已实现（Phase 5） |
| `phys-shape` | 形状体系：统一 Shape trait + 凸体/复合/网格/高度场 + 质量属性（密度/目标质量、COM、完整惯量张量、主轴、复合聚合）+ 可移植 15 位压缩惯量、`MassDistribution`、确定性 Jacobi 对角化与形状质量缓存 | ✅ 已实现（Phase 5 + 后续子集） |
| `phys-collide` | 宽相/窄相/Manifold + 查询（raycast/shape cast/closest points/AABB，含 mesh/height field/compound 遍历）+ 碰撞过滤（`CollisionFilter` trait、group layer/subsystem/system-group filter、pair 禁用表） | ✅ 已实现（Phase 7/11/14） |
| `phys-body` | hknpBody/BodyManager/Motion/MotionManager + Body/Motion 完整质量属性与世界逆惯量张量（保留 Vector4 对角兼容视图）+ Body 质量来源标记与缓存版本 | ✅ 已实现（Phase 6 + 后续子集） |
| `phys-solver` | 迭代式接触求解器 + warm starting（`ContactImpulse`/`JointImpulse` 累积冲量）+ 完整世界逆惯量张量矩阵路径（对角快路径保留）+ 关节约束（ball-socket/hinge/fixed/ragdoll/prismatic/spring/point-to-plane/wheel，含 limits 与 velocity/position/spring 电机）+ 岛内子集求解 API | ✅ 已实现（Phase 8/11/12 + 后续子集） |
| `phys-world` | hknpWorld 组装 + collide→islands→solve(contacts+joints)→integrate 管线 + 接触冲量缓存 + 关节冲量槽缓存 + 约束管理器（含 wheel：悬挂弹簧/行程限位/转向/驱动、limited-hinge/6DOF 约束框架轴限位关节）+ 质量派生、形状质量缓存、质量变更信号与共享运动完整惯量张量合并 + 查询 API + 岛级休眠/唤醒 + 按岛并行求解 + Dynamic↔Keyframed 运动类型切换 + 共享运动（attached body/attach/detach） | ✅ 已实现（Phase 8/11/12 + 后续子集） |
| `phys-extensions` | WorldHash + Vehicle 扩展层（raycast 悬挂/engine/transmission/摩擦）+ CharacterControl（simplex/proxy/状态机含 Climbing + trigger 追踪 + 本地 listener/phantom + post-solve 管理器 + 对象/角色交互 + shape-key trigger + rigid-body 路线/listener）+ Ragdoll 调度层（骨骼绑定/约束链/drive-to-pose/硬关键帧）+ Ragdoll 状态机/电机 blender/penetration utility/约束松弛 + at-rest 转换 + 用户自定义状态 + shared-motion 驱动 | ✅ 已实现（Phase 10/13/15 + 后续子集） |
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
- `create_body` 把 SDK 的 `allocateBody` + `addBody` 折叠为一步；Phase 6 时质量 `<0`（使用 shape 质量）暂按无限质量处理，形状质量属性已在后续「Ragdoll 深度补齐」子集接入并按形状派生质量/COM/惯量
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
- Phase 8 时质量 `< 0` 仍按无限质量处理（沿用 Phase 6 决策），动态 body 需显式 `with_mass`；该限制已在后续「Ragdoll 深度补齐」子集解除：默认负质量现在从形状派生质量/COM/完整惯量，显式正质量仍保持旧的零惯量兼容语义
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
- **resolveContacts 子集**：仅线性冲量（跳过角惯性项），经 `set_body_linear_velocity` + `wake_body` 写回；本阶段未移植 phantom presence/trigger/listener/LOD/multi-thread task graph（本地 listener/phantom 见后续子集），proxy 为独立对象按调用传 `&World`
- StateMachine：`Climbing`/`Flying` 类型保留但本阶段默认管理器不注册（原版 Flying 也从不进入；Climbing 当时依赖梯子查询）；user states/克隆/`CharacterType`/弃用滤波分支（`#if 0`）未移植；`m_atLadder` 当时随 Climbing 一并裁剪（Climbing 与 trigger 追踪见后续子集）
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
- **不支持**：函数型/type-change 版本 patching（结构型 rename/remove/add/default-set 见后续子集）、native pointer `TPTR`、属性 decl、vtable 初始化、binary packfile 容器（仅嗅探 header）、XML/2014 写入、writer；compendium 引用已由后续子集补上；网格/HeightField 等物理形状到 `phys-shape` 的转换也未纳入。
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
- **非资产级兼容**：本阶段未处理版本 patching / compendium / vtable / binary packfile（后续子集补上结构型 patching 与 compendium）；测试用合成 `TAG0`（`Bundle::load` → `scene_from_bundle` → `World::step` 240 步落地收敛，两次加载 `position.to_bits` 逐位一致），仍不宣称与真实 Havok 资产逐位兼容

- **验证**：5 个集成测试覆盖形状转换（Box/Sphere 具体类型与半径/半长）、body 字段、错误输入、端到端仿真落球、跨加载确定性；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### Phase 16（后续子集）：hkx 类型 compendium（✅ 已完成，2026-11-18）
交付（`base-serialize` 单元测试 18 → 27 个；门面集成测试 `hkx_object_loading.rs` 2 → 3 个）：

| 模块 | 内容 |
|---|---|
| `base-serialize/compendium.rs` | `Compendium::load`：`TCM0` 类型合订本解析——`TCID` 签名数组（`u64 LE`，8 字节对齐）+ `TYPE` 元数据；`contains_signature`；按调用顺序合并类型表（`None` 不覆盖已有定义），bundle 自身 `TYPE` 最后合并 |
| `base-serialize/tagfile.rs` | 新增 `TCRF` / `TCID` 常量与 `Tagfile::compendium_reference()`：解码 8 字节小端签名，忽略 inplace 指针尾部，短段返回错误 |
| `base-serialize/object.rs` | `Bundle::load_with_compendia`：校验 `TCRF` 与已加载 compendium 签名匹配（不匹配返回 `CompendiumNotFound`）；无 `TYPE` 的 bundle 可由 compendium 提供类型；`Bundle::load` 行为保持不变 |

决策与偏差：

- 签名语义按原版 `handleCompendiumId` / `handleCompendiumReference`：`TCID` 是合订本各中间状态的签名数组，`TCRF` 引用其中一个；8 字节之后的 inplace `hkArrayView` 尾部不加载（非 inplace 路线）。
- 仅移植类型表合并；`TSHA` / `TSEQ` 滚动哈希、增量类型检查器、vtable 初始化与写格式未移植。
- `Bundle::load`（无 compendium）遇到 `TCRF` 直接报 `CompendiumNotFound`，不静默忽略引用；合成测试覆盖共享类型加载、签名不匹配、本地 `TYPE` 覆盖与非法 `TCID` 长度。
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### Phase 16（后续子集）：legacy binary tagfile（2014）解析（✅ 已完成，2026-11-18）
交付（`base-serialize` 单元测试 27 → 31 个；门面集成测试 `hkx_object_loading.rs` 3 → 4 个）：

| 模块 | 内容 |
|---|---|
| `base-serialize/tagfile2014.rs` | `hkTagfileReadFormat2014::Reader` 子集：`CAB00D1E`/`D011FACE` magic 与大小端探测、符号-幅度变长整数、字符串表（正数新建 / 负数回引）、`TAG_FILE_INFO`（v2–v6，含 SDK 字符串、predicate 列表、v6 双精度）、`TAG_METADATA` 类定义（name/version/parent/fields + `hkLegacyType`）、`TAG_OBJECT*`/`TAG_OBJECT_BACKREF`/`TAG_FILE_END` 对象树；按成员存在位图重建 `Value::Record`，解析后统一解析前向引用 |

决策与偏差：

- **两套容器不可混同**：`CAB00D1E`/`D011FACE` 是 2014 legacy 变长 tag 流，没有 HFF 段树，也没有 `DATA`/`TYPE`/`INDX`；真正的 packfile 容器由 SDK 外部 DLL 转换后交给同一 reader，本子集不含该转换路径。
- **版本**：仅支持 v2–v6；v0/v1 依赖静态 tag 字符串表（`HK_TAG_STRING_LIST`），显式返回 `LegacyVersion`，未知版本返回 `UnsupportedVersion`。V6 按原版把 real 读为 64 位。
- **字段类型**：支持 void / byte / int / real / vec4/8/12/16 / cstring / struct / object 指针，以及定长 tuple、动态数组（每元素的 child-size 前缀）、同构 struct 数组；未知 `hkLegacyType` 返回 `UnsupportedType`，不做静默跳过。
- **引用语义**：`TAG_OBJECT_REMEMBER` 的 id 从 1 起（0 为 null）；前向引用记录为 `ForwardRef`，全流读入后按 remembered 表解析，未解析则报错。
- **非资产级兼容**：测试用按原版线格式手写的合成 writer（含变长整数与字符串回引），仓库与原版目录均无真实 `.hkx`/tagfile 样例，不宣称与真实 Havok 资产逐位兼容。
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：Ragdoll 调度层（✅ 已完成，2026-11-20）
交付（`phys-extensions` 单元测试 46 → 49 个；新增门面集成测试 `ragdoll_scheduling.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/ragdoll.rs` | `hknpRagdoll` / `hknpRagdollController::BodyController` / `hknpKeyFrameUtil` 子集：`RagdollData` 骨骼层级（name/parent/body/reference pose/twist+plane axis/limits）→ `Ragdoll::create` 从两体当前世界位姿推导局部 pivot/轴并建 `RagdollCinfo` 链（相邻体碰撞屏蔽）；`calc_desired_pose_world_space`（父子局部姿态合成）、`drive_to_pose`（速度驱动 + strength 插值）、`apply_hard_keyframe`（精确设姿 + KeyFrameUtil 所需速度）、`pose_world_space`、`set_enabled` |

决策与偏差：

- 原版 `hknpRagdollData` 直接携带 `hknpPhysicsSystemData` 的约束 cinfo；本子集改由骨骼参考姿态与建链时的 body pose 推导 pivot/轴，joint 类型固定为已有 `RagdollCinfo`（cone/twist），不移植 `hknpPhysicsSystemData` 与 `hkaSkeleton`。
- **显式驱动代替 Action/StateMachine**：调用方在 `World::step` 前调用 `drive_to_pose` 或在需要同步时调用 `apply_hard_keyframe`；状态机（Keyframed/Dynamic/Powered）、电机 blender、penetration utility、`hknpEaseConstraintsAction` 与 motion sharing 未移植。
- 角速度按 `hknpKeyFrameUtil::calcRequiredVelocity`：小角度用四元数虚部近似、大于 20° 用轴角精确式；`drive_to_pose` 按 `strength × bone_strength` 与当前速度插值，写回经 `set_body_*` 自动唤醒。
- 无 body 的末端骨骼按父世界姿态 × 参考变换回退（对应原版 `getWorldFromBoneTransform` 的末端逻辑）；骨骼顺序要求父在子前，`create` 校验并在非法索引时返回 `InvalidBone`。
- 验证：层级姿态合成、非法 dt/strength/pose 长度、硬关键帧轴角速度；集成覆盖三体链约束建立与碰撞屏蔽、90 步 drive 收敛、硬关键帧传送、两次运行 `hvk-trace v1` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：Ragdoll 状态机 + powered 电机 blender（✅ 已完成，2026-11-20）
交付（`phys-world` 新增 1 个单元测试，78 → 79 个；`phys-extensions` 新增 4 个单元测试，49 → 53 个；新增集成测试 `tests/ragdoll_state_machine.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-world` | `World::set_body_motion_type`（Dynamic ↔ Keyframed，`setBodyMotionType` 子集）：改 motion type 与 body flags、清理休眠状态并唤醒动态体；静态体/未知 ID 返回 false，不重分配 motion slot |
| `phys-extensions/ragdoll_controller.rs` | `RagdollState::{Keyframed,Dynamic,Powered}` 状态机（tag mask 以 bone strength 表示，0 表示排除）、`set_state` 切换 tagged body 运动类型、`keyframeRoot` 精确锁根；`begin_timed_transition` 复刻 `AutoTransitionTimed` 的 `1 - elapsed/total` 淡出后进入终态；`RagdollMotorParams::blend` 按 SDK `lerp(lower, upper, sqrt(strength))` 混合 maxForce/tau/damping（recovery velocity 原样透传）；`Ragdoll::apply_hard_keyframe_bone` 支持单骨 keyframe |

决策与偏差：

- **powered 用速度驱动代替位置电机**：本仓库 `RagdollJoint` 是 limits-only 行集，未移植 constraint atom / 位置电机；blender 仍按原版公式计算 SDK 电机参数并暴露 `last_motor()` 供校验，实际位姿驱动走已移植的 `Ragdoll::drive_to_pose` 速度插值（根骨按 `keyframeRoot` 精确设姿）。
- **状态切换仅 Dynamic ↔ Keyframed**：`setBodyMotionType(STATIC)` 需要重新分配 motion slot，本子集拒绝静态转换（状态机三种基本状态不依赖静态切换）；timed transition 采用"淡出后切换终态"，未移植 `AutoTransitionAtRest`、约束松弛 action 与 motion sharing（penetration utility 见下一小节）。
- **淡出时序照抄原版**：先按 `elapsed/duration` 计算强度并执行 via 状态，再累加 `elapsed`；`elapsed + dt >= duration` 时强度置 0、执行最后一次后切到终态。
- **验证**：Keyframed 重力下 60 步保持精确位姿、Powered 逐步注入驱动速度并产出正确 `sqrt` 混合参数、timed transition 强度单调淡出并进入 Dynamic、混合场景两次运行 `hvk-trace v1` 逐位一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：Ragdoll penetration utility（✅ 已完成，2026-11-20）
交付（`phys-extensions` 新增 3 个单元测试，53 → 56 个；新增集成测试 `tests/ragdoll_penetration.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/ragdoll_penetration.rs` | `hknpRagdollPenetrationUtil` 子集：`PenetrationStatus`（RAY=1/ANCESTOR=2/CLOSEST_POINT=4）、`ResolveMode`（RESOLVE/RESOLVE_HIERARCHICALLY/RESOLVE_TOWARD_POSITION）与 SDK 默认 `Input`；`BodyHierarchy::from_ragdoll`（bone strength 充当 tag mask）、根嵌入检测（closest-points 深度阈值 + 回退射线）、父子骨骼段射线、祖先状态级联、分层 `DONT_COLLIDE` 掩码、`corrective_impulse`（目标分离速度 `maxResolutionSpeed`）、重力抵消与线/角冲量回写；`RagdollController::set_penetration_input` 在状态步前调用，`last_penetration()` 暴露检测结果 |

决策与偏差：

- **查询能力差异**：`closest_points_to_shape` 需要正搜索半径，因此用 `radius = max(|rootPenetrationTolerance|, 0.1)` 查询后按 `distance <= tolerance` 判定穿透；射线查询不支持"仅背面三角形"与"忽略整个 ragdoll"，用 `QueryFilter::ignore_body` 忽略被查 body、其余靠 `filter_info`；`raycast` 返回最近命中而非 batch collector。
- **冲量回写**：世界层无 `applyBodyImpulseAt`，按 `v(p)=v_linear+ω×(p−COM)` 计算两体点速度，修正量 `dir·velocityDiff·mass + additionalImpulse` 直接写入 motion 的线速度，并以 `orientation.rotate_inverse` → 对角 inverse inertia → `rotate` 的近似路径更新角速度（不做完整惯量张量）；检测快照判定"是否解决"，与原版一致不在施冲量后重查。
- **掩码语义**：根嵌入时给所有"有子节点的父 body"置 `DONT_COLLIDE`（单骨 ragdoll 无父节点故不置位，与原版一致）；普通穿透时 RAY/ANCESTOR 节点置位、干净节点清除；`reenable_collisions` 清除全部标志。
- **验证**：根嵌入球体被向上推离、无穿透场景返回 resolved、层级射线命中地面并记录父节点 RAY + 子节点 ANCESTOR、控制器在状态步前完成修正；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：Ragdoll 约束松弛 + at-rest 自动转换（✅ 已完成，2026-11-20）
交付（`phys-extensions` 新增 2 个单元测试，56 → 58 个；新增集成测试 `tests/ragdoll_easing.rs` 3 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/ragdoll_ease.rs` | `EaseConstraintsAction`：`_saveLimits` / `loosenConstraints` / `restoreConstraints(duration)` / `applyAction` 子集——保存 ragdoll cone/twist 原值，按当前相对位姿 + `SMALL_TOLERANCE=0.01` 放宽，按 SDK 权重 `dt/(duration−elapsed)` 线性回弹，`duration=0` 立即精确还原；`ConstraintEasingMode::{Enforce,LoosenNoRestore,LoosenThenRestore}` 与状态映射（Dynamic=LoosenThenRestore/4s，Keyframed/Powered=Enforce） |
| `phys-extensions/ragdoll_controller.rs` | 状态切换按 SDK `hknpRagdollStateManager::setState` 规则处理旧 action（进入 Enforce 态立即还原，否则直接丢弃）、新状态按需创建 action；`begin_at_rest_transition` 复刻 `AutoTransitionAtRest`——via 状态逐步执行，所有 tagged body 的线速度各分量 `≤1e-2`、角速度 `≤1e-1` 时切入终态 |

决策与偏差：

- **只支持 ragdoll 约束**：本仓库没有 limited-hinge / 6DOF 的 ease atom，plane-angle limits 也未移植，因此 action 仅记录/修改 `RagdollCinfo` 的 cone/twist 三个值；`lerp(current, original, weight)` 与原版宏一致，观测到的角由两 body 的世界 twist/plane 轴算出（twist 使用原版 `atan2` + 归一化到 limits 窗口）。
- **状态切换按"新状态"决定**：原版在 `setState` 中先把 `m_stateId` 改为目标态再判断 easing mode；因此从 Dynamic 进入 Keyframed/Powered 会立即精确还原，而非按离开的状态判断。
- **at-rest 无计时器**：原版 `AutoTransitionAtRest` 的头部注释描述的是 timed 行为，实际实现仅做速度阈值判定；本子集照实际实现处理，tagged body 为空时视为已静止。
- **验证**：保存/放宽/精确还原、4 步回弹收敛；控制器 Dynamic 进入后放宽、进入 Keyframed 立即还原、速度超阈值保持 via 状态且归零后切入终态；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：hkx 格式嗅探 + 版本 patching（✅ 已完成，2026-11-20）
交付（`base-serialize` 新增 3 个单元测试，31 → 34 个；新增集成测试 `tests/hkx_version_patching.rs` 3 个）：

| 模块 | 内容 |
|---|---|
| `base-serialize/version.rs` | `detect_format` 复刻 `hkSerializeDetail` 嗅探（2014 packfile 16 字节头 `57E0E057/10C0C010`、TAG0/TCM0/TAGF/TCMP、NT6 `CAB00D1E/DEADC0DE`、2014 tagfile `CAB00D1E/D011FACE` 双端序、XML）；`LegacyType` / `PatchInfo` / `PatchComponent` / `PatchSet` 建模 `hkReflect::Version::PatchInfo`，支持 member rename/remove/add-with-default/default-set 四种结构操作，递归应用于 `Bundle.objects` 的 `Value::Record`/`Array` 并产出 `PatchReport` |
| `base-serialize/object.rs` | `Bundle.sdk_version: Option<String>` 保留 `SDKV` 字符串作为版本键（此前被丢弃） |

决策与偏差：

- **patch 表由调用方提供**：2021.2 源码中旧的 `hkVersionRegistry`/`hkVersionPatch` 已移除，patch 是编译期 C++ 常量（含函数指针），hkx 文件内**不含** patch 定义；本子集用 Rust 值建模结构型 patch，未移植函数型/type-change/template 参数 patch 与 `PatchFunctionHelper`。
- **真正的 packfile 容器仍阻塞**：仓库内只有 16 字节 header sniffer，容器解析由外部 `hkCompatFormats.dll` 完成并转换为 2014 tagfile，本子集只做格式识别，不宣称能加载二进制 packfile。
- **版本匹配按 `TypeMeta.version` + 类名**：`TBDY` 的 `OPT_VERSION`（bit 4）此前已解码但无 fixture 覆盖；本子集新增带 version 的合成 `TAG0`，以 `old_name`/`old_version` 精确选 patch，未按 SDK 的依赖图做链式解析（调用方可连续登记各版本 patch）。
- **验证**：SDKV 与类型版本解码、rename/default-set/add 应用与 `PatchReport` 计数、packfile/tagfile/NT6/XML 头嗅探；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：hkx packfile 适配 + 函数/type-change patching（✅ 已完成，2026-11-21）
交付（`base-serialize` 单元测试 34 → 40 个；新增集成测试 `tests/hkx_packfile_and_function_patches.rs` 7 个）：

| 模块 | 内容 |
|---|---|
| `base-serialize/packfile.rs` | `BinaryPackfile2014` 16 字节头解析（`magic[2]` / `userTag` / `fileVersion`，当前版本常量 11）、`PackfilePolicy` 可选 tag/version 校验、`PackfileConverter` 外部转换 trait、`convert_packfile` 统一校验与输出格式检查；错误覆盖短头、坏 magic、策略不匹配、缺少转换器、转换失败与非 tagfile 输出 |
| `base-serialize/loader.rs` | `load_hkx` 统一入口与 `LoadOptions`：`detect_format` 分流到 `Bundle` / `Compendium` / `Bundle2014`，packfile 先经转换器，再复用现有 reader；加载后可选应用 patch chain；`LoadedHkx` 与 `LoadHkxError` 提供统一结果/错误语义 |
| `base-serialize/version.rs` | `PatchComponent` 新增 `DataObjectFunction` / `VarFunction` / `TypeChanged` / `VarTypeChanged`；`PatchFunctionHelper` 提供确定性 `new_record`/`new_array`/`new_string`/`null_pointer`；`PatchSet::apply_chain`/`apply_checked` 按 `(类名, 版本)` 链式升级，推进 `TypeInfo` 元数据并检测重复 patch、循环与超过 64 步的链；`PatchReport` 增加函数补丁、type-change 与链步计数 |

决策与偏差：

- **packfile 仍不内置容器解析**：原版 `hkSerializeDetail.cpp` 只注册 detector，`hkCompatFormats.dll` 才把 packfile 转成 2014 tagfile；本子集解析/校验头部并提供可注入转换器，不声称能直接读取真实 packfile 正文。
- **函数补丁用函数指针**：不移植 C++ `hkDataObject`/`hkReflect::Var` 与原生函数指针状态；回调只操作 generic `Value`，新对象通过显式 helper 构造，保证可比较、可重复、无闭包捕获环境。
- **type-change 按泛型值建模**：记录旧/新 `LegacyType`、类型名与 tuples 字符串，转换函数接收旧/新描述并写回 `Value`；不移植内存偏移式原地改写、指针/数组 ID 失效与 vtable 初始化。
- **版本链显式报错**：原版注册表对重复 `(类名, 版本)` 忽略后者；本子集用 `PatchSet::conflicts` / `PatchError` 暴露重复、循环与超长链，避免静默停止。
- **未覆盖**：template 参数 patch、`DEPEND`/`PARENT_SET`、native pointer 段、writer、真实 `.hkx` 资产与 DLL 转换结果等价性；测试全部使用合成二进制。
- **验证**：packfile 头/策略/转换器失败路径、统一入口分流、函数补丁修改记录与字段、type-change 值转换、v1→v2→v3 链、重复/循环错误、相同输入两次加载逐值一致；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：CharacterControl Climbing + trigger 追踪（✅ 已完成，2026-11-20）
交付（`phys-extensions` 新增 1 个单元测试，58 → 59 个；新增集成测试 `tests/character_climbing.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/character/state_machine.rs` | `CharacterInput::at_ladder`（原版 `m_atLadder`）、`states::Climbing`（gain=1、climb speed=10、ladder attraction=-0.1、max delta=100，使用已移植的 `calculate_movement` 在梯面坐标系求解），默认管理器注册 Climbing；OnGround/InAir 在 jump/supported 判断后按 `at_ladder` 转入 Climbing，Climbing 退出时无梯则 InAir、跳跃则沿梯面法线施加 `|gravity|` 速度后 InAir |
| `phys-extensions/character/triggers.rs` | `TriggerTracker` 逐帧用扩展 AABB 取候选 body 并与上一帧集合做差，产出按 `BodyId` 排序的 `TriggerEvent::{Entered,Exited}`；`clear()` 发出全部退出事件 |

决策与偏差：

- **梯子是应用层概念**：2021.2 源码内没有 `hknpCharacterLadderUtils` 或 ladder volume 查询，原版仅由应用设置 `m_atLadder` 与 surface normal；本子集照此提供状态和输入字段，不内置梯子检测。
- **trigger 用 AABB 近似**：原版通过 material `TRIGGER_TYPE` 在查询 collector 中分离 trigger hit、并维护 phantom body 与 `hknpProxyListener` 回调；本子集当时只用 `World::query_aabb` + body AABB 做重叠判定，按 `BodyId` 识别，未移植 shape key、`STATUS_UPDATED`、listener trait、phantom 与 one-frame pass-through 的同帧双事件（listener/phantom 见下一小节）。
- **验证**：OnGround/InAir 进梯、Climbing 移动速度非零、沿法线跳跃速度精确、离梯回 InAir；trigger 进入/静止无事件/退出/清空；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：CharacterControl 本地 listener + phantom presence（✅ 已完成，2026-11-20）
交付（新增集成测试 `tests/character_listeners.rs` 4 个）：

| 模块 | 内容 |
|---|---|
| `phys-extensions/character/listener.rs` | `CharacterProxyListener` 本地回调子集：`contact_added` / `contact_removed` / `shape_changed` / `character_callback` / `trigger_volume_interaction`，附 `CharacterProxyState`（position/velocity/contact_count）快照 |
| `phys-extensions/character/proxy.rs` | `add_listener` / `remove_listener`（按 id，按注册逆序派发；add 时立即 `shape_changed`）、`dispatch_trigger_events`；`create_phantom` / `phantom` / `destroy_phantom`（keyframed + `DONT_COLLIDE` + 同名 + 同 shape/filter，并从 `QueryFilter::ignore_body` 排除自身）；`check_support` / `refresh_manifold` / `integrate` 按调用前后 manifold 差分批量派发接触事件，`integrate` 末尾同步 phantom 并触发一次 `character_callback` |

决策与偏差：

- **回调时机**：原版在每个 manifold 变更点即时派发，`characterCallback` 由 world post-solve signal 触发；本仓库无 signal/action manager，改为每个 public proxy 调用后批量差分派发，`characterCallback` 在 `integrate` 末尾触发一次（顺序为 removed → added → step）。
- **phantom 子集**：用 `BodyCinfo` 在 `World` 中创建 keyframed、`DONT_COLLIDE` 的独立 body 并在 `integrate` 末尾同步 transform；没有 body property 标记、vtable、`setShape` 同步、deferred update 或多线程任务图。
- **未移植回调**：constraint 过滤、proxy↔proxy / object interaction、VDB monitor 及 rigid-body route listener；trigger 仍是 AABB 近似 + `BodyId` 标识，不提供 shape key 级事件。
- **验证**：add 立即通知与逆序派发、remove 生效、refresh 时接触增加/破坏 body 后接触移除、integrate 一次 step 回调、phantom 创建/跟随/销毁；`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿。

### 后续子集：Ragdoll 扩展余项（✅ 已完成，2026-11-21）
交付（`phys-world` 新增 `LimitedHingeCinfo`/`SixDofCinfo`/`SixDofLimitsCinfo` 与 2 个关节族，`phys-solver` 新增 `SixDofJoint`，`phys-extensions` easing/控制器泛化；新增 4 个集成测试文件共 16 个测试）：

| 模块 | 内容 |
|---|---|
| `phys-world/constraint.rs` + `world.rs` | `LimitedHingeCinfo`（复用 hinge 几何与角度限位、无电机）与 `SixDofCinfo`/`SixDofLimitsCinfo`（twist ±、swing0/1、线性距离上限）；`create_limited_hinge_constraint` / `create_6dof_constraint` / `set_limited_hinge_limits` / `set_6dof_limits`；`WorldJoint`/`JointType` 各新增两个变体，按槽序构建 solver 行 |
| `phys-solver/constraint.rs` | `SixDofJoint` 四行（距离单边、twist、swing0、swing1），固定行序 warm-start 回放；limited-hinge 复用 `HingeJoint` 行且忽略电机；`MAX_JOINT_ROWS = 8` 不变 |
| `phys-extensions/ragdoll_ease.rs` | `LimitSnapshot` 枚举（Ragdoll/LimitedHinge/SixDof）泛化 `EaseConstraintsAction`；`new` 保持 id 输入序并跳过不支持的族；`loosen` 按族放宽，回弹仍用 SDK 权重 `dt/(duration−elapsed)`；导出 `LimitedHingeLimitSnapshot` / `SixDofLimitSnapshot` |
| `phys-extensions/ragdoll_controller.rs` | `RagdollState::Custom(u16)` + `USER_STATE_COUNT = 10`、`RagdollStateBehavior` trait（initialize/set_strength/enter/leave/step/easing/clone_state）与 `RagdollStateContext`；`register_state` / `unregister_state` / `has_custom_state` / `queue_state_change`；自定义状态可返回后继状态、声明 easing 模式；控制器 Clone 通过显式 `clone_state` 深拷贝 |
| `phys-world/world.rs` | 共享运动：`create_attached_body` / `attach_bodies` / `detach_bodies` / `bodies_sharing_motion` / `body_motion_id`；`MotionGroup` 成员表按插入序、`BodyId` 索引（无 `MotionId` serial 风险）；`set_body_transform` 同步全体成员、`set_body_motion_type` 按组切换、`destroy_body` 最后一个成员保留 motion、step 对共享 motion 只积分一次 |

决策与偏差：

- SDK 2021.2 的 `hknpRagdollState` **没有**虚拟 `getNextState` / `createAction`；状态切换由 `queueStateChange` 完成，easing action 由状态管理器在 `setState` 中创建/销毁。Rust 侧照此设计：自定义状态只在 `step` 返回后继状态并声明 easing 模式，不提供 action 工厂钩子。
- SDK `hknpEaseConstraintsAction` 构造只接受 ragdoll / limited-hinge，`hknpConstraintDataUtils::loosenConstraintLimits` 仅放宽 ragdoll / limited-hinge；6DOF 有 save/restore/partial-restore 代码但无放宽实现。Rust 侧 6DOF 采用显式保守放宽（按当前 twist/swing/距离扩大保存的限制），标记为子集而非逐式直译；plane-angle limits 与 per-axis 6DOF 平移限位由后续「Ragdoll 细节补齐」子集补上。
- 共享运动以 keyframed 刚体组为主要用例，目标必须非静态；`attach_bodies` 默认保留目标 motion 的质量属性，后续「Ragdoll 细节补齐」新增 `KEEP/UPDATE/REBUILD` 显式模式；detach 用 `v + ω × r` 近似把速度平移到新质心；组缩减到 1 人即解散（剩余 body 持有原 motion slot）。
- `LimitedHingeCinfo` 是 `HingeCinfo` 的类型别名，limited-hinge 关节在 solver 层复用 hinge 行集但忽略电机，以此保持确定性与最小改动。
- 验证：新增 `ragdoll_joint_families.rs`（3）/`ragdoll_easing_extended.rs`（3）/`ragdoll_custom_states.rs`（5）/`ragdoll_motion_sharing.rs`（5）共 16 个集成测试，含四类场景 `hvk-trace v1` 逐位一致；全 workspace `cargo test` 通过。

### 后续子集：Ragdoll 细节补齐（✅ 已完成，2026-11-21）
交付（新增集成测试 `tests/ragdoll_details.rs` 6 个）：

| 模块 | 内容 |
|---|---|
| `phys-world/constraint.rs`、`world.rs` | `RagdollCinfo::with_plane_limits` + `World::set_ragdoll_plane_limits`（SDK 平面角 = 与 90° 的有符号偏差，默认关闭）；`SixDofAxisMode::{Unlimited,Limited,Fixed}` 与 `SixDofLimitsCinfo::with_linear_axis`，保留原 `linear_limit` 距离行；`set_6dof_limits` 保持新字段；`MotionMassUpdateMode::{Keep,Update,Rebuild}` 与 `attach_bodies_with_mode`/`detach_bodies_with_mode`、`set_body_inverse_inertia` |
| `phys-solver/constraint.rs` | ragdoll 新增 plane 上下两行（rows 5/6），约束 A 的 twist 轴与 B 的 plane 轴夹角，行方向按 `φ−π/2` 与 SDK 用户角互补映射；6DOF 新增最多三轴平移单边行（rows 4..6），`FIXED` 用零宽区间；两族 warm-start 按越界行回放，行序固定、`MAX_JOINT_ROWS = 8` 不变 |
| `phys-extensions/ragdoll.rs`、`ragdoll_ease.rs` | `RagdollLimits::with_plane_limits` 从骨骼层透传；`LimitSnapshot` 的 Ragdoll/SixDof 变体保存新字段，`loosen` 按当前姿态扩张区间（`FIXED` 放宽为窄 `Limited`），`restore` 回弹所有字段 |
| `phys-body/body.rs` | `Body` 缓存 `mass`/`inverse_inertia` 作为共享运动质量合并的逐体数据源，创建时从 `BodyCinfo`/motion 复制 |

决策与偏差：

- SDK 的平面角 API 是 `setPlaneMinAngularLimit` / `setPlaneMaxAngularLimit`，atom 内部用 `π/2 − angle` 互补存储，默认 `π/2 ± 0.025`（即用户角 `±0.025`）；Rust 侧保存用户角并在 solver 行里转成 `φ−π/2`，上/下限符号互换。新字段默认 `None`（关闭），避免改变既有 ragdoll 链的确定性基线。
- `AxisMode` 按 SDK 精确命名为 `Unlimited/Limited/Fixed`（固定轴用 `[0,0]` 零宽区间表达），三轴默认 `Unlimited`；SDK 的 `setLinearLimit` 是独立 stiff-spring 球半径距离，与三轴限制并存，Rust 继续保留旧 `linear_limit` 距离行。
- 质量模式按 SDK `UpdateMotionMode` 命名为 `Keep/Update/Rebuild`，默认 `Keep`；`Update` 在本子集复用确定性重建，不做增量负质量更新。该子集的惯量重建最初只保留世界对角项；后续「Ragdoll 深度补齐」已升级为完整世界惯量张量并接入形状质量属性，但 SDK 的压缩惯量表示与信号驱动缓存仍未移植，故不宣称与原版复合质量逐位一致；keyframed 目标不重建。
- 验证：`tests/ragdoll_details.rs` 覆盖默认关闭/非法输入拒绝/越界回推/模式保持/质量与质心/放宽回弹/逐位确定性；后续深度补齐再由 `tests/mass_properties.rs` 覆盖质量派生、非对角惯量与完整张量合并，全 workspace `cargo test` 全绿。

### 后续子集：CharacterControl 剩余项（✅ 已完成，2026-11-22）

交付（新增集成测试 `character_post_solve.rs` 2 个、`character_interactions.rs` 2 个、`character_shape_key_triggers.rs` 2 个、`character_multithreading.rs` 1 个、`character_rigid_body.rs` 3 个）：

| 模块 | 内容 |
|---|---|
| `phys-world/post_solve.rs`、`world.rs` | 线程安全 `PostSolveSignal` + 可复制 `PostSolveEvent`（step/delta/contact/island 摘要），`step`/`step_with_threads` 积分完成后按订阅序派发；`subscribe_post_solve` / `unsubscribe_post_solve` 与 `mass_signal` 同为 `Arc<Mutex<_>>`，保持 `World: Send + Sync` |
| `phys-collide/query.rs`、`shape_cast.rs`、`phys-world/query.rs` | `RayHit` / `ShapeCastHit` / `RayCastHit` / `ShapeCastHitInfo` 附带确定性本地 `ShapeKey`：mesh 叶三角形、height field cell/子三角形、compound 子形状索引（含嵌套 key）；既有命中排序与容差不变 |
| `phys-extensions/character/listener.rs`、`proxy.rs` | 新增 `character_interaction` / `object_interaction` / `process_constraints` 默认空回调；`Contact` 带 `shape_key`；对象交互结果可被逆序 listener 修改（`CharacterObjectInteractionResult`）；standalone proxy 保持 `integrate` 末尾回调，管理器 proxy 改由 post-solve 派发，避免双发；`apply_deferred_updates` 串行提交冲量与 phantom 同步 |
| `phys-extensions/character/manager.rs` | `CharacterProxyManager` 注册 proxy/rigid-body、连接/断开世界 post-solve、按 id 派发回调与 character↔character 交互；批量步进先用 `parallel_for_chunks` 并行算交互候选，再按 id 串行 integrate 与提交冲量，1/4 线程逐位一致 |
| `phys-extensions/character/triggers.rs` | `TriggerTracker::update_observations` 按 `(BodyId, ShapeKey)` 差集产出 `Entered` / `Updated` / `Exited`；旧的 body 级 AABB `update` 保持兼容；`Updated` 为显式观测状态，不默认打开 |
| `phys-extensions/character/rigid_body.rs`、`rigid_body_listener.rs` | `CharacterRigidBodyCinfo` 默认质量 75（SDK 构造函数值）、maxForce/maxSlope/supportDistance；真实 dynamic body + `check_support` / `step` / `set_linear_velocity`；listener 三入口 `filter_contact_point` / `on_post_collide` / `on_post_solve` |

决策与偏差：

- SDK 的 `characterCallback` 由 proxy 自己订阅 world `m_postSolve`（不是 `ProxyManager`），listener 逆注册序派发；Rust 的世界信号按订阅序派发，manager 内部对 proxy 按 id 升序派发，standalone proxy 仍在 `integrate` 末尾派发，两条路径互斥。
- 原版多线程按流形接触数快排 + 原子抢任务，`applyDeferredUpdates` 按排序后索引串行提交；Rust 用确定性分块并行计算交互候选，回调和冲量提交始终按 id 串行，跨线程数逐位一致。
- 查询 `ShapeKey` 是可诊断的迁移编码（mesh/height field/compound 叶索引组合），不宣称与 SDK 二进制 `hknpShapeKey` 逐位一致；trigger 的 `STATUS_UPDATED` 仅在观测位置/法线变化时产生，原版 world 级 merge dispatcher 的 UPDATED（默认关闭）未移植。
- rigid-body 路线没有 `hknpSolverData` / Jacobian 改写、material trigger modifier、VDB monitor 与 world shift；listener 收到可复制的局部接触摘要与状态快照，`RayCasting` 路线仍未移植。
- `CharacterProxyListener` / `CharacterRigidBodyListener` 增加 `Send` 约束以满足世界信号的线程安全订阅；现有测试监听器改为 `Arc<Mutex<_>>` 日志。
- 验证：`cargo fmt --all -- --check`、`cargo clippy --workspace --all-targets -- -D warnings`、`cargo test --workspace`、严格 `cargo doc`、`cargo build --workspace --release`、`cargo bench -p proton-engine --bench simulation -- --quick` 全绿。

### 后续子集：CharacterControl 补齐（✅ 已完成，2026-11-22）

交付（`phys-world` 单元测试增至 88 个；新增集成测试 `character_ray_casting.rs` 5 个、`character_contact_editing.rs` 3 个、`character_trigger_events.rs` 2 个）：

| 模块 | 内容 |
|---|---|
| `phys-world/events.rs`、`signals.rs` | `TriggerEvent`/`TriggerStatus`/`TriggerEventMerger`：按 `(body_a, body_b, shape_key_a, shape_key_b)` 累加 enter(+1)/exit(-1)，单步末正数→`Entered`、负数→`Exited`、零且开关打开→`Updated`，否则丢弃；`BTreeMap` 保证派发顺序确定。`EventSignal<E>` 统一 trigger/post-collide 订阅（订阅序、firing 中退订延迟、快照后锁外回调，`World: Send + Sync`） |
| `phys-world/contact_modifier.rs` | `ContactModifier` trait + `EditableContact`/`ContactEditAction::{Keep,Drop}` + 有序注册表；在 `World::build_contact_rows` 中应用，`Drop` 同步剔除 `constraints`/`contact_keys`/`contact_impulses` 三个平行数组，避免 warm-start 错位；`step` 与 `step_with_threads` 共用同一建行路径 |
| `phys-world/cinfo.rs`、`world.rs` | `WorldCinfo::raise_trigger_updated_events`（默认 false）+ builder；`World::set_body_trigger`/`is_body_trigger`/`record_trigger_transition`、`subscribe_trigger_events`；`subscribe_post_collide`/`register_contact_modifier`；AABB 宽相 trigger 对检测在 collide 后、建约束前派发，post-collide 快照在 island 构建前派发，`destroy_body` 清理 trigger 状态 |
| `phys-extensions/character/ray_casting.rs` | `hknpRayCastingCharacterControllerUtil` 子集：`CharacterParam`/`RayCastingCharacterCinfo`/`get_character_params`——`up = normalize(-gravity)`、绕 up 每 45° 生成 8 方向、`dot(input,dir) >= -0.1` 筛选（无输入退化为中心一条）、`raycast_filtered` + 无支撑 `cast_shape`、`maxStepHeight + bodyHeight*0.5 - distance` 差值、0.01 容差下 min/next-to-min/max 与 ups/downs、水平 bump cast 与 SDK 默认缓动 0.15/0.25 |
| `phys-extensions/character/contact_editing.rs`、`manager.rs` | `CharacterContactBridge` 同时作为 `ContactModifier` 和 post-collide 派发器：按 body id 找共享 listener 槽，把 solver 接触映射成 `ContactPointInfo`，`filter_contact_point` 的 false→`Drop`，允许改写法线/穿透；`CharacterProxyManager::attach`/`detach` 注册/注销接触编辑与信号，`add_rigid_body`/`remove_rigid_body` 管理桥接条目，`contact_connection_count()` 暴露生命周期 |
| `phys-extensions/character/triggers.rs`、`phys-extensions/src/lib.rs` | 明确 query 级 `Updated`（位置/法线变化）与世界级 `WorldTriggerStatus::Updated`（同帧 enter+exit 合并）语义互斥、不双发；世界事件以 `WorldTrigger*` 别名重导出，避免与角色 `TriggerEvent`/`TriggerStatus` 冲突 |

决策与偏差：

- 原版 2021.2 **没有** `hknpCharacterRaycastRigidBody`（全库搜索无命中），只有 `hknpRayCastingCharacterControllerUtil`；本阶段按工具算法移植为 Rust 侧查询函数，不虚构可直接直译的刚体类。凸体 raycast 的法线来自 support 映射（近似），max-slope 统计对 mesh 叶三角形使用精确面法线；测试据此覆盖。
- 世界 trigger 用确定性 AABB 宽相重叠而非窄相流形：静态/睡眠/非凸体也能参与，携带 `ShapeKey::INVALID`；叶级 key 继续由 `TriggerTracker` 查询观察路径提供，两条流互不派发。`record_trigger_transition` 显式合并同一对 body 的单步 enter+exit，用于 `Updated` 语义。
- 接触编辑点选在世界建约束处（`Solver::solve*` 接触切片不可变）：不暴露 `hknpSolverData`/Jacobian，也不做 VDB monitor；摩擦默认 0.5 可被回调覆盖。`Drop` 后 warm-start 缓存按实际求解行重建，避免陈旧冲量。
- `CharacterRigidBody` 的 listener 改存 `Arc<Mutex<Box<dyn Listener>>>` 共享槽：既有 `Box<dyn Listener>` 设置 API 不变，管理器可复用同一监听器接入世界钩子；query 级 post-collide 与世界级 post-collide 是两个独立来源，均按设计派发。
- 验证：raycast 平地/台阶/bump/陡坡/确定性，接触编辑 Drop/摩擦改写/桥接生命周期/1 与 4 线程 `hvk-trace v1` 逐位一致，世界 trigger Entered/Exited/Updated 与 query trigger 独立；`fmt` / `clippy -D warnings` / `test` / 严格 `doc` / release build / `bench --quick` 全绿。

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
| CharacterControl Climbing/trigger | 原版无 ladder 查询工具，`m_atLadder` 与梯面 normal 由应用提供；Climbing 在梯面坐标系用 `calculate_movement`，desired=(inputUD·10, inputLR·10, −0.1)、gain=1、maxDelta=100，跳跃沿梯面法线加 `|gravity|` 速度；trigger 用 AABB 重叠差分按 `BodyId` 产出 Entered/Exited（按 id 排序），不模拟 material trigger 与 shape key |
| CharacterControl listener/phantom | 本地 listener 按注册逆序派发，add 时立即 shape_changed；contact add/remove 用 public 调用前后 manifold 差分批量派发（removed→added），`character_callback` 在 `integrate` 末尾触发一次；phantom 由 `BodyCinfo` 创建为 keyframed + `DONT_COLLIDE`，经 `QueryFilter::ignore_body` 排除自身并在每次 integrate 后同步 transform |
| 世界 trigger 事件 | AABB 宽相重叠按 `(body,body)` 规范化，跨步差集产生 enter/exit；`TriggerEventMerger` 同键 +1/-1 累加，正/负/零分别映射 Entered/Exited/Updated（零默认丢弃）；`BTreeMap` + 订阅序保证跨线程确定性，叶级 key 由 query trigger 独立维护 |
| 接触编辑 | `ContactModifier` 按注册序在 `build_contact_rows` 改法线/位置/穿透/摩擦或 `Drop`；三平行数组（rows/keys/impulses）同步删行，warm-start 按实际行重建；`step`/`step_with_threads` 共用建行 helper，`Solver::solve*` 签名不变 |
| RayCasting 角色工具 | 无 `hknpCharacterRaycastRigidBody` 类，只有 8 方向 + 中心向下的 `getCharacterParams` 工具；`raycast_filtered` 排除自身，`cast_shape` 只在 `minDistance > maxStepHeight` 时兜底；`min/next-to-min/max` 与 ups/downs 计数照抄 SDK（含 0.01 容差与 `HK_FLOAT_MIN` 初始化），输出 displacement/bump/easing/support/climb |
| Ragdoll 调度层 | 骨骼参考姿态（父在子前）在建链时推导 body 局部 pivot/轴，复用 `RagdollCinfo` cone/twist 关节并屏蔽相邻体碰撞；`calcDesiredPoseWorldSpace` 逐骨父子合成，`driveToPose` 用 KeyFrameUtil 风格速度按 strength 插值写回，`applyHardKeyframe` 精确设姿并写入所需线/角速度（>20° 用轴角、小角用虚部近似）；显式调用代替 action/state machine |
| Ragdoll 状态机 + 电机 blender | Keyframed/Dynamic/Powered 按 bone strength（tag mask 子集）切换 body 运动类型，`keyframeRoot` 用单骨硬关键帧锁根；Powered 因 `RagdollJoint` 无位置电机而走 `drive_to_pose` 速度驱动，`maxForce`/`tau`/`damping` 仍按原版 `lerp(lower, upper, sqrt(strength))` 计算并暴露；timed transition 先按 `elapsed/duration` 淡出再累加时间，终态在 `elapsed+dt>=duration` 时切入 |
| Ragdoll penetration utility | 根用 closest-point 深度阈值（负 tolerance）判定嵌入、回退射线找背面；父→子骨骼段 ray 命中记在父节点并级联 ANCESTOR；只给 RAY 且非 ANCESTOR 的顶层节点施加 `maxResolutionSpeed - 相对点速度·法线` 修正冲量（含重力抵消），分层场景下对被穿透子树置 `DONT_COLLIDE`；状态步前检测，快照无穿透即视为已解决 |
| Ragdoll 约束松弛 | 状态按 `Enforce` / `LoosenNoRestore` / `LoosenThenRestore` 管理 action（切换时按**目标态**决定是否立即还原）；放宽量 = 当前相对 twist/cone + 0.01，回弹权重 `dt/(duration−elapsed)` 保证任意 dt 序列下线性收敛；`LimitSnapshot` 已覆盖 ragdoll、limited-hinge、6DOF 三族，其中 6DOF 放宽为 Rust 侧保守实现 |
| 测试/黄金基准 | 41 个原版 UnitTest：6 Ported + 3 Partial + 32 Blocked（依赖未迁移系统）；Phase 9 新增 19 个集成测试，后续子集持续扩充；`World::snapshot` 输出 `hvk-trace v1` 位级文本；C++ 黄金基准因 SDK 无生成构建链暂阻塞，不宣称逐位对齐 |
| 多线程/序列化 | 窄相按确定性 chunk 并行，结果按 pair 序回排，接触顺序与单线程一致；HFF/Tagfile 解析容器，Phase 16 重建 generic `Value` 树、Phase 17 由门面 `convert` 按 type name 分发为 shape/body（不做反向依赖）；`TCM0` compendium 用 `TCID` 签名校验 `TCRF` 引用后合并类型表，bundle 本地 `TYPE` 最后覆盖；2014 legacy tagfile 为独立变长 tag 流（符号-幅度整数 + 字符串表 + 元数据 + 对象位图 + 前向引用），不套 HFF 段树；WorldHash 按 snapshot 的 f32 位值做 FNV-1a |
| hkx 格式/版本 patching | 格式嗅探覆盖 2014 packfile 16 字节头、TAG0/TCM0/TAGF/TCMP、NT6、2014 tagfile 双端序 magic 与 XML；2021.2 的 patch 是编译期 `PatchInfo` 常量而非文件数据，Rust 侧用 `PatchSet` 按 `TypeMeta.name`/`version` 选择并递归改写 `Value` 树（rename/remove/add/default-set），SDKV 字符串保留在 `Bundle.sdk_version`；真 packfile 转换仍需外部 DLL |
| hkx packfile 适配与扩展 patching | `PackfileHeader` 校验 16 字节头并按可选 `PackfilePolicy` 检查 user tag/version，`PackfileConverter` 只负责把 packfile 转成现有 TAG0/2014 tagfile reader 可消费的字节；`load_hkx` 统一分流 TAG0/TCM0/legacy/packfile 并在加载后应用补丁；`PatchComponent` 增加 data-object/field 函数回调与 type-change 转换，函数指针 + 显式 helper 保证确定性；`apply_chain` 推进类型元数据并报告重复、循环、超长链 |
| limited-hinge / 6DOF 关节 | limited-hinge 复用 hinge 行集、忽略电机；6DOF 为距离单边 + twist + swing0/1 + 三轴平移的固定行序，warm-start 按行回放；constraint atom 仍未移植，但 plane-angle limits 与 per-axis 平移限位已按 SDK 约束框架轴实现 |
| hkx 之外的新 easing 族 | `LimitSnapshot` 按关节族分派，Ragdoll/LimitedHinge 按当前姿态放宽、6DOF 显式保守放宽（SDK 此版本无 6DOF loosen 实现）；回弹权重 `dt/(duration−elapsed)` 保持跨族一致 |
| 用户自定义 ragdoll 状态 | 10 个固定 user-state 槽按注册序存储，trait 对象显式 `clone_state` 支持控制器 Clone；SDK 无虚拟 getNextState/createAction，后继状态通过 `queue_state_change` 在 step 末切换，easing 模式由状态自身声明 |
| 共享运动 | 成员表按 `BodyId` 与插入序维护（规避 `MotionId` 无 serial 的陈旧风险）；`set_body_transform` 同步整组、共享 motion 每步只积分一次、组解散时最后一个成员保留 motion slot；质量属性在 `Rebuild`/`Update` 时按成员姿态完整合并 |
| Ragdoll plane-angle limits | SDK 用户角与 atom 角互补（`π/2−angle`），Rust 保存用户角、solver 用 `φ−π/2` 与 `twist_a×plane_b` 轴做两条单边行；默认关闭，不破坏既有 ragdoll 链基线 |
| 6DOF per-axis 平移 | `Unlimited/Limited/Fixed` 三模式按 SDK 约束框架列 `[twist, plane, twist×plane]` 追加单边行，`Fixed` 用零宽区间；旧球面距离行保留兼容；世界构建、solver、warm start 与 easing 共用同一组轴 |
| 形状质量属性 | `MassConfig` 按 SDK `m_massOrNegativeDensity` 区分显式质量与密度；球/盒/胶囊/圆柱/薄三角/凸包有解析惯量，复合体按插入序聚合，mesh/height field 按质量等级使用三角形聚合或 AABB 近似；`BodyCinfo.mass < 0` 时按形状派生质量、COM 与惯量 |
| 完整惯量张量与求解 | `Body`/`Motion`/`SolverBody` 保存世界空间逆惯量 `Matrix3`，旧 `Vector4` 作为对角兼容视图；接触和关节冲量走矩阵路径，非对角项为零时保留旧逐分量顺序；共享运动重建按成员姿态与完整张量应用平行轴定理，再求逆写回 |
| 共享运动质量合并 | `Keep/Update/Rebuild` 默认 Keep；重建按成员质量加权 COM、完整世界惯量 + 平行轴定理，不再丢弃非对角项；逐体质量属性/惯量缓存于 `Body`，`Update` 在本子集复用重建 |
| 压缩惯量 | `CompressedMassProperties` 用共享指数 15 位向量打包 COM/对角惯量，四元数各分量用 15 位定点；固定对角化与半球规范化保证确定性，但不宣称与 SDK 浮点位打包逐位一致；`MassPropertiesTree` 对每 4 个子形状压缩聚合 |
| 质量分布 | `MassDistribution` 按 `hknpMassDistribution` 保存 COM+体积、主轴四元数与单位质量对角惯量；零惯量分量表示对应轴无限惯量；固定轮数 Jacobi 对角化并排序特征值，主轴符号按最大分量规范化 |
| 质量变更信号与缓存 | `ShapeMassCache` 按形状身份与 `MassConfig` 插入序缓存，`Body` 记录质量来源和版本号；`World` 的形状/质量配置/逆惯量变更在重建后派发线程安全 `MassChangeEvent`，订阅序、退订与嵌套 emit 语义与 `base-types::Signal` 对齐 |
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
- [x] Phase 16 后续子集（hkx 类型 compendium，2026-11-18）：`TCM0` / `TCID` 签名解析、`TCRF` 引用校验与类型表合并；`base-serialize` 18 → 27 个单元测试，门面集成测试 2 → 3 个
- [x] Phase 16 后续子集（2014 legacy binary tagfile，2026-11-18）：magic/变长整数/字符串表/元数据/对象树与前向引用解析；`base-serialize` 27 → 31 个单元测试，门面集成测试 3 → 4 个
- [x] 后续子集（Ragdoll 调度层，2026-11-20）：骨骼层级绑定 + `RagdollCinfo` 链 + `drive_to_pose`/`apply_hard_keyframe`；`phys-extensions` 46 → 49 个单元测试，新增集成测试 `ragdoll_scheduling.rs` 4 个
- [x] 后续子集（Ragdoll 状态机 + powered 电机 blender，2026-11-20）：Keyframed/Dynamic/Powered 状态与 tagged body 运动类型切换、`keyframeRoot` 单骨 keyframe、timed transition 淡出、sqrt 混合电机参数；`World::set_body_motion_type`；`phys-world` 78 → 79、`phys-extensions` 49 → 53 个单元测试，新增集成测试 `ragdoll_state_machine.rs` 4 个
- [x] 后续子集（Ragdoll penetration utility，2026-11-20）：根嵌入检测/回退射线、父→子骨骼段射线与祖先状态级联、`DONT_COLLIDE` 掩码、目标分离速度冲量 + 重力抵消、控制器状态步前修正；`phys-extensions` 53 → 56 个单元测试，新增集成测试 `ragdoll_penetration.rs` 4 个
- [x] 后续子集（Ragdoll 约束松弛 + at-rest 自动转换，2026-11-20）：`EaseConstraintsAction` 保存/放宽/线性回弹 cone/twist；Dynamic 状态 4s 回弹、进入 enforcing 态立即还原；`begin_at_rest_transition` 按速度阈值切换终态；`phys-extensions` 56 → 58 个单元测试，新增集成测试 `ragdoll_easing.rs` 3 个
- [x] 后续子集（hkx 格式嗅探 + 版本 patching，2026-11-20）：`detect_format`（packfile/TAG0/TCM0/NT6/2014 tagfile/XML）、`PatchInfo`/`PatchSet` 结构型 patch（rename/remove/add/default-set）递归应用于 `Value` 树、`Bundle.sdk_version` 保留 SDKV；`base-serialize` 31 → 34 个单元测试，新增集成测试 `hkx_version_patching.rs` 3 个
- [x] 后续子集（CharacterControl Climbing + trigger 追踪，2026-11-20）：`at_ladder` 输入、默认注册 Climbing 状态与梯面移动/跳跃公式、AABB 差分 trigger 进入/退出事件；`phys-extensions` 58 → 59 个单元测试，新增集成测试 `character_climbing.rs` 4 个
- [x] 后续子集（CharacterControl 本地 listener + phantom presence，2026-11-20）：`CharacterProxyListener` 五类本地回调、按 id 注册/逆序派发、manifold 差分 contact 事件、`character_callback`、phantom body 创建/同步/销毁；新增集成测试 `character_listeners.rs` 4 个
- [x] 后续子集（Ragdoll 扩展余项，2026-11-21）：limited-hinge/6DOF 关节族（世界槽 API + solver 行 + warm start）、easing 泛化至三族、用户自定义状态（10 槽 trait 对象注册 + queued transition + easing 模式）、共享运动（attached body/attach/detach/transform 同步/生命周期）；新增集成测试 `ragdoll_joint_families.rs` 3 个、`ragdoll_easing_extended.rs` 3 个、`ragdoll_custom_states.rs` 5 个、`ragdoll_motion_sharing.rs` 5 个
- [x] 后续子集（hkx packfile 适配 + 函数/type-change patching，2026-11-21）：`PackfileHeader`/`PackfilePolicy`/`PackfileConverter` 头部校验与外部转换边界，`load_hkx` 统一分流 TAG0/TCM0/legacy/packfile，`PatchComponent` 函数型与 type-change 组件、`PatchSet::apply_chain` 版本链/冲突/循环检测；`base-serialize` 34 → 40 个单元测试，新增集成测试 `hkx_packfile_and_function_patches.rs` 7 个
- [x] 后续子集（Ragdoll 细节补齐，2026-11-21）：plane-angle limits（SDK 互补角映射 + 两条单边行）、6DOF `Unlimited/Limited/Fixed` 三轴平移限位、`MotionMassUpdateMode` 共享运动质量/COM/对角惯量重建；easing 同步保存放宽回弹；新增集成测试 `ragdoll_details.rs` 6 个
- [x] 后续子集（Ragdoll 深度补齐，2026-11-21）：`phys-shape/mass.rs` 新增 `MassConfig`/`MassProperties`/`MassQuality` 与各形状解析/聚合质量属性；`Body`/`Motion`/`SolverBody` 增加世界逆惯量 `Matrix3` 并保留 `Vector4` 对角兼容视图；接触/关节冲量与共享运动合并使用完整张量；6DOF 三轴平移改用 SDK 约束框架列 `[twist, plane, twist×plane]`，world/solver/warm-start/easing 共用；新增集成测试 `mass_properties.rs` 5 个，更新 `ragdoll_details.rs`/`ragdoll_easing_extended.rs` 断言；`fmt`/`clippy -D warnings`/`test`/`release build`/严格 `doc`/`bench --quick` 全绿
- [x] 后续子集（质量属性后续边界，2026-11-22）：新增可移植 15 位 `CompressedMassProperties`/`PackedVector3`/`PackedUnitVector4`、固定轮数 Jacobi 主轴对角化、`MassDistribution` 单位质量分布与 `MassPropertiesTree` 四子形状压缩聚合；`ShapeMassCache`、`Body` 质量来源/版本与 `World` 线程安全 `MassChangeEvent` 信号接入形状替换、质量配置、逆惯量设置与共享运动重建；新增集成测试 `mass_compression.rs` 3 个、`mass_distribution.rs` 2 个、`mass_property_signals.rs` 4 个，`mass_properties.rs` 增补缓存测试；`fmt`/`clippy -D warnings`/`test`/`release build`/严格 `doc`/`bench --quick` 全绿
- [x] 后续子集（CharacterControl 剩余项，2026-11-22）：`World` post-solve signal 与确定性派发、查询命中携带本地 `ShapeKey`、object/character 交互回调与可修改结果、`CharacterProxyManager` 多线程候选计算 + 串行回调/冲量提交、shape-key trigger（`Entered`/`Updated`/`Exited`）、`CharacterRigidBody` 与三入口 listener；新增集成测试 10 个，1/4 线程 `hvk-trace v1` 逐位一致
- [x] 后续子集（CharacterControl 补齐，2026-11-22）：`phys-world` 世界级 trigger 事件（enter/exit 合并、可选 UPDATED）与 `ContactModifier` 接触编辑（Drop 同步三平行数组）；`phys-extensions` `get_character_params` RayCasting 工具与 `CharacterContactBridge`（manager attach/detach 生命周期）；新增集成测试 `character_ray_casting.rs` 5 个、`character_contact_editing.rs` 3 个、`character_trigger_events.rs` 2 个，`phys-world` 单元测试增至 88 个；1/4 线程 `hvk-trace v1` 逐位一致
- [ ] 后续：packfile 正文仍需外部 `hkCompatFormats.dll` 或等效转换器（源码分发内无解析器），真实 `.hkx` 资产与 DLL 转换结果等价性未验证；VDB monitor、原始 `hknpSolverData`/Jacobian 直写与 `RayCasting` 独立刚体路线（SDK 无此类）未移植；SDK 浮点位压缩字节、`hknpMassPropertiesTree` 的 32 子字缓存、形状质量属性附加信号与完整 `hknpMassDistribution` 应用链仍未逐位对齐

仓库现状（2026-11-22）：

- 工具链：`rustc 1.101.0-nightly`；workspace `rust-version = "1.85"`、`edition = "2024"`
- 命名迁移（2026-11-16 完成）：门面 crate 迁至 workspace 根并改名 `proton-engine`（lib 名 `proton`）；14 个子 crate 目录与 `Cargo.toml`、代码引用由 `hk-*` / `hknp-*` 改为 `base-*` / `phys-*`（`hk_types → base_types` 等），C++ 符号名（`hknpWorld`、`hkVector4f`、`HK_ASSERT` 等）保持不变；clippy 与全量测试通过
- git：Phase 12/13/14/15 均已提交（`faf2ce5 CharacterControl`）；Phase 16 为 `37497d6`（hkx 对象加载子集）、Phase 17 为 `a8873e2`（generic Value 树桥接物理对象）、compendium 为 `f2ce9d8`、2014 legacy tagfile 为 `504dfff`、Ragdoll 调度层为 `728e672`、Ragdoll 状态机为 `f24ab05`、penetration utility 为 `d70f86d`、约束松弛/at-rest 为 `31eed77`、hkx 嗅探/patching 为 `1357eab`、Climbing/trigger 为 `381d0c2`、CharacterControl listener/phantom 为 `0cc9743`、Ragdoll 扩展余项为 `4914e50`；hkx packfile 适配 + 函数/type-change patching 为 `5478d25`、Ragdoll 细节补齐为 `74b7217`、Ragdoll 深度补齐为 `93fde26`、质量属性后续边界为 `742210e`、CharacterControl 剩余项为 `d114a1a`；本次 CharacterControl 补齐在工作区，未提交
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

