# Havok Physics 2021.2 → Rust 迁移项目

本项目旨在将 **Havok Physics SDK 2021.2**（BUILD #20211202，C++）迁移为 Rust 实现。

**目录边界**

- `havok/`（本仓库）：Rust 目标工程（workspace）。
- `../original/`（同级目录，**不纳入本仓库 / git**）：原始 C++ SDK 源码（约 5861 个文件）。

> **快速开始**：需要 Rust（edition 2024 / MSRV 1.85），在仓库根目录执行
> `cargo test --workspace`。完整命令与仓库约定见第 7 节。

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
havok/                             # 本仓库根目录
├── Cargo.toml                     # [workspace] members = ["crates/*"]
├── README.md
├── crates/
```

| crate | 职责 | 状态（2026-11-09） |
|---|---|---|
| `hk-types` | 基础类型：hkReal、句柄、RefPtr、位运算、错误/断言 | ✅ 已实现（Phase 1） |
| `hk-math` | 向量/矩阵/四元数/变换/Aabb（**自研直译 + SIMD**，不用 glam） | ✅ 已实现（标量后端，Phase 2；SIMD 待优化） |
| `hk-memory` | 内存系统：分配器接口、系统堆分配器、分配统计 | ✅ 已实现（Phase 3） |
| `hk-container` | hkArray / 块流 / 确定性 HashMap、HashSet | ✅ 已实现（Phase 3） |
| `hk-thread` | 线程池、任务图（可先用 rayon） | 🚧 骨架 |
| `hk-geometry` | GeometryUtilities：凸包、网格焊接 | 🚧 骨架 |
| `hk-serialize` | [P1] hkx tagfile / packfile | 🚧 骨架（P1 未启动） |
| `hknp-types` | hknpTypes：ID、MotionType、标志位 | 🚧 骨架 |
| `hknp-shape` | 形状体系（Sphere/Box/Capsule/Hull/Mesh/HeightField） | 🚧 骨架 |
| `hknp-collide` | 宽相/窄相/Manifold/查询 | 🚧 骨架 |
| `hknp-body` | hknpBody/BodyManager/Motion | 🚧 骨架 |
| `hknp-solver` | 约束原子/接触求解/积分器 | 🚧 骨架 |
| `hknp-world` | hknpWorld 组装 + 步进管线（单线程优先） | 🚧 骨架 |
| `hknp-extensions` | [P1] Ragdoll/Vehicle/Character | 🚧 骨架（P1 未启动） |
| `havok` | 门面 crate，重导出公共 API | 🚧 骨架（重导出已接好） |

依赖方向严格单向：`types → math → memory → container → ... → world → extensions`。

---

## 4. 分阶段迁移计划

### Phase 0：工程脚手架（✅ 已完成）
- 将 `havok/` 改造为虚拟 workspace（`[workspace]` + `crates/*`，共 15 个 crate）
- `.gitignore` 忽略 `/target`
- 检查命令：`cargo fmt`、`clippy -D warnings`、`test`、`doc -D warnings`、release build（CI 未配置，本地执行，见第 7 节）
- 基础依赖决策：数学库定为**从 C++ 直译**（不用 glam，保证 float32 确定性）；`rayon`/`thiserror`/`bytemuck` 按阶段引入

### Phase 1：`hk-types`（✅ 已完成，2026-11-09）
交付位于 `crates/hk-types/src/`（49 个单元测试 + 13 个文档测试）：

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

### Phase 2：`hk-math`（✅ 核心已完成，2026-11-09；标量后端）
交付位于 `crates/hk-math/src/`（52 个单元测试 + 5 个文档测试）：

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

### Phase 3：`hk-memory` + `hk-container`（✅ 已完成，2026-11-09）
交付（`hk-memory` 8 个单元测试；`hk-container` 27 个单元测试 + 5 个文档测试）：

| crate | 内容 |
|---|---|
| `hk-memory` | `Allocator` trait（`Layout` 风格 `alloc`/`alloc_zeroed`/`dealloc`/`realloc`）、`AllocStats`、`SystemAllocator`（转发全局分配器 + 原子统计/峰值）、`global()` 进程实例；失败即 panic，与 SDK 的 fail-fast 语义一致 |
| `hk-container` | `Array`（`hkArray`：`removeAt` 无序换尾、`removeAtAndCopy` 有序前移、`reserve`/`expandBy`/`insertAt`/`append`/`optimizeCapacity`/`indexOf`）、`BlockStream`（`hkBlockStream` 单线程子集：4000 字节定长块、确定性迭代、随机访问、`clear` 复用块、字节统计）、`HashMap`/`HashSet`（`std::collections` + 固定种子 FNV-1a；`insert`/`tryInsert`/`tryGet`/`getOrInsertDefault`/`getWithDefault`/`at`） |

决策与偏差：
- **保持零外部依赖**：`hashbrown` 暂缓，使用 `std::collections`；禁用 `RandomState`，改固定种子 FNV-1a，保证跨运行可复现。迭代顺序确定，但**不等同于** C++ `hkHashBase` 的数组序；依赖顺序的代码请用 `Array`
- `Array` 底层直接用 `Vec<T>`（容量倍增与 SDK 一致）；保留 `as_ptr`/`as_mut_ptr` 供 FFI 风格代码
- `BlockStream` 仅移植单线程 append/iterate/random-access 子集；reader/consumer/range、块头原子计数与线程局部块分配器留给 Phase 10
- 未纳入：`hkLocalArray`/`hkInplaceArray`、`hkArrayView`、`hkString*`、`hkPointerMap`/`hkRelArray`/`hkTree`、并发容器；内存路由、`hkMemoryDebug`、FreeList/Lifo/Thread allocator 按 README 策略待性能分析后再引入
- **核心原则**：指针访问全面改为 ID/索引访问，天然契合 Rust 借用检查

### Phase 4：`hk-thread` + 任务系统
- 先用 `rayon` 替代 hkJobQueue/hkTaskGraph 的单线程场景
- `hknpSimulation` 先实现单线程版本，多线程留到 Phase 10

### Phase 5：`hknp-types` + `hknp-shape`
- `hknpTypes.h`：ShapeKey / SolverId / ImmediateConstraintId 等 `HK_DECLARE_HANDLE` 句柄；Body/Motion/Material 等 Id 分布在模块头文件，`hknpMotionType`
- 形状体系（173 文件）：
  - Convex：Sphere → Box → Capsule → Cylinder → Triangle → ConvexHull
  - Composite：Mesh（含 BVH）→ HeightField → Compound
- 统一 trait 设计（对应 C 的 `hknpShapeFunctions` 虚表分发）

### Phase 6：`hknp-body` + `hknp-world`
- `hknpBody` 结构体、`hknpBodyManager`（索引池管理生命周期）
- `hknpWorldCinfo` → builder pattern
- `hknpWorld`：`setGravity`/`setCollisionFilter`/`step()` 骨架

### Phase 7：`hknp-collide`（技术核心）
- 宽相：网格宽相（CsGrid）
- 窄相：GJK 单纯形求解器、凸-凸 Manifold 生成、各形状对检测器
- 查询：RayCast / AabbQuery / ClosestPointsQuery / ShapeCast
- 碰撞过滤：`hknpCollisionFilter` trait + 默认组过滤

### Phase 8：`hknp-solver` + 完整步进（技术核心）
- 约束原子系统（`Solver/ConstraintAtom`，49 文件）
- 接触约束求解器（迭代式）
- 积分器（半隐式欧拉）
- 组装 `hknpWorld::step()`：Collide → Solve → Integrate，对齐 SimulationStage 状态机

### Phase 9：测试与验证（持续）
- 移植 `Physics/Test/UnitTest/`（41 个测试）
- 黄金基准：原 C++ 版跑场景记录每步 body transform，Rust 版逐位对比
- 性能 benchmark：`criterion`

### Phase 10：多线程 + 序列化 + 扩展（后续迭代）
- `hknpMultithreadedSimulation`：任务图并行化
- `hk-serialize`：hkx tagfile 解析
- `hknp-extensions`：Ragdoll → Vehicle → CharacterControl

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
| Signal 重入语义 | 触发中退订 / 嵌套 emit / 触发中新增订阅（延迟到下一轮）已由 `hk-types` 测试锁定，后续改动不得破坏 |

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

# 文档链接检查（可选）
$env:RUSTDOCFLAGS='-D warnings'; cargo doc --workspace --no-deps
```

CI：当前**未配置**（仓库内无 `.github/`），上述检查需在本地或外部流水线执行。

脚手架约定：
- 所有 crate 设为 `publish = false`（源码衍生自专有 Havok 代码，防止误发布）
- crate 间依赖严格单向，方向见第 3 节
- 当前零外部依赖，`rayon`/`thiserror`/`bytemuck` 按阶段引入
- `Cargo.lock` 应提交入库（应用型 workspace，锁定依赖；当前尚未跟踪）
- `../original/` 与 `target/` 不纳入仓库（后者已由 `.gitignore` 忽略）

## 8. 当前状态

- [x] 原项目结构分析
- [x] 迁移计划制定
- [x] Phase 0：工程脚手架 —— workspace 根 + 15 个 crate 骨架 + `.gitignore`（检查命令见第 7 节；CI 未配置）
- [x] Phase 1：`hk-types` —— scalar / bitops / Pair / Handle / RefPtr / UFloat8 / Signal / HkResult 全部实现，49 个单元测试 + 13 个文档测试
- [x] Phase 2：`hk-math` —— Vector4/D、Quaternion、Matrix3、Transform、QTransform、QsTransform、Aabb、Half 完成（标量后端），52 个单元测试 + 5 个文档测试
- [x] Phase 3：`hk-memory` + `hk-container` —— 分配器/统计、Array、BlockStream、确定性 HashMap/HashSet 完成，8 + 27 个单元测试 + 5 个文档测试
- [ ] Phase 4：`hk-thread` + 任务系统（下一步）
- [ ] 其余阶段待启动

仓库现状（2026-11-09）：

- 工具链：`rustc 1.101.0-nightly`；workspace `rust-version = "1.85"`、`edition = "2024"`
- git：最新提交 `1cba108 类型系统`；`hk-math` / `hk-memory` / `hk-container` 的新增源码尚未提交
- 外部依赖：0
