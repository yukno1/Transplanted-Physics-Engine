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

| crate | 职责 | 状态（2026-11-10） |
|---|---|---|
| `hk-types` | 基础类型：hkReal、句柄、RefPtr、位运算、错误/断言 | ✅ 已实现（Phase 1） |
| `hk-math` | 向量/矩阵/四元数/变换/Aabb（**自研直译 + SIMD**，不用 glam） | ✅ 已实现（标量后端，Phase 2；SIMD 待优化） |
| `hk-memory` | 内存系统：分配器接口、系统堆分配器、分配统计 | ✅ 已实现（Phase 3） |
| `hk-container` | hkArray / 块流 / 确定性 HashMap、HashSet | ✅ 已实现（Phase 3） |
| `hk-thread` | 线程池、任务图、自旋锁（**std::thread，零依赖**） | ✅ 已实现（Phase 4） |
| `hk-geometry` | GeometryUtilities：凸包、网格焊接 | 🚧 骨架 |
| `hk-serialize` | HFF 结构化流 + Tagfile 容器解析 | 🚧 部分实现（Phase 10） |
| `hknp-types` | hknpTypes：BodyId/BodyIndex、句柄、MotionType/ShapeType、标志位 | ✅ 已实现（Phase 5） |
| `hknp-shape` | 形状体系：统一 Shape trait + 凸体/复合/网格/高度场 | ✅ 已实现（Phase 5） |
| `hknp-collide` | 宽相/窄相/Manifold | ✅ 已实现（Phase 7） |
| `hknp-body` | hknpBody/BodyManager/Motion/MotionManager | ✅ 已实现（Phase 6） |
| `hknp-solver` | 迭代式接触求解器（SolverBody/ContactConstraint） | ✅ 已实现（Phase 8） |
| `hknp-world` | hknpWorld 组装 + collide→solve→integrate 单线程管线 | ✅ 已实现（Phase 8） |
| `hknp-extensions` | WorldHash（Ragdoll/Vehicle/Character 阻塞） | 🚧 部分实现（Phase 10） |
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

### Phase 4：`hk-thread` + 任务系统（✅ 已完成，2026-11-10）
交付位于 `crates/hk-thread/src/`（29 个单元测试 + 5 个文档测试）：

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

### Phase 5：`hknp-types` + `hknp-shape`（✅ 已完成，2026-11-10）
交付（`hknp-types` 16 个单元测试；`hknp-shape` 41 个单元测试）：

| crate | 内容 |
|---|---|
| `hknp-types` | `BodyId`（u64，MSB 区分内部索引/用户 ID）与 `BodyIndex`（24 位索引 + 8 位 serial）；`MotionId`/`MaterialId`/`ShapeKey`/`SolverId`/`ImmediateConstraintId`/`BodyQualityId`/`MotionPropertiesId`/`ConstraintSolverId` 等句柄（invalid 哨兵与 SDK 完全一致）；`MotionType`、完整 `ShapeType`、`CollisionDispatchType`；`CollisionFlags`/`ShapeFlags`/`WorldAddedState` |
| `hknp-shape` | 统一 `Shape` trait（`shape_type`/`dispatch_type`/`flags`/`aabb`/`bounding_radius`/`calc_aabb`/`volume`/`support`/`is_convex`/`as_any`）；`SphereShape`、`BoxShape`、`CapsuleShape`、`CylinderShape`、`TriangleShape`、`ConvexHullShape`；`CompoundShape`（`ShapeInstance` + `Arc<dyn Shape>`）；`MeshShape`（顶点+三角形+中值分割 BVH，叶子 4 个三角形）；`HeightFieldShape`（XZ 网格 + 高度采样） |

决策与偏差：
- **Box**：本阶段为“局部轴对齐半长 + convex radius”，OBB 变换由 body/实例携带（Phase 6）；**Cylinder** 的 AABB 使用有限圆柱精确支撑公式（`h*|axis_i| + r*sqrt(1-axis_i²)`），而非保守球扫掠
- **凸体 support 语义**：盒取方向符号角点 + `radius*dir`、胶囊取两端点中的极值点 + `radius*dir`（共线 tie 取 A/正向轴）、圆柱为轴向分量 + 径向投影；**体积按名义实体计算**（不含 convex radius），与 SDK 的“几何 + 圆角分离”一致
- `ConvexHullShape` 未实现点云凸包构建（`hknpConvexHullUtil`），仅支持外部给定顶点 + 可选三角面；Mesh 暂不含 section/material/compressed-mesh；Compound 的 BVH 留 Phase 7
- `HeightFieldShape` 简化为 XZ 网格 + `(x,y,z)` scale，未含每 cell BVH/SDF；`hknpShapeFunctions` 的查询/GSK/face 接口留 Phase 7
- 未纳入：`hknpCompressedMeshShape`、`hknpExternMeshShape`、`hknpBreakableCompoundShape`、LOD、decorator/masked 形状、shape tag codec、shape key mask/iterator、`hknpShapeMassProperties`
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 6：`hknp-body` + `hknp-world`（✅ 已完成，2026-11-10）
交付（`hknp-body` 18 个单元测试 + 1 个文档测试；`hknp-world` 10 个单元测试 + 1 个文档测试）：

| crate | 内容 |
|---|---|
| `hknp-body` | `Body`/`BodyCinfo`（shape、flags、material/quality、filter info、user data、位置/朝向/速度/质量）；`BodyManager` 索引池（24 位索引 + 8 位 serial，free-list 复用槽，复用后旧 `BodyId` 失效）；`Motion`（orientation、质心、线/角速度、mass/inverse inertia、积分与冲量）；`MotionManager`（slot 0 固定静态 motion `PRESET_STATIC`，动态槽复用） |
| `hknp-world` | `WorldCinfo` builder（gravity、body/motion 容量、collisionTolerance、solverTau/Damp/Iterations、deactivation、expectedDeltaTime）；`World` 组装 body/motion manager，`create_body`/`destroy_body`、transform/position/velocity/AABB 访问器、`set_gravity`、`stats`；`step()` 单线程运动学骨架：动态 motion 做平移 + 旋转积分并同步 body transform/AABB，静态/Keyframed 保持不动（Phase 8 已扩展为 gravity→collide→solve→integrate 完整管线） |

决策与偏差：
- `Body::transform` 暂用 f32 `hkTransform`；SDK 的 `hkPreciseTransform`（double 平移）留到 Phase 10 大世界/shift 工作
- Motion 角速度按**世界空间**暴露（SDK 内部存局部/惯量空间并在访问时转换）；积分采用 `q' = normalize(Δq * q)`，对世界空间输入等价
- `create_body` 把 SDK 的 `allocateBody` + `addBody` 折叠为一步；质量 `<0`（使用 shape 质量）暂按无限质量处理，形状质量属性随 Phase 8 求解器一起接入
- `destroy_body` 会释放非静态 body 的 motion；暂不支持 `allocateAttachedBody` 的 motion 共享；`MotionId` 本身无 serial（与 SDK 一致），跨销毁持有需先用 `BodyId` 校验
- `step()` 在 Phase 6 时仅做运动学积分；Phase 8 已接入重力、宽窄相碰撞与接触求解（见 Phase 8 段）
- 未纳入：broadphase/space splitter、body 添加/延迟队列、command queue 与确定性检查、deactivation manager、collision filter、事件/modifier、用户 body buffer、`hknpWorldReader`/`hknpWorldWriter` traits
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 7：`hknp-collide`（✅ 已完成，2026-11-10）
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

### Phase 8：`hknp-solver` + 完整步进（✅ 已完成，2026-11-10）
交付（`hknp-solver` 5 个单元测试；`hknp-world` 13 个单元测试 + 1 个文档测试）：

| crate | 内容 |
|---|---|
| `hknp-solver` | `SolverBody`（inverse mass/inertia、质心、线/角速度、点冲量）；`ContactConstraint`（body 索引、B→A 法线、世界接触点、穿透、摩擦）；`SolverSettings`（iterations/baumgarte/max_recovery_speed/friction）；`Solver` 顺序脉冲 Gauss-Seidel：法向累积冲量非负 clamp、Baumgarte 穿透恢复速度上限、单切向摩擦按 `μ * 累积法向冲量` clamp；`SolverStats` |
| `hknp-world` | `World::step` 完整管线：重力 → `collect_contacts`（网格宽相 + GJK/EPA 窄相）→ 构建 SolverBody/接触约束 → 求解 → 写回速度 → 半隐式欧拉积分并刷新 transform/AABB；`collect_contacts` 公开 API；`WorldStats` 增加 `last_manifold_count`/`last_contact_count`；静态-静态与 `DONT_COLLIDE` 过滤 |

决策与偏差：
- 接触仍是**单点**（Phase 7 限制）；材质库未接入，摩擦系数固定 0.5、restitution 0，`WorldCinfo.solverTau/solverDamp` 暂时只保留字段，稳定化使用固定 Baumgarte 0.2
- 未移植 `Solver/ConstraintAtom` 的约束原子/Jacobian 系统、warm starting、island 分区、deactivation；当前是“每对单点约束 + 4 次顺序脉冲”的简化求解器
- 质量 `< 0` 仍按无限质量处理（Phase 6 决策），动态 body 需要显式 `with_mass`；shape 质量属性/惯量张量尚未接入，因此球的 inverse inertia 默认为 0（只做平动响应）
- `step` 的宽相使用上一步缓存的 AABB，积分后刷新；确定性依赖 body slot 顺序、宽相 `(a,b)` 顺序与求解器插入顺序
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿

### Phase 9：测试与验证（✅ Rust 侧完成；C++ 黄金基准外部阻塞，2026-11-11）
交付：

- **测试矩阵**：[docs/phase9-test-matrix.md](docs/phase9-test-matrix.md)——41 个原版 UnitTest 的逐项映射（6 个 Ported、3 个 Partial、32 个 Blocked 并注明前置系统），以及 S1–S12 场景矩阵、基准矩阵与验收标准
- **新增集成测试 19 个**（`crates/havok/tests/`）：
  - `simulation_scenarios.rs`（10）：自由落体解析解、球/盒堆叠/胶囊/圆柱落地、摩擦滑动、流形约定、宽相确定性、DONT_COLLIDE、transform/orientation
  - `original_unit_tests.rs`（6）：`NpBodyAllocationTest`、`NpBoxShapeUnitTest`、`NpBoundingSphereTest`、`NpMeshShapeUnitTest`（结构）、`NpHeightFieldShapeUnitTest`（结构）、`NpManyBodiesTest`
  - `determinism.rs`（3）：同场景两次运行逐位一致（`f32::to_bits`）、`hvk-trace v1` 文本稳定、`NpWorldSnapshotTest` 部分映射
- **确定性快照**：`hknp-world` 新增 `World::snapshot()` / `WorldSnapshot` / `BodySnapshot` 与 `to_text()`，浮点用 `to_bits()` 输出，格式 `hvk-trace v1`
- **基准 harness**：`crates/havok/benches/simulation.rs`（`harness = false`，仅 `std::time`，无 criterion）；覆盖宽相、GJK 球/盒、窄相盒对、求解器、`world_fall_n`、`world_stack_10`
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；`cargo bench --quick` 输出全部指标

黄金基准状态：`../original` 是源码分发，没有生成好的 `.sln`/Physics 测试可执行文件，完整构建依赖原版 MSBuild 工具链，当前环境无法运行 C++ 场景。因此**不宣称 C++ 逐位对齐**；已实现 Rust 侧逐位确定性与 trace 格式，待原版 SDK 可构建后按矩阵 §5 执行对比。criterion 因 crates.io 不可达改用自研 harness。

### Phase 10：多线程 + 序列化 + 扩展（🚧 部分完成，2026-11-11）
已交付子集：

- **多线程窄相**（`hknp-world`）：新增 `World::collect_contacts_parallel(threads)` 与 `World::step_with_threads(dt, threads)`；使用 `hk_thread::parallel_for_chunks` 做确定性连续分块，chunk 结果按宽相 pair 索引回排，因此接触顺序与单线程完全一致。求解器与积分仍为串行；候选数 <32 时回退单线程（避免每步创建线程的开销）。新增测试：并行接触结果 == 单线程、双 world 120 步逐位一致（`hknp-world` 15 个单元测试）。
- **`hk-serialize` 容器层**：HFF 结构化流解析（branch/leaf、30 位 size、FourCC、错误）与 Tagfile 容器（`TAG0`/`TCM0` 根、`SDKV` 版本、`DATA`/`TYPE`/`INDX` 访问），10 个单元测试。仅容器解析，不做反射对象重建。
- **`hknp-extensions` WorldHash**：对 `WorldSnapshot` 做 FNV-1a 64 位确定性哈希（浮点按 `to_bits`），5 个单元测试。
- 基准新增 `world_fall_n_parallel` 场景（同一 step 管线，4 线程窄相）。
- **约束/joint 系统**（2026-11-12，Phase 10 后续子集）：
  - `hknp-types`：新增 `ConstraintIndex`（u32，invalid `HK_INT32_MAX`）与 `ConstraintId`（u64，MSB 区分内部索引/用户 ID，语义照抄 `hknpConstraintId.h`）。
  - `hknp-solver`：新增 `constraint.rs`——世界空间 solver 级 joint（`BallAndSocketJoint` / `HingeJoint` / `FixedJoint`），顺序脉冲求解：线性 3 轴拉拢 pivot（分离量 Baumgarte bias）、铰链轴对齐（垂直轴角速度 + 轴误差 bias）、固定关节锁相对角速度（世界 X/Y/Z 轴 + 朝向误差 bias）。joint 在 slice 顺序、固定轴序下求解，确定性。新增 14 个单元测试。
  - `hknp-world`：新增 `constraint.rs`——`BallAndSocketCinfo` / `HingeCinfo` / `FixedCinfo`（pivot/axis 存 body 局部空间；`FixedCinfo::between_bodies` 按当前位姿绑定）+ `WorldJoint` 槽表（**槽不复用**，ID 永不重指）与 `create_*_constraint` / `destroy_constraint` / `set_constraint_enabled` API；`step` 在接触求解后按槽序构建 solver joints 并求解；`destroy_body` 级联删除关联约束。新增 6 个约束单元测试。
  - 局限：速度级求解（无位置修正投影）、无 limits/motors/springs、无约束岛与 warm starting；球的 inverse inertia 默认 0，摆动类场景需手动设置 `motion.inverse_inertia`。
- **查询系统**（2026-11-12，Phase 10 后续子集）：
  - `hknp-collide`：新增 `query.rs`——凸体 raycast（**conservative advancement**：GJK 点-形状距离除以射线与法线夹角推进，sphere/box/capsule/cylinder/hull 单一实现路径；射线起点在内部按 fraction 0 命中）、`point_distance`（点-形状 GJK 距离，供最近点查询）、`ray_aabb` slab 粗测（固定 x/y/z 轴序）。新增 10 个单元测试。
  - `hknp-world`：新增 `query.rs`——`World::raycast`（最近命中，槽序遍历 + AABB 粗测 + 同 fraction 低 `BodyId` 优先，确定性）与 `World::closest_point`（凸体最近表面点/距离，内部点返回 distance 0）、`World::body_transform`。查询不受碰撞过滤影响（与 SDK 一致）；非凸形状（Mesh/Compound/HeightField）暂不参与。
  - 新增集成测试 `crates/havok/tests/constraints_and_queries.rs`（9 个）。
- **铰链 limits/motors**（2026-11-12，Phase 10 后续子集）：
  - `hknp-solver`：`HingeJoint` 新增 `limits: Option<HingeLimits>` 与 `motor: Option<HingeMotor>`——limits 为单边约束（仅当角度越界时施加回复脉冲，方向由 lambda 符号约束，Baumgarte bias 上限同接触求解），motor 为绕铰链轴的速度电机（**每次 solve 施加一次限幅冲量**，与迭代次数无关，确定性）。角度/速度约定为 **B 相对 A 绕铰链轴**（与角度测量 `sin = t_a × t_b · axis` 一致）。新增 8 个单元测试。
  - `hknp-world`：`HingeCinfo` 新增 `limits`/`motor` 字段与 `HingeLimitsCinfo`（twist 参考轴对 + min/max 角）/`HingeMotorCinfo`（目标速度 + 冲量上限）；每步由世界空间 twist 轴计算签名角 `hinge_angle`（投影到垂直于铰链轴的平面，atan2 取值 `(-pi, pi]`）传入 solver；新增 `set_hinge_limits`/`set_hinge_motor` 运行时 API（非铰链/无效 ID 返回 false）。新增 3 个单元测试；`constraints_and_queries.rs` 集成测试增至 12 个。
  - 局限：速度级 limit（越界小幅穿越后由 bias 拉回，无硬位置钳制）、仅速度电机（无位置/弹性电机、无 springs）、motors 不参与迭代内与 limits 联合收敛（预施加一次）。

未完成/阻塞项：

- **完整 `hknpMultithreadedSimulation`**：任务图/求解器/积分并行、work stealing、`hknpTaskGraph` 物理调度未移植；当前只并行窄相，且每步重建 scoped worker（小场景并行反而更慢，见 bench）。
- **`hkx` 对象加载**：TYPE/DATA 反射类型注册、对象 rebuild、patch/fixup、binary packfile、writer/XML/2014 兼容均未移植；`hk-serialize` 不能替代资产加载。
- **Ragdoll / Vehicle / CharacterControl**：前置的约束/joint 系统、射线查询与铰链 limits/motors 已交付（见上）；仍缺 springs/位置电机、shape cast（非 raycast）、碰撞过滤、action 调度、多体约束岛。均未移植。
- 大世界 `hkPreciseTransform`（double 平移）仍待（当前 f32 Transform）。
- 验证：`fmt` / `clippy -D warnings` / `test` / `doc -D warnings` / release build 全绿；`cargo bench --quick` 输出全部指标。

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
| 求解器/步进 | 顺序脉冲：法向累积 clamp≥0、Baumgarte 穿透恢复、单切向摩擦 clamp；step 单线程按 body 槽位/宽相顺序/约束插入顺序确定性执行；材质未接入时摩擦 0.5、restitution 0 |
| 测试/黄金基准 | 41 个原版 UnitTest：6 Ported + 3 Partial + 32 Blocked（依赖未迁移系统）；新增 19 个集成测试；`World::snapshot` 输出 `hvk-trace v1` 位级文本；C++ 黄金基准因 SDK 无生成构建链暂阻塞，不宣称逐位对齐 |
| 多线程/序列化 | 窄相按确定性 chunk 并行，结果按 pair 序回排，接触顺序与单线程一致；HFF/Tagfile 只解析容器，不重建对象；WorldHash 按 snapshot 的 f32 位值做 FNV-1a |
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

# Phase 9 benchmark（无外部依赖；--quick 为小规模冒烟）
cargo bench -p havok --bench simulation -- --quick

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
- [x] Phase 1：`hk-types` —— scalar / bitops / Pair / Handle / RefPtr / UFloat8 / Signal / HkResult 全部实现，49 个单元测试 + 13 个文档测试
- [x] Phase 2：`hk-math` —— Vector4/D、Quaternion、Matrix3、Transform、QTransform、QsTransform、Aabb、Half 完成（标量后端），52 个单元测试 + 5 个文档测试
- [x] Phase 3：`hk-memory` + `hk-container` —— 分配器/统计、Array、BlockStream、确定性 HashMap/HashSet 完成，8 + 27 个单元测试 + 5 个文档测试
- [x] Phase 4：`hk-thread` —— SpinLock、ThreadPool、JobQueue、TaskGraph、parallel_for 完成（std::thread，零依赖），29 个单元测试 + 5 个文档测试
- [x] Phase 5：`hknp-types` + `hknp-shape` —— 句柄/枚举/标志位、统一 Shape trait、6 种凸体、Compound、Mesh+BVH、HeightField 完成，16 + 41 个单元测试
- [x] Phase 6：`hknp-body` + `hknp-world` —— Body/BodyCinfo/BodyManager、Motion/MotionManager、WorldCinfo/World 与单线程运动学 step 骨架完成，18 + 10 个单元测试
- [x] Phase 7：`hknp-collide` —— 网格宽相、GJK 最近点/相交、EPA 穿透、单点凸-凸流形与 sphere-sphere 解析特化完成，22 个单元测试
- [x] Phase 8：`hknp-solver` + 完整步进 —— 顺序脉冲接触求解器与 gravity→collide→solve→integrate 管线完成，5 + 13 个单元测试
- [x] Phase 9：测试与验证（Rust 侧）—— 41 个原版 UnitTest 矩阵（6 Ported / 3 Partial / 32 Blocked）、19 个集成测试、确定性 trace、无依赖基准 harness 完成；C++ 黄金基准待原版 SDK 可构建
- [x] Phase 10（可交付子集）：并行窄相 + `step_with_threads`、HFF + Tagfile 容器解析、WorldHash 完成
- [x] Phase 10（后续子集，2026-11-12）：约束/joint 系统（BallSocket/Hinge/Fixed + 世界接入）、查询系统（world raycast / closest point）与铰链 limits/motors 完成
- [ ] 后续：约束岛与 warm starting、springs/位置电机、shape cast 与 AABB cast 查询、碰撞过滤 → Ragdoll/Vehicle/CharacterControl + `hkx` 对象加载（packfile 反射/rebuild）

仓库现状（2026-11-12）：

- 工具链：`rustc 1.101.0-nightly`；workspace `rust-version = "1.85"`、`edition = "2024"`
- git：最新提交 `927b8fd havok/tests & havok/benches & hknp-world`；Phase 10 及后续改动（`hk-serialize`/`hknp-extensions`/并行窄相/约束与查询/bench/README）尚未提交
- 外部依赖：0

