# Phase 9 测试矩阵与基准场景

> 状态：Rust 侧测试/基准实施计划（2026-11-11）。原版 C++ 黄金基准运行受外部构建条件阻塞，详见 §5。

## 1. 目标与原则

Phase 9 的目标不是“补几个测试”，而是给迁移建立可重复的验证层次：

- **L0 单元层**：`hk-types` / `hk-math` / `hk-memory` / `hk-container` / `hk-thread` 已有 165 个单元测试（另含文档测试）。
- **L1 形状层**：AABB、support、bounding radius、体积、Mesh BVH、HeightField 采样。
- **L2 生命周期层**：BodyId/BodyIndex 分配/销毁/槽复用、serial 失效、Motion 生命周期。
- **L3 碰撞层**：宽相确定性、GJK 距离/相交、EPA 穿透、流形约定。
- **L4 求解层**：法向冲量停止接近速度、摩擦削减切向速度、静态/动态响应。
- **L5 集成层**：`World::step` 的 gravity→collide→solve→integrate 闭环（自由落体、落地、堆叠、滑动、胶囊/圆柱）。
- **L6 确定性与基准**：同场景两次运行逐位一致；`hvk-trace v1` 可记录、可外部对齐；性能场景可复现。
- **L7 回归**：固定值断言（解析解/容差/不变量），避免“只跑不崩”。

所有新增测试必须满足：

1. 无外部依赖、无网络、无 C++ 运行时；
2. 失败信息包含实际值（便于定位 f32 误差）；
3. 不依赖 wall-clock；只有 benchmark 使用计时；
4. 浮点比较用解析容差或位相等（确定性用 `to_bits`），禁止 `approx` 依赖。

## 2. 原版 41 个 UnitTest 映射

原版位于 `original/Physics/Test/UnitTest/Physics/Physics/`。状态含义：

- **Ported**：Rust 侧有同名/同语义测试，且当前 API 能表达。
- **Partial**：覆盖原测试的结构性子集（查询/序列化等部分缺失）。
- **Blocked**：依赖尚未迁移的系统（原因列在最后一列）。

| # | 原版测试 | 覆盖点 | Rust 状态 | 说明/阻塞原因 |
|---|---|---|---|---|
| 1 | `NpBodyAllocationTest` | body 分配/销毁/索引复用 | Ported | `BodyManager` + serial；显式 desired `BodyIndex` 预留未实现 |
| 2 | `NpBodyToConstraintsMapTest` | body↔constraint 映射 | Blocked | 约束系统（ConstraintAtom）未移植 |
| 3 | `NpBoundingSphereTest` | 形状包围球 | Ported（凸体） | sphere/box/capsule/hull 的 `bounding_radius`；scaled/compound 依赖未移植 |
| 4 | `NpBoxShapeUnitTest` | box AABB/凸半径 | Ported（基础） | AABB/support/volume；`shrinkByRadius`/makeFromVertices 未实现 |
| 5 | `NpBreakableCompoundShapeTest` | 可破坏复合体 | Blocked | `hknpBreakableCompoundShape` 未移植 |
| 6 | `NpCalcAabbWithScaleTest` | 缩放形状 AABB | Blocked | `hknpScaledConvexShape` 未移植 |
| 7 | `NpCollisionCachePropertiesTest` | 碰撞缓存属性 | Blocked | collision cache 未移植 |
| 8 | `NpCompressedIntArrayTest` | 压缩整数数组 | Blocked | 相关容器未移植 |
| 9 | `NpConnectivityGraphTest` | 凸体连接图 | Blocked | connectivity graph 未移植 |
| 10 | `NpConstraintDataMemoryMapTest` | 约束内存布局 | Blocked | 约束系统未移植 |
| 11 | `NpConstraintDeepCopyTest` | 约束深拷贝 | Blocked | 约束系统未移植 |
| 12 | `NpConvexHullSimplificationLimitTest` | 凸包简化上限 | Blocked | 凸包构建器未移植 |
| 13 | `NpConvexHullUtilTest` | 点云→凸包 | Blocked | `hknpConvexHullUtil` 未移植 |
| 14 | `NpConvexShapeConstructionTest` | 凸形状构建/面信息 | Blocked | 凸包构建与 face connectivity 未移植 |
| 15 | `NpDeflectedSphereCastTest` | 偏转球投射 | Blocked | shape cast/查询未移植 |
| 16 | `NpEstimateNumShapeKeysTest` | shape key 数量估计 | Blocked | shape key 迭代器未移植 |
| 17 | `NpEventsAfterWorldCopyTest` | world 拷贝后事件 | Blocked | world copy/事件未移植 |
| 18 | `NpHeightFieldShapeUnitTest` | 高度场采样/AABB/单元 | Partial | 网格/scale/AABB 已覆盖；shape key、min/max 查询未移植 |
| 19 | `NpInnerSphereTest` | 内接球 | Blocked | `calcInnerSphere` 未移植 |
| 20 | `NpManyBodiesTest` | 大量 body 创建/销毁 | Ported（结构） | body 数量/生命周期；原测试的 capacity/碰撞统计语义有差异 |
| 21 | `NpMassPropertiesTreeTest` | 质量属性树 | Blocked | mass properties 未移植 |
| 22 | `NpMeshShapeCompressionTests` | 压缩网格 | Blocked | `hknpCompressedMeshShape` 未移植 |
| 23 | `NpMeshShapeInteriorPrimitivesTest` | 网格内部图元 | Blocked | 复合窄相/内部三角形未移植 |
| 24 | `NpMeshShapeUnitTest` | 网格 BVH/三角形 | Partial | 顶点/三角形/BVH 叶子覆盖已覆盖；leaf shape 查询未移植 |
| 25 | `NpMotionSpeedLimitTest` | 速度上限 | Blocked | motion speed limit 未移植 |
| 26 | `NpMutableConvexShapeTest` | 可变凸形状 | Blocked | mutability/signals 未移植 |
| 27 | `NpMutablePrimitiveConvexShapeTest` | 可变基础凸体 | Blocked | 同上 |
| 28 | `NpRayCastTest` | 射线查询 | Blocked | 查询系统未移植 |
| 29 | `NpRebuildConvexShapeMassPropertiesAccuracyTest` | 重建质量属性精度 | Blocked | mass properties 未移植 |
| 30 | `NpSerializeConvexTest` | 凸体序列化 | Blocked | hkx 序列化未移植 |
| 31 | `NpShapeCastTest` | 形状投射 | Blocked | 查询系统未移植 |
| 32 | `NpShapeManagerUnitTest` | shape manager | Blocked | shape manager 未移植 |
| 33 | `NpShapeMemoryMapTest` | shape 内存布局 | Blocked | in-place shape buffer 未移植 |
| 34 | `NpShapeSerializeTest` | shape 序列化 | Blocked | hkx 序列化未移植 |
| 35 | `NpSpatialQueryUnitTest` | 空间查询 | Blocked | 查询系统未移植 |
| 36 | `NpThreadSafeObjectPoolTest` | 线程安全对象池 | Blocked | object pool 未移植（BodyManager 是索引池，不是同一设施） |
| 37 | `NpThreadSafeQueueTest` | 线程安全队列 | Blocked | `hknpThreadSafeQueue` 未移植 |
| 38 | `NpUFMShapeTagCodecTest` | UFM shape tag codec | Blocked | shape tag codec 未移植 |
| 39 | `NpWorldInspectorTest` | world inspector | Blocked | inspector 未移植 |
| 40 | `NpWorldSnapshotTest` | world 快照/拷贝 | Partial | 已新增 `World::snapshot`/`hvk-trace`；完整 world copy 未移植 |
| 41 | `TimeStampHash` | 时间戳哈希（辅助） | Blocked | 原版 determinism util 未移植 |

**结论**：41 个中当前可落地 6 个 Ported + 3 个 Partial，其余 32 个有明确前置系统。Phase 9 不假装“41 个全部通过”，而是把可测部分测透、把不可测部分记录为依赖阻塞。

## 3. Rust 测试矩阵（新增）

| ID | 场景 | 层 | 主要断言 | 文件 |
|---|---|---|---|---|
| S1 | 自由落体解析解 | L5 | `y(n) = y0 - g·dt²·n(n+1)/2`，速度 `-g·dt·n`，容差 1e-4 | `crates/havok/tests/simulation_scenarios.rs` |
| S2 | 球落静态盒并静止 | L5 | 最终 y∈[0.35,0.75]、`|vy|<0.2`、最后一步 manifold/contact>0 | 同上 |
| S3 | 三盒堆叠 | L5 | 三盒垂直顺序保持、无穿透地面、速度收敛 | 同上 |
| S4 | 摩擦滑动 | L5 | 水平速度衰减、位移小于无摩擦上界 | 同上 |
| S5 | 胶囊落地 | L5 | 胶囊在盒地面上稳定、不穿地 | 同上 |
| S6 | 圆柱落地 | L5 | 圆柱稳定、不穿地 | 同上 |
| S7 | 流形约定 | L3 | normal 单位化、`position+normal*distance=point_on_a`、负距离=穿透 | 同上 |
| S8 | 宽相确定性 | L3 | 输入乱序 → 输出 `(a,b)` 排序去重一致；空 AABB 忽略 | 同上 |
| S9 | 双 world 逐位一致 | L6 | 同场景 180 步，每个 body 的 pos/orientation/vel 的 `to_bits` 完全相同 | `crates/havok/tests/determinism.rs` |
| S10 | snapshot/trace 稳定 | L6 | 重复运行 trace 文本相同、头部 `# hvk-trace v1`、含 step/body 行 | 同上 |
| S11 | body 生命周期 | L2 | 复用槽后旧 BodyId 失效、新 id serial 不同 | `crates/havok/tests/original_unit_tests.rs` |
| S12 | 原版结构性映射 | L1/L2 | 6 个 Ported + 3 个 Partial 的最小断言 | `crates/havok/tests/original_unit_tests.rs` |

## 4. 基准场景（`cargo bench -p havok --bench simulation`）

无 criterion（crates.io 不可达）；使用 `harness = false` + `std::time::Instant` 的自研 harness。参数：

- `--quick`：小规模冒烟（CI/本地验证）；
- `--bodies N`：覆盖 body 规模；
- `--steps N`：覆盖积分步数；
- 输出 `name<TAB>value<TAB>unit`，便于脚本解析。

| 基准 | 规模（默认） | 指标 | 目的 |
|---|---|---|---|
| `broadphase_grid` | 256 个 AABB | ms/iter + pair 数 | 网格宽相扩展性 |
| `gjk_sphere_pair` | 1000 次 | µs/pair | GJK 光滑形状开销 |
| `gjk_box_pair` | 1000 次 | µs/pair | GJK 多面体开销 |
| `narrowphase_box_pairs` | 1000 次 | µs/pair | GJK+EPA+流形整链 |
| `solver_100_contacts` | 100 body/100 约束 | ms/solve | 求解器迭代成本 |
| `world_fall_n` | 64 个动态球 + 地面，60 步 | ms/step + contacts | 完整 step 管线 |
| `world_stack_10` | 10 个盒子，120 步 | ms/step + contacts | 堆叠稳定性/成本 |

基准只记录本机数值，不设置跨机器硬阈值；README 记录运行命令，Phase 9 验收以“harness 能稳定运行并输出指标”为准。

## 5. 黄金基准方案与当前阻塞

原版 `original/Physics/` 是**源码分发**：没有生成好的 `.sln`、没有可直接运行的 Physics 测试可执行文件/库；`configureHavok.exe` 只是配置工具，完整构建还依赖原版 MSBuild/编译器链。当前环境无法直接跑 C++ 场景，因此：

1. 已实现 `World::snapshot()`：按 body 槽位顺序输出去重后的位置/朝向/线速度/角速度；
2. 定义 `hvk-trace v1` 文本格式（浮点用 `to_bits()` 十六进制，避免文本格式化漂移）；
3. Rust 侧先做**同场景两次运行逐位一致**（S9）和 trace 稳定性（S10）；
4. C++ 黄金基准需要：生成/配置原版解决方案 → 构建 Physics + UnitTest → 跑固定场景导出 `hvk-trace v1` → 与 Rust 输出逐行比较。该步骤在 `../original` 当前状态下不可执行，列为外部阻塞项，不做“已逐位对齐”的虚假声明。

### `hvk-trace v1` 格式

```text
# hvk-trace v1
step <u64>
body <BodyId hex u64> p <f32 bits> <f32 bits> <f32 bits> q <f32 bits> <f32 bits> <f32 bits> <f32 bits> v <f32 bits> <f32 bits> <f32 bits> w <f32 bits> <f32 bits> <f32 bits>
```

- `BodyId` 用 `value()` 的 16 位十六进制；
- 每个向量/四元数按分量 `to_bits()` 输出 `%08x`；
- 行顺序 = body manager 槽位顺序（确定性）。

## 6. 验收标准

- `cargo test --workspace` 全绿，新增集成测试 ≥ 10 个；
- `cargo bench -p havok --bench simulation -- --quick` 正常退出并打印全部指标；
- `cargo fmt/clippy/doc/release` 全绿；
- README Phase 9 段如实记录 Ported/Partial/Blocked 数量与 C++ 基准阻塞项；
- 不新增外部依赖。
