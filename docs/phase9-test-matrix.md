# Phase 9 测试矩阵与基准场景

> 状态：Rust 侧测试/基准实施计划（2026-11-11）。原版 C++ 黄金基准运行受外部构建条件阻塞，详见 §5。

## 1. 目标与原则

Phase 9 的目标不是“补几个测试”，而是给迁移建立可重复的验证层次：

- **L0 单元层**：`base-types` / `base-math` / `base-memory` / `base-container` / `base-thread` 已有 165 个单元测试（另含文档测试）。
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
| 2 | `NpBodyToConstraintsMapTest` | body↔constraint 映射 | Partial（Phase 11） | `World::constraints_of_body` / `constraints_of_body` 槽序遍历已实现；SDK 的 `hknpBodyToConstraintsMap` 增量映射对象未移植 |
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
| 15 | `NpDeflectedSphereCastTest` | 偏转球投射 | Blocked | 基础 `shape_cast` 已实现（Phase 11）；`hknpDeflectedLinearCast` 未移植 |
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
| 28 | `NpRayCastTest` | 射线查询 | Ported（Phase 11） | 凸体 raycast + mesh/height field/compound 遍历、范围/内部命中、确定性排序；`hkcdRayQueryFlags`/shape tag 解码未移植 |
| 29 | `NpRebuildConvexShapeMassPropertiesAccuracyTest` | 重建质量属性精度 | Blocked | mass properties 未移植 |
| 30 | `NpSerializeConvexTest` | 凸体序列化 | Blocked | hkx 序列化未移植 |
| 31 | `NpShapeCastTest` | 形状投射 | Ported（Phase 11） | 凸-凸 conservative advancement + 复合目标叶子遍历、容差、起始穿透；旋转 cast 未移植 |
| 32 | `NpShapeManagerUnitTest` | shape manager | Blocked | shape manager 未移植 |
| 33 | `NpShapeMemoryMapTest` | shape 内存布局 | Blocked | in-place shape buffer 未移植 |
| 34 | `NpShapeSerializeTest` | shape 序列化 | Blocked | hkx 序列化未移植 |
| 35 | `NpSpatialQueryUnitTest` | 空间查询 | Partial（Phase 11） | raycast / shape cast / closest point(s) / AABB 查询与 `QueryFilter` 已实现；broad-phase layer、shape tag codec、query determinism 检查未移植 |
| 36 | `NpThreadSafeObjectPoolTest` | 线程安全对象池 | Blocked | object pool 未移植（BodyManager 是索引池，不是同一设施） |
| 37 | `NpThreadSafeQueueTest` | 线程安全队列 | Blocked | `hknpThreadSafeQueue` 未移植 |
| 38 | `NpUFMShapeTagCodecTest` | UFM shape tag codec | Blocked | shape tag codec 未移植 |
| 39 | `NpWorldInspectorTest` | world inspector | Blocked | inspector 未移植 |
| 40 | `NpWorldSnapshotTest` | world 快照/拷贝 | Partial | 已新增 `World::snapshot`/`hvk-trace`；完整 world copy 未移植 |
| 41 | `TimeStampHash` | 时间戳哈希（辅助） | Blocked | 原版 determinism util 未移植 |

**结论**：41 个中当前可落地 6 个 Ported + 5 个 Partial（Phase 11 将 #28/#31 转为 Ported、#2/#35 转为 Partial），其余 30 个有明确前置系统。Phase 9 不假装“41 个全部通过”，而是把可测部分测透、把不可测部分记录为依赖阻塞。

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
| S12 | 原版结构性映射 | L1/L2 | 6 个 Ported + 5 个 Partial 的最小断言 | `crates/havok/tests/original_unit_tests.rs` |
| S13 | ball-socket 摆锤 | L4/L5 | 世界 pivot 与锚点始终接近（<0.4）、约束随 body 销毁 | `constraints_and_queries.rs` |
| S14 | hinge 角度 limits | L4/L5 | 越过上限后角度被拉回窗口内；初始越界 −0.1 rad 以上 | 同上 |
| S15 | hinge 速度 motor | L4/L5 | 60 步后绕轴角速度追踪目标（±0.2）、关闭后不制动 | 同上 |
| S16 | fixed 刚性对 | L4/L5 | 双体下落间距保持（±0.3）、整体下落 | 同上 |
| S17 | ragdoll cone/twist | L4/L5 | cone 外的摆动回到 cone 内（<0.35 rad）、twist 越界回复 | 同上 + `phys-world` 单元 |
| S18 | prismatic 滑块 | L4/L5 | 仅沿轴移动（垂直漂移 <0.15）、越界停在 limit 附近 | 同上 |
| S19 | spring / point-to-plane | L4/L5 | 距离收敛到 rest length（±0.35）、pivot 保持在平面（±0.25） | 同上 |
| S20 | 约束碰撞屏蔽 + body↔constraint 映射 | L4 | `set_constraint_collisions_enabled(false)` → 接触为空；`constraints_of_body` 槽序 | 同上 |
| S21 | world raycast（复合体） | L3/L5 | mesh/height field/compound 命中距离与法线、过滤忽略 body、确定性 | `phys-world` 单元 |
| S22 | shape cast（复合体） | L3/L5 | mesh 地面分数 4.5、起始穿透 `start_penetrating`、忽略 caster | `phys-collide` 单元 + 集成 |
| S23 | 查询 filter / closest points / AABB | L3 | ignore body、motion-type 开关、重叠负距离、槽序 AABB 结果 | `phys-world` 单元 + 集成 |
| S24 | 关节场景逐位一致 | L6 | 含 ball-socket/hinge/prismatic 的 180 步双 world snapshot 逐位相等 | `constraints_and_queries.rs` |

## 4. 基准场景（`cargo bench -p proton-engine --bench simulation`）

无 criterion（crates.io 不可达）；使用 `harness = false` + `std::time::Instant` 的自研 harness。参数：

- `--quick`：小规模冒烟（CI/本地验证）；
- 输出 `name<TAB>value<TAB>unit<TAB>detail`，便于脚本解析；
- 场景类指标用 `ms/iter` / `ms/step`，数学微基准用 `us/iter`（见 `docs/performance-baseline.md`）。

| 基准 | 规模（默认） | 指标 | 目的 |
|---|---|---|---|
| `math_vector4_add` / `math_vector4_mul` | 2,000,000 次 | us/iter | 分量算术 SIMD 微基准 |
| `math_vector4_dot3` / `math_vector4_normalize3` | 2,000,000 次 | us/iter | 归约与归一化顺序基线 |
| `math_quaternion_mul` / `math_matrix3_mul_vec` / `math_aabb_include_point` | 2,000,000 次 | us/iter | 四元数/矩阵/AABB 热点判定 |
| `broadphase_grid` | 512 个 AABB | ms/iter + pair 数 | 网格宽相扩展性 |
| `gjk_sphere_pair` / `gjk_box_pair` | 2000 次 | ms/iter | GJK 距离开销 |
| `narrowphase_box_pairs` | 2000 次 | ms/iter | GJK+EPA+流形整链 |
| `solver_contact_pair` | 2 body/1 contact | ms/iter | 求解器单接触成本 |
| `world_fall_n` / `world_fall_n_parallel` | 64 动态球，60 步 | ms/step + contacts | 单线程/4 线程 step 管线 |
| `world_stack_10` | 10 个盒子，120 步 | ms/step + contacts | 堆叠稳定性/成本 |
| `raycast_world` / `shape_cast_world` | 2000 次 | ms/iter | 世界查询 |
| `joint_chain_ragdoll` | 24 个链接，60 步 | ms/step | 关节求解/重建成本 |

基准只记录本机数值，不设置跨机器硬阈值；README 记录运行命令，Phase 9 验收以“harness 能稳定运行并输出指标”为准。

## 5. 黄金基准方案与当前阻塞

原版 `original/Physics/` 是**源码分发**：没有生成好的 `.sln`、没有可直接运行的 Physics 测试可执行文件/库；`configureHavok.exe` 只是配置工具，完整构建还依赖原版 MSBuild/编译器链。当前环境无法直接跑 C++ 场景，因此：

1. 已实现 `World::snapshot()`：按 body 槽位顺序输出去重后的位置/朝向/线速度/角速度；
2. 定义 `hvk-trace v1` 文本格式（浮点用 `to_bits()` 十六进制，避免文本格式化漂移）；
3. Rust 侧先做**同场景两次运行逐位一致**（S9）和 trace 稳定性（S10）；
4. C++ 黄金基准需要：生成/配置原版解决方案 → 构建 Physics + UnitTest → 跑固定场景导出 `hvk-trace v1` → 与 Rust 输出逐行比较。该步骤在 `../original` 当前状态下不可执行，列为外部阻塞项，不做“已逐位对齐”的虚假声明。
5. 2026-11-24 起，任何数学后端改动还须通过 `crates/base-math/tests/simd_parity.rs`（`-0.0`、denormal、NaN/Inf、比较掩码、归约、AABB）和 `tests/determinism.rs::GOLDEN_TRACE_30`；性能验收见 `docs/performance-baseline.md`，只宣称与 Rust 标量路径逐位一致。

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
- `cargo bench -p proton-engine --bench simulation -- --quick` 正常退出并打印全部指标；
- `crates/base-math/tests/simd_parity.rs` 与固定 `GOLDEN_TRACE_30` 全绿（数学后端改动必需）；
- `cargo fmt/clippy/doc/release` 全绿；
- README Phase 9 段如实记录 Ported/Partial/Blocked 数量与 C++ 基准阻塞项；
- 不新增外部依赖。
