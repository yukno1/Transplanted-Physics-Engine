# 重构计划：数学层迁移至 glam

> 状态：执行中
> - **Phase 0（基线与死代码清理）：✅ 完成（2026-11-25）** — 删除五常量、零引用方法及自测试；全量测试迁移前基线全绿（lib 638 + 集成 ~190）
> - **Phase 1（确定性策略）：✅ 决策定案（2026-11-25）** — golden trace 外部基准被阻塞（`phase9-test-matrix.md` §5），采用**方案 B**（glam 类型为主体 + SDK 语义扩展 trait `Vec4Ext`/`QuatExt`/`Mat3Ext`）；确定性由 `tests/glam_parity.rs` 位级回归门 + `tests/determinism.rs` 同运行逐位比对保证
> - **Phase 2（过渡层）：✅ 完成（2026-11-25）** — `base_math` 改为 glam re-export + 扩展 trait；整删 `qtransform.rs`/`qstransform.rs`/`half.rs`/`simd.rs`；`Aabb` 字段换 `Vec3A`、`PreciseTransform` 换 `Mat3`/`Vec4`（double-float 逻辑逐行保留）；`world.rs` 大世界路径换 `DVec4`；新增 `tests/glam_parity.rs` 位级回归门（含 glam SIMD `min` NaN 语义差异的显式覆盖）
> - **Phase 3-5：待执行** — 旧自实现类型（`vector.rs`/`quaternion.rs`/`matrix3.rs`/`transform.rs`/`Vector4d`）过渡期保留，待逐模块批量替换
> 依据：2026-11 全库盘点（base_math 实现细节 + 12 类型全库使用统计）
> 目标：取消自实现数学类型，主体类型直换 `glam 0.34.3`（已在 `Cargo.toml` 声明但未使用），仅保留 glam 无对应物的最小残留。

## 1. 现状盘点

### 1.1 自实现数学层（`src/base_math/`）

| 类型 | 定义位置 | 外部使用量 (src / tests / benches) | glam 0.34.3 对应 | 结论 |
|---|---|---|---|---|
| `Vector4` | `vector.rs:147-751`（宏生成） | ~2079 / 623 / 51 | `Vec4`（备选 `Vec3A`） | 核心类型，遍布结构体字段；w 位为填充语义 |
| `Transform` | `transform.rs` | ~269 / 91 / 6 | `Affine3A` | `Matrix3 + Vector4` |
| `Quaternion` | `quaternion.rs:38-310` | ~197 / 55 / 6 | `Quat` | 自实现 `Default` 为全零（glam 为 IDENTITY），需逐处核查 |
| `Aabb` | `aabb.rs:25-281` | ~251 / 24 / 5 | **无对应**（保留自定义） | 比较仅判 xyz、`select` 位选角点等 SDK 语义 |
| `Matrix3` | `matrix3.rs:26-256` | ~85 / 7 / 2 | `Mat3` | 列存储 → glam `x_axis/y_axis/z_axis` |
| `Vector4d` | `vector.rs`（宏生成） | 6 / 4 / 0 | `DVec4` | 仅 `phys_world/world.rs` 大世界路径 |
| `PreciseTransform` | `precise_transform.rs` | 3 / 0 / 0 | **无对应**（保留自定义） | double-float 补偿加法，仅 `world.rs` 一个消费者 |
| `Vector4Mask` | `vector.rs:20-145` | 0（仅 `aabb.rs` 内部） | `BVec4` / `bitmask()` | 随 Aabb 迁移消失 |
| `Vector4dMask` | `vector.rs:144-145` | 0 | — | 纯死代码 |
| `QTransform` | `qtransform.rs` | 0 | — | 纯死代码 |
| `QsTransform` | `qstransform.rs` | 0 | — | 纯死代码（含 SDK 特有的 `setMul` / `setMulScaled` 区分，无人用） |
| `Half` | `half.rs` | 0 | — | 纯死代码（含 SDK 位级怪癖；直接删，不必引入 `half` crate） |

### 1.2 关键事实（影响迁移策略）

- 全部数学类型为 `#[repr(C, align(16))]`、字段 `pub`；但**无** `transmute` / 指针布局依赖，对齐约束仅服务于内部 SIMD。
- 测试与 bench **零结构体字面量**构造，全部走构造函数/常量 → 迁移摩擦低。唯一字面量：`src/convert.rs:211` 的 `Quaternion { x, y, z, w }`。
- `base_math/simd.rs`（301 行 SSE2 手写后端）只被 `vector.rs` 引用，随 Vector4 消亡。
- `convert.rs` 是 tagfile `Value` 树 → 物理对象的桥（解析 `m_` 前缀 Havok 字段），需适配新构造 API；它不做内部数学类型互转。
- **确定性约束**：工程要求 bit-exact 浮点复现（golden trace，Phase 9）；自实现层刻意保持 SDK 标量运算顺序、禁用 FMA 收缩（`simd.rs:13-14`、`vector.rs:418-421`）。

### 1.3 SDK 特有、glam 不覆盖的语义（迁移时需保留或显式放弃）

| 语义 | 位置 | 处理 |
|---|---|---|
| `dot3` / `dot4` 分离、`horizontal_add3/4` 固定加法顺序 | `vector.rs` | `Vec4Ext` 扩展 trait |
| `normalize` 零向量 → 零向量（`HK_SQRT_SET_ZERO`） | `vector.rs:363` | `Vec4Ext::normalize_or_zero` |
| 比较掩码返回 `Vector4Mask(u8)` + `select` 位选 | `vector.rs` / `aabb.rs` | 改用 `BVec4`（`bitmask()`）+ 重写 `select` |
| `min/max` 走 Rust `f32::min` NaN 规则（非 SSE `minps`） | `vector.rs:418-421` | 保持调用 Rust 内建 |
| `Quat::rotate` 采用 SDK `_setRotatedDir` 公式 | `quaternion.rs:195-205` | `QuatExt::rotate_sdk`（方案 B）或接受差异（方案 A） |
| `from_matrix`：4 分支 trace 算法 + 后置归一化 | `quaternion.rs:221-260` | 同上 |
| `Matrix3::inverse` 的 eps 判据（`feps·L1` 积）与奇异 → 零矩阵 | `matrix3.rs:189-201` | `Mat3Ext::inverse_sdk` |
| `Aabb` 的 xyz-only 比较、空盒 = `±f32::MAX`、`signed_distance_to_point` | `aabb.rs` | 自定义 `Aabb` 保留 |
| `PreciseTransform` 的 double-float 分解 / 补偿加法 / `relative_to` | `precise_transform.rs` | 原样保留 |

## 2. 使用 glam 后的无用项清单

### 2.1 整文件 / 整类型（随迁移删除）

- `src/base_math/simd.rs`（SSE2 后端，仅 `vector.rs` 使用）
- `src/base_math/qtransform.rs`（`QTransform`，零外部引用）
- `src/base_math/qstransform.rs`（`QsTransform`，零外部引用）
- `src/base_math/half.rs`（`Half`，零外部引用）
- `Vector4Mask` / `Vector4dMask`（`vector.rs:20-145`）
- `src/base_math/tests/simd_parity.rs`（SIMD/标量位级一致性门，随 simd.rs 失去意义；思路改写为 glam_parity 后保留至 Phase 5）

### 2.2 零引用方法 / 常量（与 glam 无关，Phase 0 即可删）

| 项 | 位置 |
|---|---|
| `Quaternion::set_real` | `quaternion.rs:82` |
| `Matrix3::mul_quaternion` | `matrix3.rs:155` |
| `Matrix3::frobenius_norm_squared` | `matrix3.rs:246` |
| `Aabb::from_line` | `aabb.rs:91` |
| `Aabb::center2` | `aabb.rs:161` |
| `Aabb::supporting_vertex` | `aabb.rs:253`（仅自测试使用） |
| `Vector4/Vector4d::dot4`、`length4/length_squared4`、`normalized4`、`horizontal_add4`、`flip_sign` | `vector.rs:333-461`（仅内部调用与 `tests/simd_parity.rs`） |
| `Vector4::greater_zero` | `vector.rs:536-544`（`less_zero` 仅 `Aabb::supporting_vertex` 使用，随其一同删） |
| `Vector4::basis` | `vector.rs:229`（仅自测试使用） |
| `Matrix3::is_orthonormal` | `matrix3.rs:225`（仅自测试使用） |
| `Half::from_f32_truncate` | `half.rs:84`（仅自测试使用） |
| `Vector4Mask::X/Y/Z/W` 单位常量 | `vector.rs:33-39` |
| 常量 `PI` / `TWO_PI` / `HALF_PI` / `EPSILON` / `MAX_REAL` | `base_math/mod.rs:75-83`（全库各处直接用 `core::f32::consts` / `f32::EPSILON`） |

### 2.3 迁移后冗余的手写辅助（glam 直接覆盖）

- `src/phys_shape/mass.rs:324-356`：`matrix_diagonal` / `matrix_outer` / `matrix_add` / `matrix_sub` / `matrix_scale` → `Mat3::from_diagonal`、`Mat3::from_cols` 外积、运算符。
- `src/phys_body/motion.rs:9` `inverse_inertia_diagonal_matrix(Vector4) -> Matrix3`、`src/phys_solver/solver.rs:51` `diagonal_inverse_inertia(Vector4) -> Matrix3` → `Mat3::from_diagonal`。
- `src/base_geometry/weld.rs:192`、`convex_hull.rs:488` 的局部 `fn point(x,y,z)` 包装 → 直接用 `Vec4::new` / `Vec3A::new`。
- 可选：`src/base_types/bitops.rs:295-303` `bit_cast`（`transmute_copy`）→ `bytemuck`（非 glam 范畴，独立决策）。

### 2.4 消失的自实现方法总量

Vector4/Vector4d ~94 个、Quaternion ~24 个、Matrix3 ~20 个、Transform ~11 个固有方法与全部手写 trait impl（Add/Sub/Mul/Div/Assign/Index 等）由 glam 提供，合计约 **250 个方法**不再维护；`base_math` 最终仅保留 re-export + 3 个扩展 trait + `Aabb` + `PreciseTransform`。

## 3. 分阶段执行计划

### Phase 0 — 基线与死代码清理（约 0.5 天）✅ 已完成（2026-11-25）

1. `cargo test` / `cargo bench` 全绿并记录基线（对照 `docs/performance-baseline.md`）。
2. 删除 §2.2 全部零引用项（与 glam 无关，独立提交，便于回滚）。

### Phase 1 — 确定性策略决策（关键决策点）✅ 定案：方案 B（2026-11-25，golden trace 阻塞详见文件头）

自实现层刻意复现 SDK 标量运算顺序以支撑 golden trace；glam 的 `Quat::rotate`、`Mat3::mul_vec`、归一化等实现顺序不同。三个方案：

| 方案 | 说明 | 评估 |
|---|---|---|
| A. 全量直换 + 容差验证 | 接受 `sin/cos/normalize` 类差异，全部用现有容差测试 | ⚠️ golden trace 若逐位比对则不可行 |
| B. 直换 + SDK 语义扩展 trait（**推荐**） | glam 类型为主体；仅为热点保留薄扩展（实现按 SDK 标量顺序）：`Vec4Ext::dot3/horizontal_add3/normalize_or_zero`、`QuatExt::rotate_sdk/from_matrix_sdk`、`Mat3Ext::inverse_sdk` | 折中；扩展 trait 数量少、实现量可控 |
| C. 保留包装类型（`struct Vector4(glam::Vec4)` 内部委托） | 完全保持现有 API | 违背"取消自实现类型"的目标，不采用 |

**行动**：先做一次 golden trace 对比实验（glam 原生实现 vs 期望基线），据结果在 A/B 间定案。

### Phase 2 — 类型映射与过渡层（约 1-2 天）✅ 已完成（2026-11-25）

实施备注（与原计划的差异）：
- glam `Vec4` 原生已有 `select(BVec4A, ..)`，扩展 trait 未重复提供位选；`Aabb` 位选角点改为纯标量 `vertex()`（原 `select` 版本为死代码）。
- glam SSE2 后端的 `Vec3A::min/max`（`minps` 操作数序）与旧实现的 `f32::min/max` NaN 语义不同，`Aabb::include_point/include_aabb` 保留逐分量 `f32::min/max`，并已由 glam_parity 用 NaN 用例锁定。
- `PreciseTransform::transform_point` 等公开 API 保持 legacy `Vector4` 边界类型，仅在存储层使用 glam，调用方零改动。

1. `base_math` 改为 re-export 层：
   - `pub use glam::{Vec4, Vec3A, Quat, Mat3, Affine3A, BVec4, DVec4};`
   - 过渡期类型别名 `pub type Vector4 = Vec4;` 等，保证 27 个测试文件先编译通过；迁移完成后删除别名。
2. 落地扩展 trait（仅 §1.3 中标记保留的语义）。
3. 新增/改造 glam 无对应物的 2 个自定义类型：
   - `Aabb { min: Vec3A, max: Vec3A }`：~28 个方法用 `Vec3A::min/max/cmpge/bitmask` 重写；空盒 = `±f32::MAX`、xyz-only 比较语义保留。
   - `PreciseTransform { rotation: Mat3, translation: Vec4, adjustment: Vec4 }`：double-float 逻辑原样保留，仅底层数据结构换 glam。
4. 复用 `simd_parity.rs` 思路编写 glam_parity 测试（旧实现 vs 新实现位级对比），作为 Phase 3 每步的回归门。

### Phase 3 — 逐模块替换（约 3-5 天，按依赖序推进）

| 步骤 | 范围 | 要点 |
|---|---|---|
| 1 | `base_geometry`、`base_serialize`、`convert.rs` | 构造 API 适配；删除 `weld.rs`/`convex_hull.rs` 的 `point()` 包装；`convert.rs:211` 字面量改 `Quat::from_xyzw` |
| 2 | `phys_shape`（含 `mass.rs`） | 删除 §2.3 手写矩阵辅助；惯性矩阵改 `Mat3::from_diagonal` |
| 3 | `phys_body` | `motion.rs` 对角惯性辅助删除；核查 `Default` 依赖（Quaternion 语义差异） |
| 4 | `phys_collide`（gjk / epa / query / shape_cast / broadphase） | 热点密集；每文件提交后跑 glam_parity + 单测 |
| 5 | `phys_solver`、`phys_world` | `world.rs` 大世界路径：`Vector4d→DVec4`、`PreciseTransform` 适配 |
| 6 | `phys_extensions`（vehicle / ragdoll） | 纯机械替换为主 |
| 7 | `tests/`、`benches/` | 构造函数风格，机械替换；过渡期别名兜底 |

### Phase 4 — 收尾清理（约 0.5 天）

- 删除 `base_math` 全部临时别名与残留文件；最终 `base_math` 仅含：glam re-export、`Vec4Ext` / `QuatExt` / `Mat3Ext`、`Aabb`、`PreciseTransform`。
- 明确 `Vec4` w 位约定（若热点保留 `Vec4` 则文档化；改 `Vec3A` 处 w 逻辑自然消失）。
- `cargo clippy` / `cargo doc` 检查；更新 `README.md` 与 `Cargo.toml` 注释（"零外部依赖"表述已过时）。

### Phase 5 — 验证（约 1 天）

1. 全量测试 + bench 前后性能对比（glam SSE2/自动向量化应持平或更好；重点确认 `dot3`、mask、Aabb 热点）。
2. golden trace 逐位比对。
3. glam_parity 测试全绿后移除其依赖的旧实现代码路径。

## 4. 风险清单

| # | 风险 | 缓解 |
|---|---|---|
| 1 | **确定性回归**（golden trace 逐位比对失败）— 最大风险 | Phase 1 决策先行；方案 B 的扩展 trait 保持 SDK 标量顺序 |
| 2 | `Quaternion::Default` 语义差异（自实现 = 全零，glam = IDENTITY） | 迁移时逐处核查依赖 `Default` 的代码，显式改 `Quat::IDENTITY` 或零值构造 |
| 3 | `QsTransform::Mul` 的"中间平移不乘 S1"等 SDK 特有语义 | 已确认死代码，直接删除，无迁移成本 |
| 4 | `convert.rs` 结构体字面量与 `m_` 字段解析耦合 | Phase 3 步骤 1 单独适配 + 单测 |
| 5 | Aabb / PreciseTransform 无 glam 对应物 | 预期内最小残留（2 个类型），非"取消自实现"的例外扩大 |
| 6 | 性能回归（mask / `select` / `dot3` 热点） | Phase 5 bench 对比；必要时热点改用 `Vec3A` / `glam::f32s`（core-simd） |

## 5. 工作量与顺序

- 总量估计：**约 7-10 个工作日**。
- 严格按 Phase 0 → 1 → 2 → 3 → 4 → 5 顺序；Phase 0 与 Phase 1 的实验可并行。
- 每个提交保持 `cargo test` 全绿；Phase 3 每步提交后运行 glam_parity 回归门。
