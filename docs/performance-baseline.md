# 性能基线与 SIMD 优化记录

> 本文件是 SIMD/性能阶段的可复现基线。基准只用于同一环境下的相对比较，不承诺绝对耗时；C++ 黄金轨迹仍因原版 SDK 无可构建链而阻塞，本阶段只与 Rust 标量路径做逐位对照。

## 1. 测试环境

| 项 | 值 |
|---|---|
| 日期 | 2026-11-24 |
| 仓库状态 | `84421d0` 之后的优化工作区；基线采集时未改动 `src/base_math` |
| OS | Microsoft Windows NT 10.0.26300.0 |
| CPU | 12th Gen Intel(R) Core(TM) i9-12900H |
| Rust | `rustc 1.102.0-nightly (3ef0c22cd 2026-11-11)` |
| Cargo | `cargo 1.102.0-nightly (c07ac2c4d 2026-11-06)` |
| 工程布局 | 单 crate；`edition = "2024"`，`rust-version = "1.85"`，零外部依赖 |
| 命令 | `cargo bench --bench simulation -- --quick` / `cargo bench --bench simulation` |

## 2. 基准参数

| 参数 | quick | full |
|---|---:|---:|
| `broadphase_count` | 64 | 512 |
| `broadphase_iterations` | 3 | 10 |
| `gjk_iterations` | 200 | 2000 |
| `narrowphase_iterations` | 200 | 2000 |
| `solver_iterations` | 5 | 20 |
| `fall_bodies` / `fall_steps` | 16 / 10 | 64 / 60 |
| `stack_steps` | 30 | 120 |
| `query_iterations` | 200 | 2000 |
| `joint_bodies` / `joint_steps` | 8 / 10 | 24 / 60 |
| `math_iterations`（新增） | 100,000 | 2,000,000 |

基准输出格式保持 `# hvk-bench v1` 与 `name\tvalue\tunit\tdetail` 四列。场景类指标沿用 `ms/iter`、`ms/step`，新增数学微基准使用 `us/iter` 以显示亚微秒差异。

## 3. 原始基线：优化前场景基准

在加入微基准前采集，用于确认新增微基准没有改变既有场景指标的可比性。

### quick

```text
# hvk-bench v1 quick=true
# name	value	unit	detail
broadphase_grid	0.027	ms/iter	bodies=64 pairs=156
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.055	ms/step	bodies=16 contacts=15
world_fall_n_parallel	0.026	ms/step	bodies=16 contacts=15
world_stack_10	0.012	ms/step	bodies=10 contacts=3
raycast_world	0.004	ms/iter	bodies=17 hits=201
shape_cast_world	0.001	ms/iter	bodies=1 hits=201
joint_chain_ragdoll	0.027	ms/step	bodies=9 joints=8
```

### full

```text
# hvk-bench v1 quick=false
# name	value	unit	detail
broadphase_grid	0.244	ms/iter	bodies=512 pairs=1906
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.064	ms/step	bodies=64 contacts=24
world_fall_n_parallel	0.172	ms/step	bodies=64 contacts=24
world_stack_10	0.016	ms/step	bodies=10 contacts=8
raycast_world	0.004	ms/iter	bodies=17 hits=2001
shape_cast_world	0.002	ms/iter	bodies=1 hits=2001
joint_chain_ragdoll	0.051	ms/step	bodies=25 joints=24
```

## 4. 原始基线：加入微基准后

微基准使用逐次变化的输入，避免 LLVM 把常量运算提升出循环。以下数据作为 SIMD 改造的直接比较对象。

### quick

```text
# hvk-bench v1 quick=true
# name	value	unit	detail
math_vector4_add	0.0013	us/iter	iterations=100000
math_vector4_mul	0.0012	us/iter	iterations=100000
math_vector4_dot3	0.0018	us/iter	iterations=100000
math_vector4_normalize3	0.0016	us/iter	iterations=100000
math_quaternion_mul	0.0083	us/iter	iterations=100000
math_matrix3_mul_vec	0.0015	us/iter	iterations=100000
math_aabb_include_point	0.0027	us/iter	iterations=100000
broadphase_grid	0.028	ms/iter	bodies=64 pairs=156
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.033	ms/step	bodies=16 contacts=15
world_fall_n_parallel	0.025	ms/step	bodies=16 contacts=15
world_stack_10	0.012	ms/step	bodies=10 contacts=3
raycast_world	0.003	ms/iter	bodies=17 hits=201
shape_cast_world	0.001	ms/iter	bodies=1 hits=201
joint_chain_ragdoll	0.023	ms/step	bodies=9 joints=8
```

### full

```text
# hvk-bench v1 quick=false
# name	value	unit	detail
math_vector4_add	0.0012	us/iter	iterations=2000000
math_vector4_mul	0.0014	us/iter	iterations=2000000
math_vector4_dot3	0.0014	us/iter	iterations=2000000
math_vector4_normalize3	0.0016	us/iter	iterations=2000000
math_quaternion_mul	0.0085	us/iter	iterations=2000000
math_matrix3_mul_vec	0.0024	us/iter	iterations=2000000
math_aabb_include_point	0.0044	us/iter	iterations=2000000
broadphase_grid	0.387	ms/iter	bodies=512 pairs=1906
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.088	ms/step	bodies=64 contacts=24
world_fall_n_parallel	0.283	ms/step	bodies=64 contacts=24
world_stack_10	0.017	ms/step	bodies=10 contacts=8
raycast_world	0.004	ms/iter	bodies=17 hits=2001
shape_cast_world	0.001	ms/iter	bodies=1 hits=2001
joint_chain_ragdoll	0.054	ms/step	bodies=25 joints=24
```

## 5. 热点判断

- `math_quaternion_mul` 是微基准中明显最贵的单项（约 8.3–8.5 ns/iter），由 16 次标量乘加与交叉项组成，是首要观察对象。
- `math_aabb_include_point` 与 `math_matrix3_mul_vec` 次之，但它们分别包含分支/方法与 3 次点乘；是否向量化取决于逐位对照结果。
- `math_vector4_add` / `math_vector4_mul` 已接近单次函数调用量级；SSE2 版本只有在编译器未自动标量展开时才可能体现收益。
- 整场景指标噪声较大（同一环境下 `broadphase_grid` 从 0.244 到 0.387 ms/iter），因此优化验收以“微基准趋势 + 世界 trace 逐位不变”为主，不以单次绝对耗时作结论。

## 6. 后续更新约定

每完成一个 SIMD 阶段，在本文件追加一节，记录：

1. 改动范围（只改了哪些数学方法）；
2. quick/full 原始输出；
3. 与本章 §4 基线的相对变化；
4. 逐位对照测试与 `hvk-trace v1` 结果；
5. 未达成收益时的结论（保留代码或回退）。

## 7. SIMD 阶段结果（2026-11-24）

### 7.1 落地范围

- `src/base_math/simd.rs`：零依赖内部 helper。`x86_64` 上用 SSE2 实现 `add/sub/mul/div/neg/add_mul/sub_mul/abs/flip_sign`；非 `x86_64` 与全部 `f64` 走标量参考路径。
- `src/base_math/vector.rs`：`Vector4`/`Vector4d` 的公开 API 不变，分量算术改走 helper。
- 明确**不**向量化的部分：
  - 点积、叉积、水平求和、归一化：水平归约会改变加法顺序，破坏 `hvk-trace v1` 位级基线；
  - `min`/`max`：Rust 的 NaN/带符号零规则与 `_mm_min_ps` 不同；
  - 比较掩码与 `select`：实测令 `math_aabb_include_point` 从约 0.0027 退化到 0.0086 us/iter（约 3 倍），已回退为标量；
  - 四元数乘法、矩阵乘向量、AABB：微基准未显示稳定收益，保持原运算顺序并在源码中注明原因。
- AVX2、NEON、运行时特性检测、FMA 与本轮无关，未启用。`sin`/`cos`/`sqrt` 仍来自 `std`，只做容差断言。

### 7.2 最终 quick 输出

```text
# hvk-bench v1 quick=true
# name	value	unit	detail
math_vector4_add	0.0012	us/iter	iterations=100000
math_vector4_mul	0.0012	us/iter	iterations=100000
math_vector4_dot3	0.0012	us/iter	iterations=100000
math_vector4_normalize3	0.0013	us/iter	iterations=100000
math_quaternion_mul	0.0076	us/iter	iterations=100000
math_matrix3_mul_vec	0.0013	us/iter	iterations=100000
math_aabb_include_point	0.0025	us/iter	iterations=100000
broadphase_grid	0.019	ms/iter	bodies=64 pairs=156
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.026	ms/step	bodies=16 contacts=15
world_fall_n_parallel	0.019	ms/step	bodies=16 contacts=15
world_stack_10	0.009	ms/step	bodies=10 contacts=3
raycast_world	0.003	ms/iter	bodies=17 hits=201
shape_cast_world	0.001	ms/iter	bodies=1 hits=201
joint_chain_ragdoll	0.016	ms/step	bodies=9 joints=8
```

### 7.3 最终 full 输出

```text
# hvk-bench v1 quick=false
# name	value	unit	detail
math_vector4_add	0.0012	us/iter	iterations=2000000
math_vector4_mul	0.0012	us/iter	iterations=2000000
math_vector4_dot3	0.0015	us/iter	iterations=2000000
math_vector4_normalize3	0.0015	us/iter	iterations=2000000
math_quaternion_mul	0.0076	us/iter	iterations=2000000
math_matrix3_mul_vec	0.0013	us/iter	iterations=2000000
math_aabb_include_point	0.0026	us/iter	iterations=2000000
broadphase_grid	0.243	ms/iter	bodies=512 pairs=1906
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.052	ms/step	bodies=64 contacts=24
world_fall_n_parallel	0.182	ms/step	bodies=64 contacts=24
world_stack_10	0.016	ms/step	bodies=10 contacts=8
raycast_world	0.003	ms/iter	bodies=17 hits=2001
shape_cast_world	0.001	ms/iter	bodies=1 hits=2001
joint_chain_ragdoll	0.041	ms/step	bodies=25 joints=24
```

### 7.4 与 §4 基线的对比和结论

- `math_vector4_add` / `math_vector4_mul`：0.0012–0.0013 → 0.0011–0.0012 us/iter，处在噪声范围内；SSE2 路径不劣化，但也没有显著收益。
- `math_quaternion_mul`：0.0083/0.0085 → 0.0076/0.0076 us/iter，小幅波动，不足以证明 SIMD 重写四元数有稳定收益；保持标量。
- `math_aabb_include_point`：0.0027/0.0044 → 0.0025/0.0026 us/iter；中间曾因比较掩码向量化退化到 0.0086，回退后恢复。
- 整场景指标在噪声范围内波动（`broadphase_grid` 0.387 → 0.243 ms/iter 与早期 0.244 基线同量级），不把单次差值归因为 SIMD。
- 确定性门禁：`src/base_math/tests/simd_parity.rs` 6 个测试逐位比较特殊浮点值、比较掩码、归约与 AABB；`tests/determinism.rs` 的 `GOLDEN_TRACE_30` 固定 trace 未漂移；`cargo clippy --all-targets -- -D warnings` 通过。
- 结论：保留 x86_64 SSE2 分量算术路径作为后端骨架；本阶段只宣称“与 Rust 标量路径逐位一致”，不宣称与 C++ 或真实 `.hkx` 资产逐位等价。

## 8. 持久池 + 确定性任务图阶段（2026-11-24）

### 8.1 落地范围

- `World` 持有可选 `Arc<ThreadPool>`：`set_thread_pool` / `clear_thread_pool` / `thread_pool`；`step_with_threads(dt, n)` 在 `n > 1` 时惰性创建（或按需扩容到至少 `n` 个 worker）并复用同一池，`step(dt)` 不创建也不使用池。
- `src/phys_world/tasks.rs` 用 `TaskGraph` 表达两个计算阶段：窄相候选分块、按岛求解分块。每个 chunk 写入带索引的结果槽，依赖的 merge task 按候选/岛索引排序后回写；同阶段任务只读取不可变快照，结果归并始终在调用线程完成。
- 保持串行：重力、trigger/post-collide/post-solve 事件派发、接触行构建与修改器、warm-start 回写、速度回写、休眠计时、积分。共享 motion 的积分仍只执行一次，避免借用与顺序变化。
- 保留小场景阈值：候选 `<32` 走串行窄相，岛数 `<2` 走串行岛求解；`collect_contacts_parallel` 公共 API 仍使用 scoped 分块，便于对比。
- `benches/simulation.rs` 新增 `world_fall_n_persistent_pool`（显式预建 4 线程池），输出格式仍为 `# hvk-bench v1`。

### 8.2 最终 quick 输出

```text
# hvk-bench v1 quick=true
# name	value	unit	detail
math_vector4_add	0.0012	us/iter	iterations=100000
math_vector4_mul	0.0011	us/iter	iterations=100000
math_vector4_dot3	0.0013	us/iter	iterations=100000
math_vector4_normalize3	0.0015	us/iter	iterations=100000
math_quaternion_mul	0.0084	us/iter	iterations=100000
math_matrix3_mul_vec	0.0013	us/iter	iterations=100000
math_aabb_include_point	0.0025	us/iter	iterations=100000
broadphase_grid	0.020	ms/iter	bodies=64 pairs=156
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.030	ms/step	bodies=16 contacts=15
world_fall_n_parallel	0.038	ms/step	bodies=16 contacts=15
world_fall_n_persistent_pool	0.025	ms/step	bodies=16 contacts=15 threads=4 task-graph
world_stack_10	0.010	ms/step	bodies=10 contacts=3
raycast_world	0.003	ms/iter	bodies=17 hits=201
shape_cast_world	0.001	ms/iter	bodies=1 hits=201
joint_chain_ragdoll	0.017	ms/step	bodies=9 joints=8
```

### 8.3 最终 full 输出

```text
# hvk-bench v1 quick=false
# name	value	unit	detail
math_vector4_add	0.0011	us/iter	iterations=2000000
math_vector4_mul	0.0010	us/iter	iterations=2000000
math_vector4_dot3	0.0013	us/iter	iterations=2000000
math_vector4_normalize3	0.0014	us/iter	iterations=2000000
math_quaternion_mul	0.0077	us/iter	iterations=2000000
math_matrix3_mul_vec	0.0014	us/iter	iterations=2000000
math_aabb_include_point	0.0027	us/iter	iterations=2000000
broadphase_grid	0.219	ms/iter	bodies=512 pairs=1906
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.076	ms/step	bodies=64 contacts=24
world_fall_n_parallel	0.104	ms/step	bodies=64 contacts=24
world_fall_n_persistent_pool	0.082	ms/step	bodies=64 contacts=24 threads=4 task-graph
world_stack_10	0.016	ms/step	bodies=10 contacts=8
raycast_world	0.003	ms/iter	bodies=17 hits=2001
shape_cast_world	0.001	ms/iter	bodies=1 hits=2001
joint_chain_ragdoll	0.039	ms/step	bodies=25 joints=24
```

### 8.4 对比与结论

- 与 §7.3 的 `world_fall_n_parallel` 相比，full 从 0.182 降到 0.104（惰性池） / 0.082 ms/step（显式预建池）；quick 从 0.019 升到 0.038 / 0.025 ms/step。整场景计时噪声较大，不把单次差值单独归因于任务图；可确认的是每步不再创建 scoped worker。
- 任务图本身有提交和两次 barrier 的开销，小场景（`world_stack_10`、`joint_chain_ragdoll`）不追求加速；保留 `<32` 候选与 `<2` 岛的串行阈值。
- 确定性门禁：`tests/sleeping_and_parallel_solving.rs` 的 `explicit_persistent_pool_matches_single_threaded` 在 36 球场景比较 120 步 `hvk-trace v1`；`thread_pool_is_lazy_and_grows_with_requested_threads` 覆盖池生命周期；`tests/determinism.rs::pooled_task_graph_trace_matches_scalar_golden` 断言 30 步输出等于 `GOLDEN_TRACE_30`。原 `parallel_island_solving_matches_single_threaded` / `parallel_solving_with_warm_starting_and_joints_is_bitwise_identical` 与 `phys-world` 单元 `step_with_threads_matches_single_threaded` 仍全绿。
- 结论：保留持久池 + 确定性任务图作为后续完整物理任务图的调度骨架；本阶段只宣称“与 Rust 单线程路径逐位一致”，不宣称与 C++、真实 `.hkx` 资产或真实 VDB 轨迹逐位等价。

## 9. 优先级与 work-stealing 后端阶段（2026-11-25）

### 9.1 落地范围

- `TaskExecutor` 提供对象安全提交接口；`ThreadPool` 实现 FIFO 后端，新增 `WorkStealingPool` 使用每 worker 本地 deque、固定轮转投递和固定顺序窃取，worker panic 被捕获以保持工作线程可用，并通过 `JobHandle` 传播失败。
- `TaskGraph::execute_layered_with_priority` 保持拓扑层与 barrier 不变；开启优先级时仅在同一层内按优先级降序、任务 id 升序提交。世界层 compute chunk 使用 `High`、merge 使用 `Medium`，保证启用/禁用优先级时同层顺序都确定。
- `World` 新增 `SchedulerConfig` / `ExecutorKind` 与 `set_scheduler_config` / `set_executor` / `clear_executor` / `step_with_scheduler`；默认 `step` / `step_with_threads` 仍走原 FIFO 非优先级路径。
- `benches/simulation.rs` 新增 `world_fall_n_priority_pool`、`world_fall_n_work_stealing`、`world_fall_n_priority_work_stealing`。

### 9.2 最终 quick 输出

```text
# hvk-bench v1 quick=true
# name	value	unit	detail
math_vector4_add	0.0012	us/iter	iterations=100000
math_vector4_mul	0.0011	us/iter	iterations=100000
math_vector4_dot3	0.0014	us/iter	iterations=100000
math_vector4_normalize3	0.0015	us/iter	iterations=100000
math_quaternion_mul	0.0078	us/iter	iterations=100000
math_matrix3_mul_vec	0.0012	us/iter	iterations=100000
math_aabb_include_point	0.0025	us/iter	iterations=100000
broadphase_grid	0.024	ms/iter	bodies=64 pairs=156
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.032	ms/step	bodies=16 contacts=15
world_fall_n_parallel	0.059	ms/step	bodies=16 contacts=15
world_fall_n_persistent_pool	0.031	ms/step	bodies=16 contacts=15 threads=4 task-graph
world_fall_n_priority_pool	0.031	ms/step	bodies=16 contacts=15 threads=4 SchedulerConfig { executor: Pool, respect_priority: true }
world_fall_n_work_stealing	0.042	ms/step	bodies=16 contacts=15 threads=4 SchedulerConfig { executor: WorkStealing, respect_priority: false }
world_fall_n_priority_work_stealing	0.033	ms/step	bodies=16 contacts=15 threads=4 SchedulerConfig { executor: WorkStealing, respect_priority: true }
world_stack_10	0.011	ms/step	bodies=10 contacts=3
raycast_world	0.003	ms/iter	bodies=17 hits=201
shape_cast_world	0.001	ms/iter	bodies=1 hits=201
joint_chain_ragdoll	0.022	ms/step	bodies=9 joints=8
```

### 9.3 最终 full 输出

```text
# hvk-bench v1 quick=false
# name	value	unit	detail
math_vector4_add	0.0011	us/iter	iterations=2000000
math_vector4_mul	0.0012	us/iter	iterations=2000000
math_vector4_dot3	0.0013	us/iter	iterations=2000000
math_vector4_normalize3	0.0014	us/iter	iterations=2000000
math_quaternion_mul	0.0084	us/iter	iterations=2000000
math_matrix3_mul_vec	0.0012	us/iter	iterations=2000000
math_aabb_include_point	0.0027	us/iter	iterations=2000000
broadphase_grid	0.257	ms/iter	bodies=512 pairs=1906
gjk_sphere_pair	0.000	ms/iter	separated
gjk_box_pair	0.000	ms/iter	separated
narrowphase_box_pairs	0.001	ms/iter	overlapping
solver_contact_pair	0.000	ms/iter	2 bodies, 1 contact, 4 iterations
world_fall_n	0.084	ms/step	bodies=64 contacts=24
world_fall_n_parallel	0.128	ms/step	bodies=64 contacts=24
world_fall_n_persistent_pool	0.093	ms/step	bodies=64 contacts=24 threads=4 task-graph
world_fall_n_priority_pool	0.126	ms/step	bodies=64 contacts=24 threads=4 SchedulerConfig { executor: Pool, respect_priority: true }
world_fall_n_work_stealing	0.088	ms/step	bodies=64 contacts=24 threads=4 SchedulerConfig { executor: WorkStealing, respect_priority: false }
world_fall_n_priority_work_stealing	0.091	ms/step	bodies=64 contacts=24 threads=4 SchedulerConfig { executor: WorkStealing, respect_priority: true }
world_stack_10	0.018	ms/step	bodies=10 contacts=8
raycast_world	0.004	ms/iter	bodies=17 hits=2001
shape_cast_world	0.001	ms/iter	bodies=1 hits=2001
joint_chain_ragdoll	0.042	ms/step	bodies=25 joints=24
```

### 9.4 对比与结论

- full 场景中 work-stealing（0.088 / 0.091 ms/step）与持久 FIFO 池（0.093）同量级，priority FIFO（0.126）明显更慢；quick 场景各后端在 0.031–0.042 ms/step 间波动。没有稳定到足以改变默认后端的可归因加速。
- 优先级在本阶段仅影响同层提交顺序，且世界层 compute chunk 共享同一优先级，因此 priority FIFO 的额外开销未换来收益；保留为显式可选项，不设为默认。
- 确定性门禁：`tests/sleeping_and_parallel_solving.rs::scheduler_backends_match_single_threaded` 比较四种配置在 36 球场景的 120 步 trace，`explicit_executor_overrides_scheduler_backend` 验证显式执行器优先级；`tests/determinism.rs::scheduler_backends_match_scalar_golden` 断言四种配置 30 步 trace 等于 `GOLDEN_TRACE_30`；`src/base_thread` 新增优先级提交顺序与 work-stealing 队列/panic 单元测试。
- 结论：保留 `TaskExecutor` 抽象与两个可互换后端作为后续完整物理任务图的执行层；本阶段仍只宣称“与 Rust 单线程路径逐位一致”，不宣称与 C++、真实 `.hkx` 资产或真实 VDB 轨迹逐位等价。
