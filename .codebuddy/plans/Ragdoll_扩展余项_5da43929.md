---
name: Ragdoll 扩展余项
overview: 为 Rust 迁移项目规划下一步：补齐 Ragdoll 扩展的三个剩余能力——motion sharing、用户自定义状态、limited-hinge/6DOF 约束松弛，并给出分阶段实施、测试与文档更新方案。
todos:
  - id: verify-sdk-semantics
    content: 使用 [subagent:code-explorer] 考证 SDK 三项语义与修改点
    status: completed
  - id: add-joint-families
    content: 新增 limited-hinge/6DOF 关节行与世界槽 API
    status: completed
    dependencies:
      - verify-sdk-semantics
  - id: generalize-easing
    content: 泛化 easing 支持新关节族限位保存与回弹
    status: completed
    dependencies:
      - add-joint-families
  - id: implement-motion-sharing
    content: 实现 keyframed 共享运动与生命周期管理
    status: completed
    dependencies:
      - verify-sdk-semantics
  - id: support-custom-states
    content: 加入用户自定义状态注册与调度
    status: completed
    dependencies:
      - verify-sdk-semantics
  - id: tests-docs-validation
    content: 补集成测试并更新 README 与全量验证
    status: completed
    dependencies:
      - generalize-easing
      - implement-motion-sharing
      - support-custom-states
---

## User Requirements

用户要求读取项目进度后，先为“下一步”制定计划，且当前阶段只计划、不执行修改。澄清后确定下一步方向为“Ragdoll 扩展余项”，即承接最近四个 ragdoll 子集，补齐 README 第 8 节未完成项中的三项能力：motion sharing、用户自定义状态、limited-hinge/6DOF 约束松弛。

## Product Overview

本任务为 Rust 物理引擎的 Ragdoll 扩展层增加三类行为能力，不涉及界面改动。完成后，开发者可将多个布娃娃骨骼 body 挂接到共享运动、注册并使用自定义 ragdoll 状态、对 limited-hinge 与 6DOF 关节执行与原版一致的“保存当前受限值、按当前姿态放宽、按 SDK 权重回弹”流程。效果体现在仿真行为与确定性 trace 中，而非视觉界面。

## Core Features

- 共享运动：允许一组 keyframed body 复用同一 motion，正确维护变换同步、运动类型切换与销毁时的引用生命周期。
- 用户自定义状态：在现有 Keyframed/Dynamic/Powered 之外注册用户状态，支持状态 ID、默认 bone strength 标签掩码、进入/退出与逐步驱动钩子，以及排队切换。
- 扩展 easing：新增 limited-hinge 与 6DOF 世界级关节，并把 `EaseConstraintsAction` 从 ragdoll cone/twist 三值泛化到多关节族限位保存、按姿态放宽和确定性回弹。
- 沿用既有交付标准：单元测试、集成测试、`hvk-trace v1` 逐位确定性验证、README 状态与决策记录、全量检查命令通过。

[object Object]]]>

## Agent Extensions

### SubAgent

- **code-explorer**
- Purpose: 对 Rust 侧关节/easing/controller/motion 管理与原版 SDK 的 limited-hinge、6DOF、ragdoll state、attachBodies 语义做跨目录考证，确认精确修改点与取舍边界。
- Expected outcome: 输出逐文件的现状结论、SDK 对应行为摘录、三项子任务的依赖顺序与风险清单，供后续实现直接落地。