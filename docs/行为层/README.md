# 行为层文档索引

行为层对应 `include/xsf_behavior/`，现在分成两个逻辑子层：

- 原生制导程序层：尽量按 `xsf-core` 的 `GuidanceProgram` 语义提取
- 原子控制器层：面向外部框架直接托管的轻量动作控制器

## 设计目标

- 不引入完整仿真内核
- 不负责状态推进和任务调度
- 不维护实体生命周期
- 只输出俯仰、滚转、航向率、垂向速度、过载等控制意图

## 当前层级

### 1. 原生制导程序层

对应 `include/xsf_behavior/guidance_programs.hpp`。

当前已迁入的第一批程序：

- `legacy_guidance_program`
- `altitude_guidance_program`
- `intercept_guidance_program`
- `legacy_flight_path_angle_program`
- `gravity_bias_program`
- `gravity_turn_program`

### 2. 原子控制器层

对应 `include/xsf_behavior/basic_controllers.hpp`。

- `pull_up_controller`
- `coordinated_turn_controller`
- `descent_controller`
- `level_hold_controller`
- `waypoint_track_controller`
- `approach_glideslope_controller`
- `heading_hold_controller`
- `flare_controller`

## 推荐阅读

1. `原生制导程序层.md`
2. `原子控制器总览.md`
3. `飞行行为工作流.md`

## 与算法层的边界

行为层依赖算法层的：

- 大气与速度/动压估算
- 常量和基础数学
- 坐标与几何量表达

行为层新增的是：

- 动作目标结构
- 动作约束结构
- 动作输出命令
- 原子控制器之间的推荐工作流

算法层的详细解释见：

- `../算法层/README.md`
- `../算法层/算法链路与逻辑详解.md`
- `../算法层/专题知识/`
