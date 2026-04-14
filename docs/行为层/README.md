# 行为层文档索引

行为层对应 `include/xsf_behavior/`。

当前目录只包含一个已落地子域：

- `飞行/`
  飞行相关行为，包括飞行状态、原子控制器和原生程序层。

其他行为子域，例如导弹制导行为、雷达探测行为、传感器管理行为，当前尚未进入行为层目录。

## 设计目标

- 不引入完整仿真内核
- 不负责状态推进和任务调度
- 不维护实体生命周期
- 只输出俯仰、滚转、航向率、垂向速度、过载等控制意图

## 当前已实现范围

### 1. 飞行原生程序层

对应 `include/xsf_behavior/flight/guidance_programs.hpp`。

当前已迁入的程序：

- `legacy_guidance_program`
- `altitude_guidance_program`
- `intercept_guidance_program`
- `legacy_flight_path_angle_program`
- `gravity_bias_program`
- `gravity_turn_program`
- `attitude_guidance_program`
- `flight_path_angle_guidance_program`
- `orbit_insertion_program`

### 2. 飞行原子控制器层

对应 `include/xsf_behavior/flight/basic_controllers.hpp`。

- `pull_up_controller`
- `coordinated_turn_controller`
- `descent_controller`
- `level_hold_controller`
- `waypoint_track_controller`
- `approach_glideslope_controller`
- `heading_hold_controller`
- `flare_controller`

## 阅读路径

1. `飞行/README.md`
2. `飞行/飞行器飞行心智模型.md`
3. `飞行/原生制导程序层.md`
4. `飞行/原子控制器总览.md`
5. `飞行/飞行行为工作流.md`

## 与算法层的边界

行为层依赖算法层的：

- 大气与速度/动压估算
- 常量和基础数学
- 坐标与几何量表达

行为层新增的是：

- 动作目标结构
- 动作约束结构
- 动作输出命令
- 飞行原子控制器之间的工作流

算法层的详细解释见：

- `../算法层/README.md`
- `../算法层/算法链路与逻辑详解.md`
- `../算法层/专题知识/`
