# 杀伤效能与 Pk

> 本文对应 `include/xsf_math/lethality/pk_model.hpp`。

## 1. 当前实现能力

当前杀伤效能模块包括：

- `pk_curve`
- `kill_assessment`
- `single_shot_pk(...)`
- `cumulative_pk(...)`
- `monte_carlo_kill`
- `target_class`
- `typical_lethal_radius(...)`
- `ew_degraded_miss_distance(...)`

## 2. Pk 曲线

`pk_curve` 用于表达：

- 脱靶量越小，杀伤概率越高
- 随距离增加，Pk 平滑衰减

当前内置生成器包括：

- `blast_fragmentation(...)`
- `continuous_rod(...)`

这适合快速形成战斗部效能曲线。

## 3. 单发与累计 Pk

当前库支持两种常见统计视角：

- 单发杀伤概率
- 多次独立射击下的累计杀伤概率

对于上层应用，这两种能力足以支撑：

- 单次拦截评估
- 齐射或多轮交战评估

## 4. 蒙特卡洛杀伤判定

`monte_carlo_kill` 提供轻量随机试验能力，可用于：

- 基于 Pk 的离散化样本仿真
- 快速统计重复试验结果

当前示例中已经使用它来估算交战场景下的命中/杀伤比例。

## 5. 与 EW 的关系

`ew_degraded_miss_distance(...)` 体现了一个直接工程假设：

- 干扰会恶化跟踪精度
- 跟踪误差增大会映射成更大的脱靶量
- 脱靶量增加再通过 `pk_curve` 映射成更低 Pk

这让电子战和杀伤评估之间有了一个简单清晰的接口。

## 6. 相关源码

- `include/xsf_math/lethality/pk_model.hpp`
- `examples/missile_engagement_example.cpp`
- `tests/test_guidance.cpp`
