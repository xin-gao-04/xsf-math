# 算法层文档索引

算法层对应 `include/xsf_math/`，负责数学模型、物理量计算、领域公式和工程估算能力。

它的角色不是“把所有公式散着放在头文件里”，而是把上层仿真最常用的计算问题拆成一组可独立调用的算法链：

- 几何与坐标链
- 探测与传播链
- 跟踪与制导链
- 气动与能力约束链
- 引信与杀伤评估链
- 轨道传播与机动链

## 公开入口

- 聚合入口：`#include <xsf_math/xsf_math.hpp>`
- 业务域入口：按业务域包含 `domains/foundation.hpp`、`domains/perception.hpp`、`domains/tracking.hpp`、`domains/engagement.hpp`、`domains/flight_dynamics.hpp`、`domains/electronic_warfare.hpp`、`domains/orbital_dynamics.hpp`
- 技术模块入口：按模块包含 `core/`、`radar/`、`tracking/`、`guidance/`、`aero/`、`ew/`、`lethality/`、`orbital/`

## 算法层的核心逻辑

### 1. 基础层先统一“量”的表达

`core` 不是杂项集合，而是所有上层模块共享的量纲和几何基础：

- `constants`
  统一物理常数和角度换算
- `vec3 / mat3`
  统一空间向量和旋转表达
- `interpolation`
  统一查表和工程数据曲线
- `coordinate_transform`
  统一坐标系之间的转换
- `atmosphere`
  统一高度到密度、音速、动压的映射
- `rcs`
  统一目标散射截面的数据和模型入口

这意味着上层模块几乎都不是“从零开始算”，而是在同一套基础表达上继续推导。

### 2. 领域层解决“链路问题”

算法层真正面向的是一条条工程链路，而不是单个公式：

- `radar`
  解决“目标能不能被看见、在什么条件下能看见”
- `tracking`
  解决“量测来了以后如何稳健估计目标状态”
- `guidance`
  解决“已知弹目几何关系后该如何产生机动加速度”
- `aero`
  解决“平台当前有没有能力执行这个机动”
- `ew`
  解决“干扰和诱饵如何改变探测/识别条件”
- `lethality`
  解决“接近之后是否触发、是否命中、命中后概率多大”
- `orbital`
  解决“在轨目标如何传播、如何做长期摄动和常见机动”

### 3. 算法层的价值在于“可组合”

单个模块本身并不代表完整作战流程，但它们组合起来能形成完整的计算闭环。例如：

1. `core + radar`
   形成探测与传播闭环
2. `tracking + guidance + aero`
   形成估计与制导闭环
3. `guidance + lethality`
   形成拦截到杀伤闭环
4. `core + orbital`
   形成轨道传播与机动闭环

这也是算法层需要详细文档的原因：重点不是“某个函数怎么调”，而是“这些模块为什么要这样配合”。

## 业务域总览

- `基础支撑`
  对应 `domains/foundation.hpp`，统一常量、向量/矩阵、插值、坐标变换和大气基础量。
- `感知与探测`
  对应 `domains/perception.hpp`，统一 RCS、传播、天线、雷达方程、杂波和统计检测。
- `跟踪估计`
  对应 `domains/tracking.hpp`，统一滤波与关联。
- `交战与杀伤`
  对应 `domains/engagement.hpp`，统一导引、引信、Pk 和发射杀伤概率表。
- `飞行与气动`
  对应 `domains/flight_dynamics.hpp`，统一大气与气动能力计算。
- `电子战`
  对应 `domains/electronic_warfare.hpp`，统一干扰、诱饵和干信比退化。
- `轨道动力学`
  对应 `domains/orbital_dynamics.hpp`，统一开普勒传播、J2 摄动和常见轨道机动。

## 技术模块总览

- `core`
  常量、向量/矩阵、插值、坐标变换、大气和 RCS 基础表达。
- `radar`
  天线、传播、杂波、雷达方程、Marcum-Swerling 探测。
- `tracking`
  卡尔曼滤波和量测关联。
- `guidance`
  比例导引及相关导引计算。
- `aero`
  气动、动压、能力限制和飞行性能估算。
- `ew`
  干扰、诱饵和电子战效应建模。
- `lethality`
  引信、PCA、Pk 和发射杀伤概率表。
- `orbital`
  开普勒轨道、J2 摄动和常见轨道机动。

## 阅读路径

1. `模块依赖图谱.md`
2. `算法链路与逻辑详解.md`
3. `基础支撑/README.md`
4. `感知与探测/README.md`
5. `交战与杀伤/README.md`
6. `轨道动力学/README.md`

## 基础知识补充

每个业务域目录下均增加了 `基础知识整理.md`，用于在阅读代码前先统一：

- 常用术语
- 关键公式的物理含义
- 当前库中相关头文件的职责分工
- 常见误区
- 外部参考资料

## 与行为层的边界

算法层不负责：

- 动作阶段切换
- 飞机/导弹的行为编排
- 控制意图输出
- 平飞、转弯、进近、拉平等动作语义

这些内容统一放到 `include/xsf_behavior/` 和 `docs/行为层/` 下描述。
