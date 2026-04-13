# xsf-math 文档索引

本文档目录面向 `xsf-math` 这一头文件数学/建模库，目标是把常用能力、模块边界、集成方式和专题知识沉淀为可直接阅读的项目文档。

## 文档列表

- `架构与构建分析.md`
  当前仓库的定位、构建方式、目录结构和示例/测试组织方式。
- `模块依赖图谱.md`
  各数学模块的职责边界、依赖方向和推荐使用入口。
- `集成指南.md`
  如何把 `xsf-math` 作为头文件库接入上层工程，以及推荐的最小使用面。

## 专题知识

- `knowledge/坐标系统与变换.md`
- `knowledge/雷达信号处理.md`
- `knowledge/Marcum-Swerling探测.md`
- `knowledge/雷达散射截面与电磁散射.md`
- `knowledge/比例导引.md`
- `knowledge/气动与飞行.md`
- `knowledge/电子战.md`
- `knowledge/引信与PCA.md`
- `knowledge/杀伤效能与Pk.md`
- `knowledge/跟踪与滤波.md`
- `knowledge/轨道力学.md`

## 示例与验证入口

- `examples/radar_detection_example.cpp`
- `examples/missile_engagement_example.cpp`
- `examples/orbital_example.cpp`
- `tests/test_core.cpp`
- `tests/test_radar.cpp`
- `tests/test_guidance.cpp`
- `tests/test_algorithm_io.cpp`

## 建议阅读顺序

1. 先看 `集成指南.md`，确认仓库的接入方式和最小使用入口。
2. 再看 `模块依赖图谱.md`，建立模块边界和依赖认知。
3. 然后参考 `架构与构建分析.md`，理解构建方式、示例和测试布局。
4. 最后按需要进入 `knowledge/` 下的专题文档。
