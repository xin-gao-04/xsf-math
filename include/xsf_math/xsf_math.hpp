#pragma once

// XSF-Math：用于国防/航天建模的头文件库
// 从 XSF-Core 仿真框架中抽离并解耦
//
// 说明：
// 1. `core/radar/tracking/guidance/aero/ew/lethality/orbital`
//    主要属于“数学算法层”或“领域计算层”，侧重公式、模型和数值计算。
// 2. `xsf_behavior`
//    属于“行为控制层”，侧重把底层算法组织成可供外部仿真框架调用的原子控制器。
//    这一层不负责状态推进、任务编排或实体生命周期，只输出控制意图。

// === 数学算法层：核心 ===
#include "core/constants.hpp"
#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/interpolation.hpp"
#include "core/coordinate_transform.hpp"
#include "core/atmosphere.hpp"
#include "core/rcs.hpp"

// === 数学算法层：雷达 ===
#include "radar/marcum_swerling.hpp"
#include "radar/antenna.hpp"
#include "radar/propagation.hpp"
#include "radar/radar_equation.hpp"
#include "radar/clutter.hpp"

// === 数学算法层：跟踪 ===
#include "tracking/kalman_filter.hpp"
#include "tracking/track_association.hpp"

// === 数学算法层：制导 ===
#include "guidance/proportional_nav.hpp"

// === 行为控制层：飞行动作 ===
// 基于上面的数学算法和气动/大气状态，输出俯仰、滚转、过载等控制指令。
// 文件层级上，这一层已提升到独立的 `include/xsf_behavior/` 目录。
#include <xsf_behavior/xsf_behavior.hpp>

// === 数学算法层：气动 ===
#include "aero/aerodynamics.hpp"

// === 数学算法层：电子战 ===
#include "ew/electronic_warfare.hpp"

// === 数学算法层：杀伤效能 ===
#include "lethality/fuze.hpp"
#include "lethality/pk_model.hpp"
#include "lethality/launch_pk_table.hpp"

// === 数学算法层：轨道力学 ===
#include "orbital/kepler.hpp"
#include "orbital/j2.hpp"
#include "orbital/maneuvers.hpp"
