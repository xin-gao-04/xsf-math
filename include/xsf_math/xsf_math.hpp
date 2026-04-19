#pragma once

// XSF-Math：国防/航天建模头文件库（Defense/Aerospace Modeling Header-Only Library）
// 算法层同时保留两套入口（The algorithm layer maintains two entry points）：
// 1. `core/radar/tracking/guidance/aero/ew/lethality/orbital`
//    技术模块视图（Technical Module View）：按算法类别组织的原始接口
// 2. `domains/*`
//    业务域视图（Domain View）：按业务链路（杀伤链、OODA 环）组织的聚合接口
//
// `xsf_behavior`（行为控制层，Behavior Control Layer）
// 当前已覆盖 flight（飞行）、sensor（传感器）、tracking（跟踪）、engagement（交战）、
// ew（电子战）、orbital（轨道）六个行为子域（Behavior Subdomains），
// 侧重输出可被外部仿真框架消费的控制命令（Control Commands），不直接修改仿真状态

// === 数学算法层：业务域入口（Algorithm Layer: Domain Entry Points） ===
#include "domains/foundation.hpp"        // 基础域（Foundation Domain）：向量、坐标变换、常数
#include "domains/perception.hpp"        // 感知域（Perception Domain）：雷达、红外、ESM
#include "domains/tracking.hpp"          // 跟踪域（Tracking Domain）：数据关联、滤波、航迹管理
#include "domains/engagement.hpp"        // 交战域（Engagement Domain）：制导律、引信、杀伤评估
#include "domains/flight_dynamics.hpp" // 飞行动力学域（Flight Dynamics Domain）：六自由度、气动、控制
#include "domains/electronic_warfare.hpp" // 电子战域（Electronic Warfare Domain）：干扰、欺骗、ESM
#include "domains/mission_systems.hpp"   // 任务系统域（Mission Systems Domain）：导航、通信、水声
#include "domains/orbital_dynamics.hpp"// 轨道动力学域（Orbital Dynamics Domain）：SGP4、Lambert、再入

// === 行为控制层：飞行动作（Behavior Control Layer: Flight Actions） ===
#include <xsf_behavior/xsf_behavior.hpp>
