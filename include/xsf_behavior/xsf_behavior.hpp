#pragma once

// 行为控制层聚合入口（Behavior Control Layer Aggregate Header）
// 行为层按业务子域分组，每个子域输出"当前帧的意图"，不修改仿真状态（Behavior layer grouped by business subdomain; each subdomain outputs the "intent of the current frame" without modifying the simulation state）
//
// - flight      飞行与制导阶段控制（Flight and Guidance Phase Control）
// - sensor      雷达调度与探测决策（Radar Scheduling and Detection Decision）
// - tracking    航迹起始、关联、确认、丢弃（Track Initiation, Association, Confirmation, and Dropping）
// - engagement  发射计算、制导/引信组合、交战状态机（Launch Computation, Guidance/Fuze Combination, Engagement State Machine）
// - ew          干扰方式选择与 EW 技术库（Jamming Mode Selection and EW Technique Library）
// - orbital     轨道机动序列规划（Orbital Maneuver Sequence Planning）

#include "flight/flight_state.hpp"       // 飞行状态聚合（Flight State Aggregation）
#include "flight/basic_controllers.hpp"  // 基础飞行控制器（Basic Flight Controllers）：高度/速度/航向保持
#include "flight/guidance_programs.hpp"  // 制导程序（Guidance Programs）：航路点跟踪

#include "sensor/sensor_schedule.hpp"    // 传感器调度（Sensor Schedule）：多模式时间片分配
#include "sensor/detection_controller.hpp" // 探测控制器（Detection Controller）：信噪比评估与检测判决

#include "tracking/track_manager.hpp"    // 航迹管理器（Track Manager）：KF 预测+关联+更新+淘汰
#include "tracking/track_initiator.hpp"  // 航迹起始器（Track Initiator）：M/N 逻辑确认新航迹

#include "engagement/launch_computer.hpp"    // 发射计算机（Launch Computer）：发射包络与命中概率
#include "engagement/fuze_controller.hpp"    // 引信控制器（Fuze Controller）：起爆时机控制
#include "engagement/engagement_controller.hpp" // 交战控制器（Engagement Controller）：六阶段状态机
#include "engagement/weapon_assignment.hpp"  // 武器分配器（Weapon Assignment）：威胁优先级与匹配

#include "ew/jam_assignment.hpp"         // 干扰分配器（Jam Assignment）：频率匹配与功率裕度
#include "ew/ew_technique_controller.hpp" // EW 技术控制器（EW Technique Controller）：技术库注册与效果合成

#include "orbital/maneuver_planner.hpp"  // 轨道机动规划器（Orbital Maneuver Planner）：脉冲/连续推力序列
