#pragma once

// 行为控制层聚合入口。
// 行为层按业务子域分组，每个子域输出“当前帧的意图”，不修改仿真状态。
//
// - flight      飞行与制导阶段控制
// - sensor      雷达调度与探测决策
// - tracking    航迹起始、关联、确认、丢弃
// - engagement  发射计算、制导/引信组合、交战状态机
// - ew          干扰方式选择与 EW 技术库
// - orbital     轨道机动序列规划

#include "flight/flight_state.hpp"
#include "flight/basic_controllers.hpp"
#include "flight/guidance_programs.hpp"

#include "sensor/sensor_schedule.hpp"
#include "sensor/detection_controller.hpp"

#include "tracking/track_manager.hpp"
#include "tracking/track_initiator.hpp"

#include "engagement/launch_computer.hpp"
#include "engagement/fuze_controller.hpp"
#include "engagement/engagement_controller.hpp"
#include "engagement/weapon_assignment.hpp"

#include "ew/jam_assignment.hpp"
#include "ew/ew_technique_controller.hpp"

#include "orbital/maneuver_planner.hpp"
