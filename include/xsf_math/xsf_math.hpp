#pragma once

// XSF-Math：用于国防/航天建模的头文件库。
// 算法层同时保留两套入口：
// 1. `core/radar/tracking/guidance/aero/ew/lethality/orbital`
//    技术模块视图。
// 2. `domains/*`
//    按业务链路组织的业务域视图。
// `xsf_behavior`
// 当前已覆盖 flight / sensor / tracking / engagement / ew / orbital 六个行为子域，
// 侧重输出可被外部框架消费的控制命令。

// === 数学算法层：业务域入口 ===
#include "domains/foundation.hpp"
#include "domains/perception.hpp"
#include "domains/tracking.hpp"
#include "domains/engagement.hpp"
#include "domains/flight_dynamics.hpp"
#include "domains/electronic_warfare.hpp"
#include "domains/mission_systems.hpp"
#include "domains/orbital_dynamics.hpp"

// === 行为控制层：飞行动作 ===
#include <xsf_behavior/xsf_behavior.hpp>
