#pragma once

// XSF-Math：用于国防/航天建模的头文件库
// 从 XSF-Core 仿真框架中抽离并解耦

// === 核心 ===
#include "core/constants.hpp"
#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/interpolation.hpp"
#include "core/coordinate_transform.hpp"
#include "core/atmosphere.hpp"
#include "core/rcs.hpp"

// === 雷达 ===
#include "radar/marcum_swerling.hpp"
#include "radar/antenna.hpp"
#include "radar/propagation.hpp"
#include "radar/radar_equation.hpp"
#include "radar/clutter.hpp"

// === 跟踪 ===
#include "tracking/kalman_filter.hpp"
#include "tracking/track_association.hpp"

// === 制导 ===
#include "guidance/proportional_nav.hpp"

// === 气动 ===
#include "aero/aerodynamics.hpp"

// === 电子战 ===
#include "ew/electronic_warfare.hpp"

// === 杀伤效能 ===
#include "lethality/fuze.hpp"
#include "lethality/pk_model.hpp"
#include "lethality/launch_pk_table.hpp"

// === 轨道力学 ===
#include "orbital/kepler.hpp"
#include "orbital/j2.hpp"
#include "orbital/maneuvers.hpp"
