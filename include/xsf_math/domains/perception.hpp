#pragma once

// 感知与探测业务域入口 (Perception and detection domain header)
// Includes radar, infrared, ESM/ELINT, sonar, and propagation models

#include "../core/rcs.hpp"
#include "../rcs/simple_geometries.hpp"
#include "../radar/cfar.hpp"
#include "../radar/infrared.hpp"
#include "../radar/antenna.hpp"
#include "../radar/propagation.hpp"
#include "../radar/radar_equation.hpp"
#include "../radar/clutter.hpp"
#include "../radar/marcum_swerling.hpp"
#include "../ew/esm_elint.hpp"
#include "../sonar/sonar.hpp"
