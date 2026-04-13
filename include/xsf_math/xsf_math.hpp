#pragma once

// XSF-Math: Header-only computation library for defense/aerospace modeling
// Extracted and decoupled from XSF-Core simulation framework

// === Core ===
#include "core/constants.hpp"
#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/interpolation.hpp"
#include "core/coordinate_transform.hpp"
#include "core/atmosphere.hpp"
#include "core/rcs.hpp"

// === Radar ===
#include "radar/marcum_swerling.hpp"
#include "radar/antenna.hpp"
#include "radar/propagation.hpp"
#include "radar/radar_equation.hpp"
#include "radar/clutter.hpp"

// === Tracking ===
#include "tracking/kalman_filter.hpp"
#include "tracking/track_association.hpp"

// === Guidance ===
#include "guidance/proportional_nav.hpp"

// === Aerodynamics ===
#include "aero/aerodynamics.hpp"

// === Electronic Warfare ===
#include "ew/electronic_warfare.hpp"

// === Lethality ===
#include "lethality/fuze.hpp"
#include "lethality/pk_model.hpp"

// === Orbital Mechanics ===
#include "orbital/kepler.hpp"
