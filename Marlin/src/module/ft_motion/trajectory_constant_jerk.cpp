/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#if __has_include("../../inc/MarlinConfig.h")
  #include "../../inc/MarlinConfig.h"
#endif

#ifndef ENABLED
  #define ENABLED(V) true
#endif

#if ENABLED(FTM_CONSTANT_JERK)

#include "trajectory_constant_jerk.h"

void ConstantJerkTrajectoryGenerator::plan(
    const float, const float, const float, const float, const float) {
  planner_.planNext(*this, *jerk_max_ptr_);
}

void ConstantJerkTrajectoryGenerator::planRunout(const float duration) {
  planner_.resetPlannerState();
  reset();
  // Cruise at zero speed for the entire duration (same as trapezoidal)
  phase_dt[3] = duration;
  total_duration = duration;
  buildPhaseCache();
}

bool ConstantJerkTrajectoryGenerator::planNext(float j_max) {
  return planner_.planNext(*this, j_max);
}

#endif // FTM_CONSTANT_JERK
