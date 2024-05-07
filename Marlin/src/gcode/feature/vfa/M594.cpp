/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../../inc/MarlinConfig.h"

#if HAS_ZV_SHAPING

#include "../../gcode.h"
#include "../../../module/stepper.h"

void GcodeSuite::M594_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F("VFA"));
  #if ENABLED(VFA_X)
    SERIAL_ECHOLNPGM("  M594 X"
      " A", stepper.get_vfa_amplitude(X_AXIS),
      " P", stepper.get_vfa_phase(X_AXIS),
      " W", stepper.get_vfa_width(X_AXIS)
    );
  #endif
  #if ENABLED(VFA_Y)
    TERN_(VFA_Y, report_echo_start(forReplay));
    SERIAL_ECHOLNPGM("  M594 Y"
      " A", stepper.get_vfa_amplitude(Y_AXIS),
      " P", stepper.get_vfa_phase(Y_AXIS),
      " W", stepper.get_vfa_width(Y_AXIS)
    );
  #endif
}

/**
 * M594: Get or Set Input Shaping Parameters
 *  D<factor>    Set the zeta/damping factor. If axes (X, Y, etc.) are not specified, set for all axes.
 *  F<frequency> Set the frequency. If axes (X, Y, etc.) are not specified, set for all axes.
 *  T[map]       Input Shaping type, 0:ZV, 1:EI, 2:2H EI (not implemented yet)
 *  X            Set the given parameters only for the X axis.
 *  Y            Set the given parameters only for the Y axis.
 */
void GcodeSuite::M594() {
  if (!parser.seen_any()) return M594_report();

  const bool seen_X = TERN0(VFA_X, parser.seen_test('X')),
             seen_Y = TERN0(VFA_Y, parser.seen_test('Y')),
             for_X = seen_X || TERN0(VFA_X, (!seen_X && !seen_Y)),
             for_Y = seen_Y || TERN0(VFA_Y, (!seen_X && !seen_Y));

  if (parser.seen('A')) {
    const float amplitude = parser.value_float();
    if (WITHIN(amplitude, 0, 1)) {
      if (for_X) stepper.set_vfa_amplitude(X_AXIS, amplitude);
      if (for_Y) stepper.set_vfa_amplitude(Y_AXIS, amplitude);
    }
    else
      SERIAL_ECHO_MSG("?Amplitude (A) value out of range (0-1)");
  }

  if (parser.seen('P')) {
    const float phase = parser.value_float();
    if (WITHIN(phase, 0, 1)) {
      if (for_X) stepper.set_vfa_phase(X_AXIS, phase);
      if (for_Y) stepper.set_vfa_phase(Y_AXIS, phase);
    }
    else
      SERIAL_ECHO_MSG("?Phase (A) value out of range (0-1)");
  }
  
  if (parser.seen('W')) {
    const float width = parser.value_float();
    if (WITHIN(width, 0, 1024)) {
      if (for_X) stepper.set_vfa_width(X_AXIS, width);
      if (for_Y) stepper.set_vfa_width(Y_AXIS, width);
    }
    else
      SERIAL_ECHO_MSG("?Width (A) value out of range (0-1024)");
  }

  
}

#endif
