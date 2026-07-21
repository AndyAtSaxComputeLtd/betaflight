/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/axis.h"

typedef enum {
    AUTO_ACRO_MANEUVER_NONE = 0,
    AUTO_ACRO_MANEUVER_ROLL,
    AUTO_ACRO_MANEUVER_FLIP,
    AUTO_ACRO_MANEUVER_POWER_LOOP,
    AUTO_ACRO_MANEUVER_YAW,
} autoAcroManeuver_e;

void autoAcroUpdateTriggers(void);
void autoAcroUpdate(const float gyroRates[XYZ_AXIS_COUNT], float dT);
bool autoAcroGetSetpoint(flight_dynamics_index_t axis, float *setpoint);
bool autoAcroShouldOverrideThrottle(void);
float autoAcroGetThrottle(void);
autoAcroManeuver_e autoAcroGetManeuver(void);
