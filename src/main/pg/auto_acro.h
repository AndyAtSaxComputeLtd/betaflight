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
#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    AUTO_ACRO_HIGH_POWER_LOOP = 0,
    AUTO_ACRO_HIGH_YAW,
} autoAcroHighManeuver_e;

typedef struct autoAcroConfig_s {
    bool roll;
    bool flip;
    bool powerLoop;
    bool yaw;
    uint16_t rollSpeed;
    uint16_t flipSpeed;
    uint16_t powerLoopSpeed;
    uint16_t yawSpeed;
    uint8_t powerLoopThrottle;
    uint8_t rollTrickAux;
    uint8_t flipTrickAux;
    uint8_t powerLoopTrickAux;
    uint8_t highManeuver;
    uint8_t speedDamperAux;
} autoAcroConfig_t;

PG_DECLARE(autoAcroConfig_t, autoAcroConfig);
