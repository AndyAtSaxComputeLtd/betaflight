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

#include "platform.h"

#ifdef USE_AUTOACRO

#include "fc/rc_controls.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "auto_acro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autoAcroConfig_t, autoAcroConfig, PG_AUTO_ACRO_CONFIG, 4);

PG_RESET_TEMPLATE(autoAcroConfig_t, autoAcroConfig,
    .roll = true,
    .flip = true,
    .powerLoop = true,
    .yaw = true,
    .rollSpeed = 720,
    .flipSpeed = 720,
    .powerLoopSpeed = 360,
    .yawSpeed = 360,
    .powerLoopThrottle = 75,
    .rollTrickAux = AUX9,
    .flipTrickAux = AUX10,
    .powerLoopTrickAux = AUX11,
    .highManeuver = AUTO_ACRO_HIGH_POWER_LOOP,
    .speedDamperAux = 15,
);

#endif // USE_AUTOACRO
