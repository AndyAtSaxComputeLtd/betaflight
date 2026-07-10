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

#ifndef EVENTLOG_DEFAULT_ENABLED
#define EVENTLOG_DEFAULT_ENABLED true
#endif

#ifndef EVENTLOG_DEFAULT_SIZE_KB
#define EVENTLOG_DEFAULT_SIZE_KB 256
#endif

#ifndef EVENTLOG_DEFAULT_GPS_LOGGING
#define EVENTLOG_DEFAULT_GPS_LOGGING false
#endif

typedef struct eventlogConfig_s {
    bool enabled;
    bool gpsLoggingEnabled;
    uint16_t sizeKb;
} eventlogConfig_t;

PG_DECLARE(eventlogConfig_t, eventlogConfig);
