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
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

/*
 * Event logger — runs alongside the blackbox on the same SD card.
 *
 * Files:  log_XXXX.txt  (4-digit, rolls at 9999)
 * Format: CSV, 1 record per state-change, rate-limited to 1 record/second.
 *
 * CSV columns:
 *   row, timestamp_ms, gps_time, event, detail,
 *   lat_deg7, lon_deg7, lat_dd, lon_dd, google_maps_url
 */

// Call once after SD card / filesystem is ready.
void eventlogInit(void);

// Call every FC task iteration (e.g. from taskMainPidLoop or scheduler task).
void eventlogUpdate(timeUs_t currentTimeUs);

// Explicitly flush buffered data to disk (call on disarm / shutdown).
void eventlogFlush(void);

// Close the current log file (call on power-down path).
void eventlogClose(void);

// Add an explicit diagnostic row. Safe to call before eventlogInit(); a small
// early-boot backlog is flushed once the logger is ready.
void eventlogAdd(const char *event, const char *detail);
