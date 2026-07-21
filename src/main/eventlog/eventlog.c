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

/*
 * eventlog.c
 *
 * Custom event logger that runs alongside the Betaflight blackbox.
 *
 * Files are stored in the same SD card directory as the blackbox logs.
 * Filename:  log_XXXX.txt   (XXXX = 0001–9999, wraps to 0001)
 *
 * CSV format per line:
 *   row, uptime, gps_time, event, detail,
 *   lat_deg7, lon_deg7, lat_dd, lon_dd, google_maps_url
 *
 * Rate-limiting: only one log entry is written per second, per event type,
 * unless the state has changed (state-change events are always written).
 */

#include "platform.h"

#ifdef USE_EVENTLOG

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "eventlog.h"

#ifdef USE_FLASHFS
#include "eventlog/eventlog_flash.h"
#endif

#ifdef USE_BLACKBOX
#include "blackbox/blackbox.h"
#endif

#include "common/maths.h"
#include "common/printf.h"
#include "common/strtol.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "io/beeper.h"
#ifdef USE_GPS
#include "io/gps.h"
#endif
#ifdef USE_SDCARD
#include "io/asyncfatfs/asyncfatfs.h"
#endif

#include "pg/beeper.h"
#include "pg/eventlog.h"
#include "sensors/battery.h"

#ifdef USE_RTC_TIME
#include "common/time.h"
#endif

// --------------------------------------------------------------------------
// Configuration
// --------------------------------------------------------------------------

#define EVENTLOG_DIR            "."          // same dir as blackbox logs
#define EVENTLOG_PREFIX         "log_"
#define EVENTLOG_SUFFIX         ".txt"
#define EVENTLOG_MAX_NUMBER     9999
#define EVENTLOG_RATE_LIMIT_MS  1000         // minimum ms between repeated messages
#define EVENTLOG_PENDING_COUNT  32
#define EVENTLOG_EVENT_LEN      20
#define EVENTLOG_DETAIL_LEN     96

// Columns written to every row
#define GMAPS_URL_PREFIX        "https://maps.google.com/?q="

static const char eventlogCsvHeader[] =
    "row,uptime,gps_time,event,detail,"
    "lat_deg7,lon_deg7,lat_dd,lon_dd,google_maps_url\r\n";

// --------------------------------------------------------------------------
// Private state
// --------------------------------------------------------------------------

typedef enum {
    EVENTLOG_BACKEND_NONE = 0,
    EVENTLOG_BACKEND_SDCARD,
    EVENTLOG_BACKEND_FLASH,
} eventlogBackend_e;

static eventlogBackend_e eventlogBackend = EVENTLOG_BACKEND_NONE;
#ifdef USE_SDCARD
static afatfsFilePtr_t  eventlogFile   = NULL;
static bool             eventlogOpening = false;  // async open in progress
#endif
static bool             eventlogReady  = false;
static uint32_t         rowNumber      = 0;
static timeMs_t         eventlogTimeOffsetMs = 0;

typedef struct eventlogPending_s {
    char event[EVENTLOG_EVENT_LEN];
    char detail[EVENTLOG_DETAIL_LEN];
} eventlogPending_t;

static eventlogPending_t eventlogPending[EVENTLOG_PENDING_COUNT];
static uint8_t eventlogPendingHead = 0;
static uint8_t eventlogPendingCount = 0;

// Last-seen state for change detection
static bool             lastArmed             = false;
static batteryState_e   lastBatteryState      = BATTERY_INIT;
static failsafePhase_e  lastFailsafePhase     = FAILSAFE_IDLE;
static bool             lastGpsRescueMode     = false;
static bool             lastCrashFlipMode     = false;
static bool             lastFailsafeMode      = false;
static bool             takeoffLoggedThisArm  = false;
static armingDisableFlags_e lastArmingDisableFlags = 0;
static beeperMode_e     lastBeeperMode        = BEEPER_SILENCE;
#ifdef USE_GPS
static bool             lastGpsFix            = false;
#endif

// Rate-limit timestamps (ms) per category
#ifdef USE_GPS
static timeMs_t         nextGpsLogMs          = 0;
#endif
static timeMs_t         nextBeeperRepeatMs    = 0;

// --------------------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------------------

static void eventlogWriteLine(const char *event, const char *detail,
                               timeMs_t nowMs);
static void eventlogFlushPending(void);
static timeMs_t eventlogNowMs(void);
static void eventlogFormatArmingDisableFlags(armingDisableFlags_e flags, char *detail, size_t detailLen);
#ifdef USE_FLASHFS
static void eventlogResumeCountersFromFlash(void);
#endif
#ifdef USE_SDCARD
static void eventlogOpenFile(void);
#endif

// --------------------------------------------------------------------------
// Async file-open callback
// --------------------------------------------------------------------------

#ifdef USE_SDCARD
static void eventlogFileOpened(afatfsFilePtr_t file)
{
    eventlogOpening = false;
    if (file == NULL) {
        // Retry on next update call
        return;
    }

    eventlogFile  = file;
    eventlogReady = true;
    rowNumber     = 0;

    afatfs_fwrite(eventlogFile, (const uint8_t *)eventlogCsvHeader, strlen(eventlogCsvHeader));

    // First record — power-on marker
    eventlogWriteLine("POWER_UP", "system_init", eventlogNowMs());
    eventlogFlushPending();
}
#endif

// --------------------------------------------------------------------------
// File management — enumerate existing log_XXXX.txt files on the SD card
// --------------------------------------------------------------------------

#ifdef USE_SDCARD
typedef struct {
    afatfsFinder_t      finder;
    afatfsFilePtr_t     logDir;
    int32_t             largest;

    enum {
        ELOPEN_INITIAL,
        ELOPEN_WAITING_CHDIR,
        ELOPEN_ENUMERATE,
        ELOPEN_CREATE
    } state;
} eventlogOpenState_t;

static eventlogOpenState_t elOpen;
#endif

static timeMs_t eventlogNowMs(void)
{
    return millis() + eventlogTimeOffsetMs;
}

#ifdef USE_FLASHFS
static bool eventlogFlashHasValidHeader(void)
{
    char header[sizeof(eventlogCsvHeader)];

    if (eventlogFlashGetUsedSize() < sizeof(eventlogCsvHeader) - 1) {
        return false;
    }

    const int bytesRead = eventlogFlashReadAbs(0, (uint8_t *)header, sizeof(eventlogCsvHeader) - 1);

    return bytesRead == (int)(sizeof(eventlogCsvHeader) - 1)
        && memcmp(header, eventlogCsvHeader, sizeof(eventlogCsvHeader) - 1) == 0;
}
#endif

static void eventlogQueue(const char *event, const char *detail)
{
    if (!event || eventlogPendingCount >= EVENTLOG_PENDING_COUNT) {
        return;
    }

    const uint8_t index = (eventlogPendingHead + eventlogPendingCount) % EVENTLOG_PENDING_COUNT;
    strncpy(eventlogPending[index].event, event, EVENTLOG_EVENT_LEN - 1);
    eventlogPending[index].event[EVENTLOG_EVENT_LEN - 1] = '\0';

    if (detail) {
        strncpy(eventlogPending[index].detail, detail, EVENTLOG_DETAIL_LEN - 1);
        eventlogPending[index].detail[EVENTLOG_DETAIL_LEN - 1] = '\0';
    } else {
        eventlogPending[index].detail[0] = '\0';
    }

    eventlogPendingCount++;
}

static void eventlogFlushPending(void)
{
    while (eventlogPendingCount > 0) {
        eventlogPending_t *pending = &eventlogPending[eventlogPendingHead];
        eventlogWriteLine(pending->event, pending->detail, eventlogNowMs());
        eventlogPendingHead = (eventlogPendingHead + 1) % EVENTLOG_PENDING_COUNT;
        eventlogPendingCount--;
    }
}

void eventlogAdd(const char *event, const char *detail)
{
    if (!eventlogConfig()->enabled) {
        return;
    }

    if (!eventlogReady) {
        eventlogQueue(event, detail);
        return;
    }

    eventlogWriteLine(event, detail ? detail : "", eventlogNowMs());
}

#ifdef USE_FLASHFS
static bool eventlogParseCsvCounters(const char *line, uint32_t *row, timeMs_t *timestamp)
{
    if (!line || !row || !timestamp || line[0] < '0' || line[0] > '9') {
        return false;
    }

    uint32_t parsedRow = 0;
    const char *cursor = line;
    while (*cursor >= '0' && *cursor <= '9') {
        parsedRow = (parsedRow * 10) + (*cursor++ - '0');
    }

    if (*cursor++ != ',') {
        return false;
    }

    uint32_t firstTimestampField = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        firstTimestampField = (firstTimestampField * 10) + (*cursor++ - '0');
    }

    timeMs_t parsedTimestamp = 0;
    if (*cursor == ':') {
        cursor++;

        uint32_t seconds = 0;
        while (*cursor >= '0' && *cursor <= '9') {
            seconds = (seconds * 10) + (*cursor++ - '0');
        }

        if (*cursor++ != ':') {
            return false;
        }

        uint32_t millis = 0;
        while (*cursor >= '0' && *cursor <= '9') {
            millis = (millis * 10) + (*cursor++ - '0');
        }

        parsedTimestamp = ((timeMs_t)firstTimestampField * 60 * 1000) + (seconds * 1000) + millis;
    } else {
        parsedTimestamp = firstTimestampField;
    }

    if (*cursor != ',') {
        return false;
    }

    *row = parsedRow;
    *timestamp = parsedTimestamp;
    return true;
}

static void eventlogResumeCountersFromFlash(void)
{
    const uint32_t usedSize = eventlogFlashGetUsedSize();
    if (usedSize == 0) {
        return;
    }

    uint8_t buffer[256];
    const uint32_t readSize = MIN((uint32_t)sizeof(buffer) - 1, usedSize);
    const uint32_t readOffset = usedSize - readSize;
    const int bytesRead = eventlogFlashReadAbs(readOffset, buffer, readSize);
    if (bytesRead <= 0) {
        return;
    }

    buffer[bytesRead] = '\0';

    int end = bytesRead - 1;
    while (end >= 0 && (buffer[end] == '\r' || buffer[end] == '\n' || buffer[end] == '\0')) {
        end--;
    }

    int start = end;
    while (start >= 0 && buffer[start] != '\n') {
        start--;
    }
    start++;

    if (start > end) {
        return;
    }

    buffer[end + 1] = '\0';

    uint32_t lastRow = 0;
    timeMs_t lastTimestamp = 0;
    if (eventlogParseCsvCounters((const char *)&buffer[start], &lastRow, &lastTimestamp)) {
        rowNumber = lastRow;
        const timeMs_t now = millis();
        if (lastTimestamp >= now) {
            eventlogTimeOffsetMs = lastTimestamp - now + 1;
        }
    }
}
#endif

void eventlogInit(void)
{
    if (!eventlogConfig()->enabled) {
        eventlogBackend = EVENTLOG_BACKEND_NONE;
        eventlogReady = false;
        eventlogPendingHead = 0;
        eventlogPendingCount = 0;
        return;
    }

#ifdef USE_SDCARD
    eventlogFile = NULL;
    eventlogOpening = false;
#endif
    eventlogBackend = EVENTLOG_BACKEND_NONE;
    eventlogReady = false;
    rowNumber = 0;
    eventlogTimeOffsetMs = 0;

    lastArmed = false;
    lastBatteryState = BATTERY_INIT;
    lastFailsafePhase = FAILSAFE_IDLE;
    lastGpsRescueMode = false;
    lastCrashFlipMode = false;
    lastFailsafeMode = false;
    takeoffLoggedThisArm = false;
    lastArmingDisableFlags = 0;
    lastBeeperMode = BEEPER_SILENCE;
#ifdef USE_GPS
    lastGpsFix = false;

    nextGpsLogMs = 0;
#endif
    nextBeeperRepeatMs = 0;

#ifdef USE_SDCARD
    memset(&elOpen, 0, sizeof(elOpen));
    elOpen.state = ELOPEN_INITIAL;
#endif

#if defined(USE_FLASHFS) && defined(USE_BLACKBOX)
    if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH && eventlogFlashInit()) {
        eventlogBackend = EVENTLOG_BACKEND_FLASH;
        eventlogReady = true;
        rowNumber = 0;

        if (eventlogFlashGetUsedSize() == 0 || !eventlogFlashHasValidHeader()) {
            eventlogFlashErase();
            eventlogFlashWrite((const uint8_t *)eventlogCsvHeader, strlen(eventlogCsvHeader));
        } else {
            eventlogResumeCountersFromFlash();
        }
        eventlogWriteLine("POWER_UP", "system_init", eventlogNowMs());
        eventlogFlushPending();
        return;
    }
#endif

#ifdef USE_SDCARD
#ifdef USE_BLACKBOX
    if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD)
#endif
    {
        eventlogBackend = EVENTLOG_BACKEND_SDCARD;
    }
#endif
}

#ifdef USE_SDCARD
static void eventlogOpenFile(void)
{
    // Kick off async directory scan / file creation state machine.
    // This function drives the state machine one step each call;
    // eventlogUpdate() calls it repeatedly until open is complete.

    switch (elOpen.state) {

    case ELOPEN_INITIAL:
        if (afatfs_getFilesystemState() != AFATFS_FILESYSTEM_STATE_READY) {
            return;
        }
        elOpen.largest = 0;
        elOpen.state   = ELOPEN_WAITING_CHDIR;
        if (!afatfs_chdir(NULL)) {
            // async chdir not needed — already at root; go straight to enumerate
            elOpen.state = ELOPEN_ENUMERATE;
            afatfs_findFirst(NULL, &elOpen.finder);
        }
        break;

    case ELOPEN_WAITING_CHDIR:
        // Wait for chdir to complete — just return and let the state advance
        break;

    case ELOPEN_ENUMERATE: {
        fatDirectoryEntry_t *entry;
        afatfsOperationStatus_e status =
            afatfs_findNext(NULL, &elOpen.finder, &entry);

        if (status == AFATFS_OPERATION_SUCCESS) {
            if (entry && !fat_isDirectoryEntryTerminator(entry)) {
                // Check for log_XXXX.txt pattern
                const size_t prefixLen = strlen(EVENTLOG_PREFIX);
                // FAT 8.3 names: "log_" = 4, "XXXX" = 4, ".txt" ext = 3
                // Stored in d_name as "LOG_XXXX" + ext "TXT" (upper-case, space-padded)
                if (strncasecmp(entry->filename, EVENTLOG_PREFIX, prefixLen) == 0) {
                    char numStr[5];
                    memcpy(numStr, entry->filename + prefixLen, 4);
                    numStr[4] = '\0';
                    int32_t n = (int32_t)atoi(numStr);
                    if (n > elOpen.largest) {
                        elOpen.largest = n;
                    }
                }
                // Keep enumerating
            } else {
                // End of directory
                afatfs_findLast(NULL);
                elOpen.state = ELOPEN_CREATE;
            }
        }
        // If AFATFS_OPERATION_IN_PROGRESS, just return and retry next call
        break;
    }

    case ELOPEN_CREATE: {
        int32_t next = (elOpen.largest % EVENTLOG_MAX_NUMBER) + 1;
        char filename[16]; // "log_9999.txt\0" = 13 chars
        tfp_sprintf(filename, "%s%04d%s",
                 EVENTLOG_PREFIX, (int)next, EVENTLOG_SUFFIX);
        eventlogOpening = true;
        afatfs_fopen(filename, "w", eventlogFileOpened);
        elOpen.state = ELOPEN_INITIAL; // reset for next log session
        break;
    }

    default:
        break;
    }
}
#endif

void eventlogFlush(void)
{
    if (eventlogBackend == EVENTLOG_BACKEND_FLASH) {
#ifdef USE_FLASHFS
        eventlogFlashFlush();
#endif
        return;
    }

#ifdef USE_SDCARD
    if (eventlogBackend == EVENTLOG_BACKEND_SDCARD && eventlogFile) {
        afatfs_flush();
    }
#endif
}

void eventlogClose(void)
{
    if (eventlogBackend == EVENTLOG_BACKEND_FLASH) {
#ifdef USE_FLASHFS
        eventlogFlashClose();
#endif
        eventlogReady = false;
        return;
    }

#ifdef USE_SDCARD
    if (eventlogBackend == EVENTLOG_BACKEND_SDCARD && eventlogFile) {
        afatfs_fclose(eventlogFile, NULL);
        eventlogFile  = NULL;
        eventlogReady = false;
    }
#endif
}

// --------------------------------------------------------------------------
// CSV line writer
// --------------------------------------------------------------------------

static void eventlogWriteLine(const char *event, const char *detail,
                               timeMs_t nowMs)
{
    if (!eventlogConfig()->enabled || !eventlogReady) {
        return;
    }

    const uint32_t nextRowNumber = rowNumber + 1;

    // Build timestamp strings
    char uptimeStr[16];
    const uint32_t uptimeMinutes = nowMs / 60000;
    const uint32_t uptimeSeconds = (nowMs / 1000) % 60;
    const uint32_t uptimeMillis = nowMs % 1000;
    tfp_sprintf(uptimeStr, "%lu:%02lu:%03lu",
        (unsigned long)uptimeMinutes,
        (unsigned long)uptimeSeconds,
        (unsigned long)uptimeMillis);

    char timeStr[24] = "";

#ifdef USE_RTC_TIME
    dateTime_t dt;
    if (rtcHasTime() && rtcGetDateTime(&dt)) {
        tfp_sprintf(timeStr,
                 "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 dt.year, dt.month, dt.day,
                 dt.hours, dt.minutes, dt.seconds);
    }
#endif

    // GPS coordinates
    int32_t lat7 = 0, lon7 = 0;
    char latDd[20] = "", lonDd[20] = "", mapsUrl[80] = "";

#ifdef USE_GPS
    const bool hasFix = STATE(GPS_FIX) && (gpsSol.numSat >= 4);

    if (hasFix) {
        lat7 = gpsSol.llh.lat;
        lon7 = gpsSol.llh.lon;
        // Convert from deg*1e7 to decimal degrees (6 decimal places)
        int32_t latAbs   = lat7 < 0 ? -lat7 : lat7;
        int32_t lonAbs   = lon7 < 0 ? -lon7 : lon7;
        int32_t latDeg   = latAbs / GPS_DEGREES_DIVIDER;
        int32_t lonDeg   = lonAbs / GPS_DEGREES_DIVIDER;
        uint32_t latFrac = (uint32_t)(latAbs % GPS_DEGREES_DIVIDER);
        uint32_t lonFrac = (uint32_t)(lonAbs % GPS_DEGREES_DIVIDER);

        tfp_sprintf(latDd, "%s%ld.%07lu",
                 lat7 < 0 ? "-" : "", (long)latDeg, (unsigned long)latFrac);
        tfp_sprintf(lonDd, "%s%ld.%07lu",
                 lon7 < 0 ? "-" : "", (long)lonDeg, (unsigned long)lonFrac);
        tfp_sprintf(mapsUrl,
                 GMAPS_URL_PREFIX "%s,%s", latDd, lonDd);
    }
#endif

    // Build and write the CSV line
    char buf[200];
    int len = tfp_sprintf(buf,
        "%lu,%s,%s,%s,%s,"
        "%ld,%ld,%s,%s,%s\r\n",
        (unsigned long)nextRowNumber,
        uptimeStr,
        timeStr,
        event,
        detail,
        (long)lat7,
        (long)lon7,
        latDd,
        lonDd,
        mapsUrl
    );

    if (len > 0 && len < (int)sizeof(buf)) {
        if (eventlogBackend == EVENTLOG_BACKEND_FLASH) {
#ifdef USE_FLASHFS
            if (eventlogFlashWrite((const uint8_t *)buf, (uint32_t)len)) {
                rowNumber = nextRowNumber;
            }
#endif
            return;
        }

#ifdef USE_SDCARD
        if (eventlogBackend == EVENTLOG_BACKEND_SDCARD && eventlogFile) {
            afatfs_fwrite(eventlogFile, (const uint8_t *)buf, (uint32_t)len);
            rowNumber = nextRowNumber;
        }
#endif
    }
}

// --------------------------------------------------------------------------
// Beeper helper — returns the highest-priority enabled beeper mode.
//
// We walk the beeperTable in index order (index 0 = highest priority) and
// return the first mode that is not masked off in beeperConfig.
// This mirrors the priority logic inside beeper.c without needing access to
// the static currentBeeperEntry pointer.
// --------------------------------------------------------------------------

static beeperMode_e eventlogCurrentBeeperMode(void)
{
    const int count = beeperTableEntryCount();
    for (int i = 0; i < count; i++) {
        beeperMode_e m = beeperModeForTableIndex(i);
        if (m == BEEPER_SILENCE || m == BEEPER_ALL) {
            continue;
        }
        uint32_t flag = beeperModeMaskForTableIndex(i);
        if (flag && !(beeperConfig()->beeper_off_flags & flag)) {
            return m;
        }
    }
    return BEEPER_SILENCE;
}

// Return the name string for a beeper mode (NULL if not found).
static const char *eventlogBeeperModeName(beeperMode_e mode)
{
    const int count = beeperTableEntryCount();
    for (int i = 0; i < count; i++) {
        if (beeperModeForTableIndex(i) == mode) {
            return beeperNameForTableIndex(i);
        }
    }
    return "";
}

static void eventlogFormatArmingDisableFlags(armingDisableFlags_e flags, char *detail, size_t detailLen)
{
    if (!detail || detailLen == 0) {
        return;
    }

    if (!flags) {
        tfp_sprintf(detail, "flags=0x%08lx active=NONE", (unsigned long)flags);
        return;
    }

    const armingDisableFlags_e firstFlag = 1 << (ffs(flags) - 1);
    tfp_sprintf(detail, "flags=0x%08lx active=%s",
        (unsigned long)flags,
        getArmingDisableFlagName(firstFlag));
}

// --------------------------------------------------------------------------
// Main update — called every FC loop iteration
// --------------------------------------------------------------------------

void eventlogUpdate(timeUs_t currentTimeUs)
{
    if (!eventlogConfig()->enabled) {
        return;
    }

    // Drive the async file-open state machine until the file is ready.
    if (!eventlogReady) {
#ifdef USE_SDCARD
        if (eventlogBackend == EVENTLOG_BACKEND_SDCARD) {
        if (!eventlogOpening) {
            eventlogOpenFile();
        }
        }
#endif
        return;
    }

    const timeMs_t nowMs = (currentTimeUs / 1000) + eventlogTimeOffsetMs;

    // -----------------------------------------------------------------------
    // ARM / DISARM
    // -----------------------------------------------------------------------
    const bool armed = ARMING_FLAG(ARMED);
    if (armed != lastArmed) {
        if (armed) {
            eventlogWriteLine("ARM", "", nowMs);
            takeoffLoggedThisArm = false;
        } else {
            takeoffLoggedThisArm = false;
        }
        lastArmed = armed;
    }

    const bool throttleRaised = calculateThrottleStatus() != THROTTLE_LOW;
    if (armed && throttleRaised && !takeoffLoggedThisArm) {
        eventlogWriteLine("TAKEOFF", "throttle_active", nowMs);
        takeoffLoggedThisArm = true;
    }

    // -----------------------------------------------------------------------
    // ARMING DISABLE / WARNING FLAGS
    // -----------------------------------------------------------------------
    const armingDisableFlags_e armingDisableFlags = getArmingDisableFlags();
    if (armingDisableFlags != lastArmingDisableFlags) {
        char detail[64];
        eventlogFormatArmingDisableFlags(armingDisableFlags, detail, sizeof(detail));
        eventlogWriteLine("ARMING_DISABLED", detail, nowMs);
        lastArmingDisableFlags = armingDisableFlags;
    }

    // -----------------------------------------------------------------------
    // FAILSAFE PHASE
    // -----------------------------------------------------------------------
    const failsafePhase_e currentFailsafePhase = failsafePhase();
    if (currentFailsafePhase != lastFailsafePhase) {
        const char *detail = "";
        switch (currentFailsafePhase) {
        case FAILSAFE_IDLE:                detail = "IDLE";            break;
        case FAILSAFE_RX_LOSS_DETECTED:    detail = "RX_LOSS";         break;
        case FAILSAFE_LANDING:             detail = "LANDING";         break;
        case FAILSAFE_LANDED:              detail = "LANDED";          break;
        case FAILSAFE_RX_LOSS_MONITORING:  detail = "RX_MONITORING";   break;
        case FAILSAFE_RX_LOSS_RECOVERED:   detail = "RX_RECOVERED";    break;
        case FAILSAFE_GPS_RESCUE:          detail = "GPS_RESCUE";      break;
        default:                           detail = "UNKNOWN";         break;
        }
        eventlogWriteLine("FAILSAFE", detail, nowMs);
        lastFailsafePhase = currentFailsafePhase;
    }

    // -----------------------------------------------------------------------
    // FAILSAFE FLIGHT MODE FLAG
    // -----------------------------------------------------------------------
    const bool failsafeMode = FLIGHT_MODE(FAILSAFE_MODE) != 0;
    if (failsafeMode != lastFailsafeMode) {
        eventlogWriteLine("FAILSAFE_MODE", failsafeMode ? "ACTIVE" : "CLEARED", nowMs);
        lastFailsafeMode = failsafeMode;
    }

    // -----------------------------------------------------------------------
    // GPS RESCUE MODE
    // -----------------------------------------------------------------------
    const bool gpsRescueMode = FLIGHT_MODE(GPS_RESCUE_MODE) != 0;
    if (gpsRescueMode != lastGpsRescueMode) {
        eventlogWriteLine("GPS_RESCUE", gpsRescueMode ? "ACTIVE" : "CLEARED", nowMs);
        lastGpsRescueMode = gpsRescueMode;
    }

    // -----------------------------------------------------------------------
    // TURTLE / CRASH FLIP MODE
    // -----------------------------------------------------------------------
    const bool crashFlipMode = IS_RC_MODE_ACTIVE(BOXCRASHFLIP);
    if (crashFlipMode != lastCrashFlipMode) {
        eventlogWriteLine("TURTLE_MODE", crashFlipMode ? "ACTIVE" : "CLEARED", nowMs);
        lastCrashFlipMode = crashFlipMode;
    }

    // -----------------------------------------------------------------------
    // BATTERY STATE
    // -----------------------------------------------------------------------
    const batteryState_e batteryState = getBatteryState();
    if (batteryState != lastBatteryState) {
        const char *detail = "";
        switch (batteryState) {
        case BATTERY_OK:           detail = "OK";           break;
        case BATTERY_WARNING:      detail = "LOW";          break;
        case BATTERY_CRITICAL:     detail = "FLAT";         break;
        case BATTERY_NOT_PRESENT:  detail = "NOT_PRESENT";  break;
        case BATTERY_INIT:         detail = "INIT";         break;
        default:                   detail = "UNKNOWN";      break;
        }
        eventlogWriteLine("BATTERY", detail, nowMs);
        lastBatteryState = batteryState;
    }

    // -----------------------------------------------------------------------
    // BEEPER — log mode changes (rate-limited for repeat modes)
    // -----------------------------------------------------------------------
    const beeperMode_e currentBeeper = eventlogCurrentBeeperMode();

    if (currentBeeper != lastBeeperMode) {
        if (nowMs >= nextBeeperRepeatMs || currentBeeper == BEEPER_SILENCE) {
            const char *beeperName = eventlogBeeperModeName(currentBeeper);
            eventlogWriteLine("BEEPER",
                              beeperName ? beeperName : "",
                              nowMs);
            lastBeeperMode     = currentBeeper;
            nextBeeperRepeatMs = nowMs + EVENTLOG_RATE_LIMIT_MS;
        }
    }

    // -----------------------------------------------------------------------
    // GPS FIX CHANGE
    // -----------------------------------------------------------------------
#ifdef USE_GPS
    const bool gpsLoggingEnabled = eventlogConfig()->gpsLoggingEnabled;
    const bool gpsFix = STATE(GPS_FIX);
    if (gpsLoggingEnabled && gpsFix != lastGpsFix) {
        char detail[32];
        tfp_sprintf(detail, "sats=%u", (unsigned)gpsSol.numSat);
        eventlogWriteLine(gpsFix ? "GPS_FIX" : "GPS_LOST", detail, nowMs);
        lastGpsFix = gpsFix;
    } else if (!gpsLoggingEnabled) {
        lastGpsFix = gpsFix;
    }

    // -----------------------------------------------------------------------
    // GPS POSITION — rate-limited 1/sec when armed and fix available
    // -----------------------------------------------------------------------
    if (gpsLoggingEnabled && armed && gpsFix && nowMs >= nextGpsLogMs) {
        char detail[48];
        tfp_sprintf(detail, "sats=%u alt=%ldcm spd=%u",
                 (unsigned)gpsSol.numSat,
                 (long)gpsSol.llh.altCm,
                 (unsigned)gpsSol.groundSpeed);
        eventlogWriteLine("GPS_POS", detail, nowMs);
        nextGpsLogMs = nowMs + EVENTLOG_RATE_LIMIT_MS;
    }
#endif

    // -----------------------------------------------------------------------
    // Periodic flush to avoid losing data on unexpected power loss
    // -----------------------------------------------------------------------
    static timeMs_t nextFlushMs = 0;
    if (nowMs >= nextFlushMs) {
        if (eventlogBackend == EVENTLOG_BACKEND_FLASH) {
#ifdef USE_FLASHFS
            eventlogFlashFlush();
#endif
        }
#ifdef USE_SDCARD
        if (eventlogBackend == EVENTLOG_BACKEND_SDCARD) {
            afatfs_flush();
        }
#endif
        nextFlushMs = nowMs + 5000; // flush every 5 s
    }
}

#endif // USE_EVENTLOG
