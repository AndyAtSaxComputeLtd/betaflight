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

#include <math.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#ifdef USE_EVENTLOG
#include "eventlog/eventlog.h"
#endif

#include "pg/auto_acro.h"

#include "rx/rx.h"

#include "auto_acro.h"

#define AUTO_ACRO_TRIGGER_HIGH 1750
#define AUTO_ACRO_TRIGGER_LOW  1250
#define AUTO_ACRO_TRICK_DEGREES 360.0f
#define AUTO_ACRO_THROTTLE     1750.0f

typedef enum {
    AUTO_ACRO_MANEUVER_NONE = 0,
    AUTO_ACRO_MANEUVER_ROLL,
    AUTO_ACRO_MANEUVER_FLIP,
    AUTO_ACRO_MANEUVER_POWER_LOOP,
} autoAcroManeuver_e;

typedef struct autoAcroRuntime_s {
    autoAcroManeuver_e maneuver;
    flight_dynamics_index_t axis;
    float rateSetpoint;
    float throttle;
    float degreesIntegrated;
    bool flightDetected;
    bool rollTriggerArmed;
    bool flipTriggerArmed;
    bool powerLoopTriggerArmed;
    bool rollAuxTriggered;
    bool flipAuxTriggered;
    bool powerLoopAuxTriggered;
} autoAcroRuntime_t;

static autoAcroRuntime_t autoAcroRuntime = {
    .maneuver = AUTO_ACRO_MANEUVER_NONE,
    .axis = FD_ROLL,
    .rateSetpoint = 0.0f,
    .throttle = AUTO_ACRO_THROTTLE,
    .degreesIntegrated = 0.0f,
    .flightDetected = false,
    .rollTriggerArmed = false,
    .flipTriggerArmed = false,
    .powerLoopTriggerArmed = false,
    .rollAuxTriggered = false,
    .flipAuxTriggered = false,
    .powerLoopAuxTriggered = false,
};

static bool autoAcroAuxIsHigh(uint8_t aux)
{
    return aux < MAX_SUPPORTED_RC_CHANNEL_COUNT && rcData[aux] > AUTO_ACRO_TRIGGER_HIGH;
}

static bool autoAcroAuxIsCentered(uint8_t aux)
{
    return aux >= MAX_SUPPORTED_RC_CHANNEL_COUNT
        || (rcData[aux] > AUTO_ACRO_TRIGGER_LOW && rcData[aux] < AUTO_ACRO_TRIGGER_HIGH);
}

static bool autoAcroAuxIsLow(uint8_t aux)
{
    return aux < MAX_SUPPORTED_RC_CHANNEL_COUNT && rcData[aux] < AUTO_ACRO_TRIGGER_LOW;
}

#ifdef USE_EVENTLOG
static const char *autoAcroManeuverName(autoAcroManeuver_e maneuver)
{
    switch (maneuver) {
    case AUTO_ACRO_MANEUVER_ROLL:
        return "ROLL";
    case AUTO_ACRO_MANEUVER_FLIP:
        return "FLIP";
    case AUTO_ACRO_MANEUVER_POWER_LOOP:
        return "POWER_LOOP";
    default:
        return "NONE";
    }
}

static const char *autoAcroDirectionName(autoAcroManeuver_e maneuver, float rateSetpoint)
{
    switch (maneuver) {
    case AUTO_ACRO_MANEUVER_ROLL:
        return rateSetpoint < 0.0f ? "LEFT" : "RIGHT";
    case AUTO_ACRO_MANEUVER_FLIP:
        return rateSetpoint < 0.0f ? "BACK" : "FRONT";
    case AUTO_ACRO_MANEUVER_POWER_LOOP:
        return "FORWARD";
    default:
        return "";
    }
}

static void autoAcroLogStart(autoAcroManeuver_e maneuver, float rateSetpoint)
{
    char detail[64];
    tfp_sprintf(detail, "%s_%s_START", autoAcroManeuverName(maneuver), autoAcroDirectionName(maneuver, rateSetpoint));
    eventlogAdd("AUTOACRO", detail);
}

static void autoAcroLogStop(autoAcroManeuver_e maneuver, const char *reason)
{
    char detail[64];
    tfp_sprintf(detail, "%s_%s", autoAcroManeuverName(maneuver), reason);
    eventlogAdd("AUTOACRO", detail);
}

static void autoAcroLogAuxTrigger(const char *detail, bool *latched, bool active)
{
    if (active && !*latched) {
        eventlogAdd("AUTOACRO_AUX", detail);
    }
    *latched = active;
}
#else
#define autoAcroLogStart(maneuver, rateSetpoint) do { UNUSED(maneuver); UNUSED(rateSetpoint); } while (0)
#define autoAcroLogStop(maneuver, reason) do { UNUSED(maneuver); UNUSED(reason); } while (0)
#define autoAcroLogAuxTrigger(detail, latched, active) do { UNUSED(detail); UNUSED(latched); UNUSED(active); } while (0)
#endif

static void autoAcroStop(const char *reason)
{
    if (autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE) {
        autoAcroLogStop(autoAcroRuntime.maneuver, reason);
    }

    autoAcroRuntime.maneuver = AUTO_ACRO_MANEUVER_NONE;
    autoAcroRuntime.rateSetpoint = 0.0f;
    autoAcroRuntime.degreesIntegrated = 0.0f;
}

static bool autoAcroIsInFlight(void)
{
    if (!ARMING_FLAG(ARMED)) {
        autoAcroRuntime.flightDetected = false;
        return false;
    }

    if (calculateThrottleStatus() != THROTTLE_LOW) {
        autoAcroRuntime.flightDetected = true;
    }

    return autoAcroRuntime.flightDetected;
}

static void autoAcroStart(autoAcroManeuver_e maneuver, flight_dynamics_index_t axis, float rateSetpoint, float throttle)
{
    autoAcroLogStart(maneuver, rateSetpoint);

    autoAcroRuntime.maneuver = maneuver;
    autoAcroRuntime.axis = axis;
    autoAcroRuntime.rateSetpoint = rateSetpoint;
    autoAcroRuntime.throttle = throttle;
    autoAcroRuntime.degreesIntegrated = 0.0f;
}

static float autoAcroThrottlePercentToRcCommand(uint8_t throttlePercent)
{
    return 1000.0f + (constrain(throttlePercent, 0, 100) * 10.0f);
}

void autoAcroUpdateTriggers(void)
{
    const autoAcroConfig_t *config = autoAcroConfig();

    const bool inFlight = autoAcroIsInFlight();

    if (!inFlight) {
        autoAcroStop("STOPPED");
    }

    autoAcroRuntime.rollTriggerArmed = autoAcroAuxIsCentered(config->rollTrickAux)
        ? true : (autoAcroAuxIsLow(config->rollTrickAux) || autoAcroAuxIsHigh(config->rollTrickAux) ? false : autoAcroRuntime.rollTriggerArmed);
    autoAcroRuntime.flipTriggerArmed = autoAcroAuxIsCentered(config->flipTrickAux)
        ? true : (autoAcroAuxIsLow(config->flipTrickAux) || autoAcroAuxIsHigh(config->flipTrickAux) ? false : autoAcroRuntime.flipTriggerArmed);
    autoAcroRuntime.powerLoopTriggerArmed = !autoAcroAuxIsHigh(config->powerLoopTrickAux);

    const bool rollLow = config->roll && autoAcroAuxIsLow(config->rollTrickAux);
    const bool rollHigh = config->roll && autoAcroAuxIsHigh(config->rollTrickAux);
    const bool flipLow = config->flip && autoAcroAuxIsLow(config->flipTrickAux);
    const bool flipHigh = config->flip && autoAcroAuxIsHigh(config->flipTrickAux);
    const bool powerLoopHigh = config->powerLoop && autoAcroAuxIsHigh(config->powerLoopTrickAux);

    autoAcroLogAuxTrigger(rollLow ? "ROLL_LEFT" : "ROLL_RIGHT", &autoAcroRuntime.rollAuxTriggered, rollLow || rollHigh);
    autoAcroLogAuxTrigger(flipLow ? "FLIP_BACK" : "FLIP_FRONT", &autoAcroRuntime.flipAuxTriggered, flipLow || flipHigh);
    autoAcroLogAuxTrigger("POWER_LOOP_FORWARD", &autoAcroRuntime.powerLoopAuxTriggered, powerLoopHigh);

    if (!inFlight || autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE) {
        return;
    }

    if (config->roll && autoAcroRuntime.rollTriggerArmed) {
        if (rollLow) {
            autoAcroStart(AUTO_ACRO_MANEUVER_ROLL, FD_ROLL, -(float)config->rollSpeed, AUTO_ACRO_THROTTLE);
            autoAcroRuntime.rollTriggerArmed = false;
            return;
        }
        if (rollHigh) {
            autoAcroStart(AUTO_ACRO_MANEUVER_ROLL, FD_ROLL, config->rollSpeed, AUTO_ACRO_THROTTLE);
            autoAcroRuntime.rollTriggerArmed = false;
            return;
        }
    }

    if (config->flip && autoAcroRuntime.flipTriggerArmed) {
        if (flipLow) {
            autoAcroStart(AUTO_ACRO_MANEUVER_FLIP, FD_PITCH, -(float)config->flipSpeed, AUTO_ACRO_THROTTLE);
            autoAcroRuntime.flipTriggerArmed = false;
            return;
        }
        if (flipHigh) {
            autoAcroStart(AUTO_ACRO_MANEUVER_FLIP, FD_PITCH, config->flipSpeed, AUTO_ACRO_THROTTLE);
            autoAcroRuntime.flipTriggerArmed = false;
            return;
        }
    }

    if (config->powerLoop && autoAcroRuntime.powerLoopTriggerArmed && powerLoopHigh) {
        autoAcroStart(AUTO_ACRO_MANEUVER_POWER_LOOP, FD_PITCH, config->powerLoopSpeed, autoAcroThrottlePercentToRcCommand(config->powerLoopThrottle));
        autoAcroRuntime.powerLoopTriggerArmed = false;
    }
}

void autoAcroUpdate(const float gyroRates[XYZ_AXIS_COUNT], float dT)
{
    if (!autoAcroIsInFlight()) {
        autoAcroStop("STOPPED");
        return;
    }

    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_NONE) {
        return;
    }

    autoAcroRuntime.degreesIntegrated += fabsf(gyroRates[autoAcroRuntime.axis]) * dT;

    if (autoAcroRuntime.degreesIntegrated >= AUTO_ACRO_TRICK_DEGREES) {
        autoAcroStop("COMPLETE");
    }
}

bool autoAcroGetSetpoint(flight_dynamics_index_t axis, float *setpoint)
{
    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_NONE || axis != autoAcroRuntime.axis) {
        return false;
    }

    *setpoint = autoAcroRuntime.rateSetpoint;
    return true;
}

bool autoAcroShouldOverrideThrottle(void)
{
    return autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE;
}

float autoAcroGetThrottle(void)
{
    return autoAcroRuntime.throttle;
}

#endif // USE_AUTOACRO
