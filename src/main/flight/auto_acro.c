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

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#ifdef USE_EVENTLOG
#include "eventlog/eventlog.h"
#endif

#include "pg/auto_acro.h"

#include "rx/rx.h"

#include "auto_acro.h"

#define AUTO_ACRO_TRIGGER_LOW_MAX     1100
#define AUTO_ACRO_TRIGGER_CENTER_LOW  1400
#define AUTO_ACRO_TRIGGER_CENTER_HIGH 1600
#define AUTO_ACRO_TRIGGER_HIGH_MIN    1900
#define AUTO_ACRO_TRICK_DEGREES 360.0f
#define AUTO_ACRO_YAW_DEGREES   160.0f
#define AUTO_ACRO_YAW_PITCH_GAIN 10.0f
#define AUTO_ACRO_YAW_PITCH_RATE_MAX 500.0f
#define AUTO_ACRO_YAW_PITCH_TOLERANCE 2.0f
#define AUTO_ACRO_THROTTLE     1750.0f
#define AUTO_ACRO_RUNAWAY_REENABLE_DELAY_MS 1000

typedef struct autoAcroRuntime_s {
    autoAcroManeuver_e maneuver;
    flight_dynamics_index_t axis;
    float rateSetpoint;
    float throttle;
    float degreesIntegrated;
    float startPitch;
    bool yawRotationComplete;
    bool flightDetected;
    bool rollTriggerArmed;
    bool flipTriggerArmed;
    bool powerLoopTriggerArmed;
    bool yawTriggerArmed;
    bool rollAuxTriggered;
    bool flipAuxTriggered;
    bool powerLoopAuxTriggered;
    bool yawAuxTriggered;
    uint8_t loggedDamperPercent;
    bool damperLogged;
} autoAcroRuntime_t;

static autoAcroRuntime_t autoAcroRuntime = {
    .maneuver = AUTO_ACRO_MANEUVER_NONE,
    .axis = FD_ROLL,
    .rateSetpoint = 0.0f,
    .throttle = AUTO_ACRO_THROTTLE,
    .degreesIntegrated = 0.0f,
    .startPitch = 0.0f,
    .yawRotationComplete = false,
    .flightDetected = false,
    .rollTriggerArmed = false,
    .flipTriggerArmed = false,
    .powerLoopTriggerArmed = false,
    .yawTriggerArmed = false,
    .rollAuxTriggered = false,
    .flipAuxTriggered = false,
    .powerLoopAuxTriggered = false,
    .yawAuxTriggered = false,
    .loggedDamperPercent = 0,
    .damperLogged = false,
};

static timeMs_t autoAcroRunawayReenableAtMs = 0;

static bool autoAcroAuxIsHigh(uint8_t aux)
{
    return aux < MAX_SUPPORTED_RC_CHANNEL_COUNT && rcData[aux] >= AUTO_ACRO_TRIGGER_HIGH_MIN;
}

static bool autoAcroAuxIsCentered(uint8_t aux)
{
    return aux >= MAX_SUPPORTED_RC_CHANNEL_COUNT
        || (rcData[aux] >= AUTO_ACRO_TRIGGER_CENTER_LOW && rcData[aux] <= AUTO_ACRO_TRIGGER_CENTER_HIGH);
}

static bool autoAcroAuxIsLow(uint8_t aux)
{
    return aux < MAX_SUPPORTED_RC_CHANNEL_COUNT && rcData[aux] <= AUTO_ACRO_TRIGGER_LOW_MAX;
}

static uint8_t autoAcroDamperPercent(const autoAcroConfig_t *config)
{
    if (config->speedDamperAux >= MAX_SUPPORTED_RC_CHANNEL_COUNT) {
        return 100;
    }

    const float auxValue = constrainf(rcData[config->speedDamperAux], PWM_RANGE_MIN, PWM_RANGE_MAX);
    return 50 + lrintf((auxValue - PWM_RANGE_MIN) * 50.0f / PWM_RANGE);
}

static float autoAcroDampedSpeed(const autoAcroConfig_t *config, uint16_t configuredSpeed)
{
    return configuredSpeed * autoAcroDamperPercent(config) * 0.01f;
}

static void autoAcroSetRunawayTakeoffDisabled(bool disabled)
{
#ifdef USE_RUNAWAY_TAKEOFF
    runawayTakeoffTemporaryDisable(disabled);
#else
    UNUSED(disabled);
#endif
}

static void autoAcroDelayRunawayTakeoffReenable(void)
{
    autoAcroRunawayReenableAtMs = millis() + AUTO_ACRO_RUNAWAY_REENABLE_DELAY_MS;
}

static void autoAcroUpdateRunawayTakeoffSuppression(void)
{
    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_NONE && autoAcroRunawayReenableAtMs != 0 && millis() >= autoAcroRunawayReenableAtMs) {
        autoAcroSetRunawayTakeoffDisabled(false);
        autoAcroRunawayReenableAtMs = 0;
    }
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
    case AUTO_ACRO_MANEUVER_YAW:
        return "YAW";
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
    case AUTO_ACRO_MANEUVER_YAW:
        return rateSetpoint < 0.0f ? "LEFT" : "RIGHT";
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

static void autoAcroLogDamper(uint8_t percent)
{
    const uint8_t quantizedPercent = ((percent + 2) / 5) * 5;
    if (!autoAcroRuntime.damperLogged) {
        autoAcroRuntime.loggedDamperPercent = quantizedPercent;
        autoAcroRuntime.damperLogged = true;
        return;
    }

    if (quantizedPercent != autoAcroRuntime.loggedDamperPercent) {
        char detail[32];
        tfp_sprintf(detail, "SPEED_%u_PERCENT", quantizedPercent);
        eventlogAdd("AUTOACRO_DAMPER", detail);
        autoAcroRuntime.loggedDamperPercent = quantizedPercent;
    }
}
#else
#define autoAcroLogStart(maneuver, rateSetpoint) do { UNUSED(maneuver); UNUSED(rateSetpoint); } while (0)
#define autoAcroLogStop(maneuver, reason) do { UNUSED(maneuver); UNUSED(reason); } while (0)
#define autoAcroLogAuxTrigger(detail, latched, active) do { UNUSED(detail); UNUSED(latched); UNUSED(active); } while (0)
#define autoAcroLogDamper(percent) do { UNUSED(percent); } while (0)
#endif

static void autoAcroStop(const char *reason)
{
    if (autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE) {
        autoAcroLogStop(autoAcroRuntime.maneuver, reason);
        autoAcroSetRunawayTakeoffDisabled(false);
    }

    autoAcroRuntime.maneuver = AUTO_ACRO_MANEUVER_NONE;
    autoAcroRuntime.rateSetpoint = 0.0f;
    autoAcroRuntime.degreesIntegrated = 0.0f;
    autoAcroRuntime.yawRotationComplete = false;
}

static void autoAcroComplete(void)
{
    if (autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE) {
        autoAcroLogStop(autoAcroRuntime.maneuver, "COMPLETE");
        autoAcroDelayRunawayTakeoffReenable();
    }

    autoAcroRuntime.maneuver = AUTO_ACRO_MANEUVER_NONE;
    autoAcroRuntime.rateSetpoint = 0.0f;
    autoAcroRuntime.degreesIntegrated = 0.0f;
    autoAcroRuntime.yawRotationComplete = false;
}

static bool autoAcroIsInFlight(void)
{
    if (!ARMING_FLAG(ARMED) || failsafeIsActive() || FLIGHT_MODE(FAILSAFE_MODE)) {
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
    autoAcroRunawayReenableAtMs = 0;
    autoAcroSetRunawayTakeoffDisabled(true);
    autoAcroLogStart(maneuver, rateSetpoint);

    autoAcroRuntime.maneuver = maneuver;
    autoAcroRuntime.axis = axis;
    autoAcroRuntime.rateSetpoint = rateSetpoint;
    autoAcroRuntime.throttle = throttle;
    autoAcroRuntime.degreesIntegrated = 0.0f;
    autoAcroRuntime.startPitch = attitude.values.pitch * 0.1f;
    autoAcroRuntime.yawRotationComplete = false;
}

static float autoAcroThrottlePercentToRcCommand(uint8_t throttlePercent)
{
    return 1000.0f + (constrain(throttlePercent, 0, 100) * 10.0f);
}

void autoAcroUpdateTriggers(void)
{
    autoAcroUpdateRunawayTakeoffSuppression();

    const autoAcroConfig_t *config = autoAcroConfig();
    autoAcroLogDamper(autoAcroDamperPercent(config));

    const bool inFlight = autoAcroIsInFlight();

    if (!inFlight) {
        autoAcroStop("STOPPED");
    }

    const bool rollLow = config->roll && autoAcroAuxIsLow(config->rollTrickAux);
    const bool rollHigh = config->roll && autoAcroAuxIsHigh(config->rollTrickAux);
    const bool rollCentered = autoAcroAuxIsCentered(config->rollTrickAux);
    const bool flipLow = config->flip && autoAcroAuxIsLow(config->flipTrickAux);
    const bool flipHigh = config->flip && autoAcroAuxIsHigh(config->flipTrickAux);
    const bool flipCentered = autoAcroAuxIsCentered(config->flipTrickAux);
    const bool sharedAuxLow = autoAcroAuxIsLow(config->powerLoopTrickAux);
    const bool sharedAuxHigh = autoAcroAuxIsHigh(config->powerLoopTrickAux);
    const bool powerLoopCentered = autoAcroAuxIsCentered(config->powerLoopTrickAux);
    const bool powerLoopHighSelected = config->highManeuver == AUTO_ACRO_HIGH_POWER_LOOP;
    const bool powerLoopActive = config->powerLoop && (powerLoopHighSelected ? sharedAuxHigh : sharedAuxLow);
    const bool yawLow = config->yaw && powerLoopHighSelected && sharedAuxLow;
    const bool yawHigh = config->yaw && !powerLoopHighSelected && sharedAuxHigh;

    const bool rollTriggerArmed = autoAcroRuntime.rollTriggerArmed;
    const bool flipTriggerArmed = autoAcroRuntime.flipTriggerArmed;
    const bool powerLoopTriggerArmed = autoAcroRuntime.powerLoopTriggerArmed;
    const bool yawTriggerArmed = autoAcroRuntime.yawTriggerArmed;

    autoAcroLogAuxTrigger(rollLow ? "ROLL_LEFT" : "ROLL_RIGHT", &autoAcroRuntime.rollAuxTriggered, rollLow || rollHigh);
    autoAcroLogAuxTrigger(flipLow ? "FLIP_BACK" : "FLIP_FRONT", &autoAcroRuntime.flipAuxTriggered, flipLow || flipHigh);
    autoAcroLogAuxTrigger("POWER_LOOP_FORWARD", &autoAcroRuntime.powerLoopAuxTriggered, powerLoopActive);
    autoAcroLogAuxTrigger(yawLow ? "YAW_LEFT" : "YAW_RIGHT", &autoAcroRuntime.yawAuxTriggered, yawLow || yawHigh);

    autoAcroRuntime.rollTriggerArmed = rollCentered
        ? true : ((rollLow || rollHigh) ? false : autoAcroRuntime.rollTriggerArmed);
    autoAcroRuntime.flipTriggerArmed = flipCentered
        ? true : ((flipLow || flipHigh) ? false : autoAcroRuntime.flipTriggerArmed);
    autoAcroRuntime.powerLoopTriggerArmed = powerLoopCentered
        ? true : (powerLoopActive ? false : autoAcroRuntime.powerLoopTriggerArmed);
    autoAcroRuntime.yawTriggerArmed = powerLoopCentered
        ? true : ((yawLow || yawHigh) ? false : autoAcroRuntime.yawTriggerArmed);

    if (!inFlight || autoAcroRuntime.maneuver != AUTO_ACRO_MANEUVER_NONE) {
        return;
    }

    if (config->roll && rollTriggerArmed) {
        if (rollLow) {
            autoAcroStart(AUTO_ACRO_MANEUVER_ROLL, FD_ROLL, -autoAcroDampedSpeed(config, config->rollSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.rollTriggerArmed = false;
            return;
        }
        if (rollHigh) {
            autoAcroStart(AUTO_ACRO_MANEUVER_ROLL, FD_ROLL, autoAcroDampedSpeed(config, config->rollSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.rollTriggerArmed = false;
            return;
        }
    }

    if (config->flip && flipTriggerArmed) {
        if (flipLow) {
            autoAcroStart(AUTO_ACRO_MANEUVER_FLIP, FD_PITCH, -autoAcroDampedSpeed(config, config->flipSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.flipTriggerArmed = false;
            return;
        }
        if (flipHigh) {
            autoAcroStart(AUTO_ACRO_MANEUVER_FLIP, FD_PITCH, autoAcroDampedSpeed(config, config->flipSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.flipTriggerArmed = false;
            return;
        }
    }

    if (config->powerLoop && powerLoopTriggerArmed && powerLoopActive) {
        autoAcroStart(AUTO_ACRO_MANEUVER_POWER_LOOP, FD_PITCH, autoAcroDampedSpeed(config, config->powerLoopSpeed), autoAcroThrottlePercentToRcCommand(config->powerLoopThrottle));
        autoAcroRuntime.powerLoopTriggerArmed = false;
        return;
    }

    if (config->yaw && yawTriggerArmed) {
        if (yawLow) {
            autoAcroStart(AUTO_ACRO_MANEUVER_YAW, FD_YAW, -autoAcroDampedSpeed(config, config->yawSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.yawTriggerArmed = false;
            return;
        }
        if (yawHigh) {
            autoAcroStart(AUTO_ACRO_MANEUVER_YAW, FD_YAW, autoAcroDampedSpeed(config, config->yawSpeed), AUTO_ACRO_THROTTLE);
            autoAcroRuntime.yawTriggerArmed = false;
            return;
        }
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

    if (!autoAcroRuntime.yawRotationComplete) {
        autoAcroRuntime.degreesIntegrated += fabsf(gyroRates[autoAcroRuntime.axis]) * dT;
    }

    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_YAW) {
        if (autoAcroRuntime.degreesIntegrated >= AUTO_ACRO_YAW_DEGREES) {
            autoAcroRuntime.yawRotationComplete = true;
        }

        const float pitchError = autoAcroRuntime.startPitch - attitude.values.pitch * 0.1f;
        if (autoAcroRuntime.yawRotationComplete && fabsf(pitchError) <= AUTO_ACRO_YAW_PITCH_TOLERANCE) {
            autoAcroComplete();
        }
    } else if (autoAcroRuntime.degreesIntegrated >= AUTO_ACRO_TRICK_DEGREES) {
        autoAcroComplete();
    }
}

bool autoAcroGetSetpoint(flight_dynamics_index_t axis, float *setpoint)
{
    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_YAW && axis == FD_PITCH) {
        const float pitchError = autoAcroRuntime.startPitch - attitude.values.pitch * 0.1f;
        *setpoint = constrainf(pitchError * AUTO_ACRO_YAW_PITCH_GAIN, -AUTO_ACRO_YAW_PITCH_RATE_MAX, AUTO_ACRO_YAW_PITCH_RATE_MAX);
        return true;
    }

    if (autoAcroRuntime.maneuver == AUTO_ACRO_MANEUVER_NONE || axis != autoAcroRuntime.axis) {
        return false;
    }

    *setpoint = autoAcroRuntime.yawRotationComplete ? 0.0f : autoAcroRuntime.rateSetpoint;
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

autoAcroManeuver_e autoAcroGetManeuver(void)
{
    return autoAcroRuntime.maneuver;
}

#endif // USE_AUTOACRO
