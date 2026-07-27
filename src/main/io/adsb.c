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
 *
 * ADS-B / FLARM traffic awareness. Derived from iNav's src/main/io/adsb.c.
 * The distance/bearing/vertical math (which in iNav relied on its navigation
 * subsystem) is re-based on Betaflight's GPS helpers: distance and bearing to a
 * vehicle come from GPS_distance_cm_bearing() relative to the current GPS
 * position, and the vertical delta is the vehicle MSL altitude minus the FC's
 * own GPS MSL altitude.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADSB

#include "io/adsb.h"

#include "common/maths.h"
#include "common/utils.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-function"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#include "fc/runtime_config.h"
#include "io/gps.h"
#include "sensors/sensors.h"

static adsbVehicle_t adsbVehiclesList[MAX_ADSB_VEHICLES];
static adsbVehicleStatus_t adsbVehiclesStatus;
static adsbVehicleValues_t vehicleValues;

adsbVehicleValues_t *getVehicleForFill(void)
{
    return &vehicleValues;
}

// Short (6-char) strings for the ADSB_EMITTER_TYPE enum, for the OSD.
static const char *const adsbEmitterTypeStrings[] = {
    "NOINFO", // 0  no information
    "LIGHT ", // 1  light aircraft
    "SMALL ", // 2  small aircraft
    "LARGE ", // 3  large aircraft
    "HVLARG", // 4  high vortex large
    "HEAVY ", // 5  heavy aircraft
    "HMANUV", // 6  highly maneuverable
    "ROTORC", // 7  rotorcraft
    "UNASGN", // 8  unassigned
    "GLIDER", // 9  glider / sailplane
    "LTAIR ", // 10 lighter-than-air
    "PARACH", // 11 parachute
    "ULTLIT", // 12 ultralight
    "UNASG2", // 13 unassigned 2
    "UAV   ", // 14 unmanned aerial vehicle
    "SPACE ", // 15 spacecraft
    "UNASG3", // 16 unassigned 3
    "EMRSUR", // 17 emergency surface
    "SERSUR", // 18 service surface
    "POBSTC", // 19 point obstacle
};

const char *getAdsbEmitterTypeString(uint8_t emitterType)
{
    if (emitterType < ARRAYLEN(adsbEmitterTypeStrings)) {
        return adsbEmitterTypeStrings[emitterType];
    }
    return adsbEmitterTypeStrings[0];
}

static adsbVehicle_t *findVehicleByIcao(uint32_t avicao)
{
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (avicao == adsbVehiclesList[i].vehicleValues.icao) {
            return &adsbVehiclesList[i];
        }
    }
    return NULL;
}

static adsbVehicle_t *findVehicleFarthest(void)
{
    adsbVehicle_t *adsbLocal = NULL;
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (adsbVehiclesList[i].ttl > 0 && adsbVehiclesList[i].calculatedVehicleValues.valid
            && (adsbLocal == NULL || adsbLocal->calculatedVehicleValues.dist < adsbVehiclesList[i].calculatedVehicleValues.dist)) {
            adsbLocal = &adsbVehiclesList[i];
        }
    }
    return adsbLocal;
}

uint8_t getActiveVehiclesCount(void)
{
    uint8_t total = 0;
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (adsbVehiclesList[i].ttl > 0) {
            total++;
        }
    }
    return total;
}

// find the closest vehicle, ignoring planes higher than maxVerticalDistance above us (0 = no limit)
adsbVehicle_t *findVehicleClosestLimit(int32_t maxVerticalDistance)
{
    adsbVehicle_t *adsbLocal = NULL;
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (adsbVehiclesList[i].ttl > 0 && adsbVehiclesList[i].calculatedVehicleValues.valid) {
            if (adsbVehiclesList[i].calculatedVehicleValues.verticalDistance > 0 && maxVerticalDistance > 0
                && adsbVehiclesList[i].calculatedVehicleValues.verticalDistance > maxVerticalDistance) {
                continue;
            }
            if (adsbLocal == NULL || adsbLocal->calculatedVehicleValues.dist > adsbVehiclesList[i].calculatedVehicleValues.dist) {
                adsbLocal = &adsbVehiclesList[i];
            }
        }
    }
    return adsbLocal;
}

static adsbVehicle_t *findFreeSpaceInList(void)
{
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (adsbVehiclesList[i].ttl == 0) {
            return &adsbVehiclesList[i];
        }
    }
    return NULL;
}

static adsbVehicle_t *findVehicleNotCalculated(void)
{
    for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
        if (adsbVehiclesList[i].calculatedVehicleValues.valid == false) {
            return &adsbVehiclesList[i];
        }
    }
    return NULL;
}

adsbVehicle_t *findVehicle(uint8_t index)
{
    if (index < MAX_ADSB_VEHICLES) {
        return &adsbVehiclesList[index];
    }
    return NULL;
}

adsbVehicleStatus_t *getAdsbStatus(void)
{
    return &adsbVehiclesStatus;
}

bool adsbHeartbeat(void)
{
    adsbVehiclesStatus.heartbeatMessagesTotal++;
    return true;
}

bool isEnvironmentOkForCalculatingADSBDistanceBearing(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= GPS_MIN_SAT_COUNT;
}

void recalculateVehicle(adsbVehicle_t *vehicle)
{
    if (vehicle->ttl == 0) {
        return;
    }

    uint32_t dist;
    int32_t dir;
    GPS_distance_cm_bearing(&gpsSol.llh, &vehicle->vehicleValues.gps, false, &dist, &dir);

    vehicle->calculatedVehicleValues.dist = dist;
    vehicle->calculatedVehicleValues.dir = dir; // centidegrees

    if (vehicle->calculatedVehicleValues.dist > ADSB_LIMIT_CM) {
        vehicle->ttl = 0;
        return;
    }

    vehicle->calculatedVehicleValues.verticalDistance = vehicle->vehicleValues.alt - gpsSol.llh.altCm;
    vehicle->calculatedVehicleValues.valid = true;
}

void adsbNewVehicle(adsbVehicleValues_t *vehicleValuesLocal)
{
    if (vehicleValuesLocal->icao == 0) {
        return;
    }

    // no valid lat/lon or altitude
    if ((vehicleValuesLocal->flags & (ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_COORDS)) != (ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_COORDS)) {
        return;
    }

    adsbVehiclesStatus.vehiclesMessagesTotal++;

    adsbVehicle_t *vehicle = findVehicleByIcao(vehicleValuesLocal->icao);

    if (vehicleValuesLocal->tslc > ADSB_MAX_SECONDS_KEEP_INACTIVE_PLANE_IN_LIST) {
        if (vehicle != NULL) {
            vehicle->ttl = 0;
        }
        return;
    }

    if (!isEnvironmentOkForCalculatingADSBDistanceBearing()) {
        // no GPS fix: keep the vehicle in the list, but without calculated values
        if (vehicle == NULL) {
            vehicle = findFreeSpaceInList();
        }
        if (vehicle != NULL) {
            memcpy(&(vehicle->vehicleValues), vehicleValuesLocal, sizeof(vehicle->vehicleValues));
            vehicle->ttl = MAX(0, ADSB_MAX_SECONDS_KEEP_INACTIVE_PLANE_IN_LIST - vehicleValuesLocal->tslc);
            vehicle->calculatedVehicleValues.valid = false;
        }
        return;
    }

    // GPS fix present: assign a slot, preferring free/uncalculated, else replace the farthest if closer
    if (vehicle == NULL) {
        vehicle = findFreeSpaceInList();
    }
    if (vehicle == NULL) {
        vehicle = findVehicleNotCalculated();
    }
    if (vehicle == NULL) {
        vehicle = findVehicleFarthest();
        if (vehicle != NULL) {
            uint32_t newDist;
            GPS_distance_cm_bearing(&gpsSol.llh, &vehicleValuesLocal->gps, false, &newDist, NULL);
            if (newDist > vehicle->calculatedVehicleValues.dist) {
                // the plane already stored is closer, keep it
                vehicle = NULL;
            }
        }
    }

    if (vehicle != NULL) {
        memcpy(&(vehicle->vehicleValues), vehicleValuesLocal, sizeof(vehicle->vehicleValues));
        recalculateVehicle(vehicle);
        vehicle->ttl = MAX(0, ADSB_MAX_SECONDS_KEEP_INACTIVE_PLANE_IN_LIST - vehicleValuesLocal->tslc);
    }
}

void adsbTtlClean(timeUs_t currentTimeUs)
{
    static timeUs_t adsbTtlLastCleanServiced = 0;
    timeDelta_t adsbTtlSinceLastCleanServiced = cmpTimeUs(currentTimeUs, adsbTtlLastCleanServiced);

    if (adsbTtlSinceLastCleanServiced > 1000000) { // 1s
        for (uint8_t i = 0; i < MAX_ADSB_VEHICLES; i++) {
            if (adsbVehiclesList[i].ttl > 0) {
                adsbVehiclesList[i].ttl--;
            }
            if (adsbVehiclesList[i].ttl > 0) {
                recalculateVehicle(&adsbVehiclesList[i]);
            }
        }
        adsbTtlLastCleanServiced = currentTimeUs;
    }
}

#endif // USE_ADSB
