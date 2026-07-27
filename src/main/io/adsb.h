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
 * ADS-B / FLARM traffic awareness. Derived from iNav's src/main/io/adsb.c,
 * with the distance/bearing math re-based on Betaflight's GPS helpers.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "io/gps.h"

#define ADSB_CALL_SIGN_MAX_LENGTH 9
#define ADSB_MAX_SECONDS_KEEP_INACTIVE_PLANE_IN_LIST 10
#define MAX_ADSB_VEHICLES 5
#define ADSB_LIMIT_CM 6400000 // 64 km - beyond this a vehicle is dropped

typedef struct {
    bool valid;
    int32_t dir;                // centidegrees, direction from the FC to the plane
    uint32_t dist;              // cm, horizontal distance from the FC to the plane
    int32_t verticalDistance;   // cm, vertical distance (plane - FC), positive = plane above
} adsbVehicleCalculatedValues_t;

typedef struct {
    uint32_t icao;              // ICAO address
    uint16_t horVelocity;       // cm/s
    gpsLocation_t gps;          // plane position (lat/lon)
    int32_t alt;                // cm, barometric/geometric altitude (MSL)
    uint16_t heading;           // centidegrees, course over ground
    uint16_t flags;             // ADSB_FLAGS bitmask
    uint8_t altitudeType;       // ADSB_ALTITUDE_TYPE
    char callsign[ADSB_CALL_SIGN_MAX_LENGTH]; // 8 chars + NUL
    uint8_t emitterType;        // ADSB_EMITTER_TYPE
    uint8_t tslc;               // s, time since last communication
} adsbVehicleValues_t;

typedef struct {
    adsbVehicleValues_t vehicleValues;
    adsbVehicleCalculatedValues_t calculatedVehicleValues;
    uint8_t ttl;
} adsbVehicle_t;

typedef struct {
    uint32_t vehiclesMessagesTotal;
    uint32_t heartbeatMessagesTotal;
} adsbVehicleStatus_t;

void adsbNewVehicle(adsbVehicleValues_t *vehicleValuesLocal);
bool adsbHeartbeat(void);
adsbVehicle_t *findVehicleClosestLimit(int32_t maxVerticalDistance);
adsbVehicle_t *findVehicle(uint8_t index);
uint8_t getActiveVehiclesCount(void);
void adsbTtlClean(timeUs_t currentTimeUs);
adsbVehicleStatus_t *getAdsbStatus(void);
adsbVehicleValues_t *getVehicleForFill(void);
bool isEnvironmentOkForCalculatingADSBDistanceBearing(void);
void recalculateVehicle(adsbVehicle_t *vehicle);
const char *getAdsbEmitterTypeString(uint8_t emitterType);
