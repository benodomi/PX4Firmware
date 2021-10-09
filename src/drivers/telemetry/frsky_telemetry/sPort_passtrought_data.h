/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sPort_passtrought_data.h
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky SmartPort passtrought telemetry implementation.
 *
 */
#ifndef _SPORT_DATA_H
#define _SPORT_DATA_H

#include <sys/types.h>
#include <stdbool.h>

#define PASSTROUGHT_ID_ROLLPITCH   0x5006		// roll, pitch, range
#define PASSTROUGHT_ID_VELANDYAW   0x5005		// vertical speed, yaw
#define PASSTROUGHT_ID_AP_STATUS   0x5001       // Flight mode, SimpleMode, LandComplete, statusArmed, battFailSafe, ekfFailSafe, failSafe, fencePresent, fenceBreached, throttle, imuTemp
#define PASSTROUGHT_ID_GPS_STATUS  0x5002       // numSats, gpsStatus, HdopC, alt
#define PASSTROUGHT_ID_BATT1       0x5003       // bat1Vol, bat1Current, bat1mah
#define PASSTROUGHT_ID_BATT2       0x5008       // bat2Vol, bat2Current, bat2mah
#define PASSTROUGHT_ID_HOME        0x5004       // homeDist, homeAlt, homeAngle
#define PASSTROUGHT_ID_MESSAGES    0x5000
#define PASSTROUGHT_ID_PARAMS      0x5007        // ParamID, paramValue
#define PASSTROUGHT_ID_WAYPOINTS   0x5009        // wpNumber, wpDistance, wpXTError, wpBearing
#define PASSTROUGHT_ID_RPM         0x500A        //RPM1, RPM2
#define PASSTROUGHT_ID_TERRAIN     0x500B        // HeightAboveTerrain, TerrainUnhealty
#define PASSTROUGHT_ID_RC_CHANNELS 0x50F1        // channels 1-32
#define PASSTROUGHT_ID_VFR         0x50F2        // Airspeed, throttle, baro_alt_meter

// Public functions
bool sPort_init(void);
void sPort_deinit(void);
void sPort_update_topics(void);
void sPort_send_data(int uart, uint16_t id, uint32_t data);
void sPort_send_BATV(int uart);
void sPort_send_CUR(int uart);
void sPort_send_ALT(int uart);
void sPort_send_RPM(int uart);
void sPort_send_SPD(int uart);
void sPort_send_VSPD(int uart, float speed);
void sPort_send_FUEL(int uart);
void sPort_send_GPS_LON(int uart);
void sPort_send_GPS_LAT(int uart);
void sPort_send_GPS_ALT(int uart);
void sPort_send_GPS_SPD(int uart);
void sPort_send_GPS_CRS(int uart);
void sPort_send_GPS_TIME(int uart);
void sPort_send_flight_mode(int uart);
void sPort_send_GPS_info(int uart);

void sPort_send_NAV_STATE(int uart);
void sPort_send_GPS_FIX(int uart);

#endif /* _SPORT_TELEMETRY_H */
