// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// adsb2flarm.c
//
// Copyright (c) 2022 Denis G Dugushkin (denis.dugushkin@gmail.com)
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"

struct _Modes Modes;
struct _Sensors sensbox;

#include <stdarg.h>

#include "sensors.h"
#include "beeper.h"
#include "adsb2flarm.h"
#include "nmea_utils.h"
#include "uart_io.h"

#include <sys/time.h>

char* pflau_not_ready = "$PFLAU,0,0,0,1,0,,,,,*7F\r\n";

void tcas(void);
void sync_rtc_from_gnss();

void broadcastNMEA(struct nmea_msg *m);
void broadcastInternalNMEA(char *nmea, nmea_sentence_t type);

long myclock(void);

long myclock(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000000) + tv.tv_usec;
}

void receiverPositionChanged(float lat, float lon, float alt) {
	/* nothing */
	(void) lat;
	(void) lon;
	(void) alt;
}

static inline void update_altitude_sensors(void) {

	static uint64_t alt_next_update;

	uint64_t now = mstime();
	if (now < alt_next_update) {
		return;
	}

	// scan once a 1/100 second at most
	alt_next_update = now + modesf.altitude_update_time;

	updateAllBaroSensorsHW();
	periodicBaroSensorCheck();

	if (sensbox.active_altitude) {
		if (sensbox.active_altitude->hw == SENSOR_BARO_BMX280) {
			char nmea[NMEA_MAX_MESSAGE];
			generate_nmea_pgrmz(nmea, NMEA_MAX_MESSAGE,
					sensbox.active_altitude->altitude_feet);
			broadcastInternalNMEA(nmea, NMEA_ALT);
		}
	}
}

#define randnum(min, max) \
    ((rand() % (int)(((max) + 1) - (min))) + (min))

static void generate_aircraft_circle(int head) {

	char nmea[NMEA_MAX_MESSAGE];
	struct pflaa_data pflaa;
		for (int i = 0; i < 12; i++) {
			pflaa.ID = (i + 1) * 1024;
			pflaa.IDType = 1;
			pflaa.AlarmLevel = 0;

			if (i == head) pflaa.AlarmLevel = 3;
			if (i == (head + 1)) pflaa.AlarmLevel = 2;
			if (i == (head + 2)) pflaa.AlarmLevel = 1;

			pflaa.RelativeNorth = 5000 * cos(2 * M_PI * i / 12);
			pflaa.RelativeEast = 5000 * sin(2 * M_PI * i / 12);
			pflaa.RelativeVertical = (head + 1) * 100;
			pflaa.Track = i * 3 + head * 20;
			if (pflaa.Track >= 360) pflaa.Track -= 360;
			pflaa.ClimbRate = (float) randnum(0, 32);
			pflaa.GroundSpeed = 100 + randnum(0, 50);
			pflaa.TurnRate = 0;
			pflaa.AcftType = 9;
			pflaa.Source = 1;
			pflaa.NoTrack = 0;
			pflaa.RSSI = 11.1;
			generate_nmea_pflaa(nmea, NMEA_MAX_MESSAGE, &pflaa);
			broadcastInternalNMEA(nmea, NMEA_FLARM_PFLAA);
		}

		struct pflau_data pflau;
		pflau.RX = 12;
		pflau.TX = 1;
		pflau.GPS = 2;
		pflau.Power = 1;
		pflau.AlarmLevel = 0;
		pflau.AlarmType = 0;
		pflau.RelativeBearing = 0;
		pflau.RelativeVertical = 0;
		pflau.RelativeDistance = 0;
		pflau.ID = 0;
		generate_nmea_pflau(nmea, NMEA_MAX_MESSAGE, &pflau);
		broadcastInternalNMEA(nmea, NMEA_FLARM_PFLAU);
}

static inline void bme280_send_nmea(const double p, const double t, const double h) {

	char nmea[NMEA_MAX_MESSAGE];
	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX HUMIDITY: %0.1f %%", h);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastInternalNMEA(nmea, NMEA_TXT);

	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX PRESSURE: %0.2f hPa", p / 100);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastInternalNMEA(nmea, NMEA_TXT);

	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX TEMPERATURE: %0.1f C", t);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastInternalNMEA(nmea, NMEA_TXT);
}

static inline void print_gnss() {

	static uint64_t print_next_update;

	uint64_t now = mstime();
	if (now < print_next_update) {
		return;
	}

	// scan once a 1/2 second at most
	print_next_update = now + 500;

	struct gnss_data *pri, *sec, *act;
	pri = &sensbox.gnss_pri;
	sec = &sensbox.gnss_sec;
	act = sensbox.active_gnss;
	system("clear");
	printf("active_gnss:         %s\n\n", act ? act->name : "(null)");
	printf("                     %s %s\n", pri->name, sec->name);
	printf("gnss_time:           %jd%14jd\n",(intmax_t)pri->gnss_time.tv_sec, (intmax_t)sec->gnss_time.tv_sec);
	printf("gnss_lat:            %0.4f         %0.4f\n", pri->lat, sec->lat);
	printf("gnss_lon:            %0.4f         %0.4f\n", pri->lon, sec->lon);
	printf("gnss_speed:          %0.4f         %0.4f\n", pri->speed, sec->speed);
	printf("gnss_course:         %0.4f         %0.4f\n", pri->course, sec->course);
	printf("gnss_true_track:     %0.4f         %0.4f\n", pri->true_track_degrees, sec->true_track_degrees);
	printf("gnss_mag_track:      %0.4f         %0.4f\n", pri->magnetic_track_degrees, sec->magnetic_track_degrees);
	printf("gnss_speed_kph:      %0.4f         %0.4f\n", pri->speed_kph, sec->speed_kph);
	printf("gnss_variation:      %0.4f         %0.4f\n", pri->variation, sec->variation);
	printf("gnss_altitude_m:     %0.4f%c       %0.4f%c\n", pri->altitude_gnss, pri->altitude_gnss_units, sec->altitude_gnss, sec->altitude_gnss_units);
	printf("gnss_altitude_f:     %0.4f         %0.4f\n", pri->altitude_gnss_feets, sec->altitude_gnss_feets);
	printf("gnss_altitude_valid: %s            %s\n", pri->alt_gnss_valid ? "true":"false", sec->alt_gnss_valid ? "true":"false");
	printf("gnss_fix_qual:       %u            %u\n", pri->fix_quality, sec->fix_quality);
	printf("gnss_fix_type:       %u            %u\n", pri->fix_type, sec->fix_type);
	printf("gnss_pdop:           %0.4f         %0.4f\n", pri->pdop, sec->pdop);
	printf("gnss_hdop:           %0.4f         %0.4f\n", pri->hdop, sec->hdop);
	printf("gnss_vdop:           %0.4f         %0.4f\n", pri->vdop, sec->vdop);
	printf("gnss_sat_trk:        %d            %d\n", pri->satellites_tracked, sec->satellites_tracked);
	printf("gnss_score:          %lu           %lu\n", pri->score, sec->score);
	printf("gnss_valid:          %s            %s\n", pri->gnss_valid ? "true":"false", sec->gnss_valid ? "true":"false");
	printf("gnss_active:         %s            %s\n", pri->active ? "true":"false", sec->active ? "true":"false");
	printf("gnss_rate:           %u            %u\n", pri->gnss_rate, sec->gnss_rate);
	printf("gnss_update_rate:    %u            %u\n", pri->rate, sec->rate);
	printf("gnss_last_upd:       %lu           %lu\n", pri->last_update, sec->last_update);
	printf("\n");
	printf("baro_active_addr:    %p\n", (void*) sensbox.active_altitude);
	if (sensbox.active_altitude) {
	printf("baro_feet:           %0.4f\n", sensbox.active_altitude->altitude_feet);
	printf("baro_m:              %0.4f\n", sensbox.active_altitude->altitude_m);
	}

	for(int i=0; i<ALTITUDE_SENSORS_COUNT;i++) {
		printf("_main Addr: %p VALID: %s ALT_F: %0.1f  ALT_M: %0.1f\n", (void*) &sensbox.alt_data[i], sensbox.alt_data[i].valid ? "true" : "false", sensbox.alt_data[i].altitude_feet, sensbox.alt_data[i].altitude_m);
	}
	printf("\n");

	for(int i=0; i<sensbox.alt_sensor_oount;i++) {
		printf("_prior Addr: %p valid: %s ALT_F: %0.1f  ALT_M: %0.1f\n", (void*) &sensbox.alt_data[sensbox.alt_priority[i]], sensbox.alt_data[sensbox.alt_priority[i]].valid ? "true" : "false", sensbox.alt_data[sensbox.alt_priority[i]].altitude_feet, sensbox.alt_data[sensbox.alt_priority[i]].altitude_m);
	}
	printf("\n");

}

static inline void relative_north_east_position(double lat1, double lon1,
		double lat2, double lon2, int32_t *north, int32_t *east) {
	int32_t bearing = (int32_t) get_bearing(lat1, lon1, lat2, lon2);
	int32_t is_north;
	int32_t is_east;

	if (bearing >= 270 || bearing <= 90)
		is_north = 1;
	else
		is_north = -1;

	if (bearing >= 0 && bearing <= 180)
		is_east = 1;
	else
		is_east = -1;

	*north = is_north * (int32_t) greatcircle(lat1, lon1, lat2, lon1);
	*east = is_east * (int32_t) greatcircle(lat1, lon1, lat1, lon2);
}

/*
static inline int sine_sign(double angle) {
	return ((angle >= 0.0 && angle <= 90.0) || (angle >= 270.0 && angle < 360.0)) ?
			1 : -1;
}

static inline int cosine_sign(double angle) {
	return (angle >= 0.0 && angle <= 180.0) ? 1 : -1;
}

static inline bool sine_limit(double bearing) {
	return (bearing >= 270.0 && bearing <= 300.0)
			|| (bearing >= 90.0 && bearing <= 120.0);

}

static inline double rotate_ccw(double angle, double offset) {
	return (angle - offset) >= 0.0 ? angle - offset : 360.0 - (offset - angle);
}

static inline bool compare_direction(double bearing1, double bearing2,
		double offset) {

	bearing1 = rotate_ccw(bearing1, offset);
	bearing2 = rotate_ccw(bearing2, offset);

	return sine_sign(bearing1) > 0 ?
			((sine_sign(bearing1) + sine_sign(bearing2) == 0)
					|| sine_limit(bearing2))
					&& (cosine_sign(bearing1) + cosine_sign(bearing2) == 0) :
			(sine_sign(bearing1) + sine_sign(bearing2) == 0)
					&& (cosine_sign(bearing1) + cosine_sign(bearing2) == 0);
}
*/

/*
 #define sine_sign(angle) (((angle >= 0.0 && angle <= 90.0) || (angle >= 270.0 && angle < 360.0)) ? 1 : -1)
 #define cosine_sign(angle) ((angle >= 0.0 && angle <= 180.0) ? 1 : -1)
 #define sine_limit (bearing) (bearing >= 270.0 && bearing <= 300.0) || (bearing >= 90.0 && bearing <= 120.0) ? true : false
 #define compare_direction(bearing1, bearing2) (sine_sign(bearing1) > 0 ? ((sine_sign(bearing1) + sine_sign(bearing2) == 0) || sine_limit(bearing2)) && (cosine_sign(bearing1) + cosine_sign(bearing2) == 0) : (sine_sign(bearing1) + sine_sign(bearing2) == 0) && (cosine_sign(bearing1) + cosine_sign(bearing2) == 0))
 */


/* Generic */

/*
static inline bool flarm_compare_speed_and_bearing(float v1, float bearing1, float v2, float bearing2, float rel_bearing) {
	float Vtx = (v1 * sin(bearing1 * M_PI/180) - v2 * sin(bearing2 * M_PI/180)) * sin(rel_bearing * M_PI/180); // knots
	float Vty = (v1 * cos(bearing1 * M_PI/180) - v2 * cos(bearing2 * M_PI/180)) * cos(rel_bearing * M_PI/180); // knots
	return (Vtx + Vty) > 0.0;
}
*/

/* Optimized version */
/* Critically important to keep parameters' order. V1 should be for glider and V2 for other aircraft */
static inline bool flarm_compare_speed_and_bearing(float v1, float bearing1, float v2, float bearing2, float rel_bearing) {
	float Vt1 = v1 * cos((rel_bearing - bearing1) * M_PI/180); // knots
	float Vt2 = v2 * cos((rel_bearing - bearing2) * M_PI/180); // knots
	return (Vt1 - Vt2) > 0.0;
}


/*
static inline bool flarm_compare_speed_and_bearing_std(float v1, float bearing1, float v2, float bearing2, float rel_bearing, struct speeds *result) {
	float Vtx = (v1 * sin(bearing1 * M_PI/180) - v2 * sin(bearing2 * M_PI/180)) * sin(rel_bearing * M_PI/180); // knots
	float Vty = (v1 * cos(bearing1 * M_PI/180) - v2 * cos(bearing2 * M_PI/180)) * cos(rel_bearing * M_PI/180); // knots
	result->V1 = Vtx;
	result->V2 = Vty;
	return true;
}

static inline bool flarm_compare_speed_and_bearing_cosine(float v1, float bearing1, float v2, float bearing2, float rel_bearing, struct speeds *result) {
	float Vt1 = v1 * cos((rel_bearing - bearing1) * M_PI/180); // knots
	float Vt2 = v2 * cos((rel_bearing - bearing2) * M_PI/180); // knots
	result->V1 = Vt1;
	result->V2 = Vt2;
	return true;
}

static inline void compare_alrorithms() {
	struct speeds result1, result2;
	float sum1, sum2;
	bool cmp;

	flarm_compare_speed_and_bearing_std(400.0, 60.0, 100.0, 0.0, 30.0, &result1);
	flarm_compare_speed_and_bearing_cosine(400.0, 60.0, 100.0, 0.0, 30.0, &result2);

	sum1 = result1.V1 + result1.V2;
	sum2 = result2.V1 - result2.V2;
	cmp = fabs(sum1 - sum2) < 0.01;

	printf("rb = %d, b1 = %d, b2 = %d :: STD: (Vx+Vy) = %0.2f; COSINE: (V1+V2) = %0.2f, comparing = %s\n", 30, 60, 0, sum1, sum2, cmp ? "true" : "false");


	uint64_t t1 = mstime();
	printf("STD starts at %lu\n...", t1);
	for(int rel_b=0; rel_b<360; rel_b++) {
		for(int bearing1=0; bearing1<360; bearing1++)
			for(int bearing2=0; bearing2<360; bearing2++) {
				flarm_compare_speed_and_bearing_std(450.0, (float) bearing1, 100.0, (float) bearing2, (float) rel_b, &result1);
				//flarm_compare_speed_and_bearing_cosine(450.0, (float) bearing1, 100.0, (float) bearing2, (float) rel_b, &result2);

				sum1 = result1.V1 + result1.V2;
				//UNUSED(sum1);
				if(sum1>10000000.0) printf("Error!\n");
				//sum2 = result2.V1 - result2.V2;
				//cmp = fabs(sum1 - sum2) < 0.01;
				//if(!cmp)
				//printf("rb = %d, b1 = %d, b2 = %d :: STD: (Vx+Vy) = %0.2f; COSINE: (V1+V2) = %0.2f, comparing = %s\n", rel_b, bearing1, bearing2, sum1, sum2, cmp ? "true" : "false");
			}
	}

	uint64_t t2 = mstime() - t1;
	printf("STD finished, elapsed %lu ms\n...", t2);

	t1 = mstime();
	printf("COSINE starts at %lu\n...", t1);
	for(int rel_b=0; rel_b<360; rel_b++) {
		for(int bearing1=0; bearing1<360; bearing1++)
			for(int bearing2=0; bearing2<360; bearing2++) {
				//flarm_compare_speed_and_bearing_std(450.0, (float) bearing1, 100.0, (float) bearing2, (float) rel_b, &result1);
				flarm_compare_speed_and_bearing_cosine(450.0, (float) bearing1, 100.0, (float) bearing2, (float) rel_b, &result2);

				//sum1 = result1.V1 + result1.V2;
				sum2 = result2.V1 - result2.V2;
				if(sum2>10000000.0) printf("Error!\n");
				//cmp = fabs(sum1 - sum2) < 0.01;
				//if(!cmp)
				//printf("rb = %d, b1 = %d, b2 = %d :: STD: (Vx+Vy) = %0.2f; COSINE: (V1+V2) = %0.2f, comparing = %s\n", rel_b, bearing1, bearing2, sum1, sum2, cmp ? "true" : "false");
			}
	}
	t2 = mstime() - t1;
	printf("COSINE finished, elapsed %lu ms\n...", t2);
}
*/

static inline int cat_dump1090_to_flarm(unsigned char d_cat) {
	int flarm_cat = 0;

	switch(d_cat) {
	case 0xA1: flarm_cat = 0x08; break; // aircraft with reciprocating engine(s)
	case 0xA7: flarm_cat = 0x03; break; // helicopter/gyrocopter/rotorcraft
	case 0xA2: case 0xA3: case 0xA5: default: flarm_cat = 0x09; break; // aircraft with jet/turboprop engine(s)
	case 0xB1: flarm_cat = 0x01; break; // glider/motor glider (turbo, self-launch, jet) / TMG
	case 0xB2: flarm_cat = 0x0B; break; // balloon (hot, gas, weather, static)
	case 0xB3: flarm_cat = 0x04; break; // skydiver, parachute (Do not use for drop plane!)
	}
	return flarm_cat;
}

static inline int determine_flarm_level(struct aircraft *a) {

	int flarm_level = -1;

	// Check if GNSS and Altitude available
	if (!sensbox.active_gnss || !sensbox.active_altitude) {
		broadcastInternalNMEA(pflau_not_ready, NMEA_FLARM_PFLAU);
		return flarm_level;
	}

	// Proceed only reliable aircrafts
	if (!a->reliable) return flarm_level;

	// Skip aircrafts without position
	if (!trackDataValid(&a->position_valid)) return flarm_level;

	// Skip aircrafts on the ground
	if (trackDataValid(&a->airground_valid)
			&& a->airground_valid.source >= SOURCE_MODE_S_CHECKED
			&& a->airground == AG_GROUND)
		return flarm_level;


	struct gnss_data *gnss_active = sensbox.active_gnss;
	float self_lat = gnss_active->lat; // 59.7992; //59.931770;
	float self_lon = gnss_active->lon; // 30.7736;//31.797262;
	float self_track = gnss_active->course;
	float self_alt_baro_feet = sensbox.active_altitude->altitude_feet;//gnss_active->altitude_baro_feets;
	float self_speed = gnss_active->speed; // knots(!)

	// Absolute distance to aircraft
	float distance = greatcircle(self_lat, self_lon, a->lat, a->lon);
	// Skip so far aircrafts
	if (distance > flarm_param.distance) return flarm_level;

	float abs_alt_diff = COEF_FEETS_TO_METERS * fabs(a->altitude_baro - self_alt_baro_feet); // Absolute difference

    if (abs_alt_diff <= flarm_param.level_3) {
    	flarm_level = 3;  // FLARM level 3
    } else
    if (abs_alt_diff <= flarm_param.level_2) {
    	flarm_level = 2;  // FLARM level 2
    } else
    if (abs_alt_diff <= flarm_param.level_1) {
    	flarm_level = 1;  // FLARM level 1
    } else
    if (abs_alt_diff <= flarm_param.level_0) {
    	flarm_level = 0;  // FLARM level 0 (display, but no audio signal)
    } else return flarm_level;

    float rel_bearing = get_bearing(self_lat, self_lon, a->lat, a->lon);

    /* Critically important to keep parameters' order. V1 should be for glider and V2 for other aircraft */
    flarm_level = flarm_compare_speed_and_bearing(self_speed, self_track, a->gs, a->track, rel_bearing) ? flarm_level : 0;

  return flarm_level;
}

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define inc(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a = _a + _b; })


void tcas(void) {
	struct aircraft *a;
	int total = 0;
	int max_flarm_level = 0;
	char nmea[NMEA_MAX_MESSAGE];

	static uint64_t tcas_next_update;

	uint64_t now = mstime();
	if (now < tcas_next_update)
		return;

	// scan once a 1/100 second at most
	tcas_next_update = now + modesf.flarm_update_time;

	// Check if GNSS and Altitude available
	if (!sensbox.active_gnss || !sensbox.active_altitude) {
		broadcastInternalNMEA(pflau_not_ready, NMEA_FLARM_PFLAU);
		return;
	}

	struct gnss_data *gnss_active = sensbox.active_gnss;

	float self_lat = gnss_active -> lat; // 59.7992; //59.931770;
	float self_lon = gnss_active -> lon; // 30.7736;//31.797262;
	float self_alt_baro = sensbox.active_altitude->altitude_feet;

	for (a = Modes.aircrafts; a; a = a->next) {
		// Skip aircrafts if there is no data for 5000 ms
		if ((now - a->seen) > 5000) continue;

		if (trackDataValid(&a->track_valid)) {
			int32_t north, east;

			int flarm_level = determine_flarm_level(a);
			//printf("%d \n", flarm_level);
			if (flarm_level < 0) continue;

			total++;
			max_flarm_level = max(max_flarm_level, flarm_level);

			relative_north_east_position(self_lat, self_lon, a->lat, a->lon,
					&north, &east);

			struct pflaa_data pflaa;
			pflaa.ID = a->addr & 0xFFFFFF; // HEX
			pflaa.IDType = 1; // official ICAO 24-bit aircraft address
			pflaa.AlarmLevel = flarm_level;
			pflaa.RelativeNorth = north;
			pflaa.RelativeEast = east;
			pflaa.RelativeVertical = COEF_FEETS_TO_METERS * (a->altitude_baro - self_alt_baro); // feets to meters
			pflaa.Track = (int) a->track;
			pflaa.ClimbRate = a->baro_rate / 197; // convert [fpm] to [m/s]
			pflaa.GroundSpeed = (int) (a->gs * 0.514444); // convert [knots] to [m/s]
			pflaa.TurnRate = (int) a->roll;
			pflaa.AcftType = cat_dump1090_to_flarm(a->category);
			pflaa.Source = 1; // ADS-B
			pflaa.NoTrack = 0;
			pflaa.RSSI = 10
					* log10(
							(a->signalLevel[0] + a->signalLevel[1]
									+ a->signalLevel[2] + a->signalLevel[3]
									+ a->signalLevel[4] + a->signalLevel[5]
									+ a->signalLevel[6] + a->signalLevel[7]
									+ 1e-5) / 8);
			generate_nmea_pflaa(nmea, NMEA_MAX_MESSAGE, &pflaa);
			broadcastInternalNMEA(nmea, NMEA_FLARM_PFLAA);

		}
	}
	struct pflau_data pflau;
	pflau.RX = total;
	if (sensbox.pflau_pri.valid) inc(pflau.RX, sensbox.pflau_pri.ac_count);
	if (sensbox.pflau_sec.valid) inc(pflau.RX, sensbox.pflau_sec.ac_count);

	pflau.TX = 1;
	pflau.GPS = 2;
	pflau.Power = 1;
	pflau.AlarmLevel = 0;
	pflau.AlarmType = 0;
	pflau.RelativeBearing = 0;
	pflau.RelativeVertical = 0;
	pflau.RelativeDistance = 0;
	pflau.ID = 0;
	generate_nmea_pflau(nmea, NMEA_MAX_MESSAGE, &pflau);
	broadcastInternalNMEA(nmea, NMEA_FLARM_PFLAU);

	beeper_play_flarm(&modesf.beeper, max_flarm_level);
}

void sync_rtc_from_gnss() {

	static uint64_t gnss_time_next_update;

	uint64_t now = mstime();
	if (now < gnss_time_next_update)
		return;

	// update once a 10 minutes
	gnss_time_next_update = now + 1000 * 600;

	struct gnss_data *gnss_active = sensbox.active_gnss;
	if (!gnss_active) return;

	if (gnss_active->gnss_time.tv_sec > 1680363134) { // It's just a magic value

    int rc = clock_settime(CLOCK_REALTIME, &gnss_active->gnss_time);
	char nmea[NMEA_MAX_MESSAGE];
    snprintf(nmea, NMEA_MAX_MESSAGE, rc == 0 ?
    		"SYSTEM TIME WAS SUCCESSFULLY SYNCHRONIZED WITH GNSS TIME: %d, %s":
    		"FAILED TO SYNC SYSTEM TIME WITH GNSS TIME: ERROR CODE: %d, %s", errno, strerror(errno));
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastInternalNMEA(nmea, NMEA_TXT);
	}
}

//
// =============================== Initialization ===========================
//
static void globalInitConfig(void) {
	// Default everything to zero/NULL
	memset(&Modes, 0, sizeof(Modes));

	// Now initialise things that should not be 0/NULL to their defaults
	Modes.nfix_crc = 1;
	Modes.check_crc = 1;
	Modes.fix_df = 1;
	Modes.net = 1;
	Modes.net_heartbeat_interval = MODES_NET_HEARTBEAT_INTERVAL;
	Modes.maxRange = 1852 * 360; // 360NM default max range; this also disables receiver-relative positions
	Modes.quiet = 1;
	Modes.net_output_flush_size = 128;
	Modes.net_output_flush_interval = 10; // milliseconds
	Modes.faup_rate_multiplier = FAUP_DEFAULT_RATE_MULTIPLIER;
	modesf.uart_pri_out = false;
	modesf.uart_sec_out = false;
	modesf.beeper.gpio_pin = BEEPER_PIN;

	modesf.net_filter.enable_flarm = false;
	modesf.net_filter.enable_gnss = false;
	modesf.net_filter.enable_baro = false;
	modesf.net_filter.enable_unknown = false;
	modesf.net_filter.enable_invalid = false;

	modesf.uart_filter.enable_flarm = false;
	modesf.uart_filter.enable_gnss = false;
	modesf.uart_filter.enable_baro = false;
	modesf.uart_filter.enable_unknown = false;
	modesf.uart_filter.enable_invalid = false;

	modesf.altitude_update_time = ALTITUDE_UPDATE_TIME;

	// Debug and test modes
	modesf.debug_targets_test = false;
	modesf.debug_beeper_test = false;
	modesf.debug_view = false;
	modesf.startup_test = 0;

	// Default FLARM settings
	flarm_param.distance = FLARM_DISTANCE;
	flarm_param.level_0 = FLARM_ALT_LEVEL0;
	flarm_param.level_1 = FLARM_ALT_LEVEL1;
	flarm_param.level_2 = FLARM_ALT_LEVEL2;
	flarm_param.level_3 = FLARM_ALT_LEVEL3;
	modesf.flarm_update_time = FLARM_UPDATE_TIME;

}

//
//=========================================================================
//
static void globalInit(void) {
	// Validate the users Lat/Lon home location inputs
	if ((Modes.fUserLat > 90.0)  // Latitude must be -90 to +90
	|| (Modes.fUserLat < -90.0)  // and
			|| (Modes.fUserLon > 360.0)  // Longitude must be -180 to +360
			|| (Modes.fUserLon < -180.0)) {
		Modes.fUserLat = Modes.fUserLon = 0.0;
	} else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
		Modes.fUserLon -= 360.0;
	}
	// If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
	// Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
	// Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
	// is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both.
	// Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
	Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
	if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
		Modes.bUserFlags |= MODES_USER_LATLON_VALID;
	}

	// Prepare error correction tables
	modesChecksumInit(1);
	icaoFilterInit();
	modeACInit();
}

/* Extremely unsafe but in this case quite enough */
/* We should restore deleted the /n delimeter in the parsing routine before */
static inline void restoreDelim(char *s) {
	char *p = s + strlen(s);
	*p = '\n';
	p++;
	*p = '\0';
}

void broadcastNMEA(struct nmea_msg *m) {

	bool send_net = false;
	bool send_uart = false;

	//printf("_broadcastNMEA(): m=%p, sensbox.active_gnss=%p, flarm_io.flarm_net_service=%p, flarm_io.flarm_serial_pri_service=%p, flarm_io.flarm_serial_sec_service=%p\n", m, sensbox.active_gnss, flarm_io.flarm_net_service, flarm_io.flarm_serial_pri_service, flarm_io.flarm_serial_sec_service);
	//printf("m->type=%d, m-source=%d, m->nmea=%s\n", m->type, m->source, m->nmea);

	switch(m->type) {
	case NMEA_GNSS:
		//printf("GNSS\n");
		//if(!sensbox.active_gnss) break;
		//bool cond = sensbox.active_gnss ? sensbox.active_gnss->source == m->source : true;
		if(sensbox.active_gnss ? sensbox.active_gnss->source == m->source : true) {
			send_net = modesf.net_filter.enable_gnss;
			send_uart = modesf.uart_filter.enable_gnss;
		}
		break;
	case NMEA_FLARM_PFLAU: {
		//printf("PFLAU\n");
		send_net = modesf.net_filter.enable_flarm && (m->source == SRC_INTERNAL) ? true : false;
		send_uart = modesf.uart_filter.enable_flarm && (m->source == SRC_INTERNAL) ? true : false;
		break;
	}
	case NMEA_FLARM_PFLAA:
		//printf("PFLAA\n");
		send_net = modesf.net_filter.enable_flarm;
		send_uart = modesf.uart_filter.enable_flarm;
		break;
	case NMEA_ALT:
		//printf("ALTITUDE\n");
		//printf("alt=%p, m->type=%d, m-source=%d, m->nmea=%s\n", sensbox.active_altitude, m->type, m->source, m->nmea);
		if(!sensbox.active_altitude) break;
		//printf("active_altitude->source=%d\n", sensbox.active_altitude->source);
		if(sensbox.active_altitude->source == m->source) {
			send_net = modesf.net_filter.enable_baro;
			send_uart = modesf.uart_filter.enable_baro;
		}
		break;
	case NMEA_UNKNOWN: case NMEA_OGN:
		//printf("UNKNOWN/OGN \n");
		send_net = modesf.net_filter.enable_unknown;
		send_uart = modesf.uart_filter.enable_unknown;
		break;
	case NMEA_INVALID:
		//printf("INVALID\n");
		send_net = modesf.net_filter.enable_invalid;
		send_uart = modesf.uart_filter.enable_invalid;
		break;
	case NMEA_TXT:
		//printf("TEXT\n");
		send_net = true;
		send_uart = true;
		break;
	}

	//printf("_writeFlarmOutput()\n");
	if (send_net) writeFlarmOutput(flarm_io.flarm_net_service, m->nmea);

	//printf("_writeFlarmOutput(flarm_io.flarm_serial_pri_service, m->nmea)\n");
	if (modesf.uart_pri_out && send_uart)
		writeFlarmOutput(flarm_io.flarm_serial_pri_service, m->nmea);

	//printf("_writeFlarmOutput(flarm_io.flarm_serial_sec_service, m->nmea)\n");
	if (modesf.uart_sec_out && send_uart)
		writeFlarmOutput(flarm_io.flarm_serial_sec_service, m->nmea);

	//printf("_broadcastNMEA(): exit \n\n");
}

void broadcastInternalNMEA(char *nmea, nmea_sentence_t type) {
	//printf("_broadcastInternalNMEA()\n");
	struct nmea_msg m;
	m.source = SRC_INTERNAL;
	m.nmea = nmea;
	m.type = type;
	broadcastNMEA(&m);
	//printf("_broadcastInternalNMEA(): exit\n\n");
}

static int handleUARTcallback(struct client *c, char *nmea) {
	restoreDelim(nmea);

	struct nmea_msg m;
	m.source = (c->service == flarm_io.flarm_serial_pri_service) ? UART_PRI : UART_SEC;
	m.nmea = nmea;
	m.type = NMEA_INVALID;

	proceedGenericNMEA(&m);
	broadcastNMEA(&m);

	return 0;
}

//
// ================================ Main ====================================
//
static void showHelp(void) {
	printf(
			"------------------------------------------------------------------------------------------------------\n"
			"| ADSB-to-FLARM: The BEAST to FLARM translator and NMEA mixer                                     v3.0|\n"
			"------------------------------------------------------------------------------------------------------\n"
			"(c) 2020-2023 Denis G Dugushkin aka Flangeneer, telegram: @flangeneer, tel: +7-952-242-42-88\n\n"
			"usage: adsb2flarm <uart> [options]\n"
			"At least one UART port must be specified. Hardware barometric sensors are optional.\n\n"

			"NETWORK AND UART IO:\n"
			"--net-bo-ipaddr <addr>        IP address to connect to for Beast data (default: 127.0.0.1)\n"
			"--net-bo-port <port>          Port to connect for Beast data (default: 30005)\n"
			"--net-flarm-port <port>       Port for FLARM data output (default: 2000; use 0 for disable net out)\n"
			"--uart-pri <addr>             Primary UART address (e.g. /dev/ttyUSB0)\n"
			"--uart-sec <addr>             Secondary UART address (e.g. /dev/ttyS0)\n"
			"--uart-pri-out                Enable general data output on primary UART (default: disabled)\n"
			"--uart-sec-out                Enable general data output on secondary UART (default: disabled)\n"
			"\nNMEA MIXING OPTIONS:\n"
			"--uart-out-flarm              Enable PFLAU, PFLAA sentences on UART(s) output (default: disabled)\n"
			"--net-out-flarm               Same as above, but for net output (default: disabled)\n"
			"--uart-out-gnss               Enable RMC, GGA, GSA, GSV sentences on UART(s) output (default: disabled)\n"
			"--net-out-gnss                Same as above, but for net output (default: disabled)\n"
			"--uart-out-baro               Enable LXWP0, GPRMZ, POGNB sentences on UART(s) output (default: disabled)\n"
			"--net-out-baro                Same as above, but for net output (default: disabled)\n"
			"--uart-out-unknown            Enable unknown sentences on UART(s) output (default: disabled)\n"
			"--net-out-unknown             Same as above, but for net output (default: disabled)\n"
			"--uart-out-invalid            Enable invalid sentences on UART(s) output (default: disabled)\n"
			"--net-out-invalid             Same as above, but for net output (default: disabled)\n"
			"\nSENSORS AND BEEPER:\n"
			"--sensor-baro <addr>          I2C address of BMP280/BME280 sensor. May be repeated. (e.g. /dev/i2c-0)\n"
			"--beeper-gpio-pin <value>     Specify beeper GPIO pin (default: 423 for Radxa Zero SBC)\n"
			"--alt-update-period <value>   Update period for altitude sensors (ms) (default: 100)\n"
			"--alt-sensor-priority <opt>   Altitude sensor order priority (default: LOHR)\n"
			"   options are: H - hardware barosensor, L - LXWP0 sentence, O - POGNB sentence, R - PGRMZ sentence\n"
			"   e.g. 'HLOR' means that use h/w sensor(s) first, if h/w not available\n"
			"   use altitude data from NMEA messages in order of availability: LXWP0, POGNB, PGRMZ\n"
			"\nFLARM:\n"
			"--flarm-dist <value>          Filter aircrafts far than distance value (meters) (default: 25000)\n"
			"--flarm-lev0 <value>          Altitude difference for FLARM0 alarm (meters) (default: 10000)\n"
			"--flarm-lev1 <value>          Altitude difference for FLARM1 alarm (meters) (default: 2500)\n"
			"--flarm-lev2 <value>          Altitude difference for FLARM2 alarm (meters) (default: 1250)\n"
			"--flarm-lev3 <value>          Altitude difference for FLARM3 alarm (meters) (default: 625)\n"
			"--flarm-update-period <value> Update period of FLARM routines (ms) (default: 100)\n"
			"\nTESTING AND MAINTAIN:\n"
			"--targets-test                Simulate the circle of targets around\n"
			"--beeper-test                 Play repeatedly levels 1,2,3\n"
			"--startup-test <value>        Simulate targets at startup for <n> cycles (default 0: disabled)\n"
			"\n"
			"--help                        Show this help\n"
			"\n"
			);
}



//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, refreshing the screen in interactive mode, and so forth
//
static void backgroundTasks(void) {

	icaoFilterExpire();
	trackPeriodicUpdate();

	modesNetPeriodicWork();

	update_altitude_sensors();
	periodicGNSSCheck();

	tcas();

	sync_rtc_from_gnss();

	if (modesf.debug_view) print_gnss();
}

static void do_test_targets_cycle() {
	int a = 18;
	while (a--) {
		generate_aircraft_circle(a);
		int b = 6;
		while (b--) {
			backgroundTasks();
			usleep(25 * 1000);
		}
	}
}

#define randnum(min, max) \
    ((rand() % (int)(((max) + 1) - (min))) + (min))

//
//=========================================================================
//

int main(int argc, char **argv) {
	int j;
	char *bo_connect_ipaddr = "127.0.0.1";
	int bo_connect_port = 30005;
	char *flarm_net_port = "2000";
	char *uart_pri_addr = NULL;
	char *uart_sec_addr = NULL;
	char *alt_sensor_priority = NULL;

	struct client *c;
	struct net_service *beast_input, *nmea_io_pri, *nmea_io_sec;

	// Set sane defaults
	globalInitConfig();

	// Parse the command line options
	for (j = 1; j < argc; j++) {
		int more = j + 1 < argc; // There are more arguments

		if (!strcmp(argv[j], "--net-bo-port") && more) {
			bo_connect_port = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--debug_view")) {
			modesf.debug_view = true;
		} else if (!strcmp(argv[j], "--targets-test")) {
			modesf.debug_targets_test = true;
		} else if (!strcmp(argv[j], "--beeper-test")) {
			modesf.debug_beeper_test = true;
		} else if (!strcmp(argv[j], "--startup-test") && more) {
			modesf.startup_test = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--uart-pri-out")) {
			modesf.uart_pri_out = true;
		} else if (!strcmp(argv[j], "--uart-sec-out")) {
			modesf.uart_sec_out = true;
		} else if (!strcmp(argv[j], "--net-bo-ipaddr") && more) {
			bo_connect_ipaddr = argv[++j];
		} else if (!strcmp(argv[j], "--net-flarm-port") && more) {
			flarm_net_port = argv[++j];
		} else if (!strcmp(argv[j], "--uart-pri") && more) {
			uart_pri_addr = argv[++j];
		} else if (!strcmp(argv[j], "--uart-sec") && more) {
			uart_sec_addr = argv[++j];
		} else if (!strcmp(argv[j], "--alt-update-period") && more) {
			modesf.altitude_update_time = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--flarm-update-period") && more) {
			modesf.flarm_update_time = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--uart-out-flarm")) {
			modesf.uart_filter.enable_flarm = true;
		} else if (!strcmp(argv[j], "--uart-out-gnss")) {
			modesf.uart_filter.enable_gnss = true;
		} else if (!strcmp(argv[j], "--uart-out-baro")) {
			modesf.uart_filter.enable_baro = true;
		} else if (!strcmp(argv[j], "--uart-out-unknown")) {
			modesf.uart_filter.enable_unknown = true;
		} else if (!strcmp(argv[j], "--uart-out-invalid")) {
			modesf.uart_filter.enable_invalid = true;
		} else if (!strcmp(argv[j], "--net-out-flarm")) {
			modesf.net_filter.enable_flarm = true;
		} else if (!strcmp(argv[j], "--net-out-gnss")) {
			modesf.net_filter.enable_gnss = true;
		} else if (!strcmp(argv[j], "--net-out-baro")) {
			modesf.net_filter.enable_baro = true;
		} else if (!strcmp(argv[j], "--net-out-unknown")) {
			modesf.net_filter.enable_unknown = true;
		} else if (!strcmp(argv[j], "--net-out-invalid")) {
			modesf.net_filter.enable_invalid = true;
		} else if (!strcmp(argv[j], "--sensor-baro") && more) {
			initBaroSensorHW(argv[++j], BME280_I2C_ADDR_PRIM, &sensbox.alt_data[ALTITUDE_DATA_HW_PRI + sensbox.hw_baro_count]);
		} else if (!strcmp(argv[j], "--alt-sensor-priority") && more) {
			alt_sensor_priority = argv[++j];
		} else if (!strcmp(argv[j], "--flarm-dist") && more) {
			flarm_param.distance = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--flarm-lev0") && more) {
			flarm_param.level_0 = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--flarm-lev1") && more) {
			flarm_param.level_1 = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--flarm-lev2") && more) {
			flarm_param.level_2 = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--flarm-lev3") && more) {
			flarm_param.level_3 = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--beeper-gpio-pin") && more) {
			modesf.beeper.gpio_pin = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--help")) {
			showHelp();
			exit(0);
		} else {
			fprintf(stderr,
					"Unknown or not enough arguments for option '%s'.\n\n",
					argv[j]);
			showHelp();
			exit(1);
		}
	}

	bool b = modesf.net_filter.enable_flarm || modesf.net_filter.enable_gnss || modesf.net_filter.enable_baro || modesf.net_filter.enable_unknown || modesf.net_filter.enable_invalid ||
			modesf.uart_filter.enable_flarm || modesf.uart_filter.enable_gnss || modesf.uart_filter.enable_baro || modesf.uart_filter.enable_unknown || modesf.uart_filter.enable_invalid;

	if (!b) {
		fprintf(stderr, "[ERR] Filter settings not specified properly. Enable at least one NMEA mixing option\n");
		exit(1);
	}

	fprintf(stderr, "[INFO] FLARM settings: DISTANCE=%0.1fm, FL0=%0.1fm, FL1=%0.1fm, FL2=%0.1fm, FL3=%0.1fm\n", flarm_param.distance, flarm_param.level_0, flarm_param.level_1, flarm_param.level_2, flarm_param.level_3);

	// Initialization
	globalInit();
	modesInitNet();

	initBaroSensorPriority(alt_sensor_priority);

	if (!sensbox.hw_baro_count) {
		fprintf(stderr, "[WARN] No hardware barometric sensors specified. Will try to use NMEA or GNSS (unsafe!)\n");
	}

	// Set up input connection
	int i = 0;
	c = NULL;
	while ((!c) && (i < 5)) {
	beast_input = makeBeastInputService();

	c = serviceConnect(beast_input, bo_connect_ipaddr, bo_connect_port);
		if (!c) {
			i++;
			printf("[WARN] Attempt %d of 5. Failed to connect to %s:%d, %s. Wait for 5s to retry...\n", i, bo_connect_ipaddr, bo_connect_port, Modes.aneterr);
			usleep(5000 * 1000);
		}
	}

	if (!c) {
		fprintf(stderr,
				"[ERR] Failed to connect to %s:%d (is dump1090 running?): %s\n",
				bo_connect_ipaddr, bo_connect_port, Modes.aneterr);
		exit(1);
	}

	sendBeastSettings(c, "CdfjV"); // Beast binary, no filters, CRC checks on, no mode A/C, verbatim mode on



	// Set up UARTs
	if (!uart_pri_addr && !uart_sec_addr) {
		fprintf(stderr,
				"[ERR] No UART ports are specified\n");
		exit(1);
	}

	// Set up FLAEM output
	flarm_io.net_output_flarm_ports = flarm_net_port;
	makeFlarmNetOutputService();

	initGNSS();

	int uart_count = 0;

	if (uart_pri_addr) {
	nmea_io_pri = makeNmeaSerialPriInputOutputService(handleUARTcallback);
	c = createSerialClient(nmea_io_pri, uart_pri_addr);
		if (c) {
			flarm_io.flarm_serial_pri_service = nmea_io_pri;
			uart_count++;
		}
	}

	if (uart_sec_addr) {
	nmea_io_sec = makeNmeaSerialSecInputOutputService(handleUARTcallback);
	c = createSerialClient(nmea_io_sec, uart_sec_addr);
		if (c) {
			flarm_io.flarm_serial_sec_service = nmea_io_sec;
			uart_count++;
		}
	}

	if (!uart_count) {
		fprintf(stderr, "[ERR] No working UARTs were detected, abnormal exit (check switches and hardware)\n");
		exit(1);
	}


	// Set up beeper
	beeper_init(&modesf.beeper);
	usleep(1000 * 1000);
	beeper_play_flarm(&modesf.beeper, 4);
	usleep(1000 * 1000);
	beeper_play_flarm(&modesf.beeper, 0);


	if (modesf.debug_beeper_test) {
		fprintf(stderr, "Entering debug mode, testing beeper until break (Ctrl+C)...\n");
		while (!Modes.exit) {
			beeper_play_flarm(&modesf.beeper, 0);
			usleep(3000 * 1000);
			beeper_play_flarm(&modesf.beeper, 1);
			usleep(3000 * 1000);
			beeper_play_flarm(&modesf.beeper, 2);
			usleep(3000 * 1000);
			beeper_play_flarm(&modesf.beeper, 3);
			usleep(3000 * 1000);
		}
	}

	if (modesf.debug_targets_test) {
		fprintf(stderr, "Entering debug mode, generating circle of targets until break (Ctrl+C)...\n");
		while (!Modes.exit) {
			do_test_targets_cycle();
			}
	}

	if (modesf.startup_test) {
		while(modesf.startup_test--) {
			do_test_targets_cycle();
		}
	}



	fprintf(stderr, "System config enough to work, starting background tasks\n");

	while (!Modes.exit && beast_input->connections) {
			backgroundTasks();
			usleep(500);
	}

	 beeper_close(&modesf.beeper);
	 closeAllBaroSensorsHW();

	exit(1);

	fprintf(stderr, "[INFO] Normal exit, return 0\n");
	return 0;
}
//
//=========================================================================
//
