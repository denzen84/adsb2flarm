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

#include <stdarg.h>

#include "sensors.h"
#include "beeper.h"
#include "adsb2flarm.h"
#include "nmea_utils.h"
#include "uart_io.h"

#include <sys/time.h>

const char* pflau_not_ready = "$PFLAU,0,0,0,1,0,,,,,*7F\r\n";


void tcas(void);
void sync_rtc_from_gnss();

void broadcastNMEA(const char *nmea);

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

static inline void update_baro_alt(void) {
	char nmea[NMEA_MAX_MESSAGE];
	static uint64_t baro_next_update;

	if (!adsb2flarm.baro_master.active)
		return;

	uint64_t now = mstime();
	if (now < baro_next_update) {
		return;
	}

	// scan once a 1/100 second at most
	baro_next_update = now + 100;

	sensor_bme280_read_altitude_feet(&adsb2flarm.baro_master,
			&adsb2flarm.gnss_master);
	//sensor_bme280_read_altitude_feet(&adsb2flarm.baro_master, &adsb2flarm.gnss_slave);

	if (adsb2flarm.gnss_master.alt_baro_valid) {
		generate_nmea_pgrmz(nmea, NMEA_MAX_MESSAGE,
				adsb2flarm.gnss_master.altitude_baro_feets);
		broadcastNMEA(nmea);
	}
}
/*
#define randnum(min, max) \
    ((rand() % (int)(((max) + 1) - (min))) + (min))

void generate_aircraft_circle(void) {

	char nmea[NMEA_MAX_MESSAGE];
	struct pflaa_data pflaa;

	for (int a = 2; a >= 2; a--) {
		for (int i = 0; i < 12; i++) {
			pflaa.ID = (i + 1) * 1024;
			pflaa.IDType = 1;
			pflaa.AlarmLevel = a;
			pflaa.RelativeNorth = a * 1000 + 3000 * cos(2 * M_PI * i / 12);
			pflaa.RelativeEast = a * 1000 + 3000 * sin(2 * M_PI * i / 12);
			pflaa.RelativeVertical = (a + 1) * 100;
			pflaa.Track = randnum(0, 359);
			pflaa.ClimbRate = (float) randnum(0, 32);
			pflaa.GroundSpeed = 100 + randnum(0, 50);
			pflaa.TurnRate = 0;
			pflaa.AcftType = 9;
			pflaa.Source = 1;
			pflaa.NoTrack = 0;
			pflaa.RSSI = 11.1;
			generate_nmea_pflaa(nmea, NMEA_MAX_MESSAGE, &pflaa);
			broadcastNMEA(nmea);
		}
	}
	struct pflau_data pflau;
	pflau.RX = 12 * 4;
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
	broadcastNMEA(nmea);
}
*/

static inline void bme280_send_nmea(const double p, const double t, const double h) {

	char nmea[NMEA_MAX_MESSAGE];

	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX HUMIDITY: %0.1f %%", h);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastNMEA(nmea);

	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX PRESSURE: %0.2f hPa", p / 100);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastNMEA(nmea);

	snprintf(nmea, NMEA_MAX_MESSAGE, "INBOX TEMPERATURE: %0.1f C", t);
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastNMEA(nmea);
}

static inline void update_bme280(void) {
	static uint64_t bme280_next_update;

	if (!adsb2flarm.baro_master.active)
		return;

	uint64_t now = mstime();
	if (now < bme280_next_update) {
		return;
	}

	// send every 5s
	bme280_next_update = now + 5000;

	sensor_bme280_read_all(&adsb2flarm.baro_master);
	bme280_send_nmea(adsb2flarm.baro_master.comp_data.pressure,
			adsb2flarm.baro_master.comp_data.temperature,
			adsb2flarm.baro_master.comp_data.humidity);
	//generate_aircraft_circle();
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

/*
 #define sine_sign(angle) (((angle >= 0.0 && angle <= 90.0) || (angle >= 270.0 && angle < 360.0)) ? 1 : -1)
 #define cosine_sign(angle) ((angle >= 0.0 && angle <= 180.0) ? 1 : -1)
 #define sine_limit (bearing) (bearing >= 270.0 && bearing <= 300.0) || (bearing >= 90.0 && bearing <= 120.0) ? true : false
 #define compare_direction(bearing1, bearing2) (sine_sign(bearing1) > 0 ? ((sine_sign(bearing1) + sine_sign(bearing2) == 0) || sine_limit(bearing2)) && (cosine_sign(bearing1) + cosine_sign(bearing2) == 0) : (sine_sign(bearing1) + sine_sign(bearing2) == 0) && (cosine_sign(bearing1) + cosine_sign(bearing2) == 0))
 */



static inline bool flarm_compare_speed_and_bearing(float v1, float bearing1, float v2, float bearing2, float rel_bearing) {
	float Vtx = (v1 * sin(bearing1 * M_PI/180) - v2 * sin(bearing2 * M_PI/180)) * sin(rel_bearing * M_PI/180); // knots
	float Vty = (v1 * cos(bearing1 * M_PI/180) - v2 * cos(bearing2 * M_PI/180)) * cos(rel_bearing * M_PI/180); // knots
	return (Vtx + Vty) > 0.0;
}

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

static inline struct gnss_data* get_active_gnss(void) {
	return &adsb2flarm.gnss_master;
}

static inline int determine_flarm_level(struct aircraft *a, struct gnss_data *gnss_active) {

	int flarm_level = -1;

	// Proceed only reliable aircrafts
	if (!a->reliable) return flarm_level;

	// Skip aircrafts without position
	if (!trackDataValid(&a->position_valid)) return flarm_level;

	// Skip aircrafts on the ground
	if (trackDataValid(&a->airground_valid)
			&& a->airground_valid.source >= SOURCE_MODE_S_CHECKED
			&& a->airground == AG_GROUND)
		return flarm_level;


	float self_lat = gnss_active->lat; // 59.7992; //59.931770;
	float self_lon = gnss_active->lon; // 30.7736;//31.797262;
	float self_track = gnss_active->course;
	float self_alt_baro = gnss_active->altitude_baro_feets;
	float self_speed = gnss_active->speed; // knots(!)

	// Absolute distance to aircraft
	float distance = greatcircle(self_lat, self_lon, a->lat, a->lon);
	// Skip so far aircrafts
	if (distance > FLARM_DISTANCE) return flarm_level;

	float abs_alt_diff = 0.3048 * fabs(a->altitude_baro - self_alt_baro); // Absolute difference

    if (abs_alt_diff <= FLARM_ALT_LEVEL3) {
    	flarm_level = 3;  // FLARM level 3
    } else
    if (abs_alt_diff <= FLARM_ALT_LEVEL2) {
    	flarm_level = 2;  // FLARM level 2
    } else
    if (abs_alt_diff <= FLARM_ALT_LEVEL1) {
    	flarm_level = 1;  // FLARM level 1
    } else
    if (abs_alt_diff <= FLARM_ALT_LEVEL0) {
    	flarm_level = 0;  // FLARM level 0 (display, but no audio signal)
    } else return flarm_level;

    float rel_bearing = get_bearing(self_lat, self_lon, a->lat, a->lon);
    flarm_level = flarm_compare_speed_and_bearing(self_speed, self_track, a->gs, a->track, rel_bearing) ? flarm_level : 0;

  return flarm_level;
}

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

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
	tcas_next_update = now + 200;

	struct gnss_data *gnss_active = get_active_gnss();

	if (!gnss_active->gnss_valid) {
		broadcastNMEA(pflau_not_ready);
		return;
	}

	float self_lat = gnss_active -> lat; // 59.7992; //59.931770;
	float self_lon = gnss_active -> lon; // 30.7736;//31.797262;
	float self_alt_baro = gnss_active -> altitude_baro_feets;

	//system("clear");

	for (a = Modes.aircrafts; a; a = a->next) {
		// Skip aircrafts if there is no data for 2000 ms
		if ((now - a->seen) > 5000) continue;

		if (trackDataValid(&a->track_valid)) {
			int32_t north, east;

			int flarm_level = determine_flarm_level(a, gnss_active);
			//printf("%d \n", flarm_level);
			if (flarm_level < 0) continue;

			total++;
			max_flarm_level = MAX(max_flarm_level, flarm_level);

			relative_north_east_position(self_lat, self_lon, a->lat, a->lon,
					&north, &east);

			struct pflaa_data pflaa;
			pflaa.ID = a->addr & 0xFFFFFF; // HEX
			pflaa.IDType = 1; // official ICAO 24-bit aircraft address
			pflaa.AlarmLevel = flarm_level;
			pflaa.RelativeNorth = north;
			pflaa.RelativeEast = east;
			pflaa.RelativeVertical = 0.3048 * (a->altitude_baro - self_alt_baro); // feets to meters
			pflaa.Track = (int) a->track;
			pflaa.ClimbRate = a->baro_rate / 197; // convert fpm to m/s
			pflaa.GroundSpeed = (int) (a->gs * 0.51);
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
			broadcastNMEA(nmea);

		}
	}
	struct pflau_data pflau;
	pflau.RX = total;
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
	broadcastNMEA(nmea);

	beeper_play_flarm(&adsb2flarm.beeper, max_flarm_level);
}

void sync_rtc_from_gnss() {

	static uint64_t gnss_time_next_update;

	uint64_t now = mstime();
	if (now < gnss_time_next_update)
		return;

	// update once a 10 minutes
	gnss_time_next_update = now + 1000 * 600;

	struct gnss_data *gnss_active = get_active_gnss();

	if (gnss_active->gnss_time.tv_sec > 1519842712) { // It's just a magic value

    int rc = clock_settime(CLOCK_REALTIME, &gnss_active->gnss_time);
	char nmea[NMEA_MAX_MESSAGE];


    if (rc == 0) {
    	snprintf(nmea, NMEA_MAX_MESSAGE, "SYSTEM TIME WAS SUCCESSFULLY SYNCHRONIZED WITH GNSS TIME");
    }
    else {
    	snprintf(nmea, NMEA_MAX_MESSAGE, "FAILED TO SYNC SYSTEM TIME WITH GNSS TIME: ERROR CODE: %d, %s", errno, strerror(errno));
    }
	generate_nmea_gptxt(nmea, NMEA_MAX_MESSAGE, nmea);
	broadcastNMEA(nmea);
	}
}

//
// =============================== Initialization ===========================
//
static void faupInitConfig(void) {
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
	Modes.net_output_flush_size = 1024;
	Modes.net_output_flush_interval = 50; // milliseconds
	Modes.faup_rate_multiplier = FAUP_DEFAULT_RATE_MULTIPLIER;
	//Modes.net_output_stratux_ports = "4353";
	//Modes.net_bind_address = "0.0.0.0";
}

//
//=========================================================================
//
static void faupInit(void) {
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

void broadcastNMEA(const char *nmea) {
	writeFlarmOutput(flarm_io.flarm_net_service, nmea);
	writeFlarmOutput(flarm_io.flarm_serial_service, nmea);
}

static int handleMasterNMEA(struct client *c, char *nmea) {
	UNUSED(c);
	restoreDelim(nmea);
	readGnssNmea(nmea, &adsb2flarm.gnss_master);
	broadcastNMEA(nmea);
	return 0;
}

static int handleSlaveNMEA(struct client *c, char *nmea) {
	UNUSED(c);
	restoreDelim(nmea);
	readGnssNmea(nmea, &adsb2flarm.gnss_slave);
	//if (some condition) broadcastNMEA(nmea);
	return 0;
}

//
// ================================ Main ====================================
//
static void showHelp(void) {
	printf(
			"-----------------------------------------------------------------------------\n"
					"| ADSB-to-FLARM      %45s |\n"
					"-----------------------------------------------------------------------------\n"
					"--net-bo-ipaddr <addr>   IP address to connect to for Beast data (default: 127.0.0.1)\n"
					"--net-bo-port <port>     Port to connect for Beast data (default: 30005)\n"
					"--sensor-baro <addr>     I2C address of BMP280/BME280 barometric sensor\n"
					"--update-rate <value>    Refresh rate of FLARM and baro data\n"

					"--help                   Show this help\n"
					"\n",
			MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION);
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

	update_baro_alt();
	update_bme280();
	tcas();

	sync_rtc_from_gnss();
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
	struct client *c;
	struct net_service *beast_input, *nmea_input_master, *nmea_input_slave; //, *fatsv_output, *fa_cmd_input;

	// Set sane defaults
	faupInitConfig();

	// Parse the command line options
	for (j = 1; j < argc; j++) {
		int more = j + 1 < argc; // There are more arguments

		if (!strcmp(argv[j], "--net-bo-port") && more) {
			bo_connect_port = atoi(argv[++j]);
		} else if (!strcmp(argv[j], "--net-bo-ipaddr") && more) {
			bo_connect_ipaddr = argv[++j];
		} else if (!strcmp(argv[j], "--lat") && more) {
			Modes.fUserLat = atof(argv[++j]);
		} else if (!strcmp(argv[j], "--lon") && more) {
			Modes.fUserLon = atof(argv[++j]);
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

	// Initialization

	faupInit();
	modesInitNet();

	// Set up sensors
	adsb2flarm.baro_master.addr = "/dev/i2c-1";
	adsb2flarm.baro_master.active = (sensor_bme280_init(&adsb2flarm.baro_master)
			== BME280_OK);

	// Set up beeper
	beeper_init(&adsb2flarm.beeper);
	beeper_play_flarm(&adsb2flarm.beeper, 2);
	usleep(1000 * 1000);
	beeper_play_flarm(&adsb2flarm.beeper, 0);
	/*
	while(true) {
		int randv = randnum(0,3);
		printf("New value: %d\n", randv);
		beeper_play_flarm(&adsb2flarm.beeper, randv);
		usleep(2000 * 1000);
	}
	*/


	// Set up input connection
	beast_input = makeBeastInputService();
	c = serviceConnect(beast_input, bo_connect_ipaddr, bo_connect_port);
	if (!c) {
		fprintf(stderr,
				"adsb2flarm: failed to connect to %s:%d (is dump1090 running?): %s\n",
				bo_connect_ipaddr, bo_connect_port, Modes.aneterr);
		exit(1);
	}

	sendBeastSettings(c, "CdfjV"); // Beast binary, no filters, CRC checks on, no mode A/C, verbatim mode on

	flarm_io.net_output_flarm_ports = "2000";
	makeFlarmNetOutputService();

	nmea_input_master = makeNmeaSerialInputService(handleMasterNMEA);
	createSerialClient(nmea_input_master, "/dev/ttyS2");

	nmea_input_slave = makeNmeaSerialInputOutputService(handleSlaveNMEA);
	createSerialClient(nmea_input_slave, "/dev/ttyS1");
	flarm_io.flarm_serial_service = nmea_input_slave;

	usleep(2000 * 1000);
	// Run it until we've lost either connection
	while (!Modes.exit && beast_input->connections) {
		backgroundTasks();
		usleep(5 * 1000);
	}

	return 0;
}
//
//=========================================================================
//
