#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define UNUSED(x) (void)(x)

#define ALTITUDE_UPDATE_TIME 100
#define FLARM_UPDATE_TIME 100
#define FLARM_DISTANCE 25000.0
#define FLARM_ALT_LEVEL0 10000
#define FLARM_ALT_LEVEL1 2500
#define FLARM_ALT_LEVEL2 1250
#define FLARM_ALT_LEVEL3 625

struct _flarm_param {
	float distance;
	float level_0;
	float level_1;
	float level_2;
	float level_3;
};

struct _flarm_param flarm_param;

struct nmea_filter {
	bool enable_flarm;
	bool enable_gnss;
	bool enable_baro;
	bool enable_unknown;
	bool enable_invalid;
};

struct _modesf {
	// Devices
	struct beeper_gpio beeper;

	// Networking and UART
	struct net_service *flarm_net_service;
	struct net_writer flarm_out;
	char *net_output_flarm_port;

	struct nmea_filter net_filter;
	struct nmea_filter uart_filter;

	bool uart_pri_out;
	bool uart_sec_out;

	uint64_t flarm_update_time;
	uint64_t altitude_update_time;


	bool debug_view;
	bool debug_targets_test;
	bool debug_beeper_test;
	int startup_test;
};

struct _modesf modesf;

struct speeds {
	float V1;
	float V2;
};
