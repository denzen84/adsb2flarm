#include <stdbool.h>
#include <stdlib.h>

#define UNUSED(x) (void)(x)

struct _adsb2flarm {
	char* sensor_altbaro_addr;     // Default: /dev/i2c-1
	char* sensor_gnss_master_addr; // Default: /dev/ttyS2
	char* sensor_gnss_slave_addr;  // Default: /dev/ttyS1

	// Networking
	struct net_service *flarm_net_service;
	struct net_writer flarm_out;
	char *net_output_flarm_ports;

	struct gnss_data gnss_master;
	struct gnss_data gnss_slave;

	struct sensor_bme280 baro_master;
	struct beeper_gpio beeper;

};

struct _adsb2flarm adsb2flarm;

#define FLARM_DISTANCE 25000.0
#define FLARM_ALT_LEVEL0 10000
#define FLARM_ALT_LEVEL1 2500
#define FLARM_ALT_LEVEL2 1250
#define FLARM_ALT_LEVEL3 625

