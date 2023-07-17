#ifndef __SENSORS_H
#define __SENSORS_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#define MINMEA_MAX_SENTENCE_LENGTH 250
#define BME280_FLOAT_ENABLE
#include "bme280.h"
#include "minmea.h"
#include "minmea_ext.h"

#define UNUSED(x) (void)(x)

#define NSEC_MULT 1000000
#define COEF_METERS_TO_FEETS 3.28084
#define COEF_FEETS_TO_METERS 0.3048
#define PRESSURE_BASE_HPA 1013.25 // hPa
#define BAD_ALT 0xBADA17

#define DEFAULT_ALTITUDE_SENSOR_PRIORITY "LOHR"
#define SENSOR_TIMEOUT 2500 // [ms] timeout for sensor data validity

#define HW_BARO_SENSORS_COUNT 3 // Hardware baro sensors

#define ALTITUDE_SENSORS_COUNT 11  // Total altitude sensors
#define ALTITUDE_DATA_HW_PRI 0
#define ALTITUDE_DATA_HW_SEC 1
#define ALTITUDE_DATA_HW_EXT 2
#define ALTITUDE_DATA_SW_LXNAV_UART_PRI 3
#define ALTITUDE_DATA_SW_LXNAV_UART_SEC 4
#define ALTITUDE_DATA_SW_OGN_UART_PRI 5
#define ALTITUDE_DATA_SW_OGN_UART_SEC 6
#define ALTITUDE_DATA_SW_GNSS_UART_PRI 7
#define ALTITUDE_DATA_SW_GNSS_UART_SEC 8
#define ALTITUDE_DATA_SW_PGRMZ_UART_PRI 9
#define ALTITUDE_DATA_SW_PGRMZ_UART_SEC 10

typedef enum {
    SENSOR_BARO_BMX280,
	SENSOR_BARO_NMEA_RMZ,
	SENSOR_BARO_NMEA_LXNAV,
	SENSOR_BARO_NMEA_OGN,
	SENSOR_GEOM_NMEA_GNSS,
	SENSOR_OTHER
} sensor_alt_t;

typedef enum {
    HW_BME280,
	HW_BMP280
} bmx_hw_t;

typedef enum {
    UART_PRI,
	UART_SEC,
	SRC_INTERNAL
} gnss_source_t;

typedef enum {
    NMEA_GNSS,
	NMEA_FLARM_PFLAU,
	NMEA_FLARM_PFLAA,
	NMEA_OGN,
	NMEA_ALT,
	NMEA_TXT,
	NMEA_UNKNOWN,
	NMEA_INVALID
} nmea_sentence_t;

struct gnss_data {
	gnss_source_t source;
	struct timespec gnss_time;
	unsigned int gnss_rate;
	float lat;
	float lon;
	float speed;
	float course;
	float true_track_degrees;
	float magnetic_track_degrees;
	float speed_kph;
	float variation;
	float altitude_gnss;
	char altitude_gnss_units;
	float altitude_gnss_feets;
	bool alt_gnss_valid;
	unsigned int fix_quality;
	unsigned int fix_type;
	float pdop;
	float hdop;
	float vdop;
	int satellites_tracked;
	uint64_t score;
	bool gnss_valid;
	bool active;
	unsigned int rate;
	uint64_t last_update;
	char *name;
};

struct altitude_data {
	float altitude_feet;
	float altitude_m;
	sensor_alt_t hw;
	gnss_source_t source;
	bool valid;
	uint64_t last_update;
};

struct _pflau_data {
	int ac_count;
	bool valid;
	uint64_t last_update;
};

/* BME 280 */
struct identifier {
	/* Variable to hold device address */
	uint8_t dev_addr;
	/* Variable that contains file descriptor */
	int8_t fd;
};

struct sensor_bme280 {
	struct bme280_dev dev;
	struct identifier id;
	struct bme280_data comp_data;
	char *addr;
	bool active;
};

struct sensor_bmx280 {
	struct bme280_dev dev;
	struct bme280_data comp_data;
	int id;
	char *i2c_addr;
	uint8_t dev_addr;
	int8_t fd;
	bmx_hw_t hw;
	struct altitude_data *data;
	bool active;
};

struct nmea_msg {
	gnss_source_t source;
	nmea_sentence_t type;
	char *nmea;
};

struct _Sensors {
	// Sensors data
	struct altitude_data alt_data[ALTITUDE_SENSORS_COUNT];
	int alt_priority[ALTITUDE_SENSORS_COUNT];
	int alt_sensor_oount;

	// Hardware barometric sensors
	struct sensor_bmx280 hw_baro[HW_BARO_SENSORS_COUNT];
	int hw_baro_count;

	// GNSS
	struct gnss_data gnss_pri; // Primary UART
	struct gnss_data gnss_sec; // Secondary UART

	// Active data
	struct gnss_data *active_gnss;
	struct altitude_data *active_altitude;

	//PFLAU
	struct _pflau_data pflau_pri;
	struct _pflau_data pflau_sec;
};
extern struct _Sensors sensbox;


void initBaroSensorPriority(char *s);
void periodicBaroSensorCheck();

void initBaroSensorHW(char *i2c_addr, uint8_t dev_addr, struct altitude_data *data);
void updateAllBaroSensorsHW();
void closeAllBaroSensorsHW();

void initGNSS();
void periodicGNSSCheck();
void proceedExtededNMEA(struct nmea_msg *msg);
void proceedGenericNMEA(struct nmea_msg *msg);

#endif
