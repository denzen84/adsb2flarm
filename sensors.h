#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#define BME280_FLOAT_ENABLE
#include "bme280.h"
#include "minmea.h"

#define UNUSED(x) (void)(x)

#define COEF_METERS_TO_FEETS 3.28084
#define PRESSURE_BASE_HPA 1013.25 // hPa
#define BAD_ALT 0xBADA17

struct gnss_data {
	struct timespec gnss_time;
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
	float altitude_baro_feets;
	bool alt_baro_valid;
	int fix_quality;
	int fix_type;
	float pdop;
	float hdop;
	float vdop;
	int satellites_tracked;
	bool gnss_valid;
	bool active;
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

int sensor_bme280_init(struct sensor_bme280 *bme280);
int sensor_bme280_read_all(struct sensor_bme280 *bme280);
int sensor_bme280_read_altitude_feet(struct sensor_bme280 *bme280, struct gnss_data *data);
void sensor_bme280_close(struct sensor_bme280 *bme280);

void readGnssNmea(char *s, struct gnss_data *data);


