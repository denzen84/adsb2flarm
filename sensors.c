#include <stdio.h>
#include "sensors.h"
#include "util.h"

struct altitude_data *altitudeSensorIndexBySource(
		gnss_source_t src_uart, int sentence);

int sensor_bmx280_init(struct sensor_bmx280 *bmx280);
int sensor_bmx280_read_all(struct sensor_bmx280 *bmx280);
void sensor_bmx280_read_altitude(struct sensor_bmx280 *bmx280);
void sensor_bmx280_close(struct sensor_bmx280 *bmx280);

int8_t bmx280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
		void *intf_ptr);
int8_t bmx280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
		void *intf_ptr);
void bmx280_delay_us(uint32_t period, void *intf_ptr);
void bmx280_print_calib_data(struct sensor_bmx280 *bmx280);


int8_t bmx280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
		void *intf_ptr) {
	struct sensor_bmx280 bmx280 = *((struct sensor_bmx280*) intf_ptr);
	struct i2c_msg read_user_reg[2] = { { bmx280.dev_addr, 0, 1, &reg_addr }, {
			bmx280.dev_addr, I2C_M_RD, (uint16_t) len, data } };
	struct i2c_rdwr_ioctl_data messagebuffer = { read_user_reg, 2 };
	int rc = ioctl(bmx280.fd, I2C_RDWR, &messagebuffer);
	if (rc < 0)
		return BME280_E_COMM_FAIL; // Read error
	// Hack to use signle driver both for BME280 and BMP280
	if (reg_addr == BME280_CHIP_ID_ADDR) {
		fprintf(stderr, "Detected sensor: %s\n",
				*data == 0x60 ? "BME280" : "BMP280");
		bmx280.hw = *data == 0x60 ? HW_BME280 : HW_BMP280;
		*data = 0x60;
		data++;
		*data = 0;
	}
	return BME280_OK;
}

void bmx280_delay_us(uint32_t period, void *intf_ptr) {
	UNUSED(intf_ptr);
	usleep(period);
}

int8_t bmx280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
		void *intf_ptr) {
	struct sensor_bmx280 bmx280 = *((struct sensor_bmx280*) intf_ptr);
	uint8_t *buf;
	uint16_t buf_len = len + 1;
	buf = (uint8_t*) malloc(buf_len);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);
	struct i2c_msg write_reg = { bmx280.dev_addr, 0, buf_len, buf };
	struct i2c_rdwr_ioctl_data messagebuffer;
	messagebuffer.nmsgs = 1;
	messagebuffer.msgs = &write_reg;
	int rc = ioctl(bmx280.fd, I2C_RDWR, &messagebuffer);
	free(buf);
	if (rc < 0)
		return BME280_E_COMM_FAIL; // Write error
	return BME280_OK;
}

void bmx280_print_calib_data(struct sensor_bmx280 *bmx280) {
	fprintf(stderr, "Calibration data:\n");
	char *p = (char*) &bmx280->dev.calib_data;
	for (long unsigned int i = 0; i < sizeof(struct bme280_calib_data); i++) {
		fprintf(stderr, "%02X ", *p);
		p++;
	}
	fprintf(stderr, "\n");
}

int sensor_bmx280_init(struct sensor_bmx280 *bmx280) {
	int8_t rslt = BME280_OK;
	if ((bmx280->fd = open(bmx280->i2c_addr, O_RDWR)) < 0)
		return BME280_E_COMM_FAIL;
	if (ioctl(bmx280->fd, I2C_SLAVE_FORCE, bmx280->dev_addr) < 0)
		return rslt;
	bmx280->dev.intf = BME280_I2C_INTF;
	bmx280->dev.read = bmx280_i2c_read;
	bmx280->dev.write = bmx280_i2c_write;
	bmx280->dev.delay_us = bmx280_delay_us;

	/* Update interface pointer with the structure that contains both device address and file descriptor */
	bmx280->dev.intf_ptr = bmx280;

	/* Initialize the bme280 */
	rslt = bme280_init(&bmx280->dev);
	if (rslt != BME280_OK)
		return rslt;
	uint8_t settings_sel = 0;

	/* Recommended mode of operation: Indoor navigation */
	bmx280->dev.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
	bmx280->dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	bmx280->dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	bmx280->dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	bmx280->dev.settings.filter = BME280_FILTER_COEFF_16;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL
			| BME280_OSR_HUM_SEL | BME280_FILTER_SEL | BME280_STANDBY_SEL;

	/* Set the sensor settings */
	rslt = bme280_set_sensor_settings(settings_sel, &bmx280->dev);
	if (rslt != BME280_OK)
		return rslt;

	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bmx280->dev);
	if (rslt != BME280_OK)
		return rslt;
	bmx280_print_calib_data(bmx280);
	// First initial read
	rslt = bme280_get_sensor_data(BME280_ALL, &bmx280->comp_data, &bmx280->dev);
	if (rslt != BME280_OK)
		return rslt;
	return BME280_OK;
}

int sensor_bmx280_read_all(struct sensor_bmx280 *bmx280) {
	return bme280_get_sensor_data(BME280_ALL, &bmx280->comp_data, &bmx280->dev);
}

void sensor_bmx280_read_altitude(struct sensor_bmx280 *bmx280) {
	if (!bmx280->data)
		return;
	if (bmx280->active) {
		if (bme280_get_sensor_data(BME280_ALL, &bmx280->comp_data,
				&bmx280->dev) == BME280_OK) {
			bmx280->data->altitude_feet = 145366.45
					* (1
							- (pow(
									(0.01 * bmx280->comp_data.pressure)
											/ PRESSURE_BASE_HPA, 0.190294)));
			bmx280->data->altitude_m = bmx280->data->altitude_feet
					* COEF_FEETS_TO_METERS;
			bmx280->data->hw = SENSOR_BARO_BMX280;
			bmx280->data->valid = true;
			bmx280->data->last_update = mstime();
		} else {
			bmx280->data->valid = false;
			bmx280->active = false; // Disable errornous sensor
			fprintf(stderr, "Hardware barometric sensor at 0x%X@%s failed to read. Sensor was disabled\n", bmx280->dev_addr, bmx280->i2c_addr);
		}
	}
}

void sensor_bmx280_close(struct sensor_bmx280 *bmx280) {
	free(bmx280->i2c_addr);
	close(bmx280->fd);
}

void initBaroSensorHW(char *i2c_addr, uint8_t dev_addr,
		struct altitude_data *data) {
	fprintf(stderr, "Init barometric I2C sensor at 0x%X@%s...\n", dev_addr,
			i2c_addr);
	if (sensbox.hw_baro_count < HW_BARO_SENSORS_COUNT) {
		struct sensor_bmx280 *bmx280 = &sensbox.hw_baro[sensbox.hw_baro_count];
		bmx280->i2c_addr = strdup(i2c_addr);
		bmx280->dev_addr = dev_addr;
		bmx280->data = data;
		bmx280->data->source = SRC_INTERNAL;
		bmx280->active = (bool) (sensor_bmx280_init(bmx280) == BME280_OK);
		if (bmx280->active) {
			sensbox.hw_baro_count++;
			fprintf(stderr, "Init OK, total sensors count: %d\n",
					sensbox.hw_baro_count);
		} else {
			fprintf(stderr, "Init FAILED\n");
			sensor_bmx280_close(bmx280);
		}
	} else
		fprintf(stderr, "More than %d hardware barometric sensors. Ignored\n",
		HW_BARO_SENSORS_COUNT);
	fprintf(stderr, "\n");
}

void updateAllBaroSensorsHW() {
	for (int i = 0; i < sensbox.hw_baro_count; i++) {
		if (sensbox.hw_baro[i].active)
			sensor_bmx280_read_altitude(&sensbox.hw_baro[i]);
	}
}

void closeAllBaroSensorsHW() {
	for (int i = 0; i < sensbox.hw_baro_count; i++)
		sensor_bmx280_close(&sensbox.hw_baro[i]);
}


void periodicBaroSensorCheck() {

	sensbox.active_altitude = NULL;

	for(int i=0; i<sensbox.alt_sensor_oount; i++) {
		uint64_t t = mstime() - sensbox.alt_data[i].last_update;
		sensbox.alt_data[i].valid = (bool) (t < SENSOR_TIMEOUT);
	}

	for(int i=0; i<sensbox.alt_sensor_oount; i++) {
		struct altitude_data *alt_data = &sensbox.alt_data[sensbox.alt_priority[i]];
		if (alt_data->valid) {sensbox.active_altitude = alt_data; break;}
	}
}

void initBaroSensorPriority(char *s) {

	if (!s) {
		fprintf(stderr, "[WARN] No barometric sensors priority specified. Using default option: 'LOHR' (1. LXWP0 2. POGNB 3. Built-in h/w sensor 4. PGRMZ 5. GNGGA)\n");
		initBaroSensorPriority(DEFAULT_ALTITUDE_SENSOR_PRIORITY);
		return;
	}

	char *p = s;
	char *endp = p + strlen(s);
	int i = 0;

	while (p < endp) {
		switch(*p) {
		case 'H': case 'h':
			for(int s=0; s<sensbox.hw_baro_count;s++)
			{
			sensbox.alt_priority[i] = ALTITUDE_DATA_HW_PRI + s; i++;
			}
			//sensbox.alt_priority[i] = ALTITUDE_DATA_HW_SEC; i++;
			//sensbox.alt_priority[i] = ALTITUDE_DATA_HW_EXT; i++;
			break;
		case 'L': case 'l':
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_LXNAV_UART_PRI; i++;
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_LXNAV_UART_SEC; i++;
			break;
		case 'O': case 'o':
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_OGN_UART_PRI; i++;
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_OGN_UART_SEC; i++;
			break;
		case 'R': case 'r':
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_PGRMZ_UART_PRI; i++;
			sensbox.alt_priority[i] = ALTITUDE_DATA_SW_PGRMZ_UART_SEC; i++;
			break;
		}
		p++;
	}
	if (!i) {
		fprintf(stderr, "[WARN] Priority option '%s' for barometric sensors parsing error. Using default option: 'LOHR'\n", s);
		initBaroSensorPriority(DEFAULT_ALTITUDE_SENSOR_PRIORITY);
		return;
	}
	sensbox.alt_priority[i] = ALTITUDE_DATA_SW_GNSS_UART_PRI; i++;
	sensbox.alt_priority[i] = ALTITUDE_DATA_SW_GNSS_UART_SEC; i++;
	sensbox.alt_sensor_oount = i;

	sensbox.active_altitude = NULL;

	for(i=0; i<sensbox.alt_sensor_oount; i++) {
		struct altitude_data *sensor = &sensbox.alt_data[sensbox.alt_priority[i]];
		sensor->last_update = mstime() - 2 * SENSOR_TIMEOUT; // make sensor invalid in the next cycle
		sensor->valid = false;
	}
}

struct altitude_data *altitudeSensorIndexBySource(
		gnss_source_t src_uart, int sentence) {

	struct altitude_data *result;
	switch (sentence) {
	case MINMEA_SENTENCE_GGA: case MINMEA_SENTENCE_GSA: {
		result =
				(src_uart == UART_PRI) ?
						&sensbox.alt_data[ALTITUDE_DATA_SW_GNSS_UART_PRI] :
						&sensbox.alt_data[ALTITUDE_DATA_SW_GNSS_UART_SEC];
		break;
	}
	case MINMEA_EXT_SENTENCE_POGNB: {
		result =
				(src_uart == UART_PRI) ?
						&sensbox.alt_data[ALTITUDE_DATA_SW_OGN_UART_PRI] :
						&sensbox.alt_data[ALTITUDE_DATA_SW_OGN_UART_SEC];
		break;
	}
	case MINMEA_EXT_SENTENCE_LXWP0: {
		result =
				(src_uart == UART_PRI) ?
						&sensbox.alt_data[ALTITUDE_DATA_SW_LXNAV_UART_PRI] :
						&sensbox.alt_data[ALTITUDE_DATA_SW_LXNAV_UART_SEC];
		break;
	}
	case MINMEA_EXT_SENTENCE_PGRMZ: {
		result =
				(src_uart == UART_PRI) ?
						&sensbox.alt_data[ALTITUDE_DATA_SW_PGRMZ_UART_PRI] :
						&sensbox.alt_data[ALTITUDE_DATA_SW_PGRMZ_UART_SEC];
		break;
	}
	default: {
		result = NULL;
		break;
	}
	}
	if (result == NULL)
		fprintf(stderr,
				"WARNING! *altitudeSensorIndexBySource returns NULL. Revise the code!\n");
	return result;
}

void initGNSS(){
	sensbox.gnss_pri.source = UART_PRI;
	sensbox.gnss_pri.gnss_valid = false;
	sensbox.gnss_pri.active = false;
	sensbox.gnss_pri.score = 0;
	sensbox.gnss_pri.hdop = 99.9;
	sensbox.gnss_pri.vdop = 99.9;
	sensbox.gnss_pri.pdop = 99.9;
	sensbox.gnss_pri.last_update = mstime() - 2 * SENSOR_TIMEOUT; // make sensor invalid in the next cycle;
	sensbox.gnss_pri.name = "GNSS PRIMARY";

	sensbox.gnss_sec.source = UART_SEC;
	sensbox.gnss_sec.gnss_valid = false;
	sensbox.gnss_sec.active = false;
	sensbox.gnss_sec.score = 0;
	sensbox.gnss_sec.hdop = 99.9;
	sensbox.gnss_sec.vdop = 99.9;
	sensbox.gnss_sec.pdop = 99.9;
	sensbox.gnss_sec.last_update = mstime() - 2 * SENSOR_TIMEOUT; // make sensor invalid in the next cycle;
	sensbox.gnss_sec.name = "GNSS SECONDARY";

	sensbox.pflau_pri.ac_count = 0;
	sensbox.pflau_pri.last_update = mstime() - 2 * SENSOR_TIMEOUT; // make sensor invalid in the next cycle;

	sensbox.pflau_sec.ac_count = 0;
	sensbox.pflau_sec.last_update = mstime() - 2 * SENSOR_TIMEOUT; // make sensor invalid in the next cycle;

	sensbox.active_gnss = NULL;
}

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

void periodicGNSSCheck() {
	uint64_t t = mstime() - sensbox.gnss_pri.last_update;
	sensbox.gnss_pri.active = (bool) (t < SENSOR_TIMEOUT);

	t = mstime() - sensbox.gnss_sec.last_update;
	sensbox.gnss_sec.active = (bool) (t < SENSOR_TIMEOUT);

	t = mstime() - sensbox.pflau_pri.last_update;
	sensbox.pflau_pri.ac_count = (t < SENSOR_TIMEOUT) ? sensbox.pflau_pri.ac_count : 0;

	t = mstime() - sensbox.pflau_sec.last_update;
	sensbox.pflau_sec.ac_count = (t < SENSOR_TIMEOUT) ? sensbox.pflau_sec.ac_count : 0;

	uint64_t pri_score = (uint64_t)sensbox.gnss_pri.active * (uint64_t) sensbox.gnss_pri.gnss_valid *  sensbox.gnss_pri.score;
	uint64_t sec_score = (uint64_t)sensbox.gnss_sec.active * (uint64_t) sensbox.gnss_sec.gnss_valid *  sensbox.gnss_sec.score;

	sensbox.active_gnss = pri_score >= sec_score ? &sensbox.gnss_pri : &sensbox.gnss_sec;
	sensbox.active_gnss = max(pri_score, sec_score) == 0 ? NULL : sensbox.active_gnss;
}

void proceedExtededNMEA(struct nmea_msg *msg) {
	gnss_source_t src_uart = msg->source;
	char *s = msg->nmea;

	switch (minmea_ext_sentence_id(s)) {
	case MINMEA_EXT_SENTENCE_PFLAU: {
		struct minmea_sentence_pflau frame;
		struct _pflau_data *pflau = (msg->source == UART_PRI) ? &sensbox.pflau_pri : &sensbox.pflau_sec;
		if (minmea_parse_pflau(&frame, s)) {
			pflau->ac_count = frame.rx;
			pflau->last_update = mstime();
			msg->type = NMEA_FLARM_PFLAU;
		} else pflau->ac_count = 0;
	}
		break;
	case MINMEA_EXT_SENTENCE_POGNB: {
		struct minmea_sentence_pognb frame;
		struct altitude_data *alt = altitudeSensorIndexBySource(src_uart,
							MINMEA_EXT_SENTENCE_POGNB);
		if (minmea_parse_pognb(&frame, s)) {
			alt->source = msg->source;
			alt->altitude_m = minmea_tofloat(&frame.altitude_baro_m);
			alt->altitude_feet = alt->altitude_m * COEF_METERS_TO_FEETS;
			alt->hw = SENSOR_BARO_NMEA_OGN;
			alt->valid = true;
			alt->last_update = mstime();
			msg->type = NMEA_ALT;
		} else alt->valid = false;
	}
		break;
	case MINMEA_EXT_SENTENCE_LXWP0: {
		struct minmea_sentence_lxwp0 frame;
		struct altitude_data *alt = altitudeSensorIndexBySource(src_uart,
							MINMEA_EXT_SENTENCE_LXWP0);
		if (minmea_parse_lxwp0(&frame, s)) {
			alt->source = msg->source;
			alt->altitude_m = minmea_tofloat(&frame.altitude_baro_m);
			alt->altitude_feet = alt->altitude_m * COEF_METERS_TO_FEETS;
			alt->hw = SENSOR_BARO_NMEA_LXNAV;
			alt->valid = true;
			alt->last_update = mstime();
			msg->type = NMEA_ALT;
		} else alt->valid = false;
	}
		break;
	case MINMEA_EXT_SENTENCE_PGRMZ: {
		struct minmea_sentence_pgrmz frame;
		struct altitude_data *alt = altitudeSensorIndexBySource(src_uart,
							MINMEA_EXT_SENTENCE_PGRMZ);
		if (minmea_parse_pgrmz(&frame, s)) {
			switch(frame.units) {
			case 'M': case 'm': { alt->altitude_m = minmea_tofloat(&frame.altitude); alt->altitude_feet = alt->altitude_m * COEF_METERS_TO_FEETS; break; }
			case 'F': case 'f': { alt->altitude_feet = minmea_tofloat(&frame.altitude); alt->altitude_m = alt->altitude_feet * COEF_FEETS_TO_METERS; break; }
			}
			alt->source = msg->source;
			alt->hw = SENSOR_BARO_NMEA_RMZ;
			alt->valid = true;
			alt->last_update = mstime();
			msg->type = NMEA_ALT;
		} else alt->valid = false;
	}
		break;

	case MINMEA_EXT_SENTENCE_PFLAA: msg->type = NMEA_FLARM_PFLAA; break;
	case MINMEA_EXT_INVALID: msg->type = NMEA_INVALID; break;
	case MINMEA_EXT_UNKNOWN: msg->type = NMEA_UNKNOWN; break;

	}
}

void proceedGenericNMEA(struct nmea_msg *msg) {

	gnss_source_t src_uart = msg->source;
	char *s = msg->nmea;
	struct gnss_data *gnss = (src_uart == UART_PRI) ? &sensbox.gnss_pri : &sensbox.gnss_sec;
	msg->type = NMEA_UNKNOWN;


	switch (minmea_sentence_id(s, false)) {
	case MINMEA_SENTENCE_RMC: {
		struct minmea_sentence_rmc frame;
		if (minmea_parse_rmc(&frame, s)) {

			/* Latitude */
			gnss->lat = minmea_tocoord(&frame.latitude);
			gnss->lon = minmea_tocoord(&frame.longitude);

			/* Speed */
			gnss->speed = minmea_tofloat(&frame.speed);

			/* Course */
			gnss->course = minmea_tofloat(&frame.speed);
			//gnss->course = isnan(gnss->course) ? gnss->course : 0;

			/* Date and time, also with gnss sentences rate*/
			uint64_t t_prev = (uint64_t) (gnss->gnss_time.tv_sec * 1000 + gnss->gnss_time.tv_nsec / NSEC_MULT);
			minmea_gettime(&gnss->gnss_time, &frame.date, &frame.time);
			uint64_t t_cur = (uint64_t) (gnss->gnss_time.tv_sec * 1000 + gnss->gnss_time.tv_nsec / NSEC_MULT);
			gnss->gnss_rate = (unsigned int) (t_cur - t_prev);

			/* Valid */
			gnss->gnss_valid = frame.valid;

			/* Timings and update rate */
			t_cur = mstime();
			gnss->rate = (unsigned int) (t_cur - gnss->last_update);
			gnss->last_update = t_cur;
			gnss->active = true;

			msg->type = NMEA_GNSS;
		}
	}
		break;

	case MINMEA_SENTENCE_GGA: {
		struct minmea_sentence_gga frame;
		struct altitude_data *alt = altitudeSensorIndexBySource(src_uart,
							MINMEA_SENTENCE_GGA);
		if (minmea_parse_gga(&frame, s)) {

			/* Quality */
			gnss->satellites_tracked = frame.satellites_tracked;
			gnss->fix_quality = frame.fix_quality;
			gnss->gnss_valid = (bool) gnss->fix_quality;
			/* Simply scroring of GNSS receiver */
			gnss->score = (1048576 * (1 + gnss->fix_quality)) - (int)(4096 * gnss->vdop) - (16 * (gnss->rate + gnss->gnss_rate)) + (1024 * gnss->satellites_tracked);

			/* Altitude */
			gnss->altitude_gnss = minmea_tofloat(&frame.altitude);
			gnss->altitude_gnss_units = frame.altitude_units;

			switch (gnss->altitude_gnss_units) {
			case 'M':
			case 'm':
				gnss->altitude_gnss_feets = gnss->altitude_gnss
						* COEF_METERS_TO_FEETS;
				alt->altitude_m = gnss->altitude_gnss;
				alt->altitude_feet = alt->altitude_m * COEF_METERS_TO_FEETS;
				break;
			case 'F':
			case 'f':
				gnss->altitude_gnss_feets = gnss->altitude_gnss;
				alt->altitude_feet = gnss->altitude_gnss;
				alt->altitude_m = alt->altitude_feet * COEF_FEETS_TO_METERS;
				break; // Does anybody see 'F' in the GGA sentence?
			default:
				gnss->alt_gnss_valid = false;
				break;
			}
			alt->source = msg->source;
			alt->altitude_feet = gnss->altitude_gnss_feets;
			alt->hw = SENSOR_GEOM_NMEA_GNSS;
			alt->valid = gnss->alt_gnss_valid && gnss->gnss_valid;
			if(alt->valid) alt->last_update = mstime();
			gnss->active = true;

			msg->type = NMEA_GNSS;
		}
	}
		break;

	case MINMEA_SENTENCE_GSA: {
		struct minmea_sentence_gsa frame;
		if (minmea_parse_gsa(&frame, s)) {
			/* Quality */
			gnss->fix_type = frame.fix_type;
			gnss->pdop = minmea_tofloat(&frame.pdop);
			gnss->hdop = minmea_tofloat(&frame.hdop);
			gnss->vdop = minmea_tofloat(&frame.vdop);

			gnss->alt_gnss_valid = (bool) (gnss->fix_type > 2);
			gnss->gnss_valid = (gnss->fix_type > 1);
			gnss->active = true;

			msg->type = NMEA_GNSS;
		}
	}
		break;

	case MINMEA_SENTENCE_VTG: {
		struct minmea_sentence_vtg frame;
		if (minmea_parse_vtg(&frame, s)) {

			/* Track */
			gnss->true_track_degrees = minmea_tofloat(
					&frame.true_track_degrees);
			gnss->true_track_degrees =
					isnan(gnss->true_track_degrees) ?
							gnss->true_track_degrees : 0;

			gnss->magnetic_track_degrees = minmea_tofloat(
					&frame.magnetic_track_degrees);

			/* Speed */
			gnss->speed_kph = minmea_tofloat(&frame.speed_kph);
			gnss->active = true;

			msg->type = NMEA_GNSS;
		}
	}
		break;
	case MINMEA_UNKNOWN: proceedExtededNMEA(msg); break;
	case MINMEA_SENTENCE_GBS:
	case MINMEA_SENTENCE_GLL:
	case MINMEA_SENTENCE_GST:
	case MINMEA_SENTENCE_GSV:
	case MINMEA_SENTENCE_ZDA: msg->type = NMEA_GNSS; break;
	case MINMEA_INVALID: msg->type = NMEA_INVALID; break;
	}
}
