#include "sensors.h"

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
		void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
		void *intf_ptr);


void user_delay_us(uint32_t period, void *intf_ptr) {
	UNUSED(intf_ptr);
	usleep(period);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
		void *intf_ptr) {
	struct identifier id;
	id = *((struct identifier*) intf_ptr);
	struct i2c_msg read_user_reg[2] = { { id.dev_addr, 0, 1, &reg_addr }, {
			id.dev_addr, I2C_M_RD, (uint16_t) len, data } };

	struct i2c_rdwr_ioctl_data messagebuffer = {read_user_reg, 2};

	int rc = ioctl(id.fd, I2C_RDWR, &messagebuffer);
	if (rc < 0) return -1; // Read error
	return BME280_OK;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
		void *intf_ptr) {
	struct identifier id;
	id = *((struct identifier*) intf_ptr);
	uint8_t *buf;
	uint16_t buf_len = len + 1;

	buf = (uint8_t*) malloc(buf_len);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);

	struct i2c_msg write_reg = { id.dev_addr, 0, buf_len, buf };
	struct i2c_rdwr_ioctl_data messagebuffer;
	messagebuffer.nmsgs = 1;
	messagebuffer.msgs = &write_reg;

	int rc = ioctl(id.fd, I2C_RDWR, &messagebuffer);
	free(buf);

	if (rc < 0) return -1; // Write error

	return BME280_OK;
}


int sensor_bme280_init(struct sensor_bme280 *bme280) {

	int8_t rslt = BME280_OK;
	if ((bme280->id.fd = open(bme280->addr, O_RDWR)) < 0)
		return -1;

	if (ioctl(bme280->id.fd, I2C_SLAVE_FORCE, bme280->id.dev_addr) < 0)
		return -2;


	/* Make sure to select BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC as needed */
	bme280->id.dev_addr = BME280_I2C_ADDR_PRIM;

	bme280->dev.intf = BME280_I2C_INTF;
	bme280->dev.read = user_i2c_read;
	bme280->dev.write = user_i2c_write;
	bme280->dev.delay_us = user_delay_us;

	/* Update interface pointer with the structure that contains both device address and file descriptor */
	bme280->dev.intf_ptr = &bme280->id;

	/* Initialize the bme280 */
	rslt = bme280_init(&bme280->dev);
	if (rslt != BME280_OK)
		return -3;

	uint8_t settings_sel = 0;

	/* Recommended mode of operation: Indoor navigation */
	bme280->dev.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
	bme280->dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280->dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280->dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280->dev.settings.filter = BME280_FILTER_COEFF_16;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL
			| BME280_OSR_HUM_SEL | BME280_FILTER_SEL | BME280_STANDBY_SEL;
	/* Set the sensor settings */
	rslt = bme280_set_sensor_settings(settings_sel, &bme280->dev);
	if (rslt != BME280_OK)
		return -4;

	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280->dev);
	if (rslt != BME280_OK)
		return -5;

	// First initial read
	bme280_get_sensor_data(BME280_ALL, &bme280->comp_data, &bme280->dev);
	return rslt;
}

int sensor_bme280_read_altitude_feet(struct sensor_bme280 *bme280, struct gnss_data *data) {

	int rslt = bme280_get_sensor_data(BME280_PRESS, &bme280->comp_data, &bme280->dev);
	if (rslt != BME280_OK) {
		data->alt_baro_valid = false;
		return BAD_ALT;
	}
	data->alt_baro_valid = true;
	data->altitude_baro_feets = 145366.45 * (1 - (pow((0.01 * bme280->comp_data.pressure) / PRESSURE_BASE_HPA, 0.190294)));
	return 0;
}

int sensor_bme280_read_all(struct sensor_bme280 *bme280) {
	int rslt = bme280_get_sensor_data(BME280_ALL, &bme280->comp_data, &bme280->dev);
	if (rslt != BME280_OK) return -1;
	return 0;
}

void sensor_bme280_close(struct sensor_bme280 *bme280) {
	close(bme280->id.fd);
}


void readGnssNmea(char *s, struct gnss_data *data) {
    switch (minmea_sentence_id(s, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, s)) {

            	/* Latitude */
            	data->lat = minmea_tocoord(&frame.latitude);
            	data->lon = minmea_tocoord(&frame.longitude);

            	/* Speed */
            	data->speed = minmea_tofloat(&frame.speed);

            	/* Course */
            	data->course = minmea_tofloat(&frame.speed);
            	data->course = isnan(data->course) ? data->course : 0;

            	/* Date and time */
            	minmea_gettime(&data->gnss_time, &frame.date, &frame.time);

            	/* Valid */
            	data->gnss_valid = frame.valid;
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, s)) {

            	/* Quality */
            	data->satellites_tracked = frame.satellites_tracked;
            	data->fix_quality = frame.fix_quality;
            	data->gnss_valid = (bool) data->fix_quality;

            	/* Altitude */
            	data->alt_gnss_valid = (data->fix_quality == 0);
            	data->altitude_gnss = minmea_tofloat(&frame.altitude);
            	data->altitude_gnss_units = frame.altitude_units;
        		switch (data->altitude_gnss_units) {
        		case 'M': data->altitude_gnss_feets = data->altitude_gnss * COEF_METERS_TO_FEETS; break;
        		case 'F': data->altitude_gnss_feets = data->altitude_gnss; break; // Does anybody see 'F' in the GGA sentence?
        		default: data->alt_gnss_valid = false; break;
        		}
            }
        } break;

        case MINMEA_SENTENCE_GSA: {
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, s)) {

            	/* Quality */
            	data->fix_type = frame.fix_type;
            	data->pdop = minmea_tofloat(&frame.pdop);
            	data->hdop = minmea_tofloat(&frame.hdop);
            	data->vdop = minmea_tofloat(&frame.vdop);

            	data->alt_gnss_valid = (data->fix_type == 3);
            	data->gnss_valid = (data->fix_type > 1);
            }
        } break;

        case MINMEA_SENTENCE_VTG: {
            struct minmea_sentence_vtg frame;
            if (minmea_parse_vtg(&frame, s)) {

            	/* Track */
            	data->true_track_degrees = minmea_tofloat(&frame.true_track_degrees);
            	data->true_track_degrees = isnan(data->true_track_degrees) ? data->true_track_degrees : 0;

            	data->magnetic_track_degrees = minmea_tofloat(&frame.magnetic_track_degrees);

            	/* Speed */
            	data->speed_kph = minmea_tofloat(&frame.speed_kph);
            }
        } break;

        case MINMEA_INVALID: case MINMEA_UNKNOWN: case MINMEA_SENTENCE_GBS: case MINMEA_SENTENCE_GLL: case MINMEA_SENTENCE_GST: case MINMEA_SENTENCE_GSV: case MINMEA_SENTENCE_ZDA: break;
    }
}
