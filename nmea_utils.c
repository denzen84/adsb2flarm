#include "nmea_utils.h"

/* Global buffer */
static char nmea_buffer[NMEA_MAX_BUFFER];

uint8_t calc_nmea_checksum(char *s, int len) {
	uint8_t checksum = 0x00;
	if (!s) return checksum;
	char *p;
	p = s;
	while (p < s + len) checksum ^= *p++;
	return checksum;
}

static inline char *update_nmea_checksum(char *nmea, int max_len) {
	snprintf(nmea, max_len, "$%s*%02X\r\n", nmea_buffer, calc_nmea_checksum(nmea_buffer, strlen(nmea_buffer)));
	return nmea;
}

char* generate_nmea_pgrmz(char *nmea, int max_len, double alt_feet) {
	snprintf(nmea_buffer, NMEA_MAX_BUFFER, "PGRMZ,%.2f,F,3", alt_feet);
	return update_nmea_checksum(nmea, max_len);
}

char* generate_nmea_gptxt(char *nmea, int max_len, const char* txt) {
	snprintf(nmea_buffer, NMEA_MAX_BUFFER, "GPTXT,%s", txt);
	return update_nmea_checksum(nmea, max_len);
}

// This is canonical code but we never use PFLAU for warnings
/*
char* generate_nmea_pflau(char *nmea, int max_len, struct pflau_data *pflau) {
	snprintf(nmea_buffer, NMEA_MAX_BUFFER, "PFLAU,%d,%d,%d,%d,%d,%d,%02X,%d,%d,%06X",
			pflau->RX,
			pflau->TX,
			pflau->GPS,
			pflau->Power,
			pflau->AlarmLevel,
			pflau->RelativeBearing,
			pflau->AlarmType,
			pflau->RelativeVertical,
			pflau->RelativeDistance,
			pflau->ID);
	return update_nmea_checksum(nmea, max_len);
}
*/

char* generate_nmea_pflau(char *nmea, int max_len, struct pflau_data *pflau) {
	snprintf(nmea_buffer, NMEA_MAX_BUFFER, "PFLAU,%d,%d,%d,%d,%d,,%02X,,,",
			pflau->RX,
			pflau->TX,
			pflau->GPS,
			pflau->Power,
			pflau->AlarmLevel,
			//pflau->RelativeBearing,
			pflau->AlarmType
			//pflau->RelativeVertical,
			//pflau->RelativeDistance,
			//pflau->ID
			);
	return update_nmea_checksum(nmea, max_len);
}


char* generate_nmea_pflaa(char *nmea, int max_len, struct pflaa_data *pflaa) {
	snprintf(nmea_buffer, NMEA_MAX_BUFFER, "PFLAA,%d,%d,%d,%d,%d,%06X,%d,%d,%d,%.1f,%d,%d,%d,%.1f",
			pflaa->AlarmLevel,
			pflaa->RelativeNorth,
			pflaa->RelativeEast,
			pflaa->RelativeVertical,
			pflaa->IDType,
			pflaa->ID,
			pflaa->Track,
			pflaa->TurnRate,
			pflaa->GroundSpeed,
			pflaa->ClimbRate,
			pflaa->AcftType,
			pflaa->NoTrack,
			pflaa->Source,
			pflaa->RSSI);
	return update_nmea_checksum(nmea, max_len);
}

