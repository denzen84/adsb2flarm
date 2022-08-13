#include <stdio.h>
#include <string.h>
#include <stdint.h>


#define NMEA_MAX_MESSAGE 256 // Quite enough
#define NMEA_MAX_BUFFER 192  // Quite enough

struct pflau_data {
	int RX;
	int TX;
	int GPS;
	int Power;
	int AlarmLevel;
	int RelativeBearing;
	int AlarmType;
	int RelativeVertical;
	int RelativeDistance;
	int ID;
};

struct pflaa_data {
	int AlarmLevel;
	int RelativeNorth;
	int RelativeEast;
	int RelativeVertical;
	int IDType;
	int ID;
	int Track;
	int TurnRate;
	int GroundSpeed;
	float ClimbRate;
	int AcftType;
	int NoTrack;
	int Source;
	float RSSI;
};

uint8_t calc_nmea_checksum(char *s, int len);
char* generate_nmea_pgrmz(char *nmea, int max_len, double alt_feet);
char* generate_nmea_gptxt(char *nmea, int max_len, const char* txt);
char* generate_nmea_pflau(char *nmea, int max_len, struct pflau_data *pflau);
char* generate_nmea_pflaa(char *nmea, int max_len, struct pflaa_data *pflaa);
