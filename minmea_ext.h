/*
 * Copyright © 2023 Denis G Dugushkin <denis.dugushkin@gmail.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */

#ifndef MINMEA_EXT_H
#define MINMEA_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "minmea.h"

enum minmea_ext_sentence_id {
    MINMEA_EXT_INVALID = -1,
    MINMEA_EXT_UNKNOWN = 0,
	MINMEA_EXT_SENTENCE_PFLAA = MINMEA_SENTENCE_ZDA + 1,
    MINMEA_EXT_SENTENCE_PFLAU,
	MINMEA_EXT_SENTENCE_PGRMZ,
	MINMEA_EXT_SENTENCE_POGNB,
	MINMEA_EXT_SENTENCE_LXWP0
};

struct minmea_sentence_pflau {
	int rx;
	int tx;
	int gps;
	int power;
	int alarm_level;
	int relative_bearing;
	int alarm_type;
	int relative_vertical;
	unsigned int relative_distance;
	char* id;
};

struct minmea_sentence_pgrmz {
	struct minmea_float altitude;
	char units;
	int pos_fix_dimenstions;
};

struct minmea_sentence_lxwp0 {
	struct minmea_float ias; 			 // IAS [km/h]
	struct minmea_float altitude_baro_m; // baroaltitude [m]
	struct minmea_float vario; 			 // vario values [m/s]
	int heading; 		 // heading of plane [deg]
	struct minmea_float windcourse;		 // windcourse [deg]
	struct minmea_float windspeed; 		 // windspeed [km/h]
};

struct minmea_sentence_pognb {
	struct minmea_float interval;        // Time interval (s)
	struct minmea_float temperature;     // Temperature [C]
	struct minmea_float altitude_baro_m; // Baro altitude [m]
	struct minmea_float altitude_cr;     // Correlated altiutude with GNSS [m]
	struct minmea_float climb_rate;		 // Climb rate [m/s]
};

enum minmea_ext_sentence_id minmea_ext_sentence_id(const char *sentence);
bool minmea_parse_pflau(struct minmea_sentence_pflau *frame, const char *sentence);
bool minmea_parse_pgrmz(struct minmea_sentence_pgrmz *frame, const char *sentence);
bool minmea_parse_lxwp0(struct minmea_sentence_lxwp0 *frame, const char *sentence);
bool minmea_parse_pognb(struct minmea_sentence_pognb *frame, const char *sentence);

#ifdef __cplusplus
}
#endif

#endif /* MINMEA_EXT_H */
