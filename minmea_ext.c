/*
 * Copyright © 2023 Denis G Dugushkin <denis.dugushkin@gmail.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */

#include <string.h>
#include <stdio.h>
#include "minmea_ext.h"

enum minmea_ext_sentence_id minmea_ext_sentence_id(const char *sentence)
{
    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return MINMEA_EXT_INVALID;

    if (!strcmp(type, "PFLAA"))
        return MINMEA_EXT_SENTENCE_PFLAA;
    if (!strcmp(type, "PFLAU"))
        return MINMEA_EXT_SENTENCE_PFLAU;
    if (!strcmp(type, "PGRMZ"))
        return MINMEA_EXT_SENTENCE_PGRMZ;
    if (!strcmp(type, "POGNB"))
        return MINMEA_EXT_SENTENCE_POGNB;
    if (!strcmp(type, "LXWP0"))
        return MINMEA_EXT_SENTENCE_LXWP0;

    return MINMEA_EXT_UNKNOWN;
}

bool minmea_parse_pflau(struct minmea_sentence_pflau *frame, const char *sentence)
{
    // $PFLAU,2,1,2,1,1,-45,2,50,75,1A304C*62
    char type[6];
    if (!minmea_scan(sentence, "tiiiiiiiii;s",
            type,
            &frame->rx,
            &frame->tx,
            &frame->gps,
            &frame->power,
            &frame->alarm_level,
            &frame->relative_bearing,
            &frame->alarm_type,
            &frame->relative_vertical,
			&frame->relative_distance,
			&frame->id
            ))
        return false;
    if (strcmp(type, "PFLAU"))
        return false;

    return true;
}

bool minmea_parse_pgrmz(struct minmea_sentence_pgrmz *frame, const char *sentence)
{
    // $PGRMZ,246,f,3*1B
    char type[6];
    if (!minmea_scan(sentence, "tfci",
            type,
            &frame->altitude,
            &frame->units,
            &frame->pos_fix_dimenstions
            ))
        return false;
    if (strcmp(type, "PGRMZ"))
        return false;

    return true;
}

bool minmea_parse_lxwp0(struct minmea_sentence_lxwp0 *frame, const char *sentence)
{
	  // $LXWP0,logger_stored, airspeed, airaltitude,
	  //   v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
	  //
	  // 0 loger_stored : [Y|N] (not used in LX1600)
	  // 1 IAS [km/h] ----> Condor uses TAS!
	  // 2 baroaltitude [m]
	  // 3-8 vario values [m/s] (last 6 measurements in last second)
	  // 9 heading of plane (not used in LX1600)
	  // 10 windcourse [deg] (not used in LX1600)
	  // 11 windspeed [km/h] (not used in LX1600)
	  //
	  // e.g.:
	  // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1*47
    char type[6];
    if (!minmea_scan(sentence, "t_fff_____iff",
            type,
            &frame->ias,
            &frame->altitude_baro_m,
            &frame->vario,
            &frame->heading,
            &frame->windcourse,
            &frame->windspeed
            ))
    	return false;
    if (strcmp(type, "LXWP0"))
    	return false;

    return true;
}

bool minmea_parse_pognb(struct minmea_sentence_pognb *frame, const char *sentence)
{
	  // $POGNB,<Time>,<Temp>,<NS>,<????>,<PAlt>,<CrAlt>,<CRate>*CHK
    char type[6];
    if (!minmea_scan(sentence, "tff__fff",
            type,
			&frame->interval,
            &frame->temperature,
            &frame->altitude_baro_m,
            &frame->altitude_cr,
            &frame->climb_rate
            ))
        return false;
    if (strcmp(type, "POGNB"))
        return false;

    return true;
}
