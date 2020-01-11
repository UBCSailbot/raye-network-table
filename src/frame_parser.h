// Copyright 2017 UBC Sailbot

#ifndef FRAME_PARSER_H_
#define FRAME_PARSER_H_

#define BMS_FRAME_ID_1       0x00000008
#define BMS_FRAME_ID_2       0x00000009  // DON'T KNOW THE ACTUAL IDS YET. MAKE SURE TO COORDINATE AND SET IDS ACCORDINGLY
#define BMS_FRAME_ID_3       0x0000000A  // DON'T KNOW THE ACTUAL IDS YET. MAKE SURE TO COORDINATE AND SET IDS ACCORDINGLY
#define BMS_FRAME_ID_4       0x0000000B  // DON'T KNOW THE ACTUAL IDS YET. MAKE SURE TO COORDINATE AND SET IDS ACCORDINGLY
#define BMS_FRAME_ID_5       0x0000000C  // DON'T KNOW THE ACTUAL IDS YET. MAKE SURE TO COORDINATE AND SET IDS ACCORDINGLY
#define BMS_FRAME_ID_6       0x0000000D  // DON'T KNOW THE ACTUAL IDS YET. MAKE SURE TO COORDINATE AND SET IDS ACCORDINGLY

#define GPS_LAT_FRAME_ID     0x00000011
#define GPS_LONG_FRAME_ID    0x00000100
#define GPS_OTHER_FRAME_ID   0x00000101
#define GPS_DATE_FRAME_ID    0x00000110

#define SAILENCODER_FRAME_ID 0x0000000F

#define WIND_SENS0_FRAME_ID  0x00000000
#define WIND_SENS1_FRAME_ID  0x00000010
#define WIND_SENS2_FRAME_ID  0x00000111

#define ACCEL_FRAME_ID       0x000000AC

//GPS
#define GET_GPS_LONG(data) {\
    (((data[0] << 0) + \
	(data[1] << 8) + \
	(data[2] << 16) + \
	(data[3] << 24)) + \
    \
    (((data[4] << 0) + \
    (data[5] << 8) + \
    (data[6] << 16) + \
    (data[7] << 24))/10000000.0))\
}

#define GET_GPS_LAT(data) {\
    (((data[0] << 0) + \
	(data[1] << 8) + \
	(data[2] << 16) + \
	(data[3] << 24)) + \
    \
    (((data[4] << 0) + \
    (data[5] << 8) + \
    (data[6] << 16) + \
    (data[7] << 24))/10000000.0))\
}

#define GET_GPS_GND_SPEED(data) {\
	(float)(((data[0]) + \
	(data[1] << 8))/100.0)\
}
#define GET_GPS_MAG_VAR(data) {\
	(float)(((data[2]) + \
	(data[3] << 8))/10.0)\
}
#define GET_GPS_TMG(data) {\
	(float)(((data[4]) + \
	(data[5] << 8))/100.0)\
}

#define GET_HOUR(data) {\
    data[0]\
}
#define GET_MINUTE(data) {\
    data[1]\
}
#define GET_SECOND(data) {\
    ((data[2]) +\
    (data[3] << 8))/100\
}
#define GET_DAY(data) {\
    (data[4])\
}
#define GET_MONTH(data) {\
    (data[5])\
}
#define GET_YEAR(data) {\
    (data[6])\
}
#define GET_STATUS(data) {\
    (bool)(data[7] & 0b0001)\
}
#define GET_VAR_WEST(data) {\
    (bool)(data[7] & 0b0010)\
}
#define GET_VAR_NORTH(data) {\
    (bool)(data[7] & 0b0100)\
}
#define GET_LONG_WEST(data) {\
    (bool)(data[7] & 0b1000)\
}

// SAIL ENCODER
#define GET_SAILENCODER_ANGLE(data) {\
	(data[0])\
}

// WIND SENSORS
#define GET_WIND_ANGLE(data) {\
	(data[0] + \
	(data[1] << 8) + \
	(data[2] << 16) + \
	(data[3] << 24))\
}
#define GET_WIND_SPEED(data) {\
	((data[4] + \
	(data[5] << 8) + \
	(data[6] << 16) + \
	(data[7] << 24)) / 10)\
}

//BMS
#define GET_BMS_VOLT_DATA(data) {\
    (short)((data[0] + (data[1] << 8)) / 100.0) \
}

#define GET_BMS_CURR_DATA(data) {\
    (short)((data[2] + (data[3] << 8)) / 100.0) \
}

#define GET_BMS_MAXCELL_DATA(data) {\
    (short)(data[4] + (data[5] << 8)) \
}

#define GET_BMS_MINCELL_DATA(data) {\
    (short)(data[6] + (data[7] << 8)) \
}

// ACCELEROMETER
#define GET_ACCEL_X_DATA(data) {\
     (data[0] + \
     (data[1] << 8)) \
}\

#define GET_ACCEL_Y_DATA(data) {\
    (data[2] + \
    (data[3] << 8)) \
}\

#define GET_ACCEL_Z_DATA(data) {\
    (data[4] + \
    (data[5] << 8)) \
}\

#endif  // FRAME_PARSER_H_
