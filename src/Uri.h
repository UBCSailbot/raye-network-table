// Copyright 2017 UBC Sailbot

#ifndef URI_H_
#define URI_H_

#define SAILENCODER_ANGLE "/sailencoder/sensor_data/angle"

#define WIND1_SPEED "/wind_sensor_1/iimwv/wind_speed"
#define WIND1_ANGLE "/wind_sensor_1/iimwv/wind_angle"

#define WIND2_SPEED "/wind_sensor_2/iimwv/wind_speed"
#define WIND2_ANGLE "/wind_sensor_2/iimwv/wind_angle"

#define WIND3_SPEED "/wind_sensor_3/iimwv/wind_speed"
#define WIND3_ANGLE "/wind_sensor_3/iimwv/wind_angle"

#define GPS_CAN_TIME "/gps_0/gprmc/utc_timestamp"
#define GPS_CAN_LAT "/gps_0/gprmc/latitude"
#define GPS_CAN_LON "/gps_0/gprmc/longitude"
#define GPS_CAN_GNDSPEED "/gps_0/gprmc/ground_speed"
#define GPS_CAN_TMG "/gps_0/gprmc/track_made_good"
#define GPS_CAN_TRUE_HEADING "/gps_0/gprmc/true_heading"
#define GPS_CAN_MAGVAR "/gps_0/gprmc/magnetic_variation"

#define GPS_AIS_TIME "/gps_1/gprmc/utc_timestamp"
#define GPS_AIS_LAT "/gps_1/gprmc/latitude"
#define GPS_AIS_LON "/gps_1/gprmc/longitude"
#define GPS_AIS_GNDSPEED "/gps_1/gprmc/ground_speed"
#define GPS_AIS_TMG "/gps_1/gprmc/track_made_good"
#define GPS_AIS_TRUE_HEADING "/gps_1/gprmc/true_heading"
#define GPS_AIS_MAGVAR "/gps_1/gprmc/magnetic_variation"

#define BMS1_CURRENT "/bms_1/battery_pack_data/current"
#define BMS1_VOLTAGE "/bms_1/battery_pack_data/total_voltage"
#define BMS1_MAXCELL "/bms_1/battery_pack_data/max_cell"
#define BMS1_MINCELL "/bms_1/battery_pack_data/min_cell"

#define BMS2_CURRENT "/bms_2/battery_pack_data/current"
#define BMS2_VOLTAGE "/bms_2/battery_pack_data/total_voltage"
#define BMS2_MAXCELL "/bms_2/battery_pack_data/max_cell"
#define BMS2_MINCELL "/bms_2/battery_pack_data/min_cell"

#define BMS3_CURRENT "/bms_3/battery_pack_data/current"
#define BMS3_VOLTAGE "/bms_3/battery_pack_data/total_voltage"
#define BMS3_MAXCELL "/bms_3/battery_pack_data/max_cell"
#define BMS3_MINCELL "/bms_3/battery_pack_data/min_cell"

#define BMS4_CURRENT "/bms_4/battery_pack_data/current"
#define BMS4_VOLTAGE "/bms_4/battery_pack_data/total_voltage"
#define BMS4_MAXCELL "/bms_4/battery_pack_data/max_cell"
#define BMS4_MINCELL "/bms_4/battery_pack_data/min_cell"

#define BMS5_CURRENT "/bms_5/battery_pack_data/current"
#define BMS5_VOLTAGE "/bms_5/battery_pack_data/total_voltage"
#define BMS5_MAXCELL "/bms_5/battery_pack_data/max_cell"
#define BMS5_MINCELL "/bms_5/battery_pack_data/min_cell"

#define BMS6_CURRENT "/bms_6/battery_pack_data/current"
#define BMS6_VOLTAGE "/bms_6/battery_pack_data/total_voltage"
#define BMS6_MAXCELL "/bms_6/battery_pack_data/max_cell"
#define BMS6_MINCELL "/bms_6/battery_pack_data/min_cell"

#define ACCELEROMETER_X "/accelerometer/boat_orientation_data/x_axis_acceleration"
#define ACCELEROMETER_Y "/accelerometer/boat_orientation_data/y_axis_acceleration"
#define ACCELEROMETER_Z "/accelerometer/boat_orientation_data/z_axis_acceleration"

#define GYROSCOPE_X "/gyroscope/x_velocity"
#define GYROSCOPE_Y "/gyroscope/y_velocity"
#define GYROSCOPE_Z "/gyroscope/z_velocity"

#define RUDDER_PORT_ANGLE "/actuation_angle/rudder/rudder_port/angle"
#define RUDDER_STBD_ANGLE "/actuation_angle/rudder/rudder_stbd/angle"
#define WINCH_MAIN_ANGLE "/actuation_angle/winch/winch_main/angle"
#define WINCH_JIB_ANGLE "/actuation_angle/winch/winch_jib/angle"

//TODO: Add UCCM Uris

#endif  // URI_H_
