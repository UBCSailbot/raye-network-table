// Copyright 2017 UBC Sailbot

#ifndef URI_H_
#define URI_H_

#define SAILENCODER "/sailencoder"
#define SAILENCODER_ANGLE "/sailencoder/sensor_data/angle"

#define WIND_SPEED "/iimwv/wind_speed"
#define WIND_ANGLE "/iimwv/wind_angle"

#define WIND1 "/wind_sensor_1"
#define WIND1_SPEED "/wind_sensor_1/iimwv/wind_speed"
#define WIND1_ANGLE "/wind_sensor_1/iimwv/wind_angle"

#define WIND2 "/wind_sensor_2"
#define WIND2_SPEED "/wind_sensor_2/iimwv/wind_speed"
#define WIND2_ANGLE "/wind_sensor_2/iimwv/wind_angle"

#define WIND3 "/wind_sensor_3"
#define WIND3_SPEED "/wind_sensor_3/iimwv/wind_speed"
#define WIND3_ANGLE "/wind_sensor_3/iimwv/wind_angle"

// Dependency within global-pathfinding - Do NOT change
#define GPS_CAN "/gps_can"
#define GPS_CAN_LAT "/gps_can/gprmc/latitude"
#define GPS_CAN_LON "/gps_can/gprmc/longitude"

#define GPS_CAN_GNDSPEED "/gps_can/gprmc/ground_speed"
#define GPS_CAN_TMG "/gps_can/gprmc/track_made_good"
#define GPS_CAN_TRUE_HEADING "/gps_can/gprmc/true_heading"
#define GPS_CAN_MAGVAR "/gps_can/gprmc/magnetic_variation"
#define GPS_CAN_TIME "/gps_can/gprmc/utc_timestamp"

#define GPS_CAN_VALID "/gps_can/gprmc/valid"
#define GPS_CAN_VARWEST "/gps_can/gprmc/var_west"
#define GPS_CAN_LATNORTH "/gps_can/gprmc/lat_north"
#define GPS_CAN_LONWEST "/gps_can/gprmc/lon_west"

#define GPS_AIS "/gps_ais"
#define GPS_AIS_TIME "/gps_ais/gprmc/utc_timestamp"
#define GPS_AIS_LAT "/gps_ais/gprmc/latitude"
#define GPS_AIS_LON "/gps_ais/gprmc/longitude"
#define GPS_AIS_GNDSPEED "/gps_ais/gprmc/ground_speed"
#define GPS_AIS_TMG "/gps_ais/gprmc/track_made_good"
#define GPS_AIS_TRUE_HEADING "/gps_ais/gprmc/true_heading"
#define GPS_AIS_MAGVAR "/gps_ais/gprmc/magnetic_variation"

#define POWER "/bms"
#define POWER_CONTROLLER "/bms/battery_state"
#define POWER_PV_MPPT "/bms/battery_state/pv_mppt_engage"
#define POWER_PWR "/bms/battery_state/pwr_engage"
#define POWER_MPPT "/bms/battery_state/mppt_engage"

#define BMS_CURRENT "/battery_pack_data/current"
#define BMS_VOLTAGE "/battery_pack_data/total_voltage"
#define BMS_MAXCELL "/battery_pack_data/max_cell"
#define BMS_MINCELL "/battery_pack_data/min_cell"

#define BMS1 "/bms_1"
#define BMS1_CURRENT "/bms_1/battery_pack_data/current"
#define BMS1_VOLTAGE "/bms_1/battery_pack_data/total_voltage"
#define BMS1_MAXCELL "/bms_1/battery_pack_data/max_cell"
#define BMS1_MINCELL "/bms_1/battery_pack_data/min_cell"

#define BMS2 "/bms_2"
#define BMS2_CURRENT "/bms_2/battery_pack_data/current"
#define BMS2_VOLTAGE "/bms_2/battery_pack_data/total_voltage"
#define BMS2_MAXCELL "/bms_2/battery_pack_data/max_cell"
#define BMS2_MINCELL "/bms_2/battery_pack_data/min_cell"

#define BMS3 "/bms_3"
#define BMS3_CURRENT "/bms_3/battery_pack_data/current"
#define BMS3_VOLTAGE "/bms_3/battery_pack_data/total_voltage"
#define BMS3_MAXCELL "/bms_3/battery_pack_data/max_cell"
#define BMS3_MINCELL "/bms_3/battery_pack_data/min_cell"

#define BMS4 "/bms_4"
#define BMS4_CURRENT "/bms_4/battery_pack_data/current"
#define BMS4_VOLTAGE "/bms_4/battery_pack_data/total_voltage"
#define BMS4_MAXCELL "/bms_4/battery_pack_data/max_cell"
#define BMS4_MINCELL "/bms_4/battery_pack_data/min_cell"

#define BMS5 "/bms_5"
#define BMS5_CURRENT "/bms_5/battery_pack_data/current"
#define BMS5_VOLTAGE "/bms_5/battery_pack_data/total_voltage"
#define BMS5_MAXCELL "/bms_5/battery_pack_data/max_cell"
#define BMS5_MINCELL "/bms_5/battery_pack_data/min_cell"

#define BMS6 "/bms_6"
#define BMS6_CURRENT "/bms_6/battery_pack_data/current"
#define BMS6_VOLTAGE "/bms_6/battery_pack_data/total_voltage"
#define BMS6_MAXCELL "/bms_6/battery_pack_data/max_cell"
#define BMS6_MINCELL "/bms_6/battery_pack_data/min_cell"

#define ACCELEROMETER "/accelerometer"
#define ACCELEROMETER_X "/accelerometer/boat_orientation_data/x_axis_acceleration"
#define ACCELEROMETER_Y "/accelerometer/boat_orientation_data/y_axis_acceleration"
#define ACCELEROMETER_Z "/accelerometer/boat_orientation_data/z_axis_acceleration"

#define GYROSCOPE "/gyroscope"
#define GYROSCOPE_X "/gyroscope/x_velocity"
#define GYROSCOPE_Y "/gyroscope/y_velocity"
#define GYROSCOPE_Z "/gyroscope/z_velocity"

#define ACTUATION "/actuation_angle"
#define RUDDER "/actuation_angle/rudder"
#define RUDDER_PORT_ANGLE "/actuation_angle/rudder/rudder_port/angle"
#define RUDDER_STBD_ANGLE "/actuation_angle/rudder/rudder_stbd/angle"
#define WINCH "/actuation_angle/winch"
#define WINCH_MAIN_ANGLE "/actuation_angle/winch/winch_main/angle"
#define WINCH_JIB_ANGLE "/actuation_angle/winch/winch_jib/angle"

// Dependency within global-pathfinding - Do NOT change
#define WAYPOINTS_GP "waypoints"

//TODO: Add UCCM Uris

#endif  // URI_H_
