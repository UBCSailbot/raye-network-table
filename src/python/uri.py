""" NOTE: These are Not actually constants.
If referenced in another file, the values can
be reassigned in the scope of that specific project.
"""

# accelerometer
ACCELEROMETER = "/accelerometer"
ACCELEROMETER_X = "/accelerometer/boat_orientation_data/x_axis_acceleration"
ACCELEROMETER_Y = "/accelerometer/boat_orientation_data/y_axis_acceleration"
ACCELEROMETER_Z = "/accelerometer/boat_orientation_data/z_axis_acceleration"

BMS = "/bms"
BMS_CURRENT = "/battery_pack_data/battery_current"
BMS_VOLTAGE = "/battery_pack_data/battery_voltage"
BMS_MAXCELL = "/battery_pack_data/battery_max_voltage"
BMS_MINCELL = "/battery_pack_data/battery_min_voltage"

CHARGER_CURRENT = "/charger_data/charger_current"
CHARGER_VOLTAGE = "/charger_data/charger_voltage"
CHARGER_MPPT_STATUS = "/charger_data/mppt_status"
CHARGER_CFET_STATUS = "/charger_data/cfet_status"
CHARGER_LFET_STATUS = "/charger_data/lfet_status"

POWER_CONTROLLER = "/bms/battery_state"
POWER_PV_MPPT = "/bms/battery_state/pv_mppt_engage"
POWER_PWR = "/bms/battery_state/pwr_engage"
POWER_MPPT = "/bms/battery_state/mppt_engage"

BMS1 = "/bms_1"
BMS1_CURRENT = "/bms_1/battery_pack_data/battery_current"
BMS1_VOLTAGE = "/bms_1/battery_pack_data/battery_voltage"
BMS1_MAXCELL = "/bms_1/battery_pack_data/battery_max_voltage"
BMS1_MINCELL = "/bms_1/battery_pack_data/battery_min_voltage"
BMS1_CHARGER_CURRENT = "/bms_1/charger_data/charger_current"
BMS1_CHARGER_VOLTAGE = "/bms_1/charger_data/charger_voltage"
BMS1_CHARGER_MPPT_STATUS = "/bms_1/charger_data/mppt_status"
BMS1_CHARGER_CFET_STATUS = "/bms_1/charger_data/cfet_status"
BMS1_CHARGER_LFET_STATUS = "/bms_1/charger_data/lfet_status"

BMS2 = "/bms_2"
BMS2_CURRENT = "/bms_2/battery_pack_data/battery_current"
BMS2_VOLTAGE = "/bms_2/battery_pack_data/battery_voltage"
BMS2_MAXCELL = "/bms_2/battery_pack_data/battery_max_voltage"
BMS2_MINCELL = "/bms_2/battery_pack_data/battery_min_voltage"
BMS2_CHARGER_CURRENT = "/bms_2/charger_data/charger_current"
BMS2_CHARGER_VOLTAGE = "/bms_2/charger_data/charger_voltage"
BMS2_CHARGER_MPPT_STATUS = "/bms_2/charger_data/mppt_status"
BMS2_CHARGER_CFET_STATUS = "/bms_2/charger_data/cfet_status"
BMS2_CHARGER_LFET_STATUS = "/bms_2/charger_data/lfet_status"

BMS3 = "/bms_3"
BMS3_CURRENT = "/bms_3/battery_pack_data/battery_current"
BMS3_VOLTAGE = "/bms_3/battery_pack_data/battery_voltage"
BMS3_MAXCELL = "/bms_3/battery_pack_data/battery_max_voltage"
BMS3_MINCELL = "/bms_3/battery_pack_data/battery_min_voltage"
BMS3_CHARGER_CURRENT = "/bms_3/charger_data/charger_current"
BMS3_CHARGER_VOLTAGE = "/bms_3/charger_data/charger_voltage"
BMS3_CHARGER_MPPT_STATUS = "/bms_3/charger_data/mppt_status"
BMS3_CHARGER_CFET_STATUS = "/bms_3/charger_data/cfet_status"
BMS3_CHARGER_LFET_STATUS = "/bms_3/charger_data/lfet_status"

BMS4 = "/bms_4"
BMS4_CURRENT = "/bms_4/battery_pack_data/battery_current"
BMS4_VOLTAGE = "/bms_4/battery_pack_data/battery_voltage"
BMS4_MAXCELL = "/bms_4/battery_pack_data/battery_max_voltage"
BMS4_MINCELL = "/bms_4/battery_pack_data/battery_min_voltage"
BMS4_CHARGER_CURRENT = "/bms_4/charger_data/charger_current"
BMS4_CHARGER_VOLTAGE = "/bms_4/charger_data/charger_voltage"
BMS4_CHARGER_MPPT_STATUS = "/bms_4/charger_data/mppt_status"
BMS4_CHARGER_CFET_STATUS = "/bms_4/charger_data/cfet_status"
BMS4_CHARGER_LFET_STATUS = "/bms_4/charger_data/lfet_status"

BMS5 = "/bms_5"
BMS5_CURRENT = "/bms_5/battery_pack_data/battery_current"
BMS5_VOLTAGE = "/bms_5/battery_pack_data/battery_voltage"
BMS5_MAXCELL = "/bms_5/battery_pack_data/battery_max_voltage"
BMS5_MINCELL = "/bms_5/battery_pack_data/battery_min_voltage"
BMS5_CHARGER_CURRENT = "/bms_5/charger_data/charger_current"
BMS5_CHARGER_VOLTAGE = "/bms_5/charger_data/charger_voltage"
BMS5_CHARGER_MPPT_STATUS = "/bms_5/charger_data/mppt_status"
BMS5_CHARGER_CFET_STATUS = "/bms_5/charger_data/cfet_status"
BMS5_CHARGER_LFET_STATUS = "/bms_5/charger_data/lfet_status"

BMS6 = "/bms_6"
BMS6_CURRENT = "/bms_6/battery_pack_data/battery_current"
BMS6_VOLTAGE = "/bms_6/battery_pack_data/battery_voltage"
BMS6_MAXCELL = "/bms_6/battery_pack_data/battery_max_voltage"
BMS6_MINCELL = "/bms_6/battery_pack_data/battery_min_voltage"
BMS6_CHARGER_CURRENT = "/bms_6/charger_data/charger_current"
BMS6_CHARGER_VOLTAGE = "/bms_6/charger_data/charger_voltage"
BMS6_CHARGER_MPPT_STATUS = "/bms_6/charger_data/mppt_status"
BMS6_CHARGER_CFET_STATUS = "/bms_6/charger_data/cfet_status"
BMS6_CHARGER_LFET_STATUS = "/bms_6/charger_data/lfet_status"

# gps sensors
GPS_CAN = "/gps_can"
GPS_CAN_TIME = "/gps_can/gprmc/utc_timestamp"
GPS_CAN_LAT = "/gps_can/gprmc/latitude"
GPS_CAN_LON = "/gps_can/gprmc/longitude"
GPS_CAN_GNDSPEED = "/gps_can/gprmc/ground_speed"
GPS_CAN_TMG = "/gps_can/gprmc/track_made_good"
GPS_CAN_TRUE_HEADING = "/gps_can/gprmc/true_heading"
GPS_CAN_MAGVAR = "/gps_can/gprmc/magnetic_variation"

GPS_CAN_VALID = "/gps_can/gprmc/data_valid"
GPS_CAN_VARWEST = "/gps_can/gprmc/magvar_west"
GPS_CAN_LATNORTH = "/gps_can/gprmc/lat_north"
GPS_CAN_LONWEST = "/gps_can/gprmc/lon_west"

GPS_AIS = "/gps_ais"
GPS_AIS_TIME = "/gps_ais/gprmc/utc_timestamp"
GPS_AIS_LAT = "/gps_ais/gprmc/latitude"
GPS_AIS_LON = "/gps_ais/gprmc/longitude"
GPS_AIS_GNDSPEED = "/gps_ais/gprmc/ground_speed"
GPS_AIS_TMG = "/gps_ais/gprmc/track_made_good"
GPS_AIS_TRUE_HEADING = "/gps_ais/gprmc/true_heading"
GPS_AIS_MAGVAR = "/gps_ais/gprmc/magnetic_variation"

GPS_AIS_VALID = "/gps_ais/gprmc/data_valid"
GPS_AIS_VARWEST = "/gps_ais/gprmc/magvar_west"
GPS_AIS_LATNORTH = "/gps_ais/gprmc/lat_north"
GPS_AIS_LONWEST = "/gps_ais/gprmc/lon_west"

# gyroscope
GYROSCOPE = "/gyroscope"
GYROSCOPE_X = "/gyroscope/angular_motion_data/x_velocity"
GYROSCOPE_Y = "/gyroscope/angular_motion_data/y_velocity"
GYROSCOPE_Z = "/gyroscope/angular_motion_data/z_velocity"

# actuators
ACTUATION = "/actuation_angle"
RUDDER = "/actuation_angle/rudder"
RUDDER_PORT = "/actuation_angle/rudder/rudder_port"
RUDDER_STBD = "/actuation_angle/rudder/rudder_stbd"
RUDDER_PORT_ANGLE = "/actuation_angle/rudder/rudder_port/angle"
RUDDER_STBD_ANGLE = "/actuation_angle/rudder/rudder_stbd/angle"

WINCH = "/actuation_angle/winch"
WINCH_MAIN = "/actuation_angle/winch/winch_main"
WINCH_JIB = "/actuation_angle/winch/winch_jib"
WINCH_MAIN_ANGLE = "/actuation_angle/winch/winch_main/angle"
WINCH_JIB_ANGLE = "/actuation_angle/winch/winch_jib/angle"


# boom sensor
SAILENCODER = "/sailencoder"
SAILENCODER_ANGLE = "/sailencoder/boom_angle_data/angle"

# wind sensors
WIND1 = "/wind_sensor_1"
WIND1_SPEED = "/wind_sensor_1/iimwv/wind_speed"
WIND1_ANGLE = "/wind_sensor_1/iimwv/wind_angle"

WIND2 = "/wind_sensor_2"
WIND2_SPEED = "/wind_sensor_2/iimwv/wind_speed"
WIND2_ANGLE = "/wind_sensor_2/iimwv/wind_angle"

WIND3 = "/wind_sensor_3"
WIND3_SPEED = "/wind_sensor_3/iimwv/wind_speed"
WIND3_ANGLE = "/wind_sensor_3/iimwv/wind_angle"

# waypoints
WAYPOINTS_GP = "waypoints"

# UCCM
# boom sensor
BOOM_UCCM_CURRENT = "/boom_angle_sensor/uccm/current"
BOOM_UCCM_VOLTAGE = "/boom_angle_sensor/uccm/voltage"
BOOM_UCCM_TEMP = "/boom_angle_sensor/uccm/temperature"
BOOM_UCCM_STATUS = "/boom_angle_sensor/uccm/status"

# rudders
RUDDER0_UCCM_CURRENT = "/rudder_motor_control_0/uccm/current"
RUDDER0_UCCM_VOLTAGE = "/rudder_motor_control_0/uccm/voltage"
RUDDER0_UCCM_TEMP = "/rudder_motor_control_0/uccm/temperature"
RUDDER0_UCCM_STATUS = "/rudder_motor_control_0/uccm/status"

RUDDER1_UCCM_CURRENT = "/rudder_motor_control_1/uccm/current"
RUDDER1_UCCM_VOLTAGE = "/rudder_motor_control_1/uccm/voltage"
RUDDER1_UCCM_TEMP = "/rudder_motor_control_1/uccm/temperature"
RUDDER1_UCCM_STATUS = "/rudder_motor_control_1/uccm/status"

# winch
WINCH0_UCCM_CURRENT = "/winch_motor_control_0/uccm/current"
WINCH0_UCCM_VOLTAGE = "/winch_motor_control_0/uccm/voltage"
WINCH0_UCCM_TEMP = "/winch_motor_control_0/uccm/temperature"
WINCH0_UCCM_STATUS = "/winch_motor_control_0/uccm/status"

WINCH1_UCCM_CURRENT = "/winch_motor_control_1/uccm/current"
WINCH1_UCCM_VOLTAGE = "/winch_motor_control_1/uccm/voltage"
WINCH1_UCCM_TEMP = "/winch_motor_control_1/uccm/temperature"
WINCH1_UCCM_STATUS = "/winch_motor_control_1/uccm/status"

# wind sensors
WIND0_UCCM_CURRENT = "/wind_sensor_0/uccm/current"
WIND0_UCCM_VOLTAGE = "/wind_sensor_0/uccm/voltage"
WIND0_UCCM_TEMP = "/wind_sensor_0/uccm/temperature"
WIND0_UCCM_STATUS = "/wind_sensor_0/uccm/status"

WIND1_UCCM_CURRENT = "/wind_sensor_1/uccm/current"
WIND1_UCCM_VOLTAGE = "/wind_sensor_1/uccm/voltage"
WIND1_UCCM_TEMP = "/wind_sensor_1/uccm/temperature"
WIND1_UCCM_STATUS = "/wind_sensor_1/uccm/status"

WINCH1_UCCM = "/winch_motor_control_1"

WIND2_UCCM_CURRENT = "/wind_sensor_2/uccm/current"
WIND2_UCCM_VOLTAGE = "/wind_sensor_2/uccm/voltage"
WIND2_UCCM_TEMP = "/wind_sensor_2/uccm/temperature"
WIND2_UCCM_STATUS = "/wind_sensor_2/uccm/status"

# gps
GPS_CAN_UCCM_CURRENT = "/gps_can/uccm/current"
GPS_CAN_UCCM_VOLTAGE = "/gps_can/uccm/voltage"
GPS_CAN_UCCM_TEMP = "/gps_can/uccm/temperature"
GPS_CAN_UCCM_STATUS = "/gps_can/uccm/status"

GPS_AIS_UCCM_CURRENT = "/gps_ais/uccm/current"
GPS_AIS_UCCM_VOLTAGE = "/gps_ais/uccm/voltage"
GPS_AIS_UCCM_TEMP = "/gps_ais/uccm/temperature"
GPS_AIS_UCCM_STATUS = "/gps_ais/uccm/status"

# bms
BMS0_UCCM_CURRENT = "/bms_0/uccm/current"
BMS0_UCCM_VOLTAGE = "/bms_0/uccm/total_voltage"
BMS0_UCCM_TEMP = "/bms_0/uccm/temperature"
BMS0_UCCM_STATUS = "/bms_0/uccm/status"

BMS1_UCCM_CURRENT = "/bms_1/uccm/current"
BMS1_UCCM_VOLTAGE = "/bms_1/uccm/total_voltage"
BMS1_UCCM_TEMP = "/bms_1/uccm/temperature"
BMS1_UCCM_STATUS = "/bms_1/uccm/status"

BMS2_UCCM_CURRENT = "/bms_2/uccm/current"
BMS2_UCCM_VOLTAGE = "/bms_2/uccm/total_voltage"
BMS2_UCCM_TEMP = "/bms_2/uccm/temperature"
BMS2_UCCM_STATUS = "/bms_2/uccm/status"

BMS3_UCCM_CURRENT = "/bms_3/uccm/current"
BMS3_UCCM_VOLTAGE = "/bms_3/uccm/total_voltage"
BMS3_UCCM_TEMP = "/bms_3/uccm/temperature"
BMS3_UCCM_STATUS = "/bms_3/uccm/status"

BMS4_UCCM_CURRENT = "/bms_4/uccm/current"
BMS4_UCCM_VOLTAGE = "/bms_4/uccm/total_voltage"
BMS4_UCCM_TEMP = "/bms_4/uccm/temperature"
BMS4_UCCM_STATUS = "/bms_4/uccm/status"

BMS5_UCCM_CURRENT = "/bms_5/uccm/current"
BMS5_UCCM_VOLTAGE = "/bms_5/uccm/total_voltage"
BMS5_UCCM_TEMP = "/bms_5/uccm/temperature"
BMS5_UCCM_STATUS = "/bms_5/uccm/status"

# accelerometer
ACCEL_UCCM_CURRENT = "/accelerometer/uccm/current"
ACCEL_UCCM_VOLTAGE = "/accelerometer/uccm/voltage"
ACCEL_UCCM_TEMP = "/accelerometer/uccm/temperature"
ACCEL_UCCM_STATUS = "/accelerometer/uccm/status"

# remote can command
REMOTE_CAN_CMD = "/remote_can_cmd"
REMOTE_CAN_CMD_ID = "/remote_can_cmd/id"
REMOTE_CAN_CMD_data = "/remote_can_cmd/data"
