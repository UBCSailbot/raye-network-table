""" NOTE: These are Not actually constants.
If referenced in another file, the values can
be reassigned in the scope of that specific project.
"""

# boom sensor
SAILENCODER_ANGLE = "/sailencoder/sensor_data/angle"

# wind sensors
WIND1_SPEED = "/wind_sensor_1/iimwv/wind_speed"
WIND1_ANGLE = "/wind_sensor_1/iimwv/wind_angle"

WIND2_SPEED = "/wind_sensor_2/iimwv/wind_speed"
WIND2_ANGLE = "/wind_sensor_2/iimwv/wind_angle"

WIND3_SPEED = "/wind_sensor_3/iimwv/wind_speed"
WIND3_ANGLE = "/wind_sensor_3/iimwv/wind_angle"

# gps sensors
GPS_CAN_TIME = "/gps_can/gprmc/utc_timestamp"
GPS_CAN_LAT = "/gps_can/gprmc/latitude"
GPS_CAN_LON = "/gps_can/gprmc/longitude"
GPS_CAN_GNDSPEED = "/gps_can/gprmc/ground_speed"
GPS_CAN_TMG = "/gps_can/gprmc/track_made_good"
GPS_CAN_TRUE_HEADING = "/gps_can/gprmc/true_heading"
GPS_CAN_MAGVAR = "/gps_can/gprmc/magnetic_variation"

GPS_AIS_TIME = "/gps_ais/gprmc/utc_timestamp"
GPS_AIS_LAT = "/gps_ais/gprmc/latitude"
GPS_AIS_LON = "/gps_ais/gprmc/longitude"
GPS_AIS_GNDSPEED = "/gps_ais/gprmc/ground_speed"
GPS_AIS_TMG = "/gps_ais/gprmc/track_made_good"
GPS_AIS_TRUE_HEADING = "/gps_ais/gprmc/true_heading"
GPS_AIS_MAGVAR = "/gps_ais/gprmc/magnetic_variation"

# bms
BMS1_CURRENT = "/bms_1/battery_pack_data/current"
BMS1_VOLTAGE = "/bms_1/battery_pack_data/total_voltage"
BMS1_MAX_CELL = "/bms_1/battery_pack_data/max_cell"
BMS1_MIN_CELL = "/bms_1/battery_pack_data/min_cell"

BMS2_CURRENT = "/bms_2/battery_pack_data/current"
BMS2_VOLTAGE = "/bms_2/battery_pack_data/total_voltage"
BMS2_MAX_CELL = "/bms_2/battery_pack_data/max_cell"
BMS2_MIN_CELL = "/bms_2/battery_pack_data/min_cell"

BMS3_CURRENT = "/bms_3/battery_pack_data/current"
BMS3_VOLTAGE = "/bms_3/battery_pack_data/total_voltage"
BMS3_MAX_CELL = "/bms_3/battery_pack_data/max_cell"
BMS3_MIN_CELL = "/bms_3/battery_pack_data/min_cell"

BMS4_CURRENT = "/bms_4/battery_pack_data/current"
BMS4_VOLTAGE = "/bms_4/battery_pack_data/total_voltage"
BMS4_MAX_CELL = "/bms_4/battery_pack_data/max_cell"
BMS4_MIN_CELL = "/bms_4/battery_pack_data/min_cell"

BMS5_CURRENT = "/bms_5/battery_pack_data/current"
BMS5_VOLTAGE = "/bms_5/battery_pack_data/total_voltage"
BMS5_MAX_CELL = "/bms_5/battery_pack_data/max_cell"
BMS5_MIN_CELL = "/bms_5/battery_pack_data/min_cell"

BMS6_CURRENT = "/bms_6/battery_pack_data/current"
BMS6_VOLTAGE = "/bms_6/battery_pack_data/total_voltage"
BMS6_MAX_CELL = "/bms_6/battery_pack_data/max_cell"
BMS6_MIN_CELL = "/bms_6/battery_pack_data/min_cell"

# accelerometer
ACCELEROMETER_X = "/accelerometer/boat_orientation_data/x_axis_acceleration"
ACCELEROMETER_Y = "/accelerometer/boat_orientation_data/y_axis_acceleration"
ACCELEROMETER_Z = "/accelerometer/boat_orientation_data/z_axis_acceleration"

# gyroscope
GYROSCOPE_X = "/gyroscope/x_velocity"
GYROSCOPE_Y = "/gyroscope/y_velocity"
GYROSCOPE_Z = "/gyroscope/z_velocity"

# actuators
ACTUATION = "/actuation_angle"
RUDDER = "/actuation_angle/rudder"
RUDDER_PORT_ANGLE = "/actuation_angle/rudder/rudder_port/angle"
RUDDER_STBD_ANGLE = "/actuation_angle/rudder/rudder_stbd/angle"
WINCH = "/actuation_angle/winch"
WINCH_MAIN_ANGLE = "/actuation_angle/winch/winch_main/angle"
WINCH_JIB_ANGLE = "/actuation_angle/winch/winch_jib/angle"

# UCCM
# TODO: need to update these eventually
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
WIND0_UCCM_CURRENT = "/wind_sensor_0/uccm/status"

WIND1_UCCM_CURRENT = "/wind_sensor_1/uccm/current"
WIND1_UCCM_VOLTAGE = "/wind_sensor_1/uccm/voltage"
WIND1_UCCM_TEMP = "/wind_sensor_1/uccm/temperature"
WIND1_UCCM_CURRENT = "/wind_sensor_1/uccm/status"

WIND2_UCCM_CURRENT = "/wind_sensor_2/uccm/current"
WIND2_UCCM_VOLTAGE = "/wind_sensor_2/uccm/voltage"
WIND2_UCCM_TEMP = "/wind_sensor_2/uccm/temperature"
WIND2_UCCM_CURRENT = "/wind_sensor_2/uccm/status"

# gps
GPS0_UCCM_CURRENT = "/gps_0/uccm/current"
GPS0_UCCM_VOLTAGE = "/gps_0/uccm/voltage"
GPS0_UCCM_TEMP = "/gps_0/uccm/temperature"
GPS0_UCCM_STATUS = "/gps_0/uccm/status"

GPS1_UCCM_CURRENT = "/gps_1/uccm/current"
GPS1_UCCM_VOLTAGE = "/gps_1/uccm/voltage"
GPS1_UCCM_TEMP = "/gps_1/uccm/temperature"
GPS1_UCCM_STATUS = "/gps_1/uccm/status"

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
