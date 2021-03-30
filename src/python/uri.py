""" NOTE: These are Not actually constants.
If referenced in another file, the values can
be reassigned in the scope of that specific project.
"""

# boom sensor
BOOM_ANGLE = "/boom_angle_sensor/sensor_data/angle"

# wind sensors
WIND0_SPEED = "/wind_sensor_0/iimwv/wind_speed"
WIND0_DIRECTION = "/wind_sensor_0/iimwv/wind_direction"
WIND0_REFERENCE = "/wind_sensor_0/iimwv/wind_reference"
WIND0_TEMP = "/wind_sensor_0/wixdir/wind_temperature"

WIND1_SPEED = "/wind_sensor_1/iimwv/wind_speed"
WIND1_DIRECTION = "/wind_sensor_1/iimwv/wind_direction"
WIND1_REFERENCE = "/wind_sensor_1/iimwv/wind_reference"
WIND1_TEMP = "/wind_sensor_1/wixdir/wind_temperature"

WIND2_SPEED = "/wind_sensor_2/iimwv/wind_speed"
WIND2_DIRECTION = "/wind_sensor_2/iimwv/wind_direction"
WIND2_REFERENCE = "/wind_sensor_2/iimwv/wind_reference"
WIND2_TEMP = "/wind_sensor_2/wixdir/wind_temperature"

# gps sensors
GPS0_TIME = "/gps_0/gprmc/utc_timestamp"
GPS0_LAT = "/gps_0/gprmc/latitude"
GPS0_LON = "/gps_0/gprmc/longitude"
GPS0_LATLOC = "/gps_0/gprmc/latitude_loc"
GPS0_LONLOC = "/gps_0/gprmc/longitude_loc"
GPS0_GNDSPEED = "/gps_0/gprmc/ground_speed"
GPS0_TMG = "/gps_0/gprmc/track_made_good"
GPS0_MAGVAR = "/gps_0/gprmc/magnetic_variation"
GPS0_MAGVARSENSE = "/gps_0/gprmc/magnetic_variation_sense"
GPS0_QUALITY = "/gps_0/gpgga/quality_indicator"
GPS0_HDOP = "/gps_0/gpgga/hdop"
GPS0_ANTENNA = "/gps_0/gpgga/antenna_altitude"
GPS0_GEOIDAL = "/gps_0/gpgga/geoidal_separation"

GPS1_TIME = "/gps_1/gprmc/utc_timestamp"
GPS1_LAT = "/gps_1/gprmc/latitude"
GPS1_LON = "/gps_1/gprmc/longitude"
GPS1_LATLOC = "/gps_1/gprmc/latitude_loc"
GPS1_LONLOC = "/gps_1/gprmc/longitude_loc"
GPS1_GNDSPEED = "/gps_1/gprmc/ground_speed"
GPS1_TMG = "/gps_1/gprmc/track_made_good"
GPS1_MAGVAR = "/gps_1/gprmc/magnetic_variation"
GPS1_MAGVARSENSE = "/gps_1/gprmc/magnetic_variation_sense"
GPS1_QUALITY = "/gps_1/gpgga/quality_indicator"
GPS1_HDOP = "/gps_1/gpgga/hdop"
GPS1_ANTENNA = "/gps_1/gpgga/antenna_altitude"
GPS1_GEOIDAL = "/gps_1/gpgga/geoidal_separation"

# bms
BMS0_CURRENT = "/bms_0/battery_pack_data/current"
BMS0_VOLTAGE = "/bms_0/battery_pack_data/total_voltage"
BMS0_TEMP = "/bms_0/battery_pack_data/temperature"

BMS1_CURRENT = "/bms_1/battery_pack_data/current"
BMS1_VOLTAGE = "/bms_1/battery_pack_data/total_voltage"
BMS1_TEMP = "/bms_1/battery_pack_data/temperature"

BMS2_CURRENT = "/bms_2/battery_pack_data/current"
BMS2_VOLTAGE = "/bms_2/battery_pack_data/total_voltage"
BMS2_TEMP = "/bms_2/battery_pack_data/temperature"

BMS2_CURRENT = "/bms_2/battery_pack_data/current"
BMS3_VOLTAGE = "/bms_3/battery_pack_data/total_voltage"
BMS3_TEMP = "/bms_3/battery_pack_data/temperature"

BMS4_CURRENT = "/bms_4/battery_pack_data/current"
BMS4_VOLTAGE = "/bms_4/battery_pack_data/total_voltage"
BMS4_TEMP = "/bms_4/battery_pack_data/temperature"

BMS5_CURRENT = "/bms_5/battery_pack_data/current"
BMS5_VOLTAGE = "/bms_5/battery_pack_data/total_voltage"
BMS5_TEMP = "/bms_5/battery_pack_data/temperature"

# accelerometer
ACCELEROMETER_X = "/accelerometer/boat_orientation_data/x_axis_acceleration"
ACCELEROMETER_Y = "/accelerometer/boat_orientation_data/y_axis_acceleration"
ACCELEROMETER_Z = "/accelerometer/boat_orientation_data/z_axis_acceleration"

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
