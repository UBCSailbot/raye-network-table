from generated_python import Value_pb2
from .uri import *


class Help:
    def create_int_val(self, value):
        int_val = Value_pb2.Value()
        int_val.type = Value_pb2.Value.Type.INT
        int_val.int_data = int(value)
        return int_val

    def create_str_val(self, value):
        str_val = Value_pb2.Value()
        str_val.type = Value_pb2.Value.Type.STRING
        str_val.string_data = value
        return str_val

    def create_float_val(self, value):
        float_val = Value_pb2.Value()
        float_val.type = Value_pb2.Value.Type.FLOAT
        float_val.float_data = value
        return float_val

    def create_bool_val(self, value):
        bool_val = Value_pb2.Value()
        bool_val.type = Value_pb2.Value.Type.BOOL
        bool_val.bool_data = value
        return bool_val

    def sensors_to_root(self, sensors):
        values = {}

        # Store boom sensor values
        val = self.create_int_val(sensors.sailencoder_sensor.boom_angle_data.angle)
        values[SAILENCODER_ANGLE] = val

        # Store wind sensor values
        val = self.create_float_val(sensors.wind_sensor_1.iimwv.wind_speed)
        values[WIND1_SPEED] = val
        val = self.create_int_val(sensors.wind_sensor_1.iimwv.wind_angle)
        values[WIND1_ANGLE] = val

        val = self.create_float_val(sensors.wind_sensor_2.iimwv.wind_speed)
        values[WIND2_SPEED] = val
        val = self.create_int_val(sensors.wind_sensor_2.iimwv.wind_angle)
        values[WIND2_ANGLE] = val

        val = self.create_float_val(sensors.wind_sensor_3.iimwv.wind_speed)
        values[WIND3_SPEED] = val
        val = self.create_int_val(sensors.wind_sensor_3.iimwv.wind_angle)
        values[WIND3_ANGLE] = val

        # Store gps sensor values
        val = self.create_str_val(sensors.gps_can.gprmc.utc_timestamp)
        values[GPS_CAN_TIME] = val
        val = self.create_float_val(sensors.gps_can.gprmc.latitude)
        values[GPS_CAN_LAT] = val
        val = self.create_float_val(sensors.gps_can.gprmc.longitude)
        values[GPS_CAN_LON] = val
        val = self.create_float_val(sensors.gps_can.gprmc.ground_speed)
        values[GPS_CAN_GNDSPEED] = val
        val = self.create_float_val(sensors.gps_can.gprmc.track_made_good)
        values[GPS_CAN_TMG] = val
        val = self.create_float_val(sensors.gps_can.gprmc.magnetic_variation)
        values[GPS_CAN_MAGVAR] = val
        val = self.create_float_val(sensors.gps_can.gprmc.true_heading)
        values[GPS_CAN_TRUE_HEADING] = val
        val = self.create_bool_val(sensors.gps_can.gprmc.data_valid)
        values[GPS_CAN_VALID] = val
        val = self.create_bool_val(sensors.gps_can.gprmc.magvar_west)
        values[GPS_CAN_VARWEST] = val
        val = self.create_bool_val(sensors.gps_can.gprmc.lat_north)
        values[GPS_CAN_LATNORTH] = val
        val = self.create_bool_val(sensors.gps_can.gprmc.lon_west)
        values[GPS_CAN_LONWEST] = val

        val = self.create_str_val(sensors.gps_ais.gprmc.utc_timestamp)
        values[GPS_AIS_TIME] = val
        val = self.create_float_val(sensors.gps_ais.gprmc.latitude)
        values[GPS_AIS_LAT] = val
        val = self.create_float_val(sensors.gps_ais.gprmc.longitude)
        values[GPS_AIS_LON] = val
        val = self.create_float_val(sensors.gps_ais.gprmc.ground_speed)
        values[GPS_AIS_GNDSPEED] = val
        val = self.create_float_val(sensors.gps_ais.gprmc.track_made_good)
        values[GPS_AIS_TMG] = val
        val = self.create_float_val(sensors.gps_ais.gprmc.magnetic_variation)
        values[GPS_AIS_MAGVAR] = val

        # Store bms sensor values
        val = self.create_float_val(sensors.bms_1.battery_pack_data.battery_current)
        values[BMS1_CURRENT] = val
        val = self.create_float_val(sensors.bms_1.battery_pack_data.battery_voltage)
        values[BMS1_VOLTAGE] = val

        val = self.create_float_val(sensors.bms_2.battery_pack_data.battery_current)
        values[BMS2_CURRENT] = val
        val = self.create_float_val(sensors.bms_2.battery_pack_data.battery_voltage)
        values[BMS2_VOLTAGE] = val

        val = self.create_float_val(sensors.bms_3.battery_pack_data.battery_current)
        values[BMS3_CURRENT] = val
        val = self.create_float_val(sensors.bms_3.battery_pack_data.battery_voltage)
        values[BMS3_VOLTAGE] = val

        val = self.create_float_val(sensors.bms_4.battery_pack_data.battery_current)
        values[BMS4_CURRENT] = val
        val = self.create_float_val(sensors.bms_4.battery_pack_data.battery_voltage)
        values[BMS4_VOLTAGE] = val

        val = self.create_float_val(sensors.bms_5.battery_pack_data.battery_current)
        values[BMS5_CURRENT] = val
        val = self.create_float_val(sensors.bms_5.battery_pack_data.battery_voltage)
        values[BMS5_VOLTAGE] = val

        val = self.create_float_val(sensors.bms_6.battery_pack_data.battery_current)
        values[BMS6_CURRENT] = val
        val = self.create_float_val(sensors.bms_6.battery_pack_data.battery_voltage)
        values[BMS6_VOLTAGE] = val

        val = self.create_float_val(
            sensors.accelerometer.boat_orientation_data.x_axis_acceleration)
        values[ACCELEROMETER_X] = val
        val = self.create_float_val(
            sensors.accelerometer.boat_orientation_data.y_axis_acceleration)
        values[ACCELEROMETER_Y] = val
        val = self.create_float_val(
            sensors.accelerometer.boat_orientation_data.z_axis_acceleration)
        values[ACCELEROMETER_Z] = val

        val = self.create_float_val(sensors.gyroscope.angular_motion_data.x_velocity)
        values[GYROSCOPE_X] = val
        val = self.create_float_val(sensors.gyroscope.angular_motion_data.y_velocity)
        values[GYROSCOPE_Y] = val
        val = self.create_float_val(sensors.gyroscope.angular_motion_data.z_velocity)
        values[GYROSCOPE_Z] = val

        return values

    def uccms_to_root(self, uccms):
        values = {}

        # Store boom sensor uccm values
        val = self.create_int_val(uccms.boom_angle_sensor.current)
        values[BOOM_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.boom_angle_sensor.voltage)
        values[BOOM_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.boom_angle_sensor.temperature)
        values[BOOM_UCCM_TEMP] = val
        val = self.create_str_val(uccms.boom_angle_sensor.status)
        values[BOOM_UCCM_STATUS] = val

        # Store rudder motor uccm values
        val = self.create_int_val(uccms.rudder_motor_control_0.current)
        values[RUDDER0_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.rudder_motor_control_0.voltage)
        values[RUDDER0_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.rudder_motor_control_0.temperature)
        values[RUDDER0_UCCM_TEMP] = val
        val = self.create_str_val(uccms.rudder_motor_control_0.status)
        values[RUDDER0_UCCM_STATUS] = val

        val = self.create_int_val(uccms.rudder_motor_control_1.current)
        values[RUDDER1_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.rudder_motor_control_1.voltage)
        values[RUDDER1_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.rudder_motor_control_1.temperature)
        values[RUDDER1_UCCM_TEMP] = val
        val = self.create_str_val(uccms.rudder_motor_control_1.status)
        values[RUDDER1_UCCM_STATUS] = val

        # Store winch motor uccm values
        val = self.create_int_val(uccms.winch_motor_control_0.current)
        values[WINCH0_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.winch_motor_control_0.voltage)
        values[WINCH0_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.winch_motor_control_0.temperature)
        values[WINCH0_UCCM_TEMP] = val
        val = self.create_str_val(uccms.winch_motor_control_0.status)
        values[WINCH0_UCCM_STATUS] = val

        val = self.create_int_val(uccms.winch_motor_control_1.current)
        values[WINCH1_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.winch_motor_control_1.voltage)
        values[WINCH1_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.winch_motor_control_1.temperature)
        values[WINCH1_UCCM_TEMP] = val
        val = self.create_str_val(uccms.winch_motor_control_1.status)
        values[WINCH1_UCCM_STATUS] = val

        # Store wind sensor uccm values
        val = self.create_int_val(uccms.wind_sensor_0.current)
        values[WIND0_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.wind_sensor_0.voltage)
        values[WIND0_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.wind_sensor_0.temperature)
        values[WIND0_UCCM_TEMP] = val
        val = self.create_str_val(uccms.wind_sensor_0.status)
        values[WIND0_UCCM_STATUS] = val

        val = self.create_int_val(uccms.wind_sensor_1.current)
        values[WIND1_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.wind_sensor_1.voltage)
        values[WIND1_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.wind_sensor_1.temperature)
        values[WIND1_UCCM_TEMP] = val
        val = self.create_str_val(uccms.wind_sensor_1.status)
        values[WIND1_UCCM_STATUS] = val

        val = self.create_int_val(uccms.wind_sensor_2.current)
        values[WIND2_UCCM_CURRENT] = val
        val = self.create_int_val(
            uccms.wind_sensor_2.voltage)
        values[WIND2_UCCM_VOLTAGE] = val
        val = self.create_int_val(
            uccms.wind_sensor_2.temperature)
        values[WIND2_UCCM_TEMP] = val
        val = self.create_str_val(
            uccms.wind_sensor_2.status)
        values[WIND2_UCCM_STATUS] = val

        # Store gps uccm values
        val = self.create_int_val(uccms.gps_can.current)
        values[GPS_CAN_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.gps_can.voltage)
        values[GPS_CAN_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.gps_can.temperature)
        values[GPS_CAN_UCCM_TEMP] = val
        val = self.create_str_val(uccms.gps_can.status)
        values[GPS_CAN_UCCM_STATUS] = val

        val = self.create_int_val(uccms.gps_ais.current)
        values[GPS_AIS_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.gps_ais.voltage)
        values[GPS_AIS_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.gps_ais.temperature)
        values[GPS_AIS_UCCM_TEMP] = val
        val = self.create_str_val(uccms.gps_ais.status)
        values[GPS_AIS_UCCM_STATUS] = val

        # Store bms uccm values
        val = self.create_int_val(uccms.bms_0.current)
        values[BMS0_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_0.voltage)
        values[BMS0_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_0.temperature)
        values[BMS0_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_0.status)
        values[BMS0_UCCM_STATUS] = val

        val = self.create_int_val(uccms.bms_1.current)
        values[BMS1_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_1.voltage)
        values[BMS1_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_1.temperature)
        values[BMS1_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_1.status)
        values[BMS1_UCCM_STATUS] = val

        val = self.create_int_val(uccms.bms_2.current)
        values[BMS2_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_2.voltage)
        values[BMS2_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_2.temperature)
        values[BMS2_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_2.status)
        values[BMS2_UCCM_STATUS] = val

        val = self.create_int_val(uccms.bms_3.current)
        values[BMS3_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_3.voltage)
        values[BMS3_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_3.temperature)
        values[BMS3_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_3.status)
        values[BMS3_UCCM_STATUS] = val

        val = self.create_int_val(uccms.bms_4.current)
        values[BMS4_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_4.voltage)
        values[BMS4_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_4.temperature)
        values[BMS4_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_4.status)
        values[BMS4_UCCM_STATUS] = val

        val = self.create_int_val(uccms.bms_5.current)
        values[BMS5_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.bms_5.voltage)
        values[BMS5_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.bms_5.temperature)
        values[BMS5_UCCM_TEMP] = val
        val = self.create_str_val(uccms.bms_5.status)
        values[BMS5_UCCM_STATUS] = val

        # Store accelerometer uccm values
        val = self.create_int_val(uccms.accelerometer.current)
        values[ACCEL_UCCM_CURRENT] = val
        val = self.create_int_val(uccms.accelerometer.voltage)
        values[ACCEL_UCCM_VOLTAGE] = val
        val = self.create_int_val(uccms.accelerometer.temperature)
        values[ACCEL_UCCM_TEMP] = val
        val = self.create_str_val(uccms.accelerometer.status)
        values[ACCEL_UCCM_STATUS] = val

        return values
