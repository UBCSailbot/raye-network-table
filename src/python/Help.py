from generated_python import Value_pb2


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
        val = self.create_int_val(
            sensors.boom_angle_sensor.sensor_data.angle)
        values["/boom_angle_sensor/boom_angle_sensor/angle"] = val

        # Store wind sensor values
        val = self.create_int_val(sensors.wind_sensor_0.iimwv.wind_speed)
        values["/wind_sensor_0/iimwv/wind_speed"] = val
        val = self.create_int_val(
            sensors.wind_sensor_0.iimwv.wind_direction)
        values["/wind_sensor_0/iimwv/wind_direction"] = val
        val = self.create_int_val(
            sensors.wind_sensor_0.iimwv.wind_reference)
        values["/wind_sensor_0/iimwv/wind_reference"] = val
        val = self.create_int_val(
            sensors.wind_sensor_0.wixdir.wind_temperature)
        values["/wind_sensor_0/wixdir/wind_temperature"] = val

        val = self.create_int_val(sensors.wind_sensor_1.iimwv.wind_speed)
        values["/wind_sensor_1/iimwv/wind_speed"] = val
        val = self.create_int_val(
            sensors.wind_sensor_1.iimwv.wind_direction)
        values["/wind_sensor_1/iimwv/wind_direction"] = val
        val = self.create_int_val(
            sensors.wind_sensor_1.iimwv.wind_reference)
        values["/wind_sensor_1/iimwv/wind_reference"] = val
        val = self.create_int_val(
            sensors.wind_sensor_1.wixdir.wind_temperature)
        values["/wind_sensor_1/wixdir/wind_temperature"] = val

        val = self.create_int_val(sensors.wind_sensor_2.iimwv.wind_speed)
        values["/wind_sensor_2/iimwv/wind_speed"] = val
        val = self.create_int_val(
            sensors.wind_sensor_2.iimwv.wind_direction)
        values["/wind_sensor_2/iimwv/wind_direction"] = val
        val = self.create_int_val(
            sensors.wind_sensor_2.iimwv.wind_reference)
        values["/wind_sensor_2/iimwv/wind_reference"] = val
        val = self.create_int_val(
            sensors.wind_sensor_2.wixdir.wind_temperature)
        values["/wind_sensor_2/wixdir/wind_temperature"] = val

        # Store gps sensor values
        val = self.create_str_val(sensors.gps_0.gprmc.utc_timestamp)
        values["/gps_0/gprmc/utc_timestamp"] = val
        val = self.create_float_val(sensors.gps_0.gprmc.latitude)
        values["/gps_0/gprmc/latitude"] = val
        val = self.create_float_val(sensors.gps_0.gprmc.longitude)
        values["/gps_0/gprmc/longitude"] = val
        val = self.create_bool_val(sensors.gps_0.gprmc.latitude_loc)
        values["/gps_0/gprmc/latitude_loc"] = val
        val = self.create_bool_val(sensors.gps_0.gprmc.longitude_loc)
        values["/gps_0/gprmc/longitude_loc"] = val
        val = self.create_int_val(sensors.gps_0.gprmc.ground_speed)
        values["/gps_0/gprmc/ground_speed"] = val
        val = self.create_int_val(sensors.gps_0.gprmc.track_made_good)
        values["/gps_0/gprmc/track_made_good"] = val
        val = self.create_int_val(sensors.gps_0.gprmc.magnetic_variation)
        values["/gps_0/gprmc/magnetic_variation"] = val
        val = self.create_bool_val(
            sensors.gps_0.gprmc.magnetic_variation_sense)
        values["/gps_0/gprmc/magnetic_variation_sense"] = val
        val = self.create_int_val(sensors.gps_0.gpgga.quality_indicator)
        values["/gps_0/gpgga/quality_indicator"] = val
        val = self.create_int_val(sensors.gps_0.gpgga.hdop)
        values["/gps_0/gpgga/hdop"] = val
        val = self.create_int_val(sensors.gps_0.gpgga.antenna_altitude)
        values["/gps_0/gpgga/antenna_altitude"] = val
        val = self.create_int_val(sensors.gps_0.gpgga.geoidal_separation)
        values["/gps_0/gpgga/geoidal_separation"] = val

        val = self.create_str_val(sensors.gps_0.gprmc.utc_timestamp)
        values["/gps_1/gprmc/utc_timestamp"] = val
        val = self.create_float_val(sensors.gps_1.gprmc.latitude)
        values["/gps_1/gprmc/latitude"] = val
        val = self.create_float_val(sensors.gps_1.gprmc.longitude)
        values["/gps_1/gprmc/longitude"] = val
        val = self.create_bool_val(sensors.gps_1.gprmc.latitude_loc)
        values["/gps_1/gprmc/latitude_loc"] = val
        val = self.create_bool_val(sensors.gps_1.gprmc.longitude_loc)
        values["/gps_1/gprmc/longitude_loc"] = val
        val = self.create_int_val(sensors.gps_1.gprmc.ground_speed)
        values["/gps_1/gprmc/ground_speed"] = val
        val = self.create_int_val(sensors.gps_1.gprmc.track_made_good)
        values["/gps_1/gprmc/track_made_good"] = val
        val = self.create_int_val(sensors.gps_1.gprmc.magnetic_variation)
        values["/gps_1/gprmc/magnetic_variation"] = val
        val = self.create_bool_val(
            sensors.gps_1.gprmc.magnetic_variation_sense)
        values["/gps_1/gprmc/magnetic_variation_sense"] = val
        val = self.create_int_val(sensors.gps_1.gpgga.quality_indicator)
        values["/gps_1/gpgga/quality_indicator"] = val
        val = self.create_int_val(sensors.gps_1.gpgga.hdop)
        values["/gps_1/gpgga/hdop"] = val
        val = self.create_int_val(sensors.gps_1.gpgga.antenna_altitude)
        values["/gps_1/gpgga/antenna_altitude"] = val
        val = self.create_int_val(sensors.gps_1.gpgga.geoidal_separation)
        values["/gps_1/gpgga/geoidal_separation"] = val

        # Store bms sensor values
        val = self.create_int_val(
            sensors.bms_0.battery_pack_data.current)
        values["/bms_0/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_0.battery_pack_data.total_voltage)
        values["/bms_0/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_0.battery_pack_data.temperature)
        values["/bms_0/battery_pack_data/temperature"] = val

        val = self.create_int_val(sensors.bms_1.battery_pack_data.current)
        values["/bms_1/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_1.battery_pack_data.total_voltage)
        values["/bms_1/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_1.battery_pack_data.temperature)
        values["/bms_1/battery_pack_data/temperature"] = val

        val = self.create_int_val(sensors.bms_2.battery_pack_data.current)
        values["/bms_2/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_2.battery_pack_data.total_voltage)
        values["/bms_2/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_2.battery_pack_data.temperature)
        values["/bms_2/battery_pack_data/temperature"] = val

        val = self.create_int_val(sensors.bms_3.battery_pack_data.current)
        values["/bms_3/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_3.battery_pack_data.total_voltage)
        values["/bms_3/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_3.battery_pack_data.temperature)
        values["/bms_3/battery_pack_data/temperature"] = val

        val = self.create_int_val(sensors.bms_4.battery_pack_data.current)
        values["/bms_4/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_4.battery_pack_data.total_voltage)
        values["/bms_4/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_4.battery_pack_data.temperature)
        values["/bms_4/battery_pack_data/temperature"] = val

        val = self.create_int_val(sensors.bms_5.battery_pack_data.current)
        values["/bms_5/battery_pack_data/current"] = val
        val = self.create_int_val(
            sensors.bms_5.battery_pack_data.total_voltage)
        values["/bms_5/battery_pack_data/total_voltage"] = val
        val = self.create_int_val(
            sensors.bms_5.battery_pack_data.temperature)
        values["/bms_5/battery_pack_data/temperature"] = val

        val = self.create_int_val(
            sensors.accelerometer.boat_orientation_data.x_axis_acceleration)
        values["/accelerometer/boat_orientation_data/x_axis_acceleration"] = val
        val = self.create_int_val(
            sensors.accelerometer.boat_orientation_data.y_axis_acceleration)
        values["/accelerometer/boat_orientation_data/y_axis_acceleration"] = val
        val = self.create_int_val(
            sensors.accelerometer.boat_orientation_data.z_axis_acceleration)
        values["/accelerometer/boat_orientation_data/z_axis_acceleration"] = val

        return values

    def uccms_to_root(self, uccms):
        values = {}

        # Store boom sensor uccm values
        val = self.create_int_val(uccms.boom_angle_sensor.current)
        values["/boom_angle_sensor/uccm/current"] = val
        val = self.create_int_val(uccms.boom_angle_sensor.voltage)
        values["/boom_angle_sensor/uccm/voltage"] = val
        val = self.create_int_val(uccms.boom_angle_sensor.temperature)
        values["/boom_angle_sensor/uccm/temperature"] = val
        val = self.create_str_val(uccms.boom_angle_sensor.status)
        values["/boom_angle_sensor/uccm/status"] = val

        # Store rudder motor uccm values
        val = self.create_int_val(uccms.rudder_motor_control_0.current)
        values["/rudder_motor_control_0/uccm/current"] = val
        val = self.create_int_val(uccms.rudder_motor_control_0.voltage)
        values["/rudder_motor_control_0/uccm/voltage"] = val
        val = self.create_int_val(uccms.rudder_motor_control_0.temperature)
        values["/rudder_motor_control_0/uccm/temperature"] = val
        val = self.create_str_val(uccms.rudder_motor_control_0.status)
        values["/rudder_motor_control_0/uccm/status"] = val

        val = self.create_int_val(uccms.rudder_motor_control_1.current)
        values["/rudder_motor_control_1/uccm/current"] = val
        val = self.create_int_val(uccms.rudder_motor_control_1.voltage)
        values["/rudder_motor_control_1/uccm/voltage"] = val
        val = self.create_int_val(uccms.rudder_motor_control_1.temperature)
        values["/rudder_motor_control_1/uccm/temperature"] = val
        val = self.create_str_val(uccms.rudder_motor_control_1.status)
        values["/rudder_motor_control_1/uccm/status"] = val

        # Store winch motor uccm values
        val = self.create_int_val(uccms.winch_motor_control_0.current)
        values["/winch_motor_control_0/uccm/current"] = val
        val = self.create_int_val(uccms.winch_motor_control_0.voltage)
        values["/winch_motor_control_0/uccm/voltage"] = val
        val = self.create_int_val(uccms.winch_motor_control_0.temperature)
        values["/winch_motor_control_0/uccm/temperature"] = val
        val = self.create_str_val(uccms.winch_motor_control_0.status)
        values["/winch_motor_control_0/uccm/status"] = val

        val = self.create_int_val(uccms.winch_motor_control_1.current)
        values["/winch_motor_control_1/uccm/current"] = val
        val = self.create_int_val(uccms.winch_motor_control_1.voltage)
        values["/winch_motor_control_1/uccm/voltage"] = val
        val = self.create_int_val(uccms.winch_motor_control_1.temperature)
        values["/winch_motor_control_1/uccm/temperature"] = val
        val = self.create_str_val(uccms.winch_motor_control_1.status)
        values["/winch_motor_control_1/uccm/status"] = val

        # Store wind sensor uccm values
        val = self.create_int_val(uccms.wind_sensor_0.current)
        values["/wind_sensor_0/uccm/current"] = val
        val = self.create_int_val(
            uccms.wind_sensor_0.voltage)
        values["/wind_sensor_0/uccm/voltage"] = val
        val = self.create_int_val(
            uccms.wind_sensor_0.temperature)
        values["/wind_sensor_0/uccm/termperature"] = val
        val = self.create_str_val(
            uccms.wind_sensor_0.status)
        values["/wind_sensor_0/uccm/status"] = val

        val = self.create_int_val(uccms.wind_sensor_1.current)
        values["/wind_sensor_1/uccm/current"] = val
        val = self.create_int_val(
            uccms.wind_sensor_1.voltage)
        values["/wind_sensor_1/uccm/voltage"] = val
        val = self.create_int_val(
            uccms.wind_sensor_1.temperature)
        values["/wind_sensor_1/uccm/termperature"] = val
        val = self.create_str_val(
            uccms.wind_sensor_1.status)
        values["/wind_sensor_1/uccm/status"] = val

        val = self.create_int_val(uccms.wind_sensor_2.current)
        values["/wind_sensor_2/uccm/current"] = val
        val = self.create_int_val(
            uccms.wind_sensor_2.voltage)
        values["/wind_sensor_2/uccm/voltage"] = val
        val = self.create_int_val(
            uccms.wind_sensor_2.temperature)
        values["/wind_sensor_2/uccm/termperature"] = val
        val = self.create_str_val(
            uccms.wind_sensor_2.status)
        values["/wind_sensor_2/uccm/status"] = val

        # Store gps uccm values
        val = self.create_int_val(uccms.gps_0.current)
        values["/gps_0/uccm/current"] = val
        val = self.create_int_val(uccms.gps_0.voltage)
        values["/gps_0/uccm/voltage"] = val
        val = self.create_int_val(uccms.gps_0.temperature)
        values["/gps_0/uccm/temperature"] = val
        val = self.create_str_val(uccms.gps_0.status)
        values["/gps_0/uccm/status"] = val

        val = self.create_int_val(uccms.gps_0.current)
        values["/gps_1/uccm/current"] = val
        val = self.create_int_val(uccms.gps_1.voltage)
        values["/gps_1/uccm/voltage"] = val
        val = self.create_int_val(uccms.gps_1.temperature)
        values["/gps_1/uccm/temperature"] = val
        val = self.create_str_val(uccms.gps_1.status)
        values["/gps_1/uccm/status"] = val

        # Store bms uccm values
        val = self.create_int_val(uccms.bms_0.current)
        values["/bms_0/uccm/current"] = val
        val = self.create_int_val(uccms.bms_0.voltage)
        values["/bms_0/uccm/total_voltage"] = val
        val = self.create_int_val(uccms.bms_0.temperature)
        values["/bms_0/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_0.status)
        values["/bms_0/uccm/status"] = val

        val = self.create_int_val(uccms.bms_0.current)
        values["/bms_1/uccm/current"] = val
        val = self.create_int_val(uccms.bms_1.voltage)
        values["/bms_1/uccm/voltage"] = val
        val = self.create_int_val(uccms.bms_1.temperature)
        values["/bms_1/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_1.status)
        values["/bms_1/uccm/status"] = val

        val = self.create_int_val(uccms.bms_1.current)
        values["/bms_2/uccm/current"] = val
        val = self.create_int_val(uccms.bms_2.voltage)
        values["/bms_2/uccm/voltage"] = val
        val = self.create_int_val(uccms.bms_2.temperature)
        values["/bms_2/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_2.status)
        values["/bms_2/uccm/status"] = val

        val = self.create_int_val(uccms.bms_3.current)
        values["/bms_3/uccm/current"] = val
        val = self.create_int_val(uccms.bms_3.voltage)
        values["/bms_3/uccm/voltage"] = val
        val = self.create_int_val(uccms.bms_3.temperature)
        values["/bms_3/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_3.status)
        values["/bms_3/uccm/status"] = val

        val = self.create_int_val(uccms.bms_3.current)
        values["/bms_4/uccm/current"] = val
        val = self.create_int_val(uccms.bms_4.voltage)
        values["/bms_4/uccm/voltage"] = val
        val = self.create_int_val(uccms.bms_4.temperature)
        values["/bms_4/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_4.status)
        values["/bms_4/uccm/status"] = val

        val = self.create_int_val(uccms.bms_5.current)
        values["/bms_5/uccm/current"] = val
        val = self.create_int_val(uccms.bms_5.voltage)
        values["/bms_5/uccm/voltage"] = val
        val = self.create_int_val(uccms.bms_5.temperature)
        values["/bms_5/uccm/temperature"] = val
        val = self.create_str_val(uccms.bms_5.status)
        values["/bms_5/uccm/status"] = val

        # Store accelerometer uccm values
        val = self.create_int_val(uccms.accelerometer.current)
        values["/accelerometer/uccm/current"] = val
        val = self.create_int_val(uccms.accelerometer.voltage)
        values["/accelerometer/uccm/voltage"] = val
        val = self.create_int_val(uccms.accelerometer.temperature)
        values["/accelerometer/uccm/temperature"] = val
        val = self.create_str_val(uccms.accelerometer.status)
        values["/accelerometer/uccm/status"] = val

        return values
