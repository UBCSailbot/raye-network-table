/**
*
*  Copyright 2017 UBC Sailbot
*
*  @file  client/main.cpp
*  @brief Continuously populates the network table with
*         random mock client data
*
*  Currently only populates the sensor fields
*  TODO(brielle): Populate uccm fields after getting clarity
*                 on how we are receiving the data from CANbus
*
*  @author Brielle Law (briellelaw)
*
*/

#include "Connection.h"
#include "Help.h"
#include "Value.pb.h"
#include "Node.pb.h"
#include "Exceptions.h"
#include "Uri.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <cfloat>
#include <limits>

#define MAX_FLOAT_DATA 360.00
#define MAX_INT_DATA   360

float getRandFloat() {
    float float_rand = \
        static_cast <float> (rand())/ static_cast <float> (RAND_MAX/MAX_FLOAT_DATA);  // NOLINT(runtime/threadsafe_fn)

    return float_rand;
}

float getRandInt() {
    int int_rand = rand() % MAX_INT_DATA;  // NOLINT(runtime/threadsafe_fn)
    return int_rand;
}

float getRandInt(int min, int max) {
    int int_rand = rand() % (max-min + 1) + min;  // NOLINT(runtime/threadsafe_fn)
    return int_rand;
}

int main() {
    NetworkTable::Connection connection;
    try {
        connection.Connect(5000);
    } catch (NetworkTable::InterruptedException) {
        std::cout << "Received interrupt signal" << std::endl;
        return 0;
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out" << std::endl;
        return 0;
    }

    // initialize random seed:
    srand(time(NULL));

    while (true) {
        int int_rand = getRandInt();
        float float_rand = getRandFloat();

        try {
            std::map<std::string, NetworkTable::Value> values;

            // Set GPS_CAN values
            NetworkTable::Value latitude;
            latitude.set_type(NetworkTable::Value::FLOAT);
            latitude.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_LAT, latitude));

            NetworkTable::Value longitude;
            longitude.set_type(NetworkTable::Value::FLOAT);
            longitude.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_LON, longitude));

            NetworkTable::Value ground_speed;
            ground_speed.set_type(NetworkTable::Value::INT);
            ground_speed.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_GNDSPEED, ground_speed));

            NetworkTable::Value track_made_good;
            track_made_good.set_type(NetworkTable::Value::INT);
            track_made_good.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_TMG, track_made_good));

            NetworkTable::Value magnetic_variation;
            magnetic_variation.set_type(NetworkTable::Value::INT);
            magnetic_variation.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_MAGVAR, magnetic_variation));

            NetworkTable::Value utc_timestamp;
            utc_timestamp.set_type(NetworkTable::Value::STRING);
            utc_timestamp.set_string_data("02/06/19, 22:45:32.54");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_TIME, utc_timestamp));

            int_rand = getRandInt(0, 1);

            NetworkTable::Value valid;
            valid.set_type(NetworkTable::Value::INT);
            valid.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_VALID, valid));

            NetworkTable::Value var_west;
            var_west.set_type(NetworkTable::Value::INT);
            var_west.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_VARWEST, var_west));

            NetworkTable::Value lat_north;
            lat_north.set_type(NetworkTable::Value::INT);
            lat_north.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_VARWEST, lat_north));

            NetworkTable::Value lon_west;
            lon_west.set_type(NetworkTable::Value::INT);
            lon_west.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_CAN_LONWEST, lon_west));

            // Set GPS_AIS values
            latitude.set_type(NetworkTable::Value::FLOAT);
            latitude.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_AIS_LAT, latitude));

            longitude.set_type(NetworkTable::Value::FLOAT);
            longitude.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GPS_AIS_LON, longitude));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting gps" << std::endl;
        }

        // SET sailencoder
        int_rand = getRandInt();
        try {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value angle;
            angle.set_type(NetworkTable::Value::INT);
            angle.set_int_data(int_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (SAILENCODER_ANGLE, angle));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting boom_angle_sensor" << std::endl;
        }

        // SET rudder motor control
        float_rand = getRandFloat();
        try {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value rudder_angle;
            rudder_angle.set_type(NetworkTable::Value::FLOAT);
            rudder_angle.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (RUDDER_PORT_ANGLE, rudder_angle));
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (RUDDER_STBD_ANGLE, rudder_angle));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting rudder_motor_control" << std::endl;
        }

        // SET winch motor control
        float_rand = getRandFloat();
        try {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value winch_angle;
            winch_angle.set_type(NetworkTable::Value::FLOAT);
            winch_angle.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (WINCH_MAIN_ANGLE, winch_angle));
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (WINCH_JIB_ANGLE, winch_angle));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting winch_motor_control" << std::endl;
        }

        // SET wind sensor
        try {
            for (int i = 1; i <= 3; i++) {
                std::map<std::string, NetworkTable::Value> values;

                float_rand = getRandFloat();
                NetworkTable::Value wind_speed;
                wind_speed.set_type(NetworkTable::Value::FLOAT);
                wind_speed.set_float_data(float_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("wind_sensor_"+std::to_string(i)+WIND_SPEED, wind_speed));

                float_rand = getRandFloat();
                NetworkTable::Value wind_angle;
                wind_angle.set_type(NetworkTable::Value::INT);
                wind_angle.set_float_data(float_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("wind_sensor_"+std::to_string(i)+WIND_ANGLE, wind_angle));

                connection.SetValues(values);
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting wind_sensor" << std::endl;
        }

        // SET bms
        try {
            for (int i = 1; i <= 6; i++) {
                std::map<std::string, NetworkTable::Value> values;

                int_rand = getRandInt();

                NetworkTable::Value current;
                current.set_type(NetworkTable::Value::INT);
                current.set_int_data(int_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms_"+std::to_string(i)+BMS_CURRENT, current));

                NetworkTable::Value total_voltage;
                total_voltage.set_type(NetworkTable::Value::INT);
                total_voltage.set_int_data(int_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms_"+std::to_string(i)+BMS_VOLTAGE, total_voltage));

                NetworkTable::Value maxcell;
                maxcell.set_type(NetworkTable::Value::INT);
                maxcell.set_int_data(int_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms_"+std::to_string(i)+BMS_MAXCELL, maxcell));

                NetworkTable::Value mincell;
                mincell.set_type(NetworkTable::Value::INT);
                mincell.set_int_data(int_rand);
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        ("bms_"+std::to_string(i)+BMS_MINCELL, mincell));

                connection.SetValues(values);
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting bms" << std::endl;
        }

        // SET accelerometer
        try {
            std::map<std::string, NetworkTable::Value> values;

            float_rand = getRandFloat();

            NetworkTable::Value x_axis_acceleration;
            x_axis_acceleration.set_type(NetworkTable::Value::FLOAT);
            x_axis_acceleration.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (ACCELEROMETER_X, x_axis_acceleration));

            NetworkTable::Value y_axis_acceleration;
            y_axis_acceleration.set_type(NetworkTable::Value::FLOAT);
            y_axis_acceleration.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (ACCELEROMETER_Y, y_axis_acceleration));

            NetworkTable::Value z_axis_acceleration;
            z_axis_acceleration.set_type(NetworkTable::Value::FLOAT);
            z_axis_acceleration.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (ACCELEROMETER_Z, z_axis_acceleration));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting accelerometer" << std::endl;
        }

        // SET gyroscope
        try {
            std::map<std::string, NetworkTable::Value> values;

            float_rand = getRandFloat();

            NetworkTable::Value x_axis_velocity;
            x_axis_velocity.set_type(NetworkTable::Value::FLOAT);
            x_axis_velocity.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GYROSCOPE_X, x_axis_velocity));

            NetworkTable::Value y_axis_velocity;
            y_axis_velocity.set_type(NetworkTable::Value::FLOAT);
            y_axis_velocity.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GYROSCOPE_Y, y_axis_velocity));

            NetworkTable::Value z_axis_velocity;
            z_axis_velocity.set_type(NetworkTable::Value::FLOAT);
            z_axis_velocity.set_float_data(float_rand);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    (GYROSCOPE_Z, z_axis_velocity));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting accelerometer" << std::endl;
        }
        std::this_thread::sleep_for(\
                std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)
    }

    connection.Disconnect();
}

