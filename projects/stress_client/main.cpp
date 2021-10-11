// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Help.h"
#include "Value.pb.h"
#include "Node.pb.h"
#include "Exceptions.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

std::atomic_bool gps_quality_callback_called(false);  // It would be better to
                                                      // check that it was called
                                                      // the correct number of times
                                                      // but this depends on what other
                                                      // clients are doing.
std::atomic_bool wrong_gps_quality_data_received(false);
void GpsQualityCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    gps_quality_callback_called = true;
    auto data = node.value().int_data();
    if (data != 3) {
        std::cout << "Received gps_0/gpgga/quality_indicator data: " << data << std::endl;
        std::cout << "Expected: " << 3 << std::endl;
        wrong_gps_quality_data_received = true;
    }
}

std::atomic_bool root_callback_called(false);
std::atomic_bool wrong_root_data_received(false);
void RootCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    root_callback_called = true;
    auto data = node.children().at("gps_0").children().at("gpgga")\
            .children().at("quality_indicator").value().int_data();
    if (data != 3) {
        std::cout << "(root) Received gps_0/gpgga/quality_indicator data: " << data << std::endl;
        std::cout << "Expected: " << 3 << std::endl;
        wrong_root_data_received = true;
    }
}

/*
 * This is a stress test
 * for the network table server.
 * This program will return 0 if no tests failed,
 * otherwise it will return 1.
 */
int main() {
    int any_test_failed = 0;
    int num_queries = 5;  // How many times the set of tests is run.

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

try_subscribe_quality_indicator:
    try {
        connection.Subscribe("gps_0/gpgga/quality_indicator", &GpsQualityCallback);
    } catch (NetworkTable::TimeoutException) {
            goto try_subscribe_quality_indicator;
    }

    // Subscribe to root of tree.
    // Then, when something deep in the tree gets updated, we should
    // receive the update
try_subscribe_root:
    try {
        connection.Subscribe("/", &RootCallback);
    } catch (NetworkTable::TimeoutException) {
        goto try_subscribe_root;
    }

    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%2000));  // NOLINT(runtime/threadsafe_fn)

    // SET gps (this must be first or the callbacks will fail with
    // key not found error)
    try {
        for (int i = 0; i < 2; i++) {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value uccm_current;
            uccm_current.set_type(NetworkTable::Value::INT);
            uccm_current.set_int_data(14);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/uccm/current", uccm_current));

            NetworkTable::Value uccm_voltage;
            uccm_voltage.set_type(NetworkTable::Value::INT);
            uccm_voltage.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/uccm/voltage", uccm_voltage));

            NetworkTable::Value uccm_temperature;
            uccm_temperature.set_type(NetworkTable::Value::INT);
            uccm_temperature.set_int_data(40);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/uccm/temperature", uccm_temperature));

            NetworkTable::Value uccm_status;
            uccm_status.set_type(NetworkTable::Value::STRING);
            uccm_status.set_string_data("ON");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/uccm/status", uccm_status));

            NetworkTable::Value utc_timestamp;
            utc_timestamp.set_type(NetworkTable::Value::STRING);
            utc_timestamp.set_string_data("02/06/19, 22:45:32.54");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/utc_timestamp", utc_timestamp));

            NetworkTable::Value latitude;
            latitude.set_type(NetworkTable::Value::FLOAT);
            latitude.set_float_data(11.33214);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/latitude", latitude));

            NetworkTable::Value longitude;
            longitude.set_type(NetworkTable::Value::FLOAT);
            longitude.set_float_data(-105.451);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/longitude", longitude));

            NetworkTable::Value latitude_loc;
            latitude_loc.set_type(NetworkTable::Value::INT);
            latitude_loc.set_int_data(22);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/latitude_loc", latitude_loc));

            NetworkTable::Value longitude_loc;
            longitude_loc.set_type(NetworkTable::Value::INT);
            longitude_loc.set_int_data(-21);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/longitude_loc", longitude_loc));

            NetworkTable::Value ground_speed;
            ground_speed.set_type(NetworkTable::Value::INT);
            ground_speed.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/ground_speed", ground_speed));

            NetworkTable::Value track_made_good;
            track_made_good.set_type(NetworkTable::Value::INT);
            track_made_good.set_int_data(1);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/track_made_good", track_made_good));

            NetworkTable::Value magnetic_variation;
            magnetic_variation.set_type(NetworkTable::Value::INT);
            magnetic_variation.set_int_data(1);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/magnetic_variation", magnetic_variation));

            NetworkTable::Value magnetic_variation_sense;
            magnetic_variation_sense.set_type(NetworkTable::Value::BOOL);
            magnetic_variation_sense.set_bool_data(true);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gprmc/magnetic_variation_sense", magnetic_variation_sense));

            NetworkTable::Value quality_indicator;
            quality_indicator.set_type(NetworkTable::Value::INT);
            quality_indicator.set_int_data(3);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gpgga/quality_indicator", quality_indicator));

            NetworkTable::Value hdop;
            hdop.set_type(NetworkTable::Value::INT);
            hdop.set_int_data(11);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gpgga/hdop", hdop));

            NetworkTable::Value antenna_altitude;
            antenna_altitude.set_type(NetworkTable::Value::INT);
            antenna_altitude.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gpgga/antenna_altitude", antenna_altitude));

            NetworkTable::Value geoidal_separation;
            geoidal_separation.set_type(NetworkTable::Value::INT);
            geoidal_separation.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("gps_"+std::to_string(i)+"/gpgga/geoidal_separation", geoidal_separation));

            connection.SetValues(values);
        }
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting gps" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)


    // SET boom angle sensor
    try {
        std::map<std::string, NetworkTable::Value> values;

        NetworkTable::Value uccm_current;
        uccm_current.set_type(NetworkTable::Value::INT);
        uccm_current.set_int_data(12);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("boom_angle_sensor/uccm/current", uccm_current));

        NetworkTable::Value uccm_voltage;
        uccm_voltage.set_type(NetworkTable::Value::INT);
        uccm_voltage.set_int_data(6);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("boom_angle_sensor/uccm/voltage", uccm_voltage));

        NetworkTable::Value uccm_temperature;
        uccm_temperature.set_type(NetworkTable::Value::INT);
        uccm_temperature.set_int_data(32);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("boom_angle_sensor/uccm/temperature", uccm_temperature));

        NetworkTable::Value uccm_status;
        uccm_status.set_type(NetworkTable::Value::STRING);
        uccm_status.set_string_data("ON");
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("boom_angle_sensor/uccm/status", uccm_status));

        NetworkTable::Value angle;
        angle.set_type(NetworkTable::Value::INT);
        angle.set_int_data(2);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("boom_angle_sensor/sensor_data/angle", angle));


        connection.SetValues(values);
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting boom_angle_sensor" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    // SET rudder motor control
    try {
        for (int i = 0; i < 2; i++) {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value uccm_current;
            uccm_current.set_type(NetworkTable::Value::INT);
            uccm_current.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("rudder_motor_control_"+std::to_string(i)+"/uccm/current", uccm_current));

            NetworkTable::Value uccm_voltage;
            uccm_voltage.set_type(NetworkTable::Value::INT);
            uccm_voltage.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("rudder_motor_control_"+std::to_string(i)+"/uccm/voltage", uccm_voltage));

            NetworkTable::Value uccm_temperature;
            uccm_temperature.set_type(NetworkTable::Value::INT);
            uccm_temperature.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("rudder_motor_control_"+std::to_string(i)+"/uccm/temperature", uccm_temperature));

            NetworkTable::Value uccm_status;
            uccm_status.set_type(NetworkTable::Value::STRING);
            uccm_status.set_string_data("ON");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("rudder_motor_control_"+std::to_string(i)+"/uccm/status", uccm_status));

            connection.SetValues(values);
        }
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting rudder_motor_control" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    // SET winch motor control
    try {
        for (int i = 0; i < 2; i++) {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value uccm_current;
            uccm_current.set_type(NetworkTable::Value::INT);
            uccm_current.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("winch_motor_control_"+std::to_string(i)+"/uccm/current", uccm_current));

            NetworkTable::Value uccm_voltage;
            uccm_voltage.set_type(NetworkTable::Value::INT);
            uccm_voltage.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("winch_motor_control_"+std::to_string(i)+"/uccm/voltage", uccm_voltage));

            NetworkTable::Value uccm_temperature;
            uccm_temperature.set_type(NetworkTable::Value::INT);
            uccm_temperature.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("winch_motor_control_"+std::to_string(i)+"/uccm/temperature", uccm_temperature));

            NetworkTable::Value uccm_status;
            uccm_status.set_type(NetworkTable::Value::STRING);
            uccm_status.set_string_data("ON");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("winch_motor_control_"+std::to_string(i)+"/uccm/status", uccm_status));

            connection.SetValues(values);
        }
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting winch_motor_control" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    // SET wind sensor
    try {
        for (int i = 0; i < 3; i++) {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value uccm_current;
            uccm_current.set_type(NetworkTable::Value::INT);
            uccm_current.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/uccm/current", uccm_current));

            NetworkTable::Value uccm_voltage;
            uccm_voltage.set_type(NetworkTable::Value::INT);
            uccm_voltage.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/uccm/voltage", uccm_voltage));

            NetworkTable::Value uccm_temperature;
            uccm_temperature.set_type(NetworkTable::Value::INT);
            uccm_temperature.set_int_data(40);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/uccm/temperature", uccm_temperature));

            NetworkTable::Value uccm_status;
            uccm_status.set_type(NetworkTable::Value::STRING);
            uccm_status.set_string_data("ON");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/uccm/status", uccm_status));

            NetworkTable::Value wind_speed;
            wind_speed.set_type(NetworkTable::Value::INT);
            wind_speed.set_int_data(11);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/iimwv/wind_speed", wind_speed));

            NetworkTable::Value wind_direction;
            wind_direction.set_type(NetworkTable::Value::INT);
            wind_direction.set_int_data(12);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/iimwv/wind_direction", wind_direction));

            NetworkTable::Value wind_reference;
            wind_reference.set_type(NetworkTable::Value::INT);
            wind_reference.set_int_data(3);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/iimwv/wind_reference", wind_reference));

            NetworkTable::Value wind_temperature;
            wind_temperature.set_type(NetworkTable::Value::INT);
            wind_temperature.set_int_data(3);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("wind_sensor_"+std::to_string(i)+"/wixdir/wind_temperature", wind_temperature));

            connection.SetValues(values);
        }
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting wind_sensor" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    // SET bms
    try {
        for (int i = 0; i < 6; i++) {
            std::map<std::string, NetworkTable::Value> values;

            NetworkTable::Value uccm_current;
            uccm_current.set_type(NetworkTable::Value::INT);
            uccm_current.set_int_data(4);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/uccm/current", uccm_current));

            NetworkTable::Value uccm_voltage;
            uccm_voltage.set_type(NetworkTable::Value::INT);
            uccm_voltage.set_int_data(5);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/uccm/voltage", uccm_voltage));

            NetworkTable::Value uccm_temperature;
            uccm_temperature.set_type(NetworkTable::Value::INT);
            uccm_temperature.set_int_data(2);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/uccm/temperature", uccm_temperature));

            NetworkTable::Value uccm_status;
            uccm_status.set_type(NetworkTable::Value::STRING);
            uccm_status.set_string_data("ON");
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/uccm/status", uccm_status));

            NetworkTable::Value current;
            current.set_type(NetworkTable::Value::INT);
            current.set_int_data(2);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/battery_pack_data/current", current));

            NetworkTable::Value total_voltage;
            total_voltage.set_type(NetworkTable::Value::INT);
            total_voltage.set_int_data(2);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/battery_pack_data/total_voltage", total_voltage));

            NetworkTable::Value temperature;
            temperature.set_type(NetworkTable::Value::INT);
            temperature.set_int_data(2);
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("bms_"+std::to_string(i)+"/battery_pack_data/temperature", temperature));

            connection.SetValues(values);
        }
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting bms" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    // SET accelerometer
    try {
        std::map<std::string, NetworkTable::Value> values;

        NetworkTable::Value uccm_current;
        uccm_current.set_type(NetworkTable::Value::INT);
        uccm_current.set_int_data(4);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/uccm/current", uccm_current));

        NetworkTable::Value uccm_voltage;
        uccm_voltage.set_type(NetworkTable::Value::INT);
        uccm_voltage.set_int_data(5);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/uccm/voltage", uccm_voltage));

        NetworkTable::Value uccm_temperature;
        uccm_temperature.set_type(NetworkTable::Value::INT);
        uccm_temperature.set_int_data(2);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/uccm/temperature", uccm_temperature));

        NetworkTable::Value uccm_status;
        uccm_status.set_type(NetworkTable::Value::STRING);
        uccm_status.set_string_data("ON");
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/uccm/status", uccm_status));

        NetworkTable::Value x_axis_acceleration;
        x_axis_acceleration.set_type(NetworkTable::Value::INT);
        x_axis_acceleration.set_int_data(2);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/boat_orientation_data/x_axis_acceleration", x_axis_acceleration));

        NetworkTable::Value y_axis_acceleration;
        y_axis_acceleration.set_type(NetworkTable::Value::INT);
        y_axis_acceleration.set_int_data(2);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/boat_orientation_data/y_axis_acceleration", y_axis_acceleration));

        NetworkTable::Value z_axis_acceleration;
        z_axis_acceleration.set_type(NetworkTable::Value::INT);
        z_axis_acceleration.set_int_data(2);
        values.insert(std::pair<std::string, NetworkTable::Value>\
                ("accelerometer/boat_orientation_data/z_axis_acceleration", z_axis_acceleration));

        connection.SetValues(values);
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting accelerometer" << std::endl;
        any_test_failed = 1;
    }
    std::this_thread::sleep_for(\
            std::chrono::milliseconds(rand()%250));  // NOLINT(runtime/threadsafe_fn)

    for (int i = 0; i < num_queries; i++) {
        // GET
        try {
            NetworkTable::Value value = connection.GetValue("gps_0/gpgga/quality_indicator");
            if (value.int_data() != 3) {
                std::cout << "Wrong gps_0/gpgga/quality_indicator data" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting gps_0/gpgga/quality_indicator" << std::endl;
            any_test_failed = 1;
        }
        // GET garbage
        try {
            // This should throw NodeNotFoundException
            NetworkTable::Value value = connection.GetValue("garbage");
            std::cout << "Got garbage without throwing exception" << std::endl;
            any_test_failed = 1;
        } catch (NetworkTable::TimeoutException) {
        } catch (NetworkTable::NodeNotFoundException) {
        } catch (...) {
            std::cout << "Error getting garbage" << std::endl;
            any_test_failed = 1;
        }
        // SET
        /*
         * If we were the only people running, then
         * our messages should arrive in order.
         * So we just set the boom angle sensor uccm voltage to a value,
         * and then check and see that its that same value when we get it back?
         *
         * Unfortunately, when running the stress test
         * this is not the case because many other processes are running,
         * and they may still be on the code farther above.
         * They might overwrite SetValues and instead set it to something else.
         * So for this part of the test, we use completely new
         * uris that are only ever set to a single value.
         */
        NetworkTable::Value dummy_sensor_uccm_voltage;
        dummy_sensor_uccm_voltage.set_type(NetworkTable::Value::INT);
        dummy_sensor_uccm_voltage.set_int_data(25);
        NetworkTable::Value dummy_sensor_uccm_current;
        dummy_sensor_uccm_current.set_type(NetworkTable::Value::INT);
        dummy_sensor_uccm_current.set_int_data(4);


        /*
         * Use this bool to keep trying to set dummy data until
         * we are guaranteed its in the network table.
         * Or else, you can get bad timing where you timeout,
         * don't set dummy data, and then try to get it.
         */
        bool dummy_data_set = false;
        while (!dummy_data_set) {
            try {
                std::map<std::string, NetworkTable::Value> values;
                values.insert(std::pair<std::string, NetworkTable::Value>(\
                            "dummy_sensor/uccm/voltage", dummy_sensor_uccm_voltage));
                values.insert(std::pair<std::string, NetworkTable::Value>(\
                            "dummy_sensor/uccm/current", dummy_sensor_uccm_current));

                connection.SetValues(values);
                dummy_data_set = true;
            } catch (NetworkTable::TimeoutException) {
            } catch (...) {
                std::cout << "Error setting dummy_sensor/uccm" << std::endl;
                any_test_failed = 1;
            }
        }

        // GET
        std::map<std::string, NetworkTable::Value> values;
        try {
            std::set<std::string> keys;
            keys.insert("dummy_sensor/uccm/voltage");
            keys.insert("dummy_sensor/uccm/current");

            values = connection.GetValues(keys);
            if (values["dummy_sensor/uccm/voltage"].int_data()\
                            != dummy_sensor_uccm_voltage.int_data()) {
                std::cout << "Error, wrong value for dummy_sensor/uccm/voltage" << std::endl;
                std::cout << " expected: " << dummy_sensor_uccm_voltage.int_data()\
                          << " got: " << values["dummy_sensor/uccm/voltage"].int_data() << std::endl;
                any_test_failed = 1;
            }
            if (values["dummy_sensor/uccm/current"].int_data()\
                            != dummy_sensor_uccm_current.int_data()) {
                std::cout << "Error, wrong value for dummy_sensor/uccm/current" << std::endl;
                std::cout << " expected: " << dummy_sensor_uccm_current.int_data()\
                          << " got: " << values["dummy_sensor/uccm/current"].int_data() << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting dummy_sensor/uccm" << std::endl;
            for (const auto &pair : values) {
                std::cout << pair.first << std::endl;
            }
            any_test_failed = 1;
        }
        // GET the whole tree
        NetworkTable::Node root;
        try {
            root = connection.GetNode("/");
            int received_voltage = root.children().at("dummy_sensor").children().at("uccm")\
                            .children().at("voltage").value().int_data();
            if (received_voltage != dummy_sensor_uccm_voltage.int_data()) {
                std::cout << "Error, dummy_sensor/uccm/voltage was wrong when getting whole tree" << std::endl;
                std::cout << " expected: " << dummy_sensor_uccm_voltage.int_data()\
                          << " got: " << received_voltage << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting whole tree: " << std::endl;
            NetworkTable::PrintNode(root);
            any_test_failed = 1;
        }
    }

    // Check that everything went OK with the callbacks
    if (!gps_quality_callback_called) {
        std::cout << "Error, gps quality callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (wrong_gps_quality_data_received && gps_quality_callback_called) {
        std::cout << "Wrong gps quality data received" << std::endl;
        any_test_failed = 1;
    }
    if (!root_callback_called) {
        std::cout << "Error, root callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (wrong_root_data_received && root_callback_called) {
        std::cout << "Error, wrong root data recieved" << std::endl;
        any_test_failed = 1;
    }

    connection.Disconnect();
    return any_test_failed;
}

