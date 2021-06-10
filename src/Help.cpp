// Copyright 2017 UBC Sailbot

#include "Help.h"
#include "Exceptions.h"
#include "Uri.h"

#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdio>

void PrintTree(NetworkTable::Node root, int depth);

void NetworkTable::PrintNode(const NetworkTable::Node &root) {
    ::PrintTree(root, 0);
    std::cout << std::endl;
}

// Helper function
void PrintTree(NetworkTable::Node root, int depth) {
    auto children = root.children();

    // If root is a leaf, just print out the data.
    if (children.size() == 0) {
        std::cout << '\t';
        NetworkTable::Value value = root.value();
        switch (value.type()) {
            case NetworkTable::Value::INT : std::cout << value.int_data();
                                            break;
            case NetworkTable::Value::BOOL : std::cout << value.bool_data();
                                            break;
            case NetworkTable::Value::FLOAT : std::cout << value.float_data();
                                            break;
            case NetworkTable::Value::STRING : std::cout << value.string_data();
                                            break;
            case NetworkTable::Value::BYTES : std::cout << value.bytes_data();
                                            break;
            case NetworkTable::Value::BOATS :
                for (int i = 0; i < value.boats_size(); i++) {
                    auto boat = value.boats(i);
                    std::cout << " mmsi: " << boat.m_mmsi();
                    std::cout << " latitude: " << boat.m_latitude();
                    std::cout << " longitude: " << boat.m_longitude();
                    std::cout << " | ";
                }
                break;
            case NetworkTable::Value::WAYPOINTS :
                for (int i = 0; i < value.waypoints_size(); i++) {
                    auto waypoint = value.waypoints(i);
                    std::cout << " latitude: " << waypoint.latitude();
                    std::cout << " longtitude: " << waypoint.longitude();
                    std::cout << " | ";
                }
                break;

            default: std::cout << "Unknown type\n";
        }
    } else {
        // Otherwise, recursively print out the children
        for (auto it = children.begin(); it != children.end(); ++it) {
            std::cout << std::endl;
            for (int i = 0; i < depth; i++) {
                std::cout << '\t';
            }
            std::cout << it->first;
            PrintTree(it->second, depth + 1);
        }
    }
}

NetworkTable::Node NetworkTable::GetNode(std::string uri, NetworkTable::Node *root) {
    if (uri == "/" || uri == "") {
        return *root;
    }

    // Get each "slice" of the uri. eg "/gps/lat"
    // becomes {"gps", "lat"}:
    boost::trim_left_if(uri, boost::is_any_of("/"));
    std::vector<std::string> slices;
    boost::split(slices, uri, boost::is_any_of("/"));

    NetworkTable::Node *current_node = root;
    for (unsigned int i = 0; i < slices.size(); i++) {
        if (current_node->children().find(slices[i]) != current_node->children().end()) {
            current_node = &(current_node->mutable_children()->at(slices[i]));
        } else {
            throw NetworkTable::NodeNotFoundException("Could not find: " + uri);
        }
    }

    return *current_node;
}

void NetworkTable::SetNode(std::string uri, NetworkTable::Value value, NetworkTable::Node *root) {
    // Get each "slice" of the uri. eg "/gps/lat/"
    // becomes {"gps", "lat"}:
    // Note: trailing left and right uris are removed
    boost::trim_left_if(uri, boost::is_any_of("/"));
    boost::trim_right_if(uri, boost::is_any_of("/"));
    std::vector<std::string> slices;
    boost::split(slices, uri, boost::is_any_of("/"));

    NetworkTable::Node *current_node = root;
    for (unsigned int i = 0; i < slices.size(); i++) {
        std::string slice = slices[i];
        auto children = current_node->mutable_children();

        current_node = &(*children)[slice];
    }

    current_node->set_allocated_value(new NetworkTable::Value(value));
}

void NetworkTable::Write(std::string filepath, const NetworkTable::Node &root) {
    /*
     * Instead of writing to the actual file,
     * write to a swap file.
     * After that, delete the old file,
     * then rename the .swp file to the
     * proper filename.
     * This is to help prevent corrupting the file
     * in case of a crash.
     */
    std::string swapfile(filepath + ".swp");

    std::ofstream ofs(swapfile);
    ofs << root.SerializeAsString();
    ofs.close();

    std::remove(filepath.c_str());
    std::rename(swapfile.c_str(), filepath.c_str());
}

NetworkTable::Node NetworkTable::Load(const std::string &filepath) {
    NetworkTable::Node root;
    // http://insanecoding.blogspot.ca/2011/11/how-to-read-in-file-in-c.html
    std::ifstream input_filestream(filepath);
    std::ostringstream contents;
    if (input_filestream) {
        contents << input_filestream.rdbuf();
        root.ParseFromString(contents.str());
    } else {
        throw(errno);
    }
    return root;
}

NetworkTable::Sensors NetworkTable::RootToSensors(NetworkTable::Node *root) {
    /*
     * Have to do this all by hand :(((
     * Use copy/paste and search/replace
     */
    NetworkTable::Sensors sensors;

    try {
        sensors.mutable_boom_angle_sensor()->mutable_sensor_data()->set_angle(\
                GetNode(SAILENCODER_ANGLE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_wind_sensor_1()->mutable_iimwv()->set_wind_speed(\
                GetNode(WIND1_SPEED, root).value().int_data());
        sensors.mutable_wind_sensor_1()->mutable_iimwv()->set_wind_direction(\
                GetNode(WIND1_ANGLE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_wind_sensor_2()->mutable_iimwv()->set_wind_speed(\
                GetNode(WIND2_SPEED, root).value().int_data());
        sensors.mutable_wind_sensor_2()->mutable_iimwv()->set_wind_direction(\
                GetNode(WIND2_ANGLE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_wind_sensor_3()->mutable_iimwv()->set_wind_speed(\
                GetNode(WIND3_SPEED, root).value().int_data());
        sensors.mutable_wind_sensor_3()->mutable_iimwv()->set_wind_direction(\
                GetNode(WIND3_ANGLE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_gps_can()->mutable_gprmc()->set_utc_timestamp(\
                GetNode(GPS_CAN_TIME, root).value().string_data());
        sensors.mutable_gps_can()->mutable_gprmc()->set_latitude(\
                GetNode(GPS_CAN_LAT, root).value().float_data());
        sensors.mutable_gps_can()->mutable_gprmc()->set_longitude(\
                GetNode(GPS_CAN_LON, root).value().float_data());
        sensors.mutable_gps_can()->mutable_gprmc()->set_ground_speed(\
                GetNode(GPS_CAN_GNDSPEED, root).value().int_data());
        sensors.mutable_gps_can()->mutable_gprmc()->set_track_made_good(\
                GetNode(GPS_CAN_TMG, root).value().int_data());
        sensors.mutable_gps_can()->mutable_gprmc()->set_magnetic_variation(\
                    GetNode(GPS_CAN_MAGVAR, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_gps_ais()->mutable_gprmc()->set_utc_timestamp(\
                GetNode(GPS_AIS_TIME, root).value().string_data());
        sensors.mutable_gps_ais()->mutable_gprmc()->set_latitude(\
                GetNode(GPS_AIS_LAT, root).value().int_data());
        sensors.mutable_gps_ais()->mutable_gprmc()->set_longitude(\
                GetNode(GPS_AIS_LON, root).value().int_data());
        sensors.mutable_gps_ais()->mutable_gprmc()->set_ground_speed(\
                GetNode(GPS_AIS_GNDSPEED, root).value().int_data());
        sensors.mutable_gps_ais()->mutable_gprmc()->set_track_made_good(\
                GetNode(GPS_AIS_TMG, root).value().int_data());
        sensors.mutable_gps_ais()->mutable_gprmc()->set_magnetic_variation(\
                GetNode(GPS_AIS_MAGVAR, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_1()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS1_CURRENT, root).value().int_data());
        sensors.mutable_bms_1()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS1_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_2()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS2_CURRENT, root).value().int_data());
        sensors.mutable_bms_2()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS2_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_3()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS3_CURRENT, root).value().int_data());
        sensors.mutable_bms_3()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS3_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_4()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS4_CURRENT, root).value().int_data());
        sensors.mutable_bms_4()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS4_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_5()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS5_CURRENT, root).value().int_data());
        sensors.mutable_bms_5()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS5_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_bms_6()->mutable_battery_pack_data()->set_current(\
                GetNode(BMS6_CURRENT, root).value().int_data());
        sensors.mutable_bms_6()->mutable_battery_pack_data()->set_total_voltage(\
                GetNode(BMS6_VOLTAGE, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_x_axis_acceleration(\
                GetNode(ACCELEROMETER_X, root).value().int_data());
        sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_y_axis_acceleration(\
                GetNode(ACCELEROMETER_Y, root).value().int_data());
        sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_z_axis_acceleration(\
                GetNode(ACCELEROMETER_Z, root).value().int_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    return sensors;
}

NetworkTable::Node NetworkTable::SensorsToRoot(const NetworkTable::Sensors &sensors) {
    /*
     * Have to do this all by hand :(((
     * Use copy/paste and search/replace
     */
    NetworkTable::Node root;
    NetworkTable::Value val;

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.boom_angle_sensor().sensor_data().angle());
        SetNode(SAILENCODER_ANGLE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_1().iimwv().wind_speed());
        SetNode(WIND1_SPEED, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_1().iimwv().wind_direction());
        SetNode(WIND1_ANGLE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_2().iimwv().wind_speed());
        SetNode(WIND2_SPEED, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_2().iimwv().wind_direction());
        SetNode(WIND2_ANGLE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_3().iimwv().wind_speed());
        SetNode(WIND3_SPEED, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.wind_sensor_3().iimwv().wind_direction());
        SetNode(WIND3_ANGLE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(sensors.gps_can().gprmc().utc_timestamp());
        SetNode(GPS_CAN_TIME, val, &root);
        val.set_type(NetworkTable::Value::FLOAT);
        val.set_float_data(sensors.gps_can().gprmc().latitude());
        SetNode(GPS_CAN_LAT, val, &root);
        val.set_type(NetworkTable::Value::FLOAT);
        val.set_float_data(sensors.gps_can().gprmc().longitude());
        SetNode(GPS_CAN_LON, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_can().gprmc().ground_speed());
        SetNode(GPS_CAN_GNDSPEED, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_can().gprmc().track_made_good());
        SetNode(GPS_CAN_TMG, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_can().gprmc().magnetic_variation());
        SetNode(GPS_CAN_MAGVAR, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(sensors.gps_ais().gprmc().utc_timestamp());
        SetNode(GPS_AIS_TIME, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_ais().gprmc().latitude());
        SetNode(GPS_AIS_LAT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_ais().gprmc().longitude());
        SetNode(GPS_AIS_LON, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_ais().gprmc().ground_speed());
        SetNode(GPS_AIS_GNDSPEED, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_ais().gprmc().track_made_good());
        SetNode(GPS_AIS_TMG, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.gps_ais().gprmc().magnetic_variation());
        SetNode(GPS_AIS_MAGVAR, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_1().battery_pack_data().current());
        SetNode(BMS1_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_1().battery_pack_data().total_voltage());
        SetNode(BMS1_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_2().battery_pack_data().current());
        SetNode(BMS2_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_2().battery_pack_data().total_voltage());
        SetNode(BMS2_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_3().battery_pack_data().current());
        SetNode(BMS3_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_3().battery_pack_data().total_voltage());
        SetNode(BMS3_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_4().battery_pack_data().current());
        SetNode(BMS4_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_4().battery_pack_data().total_voltage());
        SetNode(BMS4_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_5().battery_pack_data().current());
        SetNode(BMS5_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_5().battery_pack_data().total_voltage());
        SetNode(BMS5_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_6().battery_pack_data().current());
        SetNode(BMS6_CURRENT, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.bms_6().battery_pack_data().total_voltage());
        SetNode(BMS6_VOLTAGE, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.accelerometer().boat_orientation_data().x_axis_acceleration());
        SetNode(ACCELEROMETER_X, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.accelerometer().boat_orientation_data().y_axis_acceleration());
        SetNode(ACCELEROMETER_Y, val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(sensors.accelerometer().boat_orientation_data().z_axis_acceleration());
        SetNode(ACCELEROMETER_Z, val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    return root;
}

NetworkTable::Uccms NetworkTable::RootToUccms(NetworkTable::Node *root) {
    NetworkTable::Uccms uccms;

    try {
        uccms.mutable_boom_angle_sensor()->set_current(\
                GetNode("/boom_angle_sensor/uccm/current", root).value().int_data());
        uccms.mutable_boom_angle_sensor()->set_voltage(\
                GetNode("/boom_angle_sensor/uccm/voltage", root).value().int_data());
        uccms.mutable_boom_angle_sensor()->set_temperature(\
                GetNode("/boom_angle_sensor/uccm/temperature", root).value().int_data());
        uccms.mutable_boom_angle_sensor()->set_status(\
                GetNode("/boom_angle_sensor/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_rudder_motor_control_0()->set_current(\
                GetNode("/rudder_motor_control_0/uccm/current", root).value().int_data());
        uccms.mutable_rudder_motor_control_0()->set_voltage(\
                GetNode("/rudder_motor_control_0/uccm/voltage", root).value().int_data());
        uccms.mutable_rudder_motor_control_0()->set_temperature(\
                GetNode("/rudder_motor_control_0/uccm/temperature", root).value().int_data());
        uccms.mutable_rudder_motor_control_0()->set_status(\
                GetNode("/rudder_motor_control_0/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_rudder_motor_control_1()->set_current(\
                GetNode("/rudder_motor_control_1/uccm/current", root).value().int_data());
        uccms.mutable_rudder_motor_control_1()->set_voltage(\
                GetNode("/rudder_motor_control_1/uccm/voltage", root).value().int_data());
        uccms.mutable_rudder_motor_control_1()->set_temperature(\
                GetNode("/rudder_motor_control_1/uccm/temperature", root).value().int_data());
        uccms.mutable_rudder_motor_control_1()->set_status(\
                GetNode("/rudder_motor_control_1/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_winch_motor_control_0()->set_current(\
                GetNode("/winch_motor_control_0/uccm/current", root).value().int_data());
        uccms.mutable_winch_motor_control_0()->set_voltage(\
                GetNode("/winch_motor_control_0/uccm/voltage", root).value().int_data());
        uccms.mutable_winch_motor_control_0()->set_temperature(\
                GetNode("/winch_motor_control_0/uccm/temperature", root).value().int_data());
        uccms.mutable_winch_motor_control_0()->set_status(\
                GetNode("/winch_motor_control_0/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_winch_motor_control_1()->set_current(\
                GetNode("/winch_motor_control_1/uccm/current", root).value().int_data());
        uccms.mutable_winch_motor_control_1()->set_voltage(\
                GetNode("/winch_motor_control_1/uccm/voltage", root).value().int_data());
        uccms.mutable_winch_motor_control_1()->set_temperature(\
                GetNode("/winch_motor_control_1/uccm/temperature", root).value().int_data());
        uccms.mutable_winch_motor_control_1()->set_status(\
                GetNode("/winch_motor_control_1/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_wind_sensor_0()->set_current(\
                GetNode("/wind_sensor_0/uccm/current", root).value().int_data());
        uccms.mutable_wind_sensor_0()->set_voltage(\
                GetNode("/wind_sensor_0/uccm/voltage", root).value().int_data());
        uccms.mutable_wind_sensor_0()->set_temperature(\
                GetNode("/wind_sensor_0/uccm/temperature", root).value().int_data());
        uccms.mutable_wind_sensor_0()->set_status(\
                GetNode("/wind_sensor_0/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_wind_sensor_1()->set_current(\
                GetNode("/wind_sensor_1/uccm/current", root).value().int_data());
        uccms.mutable_wind_sensor_1()->set_voltage(\
                GetNode("/wind_sensor_1/uccm/voltage", root).value().int_data());
        uccms.mutable_wind_sensor_1()->set_temperature(\
                GetNode("/wind_sensor_1/uccm/temperature", root).value().int_data());
        uccms.mutable_wind_sensor_1()->set_status(\
                GetNode("/wind_sensor_1/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_wind_sensor_2()->set_current(\
                GetNode("/wind_sensor_2/uccm/current", root).value().int_data());
        uccms.mutable_wind_sensor_2()->set_voltage(\
                GetNode("/wind_sensor_2/uccm/voltage", root).value().int_data());
        uccms.mutable_wind_sensor_2()->set_temperature(\
                GetNode("/wind_sensor_2/uccm/temperature", root).value().int_data());
        uccms.mutable_wind_sensor_2()->set_status(\
                GetNode("/wind_sensor_2/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_gps_can()->set_current(\
                GetNode("/gps_can/uccm/current", root).value().int_data());
        uccms.mutable_gps_can()->set_voltage(\
                GetNode("/gps_can/uccm/voltage", root).value().int_data());
        uccms.mutable_gps_can()->set_temperature(\
                GetNode("/gps_can/uccm/temperature", root).value().int_data());
        uccms.mutable_gps_can()->set_status(\
                GetNode("/gps_can/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_gps_ais()->set_current(\
                GetNode("/gps_ais/uccm/current", root).value().int_data());
        uccms.mutable_gps_ais()->set_voltage(\
                GetNode("/gps_ais/uccm/voltage", root).value().int_data());
        uccms.mutable_gps_ais()->set_temperature(\
                GetNode("/gps_ais/uccm/temperature", root).value().int_data());
        uccms.mutable_gps_ais()->set_status(\
                GetNode("/gps_ais/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_0()->set_current(\
                GetNode("/bms_0/uccm/current", root).value().int_data());
        uccms.mutable_bms_0()->set_voltage(\
                GetNode("/bms_0/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_0()->set_temperature(\
                GetNode("/bms_0/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_0()->set_status(\
                GetNode("/bms_0/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_1()->set_current(\
                GetNode("/bms_1/uccm/current", root).value().int_data());
        uccms.mutable_bms_1()->set_voltage(\
                GetNode("/bms_1/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_1()->set_temperature(\
                GetNode("/bms_1/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_1()->set_status(\
                GetNode("/bms_1/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_2()->set_current(\
                GetNode("/bms_2/uccm/current", root).value().int_data());
        uccms.mutable_bms_2()->set_voltage(\
                GetNode("/bms_2/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_2()->set_temperature(\
                GetNode("/bms_2/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_2()->set_status(\
                GetNode("/bms_2/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_3()->set_current(\
                GetNode("/bms_3/uccm/current", root).value().int_data());
        uccms.mutable_bms_3()->set_voltage(\
                GetNode("/bms_3/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_3()->set_temperature(\
                GetNode("/bms_3/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_3()->set_status(\
                GetNode("/bms_3/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_4()->set_current(\
                GetNode("/bms_4/uccm/current", root).value().int_data());
        uccms.mutable_bms_4()->set_voltage(\
                GetNode("/bms_4/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_4()->set_temperature(\
                GetNode("/bms_4/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_4()->set_status(\
                GetNode("/bms_4/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_bms_5()->set_current(\
                GetNode("/bms_5/uccm/current", root).value().int_data());
        uccms.mutable_bms_5()->set_voltage(\
                GetNode("/bms_5/uccm/voltage", root).value().int_data());
        uccms.mutable_bms_5()->set_temperature(\
                GetNode("/bms_5/uccm/temperature", root).value().int_data());
        uccms.mutable_bms_5()->set_status(\
                GetNode("/bms_5/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        uccms.mutable_accelerometer()->set_current(\
                GetNode("/accelerometer/uccm/current", root).value().int_data());
        uccms.mutable_accelerometer()->set_voltage(\
                GetNode("/accelerometer/uccm/voltage", root).value().int_data());
        uccms.mutable_accelerometer()->set_temperature(\
                GetNode("/accelerometer/uccm/temperature", root).value().int_data());
        uccms.mutable_accelerometer()->set_status(\
                GetNode("/accelerometer/uccm/status", root).value().string_data());
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    return uccms;
}

NetworkTable::Node NetworkTable::UccmsToRoot(const NetworkTable::Uccms &uccms) {
    /*
     * Have to do this all by hand :(((
     * Use copy/paste and search/replace
     */
    NetworkTable::Node root;
    NetworkTable::Value val;

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.boom_angle_sensor().current());
        SetNode("/boom_angle_sensor/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.boom_angle_sensor().voltage());
        SetNode("/boom_angle_sensor/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.boom_angle_sensor().temperature());
        SetNode("/boom_angle_sensor/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.boom_angle_sensor().status());
        SetNode("/boom_angle_sensor/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_0().current());
        SetNode("/rudder_motor_control_0/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_0().voltage());
        SetNode("/rudder_motor_control_0/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_0().temperature());
        SetNode("/rudder_motor_control_0/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.rudder_motor_control_0().status());
        SetNode("/rudder_motor_control_0/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_1().current());
        SetNode("/rudder_motor_control_1/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_1().voltage());
        SetNode("/rudder_motor_control_1/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.rudder_motor_control_1().temperature());
        SetNode("/rudder_motor_control_1/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.rudder_motor_control_1().status());
        SetNode("/rudder_motor_control_1/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_0().current());
        SetNode("/winch_motor_control_0/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_0().voltage());
        SetNode("/winch_motor_control_0/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_0().temperature());
        SetNode("/winch_motor_control_0/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.winch_motor_control_0().status());
        SetNode("/winch_motor_control_0/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_1().current());
        SetNode("/winch_motor_control_1/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_1().voltage());
        SetNode("/winch_motor_control_1/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.winch_motor_control_1().temperature());
        SetNode("/winch_motor_control_1/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.winch_motor_control_1().status());
        SetNode("/winch_motor_control_1/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_0().current());
        SetNode("/wind_sensor_0/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_0().voltage());
        SetNode("/wind_sensor_0/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_0().temperature());
        SetNode("/wind_sensor_0/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.wind_sensor_0().status());
        SetNode("/wind_sensor_0/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_1().current());
        SetNode("/wind_sensor_1/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_1().voltage());
        SetNode("/wind_sensor_1/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_1().temperature());
        SetNode("/wind_sensor_1/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.wind_sensor_1().status());
        SetNode("/wind_sensor_1/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_2().current());
        SetNode("/wind_sensor_2/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_2().voltage());
        SetNode("/wind_sensor_2/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.wind_sensor_2().temperature());
        SetNode("/wind_sensor_2/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.wind_sensor_2().status());
        SetNode("/wind_sensor_2/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_can().current());
        SetNode("/gps_can/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_can().voltage());
        SetNode("/gps_can/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_can().temperature());
        SetNode("/gps_can/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.gps_can().status());
        SetNode("/gps_can/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_ais().current());
        SetNode("/gps_ais/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_ais().voltage());
        SetNode("/gps_ais/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.gps_ais().temperature());
        SetNode("/gps_ais/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.gps_ais().status());
        SetNode("/gps_ais/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_0().current());
        SetNode("/bms_0/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_0().voltage());
        SetNode("/bms_0/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_0().temperature());
        SetNode("/bms_0/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_0().status());
        SetNode("/bms_0/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_1().current());
        SetNode("/bms_1/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_1().voltage());
        SetNode("/bms_1/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_1().temperature());
        SetNode("/bms_1/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_1().status());
        SetNode("/bms_1/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_2().current());
        SetNode("/bms_2/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_2().voltage());
        SetNode("/bms_2/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_2().temperature());
        SetNode("/bms_2/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_2().status());
        SetNode("/bms_2/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_3().current());
        SetNode("/bms_3/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_3().voltage());
        SetNode("/bms_3/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_3().temperature());
        SetNode("/bms_3/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_3().status());
        SetNode("/bms_3/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_4().current());
        SetNode("/bms_4/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_4().voltage());
        SetNode("/bms_4/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_4().temperature());
        SetNode("/bms_4/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_4().status());
        SetNode("/bms_4/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_5().current());
        SetNode("/bms_5/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_5().voltage());
        SetNode("/bms_5/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.bms_5().temperature());
        SetNode("/bms_5/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.bms_5().status());
        SetNode("/bms_5/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    try {
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.accelerometer().current());
        SetNode("/accelerometer/uccm/current", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.accelerometer().voltage());
        SetNode("/accelerometer/uccm/voltage", val, &root);
        val.set_type(NetworkTable::Value::INT);
        val.set_int_data(uccms.accelerometer().temperature());
        SetNode("/accelerometer/uccm/temperature", val, &root);
        val.set_type(NetworkTable::Value::STRING);
        val.set_string_data(uccms.accelerometer().status());
        SetNode("/accelerometer/uccm/status", val, &root);
    } catch (const NetworkTable::NodeNotFoundException &e) {
    }

    return root;
}
