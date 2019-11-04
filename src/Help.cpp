// Copyright 2017 UBC Sailbot

#include "Help.h"
#include "Exceptions.h"

#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

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
        switch (root.value().type()) {
            case NetworkTable::Value::INT : std::cout << root.value().int_data();
                                            break;
            case NetworkTable::Value::BOOL : std::cout << root.value().bool_data();
                                            break;
            case NetworkTable::Value::FLOAT : std::cout << root.value().float_data();
                                            break;
            case NetworkTable::Value::STRING : std::cout << root.value().string_data();
                                            break;
            case NetworkTable::Value::BYTES : std::cout << root.value().bytes_data();
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
    // Get each "slice" of the uri. eg "/gps/lat"
    // becomes {"gps", "lat"}:
    boost::trim_left_if(uri, boost::is_any_of("/"));
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
    std::ofstream output_filestream(filepath);
    output_filestream << root.SerializeAsString();
    output_filestream.close();
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

    sensors.mutable_boom_angle_sensor()->mutable_sensor_data()->set_angle(\
            GetNode("/boom_angle_sensor/sensor_data/angle", root).value().int_data());

    sensors.mutable_wind_sensor_0()->mutable_iimwv()->set_wind_speed(\
            GetNode("/wind_sensor_0/iimwv/wind_speed", root).value().int_data());
    sensors.mutable_wind_sensor_0()->mutable_iimwv()->set_wind_direction(\
            GetNode("/wind_sensor_0/iimwv/wind_direction", root).value().int_data());
    sensors.mutable_wind_sensor_0()->mutable_iimwv()->set_wind_reference(\
            GetNode("/wind_sensor_0/iimwv/wind_reference", root).value().int_data());
    sensors.mutable_wind_sensor_0()->mutable_wixdir()->set_wind_temperature(\
            GetNode("/wind_sensor_0/wixdir/wind_temperature", root).value().int_data());

    sensors.mutable_wind_sensor_1()->mutable_iimwv()->set_wind_speed(\
            GetNode("/wind_sensor_1/iimwv/wind_speed", root).value().int_data());
    sensors.mutable_wind_sensor_1()->mutable_iimwv()->set_wind_direction(\
            GetNode("/wind_sensor_1/iimwv/wind_direction", root).value().int_data());
    sensors.mutable_wind_sensor_1()->mutable_iimwv()->set_wind_reference(\
            GetNode("/wind_sensor_1/iimwv/wind_reference", root).value().int_data());
    sensors.mutable_wind_sensor_1()->mutable_wixdir()->set_wind_temperature(\
            GetNode("/wind_sensor_1/wixdir/wind_temperature", root).value().int_data());

    sensors.mutable_wind_sensor_2()->mutable_iimwv()->set_wind_speed(\
            GetNode("/wind_sensor_2/iimwv/wind_speed", root).value().int_data());
    sensors.mutable_wind_sensor_2()->mutable_iimwv()->set_wind_direction(\
            GetNode("/wind_sensor_2/iimwv/wind_direction", root).value().int_data());
    sensors.mutable_wind_sensor_2()->mutable_iimwv()->set_wind_reference(\
            GetNode("/wind_sensor_2/iimwv/wind_reference", root).value().int_data());
    sensors.mutable_wind_sensor_2()->mutable_wixdir()->set_wind_temperature(\
            GetNode("/wind_sensor_2/wixdir/wind_temperature", root).value().int_data());

    sensors.mutable_gps_0()->mutable_gprmc()->set_utc_timestamp(\
            GetNode("/gps_0/gprmc/utc_timestamp", root).value().string_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_latitude(\
            GetNode("/gps_0/gprmc/latitude", root).value().float_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_longitude(\
            GetNode("/gps_0/gprmc/longitude", root).value().float_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_latitude_loc(\
            GetNode("/gps_0/gprmc/latitude_loc", root).value().bool_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_longitude_loc(\
            GetNode("/gps_0/gprmc/longitude_loc", root).value().bool_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_ground_speed(\
            GetNode("/gps_0/gprmc/ground_speed", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_track_made_good(\
            GetNode("/gps_0/gprmc/track_made_good", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_magnetic_variation(\
            GetNode("/gps_0/gprmc/magnetic_variation", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gprmc()->set_magnetic_variation_sense(\
            GetNode("/gps_0/gprmc/magnetic_variation_sense", root).value().bool_data());
    sensors.mutable_gps_0()->mutable_gpgga()->set_quality_indicator(\
            GetNode("/gps_0/gpgga/quality_indicator", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gpgga()->set_hdop(\
            GetNode("/gps_0/gpgga/hdop", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gpgga()->set_antenna_altitude(\
            GetNode("/gps_0/gpgga/antenna_altitude", root).value().int_data());
    sensors.mutable_gps_0()->mutable_gpgga()->set_geoidal_separation(\
            GetNode("/gps_0/gpgga/geoidal_separation", root).value().int_data());

    sensors.mutable_gps_1()->mutable_gprmc()->set_utc_timestamp(\
            GetNode("/gps_1/gprmc/utc_timestamp", root).value().string_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_latitude(\
            GetNode("/gps_1/gprmc/latitude", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_longitude(\
            GetNode("/gps_1/gprmc/longitude", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_latitude_loc(\
            GetNode("/gps_1/gprmc/latitude_loc", root).value().bool_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_longitude_loc(\
            GetNode("/gps_1/gprmc/longitude_loc", root).value().bool_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_ground_speed(\
            GetNode("/gps_1/gprmc/ground_speed", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_track_made_good(\
            GetNode("/gps_1/gprmc/track_made_good", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_magnetic_variation(\
            GetNode("/gps_1/gprmc/magnetic_variation", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gprmc()->set_magnetic_variation_sense(\
            GetNode("/gps_1/gprmc/magnetic_variation_sense", root).value().bool_data());
    sensors.mutable_gps_1()->mutable_gpgga()->set_quality_indicator(\
            GetNode("/gps_1/gpgga/quality_indicator", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gpgga()->set_hdop(\
            GetNode("/gps_1/gpgga/hdop", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gpgga()->set_antenna_altitude(\
            GetNode("/gps_1/gpgga/antenna_altitude", root).value().int_data());
    sensors.mutable_gps_1()->mutable_gpgga()->set_geoidal_separation(\
            GetNode("/gps_1/gpgga/geoidal_separation", root).value().int_data());

    sensors.mutable_bms_0()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_0/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_0()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_0/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_0()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_0/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_bms_1()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_1/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_1()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_1/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_1()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_1/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_bms_2()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_2/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_2()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_2/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_2()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_2/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_bms_3()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_3/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_3()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_3/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_3()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_3/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_bms_4()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_4/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_4()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_4/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_4()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_4/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_bms_5()->mutable_battery_pack_data()->set_current(\
            GetNode("/bms_5/battery_pack_data/current", root).value().int_data());
    sensors.mutable_bms_5()->mutable_battery_pack_data()->set_total_voltage(\
            GetNode("/bms_5/battery_pack_data/total_voltage", root).value().int_data());
    sensors.mutable_bms_5()->mutable_battery_pack_data()->set_temperature(\
            GetNode("/bms_5/battery_pack_data/temperature", root).value().int_data());

    sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_x_axis_acceleration(\
            GetNode("/accelerometer/boat_orientation_data/x_axis_acceleration", root).value().int_data());
    sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_y_axis_acceleration(\
            GetNode("/accelerometer/boat_orientation_data/y_axis_acceleration", root).value().int_data());
    sensors.mutable_accelerometer()->mutable_boat_orientation_data()->set_z_axis_acceleration(\
            GetNode("/accelerometer/boat_orientation_data/z_axis_acceleration", root).value().int_data());
    return sensors;
}

NetworkTable::Node NetworkTable::SensorsToRoot(const NetworkTable::Sensors &sensors) {
    /*
     * Have to do this all by hand :(((
     * Use copy/paste and search/replace
     */
    NetworkTable::Node root;
    NetworkTable::Value val;

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.boom_angle_sensor().sensor_data().angle());
    SetNode("/boom_angle_sensor/boom_angle_sensor/angle", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_0().iimwv().wind_speed());
    SetNode("/wind_sensor_0/iimwv/wind_speed", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_0().iimwv().wind_direction());
    SetNode("/wind_sensor_0/iimwv/wind_direction", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_0().iimwv().wind_reference());
    SetNode("/wind_sensor_0/iimwv/wind_reference", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_0().wixdir().wind_temperature());
    SetNode("/wind_sensor_0/wixdir/wind_temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_1().iimwv().wind_speed());
    SetNode("/wind_sensor_1/iimwv/wind_speed", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_1().iimwv().wind_direction());
    SetNode("/wind_sensor_1/iimwv/wind_direction", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_1().iimwv().wind_reference());
    SetNode("/wind_sensor_1/iimwv/wind_reference", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_1().wixdir().wind_temperature());
    SetNode("/wind_sensor_1/wixdir/wind_temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_2().iimwv().wind_speed());
    SetNode("/wind_sensor_2/iimwv/wind_speed", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_2().iimwv().wind_direction());
    SetNode("/wind_sensor_2/iimwv/wind_direction", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_2().iimwv().wind_reference());
    SetNode("/wind_sensor_2/iimwv/wind_reference", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.wind_sensor_2().wixdir().wind_temperature());
    SetNode("/wind_sensor_2/wixdir/wind_temperature", val, &root);

    val.set_type(NetworkTable::Value::STRING);
    val.set_string_data(sensors.gps_0().gprmc().utc_timestamp());
    SetNode("/gps_0/gprmc/utc_timestamp", val, &root);
    val.set_type(NetworkTable::Value::FLOAT);
    val.set_float_data(sensors.gps_0().gprmc().latitude());
    SetNode("/gps_0/gprmc/latitude", val, &root);
    val.set_type(NetworkTable::Value::FLOAT);
    val.set_float_data(sensors.gps_0().gprmc().longitude());
    SetNode("/gps_0/gprmc/longitude", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_0().gprmc().latitude_loc());
    SetNode("/gps_0/gprmc/latitude_loc", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_0().gprmc().longitude_loc());
    SetNode("/gps_0/gprmc/longitude_loc", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gprmc().ground_speed());
    SetNode("/gps_0/gprmc/ground_speed", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gprmc().track_made_good());
    SetNode("/gps_0/gprmc/track_made_good", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gprmc().magnetic_variation());
    SetNode("/gps_0/gprmc/magnetic_variation", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_0().gprmc().magnetic_variation_sense());
    SetNode("/gps_0/gprmc/magnetic_variation_sense", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gpgga().quality_indicator());
    SetNode("/gps_0/gpgga/quality_indicator", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gpgga().hdop());
    SetNode("/gps_0/gpgga/hdop", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gpgga().antenna_altitude());
    SetNode("/gps_0/gpgga/antenna_altitude", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_0().gpgga().geoidal_separation());
    SetNode("/gps_0/gpgga/geoidal_separation", val, &root);

    val.set_type(NetworkTable::Value::STRING);
    val.set_string_data(sensors.gps_1().gprmc().utc_timestamp());
    SetNode("/gps_1/gprmc/utc_timestamp", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gprmc().latitude());
    SetNode("/gps_1/gprmc/latitude", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gprmc().longitude());
    SetNode("/gps_1/gprmc/longitude", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_1().gprmc().latitude_loc());
    SetNode("/gps_1/gprmc/latitude_loc", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_1().gprmc().longitude_loc());
    SetNode("/gps_1/gprmc/longitude_loc", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gprmc().ground_speed());
    SetNode("/gps_1/gprmc/ground_speed", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gprmc().track_made_good());
    SetNode("/gps_1/gprmc/track_made_good", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gprmc().magnetic_variation());
    SetNode("/gps_1/gprmc/magnetic_variation", val, &root);
    val.set_type(NetworkTable::Value::BOOL);
    val.set_bool_data(sensors.gps_1().gprmc().magnetic_variation_sense());
    SetNode("/gps_1/gprmc/magnetic_variation_sense", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gpgga().quality_indicator());
    SetNode("/gps_1/gpgga/quality_indicator", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gpgga().hdop());
    SetNode("/gps_1/gpgga/hdop", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gpgga().antenna_altitude());
    SetNode("/gps_1/gpgga/antenna_altitude", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.gps_1().gpgga().geoidal_separation());
    SetNode("/gps_1/gpgga/geoidal_separation", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_0().battery_pack_data().current());
    SetNode("/bms_0/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_0().battery_pack_data().total_voltage());
    SetNode("/bms_0/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_0().battery_pack_data().temperature());
    SetNode("/bms_0/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_1().battery_pack_data().current());
    SetNode("/bms_1/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_1().battery_pack_data().total_voltage());
    SetNode("/bms_1/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_1().battery_pack_data().temperature());
    SetNode("/bms_1/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_2().battery_pack_data().current());
    SetNode("/bms_2/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_2().battery_pack_data().total_voltage());
    SetNode("/bms_2/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_2().battery_pack_data().temperature());
    SetNode("/bms_2/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_3().battery_pack_data().current());
    SetNode("/bms_3/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_3().battery_pack_data().total_voltage());
    SetNode("/bms_3/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_3().battery_pack_data().temperature());
    SetNode("/bms_3/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_4().battery_pack_data().current());
    SetNode("/bms_4/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_4().battery_pack_data().total_voltage());
    SetNode("/bms_4/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_4().battery_pack_data().temperature());
    SetNode("/bms_4/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_5().battery_pack_data().current());
    SetNode("/bms_5/battery_pack_data/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_5().battery_pack_data().total_voltage());
    SetNode("/bms_5/battery_pack_data/total_voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.bms_5().battery_pack_data().temperature());
    SetNode("/bms_5/battery_pack_data/temperature", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.accelerometer().boat_orientation_data().x_axis_acceleration());
    SetNode("/accelerometer/boat_orientation_data/x_axis_acceleration", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.accelerometer().boat_orientation_data().y_axis_acceleration());
    SetNode("/accelerometer/boat_orientation_data/y_axis_acceleration", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(sensors.accelerometer().boat_orientation_data().z_axis_acceleration());
    SetNode("/accelerometer/boat_orientation_data/z_axis_acceleration", val, &root);

    return root;
}

NetworkTable::Uccms NetworkTable::RootToUccms(NetworkTable::Node *root) {
    NetworkTable::Uccms uccms;
    uccms.mutable_boom_angle_sensor()->set_current(\
            GetNode("/boom_angle_sensor/uccm/current", root).value().int_data());
    uccms.mutable_boom_angle_sensor()->set_voltage(\
            GetNode("/boom_angle_sensor/uccm/voltage", root).value().int_data());
    uccms.mutable_boom_angle_sensor()->set_temperature(\
            GetNode("/boom_angle_sensor/uccm/temperature", root).value().int_data());
    uccms.mutable_boom_angle_sensor()->set_status(\
            GetNode("/boom_angle_sensor/uccm/status", root).value().string_data());

    uccms.mutable_rudder_motor_control_0()->set_current(\
            GetNode("/rudder_motor_control_0/uccm/current", root).value().int_data());
    uccms.mutable_rudder_motor_control_0()->set_voltage(\
            GetNode("/rudder_motor_control_0/uccm/voltage", root).value().int_data());
    uccms.mutable_rudder_motor_control_0()->set_temperature(\
            GetNode("/rudder_motor_control_0/uccm/temperature", root).value().int_data());
    uccms.mutable_rudder_motor_control_0()->set_status(\
            GetNode("/rudder_motor_control_0/uccm/status", root).value().string_data());

    uccms.mutable_rudder_motor_control_1()->set_current(\
            GetNode("/rudder_motor_control_1/uccm/current", root).value().int_data());
    uccms.mutable_rudder_motor_control_1()->set_voltage(\
            GetNode("/rudder_motor_control_1/uccm/voltage", root).value().int_data());
    uccms.mutable_rudder_motor_control_1()->set_temperature(\
            GetNode("/rudder_motor_control_1/uccm/temperature", root).value().int_data());
    uccms.mutable_rudder_motor_control_1()->set_status(\
            GetNode("/rudder_motor_control_1/uccm/status", root).value().string_data());

    uccms.mutable_winch_motor_control_0()->set_current(\
            GetNode("/winch_motor_control_0/uccm/current", root).value().int_data());
    uccms.mutable_winch_motor_control_0()->set_voltage(\
            GetNode("/winch_motor_control_0/uccm/voltage", root).value().int_data());
    uccms.mutable_winch_motor_control_0()->set_temperature(\
            GetNode("/winch_motor_control_0/uccm/temperature", root).value().int_data());
    uccms.mutable_winch_motor_control_0()->set_status(\
            GetNode("/winch_motor_control_0/uccm/status", root).value().string_data());

    uccms.mutable_winch_motor_control_1()->set_current(\
            GetNode("/winch_motor_control_1/uccm/current", root).value().int_data());
    uccms.mutable_winch_motor_control_1()->set_voltage(\
            GetNode("/winch_motor_control_1/uccm/voltage", root).value().int_data());
    uccms.mutable_winch_motor_control_1()->set_temperature(\
            GetNode("/winch_motor_control_1/uccm/temperature", root).value().int_data());
    uccms.mutable_winch_motor_control_1()->set_status(\
            GetNode("/winch_motor_control_1/uccm/status", root).value().string_data());

    uccms.mutable_wind_sensor_0()->set_current(\
            GetNode("/wind_sensor_0/uccm/current", root).value().int_data());
    uccms.mutable_wind_sensor_0()->set_voltage(\
            GetNode("/wind_sensor_0/uccm/voltage", root).value().int_data());
    uccms.mutable_wind_sensor_0()->set_temperature(\
            GetNode("/wind_sensor_0/uccm/temperature", root).value().int_data());
    uccms.mutable_wind_sensor_0()->set_status(\
            GetNode("/wind_sensor_0/uccm/status", root).value().string_data());

    uccms.mutable_wind_sensor_1()->set_current(\
            GetNode("/wind_sensor_1/uccm/current", root).value().int_data());
    uccms.mutable_wind_sensor_1()->set_voltage(\
            GetNode("/wind_sensor_1/uccm/voltage", root).value().int_data());
    uccms.mutable_wind_sensor_1()->set_temperature(\
            GetNode("/wind_sensor_1/uccm/temperature", root).value().int_data());
    uccms.mutable_wind_sensor_1()->set_status(\
            GetNode("/wind_sensor_1/uccm/status", root).value().string_data());

    uccms.mutable_wind_sensor_2()->set_current(\
            GetNode("/wind_sensor_2/uccm/current", root).value().int_data());
    uccms.mutable_wind_sensor_2()->set_voltage(\
            GetNode("/wind_sensor_2/uccm/voltage", root).value().int_data());
    uccms.mutable_wind_sensor_2()->set_temperature(\
            GetNode("/wind_sensor_2/uccm/temperature", root).value().int_data());
    uccms.mutable_wind_sensor_2()->set_status(\
            GetNode("/wind_sensor_2/uccm/status", root).value().string_data());

    uccms.mutable_gps_0()->set_current(\
            GetNode("/gps_0/uccm/current", root).value().int_data());
    uccms.mutable_gps_0()->set_voltage(\
            GetNode("/gps_0/uccm/voltage", root).value().int_data());
    uccms.mutable_gps_0()->set_temperature(\
            GetNode("/gps_0/uccm/temperature", root).value().int_data());
    uccms.mutable_gps_0()->set_status(\
            GetNode("/gps_0/uccm/status", root).value().string_data());

    uccms.mutable_gps_1()->set_current(\
            GetNode("/gps_1/uccm/current", root).value().int_data());
    uccms.mutable_gps_1()->set_voltage(\
            GetNode("/gps_1/uccm/voltage", root).value().int_data());
    uccms.mutable_gps_1()->set_temperature(\
            GetNode("/gps_1/uccm/temperature", root).value().int_data());
    uccms.mutable_gps_1()->set_status(\
            GetNode("/gps_1/uccm/status", root).value().string_data());

    uccms.mutable_bms_0()->set_current(\
            GetNode("/bms_0/uccm/current", root).value().int_data());
    uccms.mutable_bms_0()->set_voltage(\
            GetNode("/bms_0/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_0()->set_temperature(\
            GetNode("/bms_0/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_0()->set_status(\
            GetNode("/bms_0/uccm/status", root).value().string_data());

    uccms.mutable_bms_1()->set_current(\
            GetNode("/bms_1/uccm/current", root).value().int_data());
    uccms.mutable_bms_1()->set_voltage(\
            GetNode("/bms_1/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_1()->set_temperature(\
            GetNode("/bms_1/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_1()->set_status(\
            GetNode("/bms_1/uccm/status", root).value().string_data());

    uccms.mutable_bms_2()->set_current(\
            GetNode("/bms_2/uccm/current", root).value().int_data());
    uccms.mutable_bms_2()->set_voltage(\
            GetNode("/bms_2/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_2()->set_temperature(\
            GetNode("/bms_2/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_2()->set_status(\
            GetNode("/bms_2/uccm/status", root).value().string_data());

    uccms.mutable_bms_3()->set_current(\
            GetNode("/bms_3/uccm/current", root).value().int_data());
    uccms.mutable_bms_3()->set_voltage(\
            GetNode("/bms_3/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_3()->set_temperature(\
            GetNode("/bms_3/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_3()->set_status(\
            GetNode("/bms_3/uccm/status", root).value().string_data());

    uccms.mutable_bms_4()->set_current(\
            GetNode("/bms_4/uccm/current", root).value().int_data());
    uccms.mutable_bms_4()->set_voltage(\
            GetNode("/bms_4/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_4()->set_temperature(\
            GetNode("/bms_4/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_4()->set_status(\
            GetNode("/bms_4/uccm/status", root).value().string_data());

    uccms.mutable_bms_5()->set_current(\
            GetNode("/bms_5/uccm/current", root).value().int_data());
    uccms.mutable_bms_5()->set_voltage(\
            GetNode("/bms_5/uccm/voltage", root).value().int_data());
    uccms.mutable_bms_5()->set_temperature(\
            GetNode("/bms_5/uccm/temperature", root).value().int_data());
    uccms.mutable_bms_5()->set_status(\
            GetNode("/bms_5/uccm/status", root).value().string_data());

    uccms.mutable_accelerometer()->set_current(\
            GetNode("/accelerometer/uccm/current", root).value().int_data());
    uccms.mutable_accelerometer()->set_voltage(\
            GetNode("/accelerometer/uccm/voltage", root).value().int_data());
    uccms.mutable_accelerometer()->set_temperature(\
            GetNode("/accelerometer/uccm/temperature", root).value().int_data());
    uccms.mutable_accelerometer()->set_status(\
            GetNode("/accelerometer/uccm/status", root).value().string_data());

    return uccms;
}

NetworkTable::Node NetworkTable::UccmsToRoot(const NetworkTable::Uccms &uccms) {
    /*
     * Have to do this all by hand :(((
     * Use copy/paste and search/replace
     */
    NetworkTable::Node root;
    NetworkTable::Value val;

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

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_0().current());
    SetNode("/gps_0/uccm/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_0().voltage());
    SetNode("/gps_0/uccm/voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_0().temperature());
    SetNode("/gps_0/uccm/temperature", val, &root);
    val.set_type(NetworkTable::Value::STRING);
    val.set_string_data(uccms.gps_0().status());
    SetNode("/gps_0/uccm/status", val, &root);

    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_1().current());
    SetNode("/gps_1/uccm/current", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_1().voltage());
    SetNode("/gps_1/uccm/voltage", val, &root);
    val.set_type(NetworkTable::Value::INT);
    val.set_int_data(uccms.gps_1().temperature());
    SetNode("/gps_1/uccm/temperature", val, &root);
    val.set_type(NetworkTable::Value::STRING);
    val.set_string_data(uccms.gps_1().status());
    SetNode("/gps_1/uccm/status", val, &root);

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

    return root;
}
