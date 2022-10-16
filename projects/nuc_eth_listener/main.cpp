// Copyright 2017 UBC Sailbot

#include <zmq.hpp>
#include <thread>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "Help.h"
#include "Controller.pb.h"
#include "Node.pb.h"
#include "Sensors.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "sailbot_msg/Sensors.h"
#include "sailbot_msg/AISMsg.h"
#include "sailbot_msg/AISShip.h"
#include "sailbot_msg/path.h"
#include "sailbot_msg/manual_override.h"
#include "sailbot_msg/latlon.h"
#include "Controller.pb.h"
#include "Uri.h"

#define ROS_ACTUATION_NODE          "/rudder_winch_actuation_angle"
#define ROS_SENSORS_NODE            "sensors"
#define ROS_WAYPOINTS_NODE          "globalPath"
#define ROS_AIS_NODE                "AIS"
#define ROS_MANUAL_OVERRIDE_NODE    "/manual_override"
#define ROS_MIN_VOLTAGE_NODE        "/min_voltage"
// Make sure manual override node matches whatever is in nuc_manual_override project

#define MAX_BMS_VOLTAGE 19.5  // ELEC team set 19.5 V as the limit
#define JIB_ROT_LIMIT   350   // Modify if the actual limit when rigged is less than specified limit of 360

/*
 * Needed for communication over ethernet
 */
zmq::context_t context(1);
zmq::socket_t eth_socket(context, ZMQ_PAIR);

int send(const std::string &serialized_data) {
    zmq::message_t message(serialized_data.length());
    memcpy(message.data(), serialized_data.data(), serialized_data.length());
    return eth_socket.send(message, ZMQ_NOBLOCK);
}

/*
 * Needed for communication with other ROS nodes
 */
ros::Subscriber nt_sub;  // Actuation angle subscriber
ros::Subscriber manual_override_sub;
ros::Publisher sensors_pub;
ros::Publisher ais_msg_pub;
ros::Publisher waypoint_msg_pub;
ros::Publisher min_voltage_msg_pub;

bool manual_override_active = false;

void SendActuationAngle(double rudder_angle_radians, int sail_winch_position, int jib_winch_position) {
    // Both angles are in radians
    NetworkTable::Controller controller;
    jib_winch_position = jib_winch_position > JIB_ROT_LIMIT ? JIB_ROT_LIMIT : jib_winch_position;

    controller.set_type(NetworkTable::Controller::ACTUATION_DATA);
    controller.mutable_actuation_angle_data()->set_rudder_angle(rudder_angle_radians);
    controller.mutable_actuation_angle_data()->set_winch_angle(sail_winch_position);
    controller.mutable_actuation_angle_data()->set_jib_angle(jib_winch_position);

    std::string serialized_controller_actuation_data;
    controller.SerializeToString(&serialized_controller_actuation_data);

    std::cout << controller.DebugString() << std::endl;
    std::cout << serialized_controller_actuation_data << std::endl;

    std::cout << "Sending rudder angle (radians): " << controller.actuation_angle_data().rudder_angle()
        << " winch position: " << controller.actuation_angle_data().winch_angle()
        << " jib position: " << controller.actuation_angle_data().jib_angle() << std::endl;

    send(serialized_controller_actuation_data);
}

void ActuationCallBack(const sailbot_msg::actuation_angle ros_actuation_angle) {
    /*
     * Receive motor actuation data and send it
     * to the BBB.
     */
    if (ros::ok()) {
        if (!manual_override_active)
            SendActuationAngle(ros_actuation_angle.rudder_angle_radians,
                ros_actuation_angle.sail_winch_position,
                ros_actuation_angle.jib_winch_position);
    } else {
        std::cout << "Failed to receive actuation angle" << std::endl;
    }
}

void ManualOverrideCallback(const sailbot_msg::manual_override ros_actuation_angle) {
    if (ros::ok()) {
        if (ros_actuation_angle.manual_override_active) {
            manual_override_active = true;
            SendActuationAngle(ros_actuation_angle.rudder_angle_radians,
                ros_actuation_angle.sail_winch_position,
                ros_actuation_angle.jib_winch_position);
        } else {
            manual_override_active = false;
        }
    } else {
        manual_override_active = false;
    }
}

/*
void PowerControllerCallBack(const sailbot_msg::power_controller ros_power_data) {
    /*
     * Receive motor actuation data and send it
     * to the BBB.
     *
    if (ros::ok()) {
        // Both angles are in radians (CONFIRM WITH BRUCE)
	    NetworkTable::Controller controller;
        // Cant actually test this yet until Bruce adds this ROS node.
        // Can test using the ActuationAngle Callback function -> pass in ros_actuation_angle to test logic for the moment
        // TODO: Test once Bruce gets the ROS node working
	    controller.set_type(NetworkTable::Controller::POWER_DATA);
	    controller.mutable_power_controller_data()->set_pv_mppt_engage(ros_power_data.PV_MPPT_Engage);
	    controller.mutable_power_controller_data()->set_pwr_engage(ros_power_data.PWR_Engage);
	    controller.mutable_power_controller_data()->set_mppt_engage(ros_power_data.MPPT_Engage);

        std::string serialized_controller_power_data;
        controller.SerializeToString(&serialized_controller_power_data);

        std::cout << "PV_MPPT_Engage data: " << controller.power_controller_data().pv_mppt_engage() \
            << " PWR_Engage data: " << controller.power_controller_data().pwr_engage() \
	    << " MPPT_Engage data: " << controller.power_controller_data().mppt_engage() << std::endl;

        send(serialized_controller_power_data);
    } else {
        std::cout << "Failed to receive power data" << std::endl;
    }
}
*/

void PublishSensorData() {
    while (true) {
        /*
         * Receive updated sensor data and publish it to a ROS topic
         */
        zmq::message_t message;
        int rc = eth_socket.recv(&message);

        if (rc > 0 && ros::ok()) {
            std::string message_serialized(static_cast<char*>(message.data()), message.size());
            NetworkTable::Node node;
            node.ParseFromString(message_serialized);

            sailbot_msg::AISMsg ais_msg;
            try {
                auto proto_ais_Allboats = node.children().at("ais").children().at("boats").value().boats();
                for (const auto& proto_ais_boat : proto_ais_Allboats) {
                    sailbot_msg::AISShip ais_ship;
                    ais_ship.ID = proto_ais_boat.m_mmsi();
                    ais_ship.lat = proto_ais_boat.m_latitude();
                    ais_ship.lon = proto_ais_boat.m_longitude();
                    // double check that this the correct data
                    ais_ship.headingDegrees = proto_ais_boat.m_trueheading();
                    // is m_sog in m/s or km/h?
                    ais_ship.speedKmph = proto_ais_boat.m_sog();
                    ais_msg.ships.push_back(ais_ship);
                    std::cout << "AIS boat: " << proto_ais_boat.m_mmsi() << std::endl;
                }
            } catch(...) {
                std::cout << "ERROR** Ais boat data not found" << std::endl;
            }

            sailbot_msg::path waypoint_msg;
            try {
                auto proto_Allwaypoints = node.children().at("waypoints").value().waypoints();
                for (const auto& proto_waypoint : proto_Allwaypoints) {
                    sailbot_msg::latlon waypoint_data;
                    waypoint_data.lat = proto_waypoint.latitude();
                    waypoint_data.lon = proto_waypoint.longitude();
                    waypoint_msg.waypoints.push_back(waypoint_data);
                    std::cout << "Waypoint: " << proto_waypoint.latitude() << std::endl;
                }
            } catch(...) {
                std::cout << "ERROR** waypoint data not found" << std::endl;
            }

            NetworkTable::Sensors proto_sensors = NetworkTable::RootToSensors(&node);
            sailbot_msg::Sensors sensors;

            // Sailencoder
            sensors.sailencoder_degrees = proto_sensors.sailencoder_sensor().boom_angle_data().angle();

            // Wind Sensors
            sensors.wind_sensor_1_speed_knots = proto_sensors.wind_sensor_1().iimwv().wind_speed();
            sensors.wind_sensor_1_angle_degrees = proto_sensors.wind_sensor_1().iimwv().wind_angle();
            sensors.wind_sensor_2_speed_knots = proto_sensors.wind_sensor_2().iimwv().wind_speed();
            sensors.wind_sensor_2_angle_degrees = proto_sensors.wind_sensor_2().iimwv().wind_angle();
            sensors.wind_sensor_3_speed_knots = proto_sensors.wind_sensor_3().iimwv().wind_speed();
            sensors.wind_sensor_3_angle_degrees = proto_sensors.wind_sensor_3().iimwv().wind_angle();

            // GPS
            sensors.gps_can_timestamp_utc = proto_sensors.gps_can().gprmc().utc_timestamp();
            sensors.gps_can_latitude_degrees = proto_sensors.gps_can().gprmc().latitude();
            sensors.gps_can_longitude_degrees = proto_sensors.gps_can().gprmc().longitude();
            sensors.gps_can_groundspeed_knots = proto_sensors.gps_can().gprmc().ground_speed();
            sensors.gps_can_track_made_good_degrees = proto_sensors.gps_can().gprmc().track_made_good();
            sensors.gps_can_magnetic_variation_degrees = proto_sensors.gps_can().gprmc().magnetic_variation();
            sensors.gps_can_true_heading_degrees = proto_sensors.gps_can().gprmc().true_heading();

            sensors.gps_ais_timestamp_utc = proto_sensors.gps_can().gprmc().utc_timestamp();
            sensors.gps_ais_latitude_degrees = proto_sensors.gps_can().gprmc().latitude();
            sensors.gps_ais_longitude_degrees = proto_sensors.gps_can().gprmc().longitude();
            sensors.gps_ais_groundspeed_knots = proto_sensors.gps_can().gprmc().ground_speed();
            sensors.gps_ais_track_made_good_degrees = proto_sensors.gps_can().gprmc().track_made_good();
            sensors.gps_ais_magnetic_variation_degrees = proto_sensors.gps_can().gprmc().magnetic_variation();

            // Accelerometer
            sensors.accelerometer_x_force_millig = \
                proto_sensors.accelerometer().boat_orientation_data().x_axis_acceleration();
            sensors.accelerometer_y_force_millig = \
                proto_sensors.accelerometer().boat_orientation_data().y_axis_acceleration();
            sensors.accelerometer_z_force_millig = \
                proto_sensors.accelerometer().boat_orientation_data().z_axis_acceleration();

            // Gyroscope
            sensors.gyroscope_x_velocity_millidegreesps = \
                proto_sensors.gyroscope().angular_motion_data().x_velocity();
            sensors.gyroscope_y_velocity_millidegreesps = \
                proto_sensors.gyroscope().angular_motion_data().y_velocity();
            sensors.gyroscope_z_velocity_millidegreesps = \
                proto_sensors.gyroscope().angular_motion_data().z_velocity();

            // BMS (Handle separately from other sensors for low power control)
            std_msgs::Float32 min_voltage_msg;
            std::vector<float> voltages;
            float bms1_V = proto_sensors.bms_1().battery_pack_data().battery_voltage();
            float bms2_V = proto_sensors.bms_2().battery_pack_data().battery_voltage();
            float bms3_V = proto_sensors.bms_3().battery_pack_data().battery_voltage();
            float bms4_V = proto_sensors.bms_4().battery_pack_data().battery_voltage();
            float bms5_V = proto_sensors.bms_5().battery_pack_data().battery_voltage();
            float bms6_V = proto_sensors.bms_6().battery_pack_data().battery_voltage();
            std::cout << "BMS Voltages: " << std::endl;
            std::cout << bms1_V << std::endl;
            std::cout << bms2_V << std::endl;
            std::cout << bms3_V << std::endl;
            std::cout << bms4_V << std::endl;
            std::cout << bms5_V << std::endl;
            std::cout << bms6_V << std::endl;
            voltages.push_back(bms1_V > 0 ? bms1_V : MAX_BMS_VOLTAGE);
            voltages.push_back(bms2_V > 0 ? bms2_V : MAX_BMS_VOLTAGE);
            voltages.push_back(bms3_V > 0 ? bms3_V : MAX_BMS_VOLTAGE);
            voltages.push_back(bms4_V > 0 ? bms4_V : MAX_BMS_VOLTAGE);
            voltages.push_back(bms5_V > 0 ? bms5_V : MAX_BMS_VOLTAGE);
            voltages.push_back(bms6_V > 0 ? bms6_V : MAX_BMS_VOLTAGE);
            std::vector<float>::iterator result = std::min_element(voltages.begin(), voltages.end());
            min_voltage_msg.data = *result;
            if (*result == MAX_BMS_VOLTAGE)
                std::cout << \
                    "Warning, BMS voltages are outside expected range. Either an error or shore power" << std::endl;
            else
                std::cout << "Min voltage: " << *result << std::endl;

            sensors_pub.publish(sensors);
            ais_msg_pub.publish(ais_msg);
            waypoint_msg_pub.publish(waypoint_msg);
            min_voltage_msg_pub.publish(min_voltage_msg);

            std::cout << "Publishing sensor data. ex: wind_sensor_1 speed: " \
                << sensors.wind_sensor_1_speed_knots << std::endl;

        } else {
            std::cout << "Failed to send ros message" << std::endl;
        }
    }
}

void PrintUsage() {
    std::cout << "Provide the ip address and port to connect to. ex\n"
        "./nuc_eth_listener 192.168.1.60 5555" << std::endl;
}

/*
 * Background thread will subscribe to actuation_angle
 * and send any updates to the BBB.
 * Main thread receives sensor data from BBB and
 * publishes it to a ROS topic.
 */
int main(int argc, char** argv) {
    if (argc != 3) {
        PrintUsage();
        return 0;
    }

    ros::init(argc, argv, "nuc_eth_listener");
    ros::NodeHandle n;

    try {
        eth_socket.connect("tcp://" + std::string(argv[1]) + ":" + argv[2]);
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
        PrintUsage();
        return 0;
    }
    std::thread publish_sensor_data(PublishSensorData);

    sensors_pub = n.advertise<sailbot_msg::Sensors>(ROS_SENSORS_NODE, 100);
    ais_msg_pub = n.advertise<sailbot_msg::AISMsg>(ROS_AIS_NODE, 100);
    waypoint_msg_pub = n.advertise<sailbot_msg::path>(ROS_WAYPOINTS_NODE, 100);
    min_voltage_msg_pub = n.advertise<std_msgs::Float32>(ROS_MIN_VOLTAGE_NODE, 100);

    /* Because of how the manual override suspends regular operation, we have to set the
       subscription queue to a size of 1 so that very old angles are not used. */
    nt_sub = n.subscribe(ROS_ACTUATION_NODE, 1, ActuationCallBack);
    manual_override_sub = n.subscribe(ROS_MANUAL_OVERRIDE_NODE, 1, ManualOverrideCallback);
    // nt_sub = n.subscribe(POWER_CONTROLLER, 100, PowerControllerCallBack);
    ros::spin();

    publish_sensor_data.join();
}
