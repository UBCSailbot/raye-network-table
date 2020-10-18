// Copyright 2017 UBC Sailbot

#include <zmq.hpp>
#include <thread>

#include <ros/ros.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "Node.pb.h"
#include "Sensors.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "sailbot_msg/Sensors.h"
#include "sailbot_msg/AISMsg.h"
#include "sailbot_msg/AISShip.h"

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
ros::Subscriber nt_sub;
ros::Publisher sensors_pub;
ros::Publisher ais_msg_pub;

void ActuationCallBack(const sailbot_msg::actuation_angle ros_actuation_angle) {
    /*
     * Receive motor actuation data and send it
     * to the BBB.
     */
    if (ros::ok()) {
        // Both angles are in radians (CONFIRM WITH BRUCE)
        NetworkTable::ActuationAngle nt_actuation_angle;
        nt_actuation_angle.set_rudder_angle(ros_actuation_angle.rudder);
        nt_actuation_angle.set_winch_angle(ros_actuation_angle.winch);

        std::string serialized_nt_actuation_angle;
        nt_actuation_angle.SerializeToString(&serialized_nt_actuation_angle);

        std::cout << "Sending rudder angle: " << nt_actuation_angle.rudder_angle() \
            << " winch angle: " << nt_actuation_angle.winch_angle() << std::endl;

        send(serialized_nt_actuation_angle);
    } else {
        std::cout << "Failed to receive actuation angle" << std::endl;
    }
}

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

            auto proto_ais_Allboats = node.children().at("ais").children().at("boats").value().boats();
            sailbot_msg::AISMsg ais_msg;
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

            node.ParseFromString(message_serialized);
            NetworkTable::Sensors proto_sensors = NetworkTable::RootToSensors(&node);
            sailbot_msg::Sensors sensors;
            sensors.boom_angle_sensor_angle = proto_sensors.boom_angle_sensor().sensor_data().angle();
            sensors.wind_sensor_0_speed = proto_sensors.wind_sensor_0().iimwv().wind_speed();
            sensors.wind_sensor_0_direction = proto_sensors.wind_sensor_0().iimwv().wind_direction();
            sensors.wind_sensor_0_reference = proto_sensors.wind_sensor_0().iimwv().wind_reference();
            sensors.wind_sensor_1_speed = proto_sensors.wind_sensor_1().iimwv().wind_speed();
            sensors.wind_sensor_1_direction = proto_sensors.wind_sensor_1().iimwv().wind_direction();
            sensors.wind_sensor_1_reference = proto_sensors.wind_sensor_1().iimwv().wind_reference();
            sensors.wind_sensor_2_speed = proto_sensors.wind_sensor_2().iimwv().wind_speed();
            sensors.wind_sensor_2_direction = proto_sensors.wind_sensor_2().iimwv().wind_direction();
            sensors.wind_sensor_2_reference = proto_sensors.wind_sensor_2().iimwv().wind_reference();
            sensors.gps_0_timestamp = proto_sensors.gps_0().gprmc().utc_timestamp();
            sensors.gps_0_latitude = proto_sensors.gps_0().gprmc().latitude();
            sensors.gps_0_longitude = proto_sensors.gps_0().gprmc().longitude();
            sensors.gps_0_latitude_loc = proto_sensors.gps_0().gprmc().latitude_loc();
            sensors.gps_0_longitude_loc = proto_sensors.gps_0().gprmc().longitude_loc();
            sensors.gps_0_groundspeed = proto_sensors.gps_0().gprmc().ground_speed();
            sensors.gps_0_track_made_good = proto_sensors.gps_0().gprmc().track_made_good();
            sensors.gps_0_magnetic_variation = proto_sensors.gps_0().gprmc().magnetic_variation();
            sensors.gps_0_magnetic_variation_sense = proto_sensors.gps_0().gprmc().magnetic_variation_sense();
            sensors.gps_1_timestamp = proto_sensors.gps_1().gprmc().utc_timestamp();
            sensors.gps_1_latitude = proto_sensors.gps_1().gprmc().latitude();
            sensors.gps_1_longitude = proto_sensors.gps_1().gprmc().longitude();
            sensors.gps_1_latitude_loc = proto_sensors.gps_1().gprmc().latitude_loc();
            sensors.gps_1_longitude_loc = proto_sensors.gps_1().gprmc().longitude_loc();
            sensors.gps_1_groundspeed = proto_sensors.gps_1().gprmc().ground_speed();
            sensors.gps_1_track_made_good = proto_sensors.gps_1().gprmc().track_made_good();
            sensors.gps_1_magnetic_variation = proto_sensors.gps_1().gprmc().magnetic_variation();
            sensors.gps_1_magnetic_variation_sense = proto_sensors.gps_1().gprmc().magnetic_variation_sense();
            sensors.accelerometer_x_axis_acceleration = \
                proto_sensors.accelerometer().boat_orientation_data().x_axis_acceleration();
            sensors.accelerometer_y_axis_acceleration = \
            proto_sensors.accelerometer().boat_orientation_data().y_axis_acceleration();
            sensors.accelerometer_z_axis_acceleration = \
            proto_sensors.accelerometer().boat_orientation_data().z_axis_acceleration();

            sensors_pub.publish(sensors);
            ais_msg_pub.publish(ais_msg);

            std::cout << "Publishing sensor data. ex: wind_sensor_0 speed: " \
                << sensors.wind_sensor_0_speed << std::endl;

        } else {
            std::cout << "Failed to send ros message" << std::endl;
        }
    }
}

void PrintUsage() {
    std::cout << "Provide the ip address and port to connect to. ex\n"
        "./nuc_eth_listener 10.0.0.8 5555" << std::endl;
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

    sensors_pub = n.advertise<sailbot_msg::Sensors>("sensors", 100);
    ais_msg_pub = n.advertise<sailbot_msg::AISMsg>("ais_msg", 100);

    nt_sub = n.subscribe("/actuation_angles", 100, ActuationCallBack);
    ros::spin();

    publish_sensor_data.join();
}
