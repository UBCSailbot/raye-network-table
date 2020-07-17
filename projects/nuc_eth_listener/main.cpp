// Copyright 2017 UBC Sailbot

#include <zmq.hpp>
#include <thread>

#include "ros/ros.h"
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "Node.pb.h"
#include "Sensors.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "sailbot_msg/windSensor.h"
#include "sailbot_msg/GPS.h"

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
ros::Publisher gps_0_pub;
ros::Publisher gps_1_pub;
ros::Publisher wind_sensor_0_pub;
ros::Publisher wind_sensor_1_pub;
ros::Publisher wind_sensor_2_pub;

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
            node.ParseFromString(message_serialized);

            sailbot_msg::windSensor wind_sensor_0;
            sailbot_msg::windSensor wind_sensor_1;
            sailbot_msg::windSensor wind_sensor_2;
            sailbot_msg::GPS gps_0;
            sailbot_msg::GPS gps_1;
            NetworkTable::Sensors proto_sensors = NetworkTable::RootToSensors(&node);

            gps_0.lat = proto_sensors.gps_0().gprmc().latitude();
            gps_0.lon = proto_sensors.gps_0().gprmc().longitude();
            gps_0.speedKmph = proto_sensors.gps_0().gprmc().ground_speed();
            gps_1.lat = proto_sensors.gps_1().gprmc().latitude();
            gps_1.lon = proto_sensors.gps_1().gprmc().longitude();
            gps_1.speedKmph = proto_sensors.gps_1().gprmc().ground_speed();

            wind_sensor_0.measuredDirectionDegrees = proto_sensors.wind_sensor_0().iimwv().wind_direction();
            wind_sensor_0.measuredSpeedKmph = proto_sensors.wind_sensor_0().iimwv().wind_speed();
            wind_sensor_1.measuredDirectionDegrees = proto_sensors.wind_sensor_0().iimwv().wind_direction();
            wind_sensor_1.measuredSpeedKmph = proto_sensors.wind_sensor_1().iimwv().wind_speed();
            wind_sensor_2.measuredDirectionDegrees = proto_sensors.wind_sensor_2().iimwv().wind_direction();
            wind_sensor_2.measuredSpeedKmph = proto_sensors.wind_sensor_2().iimwv().wind_speed();

            // TODO(anyone): still need accelerometer and boom angle sensor

            std::cout << "Publishing sensor data. ex: wind_sensor_0 speed: " \
                << wind_sensor_0.measuredSpeedKmph << std::endl;

            gps_0_pub.publish(gps_0);
            gps_1_pub.publish(gps_1);
            wind_sensor_0_pub.publish(wind_sensor_0);
            wind_sensor_1_pub.publish(wind_sensor_1);
            wind_sensor_2_pub.publish(wind_sensor_2);
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

    gps_0_pub = n.advertise<sailbot_msg::GPS>("gps_0", 100);
    gps_1_pub = n.advertise<sailbot_msg::GPS>("gps_1", 100);
    wind_sensor_0_pub = n.advertise<sailbot_msg::windSensor>("wind_sensor_0", 100);
    wind_sensor_1_pub = n.advertise<sailbot_msg::windSensor>("wind_sensor_1", 100);
    wind_sensor_2_pub = n.advertise<sailbot_msg::windSensor>("wind_sensor_2", 100);

    nt_sub = n.subscribe("/actuation_angles", 100, ActuationCallBack);
    ros::spin();

    publish_sensor_data.join();
}
