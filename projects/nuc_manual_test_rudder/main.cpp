// Copyright 2017 UBC Sailbot

#include <thread>
#include <iostream>

#include <ros/ros.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "Controller.pb.h"
#include "Uri.h"

ros::Publisher manual_override_pub;

void PublishManualOverride() {
    while (ros::ok()) {
        double rudder_angle_degrees, abs_sail_angle_degrees;

        std::cout << "Manually enter rudder angle and sail angle (degrees):" << std::endl;

        std::cin >> rudder_angle_degrees;
        std::cin >> abs_sail_angle_degrees;

        sailbot_msg::actuation_angle ros_actuation_angle;
        ros_actuation_angle.rudder_angle_degrees = rudder_angle_degrees;
        ros_actuation_angle.abs_sail_angle_degrees = abs_sail_angle_degrees;

        manual_override_pub.publish(ros_actuation_angle);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "nuc_manual_test_rudder");

    ros::NodeHandle n;

    manual_override_pub = n.advertise<sailbot_msg::actuation_angle>("rudder_winch_actuation_angle", 1);

    std::thread publish_manual_override(PublishManualOverride);

    // ros::spin();

    publish_manual_override.join();
}
