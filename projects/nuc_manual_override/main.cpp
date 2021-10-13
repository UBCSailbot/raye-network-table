// Copyright 2017 UBC Sailbot

#include <thread>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "Uri.h"

ros::Publisher manual_override_pub;

void PublishManualOverride() {
    while (ros::ok()) {
        std::string input;

        std::cout << "Manually enter rudder angle and sail angle (degrees), "
                    "or \"stop\" to suspend manual override:" << std::endl;

        std::getline(std::cin, input);

        if (input == "stop") {
            // set global manual override flag to false
            std::cout << "Manual override suspended." << std::endl;
        } else if (input == "exit" || input == "quit") {
            std::cout << "Exiting manual override."<< std::endl;
            break;
        } else {
            try {
                std::size_t size;
                double rudder_angle_degrees = std::stod(input, &size);
                // Gets second number in string.
                double abs_sail_angle_degrees = std::stod(input.substr(size));

                sailbot_msg::actuation_angle ros_actuation_angle;
                ros_actuation_angle.rudder_angle_degrees = rudder_angle_degrees;
                ros_actuation_angle.abs_sail_angle_degrees = abs_sail_angle_degrees;

                std::cout << "Setting rudder angle to: " << rudder_angle_degrees
                << " degrees, sail angle to: " << abs_sail_angle_degrees << " degrees." << std::endl;
                // set global manual override flag to true

                manual_override_pub.publish(ros_actuation_angle);
            } catch (const std::invalid_argument& ia) {
                std::cout << "Invalid input: " << input << std::endl;
            }
        }
    }
}


int main(int argc, char** argv) {
    std::string topic;
    if (argc == 2 && std::string(argv[1]) == "test") {
        topic = ACTUATION;
    } else {
        topic = "rudder_winch_actuation_angle";
    }
    ros::init(argc, argv, "nuc_manual_override");

    ros::NodeHandle n;

    manual_override_pub = n.advertise<sailbot_msg::actuation_angle>(topic, 1);

    std::thread publish_manual_override(PublishManualOverride);

    // ros::spin();

    publish_manual_override.join();

    return 0;
}
