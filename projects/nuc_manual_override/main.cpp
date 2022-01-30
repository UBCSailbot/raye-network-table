// Copyright 2017 UBC Sailbot

#include <thread>
#include <iostream>
#include <string>
#include <ros/ros.h>
// #include <unistd.h>
#include <getopt.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "sailbot_msg/actuation_angle.h"
#include "Uri.h"

#define ROS_ACTUATION_NODE "/rudder_winch_actuation_angle"
#define DEFAULT_TEST_NODE  "/manual_override_test"

// This is a global variable read in "nuc_eth_listener.cpp".
bool manual_override_flag;

/* 
 * When testing without the network controller, we want to publish to
 * "/rudder_winch_actuation_angle" instead of "/manual_overwride". To
 * make sure the nuc_eth_listener reads from "/rudder_winch_actuation_angle"
 * in this case, the override_mask makes sure that manual_override_flag
 * is always false.  
 */
bool override_mask = true;
ros::Publisher manual_override_pub;

void PublishManualOverride(std::string angles) {
    try {
        std::size_t size;
        double rudder_angle_degrees = std::stod(angles, &size);
        // Gets second number in string.
        double abs_sail_angle_degrees = std::stod(angles.substr(size));

        sailbot_msg::actuation_angle ros_actuation_angle;
        ros_actuation_angle.rudder_angle_degrees = rudder_angle_degrees;
        ros_actuation_angle.abs_sail_angle_degrees = abs_sail_angle_degrees;

        std::cout << "Setting rudder angle to: " << rudder_angle_degrees
        << " degrees, sail angle to: " << abs_sail_angle_degrees << " degrees." << std::endl;
        manual_override_flag = true & override_mask;

        manual_override_pub.publish(ros_actuation_angle);
    } catch (const std::invalid_argument& ia) {
        std::cout << "Invalid input: " << angles << std::endl;
    }
}
void PublishManualOverrideLoop() {
    while (ros::ok()) {
        std::string input;

        std::cout << "Manually enter rudder angle and sail angle (degrees), "
                    "or \"stop\" to suspend manual override:" << std::endl;

        std::getline(std::cin, input);

        if (input == "stop") {
            manual_override_flag = false;
            std::cout << "Manual override suspended." << std::endl;
        } else if (input == "exit" || input == "quit" || input == "e" || input == "q") {
            manual_override_flag = false;
            std::cout << "Exiting manual override."<< std::endl;
            break;
        } else {
            PublishManualOverride(input);
        }
    }
}

void PrintUsage(void) {
    std::cout << "Placeholder" << std::endl;
}
int main(int argc, char** argv) {
    std::cout << "Starting manual override." << std::endl;
    std::string topic = ROS_ACTUATION_NODE;
    int opt;
    bool once = false;
    std::string msg_once = "";

    static struct option long_options[] = {
        {"help", no_argument,       NULL, 'h'},
        {"once", required_argument, NULL, 'o'}, // Send angle once
        {"test", optional_argument, NULL, 't'}, // Send angle to a test node that won't atually change rudder/winch angle
        {NULL,   0,                 NULL,   0}  // Required to have all zeros.
    };

    while ((opt = getopt_long(argc, argv, ":ho:t::", long_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                PrintUsage();
                exit(EXIT_SUCCESS);
            case 'o':
                msg_once = optarg;
                once = true;
                break;
            case 't':
                topic = optarg ? optarg : DEFAULT_TEST_NODE;
                override_mask = false;
                break;
            case '?':
                std::cout << "Unknown option: " << optopt << std::endl;
                PrintUsage();
                break; // Continue parsing even with unknown option.
            case ':': // When a required argument is missing.
                std::cout << "Missing argument for: " << optopt << std::endl;
                PrintUsage();
                exit(EXIT_FAILURE);
            default:
                std::cout << "Invalid input for " << optopt <<", exiting... " << std::endl;
                PrintUsage();
                exit(EXIT_FAILURE);
        }
    }
    std::cout << "Sending angles to topic: " << topic << \
    (once ? (" once with values " + msg_once) : " continuously") << std::endl;

    ros::init(argc, argv, "nuc_manual_override");

    ros::NodeHandle n;

    manual_override_pub = n.advertise<sailbot_msg::actuation_angle>(topic, 1);

    if (once)
        PublishManualOverride(msg_once);
    else
        PublishManualOverrideLoop();
    std::cout << "Manual override program exited." << std::endl;
    return 0;
}
