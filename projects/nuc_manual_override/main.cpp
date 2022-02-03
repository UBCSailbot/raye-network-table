// Copyright 2017 UBC Sailbot

#include <thread>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <getopt.h>
#include "Help.h"
#include "ActuationAngle.pb.h"
#include "sailbot_msg/manual_override.h"
#include "Uri.h"

#define ROS_MANUAL_OVERRIDE_NODE "/manual_override"
#define DEFAULT_TEST_NODE  "/manual_override_test"

/* 
 * When testing without the network controller, we want to publish to
 * "/rudder_winch_actuation_angle" instead of "/manual_overwride". To
 * make sure the nuc_eth_listener reads from "/rudder_winch_actuation_angle"
 * in this case, the override_mask makes sure that manual_override_flag
 * is always false.  
 */
ros::Publisher manual_override_pub;

void PublishManualOverride(std::string angles) {
    try {
        std::size_t size;
        double rudder_angle_degrees = std::stod(angles, &size);
        // Gets second number in string.
        double abs_sail_angle_degrees = std::stod(angles.substr(size));

        sailbot_msg::manual_override ros_manual_override;
        ros_manual_override.rudder_angle_degrees = rudder_angle_degrees;
        ros_manual_override.abs_sail_angle_degrees = abs_sail_angle_degrees;
        // Nav-247: Add something to deal with jib angle
        ros_manual_override.manual_override_active = true;

        std::cout << "Setting rudder angle to: " << rudder_angle_degrees
        << " degrees, sail angle to: " << abs_sail_angle_degrees << " degrees." << std::endl;

        manual_override_pub.publish(ros_manual_override);
    } catch (const std::invalid_argument& ia) {
        std::cout << "Invalid input: " << angles << std::endl;
    }
}

void StopManualOverride() {
    sailbot_msg::manual_override ros_manual_override;
    ros_manual_override.manual_override_active = false;
    manual_override_pub.publish(ros_manual_override);
}

void PublishManualOverrideLoop() {
    while (ros::ok()) {
        std::string input;

        std::cout << "Manually enter rudder angle and sail angle (degrees), "
                    "or \"stop\" to suspend manual override:" << std::endl;

        std::getline(std::cin, input);

        if (input == "stop") {
            std::cout << "Manual override suspended." << std::endl;
            StopManualOverride();
        } else if (input == "exit" || input == "quit" || input == "e" || input == "q") {
            std::cout << "Exiting manual override."<< std::endl;
            StopManualOverride();
            break;
        } else {
            PublishManualOverride(input);
        }
    }
}

void PrintUsage(void) {
    std::cout << "Run the program with or without arguments.\n"
        "./nuc_manual_override [-h] [-o \"Angles\"] [-t<ROS_NODE>]\n"
        "Ex. ./nuc_manual_override\n"
        "See README for more details." << std::endl;
}
int main(int argc, char** argv) {
    std::cout << "Starting manual override." << std::endl;
    std::string topic = ROS_MANUAL_OVERRIDE_NODE;
    int opt;
    bool once = false;
    std::string msg_once;

    static struct option long_options[] = {
        {"help", no_argument,       NULL, 'h'},
        {"once", required_argument, NULL, 'o'},  // Send angle once
        {"test", optional_argument, NULL, 't'},  // Send angle to a test ROS node
        {NULL,   0,                 NULL,   0}   // Required to have all zeros.
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
                break;
            case '?':
                std::cout << "Unknown option: " << optopt << std::endl;
                PrintUsage();
                break;  // Continue parsing even with unknown option.
            case ':':  // When a required argument is missing.
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

    manual_override_pub = n.advertise<sailbot_msg::manual_override>(topic, 1);

    if (once) {
        // ROS is quite finicky when dealing with just one message.
        PublishManualOverride(msg_once);
        std::thread PublishWaitThread([]{usleep(1000000);});
        PublishWaitThread.join();
        // Repeat the function call to clear the ROS queue.
        PublishManualOverride(msg_once);
        StopManualOverride();
        PublishWaitThread = std::thread([]{usleep(1000000);});
        PublishWaitThread.join();
        // Calling the next function is just to clear the ROS queue.
        StopManualOverride();
    } else {
        PublishManualOverrideLoop();
    }
    std::cout << "Manual override program exited." << std::endl;
    return 0;
}
