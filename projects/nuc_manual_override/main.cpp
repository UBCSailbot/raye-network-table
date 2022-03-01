// Copyright 2017 UBC Sailbot

#include <thread>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <getopt.h>
#include "Help.h"
#include "Controller.pb.h"
#include "sailbot_msg/manual_override.h"
#include "Uri.h"

#define ROS_MANUAL_OVERRIDE_NODE "/manual_override"
#define DEFAULT_TEST_NODE  "/manual_override_test"

// Limits for the rudders, winch, and jib
#define PI                          3.14159265358979323846
#define MAX_RUDDER_ANGLE_RAD        PI / 4
#define MAX_WINCH_JIB_POS           360

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
        std::size_t size, size_2;
        double rudder_angle_radians = std::stod(angles, &size);
        if (rudder_angle_radians > MAX_RUDDER_ANGLE_RAD) {
            std::cerr << "WARNING: rudder angle input greater than max (PI/4), sending PI/4 instead." << std::endl;
            rudder_angle_radians = MAX_RUDDER_ANGLE_RAD;
        } else if (rudder_angle_radians < -MAX_RUDDER_ANGLE_RAD) {
            std::cerr << "WARNING: rudder angle input less than min (-PI/4), sending PI/4 instead." << std::endl;
            rudder_angle_radians = -MAX_RUDDER_ANGLE_RAD;
        }
        // Gets second number in string.
        int sail_winch_position = std::stoi(angles.substr(size), &size_2);
        if (sail_winch_position > MAX_WINCH_JIB_POS) {
            std::cerr << "WARNING: winch position input greater than max " << MAX_WINCH_JIB_POS << ", sending " <<
                MAX_WINCH_JIB_POS << " instead." << std::endl;
            sail_winch_position = MAX_WINCH_JIB_POS;
        } else if (sail_winch_position < 0) {
            std::cerr << "WARNING: winch position less than minimum 0, sending 0 instead." << std::endl;
            sail_winch_position = 0;
        }
        // Gets third number.
        int jib_winch_position = std::stoi(angles.substr(size + size_2));
        if (jib_winch_position > MAX_WINCH_JIB_POS) {
            std::cerr << "WARNING: jib position input greater than max " << MAX_WINCH_JIB_POS << ", sending " <<
                MAX_WINCH_JIB_POS << " instead." << std::endl;
            sail_winch_position = MAX_WINCH_JIB_POS;
        } else if (jib_winch_position < 0) {
            std::cerr << "WARNING: jib position less than minimum 0, sending 0 instead." << std::endl;
            jib_winch_position = 0;
        }

        sailbot_msg::manual_override ros_manual_override;
        ros_manual_override.rudder_angle_radians = rudder_angle_radians;
        ros_manual_override.sail_winch_position = sail_winch_position;
        ros_manual_override.jib_winch_position = jib_winch_position;
        ros_manual_override.manual_override_active = true;

        std::cout << "Setting rudder angle to: " << rudder_angle_radians
        << " radians, sail position to: " << sail_winch_position << ", set jib position to: "
        << jib_winch_position << std::endl;

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
    std::string prompt =    "Manually enter rudder angle (radians) and sail position and winch position, "
                            "or \"stop\" to suspend manual override:";
    std::cout << prompt << std::endl;
    while (ros::ok()) {
        std::string input;

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
