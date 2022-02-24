#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "Uri.h"
#include "uccm-sensors/frame_parser.h"

#define CAN_DLC 8 // This must match what's in bbb_canbus_listener

int s;

void WinchJibCMD(void) {
    std::string help = "Control the winch or jib's angle (16 bit unsigned int). \n \
                        [h]         Help\n \
                        [w]         Set the target to the winch\n \
                        [w <x>]     Set the winch angle to x\n \
                        [j]         Set the target to the jib\n \
                        [j <x>]     Set the jib angle to x\n \
                        [<x>]       Set the current target to angle x (default winch)\n \
                        [q]         Quit winch and jib commands\n";
    std::cout << help;
    std::string input = "";
    struct can_frame frame;
    frame.can_dlc = CAN_DLC;
    frame.can_id = WINCH_MAIN_CMD_FRAME_ID;
    uint16_t angle;

    while (true) {
winch_jib_cmd_start:
        std::getline(std::cin, input);
        if (input.length() == 0) {
            std::cout << "Error, empty input" << std::endl;
            continue;
        }
        try {
            switch(input[0]) {
                case 'h':
                    std::cout << help;
                    goto winch_jib_cmd_start;
                case 'w':
                    frame.can_id = WINCH_MAIN_CMD_FRAME_ID;
                    if (input.length() > 2)
                        angle = std::stoi(input.substr(2));
                    else
                        goto winch_jib_cmd_start;
                    break;
                case 'j':
                    frame.can_id = WINCH_JIB_CMD_FRAME_ID;
                    if (input.length() > 2)
                        angle = std::stoi(input.substr(2));
                    else
                        goto winch_jib_cmd_start;
                    break;
                case 'q':
                    std::cout << "Quitting winch and jib command interface" << std::endl;
                    return;
                default:
                    angle = stoi(input);
                frame.data[0] = angle & 0xFF;
                frame.data[1] = (angle >> 8) & 0xFF;
                std::cout << "Setting winch/jib angle: " << angle << std::endl;
                if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                    perror("Write");
                    return;
                }
            }
        } catch (const std::invalid_argument& ia) {
            std::cout << "Invalid angle inputted" << std::endl;
        }
    }
}

void RudderCMD(void) {
    std::string help = "Control the port or starboard rudder's angle (float). \n \
                        [h]         Help\n \
                        [p]         Set the current rudder to port\n \
                        [p <x>]     Set the port rudder to angle x\n \
                        [s]         Set the current rudder to starboard\n \
                        [s <x>]     Set the starboard rudder to angle x\n \
                        [<x>]       Set the current rudder to angle x (default port)\n \
                        [q]         Quit rudder commands\n";
    std::cout << help;
    std::string input = "";
    struct can_frame frame;
    frame.can_dlc = CAN_DLC;
    frame.can_id = RUDDER_PORT_CMD_FRAME_ID;
    float angle;

    while (true) {
rudder_CMD_start:
        std::getline(std::cin, input);
        if (input.length() == 0) {
            std::cout << "Error, empty input" << std::endl;
            continue;
        }
        try {
            switch(input[0]) {
                case 'h':
                    std::cout << help;
                    goto rudder_CMD_start;
                case 'p':
                    frame.can_id = RUDDER_PORT_CMD_FRAME_ID;
                    if (input.length() > 2) 
                        angle = std::stof(input.substr(2));
                    else
                        goto rudder_CMD_start;
                    break;
                case 's':
                    frame.can_id = RUDDER_STBD_CMD_FRAME_ID;
                    if (input.length() > 2)
                        angle = std::stof(input.substr(2));
                    else
                        goto rudder_CMD_start;
                    break;
                case 'q':
                    std::cout << "Quitting rudder interface" << std::endl;
                    return;
                default:
                    angle = std::stof(input);
                uint8_t const *angle_array = reinterpret_cast<uint8_t *>(&angle);
                frame.data[0] = angle_array[0];
                frame.data[1] = angle_array[1];
                frame.data[2] = angle_array[2];
                frame.data[3] = angle_array[3];
                std::cout << "Sending rudder angle: " << angle << std::endl;
                if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                    perror("Write");
                    return;
                }
            }
        } catch (const std::invalid_argument& ia) {
            std::cout << "Invalid angle inputted" << std::endl;
        }
    }
}

void NavLightCMD(void) {
    std::string help = "Control the nav light on the mast.\n \
                        [h]         Help\n \
                        [o]         Turn the light on constantly\n \
                        [f <f/s>]   Flash the light fast (period of 0.5s) or slow (period of 2s)\n \
                        [s]         Stop the light and turn it off\n \
                        [q]         Quit nav light commands\n";
    std::cout << help;
    std::string input = "";
    struct can_frame frame;
    frame.can_dlc = CAN_DLC;
    frame.can_id = NAV_LIGHT_FRAME_ID;
    bool quit = false;
    while (!quit) {
nav_light_cmd_start:
        std::getline(std::cin, input);
        if (input.length() == 0) {
            std::cout << "Error, empty input" << std::endl;
            continue;
        }
        switch(input[0]) {
            case 'h':
                std::cout << help;
                goto nav_light_cmd_start;
            case 'f':
                if (input.length() < 3) {
                    std::cout << "Error, flash needs to either be fast <f> or slow <s>" << std::endl;
                    goto nav_light_cmd_start;
                }
                if (input[3] == 'f')
                    frame.data[1] = 0x00;
                else if (input[3] == 's')
                    frame.data[1] = 0x20;
                else 
                    std::cout << "Error, flash needs to either be fast <f> or slow <s>" << std::endl;
                frame.data[0] = 1;
                break;
            case 'o':
                frame.data[1] = 0x10;
                frame.data[0] = 1;
                break;
            case 'q':
                quit = true;
                std::cout << "Quitting nav light interface" << std::endl;
            case 's':
                frame.data[0] = 0;
                break;
            default:
                std::cout << "Error, flash needs to either be fast <f> or slow <s>" << std::endl;
        }
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return;
        }
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "Please provide the name of the canbus interface." << std::endl;
        std::cout << "Example usage: './bbb_uccm_command_interface can0'" << std::endl;
        return 0;
    }
    std::cout << "Running bbb_uccm_command_interface" << std::endl;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout << "Error while opening socket" << std::endl;
        return -1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;

    char *ifname = argv[1];

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        std::cout << "Error in socket bind" << std::endl;
        return -2;
    }

    std::string help = "Select the CAN device you want to interface with:\n \
                        [h]     Help\n \
                        [n]     Nav light\n \
                        [r]     Rudder\n \
                        [w]     Winch and jib\n \
                        [q]     Quit interface\n";
    std::cout << help;
    std::string input = "";
    while (true) {
        std::getline(std::cin, input);
        if (input.length() == 0) {
            std::cout << "Error, empty input" << std::endl;
            continue;
        }
        switch(input[0]) {
            case 'h':
                std::cout << help;
                break;
            case 'n':
                NavLightCMD();
                break;
            case 'r':
                RudderCMD();
                break;
            case 'w':
                WinchJibCMD();
                break;
            case 'q':
                std::cout << "Exiting interface" << std::endl;
                return 0;
            default:
                std::cout << "Error, invalid input" << std::endl;
        }
    }
    return 0;
}