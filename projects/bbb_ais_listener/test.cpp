// Copyright 2017 UBC Sailbot
// Standard header files
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <vector>

// Ais header files
#include "Boat.h"
#include "zmq.hpp"

#define SOCKET_ENDPOINT "ipc:///tmp/sailbot/AisReceiverControllerQuery"

int num_boats;
int num_loop;
std::vector<Boat*> boat_vector;

// Prints out the generated test boats to reference during testing
void printBoats() {
    int i = 0;
    for (Boat* b : boat_vector) {
        std::cout << i << ":  " << b->m_transcieverClass \
            << " | " << b->m_mmsi << " | " << b->m_navigationStatus \
            << " | " << b->m_rateOfTurn << " | " << b->m_latitude \
            << " | " << b->m_longitude << " | " << b->m_cog \
            << " | " << b->m_trueHeading << " | " << b->m_maneuverIndicator \
            << " | " << b->m_rateOfTurnValid;

        std::cout << std::endl;
        i++;
    }
}


int main(int argc, char** argv) {
    std::cout << "Enter number of boats to send through socket: ";
    std::cin >> num_boats;
    std::cout << std::endl << "Enter number of times boats are sent through socket: ";
    std::cin >> num_loop;

    zmq::context_t ctx;
    zmq::socket_t *m_socket = new zmq::socket_t(ctx, ZMQ_REP);
    m_socket->bind(SOCKET_ENDPOINT);

    zmq::message_t request;

    for (int i = 0; i < num_loop; i++) {
        // recv blocking so program will only generate test boats when requested by query from bbb_ais_listener main
        m_socket->recv(&request);

        // Generate boats with random mmsi numbers
        for (int j = 0; j < num_boats; j++) {
            // Used for testing both true and false for boolean fields
            bool bool_test = j % 2;  // Flips between true and false

            // Using no parameter constructor easier to follow which field is set to what
            Boat *boat = new Boat();

            // Filling new boat member fields
            boat->m_transcieverClass = static_cast<Boat::TranscieverClass>(j % 3);  // enum goes from 0->2
            boat->m_mmsi = j;
            boat->m_navigationStatus = static_cast<Boat::NavigationStatus>(j % 16);  // enum goes from 0->15
            boat->m_rateOfTurn = static_cast<float>(j);
            boat->m_highAccuracy = bool_test;  // Test both true and false
            boat->m_latitude = static_cast<double>(j);
            boat->m_longitude = static_cast<double>(j);
            boat->m_sog = static_cast<float>(j);
            boat->m_cog = static_cast<float>(j);
            boat->m_trueHeading = j;
            boat->m_timeStamp = j;
            boat->m_maneuverIndicator = static_cast<Boat::ManeuverIndicator>(j % 3);  // enum goes from 0->2
            boat->m_timeReceived = std::chrono::steady_clock::now();  // idk what this means
            boat->m_rateOfTurnValid = bool_test;
            boat->m_sogValid = bool_test;
            boat->m_trueHeadingValid = bool_test;
            boat->m_positionValid = bool_test;
            boat->m_timeStampValid = bool_test;

            boat_vector.push_back(boat);
        }

        size_t boat_buff_size = sizeof(Boat) * boat_vector.size();
        Boat *boat_buff = static_cast<Boat*>(malloc(boat_buff_size));
        zmq::message_t reply(boat_buff_size);

        // Copy all boats to boat_buff from generated boat_vector
        for (unsigned int j = 0; j < boat_vector.size(); j++) {
            boat_buff[j] = *(boat_vector[j]);
        }

        memcpy(reply.data(), boat_buff, boat_buff_size);
        m_socket->send(reply);  // Send reply to socket

        printBoats();
    }

    return 0;
}
