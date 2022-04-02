/**
*
*  Copyright 2017 UBC Sailbot
* 
*  @file  bbb_ais_listener.cpp
*  @brief Facilitates communication between AIS and BBB Network Table
*  
*  @author Alex Macdonald (Alexmac22347)
*  @author Rick Cheng (yd-cheng)
*
*/

// Standard header files
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

// Ais header files
#include "Boat.h"
#include "Connection.h"
#include "AisControllerConnection.h"
#include "Value.pb.h"
#include "Uri.h"
#include "Exceptions.h"

#define SOCKET_ENDPOINT "ipc:///tmp/sailbot/AisReceiverControllerQuery"
#define DEFAULT_SLEEP 10  // sleep in seconds
#define RAYE_MMSI 316046014


// Testing purposes
void printBoats(std::list<Boat> ais_boats) {
    int i = 0;
    for (Boat b : ais_boats) {
        std::cout << i << ":  " << b.m_transcieverClass \
            << " | " << b.m_mmsi << " | " << b.m_navigationStatus \
            << " | " << b.m_rateOfTurn << " | " << b.m_latitude \
            << " | " << b.m_longitude << " | " << b.m_cog \
            << " | " << b.m_trueHeading << " | " << b.m_maneuverIndicator \
            << " | " << b.m_rateOfTurnValid;
        std::cout << std::endl;
        i++;
    }
}

// Converts boat.h object to proto boat
void ConvertToProto(Boat boat, NetworkTable::Value::Boat* proto_boat) {
    // std::time_t is already a long (int64) representing # of seconds since Unix epoch
    std::chrono::steady_clock::time_point timePoint = boat.m_timeReceived;
    std::time_t timestamp = std::chrono::system_clock::to_time_t(\
            std::chrono::system_clock::now() + (timePoint - std::chrono::steady_clock::now()));

    // Setting proto_boat variables
    proto_boat->set_m_mmsi(boat.m_mmsi);
    proto_boat->set_m_navigationstatus(boat.m_navigationStatus);
    proto_boat->set_m_rateofturn(boat.m_rateOfTurn);
    proto_boat->set_m_highaccuracy(boat.m_highAccuracy);
    proto_boat->set_m_latitude(boat.m_latitude);
    proto_boat->set_m_longitude(boat.m_longitude);
    proto_boat->set_m_sog(boat.m_sog);
    proto_boat->set_m_cog(boat.m_cog);
    proto_boat->set_m_trueheading(boat.m_trueHeading);
    proto_boat->set_m_timestamp(boat.m_timeStamp);  // Alan said DO NOT USE
    proto_boat->set_m_maneuverindicator(boat.m_maneuverIndicator);
    proto_boat->set_m_timereceived(timestamp);

    // Setting proto_boat valid flags
    proto_boat->set_m_rateofturnvalid(boat.m_rateOfTurnValid);
    proto_boat->set_m_sogvalid(boat.m_sogValid);
    proto_boat->set_m_cogvalid(boat.m_cogValid);
    proto_boat->set_m_trueheadingvalid(boat.m_trueHeadingValid);
    proto_boat->set_m_positionvalid(boat.m_positionValid);
    proto_boat->set_m_timestampvalid(boat.m_timeStampValid);
    proto_boat->set_m_transcieverclass(boat.m_transcieverClass);
}


int main(int argc, char** argv) {
    // Attempt connection to network table
    NetworkTable::Connection connection;
    try {
        connection.Connect(-1);
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to server" << std::endl;
        return 0;
    }

    // If user supplied sleep argument from command line in seconds, will override the default of 10 seconds
    int sleep_time = DEFAULT_SLEEP;  // Adjustable loop sleep time in seconds
    if (argc == 2) {
        try {
            sleep_time = std::stoi(argv[1]);
        } catch (std::invalid_argument) {
            std::cout << "Supplied sleep argument must be integer" << std::endl;
            std::cout << "Setting loop sleep time to default of 10 seconds" << std::endl << std::endl;
        }
    }

    // Create ais_connection object that connects to the endpoint specified in ais README.md
    zmq::context_t ctx;
    AisControllerConnection ais_connection(ctx, SOCKET_ENDPOINT);

    // Create NetworkTable::Value object to store all proto_boats
    NetworkTable::Value proto_boats;
    proto_boats.set_type(NetworkTable::Value::BOATS);

    std::cout << "Starting loop" << std::endl;

    // Main loop to continuously request list of boats from ais_connection
    std::list<Boat> ais_boats;  // Stores boat query from ENDPOINT
    while (true) {
        std::cout << "looped" << std::endl;
        ais_boats = ais_connection.getBoats();  // Query list of boats from zmq socket

        // Clear all proto_boats so we don't have extra/repeated boats after each ais_connection query
        proto_boats.clear_boats();

        // std::cout << "raye's mmsi is: " << RAYE_MMSI << std::endl;

        for (Boat b : ais_boats) {
            // std::cout << "current boat mmsi is: " << b.m_mmsi << std::endl;
            if (b.m_mmsi == RAYE_MMSI) {
                // std::cout << "setting RAYE GPS Values in Network Table" << std::endl;
                // std::cout << "raye's latitude is: " << b.m_latitude << std::endl;
                // std::cout << "raye's longitude is: " << b.m_longitude << std::endl;

                // map to format the gps coordinates for connection.SetValues()
                std::map<std::string, NetworkTable::Value> values;

                // Creates and configures NetworkTable values for insertion
                NetworkTable::Value ais_gps_latitude;
                NetworkTable::Value ais_gps_longitude;

                double raye_latitude = b.m_latitude;
                double raye_longitude = b.m_longitude;

                ais_gps_latitude.set_type(NetworkTable::Value::FLOAT);
                ais_gps_longitude.set_type(NetworkTable::Value::FLOAT);

                ais_gps_latitude.set_float_data(static_cast<float>(raye_latitude));
                ais_gps_longitude.set_float_data(static_cast<float>(raye_longitude));

                // Formats the Networktable::Value in the values map
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_AIS_LAT, ais_gps_latitude));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_AIS_LON, ais_gps_longitude));

                // Sets the AIS_GPS values
                try {
                    connection.SetValues(values);
                } catch (NetworkTable::NotConnectedException) {
                    std::cout << "Failed to set value" << std::endl;
                } catch (NetworkTable::TimeoutException) {
                    std::cout << "Timeout" << std::endl;
                }
            } else {
                // std::cout << "converted boat that isin't raye to proto" << std::endl;
                ConvertToProto(b, proto_boats.add_boats());
            }
        }
        printBoats(ais_boats);

        // Write proto_boats to network table
        std::map<std::string, NetworkTable::Value> values;
        values.insert(std::pair<std::string, NetworkTable::Value>("ais/boats", proto_boats));
        connection.SetValues(values);

        sleep(sleep_time);  // Sleep so we don't query too often
    }

    return 0;
}
