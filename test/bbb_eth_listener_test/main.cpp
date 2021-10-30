/**
*
*  Copyright 2017 UBC Sailbot
* 
*  @file  bbb_eth_listener.cpp
*  @brief Facilitates communication between the NUC and BBB 
*         Network Table via ethernet connection
*  
*  Polls sensor data from the CANbus and publishes 
*  it to the network table. Writes actuation angles
*  outputted from the boat controller to the CANbus
*  
*  @author Alex Macdonald (Alexmac22347)
*  @author Brielle Law (briellelaw)
*  @author John Ahn (jahn18)
*
*  Copied from bbb_eth_listener
*/

#include "Connection.h"
#include "Controller.pb.h"
#include "Value.pb.h"
#include "Help.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <map>
#include "Exceptions.h"
#include "Uri.h"

zmq::context_t context(1);
zmq::socket_t eth_socket(context, ZMQ_PAIR);

/**
 *  Sends data over the ethernet connection.
 *
 *  @param serialized_data data to be transmitted over ethernet
 *
 */
int send(const std::string &serialized_data) {
    zmq::message_t message(serialized_data.length());
    memcpy(message.data(), serialized_data.data(), serialized_data.length());
    return eth_socket.send(message, ZMQ_NOBLOCK);
}

/**
 *  Sends any nt updates to the NUC via ethernet
 *
 *  @param node          Updated network table node (expecting ActuationAngle)
 *  @param diffs         Uri and value of updated nt node
 *  @param is_self_reply Prevents recursive callbacks
 *
 */
void RootCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
        bool is_self_reply) {
    // Send any changes to the network table to the NUC
    if (!is_self_reply) {
        std::string serialized_root;

        // Serialize the data before sending
        node.SerializeToString(&serialized_root);

        /*
         * Since bandwidth isnt a major issue on the ethernet cable, 
         * just send over the entire thing, and let the NUC extract 
         * the data it wants.
         * The BBB is a lot slower so it *should* be better to do
         * it on the NUC side .
         */
        std::cout << "Sending sensor data to the NUC" << std::endl;
        send(serialized_root);
    }
}

/**
 * Prints instructions on how to run this program
 *
 */
void PrintUsage() {
    std::cout << "Provide the ip address and port to bind to. ex\n"
        "./bbb_eth_listener 192.168.1.60 5555" << std::endl;
}

/**
 * Sends all network table updates to the NUC and
 * sends all actuation angle & power controller updates
 * outputted from the controller to the BBB
 *
 */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        PrintUsage();
        return 0;
    }

    NetworkTable::Connection connection;

    // TODO(brielle): Surround in try catch
    connection.Connect(1000, true);

    // Bind to the ethernet port linking BBB and NUC
    try {
        eth_socket.bind("tcp://" + std::string(argv[1]) + ":" + argv[2]);
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
        PrintUsage();
        return 0;
    } catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
        PrintUsage();
        return 0;
    }

    bool is_subscribed = false;

    // Subscribe to the root node of the network table, so all updates
    // are written to the NUC
    while (!is_subscribed) {
        try {
            connection.Subscribe("/", &RootCallback);
            is_subscribed = true;
        }
        catch (NetworkTable::NotConnectedException) {
            std::cout << "Failed to connect" << std::endl;
            sleep(1);
        }
    }

    while (true) {
        // Poll the ethernet socket and write updates to the network table
        zmq::message_t message;
        int rc = eth_socket.recv(&message);

        if (rc > 0) {
            std::string message_serialized( \
                static_cast<char*>(message.data()), message.size());

            // Expecting a Controller obj that either encapsulates actuation or power data
            NetworkTable::Controller controller_data;

            std::cout << message_serialized << std::endl;
            controller_data.ParseFromString(message_serialized);

            std::map<std::string, NetworkTable::Value> values;

            if (controller_data.type() == NetworkTable::Controller::ACTUATION_DATA) {
                // Create new nodes populated with the outputted actuation angles
                NetworkTable::Value winch_angle;
                NetworkTable::Value rudder_angle;
                winch_angle.set_type(
                        NetworkTable::Value::INT);
                rudder_angle.set_type(
                        NetworkTable::Value::FLOAT);
                winch_angle.set_int_data(controller_data.actuation_angle_data().winch_angle());
                rudder_angle.set_float_data(controller_data.actuation_angle_data().rudder_angle());

                // Map the new actuation nodes to their corresponding uris
                // TODO(brielle): May have to check if port or stbd rudder
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (WINCH_MAIN_ANGLE, winch_angle));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (RUDDER_PORT_ANGLE, rudder_angle));

                std::cout << "received rudder angle: " << controller_data.actuation_angle_data().rudder_angle() \
                    << " winch angle: " << controller_data.actuation_angle_data().winch_angle() << std::endl;
            } else if (controller_data.type() == NetworkTable::Controller::MOCK_GPS_DATA) {
                // Create new nodes populated with the outputted GPS data
                NetworkTable::Value latitude;
                NetworkTable::Value longitude;
                longitude.set_type(NetworkTable::Value::FLOAT);
                latitude.set_type(NetworkTable::Value::FLOAT);

                longitude.set_float_data(controller_data.mock_gps_data().longitude());
                latitude.set_float_data(controller_data.mock_gps_data().latitude());

                // Map mock gps nodes to corresponding uris
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LON, longitude));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (GPS_CAN_LAT, latitude));

                std::cout << "received longitude: " << controller_data.mock_gps_data().longitude() \
                    << "\nreceived latitude: " << controller_data.mock_gps_data().latitude() << std::endl;
            } else if (controller_data.type() == NetworkTable::Controller::POWER_DATA) {
                // Create new nodes populated with the outputted power controller states
                NetworkTable::Value pv_mppt;
                NetworkTable::Value pwr;
                NetworkTable::Value mppt;
                pv_mppt.set_type(NetworkTable::Value::INT);
                pwr.set_type(NetworkTable::Value::INT);
                mppt.set_type(NetworkTable::Value::INT);

                pv_mppt.set_int_data(controller_data.power_controller_data().pv_mppt_engage());
                pwr.set_int_data(controller_data.power_controller_data().pwr_engage());
                mppt.set_int_data(controller_data.power_controller_data().mppt_engage());

                // Map the new power nodes to their corresponding uris
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (POWER_PV_MPPT, pv_mppt));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (POWER_PWR, pwr));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (POWER_MPPT, mppt));

                std::cout << "received power_pv_mppt: " << controller_data.power_controller_data().pv_mppt_engage() \
                    << "received power_pwt: " << controller_data.power_controller_data().pwr_engage() \
                    << "received mppt: " << controller_data.power_controller_data().mppt_engage() << std::endl;
            } else {
                std::cout << "*ERROR*: Actuation Angle or Power Controller data not found" << std::endl;
                return 0;
            }

            // Write the new values to the network table
            try {
                connection.SetValues(values);
            } catch (NetworkTable::NotConnectedException) {
                std::cout << "Failed to connect" << std::endl;
            } catch (NetworkTable::TimeoutException) {
                std::cout << "timeout" << std::endl;
            }
        }
    }
}
