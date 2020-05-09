// Copyright 2017 UBC Sailbot

#include "Connection.h"
#include "ActuationAngle.pb.h"
#include "Value.pb.h"
#include "Help.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <map>

zmq::context_t context(1);
zmq::socket_t eth_socket(context, ZMQ_PAIR);

/*
 * Send data over the ethernet connection.
 */
int send(const std::string &serialized_data) {
    zmq::message_t message(serialized_data.length());
    memcpy(message.data(), serialized_data.data(), serialized_data.length());
    return eth_socket.send(message, ZMQ_NOBLOCK);
}

void RootCallback(NetworkTable::Node node, \
        std::map<std::string, NetworkTable::Value> diffs, \
        bool is_self_reply) {
    /*
     * Send any changes to the network table to the
     * NUC
     */
    if (!is_self_reply) {
        std::string serialized_root;
        node.SerializeToString(&serialized_root);

        /*
         * Since bandwidth isnt a major issue
         * on the ethernet cable, just send
         * over the entire thing, and let the
         * NUC extract the data it wants.
         * The beaglebone is a lot slower
         * so it *should* be better to do
         * it on the NUC side .
         */
        send(serialized_root);
    }
}

void PrintUsage() {
    std::cout << "Provide the ip address and port to bind to. ex\n"
        "./bbb_eth_listener 10.0.0.8 5555" << std::endl;
}

/*
 * Subscribe to receive any changes in the entire
 * network table. When a change occurs, send
 * over the sensor data to the NUC.
 * If we receive a change from them, put the changes
 * in our Network Table.
 */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        PrintUsage();
        return 0;
    }

    NetworkTable::Connection connection;
    connection.SetTimeout(1000);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to network table." << std::endl;
        return 0;
    }

    try {
        eth_socket.bind("tcp://" + std::string(argv[1]) + ":" + argv[2]);
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
        PrintUsage();
        return 0;
    }

    connection.Subscribe("/", &RootCallback);
    while (true) {
        /*
         * Receive actuation angle and put it into network table
         */
        zmq::message_t message;
        int rc = eth_socket.recv(&message);

        if (rc > 0) {
            std::string message_serialized(static_cast<char*>(message.data()),message.size());
            NetworkTable::ActuationAngle actuation_angle;
            actuation_angle.ParseFromString(message_serialized);

            NetworkTable::Value winch_angle;
            NetworkTable::Value rudder_angle;
            winch_angle.set_type(
                    NetworkTable::Value::FLOAT);
            rudder_angle.set_type(
                    NetworkTable::Value::FLOAT);
            winch_angle.set_float_data(actuation_angle.winch_angle());
            rudder_angle.set_float_data(actuation_angle.rudder_angle());

            std::map<std::string, NetworkTable::Value> values;
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("actuation_angle/winch", winch_angle));
            values.insert(std::pair<std::string, NetworkTable::Value>\
                    ("rudder_angle/winch", rudder_angle));

            std::cout << "received rudder angle: " << actuation_angle.rudder_angle() \
                << " winch angle: " << actuation_angle.winch_angle() << std::endl;
            connection.SetValues(values);
        }
    }
}
