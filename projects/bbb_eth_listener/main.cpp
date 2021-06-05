// Copyright 2017 UBC Sailbot

#include "Connection.h"
#include "ActuationAngle.pb.h"
#include "PowerController.pb.h"
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

/*
 * Send data over the ethernet connection.
 */
int send(const std::string &serialized_data) {
    zmq::message_t message(serialized_data.length());
    memcpy(message.data(), serialized_data.data(), serialized_data.length());
    return eth_socket.send(message, ZMQ_NOBLOCK);
}

void RootCallback(NetworkTable::Node node, \
        const std::map<std::string, NetworkTable::Value> &diffs, \
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
        std::cout << "Sending sensor data to the NUC" << std::endl;
        send(serialized_root);
    }
}

void PrintUsage() {
    std::cout << "Provide the ip address and port to bind to. ex\n"
        "./bbb_eth_listener 192.168.1.60 5555" << std::endl;
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


    connection.Connect(1000, true);

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
        /*
         * Receive actuation angle and put it into network table
         */
        zmq::message_t message;
        int rc = eth_socket.recv(&message);

        if (rc > 0) {
            std::string message_serialized( \
                static_cast<char*>(message.data()), message.size());
            NetworkTable::Controller controller_data;

            std::cout << message_serialized << std::endl;
            controller_data.ParseFromString(message_serialized);

            std::map<std::string, NetworkTable::Value> values;

            if (controller_data.type() == NetworkTable::Controller::ACTUATION_DATA) {
                NetworkTable::Value winch_angle;
                NetworkTable::Value rudder_angle;
                winch_angle.set_type(
                        NetworkTable::Value::FLOAT);
                rudder_angle.set_type(
                        NetworkTable::Value::FLOAT);
                winch_angle.set_float_data(controller_data.actuation_angle_data().winch_angle());
                rudder_angle.set_float_data(controller_data.actuation_angle_data().rudder_angle());

                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (WINCH_MAIN_ANGLE, winch_angle));
                values.insert(std::pair<std::string, NetworkTable::Value>\
                        (RUDDER_PORT_ANGLE, rudder_angle));

                std::cout << "received rudder angle: " << controller_data.actuation_angle_data().rudder_angle() \
                    << " winch angle: " << controller_data.actuation_angle_data().winch_angle() << std::endl;
            } else if (controller_data.type() == NetworkTable::Controller::POWER_DATA) {
                NetworkTable::Value pv_mppt;
                NetworkTable::Value pwr;
                NetworkTable::Value mppt;
                pv_mppt.set_type(NetworkTable::Value::INT);
                pwr.set_type(NetworkTable::Value::INT);
                mppt.set_type(NetworkTable::Value::INT);

                pv_mppt.set_int_data(controller_data.power_controller_data().pv_mppt_engage());
                pwr.set_int_data(controller_data.power_controller_data().pwr_engage());
                mppt.set_int_data(controller_data.power_controller_data().mppt_engage());

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
