// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Node.pb.h"
#include "Value.pb.h"
#include <zmq.hpp>
#include <map>
#include <string>

zmq::context_t context(1);
zmq::socket_t eth_socket(context, ZMQ_PAIR);
NetworkTable::Node cur_nt;

/*
 * Send data over the ethernet connection.
 */
void send(const std::string &data) {
    zmq::message_t request(data.size()+1);
    memcpy(request.data(), data.c_str(), data.size()+1);
    eth_socket.send(request);
}

/*
 * Receive data over the ethernet connection.
 * Returns true if something was received,
 * false otherwise
 */
bool receive(std::map<std::string, NetworkTable::Value> &diffs) {
    zmq::message_t reply;

    int rc = eth_socket.recv(&reply);

    if (rc > 0) {
        std::string request_serialized = static_cast<char*>(reply.data());
        NetworkTable::SetValuesRequest request;
        request.ParseFromString(request_serialized);
        for (auto const &entry : request.values()) {
            std::string uri = entry.first;
            NetworkTable::Value value = entry.second;
            diffs[uri] = value;
        }
        return true;
    }
    return false;
}

void RootCallback(NetworkTable::Node node, \
        std::map<std::string, NetworkTable::Value> diffs, \
        bool is_self_reply) {
    if (!is_self_reply) {
        // Use a set values request to send the info.
        NetworkTable::SetValuesRequest request;
        auto mutable_values = request.mutable_values();
        for (auto const &entry : diffs) {
            std::string uri = entry.first;
            NetworkTable::Value value = entry.second;
            (*mutable_values)[uri] = value;
        }

        std::string serialized_request;
        request.SerializeToString(&serialized_request);

        send(serialized_request);
        std::cout << "send our diff" << std::endl;
    }
}

void PrintUsage() {
    std::cout << "Provide the ip address to connect to, as well as whether"
        "  this is client or server. Example:\n"
        "./dummy-satellite client 10.0.0.8" << std::endl <<
        "./dummy-satellite server 10.0.0.8" << std::endl;
}

/*
 * Subscribe to receive any changes in the entire
 * network table. When a change occurs, send ONLY the
 * difference over the ethernet cable.
 * Also receive changes from the ethernet cable,
 * and place those changes in the local network table.
 * This program is meant to run on the beaglebone and
 * on the webserver.
 */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        PrintUsage();
        return 1;
    }

    NetworkTable::Connection connection;
    connection.SetTimeout(1000);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to server." << std::endl;
        return 0;
    }

    try {
        if (strcmp(argv[1], "server") == 0) {
            eth_socket.bind("tcp://" + std::string(argv[1]) + ":5555");
        } else if (strcmp(argv[1], "client") == 0) {
            eth_socket.connect("tcp://" + std::string(argv[1]) + ":5555");
        } else {
            PrintUsage();
            return 1;
        }
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
        return 0;
    }

    cur_nt = connection.GetNode("/");
    connection.Subscribe("/", &RootCallback);
    while (true) {
        std::map<std::string, NetworkTable::Value> their_diff;
        if (receive(their_diff)) {
            std::cout << "received their diff" << std::endl;
            connection.SetValues(their_diff);
        }
    }
}
