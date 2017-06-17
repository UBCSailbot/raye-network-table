// Copyright 2017 UBC Sailbot

#include "NetworkTableServer.h"

#include <boost/algorithm/string.hpp>

NetworkTable::Server::Server(std::string address)
    : context_(1), socket_(context_, ZMQ_REP) {
    socket_.bind(address);
}

void NetworkTable::Server::Run() {
    while (true) {
        zmq::message_t request;

        //  Wait for next request from client
        socket_.recv(&request);
        std::string message = static_cast<char*>(request.data());

        // Split the message into components:
        std::vector<std::string> message_parts;
        boost::split(message_parts, message, boost::is_any_of(":"));

        if (message_parts.size() == 2) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);

            if (action.compare("GET") == 0) {
                // TODO(alexmac): check for missing entry
                std::string reply_body = table_[key];
                zmq::message_t reply(reply_body.size()+1);
                memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
                socket_.send(reply);
            }
        } else if (message_parts.size() == 3) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);
            std::string value = message_parts.at(2);

            if (action.compare("SET") == 0) {
                table_[key] = value;

                // Send reply back to client
                std::string reply_body("success");
                zmq::message_t reply(reply_body.size()+1);
                memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
                socket_.send(reply);
            }
        }
    }
}
