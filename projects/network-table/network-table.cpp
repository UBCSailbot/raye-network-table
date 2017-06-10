// Copyright 2017 UBC Sailbot

#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <zmq.hpp>

#include "network-table.h"

void NetworkTable::Run() {
    //  Prepare our context and socket
	std::string address;
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind("tcp://*:5555");    

	std::cout << "Starting network table..." << std::endl;
    while (true) {
        zmq::message_t request;

        //  Wait for next request from client
        socket.recv (&request);
        std::string message = static_cast<char*>(request.data());
        std::cout << "Received message " << message << std::endl;

        // Split the message into components:
        std::vector<std::string> message_parts;
        boost::split(message_parts, message, boost::is_any_of(":"));

        if (message_parts.size() == 2) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);

            if (action.compare("GET") == 0) {
				//TODO(alexmac): check for missing entry
                std::string reply_body = table_[key];
		        zmq::message_t reply(reply_body.size()+1);
				memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
				socket.send(reply);
            }
        } else if (message_parts.size() == 3) {
            std::string action = message_parts.at(0);
            std::string key = message_parts.at(1);
            std::string value = message_parts.at(2);

            if (action.compare("SET") == 0) {
                table_[key] = value;

        		//  Send reply back to client
				std::string reply_body("success");
        		zmq::message_t reply(reply_body.size()+1);
        		memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        		socket.send(reply);
            }
        } 
    }
}
