// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
//

#include <zmq.hpp>
#include <string>
#include <iostream>

int main() {
    //  Prepare our context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REQ);
    socket.connect("tcp://localhost:5555");

    // Update/set a network-table entry
    {
        std::string message_body("SET:/wind/speed/:10");
        zmq::message_t request(message_body.size()+1);
        memcpy(request.data(), message_body.c_str(), message_body.size()+1);
        socket.send(request);
        std::cout << "Sent message: " << message_body << std::endl;

        zmq::message_t reply;
        socket.recv(&reply);
        std::cout << "Received reply: " << static_cast<char*>(reply.data()) << std::endl;
    }

    // Check the value of a network-table entry
    {
        std::string message_body("GET:/wind/speed/");
        zmq::message_t request(message_body.size()+1);
        memcpy(request.data(), message_body.c_str(), message_body.size()+1);
        socket.send(request);
        std::cout << "Sent message: " << message_body << std::endl;

        zmq::message_t reply;
        socket.recv(&reply);
        std::cout << "Received reply: " << static_cast<char*>(reply.data()) << std::endl;

    }
}
