//Copyright 2017 UBC Sailbot

#include "network-table.h"

// Initializing socket_ since it's static
zmq::context_t context(NetworkTable::kCentralControllerContext);
zmq::socket_t NetworkTable::socket_(context, ZMQ_REQ);


std::string NetworkTable::Get(std::string key) {
    std::string message_body("GET:" + key);
    zmq::message_t request(message_body.size()+1);
    memcpy(request.data(), message_body.c_str(), message_body.size()+1);
    NetworkTable::socket_.send(request);
    
    zmq::message_t reply;
    NetworkTable::socket_.recv(&reply);
    return static_cast<char*>(reply.data());
}

bool NetworkTable::Set(std::string key, std::string value) {
    std::string message_body("SET:" + key + ":" + value);
    zmq::message_t request(message_body.size()+1);
    memcpy(request.data(), message_body.c_str(), message_body.size()+1);
    NetworkTable::socket_.send(request);
    
    zmq::message_t reply;
    NetworkTable::socket_.recv(&reply);
    // TODO(alexmac): return true if reply is equal to "success"
    return true;
}

NetworkTable::NetworkTable() {
    NetworkTable::socket_.connect("tcp://localhost:5555");
}

void NetworkTable::Connect() {
    socket_.connect("tcp://localhost:" + std::to_string(NetworkTable::kCentralControllerPort));
}
