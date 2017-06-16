// Copyright 2017 UBC Sailbot

#include "NetworkTableConnection.h"

NetworkTableConnection::NetworkTableConnection(std::string address) 
    : context_(1), socket_(context_, ZMQ_REQ) {
    socket_.connect(address);
}

std::string NetworkTableConnection::Get(std::string key) {
    std::string message_body("GET:" + key);
    zmq::message_t request(message_body.size()+1);
    memcpy(request.data(), message_body.c_str(), message_body.size()+1);
    socket_.send(request);

    zmq::message_t reply;
    socket_.recv(&reply);
    return static_cast<char*>(reply.data());
}

void NetworkTableConnection::Set(std::string key, std::string value) {
    std::string message_body("SET:" + key + ":" + value);
    zmq::message_t request(message_body.size()+1);
    memcpy(request.data(), message_body.c_str(), message_body.size()+1);
    socket_.send(request);

    zmq::message_t reply;
    socket_.recv(&reply);
}
