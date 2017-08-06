// Copyright 2017 UBC Sailbot

#include "Connection.h"

NetworkTable::Connection::Connection(std::string address)
    : context_(1), socket_(context_, ZMQ_PAIR) {
    // First, send a request to Network Table Server
    // to get a location to connect to. This is done
    // using request/reply sockets.
    zmq::socket_t init_socket(context_, ZMQ_REQ);
    init_socket.connect(address);

    // The request body must be "connect"
    // in order for the server to reply.
    std::string request_body = "connect";
    zmq::message_t request(request_body.size()+1);
    memcpy(request.data(), request_body.c_str(), request_body.size()+1);
    init_socket.send(request);

    // The server replies with a location to connect to.
    // after this, this ZMQ_REQ socket is no longer needed.
    zmq::message_t reply;
    init_socket.recv(&reply);

    std::string filepath = static_cast<char*>(reply.data());

    socket_.connect(filepath);
}

std::string NetworkTable::Connection::Send(NetworkTable::Message *message_body) {
    // Send the message to the server.
    zmq::message_t message(sizeof(NetworkTable::Message));
    memcpy(message.data(), message_body, sizeof(NetworkTable::Message));
    socket_.send(message);

    // Receive and return the reply.
    zmq::message_t reply;
    socket_.recv(&reply);

    return static_cast<char*>(reply.data());
}
