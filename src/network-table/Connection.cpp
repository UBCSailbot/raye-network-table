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

void NetworkTable::Connection::SetValue(std::string key, const NetworkTable::Value &value) {
    NetworkTable::Value *allocated_value = new NetworkTable::Value(value);

    NetworkTable::SetValueRequest *setvalue_request = new NetworkTable::SetValueRequest();
    setvalue_request->set_key(key);
    setvalue_request->set_allocated_value(allocated_value);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUE);
    request.set_allocated_setvalue_request(setvalue_request);

    Send(request);
}

NetworkTable::Value NetworkTable::Connection::GetValue(std::string key) {
    NetworkTable::GetValueRequest *getvalue_request = new NetworkTable::GetValueRequest();
    getvalue_request->set_key(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUE);
    request.set_allocated_getvalue_request(getvalue_request);

    Send(request);

    NetworkTable::Reply reply;
    Receive(&reply);

    return reply.getvalue_reply().value();
}

void NetworkTable::Connection::Send(const NetworkTable::Request &request) {
    // Serialize the message to a string
    std::string serialized_request;
    request.SerializeToString(&serialized_request);

    // Send the string to the server.
    zmq::message_t message(serialized_request.length());
    memcpy(message.data(), serialized_request.data(), serialized_request.length());
    socket_.send(message);
}

void NetworkTable::Connection::Receive(NetworkTable::Reply *reply) {
    zmq::message_t message;
    socket_.recv(&message);

    std::string serialized_reply(static_cast<char*>(message.data()), \
                            message.size());

    reply->ParseFromString(serialized_reply);
}
