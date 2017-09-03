// Copyright 2017 UBC Sailbot

#include "Connection.h"

#include "GetValuesRequest.pb.h"
#include "SetValuesRequest.pb.h"

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
    // Create a SetValuesRequest with a single
    // key/value pair. Send that to the server.
    NetworkTable::SetValuesRequest *setvalues_request = new NetworkTable::SetValuesRequest();
    NetworkTable::KeyValuePair *keyvaluepair = setvalues_request->add_keyvaluepairs();

    keyvaluepair->set_key(key);
    keyvaluepair->set_allocated_value(new NetworkTable::Value(value));

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);
    request.set_allocated_setvalues_request(setvalues_request);

    Send(request);
}

void NetworkTable::Connection::SetValues(const std::map<std::string, NetworkTable::Value> &values) {
    NetworkTable::SetValuesRequest *setvalues_request = new NetworkTable::SetValuesRequest();
    for (auto const &entry : values) {
        NetworkTable::KeyValuePair *keyvaluepair = setvalues_request->add_keyvaluepairs();
        keyvaluepair->set_key(entry.first);
        keyvaluepair->set_allocated_value(new NetworkTable::Value(entry.second));
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);
    request.set_allocated_setvalues_request(setvalues_request);

    Send(request);
}

NetworkTable::Value NetworkTable::Connection::GetValue(std::string key) {
    NetworkTable::GetValuesRequest *getvalues_request = new NetworkTable::GetValuesRequest();
    getvalues_request->add_keys(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    Send(request);

    NetworkTable::Reply reply;
    Receive(&reply);

    return reply.getvalues_reply().keyvaluepairs(0).value();
}

std::vector<NetworkTable::Value> NetworkTable::Connection::GetValues(std::vector<std::string> keys) {
    NetworkTable::GetValuesRequest *getvalues_request = new NetworkTable::GetValuesRequest();

    for (auto const &key : keys) {
        getvalues_request->add_keys(key);
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    Send(request);

    NetworkTable::Reply reply;
    Receive(&reply);

    std::vector<NetworkTable::Value> values;
    for (int i = 0; i < reply.getvalues_reply().keyvaluepairs_size(); i++) {
        values.push_back(reply.getvalues_reply().keyvaluepairs(i).value());
    }

    return values;
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
