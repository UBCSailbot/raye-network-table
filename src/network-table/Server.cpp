// Copyright 2017 UBC Sailbot

#include "Server.h"
#include "GetValueReply.pb.h"
#include "GetValuesReply.pb.h"
#include "Request.pb.h"

#include <boost/filesystem.hpp>
#include <iostream>

NetworkTable::Server::Server()
    : context_(1), init_socket_(context_, ZMQ_REP) {
    // Create the folder where sockets go.
    boost::filesystem::create_directory("/tmp/sailbot");

    init_socket_.bind("ipc:///tmp/sailbot/NetworkTable");
}

void NetworkTable::Server::Run() {
    while (true) {
        // Poll all the zmq sockets.
        // This includes init_socket_,
        // and all the ZMQ_PAIR sockets which
        // have been created.
        int num_sockets = 1 + sockets_.size();
        std::vector<zmq::pollitem_t> pollitems;

        zmq::pollitem_t pollitem;
        pollitem.socket = static_cast<void*>(init_socket_);
        pollitem.events = ZMQ_POLLIN;
        pollitems.push_back(pollitem);

        for (unsigned int i = 0; i < sockets_.size(); i++) {
            pollitem.socket = static_cast<void*>(sockets_[i]);
            pollitem.events = ZMQ_POLLIN;
            pollitems.push_back(pollitem);
        }

        // Set the timeout to -1, which means poll
        // indefinitely until something happens.
        // This line will block until a socket is ready.
        zmq::poll(pollitems.data(), num_sockets, -1);

        if (pollitems[0].revents & ZMQ_POLLIN) {
            HandleNewConnection();
        }
        for (int i = 0; i < num_sockets-1; i++) {
            if (pollitems[i+1].revents & ZMQ_POLLIN) {
                HandleRequest(&sockets_[i]);
            }
        }
    }
}

void NetworkTable::Server::HandleNewConnection() {
    zmq::message_t request;
    init_socket_.recv(&request);

    std::string message = static_cast<char*>(request.data());

    if (message == "connect") {
        /* A new client has connected to Network Table Server.
         * Create a new ZMQ_PAIR socket, and send them
         * the location of it.
         */
        // Get a location for new socket.
        std::string filepath = "ipc:///tmp/sailbot/" \
            + std::to_string(current_socket_number_);
        current_socket_number_++;

        // Add new socket to sockets_ and bind it.
        sockets_.push_back(zmq::socket_t(context_, ZMQ_PAIR));
        sockets_.back().bind(filepath);

        // Reply to client with location of socket.
        std::string reply_body = filepath;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        init_socket_.send(reply);
    } else {
        std::string reply_body = "error unknown request: " + message;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        init_socket_.send(reply);
    }
}

void NetworkTable::Server::HandleRequest(zmq::socket_t *socket) {
    zmq::message_t message;
    socket->recv(&message);
    std::string serialized_request(static_cast<char*>(message.data()), \
            message.size());
    NetworkTable::Request request;
    if (!request.ParseFromString(serialized_request)) {
        std::cout << "Error parsing message\n";
        return;
    }

    switch (request.type()) {
        case NetworkTable::Request::SETVALUE: {
            if (request.has_setvalue_request()) {
                SetValue(request.setvalue_request());
            }
            break;
        }
        case NetworkTable::Request::SETVALUES: {
            if (request.has_setvalues_request()) {
                SetValues(request.setvalues_request());
            }
            break;
        }
        case NetworkTable::Request::GETVALUE: {
            if (request.has_getvalue_request()) {
                GetValue(request.getvalue_request(), socket);
            }
            break;
        }
        case NetworkTable::Request::GETVALUES: {
            if (request.has_getvalues_request()) {
                GetValues(request.getvalues_request(), socket);
            }
            break;
        }
        default: {
            std::cout << "Don't know how to handle request: "\
                      << request.type()\
                      << std::endl;
        }
    }
}

void NetworkTable::Server::SetValue(const NetworkTable::SetValueRequest &request) {
    std::string key = request.key();
    NetworkTable::Value value = request.value();
    values_[key] = value;
}

void NetworkTable::Server::SetValues(const NetworkTable::SetValuesRequest &request) {
    for (int i = 0; i < request.setvalue_requests_size(); i++) {
        SetValue(request.setvalue_requests(i));
    }
}

void NetworkTable::Server::GetValue(const NetworkTable::GetValueRequest &request, \
        zmq::socket_t *socket) {
    std::string key = request.key();

    NetworkTable::GetValueReply *getvalue_reply = new NetworkTable::GetValueReply();

    NetworkTable::Value *value = new NetworkTable::Value(GetValue(key));
    getvalue_reply->set_allocated_value(value);

    NetworkTable::Reply reply;
    reply.set_type(NetworkTable::Reply::GETVALUE);
    reply.set_allocated_getvalue_reply(getvalue_reply);

    SendReply(reply, socket);
}

void NetworkTable::Server::GetValues(const NetworkTable::GetValuesRequest &request, \
            zmq::socket_t *socket) {
    NetworkTable::GetValuesReply *getvalues_reply = new NetworkTable::GetValuesReply();

    for (int i = 0; i < request.getvalue_requests_size(); i++) {
        std::string key = request.getvalue_requests(i).key();
        NetworkTable::GetValueReply *getvalue_reply = getvalues_reply->add_getvalue_replies();
        NetworkTable::Value *value = new NetworkTable::Value(GetValue(key));
        getvalue_reply->set_allocated_value(value);
    }

    NetworkTable::Reply reply;
    reply.set_type(NetworkTable::Reply::GETVALUES);
    reply.set_allocated_getvalues_reply(getvalues_reply);

    SendReply(reply, socket);
}

NetworkTable::Value NetworkTable::Server::GetValue(std::string key) {
    if (values_.find(key) != values_.end()) {
        // A new network table has to be created on the heap.
        // If you instead write:
        // getvalue_reply->set_allocated_value(&values_[key]);
        // you will get a segfault when this function returns.
        return values_[key];
    } else {
        // If the value wasn't found, create a new value
        // of type NONE and return that to the client.
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::NONE);
        return value;
    }
}

void  NetworkTable::Server::SendReply(const NetworkTable::Reply &reply, zmq::socket_t *socket) {
    std::string serialized_reply;
    reply.SerializeToString(&serialized_reply);

    zmq::message_t message(serialized_reply.length());
    memcpy(message.data(), serialized_reply.data(), serialized_reply.length());
    socket->send(message);
}
