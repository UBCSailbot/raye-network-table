// Copyright 2017 UBC Sailbot

#include "Server.h"

NetworkTable::Server::Server(std::string address)
    : context_(1), init_socket_(context_, ZMQ_REP) {
    init_socket_.bind(address);
}

NetworkTable::Server::~Server() {
    for (zmq::socket_t *socket : sockets_) {
        delete socket;
    }
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
            pollitem.socket = static_cast<void*>(*sockets_[i]);
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
        for (unsigned int i = 0; i < sockets_.size(); i++) {
            if (pollitems[i+1].revents & ZMQ_POLLIN) {
                HandleRequest(sockets_[i]);
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
        zmq::socket_t *socket = new zmq::socket_t(context_, ZMQ_PAIR);
        sockets_.push_back(socket);
        sockets_.back()->bind(filepath);

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
    zmq::message_t request;
    socket->recv(&request);
    NetworkTable::Message *message = static_cast<NetworkTable::Message*>(request.data());

    switch (message->action()) {
        case NetworkTable::Message::SETKEY:
          SetKey(socket);
          break;
        case NetworkTable::Message::GETKEY:
          GetKey(socket);
          break;
    }
}

void NetworkTable::Server::SetKey(zmq::socket_t *socket) {
    std::string reply_body = "I got a SetKey message";
    zmq::message_t reply(reply_body.size()+1);

    memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);

    socket->send(reply);
}

void NetworkTable::Server::GetKey(zmq::socket_t *socket) {
     std::string reply_body = "I got a GetKey message";
     zmq::message_t reply(reply_body.size()+1);

     memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);

     socket->send(reply);
}
