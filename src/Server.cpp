// Copyright 2017 UBC Sailbot

#include "Server.h"
#include "Exceptions.h"
#include "GetNodesReply.pb.h"
#include "SubscribeReply.pb.h"
#include "ErrorReply.pb.h"
#include "Request.pb.h"

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>

NetworkTable::Server::Server()
    : context_(1), welcome_socket_(context_, ZMQ_REP) {
    std::cout << "starting network table server" << std::endl;
    // Create the folders where sockets go.
    boost::filesystem::create_directory(kWelcome_Directory_);
    boost::filesystem::create_directory(kClients_Directory_);

    welcome_socket_.bind("ipc://" + kWelcome_Directory_ + "NetworkTable");

    ReconnectAbandonedSockets();

    if (boost::filesystem::exists(kValuesFilePath_)) {
        values_.Load(kValuesFilePath_);
    }
}

void NetworkTable::Server::Run() {
    while (true) {
        // Poll all the zmq sockets.
        // This includes welcome_socket_,
        // and all the ZMQ_PAIR sockets which
        // have been created.
        int num_sockets = 1 + sockets_.size();
        std::vector<zmq::pollitem_t> pollitems;

        zmq::pollitem_t pollitem;
        pollitem.socket = static_cast<void*>(welcome_socket_);
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
            CreateNewConnection();
        }
        // Do not directly pass the sockets_ vector
        // into the HandleRequest function.
        // Instead pass in a copy of the sockets_ vector.
        // This is because HandleRequest may call DisconnectSocket
        // which will modify sockets_ (specifically,
        // remove a socket from sockets_.
        std::vector<socket_ptr> sockets_copy = sockets_;

        for (int i = 0; i < num_sockets-1; i++) {
            if (pollitems[i+1].revents & ZMQ_POLLIN) {
                HandleRequest(sockets_copy[i]);
            }
        }
    }
}

void NetworkTable::Server::CreateNewConnection() {
    zmq::message_t request;
    welcome_socket_.recv(&request);

    std::string message = static_cast<char*>(request.data());

    if (message == "connect") {
        /* A new client has connected to Network Table Server.
         * Create a new ZMQ_PAIR socket, and send them
         * the location of it.
         */
        // Get a location for new socket.
        // The filename for the socket is a randomly generated id.
        std::string socket_name = boost::uuids::to_string(boost::uuids::random_generator()());
        std::string filepath = "ipc://" + kClients_Directory_ \
            +  socket_name;

        // Add new socket to sockets_ and bind it.
        socket_ptr socket = std::make_shared<zmq::socket_t>(context_, ZMQ_PAIR);
        socket->bind(filepath);
        sockets_.push_back(socket);

        // Reply to client with location of socket.
        std::string reply_body = filepath;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        welcome_socket_.send(reply);
    } else {
        std::string reply_body = "error unknown request: " + message;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        welcome_socket_.send(reply);
    }
}

void NetworkTable::Server::ReconnectAbandonedSockets() {
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(kClients_Directory_);
         itr != end_itr;
         ++itr) {
        // If the file is a socket, add it to our list of sockets.
        if (itr->status().type() == boost::filesystem::file_type::socket_file) {
            socket_ptr socket = std::make_shared<zmq::socket_t>(context_, ZMQ_PAIR);
            std::string full_path_to_socket = itr->path().root_path().string() + itr->path().relative_path().string();
            socket->bind("ipc://" + full_path_to_socket);
            sockets_.push_back(socket);
        }
    }
}

void NetworkTable::Server::HandleRequest(socket_ptr socket) {
    zmq::message_t message;
    socket->recv(&message);

    // First check to see if the client wanted to disconnect from the server.
    if (strcmp(static_cast<char*>(message.data()), "disconnect") == 0) {
        DisconnectSocket(socket);
        return;
    }

    std::string serialized_request(static_cast<char*>(message.data()), \
            message.size());

    NetworkTable::Request request;
    if (!request.ParseFromString(serialized_request)) {
        std::cout << "Error parsing message\n";
        return;
    }

    switch (request.type()) {
        case NetworkTable::Request::SETVALUES: {
            if (request.has_setvalues_request()) {
                SetValues(request.setvalues_request());
            }
            break;
        }
        case NetworkTable::Request::GETNODES: {
            if (request.has_getnodes_request()) {
                GetNodes(request.getnodes_request(), \
                       request.id(), socket);
            }
            break;
        }
        case NetworkTable::Request::SUBSCRIBE: {
            if (request.has_subscribe_request()) {
                Subscribe(request.subscribe_request(), socket);
            }
            break;
        }
        case NetworkTable::Request::UNSUBSCRIBE: {
            if (request.has_unsubscribe_request()) {
                Unsubscribe(request.unsubscribe_request(), socket);
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

void NetworkTable::Server::SetValues(const NetworkTable::SetValuesRequest &request) {
    std::set<std::string> uris;
    for (auto const &entry : request.values()) {
        std::string uri = entry.first;
        NetworkTable::Value value = entry.second;
        values_.SetNode(uri, value);

        uris.insert(uri);
    }

    values_.Write(kValuesFilePath_);

    // When the table has changed, make sure to
    // notify anyone who subscribed to those uris,
    // or any parent uris.
    NotifySubscribers(uris);
}

void NetworkTable::Server::GetNodes(const NetworkTable::GetNodesRequest &request, \
            std::string id, socket_ptr socket) {
    auto *getnodes_reply = new NetworkTable::GetNodesReply();
    auto *mutable_nodes = getnodes_reply->mutable_nodes();

    for (int i = 0; i < request.uris_size(); i++) {
        std::string uri = request.uris(i);
        try {
            NetworkTable::Node node = values_.GetNode(uri);
            (*mutable_nodes)[uri] = node;
        } catch (NetworkTable::NodeNotFoundException) {
            NetworkTable::Reply reply;
            reply.set_id(id);
            reply.set_type(NetworkTable::Reply::ERROR);
            auto *error_reply = new NetworkTable::ErrorReply;
            error_reply->set_message_data(std::string(uri + " does not exist"));
            reply.set_allocated_error_reply(error_reply);
            SendReply(reply, socket);
            return;
        }
    }

    NetworkTable::Reply reply;
    reply.set_id(id);
    reply.set_type(NetworkTable::Reply::GETNODES);
    reply.set_allocated_getnodes_reply(getnodes_reply);

    SendReply(reply, socket);
}

void NetworkTable::Server::Subscribe(const NetworkTable::SubscribeRequest &request, \
            socket_ptr socket) {
    subscriptions_table_[request.uri()].insert(socket);
}

void NetworkTable::Server::Unsubscribe(const NetworkTable::UnsubscribeRequest &request, \
            socket_ptr socket) {
    subscriptions_table_[request.uri()].erase(socket);
}

void NetworkTable::Server::DisconnectSocket(socket_ptr socket) {
    // Make sure to remove any subscriptions this socket had.
    // Without this, the server will still try to send
    // updates to the socket.
    for (auto &entry : subscriptions_table_) {
        entry.second.erase(socket);
    }

    {
        // Remove the socket from our list of sockets to poll
        // and delete it.
        auto it = std::find(sockets_.begin(), sockets_.end(), socket);
        if (it != sockets_.end()) {
            sockets_.erase(it);
        }
    }

    // Delete the ipc socket from disk.
    char endpoint_c_str[1024];
    size_t endpoint_c_str_size = sizeof(endpoint_c_str);
    socket->getsockopt(ZMQ_LAST_ENDPOINT, &endpoint_c_str, &endpoint_c_str_size);

    std::string endpoint(endpoint_c_str, endpoint_c_str_size);
    int transport_len = strlen("ipc://");
    endpoint.erase(endpoint.begin(), endpoint.begin()+transport_len);

    boost::filesystem::remove(endpoint);
}

void NetworkTable::Server::NotifySubscribers(const std::set<std::string> &uris) {
    // This will contain a list of uris
    // for which the update was already sent out to.
    // This is to make sure we don't send multiple of the same
    // update to a subscriber.
    std::set<std::string> do_not_send;

    for (std::string uri : uris) {
        // This will contain the uri itself, and all parent
        // uris. Anyone subscribed to these uris will receive
        // a single publish message.
        std::set<std::string> subscribed_uris;
        while (true) {
            subscribed_uris.insert(uri);

            size_t slash_idx = uri.find_last_of('/');
            if (slash_idx != std::string::npos) {
                uri = uri.substr(0, slash_idx);
            } else {
                break;
            }
        }

        // Also notify anyone who subscribed to the root
        subscribed_uris.insert("");
        subscribed_uris.insert("/");

        // Now, send the reply to anybody who is subscribed to those uris
        for (const std::string &subscribed_uri : subscribed_uris) {
            if (do_not_send.find(subscribed_uri) == do_not_send.end()) {
                auto *subscribe_reply = new NetworkTable::SubscribeReply();
                subscribe_reply->set_allocated_node(new NetworkTable::Node(values_.GetNode(subscribed_uri)));
                subscribe_reply->set_uri(subscribed_uri);
                NetworkTable::Reply reply;
                reply.set_type(NetworkTable::Reply::SUBSCRIBE);
                reply.set_allocated_subscribe_reply(subscribe_reply);

                std::set<socket_ptr> subscription_sockets = subscriptions_table_[subscribed_uri];
                for (const auto& socket : subscription_sockets) {
                    SendReply(reply, socket);
                }

                do_not_send.insert(subscribed_uri);
            }
        }
    }
}

void NetworkTable::Server::SendReply(const NetworkTable::Reply &reply, socket_ptr socket) {
    std::string serialized_reply;
    reply.SerializeToString(&serialized_reply);

    zmq::message_t message(serialized_reply.length());
    memcpy(message.data(), serialized_reply.data(), serialized_reply.length());
    socket->send(message, ZMQ_DONTWAIT);
}
