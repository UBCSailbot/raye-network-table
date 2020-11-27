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
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <atomic>
#include <iostream>
#include <cerrno>
#include <fstream>
#include <cstdio>
#include <csignal>
#include <stdexcept>

// Use this to check if we received
// a signal, ex SIGINT
static volatile sig_atomic_t signaled = 0;
static volatile std::atomic<bool> signal_handler_registered(false);

void signal_handler(int param) {
      signaled = 1;
}

NetworkTable::Server::Server()
    : context_(1), welcome_socket_(context_, ZMQ_REP) {
    // Register our signal handler.
    // After this, if we ctrl-c,
    // this function will be called, which allows
    // us to gracefully exit.
    if (!signal_handler_registered) {
        signal(SIGINT, signal_handler);
        signal_handler_registered = true;
    }

    // Create the folders where sockets go.
    boost::filesystem::create_directory(kWelcome_Directory_);
    boost::filesystem::create_directory(kClients_Directory_);

    welcome_socket_.bind("ipc://" + kWelcome_Directory_ + "NetworkTable");

    ReconnectAbandonedSockets();

    LoadSubscriptionTable();


    /*
     * If the swap file exists,
     * and the original file is deleted,
     * it means that the swap file
     * is not corrupted.
     * See the code in Help.cpp, NetworkTable::Write.
     */
    std::string swapfile(kRootFilePath_ + ".swp");
    if (!boost::filesystem::exists(kRootFilePath_)
            && boost::filesystem::exists(kRootFilePath_ + ".swp")) {
        std::rename(swapfile.c_str(), kRootFilePath_.c_str());
    }

    if (boost::filesystem::exists(kRootFilePath_)) {
        root_ = NetworkTable::Load(kRootFilePath_);
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
        try {
            zmq::poll(pollitems.data(), num_sockets, -1);
        } catch(const zmq::error_t &e) {
            if (signaled && e.num() == EINTR) {
                throw NetworkTable::InterruptedException(e.what());
            }
        }


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
        // If we got interrupted, we finish up what we were doing
        // and then exit.
        if (signaled) {
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }
}

void NetworkTable::Server::CreateNewConnection() {
    zmq::message_t request;
    try {
        welcome_socket_.recv(&request);
    } catch(const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            throw NetworkTable::InterruptedException(e.what());
        }
    }

    std::string message = static_cast<char*>(request.data());

    if (message == "connect") {
        /* A new client has connected to Network Table Server.
         * Create a new ZMQ_PAIR socket, and send them
         * the location of it.
         */
        // Get a location for new socket.
        // The filename for the socket is a randomly generated id.
        std::string socket_name = boost::uuids::to_string(boost::uuids::random_generator()());
        std::string filepath = kClients_Directory_ \
            +  socket_name;

        // Add new socket to sockets_ and bind it.
        socket_ptr socket = std::make_shared<zmq::socket_t>(context_, ZMQ_PAIR);
        socket->bind("ipc://" + filepath);
        sockets_.push_back(socket);

        // Reply to client with location of socket.
        std::string reply_body = filepath;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        try {
            welcome_socket_.send(reply);
        } catch(const zmq::error_t &e) {
            if (signaled && e.num() == EINTR) {
                throw NetworkTable::InterruptedException(e.what());
            }
        }
    } else {
        std::string reply_body = "error unknown request: " + message;
        zmq::message_t reply(reply_body.size()+1);
        memcpy(reply.data(), reply_body.c_str(), reply_body.size()+1);
        try {
            welcome_socket_.send(reply);
        } catch(const zmq::error_t &e) {
            if (signaled && e.num() == EINTR) {
                throw NetworkTable::InterruptedException(e.what());
            }
        }
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
    try {
        socket->recv(&message);
    } catch(const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            throw NetworkTable::InterruptedException(e.what());
        }
    }

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
                SetValues(request.setvalues_request(), \
                        socket);
                Ack(request.id(), socket);
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
                Ack(request.id(), socket);
            }
            break;
        }
        case NetworkTable::Request::UNSUBSCRIBE: {
            if (request.has_unsubscribe_request()) {
                Unsubscribe(request.unsubscribe_request(), socket);
                Ack(request.id(), socket);
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

void NetworkTable::Server::SetValues(const NetworkTable::SetValuesRequest &request, \
        socket_ptr socket) {
    std::set<std::string> uris;
    for (auto const &entry : request.values()) {
        std::string uri = entry.first;
        NetworkTable::Value value = entry.second;
        NetworkTable::SetNode(uri, value, &root_);

        uris.insert(uri);
    }

    NetworkTable::Write(kRootFilePath_, root_);

    // When the table has changed, make sure to
    // notify anyone who subscribed to those uris,
    // or any parent uris.
    NotifySubscribers(uris, request.values(), socket);
}

void NetworkTable::Server::GetNodes(const NetworkTable::GetNodesRequest &request, \
            std::string id, socket_ptr socket) {
    NetworkTable::Reply reply;
    reply.set_id(id);
    reply.set_type(NetworkTable::Reply::GETNODES);
    auto *getnodes_reply = reply.mutable_getnodes_reply();
    auto *mutable_nodes = getnodes_reply->mutable_nodes();

    for (int i = 0; i < request.uris_size(); i++) {
        std::string uri = request.uris(i);
        try {
            NetworkTable::Node node = NetworkTable::GetNode(uri, &root_);
            (*mutable_nodes)[uri] = node;
        } catch (NetworkTable::NodeNotFoundException) {
            NetworkTable::Reply ereply;  // funny name to avoid shadowing "reply" variable
            ereply.set_id(id);
            ereply.set_type(NetworkTable::Reply::ERROR);
            auto *error_reply = ereply.mutable_error_reply();
            error_reply->set_message_data(std::string(uri + " does not exist"));
            SendReply(ereply, socket);
            return;
        }
    }

    SendReply(reply, socket);
}

void NetworkTable::Server::Subscribe(const NetworkTable::SubscribeRequest &request, \
            socket_ptr socket) {
    subscriptions_table_[request.uri()].insert(socket);
    WriteSubscriptionTable();
}

void NetworkTable::Server::Unsubscribe(const NetworkTable::UnsubscribeRequest &request, \
            socket_ptr socket) {
    subscriptions_table_[request.uri()].erase(socket);
    WriteSubscriptionTable();
}

void NetworkTable::Server::DisconnectSocket(socket_ptr socket) {
    // Make sure to remove any subscriptions this socket had.
    // Without this, the server will still try to send
    // updates to the socket.
    for (auto &entry : subscriptions_table_) {
        entry.second.erase(socket);
    }

    WriteSubscriptionTable();

    {
        // Remove the socket from our list of sockets to poll
        // and delete it.
        auto it = std::find(sockets_.begin(), sockets_.end(), socket);
        if (it != sockets_.end()) {
            sockets_.erase(it);
        }
    }

    // Delete it from the disk to avoid reconnecting
    // in the future.
    boost::filesystem::remove(GetEndpoint(socket));
}

void NetworkTable::Server::NotifySubscribers(const std::set<std::string> &uris, \
        const google::protobuf::Map<std::string, NetworkTable::Value> &diffs, \
        socket_ptr responsible_socket) {
    std::string responsible_socket_filepath = GetEndpoint(responsible_socket);

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
                NetworkTable::Reply reply;
                reply.set_type(NetworkTable::Reply::SUBSCRIBE);

                auto *subscribe_reply = reply.mutable_subscribe_reply();

                auto *node = subscribe_reply->mutable_node();
                node->CopyFrom(NetworkTable::GetNode(subscribed_uri, &root_));

                subscribe_reply->set_uri(subscribed_uri);
                subscribe_reply->set_responsible_socket(responsible_socket_filepath);
                auto reply_diffs = subscribe_reply->mutable_diffs();
                for (auto const &diff : diffs) {
                    (*reply_diffs)[diff.first] = diff.second;
                }

                std::set<socket_ptr> subscription_sockets = subscriptions_table_[subscribed_uri];
                // Do the serialization here, not in the for loop
                std::string serialized_reply;
                reply.SerializeToString(&serialized_reply);
                for (const auto& socket : subscription_sockets) {
                    SendSerializedReply(serialized_reply, socket);
                }

                do_not_send.insert(subscribed_uri);
            }
        }
    }
}

void NetworkTable::Server::SendReply(const NetworkTable::Reply &reply, socket_ptr socket) {
    std::string serialized_reply;
    reply.SerializeToString(&serialized_reply);

    SendSerializedReply(serialized_reply, socket);
}

void NetworkTable::Server::SendSerializedReply(const std::string &serialized_reply, socket_ptr socket) {
    zmq::message_t message(serialized_reply.length());
    memcpy(message.data(), serialized_reply.data(), serialized_reply.length());
    try {
        socket->send(message, ZMQ_DONTWAIT);
    } catch(const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            throw NetworkTable::InterruptedException(e.what());
        }
    }
}

void NetworkTable::Server::Ack(const std::string &id, socket_ptr socket) {
    NetworkTable::Reply reply;
    reply.set_type(NetworkTable::Reply::ACK);
    reply.set_id(id);

    std::string serialized_reply;
    reply.SerializeToString(&serialized_reply);

    zmq::message_t message(serialized_reply.length());
    memcpy(message.data(), serialized_reply.data(), serialized_reply.length());
    try {
        socket->send(message, ZMQ_DONTWAIT);
    } catch(const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            throw NetworkTable::InterruptedException(e.what());
        }
    }
}

void NetworkTable::Server::WriteSubscriptionTable() {
    std::map<std::string, std::set<std::string>> simple_subscription_table;
    for (auto const& entry : subscriptions_table_) {
        auto uri = entry.first;
        auto subscription_sockets = entry.second;
        for (auto const& socket : subscription_sockets) {
            simple_subscription_table[uri].insert(GetEndpoint(socket));
        }
    }

    /*
     * Instead of writing to the actual file,
     * write to a swap file.
     * After that, delete the old file,
     * then rename the .swp file to the
     * proper filename.
     * This is to help prevent corrupting the file
     * in case of a crash.
     */
    std::string swapfile(kSubscriptionsTableFilePath_ + ".swp");
    std::ofstream ofs(swapfile);
    boost::archive::text_oarchive oarch(ofs);
    oarch << simple_subscription_table;
    ofs.close();

    std::remove(kSubscriptionsTableFilePath_.c_str());
    std::rename(swapfile.c_str(), kSubscriptionsTableFilePath_.c_str());
}

void NetworkTable::Server::LoadSubscriptionTable() {
    if (!boost::filesystem::exists(kSubscriptionsTableFilePath_)) {
        return;
    }

    /*
     * If the swap file exists,
     * and the original file is deleted,
     * it means that the swap file
     * is not corrupted.
     */
    std::string swapfile(kSubscriptionsTableFilePath_ + ".swp");
    if (!boost::filesystem::exists(kSubscriptionsTableFilePath_)
            && boost::filesystem::exists(kSubscriptionsTableFilePath_ + ".swp")) {
        std::rename(swapfile.c_str(), kSubscriptionsTableFilePath_.c_str());
    }

    std::map<std::string, std::set<std::string>> simple_subscription_table;

    try {
        std::ifstream ifs(kSubscriptionsTableFilePath_);
        boost::archive::text_iarchive iarch(ifs);
        iarch >> simple_subscription_table;
    } catch (const std::exception& e) {
        std::cout << "failed to read from " << kSubscriptionsTableFilePath_ << std::endl;
        std::cout << e.what() << std::endl;
        std::remove(kSubscriptionsTableFilePath_.c_str());
        std::cout << "continuing on, after deleting " << kSubscriptionsTableFilePath_ << std::endl;
        return;
    }

    for (auto const& entry : simple_subscription_table) {
        auto uri = entry.first;
        auto subscription_socket_endpoints = entry.second;
        for (auto const& endpoint : subscription_socket_endpoints) {
            for (auto const& socket : sockets_) {
                if (GetEndpoint(socket) == endpoint) {
                    subscriptions_table_[uri].insert(socket);
                }
            }
        }
    }
}

std::string NetworkTable::Server::GetEndpoint(socket_ptr socket) {
    char endpoint_c_str[1024];
    size_t endpoint_c_str_size = sizeof(endpoint_c_str);
    socket->getsockopt(ZMQ_LAST_ENDPOINT, &endpoint_c_str, &endpoint_c_str_size);

    std::string endpoint(endpoint_c_str, endpoint_c_str_size-1);  // Get rid of null at end
    int transport_len = strlen("ipc://");
    endpoint.erase(endpoint.begin(), endpoint.begin()+transport_len);

    return endpoint;
}
