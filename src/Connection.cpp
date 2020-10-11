// Copyright 2017 UBC Sailbot

#include "Connection.h"

#include <assert.h>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
#include "Exceptions.h"
#include "GetNodesRequest.pb.h"
#include "SetValuesRequest.pb.h"
#include "ErrorReply.pb.h"


NetworkTable::Connection::Connection() : context_(1),
                                         mst_socket_(context_, ZMQ_PAIR),
                                         connected_(false),
                                         timeout_(-1) {
}

// dummy comment
void NetworkTable::Connection::Connect() {
    assert(!connected_);

    mst_socket_.bind("inproc://#1");
    socket_thread_ = std::thread(&NetworkTable::Connection::ManageSocket, this);

    zmq::message_t message;
    mst_socket_.recv(&message);

    std::string message_data(static_cast<char*>(message.data()), message.size());
    if (strcmp(message_data.c_str(), "timeout") == 0) {
        socket_thread_.join();
        throw TimeoutException(const_cast<char*>("timed out when connecting to server"));
    }

    if (timeout_ > 0) {
        mst_socket_.setsockopt(ZMQ_RCVTIMEO, timeout_);
        mst_socket_.setsockopt(ZMQ_SNDTIMEO, timeout_);
    }
}

void NetworkTable::Connection::Disconnect() {
    assert(connected_);

    // Tell manage socket thread to disconnect
    std::string request_body = "disconnect";
    zmq::message_t request(request_body.size()+1);
    memcpy(request.data(), request_body.c_str(), request_body.size()+1);
    if (!mst_socket_.send(request)) {
        std::cout << "Disconnect timeout exception about to be thrown" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    socket_thread_.join();
}

////////////////////// PUBLIC //////////////////////

void NetworkTable::Connection::SetValue(const std::string &uri, const NetworkTable::Value &value) {
    assert(connected_);
    std::map<std::string, NetworkTable::Value> values = {{uri, value}};
    SetValues(values);
}

void NetworkTable::Connection::SetValues(const std::map<std::string, NetworkTable::Value> &values) {
    assert(connected_);

    // Create a SetValuesRequest with each of the uri/value pairs.
    auto *setvalues_request = new NetworkTable::SetValuesRequest();
    auto mutable_values = setvalues_request->mutable_values();
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value value = entry.second;
        (*mutable_values)[uri] = value;
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);
    request.set_allocated_setvalues_request(setvalues_request);

    if (!Send(request, &mst_socket_)) {
        std::cout << "Set value timeout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    WaitForAck();
}

NetworkTable::Value NetworkTable::Connection::GetValue(const std::string &uri) {
    assert(connected_);

    std::set<std::string> uris = {uri};

    return GetValues(uris)[uri];
}

std::map<std::string, NetworkTable::Value> NetworkTable::Connection::GetValues(const std::set<std::string> &uris) {
    assert(connected_);

    std::map<std::string, NetworkTable::Value> values;

    for (auto const &uriNodePair : GetNodes(uris)) {
        values[uriNodePair.first] = uriNodePair.second.value();
    }

    return values;
}

NetworkTable::Node NetworkTable::Connection::GetNode(const std::string &uri) {
    assert(connected_);

    std::set<std::string> uris = {uri};

    return GetNodes(uris)[uri];
}

std::map<std::string, NetworkTable::Node> NetworkTable::Connection::GetNodes(const std::set<std::string> &uris) {
    assert(connected_);

    auto *getnodes_request = new NetworkTable::GetNodesRequest();

    for (auto const &uri : uris) {
        getnodes_request->add_uris(uri);
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETNODES);
    request.set_allocated_getnodes_request(getnodes_request);

    if (!Send(request, &mst_socket_)) {
        std::cout << "send timeout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    NetworkTable::Reply reply;
    if (!Receive(&reply, &mst_socket_)) {
        std::cout << "receive timeout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    CheckForError(reply);

    std::map<std::string, NetworkTable::Node> nodes;
    for (auto const &entry : reply.getnodes_reply().nodes()) {
        std::string uri = entry.first;
        NetworkTable::Node node = entry.second;
        nodes[uri] = node;
    }

    return nodes;
}

void NetworkTable::Connection::Subscribe(std::string uri, \
        void (*callback)(NetworkTable::Node node, \
            const std::map<std::string, NetworkTable::Value> &diffs, \
            bool is_self_reply)) {
    assert(connected_);

    callbacks_[uri] = callback;

    auto *subscribe_request = new NetworkTable::SubscribeRequest();
    subscribe_request->set_uri(uri);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SUBSCRIBE);
    request.set_allocated_subscribe_request(subscribe_request);

    if (!Send(request, &mst_socket_)) {
        std::cout << "subscribe timout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    WaitForAck();
}

void NetworkTable::Connection::Unsubscribe(std::string uri) {
    assert(connected_);

    callbacks_.erase(uri);

    auto *unsubscribe_request = new NetworkTable::UnsubscribeRequest();
    unsubscribe_request->set_uri(uri);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::UNSUBSCRIBE);
    request.set_allocated_unsubscribe_request(unsubscribe_request);

    if (!Send(request, &mst_socket_)) {
        std::cout << "Unsubscribe timeout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }

    WaitForAck();
}

void NetworkTable::Connection::SetTimeout(int timeout) {
    timeout_ = timeout;
}

////////////////////// PRIVATE //////////////////////

int NetworkTable::Connection::Send(const NetworkTable::Request &request, zmq::socket_t *socket) {
    // Serialize the message to a string
    std::string serialized_request;
    request.SerializeToString(&serialized_request);

    // Send the string to the server.
    zmq::message_t message(serialized_request.length());
    memcpy(message.data(), serialized_request.data(), serialized_request.length());
    return socket->send(message);
}

int NetworkTable::Connection::Send(const NetworkTable::Reply &reply, zmq::socket_t *socket) {
    // Serialize the message to a string
    std::string serialized_request;
    reply.SerializeToString(&serialized_request);

    // Send the string to the server.
    zmq::message_t message(serialized_request.length());
    memcpy(message.data(), serialized_request.data(), serialized_request.length());
    return socket->send(message);
}

int NetworkTable::Connection::Receive(NetworkTable::Reply *reply, zmq::socket_t *socket) {
    zmq::message_t message;
    int rc = socket->recv(&message);
    std::string serialized_reply(static_cast<char*>(message.data()), \
                            message.size());

    reply->ParseFromString(serialized_reply);
    return rc;
}

int NetworkTable::Connection::Receive(NetworkTable::Request *request, zmq::socket_t *socket) {
    zmq::message_t message;
    int rc = socket->recv(&message);
    std::string serialized_reply(static_cast<char*>(message.data()), \
                            message.size());

    request->ParseFromString(serialized_reply);
    return rc;
}

void NetworkTable::Connection::CheckForError(const NetworkTable::Reply &reply) {
    if (reply.type() == NetworkTable::Reply::ERROR) {
        if (reply.has_error_reply()) {
            NetworkTable::ErrorReply error_reply = reply.error_reply();
            if (error_reply.type() == NetworkTable::ErrorReply::NODE_NOT_FOUND) {
                throw NetworkTable::NodeNotFoundException(error_reply.message_data());
            }
        } else {
            throw std::runtime_error("Server replied with unset error message.");
        }
    }
}

void NetworkTable::Connection::WaitForAck() {
    NetworkTable::Reply reply;
    if (!Receive(&reply, &mst_socket_)) {
        std::cout << "wait for acknowledgement timeout exception" << std::endl;
        throw TimeoutException(const_cast<char*>("timed out"));
    }
    if (reply.type() != NetworkTable::Reply::ACK) {
        throw std::runtime_error(const_cast<char*>("error receiving ack from server"));
    }
}

void NetworkTable::Connection::ManageSocket() {
    /*
     * The context must be created here so that
     * its destructor is called when this function
     * ends. For more info see:
     * http://zeromq.org/whitepapers:0mq-termination
     */
    zmq::socket_t socket(context_, ZMQ_PAIR);
    // Ensure that the destructor for
    // socket does not block and simply
    // discards any messages it is still trying to
    // send/receive:
    socket.setsockopt(ZMQ_LINGER, 0);

    // This socket will be used to talk
    // to the main thread.
    zmq::socket_t mt_socket(context_, ZMQ_PAIR);
    mt_socket.connect("inproc://#1");

    // Connect to the server
    {
        // First, send a request to Network Table Server
        // to get a location to connect to. This is done
        // using request/reply sockets.
        zmq::socket_t init_socket(context_, ZMQ_REQ);
        // Ensure that the destructor for
        // init_socket does not block and simply
        // discards any messages it is still trying to
        // send/receive:
        init_socket.setsockopt(ZMQ_LINGER, 0);
        if (timeout_ > 0) {
            init_socket.setsockopt(ZMQ_RCVTIMEO, timeout_);
        }
        init_socket.connect("ipc:///tmp/sailbot/NetworkTable");

        {
            // The request body must be "connect"
            // in order for the server to reply.
            std::string request_body = "connect";
            zmq::message_t request(request_body.size()+1);
            memcpy(request.data(), request_body.c_str(), request_body.size()+1);
            init_socket.send(request);
        }

        // The server replies with a location to connect to.
        // after this, this ZMQ_REQ socket is no longer needed.
        zmq::message_t reply;
        if (!init_socket.recv(&reply)) {
            std::string request_body = "timeout";
            zmq::message_t request(request_body.size()+1);
            memcpy(request.data(), request_body.c_str(), request_body.size()+1);
            mt_socket.send(request);
            return;
        } else {
            connected_ = true;
            std::string request_body = "connected";
            zmq::message_t request(request_body.size()+1);
            memcpy(request.data(), request_body.c_str(), request_body.size()+1);
            mt_socket.send(request);
        }

        // Connect to the ZMQ_PAIR socket which was created
        // by the server.
        socket_filepath_ = std::string(static_cast<char*>(reply.data()));
        socket.connect("ipc://" + socket_filepath_);
    }

    // Poll the two sockets.
    std::vector<zmq::pollitem_t> pollitems;

    zmq::pollitem_t pollitem;
    pollitem.socket = static_cast<void*>(socket);
    pollitem.events = ZMQ_POLLIN;
    pollitems.push_back(pollitem);

    zmq::pollitem_t mt_pollitem;
    mt_pollitem.socket = static_cast<void*>(mt_socket);
    mt_pollitem.events = ZMQ_POLLIN;
    pollitems.push_back(mt_pollitem);

    // Used to avoid stale replies
    // ie (client times out then server replies)
    std::string current_request_id = "";

    while (true) {
        zmq::poll(pollitems.data(), 2, -1);

        // If message from network table server
        if (pollitems[0].revents & ZMQ_POLLIN) {
            NetworkTable::Reply reply;
            Receive(&reply, &socket);
            // If it's a subscribe reply,
            // just run the associated callback function.
            if (reply.type() == NetworkTable::Reply::SUBSCRIBE \
                    && reply.has_subscribe_reply()) {
                std::string uri = reply.subscribe_reply().uri();
                NetworkTable::Node node = reply.subscribe_reply().node();

                bool is_self_reply = \
                  reply.subscribe_reply().responsible_socket() == socket_filepath_;

                std::map<std::string, NetworkTable::Value> diffs;
                for (const auto &diff : reply.subscribe_reply().diffs()) {
                    diffs[diff.first] = diff.second;
                }

                // Even after sending an Unsubscribe request,
                // it can take a while for that request to be processed
                // if other processes are also sending requests to the server.
                // If one of these other requests happens to update
                // a value which was subscribed to by this process,
                // a SubscribeReply can still be sent by the server,
                // even though this process just sent an UnsubscribeRequest
                // to the server.
                if (callbacks_[uri] != NULL) {
                    callbacks_[uri](node, diffs, is_self_reply);
                }
            } else {
                if (reply.id() == current_request_id) {
                    Send(reply, &mt_socket);
                    current_request_id = "";
                }
                // If the reply id is incorrect,
                // its should just be an old reply
                // which we can discard.
            }
        }

        // If request from main thread
        if (pollitems[1].revents & ZMQ_POLLIN) {
            zmq::message_t message;
            mt_socket.recv(&message);

            std::string message_data(static_cast<char*>(message.data()), message.size());
            if (strcmp(message_data.c_str(), "disconnect") == 0) {
                break;
            }

            NetworkTable::Request request;
            request.ParseFromString(message_data);
            current_request_id = boost::uuids::to_string(\
                        boost::uuids::random_generator()());
            request.set_id(current_request_id);
            Send(request, &socket);
        }
    }

    {
        // Disconnect from server.
        std::string request_body = "disconnect";
        zmq::message_t request(request_body.size()+1);
        memcpy(request.data(), request_body.c_str(), request_body.size()+1);
        socket.send(request);

        // Delete the socket ourselves, in case the server
        // is down and doesnt receive our disconnect request.
        remove(socket_filepath_.c_str());
        socket_filepath_ = "";
    }
}
