// Copyright 2017 UBC Sailbot

#include "Connection.h"

#include <assert.h>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <zmq.hpp>
#include <csignal>
#include <cerrno>
#include "Exceptions.h"
#include "GetNodesRequest.pb.h"
#include "SetValuesRequest.pb.h"
#include "ErrorReply.pb.h"

// Use this to check if we received
// a signal, ex SIGINT
static volatile sig_atomic_t signaled = 0;
static volatile std::atomic<bool> signal_handler_registered(false);

void signal_handler(int param) {
      signaled = 1;
}

NetworkTable::Connection::Connection() : context_(1),
                                         mst_socket_(context_, ZMQ_PAIR),
                                         connected_(false) {
    // Register our signal handler.
    // After this, if we ctrl-c,
    // this function will be called, which allows
    // us to gracefully exit.
    if (!signal_handler_registered) {
        signal(SIGINT, signal_handler);
        signal_handler_registered = true;
    }
}

void NetworkTable::Connection::Connect(int timeout_millis, bool async) {
    assert(!connected_);

    mst_socket_.bind("inproc://#1");
    socket_thread_ = std::thread(&NetworkTable::Connection::ManageSocket, this, timeout_millis, async);

    if (!async) {
        zmq::message_t message;
        try {
            mst_socket_.recv(&message);
        } catch (const zmq::error_t &e) {
            if (signaled && e.num() == EINTR) {
                InterruptManageSocketThread();
                throw NetworkTable::InterruptedException("Received interrupt signal");
            }
        }

        std::string message_data(static_cast<char*>(message.data()), message.size());
        if (strcmp(message_data.c_str(), "timeout") == 0) {
            socket_thread_.join();
            throw TimeoutException(const_cast<char*>("timed out when connecting to server"));
        }
    } else {
        sleep(1);
    }

    mst_socket_.setsockopt(ZMQ_RCVTIMEO, timeout_millis);
    mst_socket_.setsockopt(ZMQ_SNDTIMEO, timeout_millis);
}

void NetworkTable::Connection::Disconnect() {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to disconnect"));
    }

    // Tell manage socket thread to disconnect
    std::string request_body = "disconnect";
    zmq::message_t request(request_body.size()+1);
    memcpy(request.data(), request_body.c_str(), request_body.size()+1);
    try {
        if (!mst_socket_.send(request)) {
            throw TimeoutException(const_cast<char*>("disconnect timed out"));
        }
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }

    socket_thread_.join();
}

////////////////////// PUBLIC //////////////////////

void NetworkTable::Connection::SetValue(const std::string &uri, const NetworkTable::Value &value) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to set value"));
    }
    std::map<std::string, NetworkTable::Value> values = {{uri, value}};
    SetValues(values);
}

void NetworkTable::Connection::SetValues(const std::map<std::string, NetworkTable::Value> &values) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to set value"));
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);

    // Create a SetValuesRequest with each of the uri/value pairs.
    auto *setvalues_request = request.mutable_setvalues_request();
    auto mutable_values = setvalues_request->mutable_values();
    for (auto const &entry : values) {
        std::string uri = entry.first;
        NetworkTable::Value value = entry.second;
        (*mutable_values)[uri] = value;
    }

    try {
        if (!Send(request, &mst_socket_)) {
            throw TimeoutException(const_cast<char*>("set values timed out"));
        }

        WaitForAck();
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }
}

NetworkTable::Value NetworkTable::Connection::GetValue(const std::string &uri) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to get value"));
    }

    std::set<std::string> uris = {uri};

    return GetValues(uris)[uri];
}

std::map<std::string, NetworkTable::Value> NetworkTable::Connection::GetValues(const std::set<std::string> &uris) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to get value"));
    }

    std::map<std::string, NetworkTable::Value> values;

    for (auto const &uriNodePair : GetNodes(uris)) {
        values[uriNodePair.first] = uriNodePair.second.value();
    }

    return values;
}

NetworkTable::Node NetworkTable::Connection::GetNode(const std::string &uri) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to get node"));
    }

    std::set<std::string> uris = {uri};

    return GetNodes(uris)[uri];
}

std::map<std::string, NetworkTable::Node> NetworkTable::Connection::GetNodes(const std::set<std::string> &uris) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to get node"));
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETNODES);

    auto *getnodes_request = request.mutable_getnodes_request();

    for (auto const &uri : uris) {
        getnodes_request->add_uris(uri);
    }

    try {
        if (!Send(request, &mst_socket_)) {
            throw TimeoutException(const_cast<char*>("getnodes send timed out"));
        }
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }

    NetworkTable::Reply reply;
    try {
        if (!Receive(&reply, &mst_socket_)) {
            throw TimeoutException(const_cast<char*>("getnodes reply timed out"));
        }
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
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
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to subscribe"));
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SUBSCRIBE);

    auto *subscribe_request = request.mutable_subscribe_request();
    subscribe_request->set_uri(uri);

    try {
        if (!Send(request, &mst_socket_)) {
            throw TimeoutException(const_cast<char*>("subscribe send timed out"));
        }

        WaitForAck();
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }

    // Don't fill in our callback table until
    // after we get the ACK.
    callbacks_[uri] = callback;
}

void NetworkTable::Connection::Unsubscribe(std::string uri) {
    if (!connected_) {
        throw NotConnectedException(const_cast<char*>("fail to unsubscribe"));
    }

    callbacks_.erase(uri);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::UNSUBSCRIBE);

    auto *unsubscribe_request = request.mutable_unsubscribe_request();
    unsubscribe_request->set_uri(uri);

    try {
        if (!Send(request, &mst_socket_)) {
            throw TimeoutException(const_cast<char*>("unsubscribe send timed out"));
        }

        WaitForAck();
    } catch (const zmq::error_t &e) {
        if (signaled && e.num() == EINTR) {
            InterruptManageSocketThread();
            throw NetworkTable::InterruptedException("Received interrupt signal");
        }
    }
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
        throw TimeoutException(const_cast<char*>("ack timed out"));
    }
    if (reply.type() != NetworkTable::Reply::ACK) {
        throw std::runtime_error(const_cast<char*>(\
                "received non-ack from server while expecting an ack"));
    }
}

void NetworkTable::Connection::InterruptManageSocketThread() {
    // It turns out when you send an interrupt signal,
    // the exception is thrown by the socket in the main thread.
    // any blocking socket.recv in the manage socket thread
    // will keep on waiting like normal.
    // Because of this, a zmq::poll is done in manage socket thread
    // to listen to the main thread socket, as well as any other socket.
    // If we do get an interrupt, we can tell the manage socket thread to die,
    // avoiding any memory leaks.
    std::string interrupt_string = "interrupted";
    zmq::message_t interrupt_msg(interrupt_string.size()+1);
    memcpy(interrupt_msg.data(), interrupt_string.c_str(), interrupt_string.size()+1);
    mst_socket_.send(interrupt_msg);
    socket_thread_.join();
}

void NetworkTable::Connection::ManageSocket(int timeout_millis, bool async) {
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

        // In synchronous mode, we wait some time for the server
        // to reply, and then possibly timeout.
        // In async mode, we wait forever to the server to
        // reply. It might not even be up yet. However,
        // the main thread is not blocked, and continuous on
        // while we wait to connect to the server here.
        if (!async) {
            init_socket.setsockopt(ZMQ_RCVTIMEO, timeout_millis);
        } else {
            init_socket.setsockopt(ZMQ_RCVTIMEO, -1);
        }
        init_socket.connect("ipc://" + kWelcome_Directory_ + "NetworkTable");

        {
            // The request body must be "connect"
            // in order for the server to reply.
            std::string request_body = "connect";
            zmq::message_t request(request_body.size()+1);
            memcpy(request.data(), request_body.c_str(), request_body.size()+1);
            init_socket.send(request);
        }

        zmq::message_t reply;

        // Listen to the master socket, and the init socket.
        // We need to listen to the master socket because
        // it will tell us if the program got an interrupt signal,
        // in which case we clean up our thread and return.
        std::vector<zmq::pollitem_t> pollitems;

        zmq::pollitem_t mt_pollitem;
        mt_pollitem.socket = static_cast<void*>(mt_socket);
        mt_pollitem.events = ZMQ_POLLIN;
        pollitems.push_back(mt_pollitem);

        zmq::pollitem_t init_pollitem;
        init_pollitem.socket = static_cast<void*>(init_socket);
        init_pollitem.events = ZMQ_POLLIN;
        pollitems.push_back(init_pollitem);

        zmq::poll(pollitems.data(), 2, -1);

        if (pollitems[0].revents & ZMQ_POLLIN) {
            mt_socket.recv(&reply);
            /*
             * This looks jank (convert c string to std::string and then back),
             * but for some reason I had to do this (can't remember why. maybe you can fix it?)
             */
            std::string message_data(static_cast<char*>(reply.data()), reply.size());
            if (strcmp(message_data.c_str(), "interrupted") == 0) {
                return;
            }
        } else {
            // The server replies with a location to connect to.
            // after this, this ZMQ_REQ socket is no longer needed.
            if (!init_socket.recv(&reply)) {
                if (!async) {
                    std::string request_body = "timeout";
                    zmq::message_t request(request_body.size()+1);
                    memcpy(request.data(), request_body.c_str(), request_body.size()+1);
                    mt_socket.send(request);
                }
                return;
            }
        }

        // Connect to the ZMQ_PAIR socket which was created
        // by the server.
        socket_filepath_ = std::string(static_cast<char*>(reply.data()));
        socket.connect("ipc://" + socket_filepath_);
        connected_ = true;

        // if we're in synchronous mode, we need to tell
        // the main thread we managed to connected so that it
        // can move on. If we're not, do not send this message
        // or it will be received by the main thread
        // when the main thread expects something completely different.
        if (!async) {
            // Let the main thread know we connected
            std::string request_body = "connected";
            zmq::message_t request(request_body.size()+1);
            memcpy(request.data(), request_body.c_str(), request_body.size()+1);
            mt_socket.send(request);
        }
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

            /*
             * This looks jank (convert c string to std::string and then back),
             * but for some reason I had to do this (can't remember why. maybe you can fix it?)
             */
            std::string message_data(static_cast<char*>(message.data()), message.size());
            if (strcmp(message_data.c_str(), "disconnect") == 0) {
                break;
            }

            if (strcmp(message_data.c_str(), "interrupted") == 0) {
                return;
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
