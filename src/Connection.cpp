// Copyright 2017 UBC Sailbot

#include "Connection.h"

#include <thread>
#include "GetValuesRequest.pb.h"
#include "SetValuesRequest.pb.h"

NetworkTable::Connection::Connection()
    : is_running_(true), context_(1), socket_(context_, ZMQ_PAIR),
      socket_thread_(&NetworkTable::Connection::ManageSocket, this) {
}

NetworkTable::Connection::~Connection() {
    // Stop the ManageSocket thread.
    // This must be stopped first in order to
    // send the disconnect message on the socket.
    is_running_ = false;
    socket_thread_.join();

    // Disconnect from server.
    std::string request_body = "disconnect";
    zmq::message_t request(request_body.size()+1);
    memcpy(request.data(), request_body.c_str(), request_body.size()+1);
    socket_.send(request);
}

void NetworkTable::Connection::SetValue(std::string key, const NetworkTable::Value &value) {
    // Create a SetValuesRequest with a single
    // key/value pair.
    auto *setvalues_request = new NetworkTable::SetValuesRequest();
    NetworkTable::KeyValuePair *keyvaluepair = setvalues_request->add_keyvaluepairs();

    keyvaluepair->set_key(key);
    keyvaluepair->set_allocated_value(new NetworkTable::Value(value));

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);
    request.set_allocated_setvalues_request(setvalues_request);

    // Push the request into the shared request_queue_.
    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();
}

void NetworkTable::Connection::SetValues(const std::map<std::string, NetworkTable::Value> &values) {
    // Create a SetValuesRequest with each of the key/value pairs.
    auto *setvalues_request = new NetworkTable::SetValuesRequest();
    for (auto const &entry : values) {
        NetworkTable::KeyValuePair *keyvaluepair = setvalues_request->add_keyvaluepairs();
        keyvaluepair->set_key(entry.first);
        keyvaluepair->set_allocated_value(new NetworkTable::Value(entry.second));
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SETVALUES);
    request.set_allocated_setvalues_request(setvalues_request);

    // Push the request into the shared request_queue_.
    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();
}

NetworkTable::Value NetworkTable::Connection::GetValue(std::string key) {
    // Create a GetValuesRequest with a single key/value pair.
    auto *getvalues_request = new NetworkTable::GetValuesRequest();
    getvalues_request->add_keys(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();

    // Keep checking the reply queue until a reply is there.
    while (true) {
        reply_queue_mutex_.lock();
        if (!reply_queue_.empty()) {
            NetworkTable::Reply reply = reply_queue_.front();
            reply_queue_.pop();
            reply_queue_mutex_.unlock();

            return reply.getvalues_reply().keyvaluepairs(0).value();
        }
        reply_queue_mutex_.unlock();
    }
}

std::map<std::string, NetworkTable::Value> NetworkTable::Connection::GetValues(std::set<std::string> keys) {
    auto *getvalues_request = new NetworkTable::GetValuesRequest();

    for (auto const &key : keys) {
        getvalues_request->add_keys(key);
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();

    // Keep checking the reply queue until a reply is there.
    while (true) {
        reply_queue_mutex_.lock();
        if (!reply_queue_.empty()) {
            NetworkTable::Reply reply = reply_queue_.front();
            reply_queue_.pop();
            reply_queue_mutex_.unlock();

            std::map<std::string, NetworkTable::Value> values;
            for (int i = 0; i < reply.getvalues_reply().keyvaluepairs_size(); i++) {
                std::string key = reply.getvalues_reply().keyvaluepairs(i).key();
                NetworkTable::Value value = reply.getvalues_reply().keyvaluepairs(i).value();
                values[key] = value;
            }

            return values;
        }
        reply_queue_mutex_.unlock();
    }
}

void NetworkTable::Connection::Subscribe(std::string key, void (*callback)(NetworkTable::Value value)) {
    callbacks_[key] = callback;

    auto *subscribe_request = new NetworkTable::SubscribeRequest();
    subscribe_request->set_key(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SUBSCRIBE);
    request.set_allocated_subscribe_request(subscribe_request);

    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();
}

void NetworkTable::Connection::Unsubscribe(std::string key) {
    auto *unsubscribe_request = new NetworkTable::UnsubscribeRequest();
    unsubscribe_request->set_key(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::UNSUBSCRIBE);
    request.set_allocated_unsubscribe_request(unsubscribe_request);

    request_queue_mutex_.lock();
    request_queue_.push(request);
    request_queue_mutex_.unlock();

    callbacks_.erase(key);
}

////////////////////// PRIVATE //////////////////////

void NetworkTable::Connection::Send(const NetworkTable::Request &request) {
    // Serialize the message to a string
    std::string serialized_request;
    request.SerializeToString(&serialized_request);

    // Send the string to the server.
    zmq::message_t message(serialized_request.length());
    memcpy(message.data(), serialized_request.data(), serialized_request.length());
    socket_.send(message);
}

bool NetworkTable::Connection::Receive(NetworkTable::Reply *reply) {
    zmq::message_t message;
    if (socket_.recv(&message, ZMQ_DONTWAIT) == true) {
        std::string serialized_reply(static_cast<char*>(message.data()), \
                                message.size());

        reply->ParseFromString(serialized_reply);

        return true;
    } else {
        return false;
    }
}

void NetworkTable::Connection::ManageSocket() {
    // First, send a request to Network Table Server
    // to get a location to connect to. This is done
    // using request/reply sockets.
    zmq::socket_t init_socket(context_, ZMQ_REQ);
    init_socket.connect("ipc:///tmp/sailbot/NetworkTable");

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

    while (is_running_) {
        // First, check to see if there are any requests
        // that need to be sent.
        request_queue_mutex_.lock();
        if (!request_queue_.empty()) {
            NetworkTable::Request request = request_queue_.front();
            request_queue_.pop();
            request_queue_mutex_.unlock();
            Send(request);
        } else {
            request_queue_mutex_.unlock();
        }

        // Check to see if there are any replies from the server.
        NetworkTable::Reply reply;
        if (Receive(&reply)) {
            if (reply.type() == NetworkTable::Reply::SUBSCRIBE \
                // If it's a subscribe reply,
                // just run the associated callback function.
                    && reply.has_subscribe_reply()) {
                std::string key = reply.subscribe_reply().key();
                NetworkTable::Value value = reply.subscribe_reply().value();

                // Even after sending an Unsubscribe request,
                // it can take a while for that request to be processed
                // if other processes are also sending requests to the server.
                // If one of these other requests happens to update
                // a value which was subscribed to by this process,
                // a SubscribeReply can still be sent by the server,
                // even though this process just sent an UnsubscribeRequest
                // to the server.
                if (callbacks_[key] != NULL) {
                    callbacks_[key](value);
                }
            } else {
                // Otherwise put it in the reply queue
                // where the main thread will get it.
                reply_queue_mutex_.lock();
                reply_queue_.push(reply);
                reply_queue_mutex_.unlock();
            }
        }
    }
}
