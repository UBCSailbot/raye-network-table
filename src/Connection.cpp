// Copyright 2017 UBC Sailbot

#include "Connection.h"

#include <assert.h>
#include <chrono>
#include "GetValuesRequest.pb.h"
#include "SetValuesRequest.pb.h"


NetworkTable::Connection::Connection() : connected_(false), terminate_(false),
                                         timeout_(-1) {
}

void NetworkTable::Connection::Connect() {
    assert(!connected_);

    socket_thread_ = std::thread(&NetworkTable::Connection::ManageSocket, this);

    auto start_time = std::chrono::steady_clock::now();
    while (!connected_) {
        if (TimedOut(start_time)) {
            terminate_ = true;
            socket_thread_.join();
            terminate_ = false;
            throw TimeoutException("timed out");
        }
    }
}

void NetworkTable::Connection::Disconnect() {
    assert(connected_);

    terminate_ = true;
    socket_thread_.join();
    terminate_ = false;
}

////////////////////// PUBLIC //////////////////////

void NetworkTable::Connection::SetValue(std::string key, const NetworkTable::Value &value) {
    assert(connected_);

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
    request_queue_.push(\
            std::make_pair(boost::uuids::nil_uuid(), request));
    request_queue_mutex_.unlock();
}

void NetworkTable::Connection::SetValues(const std::map<std::string, NetworkTable::Value> &values) {
    assert(connected_);

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
    request_queue_.push(\
            std::make_pair(boost::uuids::nil_uuid(), request));
    request_queue_mutex_.unlock();
}

NetworkTable::Value NetworkTable::Connection::GetValue(std::string key) {
    assert(connected_);

    // Create a GetValuesRequest with a single key/value pair.
    auto *getvalues_request = new NetworkTable::GetValuesRequest();
    getvalues_request->add_keys(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    // We need to attach a uuid to the request
    // to ensure we get the correct reply.
    boost::uuids::uuid request_uuid = boost::uuids::random_generator()();

    request_queue_mutex_.lock();
    request_queue_.push(\
            std::make_pair(request_uuid, request));
    request_queue_mutex_.unlock();

    // Keep checking the reply queue until a reply is there.
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        reply_queue_mutex_.lock();
        if (!reply_queue_.empty()) {
            boost::uuids::uuid reply_uuid = reply_queue_.front().first;
            NetworkTable::Reply reply = reply_queue_.front().second;
            reply_queue_.pop();
            reply_queue_mutex_.unlock();

            // Check to make sure the reply has the same id as the
            // request we sent.
            // If not, just throw out the reply.
            if (reply_uuid == request_uuid) {
                return reply.getvalues_reply().keyvaluepairs(0).value();
            }
        }
        reply_queue_mutex_.unlock();

        if (TimedOut(start_time)) {
            throw TimeoutException("timed out");
        }
    }
}

std::map<std::string, NetworkTable::Value> NetworkTable::Connection::GetValues(std::set<std::string> keys) {
    assert(connected_);

    auto *getvalues_request = new NetworkTable::GetValuesRequest();

    for (auto const &key : keys) {
        getvalues_request->add_keys(key);
    }

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::GETVALUES);
    request.set_allocated_getvalues_request(getvalues_request);

    // We need to attach a uuid to the request
    // to ensure we get the correct reply.
    boost::uuids::uuid request_uuid = boost::uuids::random_generator()();

    request_queue_mutex_.lock();
    request_queue_.push(\
            std::make_pair(request_uuid, request));
    request_queue_mutex_.unlock();

    // Keep checking the reply queue until a reply is there.
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        reply_queue_mutex_.lock();
        if (!reply_queue_.empty()) {
            boost::uuids::uuid reply_uuid = reply_queue_.front().first;
            NetworkTable::Reply reply = reply_queue_.front().second;
            reply_queue_.pop();
            reply_queue_mutex_.unlock();

            // Check to make sure the reply has the same id as the
            // request we sent.
            // If not, just throw out the reply.
            if (reply_uuid == request_uuid) {
                std::map<std::string, NetworkTable::Value> values;
                for (int i = 0; i < reply.getvalues_reply().keyvaluepairs_size(); i++) {
                    std::string key = reply.getvalues_reply().keyvaluepairs(i).key();
                    NetworkTable::Value value = reply.getvalues_reply().keyvaluepairs(i).value();
                    values[key] = value;
                }

                return values;
            }
        }
        reply_queue_mutex_.unlock();

        if (TimedOut(start_time)) {
            throw TimeoutException("timed out");
        }
    }
}

void NetworkTable::Connection::Subscribe(std::string key, void (*callback)(NetworkTable::Value value)) {
    assert(connected_);

    callbacks_[key] = callback;

    auto *subscribe_request = new NetworkTable::SubscribeRequest();
    subscribe_request->set_key(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::SUBSCRIBE);
    request.set_allocated_subscribe_request(subscribe_request);

    request_queue_mutex_.lock();
    request_queue_.push(\
            std::make_pair(boost::uuids::nil_uuid(), request));
    request_queue_mutex_.unlock();
}

void NetworkTable::Connection::Unsubscribe(std::string key) {
    assert(connected_);

    auto *unsubscribe_request = new NetworkTable::UnsubscribeRequest();
    unsubscribe_request->set_key(key);

    NetworkTable::Request request;
    request.set_type(NetworkTable::Request::UNSUBSCRIBE);
    request.set_allocated_unsubscribe_request(unsubscribe_request);

    request_queue_mutex_.lock();
    request_queue_.push(\
            std::make_pair(boost::uuids::nil_uuid(), request));
    request_queue_mutex_.unlock();

    callbacks_.erase(key);
}

void NetworkTable::Connection::SetTimeout(int timeout) {
    timeout_ = timeout;
}

////////////////////// PRIVATE //////////////////////

void NetworkTable::Connection::Send(const NetworkTable::Request &request, zmq::socket_t *socket) {
    // Serialize the message to a string
    std::string serialized_request;
    request.SerializeToString(&serialized_request);

    // Send the string to the server.
    zmq::message_t message(serialized_request.length());
    memcpy(message.data(), serialized_request.data(), serialized_request.length());
    socket->send(message);
}

bool NetworkTable::Connection::Receive(NetworkTable::Reply *reply, zmq::socket_t *socket) {
    zmq::message_t message;
    if (socket->recv(&message, ZMQ_DONTWAIT) == true) {
        std::string serialized_reply(static_cast<char*>(message.data()), \
                                message.size());

        reply->ParseFromString(serialized_reply);

        return true;
    } else {
        return false;
    }
}

bool NetworkTable::Connection::TimedOut(std::chrono::steady_clock::time_point start_time) {
    if (timeout_ < 0) {
        return false;
    }

    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    return (duration.count() > timeout_);
}

void NetworkTable::Connection::ManageSocket() {
    /*
     * The context must be created here so that
     * its destructor is called when this function
     * ends. For more info see:
     * http://zeromq.org/whitepapers:0mq-termination
     */
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PAIR);
    // Ensure that the destructor for
    // socket does not block and simply
    // discards any messages it is still trying to
    // send/receive:
    socket.setsockopt(ZMQ_LINGER, 0);

    {
        // First, send a request to Network Table Server
        // to get a location to connect to. This is done
        // using request/reply sockets.
        zmq::socket_t init_socket(context, ZMQ_REQ);
        // Ensure that the destructor for
        // init_socket does not block and simply
        // discards any messages it is still trying to
        // send/receive:
        init_socket.setsockopt(ZMQ_LINGER, 0);
        if (timeout_ > 0) {
            init_socket.setsockopt(ZMQ_RCVTIMEO, timeout_);
        }
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
        if (!init_socket.recv(&reply)) {
            // If the server is not available,
            // just return.
            return;
        }

        // Connect to the ZMQ_PAIR socket which was created
        // by the server.
        std::string filepath = static_cast<char*>(reply.data());

        socket.connect(filepath);

        connected_ = true;
    }

    // This queue is used to store ids of
    // requests which have a non-nil id so that
    // the reply can have the same id.
    std::queue<boost::uuids::uuid> ids;

    while (!terminate_) {
        // First, check to see if there are any requests
        // that need to be sent.
        request_queue_mutex_.lock();
        if (!request_queue_.empty()) {
            boost::uuids::uuid request_uuid = request_queue_.front().first;
            NetworkTable::Request request = request_queue_.front().second;
            request_queue_.pop();
            request_queue_mutex_.unlock();

            if (!request_uuid.is_nil()) {
                ids.push(request_uuid);
            }

            Send(request, &socket);
        } else {
            request_queue_mutex_.unlock();
        }

        // Check to see if there are any replies from the server.
        NetworkTable::Reply reply;
        if (Receive(&reply, &socket)) {
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
                boost::uuids::uuid reply_uuid = ids.front();
                ids.pop();
                reply_queue_mutex_.lock();
                reply_queue_.push(std::make_pair(reply_uuid, reply));
                reply_queue_mutex_.unlock();
            }
        }
    }

    {
        // Disconnect from server.
        std::string request_body = "disconnect";
        zmq::message_t request(request_body.size()+1);
        memcpy(request.data(), request_body.c_str(), request_body.size()+1);
        socket.send(request);
    }
}
