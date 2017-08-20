// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include "Reply.pb.h"
#include "Request.pb.h"

#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    explicit Connection(std::string address = "ipc:///tmp/sailbot/NetworkTable");

    void Send(const NetworkTable::Request &request);

    void Receive(NetworkTable::Reply *reply);

 private:
    zmq::context_t context_;
    zmq::socket_t socket_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_CONNECTION_H_
