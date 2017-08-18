// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include "Message.pb.h"

#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    explicit Connection(std::string address = "ipc:///tmp/sailbot/NetworkTable");

    void Send(const NetworkTable::Message &message);

 private:
    zmq::context_t context_;
    zmq::socket_t socket_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_CONNECTION_H_
