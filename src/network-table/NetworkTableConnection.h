// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_NETWORKTABLECONNECTION_H_
#define NETWORK_TABLE_NETWORKTABLECONNECTION_H_

#include <string>
#include <zmq.hpp>

#include "Message.pb.h"

namespace NetworkTable {
class Connection {
 public:
    explicit Connection(std::string address = "ipc:///tmp/sailbot/NetworkTable");

    std::string Send(NetworkTable::Message *message);

 private:
    zmq::context_t context_;
    zmq::socket_t socket_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_NETWORKTABLECONNECTION_H_
