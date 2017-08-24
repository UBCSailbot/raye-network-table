// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include "Reply.pb.h"
#include "Request.pb.h"
#include "Value.pb.h"

#include <map>
#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    explicit Connection(std::string address = "ipc:///tmp/sailbot/NetworkTable");

    void SetValue(std::string key, const NetworkTable::Value &value);

    void SetValues(const std::map<std::string, NetworkTable::Value> &values);

    NetworkTable::Value GetValue(std::string key);

    std::vector<NetworkTable::Value> GetValues(std::vector<std::string> keys);

 private:
    void Send(const NetworkTable::Request &request);

    void Receive(NetworkTable::Reply *reply);

    zmq::context_t context_;
    zmq::socket_t socket_;  // Messages are sent/received over
                            // this socket.
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_CONNECTION_H_
