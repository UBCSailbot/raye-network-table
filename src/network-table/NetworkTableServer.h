// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_NETWORKTABLESERVER_H_
#define NETWORK_TABLE_NETWORKTABLESERVER_H_

#include <map>
#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Server {
 public:
    explicit Server(std::string address = "tcp://*:5555");

    /*
     * Starts the network table, which will then be able
     * to accept requests from clients.
     */
    void Run();

 private:
    std::map<std::string, std::string> table_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_NETWORKTABLESERVER_H_
