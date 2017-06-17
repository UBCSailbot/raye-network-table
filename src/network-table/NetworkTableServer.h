// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_NETWORKTABLESERVER_H_
#define NETWORK_TABLE_NETWORKTABLESERVER_H_

#include <string>
#include <map>

namespace NetworkTable {
class Server {
 public:
    /*
     * Starts the network table, which will then be able
     * to accept requests from clients.
     */
    void Run();

 private:
    std::map<std::string, std::string> table_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_NETWORKTABLESERVER_H_
