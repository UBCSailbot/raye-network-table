// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_NETWORKTABLECONNECTION_H_
#define NETWORK_TABLE_NETWORKTABLECONNECTION_H_

#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    explicit Connection(std::string address = "tcp://localhost:5555");

    /*
     * Gets a value from the network-table.
     * @param key What key entry to get.
     * @return The value of the key entry.
     */
    std::string Get(std::string key);

    /*
     * Updates a value in the network-table.
     * @param key What key entry to update.
     * @param value What value to set the key entry to.
     * @return True if the update was successful, false otherwise.
     */
    void Set(std::string key, std::string value);

 private:
    zmq::context_t context_;
    zmq::socket_t socket_;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_NETWORKTABLECONNECTION_H_
