// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include <string>
#include <zmq.hpp>

class NetworkTableConnection {
 public:
    /*
     * Constructor where connection
     * to the network table is initialized.
     */
    NetworkTableConnection(std::string address = "tcp://localhost:5555");

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

#endif  // NETWORK_TABLE_CONNECTION_H_
