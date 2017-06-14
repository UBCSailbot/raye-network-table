// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_H_
#define NETWORK_TABLE_H_

#include <string>
#include <zmq.hpp>

class NetworkTable {
  public:
    static const int kCentralControllerContext = 1;
    static const int kCentralControllerPort = 5555;
    /*
     * Gets a value from the network-table.
     * @param key What key entry to get.
     * @return The value of the key entry.
     */
    static std::string Get(std::string key);

    /*
     * Updates a value in the network-table.
     * @param key What key entry to update.
     * @param value What value to set the key entry to.
     * @return True if the update was successful, false otherwise.
     */
    static bool Set(std::string key, std::string value);

    /*
     * Connects the class member variable socket_.
     */
    static void Connect();

  private:
    static zmq::socket_t socket_;
};                             
                               
#endif  // NETWORK_TABLE_H_    
