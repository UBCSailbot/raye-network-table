// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include "Reply.pb.h"
#include "Request.pb.h"
#include "Value.pb.h"

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <queue>
#include <unordered_map>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    /*
     * Initializes a new connection to the server.
     */
    Connection();

    /*
     * Destructor.
     */
    ~Connection();

    /*
     * Set a value in the network table, or create it
     * if it doesn't exist already.
     */
    void SetValue(std::string key, const NetworkTable::Value &value);

    /*
     * Set multiple values in the network table, or create
     * them if they don't exist.
     */
    void SetValues(const std::map<std::string, NetworkTable::Value> &values);

    /*
     * Get a value from the network table.
     * Returns a NetworkTable::Value with type NONE
     * if no value at the specified key exists.
     */
    NetworkTable::Value GetValue(std::string key);

    /*
     * Get multiple values from the network table.
     */
    std::vector<NetworkTable::Value> GetValues(std::vector<std::string> keys);

    /*
     * Begin receiving updates on a key in
     * the network table. The callback function is
     * ran anytime a change occurs. The callback function
     * returns void, and takes a single argument of type
     * NetworkTable::Value.
     */
    void Subscribe(std::string key, void (*callback)(NetworkTable::Value value));

 private:
    void Send(const NetworkTable::Request &request);

    /*
     * Returns true only if a message was received.
     */
    bool Receive(NetworkTable::Reply *reply);

    void ManageSocket();

    zmq::context_t context_;
    zmq::socket_t socket_;  // Messages are sent/received over
                            // this socket.
    std::thread socket_thread_;  // This interacts with the socket.
    std::atomic_bool is_running_;  // This bool is simply used to
                                          // nicely terminate the socket_thread_.

    std::queue<NetworkTable::Request> request_queue_;
    std::queue<NetworkTable::Reply> reply_queue_;

    std::mutex request_queue_mutex_;
    std::mutex reply_queue_mutex_;

    std::unordered_map<std::string, \
        void (*)(NetworkTable::Value)> callbacks_;  // Map from table key
                                                    // to callback function.
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_CONNECTION_H_
