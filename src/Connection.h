// Copyright 2017 UBC Sailbot

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include "Reply.pb.h"
#include "Request.pb.h"
#include "Node.pb.h"
#include "Value.pb.h"

#include <atomic>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <exception>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <queue>
#include <utility>
#include <zmq.hpp>

namespace NetworkTable {
class Connection {
 public:
    Connection();

    void Connect();

    void Disconnect();

    /*
     * Set a value in the network table, or create it
     * if it doesn't exist already.
     *
     * @param uri - which uri in the table to set.
     * @param value - what value to set the value at that uri to.
     */
    void SetValue(std::string uri, const NetworkTable::Value &value);

    /*
     * Set multiple values in the network table, or create
     * them if they don't exist.
     * @param values - map from string to Value, where the string
     *                 is the uri, and Value is what to set the
     *                 value at that uri to.
     */
    void SetValues(const std::map<std::string, NetworkTable::Value> &values);

    /*
     * Get a value from the network table.
     * Returns a value with NetworkTable::Value type NONE
     * if no value at the specified uri exists.
     * 
     * @param uri - returns the value located at this uri.
     */
    NetworkTable::Value GetValue(const std::string &uri);

    /*
     * Get a node from the network table.
     * Returns a node with NetworkTable::Value type NONE
     * if no node at the specified uri exists.
     * 
     * @param uri - returns the value located at this uri.
     */
    NetworkTable::Node GetNode(const std::string &uri);

    /*
     * Get multiple values from the network table.
     * The values are returned in the same order that
     * the uris were sent in.
     *
     * @param uris - which uris to get which values for.
     */
    std::map<std::string, NetworkTable::Value> GetValues(const std::set<std::string> &uris);

    /*
     * Get multiple nodes from the network table.
     * The nodes are returned in the same order that
     * the uris were sent in.
     *
     * @param uris - which uris to get which values for.
     */
    std::map<std::string, NetworkTable::Node> GetNodes(const std::set<std::string> &uris);

    /*
     * Begin receiving updates on a uri in
     * the network table. The callback function is
     * ran anytime a change occurs.
     *
     * @param uri - what uri to subscribe to
     * @param callback - what function to call when the uri node changes.
     *                   the function takes a single argument: the new node.
     */
    void Subscribe(std::string uri, void (*callback)(NetworkTable::Node node));

    /*
     * Stop receiving updates on a uri in the network table.
     * Has no effect if the uri is not subscribed to.
     */
    void Unsubscribe(std::string uri);

    /*
     * Set timeout for all methods which communicate with server.
     * @param timeout - timeout in milliseconds; if negative then there is no timeout
     */
    void SetTimeout(int timeout);

 private:
    void Send(const NetworkTable::Request &request, zmq::socket_t *socket);

    /*
     * Returns true only if a message was received.
     */
    bool Receive(NetworkTable::Reply *reply, zmq::socket_t *socket);

    bool TimedOut(std::chrono::steady_clock::time_point start_time);

    void ManageSocket();

    std::thread socket_thread_;  // This interacts with the socket.
    std::atomic_bool connected_;  // True when connected to the server.
                                  // This is set by the manage socket thread
                                  // and read by the main thread.
    std::atomic_bool terminate_;  // This is used to gracefully
                                  // terminate the manage socket thread.
                                  // It is set by the main thread
                                  // and read by the manage socket thread.

    // The request_queue_ is a queue
    // containing pairs of uuids and
    // network table requests.
    // The uuid is used to link a request to a reply.
    std::queue< \
        std::pair< \
            boost::uuids::uuid, \
            NetworkTable::Request>> request_queue_;
    std::queue< \
        std::pair< \
            boost::uuids::uuid, \
            NetworkTable::Reply>> reply_queue_;

    std::mutex request_queue_mutex_;
    std::mutex reply_queue_mutex_;

    std::map<std::string, \
        void (*)(NetworkTable::Node)> callbacks_;  // Map from table uri
                                                   // to callback function.
    int timeout_;  // How long to wait before throwing an exception when
                   // communicating with server. This is in milliseconds.
};

/*
 * This exception can be thrown if a request to the server times out.
 * A timeout can occur because the server is busy, or because
 * the connection is lost
 */
class TimeoutException : public std::runtime_error {
 public:
     explicit TimeoutException(char* what) : std::runtime_error(what) { }
};

}  // namespace NetworkTable

#endif  // CONNECTION_H_
