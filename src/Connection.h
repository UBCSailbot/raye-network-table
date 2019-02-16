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
     * Set value in the network table, or create
     * it if it doesn't exist.
     */
    void SetValue(const std::string &uri, const NetworkTable::Value &values);

    /*
     * Set multiple values in the network table, or create
     * them if they don't exist.
     * @param values - map from string to Value, where the string
     *                 is the uri, and Value is what to set the
     *                 value at that uri to.
     */
    void SetValues(const std::map<std::string, NetworkTable::Value> &values);

    /*
     * Get value from the network table.
     */
    NetworkTable::Value GetValue(const std::string &uri);

    /*
     * Get multiple values from the network table.
     * The values are returned in the same order that
     * the uris were sent in.
     *
     * @param uris - which uris to get which values for.
     */
    std::map<std::string, NetworkTable::Value> GetValues(const std::set<std::string> &uris);

    /*
     * Get node from the network table.
     *
     */
    NetworkTable::Node GetNode(const std::string &uri);

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
    int Send(const NetworkTable::Request &request, zmq::socket_t *socket);

    int Send(const NetworkTable::Reply &reply, zmq::socket_t *socket);

    int Receive(NetworkTable::Reply *reply, zmq::socket_t *socket);

    int Receive(NetworkTable::Request *request, zmq::socket_t *socket);

    bool TimedOut(std::chrono::steady_clock::time_point start_time);

    /*
     * Checks to see if reply is an error message and
     * throws appropriate exception if it is.
     */
    void CheckForError(const NetworkTable::Reply &reply);

    void ManageSocket();

    zmq::context_t context_;
    zmq::socket_t mst_socket_;

    std::thread socket_thread_;  // This interacts with the socket.
    std::atomic_bool connected_;  // True when connected to the server.
                                  // This is set by the manage socket thread
                                  // and read by the main thread.

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
