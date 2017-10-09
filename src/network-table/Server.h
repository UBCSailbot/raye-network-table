// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_SERVER_H_
#define NETWORK_TABLE_SERVER_H_

#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <zmq.hpp>

#include "GetValuesRequest.pb.h"
#include "Reply.pb.h"
#include "SetValuesRequest.pb.h"
#include "SubscribeRequest.pb.h"
#include "UnsubscribeRequest.pb.h"
#include "Value.pb.h"

namespace NetworkTable {
class Server {
 public:
    Server();

    /*
     * Starts the network table, which will then be able
     * to accept requests from clients.
     */
    void Run();

 private:
    /*
     * Creates a new ZMQ_PAIR socket,
     * and returns its location to the client
     * who asked for it.
     */
    void CreateNewConnection();

    /*
     * Handles a request on a ZMQ_PAIR socket.
     */
    void HandleRequest(zmq::socket_t *socket);

    /*
     * Helper functions to handle specific types
     * of requests. Some of these requests
     * must send back a reply, so the helper function
     * also needs a socket to send the reply to.
     */
    void SetValues(const NetworkTable::SetValuesRequest &request);

    void GetValues(const NetworkTable::GetValuesRequest &request, \
            zmq::socket_t *socket);

    void Subscribe(const NetworkTable::SubscribeRequest &request, \
            zmq::socket_t *socket);

    void Unsubscribe(const NetworkTable::UnsubscribeRequest &request, \
            zmq::socket_t *socket);

    void DisconnectSocket(zmq::socket_t *socket);

    /*
     * Returns value stored in values_ if it exists.
     * Otherwise returns a value with type NONE.
     * Use this instead of directly getting values
     * from the table.
     */
    NetworkTable::Value GetValueFromTable(std::string key);

    /*
     * Sets a value stored in values_ if it exists, creates
     * a new entry if the entry does not already exist.
     * Also sends an update any subscribers of the key.
     */
    void SetValueInTable(std::string key, const NetworkTable::Value &value);

    /*
     * Gets any sockets which have subscribed to key, and sends value to them.
     */
    void NotifySubscribers(std::string key, const NetworkTable::Value &value);

    /*
     * Serializes a network table reply,
     * then sends it on the socket.
     */
    void SendReply(const NetworkTable::Reply &reply, zmq::socket_t *socket);

    zmq::context_t context_;  // The context which sockets are created from.
    zmq::socket_t init_socket_;  // Used to connect to the server for the first time.
    std::vector<zmq::socket_t*> sockets_;  // Each socket is a connection to another process.
    std::unordered_map<std::string, NetworkTable::Value> values_;  // This is where the actual data is stored.
    std::unordered_map<std::string, \
        std::set<zmq::socket_t*>*> subscriptions_table_;  // maps from a key in the network table
                                                          // to a set of sockets subscribe to that key.

    int current_socket_number_ = 1;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_SERVER_H_
