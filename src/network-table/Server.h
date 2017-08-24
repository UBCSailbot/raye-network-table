// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_SERVER_H_
#define NETWORK_TABLE_SERVER_H_

#include "GetValueRequest.pb.h"
#include "GetValuesRequest.pb.h"
#include "Reply.pb.h"
#include "SetValueRequest.pb.h"
#include "SetValuesRequest.pb.h"
#include "Value.pb.h"

#include <map>
#include <string>
#include <zmq.hpp>

namespace NetworkTable {
class Server {
 public:
    /*
     * @param address Location of socket where other processes 
     *                can connect to Network Table Server.
     */
    explicit Server(std::string address = "ipc:///tmp/sailbot/NetworkTable");

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
    void HandleNewConnection();

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
    void SetValue(const NetworkTable::SetValueRequest &request);

    void SetValues(const NetworkTable::SetValuesRequest &request);

    void GetValue(const NetworkTable::GetValueRequest &request, \
            zmq::socket_t *socket);

    void GetValues(const NetworkTable::GetValuesRequest &request, \
            zmq::socket_t *socket);

    /*
     * Returns value stored in values_ if it exists.
     * Otherwise returns a value with type NONE.
     */
    NetworkTable::Value GetValue(std::string key);

    /*
     * Serializes a network table reply,
     * then sends it on the socket.
     */
    void SendReply(const NetworkTable::Reply &reply, zmq::socket_t *socket);

    zmq::context_t context_;  // The context which sockets are created from.
    zmq::socket_t init_socket_;  // Used to connect to the server for the first time.
    std::vector<zmq::socket_t> sockets_;  // Each socket is a connection to another process.
    std::map<std::string, NetworkTable::Value> values_;  // This is where the actual data is stored.

    int current_socket_number_ = 1;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_SERVER_H_
