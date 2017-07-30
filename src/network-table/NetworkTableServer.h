// Copyright 2017 UBC Sailbot

#ifndef NETWORK_TABLE_NETWORKTABLESERVER_H_
#define NETWORK_TABLE_NETWORKTABLESERVER_H_

#include <string>
#include <zmq.hpp>

#include "Message.pb.h"

namespace NetworkTable {
class Server {
 public:
    /*
     * @param address Location of socket where other processes 
     *                can connect to Network Table Server.
     */
    explicit Server(std::string address = "ipc:///tmp/sailbot/NetworkTable");

    ~Server();

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

    void SetKey(zmq::socket_t *socket);

    void GetKey(zmq::socket_t *socket);

    zmq::context_t context_;
    zmq::socket_t init_socket_;
    std::vector<zmq::socket_t*> sockets_;

    int current_socket_number_ = 1;
};
}  // namespace NetworkTable

#endif  // NETWORK_TABLE_NETWORKTABLESERVER_H_
