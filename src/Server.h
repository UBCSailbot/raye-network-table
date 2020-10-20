// Copyright 2017 UBC Sailbot

#ifndef SERVER_H_
#define SERVER_H_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <zmq.hpp>

#include "GetNodesRequest.pb.h"
#include "Reply.pb.h"
#include "SetValuesRequest.pb.h"
#include "SubscribeRequest.pb.h"
#include "UnsubscribeRequest.pb.h"
#include "Help.h"
#include "Value.pb.h"

namespace NetworkTable {
class Server {
typedef std::shared_ptr<zmq::socket_t> socket_ptr;

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
     * If there are any abandoned client sockets,
     * this function will get the server to connect to them.
     * This can happen if the server crashes.
     */
    void ReconnectAbandonedSockets();

    /*
     * Handles a request on a ZMQ_PAIR socket.
     */
    void HandleRequest(socket_ptr socket);

    /*
     * Helper functions to handle specific types
     * of requests. Some of these requests
     * must send back a reply, so the helper function
     * also needs a socket to send the reply to.
     */
    void SetValues(const NetworkTable::SetValuesRequest &request, \
            socket_ptr socket);

    void GetNodes(const NetworkTable::GetNodesRequest &request, \
            std::string id, socket_ptr socket);

    void Subscribe(const NetworkTable::SubscribeRequest &request, \
            socket_ptr socket);

    void Unsubscribe(const NetworkTable::UnsubscribeRequest &request, \
            socket_ptr socket);

    void DisconnectSocket(socket_ptr socket);

    /*
     * Sets a value stored in root_ if it exists, creates
     * a new entry if the entry does not already exist.
     * Also sends an update any subscribers of the key.
     */
    void SetValueInTable(std::string key, const NetworkTable::Value &value);

    /*
     * Gets any sockets which have subscribed to key, and sends value to them.
     * Also include who caused this notify.
     */
    void NotifySubscribers(const std::set<std::string> &uris, \
            const google::protobuf::Map<std::string, NetworkTable::Value> &diffs, \
            socket_ptr responsible_socket);

    /*
     * Serializes a network table reply,
     * then sends it on the socket.
     */
    void SendReply(const NetworkTable::Reply &reply, socket_ptr socket);

    /*
     * Sends an already serialized reply.
     * If you are sending a single reply to many sockets, you can
     * avoid unnecessary serialization.
     */
    void SendSerializedReply(const std::string &serialized_reply, socket_ptr socket);

    /*
     * Sends an ack reply,
     * so the client knows its request was recieved.
     */
    void Ack(const std::string &id, socket_ptr socket);

    /*
     * Save subscription table to disk.
     */
    void WriteSubscriptionTable();

    /*
     * Load subscription table from disk.
     */
    void LoadSubscriptionTable();

    /*
     * Given a socket, returns a filesystem path to that socket.
     */
    std::string GetEndpoint(socket_ptr socket);

    zmq::context_t context_;  // The context which sockets are created from.
    zmq::socket_t welcome_socket_;  // Used to connect to the server for the first time.
    std::vector<socket_ptr> sockets_;  // Each socket is a connection to another process.
    NetworkTable::Node root_;  // This is where the actual data is stored.
    std::unordered_map<std::string, \
        std::set<socket_ptr>> subscriptions_table_;  // maps from a key in the network table
                                                      // to a set of sockets subscribe to that key.

    // location of welcoming socket
    const std::string kWelcome_Directory_ = WELCOME_DIRECTORY;  // NOLINT(runtime/string)

    // location of client sockets
    const std::string kClients_Directory_ = kWelcome_Directory_ + "clients/";  // NOLINT(runtime/string)

    // where root_ is saved (in case of crash)
    const std::string kRootFilePath_ = kWelcome_Directory_ + "root_.txt";  // NOLINT(runtime/string)

    // where info about who is subcribed to what is saved (in case of crash)
    // sorry for stupid NOLINT stuff, it has to be on the same
    // line as the lint error. try to ignore it
    const std::string kSubscriptionsTableFilePath_ = /* NOLINT(runtime/string) */\
        kWelcome_Directory_  + "subscriptions_table_.txt";
};
}  // namespace NetworkTable

#endif  // SERVER_H_
