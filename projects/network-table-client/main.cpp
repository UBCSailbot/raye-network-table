// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
//

#include <iostream>
#include <string>

#include "network-table/Connection.h"
#include "Message.pb.h"
#include "SetKeyAction.pb.h"

int main() {
    NetworkTable::Message* message = new NetworkTable::Message;
    message->set_action(NetworkTable::Message::SETKEY);
    NetworkTable::SetKeyAction* setkey_action = new NetworkTable::SetKeyAction;
    setkey_action->set_uri("alex");
    message->set_allocated_setkey_action(setkey_action);

    NetworkTable::Connection connection;
    std::string response = connection.Send(message);

    std::cout << response << std::endl;
    
    delete message;
}
