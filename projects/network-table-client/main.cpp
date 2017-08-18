// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
// Returns the number of errors which occured.

#include <iostream>
#include <string>

#include "network-table/Connection.h"
#include "Message.pb.h"
#include "SetValueAction.pb.h"

int main() {
    NetworkTable::Value *value = new NetworkTable::Value();
    value->set_type(NetworkTable::Value::STRING);
    value->set_string_data("hello_value");

    NetworkTable::SetValueAction *setvalue_action = new NetworkTable::SetValueAction();
    setvalue_action->set_key("hello_key");
    setvalue_action->set_allocated_value(value);

    NetworkTable::Message message;
    message.set_action(NetworkTable::Message::SETVALUE);
    message.set_allocated_setvalue_action(setvalue_action);

    NetworkTable::Connection connection;
    connection.Send(message);

    /*
     * For some reason, deleting value or
     * setvalue_action causes a segfault.
     */

    return 0;
}
