// Copyright 2017 UBC Sailbot
//
// View the structure of the data in the network table.
// The tree structure is printed to stdout.

#include "Connection.h"
#include "Help.h"

#include <iostream>

int main() {
    NetworkTable::Connection connection;
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out." << std::endl;
        return 0;
    }

    NetworkTable::Node root = connection.GetNode("/");
    std::cout << "sizeof serialized root: " << root.SerializeAsString().size() << std::endl;
    std::cout << "sizeof serialized nt: " << \
        NetworkTable::RootToNt(root).SerializeAsString().size() << std::endl;
    NetworkTable::PrintNode(root);

    connection.Disconnect();
}
