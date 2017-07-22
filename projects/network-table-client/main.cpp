// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
//

#include <iostream>
#include <network-table/NetworkTableConnection.h>

int main() {
    NetworkTable::Connection connection;
    std::string reply = connection.Send("Hey it's alex");
    std::cout << reply << std::endl;
}
