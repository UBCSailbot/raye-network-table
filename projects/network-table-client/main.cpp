// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
//

#include <iostream>
#include <network-table/NetworkTableConnection.h>

int main() {
    NetworkTableConnection networkTableConnection;

    std::string key = "wind/speed";
    std::string value = "4.12";

    std::cout << "SET: " << key << " to " << value << std::endl;
    networkTableConnection.Set("wind/speed", "4.12");

    std::cout << "GET: " << key << std::endl;
    std::string recieved_message = networkTableConnection.Get(key); 

    std::cout << "Recieved reply: " << recieved_message << std::endl;
}
