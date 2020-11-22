// Copyright 2017 UBC Sailbot

#include "Server.h"
#include "Exceptions.h"

#include <iostream>

int main() {
    NetworkTable::Server server;
    // WELCOME_DIRECTORY passed in as a compile flag
    // from cmake. You won't find its definition in the code
    std::cout << "Network table is running at " << WELCOME_DIRECTORY << std::endl;
    try {
        server.Run();
    } catch (NetworkTable::InterruptedException) {
        std::cout << "Network table DONE" << std::endl;
    }
}
