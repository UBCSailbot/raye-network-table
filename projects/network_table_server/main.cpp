// Copyright 2017 UBC Sailbot

#include "Server.h"
#include "Exceptions.h"

#include <iostream>

int main() {
    NetworkTable::Server server;
    try {
        server.Run();
    } catch (NetworkTable::InterruptedException) {
        std::cout << "Network table DONE" << std::endl;
    }
}
