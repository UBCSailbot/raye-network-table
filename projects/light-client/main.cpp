// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Value.pb.h"

/*
 * This program fakes 
 * a connected client.
 */
int main() {
    NetworkTable::Connection connection;
    connection.SetTimeout(100);
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to server" << std::endl;
        return 0;
    }

    int val = 0;
    while (true) {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(val++);

        connection.SetValue("/solar_panel/charge", value);

        std::cout << "Set /solar_panel/charge to " << val << std::endl;
        sleep(1);
    }
    
    connection.Disconnect();
}

