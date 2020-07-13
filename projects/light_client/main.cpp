// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Value.pb.h"
#include <iostream>

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

    int val = 4;
    while (true) {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(val++);

        connection.SetValue("/wind_sensor_0/iimwv/wind_speed", value);

        std::cout << "Set /wind_sensor_0/iimwv/wind_speed to " << val << std::endl;
        sleep(1);

        if (val > 10) {
            val = 4;
        }
    }

    connection.Disconnect();
}

