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
    try {
        connection.Connect(100);
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Failed to connect to server" << std::endl;
        return 0;
    }

    int val = 4;
    int lat = 48;
    int lon = 235;
    NetworkTable::Value lat_val;
    lat_val.set_type(NetworkTable::Value::INT);
    NetworkTable::Value lon_val;
    lon_val.set_type(NetworkTable::Value::INT);
    lat_val.set_int_data(lat);
    lon_val.set_int_data(lon);
    connection.SetValue("gps/gprmc/longitude", lon_val);
    connection.SetValue("gps/gprmc/latitude", lat_val);
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

