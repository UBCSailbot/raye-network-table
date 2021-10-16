// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Value.pb.h"
#include <iostream>
#include "Uri.h"

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

    float lat = 48.0;
    float lon = 235.0;
    NetworkTable::Value lat_val;
    lat_val.set_type(NetworkTable::Value::FLOAT);
    NetworkTable::Value lon_val;
    lon_val.set_type(NetworkTable::Value::FLOAT);
    lat_val.set_float_data(lat);
    lon_val.set_float_data(lon);
    connection.SetValue(GPS_CAN_LON, lon_val);
    connection.SetValue(GPS_CAN_LAT, lat_val);

    connection.Disconnect();
}

