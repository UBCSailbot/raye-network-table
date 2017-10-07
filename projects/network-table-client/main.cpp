// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
// Returns the number of errors which occured.

#include "network-table/Connection.h"
#include "Value.pb.h"

#include <atomic>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

std::atomic_int num_errors(0); // Total number of errors which have occured.

void WindDirectionCallback(NetworkTable::Value value) {
    if (value.int_data() != 20 && value.int_data() != 40) {
        num_errors++;
    }
}

/*
 * This is a basic "stress test"
 * for the network table server.
 * The total number of errors
 * which occur when querying the network
 * table is returned.
 */
int main() {
    int num_queries = 10; // How many times the set of tests is run.
    const double precision = .1; // Precision to use when comparing doubles.

    NetworkTable::Connection connection;

    // Subscribe to wind direction.
    connection.Subscribe("winddirection", &WindDirectionCallback);

    for (int i = 0; i < num_queries; i++) {
        // SET wind direction
        try {
            NetworkTable::Value value;
            value.set_type(NetworkTable::Value::INT);
            if (i%2 == 0) {
                value.set_int_data(20);
            } else {
                value.set_int_data(40);
            }
            connection.SetValue("winddirection", value);
        } catch (...) {
            num_errors++;
        }

        // SET windspeed
        try {
            NetworkTable::Value value;
            value.set_type(NetworkTable::Value::INT);
            value.set_int_data(100);
            connection.SetValue("windspeed", value);
        } catch (...) {
            num_errors++;
        }
        // GET windspeed
        try {
            NetworkTable::Value value = connection.GetValue("windspeed");
            // In this case I know that the value of the windspeed should be.
            // Normally it is possible for other processes to be modifying
            // the data, so there is no way to know what it should be.
            if (value.int_data() != 100) {
                num_errors++;
            }
        } catch (...) {
            num_errors++;
        }
        // GET garbage
        try {
            NetworkTable::Value value = connection.GetValue("garbage");
            if (value.type() != NetworkTable::Value::NONE) {
                num_errors++;
            }
        } catch (...) {
            num_errors++;
        }
        // SET latitude and longitude
        NetworkTable::Value latitude;
        latitude.set_type(NetworkTable::Value::DOUBLE);
        latitude.set_double_data(60.3225);
        NetworkTable::Value longitude;
        longitude.set_type(NetworkTable::Value::DOUBLE);
        longitude.set_double_data(155.9594);
        try {
            std::map<std::string, NetworkTable::Value> values;
            values.insert(std::pair<std::string, NetworkTable::Value>("latitude", latitude));
            values.insert(std::pair<std::string, NetworkTable::Value>("longitude", longitude));

            connection.SetValues(values);
        } catch (...) {
            num_errors++;
        }
        // GET latitude and longitude
        try {
            std::set<std::string> keys;
            keys.insert("latitude");
            keys.insert("longitude");
            
            auto values = connection.GetValues(keys);
            if (!(std::abs(values["latitude"].double_data() - latitude.double_data()) < precision)) {
                num_errors++;
            }
            if (!(std::abs(values["longitude"].double_data() - longitude.double_data()) < precision)) {
                num_errors++;
            }
        } catch (...) {
            num_errors++;
        }
    }

    return num_errors;
}
