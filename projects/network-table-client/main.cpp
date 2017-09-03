// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
// Returns the number of errors which occured.

#include "network-table/Connection.h"
#include "Value.pb.h"

#include <cmath>
#include <iostream>
#include <map>
#include <string>

/*
 * This is a basic "stress test"
 * for the network table server.
 * The total number of errors
 * which occur when querying the network
 * table is returned.
 */
int main() {
    int num_queries = 10; // How many times the set of tests is run.
    int num_errors = 0; // Total number of errors which have occured.
    const double precision = .1; // Precision to use when comparing doubles.

    NetworkTable::Connection connection;

    for (int i = 0; i < num_queries; i++) {
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
            std::vector<std::string> keys;
            keys.push_back("latitude");
            keys.push_back("longitude");
            
            auto values = connection.GetValues(keys);
            if (!(std::abs(values[0].double_data() - latitude.double_data()) < precision)) {
                num_errors++;
            }
            if (!(std::abs(values[1].double_data() - longitude.double_data()) < precision)) {
                num_errors++;
            }
        } catch (...) {
            num_errors++;
        }
    }

    return num_errors;
}
