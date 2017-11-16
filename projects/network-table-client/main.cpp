// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
// Returns the number of errors which occured.

#include "Connection.h"
#include "Value.pb.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

std::atomic_int num_errors(0); // Total number of errors which have occured.

void WindDirectionCallback(NetworkTable::Value value) {
    if (value.int_data() != 20 && value.int_data() != 40) {
        std::cout << "1\n";
        num_errors++;
    }
}

// This callback should never be called
void BadCallback(NetworkTable::Value value) {
    std::cout << "2\n";
    num_errors++;
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
    connection.SetTimeout(800); // This will occasionally timeout
                                 // if the integration test uses 100
                                 // clients, which is good! We should
                                 // be testing the timeout functionality.
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out." << std::endl;
        return 0;
    }

    // Subscribe to wind direction.
    // Note that even though we subscribe to badcallback first,
    // we override it with a call to winddirectioncallback.
    // This should cause no problems.
    connection.Subscribe("winddirection", &BadCallback);
    connection.Subscribe("winddirection", &WindDirectionCallback);

    // Subscribe to windspeed then immediately unsubscribe.
    connection.Subscribe("windspeed", &BadCallback);
    connection.Unsubscribe("windspeed");
 
    // Without this timeout, it is possible for BadCallback to be called.
    // What happens in between the call to subscribe with BadCallback, and
    // the time to unsubscribe, another process can change the value
    // of windspeed or winddirection, causing BadCallback to get called.
    std::this_thread::sleep_for(std::chrono::seconds(3));

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
            std::cout << "2\n";
            num_errors++;
        }

        // SET windspeed
        try {
            NetworkTable::Value value;
            value.set_type(NetworkTable::Value::INT);
            value.set_int_data(100);
            connection.SetValue("windspeed", value);
        } catch (...) {
            std::cout << "4\n";
            num_errors++;
        }
        // GET windspeed
        try {
            NetworkTable::Value value = connection.GetValue("windspeed");
            // In this case I know that the value of the windspeed should be.
            // Normally it is possible for other processes to be modifying
            // the data, so there is no way to know what it should be.
            if (value.int_data() != 100) {
                std::cout << "5\n";
                num_errors++;
            }
        } catch (NetworkTable::TimeoutException) {
            std::cout << "GetValue timed out" << std::endl;
        } catch (...) {
        std::cout << "6\n";
            num_errors++;
        }
        // GET garbage
        try {
            NetworkTable::Value value = connection.GetValue("garbage");
            if (value.type() != NetworkTable::Value::NONE) {
                std::cout << "7\n";
                num_errors++;
            }
        } catch (NetworkTable::TimeoutException) {
            std::cout << "GetValue timed out" << std::endl;
        } catch (...) {
            std::cout << "8\n";
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
            std::cout << "9\n";
            num_errors++;
        }
        // GET latitude and longitude
        try {
            std::set<std::string> keys;
            keys.insert("latitude");
            keys.insert("longitude");
            
            auto values = connection.GetValues(keys);
            if (!(std::abs(values["latitude"].double_data() - latitude.double_data()) < precision)) {
                std::cout << "10\n";
                num_errors++;
            }
            if (!(std::abs(values["longitude"].double_data() - longitude.double_data()) < precision)) {
                std::cout << "11\n";
                num_errors++;
            }
        } catch (NetworkTable::TimeoutException) {
            std::cout << "GetValue timed out" << std::endl;
        } catch (...) {
            std::cout << "12\n";
            num_errors++;
        }
    }

    connection.Disconnect();
    return num_errors;
}
