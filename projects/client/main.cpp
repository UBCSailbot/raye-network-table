// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Value.pb.h"
#include "Node.pb.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

std::atomic_bool winddirectioncallback_called(false); // It would be better to
                                                      // check that it was called
                                                      // the correct number of times
                                                      // but this depends on what other
                                                      // clients are doing.
std::atomic_bool wrong_wind_data_received(false);
void WindDirectionCallback(NetworkTable::Node node) {
    winddirectioncallback_called = true;
    if (node.value().int_data() != 20 && node.value().int_data() != 40) {
        std::cout << "Received wind direction data:" << node.value().int_data() << std::endl;
        std::cout << "Expected: " << 20 << " or " << 40 << std::endl;
        wrong_wind_data_received = true;
    }
}

std::atomic_bool batterycallback_called(false);
void BatteryCallback(NetworkTable::Node node) {
        batterycallback_called = true;
}

// This callback should never be called
std::atomic_bool badcallback_called(false);
void BadCallback(NetworkTable::Node node) {
    std::cout << "Called bad callback function" << std::endl;
    badcallback_called = true;
}

/*
 * This is a stress test
 * for the network table server.
 * This program will return 0 if no tests failed,
 * otherwise it will return 1.
 */
int main() {
    NetworkTable::Node node;
    int any_test_failed = 0;
    int num_queries = 5; // How many times the set of tests is run.
    const double precision = .1; // Precision to use when comparing doubles.

    NetworkTable::Connection connection;
    connection.SetTimeout(2000); // This will occasionally timeout
                                 // if the integration test uses 100
                                 // clients, which is good! We should
                                 // be testing the timeout functionality.
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out" << std::endl;
        return 0;
    }

    // Subscribe to wind direction.
    // Note that even though we subscribe to badcallback first,
    // we override it with a call to winddirectioncallback.
    connection.Subscribe("winddirection", &BadCallback);
    connection.Subscribe("winddirection", &WindDirectionCallback);

    // Subscribe to windspeed then immediately unsubscribe.
    connection.Subscribe("bad", &BadCallback);
    connection.Unsubscribe("bad");

    // Subscribe to something shallow.
    // Then, when something deep in the tree gets updated, we should still
    // receive the update
    connection.Subscribe("batteries", &BatteryCallback);
 
    // Without this timeout, it is possible for BadCallback to be called.
    // What happens in between the call to subscribe with BadCallback, and
    // the time to unsubscribe, another process can change the value
    // of windspeed or winddirection, causing BadCallback to get called.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // SET battery info
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::DOUBLE);
        value.set_double_data(.663);
        connection.SetValue("batteries/BAT0/charge_remaining", value);
    } catch (...) {
        std::cout << "Error setting battery info" << std::endl;
        any_test_failed = 1;
    }

    // SET wind direction
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(20);
        connection.SetValue("winddirection", value);
    } catch (...) {
        std::cout << "Error setting wind direction" << std::endl;
        any_test_failed = 1;
    }

    // SET windspeed
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(100);
        connection.SetValue("windspeed", value);
    } catch (...) {
        std::cout << "Error setting wind speed" << std::endl;
        any_test_failed = 1;
    }

    for (int i = 0; i < num_queries; i++) {
        // GET windspeed
        try {
            NetworkTable::Value value = connection.GetValue("windspeed");
            // In this case I know that the value of the windspeed should be.
            // Normally it is possible for other processes to be modifying
            // the data, so there is no way to know what it should be.
            if (value.int_data() != 100) {
                std::cout << "Wrong wind speed data" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting wind speed" << std::endl;
            any_test_failed = 1;
        }
        // GET garbage
        try {
            NetworkTable::Value value = connection.GetValue("garbage");
            if (value.type() != NetworkTable::Value::NONE) {
                std::cout << "Error, got a value for garbage" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting garbage" << std::endl;
            any_test_failed = 1;
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
            values.insert(std::pair<std::string, NetworkTable::Value>("gps/latitude", latitude));
            values.insert(std::pair<std::string, NetworkTable::Value>("gps/longitude", longitude));

            connection.SetValues(values);
        } catch (...) {
            std::cout << "Error setting latitude and longitude" << std::endl;
            any_test_failed = 1;
        }
        // GET latitude and longitude
        try {
            std::set<std::string> keys;
            keys.insert("gps/latitude");
            keys.insert("gps/longitude");
            
            auto values = connection.GetValues(keys);
            if (!(std::abs(values["gps/latitude"].double_data() - latitude.double_data()) < precision)) {
                std::cout << "Error, wrong value for gps/latitude" << std::endl;
                any_test_failed = 1;
            }
            if (!(std::abs(values["gps/longitude"].double_data() - longitude.double_data()) < precision)) {
                std::cout << "Error, wrong value for gps/longitude" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting gps/latitude and gps/longitude" << std::endl;
            any_test_failed = 1;
        }
        // GET the whole tree
        try {
            auto root = connection.GetNode("/");
            if (!(std::abs(root.children().at("gps").children().at("latitude").value().double_data() - latitude.double_data()) < precision)) {
                std::cout << "Error, gps/latitude was wrong when getting whole tree" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting whole tree" << std::endl;
            any_test_failed = 1;
        }
    }

    // Check that everything went OK with the callbacks
    if (!winddirectioncallback_called) {
        std::cout << "Error, wind direction callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (wrong_wind_data_received) {
        any_test_failed = 1;
    }
    if (!batterycallback_called) {
        std::cout << "Error, battery callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (badcallback_called) {
        std::cout << "Error, bad callback was called" << std::endl;
        any_test_failed = 1;
    }

    connection.Disconnect();
    return any_test_failed;
}

