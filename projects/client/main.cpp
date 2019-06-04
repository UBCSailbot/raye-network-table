// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table

#include "Connection.h"
#include "Value.pb.h"
#include "Node.pb.h"
#include "Exceptions.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

std::atomic_bool sensor_a_callback_called(false); // It would be better to
                                                      // check that it was called
                                                      // the correct number of times
                                                      // but this depends on what other
                                                      // clients are doing.
std::atomic_bool wrong_sensor_a_data_received(false);
void SensorACallback(NetworkTable::Node node) {
    sensor_a_callback_called = true;
    if (node.children().at("a").children().at("b").value().int_data() != 20) {
        std::cout << "Received sensor a data: "\
            << node.children().at("a").children().at("b").value().int_data() << std::endl;
        std::cout << "Expected: " << 20 << std::endl;
        wrong_sensor_a_data_received = true;
    }
}

std::atomic_bool sensor_b_callback_correct_data(false);
std::atomic_bool sensor_b_callback_called(false);
void SensorBCallback(NetworkTable::Node node) {
    sensor_b_callback_called = true;
    int a = node.children().at("a").value().int_data();
    if (a == 63) {
        sensor_b_callback_correct_data = true;
    } else {
        std::cout << "Received sensor b data: " << a << std::endl;
    }
}

/*
 * This is a stress test
 * for the network table server.
 * This program will return 0 if no tests failed,
 * otherwise it will return 1.
 */
int main() {
    int any_test_failed = 0;
    int num_queries = 5; // How many times the set of tests is run.
    const double precision = .1; // Precision to use when comparing doubles.

    NetworkTable::Connection connection;
    connection.SetTimeout(5000); // This will occasionally timeout
                                 // if the integration test uses 100
                                 // clients, which is good! We should
                                 // be testing the timeout functionality.
    try {
        connection.Connect();
    } catch (NetworkTable::TimeoutException) {
        std::cout << "Connection to server timed out" << std::endl;
        return 0;
    }

    try {
        connection.Subscribe("sensor_a", &SensorACallback);
    } catch (NetworkTable::TimeoutException) {}

    // Subscribe to something shallow.
    // Then, when something deep in the tree gets updated, we should still
    // receive the update
    try {
        connection.Subscribe("sensor_b", &SensorBCallback);
    } catch (NetworkTable::TimeoutException) {}
 
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // SET
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(20);

        std::map<std::string, NetworkTable::Value> values;
        values.insert(std::pair<std::string, NetworkTable::Value>("sensor_a/a/b", value));
        connection.SetValues(values);
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting sensor_a/a/b" << std::endl;
        any_test_failed = 1;
    }

    // SET
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(63);
        connection.SetValue("sensor_b/a", value);
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting sensor_b/a" << std::endl;
        any_test_failed = 1;
    }

    // SET
    try {
        NetworkTable::Value value;
        value.set_type(NetworkTable::Value::INT);
        value.set_int_data(100);
        connection.SetValue("sensor_c/a", value);
    } catch (NetworkTable::TimeoutException) {
    } catch (...) {
        std::cout << "Error setting sensor_c/a" << std::endl;
        any_test_failed = 1;
    }

    for (int i = 0; i < num_queries; i++) {
        // GET 
        try {
            NetworkTable::Value value = connection.GetValue("sensor_c/a");
            if (value.int_data() != 100) {
                std::cout << "Wrong sensor_c/a data" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting sensor_c/a" << std::endl;
            any_test_failed = 1;
        }
        // GET garbage
        try {
            // This should throw NodeNotFoundException
            NetworkTable::Value value = connection.GetValue("garbage");
            std::cout << "Got garbage without throwing exception" << std::endl;
            any_test_failed = 1;
        } catch (NetworkTable::TimeoutException) {
        } catch (NetworkTable::NodeNotFoundException) {
        } catch (...) {
            std::cout << "Error getting garbage" << std::endl;
            any_test_failed = 1;
        }
        // SET
        NetworkTable::Value val_a;
        val_a.set_type(NetworkTable::Value::DOUBLE);
        val_a.set_double_data(60.3225);
        NetworkTable::Value val_b;
        val_b.set_type(NetworkTable::Value::DOUBLE);
        val_b.set_double_data(155.9594);
        try {
            std::map<std::string, NetworkTable::Value> values;
            values.insert(std::pair<std::string, NetworkTable::Value>("sensor_d/a", val_a));
            values.insert(std::pair<std::string, NetworkTable::Value>("sensor_d/b", val_b));

            connection.SetValues(values);
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error setting sensor_d" << std::endl;
            any_test_failed = 1;
        }
        // GET latitude and longitude
        try {
            std::set<std::string> keys;
            keys.insert("sensor_d/a");
            keys.insert("sensor_d/b");
            
            auto values = connection.GetValues(keys);
            if (!(std::abs(values["sensor_d/a"].double_data() - val_a.double_data()) < precision)) {
                std::cout << "Error, wrong value for sensor_d/a" << std::endl;
                any_test_failed = 1;
            }
            if (!(std::abs(values["sensor_d/b"].double_data() - val_b.double_data()) < precision)) {
                std::cout << "Error, wrong value for sensor_d/b" << std::endl;
                any_test_failed = 1;
            }
        } catch (NetworkTable::TimeoutException) {
        } catch (...) {
            std::cout << "Error getting sensor_d" << std::endl;
            any_test_failed = 1;
        }
        // GET the whole tree
        try {
            auto root = connection.GetNode("/");
            if (!(std::abs(root.children().at("sensor_d").children().at("a").value().double_data() - val_a.double_data()) < precision)) {
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
    if (!sensor_a_callback_called) {
        std::cout << "Error, sensor_a_callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (wrong_sensor_a_data_received) {
        std::cout << "Wrong sensor a data received" << std::endl;
        any_test_failed = 1;
    }
    if (!sensor_b_callback_called) {
        std::cout << "Error, sensor_b_callback was never called" << std::endl;
        any_test_failed = 1;
    }
    if (!sensor_b_callback_correct_data && sensor_b_callback_called) {
        std::cout << "Error, sensor_b_callback received the wrong data" << std::endl;
        any_test_failed = 1;
    }

    connection.Disconnect();
    return any_test_failed;
}

