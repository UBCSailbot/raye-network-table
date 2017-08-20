// Copyright 2017 UBC Sailbot
//
// Sends requests to the network table
// Returns the number of errors which occured.

#include "network-table/Connection.h"
#include "Reply.pb.h"
#include "Request.pb.h"
#include "GetValueReply.pb.h"
#include "GetValueRequest.pb.h"
#include "SetValueRequest.pb.h"
#include "Value.pb.h"

#include <iostream>
#include <string>

/*
 * This is a basic "stress test"
 * for the network table server.
 * The total number of errors
 * which occur when querying the network
 * table is returned.
 */
int main() {
    int num_queries = 10;
    int num_errors = 0;

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
    }

    return num_errors;
}
