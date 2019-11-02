// Copyright 2017 UBC Sailbot

#include "HelpTest.h"
#include "Help.h"

const double precision = 0.001;

TEST_F(HelpTest, GetSetTest) {
    NetworkTable::Node root;

    // Test setting an integer
    NetworkTable::Value windspeed;
    windspeed.set_type(NetworkTable::Value::INT);
    windspeed.set_int_data(5);

    NetworkTable::SetNode("/wind/speed", windspeed, &root);
    EXPECT_EQ(NetworkTable::GetNode("wind/speed", &root).value().int_data(), \
              windspeed.int_data());

    // Try getting the trees root, and make sure it has the correct node(s)
    NetworkTable::Node node = NetworkTable::GetNode("/", &root);
    EXPECT_EQ(root.children().at("wind").children().at("speed").value().int_data(), \
             windspeed.int_data());

    // Try getting a child of the root and make sure it has the correct node
    NetworkTable::Node child_of_root  = NetworkTable::GetNode("/wind", &root);
    EXPECT_EQ(child_of_root.children().at("speed").value().int_data(), \
             windspeed.int_data());
}

TEST_F(HelpTest, WriteLoadTest) {
    NetworkTable::Node root;

    // Set some stuff in the root
    NetworkTable::Value windspeed;
    windspeed.set_type(NetworkTable::Value::INT);
    windspeed.set_int_data(5);
    NetworkTable::SetNode("/wind/speed", windspeed, &root);

    NetworkTable::Value gps_lat;
    gps_lat.set_type(NetworkTable::Value::FLOAT);
    gps_lat.set_float_data(-41.11);
    NetworkTable::SetNode("/gps/lat", gps_lat, &root);

    // Write the root to the disk
    NetworkTable::Write("/tmp/testtree.txt", root);

    // Load the root from the disk
    NetworkTable::Node new_root = NetworkTable::Load("/tmp/testtree.txt");

    EXPECT_EQ(NetworkTable::GetNode("wind/speed", &new_root).value().int_data(), \
              windspeed.int_data());

    EXPECT_NEAR(NetworkTable::GetNode("gps/lat", &new_root).value().float_data(), \
              gps_lat.float_data(), precision);
}
