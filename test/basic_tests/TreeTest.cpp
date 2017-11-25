// Copyright 2017 UBC Sailbot

#include "TreeTest.h"
#include "Tree.h"

const double precision = 0.001;

TEST_F(TreeTest, GetTest) {
    NetworkTable::Tree tree;

    // Test setting an integer
    NetworkTable::Value windspeed;
    windspeed.set_type(NetworkTable::Value::INT);
    windspeed.set_int_data(5);

    tree.SetNode("/wind/speed", windspeed);
    EXPECT_EQ(tree.GetNode("wind/speed").value().int_data(), \
              windspeed.int_data());

    // Try getting the trees root, and make sure it has the correct node(s)
    NetworkTable::Node root = tree.GetNode("/");
    EXPECT_EQ(root.children().at("wind").children().at("speed").value().int_data(), \
             windspeed.int_data());

    // Try getting a child of the root and make sure it has the correct node
    NetworkTable::Node child_of_root  = tree.GetNode("/wind");
    EXPECT_EQ(child_of_root.children().at("speed").value().int_data(), \
             windspeed.int_data());
}
