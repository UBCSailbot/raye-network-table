// Copyright 2017 UBC Sailbot

#include "ExampleTest.h"
ExampleTest::ExampleTest() {}

TEST_F(ExampleTest, AddTest) {
  example.add(2, 2);
  EXPECT_EQ(4, example.add(2, 2));
}
