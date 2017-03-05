// Copyright 2017 UBC Sailbot

#ifndef EXAMPLE_EXAMPLETEST_H_
#define EXAMPLE_EXAMPLETEST_H_

#include <example/Example.h>
#include <gtest/gtest.h>

class ExampleTest : public ::testing::Test {
 protected:
  ExampleTest();
  Example example;
};

#endif  // EXAMPLE_EXAMPLETEST_H_
