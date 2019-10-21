// Copyright 2017 UBC Sailbot

#ifndef HELPTEST_H_
#define HELPTEST_H_

#include <gtest/gtest.h>

class HelpTest : public ::testing::Test {
 protected:
    void GetSetTest();

    void WriteLoadTest();
};

#endif  // HELPTEST_H_
