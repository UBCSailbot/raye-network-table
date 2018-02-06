// Copyright 2017 UBC Sailbot

#ifndef TREETEST_H_
#define TREETEST_H_

#include <gtest/gtest.h>

class TreeTest : public ::testing::Test {
 protected:
    void GetSetTest();

    void WriteLoadTest();
};

#endif  // TREETEST_H_
