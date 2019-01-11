/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_gtest.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-08 18:55:44
  * @last_modified_date: 2019-01-11 14:18:17
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE

//CODE
#include <iostream>
#include <gtest/gtest.h>

int add(int a, int b) {
    return a + b;
}

int sub(int a, int b) {
    return a - b;
}

// case1
TEST(testdjdjdjdj, c1) {
    EXPECT_EQ(3, add(1, 2));
    //EXPECT_EQ(12, add(2, 6));
}

// case2
TEST(test, c2) {
    EXPECT_EQ(-1, sub(1, 2));
}

GTEST_API_ int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
