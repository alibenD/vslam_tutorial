/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test.cpp
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-11 09:29:46
  * @last_modified_date: 2019-01-11 09:32:35
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

//INCLUDE

#include <gtest/gtest.h>
#include <numeric>
#include <vector>
// 测试集为 MyTest，测试案例为 Sum
TEST(MyTest, Sum)
{
    std::vector<int> vec{1, 2, 3, 4, 5};
    int sum = std::accumulate(vec.begin(), vec.end(), 0);
    EXPECT_EQ(sum, 15);
}
int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

