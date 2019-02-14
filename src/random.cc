/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: random.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-03 10:38:36
  * @last_modified_date: 2019-02-03 11:31:24
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/random.hh>

//CODE
void setRandomSeed(int seed)
{
  srand(seed);
}

int randomInt(int min, int max)
{
  int d = max - min + 1;
  return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}
