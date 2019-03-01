/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_timer.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-28 09:06:05
  * @last_modified_date: 2019-02-28 09:24:51
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/time.hh>
#include <visual_slam/logger_advanced.hh>
#include <unistd.h>

//CODE
int main()
{
  LOG_INIT_CONVENIENT("Timer");
  TIMER_START("Sleep 1s");
  usleep(1000000);
  TIMER_END();
}
