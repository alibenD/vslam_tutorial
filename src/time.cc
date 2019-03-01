/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: time.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-27 18:50:29
  * @last_modified_date: 2019-02-28 09:12:52
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/time.hh>

//CODE
namespace ak
{
  namespace utils
  {
    std::chrono::time_point<std::chrono::system_clock> Timer::time_start_;
    std::chrono::time_point<std::chrono::system_clock> Timer::time_end_;
    std::chrono::duration<double> Timer::time_duration_;
    std::string Timer::title_;
  }
}
