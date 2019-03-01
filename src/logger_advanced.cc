/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: logger_advanced.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-19 12:30:27
  * @last_modified_date: 2019-01-19 12:31:07
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/logger_advanced.hh>

//CODE
void SignalHandle(const char* data, int size)
{
    std::ofstream fs("glog_dump.log",std::ios::app);
    std::string str = std::string(data,size);
    fs<<str;
    fs.close();
    LOG(ERROR)<<str;
}
