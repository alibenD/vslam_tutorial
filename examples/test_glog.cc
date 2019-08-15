/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_glog.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-08 14:54:09
  * @last_modified_date: 2019-01-14 12:47:15
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/logger.hh>
//#include <visual_slam/logger_advanced.hh>
#include <visual_slam/call_glog.hh>

//CODE
int main(int argc, char** argv)
{
  LOG_INIT_CONVENIENT(argv[0]);
  //LOG_INIT_DEFAULT;
  //LOG_COLOR_ON;
  //LOG_STDERR_ALSO;
  //LOG_INIT(argv[0]);
  //LOG_SET_INFO_DESTINATION("./log/log_info_");
  //LOG_SET_WARNING_DESTINATION("./log/log_warning_");
  LOG_SET_ERROR_DESTINATION("./log/log_error_");
  DLOG(INFO) << "HELLO INFO";
  DLOG(WARNING) << "HELLO WARNING";
  logger_call();
  LOG_SHUTDOWN;
  return 0;
}
