#ifndef __LOGGER_HH__
#define __LOGGER_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: logger.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-08 14:16:14
  * @last_modified_date: 2019-01-14 10:48:40
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <glog/logging.h>

// Declaration
#define LOG_COLOR_ON FLAGS_colorlogtostderr = true;
#define LOG_STDERR_ALSO FLAGS_alsologtostderr = true;
#define LOG_INIT(name) google::InitGoogleLogging(name);
#define LOG_SET_DESTINATION(type, path) google::SetLogDestination(type, path);
#define LOG_SET_INFO_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_INFO, path)
#define LOG_SET_WARNING_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_WARNING, path)
#define LOG_SET_ERROR_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_ERROR, path)
#define LOG_SET_FATAL_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_FATAL, path)
#define LOG_INIT_CONVENIENT(name) LOG_COLOR_ON LOG_STDERR_ALSO LOG_INIT(name)

#define LOG_SHUTDOWN google::ShutdownGoogleLogging();
#endif // __LOGGER_HH__
