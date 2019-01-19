#ifndef __LOGGER_ADVANCED_HH__
#define __LOGGER_ADVANCED_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: logger_advanced.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-08 14:16:14
  * @last_modified_date: 2019-01-19 12:31:16
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <glog/logging.h>
#include <fstream>

// Declaration
void SignalHandle(const char* data, int size);

#ifdef ENABLE_GLOG
  #define LOG_COLOR_ON FLAGS_colorlogtostderr = true;
  #define LOG_STDERR_ALSO FLAGS_alsologtostderr = true;
  #define LOG_INIT(name) google::InitGoogleLogging(name);
  #define LOG_SET_DESTINATION(type, path) google::SetLogDestination(type, path);
  #define LOG_INSTALL_FAILURE_SIGNAL_HANDLER google::InstallFailureSignalHandler();
  #define LOG_INSTALL_FAILURE_WRITER google::InstallFailureWriter(&SignalHandle);
  #define LOG_SET_INFO_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_INFO, path)
  #define LOG_SET_WARNING_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_WARNING, path)
  #define LOG_SET_ERROR_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_ERROR, path)
  #define LOG_SET_FATAL_DESTINATION(path) LOG_SET_DESTINATION(google::GLOG_FATAL, path)
  #define LOG_INIT_CONVENIENT(name) LOG_COLOR_ON\
    LOG_STDERR_ALSO\
    LOG_INIT(name)
  #define LOG_INIT_DEFAULT LOG_COLOR_ON\
    LOG_STDERR_ALSO\
    LOG_SET_ERROR_DESTINATION("./log/log_error_")\
    LOG_SET_FATAL_DESTINATION("./log/log_fatal_")\
    LOG_INIT(argv[0])
  #define LOG_SHUTDOWN google::ShutdownGoogleLogging();
  #define AK_DLOG(type) DLOG(type)
  #define AK_LOG(type) LOG(type)
#else
  #define LOG_INIT_CONVENIENT(name)
  #define LOG_INIT_DEFAULT
  #define LOG_SET_ERROR_DESTINATION(path)
  #define LOG_SET_FATAL_DESTINATION(path)
  #define LOG_SHUTDOWN
  #define AK_DLOG(type)
  #define AK_LOG(type)
  #define LOG_INIT_CONVENIENT(name) LOG_COLOR_ON LOG_STDERR_ALSO LOG_INIT(name) LOG_SET_ERROR_DESTINATION("./log/log_error_")
  #define LOG_INIT_CONVENIENT
  #define LOG_INSTALL_FAILURE_SIGNAL_HANDLER
  #define LOG_INSTALL_FAILURE_WRITER
#endif
#endif // __LOGGER_ADVANCED_HH__
