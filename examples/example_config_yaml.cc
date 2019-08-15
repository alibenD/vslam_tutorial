/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_config_yaml.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-14 11:37:00
  * @last_modified_date: 2019-01-14 14:07:36
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/config_yaml.hh>
#include <visual_slam/logger_advanced.hh>
#include <iostream>

//CODE
int main(int argc, char** argv)
{
  LOG_INIT_DEFAULT;
  AK_LOG(INFO) << "Hello world!";
  auto config = std::make_shared<ak::ConfigYAML>("./config/default.yaml");
  //ak::ConfigYAML::ptr_config_ = config->shared_from_this();
  config->useSharedThis();
  std::cout << "Dynamic count: " << config.use_count() << std::endl;
  std::cout << "Static count: " << ak::ConfigYAML::ptr_config_.use_count() << std::endl;
  LOG_SHUTDOWN;
  return 0;
}
