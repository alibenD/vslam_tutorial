/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: config_yaml.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-14 09:39:12
  * @last_modified_date: 2019-01-28 18:57:57
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/config_yaml.hh>
#include <visual_slam/logger_advanced.hh>

//CODE
namespace ak
{
  ConfigYAML::Ptr ConfigYAML::ptr_config_= nullptr;

  ConfigYAML::~ConfigYAML()
  {
    release();
    ptr_config_ = nullptr;
    delete ptr_yaml_handler_;
    ptr_yaml_handler_ = nullptr;
  }

  ConfigYAML::ConfigYAML(const std::string& path)
    //: ptr_yaml_handler_(nullptr)
  {
    //ptr_yaml_handler_ = std::make_shared<cv::FileStorage>(path.c_str(), READ);
    open(path);
    //ptr_config_ = this;
    //ptr_config_ = shared_from_this();
  }

  int ConfigYAML::open(const std::string& path)
  {
    //ptr_yaml_handler_ = std::make_shared<cv::FileStorage>(path.c_str(), READ);
    ptr_yaml_handler_ = new cv::FileStorage(path.c_str(), cv::FileStorage::READ);
    if(ptr_yaml_handler_->isOpened() == false)
    {
      AK_LOG(FATAL) << "YAML file: " << path << " CANNOT be opened(or DOES NOT exist)";
      ptr_yaml_handler_->release();
      return -1;
    }
      AK_DLOG(INFO) << "YAML file: " << path << " is opened successfully.";
    return 0;
  }

  int ConfigYAML::release()
  {
    if(ptr_yaml_handler_->isOpened() == true)
    {
      ptr_yaml_handler_->release();
      AK_DLOG(INFO) << "YAML file handler is released.";
      return 0;
    }
    else
    {
      AK_LOG(ERROR) << "YAML file handler has NOT been opened.";
      return -1;
    }
  }
}
