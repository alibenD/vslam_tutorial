/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_config_read.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-03 09:07:22
  * @last_modified_date: 2019-01-03 09:36:16
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <opencv2/core.hpp>
#include <iostream>
#include <string>

//CODE
int main(int argc, char** argv)
{
  std::string filename(argv[1]);
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    std::cout << "Failed to open " << filename << std::endl;
    return -1;
  }
  auto node = fs["dataset_dir"];
  std::cout << "dir: " << std::string(node) << std::endl;
  return 0;
}
