/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_readfilelist.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-20 21:42:28
  * @last_modified_date: 2019-02-27 10:05:28
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/io.hh>

//CODE
using namespace ak;
int main(int argc, char** argv)
{
  LOG_INIT_DEFAULT;
  std::string fold_path("/Users/aliben/project/data/slam_data/dataset/sequences/00/img_list.txt");
  std::vector<std::string> img_lists;
  auto num_img = ReadFileList(fold_path, img_lists);
  AK_LOG(INFO) << "Count: " << num_img << "\tFrom: " << fold_path << std::endl;
  return 0;
}
