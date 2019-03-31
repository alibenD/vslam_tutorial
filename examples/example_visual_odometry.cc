/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-02 13:14:15
  * @last_modified_date: 2019-03-21 17:49:39
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/visual_odometry.hh>
#include <visual_slam/logger_advanced.hh>
#include <visual_slam/utils/io.hh>
#include <opencv2/opencv.hpp>

//CODE
using namespace ak;
int main(int argc, char** argv)
{
  LOG_INIT_DEFAULT;
  std::string fold_path("/Users/aliben/project/data/slam_data/dataset/sequences/00/");
  std::string vocab("../data/ORBvoc.txt");
  std::vector<std::string> img_lists;
  auto num_img = ReadFileList(fold_path+"img_list.txt", img_lists);
  AK_LOG(INFO) << "Count: " << num_img << "\tFrom: " << fold_path << std::endl;
  std::string name_window = "Image";
  cv::namedWindow(name_window, cv::WINDOW_NORMAL);

  VisualOdometry::Ptr ptr_vo = std::make_shared<VisualOdometry>(vocab);
  for(auto img_name : img_lists)
  {
    auto img_path = fold_path + "image_0/" + img_name;
    //AK_LOG(WARNING) << "img_path: " << img_path;
    //auto img = cv::imread(img_path);
    cv::Mat img_current = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    ptr_vo->newFrame(img_current);
  }
  return 0;
}
