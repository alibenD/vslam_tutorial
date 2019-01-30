/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-29 13:33:28
  * @last_modified_date: 2019-01-30 09:54:59
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/io.hh>
#include <visual_slam/frame.hh>
#include <opencv2/opencv.hpp>

//CODE
using namespace ak;
int main(int argc, char** argv)
{
  LOG_INIT_DEFAULT;
  std::string fold_path("/Users/aliben/project/data/slam_data/dataset/sequences/00/");
  std::vector<std::string> img_lists;
  auto num_img = ReadFileList(fold_path+"img_list.txt", img_lists);
  AK_LOG(INFO) << "Count: " << num_img << "\tFrom: " << fold_path << std::endl;

  std::string name_window = "Image";
  cv::namedWindow(name_window, cv::WINDOW_NORMAL);
  cv::Mat img_previous;
  cv::Mat img_last;
  for(auto img_name : img_lists)
  {
    auto img_path = fold_path + "image_0/" + img_name;
    //AK_LOG(WARNING) << "img_path: " << img_path;
    //auto img = cv::imread(img_path);
    img_previous = img_last;
    img_last = cv::imread(img_path);
    cv::Mat img_with_keypoints;
    Frame::createFrame(img_last, img_with_keypoints);
    //Frame::createFrame(img);
    //AK_LOG(INFO) << "Path: " << img_path;
    //cv::imshow(name_window, img_with_keypoints);
    //cv::waitKey(30);
    Frame::showMatches(img_previous, img_last, name_window);
  }
  return 0;
}
