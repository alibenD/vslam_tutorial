/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-29 13:33:28
  * @last_modified_date: 2019-02-26 16:53:17
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
  cv::Mat img_last;
  cv::Mat key_img_previous;
  cv::Mat img_current;
  ak::Frame::K = (cv::Mat_<float>(3,3) << 718.85602, 0, 607.1928,
                                           0, 718.85602, 185.2157,
                                           0,       0,        1);
  ak::Frame::enable_show = true;
  //std::vector<cv::Mat> image_set;
  for(auto img_name : img_lists)
  {
    auto img_path = fold_path + "image_0/" + img_name;
    //AK_LOG(WARNING) << "img_path: " << img_path;
    //auto img = cv::imread(img_path);
    img_last = img_current;
    img_current = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    //img_lists.push_back(img_current);
    cv::Mat img_with_keypoints;
    auto pFrame = Frame::CreateFrame(img_current, img_with_keypoints);
    if(pFrame == nullptr)
    {
      continue;
    }
    //Frame::createFrame(img);
    //AK_LOG(INFO) << "Path: " << img_path;
    //Frame::ShowMatches(img_last, img_current, name_window);
    //auto idx_init = Frame::ptr_initialized_frame->id_();
    //auto idx_current = Frame::ptr_current_frame->id_();
    //Frame::ShowMatches(image_set[idx_init], image_set[idx_current], name_window);
    //Frame::ShowMatches(Frame::ptr_last_frame, Frame::ptr_current_frame, name_window);
    //Frame::ShowMatches(img_current, name_window);
  }
  return 0;
}
