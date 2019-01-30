/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-01-30 09:50:56
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/frame.hh>

//CODE
namespace ak
{
  cv::Ptr<cv::ORB> Frame::ptr_orb_ = cv::ORB::create(400);
  cv::Ptr<cv::DescriptorMatcher> Frame::matcher  = cv::DescriptorMatcher::create("BruteForce-Hamming");
  int Frame::factory_id = 0;
  Frame::Ptr Frame::ptr_last_frame = nullptr;
  Frame::Ptr Frame::ptr_previous_frame = nullptr;
  std::vector<Frame::Ptr> Frame::frames_vector;
  double Frame::match_ratio_ = 2.0;

  Frame::Frame(ID_t id)
    : id_(id)
  {
    ;
  }

  Frame::Ptr Frame::createFrame(const cv::Mat& image,
                                cv::Mat& image_with_keypoints)
  {
    auto pFrame = std::make_shared<Frame>(Frame::factory_id);
    auto num_keypoints = pFrame->extractKeyPoints(image, image_with_keypoints);
    pFrame->computeDescriptors(image);
    //AK_LOG(INFO) << "Size of kps: " << num_keypoints;
    Frame::ptr_previous_frame = Frame::ptr_last_frame;
    Frame::ptr_last_frame = pFrame;
    Frame::frames_vector.push_back(pFrame);
    if(Frame::factory_id > 0 && Frame::ptr_previous_frame != nullptr)
    {
      matchDescriptor(Frame::ptr_previous_frame->descriptors_,
                      Frame::ptr_last_frame->descriptors_);
    }
    AK_LOG(INFO) << "Size of frames: " << Frame::frames_vector.size();
    Frame::factory_id++;
    return pFrame;
  }

  size_t Frame::extractKeyPoints(const cv::Mat& image,
                                 cv::Mat& img_with_keypoints)
  {
    this->ptr_orb_->detect(image, this->keypoints_);
    this->drawKeyPoints(image, img_with_keypoints);
    return this->keypoints_.size();
  }

  int Frame::computeDescriptors(const cv::Mat& image)
  {
    this->ptr_orb_->compute(image,
                            this->keypoints_,
                            this->descriptors_);
    return 0;
  }

  int Frame::drawKeyPoints(const cv::Mat& img_origin,
                           cv::Mat& img_with_keypoints)
  {
    cv::drawKeypoints(img_origin,
                      this->keypoints_,
                      img_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    return 0;
  }

  int Frame::matchDescriptor(const cv::Mat& previous_descriptors,
                             const cv::Mat& last_descriptors)
  {
    std::vector<cv::DMatch> matches;
    matcher->match(previous_descriptors, last_descriptors, matches);
    float min_distance = std::min_element(matches.begin(),
                                          matches.end(),
                                          [](const cv::DMatch& m1, const cv::DMatch& m2)
    {
      return m1.distance < m2.distance;
    })->distance;
    Frame::ptr_previous_frame->best_matches_.clear();
    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_distance*Frame::match_ratio_, 30.0))
      {
        Frame::ptr_previous_frame->best_matches_.push_back(m);
      }
    }
    return 0;
  }

  int Frame::showMatches(const cv::Mat& previous_image,
                         const cv::Mat& last_image,
                         const std::string& window_name)
  {
    if(Frame::ptr_previous_frame != nullptr && Frame::ptr_last_frame != nullptr)
    {
      cv::Mat matched_image;
      cv::drawMatches(previous_image,
                      Frame::ptr_previous_frame->keypoints_,
                      last_image,
                      Frame::ptr_last_frame->keypoints_,
                      Frame::ptr_previous_frame->best_matches_,
                      matched_image);
      cv::imshow(window_name, matched_image);
      cv::waitKey(30);
    }
    return 0;
  }
}
