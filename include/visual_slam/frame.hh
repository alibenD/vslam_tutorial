#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-01-30 09:50:10
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/logger_advanced.hh>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// Declaration
namespace ak
{
  static cv::Mat EMPTY_MAT = cv::Mat();
  class Frame : public std::enable_shared_from_this<Frame>
  {
    public:
      using Ptr = std::shared_ptr<Frame>;
      using ID_t = unsigned long;
      friend class FramePair;
      Frame() = default;
      Frame(ID_t id);
      ~Frame() = default;

    public:
      static Frame::Ptr createFrame(const cv::Mat& image,
                                    cv::Mat& image_with_keypoints = EMPTY_MAT);
      static int showMatches(const cv::Mat& previous_image,
                             const cv::Mat& last_image,
                             const std::string& window_name="Image");
      static int factory_id;
      static Frame::Ptr ptr_previous_frame;
      static Frame::Ptr ptr_last_frame;
      static std::vector<Frame::Ptr> frames_vector;
      static double match_ratio_;


    public:
      static cv::Ptr<cv::ORB> ptr_orb_;
      static cv::Ptr<cv::DescriptorMatcher> matcher;

    protected:
      size_t extractKeyPoints(const cv::Mat& image,
                              cv::Mat& image_with_keypoints);
      int computeDescriptors(const cv::Mat& image);
      int drawKeyPoints(const cv::Mat& img_origin,
                        cv::Mat& img_with_keypoints);
      static int matchDescriptor(const cv::Mat& previous_descriptors,
                                 const cv::Mat& last_descriptors);

    private:
      ID_t id_;
      cv::Mat descriptors_;
      std::vector<cv::KeyPoint> keypoints_;
      std::vector<cv::DMatch> best_matches_;
      // Pose SE3
  };
}
#endif // __FRAME_HH__
