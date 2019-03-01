#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-02-19 21:31:28
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/logger_advanced.hh>
#include <visual_slam/utils/random.hh>
#include <memory>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <visual_slam/ORBextractor.hh>
#include "gtest/gtest_prod.h"


// Declaration
namespace ak
{
  class ORBmatcher;
  static cv::Mat EMPTY_MAT = cv::Mat();
  static cv::Mat last_keyframe_image = cv::Mat();
  const unsigned int IMAGE_COLS = 64;
  const unsigned int IMAGE_ROWS = 48;
  enum VO_STATE
  {
    NO_IMAGE = -2,
    NO_INITIALIZATION = -1,
    INITIALIZED = 0,
    TRACK = 1
  };
  
  class Frame : public std::enable_shared_from_this<Frame>
  {
    public:
      using Ptr = std::shared_ptr<Frame>;
      using ID_t = unsigned long;
      friend class FramePair;
      friend class ORBmatcher;
      FRIEND_TEST(FrameTest, getGridPosition);
      FRIEND_TEST(FrameTest, ComputeH21);
      FRIEND_TEST(FrameTest, NormalizeKeyPoints);
//      friend class FrameTest_getGridPosition_Test;
      Frame() = default;
      Frame(ID_t id);
      ~Frame() = default;

      struct GridProperty
      {
        public:
          void setup(const cv::Mat& image)
          {
            min_x = 0.0;
            min_y = 0.0;
            max_x = image.cols;
            max_y = image.rows;
            grid_width = (max_x - min_x) / static_cast<float>(IMAGE_COLS);
            grid_height = (max_y - min_y) / static_cast<float>(IMAGE_ROWS);
            grid_width_inv = 1.0 / grid_width;
            grid_height_inv = 1.0 / grid_height;
          };

          float grid_width;
          float grid_height;
          float grid_width_inv;
          float grid_height_inv;
          float min_x;
          float min_y;
          float max_x;
          float max_y;
      };

    public:
      //friend class ::ORBmatcher;
      //ID_t id_()
      //{
      //  return id_;
      //};
      inline const std::vector<cv::KeyPoint>& getKeyPoints()
      {
        return keypoints_;
      };
      inline const cv::Mat& getDescriptors()
      {
        return descriptors_;
      };
      cv::Mat image_;

    public:
      static Frame::Ptr CreateFrame(const cv::Mat& image,
                                    cv::Mat& image_with_keypoints = EMPTY_MAT);
      static int ShowMatches(const cv::Mat& last_image,
                             const cv::Mat& current_image,
                             const std::string& window_name="Image");
      static int ShowMatches(const Frame::Ptr& last_frame,
                             const Frame::Ptr& current_frame,
                             const std::string& window_name="Image");
      static int ShowMatches(const cv::Mat& current_image,
                             const std::string& window_name="Image");
      std::vector<size_t> getCandidateKeypoints(const float& x,
                                                const float& y,
                                                const float& radium,
                                                const int& min_pyramid_level,
                                                const int& max_pyramid_level);
      static int factory_id;
      static Frame::Ptr ptr_initialized_frame;
      static Frame::Ptr ptr_last_frame;
      static Frame::Ptr ptr_current_frame;
      static Frame::Ptr ptr_last_keyframe;
      static std::vector<Frame::Ptr> frames_vector;
      static std::vector<Frame::Ptr> keyframes_vector;
      static std::unordered_map<ID_t, Frame::Ptr> hash_frames;
      static std::unordered_map<ID_t, Frame::Ptr> hash_keyframes;
      static std::vector<std::vector<cv::DMatch>> MATCHED_POINTS_SET;
      //static std::vector<cv::Point3f> init_landmarks;
      static std::vector<std::pair<size_t, cv::Point3f>> init_landmarks;
      static float sigma;
      static cv::Mat K; // Temp. definition, there must have a good way to do it
      static float match_ratio_;
      static VO_STATE vo_state;
      static bool enable_show;

    public:
      //static cv::Ptr<cv::ORB> ptr_orb_;
      static ORBextractor::Ptr ptr_orb_extractor_advanced;
      static ORBextractor::Ptr ptr_orb_extractor_init_advanced;
      //static std::shared_ptr<ORBmatcher> ptr_orb_matcher_init_advanced;
      ORBmatcher* ptr_orb_matcher_init_advanced;
      static cv::Ptr<cv::DescriptorMatcher> matcher;

    protected:
      static bool InitializeVO(const Frame::Ptr& ptr_initialized_frame,
                               const Frame::Ptr& ptr_current_frame,
                               cv::Mat& Rcw,
                               cv::Mat& tcw);
      size_t extractKeyPoints(const cv::Mat& image,
                              cv::Mat& image_with_keypoints);
      int computeDescriptors(const cv::Mat& image);
      int drawKeyPoints(const cv::Mat& img_origin,
                        cv::Mat& img_with_keypoints);
      void assignFeaturePointToGrid();
      bool getGridPosition(const cv::KeyPoint& kp,
                           size_t& pos_x,
                           size_t& pos_y);
      static size_t MatchDescriptor(const cv::Mat& last_descriptors,
                                    const cv::Mat& current_descriptors);
      static size_t MatchDescriptor(const Frame::Ptr& ptr_last_keyframe,
                                    const Frame::Ptr& ptr_current_frame);
      static float FindHomography(std::vector<cv::DMatch>& matched_inliers, cv::Mat& H21);
      static float FindFundamental(std::vector<cv::DMatch>& matched_inliers, cv::Mat& F21);
      static cv::Mat ComputeH21(const std::vector<cv::KeyPoint>& keypoints_init,
                                const std::vector<cv::KeyPoint>& keypoints_cur);
      static cv::Mat ComputeF21(const std::vector<cv::KeyPoint>& keypoints_init,
                                const std::vector<cv::KeyPoint>& keypoints_cur);
      static float CheckHomography(const cv::Mat& H21,
                                    const cv::Mat& H12,
                                    std::vector<cv::DMatch>& matched_inliers,
                                    float sigma);
      static float CheckFundamental(const cv::Mat& F21,
                                     std::vector<cv::DMatch>& matched_inliers,
                                     float sigma);
      static bool ReconstructFromHomo(std::vector<cv::DMatch>& matched_inliers,
                                      cv::Mat& H21,
                                      cv::Mat& K,
                                      cv::Mat& R21,
                                      cv::Mat& t21,
                                      std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                                      float min_parallax,
                                      int min_triangulated);
      static bool ReconstructFromFund(std::vector<cv::DMatch>& matched_inliers,
                                      cv::Mat& F21,
                                      cv::Mat& K,
                                      cv::Mat& R21,
                                      cv::Mat& t21,
                                      std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                                      float min_parallax,
                                      int min_triangulated);
      static void NormalizeKeyPoints(const std::vector<cv::KeyPoint>& kps,
                                     std::vector<cv::KeyPoint>& kps_normalized,
                                     cv::Mat& Transform);
      static int CheckRT(const cv::Mat& R,
                         const cv::Mat& t,
                         const std::vector<cv::KeyPoint>& keypoints_last,
                         const std::vector<cv::KeyPoint>& keypoints_cur,
                         const std::vector<cv::DMatch>& matched_inliers,
                         const cv::Mat& K,
                         std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                         float th2,
                         float& parallax
                         );
      static void Triangulate(const cv::KeyPoint& kp_init,
                              const cv::KeyPoint& kp_cur,
                              const cv::Mat& Pose_init,
                              const cv::Mat& Pose_cur,
                              cv::Mat& x3D);
      static void DecomposeE(const cv::Mat& E,
                             cv::Mat& R1,
                             cv::Mat& R2,
                             cv::Mat& t);

    private:
      ID_t id_;
      cv::Mat descriptors_;
      std::vector<cv::KeyPoint> keypoints_;
      std::vector<cv::DMatch> best_matches_;
      std::vector<cv::DMatch> best_matches_inliers_;
      Frame::Ptr ptr_reference_frame_;
      std::vector<size_t> keypoints_grid_[IMAGE_COLS][IMAGE_ROWS];
      GridProperty grid_property_;
      // Pose SE3
  };
}
#endif // __FRAME_HH__
