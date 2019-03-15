#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-03-03 14:51:29
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
#include <visual_slam/landmark.hh>
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
      friend class VisualOdometry;
      friend class Map;
      //FRIEND_TEST(FrameTest, getGridPosition);
      //FRIEND_TEST(FrameTest, ComputeH21);
      //FRIEND_TEST(FrameTest, NormalizeKeyPoints);
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
      inline ID_t getID()
      {
        return id_;
      }
      inline const std::vector<cv::KeyPoint>& getKeyPoints()
      {
        return keypoints_;
      };
      inline const cv::Mat& getDescriptors()
      {
        return descriptors_;
      };
      inline const cv::Mat& getPose()
      {
        return transform_camera_at_world_;
      }
      inline const cv::Mat& getOrigin()
      {
        return camera_origin_at_world_;
      }
      cv::Mat image_;

    public:
      static Frame::Ptr CreateFrame(const cv::Mat& image,
                                    cv::Mat& image_with_keypoints = EMPTY_MAT);
      std::vector<size_t> getCandidateKeypoints(const float& x,
                                                const float& y,
                                                const float& radium,
                                                const int& min_pyramid_level,
                                                const int& max_pyramid_level);
      static int factory_id;
      static std::vector<std::pair<size_t, cv::Point3f>> init_landmarks;
      // keypoints_index, ptr_landmark
      std::unordered_map<size_t, Landmark::Ptr> landmarks_;
      // landmark_index, keypoint_index
      std::unordered_map<ID_t, size_t> landmarks_index_query_;

    public:
      //static cv::Ptr<cv::ORB> ptr_orb_;
      static ORBextractor::Ptr ptr_orb_extractor_advanced;
      static ORBextractor::Ptr ptr_orb_extractor_init_advanced;

    protected:
      size_t extractKeyPoints(const cv::Mat& image,
                              cv::Mat& image_with_keypoints);
      int computeDescriptors(const cv::Mat& image);
      int drawKeyPoints(const cv::Mat& img_origin,
                        cv::Mat& img_with_keypoints);
      void assignFeaturePointToGrid();
      bool getGridPosition(const cv::KeyPoint& kp,
                           size_t& pos_x,
                           size_t& pos_y);
      void insertLandmark(Landmark::Ptr& ptr_landmark, size_t kp_index);
      inline const std::unordered_map<size_t, Landmark::Ptr>& getLandmark()
      {
        return landmarks_;
      }
      void updateCovision();
      void addCovision(const Frame::Ptr& ptr_covision, unsigned int weight);
      float computeMedianDepth(int section);

    private:
      ID_t id_;
      cv::Mat descriptors_;
      std::vector<cv::KeyPoint> keypoints_;
      std::vector<cv::DMatch> best_matches_;
      std::vector<cv::DMatch> best_matches_inliers_;
      std::unordered_map<Frame::Ptr, unsigned int> covision_sets_;
      std::vector<Frame::Ptr> covision_frame_ordered_;
      std::vector<unsigned int> covision_weight_ordered_;
      std::vector<size_t> keypoints_grid_[IMAGE_COLS][IMAGE_ROWS];
      GridProperty grid_property_;
      // Pose opencv
      cv::Mat transform_camera_at_world_;  // TF21
      cv::Mat camera_origin_at_world_;
      bool init_covision_{false};
      //cv::Mat translation_;
      //cv::Mat rotation_;
      // Pose SE3
  };
}
#endif // __FRAME_HH__
