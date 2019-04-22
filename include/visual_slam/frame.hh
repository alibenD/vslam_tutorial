#ifndef __FRAME_HH__
#define __FRAME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-04-22 15:54:39
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/logger_advanced.hh>
#include <visual_slam/utils/random.hh>
#include <visual_slam/utils/type.hh>
#include <memory>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <DBoW3/DBoW3.h>
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
  
  class Frame : public std::enable_shared_from_this<Frame>
  {
    public:
      using Ptr = std::shared_ptr<Frame>;
      friend class FramePair;
      friend class ORBmatcher;
      friend class VisualOdometry;
      friend class Map;
      //FRIEND_TEST(FrameTest, getGridPosition);
      //FRIEND_TEST(FrameTest, ComputeH21);
      //FRIEND_TEST(FrameTest, NormalizeKeyPoints);
      Frame() = delete;
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
      ID_t getID();
      const std::vector<cv::KeyPoint>& getKeyPoints();
      const std::unordered_map<size_t, Landmark::Ptr>& getLandmarks();
      const DBoW3::FeatureVector& getFeatureVector();
      const cv::Mat& getDescriptors();
      const cv::Mat& getPose();
      const cv::Mat& getOrigin();
      bool isGood();

      // Set method
      void setTF(const cv::Mat& T21);
      void setReferenceFrame(const Frame::Ptr& ptr_ref_frame);
      static Frame::Ptr CreateFrame(const cv::Mat& image,
                                    const ORBextractor::Ptr& ptr_orb_extracotr,
                                    const std::shared_ptr<DBoW3::Vocabulary>& ptr_vocal);
      std::vector<size_t> getCandidateKeypoints(const float& x,
                                                const float& y,
                                                const float& radium,
                                                const int& min_pyramid_level,
                                                const int& max_pyramid_level);

    protected:
      void assignFeaturePointToGrid();
      void computeBOW();
      void insertLandmark(Landmark::Ptr& ptr_landmark, size_t kp_index);
      void addCovision(const Frame::Ptr& ptr_covision, unsigned int weight);
      void updateCovision();
      size_t extractKeyPoints(const ORBextractor::Ptr& ptr_orb_extracotr);
      int computeDescriptors(const cv::Mat& image);
      int drawKeyPoints(const cv::Mat& img_origin, cv::Mat& img_with_keypoints);
      float computeMedianDepth(int section);
      bool getGridPosition(const cv::KeyPoint& kp, size_t& pos_x, size_t& pos_y);

    private:
      GridProperty grid_property_;
      ID_t id_;
      cv::Mat image_;
      cv::Mat descriptors_;
      std::vector<cv::KeyPoint> keypoints_;
      std::vector<cv::DMatch> best_matches_;
      std::vector<cv::DMatch> best_matches_inliers_;

      //static ORBextractor::Ptr ptr_orb_extractor_advanced;
      //static ORBextractor::Ptr ptr_orb_extractor_init_advanced;

      std::unordered_map<Frame::Ptr, unsigned int> covision_sets_;
      std::vector<Frame::Ptr> covision_frame_ordered_;
      std::vector<unsigned int> covision_weight_ordered_;

      // Loop Closure
      std::shared_ptr<DBoW3::Vocabulary> ptr_vocal_{nullptr};
      std::vector<cv::Mat> descriptor_vectors_;
      DBoW3::FeatureVector feature_vector_;
      DBoW3::BowVector bow_vector_;

      std::vector<size_t> keypoints_grid_[IMAGE_COLS][IMAGE_ROWS];
      // Pose opencv
      cv::Mat transform_camera_at_world_;  // TF21
      cv::Mat T21_estimated_;
      cv::Mat T21_local_BA_;
      cv::Mat T21_global_BA_;
      cv::Mat T21_before_global_BA_;
      cv::Mat camera_origin_at_world_;
      cv::Mat transform_optimized_;
      bool init_covision_{false};
      bool is_good_{true};
      unsigned int num_ba_global_{0};
      Frame::Ptr ptr_reference_frame_{nullptr};

      static std::vector<std::pair<size_t, cv::Point3f>> raw_init_landmarks;

      // keypoints_index, ptr_landmark
      std::unordered_map<size_t, Landmark::Ptr> landmarks_;
      // landmark_index, keypoint_index
      std::unordered_map<ID_t, size_t> landmarks_index_query_;
      static int factory_id;
//      bool is_drop_{false};
  };
}
#endif // __FRAME_HH__
