#ifndef __VISUAL_ODOMETRY_HH__
#define __VISUAL_ODOMETRY_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: visual_odometry.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-01 15:20:49
  * @last_modified_date: 2019-03-31 18:37:26
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/frame.hh>
#include <visual_slam/landmark.hh>
#include <visual_slam/optimizer.hh>

// Declaration
namespace ak
{
  //enum VO_STATE
  //{
  //  NO_IMAGE = -2,
  //  NO_INITIALIZATION = -1,
  //  INITIALIZED = 0,
  //  TRACK = 1
  //};
  using ID_t = unsigned long;
  struct INIT_PARAMETER
  {
    size_t max_iterations_ = 200;
    float sigma_ = 1.0;
    float sigma_2_;
  };

  struct ORB_PARAMETER
  {
    int num_features_ = 2000;
    float level_scale_factor_ = 1.2;
    int num_levels_ = 8;
    int init_fast_ = 20;
    int min_fast_ = 7;
  };

  struct VO_PARAMETER
  {
    INIT_PARAMETER init_params_;
    ORB_PARAMETER orb_params_;
    VO_STATE vo_state_;
    bool enable_show_ = true;
    cv::Mat K_;
  };

  class VisualOdometry;
  class VisualOdometry
  {
    public:
      using Ptr = std::shared_ptr<VisualOdometry>;
      VisualOdometry();
      VisualOdometry(const std::string& vocab_path);
      ~VisualOdometry() = default;

    public:
      int newFrame(const cv::Mat& image);
      bool InitializeVO(const Frame::Ptr& ptr_initialized_frame,
                        const Frame::Ptr& ptr_current_frame,
                        cv::Mat& Rcw,
                        cv::Mat& tcw);
      int ShowMatches(const Frame::Ptr& last_frame,
                      const Frame::Ptr& current_frame,
                      const std::string& window_name="Image");

    protected:
      //=========== Initialization Functions==============
      static size_t MatchDescriptor(const cv::Mat& last_descriptors,
                                    const cv::Mat& current_descriptors);
      static size_t MatchDescriptor(const Frame::Ptr& ptr_last_keyframe,
                                    const Frame::Ptr& ptr_current_frame);
      float FindHomography(std::vector<cv::DMatch>& matched_inliers, cv::Mat& H21);
      float FindFundamental(std::vector<cv::DMatch>& matched_inliers, cv::Mat& F21);
      cv::Mat ComputeH21(const std::vector<cv::KeyPoint>& keypoints_init,
                         const std::vector<cv::KeyPoint>& keypoints_cur);
      cv::Mat ComputeF21(const std::vector<cv::KeyPoint>& keypoints_init,
                         const std::vector<cv::KeyPoint>& keypoints_cur);
      float CheckHomography(const cv::Mat& H21,
                            const cv::Mat& H12,
                            std::vector<cv::DMatch>& matched_inliers,
                            float sigma);
      float CheckFundamental(const cv::Mat& F21,
                             std::vector<cv::DMatch>& matched_inliers,
                             float sigma);
      bool ReconstructFromHomo(std::vector<cv::DMatch>& matched_inliers,
                                      cv::Mat& H21,
                                      cv::Mat& K,
                                      cv::Mat& R21,
                                      cv::Mat& t21,
                                      std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                                      float min_parallax,
                                      int min_triangulated);
      bool ReconstructFromFund(std::vector<cv::DMatch>& matched_inliers,
                               cv::Mat& F21,
                               cv::Mat& K,
                               cv::Mat& R21,
                               cv::Mat& t21,
                               std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                               float min_parallax,
                               int min_triangulated);
      void NormalizeKeyPoints(const std::vector<cv::KeyPoint>& kps,
                              std::vector<cv::KeyPoint>& kps_normalized,
                              cv::Mat& Transform);
      int CheckRT(const cv::Mat& R,
                  const cv::Mat& t,
                  const std::vector<cv::KeyPoint>& keypoints_last,
                  const std::vector<cv::KeyPoint>& keypoints_cur,
                  const std::vector<cv::DMatch>& matched_inliers,
                  const cv::Mat& K,
                  std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                  float th2,
                  float& parallax
                  );
      void Triangulate(const cv::KeyPoint& kp_init,
                       const cv::KeyPoint& kp_cur,
                       const cv::Mat& Pose_init,
                       const cv::Mat& Pose_cur,
                       cv::Mat& x3D);
      void DecomposeE(const cv::Mat& E,
                      cv::Mat& R1,
                      cv::Mat& R2,
                      cv::Mat& t);

      void InitMap();
      //=========== END Initialization Functions==============

      bool trackWithLastKeyFrame();

    public:
      /*static*/ std::vector<Frame::Ptr> frames_vector_;
      /*static*/ std::vector<Frame::Ptr> keyframes_vector_;
      /*static*/ std::unordered_map<Frame::ID_t, Frame::Ptr> hash_frames_;
      /*static*/ std::unordered_map<Frame::ID_t, Frame::Ptr> hash_keyframes_;
      std::vector<cv::DMatch> best_matches_;
      std::vector<cv::DMatch> best_matches_inliers_;

    private:
      /*static*/ Frame::Ptr ptr_initialized_frame_;
      /*static*/ Frame::Ptr ptr_last_frame_;
      /*static*/ Frame::Ptr ptr_current_frame_;
      /*static*/ Frame::Ptr ptr_last_keyframe_;
      ///*static*/ VO_STATE vo_state;
      /*static*/ std::vector<std::vector<cv::DMatch>> ransec_matched_points_set_;
      std::shared_ptr<ORBmatcher> ptr_orb_matcher_init_advanced;
      VO_PARAMETER vo_params_;
      std::unordered_map<ID_t, Landmark::Ptr> landmarks_map_;
      std::vector<Landmark::Ptr> landmarks_;
      std::shared_ptr<DBoW3::Vocabulary> ptr_vocal_;
      ///*static*/ Frame::Ptr ptr_reference_frame_;
      Optimizer::Ptr ptr_optimizer_;
  };
}
#endif // __VISUAL_ODOMETRY_HH__
