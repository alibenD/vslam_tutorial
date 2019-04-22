/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: visual_odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-01 15:20:49
  * @last_modified_date: 2019-04-22 18:13:47
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/visual_odometry.hh>
#include <visual_slam/ORBmatcher.hh>
#include <future>
#include <utility>

//CODE
namespace ak {
  /*const*/ int NUM_FEATURES = 2000;
  /*const*/ float LEVEL_SCALE_FACTOR = 1.2;
  /*const*/ int NUM_LEVELS = 8;
  /*const*/ int INIT_FAST = 20;
  /*const*/ int MIN_FAST = 7;

  //=========== Static member Initialization ============
  //Frame::Ptr ptr_initialized_frame_ = nullptr;
  //Frame::Ptr ptr_last_frame_ = nullptr;
  //Frame::Ptr ptr_current_frame_ = nullptr;
  //Frame::Ptr ptr_last_keyframe_ = nullptr;
  //=========== END (Static member Initialization) ============

  VisualOdometry::VisualOdometry()
    : ptr_initialized_frame_(nullptr),
      ptr_last_frame_(nullptr),
      ptr_current_frame_(nullptr),
      ptr_last_keyframe_(nullptr),
      ptr_current_keyframe_(nullptr),
      ptr_orb_extractor_init_(nullptr),
      ptr_orb_extractor_track_(nullptr),
      ptr_orb_matcher_init_(nullptr)
  {
    ptr_orb_extractor_init_ = std::make_shared<ORBextractor>(2*NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);
    ptr_orb_extractor_track_ = std::make_shared<ORBextractor>(NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);

    ptr_orb_matcher_init_ = std::make_shared<ORBmatcher>(0.9, true);
    ptr_optimizer_ = std::make_shared<Optimizer>();
    ptr_map_ = std::make_shared<Map>();
    vo_params_.K_ = (cv::Mat_<float>(3, 3) << 718.85602, 0, 607.1928,
      0, 718.85602, 185.2157,
      0, 0, 1);
    vo_params_.vo_state_ = NO_IMAGE;
    vo_params_.init_params_.sigma_2_ = std::pow(vo_params_.init_params_.sigma_, 2);
  }

  VisualOdometry::VisualOdometry(const std::string &vocab_path)
    : VisualOdometry()
  {
    ptr_vocal_ = std::make_shared<DBoW3::Vocabulary>();
    AK_LOG_INFO << "Loading Vocabulary from txt file";
    ptr_vocal_->load(vocab_path);
    AK_LOG_INFO << "Load finished";
  }


  int VisualOdometry::ShowMatches(const Frame::Ptr &ptr_last_frame,
                                  const Frame::Ptr &ptr_current_frame,
                                  const std::string &window_name) {
    if (ptr_last_frame != nullptr && ptr_current_frame != nullptr) {
      cv::Mat matched_image;
      cv::drawMatches(ptr_last_frame->image_,
                      ptr_last_frame->keypoints_,
                      ptr_current_frame->image_,
                      ptr_current_frame->keypoints_,
                      ptr_last_frame->best_matches_,
                      matched_image);
      cv::imshow(window_name, matched_image);
      cv::waitKey(30);
    }
    return 0;
  }

  int VisualOdometry::ShowMatches(const std::string &window_name) {
    ShowMatches(this->ptr_last_keyframe_, this->ptr_current_frame_, window_name);
    return 0;
  }

  int VisualOdometry::newFrame(const cv::Mat &image) {
    //auto pFrame = Frame::CreateFrame(image, ptr_vocal_);
    //auto num_keypoints = pFrame->getKeyPoints().size();
    if (vo_params_.vo_state_ == NO_IMAGE)
    {
      auto pFrame = Frame::CreateFrame(image, ptr_orb_extractor_init_, ptr_vocal_);
      auto num_keypoints = pFrame->getKeyPoints().size();
      if (num_keypoints >= 100)
      {
        // Add this frame as initialize_frame;
        this->ptr_initialized_frame_ = pFrame;
        //this->ptr_last_frame_ = this->ptr_current_frame_;
        this->ptr_last_frame_ = nullptr;
        this->ptr_current_frame_ = pFrame;
        vo_params_.vo_state_ = NO_INITIALIZATION;
        return 0;
      }
      else
      {
        // Drop this frame and waiting for next init frame
        this->ptr_initialized_frame_ = nullptr;
        this->ptr_last_frame_ = nullptr;
        this->ptr_current_frame_ = pFrame;
        vo_params_.vo_state_ = NO_IMAGE;
        return 0;
      }
    }
    else if (vo_params_.vo_state_ == NO_INITIALIZATION)
    {
      // TODO: (aliben.develop@gmail.com)
      // Accept a frame gap from the last to init the last frame,
      // but the biggest diff id num is 2, this could be better.
      auto pFrame = Frame::CreateFrame(image, ptr_orb_extractor_track_, ptr_vocal_);
      auto num_keypoints = pFrame->getKeyPoints().size();
      if (num_keypoints < 100)
      {
        this->ptr_initialized_frame_ = nullptr;
        this->ptr_last_frame_ = nullptr;
        this->ptr_current_frame_ = pFrame;
        vo_params_.vo_state_ = NO_IMAGE;
        return 0;
      }
      //      this->ptr_last_frame_ = this->ptr_current_frame_;
      //      this->ptr_current_frame_ = pFrame;
      auto num_matches = this->ptr_orb_matcher_init_->SearchForInitialization(ptr_initialized_frame_, pFrame, 100);
      //auto all_matches = ptr_initialized_frame_->best_matches_;
      AK_DLOG_ERROR << "Matched: " << num_matches;
      /* NOTE: (aliben.develop@gmail.com)
      // Since masOS does NOT support NSWindow start in a thread which is not the main one,
      // So the solution is that VisualOdometry run in a new thread, Pangolin and cv::imshow run in the main thread.
      */ 
      //if(vo_params_.enable_show_ == true)
      //{
      //  ShowMatches(ptr_initialized_frame_, pFrame);
      //}
      {
        std::lock_guard<std::mutex> guard(show_mutex_);
        vo_params_.enable_show_ = true;
      }
      if (num_matches < 100)
      {
        AK_DLOG_ERROR << "Matches ≤ 100, Restarting Init.";
        this->ptr_initialized_frame_ = nullptr;
        this->ptr_last_frame_ = nullptr;
        this->ptr_current_frame_ = pFrame;
        vo_params_.vo_state_ = NO_IMAGE;
        //vo_params_.vo_state_ = NO_INITIALIZATION;
        //return nullptr;
        return 0;
      }
      else
      {
        AK_DLOG_ERROR << "Init";
        exit(0);
        this->ptr_last_frame_ = this->ptr_initialized_frame_;
        this->ptr_current_frame_ = pFrame;
        cv::Mat R21, t21;
        auto isInit = InitializeVO(ptr_initialized_frame_,
                                   ptr_current_frame_,
                                   R21,
                                   t21);
        if (isInit == true)
        {
          // If Initialization successful, then calculate the transformation
          vo_params_.vo_state_ = INITIALIZED;
          cv::Mat transform_current_at_initialized = cv::Mat::eye(4, 4, CV_32F);
          R21.copyTo(transform_current_at_initialized.rowRange(0, 3).colRange(0, 3));
          t21.copyTo(transform_current_at_initialized.rowRange(0, 3).col(3));
          auto camera_origin_at_world = -R21.inv() * t21;
          // Set pose for current frame
          ptr_initialized_frame_->transform_camera_at_world_ = cv::Mat::eye(4, 4, CV_32F);
          ptr_initialized_frame_->camera_origin_at_world_ = cv::Mat::zeros(3, 1, CV_32F);
          //ptr_initialized_frame_->translation_ = ptr_initialized_frame_->transform_camera_at_world_.rowRange(0,3).col(3);
          //ptr_initialized_frame_->rotation_ = ptr_initialized_frame_->transform_camera_at_world_.rowRange(0,3).colRange(0,3);
          ptr_current_frame_->transform_camera_at_world_ = transform_current_at_initialized;
          ptr_current_frame_->camera_origin_at_world_ = camera_origin_at_world;
          AK_DLOG_INFO << "raw_init_landmarks: " << Frame::raw_init_landmarks.size();


          ptr_current_frame_->setReferenceFrame(ptr_initialized_frame_);
          //frames_vector_.push_back(ptr_last_frame_);
          //frames_vector_.push_back(ptr_current_frame_);
          //keyframes_vector_.push_back(ptr_last_frame_);
          //keyframes_vector_.push_back(ptr_current_frame_);
          //hash_frames_.insert(std::make_pair(ptr_initialized_frame_->getID(), ptr_initialized_frame_));
          //hash_keyframes_.insert(std::make_pair(ptr_initialized_frame_->getID(), ptr_initialized_frame_));
          //ptr_map_->addFrame(ptr_last_frame_);
          //ptr_map_->addFrame(ptr_current_frame_);
          ptr_map_->addKeyFrame(ptr_initialized_frame_);
          ptr_map_->addKeyFrame(ptr_current_frame_);

          AK_LOG_INFO << "Before Tcw:\n" << ptr_current_frame_->transform_camera_at_world_;
          InitMap();
          AK_LOG_INFO << "After Optimization Tcw:\n" << ptr_current_frame_->transform_camera_at_world_;

          //AK_DLOG_INFO << "MapPoint Size: " << landmarks_map_.size();
          AK_DLOG_INFO << "MapPoint Size: " << ptr_map_->getLandmarks().size();
          ptr_last_keyframe_ = ptr_initialized_frame_;
          ptr_current_keyframe_ = ptr_current_frame_;
          //          ptr_last_frame_ = ptr_current_frame_;
          AK_LOG_INFO << "Initialization Done!!";
          //exit(0);
        }
        else
        {
          vo_params_.vo_state_ = NO_IMAGE;
        }
      }
    }
    else if (vo_params_.vo_state_ == INITIALIZED)
    {
      // Initialization Done!
      // Go on track
      auto pFrame = Frame::CreateFrame(image, ptr_orb_extractor_track_, ptr_vocal_);
      auto num_keypoints = pFrame->getKeyPoints().size();
      if (ptr_current_keyframe_ != nullptr)
      {
        this->ptr_last_keyframe_ = this->ptr_current_keyframe_;
        this->ptr_current_keyframe_ = nullptr;
      }
      this->ptr_last_frame_ = this->ptr_current_frame_;
      this->ptr_current_frame_ = pFrame;
      this->ptr_current_frame_ = this->ptr_last_keyframe_;
      trackWithLastKeyFrame(this->ptr_last_keyframe_,
                            this->ptr_current_frame_);
      //AK_DLOG_INFO << "Pose optimization done!";
      //exit(0);
    }
    //AK_LOG_INFO << "Size of frames: " << Frame::frames_vector.size();
    //AK_LOG_INFO << "Size of hash_frames: " << Frame::hash_frames.size();
    //AK_LOG_INFO << "Size of keyframes: " << Frame::keyframes_vector.size();
    //AK_LOG_INFO << "Size of hash_keyframes: " << Frame::hash_keyframes.size();
    return 0;
  }

  bool VisualOdometry::InitializeVO(const Frame::Ptr &ptr_initialized_frame,
                                    const Frame::Ptr &ptr_current_frame,
                                    cv::Mat &Rcw,
                                    cv::Mat &tcw) {
    setRandomSeed(0);
    ransec_matched_points_set_ = std::vector<std::vector<cv::DMatch>>(vo_params_.init_params_.max_iterations_,
                                                                      std::vector<cv::DMatch>(8, cv::DMatch()));
    for (size_t i = 0; i < vo_params_.init_params_.max_iterations_; ++i) {
      //AK_DLOG_INFO << "Iteration: " << i;
      auto matches = ptr_initialized_frame->best_matches_;
      for (size_t j = 0; j < 8; ++j) {
        auto random_index_point_pair = randomInt(0, matches.size() - 1);
        auto index_matched = matches[random_index_point_pair];
        ransec_matched_points_set_[i][j] = index_matched;
        matches[random_index_point_pair] = matches.back();
        matches.pop_back();
        //AK_DLOG_WARNING << "Num of point: " << j;
      }
    }
    cv::Mat matrix_H21, matrix_F21;
    std::vector<cv::DMatch> inliers_H, inliers_F;
    //auto score_H = FindHomography(inliers_H, matrix_H21);
    //auto score_F = FindFundamental(inliers_F, matrix_F21);
    //std::thread threadHomoCompute(Frame::FindHomography,
    //                              std::ref(inliers_H),
    //                              std::ref(matrix_H21));
    //std::thread threadFundCompute(Frame::FindFundamental,
    //                              std::ref(inliers_F),
    //                              std::ref(matrix_F21));
    //threadHomoCompute.join();
    //threadFundCompute.join();
    std::future<float> future_scoreH = std::async(&VisualOdometry::FindHomography,
                                                  this,
                                                  std::ref(inliers_H),
                                                  std::ref(matrix_H21));
    std::future<float> future_scoreF = std::async(&VisualOdometry::FindFundamental,
                                                  this,
                                                  std::ref(inliers_F),
                                                  std::ref(matrix_F21));
    AK_DLOG_INFO << "Initializing VO...";
    float score_H = future_scoreH.get();
    float score_F = future_scoreF.get();
    //float score_F = Frame::FindFundamental(inliers_F, matrix_F21);
    float ratio_homography = score_H / (score_H + score_F);
    AK_DLOG_INFO << "inliers_H : " << inliers_H.size();
    AK_DLOG_INFO << "inliers_F : " << inliers_F.size();
    AK_DLOG_INFO << "ratio_homography: " << ratio_homography;
    if (ratio_homography > 0.40) {
      AK_DLOG_WARNING << "Reconstruct From Homography.";
      //ptr_initialized_frame_->best_matches_inliers_ = inliers_H;
      return ReconstructFromHomo(inliers_H,
                                 matrix_H21,
                                 vo_params_.K_,
                                 Rcw,
                                 tcw,
                                 Frame::raw_init_landmarks,
                                 1.0,
                                 50);
    } else {
      AK_DLOG_WARNING << "Reconstruct From Fundamental.";
      //ptr_initialized_frame_->best_matches_inliers_ = inliers_F;
      return ReconstructFromFund(inliers_F,
                                 matrix_F21,
                                 vo_params_.K_,
                                 Rcw,
                                 tcw,
                                 Frame::raw_init_landmarks,
                                 1.0,
                                 50);
    }
    return false;
  }


  float VisualOdometry::FindHomography(std::vector<cv::DMatch> &matched_inliers,
                                       cv::Mat &H21) {
    AK_DLOG_INFO << "Find homography...";
    std::vector<cv::KeyPoint> normalized_kps_init, normalized_kps_cur;
    cv::Mat transform_init, transform_cur;
    // TODO: (aliben.develop@gmail.com)
    // NormalizeKeyPoints should be optimized, cause in all iteration keypoints is the same set. It don't have to calculate each time to find homo/fund matrix.
    NormalizeKeyPoints(ptr_initialized_frame_->getKeyPoints(),
                       normalized_kps_init,
                       transform_init);
    //AK_DLOG_INFO << "Mat1_homo: \n" << transform_init;
    NormalizeKeyPoints(ptr_current_frame_->getKeyPoints(),
                       normalized_kps_cur,
                       transform_cur);
    //AK_DLOG_INFO << "Mat2_homo: \n" << transform_cur;
    cv::Mat transform_cur_transpose = transform_cur.t();
    float score = 0.0;
    std::vector<cv::KeyPoint> ransec_init_keypoints(8);
    std::vector<cv::KeyPoint> ransec_cur_keypoints(8);
    cv::Mat H21_tmp, H12_tmp;
    float current_score;
    for (size_t iter = 0; iter < vo_params_.init_params_.max_iterations_; ++iter) {
      for (auto n = 0; n < 8; ++n) {
        auto queryIdx = ransec_matched_points_set_[iter][n].queryIdx;
        auto trainIdx = ransec_matched_points_set_[iter][n].trainIdx;
        ransec_init_keypoints[n] = normalized_kps_init[queryIdx];
        ransec_cur_keypoints[n] = normalized_kps_cur[trainIdx];
      }
      // ComputeH21
      cv::Mat H21_normalized = ComputeH21(ransec_init_keypoints, ransec_cur_keypoints);
      H21_tmp = transform_cur.inv() * H21_normalized * transform_init;
      H12_tmp = H21_tmp.inv();
      // CheckH21 - sigma = 1.0
      current_score = CheckHomography(H21_tmp, H12_tmp, matched_inliers, 1.0);
      if (current_score > score) {
        H21 = H21_tmp.clone();
        score = current_score;
      }
    }
    return score;
  }


  float VisualOdometry::FindFundamental(std::vector<cv::DMatch> &matched_inliers,
                                        cv::Mat &F21) {
    AK_DLOG_INFO << "Find fundamental ...";
    std::vector<cv::KeyPoint> normalized_kps_init, normalized_kps_cur;
    cv::Mat transform_init, transform_cur;
    // TODO: (aliben.develop@gmail.com)
    // NormalizeKeyPoints should be optimized, cause in all iteration keypoints is the same set. It don't have to calculate each time to find homo/fund matrix.
    NormalizeKeyPoints(ptr_initialized_frame_->getKeyPoints(),
                       normalized_kps_init,
                       transform_init);
    NormalizeKeyPoints(ptr_current_frame_->getKeyPoints(),
                       normalized_kps_cur,
                       transform_cur);
    cv::Mat transform_cur_transpose = transform_cur.t();
    float score = 0.0;

    std::vector<cv::KeyPoint> ransec_init_keypoints(8);
    std::vector<cv::KeyPoint> ransec_cur_keypoints(8);

    cv::Mat F21_tmp;
    float current_score;
    std::vector<cv::DMatch> ransec_matched_inliers;
    for (size_t iter = 0; iter < vo_params_.init_params_.max_iterations_; ++iter) {
      for (auto n = 0; n < 8; ++n) {
        ransec_init_keypoints[n] = normalized_kps_init[ransec_matched_points_set_[iter][n].queryIdx];
        ransec_cur_keypoints[n] = normalized_kps_cur[ransec_matched_points_set_[iter][n].trainIdx];
      }
      // ComputeF21
      cv::Mat F21_normalized = ComputeF21(ransec_init_keypoints, ransec_cur_keypoints);
      F21_tmp = transform_cur_transpose * F21_normalized * transform_init;
      // CheckF21 - sigma = 1.0
      current_score = CheckFundamental(F21_tmp, ransec_matched_inliers, 1.0);
      if (current_score > score) {
        F21 = F21_tmp.clone();
        score = current_score;
        matched_inliers = ransec_matched_inliers;
      }
    }
    return score;
  }

  void VisualOdometry::NormalizeKeyPoints(const std::vector<cv::KeyPoint> &kps,
                                          std::vector<cv::KeyPoint> &kps_normalized,
                                          cv::Mat &Transform) {
    float mean_x = 0.0;
    float mean_y = 0.0;
    const size_t KEYPOINT_SIZE = kps.size();
    kps_normalized.resize(KEYPOINT_SIZE);
    for (auto &kp: kps) {
      mean_x += kp.pt.x;
      mean_y += kp.pt.y;
    }
    mean_x /= KEYPOINT_SIZE;
    mean_y /= KEYPOINT_SIZE;
    float deviation_x = 0.0;
    float deviation_y = 0.0;
    size_t i = 0;
    for (auto &kp: kps) {
      kps_normalized[i].pt.x = kp.pt.x - mean_x;
      kps_normalized[i].pt.y = kp.pt.y - mean_y;
      deviation_x += std::abs(kps_normalized[i].pt.x);
      deviation_y += std::abs(kps_normalized[i].pt.y);
      ++i;
    }
    deviation_x /= KEYPOINT_SIZE;
    deviation_y /= KEYPOINT_SIZE;
    auto deviation_x_inv = 1.0 / deviation_x;
    auto deviation_y_inv = 1.0 / deviation_y;
    for (auto &kp_normal: kps_normalized) {
      kp_normal.pt.x *= deviation_x_inv;
      kp_normal.pt.y *= deviation_y_inv;
    }
    Transform = cv::Mat::eye(3, 3, CV_32F);
    Transform.at<float>(0, 0) = deviation_x_inv;
    Transform.at<float>(1, 1) = deviation_y_inv;
    Transform.at<float>(0, 2) = -mean_x * deviation_x_inv;
    Transform.at<float>(1, 2) = -mean_y * deviation_y_inv;
  }

  cv::Mat VisualOdometry::ComputeH21(const std::vector<cv::KeyPoint> &keypoints_init,
                                     const std::vector<cv::KeyPoint> &keypoints_cur) {
    const int N = keypoints_init.size();
    cv::Mat A(2 * N, 9, CV_32F);
    for (int i = 0; i < N; i++) {
      const float u1 = keypoints_init[i].pt.x;
      const float v1 = keypoints_init[i].pt.y;
      const float u2 = keypoints_cur[i].pt.x;
      const float v2 = keypoints_cur[i].pt.y;
      A.at<float>(2 * i, 0) = 0.0;
      A.at<float>(2 * i, 1) = 0.0;
      A.at<float>(2 * i, 2) = 0.0;
      A.at<float>(2 * i, 3) = -u1;
      A.at<float>(2 * i, 4) = -v1;
      A.at<float>(2 * i, 5) = -1;
      A.at<float>(2 * i, 6) = v2 * u1;
      A.at<float>(2 * i, 7) = v2 * v1;
      A.at<float>(2 * i, 8) = v2;

      A.at<float>(2 * i + 1, 0) = u1;
      A.at<float>(2 * i + 1, 1) = v1;
      A.at<float>(2 * i + 1, 2) = 1;
      A.at<float>(2 * i + 1, 3) = 0.0;
      A.at<float>(2 * i + 1, 4) = 0.0;
      A.at<float>(2 * i + 1, 5) = 0.0;
      A.at<float>(2 * i + 1, 6) = -u2 * u1;
      A.at<float>(2 * i + 1, 7) = -u2 * v1;
      A.at<float>(2 * i + 1, 8) = -u2;
    }
    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    return vt.row(8).reshape(0, 3);
  }

  cv::Mat VisualOdometry::ComputeF21(const std::vector<cv::KeyPoint> &keypoints_init,
                                     const std::vector<cv::KeyPoint> &keypoints_cur) {
    const int N = keypoints_init.size();
    cv::Mat A(N, 9, CV_32F);

    for (int i = 0; i < N; i++) {
      const float u1 = keypoints_init[i].pt.x;
      const float v1 = keypoints_init[i].pt.y;
      const float u2 = keypoints_cur[i].pt.x;
      const float v2 = keypoints_cur[i].pt.y;

      A.at<float>(i, 0) = u2 * u1;
      A.at<float>(i, 1) = u2 * v1;
      A.at<float>(i, 2) = u2;
      A.at<float>(i, 3) = v2 * u1;
      A.at<float>(i, 4) = v2 * v1;
      A.at<float>(i, 5) = v2;
      A.at<float>(i, 6) = u1;
      A.at<float>(i, 7) = v1;
      A.at<float>(i, 8) = 1;
    }

    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat Fpre = vt.row(8).reshape(0, 3);
    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    w.at<float>(2) = 0;
    return u * cv::Mat::diag(w) * vt;
  }


  float VisualOdometry::CheckHomography(const cv::Mat &H21,
                                        const cv::Mat &H12,
                                        std::vector<cv::DMatch> &matched_inliers,
                                        float sigma) {
    matched_inliers.clear();
    const size_t N = ptr_initialized_frame_->best_matches_.size();

    const float h11 = H21.at<float>(0, 0);
    const float h12 = H21.at<float>(0, 1);
    const float h13 = H21.at<float>(0, 2);
    const float h21 = H21.at<float>(1, 0);
    const float h22 = H21.at<float>(1, 1);
    const float h23 = H21.at<float>(1, 2);
    const float h31 = H21.at<float>(2, 0);
    const float h32 = H21.at<float>(2, 1);
    const float h33 = H21.at<float>(2, 2);

    const float h11inv = H12.at<float>(0, 0);
    const float h12inv = H12.at<float>(0, 1);
    const float h13inv = H12.at<float>(0, 2);
    const float h21inv = H12.at<float>(1, 0);
    const float h22inv = H12.at<float>(1, 1);
    const float h23inv = H12.at<float>(1, 2);
    const float h31inv = H12.at<float>(2, 0);
    const float h32inv = H12.at<float>(2, 1);
    const float h33inv = H12.at<float>(2, 2);

    float score = 0;
    const float th = 5.991;
    const float inv_sigma_square = 1.0 / (sigma * sigma);
    for (size_t i = 0; i < N; ++i) {
      bool isInliers = true;
      auto index_kp_init = ptr_initialized_frame_->best_matches_[i].queryIdx;
      auto index_kp_cur = ptr_initialized_frame_->best_matches_[i].trainIdx;
      const auto &kp_init = ptr_initialized_frame_->keypoints_[index_kp_init];
      const auto &kp_cur = ptr_current_frame_->keypoints_[index_kp_cur];

      const float u1 = kp_init.pt.x;
      const float v1 = kp_init.pt.y;
      const float u2 = kp_cur.pt.x;
      const float v2 = kp_cur.pt.y;
      const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
      const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
      const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;
      const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);
      const float chiSquare1 = squareDist1 * inv_sigma_square;

      if (chiSquare1 > th) {
        isInliers = false;
      } else {
        score += th - chiSquare1;
      }

      // Reprojection error in second image
      // x1in2 = H21*x1

      const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
      const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
      const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;
      const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);
      const float chiSquare2 = squareDist2 * inv_sigma_square;

      if (chiSquare2 > th) {
        isInliers = false;
      } else {
        score += th - chiSquare2;
      }

      if (isInliers) {
        //AK_DLOG_ERROR << "One inliers captured.";
        matched_inliers.push_back(ptr_initialized_frame_->best_matches_[i]);
      }
    }
    //AK_DLOG_ERROR << "H-inliers.size = " << matched_inliers.size();
    return score;
  }

  float VisualOdometry::CheckFundamental(const cv::Mat &F21,
                                         std::vector<cv::DMatch> &matched_inliers,
                                         float sigma) {
    matched_inliers.clear();
    const int N = ptr_initialized_frame_->best_matches_.size();
    const float f11 = F21.at<float>(0, 0);
    const float f12 = F21.at<float>(0, 1);
    const float f13 = F21.at<float>(0, 2);
    const float f21 = F21.at<float>(1, 0);
    const float f22 = F21.at<float>(1, 1);
    const float f23 = F21.at<float>(1, 2);
    const float f31 = F21.at<float>(2, 0);
    const float f32 = F21.at<float>(2, 1);
    const float f33 = F21.at<float>(2, 2);

    float score = 0;
    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++) {
      bool isInliers = true;

      auto index_kp_init = ptr_initialized_frame_->best_matches_[i].queryIdx;
      auto index_kp_cur = ptr_initialized_frame_->best_matches_[i].trainIdx;
      const auto &kp_init = ptr_initialized_frame_->keypoints_[index_kp_init];
      const auto &kp_cur = ptr_current_frame_->keypoints_[index_kp_cur];

      const float u1 = kp_init.pt.x;
      const float v1 = kp_init.pt.y;
      const float u2 = kp_cur.pt.x;
      const float v2 = kp_cur.pt.y;

      // Reprojection error in second image
      // l2=F21x1=(a2,b2,c2)

      const float a2 = f11 * u1 + f12 * v1 + f13;
      const float b2 = f21 * u1 + f22 * v1 + f23;
      const float c2 = f31 * u1 + f32 * v1 + f33;

      const float num2 = a2 * u2 + b2 * v2 + c2;
      const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);
      const float chiSquare1 = squareDist1 * invSigmaSquare;

      if (chiSquare1 > th) {
        isInliers = false;
      } else {
        score += thScore - chiSquare1;
      }

      // Reprojection error in second image
      // l1 =x2tF21=(a1,b1,c1)
      const float a1 = f11 * u2 + f21 * v2 + f31;
      const float b1 = f12 * u2 + f22 * v2 + f32;
      const float c1 = f13 * u2 + f23 * v2 + f33;

      const float num1 = a1 * u1 + b1 * v1 + c1;
      const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);
      const float chiSquare2 = squareDist2 * invSigmaSquare;

      if (chiSquare2 > th) {
        isInliers = false;
      } else {
        score += thScore - chiSquare2;
      }

      if (isInliers) {
        //AK_DLOG_ERROR << "One inliers captured.";
        matched_inliers.push_back(ptr_initialized_frame_->best_matches_[i]);
        //AK_DLOG_WARNING << matched_inliers.size();
      }
    }
    //AK_DLOG_ERROR << "F-inliers.size = " << matched_inliers.size();
    return score;
  }

  bool VisualOdometry::ReconstructFromHomo(std::vector<cv::DMatch> &matched_inliers,
                                           cv::Mat &H21,
                                           cv::Mat &K,
                                           cv::Mat &R21,
                                           cv::Mat &t21,
                                           std::vector<std::pair<size_t, cv::Point3f>> &raw_init_landmarks,
                                           float min_parallax,
                                           int min_triangulated) {
    //AK_DLOG_WARNING << "Reconstructing Homography";
    cv::Mat K_inv = K.inv();
    cv::Mat A = K_inv * H21 * K;

    cv::Mat U, w, Vt, V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
    V = Vt.t();

    float s = cv::determinant(U) * cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001) {
      return false;
    }

    std::vector<cv::Mat> vector_R, vector_t, vector_n;
    vector_R.reserve(8);
    vector_t.reserve(8);
    vector_n.reserve(8);

    float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

    float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for (int i = 0; i < 4; i++) {
      cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
      Rp.at<float>(0, 0) = ctheta;
      Rp.at<float>(0, 2) = -stheta[i];
      Rp.at<float>(2, 0) = stheta[i];
      Rp.at<float>(2, 2) = ctheta;

      cv::Mat R = s * U * Rp * Vt;
      vector_R.push_back(R);

      cv::Mat tp(3, 1, CV_32F);
      tp.at<float>(0) = x1[i];
      tp.at<float>(1) = 0;
      tp.at<float>(2) = -x3[i];
      tp *= d1 - d3;

      cv::Mat t = U * tp;
      vector_t.push_back(t / cv::norm(t));

      cv::Mat np(3, 1, CV_32F);
      np.at<float>(0) = x1[i];
      np.at<float>(1) = 0;
      np.at<float>(2) = x3[i];

      cv::Mat n = V * np;
      if (n.at<float>(2) < 0)
        n = -n;
      vector_n.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for (int i = 0; i < 4; i++) {
      cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
      Rp.at<float>(0, 0) = cphi;
      Rp.at<float>(0, 2) = sphi[i];
      Rp.at<float>(1, 1) = -1;
      Rp.at<float>(2, 0) = sphi[i];
      Rp.at<float>(2, 2) = -cphi;

      cv::Mat R = s * U * Rp * Vt;
      vector_R.push_back(R);

      cv::Mat tp(3, 1, CV_32F);
      tp.at<float>(0) = x1[i];
      tp.at<float>(1) = 0;
      tp.at<float>(2) = x3[i];
      tp *= d1 + d3;

      cv::Mat t = U * tp;
      vector_t.push_back(t / cv::norm(t));

      cv::Mat np(3, 1, CV_32F);
      np.at<float>(0) = x1[i];
      np.at<float>(1) = 0;
      np.at<float>(2) = x3[i];

      cv::Mat n = V * np;
      if (n.at<float>(2) < 0)
        n = -n;
      vector_n.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;
    int bestSolutionIdx = -1;
    float bestParallax = -1;

    std::vector<std::pair<size_t, cv::Point3f>> bestP3D;
    //    std::vector<std::pair<size_t, cv::Point3f>> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for (size_t i = 0; i < 8; i++) {
      float parallaxi{0.0};
      std::vector<std::pair<size_t, cv::Point3f>> vP3Di;
      int nGood = CheckRT(vector_R[i],
                          vector_t[i],
                          ptr_initialized_frame_->getKeyPoints(),
                          ptr_current_frame_->getKeyPoints(),
                          matched_inliers,
                          K,
                          vP3Di,
                          4.0 * vo_params_.init_params_.sigma_2_,
                          parallaxi);

      if (nGood > bestGood) {
        secondBestGood = bestGood;
        bestGood = nGood;
        bestSolutionIdx = i;
        bestParallax = parallaxi;
        bestP3D = vP3Di;
      } else if (nGood > secondBestGood) {
        secondBestGood = nGood;
      }
    }

    auto N = matched_inliers.size();
    ptr_last_frame_->best_matches_inliers_ = matched_inliers;

    if (secondBestGood < 0.75 * bestGood && bestParallax >= min_parallax && bestGood > min_triangulated &&
        bestGood > 0.9 * N) {
      vector_R[bestSolutionIdx].copyTo(R21);
      vector_t[bestSolutionIdx].copyTo(t21);
      raw_init_landmarks = bestP3D;
      return true;
    }
    return false;
  }

  bool VisualOdometry::ReconstructFromFund(std::vector<cv::DMatch> &matched_inliers,
                                           cv::Mat &F21,
                                           cv::Mat &K,
                                           cv::Mat &R21,
                                           cv::Mat &t21,
                                           std::vector<std::pair<size_t, cv::Point3f>> &raw_init_landmarks,
                                           float min_parallax,
                                           int min_triangulated) {
    //AK_DLOG_WARNING << "Reconstructing Fundamental";
    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t() * F21 * K;
    cv::Mat R1, R2, t;
    // Recover the 4 motion hypotheses
    DecomposeE(E21, R1, R2, t);

    cv::Mat t1 = t;
    cv::Mat t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    //    std::vector<std::pair<size_t, cv::Point3f>> bestP3D;
    std::vector<std::pair<size_t, cv::Point3f>> vP3D1, vP3D2, vP3D3, vP3D4;
    //    std::vector<std::pair<size_t, cv::Point3f>> bestTriangulated1,bestTriangulated2,bestTriangulated3, bestTriangulated4;
    float parallax1, parallax2, parallax3, parallax4;
    int nGood1 = CheckRT(R1,
                         t1,
                         ptr_initialized_frame_->getKeyPoints(),
                         ptr_current_frame_->getKeyPoints(),
                         matched_inliers,
                         K,
                         vP3D1,
                         4.0 * vo_params_.init_params_.sigma_2_,
                         parallax1);
    int nGood2 = CheckRT(R2,
                         t1,
                         ptr_initialized_frame_->getKeyPoints(),
                         ptr_current_frame_->getKeyPoints(),
                         matched_inliers,
                         K,
                         vP3D2,
                         4.0 * vo_params_.init_params_.sigma_2_,
                         parallax2);
    int nGood3 = CheckRT(R1,
                         t2,
                         ptr_initialized_frame_->getKeyPoints(),
                         ptr_current_frame_->getKeyPoints(),
                         matched_inliers,
                         K,
                         vP3D3,
                         4.0 * vo_params_.init_params_.sigma_2_,
                         parallax3);
    int nGood4 = CheckRT(R2,
                         t2,
                         ptr_initialized_frame_->getKeyPoints(),
                         ptr_current_frame_->getKeyPoints(),
                         matched_inliers,
                         K,
                         vP3D4,
                         4.0 * vo_params_.init_params_.sigma_2_,
                         parallax4);

    int maxGood = std::max(nGood1, std::max(nGood2, std::max(nGood3, nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    auto N = matched_inliers.size();
    ptr_last_frame_->best_matches_inliers_ = matched_inliers;
    int nMinGood = std::max(static_cast<int>(0.9 * N), min_triangulated);

    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
      nsimilar++;
    if (nGood2 > 0.7 * maxGood)
      nsimilar++;
    if (nGood3 > 0.7 * maxGood)
      nsimilar++;
    if (nGood4 > 0.7 * maxGood)
      nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if (maxGood < nMinGood || nsimilar > 1) {
      AK_DLOG_ERROR << "Triangulated is too less.";
      return false;
    }

    // If best reconstruction has enough parallax initialize
    if (maxGood == nGood1) {
      if (parallax1 > min_parallax) {
        raw_init_landmarks = vP3D1;
        //        raw_init_landmarks = bestTriangulated1;
        //vbTriangulated = vbTriangulated1;
        R1.copyTo(R21);
        t1.copyTo(t21);
        return true;
      }
    } else if (maxGood == nGood2) {
      if (parallax2 > min_parallax) {
        raw_init_landmarks = vP3D2;
        //vbTriangulated = vbTriangulated2;
        //        raw_init_landmarks = bestTriangulated2;
        R2.copyTo(R21);
        t1.copyTo(t21);
        return true;
      }
    } else if (maxGood == nGood3) {
      if (parallax3 > min_parallax) {
        raw_init_landmarks = vP3D3;
        //vbTriangulated = vbTriangulated3;
        //        raw_init_landmarks = bestTriangulated3;
        R1.copyTo(R21);
        t2.copyTo(t21);
        return true;
      }
    } else if (maxGood == nGood4) {
      if (parallax4 > min_parallax) {
        raw_init_landmarks = vP3D4;
        //vbTriangulated = vbTriangulated4;
        //        raw_init_landmarks = bestTriangulated4;
        R2.copyTo(R21);
        t2.copyTo(t21);
        return true;
      }
    }
    return false;
  }


  int VisualOdometry::CheckRT(const cv::Mat &R,
                              const cv::Mat &t,
                              const std::vector<cv::KeyPoint> &keypoints_last,
                              const std::vector<cv::KeyPoint> &keypoints_cur,
                              const std::vector<cv::DMatch> &matched_inliers,
                              const cv::Mat &K,
                              std::vector<std::pair<size_t, cv::Point3f>> &raw_init_landmarks,
                              float th2,
                              float &parallax
  ) {
    // Calibration parameters
    const float fx = K.at<float>(0, 0);
    const float fy = K.at<float>(1, 1);
    const float cx = K.at<float>(0, 2);
    const float cy = K.at<float>(1, 2);

    //vbGood = vector<bool>(vKeys1.size(),false);
    //vP3D.resize(vKeys1.size()); // Just insert pair

    std::vector<float> vCosParallax;
    vCosParallax.reserve(matched_inliers.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

    cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3, 4, CV_32F);
    R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
    t.copyTo(P2.rowRange(0, 3).col(3));
    P2 = K * P2;
    cv::Mat O2 = -R.t() * t;
    int nGood = 0;

    //auto triangulated_matches = matched_inliers;
    std::vector<std::pair<size_t, cv::Point3f>> triangulated_matches;

    size_t match_size = matched_inliers.size();
    for (size_t i = 0; i < match_size; ++i) {
      //if(!vbMatchesInliers[i])
      //    continue;

      const cv::KeyPoint &kp1 = keypoints_last[matched_inliers[i].queryIdx];
      const cv::KeyPoint &kp2 = keypoints_cur[matched_inliers[i].trainIdx];
      //cv::Mat p3dC1;
      cv::Mat p3dC1 = cv::Mat(3, 1, CV_32F);
      Triangulate(kp1, kp2, P1, P2, p3dC1);
      if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2))) {
        //vbGood[vMatches12[i].first]=false;
        // Remove this match OR just ignore it
        continue;
      }

      // Check parallax
      cv::Mat normal1 = p3dC1 - O1;
      float dist1 = cv::norm(normal1);

      cv::Mat normal2 = p3dC1 - O2;
      float dist2 = cv::norm(normal2);

      float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

      // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      if (p3dC1.at<float>(2) <= 0 && cosParallax < 0.99998)
        continue;

      // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      cv::Mat p3dC2 = R * p3dC1 + t;

      if (p3dC2.at<float>(2) <= 0 && cosParallax < 0.99998)
        continue;

      // Check reprojection error in first image
      float im1x, im1y;
      float invZ1 = 1.0 / p3dC1.at<float>(2);
      im1x = fx * p3dC1.at<float>(0) * invZ1 + cx;
      im1y = fy * p3dC1.at<float>(1) * invZ1 + cy;

      float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

      if (squareError1 > th2)
        continue;

      // Check reprojection error in second image
      float im2x, im2y;
      float invZ2 = 1.0 / p3dC2.at<float>(2);
      im2x = fx * p3dC2.at<float>(0) * invZ2 + cx;
      im2y = fy * p3dC2.at<float>(1) * invZ2 + cy;

      float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

      if (squareError2 > th2)
        continue;

      vCosParallax.push_back(cosParallax);
      //vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
      auto landmark = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
      //raw_init_landmarks.push_back(std::make_pair(matched_inliers[i].queryIdx, landmark));
      raw_init_landmarks.push_back(std::make_pair(i, landmark));

      nGood++;

      if (cosParallax < 0.99998)
        //vbGood[vMatches12[i].first]=true;
        //enable i as good keypoints;
        ;
    }


    if (nGood > 0) {
      sort(vCosParallax.begin(), vCosParallax.end());
      size_t idx = std::min(50, int(vCosParallax.size() - 1));
      parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
    } else {
      parallax = 0;
    }
    return nGood;
  }


  void VisualOdometry::Triangulate(const cv::KeyPoint &kp_init,
                                   const cv::KeyPoint &kp_cur,
                                   const cv::Mat &pose_init,
                                   const cv::Mat &pose_cur,
                                   cv::Mat &x3D) {
    cv::Mat A(4, 4, CV_32F);

    A.row(0) = kp_init.pt.x * pose_init.row(2) - pose_init.row(0);
    A.row(1) = kp_init.pt.y * pose_init.row(2) - pose_init.row(1);
    A.row(2) = kp_cur.pt.x * pose_cur.row(2) - pose_cur.row(0);
    A.row(3) = kp_cur.pt.y * pose_cur.row(2) - pose_cur.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
  }

  void VisualOdometry::DecomposeE(const cv::Mat &E,
                                  cv::Mat &R1,
                                  cv::Mat &R2,
                                  cv::Mat &t) {
    cv::Mat u, w, vt;
    cv::SVD::compute(E, w, u, vt);

    u.col(2).copyTo(t);
    t = t / cv::norm(t);

    cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
    W.at<float>(0, 1) = -1;
    W.at<float>(1, 0) = 1;
    W.at<float>(2, 2) = 1;

    R1 = u * W * vt;
    if (cv::determinant(R1) < 0)
      R1 = -R1;

    R2 = u * W.t() * vt;
    if (cv::determinant(R2) < 0)
      R2 = -R2;
  }

  void VisualOdometry::InitMap() {
    auto raw_init_landmarks_size = Frame::raw_init_landmarks.size();
    AK_DLOG_WARNING << "Init landmark: " << raw_init_landmarks_size;
    for (size_t i = 0; i < raw_init_landmarks_size; ++i) {
      //cv::Mat position_at_world(Frame::raw_init_landmarks[i].second);
      auto position_at_world = Frame::raw_init_landmarks[i].second;
      auto idx_matched = Frame::raw_init_landmarks[i].first;
      auto ptr_landmark = Landmark::CreateLandmark(ptr_initialized_frame_, ptr_current_frame_, position_at_world);
      //      AK_DLOG_ERROR << "Num: " << ptr_initialized_frame_->best_matches_inliers_[idx_matched].queryIdx;
      //AK_DLOG_INFO << "Init Frame:";
      auto queryIdx = ptr_initialized_frame_->best_matches_inliers_[idx_matched].queryIdx;
      auto trainIdx = ptr_initialized_frame_->best_matches_inliers_[idx_matched].trainIdx;
      ptr_initialized_frame_->insertLandmark(ptr_landmark, queryIdx);
      //AK_DLOG_INFO << "Current Frame:";
      ptr_current_frame_->insertLandmark(ptr_landmark, trainIdx);

      // Add Observation
      ptr_landmark->addObserver(ptr_initialized_frame_, queryIdx);
      ptr_landmark->addObserver(ptr_current_frame_, trainIdx);

      ptr_landmark->updateLandmark();
      ptr_map_->addLandmark(ptr_landmark);
    }

    // Add co-vision
    ptr_initialized_frame_->updateCovision();
    ptr_current_frame_->updateCovision();

    // Optimizer setup
    AK_DLOG_INFO << "Optimization by reprojection error.";
    // Initial a scale for slam, !!! Attension, it is not real distance in the world, they differ with a scale factor, so if we need a real distance for map, there must have another source to confirm the scale factor
    //ptr_optimizer_->bundleAdjustment(keyframes_vector_, landmarks_, vo_params_.K_, vo_params_.init_params_.sigma_, 20);
    ptr_optimizer_->globalBundleAdjustment(ptr_map_, vo_params_.K_, vo_params_.init_params_.sigma_, 20);
    AK_DLOG_INFO << "Global optimizating Done!";
    auto median_depth = ptr_initialized_frame_->computeMedianDepth(2);
    auto inv_median_depth = 1.0f / median_depth;

    //    if(median_depth < 0 || )
    cv::Mat transform = ptr_current_frame_->getPose();
    transform.col(3).rowRange(0, 3) = transform.col(3).rowRange(0, 3) * inv_median_depth;
    //    ptr_current_frame->setPose()
    ptr_current_frame_->transform_camera_at_world_ = transform;

    // Scale All point
    auto landmarks = ptr_initialized_frame_->getLandmarks();
    for (const auto &landmark_pair:landmarks)
    {
      auto ptr_landmark = landmark_pair.second;
      if (ptr_landmark != nullptr)
      {
        ptr_landmark->setPosition(ptr_landmark->getPosition() * inv_median_depth);
      }
    }
  }

  bool VisualOdometry::trackWithLastKeyFrame(Frame::Ptr &ptr_last_keyframe, Frame::Ptr &ptr_new_frame) {
    ORBmatcher matcher(0.7, true);
    //    std::vector<Landmark::Ptr> landmarks_matched;
    std::unordered_map<size_t, Landmark::Ptr> landmarks_matched;
    int num_matches = matcher.SearchByBoW(ptr_last_keyframe, ptr_new_frame, landmarks_matched);

    AK_DLOG_INFO << "Origin: " << landmarks_matched.size();
    auto pose = ptr_last_keyframe->getPose();
    ptr_new_frame->setTF(pose);

    ptr_optimizer_->PoseOptimization(ptr_new_frame,
                                     vo_params_.K_,
                                     vo_params_.init_params_.sigma_);
    AK_DLOG_INFO << "Pose Optimation Done!";
    //      this->keyframes_vector.push_back(pFrame);
    //      this->hash_keyframes.insert(std::make_pair(pFrame->factory_id, pFrame));
    //      this->frames_vector.push_back(pFrame);
    //      this->hash_frames.insert(std::make_pair(pFrame->factory_id, pFrame));
    if (num_matches < 15) {
      return false;
    }

    int num_good_mappoint = 0;

    // MARK: (aliben.develop@gmail.com)
    // landmark should be observe at least one frame,
    // which means that there must have a frame recovery the landmark
    for (const auto &landmark_pair:ptr_new_frame->getLandmarks()) {
      const auto &ptr_landmark = landmark_pair.second;
      if (ptr_landmark != nullptr && ptr_landmark->isGood() == true) {
        if (ptr_landmark->getObservationTimes() > 0) {
          ++num_good_mappoint;
        }
      }
    }
    return num_good_mappoint >= 10;
  }

  bool trackLocalMap()
  {
    //updateLocalMap()
    //SearchLocalPoint()
    return false;
  }
}
