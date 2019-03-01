/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-02-28 21:54:14
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/frame.hh>
#include <visual_slam/ORBmatcher.hh>
#include <thread>
#include <future>
#include <visual_slam/utils/io.hh>
#include <visual_slam/utils/time.hh>

//CODE
namespace ak
{
  using namespace std::chrono;
  /*const*/ int NUM_FEATURES = 2000;
  /*const*/ float LEVEL_SCALE_FACTOR = 1.2;
  /*const*/ int NUM_LEVELS = 8;
  /*const*/ int INIT_FAST = 20;
  /*const*/ int MIN_FAST = 7;
  //cv::Ptr<cv::ORB> Frame::ptr_orb_ = cv::ORB::create(2000);
  ORBextractor::Ptr Frame::ptr_orb_extractor_init_advanced = std::make_shared<ORBextractor>(2*NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);
  ORBextractor::Ptr Frame::ptr_orb_extractor_advanced = std::make_shared<ORBextractor>(NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);
  //ORBmatcher::Ptr Frame::ptr_orb_matcher_init_advanced = std::make_shared<ORBextractor>(0.9, true);
  cv::Ptr<cv::DescriptorMatcher> Frame::matcher  = cv::DescriptorMatcher::create("BruteForce-Hamming");
  int Frame::factory_id = 0;
  Frame::Ptr Frame::ptr_initialized_frame = nullptr;
  Frame::Ptr Frame::ptr_current_frame = nullptr;
  Frame::Ptr Frame::ptr_last_frame = nullptr;
  Frame::Ptr Frame::ptr_last_keyframe = nullptr;
  std::vector<Frame::Ptr> Frame::frames_vector;
  std::vector<Frame::Ptr> Frame::keyframes_vector;
  std::unordered_map<Frame::ID_t, Frame::Ptr> Frame::hash_frames;
  std::unordered_map<Frame::ID_t, Frame::Ptr> Frame::hash_keyframes;
  std::vector<std::vector<cv::DMatch>> Frame::MATCHED_POINTS_SET;
  std::vector<std::pair<size_t, cv::Point3f>> Frame::init_landmarks;

  float Frame::match_ratio_ = 2.0;
  float Frame::sigma = 1.0;
  VO_STATE Frame::vo_state = NO_IMAGE;
  bool Frame::enable_show = false;
  const size_t MAX_ITERATION = 200;
  cv::Mat Frame::K;

  Frame::Frame(ID_t id)
    : id_(id)
  {
    this->ptr_orb_matcher_init_advanced = new ORBmatcher(0.9, true);
  }

  Frame::Ptr Frame::CreateFrame(const cv::Mat& image,
                                cv::Mat& image_with_keypoints)
  {
    // TODO: (aliben.develop@gmail.com)
    // Fix return value;
    auto pFrame = std::make_shared<Frame>(Frame::factory_id);
    ++Frame::factory_id;
    pFrame->grid_property_.setup(image);
    auto num_keypoints = pFrame->extractKeyPoints(image, image_with_keypoints);
    //pFrame->assignFeaturePointToGrid();
    //pFrame->computeDescriptors(image);
    pFrame->image_ = image.clone();

    Frame::ptr_last_frame = Frame::ptr_current_frame;
    //Frame::ptr_current_frame = pFrame;
    //if(Frame::factory_id == 0)
    if(Frame::vo_state == NO_IMAGE)
    {
      if(num_keypoints >= 100)
      {
        Frame::ptr_initialized_frame = pFrame;
        //Frame::ptr_last_frame = Frame::ptr_current_frame;
        Frame::ptr_last_frame = nullptr;
        Frame::ptr_current_frame = pFrame;
        Frame::vo_state = NO_INITIALIZATION;
        return pFrame;
      }
      else
      {
        Frame::ptr_initialized_frame = nullptr;
        Frame::ptr_last_frame = nullptr;
        Frame::ptr_current_frame = nullptr;
        Frame::vo_state = NO_IMAGE;
        return nullptr;
      }
    }
    else if(Frame::vo_state == NO_INITIALIZATION)
    //if(Frame::factory_id > 0 && Frame::ptr_last_frame != nullptr)
    {
      //matchDescriptor(Frame::ptr_last_frame->descriptors_,
      //                Frame::ptr_current_frame->descriptors_);
      // TODO: (aliben.develop@gmail.com)
      // Accept a frame gap from the last to init the last frame,
      // but the biggest diff id num is 2, this could be better.
      if(num_keypoints < 100)
      {
        Frame::ptr_initialized_frame = nullptr;
        Frame::ptr_last_frame = nullptr;
        Frame::ptr_current_frame = nullptr;
        Frame::vo_state = NO_IMAGE;
        return nullptr;
      }
      //auto num_matches = MatchDescriptor(Frame::ptr_initialized_frame,
      //                                   pFrame);
      auto num_matches = pFrame->ptr_orb_matcher_init_advanced->SearchForInitialization(ptr_initialized_frame, pFrame, 100);
      auto all_matches = ptr_initialized_frame->best_matches_;
//      ptr_initialized_frame->best_matches_.clear();
//      for(auto& match: all_matches)
//      {
//        if(match.distance > 50)
//        {
//          AK_DLOG_ERROR << "*****************WOHOWHOWHO";
//          continue;
//        }
//        else
//        {
//          ptr_initialized_frame->best_matches_.push_back(match);
//        }
//      }
      AK_DLOG_ERROR << "Matched: " << num_matches;
//      AK_DLOG_ERROR << "Best Matched: " << ptr_initialized_frame->best_matches_.size();
      if(Frame::enable_show == true)
      {
        Frame::ShowMatches(Frame::ptr_initialized_frame, pFrame);
      }
      if(num_matches < 100)
      {
        AK_DLOG_ERROR << "Matches ≤ 100, Restarting Init.";
        //Frame::ptr_initialized_frame = pFrame;
        //Frame::ptr_last_frame = Frame::ptr_initialized_frame;
        Frame::ptr_initialized_frame = nullptr;
        Frame::ptr_last_frame = nullptr;
        Frame::ptr_current_frame = nullptr;
        Frame::vo_state = NO_IMAGE;
        //Frame::vo_state = NO_INITIALIZATION;
        return nullptr;
      }
      else
      {
        Frame::ptr_last_frame = Frame::ptr_initialized_frame;
        Frame::ptr_current_frame = pFrame;
        cv::Mat R21, t21;

        auto isInit = InitializeVO(Frame::ptr_initialized_frame,
                                   Frame::ptr_current_frame,
                                   R21,
                                   t21);
        //static int i = 0;
        //++i;
        //if(i == 2)
        //{
        //  exit(0);
        //}
        if(isInit == true)
        {
          Frame::vo_state = INITIALIZED;
          Frame::ptr_initialized_frame->transform_camera_at_world_ = cv::Mat::eye(4, 4, CV_32F);
          cv::Mat transform_current_at_initialized = cv::Mat::eye(4, 4, CV_32F);
          R21.copyTo(transform_current_at_initialized.rowRange(0, 3).colRange(0, 3));
          t21.copyTo(transform_current_at_initialized.rowRange(0, 3).col(3));
          // Set pose for current frame
          Frame::ptr_current_frame->transform_camera_at_world_ = transform_current_at_initialized;
          AK_DLOG_INFO << "Tcw:\n" << transform_current_at_initialized;
        }
        else
        {
          Frame::vo_state = NO_IMAGE;
        }
      }
    }
    else if(Frame::vo_state == INITIALIZED)
    {
      AK_LOG_INFO << "Initialization Done!!";
      exit(0);
      Frame::keyframes_vector.push_back(pFrame);
      Frame::hash_keyframes.insert(std::make_pair(factory_id, pFrame));
      Frame::frames_vector.push_back(pFrame);
      Frame::hash_frames.insert(std::make_pair(factory_id, pFrame));
    }
    //AK_LOG_INFO << "Size of frames: " << Frame::frames_vector.size();
    //AK_LOG_INFO << "Size of hash_frames: " << Frame::hash_frames.size();
    //AK_LOG_INFO << "Size of keyframes: " << Frame::keyframes_vector.size();
    //AK_LOG_INFO << "Size of hash_keyframes: " << Frame::hash_keyframes.size();
    AK_LOG_INFO << "FrameID: " << Frame::factory_id;
    return pFrame;
  }

  size_t Frame::extractKeyPoints(const cv::Mat& image,
                                 cv::Mat& img_with_keypoints)
  {
    //this->ptr_orb_->detect(image, this->keypoints_);
    if(Frame::vo_state == NO_IMAGE || Frame::vo_state == NO_INITIALIZATION)
    {
      (*Frame::ptr_orb_extractor_init_advanced)(image,
                                                cv::Mat(),
                                                this->keypoints_,
                                                this->descriptors_);
    }
    else
    {
      (*Frame::ptr_orb_extractor_advanced)(image,
                                           cv::Mat(),
                                           this->keypoints_,
                                           this->descriptors_);
    }
    //this->drawKeyPoints(image, img_with_keypoints);
    this->assignFeaturePointToGrid();
    return this->keypoints_.size();
  }

  int Frame::computeDescriptors(const cv::Mat& image)
  {
    //this->ptr_orb_->compute(image,
    //                        this->keypoints_,
    //                        this->descriptors_);
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

  size_t Frame::MatchDescriptor(const cv::Mat& previous_descriptors,
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
    Frame::ptr_last_frame->best_matches_.clear();
    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_distance*Frame::match_ratio_, 30.0))
      {
        Frame::ptr_last_frame->best_matches_.push_back(m);
      }
    }
    return Frame::ptr_last_frame->best_matches_.size();
  }

  size_t Frame::MatchDescriptor(const Frame::Ptr& ptr_last_keyframe,
                             const Frame::Ptr& ptr_current_frame)
  {
    std::vector<cv::DMatch> matches;
    matcher->match(ptr_last_keyframe->descriptors_,
                   ptr_current_frame->descriptors_,
                   matches);
    float min_distance = std::min_element(matches.begin(),
                                          matches.end(),
                                          [](const cv::DMatch& m1, const cv::DMatch& m2)
    {
      return m1.distance < m2.distance;
    })->distance;
    ptr_last_keyframe->best_matches_.clear();
    for(cv::DMatch& m : matches)
    {
      if(m.distance < std::max<float>(min_distance*Frame::match_ratio_, 30.0))
      {
        ptr_last_keyframe->best_matches_.push_back(m);
      }
    }
    return ptr_last_keyframe->best_matches_.size();
  }

  int Frame::ShowMatches(const cv::Mat& last_image,
                         const cv::Mat& current_image,
                         const std::string& window_name)
  {
    if(Frame::ptr_last_frame != nullptr && Frame::ptr_current_frame != nullptr)
    {
      cv::Mat matched_image;
      cv::drawMatches(last_image,
                      Frame::ptr_last_frame->keypoints_,
                      current_image,
                      Frame::ptr_current_frame->keypoints_,
                      Frame::ptr_last_frame->best_matches_,
                      matched_image);
      AK_DLOG_ERROR << "Show - Matches: " << Frame::ptr_last_frame->best_matches_.size();
                     //<< "\n last_id: " << Frame::ptr_last_frame->id_
                     //<< "\t cur_id: " << Frame::ptr_current_frame->id_;
      cv::imshow(window_name, matched_image);
      cv::waitKey(30);
    }
    return 0;
  }

  int Frame::ShowMatches(const cv::Mat& last_image,
                         const std::string& window_name)
  {
    if(Frame::ptr_last_keyframe != nullptr && Frame::ptr_current_frame != nullptr)
    {
      cv::Mat matched_image;
      cv::drawMatches(last_keyframe_image,
                      Frame::ptr_last_keyframe->keypoints_,
                      last_image,
                      Frame::ptr_current_frame->keypoints_,
                      Frame::ptr_last_keyframe->best_matches_,
                      matched_image);
      cv::imshow(window_name, matched_image);
      cv::waitKey(30);
    }
    return 0;
  }

  int Frame::ShowMatches(const Frame::Ptr& ptr_last_frame,
                         const Frame::Ptr& ptr_current_frame,
                         const std::string& window_name)
  {
    if(ptr_last_frame != nullptr && ptr_current_frame != nullptr)
    {
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


  bool Frame::InitializeVO(const Frame::Ptr& ptr_initialized_frame,
                           const Frame::Ptr& ptr_current_frame,
                           cv::Mat& Rcw,
                           cv::Mat& tcw)
  {
    //if(ptr_current_frame->id_ == 1)
    //{
      setRandomSeed(0);
    //}
    MATCHED_POINTS_SET = std::vector<std::vector<cv::DMatch>>(MAX_ITERATION, std::vector<cv::DMatch>(8, cv::DMatch()));
    for(size_t i=0; i<MAX_ITERATION; ++i)
    {
      //AK_DLOG_INFO << "Iteration: " << i;
      auto matches = ptr_initialized_frame->best_matches_;
      for(size_t j=0; j<8; ++j)
      {
        auto random_index_point_pair = randomInt(0, matches.size() - 1);
        auto index_matched = matches[random_index_point_pair];
        MATCHED_POINTS_SET[i][j] = index_matched;
        matches[random_index_point_pair] = matches.back();
        matches.pop_back();
        //AK_DLOG_WARNING << "Num of point: " << j;
      }
    }
    cv::Mat matrix_H21, matrix_F21;
    std::vector<cv::DMatch> inliers_H, inliers_F;
    TIMER_START("FindHomographey");
    auto score_H = FindHomography(inliers_H, matrix_H21);
    TIMER_END();
    TIMER_START("FindFundamental");
    auto score_F = FindFundamental(inliers_F, matrix_F21);
    TIMER_END();
    //std::thread threadHomoCompute(Frame::FindHomography,
    //                              std::ref(inliers_H),
    //                              std::ref(matrix_H21));
    //std::thread threadFundCompute(Frame::FindFundamental,
    //                              std::ref(inliers_F),
    //                              std::ref(matrix_F21));
    //threadHomoCompute.join();
    //threadFundCompute.join();
//    std::future<float> future_scoreH = std::async(Frame::FindHomography,
//                                                   std::ref(inliers_H),
//                                                   std::ref(matrix_H21));
//    std::future<float> future_scoreF = std::async(Frame::FindFundamental,
//                                                   std::ref(inliers_F),
//                                                   std::ref(matrix_F21));
    AK_DLOG_INFO << "Initializing VO...";
//    float score_H = future_scoreH.get();
//    float score_F = future_scoreF.get();
    //float score_F = Frame::FindFundamental(inliers_F, matrix_F21);
    float ratio_homography = score_H / (score_H + score_F);
    AK_DLOG_INFO << "inliers_H : " << inliers_H.size();
    AK_DLOG_INFO << "inliers_F : " << inliers_F.size();
    AK_DLOG_INFO << "ratio_homography: " << ratio_homography;
    if(ratio_homography > 0.40)
    {
      AK_DLOG_WARNING << "Reconstruct From Homography.";
      return ReconstructFromHomo(inliers_H,
                                 matrix_H21,
                                 Frame::K,
                                 Rcw,
                                 tcw,
                                 Frame::init_landmarks,
                                 1.0,
                                 50);
    }
    else
    {
      AK_DLOG_WARNING << "Reconstruct From Fundamental.";
      return ReconstructFromFund(inliers_F,
                                 matrix_F21,
                                 Frame::K,
                                 Rcw,
                                 tcw,
                                 Frame::init_landmarks,
                                 1.0,
                                 50);
    }
    return false;
  }

  float Frame::FindHomography(std::vector<cv::DMatch>& matched_inliers, cv::Mat& H21)
  {
    AK_DLOG_INFO << "Find homography...";
    std::vector<cv::KeyPoint> normalized_kps_init, normalized_kps_cur;
    cv::Mat transform_init, transform_cur;
    // TODO: (aliben.develop@gmail.com)
    // NormalizeKeyPoints should be optimized, cause in all iteration keypoints is the same set. It don't have to calculate each time to find homo/fund matrix.
    NormalizeKeyPoints(Frame::ptr_initialized_frame->keypoints_,
                       normalized_kps_init,
                       transform_init);
//    AK_DLOG_INFO << "Mat1_homo: \n" << transform_init;
    NormalizeKeyPoints(Frame::ptr_current_frame->keypoints_,
                       normalized_kps_cur,
                       transform_cur);
//    AK_DLOG_INFO << "Mat2_homo: \n" << transform_cur;

    cv::Mat transform_cur_transpose = transform_cur.t();
    float score = 0.0;

    std::vector<cv::KeyPoint> ransec_init_keypoints(8);
    std::vector<cv::KeyPoint> ransec_cur_keypoints(8);

    cv::Mat H21_tmp, H12_tmp;
    float current_score;
    for(size_t iter=0; iter<MAX_ITERATION; ++iter)
    {
      for(auto n=0; n<8; ++n)
      {
        auto queryIdx = MATCHED_POINTS_SET[iter][n].queryIdx;
        auto trainIdx = MATCHED_POINTS_SET[iter][n].trainIdx;
        ransec_init_keypoints[n] = normalized_kps_init[queryIdx];
        ransec_cur_keypoints[n] = normalized_kps_cur[trainIdx];
      }

//      //============================================================
//      // Insection Test
//        std::vector<cv::KeyPoint> keypoints_init;
//        std::vector<cv::KeyPoint> keypoints_cur;
//        std::vector<cv::KeyPoint> keypoints_init_norm;
//        std::vector<cv::KeyPoint> keypoints_cur_norm;
//        std::ifstream file_kps1("../data/keys1.txt");
//        std::ifstream file_kps2("../data/keys2.txt");
//        int kps1_count;
//        int kps2_count;
//        file_kps1 >> kps1_count;
//        file_kps2 >> kps2_count;
//        float x, y, p_size, angle, response, x_norm, y_norm, elem_mat;
//        for(int i=0; i<kps1_count; ++i)
//        {
//          ReadPlainData(x, file_kps1);
//          ReadPlainData(y, file_kps1);
//          ReadPlainData(p_size, file_kps1);
//          ReadPlainData(angle, file_kps1);
//          ReadPlainData(response, file_kps1);
//          keypoints_init.push_back(cv::KeyPoint(x, y, p_size, angle, response));
//          ReadPlainData(x_norm, file_kps1);
//          ReadPlainData(y_norm, file_kps1);
//          ReadPlainData(p_size, file_kps1);
//          keypoints_init_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
//        }
//        std::vector<float> pre_mat1;
//
//        for(int i=0; i<9; ++i)
//        {
//          ReadPlainData(elem_mat, file_kps1);
//          pre_mat1.push_back(elem_mat);
//        }
//        cv::Mat temp_pre1 = cv::Mat(pre_mat1, CV_32F);
//        cv::Mat t1 = temp_pre1.reshape(3,3).clone();
//
//        for(int i=0; i<kps2_count; ++i)
//        {
//          ReadPlainData(x, file_kps2);
//          ReadPlainData(y, file_kps2);
//          ReadPlainData(p_size, file_kps2);
//          ReadPlainData(angle, file_kps2);
//          ReadPlainData(response, file_kps2);
//          keypoints_cur.push_back(cv::KeyPoint(x, y, p_size, angle, response));
//          ReadPlainData(x_norm, file_kps2);
//          ReadPlainData(y_norm, file_kps2);
//          ReadPlainData(p_size, file_kps2);
//          keypoints_cur_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
//        }
//        std::vector<float> pre_mat2;
//        for(int i=0; i<9; ++i)
//        {
//          ReadPlainData(elem_mat, file_kps2);
//          pre_mat2.push_back(elem_mat);
//        }
//        cv::Mat temp_pre2 = cv::Mat(pre_mat2, CV_32F);
//        cv::Mat t2 = temp_pre2.reshape(3,3).clone();
//        file_kps1.close();
//        file_kps2.close();
//        //===========
//        std::ifstream file_ransec;
//        file_ransec.open("../data/ransec_set.txt");
//        //std::vector<cv::KeyPoint> kps_init_normalized;
//        //std::vector<cv::KeyPoint> kps_cur_normalized;
//        int iteration, num_ransec, iteration_all;
//        //file_ransec >> iteration_all;
//        ReadPlainData(iteration_all, file_ransec);
//        //std::cout << "All_iteration: " << iteration_all << std::endl;
//
//        std::vector<float> pre_mat;
//        int init_idx, cur_idx;
//        std::vector<cv::KeyPoint> kps_norm_ransec_init(8);
//        std::vector<cv::KeyPoint> kps_norm_ransec_cur(8);
////        for(int iter=0; iter<iteration_all; ++iter)
////        {
//          for(int j=0; j<8; ++j)
//          {
//            ReadPlainData(iteration, file_ransec);
//            ReadPlainData(num_ransec, file_ransec);
//            ReadPlainData(init_idx, file_ransec);
//            ReadPlainData(cur_idx, file_ransec);
//            kps_norm_ransec_init[j] = keypoints_init_norm[init_idx];
//            kps_norm_ransec_cur[j] = keypoints_cur_norm[cur_idx];
//          }
//          pre_mat.clear();
//          for(int i=0; i<9; ++i)
//          {
//            ReadPlainData(elem_mat, file_ransec);
//            pre_mat.push_back(elem_mat);
//          }
//          cv::Mat temp1 = cv::Mat(pre_mat, CV_32F);
//          cv::Mat H21_read = temp1.reshape(3,3).clone();
//          AK_DLOG_INFO << "H21_read:\n" << H21_read;
//          NormalizeKeyPoints(keypoints_init,
//                             keypoints_init_norm,
//                             t1);
//          NormalizeKeyPoints(keypoints_cur,
//                             keypoints_cur_norm,
//                             t2);
//      // }
//        //============================================================
      // ComputeH21
      cv::Mat H21_normalized = ComputeH21(ransec_init_keypoints, ransec_cur_keypoints);
      H21_tmp = transform_cur.inv() * H21_normalized * transform_init;
      H12_tmp = H21_tmp.inv();

      //      AK_DLOG_INFO << "Compute_H21: \n" << H21_normalized;
//      cv::Mat H21_normalized = ComputeH21(kps_norm_ransec_init, kps_norm_ransec_cur);
//      H21_tmp = t2.inv() * H21_normalized * t1;
//      H12_tmp = H21_tmp.inv();

//      AK_DLOG_INFO << "t2:\n" << t2;
//      AK_DLOG_INFO << "t2_inv:\n" << t2.inv();
//      AK_DLOG_INFO << "t1:\n" << t1;

//      AK_DLOG_INFO << "H21_" << iter << ":\n" << H21_tmp;
//      AK_DLOG_INFO << "H12_" << iter << ":\n" << H12_tmp;
      // CheckH21 - sigma = 1.0
      current_score = CheckHomography(H21_tmp, H12_tmp, matched_inliers, 1.0);
      if( current_score > score )
      {
        H21 = H21_tmp.clone();
        score = current_score;
      }
    }
    return score;
  }

  float Frame::FindFundamental(std::vector<cv::DMatch>& matched_inliers, cv::Mat& F21)
  {
    AK_DLOG_INFO << "Find fundamental ...";
    std::vector<cv::KeyPoint> normalized_kps_init, normalized_kps_cur;
    cv::Mat transform_init, transform_cur;
    // TODO: (aliben.develop@gmail.com)
    // NormalizeKeyPoints should be optimized, cause in all iteration keypoints is the same set. It don't have to calculate each time to find homo/fund matrix.
    NormalizeKeyPoints(Frame::ptr_initialized_frame->keypoints_,
                       normalized_kps_init,
                       transform_init);
    NormalizeKeyPoints(Frame::ptr_current_frame->keypoints_,
                       normalized_kps_cur,
                       transform_cur);
    cv::Mat transform_cur_transpose = transform_cur.t();
    float score = 0.0;

    std::vector<cv::KeyPoint> ransec_init_keypoints(8);
    std::vector<cv::KeyPoint> ransec_cur_keypoints(8);

    cv::Mat F21_tmp;
    float current_score;
    std::vector<cv::DMatch> ransec_matched_inliers;
    for(size_t iter=0; iter<MAX_ITERATION; ++iter)
    {
      for(auto n=0; n<8; ++n)
      {
        ransec_init_keypoints[n] = normalized_kps_init[MATCHED_POINTS_SET[iter][n].queryIdx];
        ransec_cur_keypoints[n] = normalized_kps_cur[MATCHED_POINTS_SET[iter][n].trainIdx];
      }
      // ComputeF21
      cv::Mat F21_normalized = ComputeF21(ransec_init_keypoints, ransec_cur_keypoints);
      F21_tmp = transform_cur_transpose * F21_normalized * transform_init;
      // CheckF21 - sigma = 1.0
      current_score = CheckFundamental(F21_tmp, ransec_matched_inliers, 1.0);
      if( current_score > score )
      {
        F21 = F21_tmp.clone();
        score = current_score;
        matched_inliers = ransec_matched_inliers;
      }
    }
    return score;
  }

  void Frame::NormalizeKeyPoints(const std::vector<cv::KeyPoint>& kps,
                                 std::vector<cv::KeyPoint>& kps_normalized,
                                 cv::Mat& Transform)
  {
    float mean_x = 0.0;
    float mean_y = 0.0;
    const size_t KEYPOINT_SIZE = kps.size();
    kps_normalized.resize(KEYPOINT_SIZE);
    for(auto& kp: kps)
    {
      mean_x += kp.pt.x;
      mean_y += kp.pt.y;
    }
    mean_x /= KEYPOINT_SIZE;
    mean_y /= KEYPOINT_SIZE;
    float deviation_x = 0.0;
    float deviation_y = 0.0;
    size_t i = 0;
    for(auto& kp: kps)
    {
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
    for(auto& kp_normal: kps_normalized)
    {
      kp_normal.pt.x *= deviation_x_inv;
      kp_normal.pt.y *= deviation_y_inv;
    }
    Transform = cv::Mat::eye(3, 3, CV_32F);
    Transform.at<float>(0, 0) = deviation_x_inv;
    Transform.at<float>(1, 1) = deviation_y_inv;
    Transform.at<float>(0, 2) = -mean_x*deviation_x_inv;
    Transform.at<float>(1, 2) = -mean_y*deviation_y_inv;
  }

  cv::Mat Frame::ComputeH21(const std::vector<cv::KeyPoint>& keypoints_init,
                            const std::vector<cv::KeyPoint>& keypoints_cur)
  {
    const int N = keypoints_init.size();
    cv::Mat A(2*N, 9, CV_32F);
    for(int i=0; i<N; i++)
    {
        const float u1 = keypoints_init[i].pt.x;
        const float v1 = keypoints_init[i].pt.y;
        const float u2 = keypoints_cur[i].pt.x;
        const float v2 = keypoints_cur[i].pt.y;
        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;
    }
    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    return vt.row(8).reshape(0, 3);
  }

  cv::Mat Frame::ComputeF21(const std::vector<cv::KeyPoint>& keypoints_init,
                            const std::vector<cv::KeyPoint>& keypoints_cur)
  {
    const int N = keypoints_init.size();
    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = keypoints_init[i].pt.x;
        const float v1 = keypoints_init[i].pt.y;
        const float u2 = keypoints_cur[i].pt.x;
        const float v2 = keypoints_cur[i].pt.y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat Fpre = vt.row(8).reshape(0, 3);
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    w.at<float>(2)=0;
    return  u*cv::Mat::diag(w)*vt;
  }

  float Frame::CheckHomography(const cv::Mat& H21,
                                const cv::Mat& H12,
                                std::vector<cv::DMatch>& matched_inliers,
                                float sigma)
  {
    const size_t N = Frame::ptr_initialized_frame->best_matches_.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    float score = 0;
    const float th = 5.991;
    const float inv_sigma_square = 1.0 / (sigma * sigma);
    for(size_t i=0; i<N; ++i)
    {
      bool isInliers = true;
      auto index_kp_init = Frame::ptr_initialized_frame->best_matches_[i].queryIdx;
      auto index_kp_cur  = Frame::ptr_initialized_frame->best_matches_[i].trainIdx;
      const auto& kp_init = Frame::ptr_initialized_frame->keypoints_[index_kp_init];
      const auto& kp_cur = Frame::ptr_current_frame->keypoints_[index_kp_cur];

      const float u1 = kp_init.pt.x;
      const float v1 = kp_init.pt.y;
      const float u2 = kp_cur.pt.x;
      const float v2 = kp_cur.pt.y;
      const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
      const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
      const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;
      const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);
      const float chiSquare1 = squareDist1*inv_sigma_square;

      if(chiSquare1>th)
      {
        isInliers = false;
      }
      else
      {
        score += th - chiSquare1;
      }

      // Reprojection error in second image
      // x1in2 = H21*x1

      const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
      const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
      const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;
      const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
      const float chiSquare2 = squareDist2*inv_sigma_square;

      if(chiSquare2>th)
      {
        isInliers = false;
      }
      else
      {
        score += th - chiSquare2;
      }

      if(isInliers)
      {
        //AK_DLOG_ERROR << "One inliers captured.";
        matched_inliers.push_back(Frame::ptr_initialized_frame->best_matches_[i]);
      }
    }
    //AK_DLOG_ERROR << "H-inliers.size = " << matched_inliers.size();
    return score;
  }

  float Frame::CheckFundamental(const cv::Mat& F21,
                                 std::vector<cv::DMatch>& matched_inliers,
                                 float sigma)
  {
    matched_inliers.clear();
    const int N = Frame::ptr_initialized_frame->best_matches_.size();
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    float score = 0;
    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
      bool isInliers = true;

      auto index_kp_init = Frame::ptr_initialized_frame->best_matches_[i].queryIdx;
      auto index_kp_cur  = Frame::ptr_initialized_frame->best_matches_[i].trainIdx;
      const auto& kp_init = Frame::ptr_initialized_frame->keypoints_[index_kp_init];
      const auto& kp_cur = Frame::ptr_current_frame->keypoints_[index_kp_cur];

      const float u1 = kp_init.pt.x;
      const float v1 = kp_init.pt.y;
      const float u2 = kp_cur.pt.x;
      const float v2 = kp_cur.pt.y;

      // Reprojection error in second image
      // l2=F21x1=(a2,b2,c2)

      const float a2 = f11*u1+f12*v1+f13;
      const float b2 = f21*u1+f22*v1+f23;
      const float c2 = f31*u1+f32*v1+f33;

      const float num2 = a2*u2+b2*v2+c2;
      const float squareDist1 = num2*num2/(a2*a2+b2*b2);
      const float chiSquare1 = squareDist1*invSigmaSquare;

      if(chiSquare1>th)
      {
        isInliers = false;
      }
      else
      {
        score += thScore - chiSquare1;
      }

      // Reprojection error in second image
      // l1 =x2tF21=(a1,b1,c1)
      const float a1 = f11*u2+f21*v2+f31;
      const float b1 = f12*u2+f22*v2+f32;
      const float c1 = f13*u2+f23*v2+f33;

      const float num1 = a1*u1+b1*v1+c1;
      const float squareDist2 = num1*num1/(a1*a1+b1*b1);
      const float chiSquare2 = squareDist2*invSigmaSquare;

      if(chiSquare2>th)
      {
        isInliers = false;
      }
      else
      {
        score += thScore - chiSquare2;
      }

      if(isInliers)
      {
        //AK_DLOG_ERROR << "One inliers captured.";
        matched_inliers.push_back(Frame::ptr_initialized_frame->best_matches_[i]);
//        AK_DLOG_WARNING << matched_inliers.size();
      }
    }
    //AK_DLOG_ERROR << "F-inliers.size = " << matched_inliers.size();
    return score;
  }

  bool Frame::ReconstructFromHomo(std::vector<cv::DMatch>& matched_inliers,
                                  cv::Mat& H21,
                                  cv::Mat& K,
                                  cv::Mat& R21,
                                  cv::Mat& t21,
                                  std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                                  float min_parallax,
                                  int min_triangulated)
  {
    //AK_DLOG_WARNING << "Reconstructing Homography";
    //AK_DLOG_WARNING << "K:\n" << K;
    cv::Mat K_inv = K.inv();
    //AK_DLOG_WARNING << "K_inv:\n" << K_inv;
    cv::Mat A = K_inv * H21 * K;

    cv::Mat U, w, Vt, V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
    V = Vt.t();

    float s = cv::determinant(U) * cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
      return false;
    }

    std::vector<cv::Mat> vector_R, vector_t, vector_n;
    vector_R.reserve(8);
    vector_t.reserve(8);
    vector_n.reserve(8);

    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vector_R.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vector_t.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vector_n.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vector_R.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vector_t.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vector_n.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;

    std::vector<std::pair<size_t, cv::Point3f>> bestP3D;
    std::vector<std::pair<size_t, cv::Point3f>> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
      float parallaxi{0.0};
      std::vector<std::pair<size_t, cv::Point3f>> vP3Di;
      //std::vector<std::pair<size_t, cv::Point3f>> vbTriangulatedi;
      //int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);
      int nGood = Frame::CheckRT(vector_R[i],
                                 vector_t[i],
                                 Frame::ptr_initialized_frame->keypoints_,
                                 Frame::ptr_current_frame->keypoints_,
                                 matched_inliers,
                                 K,
                                 vP3Di,
                                 4.0*Frame::sigma*Frame::sigma,
                                 parallaxi);

      if(nGood>bestGood)
      {
        secondBestGood = bestGood;
        bestGood = nGood;
        bestSolutionIdx = i;
        bestParallax = parallaxi;
        bestP3D = vP3Di;
        //bestTriangulated = vbTriangulatedi;
      }
      else if(nGood>secondBestGood)
      {
        secondBestGood = nGood;
      }
    }

    auto N = matched_inliers.size();

    if(secondBestGood<0.75*bestGood && bestParallax>=min_parallax && bestGood>min_triangulated && bestGood>0.9*N)
    {
      vector_R[bestSolutionIdx].copyTo(R21);
      vector_t[bestSolutionIdx].copyTo(t21);
      init_landmarks = bestP3D;
      //vbTriangulated = bestTriangulated;
      return true;
    }
    return false;
  }

  bool Frame::ReconstructFromFund(std::vector<cv::DMatch>& matched_inliers,
                                  cv::Mat& F21,
                                  cv::Mat& K,
                                  cv::Mat& R21,
                                  cv::Mat& t21,
                                  std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                                  float min_parallax,
                                  int min_triangulated)
  {
    //AK_DLOG_WARNING << "Reconstructing Fundamental";

    // Compute Essential Matrix from Fundamental Matrix
//    std::cout << "K: \n" << K << std::endl;
//    std::cout << "F21: \n" << F21 << std::endl;

    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);


    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    //vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    std::vector<std::pair<size_t, cv::Point3f>> bestP3D;
    std::vector<std::pair<size_t, cv::Point3f>> vP3D1, vP3D2, vP3D3, vP3D4;
    std::vector<std::pair<size_t, cv::Point3f>> bestTriangulated1,bestTriangulated2,bestTriangulated3, bestTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    //int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    //int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    //int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    //int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int nGood1 = CheckRT(R1,
                         t1,
                         Frame::ptr_initialized_frame->keypoints_,
                         Frame::ptr_current_frame->keypoints_,
                         matched_inliers,
                         K,
                         vP3D1,
                         4.0*Frame::sigma*Frame::sigma,
                         parallax1);
    int nGood2 = CheckRT(R2,
                         t1,
                         Frame::ptr_initialized_frame->keypoints_,
                         Frame::ptr_current_frame->keypoints_,
                         matched_inliers,
                         K,
                         vP3D2,
                         4.0*Frame::sigma*Frame::sigma,
                         parallax2);
    int nGood3 = CheckRT(R1,
                         t2,
                         Frame::ptr_initialized_frame->keypoints_,
                         Frame::ptr_current_frame->keypoints_,
                         matched_inliers,
                         K,
                         vP3D3,
                         4.0*Frame::sigma*Frame::sigma,
                         parallax3);
    int nGood4 = CheckRT(R2,
                         t2,
                         Frame::ptr_initialized_frame->keypoints_,
                         Frame::ptr_current_frame->keypoints_,
                         matched_inliers,
                         K,
                         vP3D4,
                         4.0*Frame::sigma*Frame::sigma,
                         parallax4);

    int maxGood = std::max(nGood1,std::max(nGood2,std::max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    auto N = matched_inliers.size();
    int nMinGood = std::max(static_cast<int>(0.9*N),min_triangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
      nsimilar++;
    if(nGood2>0.7*maxGood)
      nsimilar++;
    if(nGood3>0.7*maxGood)
      nsimilar++;
    if(nGood4>0.7*maxGood)
      nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
      AK_DLOG_ERROR << "Triangulated is too less.";
      return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
      if(parallax1>min_parallax)
      {
        bestP3D = vP3D1;
        init_landmarks = bestTriangulated1;
        //vbTriangulated = vbTriangulated1;
        R1.copyTo(R21);
        t1.copyTo(t21);
        return true;
      }
    }
    else if(maxGood==nGood2)
    {
      if(parallax2>min_parallax)
      {
        bestP3D = vP3D2;
        //vbTriangulated = vbTriangulated2;
        init_landmarks = bestTriangulated2;
        R2.copyTo(R21);
        t1.copyTo(t21);
        return true;
      }
    }
    else if(maxGood==nGood3)
    {
      if(parallax3>min_parallax)
      {
        bestP3D = vP3D3;
        //vbTriangulated = vbTriangulated3;
        init_landmarks = bestTriangulated3;
        R1.copyTo(R21);
        t2.copyTo(t21);
        return true;
      }
    }
    else if(maxGood==nGood4)
    {
      if(parallax4>min_parallax)
      {
        bestP3D = vP3D4;
        //vbTriangulated = vbTriangulated4;
        init_landmarks = bestTriangulated4;
        R2.copyTo(R21);
        t2.copyTo(t21);
        return true;
      }
    }
    return false;
  }

  int Frame::CheckRT(const cv::Mat& R,
                     const cv::Mat& t,
                     const std::vector<cv::KeyPoint>& keypoints_last,
                     const std::vector<cv::KeyPoint>& keypoints_cur,
                     const std::vector<cv::DMatch>& matched_inliers,
                     const cv::Mat& K,
                     std::vector<std::pair<size_t, cv::Point3f>>& init_landmarks,
                     float th2,
                     float& parallax
                     )
  {
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    //vbGood = vector<bool>(vKeys1.size(),false);
    //vP3D.resize(vKeys1.size()); // Just insert pair

    std::vector<float> vCosParallax;
    vCosParallax.reserve(matched_inliers.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    //auto triangulated_matches = matched_inliers;
    std::vector<std::pair<size_t, cv::Point3f>> triangulated_matches;

    size_t match_size = matched_inliers.size();
    for(size_t i=0; i<match_size; ++i)
    {
      //if(!vbMatchesInliers[i])
      //    continue;

      const cv::KeyPoint &kp1 = keypoints_last[matched_inliers[i].queryIdx];
      const cv::KeyPoint &kp2 = keypoints_cur[matched_inliers[i].trainIdx];
      //cv::Mat p3dC1;
      cv::Mat p3dC1 = cv::Mat(3, 1, CV_32F);

//      TIMER_START("ReconFund_Triangulate");
      Triangulate(kp1,kp2,P1,P2,p3dC1);
//      TIMER_END();
      if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
      {
          //vbGood[vMatches12[i].first]=false;
          // Remove this match OR just ignore it
          continue;
      }

      // Check parallax
      cv::Mat normal1 = p3dC1 - O1;
      float dist1 = cv::norm(normal1);

      cv::Mat normal2 = p3dC1 - O2;
      float dist2 = cv::norm(normal2);

      float cosParallax = normal1.dot(normal2)/(dist1*dist2);

      // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
        continue;

      // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      cv::Mat p3dC2 = R*p3dC1+t;

      if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
        continue;

      // Check reprojection error in first image
      float im1x, im1y;
      float invZ1 = 1.0/p3dC1.at<float>(2);
      im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
      im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

      float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

      if(squareError1>th2)
        continue;

      // Check reprojection error in second image
      float im2x, im2y;
      float invZ2 = 1.0/p3dC2.at<float>(2);
      im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
      im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

      float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

      if(squareError2>th2)
        continue;

      vCosParallax.push_back(cosParallax);
      //vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
      auto landmark = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
      init_landmarks.push_back(std::make_pair(matched_inliers[i].queryIdx, landmark));
      
      nGood++;

      if(cosParallax<0.99998)
        //vbGood[vMatches12[i].first]=true;
        //enable i as good keypoints;
        ;
    }


    if(nGood>0)
    {
      sort(vCosParallax.begin(),vCosParallax.end());
      size_t idx = std::min(50,int(vCosParallax.size()-1));
      parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
    {
      parallax=0;
    }
    return nGood;
  }

  void Frame::Triangulate(const cv::KeyPoint& kp_init,
                          const cv::KeyPoint& kp_cur,
                          const cv::Mat& pose_init,
                          const cv::Mat& pose_cur,
                          cv::Mat& x3D)
  {
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp_init.pt.x*pose_init.row(2)-pose_init.row(0);
    A.row(1) = kp_init.pt.y*pose_init.row(2)-pose_init.row(1);
    A.row(2) = kp_cur.pt.x*pose_cur.row(2)-pose_cur.row(0);
    A.row(3) = kp_cur.pt.y*pose_cur.row(2)-pose_cur.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
  }

  void Frame::DecomposeE(const cv::Mat& E,
                         cv::Mat& R1,
                         cv::Mat& R2,
                         cv::Mat& t)
  {
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
      R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
      R2=-R2;
  }

  void Frame::assignFeaturePointToGrid()
  {
    auto kp_size = this->keypoints_.size();
//    const size_t pre_assign_mem_size = 0.5f * kp_size / (IMAGE_COLS * IMAGE_ROWS);
    //for(size_t col=0; col<IMAGE_COLS; ++col)
    //{
    //  for(size_t row=0; row<IMAGE_ROWS; ++row)
    //  {
    //    this->keypoints_grid_[col][row].reserve(pre_assign_mem_size);
    //  }
    //}
    for(size_t idx=0; idx<kp_size; ++idx)
    {
      const cv::KeyPoint& kp = this->keypoints_[idx];
      size_t grid_pos_x, grid_pos_y;
//      cv::KeyPoint kp(832, 54, 31, 36.1879578, 143);
      if(this->getGridPosition(kp, grid_pos_x, grid_pos_y))
      {
        this->keypoints_grid_[grid_pos_x][grid_pos_y].push_back(idx);
      }
    }
  }

  bool Frame::getGridPosition(const cv::KeyPoint& kp,
                              size_t& pos_x,
                              size_t& pos_y)
  {
    pos_x = round((kp.pt.x - this->grid_property_.min_x)*this->grid_property_.grid_width_inv);
    pos_y = round((kp.pt.y - this->grid_property_.min_y)*this->grid_property_.grid_height_inv);
    if(pos_x<0 || pos_x>=IMAGE_COLS || pos_y<0 || pos_y>=IMAGE_ROWS)
    {
      return false;
    }
    return true;
  }

  std::vector<size_t> Frame::getCandidateKeypoints(const float& x,
                                                   const float& y,
                                                   const float& radium,
                                                   const int& min_pyramid_level,
                                                   const int& max_pyramid_level)
  {
    std::vector<size_t> candidate_to_match_keypoints;
    candidate_to_match_keypoints.reserve(this->keypoints_.size());

    auto min_x = this->grid_property_.min_x;
    auto min_y = this->grid_property_.min_y;
    auto max_x = this->grid_property_.max_x;
    auto max_y = this->grid_property_.max_y;
    auto grid_width_inv = this->grid_property_.grid_width_inv;
    auto grid_height_inv = this->grid_property_.grid_height_inv;

    const size_t nMinCellX = std::max(0,(int)floor((x-min_x-radium)*grid_width_inv));
    if(nMinCellX>=IMAGE_COLS)
        return candidate_to_match_keypoints;

    const size_t nMaxCellX = std::min((int)IMAGE_COLS-1,(int)ceil((x-min_x+radium)*grid_width_inv));
    if(nMaxCellX<0)
        return candidate_to_match_keypoints;

    const size_t nMinCellY = std::max(0,(int)floor((y-min_y-radium)*grid_height_inv));
    if(nMinCellY>=IMAGE_ROWS)
        return candidate_to_match_keypoints;

    const size_t nMaxCellY = std::min((int)IMAGE_ROWS-1,(int)ceil((y-min_y+radium)*grid_height_inv));
    if(nMaxCellY<0)
        return candidate_to_match_keypoints;

    const bool bCheckLevels = (min_pyramid_level>0) || (max_pyramid_level>=0);

    for(size_t ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(size_t iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = keypoints_grid_[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = this->keypoints_[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<min_pyramid_level)
                        continue;
                    if(max_pyramid_level>=0)
                        if(kpUn.octave>max_pyramid_level)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(std::abs(distx)<radium && std::abs(disty)<radium)
                    candidate_to_match_keypoints.push_back(vCell[j]);
            }
        }
    }

    return candidate_to_match_keypoints;
  }
}
