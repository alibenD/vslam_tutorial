/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: map.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-03 11:06:28
  * @last_modified_date: 2019-04-25 12:22:44
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/map.hh>
#include <visual_slam/optimizer.hh>

//CODE
namespace ak
{
  int Map::initializeMap()
  {
    return 0;
  }

  void Map::resetGlobalMap()
  {
    AK_LOG_INFO << "Resetting global map.";
    frames_vector_.clear();
    keyframes_vector_.clear();
    landmarks_vector_.clear();
    landmarks_vector_valid_.clear();

    hash_frames_.clear();
    hash_keyframes_.clear();
    hash_landmarks_.clear();
    hash_landmarks_valid_.clear();
    AK_LOG_INFO << "Global map reset done.";
  }

  void Map::resetMap()
  {
    resetLocalMap();
    resetGlobalMap();
  }

  void Map::resetLocalMap()
  {
    AK_LOG_INFO << "Resetting local map.";
    local_keyframes_.clear();
    local_landmarks_.clear();
    AK_LOG_INFO << "LocalMap Reset Done.";
  }

  void Map::addLocalQueue(const Frame::Ptr& ptr_local_current_frame)
  {
    keyframes_wait_to_local_optimized_.push_back(ptr_local_current_frame);
  }

  void Map::initLocalMap(const Frame::Ptr& ptr_local_current_frame)
  {
    resetLocalMap();

    ptr_local_current_frame->computeBOW();
    const auto& landmarks = ptr_local_current_frame->getLandmarks();

    for(const auto& landmark_pair:landmarks)
    {
      const auto& ptr_landmark = landmark_pair.second;
      auto kp_index = landmark_pair.first;
      if(ptr_landmark == nullptr)
      {
        AK_DLOG_INFO << "Nullptr Landmark in local map. Frame->"
                     << ptr_local_current_frame->getID() << " LM->"
                     << ptr_landmark->getID();
        continue;
      }
      if(ptr_landmark->isAvailable() == true)
      {
        if(ptr_landmark->isInFrame(ptr_local_current_frame) == true)
        {
          ptr_landmark->addObserver(ptr_local_current_frame, kp_index);
          ptr_landmark->updateLandmark();
        }
        else
        {
          // Happen and Only if happen trackLocalMap, trackReferenceFrame
          local_landmarks_.push_back(ptr_landmark);
        }
      }
    }

    ptr_local_current_frame->updateCovision();
    this->addKeyFrame(ptr_local_current_frame);
  }

  void Map::newTriangLandmark(Frame::Ptr &ptr_local_current_frame)
  {
    auto neighbour_keyframes = ptr_local_current_frame->getBestCovisibleFrameWith(20);
    ORBmatcher matcher(0.6, false);

    // Rotation from world to camera coor.
    auto transform_local = ptr_local_current_frame->getTF().clone();
    auto position_local = ptr_local_current_frame->getOrigin();

    auto R1w = transform_local.rowRange(0,3).colRange(0,3);
    auto t1w = transform_local.rowRange(0,3).col(3);

    const float& fx = Frame::K.at<float>(0,0);
    const float& fy = Frame::K.at<float>(1,1);
    const float& cx = Frame::K.at<float>(0,2);
    const float& cy = Frame::K.at<float>(1,2);
    const float& inv_fx = fx == 0.0f ? fx: 1.0f/fx;
    const float& inv_fy = fy == 0.0f ? fy: 1.0f/fy;
    const float ratio_factor = 1.5f * Frame::level_scale_factor;

    int num_new_landmarks = 0;

    for(auto& ptr_frame:neighbour_keyframes)
    {
      //If local_keyframes is empty, just return
      auto position_neighbour = ptr_frame->getOrigin();
      auto baseline_vec = position_neighbour - position_local;
      const float baseline = cv::norm(baseline_vec);

      // Just for monocular
      const float median_depth_neighbour = ptr_frame->computeMedianDepth(2);
      const float ratio_baseline_depth = baseline/median_depth_neighbour;
      if(ratio_baseline_depth < 0.01)
      {
        continue;
      }

      // ComputeF12 for local_frame and neigh_frame
      cv::Mat transform_neigh = ptr_frame->getTF();
      cv::Mat R2w = transform_neigh.rowRange(0,3).colRange(0,3);
      cv::Mat t2w = transform_neigh.rowRange(0,3).col(3);

      cv::Mat R12 = R1w*R2w.t();
      cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

      cv::Mat t12x = (cv::Mat_<float>(3,3) << 0, -t12.at<float>(2), t12.at<float>(1), t12.at<float>(2), 0, -t12.at<float>(0), -t12.at<float>(1), t12.at<float>(0), 0);
      cv::Mat F12 = Frame::K.t().inv()* t12x* R12*Frame::K.inv();
      // End of ComputeF12

      std::vector<cv::DMatch> matches;
      matcher.SearchForTriangulation(ptr_local_current_frame, ptr_frame, F12, matches, false);

      auto nmatches = matches.size();
      for(size_t kp_idx=0; kp_idx<nmatches; ++kp_idx)
      {
        auto real_idx1 = matches[kp_idx].queryIdx;
        auto real_idx2 = matches[kp_idx].trainIdx;

        const auto& kp1 = ptr_local_current_frame->getKeyPoints()[real_idx1];
        const auto& kp2 = ptr_frame->getKeyPoints()[real_idx2];

        cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx)*inv_fx, (kp1.pt.y-cy)*inv_fy, 1);
        cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx)*inv_fx, (kp2.pt.y-cy)*inv_fy, 1);

        cv::Mat ray1 = R1w.t() * xn1;
        cv::Mat ray2 = R2w.t() * xn2;

        const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

        cv::Mat x3D;

        if(cosParallaxRays > 0 && cosParallaxRays < 0.9998)
        {
          cv::Mat A(4,4,CV_32F);
          A.row(0) = xn1.at<float>(0)*transform_local.row(2) - transform_local.row(0);
          A.row(1) = xn1.at<float>(1)*transform_local.row(2) - transform_local.row(1);
          A.row(2) = xn1.at<float>(0)*transform_neigh.row(2) - transform_neigh.row(0);
          A.row(3) = xn1.at<float>(1)*transform_neigh.row(2) - transform_neigh.row(1);

          cv::Mat w,u,vt;
          cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

          x3D = vt.row(3).t();

          if(x3D.at<float>(3) == 0)
          {
            continue;
          }
          x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
        }
        // else steoro camera
        else
        {
          continue;
        }
        cv::Mat x3Dt = x3D.t();

        float z1 = R1w.row(2).dot(x3Dt)+t1w.at<float>(2);
        if(z1 <=0)
        {
          continue;
        }

        float z2 = R2w.row(2).dot(x3Dt)+t2w.at<float>(2);
        if(z2 <=0)
        {
          continue;
        }

        const float sigmaSquare1 = std::pow(1.0, kp1.octave);
        const float x1 = R1w.row(0).dot(x3Dt)+t1w.at<float>(0);
        const float y1 = R1w.row(1).dot(x3Dt)+t1w.at<float>(1);
        const float invz1 = 1.0/z1;

        float u1 = fx*x1*invz1+cx;
        float v1 = fy*y1*invz1+cy;
        float errX1 = u1 - kp1.pt.x;
        float errY1 = v1 - kp1.pt.y;
        if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
        {
          continue;
        }

        const float sigmaSquare2 = std::pow(1.0, kp2.octave);
        const float x2 = R2w.row(0).dot(x3Dt)+t2w.at<float>(0);
        const float y2 = R2w.row(1).dot(x3Dt)+t2w.at<float>(1);
        const float invz2 = 1.0/z2;

        float u2 = fx*x2*invz2+cx;
        float v2 = fy*y2*invz2+cy;
        float errX2 = u2 - kp2.pt.x;
        float errY2 = v2 - kp2.pt.y;
        if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
        {
          continue;
        }

        // check scale consistency
        cv::Mat normal1 = x3D-position_local;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = x3D-position_neighbour;
        float dist2 = cv::norm(normal2);

        if(dist1 == 0 || dist2==0)
        {
          continue;
        }

        const float ratioDist = dist2/dist1;
        const float ratioOctave = std::pow(Frame::level_scale_factor, kp1.octave)/std::pow(Frame::level_scale_factor, kp2.octave);

        if(ratioDist*ratio_factor<ratioOctave || ratioDist>ratioOctave*ratio_factor)
        {
          continue;
        }
        // Triangulate Successfully
        //cv::Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2));
        auto ptr_new_landmark = Landmark::CreateLandmark(ptr_local_current_frame, ptr_frame, cv::Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2)));
        ptr_new_landmark->addObserver(ptr_local_current_frame, real_idx1);
        ptr_new_landmark->addObserver(ptr_frame, real_idx2);

        ptr_local_current_frame->insertLandmark(ptr_new_landmark, real_idx1);
        ptr_frame->insertLandmark(ptr_new_landmark, real_idx2);

        ptr_new_landmark->updateLandmark();

        local_landmarks_.push_back(ptr_new_landmark);
        ++num_new_landmarks;
      }
    }
  }

  void Map::optimizeLocalMap()
  {
    if(keyframes_wait_to_local_optimized_.empty() == true)
    {
      AK_DLOG_INFO << "No frame in local map need to optimize.";
      return;
    }
    ptr_local_current_frame_ = keyframes_wait_to_local_optimized_.front();
    keyframes_wait_to_local_optimized_.pop_front();
    initLocalMap(ptr_local_current_frame_);
    screenLocalLandmark(local_landmarks_);
    newTriangLandmark(ptr_local_current_frame_);

    if(keyframes_wait_to_local_optimized_.empty() == true)
    {
      searchNeighborKeyframes();
    }
    // flag_abort_BA = false;
    if(keyframes_vector_.size()>2)
    {
      //LocalBA
      float sigma2 = 1.0;
      bool abortBA = false;
      Optimizer::localBundleAdjustment(ptr_local_current_frame_,
                                       this->shared_from_this(),
                                       sigma2,
                                       Frame::K,
                                       &abortBA,
                                       true);
    }
    screenKeyFrame(ptr_local_current_frame_);

    // Insert to loop closure
  }

  void Map::screenKeyFrame(ak::Frame::Ptr &ptr_local_current_frame)
  {
    auto all_covision_frames = ptr_local_current_frame->getAllBestCovisibleFrame();

    for(auto covision_frame:all_covision_frames)
    {
      if(covision_frame->getID() == 0)
      {
        continue;
      }
      auto landmarks_pair = covision_frame->getLandmarks();
      unsigned int threshold_observation_times = 3;
      unsigned int num_redundant_observations = 0;
      unsigned int num_landmarks = 0;
      for(auto landmark_pair:landmarks_pair)
      {
        auto ptr_landmark = landmark_pair.second;
        auto idx_kp = landmark_pair.first;
        if(ptr_landmark != nullptr)
        {
          if(ptr_landmark->isAvailable() == true)
          {
            ++num_landmarks;
            if(ptr_landmark->getObservationTimes() > threshold_observation_times)
            {
              auto kp_level = covision_frame->getKeyPoints()[idx_kp].octave;
              auto observators = ptr_landmark->getObservers();
              unsigned int num_observation = 0;
              for(auto observator_pair:observators)
              {
                auto ptr_observator = observator_pair.first;
                auto idx_kpi = observator_pair.second;
                if(ptr_observator==covision_frame)
                {
                  continue;
                }
                auto kpi_level = ptr_observator->getKeyPoints()[idx_kpi].octave;
                if(kpi_level <= kp_level + 1)
                {
                  ++num_observation;
                  if(num_observation>=threshold_observation_times)
                  {
                    break;
                  }
                }
              }

              if(num_observation>=threshold_observation_times)
              {
                ++num_redundant_observations;
              }
            }
          }
        }
      }
      if(num_redundant_observations>0.9*num_landmarks)
      {
        covision_frame->setUnavailable();
      }
    }
  }

  void Map::screenLocalLandmark(std::list<Landmark::Ptr>& local_landmarks)
  {
    const int threshold_observation_times = 2;
    auto iter_landmark = local_landmarks.begin();
    const auto local_current_keyframe_id = ptr_local_current_frame_->getID();

    while(iter_landmark != local_landmarks.end())
    {
      auto ptr_landmark = *iter_landmark;
      auto first_frame_index = ptr_landmark->getFirstFramePair().first->getID();
      if(ptr_landmark == nullptr)
      {
        AK_DLOG_WARNING << "Nullptr landmark";
      }
      // TODO: (aliben.develop@gmail.com)
      // Statistics for found and visible times of landmark
      else if(ptr_landmark->calFoundRatio() > 0.25f)
      {
        ptr_landmark->setUnavailable();
        local_landmarks.erase(iter_landmark);
      }
      else if(local_current_keyframe_id - first_frame_index >= 2 &&
              ptr_landmark->getObservationTimes() <= threshold_observation_times)
      {
        ptr_landmark->setUnavailable();
        local_landmarks.erase(iter_landmark);
      }
      else if(local_current_keyframe_id-first_frame_index >=3)
      {
        ptr_landmark->setUnavailable();
        local_landmarks.erase(iter_landmark);
      }
      else
      {
        iter_landmark++;
      }
    }
  }

  void Map::searchNeighborKeyframes()
  {
    int num_covisible = 20; // For monocular
    std::vector<Frame::Ptr> target_keyframes;
    const auto covisible_frames = ptr_local_current_frame_->getBestCovisibleFrameWith(num_covisible);
    for(const auto& ptr_frame:covisible_frames)
    {
      if(ptr_frame==nullptr||ptr_frame->fuse_for_id_==ptr_local_current_frame_->id_)
      {
        continue;
      }
      target_keyframes.push_back(ptr_frame);
      ptr_frame->fuse_for_id_ = ptr_local_current_frame_->id_;
      const auto sec_level_covisible_frames = ptr_frame->getBestCovisibleFrameWith(5);
      for(const auto& ptr_sec_level_frame:sec_level_covisible_frames)
      {
        if(ptr_sec_level_frame==nullptr || ptr_sec_level_frame->fuse_for_id_==ptr_local_current_frame_->id_ || ptr_sec_level_frame->id_==ptr_local_current_frame_->id_)
        {
          continue;
        }
        target_keyframes.push_back(ptr_sec_level_frame);
      }
    }

    ORBmatcher matcher;
    auto landmarks_current_frame = ptr_local_current_frame_->getLandmarksVector();

    // Fuse KF
    for(const auto ptr_frame:target_keyframes)
    {
      matcher.Fuse(ptr_frame, landmarks_current_frame);
    }

    std::vector<Landmark::Ptr> fuse_candidate_landmarks;
    fuse_candidate_landmarks.reserve(target_keyframes.size()*landmarks_current_frame.size());
    for(const auto ptr_frame:target_keyframes)
    {
      auto landmarks_target_frame = ptr_frame->getLandmarksVector();
      for (const auto ptr_landmark:landmarks_target_frame)
      {
        if (ptr_landmark == nullptr)
        {
          continue;
        }
        if (ptr_landmark->isAvailable() == false || ptr_landmark->fuse_for_id_ == ptr_local_current_frame_->id_)
        {
          if (ptr_landmark->is_fused_ == true)
            continue;
        }
        ptr_landmark->fuse_for_id_ = ptr_local_current_frame_->id_;
        fuse_candidate_landmarks.push_back(ptr_landmark);
      }
    }

    matcher.Fuse(ptr_local_current_frame_, fuse_candidate_landmarks);

    landmarks_current_frame = ptr_local_current_frame_->getLandmarksVector();

    for(const auto ptr_landmark:landmarks_current_frame)
    {
      if(ptr_landmark==nullptr||ptr_landmark->isAvailable()==false)
      {
        continue;
      }
      ptr_landmark->updateLandmark();
    }

    ptr_local_current_frame_->updateCovision();
  }
}
