/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-04-24 18:48:27
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
#include <assert.h>

//CODE
namespace ak
{
  using namespace std::chrono;
  int Frame::factory_id = 0;
  //std::vector<std::pair<size_t, cv::Point3f>> Frame::raw_init_landmarks;
  std::vector<std::pair<cv::DMatch, cv::Point3f>> Frame::raw_init_landmarks;
  cv::Mat Frame::K;
  float Frame::level_scale_factor = 1.2;
  float Frame::level_scale_factor_log = std::log(level_scale_factor);

  Frame::Frame(ID_t id)
    : id_(id),
      fuse_for_id_(id),
      localBA_for_id_(id),
      fixedBA_for_id_(id)
  {
  }

  Frame::Ptr Frame::CreateFrame(const cv::Mat& image,
                                const ORBextractor::Ptr& ptr_orb_extractor,
                                const std::shared_ptr<DBoW3::Vocabulary>& ptr_vocal)
  {
    // TODO: (aliben.develop@gmail.com)
    auto pFrame = std::make_shared<Frame>(Frame::factory_id);
    pFrame->image_ = image.clone();
    pFrame->grid_property_.setup(image);

    pFrame->ptr_vocal_ = ptr_vocal;
    auto num_keypoints = pFrame->extractKeyPoints(ptr_orb_extractor);
    pFrame->computeBOW();
    AK_LOG_INFO << "FrameID: " << Frame::factory_id++ << "\tfeature: " << num_keypoints;
    return pFrame;
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

  void Frame::assignFeaturePointToGrid()
  {
    auto kp_size = this->keypoints_.size();
    for(size_t idx=0; idx<kp_size; ++idx)
    {
      const cv::KeyPoint& kp = this->keypoints_[idx];
      size_t grid_pos_x, grid_pos_y;
      if(this->getGridPosition(kp, grid_pos_x, grid_pos_y))
      {
        this->keypoints_grid_[grid_pos_x][grid_pos_y].push_back(idx);
      }
    }
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

  bool Frame::isInImage(const float &x, const float &y) const
  {
    return (x>=grid_property_.min_x && x<=grid_property_.max_x && y>=grid_property_.min_y && y<=grid_property_.max_y);
  }


  //==============================GetMethod===============================
  ID_t Frame::getID()
  { return id_; }

  const std::vector<cv::KeyPoint>& Frame::getKeyPoints()
  { return keypoints_;}

  const std::unordered_map<size_t, Landmark::Ptr>& Frame::getLandmarks()
  { return landmarks_;}

  const std::vector<Landmark::Ptr>& Frame::getLandmarksVector()
  { return landmarks_vector_;}

  const DBoW3::FeatureVector& Frame::getFeatureVector()
  { return feature_vector_;}

  const cv::Mat& Frame::getDescriptors()
  { return descriptors_;}

  const cv::Mat& Frame::getTF()
  { return T21_estimated_;}

  //const cv::Mat& Frame::getPose()
  //{ return camera_pose_;}

  const cv::Mat& Frame::getOrigin()
  { return camera_position_;}

  const cv::Mat& Frame::getRotation()
  { return rotation_;}

  const cv::Mat& Frame::getTranslation()
  { return translation_;}

  size_t Frame::getNumTrackedLandmarksWithObservationTimes(const size_t& min_observation_times)
  {
    size_t num_tracked_landmarks = 0;
    const bool flag_checked_observation_times = min_observation_times>0;
    for(const auto& landmark_pair:landmarks_)
    {
      auto ptr_landmark = landmark_pair.second;
      if(ptr_landmark == nullptr)
      {
        AK_LOG_ERROR << "nullptr pLandmark!";
        continue;
      }
      if(ptr_landmark->isAvailable() == false)
      {
        AK_DLOG_WARNING << "Landmark " << ptr_landmark->getID() << " is not included by map, from Frame " << this->getID();
        continue;
      }
      if(flag_checked_observation_times == true)
      {
        if(ptr_landmark->getObservationTimes() > min_observation_times)
        {
          ++num_tracked_landmarks;
        }
      }
      else
      {
        ++num_tracked_landmarks;
      }
    }
    return num_tracked_landmarks;
  }

  std::vector<Frame::Ptr> Frame::getBestCovisibleFrameWith(const size_t& num_covisible_frame)
  {
    if(covision_frame_ordered_.size() < num_covisible_frame)
    {
      AK_LOG_WARNING << "In Frame: " << this->getID() << " The number of covisible frames < " << num_covisible_frame;
      return covision_frame_ordered_;
    }
    return std::vector<Frame::Ptr>(covision_frame_ordered_.begin(), covision_frame_ordered_.begin()+num_covisible_frame);
  }

  const std::vector<Frame::Ptr>& Frame::getAllBestCovisibleFrame()
  {
    return covision_frame_ordered_;
  }


  bool Frame::isGood()
  { return is_good_;}
  //==============================END GetMethod===============================

  void Frame::replaceLandmark(const ak::Landmark::Ptr& ptr_landmark, size_t kp_idx)
  {
    auto drop_frame_id = landmarks_[kp_idx]->getID();
    landmarks_[kp_idx] = ptr_landmark;
    landmarks_index_query_.erase(drop_frame_id);
    landmarks_index_query_.insert(std::pair<ID_t, size_t>(ptr_landmark->getID(), kp_idx));
  }

  //==============================SetMethod===============================
  void Frame::setTF(const cv::Mat& T21)
  {
    T21_estimated_ = T21.clone();
    rotation_ = T21_estimated_.rowRange(0,3).colRange(0,3);
    translation_ = T21_estimated_.rowRange(0,3).col(3);
    setPose(T21);
  }

  void Frame::setPose(const cv::Mat& TF)
  {
    cv::Mat Rcw = TF.rowRange(0,3).colRange(0,3).t();
    cv::Mat tcw = TF.rowRange(0,3).col(3);
    cv::Mat camera_pose_ = cv::Mat::eye(4, 4, CV_32F);
    Rcw.copyTo(camera_pose_.rowRange(0,3).colRange(0,3));
    camera_position_ = -Rcw * tcw;
    camera_pose_.rowRange(0,3).col(3) = camera_position_;
  }

  void Frame::setGBATF(const cv::Mat& T21)
  {
    T21_global_BA_ = T21.clone();
    cv::Mat Rcw = T21_global_BA_.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = T21_global_BA_.rowRange(0,3).col(3);
    Rcw.copyTo(camera_pose_.rowRange(0,3).colRange(0,3));
    camera_pose_.rowRange(0,3).col(3) = -Rcw.t() * tcw;
  }

  void Frame::setNumGBA(unsigned long num_gba)
  { num_ba_global_ = num_gba; }

  void Frame::setReferenceFrame(const Frame::Ptr& ptr_ref_frame)
  { ptr_reference_frame_ = ptr_ref_frame;}
  //==============================END SetMethod===============================
  
  //==============================ProtectedMethod================================
  size_t Frame::extractKeyPoints(const ORBextractor::Ptr& ptr_orb_extractor)
  {
    (*ptr_orb_extractor)(this->image_, cv::Mat(), this->keypoints_, this->descriptors_);
    auto rows = this->descriptors_.rows;
    this->descriptor_vectors_.clear();
    this->descriptor_vectors_.reserve(rows);
    for(int i = 0;i<rows;++i)
    {
      this->descriptor_vectors_.push_back(this->descriptors_.row(i));
    }
    this->assignFeaturePointToGrid();
    return this->keypoints_.size();
  }

  void Frame::insertLandmark(Landmark::Ptr& ptr_landmark, size_t kp_index)
  {
    if(ptr_landmark == nullptr)
    {
      AK_DLOG_FATAL << "Insert a nullptr landmark";
      assert(ptr_landmark!=nullptr);
    }
    landmarks_.insert(std::pair<size_t, Landmark::Ptr>(kp_index, ptr_landmark));
    landmarks_index_query_.insert(std::pair<ID_t, size_t>(ptr_landmark->getID(), kp_index));
    landmarks_vector_.push_back(ptr_landmark);
    //AK_DLOG_INFO << "Landmark ID: " << ptr_landmark->getID() << "\tKP index: " << kp_index;
  }

  void Frame::updateCovision()
  {
    std::unordered_map<Frame::Ptr, unsigned int> covision_counter;
    for(const auto& landmark_pair:landmarks_)
    {
      auto ptr_landmark = landmark_pair.second;
      // Check this pointer
      if(ptr_landmark == nullptr)
      {
        AK_DLOG_WARNING << "nullptr landmark in covision";
        continue;
      }
      // TODO: (aliben.develop@gmail.com)
      // If landmark has been removed from map
      auto observation_this_landmark = ptr_landmark->getObservers();
      for(const auto& obs_frame_pair:observation_this_landmark)
      {
        auto ptr_frame = obs_frame_pair.first;
        if(id_ == ptr_frame->getID())
        {
          continue;
        }
        covision_counter[ptr_frame]++;
      }
    }

    if(covision_counter.empty() == true)
    {
      AK_DLOG_INFO << "Frame: " << id_ << " doesn't find out covision frame.";
      return;
    }

    unsigned int max_count_covision = 0;
    unsigned int threshold_co_landmark = 15;
    Frame::Ptr ptr_covision_frame = nullptr;
    std::vector<std::pair<unsigned int, Frame::Ptr>> weight_covision;
    for(const auto& covision_pair:covision_counter)
    {
      auto count_num = covision_pair.second;
      if(count_num > max_count_covision)
      {
        threshold_co_landmark = count_num;
        ptr_covision_frame = covision_pair.first;
      }
      if(count_num > threshold_co_landmark)
      {
        weight_covision.push_back(std::make_pair(covision_pair.second, covision_pair.first));
        ptr_covision_frame->addCovision(this->shared_from_this(), count_num);
      }
    }

    if(weight_covision.empty() == true)
    {
      assert(ptr_covision_frame != nullptr);
      weight_covision.push_back(std::make_pair(max_count_covision, ptr_covision_frame));
      ptr_covision_frame->addCovision(this->shared_from_this(), max_count_covision);
    }

    // Sort weight - ascent
    std::sort(weight_covision.begin(), weight_covision.end());
    std::list<Frame::Ptr> covision_frame_ordered;
    std::list<unsigned int> covision_weight_ordered;
    for(const auto& weight_pair:weight_covision)
    {
      // Decent
      covision_frame_ordered.push_front(weight_pair.second);
      covision_weight_ordered.push_front(weight_pair.first);
    }

    // TODO: (aliben.develop@gmail.com)
    // Lock for thread safe
    covision_sets_ = covision_counter;
    covision_frame_ordered_ = std::vector<Frame::Ptr>(covision_frame_ordered.begin(), covision_frame_ordered.end());
    covision_weight_ordered_ = std::vector<unsigned int>(covision_weight_ordered.begin(), covision_weight_ordered.end());
  }

  void Frame::addCovision(const Frame::Ptr& ptr_covision, unsigned int weight)
  {
    if(covision_sets_.count(ptr_covision) == 0)
    {
      covision_sets_[ptr_covision] = weight;
    }
    else if(covision_sets_[ptr_covision]!=weight)
    {
      covision_sets_[ptr_covision] = weight;
    }
    else
    {
      return;
    }

    // UpdateBestCovisibles();
  }

  float Frame::computeMedianDepth(int section)
  {
    cv::Mat transform = T21_estimated_.clone();
    std::vector<float> depth_vector;
    cv::Mat z_rotation = transform.row(2).colRange(0,3).t();
    auto z_translation = transform.at<float>(2,3);
    for(const auto& landmark_pair:landmarks_)
    {
      auto landmark = landmark_pair.second;
      if(landmark == nullptr)
      {
        AK_LOG_ERROR << "pLandmark is nullptr";
        continue;
      }
      auto lm3d = cv::Mat(landmark->getPosition());
      float z = z_rotation.dot(lm3d) + z_translation;
      depth_vector.push_back(z);
    }
    std::sort(depth_vector.begin(), depth_vector.end());
    return depth_vector[(depth_vector.size() - 1)/section];
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

  void Frame::computeBOW()
  {
    if(bow_vector_.empty() || feature_vector_.empty())
    {
      ptr_vocal_->transform(descriptor_vectors_, bow_vector_, feature_vector_, 4);
      AK_DLOG_INFO << "Initial Bow and feature";
      //AK_DLOG_INFO << "Keypoints size: " << keypoints_.size();
      AK_DLOG_INFO << "Bow_vector size: " << bow_vector_.size();
      AK_DLOG_INFO << "feature_vector size: " << feature_vector_.size();
      return;
    }
    AK_DLOG_WARNING << "No initial bow and feature";
  }
  //==========================End ProtectedMethod================================
}
