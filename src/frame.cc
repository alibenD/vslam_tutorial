/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-21 16:11:21
  * @last_modified_date: 2019-03-02 22:26:12
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
  /*const*/ int NUM_FEATURES = 2000;
  /*const*/ float LEVEL_SCALE_FACTOR = 1.2;
  /*const*/ int NUM_LEVELS = 8;
  /*const*/ int INIT_FAST = 20;
  /*const*/ int MIN_FAST = 7;
  //cv::Ptr<cv::ORB> Frame::ptr_orb_ = cv::ORB::create(2000);
  ORBextractor::Ptr Frame::ptr_orb_extractor_init_advanced = std::make_shared<ORBextractor>(2*NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);
  ORBextractor::Ptr Frame::ptr_orb_extractor_advanced = std::make_shared<ORBextractor>(NUM_FEATURES,LEVEL_SCALE_FACTOR,NUM_LEVELS,INIT_FAST,MIN_FAST);
  //ORBmatcher::Ptr Frame::ptr_orb_matcher_init_advanced = std::make_shared<ORBextractor>(0.9, true);
  int Frame::factory_id = 0;
  std::vector<std::pair<size_t, cv::Point3f>> Frame::init_landmarks;

  Frame::Frame(ID_t id)
    : id_(id)
  {
  }

  Frame::Ptr Frame::CreateFrame(const cv::Mat& image,
                                cv::Mat& image_with_keypoints)
  {
    // TODO: (aliben.develop@gmail.com)
    auto pFrame = std::make_shared<Frame>(Frame::factory_id);
    ++Frame::factory_id;
    pFrame->grid_property_.setup(image);
    auto num_keypoints = pFrame->extractKeyPoints(image, image_with_keypoints);
    pFrame->image_ = image.clone();
    AK_LOG_INFO << "FrameID: " << Frame::factory_id;
    return pFrame;
  }

  size_t Frame::extractKeyPoints(const cv::Mat& image,
                                 cv::Mat& img_with_keypoints)
  {
    // TODO: (aliben.develop@gmail.com)
    // To unify orb extractor call
      (*Frame::ptr_orb_extractor_init_advanced)(image,
                                                cv::Mat(),
                                                this->keypoints_,
                                                this->descriptors_);
      //(*Frame::ptr_orb_extractor_advanced)(image,
      //                                     cv::Mat(),
      //                                     this->keypoints_,
      //                                     this->descriptors_);
    this->assignFeaturePointToGrid();
    return this->keypoints_.size();
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

  void Frame::insertLandmark(Landmark::Ptr& ptr_landmark, size_t kp_index)
  {
      landmarks_.insert(std::pair<size_t, Landmark::Ptr>(kp_index, ptr_landmark));
      landmarks_index_query_.insert(std::pair<ID_t, size_t>(ptr_landmark->getID(), kp_index));
      AK_DLOG_INFO << "Landmark ID: " << ptr_landmark->getID() << "\tKP index: " << kp_index;
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
        continue;
      }
      // TODO: (aliben.develop@gmail.com)
      // If landmark has been removed from map
//      if(landmark)
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
    cv::Mat transform = transform_camera_at_world_.clone();
    std::vector<float> depth_vector;
    cv::Mat z_rotation = transform.row(2).colRange(0,3).t();
    auto z_translation = transform.at<float>(2,3);
    for(const auto& landmark_pair:landmarks_)
    {
      auto landmark = landmark_pair.second;
      if(landmark != nullptr)
      {
        auto lm3d = cv::Mat(landmark->getPosition());
        float z = z_rotation.dot(lm3d) + z_translation;
        depth_vector.push_back(z);
      }
    }
    std::sort(depth_vector.begin(), depth_vector.end());
    return depth_vector[(depth_vector.size() - 1)/section];
  }
}
