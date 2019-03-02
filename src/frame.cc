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
}
