/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmark.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-02 23:57:11
  * @last_modified_date: 2019-03-12 15:14:49
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/landmark.hh>
#include <visual_slam/ORBmatcher.hh>
#include <visual_slam/frame.hh>
//CODE
namespace ak
{
  Landmark::ID_t Landmark::factory_id = 0;

  Landmark::Landmark(ID_t id, float x, float y, float z)
    : Landmark(id, cv::Point3f(x, y, z))
  {

  }

  Landmark::Landmark(ID_t id, const cv::Point3f& position)
    : id_(id),
      position_at_world_(position)
  {

  }

  Landmark::Landmark(ID_t id,
                     std::shared_ptr<Frame>& ptr_last_frame,
                     std::shared_ptr<Frame>& ptr_current_frame,
                     const cv::Point3f& position)
    : id_(id),
      position_at_world_(position),
      position_normal_at_camera_(position/cv::norm(position)),
      first_frame_pair_(ptr_last_frame, ptr_current_frame)
  {

  }

  Landmark::Ptr Landmark::CreateLandmark(Frame::Ptr& ptr_last_frame,
                                         Frame::Ptr& ptr_current_frame,
                                         const cv::Point3f& position)
  {
    auto pLandmark = std::make_shared<Landmark>(Landmark::factory_id,
                                                ptr_last_frame,
                                                ptr_current_frame,
                                                position);
    ++Landmark::factory_id;
    return pLandmark;
  }

  void Landmark::addObserver(const Frame::Ptr& ptr_observer_frame,
                             size_t kp_idx)
  {
    if(observers_.find(ptr_observer_frame) != observers_.end())
    {
      AK_DLOG_WARNING << "Frame: " << ptr_observer_frame->getID() << " has been an observer for keypoint " << kp_idx;
      return;
    }
    //AK_LOG_WARNING << "Frame: " << ptr_observer_frame->getID() << " added for keypoint " << kp_idx;

    observers_.insert(std::pair<Frame::Ptr, size_t>(ptr_observer_frame, kp_idx));
    ++num_observation_times_;
  }

  void Landmark::updateLandmark()
  {
    updateMeanDescriptor();
    updateNormalRepresentation();
  }

  void Landmark::updateMeanDescriptor()
  {
    if(observers_.empty() == true)
    {
      return;
    }
    descriptors_all_observed_.reserve(observers_.size());
    for(const auto& observer : observers_)
    {
      auto ptr_frame = observer.first;
      auto kp_idx = observer.second;
      // TODO: (aliben.develop@gmail.com)
      // If this frame has been removed from this map,
      // just jump over this frame and continue
      descriptors_all_observed_.push_back(ptr_frame->getDescriptors().row(kp_idx));
    }

    if(descriptors_all_observed_.empty() == true)
    {
      // If no valid frame bind to this landmark, which means all frame relative with this landmark have been removed from map
      return;
    }
    const size_t NUM_BINDING_KEYPOINTS = descriptors_all_observed_.size();
    float distance_matrix[NUM_BINDING_KEYPOINTS][NUM_BINDING_KEYPOINTS];
    for(size_t i=0; i<NUM_BINDING_KEYPOINTS; ++i)
    {
      distance_matrix[i][i] = 0;
      for(size_t j=i+1; j<NUM_BINDING_KEYPOINTS; ++j)
      {
        int dist_i_j = ak::ORBmatcher::DescriptorDistance(descriptors_all_observed_[i], descriptors_all_observed_[j]);
        distance_matrix[i][j] = dist_i_j;
        distance_matrix[j][i] = distance_matrix[i][j];
      }
    }

    int median_whole_obs = INT_MAX;
    int median_id_whole_obs = 0;
    for(size_t i=0; i<NUM_BINDING_KEYPOINTS; ++i)
    {
      std::vector<int> distance_sort(distance_matrix[i], distance_matrix[i]+NUM_BINDING_KEYPOINTS);
      std::sort(distance_sort.begin(), distance_sort.end());
      int median = distance_sort[0.5*(NUM_BINDING_KEYPOINTS-1)];
      if(median < median_whole_obs)
      {
        median_whole_obs = median;
        median_id_whole_obs = i;
      }
    }
    mean_descriptor_ = descriptors_all_observed_[median_id_whole_obs].clone();
  }

  void Landmark::updateNormalRepresentation()
  {
    // TODO: (aliben.develop@gmail.com)
    // If this landmark would been dropped from the map, just continue
    // TODO: (aliben.develop@gmail.com
    // Consider to add lock to read landmark info
    cv::Mat obs_normal = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat position = cv::Mat(position_at_world_);
    for(const auto& observer : observers_)
    {
      auto ptr_frame_obs = observer.first;
      cv::Mat camera_origin = ptr_frame_obs->getOrigin();
      cv::Mat normal = position - camera_origin;
      obs_normal += normal/cv::norm(normal);
    }
    auto size_obs = observers_.size();
    auto kp_idx = observers_[first_frame_pair_.second];
    property_.level_at_ = first_frame_pair_.second->getKeyPoints()[kp_idx].octave;
    property_.distance_ = cv::norm(position - first_frame_pair_.second->getOrigin());
    property_.level_factor_ = 1.2f;
    property_.num_level_ = 8;

    max_distance_ = property_.distance_ * property_.level_factor_;
    min_distance_ = max_distance_/std::pow(property_.level_factor_, property_.num_level_ - 1);
    obs_normal_ = obs_normal / size_obs;
  }
}
